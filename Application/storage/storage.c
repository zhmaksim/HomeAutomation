/*
 * Copyright (C) 2024 zhmaksim <zhiharev.maxim.alexandrovich@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Includes ---------------------------------------------------------------- */

#include "storage.h"
#include "w25q.h"
#include "watch.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define DEV_FF_W25Q    0

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Статус устройства FatFs */
static volatile DSTATUS storage_ff_stat = STA_NOINIT;

/* Объекты файловой системы FatFs */
FATFS storage_ff;

TCHAR storage_ff_path[4];

DIR storage_ff_dir;

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать хранилище данных FatFs
 */
void storage_ff_init(void)
{
    FRESULT res;

    storage_ff_path[0] = DEV_FF_W25Q + '0';
    storage_ff_path[1] = ':';
    storage_ff_path[2] = '/';
    storage_ff_path[3] = 0;

    /*
     * Монтирование файловой фистемы,
     * Если файловая система не обнаружена - форматирование
     */
    res = f_mount(&storage_ff, storage_ff_path, 1);

    if (res == FR_NO_FILESYSTEM) {
        MKFS_PARM opt = {FM_FAT, 0, 0, 0, 0};

        void *buff = pvPortMalloc(W25Q_SECTOR_SIZE);
        if (buff == NULL)
            hal_error();

        res = f_mkfs(storage_ff_path, &opt, buff, W25Q_SECTOR_SIZE);
        if (res != FR_OK)
            hal_error();

        vPortFree(buff);

        res = f_mount(&storage_ff, storage_ff_path, 0);
        if (res != FR_OK)
            hal_error();
    } else if (res != FR_OK) {
        hal_error();
    }
}
/* --------------------------------------------------------------------- */

/**
 * @brief           Статус диска FatFs
 *
 * @param[in]       pdrv: Физический диск, на котором размещен том
 * @return          Статус @ref DSTATUS
 */
inline DSTATUS storage_ff_disk_status(BYTE pdrv)
{
    return pdrv == DEV_FF_W25Q ? storage_ff_stat : STA_NOINIT;
}
/* --------------------------------------------------------------------- */

/**
 * @brief           Инициализировать диск FatFs
 *
 * @param[in]       pdrv: Физический диск, на котором размещен том
 * @return          Статус @ref DSTATUS
 */
inline DSTATUS storage_ff_disk_initialize(BYTE pdrv)
{
    if (pdrv == DEV_FF_W25Q) {
        CLEAR_BIT(storage_ff_stat, STA_NOINIT);

        return storage_ff_stat;
    } else {
        return STA_NOINIT;
    }
}
/* --------------------------------------------------------------------- */

/**
 * @brief           Прочитать данные из диска FatFs
 *
 * @param[in]       pdrv: Физический диск, на котором размещен том
 * @param[out]      buff: Указатель на данные
 * @param[in]       sector: Номер сектора
 * @param[in]       count: Количество секторов
 * @return          Результат операции чтения диска @ref DRESULT
 */
inline DRESULT storage_ff_disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv == DEV_FF_W25Q && count > 0) {
        if (READ_BIT(storage_ff_stat, STA_NOINIT))
            return RES_NOTRDY;

        if (count == 1) {
            uint32_t sector_address_start = sector * W25Q_SECTOR_SIZE;

            if (w25q_fast_read(&w25q, sector_address_start, (void *) buff, W25Q_SECTOR_SIZE) != W25Q_OK)
                return RES_ERROR;

            return RES_OK;
        }
    }

    return RES_PARERR;
}
/* --------------------------------------------------------------------- */

/**
 * @brief           Записать данные на диск FatFs
 *
 * @param[in]       pdrv: Физический диск, на котором размещен том
 * @param[in]       buff: Указатель на данные
 * @param[in]       sector: Номер сектора
 * @param[in]       count: Количество секторов
 * @return          Результат операции записи диска @ref DRESULT
 */
inline DRESULT storage_ff_disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv == DEV_FF_W25Q && count > 0) {
        if (READ_BIT(storage_ff_stat, STA_NOINIT))
            return RES_NOTRDY;

        if (READ_BIT(storage_ff_stat, STA_PROTECT))
            return RES_WRPRT;

        if (count == 1) {
            uint32_t sector_address_start = sector * W25Q_SECTOR_SIZE;

            if (w25q_sector_erase(&w25q, sector_address_start) != W25Q_OK)
                return RES_ERROR;

            uint32_t address = sector_address_start;

            for (uint32_t i = 0; i < W25Q_SECTOR_SIZE / W25Q_PAGE_SIZE; i++) {
                if (w25q_page_program(&w25q, address, (void *) buff, W25Q_PAGE_SIZE) != W25Q_OK)
                    return RES_ERROR;

                address += W25Q_PAGE_SIZE;
                buff += W25Q_PAGE_SIZE;
            }

            return RES_OK;
        }
    }

    return RES_PARERR;
}
/* --------------------------------------------------------------------- */

/**
 * @brief           Управление диском FatFs
 *
 * @param[in]       pdrv: Физический диск, на котором размещен том
 * @param[in]       cmd: Команда
 * @param[out]      buff: Указатель на данные
 * @return          Результат операции @ref DRESULT
 */
inline DRESULT storage_ff_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv == DEV_FF_W25Q) {
        if (READ_BIT(storage_ff_stat, STA_NOINIT))
            return RES_NOTRDY;

        switch (cmd) {
            case CTRL_SYNC:
                return RES_OK;
            case GET_SECTOR_SIZE:
                *(DWORD *) buff = W25Q_SECTOR_SIZE;
                return RES_OK;
            case GET_SECTOR_COUNT:
                *(DWORD *) buff = (DWORD) (w25q.size /  W25Q_SECTOR_SIZE);
                return RES_OK;
            case GET_BLOCK_SIZE:
                *(DWORD *) buff = 1;
                return RES_OK;
            default:
                break;
        }
    }

    return RES_PARERR;
}
/* --------------------------------------------------------------------- */

/**
 * @brief           Время FatFs
 *
 * @return          Время DWORD
 */
DWORD get_fattime(void)
{
    union watch_datetime datetime;

    watch_now(&watch, &datetime);

    storage_ff_time_t ff_time;

    ff_time.year = datetime.year + 20;
    ff_time.month = datetime.month;
    ff_time.day = datetime.day;
    ff_time.hours = datetime.hours;
    ff_time.minutes = datetime.minutes;
    ff_time.seconds = datetime.seconds;

    return ff_time.fattime;
}
/* --------------------------------------------------------------------- */
