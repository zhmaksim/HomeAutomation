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

#ifndef STORAGE_H_
#define STORAGE_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "ff.h"
#include "diskio.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных даты и времени FatFs
 */
typedef union
{
    struct
    {
        uint32_t seconds : 5;       /*!< Cекунды, поделенные пополам (0..29) */
        uint32_t minutes : 6;       /*!< Минуты (0..59) */
        uint32_t hours   : 5;       /*!< Часы (0..23) */
        uint32_t day     : 5;       /*!< День (1..31) */
        uint32_t month   : 4;       /*!< Месяц (1..12) */
        uint32_t year    : 7;       /*!< Год, начиная с 1980 (0..127) */
    } __PACKED;
    uint32_t fattime;
} storage_ff_time_t;

/* Exported variables ------------------------------------------------------ */

extern FATFS storage_ff;

extern TCHAR storage_ff_path[4];

extern DIR storage_ff_dir;

/* Exported function prototypes -------------------------------------------- */

void storage_ff_init(void);

DSTATUS storage_ff_disk_status(BYTE pdrv);

DSTATUS storage_ff_disk_initialize(BYTE pdrv);

DRESULT storage_ff_disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count);

DRESULT storage_ff_disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count);

DRESULT storage_ff_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STORAGE_H_ */
