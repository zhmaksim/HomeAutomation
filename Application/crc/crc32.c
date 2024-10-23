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

#include "crc32.h"
#include "crc.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик CRC */
extern struct crc_handle crc;

/* Mutex CRC */
extern SemaphoreHandle_t crc_mutex;

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Рассчитать контрольную сумму CRC32
 *
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Значение контрольной суммы
 */
uint32_t crc32(const void *data, size_t size)
{
    uint32_t crc_value = UINT32_MAX;

    /* Захватить Mutex */
    if (xSemaphoreTake(crc_mutex, pdMS_TO_TICKS(50)) == pdPASS) {
        /* Рассчитать значение контрольной суммы */
        crc_value = hal_crc_value(&crc, data, size);

        /* Освободить Mutex */
        xSemaphoreGive(crc_mutex);
    }

    return crc_value;
}
/* ------------------------------------------------------------------------- */
