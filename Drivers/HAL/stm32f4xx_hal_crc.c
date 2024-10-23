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

#include "stm32f4xx_hal_crc.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать CRC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика CRC
 */
void hal_crc_init(struct crc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR, CRC_CR_RESET_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Получить значение CRC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика CRC
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Значение CRC
 */
uint32_t hal_crc_value(struct crc_handle *handle,
                       const void *data,
                       size_t size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить наличие данных */
    if (data == NULL || size == 0)
        return UINT32_MAX;

    /* Указатель на данные */
    const uint8_t *pdata = (uint8_t *) data;

    /* Сбросить CRC */
    SET_BIT(handle->instance->CR, CRC_CR_RESET_Msk);

    /* Рассчитать значение CRC */
    while (size > 0) {
        WRITE_REG(handle->instance->DR, *pdata);

        pdata++;
        size--;
    }

    /* Получить значение CRC */
    return READ_REG(handle->instance->DR);
}
/* ------------------------------------------------------------------------- */
