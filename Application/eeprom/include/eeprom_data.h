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

#ifndef EEPROM_DATA_H_
#define EEPROM_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры заголовка элемента данных EEPROM
 */
struct eeprom_data_header {
    uint16_t    id;                             /*!< Идентификатор данных */

    uint16_t    data_size;                      /*!< Размер данных */

    uint32_t    crc;                            /*!< Контрольная сумма */
};


/**
 * @brief           Определение структуры элемента данных EEPROM
 */
struct eeprom_data {
    uint16_t    id;                             /*!< Идентификатор */

    uint16_t    mem_address;                    /*!< Адрес данных в памяти */

    void       *data;                           /*!< Указатель на данные */

    size_t      size;                           /*!< Размер данных */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

struct eeprom_data * eeprom_data_by_index(uint32_t index);

struct eeprom_data * eeprom_data(const void *data);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* EEPROM_DATA_H_ */
