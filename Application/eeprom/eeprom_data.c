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

#include "eeprom_data.h"
#include "eeprom.h"

/* Private macros ---------------------------------------------------------- */

/* Private constatns ------------------------------------------------------- */

/* Карта данных
 *
 * Размер памяти: 32768 байт
 * Доступно: [64:28671]
 *
 * Примечание: Первые 64 байта данных зарезервированы для служебных параметров.
 *             Последние 512 байт зарезервированы для алгоритма записи данных. */
static const struct eeprom_data eeprom_data_map[] = {

};

static const size_t eeprom_data_map_size =
        sizeof(eeprom_data_map) / sizeof(struct eeprom_data);

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Получить структуру элемента данных EEPROM
 *
 * @param[in]       index: Индекс элемента данных
 * @return          Указатель на структуру данных, NULL - данные не найдены
 */
struct eeprom_data * eeprom_data_by_index(uint32_t index)
{
    if (index >= eeprom_data_map_size) {
        return NULL;
    } else if (eeprom_data_map[index].data && eeprom_data_map[index].size) {
        return (struct eeprom_data *) &eeprom_data_map[index];
    } else {
        return NULL;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Получить структуру элемента данных EEPROM
 *
 * @param[in]       data: Указатель на данные
 * @return          Указатель на структуру данных, NULL - данные не найдены
 */
struct eeprom_data * eeprom_data(const void *data)
{
    for (register uint32_t index = 0; index < eeprom_data_map_size; index++) {
        if (data == eeprom_data_map[index].data && eeprom_data_map[index].size)
            return (struct eeprom_data *) &eeprom_data_map[index];
    }

    return NULL;
}
/* ------------------------------------------------------------------------- */
