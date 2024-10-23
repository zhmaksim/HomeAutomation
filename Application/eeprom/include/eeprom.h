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

#ifndef EEPROM_H_
#define EEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "i2c.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define EEPROM_QUEUE_SIZE       32

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение перечисления состояний EEPROM
 */
enum eeprom_state {
    EEPROM_WORK,
    /* --- */
    EEPROM_READ_ERROR,
    EEPROM_WRITE_ERROR,
    EEPROM_MALLOC_ERROR,
};


/**
 * @brief           Определение структуры уникальных данных EEPROM
 */
struct eeprom_uid {
    uint16_t    version;                        /*!< Версия */

    uint8_t     data[8];                        /*!< Данные */
};


/**
 * @brief           Определение структуры данных обработчика EEPROM
 */
struct eeprom_handle {
    struct i2c_handle      *i2c;                /*!< Указатель на структуру данных обработчика I2C */

    SemaphoreHandle_t       i2c_mutex;          /*!< Mutex I2C */

    EventGroupHandle_t      i2c_event_group;    /*!< Event Group I2C */

    uint8_t                 slave_address;      /*!< Адрес подчиненного устройства */

    struct eeprom_uid       uid;                /*!< Уникальные данные */

    enum eeprom_state       state;              /*!< Состояние */

    QueueHandle_t           queue;              /*!< Очередь */
};

/* Exported variables ------------------------------------------------------ */

extern struct eeprom_handle eeprom;

/* Exported function prototypes -------------------------------------------- */

void eeprom_init(struct eeprom_handle *handle);

void eeprom_load(struct eeprom_handle *handle);

void eeprom_i2c_command_completed_it_handler(struct eeprom_handle *handle);

void eeprom_i2c_error_it_handler(struct eeprom_handle *handle);

hal_status_t eeprom_write(struct eeprom_handle *handle, const void *data);

bool eeprom_is_work(struct eeprom_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* EEPROM_H_ */
