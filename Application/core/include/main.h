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

#ifndef MAIN_H_
#define MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение перечисления статусов устройства
 */
enum device_status {
    DEVICE_NOT_INIT,
    DEVICE_WORK = 0xFF,
};


/**
 * @brief           Определение структуры данных версии ПО
 */
struct sw_version {
    uint16_t    major;                          /*!< Увеличивается при критических изменениях ПО */

    uint16_t    minor;                          /*!< Увеличивается при добавлении новых функций ПО */

    uint16_t    build;                          /*!< Увеличивается при исправлении ошибок ПО */
};


/**
 * @brief           Определение структуры данных даты ПО
 */
struct sw_date {
    uint8_t     day;                            /*!< День */

    uint8_t     month;                          /*!< Месяц */

    uint8_t     year;                           /*!< Год */
};

/* Exported variables ------------------------------------------------------ */

extern const uint8_t device_id;

extern enum device_status device_status;

extern const struct sw_version sw_version;

extern const struct sw_date sw_date;

extern const uint32_t sw_checksum;

/* Exported function prototypes -------------------------------------------- */

void error(void);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MAIN_H_ */
