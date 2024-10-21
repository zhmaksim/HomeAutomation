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

#ifndef WATCH_H_
#define WATCH_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "rtc.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных даты и времени часов
 */
union watch_datetime {
    struct {
        uint32_t seconds : 5;       /*!< Cекунды, поделенные пополам (0..29) */
        uint32_t minutes : 6;       /*!< Минуты (0..59) */
        uint32_t hours   : 5;       /*!< Часы (0..23) */
        uint32_t day     : 5;       /*!< День (1..31) */
        uint32_t month   : 4;       /*!< Месяц (1..12) */
        uint32_t year    : 7;       /*!< Год (0..99) */
    } __PACKED;

    uint32_t datetime;              /*!< Дата и время */
};


/**
 * @brief           Определение структуры данных обработчика часов
 */
struct watch_handle {
    struct rtc_handle      *rtc;                    /*!< Указатель на структуру данных обработчика RTC */

    SemaphoreHandle_t       rtc_mutex;              /*!< Mutex RTC */

    union watch_datetime    datetime_now;           /*!< Дата и время */

    bool                    datetime_is_setup;      /*!< Состояние настройки даты и времени */

    union watch_datetime    datetime_setup;         /*!< Дата и время для настройки */
};

/* Exported variables ------------------------------------------------------ */

extern struct watch_handle watch;

/* Exported function prototypes -------------------------------------------- */

void watch_init(struct watch_handle *handle);

hal_status_t watch_now(struct watch_handle *handle, union watch_datetime *datetime);

hal_status_t watch_setup(struct watch_handle *handle, const union watch_datetime *datetime);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* WATCH_H_ */
