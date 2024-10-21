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

#ifndef STM32F4XX_HAL_RTC_H_
#define STM32F4XX_HAL_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных RTC
 */
typedef RTC_TypeDef rtc_t;


/**
 * @brief           Определение перечисления источников тактирования RTC
 */
enum rtc_clock_source {
    RTC_NO_CLOCK,
    RTC_LSE,
    RTC_LSI,
    RTC_HSE,
};


/**
 * @brief           Определение перечисления формата времени RTC
 */
enum rtc_hour_format {
    RTC_24HOUR_DAY,
    RTC_12HOUR_DAY,
};


/**
 * @brief           Определение перечисления сигналов CALIB RTC
 */
enum rtc_calib_output {
    RTC_CALIB_OUTPUT_512HZ,
    RTC_CALIB_OUTPUT_1HZ,
};


/**
 * @brief           Определение перечисления дней недели RTC
 */
enum rtc_weekday {
    RTC_WEEKDAY_FORBIDDEN,
    RTC_MONDAY,
    RTC_TUESDAY,
    RTC_WEDNESDAY,
    RTC_THURSDAY,
    RTC_FRIDAY,
    RTC_SATURDAY,
    RTC_SUNDAY,
};


/**
 * @brief           Определение структуры данных инициализации RTC
 */
struct rtc_init {
    enum rtc_clock_source   clksource;                  /*!< Источник тактирования:
                                                            - RTC_LSE
                                                            - RTC_LSI
                                                            - RTC_HSE */

    uint16_t                sync_div;                   /*!< Синхронный делитель */

    uint16_t                async_div;                  /*!< Асинхронный делитель */

    enum rtc_hour_format    hour_format;                /*!< Формат времени:
                                                            - RTC_24HOUR_DAY
                                                            - RTC_12HOUR_DAY */

    hal_state_t             ref_clock_detect_enable;    /*!< Включить обнаружение опорной частоты (50 или 60 Гц):
                                                            - HAL_ENABLE
                                                            - HAL_DISABLE */

    hal_state_t             calib_output_enable;        /*!< Включить сигнал калибровки:
                                                            - HAL_ENABLE
                                                            - HAL_DISABLE */

    enum rtc_calib_output   calib_output_sel;           /*!< Выбор сигнала калибровки:
                                                            - RTC_CALIB_OUTPUT_512HZ
                                                            - RTC_CALIB_OUTPUT_1HZ */
};


/**
 * @brief           Определение структуры данных даты и времени RTC
 */
struct rtc_datetime {
    uint8_t    hours;                           /*!< Часы */

    uint8_t    minutes;                         /*!< Минуты */

    uint8_t    seconds;                         /*!< Секунды */

    uint8_t    weekday;                         /*!< День недели */

    uint8_t    day;                             /*!< День */

    uint8_t    month;                           /*!< Месяц */

    uint8_t    year;                            /*!< Год */
};


/**
 * @brief           Определение структуры данных обработчика RTC
 */
struct rtc_handle {
    rtc_t              *instance;               /*!< Указатель на структуру данных RTC */

    struct rtc_init     init;                   /*!< Настройки RTC */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_rtc_init(struct rtc_handle *handle);

void hal_rtc_now(struct rtc_handle *handle, struct rtc_datetime *datetime);

void hal_rtc_setup(struct rtc_handle *handle, const struct rtc_datetime *datetime);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_RTC_H_ */
