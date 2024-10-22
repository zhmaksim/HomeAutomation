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

#include "watch.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик RTC */
extern struct rtc_handle rtc;

/* Mutex RTC */
extern SemaphoreHandle_t rtc_mutex;

/* Обработчик Watch */
struct watch_handle watch = {
    .rtc = &rtc,
};

/* Private function prototypes --------------------------------------------- */

static void watch_process(void *arg);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать часы
 *
 * @param[in]       handle: Указатель на структуру данных обработчика часов
 */
void watch_init(struct watch_handle *handle)
{
    assert(handle != NULL);
    assert(handle->rtc != NULL);

    if (handle->rtc == &rtc)
        handle->rtc_mutex = rtc_mutex;

    xTaskCreate(watch_process,
                "watch",
                configMINIMAL_STACK_SIZE,
                (void *) handle,
                tskIDLE_PRIORITY + 1,
                NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать события часов
 *
 * @param[in]       arg: Указатель на параметры
 */
static void watch_process(void *arg)
{
    static const TickType_t frequency = pdMS_TO_TICKS(500);

    struct watch_handle *handle = arg;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, frequency);

        if (handle->datetime_is_setup) {
            handle->datetime_is_setup = false;

            watch_setup(handle, &handle->datetime_setup);
        }

        watch_now(handle, &handle->datetime_now);
    }

    vTaskDelete(NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Дата и время часов
 *
 * @param[in]       handle: Указатель на структуру данных обработчика часов
 * @param[out]      datetime: Указатель на структуру данных даты и времени часов
 * @return          Статус @ref hal_status_t
 */
hal_status_t watch_now(struct watch_handle *handle, union watch_datetime *datetime)
{
    assert(handle != NULL);
    assert(datetime != NULL);

    struct rtc_datetime rtc_datetime;

    if (xSemaphoreTake(handle->rtc_mutex, pdMS_TO_TICKS(50)) != pdPASS)
        return HAL_ERROR;

    hal_rtc_now(handle->rtc, &rtc_datetime);

    datetime->day = rtc_datetime.day;
    datetime->month = rtc_datetime.month;
    datetime->year = rtc_datetime.year;
    datetime->hours = rtc_datetime.hours;
    datetime->minutes = rtc_datetime.minutes;
    datetime->seconds = rtc_datetime.seconds / 2;

    xSemaphoreGive(handle->rtc_mutex);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить дату и время часов
 *
 * @param[in]       handle: Указатель на структуру данных обработчика часов
 * @param[in]       datetime: Указатель на структуру данных даты и времени часов
 * @return          Статус @ref hal_status_t
 */
hal_status_t watch_setup(struct watch_handle *handle, const union watch_datetime *datetime)
{
    assert(handle != NULL);
    assert(datetime != NULL);

    struct rtc_datetime rtc_datetime;

    if (xSemaphoreTake(handle->rtc_mutex, pdMS_TO_TICKS(50)) != pdPASS)
        return HAL_ERROR;

    rtc_datetime.day = datetime->day;
    rtc_datetime.month = datetime->month;
    rtc_datetime.year = datetime->year;
    rtc_datetime.hours = datetime->hours;
    rtc_datetime.minutes = datetime->minutes;
    rtc_datetime.seconds = datetime->seconds * 2;

    hal_rtc_setup(handle->rtc, &rtc_datetime);

    xSemaphoreGive(handle->rtc_mutex);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */
