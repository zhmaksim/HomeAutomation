/**
 * Copyright (C) 2024 Жихарев Максим <zhiharev.maxim.alexandrovich@yandex.ru>
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

#include "rtc.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик RTC */
struct rtc_handle rtc = {
    .instance = RTC,
};

/* Mutex RTC */
SemaphoreHandle_t rtc_mutex;

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать RTC
 */
void rtc_init(void)
{
    rtc.init.clksource = RTC_LSE;
    rtc.init.sync_div = 0xFF;
    rtc.init.async_div = 0x7F;
    rtc.init.hour_format = RTC_24HOUR_DAY;
    rtc.init.ref_clock_detect_enable = HAL_ENABLE;
    rtc.init.calib_output_enable = HAL_ENABLE;
    rtc.init.calib_output_sel = RTC_CALIB_OUTPUT_1HZ;

    hal_rtc_init(&rtc);

    rtc_mutex = xSemaphoreCreateMutex();
    if (rtc_mutex == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */
