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

#include "led.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчики GPIO LED ST, TX, RX */
extern struct gpio_handle gpio_led_st;
extern struct gpio_handle gpio_led_tx;
extern struct gpio_handle gpio_led_rx;

/* Обработчик LED ST */
struct led_handle led_st = {
    .gpio = &gpio_led_st,
};

/* Обработчик LED TX */
struct led_handle led_tx = {
    .gpio = &gpio_led_tx,
};

/* Обработчик LED RX */
struct led_handle led_rx = {
    .gpio = &gpio_led_rx,
};

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Включить светодиод
 *
 * @param[in]       handle: Указатель на структуру данных обработчика светодиода
 */
void led_on(struct led_handle *handle)
{
    assert(handle != NULL);

    hal_gpio_set_state(handle->gpio, GPIO_SET);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить светодиод
 *
 * @param[in]       handle: Указатель на структуру данных обработчика светодиода
 */
void led_off(struct led_handle *handle)
{
    assert(handle != NULL);

    hal_gpio_set_state(handle->gpio, GPIO_RESET);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Переключить состояние светодиода
 *
 * @param[in]       handle: Указатель на структуру данных обработчика светодиода
 */
void led_toggle(struct led_handle *handle)
{
    assert(handle != NULL);

    hal_gpio_toggle(handle->gpio);
}
/* ------------------------------------------------------------------------- */
