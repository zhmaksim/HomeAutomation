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

#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "gpio.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение перечисления состояний светодиода
 */
enum led_state {
    LED_OFF = GPIO_RESET,
    LED_ON = GPIO_SET,
};


/**
 * @brief           Определение структуры данных обработчика светодиода
 */
struct led_handle {
    struct gpio_handle  *gpio;                  /*!< Указатель на структуру данных обработчика GPIO */
};

/* Exported variables ------------------------------------------------------ */

extern struct led_handle led_st;
extern struct led_handle led_tx;
extern struct led_handle led_rx;

/* Exported function prototypes -------------------------------------------- */

void led_on(struct led_handle *handle);

void led_off(struct led_handle *handle);

void led_toggle(struct led_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LED_H_ */
