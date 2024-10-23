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

#include "usart.h"
#include "rcc.h"
#include "gpio.h"
#include "stm32f446xx_it.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик RCC */
extern struct rcc_handle rcc;

/* Обработчики GPIO RS485 */
extern struct gpio_handle gpio_rs485_usart1;
extern struct gpio_handle gpio_rs485_usart2;

/* Обработчик USART1 */
struct usart_handle usart1 = {
    .instance = USART1,
    .gpio_rs485 = &gpio_rs485_usart1,
};

/* Mutex и Event Group USART1 */
SemaphoreHandle_t usart1_mutex;
EventGroupHandle_t usart1_event_group;

/* Обработчик USART2 */
struct usart_handle usart2 = {
    .instance = USART2,
    .gpio_rs485 = &gpio_rs485_usart2,
};

/* Mutex и Event Group USART2 */
SemaphoreHandle_t usart2_mutex;
EventGroupHandle_t usart2_event_group;

/* Обработчик USART6 */
struct usart_handle usart6 = {
    .instance = USART6,
};

/* Mutex и Event Group USART6 */
SemaphoreHandle_t usart6_mutex;
EventGroupHandle_t usart6_event_group;

/* Private function prototypes --------------------------------------------- */

static void usart1_init(void);

static void usart2_init(void);

static void usart6_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать USART
 */
void usart_init(void)
{
    HAL_USART1_ENABLE_CLOCK();
    HAL_USART2_ENABLE_CLOCK();
    HAL_USART6_ENABLE_CLOCK();

    usart1_init();
    usart2_init();
    usart6_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать USART1
 */
static void usart1_init(void)
{
    usart1.init.frequency = rcc.apb2_clock;
    usart1.init.baudrate = 19200;
    usart1.init.oversampling = USART_OVERSAMPLING16;
    usart1.init.word_length = USART_WORD_9BIT;
    usart1.init.stop_bit = USART_1STOP_BIT;
    usart1.init.parity = USART_PARITY_EVEN;

    hal_usart_register_callback(&usart1,
                                USART_TRANSMIT_COMPLETED_CALLBACK,
                                USART_TransmitCompletedCallback);
    hal_usart_register_callback(&usart1,
                                USART_RECEIVE_COMPLETED_CALLBACK,
                                USART_ReceiveCompletedCallback);
    hal_usart_register_callback(&usart1,
                                USART_ERROR_CALLBACK,
                                USART_ErrorCallback);
    hal_usart_init(&usart1);
    hal_usart_enable(&usart1);
    hal_usart_enable_transiver(&usart1);
    hal_usart_enable_receiver(&usart1);

    NVIC_SetPriority(USART1_IRQn, 11);
    NVIC_EnableIRQ(USART1_IRQn);

    usart1_mutex = xSemaphoreCreateMutex();
    if (usart1_mutex == NULL)
        hal_error();

    usart1_event_group = xEventGroupCreate();
    if (usart1_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать USART2
 */
static void usart2_init(void)
{
    usart2.init.frequency = rcc.apb1_clock;
    usart2.init.baudrate = 19200;
    usart2.init.oversampling = USART_OVERSAMPLING16;
    usart2.init.word_length = USART_WORD_9BIT;
    usart2.init.stop_bit = USART_1STOP_BIT;
    usart2.init.parity = USART_PARITY_EVEN;

    hal_usart_register_callback(&usart2,
                                USART_TRANSMIT_COMPLETED_CALLBACK,
                                USART_TransmitCompletedCallback);
    hal_usart_register_callback(&usart2,
                                USART_RECEIVE_COMPLETED_CALLBACK,
                                USART_ReceiveCompletedCallback);
    hal_usart_register_callback(&usart2,
                                USART_ERROR_CALLBACK,
                                USART_ErrorCallback);
    hal_usart_init(&usart2);
    hal_usart_enable(&usart2);
    hal_usart_enable_transiver(&usart2);
    hal_usart_enable_receiver(&usart2);

    NVIC_SetPriority(USART2_IRQn, 11);
    NVIC_EnableIRQ(USART2_IRQn);

    usart2_mutex = xSemaphoreCreateMutex();
    if (usart2_mutex == NULL)
        hal_error();

    usart2_event_group = xEventGroupCreate();
    if (usart2_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать USART6
 */
static void usart6_init(void)
{
    usart6.init.frequency = rcc.apb2_clock;
    usart6.init.baudrate = 115200;
    usart6.init.oversampling = USART_OVERSAMPLING16;
    usart6.init.word_length = USART_WORD_8BIT;
    usart6.init.stop_bit = USART_1STOP_BIT;
    usart6.init.parity = USART_PARITY_NO;

    hal_usart_register_callback(&usart6,
                                USART_TRANSMIT_COMPLETED_CALLBACK,
                                USART_TransmitCompletedCallback);
    hal_usart_register_callback(&usart6,
                                USART_RECEIVE_COMPLETED_CALLBACK,
                                USART_ReceiveCompletedCallback);
    hal_usart_register_callback(&usart6,
                                USART_ERROR_CALLBACK,
                                USART_ErrorCallback);
    hal_usart_init(&usart6);
    hal_usart_enable(&usart6);
    hal_usart_enable_transiver(&usart6);
    hal_usart_enable_receiver(&usart6);

    NVIC_SetPriority(USART6_IRQn, 11);
    NVIC_EnableIRQ(USART6_IRQn);

    usart6_mutex = xSemaphoreCreateMutex();
    if (usart6_mutex == NULL)
        hal_error();

    usart6_event_group = xEventGroupCreate();
    if (usart6_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */
