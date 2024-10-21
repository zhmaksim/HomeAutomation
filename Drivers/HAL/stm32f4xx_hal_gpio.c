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

#include "stm32f4xx_hal_gpio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_gpio_setup_mode(gpio_t *instance, enum gpio_pin pin, enum gpio_mode mode);

static void hal_gpio_setup_output_type(gpio_t *instance, enum gpio_pin pin, enum gpio_output_type otype);

static void hal_gpio_setup_output_speed(gpio_t *instance, enum gpio_pin pin, enum gpio_output_speed ospeed);

static void hal_gpio_setup_pupd(gpio_t *instance, enum gpio_pin pin, enum gpio_pupd pupd);

static void hal_gpio_setup_af(gpio_t *instance, enum gpio_pin pin, uint8_t af);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO
 *
 * @param[in]       handle: Указатель на структуру данных обработчика GPIO
 */
void hal_gpio_init(struct gpio_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_gpio_setup_mode(handle->instance, handle->pin, handle->init.mode);
    hal_gpio_setup_output_type(handle->instance, handle->pin, handle->init.otype);
    hal_gpio_setup_output_speed(handle->instance, handle->pin, handle->init.ospeed);
    hal_gpio_setup_pupd(handle->instance, handle->pin, handle->init.pupd);
    hal_gpio_setup_af(handle->instance, handle->pin, handle->init.af);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить режим работы GPIO
 *
 * @param[in]       instance: Указатель на структуру данных GPIO
 * @param[in]       pin: Номер порта ввода-вывода GPIO
 * @param[in]       mode: Режим работы GPIO
 */
static void hal_gpio_setup_mode(gpio_t *instance, enum gpio_pin pin, enum gpio_mode mode)
{
    MODIFY_REG(instance->MODER,
               HAL_BITMASK(0x03, pin * 2),
               HAL_BITMASK(mode, pin * 2));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить тип вывода GPIO
 *
 * @param[in]       instance: Указатель на структуру данных GPIO
 * @param[in]       pin: Номер порта ввода-вывода GPIO
 * @param[in]       otype: Тип вывода GPIO
 */
static void hal_gpio_setup_output_type(gpio_t *instance, enum gpio_pin pin, enum gpio_output_type otype)
{
    MODIFY_REG(instance->OTYPER,
               HAL_BITMASK(0x01, pin * 1),
               HAL_BITMASK(otype, pin * 1));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить скорость работы вывода GPIO
 *
 * @param[in]       instance: Указатель на структуру данных GPIO
 * @param[in]       pin: Номер порта ввода-вывода GPIO
 * @param[in]       ospeed: Скорость работы вывода GPIO
 */
static void hal_gpio_setup_output_speed(gpio_t *instance, enum gpio_pin pin, enum gpio_output_speed ospeed)
{
    MODIFY_REG(instance->OSPEEDR,
               HAL_BITMASK(0x03, pin * 2),
               HAL_BITMASK(ospeed, pin * 2));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить подтяжку сигнала GPIO
 *
 * @param[in]       instance: Указатель на структуру данных GPIO
 * @param[in]       pin: Номер порта ввода-вывода GPIO
 * @param[in]       pupd: Подтяжка сигнала GPIO
 */
static void hal_gpio_setup_pupd(gpio_t *instance, enum gpio_pin pin, enum gpio_pupd pupd)
{
    MODIFY_REG(instance->PUPDR,
               HAL_BITMASK(0x03, pin * 2),
               HAL_BITMASK(pupd, pin * 2));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить альтернативную функцию GPIO
 *
 * @param[in]       instance: Указатель на структуру данных GPIO
 * @param[in]       pin: Номер порта ввода-вывода GPIO
 * @param[in]       af: Номер альтернативной функции GPIO
 */
static void hal_gpio_setup_af(gpio_t *instance, enum gpio_pin pin, uint8_t af)
{
    if (pin < GPIO_PIN8) {
        MODIFY_REG(instance->AFR[0],
                   HAL_BITMASK(0x0F, pin * 4),
                   HAL_BITMASK(af, pin * 4));
    } else {
        MODIFY_REG(instance->AFR[1],
                   HAL_BITMASK(0x0F, (pin - 8) * 4),
                   HAL_BITMASK(af, (pin - 8) * 4));
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить состояние порта ввода-вывода GPIO
 *
 * @param[in]       handle: Указатель на структуру данных обработчика GPIO
 * @param[in]       state: Состояние порта ввода-вывода GPIO
 */
void hal_gpio_set_state(struct gpio_handle *handle, enum gpio_state state)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    if (state == GPIO_SET) {
        SET_BIT(handle->instance->BSRR, HAL_BITMASK(0x01, handle->pin));
    } else {
        SET_BIT(handle->instance->BSRR, HAL_BITMASK(0x10000, handle->pin));
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Состояние порта ввода-вывода GPIO
 *
 * @param[in]       handle: Указатель на структуру данных обработчика GPIO
 * @return          Состояние порта ввода-вывода GPIO @ref enum gpio_state
 */
enum gpio_state hal_gpio_state(struct gpio_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    return READ_BIT(handle->instance->IDR, HAL_BITMASK(0x01, handle->pin)) ?
            GPIO_SET : GPIO_RESET;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Переключить состояние порта ввода-вывода GPIO
 *
 * @param[in]       handle: Указатель на структуру данных обработчика GPIO
 */
void hal_gpio_toggle(struct gpio_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    XOR_BIT(handle->instance->ODR, HAL_BITMASK(0x01, handle->pin));
}
/* ------------------------------------------------------------------------- */
