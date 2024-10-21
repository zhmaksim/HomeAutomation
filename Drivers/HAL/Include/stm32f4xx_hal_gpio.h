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

#ifndef STM32F4XX_HAL_GPIO_H_
#define STM32F4XX_HAL_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование GPIOA
 */
#define HAL_GPIOA_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Msk)

/**
 * @brief           Включить тактирование GPIOB
 */
#define HAL_GPIOB_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Msk)

/**
 * @brief           Включить тактирование GPIOC
 */
#define HAL_GPIOC_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Msk)

/**
 * @brief           Включить тактирование GPIOD
 */
#define HAL_GPIOD_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Msk)

/**
 * @brief           Включить тактирование GPIOE
 */
#define HAL_GPIOE_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN_Msk)

/**
 * @brief           Включить тактирование GPIOF
 */
#define HAL_GPIOF_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN_Msk)

/**
 * @brief           Включить тактирование GPIOG
 */
#define HAL_GPIOG_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN_Msk)

/**
 * @brief           Включить тактирование GPIOH
 */
#define HAL_GPIOH_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных GPIO
 */
typedef GPIO_TypeDef gpio_t;


/**
 * @brief           Определение перечисления портов ввода-вывода GPIO
 */
enum gpio_pin {
    GPIO_PIN0,
    GPIO_PIN1,
    GPIO_PIN2,
    GPIO_PIN3,
    GPIO_PIN4,
    GPIO_PIN5,
    GPIO_PIN6,
    GPIO_PIN7,
    GPIO_PIN8,
    GPIO_PIN9,
    GPIO_PIN10,
    GPIO_PIN11,
    GPIO_PIN12,
    GPIO_PIN13,
    GPIO_PIN14,
    GPIO_PIN15,
};


/**
 * @brief           Определение перечисления состояний порта ввода-вывода GPIO
 */
enum gpio_state {
    GPIO_RESET,
    GPIO_SET,
};


/**
 * @brief           Определение перечисления режмов работы порта ввода-вывода GPIO
 */
enum gpio_mode {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_AF,
    GPIO_ANALOG,
};


/**
 * @brief           Определение перечисления типов порта вывода GPIO
 */
enum gpio_output_type {
    GPIO_PUSH_PULL,
    GPIO_OPEN_DRAIN,
};


/**
 * @brief           Определение перечисления скорости работы порта вывода GPIO
 */
enum gpio_output_speed {
    GPIO_LOW_SPEED,
    GPIO_MEDIUM_SPEED,
    GPIO_HIGH_SPEED,
    GPIO_VERY_HIGH_SPEED,
};


/**
 * @brief           Определение перечисления типов подтяжек сигналов порта ввода-вывода GPIO
 */
enum gpio_pupd {
    GPIO_NO_PULL,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
};


/**
 * @brief           Определение структуры данных инициализации GPIO
 */
struct gpio_init {
    enum gpio_mode          mode;               /*!< Режим работы:
                                                    - GPIO_INPUT
                                                    - GPIO_OUTPUT
                                                    - GPIO_AF
                                                    - GPIO_ANALOG */

    enum gpio_output_type   otype;              /*!< Тип вывода:
                                                    - GPIO_PUSH_PULL
                                                    - GPIO_OPEN_DRAIN */

    enum gpio_output_speed  ospeed;             /*!< Скорость работы вывода:
                                                    - GPIO_LOW_SPEED
                                                    - GPIO_MEDIUM_SPEED
                                                    - GPIO_HIGH_SPEED
                                                    - GPIO_VERY_HIGH_SPEED */

    enum gpio_pupd          pupd;               /*!< Подтяжка сигнала:
                                                    - GPIO_NO_PULL
                                                    - GPIO_PULL_UP
                                                    - GPIO_PULL_DOWN */

    uint8_t                 af;                 /*!< Альтернативная функция: 0-15 */
};


/**
 * @brief           Определение структуры данных обработчика GPIO
 */
struct gpio_handle {
    gpio_t                 *instance;           /*!< Указатель на структуру данных GPIO */

    enum gpio_pin           pin;                /*!< Номер порта ввода-вывода GPIO:
                                                    - GPIO_PIN0
                                                    - GPIO_PIN1
                                                    ...
                                                    - GPIO_PIN15 */

    struct gpio_init        init;               /*!< Настройки GPIO */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_gpio_init(struct gpio_handle *handle);

void hal_gpio_set_state(struct gpio_handle *handle, enum gpio_state state);

enum gpio_state hal_gpio_state(struct gpio_handle *handle);

void hal_gpio_toggle(struct gpio_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* HAL_GPIO_H_ */
