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

#ifndef STM32F4XX_HAL_SYSTICK_H_
#define STM32F4XX_HAL_SYSTICK_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных SysTick
 */
typedef SysTick_Type systick_t;


/**
 * @brief           Определение перечисления источников тактирования SysTick
 */
enum systick_clock_source {
    SYSTICK_CPU_CLOCK_DIV8,
    SYSTICK_CPU_CLOCK,
};


/**
 * @brief           Определение перечисления идентификаторов функций обратного вызова SysTick
 */
enum systick_callback_id {
    SYSTICK_PERIOD_ELAPSED_CALLBACK,
};


/**
 * @brief           Определение структуры данных инициализации SysTick
 */
struct systick_init {
    uint32_t                    frequency;      /*!< Частота тактирования (Гц) */

    enum systick_clock_source   clksource;      /*!< Источник тактирования:
                                                    - SYSTICK_CPU_CLOCK_DIV8
                                                    - SYSTICK_CPU_CLOCK */
};


/**
 * @brief           Определение структуры данных обработчика SysTick
 */
struct systick_handle {
    systick_t              *instance;           /*!< Указатель на структуру данных SysTick */

    struct systick_init     init;               /*!< Настройки SysTick */

    volatile uint32_t       tick;               /*!< Таймер (мс) */

    /* --- */

    void (*period_elapsed_callback)(void);
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_systick_init(struct systick_handle *handle);

void hal_systick_it_handler(struct systick_handle *handle);

void hal_systick_start(struct systick_handle *handle);

void hal_systick_stop(struct systick_handle *handle);

uint32_t hal_systick_tick(struct systick_handle *handle);

void hal_systick_register_callback(struct systick_handle *handle,
                                   enum systick_callback_id callback_id,
                                   void (*callback)(void));

void hal_systick_unregister_callback(struct systick_handle *handle,
                                     enum systick_callback_id callback_id);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_SYSTICK_H_ */
