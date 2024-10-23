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

#ifndef STM32F4XX_HAL_TIM_H_
#define STM32F4XX_HAL_TIM_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование TIM6
 */
#define HAL_TIM6_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN_Msk)

/**
 * @brief       Включить тактирование TIM7
 */
#define HAL_TIM7_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных TIM
 */
typedef TIM_TypeDef tim_t;


/**
 * @brief           Определение перечисления идентификаторов функций обратного вызова TIM
 */
enum tim_callback_id {
    TIM_PERIOD_ELAPSED_CALLBACK,
};


/**
 * @brief           Определение структуры данных инициализации TIM
 */
struct tim_init {
    uint32_t    prescaler;                      /*!< Делитель частоты таймера */

    uint32_t    reload;                         /*!< Значение перезагрузки счетчика таймера */
};


/**
 * @brief           Определение структуры данных обработчика TIM
 */
struct tim_handle {
    tim_t              *instance;               /*!< Указатель на структуру данных TIM */

    struct tim_init     init;                   /*!< Настройки TIM */

    /* --- */

    void (*period_elapsed_callback)(struct tim_handle *handle);
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_tim_init(struct tim_handle *handle);

void hal_tim_it_handler(struct tim_handle *handle);

void hal_tim_start_it(struct tim_handle *handle);

void hal_tim_stop_it(struct tim_handle *handle);

void hal_tim_register_callback(struct tim_handle *handle,
                               enum tim_callback_id callback_id,
                               void (*callback)(struct tim_handle *handle));

void hal_tim_unregister_callback(struct tim_handle *handle,
                                 enum tim_callback_id callback_id);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_TIM_H_ */
