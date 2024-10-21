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

#include "stm32f4xx_hal_systick.h"

/* Private macros ---------------------------------------------------------- */

/**
 * @brief           Сбросить регистр управления SysTick
 *
 * @param[in]       instance: Указатель на структуру данных SysTick
 */
#define HAL_SYSTICK_RESET_CTRL(instance) \
    CLEAR_REG((instance)->CTRL)

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_systick_setup_clock_source(systick_t *instance, enum systick_clock_source clksource);

static uint32_t hal_systick_compute_reload_value(systick_t *instance, uint32_t frequency);

static void hal_systick_setup_reload_value(systick_t *instance, uint32_t value);

static void hal_systick_setup_current_value(systick_t *instance, uint32_t value);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 */
void hal_systick_init(struct systick_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    uint32_t reload_value;

    HAL_SYSTICK_RESET_CTRL(handle->instance);

    hal_systick_setup_clock_source(handle->instance, handle->init.clksource);
    reload_value = hal_systick_compute_reload_value(handle->instance, handle->init.frequency);
    hal_systick_setup_reload_value(handle->instance, reload_value);
    hal_systick_setup_current_value(handle->instance, 0);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить источник тактирования SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 * @param[in]       clksource: Источник тактирования
 */
static void hal_systick_setup_clock_source(systick_t *instance, enum systick_clock_source clksource)
{
    MODIFY_REG(instance->CTRL,
               SysTick_CTRL_CLKSOURCE_Msk,
               clksource << SysTick_CTRL_CLKSOURCE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Рассчитать значение перезагрузки счетчика SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 * @param[in]       frequency: Частота тактирования SysTick (Гц)
 * @return          Значение перезагрузки счетчика
 */
static uint32_t hal_systick_compute_reload_value(systick_t *instance, uint32_t frequency)
{
    return (frequency / 1000) - 1;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить значение перезагрузки счетчика SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 * @param[in]       value: Значение перезагрузки счетчика
 */
static void hal_systick_setup_reload_value(systick_t *instance, uint32_t value)
{
    WRITE_REG(instance->LOAD, value);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить текущее значение счетчика SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 * @param[in]       value: Текущее значение счетчика
 */
static void hal_systick_setup_current_value(systick_t *instance, uint32_t value)
{
    WRITE_REG(instance->VAL, value);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывания SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 */
void hal_systick_it_handler(struct systick_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    if (READ_BIT(handle->instance->CTRL, SysTick_CTRL_COUNTFLAG_Msk)) {
        ++handle->tick;

        if (handle->period_elapsed_callback != NULL)
            handle->period_elapsed_callback();
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Запустить SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 */
void hal_systick_start(struct systick_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_systick_setup_current_value(handle->instance, 0);

    SET_BIT(handle->instance->CTRL,
            SysTick_CTRL_TICKINT_Msk
          | SysTick_CTRL_ENABLE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Остановить SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 */
void hal_systick_stop(struct systick_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CTRL,
              SysTick_CTRL_TICKINT_Msk
            | SysTick_CTRL_ENABLE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Значение таймера SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 * @return          Текущее значение таймера (мс)
 */
uint32_t hal_systick_tick(struct systick_handle *handle)
{
    assert(handle != NULL);

    return handle->tick;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 * @param[in]       callback_id: Идентификатор функции обратного вызова
 * @param[in]       callback: Указатель на функцию обратного вызова
 */
void hal_systick_register_callback(struct systick_handle *handle,
                                   enum systick_callback_id callback_id,
                                   void (*callback)(void))
{
    assert(handle != NULL);

    switch (callback_id) {
        case SYSTICK_PERIOD_ELAPSED_CALLBACK:
            handle->period_elapsed_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова SysTick
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SysTick
 * @param[in]       callback_id: Идентификатор функции обратного вызова
 */
void hal_systick_unregister_callback(struct systick_handle* handle,
                                     enum systick_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case SYSTICK_PERIOD_ELAPSED_CALLBACK:
            handle->period_elapsed_callback = NULL;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
