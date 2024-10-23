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

#include "stm32f4xx_hal_tim.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_tim_setup_prescaler(tim_t *instance, uint32_t prescaler);

static void hal_tim_setup_reload(tim_t *instance, uint32_t reload);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать TIM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика TIM
 */
void hal_tim_init(struct tim_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_tim_setup_prescaler(handle->instance, handle->init.prescaler);
    hal_tim_setup_reload(handle->instance, handle->init.reload);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить делитель частоты счетчика TIM
 *
 * @param[in]       instance: Указатель на структуру данных TIM
 * @param[in]       prescaler: Значение делителя
 */
static void hal_tim_setup_prescaler(tim_t *instance, uint32_t prescaler)
{
    WRITE_REG(instance->PSC, prescaler);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить значение перезагрузки счетчика TIM
 *
 * @param[in]       instance: Указатель на структуру данных TIM
 * @param[in]       reload: Значение перезагрузки
 */
static void hal_tim_setup_reload(tim_t *instance, uint32_t reload)
{
    WRITE_REG(instance->ARR, reload);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание TIM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика TIM
 */
void hal_tim_it_handler(struct tim_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить статус TIM */
    if (READ_BIT(handle->instance->SR, TIM_SR_UIF_Msk)) {
        /* Сбросить статус TIM */
        CLEAR_BIT(handle->instance->SR, TIM_SR_UIF_Msk);

        if (handle->period_elapsed_callback != NULL)
            handle->period_elapsed_callback(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Запустить TIM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика TIM
 */
void hal_tim_start_it(struct tim_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Включить прерывание */
    SET_BIT(handle->instance->DIER, TIM_DIER_UIE_Msk);
    /* Включить таймер */
    SET_BIT(handle->instance->CR1, TIM_CR1_CEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Остановить TIM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика TIM
 */
void hal_tim_stop_it(struct tim_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Выключить прерывание */
    CLEAR_BIT(handle->instance->DIER, TIM_DIER_UIE_Msk);
    /* Выключить таймер */
    CLEAR_BIT(handle->instance->CR1, TIM_CR1_CEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова TIM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика TIM
 * @param[in]       callback_id: Идентификатор функции обратного вызова TIM
 * @param[in]       callback: Указатель на функцию обратного вызова TIM
 */
void hal_tim_register_callback(struct tim_handle *handle,
                               enum tim_callback_id callback_id,
                               void (*callback)(struct tim_handle *handle))
{
    assert(handle != NULL);

    switch (callback_id) {
        case TIM_PERIOD_ELAPSED_CALLBACK:
            handle->period_elapsed_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова TIM
 *
 * @param[in]       handle: Указатель на структуру данных обработчика TIM
 * @param[in]       callback_id: Идентификатор функции обратного вызова TIM
 */
void hal_tim_unregister_callback(struct tim_handle *handle,
                                 enum tim_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case TIM_PERIOD_ELAPSED_CALLBACK:
            handle->period_elapsed_callback = NULL;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
