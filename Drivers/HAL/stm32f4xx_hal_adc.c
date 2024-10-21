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

#include "stm32f4xx_hal_adc.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_adc_setup_prescaler(adc_common_t *instance_common, enum adc_div div);

static void hal_adc_setup_resolution(adc_t *instance, enum adc_resolution resolution);

static void hal_adc_setup_alignment(adc_t *instance, enum adc_align align);

static void hal_adc_setup_end_of_conv(adc_t *instance);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать ADC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 */
void hal_adc_init(struct adc_handle* handle)
{
    assert(handle != NULL);
    assert(handle->instance_common != NULL);
    assert(handle->instance != NULL);

    hal_adc_setup_prescaler(handle->instance_common, handle->init.div);
    hal_adc_setup_resolution(handle->instance, handle->init.resolution);
    hal_adc_setup_alignment(handle->instance, handle->init.align);
    hal_adc_setup_end_of_conv(handle->instance);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить делитель частоты ADC
 *
 * @param[in]       div: Значение делителя ADC
 */
static void hal_adc_setup_prescaler(adc_common_t *instance_common, enum adc_div div)
{
    MODIFY_REG(instance_common->CCR,
               ADC_CCR_ADCPRE_Msk,
               div << ADC_CCR_ADCPRE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить разрешение ADC
 *
 * @param[in]       instance: Указатель на структуру данных ADC
 * @param[in]       resolution: Резрешение данных ADC
 */
static void hal_adc_setup_resolution(adc_t *instance, enum adc_resolution resolution)
{
    MODIFY_REG(instance->CR1,
               ADC_CR1_RES_Msk,
               resolution << ADC_CR1_RES_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить выравнивание данных ADC
 *
 * @param[in]       instance: Указатель на структуру данных ADC
 * @param[in]       align: Выравнивание данных ADC
 */
static void hal_adc_setup_alignment(adc_t *instance, enum adc_align align)
{
    MODIFY_REG(instance->CR2,
               ADC_CR2_ALIGN_Msk,
               align << ADC_CR2_ALIGN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить завершение преобразований ADC
 *
 * @param[in]       instance: Указатель на структуру данных ADC
 */
static void hal_adc_setup_end_of_conv(adc_t *instance)
{
    SET_BIT(instance->CR2, ADC_CR2_EOCS_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание ADC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 */
void hal_adc_it_handler(struct adc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_common != NULL);
    assert(handle->instance != NULL);

    /* Проверить статус ADC */
    uint32_t SR = READ_REG(handle->instance->SR);

    if (READ_BIT(SR, ADC_SR_OVR_Msk)) {
        CLEAR_BIT(handle->instance->SR, ADC_SR_OVR_Msk);

        if (handle->error_callback != NULL)
            handle->error_callback(handle);
    } else if (READ_BIT(SR, ADC_SR_EOC_Msk)) {
        CLEAR_BIT(handle->instance->SR, ADC_SR_EOC_Msk);

        if (handle->measure_completed_callback != NULL)
            handle->measure_completed_callback(handle);
    }

    /* Выключить прерывания ADC */
    CLEAR_BIT(handle->instance->CR1,
              ADC_CR1_OVRIE_Msk
            | ADC_CR1_EOCIE_Msk);

    /* Выключить ADC */
    CLEAR_BIT(handle->instance->CR2, ADC_CR2_ADON_Msk);

    /* Выключить преобразование ADC */
    switch (handle->measure_id) {
        case ADC_MEASURE_VREF:
        case ADC_MEASURE_TEMP:
            CLEAR_BIT(handle->instance_common->CCR, ADC_CCR_TSVREFE_Msk);
            break;
        case ADC_MEASURE_VBAT:
            CLEAR_BIT(handle->instance_common->CCR, ADC_CCR_VBATE_Msk);
            break;
        default:
            break;
    }

    /* Сбросить идентификатор преобразования */
    handle->measure_id = ADC_MEASURE_NONE;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Получить значение ADC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 * @return          Значение ADC
 */
uint16_t hal_adc_value(struct adc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    return READ_REG(handle->instance->DR);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить преобразование VREF (ADC1)
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_adc_setup_measure_vref(struct adc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_common != NULL);
    assert(handle->instance != NULL);

    /* Проверить структуру данных ADC */
    if (handle->instance != ADC1)
        return HAL_ERROR;

    /* Включить преобразование VREF */
    SET_BIT(handle->instance_common->CCR, ADC_CCR_TSVREFE_Msk);

    /* Настроить Cycles = 480 для VREF */
    SET_BIT(handle->instance->SMPR1, ADC_SMPR1_SMP17_Msk);

    /* Настроить Rank = 1 для VREF */
    MODIFY_REG(handle->instance->SQR3,
               ADC_SQR3_SQ1_Msk,
               0x11 << ADC_SQR3_SQ1_Pos);

    /* Включить ADC */
    SET_BIT(handle->instance->CR2, ADC_CR2_ADON_Msk);

    /* Установить идентификатор преобразования */
    handle->measure_id = ADC_MEASURE_VREF;

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить преобразование TEMP (ADC1)
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_adc_setup_measure_temp(struct adc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_common != NULL);
    assert(handle->instance != NULL);

    /* Проверить структуру данных ADC */
    if (handle->instance != ADC1)
        return HAL_ERROR;

    /* Включить преобразование TEMP */
    SET_BIT(handle->instance_common->CCR, ADC_CCR_TSVREFE_Msk);

    /* Настроить Cycles = 480 для TEMP */
    SET_BIT(handle->instance->SMPR1, ADC_SMPR1_SMP16_Msk);

    /* Настроить Rank = 1 для TEMP */
    MODIFY_REG(handle->instance->SQR3,
               ADC_SQR3_SQ1_Msk,
               0x10 << ADC_SQR3_SQ1_Pos);

    /* Включить ADC */
    SET_BIT(handle->instance->CR2, ADC_CR2_ADON_Msk);

    /* Установить идентификатор преобразования */
    handle->measure_id = ADC_MEASURE_TEMP;

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить преобразование VBAT (ADC1)
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_adc_setup_measure_vbat(struct adc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance_common != NULL);
    assert(handle->instance != NULL);

    /* Проверить структуру данных ADC */
    if (handle->instance != ADC1)
        return HAL_ERROR;

    /* Включить преобразование VBAT */
    SET_BIT(handle->instance_common->CCR, ADC_CCR_VBATE_Msk);

    /* Настроить Cycles = 480 для VBAT */
    SET_BIT(handle->instance->SMPR1, ADC_SMPR1_SMP18_Msk);

    /* Настроить Rank = 1 для VBAT */
    MODIFY_REG(handle->instance->SQR3,
               ADC_SQR3_SQ1_Msk,
               0x12 << ADC_SQR3_SQ1_Pos);

    /* Включить ADC */
    SET_BIT(handle->instance->CR2, ADC_CR2_ADON_Msk);

    /* Установить идентификатор преобразования */
    handle->measure_id = ADC_MEASURE_VBAT;

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Запустить преобразование ADC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 */
void hal_adc_measure_start_it(struct adc_handle* handle)
{
    /* Включить прерывания ADC */
    SET_BIT(handle->instance->CR1,
            ADC_CR1_OVRIE_Msk
          | ADC_CR1_EOCIE_Msk);

    /* Очистить статусы ADC */
    CLEAR_BIT(handle->instance->SR,
              ADC_SR_STRT_Msk
            | ADC_SR_EOC_Msk
            | ADC_SR_OVR_Msk);

    /* Запустить преобразование */
    SET_BIT(handle->instance->CR2, ADC_CR2_SWSTART_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова ADC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 * @param[in]       callback_id: Идентификатор функции обратного вызова ADC
 * @param[in]       callback: Указатель на функцию обратного вызова ADC
 */
void hal_adc_register_callback(struct adc_handle *handle,
                               enum adc_callback_id callback_id,
                               void (*callback)(struct adc_handle *handle))
{
    assert(handle != NULL);

    switch (callback_id) {
        case ADC_MEASURE_COMPLETED_CALLBACK:
            handle->measure_completed_callback = callback;
            break;
        case ADC_ERROR_CALLBACK:
            handle->error_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова ADC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика ADC
 * @param[in]       callback_id: Идентификатор функции обратного вызова
 */
void hal_adc_unregister_callback(struct adc_handle *handle,
                                 enum adc_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case ADC_MEASURE_COMPLETED_CALLBACK:
            handle->measure_completed_callback = NULL;
            break;
        case ADC_ERROR_CALLBACK:
            handle->error_callback = NULL;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
