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

#include "sensors.h"

/* Private constants ------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик ADC1 */
extern struct adc_handle adc1;

/* Mutex и Event Group ADC1 */
extern SemaphoreHandle_t adc1_mutex;
extern EventGroupHandle_t adc1_event_group;

/* Обработчик Sensors */
struct sensors_handle sensors = {
    .adc = &adc1,
};

/* Private function prototypes --------------------------------------------- */

static void sensors_process(void *arg);

static uint16_t sensors_adc_measure_vref(struct sensors_handle *handle);

static uint16_t sensors_adc_measure_temp(struct sensors_handle *handle);

static uint16_t sensors_adc_measure_vbat(struct sensors_handle *handle);

static float sensors_compute_vref(struct sensors_handle *handle);

static float sensors_compute_temp(struct sensors_handle *handle);

static float sensors_compute_vbat(struct sensors_handle *handle);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать датчики
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 */
void sensors_init(struct sensors_handle *handle)
{
    assert(handle != NULL);
    assert(handle->adc != NULL);

    /* Назначить Mutex и Event Group */
    if (handle->adc == &adc1) {
        handle->adc_mutex = adc1_mutex;
        handle->adc_event_group = adc1_event_group;
    }

    xTaskCreate(sensors_process,
                "sensors",
                configMINIMAL_STACK_SIZE,
                (void *) handle,
                tskIDLE_PRIORITY + 1,
                NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать события датчиков
 *
 * @param[in]       arg: Указатель на параметры
 */
static void sensors_process(void *arg)
{
    static const TickType_t frequency = pdMS_TO_TICKS(300);

    struct sensors_handle *handle = arg;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, frequency);

        for (uint32_t measure = 0; measure < SENSORS_MEASURE_COUNT; measure++) {
            handle->vref_adc_value[measure] = sensors_adc_measure_vref(handle);
            handle->temp_adc_value[measure] = sensors_adc_measure_temp(handle);
            handle->vbat_adc_value[measure] = sensors_adc_measure_vbat(handle);
        }

        taskENTER_CRITICAL();
        {
            handle->vref = sensors_compute_vref(handle);
            handle->temp = sensors_compute_temp(handle);
            handle->vbat = sensors_compute_vbat(handle);

            handle->state = SENSORS_READY;
        }
        taskEXIT_CRITICAL();
    }

    vTaskDelete(NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать успешное завершение измерения ADC датчиков
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 */
void sensors_adc_measure_completed_it_handler(struct sensors_handle *handle)
{
    assert(handle != NULL);
    assert(handle->adc_event_group != NULL);

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(handle->adc_event_group,
                                  ADC_EV_MEASURE_CPLT,
                                  &higher_priority_task_woken) == pdPASS) {
        portYIELD_FROM_ISR( higher_priority_task_woken );
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать ошибку ADC датчиков
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 */
void sensors_adc_error_it_handler(struct sensors_handle *handle)
{
    assert(handle != NULL);
    assert(handle->adc_event_group != NULL);

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(handle->adc_event_group,
                                  ADC_EV_ERR,
                                  &higher_priority_task_woken) == pdPASS) {
        portYIELD_FROM_ISR( higher_priority_task_woken );
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Измерение ADC Vref
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение ADC Vref
 */
static uint16_t sensors_adc_measure_vref(struct sensors_handle *handle)
{
    uint16_t adc_value = 0;

    if (xSemaphoreTake(handle->adc_mutex, pdMS_TO_TICKS(50)) != pdPASS)
        return adc_value;

    /* Настроить преобразование */
    hal_adc_setup_measure_vref(handle->adc);
    /* Ожидание стабилизации */
    vTaskDelay(pdMS_TO_TICKS(1));
    /* Запустить преобразование */
    hal_adc_measure_start_it(handle->adc);
    /* Ожидание завершения преобразования */
    EventBits_t bits = xEventGroupWaitBits(handle->adc_event_group,
                                           ADC_EV_MEASURE_CPLT | ADC_EV_ERR,
                                           pdTRUE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (READ_BIT(bits, ADC_EV_MEASURE_CPLT_Msk))
        adc_value = hal_adc_value(handle->adc);

    xSemaphoreGive(handle->adc_mutex);

    return adc_value;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Измерение ADC Temp
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение ADC Temp
 */
static uint16_t sensors_adc_measure_temp(struct sensors_handle *handle)
{
    uint16_t adc_value = 0;

    if (xSemaphoreTake(handle->adc_mutex, pdMS_TO_TICKS(50)) != pdPASS)
        return adc_value;

    /* Настроить преобразование */
    hal_adc_setup_measure_temp(handle->adc);
    /* Ожидание стабилизации */
    vTaskDelay(pdMS_TO_TICKS(1));
    /* Запустить преобразование */
    hal_adc_measure_start_it(handle->adc);
    /* Ожидание завершения преобразования */
    EventBits_t bits = xEventGroupWaitBits(handle->adc_event_group,
                                           ADC_EV_MEASURE_CPLT | ADC_EV_ERR,
                                           pdTRUE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (READ_BIT(bits, ADC_EV_MEASURE_CPLT_Msk))
        adc_value = hal_adc_value(handle->adc);

    xSemaphoreGive(handle->adc_mutex);

    return adc_value;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Измерение ADC Vbat
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение ADC Vbat
 */
static uint16_t sensors_adc_measure_vbat(struct sensors_handle *handle)
{
    uint16_t adc_value = 0;

    if (xSemaphoreTake(handle->adc_mutex, pdMS_TO_TICKS(50)) != pdPASS)
        return adc_value;

    /* Настроить преобразование */
    hal_adc_setup_measure_vbat(handle->adc);
    /* Ожидание стабилизации */
    vTaskDelay(pdMS_TO_TICKS(1));
    /* Запустить преобразование */
    hal_adc_measure_start_it(handle->adc);
    /* Ожидание завершения преобразования */
    EventBits_t bits = xEventGroupWaitBits(handle->adc_event_group,
                                           ADC_EV_MEASURE_CPLT | ADC_EV_ERR,
                                           pdTRUE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (READ_BIT(bits, ADC_EV_MEASURE_CPLT_Msk))
        adc_value = hal_adc_value(handle->adc);

    xSemaphoreGive(handle->adc_mutex);

    return adc_value;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Рассчитать значение опорного напряжения
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение опорного напряжения
 */
static float sensors_compute_vref(struct sensors_handle *handle)
{
    float sum = 0.0;
    uint32_t count = 0;

    for (uint32_t measure = 0; measure < SENSORS_MEASURE_COUNT; measure++) {
        if (handle->vref_adc_value[measure]) {
            sum += (float)(3300 * ADC_VREF_CAL) / handle->vref_adc_value[measure] / 1000.0;
            count++;
        }
    }

    if (count > 0) {
        return (sum / count);
    } else {
        return 3.3;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Рассчитать значение температуры
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение температуры
 */
static float sensors_compute_temp(struct sensors_handle *handle)
{
    float cal1 = (3.3 / sensors.vref) * ADC_TEMP_CAL1;
    float cal2 = (3.3 / sensors.vref) * ADC_TEMP_CAL2;

    float slope = (110.0 - 30.0) / (cal2 - cal1);

    float sum = 0.0;
    uint32_t count = 0;

    for (uint32_t measure = 0; measure < SENSORS_MEASURE_COUNT; measure++) {
        if (handle->temp_adc_value[measure]) {
            sum += slope * ((float) handle->temp_adc_value[measure] - cal1) + 30.0;
            count++;
        }
    }

    if (count > 0) {
        return (sum / count);
    } else {
        return -255.0;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Рассчитать значение напряжения батарейки
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение напряжения батарейки
 */
static float sensors_compute_vbat(struct sensors_handle *handle)
{
    float sum = 0.0;
    uint32_t count = 0;

    for (uint32_t measure = 0; measure < SENSORS_MEASURE_COUNT; measure++) {
        if (handle->vbat_adc_value[measure]) {
            sum += (handle->vref * handle->vbat_adc_value[measure] * 4) / 4095.0;
            count++;
        }
    }

    if (count > 0) {
        return (sum / count);
    } else {
        return 0.0;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Состояние показаний датчиков
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Состояние @ret enum sensors_state
 */
enum sensors_state sensors_state(struct sensors_handle *handle)
{
    assert(handle != NULL);

    return handle->state;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Опорное напряжение
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение опорного напряжения
 */
float sensors_vref(struct sensors_handle *handle)
{
    assert(handle != NULL);

    return handle->vref;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Температура
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение температуры
 */
float sensors_temp(struct sensors_handle *handle)
{
    assert(handle != NULL);

    return handle->temp;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Напряжение батарейки
 *
 * @param[in]       handle: Указатель на структуру данных обработчика датчиков
 * @return          Значение напряжения батарейки
 */
float sensors_vbat(struct sensors_handle *handle)
{
    assert(handle != NULL);

    return handle->vbat;
}
/* ------------------------------------------------------------------------- */
