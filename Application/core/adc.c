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

#include "adc.h"
#include "stm32f446xx_it.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик ADC1 */
struct adc_handle adc1 = {
    .instance_common = ADC123_COMMON,
    .instance = ADC1,
};

/* Mutex и Event Group ADC1 */
SemaphoreHandle_t adc1_mutex;
EventGroupHandle_t adc1_event_group;

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать ADC
 */
void adc_init(void)
{
    HAL_ADC1_ENABLE_CLOCK();

    adc1.init.div = ADC_DIV4;
    adc1.init.resolution = ADC_12BIT;
    adc1.init.align = ADC_RIGHT_ALIGNMENT;

    hal_adc_register_callback(&adc1,
                              ADC_MEASURE_COMPLETED_CALLBACK,
                              ADC_MeasureCompletedCallback);
    hal_adc_register_callback(&adc1,
                              ADC_ERROR_CALLBACK,
                              ADC_ErrorCallback);
    hal_adc_init(&adc1);

    NVIC_SetPriority(ADC_IRQn, 15);
    NVIC_EnableIRQ(ADC_IRQn);

    adc1_mutex = xSemaphoreCreateMutex();
    if (adc1_mutex == NULL)
        hal_error();

    adc1_event_group = xEventGroupCreate();
    if (adc1_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */
