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

#ifndef STM32F4XX_HAL_ADC_H_
#define STM32F4XX_HAL_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование ADC1
 */
#define HAL_ADC1_ENABLE_CLOCK() \
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN_Msk)

/* Exported constants ------------------------------------------------------ */

#define ADC_VREF_CAL        (*(uint16_t*)(0x1FFF7A2A))
#define ADC_TEMP_CAL1       (*(uint16_t*)(0x1FFF7A2C))
#define ADC_TEMP_CAL2       (*(uint16_t*)(0x1FFF7A2E))

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных ADC
 */
typedef ADC_Common_TypeDef adc_common_t;


/**
 * @brief           Определение структуры данных ADC
 */
typedef ADC_TypeDef adc_t;


/**
 * @brief           Определение перечисления делителей ADC
 */
enum adc_div {
    ADC_DIV2,
    ADC_DIV4,
    ADC_DIV6,
    ADC_DIV8,
};


/**
 * @brief           Определение перечисления разрешения ADC
 */
enum adc_resolution {
    ADC_12BIT,
    ADC_10BIT,
    ADC_8BIT,
    ADC_6BIT,
};


/**
 * @brief           Определение перечисления выравнивания данных ADC
 */
enum adc_align {
    ADC_RIGHT_ALIGNMENT,
    ADC_LEFT_ALIGNMENT,
};


/**
 * @brief           Определение перечисления идентификаторов преобразования ADC
 */
enum adc_measure_id {
    ADC_MEASURE_NONE,
    ADC_MEASURE_VREF,
    ADC_MEASURE_TEMP,
    ADC_MEASURE_VBAT,
};


/**
 * @biref           Определение перечисления идентификаторов функций обратного вызова ADC
 */
enum adc_callback_id {
    ADC_MEASURE_COMPLETED_CALLBACK,
    ADC_ERROR_CALLBACK,
};


/**
 * @brief           Определение структуры данных инициализации ADC
 */
struct adc_init {
    enum adc_div            div;                /*!< Делитель ADC:
                                                    - ADC_DIV2
                                                    - ADC_DIV4
                                                    - ADC_DIV6
                                                    - ADC_DIV8 */

    enum adc_resolution     resolution;         /*!< Разрешение ADC:
                                                    - ADC_12BIT
                                                    - ADC_10BIT
                                                    - ADC_8BIT
                                                    - ADC_6BIT */

    enum adc_align          align;              /*!< Выравнивание данных ADC:
                                                    - ADC_RIGHT_ALIGNMENT
                                                    - ADC_LEFT_ALIGNMENT */
};


/**
 * @brief           Определение структуры данных обработчика ADC
 */
struct adc_handle {
    adc_common_t                   *instance_common;    /*!< Указатель на структуру данных ADC */

    adc_t                          *instance;           /*!< Указатель на структуру данных ADC */

    struct adc_init                 init;               /*!< Настройки ADC */

    volatile enum adc_measure_id    measure_id;         /*!< Идентификатор преобразования */

    /* --- */

    void (*measure_completed_callback)(struct adc_handle *handle);

    void (*error_callback)(struct adc_handle *handle);
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_adc_init(struct adc_handle *handle);

void hal_adc_it_handler(struct adc_handle *handle);

uint16_t hal_adc_value(struct adc_handle *handle);

hal_status_t hal_adc_setup_measure_vref(struct adc_handle *handle);

hal_status_t hal_adc_setup_measure_temp(struct adc_handle *handle);

hal_status_t hal_adc_setup_measure_vbat(struct adc_handle *handle);

void hal_adc_measure_start_it(struct adc_handle *handle);

void hal_adc_register_callback(struct adc_handle *handle,
                               enum adc_callback_id callback_id,
                               void (*callback)(struct adc_handle *handle));

void hal_adc_unregister_callback(struct adc_handle *handle,
                                 enum adc_callback_id callback_id);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_ADC_H_ */
