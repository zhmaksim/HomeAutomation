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

#ifndef STM32F4XX_HAL_DMA_H_
#define STM32F4XX_HAL_DMA_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование DMA1
 */
#define HAL_DMA1_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN_Msk)

/**
 * @brief           Включить тактирование DMA2
 */
#define HAL_DMA2_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных DMA
 */
typedef DMA_TypeDef dma_t;


/**
 * @brief           Определение структуры данных потока DMA
 */
typedef DMA_Stream_TypeDef dma_stream_t;


/**
 * @brief           Определение перечисления каналов DMA
 */
enum dma_channel {
    DMA_CHANNEL0,
    DMA_CHANNEL1,
    DMA_CHANNEL2,
    DMA_CHANNEL3,
    DMA_CHANNEL4,
    DMA_CHANNEL5,
    DMA_CHANNEL6,
    DMA_CHANNEL7,
};


/**
 * @brief           Определение перечисления приоритетов DMA
 */
enum dma_priority {
    DMA_LOW_PRIORITY,
    DMA_MEDIUM_PRIORITY,
    DMA_HIGH_PRIORITY,
    DMA_VERY_HIGH_PRIORITY,
};


/**
 * @brief           Определение перечисления направления передачи данных DMA
 */
enum dma_transfer_direction {
    DMA_PERIPHERAL_TO_MEMORY,
    DMA_MEMORY_TO_PERIPHERAL,
    DMA_MEMORY_TO_MEMORY,
};


/**
 * @brief           Определение перечисления размера данных DMA
 */
enum dma_data_size {
    DMA_8BIT,
    DMA_16BIT,
    DMA_32BIT,
};


/**
 * @brief           Определение перечисления идентификаторов функций обратного вызова DMA
 */
enum dma_callback_id {
    DMA_TRANSFER_COMPLETED_CALLBACK,
    DMA_TRANSFER_ERROR_CALLBACK,
};


/**
 * @brief           Определение структуры данных инициализации DMA
 */
struct dma_init {
    enum dma_channel                channel;                    /*!< Номер канала:
                                                                    - DMA_CHANNEL0
                                                                    - DMA_CHANNEL1
                                                                    ...
                                                                    - DMA_CHANNEL7 */

    enum dma_priority               priority;                   /*!< Приоритет:
                                                                    - DMA_LOW_PRIORITY
                                                                    - DMA_MEDIUM_PRIORITY
                                                                    - DMA_HIGH_PRIORITY
                                                                    - DMA_VERY_HIGH_PRIORITY */

    enum dma_transfer_direction     transfer_direction;         /*!< Направление передачи данных:
                                                                    - DMA_PERIPHERAL_TO_MEMORY
                                                                    - DMA_MEMORY_TO_PERIPHERAL
                                                                    - DMA_MEMORY_TO_MEMORY */

    enum dma_data_size              peripheral_data_size;       /*!< Размер данных периферии:
                                                                    - DMA_8BIT
                                                                    - DMA_16BIT
                                                                    - DMA_32BIT */

    hal_state_t                     peripheral_inc_mode;        /*!< Режим инкремента периферии:
                                                                    - HAL_ENABLE
                                                                    - HAL_DISABLE */

    enum dma_data_size              memory_data_size;           /*!< Размер данных памяти:
                                                                    - DMA_8BIT
                                                                    - DMA_16BIT
                                                                    - DMA_32BIT */

    hal_state_t                     memory_inc_mode;            /*!< Режим инкремента памяти:
                                                                    - HAL_ENABLE
                                                                    - HAL_DISABLE */

    hal_state_t                     circ_mode;                  /*!< Циклический режим:
                                                                    - HAL_ENABLE
                                                                    - HAL_DISABLE */
};


/**
 * @brief           Определение структуры данных обработчика DMA
 */
struct dma_handle {
    dma_t                  *instance;           /*!< Указатель на структуру данных DMA */

    dma_stream_t           *instance_stream;    /*!< Указатель на структуру данных потока DMA */

    uint8_t                 stream_nb;          /*!< Номер потока */

    struct dma_init         init;               /*!< Настройки DMA */

    /* --- */

    void (*transfer_completed_callback)(struct dma_handle *handle);

    void (*transfer_error_callback)(struct dma_handle *handle);
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_dma_init(struct dma_handle *handle);

void hal_dma_it_handler(struct dma_handle *handle);

void hal_dma_enable(struct dma_handle *handle);

void hal_dma_disable(struct dma_handle *handle);

hal_status_t hal_dma_transfer_it(struct dma_handle *handle,
                                 void *peripheral_data,
                                 void *memory_data,
                                 size_t size);

void hal_dma_register_callback(struct dma_handle *handle,
                               enum dma_callback_id callback_id,
                               void (*callback)(struct dma_handle *handle));

void hal_dma_unregister_callback(struct dma_handle *handle,
                                 enum dma_callback_id callback_id);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_DMA_H_ */
