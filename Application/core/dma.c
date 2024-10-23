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

#include "dma.h"
#include "stm32f446xx_it.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик DMA2 Stream 2 */
struct dma_handle dma2_stream2 = {
    .instance = DMA2,
    .instance_stream = DMA2_Stream2,
    .stream_nb = 2,
};

/* Обработчик DMA2 Stream 3 */
struct dma_handle dma2_stream3 = {
    .instance = DMA2,
    .instance_stream = DMA2_Stream3,
    .stream_nb = 3,
};

/* Обработчик DMA1 Stream 3 */
struct dma_handle dma1_stream3 = {
    .instance = DMA1,
    .instance_stream = DMA1_Stream3,
    .stream_nb = 3,
};

/* Обработчик DMA1 Stream 4 */
struct dma_handle dma1_stream4 = {
    .instance = DMA1,
    .instance_stream = DMA1_Stream4,
    .stream_nb = 4,
};

/* Обработчик DMA1 Stream 0 */
struct dma_handle dma1_stream0 = {
    .instance = DMA1,
    .instance_stream = DMA1_Stream0,
    .stream_nb = 0,
};

/* Обработчик DMA1 Stream 7 */
struct dma_handle dma1_stream7 = {
    .instance = DMA1,
    .instance_stream = DMA1_Stream7,
    .stream_nb = 7,
};

/* Private function prototypes --------------------------------------------- */

static void dma_spi1_init(void);

static void dma_spi2_init(void);

static void dma_spi3_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать DMA
 */
void dma_init(void)
{
    HAL_DMA1_ENABLE_CLOCK();
    HAL_DMA2_ENABLE_CLOCK();

    dma_spi1_init();
    dma_spi2_init();
    dma_spi3_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать DMA SPI1
 */
static void dma_spi1_init(void)
{
    /* DMA2 Stream 2 - SPI RX */
    dma2_stream2.init.channel = DMA_CHANNEL3;
    dma2_stream2.init.priority = DMA_HIGH_PRIORITY;
    dma2_stream2.init.transfer_direction = DMA_PERIPHERAL_TO_MEMORY;
    dma2_stream2.init.peripheral_data_size = DMA_8BIT;
    dma2_stream2.init.peripheral_inc_mode = HAL_DISABLE;
    dma2_stream2.init.memory_data_size = DMA_8BIT;
    dma2_stream2.init.memory_inc_mode = HAL_ENABLE;
    dma2_stream2.init.circ_mode = HAL_DISABLE;

    hal_dma_register_callback(&dma2_stream2,
                              DMA_TRANSFER_COMPLETED_CALLBACK,
                              DMA_TransferCompletedCallback);
    hal_dma_register_callback(&dma2_stream2,
                              DMA_TRANSFER_ERROR_CALLBACK,
                              DMA_TransferErrorCallback);
    hal_dma_init(&dma2_stream2);

    NVIC_SetPriority(DMA2_Stream2_IRQn, 8);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    /* DMA2 Stream 3 - SPI TX */
    dma2_stream3.init.channel = DMA_CHANNEL3;
    dma2_stream3.init.priority = DMA_HIGH_PRIORITY;
    dma2_stream3.init.transfer_direction = DMA_MEMORY_TO_PERIPHERAL;
    dma2_stream3.init.peripheral_data_size = DMA_8BIT;
    dma2_stream3.init.peripheral_inc_mode = HAL_DISABLE;
    dma2_stream3.init.memory_data_size = DMA_8BIT;
    dma2_stream3.init.memory_inc_mode = HAL_ENABLE;
    dma2_stream3.init.circ_mode = HAL_DISABLE;

    hal_dma_register_callback(&dma2_stream3,
                              DMA_TRANSFER_COMPLETED_CALLBACK,
                              DMA_TransferCompletedCallback);
    hal_dma_register_callback(&dma2_stream3,
                              DMA_TRANSFER_ERROR_CALLBACK,
                              DMA_TransferErrorCallback);
    hal_dma_init(&dma2_stream3);

    NVIC_SetPriority(DMA2_Stream3_IRQn, 8);
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать DMA SPI2
 */
static void dma_spi2_init(void)
{
    /* DMA1 Stream 3 - SPI RX */
    dma1_stream3.init.channel = DMA_CHANNEL0;
    dma1_stream3.init.priority = DMA_MEDIUM_PRIORITY;
    dma1_stream3.init.transfer_direction = DMA_PERIPHERAL_TO_MEMORY;
    dma1_stream3.init.peripheral_data_size = DMA_8BIT;
    dma1_stream3.init.peripheral_inc_mode = HAL_DISABLE;
    dma1_stream3.init.memory_data_size = DMA_8BIT;
    dma1_stream3.init.memory_inc_mode = HAL_ENABLE;
    dma1_stream3.init.circ_mode = HAL_DISABLE;

    hal_dma_register_callback(&dma1_stream3,
                              DMA_TRANSFER_COMPLETED_CALLBACK,
                              DMA_TransferCompletedCallback);
    hal_dma_register_callback(&dma1_stream3,
                              DMA_TRANSFER_ERROR_CALLBACK,
                              DMA_TransferErrorCallback);
    hal_dma_init(&dma1_stream3);

    NVIC_SetPriority(DMA1_Stream3_IRQn, 9);
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    /* DMA1 Stream 4 - SPI TX */
    dma1_stream4.init.channel = DMA_CHANNEL0;
    dma1_stream4.init.priority = DMA_MEDIUM_PRIORITY;
    dma1_stream4.init.transfer_direction = DMA_MEMORY_TO_PERIPHERAL;
    dma1_stream4.init.peripheral_data_size = DMA_8BIT;
    dma1_stream4.init.peripheral_inc_mode = HAL_DISABLE;
    dma1_stream4.init.memory_data_size = DMA_8BIT;
    dma1_stream4.init.memory_inc_mode = HAL_ENABLE;
    dma1_stream4.init.circ_mode = HAL_DISABLE;

    hal_dma_register_callback(&dma1_stream4,
                              DMA_TRANSFER_COMPLETED_CALLBACK,
                              DMA_TransferCompletedCallback);
    hal_dma_register_callback(&dma1_stream4,
                              DMA_TRANSFER_ERROR_CALLBACK,
                              DMA_TransferErrorCallback);
    hal_dma_init(&dma1_stream4);

    NVIC_SetPriority(DMA1_Stream4_IRQn, 9);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать DMA SPI3
 */
static void dma_spi3_init(void)
{
    /* DMA1 Stream 0 - SPI RX */
    dma1_stream0.init.channel = DMA_CHANNEL0;
    dma1_stream0.init.priority = DMA_VERY_HIGH_PRIORITY;
    dma1_stream0.init.transfer_direction = DMA_PERIPHERAL_TO_MEMORY;
    dma1_stream0.init.peripheral_data_size = DMA_8BIT;
    dma1_stream0.init.peripheral_inc_mode = HAL_DISABLE;
    dma1_stream0.init.memory_data_size = DMA_8BIT;
    dma1_stream0.init.memory_inc_mode = HAL_ENABLE;
    dma1_stream0.init.circ_mode = HAL_DISABLE;

    hal_dma_register_callback(&dma1_stream0,
                              DMA_TRANSFER_COMPLETED_CALLBACK,
                              DMA_TransferCompletedCallback);
    hal_dma_register_callback(&dma1_stream0,
                              DMA_TRANSFER_ERROR_CALLBACK,
                              DMA_TransferErrorCallback);
    hal_dma_init(&dma1_stream0);

    NVIC_SetPriority(DMA1_Stream0_IRQn, 7);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /* DMA1 Stream 7 - SPI TX */
    dma1_stream7.init.channel = DMA_CHANNEL0;
    dma1_stream7.init.priority = DMA_VERY_HIGH_PRIORITY;
    dma1_stream7.init.transfer_direction = DMA_MEMORY_TO_PERIPHERAL;
    dma1_stream7.init.peripheral_data_size = DMA_8BIT;
    dma1_stream7.init.peripheral_inc_mode = HAL_DISABLE;
    dma1_stream7.init.memory_data_size = DMA_8BIT;
    dma1_stream7.init.memory_inc_mode = HAL_ENABLE;
    dma1_stream7.init.circ_mode = HAL_DISABLE;

    hal_dma_register_callback(&dma1_stream7,
                              DMA_TRANSFER_COMPLETED_CALLBACK,
                              DMA_TransferCompletedCallback);
    hal_dma_register_callback(&dma1_stream7,
                              DMA_TRANSFER_ERROR_CALLBACK,
                              DMA_TransferErrorCallback);
    hal_dma_init(&dma1_stream7);

    NVIC_SetPriority(DMA1_Stream7_IRQn, 7);
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}
/* ------------------------------------------------------------------------- */
