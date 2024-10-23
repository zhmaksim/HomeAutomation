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

#include "spi.h"
#include "dma.h"
#include "stm32f446xx_it.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчики DMA */
extern struct dma_handle dma1_stream0;
extern struct dma_handle dma1_stream3;
extern struct dma_handle dma1_stream4;
extern struct dma_handle dma1_stream7;
extern struct dma_handle dma2_stream2;
extern struct dma_handle dma2_stream3;

/* Обработчик SPI1 */
struct spi_handle spi1 = {
    .instance = SPI1,
    .dma_rx = &dma2_stream2,
    .dma_tx = &dma2_stream3,
};

/* Mutex и Event Group SPI1 */
SemaphoreHandle_t spi1_mutex;
EventGroupHandle_t spi1_event_group;

/* Обработчик SPI2 */
struct spi_handle spi2 = {
    .instance = SPI2,
    .dma_rx = &dma1_stream3,
    .dma_tx = &dma1_stream4,
};

/* Mutex и Event Group SPI2 */
SemaphoreHandle_t spi2_mutex;
EventGroupHandle_t spi2_event_group;

/* Обработчик SPI3 */
struct spi_handle spi3 = {
    .instance = SPI3,
    .dma_rx = &dma1_stream0,
    .dma_tx = &dma1_stream7,
};

/* Mutex и Event Group SPI3 */
SemaphoreHandle_t spi3_mutex;
EventGroupHandle_t spi3_event_group;

/* Private function prototypes --------------------------------------------- */

static void spi1_init(void);

static void spi2_init(void);

static void spi3_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI
 */
void spi_init(void)
{
    HAL_SPI1_ENABLE_CLOCK();
    HAL_SPI2_ENABLE_CLOCK();
    HAL_SPI3_ENABLE_CLOCK();

    spi1_init();
    spi2_init();
    spi3_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI1
 */
static void spi1_init(void)
{
    spi1.init.mode = SPI_MODE0;
    spi1.init.div = SPI_DIV2;
    spi1.init.frame_size = SPI_8BIT;
    spi1.init.frame_format = SPI_MSBFIRST;

    hal_spi_register_callback(&spi1,
                              SPI_TRANSMIT_RECEIVE_COMPLETED_CALLBACK,
                              SPI_TransmitReceiveCompletedCallback);
    hal_spi_register_callback(&spi1,
                              SPI_ERROR_CALLBACK,
                              SPI_ErrorCallback);
    hal_spi_init(&spi1);
    hal_spi_enable(&spi1);

    spi1_mutex = xSemaphoreCreateMutex();
    if (spi1_mutex == NULL)
        hal_error();

    spi1_event_group = xEventGroupCreate();
    if (spi1_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI2
 */
static void spi2_init(void)
{
    spi2.init.mode = SPI_MODE0;
    spi2.init.div = SPI_DIV2;
    spi2.init.frame_size = SPI_8BIT;
    spi2.init.frame_format = SPI_MSBFIRST;

    hal_spi_register_callback(&spi2,
                              SPI_TRANSMIT_RECEIVE_COMPLETED_CALLBACK,
                              SPI_TransmitReceiveCompletedCallback);
    hal_spi_register_callback(&spi2,
                              SPI_ERROR_CALLBACK,
                              SPI_ErrorCallback);
    hal_spi_init(&spi2);
    hal_spi_enable(&spi2);

    spi2_mutex = xSemaphoreCreateMutex();
    if (spi2_mutex == NULL)
        hal_error();

    spi2_event_group = xEventGroupCreate();
    if (spi2_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI3
 */
static void spi3_init(void)
{
    spi3.init.mode = SPI_MODE0;
    spi3.init.div = SPI_DIV2;
    spi3.init.frame_size = SPI_8BIT;
    spi3.init.frame_format = SPI_MSBFIRST;

    hal_spi_register_callback(&spi3,
                              SPI_TRANSMIT_RECEIVE_COMPLETED_CALLBACK,
                              SPI_TransmitReceiveCompletedCallback);
    hal_spi_register_callback(&spi3,
                              SPI_ERROR_CALLBACK,
                              SPI_ErrorCallback);
    hal_spi_init(&spi3);
    hal_spi_enable(&spi3);

    spi3_mutex = xSemaphoreCreateMutex();
    if (spi3_mutex == NULL)
        hal_error();

    spi3_event_group = xEventGroupCreate();
    if (spi3_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */
