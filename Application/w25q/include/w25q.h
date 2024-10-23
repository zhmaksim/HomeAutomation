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

#ifndef W25Q_H_
#define W25Q_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define W25Q_PAGE_SIZE          0x100
#define W25Q_SECTOR_SIZE        0x1000
#define W25Q_BLOCK_SIZE         0x10000

#define W25Q_OK                 0
#define W25Q_ERROR             -1
#define W25Q_TX_RX_ERROR       -2
#define W25Q_MALLOC_ERROR      -3
#define W25Q_DEVICE_ID_ERROR   -4

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры идентификационных данных W25Q
 */
struct w25q_id {
    uint8_t     manufacturer_id;                /*!< Идентификатор производителя */

    uint8_t     device_id;                      /*!< Идентификатор устройства */

    uint8_t     uid[8];                         /*!< Уникальный идентификатор устройства */
};


/**
 * @brief           Определение структуры данных обработчика W25Q
 */
struct w25q_handle {
    struct spi_handle          *spi;                        /*!< Указатель на структуру данных обработчика SPI */

    SemaphoreHandle_t           spi_mutex;                  /*!< Mutex SPI */

    EventGroupHandle_t          spi_event_group;            /*!< Event Group SPI */

    struct gpio_handle         *gpio_cs;                    /*!< Указатель на структуру данных обработчика GPIO сигнала CS */

    struct w25q_id              id;                         /*!< Идентификационные данные */

    size_t                      size;                       /*!< Размер памяти */
};

/* Exported variables ------------------------------------------------------ */

extern struct w25q_handle w25q;

/* Exported function prototypes -------------------------------------------- */

int32_t w25q_init(struct w25q_handle *handle);

void w25q_spi_transmit_receive_completed_it_handler(struct w25q_handle *handle);

void w25q_spi_error_it_handler(struct w25q_handle *handle);

int32_t w25q_fast_read(struct w25q_handle *handle, uint32_t mem_addr, void *data, size_t size);

int32_t w25q_page_program(struct w25q_handle *handle, uint32_t mem_addr, const void *data, size_t size);

int32_t w25q_sector_erase(struct w25q_handle *handle, uint32_t mem_addr);

int32_t w25q_block_erase(struct w25q_handle *handle, uint32_t mem_addr);

int32_t w25q_chip_erase(struct w25q_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* W25Q_H_ */
