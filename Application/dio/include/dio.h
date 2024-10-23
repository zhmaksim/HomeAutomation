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

#ifndef DIO_H_
#define DIO_H_

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение перечисления номеров входных-выходных сигналов
 */
enum dio_nb {
    DIO1,
    DIO2,
    DIO3,
    DIO4,
    DIO5,
    DIO6,
    DIO7,
    DIO8,
    DIO9,
    DIO10,
    DIO11,
    DIO12,
    DIO13,
    DIO14,
    DIO15,
    DIO16,
    DIO17,
    DIO18,
    DIO19,
    DIO20,
    DIO21,
    DIO22,
    DIO23,
    DIO24,
};


/**
 * @brief           Определение перечисления состояний входных-выходных сигналов
 */
enum dio_state {
    DIO_RESET,
    DIO_SET,
};


/**
 * @brief           Определение структуры данных обработчика входных-выходных сигналов
 */
struct dio_handle {
    struct spi_handle          *spi;                        /*!< Указатель на структуру данных обработчика SPI */

    SemaphoreHandle_t           spi_mutex;                  /*!< Mutex SPI */

    EventGroupHandle_t          spi_event_group;            /*!< Event Group SPI */

    struct gpio_handle         *gpio_input_cs;              /*!< Указатель на структуру данных обработчика GPIO ICS */

    struct gpio_handle         *gpio_output_cs;             /*!< Указатель на структуру данных обработчика GPIO OCS */

    struct gpio_handle         *gpio_output_enable;         /*!< Указатель на структуру данных обработчика GPIO OE */

    uint8_t                     idr[3];                     /*!< Данные входных дискретных сигналов */

    uint8_t                     odr[3];                     /*!< Данные выходных дискретных сигналов */
};

/* Exported variables ------------------------------------------------------ */

extern struct dio_handle dio;

/* Exported function prototypes -------------------------------------------- */

void dio_init(struct dio_handle *handle);

void dio_spi_transmit_receive_completed_it_handler(struct dio_handle *handle);

void dio_spi_error_it_handler(struct dio_handle *handle);

enum dio_state dio_state(struct dio_handle *handle, enum dio_nb nb);

void dio_set_state(struct dio_handle *handle, enum dio_nb nb, enum dio_state state);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* DIO_H_ */
