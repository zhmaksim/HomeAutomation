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

#include "dio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик SPI */
extern struct spi_handle spi2;

/* Mutex и Event Group SPI */
extern SemaphoreHandle_t spi2_mutex;
extern EventGroupHandle_t spi2_event_group;

/* Обработчик GPIO DIO EN, CS */
extern struct gpio_handle gpio_74hc165_cs;
extern struct gpio_handle gpio_74hc595_cs;
extern struct gpio_handle gpio_74hc595_en;

/* Обработчик DIO */
struct dio_handle dio = {
    .spi = &spi2,
    .gpio_input_cs = &gpio_74hc165_cs,
    .gpio_output_cs = &gpio_74hc595_cs,
    .gpio_output_enable = &gpio_74hc595_en,
};

/* Private function prototypes --------------------------------------------- */

static void dio_process(void *arg);

static hal_status_t dio_transmit_receive(struct dio_handle *handle);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать входные-выходные сигналы
 *
 * @param[in]       handle: Указатель на структуру данных обработчика входных-выходных сигналов
 */
void dio_init(struct dio_handle *handle)
{
    assert(handle != NULL);
    assert(handle->spi != NULL);
    assert(handle->gpio_input_cs != NULL);
    assert(handle->gpio_output_cs != NULL);
    assert(handle->gpio_output_enable != NULL);

    /* Установить Mutex и Event Group */
    if (handle->spi == &spi2) {
        handle->spi_mutex = spi2_mutex;
        handle->spi_event_group = spi2_event_group;
    }

    xTaskCreate(dio_process,
                "dio",
                configMINIMAL_STACK_SIZE,
                (void *) handle,
                tskIDLE_PRIORITY + 3,
                NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать завершение приема-передачи данных SPI входных-выходных сигналов
 *
 * @param[in]       handle: Указатель на структуру данных обработчика входных-выходных сигналов
 */
void dio_spi_transmit_receive_completed_it_handler(struct dio_handle *handle)
{
    assert(handle != NULL);
    assert(handle->spi_event_group != NULL);

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(handle->spi_event_group,
                                  SPI_EV_TX_RX_CPLT,
                                  &higher_priority_task_woken) == pdPASS) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать ошибку SPI входных-выходных сигналов
 *
 * @param[in]       handle: Указатель на структуру данных обработчика входных-выходных сигналов
 */
void dio_spi_error_it_handler(struct dio_handle *handle)
{
    assert(handle != NULL);
    assert(handle->spi_event_group != NULL);

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xEventGroupSetBitsFromISR(handle->spi_event_group,
                                  SPI_EV_ERR,
                                  &higher_priority_task_woken) == pdPASS) {
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Состояние входных дискретных сигналов
 *
 * @param[in]       handle: Указатель на структуру данных обработчика входных-выходных сигналов
 * @param[in]       nb: Номер входного сигнала
 * @return          Состояние @ref enum dio_state
 */
enum dio_state dio_state(struct dio_handle *handle, enum dio_nb nb)
{
    assert(handle != NULL);

    enum dio_state state = DIO_RESET;

    taskENTER_CRITICAL();
    {
        if (nb <= DIO8) {
            uint8_t pos = 7 - nb;

            state = READ_BIT(handle->idr[0], HAL_BITMASK(0x01, pos)) ? DIO_RESET : DIO_SET;
        } else if (nb > DIO8 && nb <= DIO16) {
            uint8_t pos = 15 - nb;

            state = READ_BIT(handle->idr[1], HAL_BITMASK(0x01, pos)) ? DIO_RESET : DIO_SET;
        } else {
            uint8_t pos = 23 - nb;

            state = READ_BIT(handle->idr[2], HAL_BITMASK(0x01, pos)) ? DIO_RESET : DIO_SET;
        }
    }
    taskEXIT_CRITICAL();

    return state;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить состояние выходных дискретных сигналов
 *
 * @param[in]       handle: Указатель на структуру данных обработчика входных-выходных сигналов
 * @param[in]       nb: Номер выходного сигнала
 * @param[in]       state Состояние
 */
void dio_set_state(struct dio_handle *handle, enum dio_nb nb, enum dio_state state)
{
    assert(handle != NULL);

    taskENTER_CRITICAL();
    {
        if (nb <= DIO8) {
            uint8_t pos = 7 - nb;

            MODIFY_REG(handle->odr[2],
                       HAL_BITMASK(0x01, pos),
                       HAL_BITMASK(state, pos));
        } else if (nb > DIO8 && nb <= DIO16) {
            uint8_t pos = 15 - nb;

            MODIFY_REG(handle->odr[1],
                       HAL_BITMASK(0x01, pos),
                       HAL_BITMASK(state, pos));
        } else {
            uint8_t pos = 23 - nb;

            MODIFY_REG(handle->odr[0],
                       HAL_BITMASK(0x01, pos),
                       HAL_BITMASK(state, pos));
        }
    }
    taskEXIT_CRITICAL();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать события и команды входных-выходных сигналов
 *
 * @param[in]       arg: Указатель на параметры
 */
static void dio_process(void *arg)
{
    static const TickType_t frequency = pdMS_TO_TICKS(40);

    struct dio_handle *handle = arg;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, frequency);

        /* Прием-передача данных входных-выходных сигналов */
        dio_transmit_receive(handle);
    }

    vTaskDelete(NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием-передача данных входных-выходных сигналов
 *
 * @param[in]       handle: Указатель на структуру данных обработчика входных-выходных сигналов
 * @return          Статус @ref hal_status_t
 */
static hal_status_t dio_transmit_receive(struct dio_handle *handle)
{
    hal_status_t status = HAL_ERROR;

    uint8_t idr[3];
    uint8_t odr[3];

    /* Захватить Mutex */
    if (xSemaphoreTake(handle->spi_mutex, portMAX_DELAY) != pdPASS)
        return status;

    /* Копировать данные выходных сигналов */
    taskENTER_CRITICAL();
    {
        memcpy(odr, handle->odr, sizeof(handle->odr));
    }
    taskEXIT_CRITICAL();

    /* Установить сигнал INPUT CS = High */
    hal_gpio_set_state(handle->gpio_input_cs, GPIO_SET);
    /* Установить сигнал OUTPUT CS = Low */
    hal_gpio_set_state(handle->gpio_output_cs, GPIO_RESET);

    /* Прием-передача данных */
    hal_spi_transmit_receive_dma(handle->spi, odr, sizeof(odr), idr, sizeof(idr));

    /* Ожидание завершения приема-передачи данных */
    EventBits_t bits = xEventGroupWaitBits(handle->spi_event_group,
                                           SPI_EV_TX_RX_CPLT | SPI_EV_ERR,
                                           pdTRUE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (READ_BIT(bits, SPI_EV_TX_RX_CPLT_Msk))
        status = HAL_OK;

    /* Установить сигнал INPUT CS = Low */
    hal_gpio_set_state(handle->gpio_input_cs, GPIO_RESET);
    /* Установить сигнал OUTPUT CS = High */
    hal_gpio_set_state(handle->gpio_output_cs, GPIO_SET);
    /* Установить сигнал OUTPUT EN = Low */
    hal_gpio_set_state(handle->gpio_output_enable, GPIO_RESET);

    /* Копировать данные входных сигналов */
    taskENTER_CRITICAL();
    {
        memcpy(handle->idr, idr, sizeof(idr));
    }
    taskEXIT_CRITICAL();

    /* Освободить Mutex */
    xSemaphoreGive(handle->spi_mutex);

    return status;
}
/* ------------------------------------------------------------------------- */
