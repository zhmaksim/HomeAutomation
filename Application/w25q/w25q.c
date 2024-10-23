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

#include "w25q.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define W25Q_MANUFACTURER_ID            0xEF
#define W25Q16                          0x14
#define W25Q32                          0x15
#define W25Q64                          0x16
#define W25Q128                         0x17
#define W25Q256                         0x18
#define W25Q512                         0x19

#define W25Q_SR1_BUSY_Pos               0
#define W25Q_SR1_BUSY_Msk               HAL_BITMASK(0x01, W25Q_SR1_BUSY_Pos)
#define W25Q_SR1_BUSY                   W25Q_SR1_BUSY_Msk

#define W25Q_SR1_WEL_Pos                1
#define W25Q_SR1_WEL_Msk                HAL_BITMASK(0x01, W25Q_SR1_WEL_Pos)
#define W25Q_SR1_WEL                    W25Q_SR1_WEL_Msk

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик SPI */
extern struct spi_handle spi1;

/* Mutex и Event Group SPI */
extern SemaphoreHandle_t spi1_mutex;
extern EventGroupHandle_t spi1_event_group;

/* Обработчик GPIO W25Q CS */
extern struct gpio_handle gpio_w25q_cs;

/* Обработчик W25Q */
struct w25q_handle w25q = {
    .spi = &spi1,
    .gpio_cs = &gpio_w25q_cs,
};

/* Private function prototypes --------------------------------------------- */

static int32_t w25q_read_device_id(struct w25q_handle *handle);

static int32_t w25q_read_unique_id(struct w25q_handle *handle);

static int32_t w25q_write_enable(struct w25q_handle *handle);

static int32_t w25q_read_sr1(struct w25q_handle *handle, void *data);

static int32_t w25q_transmit_receive(struct w25q_handle *handle, void *data, size_t size);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 *                      - W25Q_MALLOC_ERROR
 *                      - W25Q_DEVICE_ID_ERROR
 */
int32_t w25q_init(struct w25q_handle *handle)
{
    assert(handle != NULL);
    assert(handle->spi != NULL);
    assert(handle->gpio_cs != NULL);

    int32_t status = W25Q_ERROR;

    /* Установить Mutex и Event Group */
    if (handle->spi == &spi1) {
        handle->spi_mutex = spi1_mutex;
        handle->spi_event_group = spi1_event_group;
    }

    /* Проверить идентификатор устройства */
    status = w25q_read_device_id(handle);
    if (status != W25Q_OK) {
        return status;
    } else if (handle->id.manufacturer_id != W25Q_MANUFACTURER_ID) {
        return W25Q_DEVICE_ID_ERROR;
    }

    /* Определить размер памяти */
    switch (handle->id.device_id) {
        case W25Q16:
            handle->size = 0x200000;
            break;
        case W25Q32:
            handle->size = 0x400000;
            break;
        case W25Q64:
            handle->size = 0x800000;
            break;
        case W25Q128:
            handle->size = 0x1000000;
            break;
        case W25Q256:
            handle->size = 0x2000000;
            break;
        case W25Q512:
            handle->size = 0x4000000;
            break;
        default:
            return W25Q_DEVICE_ID_ERROR;
    }

    /* Прочитать уникальный идентификатор */
    status = w25q_read_unique_id(handle);
    if (status != W25Q_OK)
        return status;

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершения приема-передачи данных SPI W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 */
void w25q_spi_transmit_receive_completed_it_handler(struct w25q_handle *handle)
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
 * @brief           Обработать прерывание ошибки SPI W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 */
void w25q_spi_error_it_handler(struct w25q_handle *handle)
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
 * @brief           Прочитать идентификатор W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 */
static int32_t w25q_read_device_id(struct w25q_handle *handle)
{
    int32_t status = W25Q_ERROR;

    uint8_t buff[6];

    buff[0] = 0x90;
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x00;

    status = w25q_transmit_receive(handle, buff, sizeof(buff));
    if (status != W25Q_OK)
        return W25Q_TX_RX_ERROR;

    taskENTER_CRITICAL();
    {
        handle->id.manufacturer_id = buff[4];
        handle->id.device_id = buff[5];
    }
    taskEXIT_CRITICAL();

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать уникальный идентификатор W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 */
static int32_t w25q_read_unique_id(struct w25q_handle *handle)
{
    int32_t status = W25Q_ERROR;

    uint8_t buff[13];

    buff[0] = 0x4B;
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;

    status = w25q_transmit_receive(handle, buff, sizeof(buff));
    if (status != W25Q_OK)
        return W25Q_TX_RX_ERROR;

    taskENTER_CRITICAL();
    {
        memcpy(handle->id.uid, &buff[5], 8);
    }
    taskEXIT_CRITICAL();

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить разрешение на запись W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 */
static int32_t w25q_write_enable(struct w25q_handle *handle)
{
    int32_t status = W25Q_ERROR;

    uint8_t buff[1];

    buff[0] = 0x06;

    status = w25q_transmit_receive(handle, buff, sizeof(buff));
    if (status != W25Q_OK)
        return W25Q_TX_RX_ERROR;

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать SR1 W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @param[out]      data: Указатель на данные (uint8_t)
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 */
static int32_t w25q_read_sr1(struct w25q_handle *handle, void *data)
{
    int32_t status = W25Q_ERROR;

    if (data == NULL)
        return status;

    uint8_t buff[2];

    buff[0] = 0x05;

    status = w25q_transmit_receive(handle, buff, sizeof(buff));
    if (status != W25Q_OK)
        return W25Q_TX_RX_ERROR;

    taskENTER_CRITICAL();
    {
        *(uint8_t *) data = buff[1];
    }
    taskEXIT_CRITICAL();

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать данные W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @param[in]       mem_addr: Адрес
 * @param[out]      data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 *                      - W25Q_MALLOC_ERROR
 */
int32_t w25q_fast_read(struct w25q_handle *handle, uint32_t mem_addr, void *data, size_t size)
{
    assert(handle != NULL);

    int32_t status = W25Q_ERROR;

    if (data == NULL || size == 0)
        return status;

    if (mem_addr + size > handle->size)
        return status;

    uint8_t *buff = pvPortMalloc(size + 5);
    if (buff == NULL)
        return W25Q_MALLOC_ERROR;

    buff[0] = 0x0B;
    buff[1] = (uint8_t) (mem_addr >> 16);
    buff[2] = (uint8_t) (mem_addr >> 8);
    buff[3] = (uint8_t)  mem_addr;
    buff[4] = 0x00;

    status = w25q_transmit_receive(handle, buff, size + 5);
    if (status != W25Q_OK) {
        status = W25Q_TX_RX_ERROR;
    } else {
        taskENTER_CRITICAL();
        {
            memcpy(data, &buff[5], size);
        }
        taskEXIT_CRITICAL();
    }

    vPortFree(buff);

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Записать данные W25Q
 *
 * @details         Примечание: Данные записываются в заранее стерные (0xFF) ячейки
 *                  Записываются данные, которые помещаются в страницы, остальные записаны не будут
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @param[in]       mem_addr: Адрес
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 *                      - W25Q_MALLOC_ERROR
 */
int32_t w25q_page_program(struct w25q_handle *handle, uint32_t mem_addr, const void *data, size_t size)
{
    assert(handle != NULL);

    int32_t status = W25Q_ERROR;

    if (data == NULL || size == 0)
        return status;

    if (mem_addr + size > handle->size)
        return status;

    if (w25q_write_enable(handle) != W25Q_OK)
        return status;

    uint32_t page_mem_addr_end = (mem_addr & 0xFFFFFF00) + W25Q_PAGE_SIZE;

    if (size > page_mem_addr_end - mem_addr)
        size = page_mem_addr_end - mem_addr;

    uint8_t *buff = pvPortMalloc(size + 4);
    if (buff == NULL)
        return W25Q_MALLOC_ERROR;

    buff[0] = 0x02;
    buff[1] = (uint8_t) (mem_addr >> 16);
    buff[2] = (uint8_t) (mem_addr >> 8);
    buff[3] = (uint8_t)  mem_addr;

    taskENTER_CRITICAL();
    {
        memcpy(&buff[4], data, size);
    }
    taskEXIT_CRITICAL();

    status = w25q_transmit_receive(handle, buff, size + 4);
    if (status != W25Q_OK) {
        status = W25Q_TX_RX_ERROR;
    } else {
        uint8_t SR1 = W25Q_SR1_BUSY;

        while ((status = w25q_read_sr1(handle, &SR1)) == W25Q_OK) {
            if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk))
                break;
        }
    }

    vPortFree(buff);

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Очистить сектор флэш-памяти (4Кб) W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @param[in]       mem_addr: Адрес
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 */
int32_t w25q_sector_erase(struct w25q_handle *handle, uint32_t mem_addr)
{
    assert(handle != NULL);

    int32_t status = W25Q_ERROR;

    if (mem_addr > handle->size)
        return status;

    if (w25q_write_enable(handle) != W25Q_OK)
        return status;

    mem_addr = mem_addr & 0xFFFFF000;

    uint8_t buff[4];

    buff[0] = 0x20;
    buff[1] = (uint8_t) (mem_addr >> 16);
    buff[2] = (uint8_t) (mem_addr >> 8);
    buff[3] = (uint8_t)  mem_addr;

    status = w25q_transmit_receive(handle, buff, sizeof(buff));
    if (status != W25Q_OK)
        return W25Q_TX_RX_ERROR;

    uint8_t SR1 = W25Q_SR1_BUSY;

    while ((status = w25q_read_sr1(handle, &SR1)) == W25Q_OK) {
        if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk))
            break;
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Очистить блок флэш-памяти (64Кб) W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @param[in]       mem_addr: Адрес
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 */
int32_t w25q_block_erase(struct w25q_handle *handle, uint32_t mem_addr)
{
    assert(handle != NULL);

    int32_t status = W25Q_ERROR;

    if (mem_addr > handle->size)
        return status;

    if (w25q_write_enable(handle) != W25Q_OK)
        return status;

    mem_addr = mem_addr & 0xFFFF0000;

    uint8_t buff[4];

    buff[0] = 0xD8;
    buff[1] = (uint8_t) (mem_addr >> 16);
    buff[2] = (uint8_t) (mem_addr >> 8);
    buff[3] = (uint8_t) mem_addr;

    status = w25q_transmit_receive(handle, buff, sizeof(buff));
    if (status != W25Q_OK)
        return W25Q_TX_RX_ERROR;

    uint8_t SR1 = W25Q_SR1_BUSY;

    while ((status = w25q_read_sr1(handle, &SR1)) == W25Q_OK) {
        if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk))
            break;
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Очистить флэш-память W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 *                      - W25Q_TX_RX_ERROR
 */
int32_t w25q_chip_erase(struct w25q_handle *handle)
{
    assert(handle != NULL);

    int32_t status = W25Q_ERROR;

    uint8_t buff[1];

    buff[0] = 0xC7;

    status = w25q_transmit_receive(handle, buff, sizeof(buff));
    if (status != W25Q_OK)
        return W25Q_TX_RX_ERROR;

    uint8_t SR1 = W25Q_SR1_BUSY;

    while ((status = w25q_read_sr1(handle, &SR1)) == W25Q_OK) {
        if (!READ_BIT(SR1, W25Q_SR1_BUSY_Msk))
            break;
    }

    return status;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием-передача данных SPI DMA W25Q
 *
 * @param[in]       handle: Указатель на структуру данных обработчика W25Q
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус:
 *                      - W25Q_OK
 *                      - W25Q_ERROR
 */
static int32_t w25q_transmit_receive(struct w25q_handle *handle, void *data, size_t size)
{
    assert(handle->spi_mutex != NULL);
    assert(handle->spi_event_group != NULL);

    int32_t status = W25Q_ERROR;

    /* Проверить наличие данных */
    if (data == NULL || size == 0)
        return status;

    /* Захватить Mutex */
    if (xSemaphoreTake(handle->spi_mutex, pdMS_TO_TICKS(50)) != pdPASS)
        return status;

    /* Установить сигнал CS = Low */
    hal_gpio_set_state(handle->gpio_cs, GPIO_RESET);

    /* Передача данных */
    hal_spi_transmit_receive_dma(handle->spi, data, size, data, size);

    /* Ожидание завершения приема-передачи данных */
    EventBits_t bits = xEventGroupWaitBits(handle->spi_event_group,
                                           SPI_EV_TX_RX_CPLT | SPI_EV_ERR,
                                           pdTRUE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (READ_BIT(bits, SPI_EV_TX_RX_CPLT_Msk))
        status = W25Q_OK;

    /* Установить сигнал CS = High */
    hal_gpio_set_state(handle->gpio_cs, GPIO_SET);

    /* Освободить Mutex */
    xSemaphoreGive(handle->spi_mutex);

    return status;
}
/* ------------------------------------------------------------------------- */
