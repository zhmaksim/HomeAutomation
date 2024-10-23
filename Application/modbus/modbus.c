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

#include "modbus.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define MODBUS_EXCEPTION_Pos        7
#define MODBUS_EXCEPTION_Msk        HAL_BITMASK(0x01, MODBUS_EXCEPTION_Pos)
#define MODBUS_EXCEPTION            MODBUS_EXCEPTION_Msk

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчики USART */
extern struct usart_handle usart1;
extern struct usart_handle usart2;

/* Mutex и Event Group USART */
extern SemaphoreHandle_t usart1_mutex;
extern EventGroupHandle_t usart1_event_group;

extern SemaphoreHandle_t usart2_mutex;
extern EventGroupHandle_t usart2_event_group;

/* Обработчики Modbus */
struct modbus_handle modbus[MODBUS_COUNT] = {
    {
        .usart = &usart1,
        .led_rx = &led[LED_RX],
        .led_tx = &led[LED_TX],
        .mode = MODBUS_MASTER,
        .timeout35 = 2,
        .timeout_response = 50,
    },
    {
        .usart = &usart2,
        .led_rx = &led[LED_RX],
        .led_tx = &led[LED_TX],
        .mode = MODBUS_MASTER,
        .timeout35 = 2,
        .timeout_response = 50,
    },
};

/* Private function prototypes --------------------------------------------- */

static void modbus_slave(void *arg);

static void modbus_master(void *arg);

static void modbus_receive(struct modbus_handle *handle);

static hal_status_t modbus_wait_receive_end(struct modbus_handle *handle);

static void modbus_transmit(struct modbus_handle *handle);

static void modbus_slave_message(struct modbus_handle *handle);

static void modbus_master_message(struct modbus_handle *handle);

static enum modbus_exception modbus_diagnostic(struct modbus_handle *handle);

static enum modbus_exception modbus_report_server_id(struct modbus_handle *handle);

static void modbus_request_diagnostic(struct modbus_handle *handle, struct modbus_request *request);

static void modbus_request_read_holding_registers(struct modbus_handle *handle, struct modbus_request *request);

static void modbus_request_write_multiple_registers(struct modbus_handle *handle, struct modbus_request *request);

static void modbus_response_read_holding_registers(struct modbus_handle *handle, struct modbus_request *request);

static uint16_t modbus_crc(const void *data, size_t size);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
void modbus_init(struct modbus_handle *handle)
{
    assert(handle != NULL);
    assert(handle->usart != NULL);

    /* Установить Mutex и Event Group */
    if (handle->usart == &usart1) {
        handle->usart_mutex = usart1_mutex;
        handle->usart_event_group = usart1_event_group;
    } else if (handle->usart == &usart2) {
        handle->usart_mutex = usart2_mutex;
        handle->usart_event_group = usart2_event_group;
    }

    if (handle->mode == MODBUS_SLAVE) {
        xTaskCreate(modbus_slave,
                    "modbus slave",
                    configMINIMAL_STACK_SIZE,
                    (void *) handle,
                    tskIDLE_PRIORITY + 3,
                    NULL);
    } else if (handle->mode == MODBUS_MASTER) {
        handle->state = MODBUS_IDLE;

        handle->queue = xQueueCreate(MODBUS_QUEUE_SIZE, sizeof(struct modbus_request));
        if (handle->queue == NULL)
            hal_error();

        xTaskCreate(modbus_master,
                    "modbus master",
                    configMINIMAL_STACK_SIZE * 2,
                    (void *) handle,
                    tskIDLE_PRIORITY + 3,
                    NULL);
    } else {
        hal_error();
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершение передачи данных USART Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
void modbus_usart_transmit_completed_it_handler(struct modbus_handle *handle)
{
    assert(handle != NULL);

    if (handle->state == MODBUS_TRANSMIT) {
        /* Выключить светодиод */
        if (handle->led_tx != NULL)
            led_off(handle->led_tx);

        /* Запустить прием данных */
        modbus_receive(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершение приема данных USART Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
void modbus_usart_receive_completed_it_handler(struct modbus_handle *handle)
{
    assert(handle != NULL);

    if (handle->state == MODBUS_RECEIVE) {
        /* Выключить светодиод */
        if (handle->led_rx != NULL)
            led_off(handle->led_rx);

        /* Запустить прием данных */
        modbus_receive(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание ошибки USART Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
void modbus_usart_error_it_handler(struct modbus_handle *handle)
{
    assert(handle != NULL);

    if (handle->state == MODBUS_TRANSMIT ||
        handle->state == MODBUS_RECEIVE) {
        /* Выключить светодиоды */
        if (handle->led_tx != NULL)
            led_off(handle->led_tx);
        if (handle->led_rx != NULL)
            led_off(handle->led_rx);

        /* Запустить прием данных */
        modbus_receive(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание таймера Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
void modbus_tim_it_handler(struct modbus_handle *handle)
{
    assert(handle != NULL);

    if (handle->state == MODBUS_NOT_INIT)
        return;

    handle->timer++;

    if (handle->state == MODBUS_IDLE || handle->state == MODBUS_RECEIVE_WAIT) {
        if (handle->usart->rx_counter > 0) {
            handle->state = MODBUS_RECEIVE;
            handle->data_size = handle->usart->rx_counter;
            handle->timer = 0;

            if (handle->led_rx != NULL)
                led_on(handle->led_rx);
        } else if (handle->state == MODBUS_RECEIVE_WAIT && handle->timer >= handle->timeout_response) {
            handle->state = MODBUS_TIMEOUT_RESPONSE;
            handle->data_size = handle->usart->rx_counter;

            if (handle->led_rx != NULL)
                led_off(handle->led_rx);

            hal_usart_abort_receive_it(handle->usart);

            BaseType_t higher_priority_task_woken = pdFALSE;

            if (xEventGroupSetBitsFromISR(handle->usart_event_group,
                                          USART_EV_RECEIVE_CPLT,
                                          &higher_priority_task_woken) != pdFAIL) {
                portYIELD_FROM_ISR(higher_priority_task_woken);
            }
        }
    } else if (handle->state == MODBUS_RECEIVE) {
        if (handle->usart->rx_counter > handle->data_size) {
            handle->data_size = handle->usart->rx_counter;
            handle->timer = 0;
        } else if (handle->timer >= handle->timeout35) {
            handle->state = MODBUS_RECEIVE_COMPLETED;
            handle->data_size = handle->usart->rx_counter;

            if (handle->led_rx != NULL)
                led_off(handle->led_rx);

            hal_usart_abort_receive_it(handle->usart);

            BaseType_t higher_priority_task_woken = pdFALSE;

            if (xEventGroupSetBitsFromISR(handle->usart_event_group,
                                          USART_EV_RECEIVE_CPLT,
                                          &higher_priority_task_woken) != pdFAIL) {
                portYIELD_FROM_ISR(higher_priority_task_woken);
            }
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием данных Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
static void modbus_receive(struct modbus_handle *handle)
{
    if (handle->mode == MODBUS_MASTER) {
        handle->state = MODBUS_RECEIVE_WAIT;
    } else {
        handle->state = MODBUS_IDLE;
    }

    handle->data_size = MODBUS_DATA_SIZE;
    handle->timer = 0;

    hal_usart_receive_it(handle->usart,
                         handle->data,
                         handle->data_size);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Ожидание завершения приема данных Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @return          Статус @ref hal_status_t
 */
static uint8_t modbus_wait_receive_end(struct modbus_handle *handle)
{
    EventBits_t bits = xEventGroupWaitBits(handle->usart_event_group,
                                           USART_EV_RECEIVE_CPLT,
                                           pdTRUE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (READ_BIT(bits, USART_EV_RECEIVE_CPLT_Msk))
        return HAL_OK;

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передача данных Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
static void modbus_transmit(struct modbus_handle *handle)
{
    handle->state = MODBUS_TRANSMIT;
    handle->timer = 0;

    if (handle->led_tx != NULL)
        led_on(handle->led_tx);

    hal_usart_transmit_it(handle->usart,
                          handle->data,
                          handle->data_size);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Запрос Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @param[in]       request: Указатель на структуру данных запроса
 * @return          Статус @ref hal_status_t
 */
hal_status_t modbus_request(struct modbus_handle *handle, const struct modbus_request *request)
{
    assert(handle != NULL);
    assert(handle->queue != NULL);

    if (request == NULL)
        return HAL_ERROR;

    if (xQueueSend(handle->queue, request, pdMS_TO_TICKS(300)) != pdPASS)
        return HAL_ERROR;

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать события Modbus Slave
 *
 * @param[in]       arg: Указатель на параметры
 */
static void modbus_slave(void *arg)
{
    struct modbus_handle *handle = arg;

    if (xSemaphoreTake(handle->usart_mutex, pdMS_TO_TICKS(100)) != pdPASS)
        hal_error();

    modbus_receive(handle);

    while (true)
        modbus_slave_message(handle);

    vTaskDelete(NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать события Modbus Master
 *
 * @param[in]       arg: Указатель на параметры
 */
static void modbus_master(void *arg)
{
    struct modbus_handle *handle = arg;

    if (xSemaphoreTake(handle->usart_mutex, pdMS_TO_TICKS(100)) != pdPASS)
        hal_error();

    while (true)
        modbus_master_message(handle);

    vTaskDelete(NULL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать сообщения Modbus Slave
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
static void modbus_slave_message(struct modbus_handle *handle)
{
    /* Ожидание завершения приема данных */
    if (modbus_wait_receive_end(handle) != HAL_OK) {
        modbus_receive(handle);
        return;
    }

    /* Проверить состояние */
    if (handle->state != MODBUS_RECEIVE_COMPLETED) {
        modbus_receive(handle);
        return;
    } else {
        handle->bus_message_count++;
    }

    /* Проверить контрольную сумму данных */
    if (modbus_crc(handle->data, handle->data_size)) {
        handle->bus_communication_error_count++;
        modbus_receive(handle);
        return;
    }

    /* Проверить адрес удаленного устройства */
    uint8_t server_address = handle->data[0];
    if (server_address != handle->server_address && server_address != 0) {
        modbus_receive(handle);
        return;
    }

    /* Обработать сообщение */
    enum modbus_function function = handle->data[1];
    /* Исключительное событие выполнения функции */
    enum modbus_exception exception = MODBUS_NO_EXCEPTION;

    switch (function) {
        case MODBUS_DIAGNOSTIC:
            exception = modbus_diagnostic(handle);
            break;
        case MODBUS_REPORT_SERVER_ID:
            exception = modbus_report_server_id(handle);
            break;
        default:
            exception = MODBUS_ILLEGAL_FUNCTION;
            break;
    }

    /* На широковещательное сообщение не отвечаем */
    if (server_address == 0) {
        modbus_receive(handle);
        return;
    }

    /* Проверить исключительные события обработки сообщения */
    if (exception != MODBUS_NO_EXCEPTION) {
        SET_BIT(handle->data[1], MODBUS_EXCEPTION_Msk);
        handle->data[2] = exception;
        handle->data_size = 3;
    }

    /* Рассчитать и записать контрольную сумму сообщения */
    uint16_t crc = modbus_crc(handle->data, handle->data_size);

    handle->data[handle->data_size++] = (uint8_t) (crc >> 8);
    handle->data[handle->data_size++] = (uint8_t)  crc;

    /* Запустить передачу данных */
    modbus_transmit(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать сообщения Modbus Master
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 */
static void modbus_master_message(struct modbus_handle *handle)
{
    /* Получить запрос из очереди */
    struct modbus_request request;

    if (xQueueReceive(handle->queue, &request, portMAX_DELAY) != pdPASS)
        return;

    /* Выполнить запрос */
    for (uint8_t retry = 0; retry < 3; retry++) {
        /* Создать запрос */
        switch (request.function) {
            case MODBUS_DIAGNOSTIC:
                modbus_request_diagnostic(handle, &request);
                break;
            case MODBUS_READ_HOLDING_REGISTERS:
                modbus_request_read_holding_registers(handle, &request);
                break;
            case MODBUS_WRITE_MULTIPLE_REGISTERS:
                modbus_request_write_multiple_registers(handle, &request);
                break;
            default:
                handle->state = MODBUS_IDLE;
                /* Установить статус - Ошибка запроса */
                if (request.status != NULL)
                    SET_BIT(*(uint8_t *) request.status, MODBUS_REQUEST_ERR_Msk);
                return;
        }

        /* Рассчитать и записать конрольную сообщения */
        uint16_t crc = modbus_crc(handle->data, handle->data_size);

        handle->data[handle->data_size++] = (uint8_t) (crc >> 8);
        handle->data[handle->data_size++] = (uint8_t)  crc;

        /* Передать данные, дождаться ответа */
        modbus_transmit(handle);

        /* Ожидание завершения приема данных */
        if (modbus_wait_receive_end(handle) != HAL_OK) {
            handle->state = MODBUS_IDLE;
            handle->server_no_response_count++;
            continue;
        }

        /* Если нет ответа на запрос - повторное создание и отправка сообщения */
        if (handle->state == MODBUS_RECEIVE_COMPLETED) {
            handle->state = MODBUS_IDLE;
            handle->bus_message_count++;
        } else {
            handle->state = MODBUS_IDLE;
            handle->server_no_response_count++;
            continue;
        }

        /* Проверить контрольную сумму данных и исключительное событие */
        if (modbus_crc(handle->data, handle->data_size)) {
            handle->bus_communication_error_count++;
            continue;
        } else if (READ_BIT(handle->data[1], MODBUS_EXCEPTION_Msk)) {
            handle->bus_exception_error_count++;
            /* Установить статус - Ошибка запроса */
            if (request.status != NULL)
                SET_BIT(*(uint8_t *) request.status, MODBUS_REQUEST_ERR_Msk);
            return;
        }

        /* Обработать ответ */
        switch (request.function) {
            case MODBUS_READ_HOLDING_REGISTERS:
                modbus_response_read_holding_registers(handle, &request);
                break;
            default:
                break;
        }

        /* Установить статус - Запрос выполнен */
        if (request.status != NULL)
            SET_BIT(*(uint8_t *) request.status, MODBUS_REQUEST_CPLT_Msk);
    }

    /* Установить статус - Устройство не ответило */
    if (request.status != NULL)
        SET_BIT(*(uint8_t *) request.status, MODBUS_REQUEST_NRSP_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Диагностика Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @return          Исключительное событие Modbus @ref enum modbus_exception
 */
static enum modbus_exception modbus_diagnostic(struct modbus_handle *handle)
{
    if (handle->data_size != 8)
        return MODBUS_ILLEGAL_FUNCTION;

    uint16_t func = (handle->data[2] << 8) | handle->data[3];

    switch (func) {
        case 0x00:
            break;
        case 0x0A:
            handle->bus_message_count = 0;
            handle->bus_communication_error_count = 0;
            handle->bus_exception_error_count = 0;
            break;
        case 0x0B:
            handle->data[4] = (uint8_t) (handle->bus_message_count >> 8);
            handle->data[5] = (uint8_t)  handle->bus_message_count;
            break;
        case 0x0C:
            handle->data[4] = (uint8_t) (handle->bus_communication_error_count >> 8);
            handle->data[5] = (uint8_t)  handle->bus_communication_error_count;
            break;
        case 0x0D:
            handle->data[4] = (uint8_t) (handle->bus_exception_error_count >> 8);
            handle->data[5] = (uint8_t)  handle->bus_exception_error_count;
            break;
        case 0x0F:
            handle->data[4] = (uint8_t) (handle->server_no_response_count >> 8);
            handle->data[5] = (uint8_t)  handle->server_no_response_count;
            break;
        default:
            return MODBUS_ILLEGAL_DATA_VALUE;
    }

    /* Установить размер ответа (без контрольной суммы) */
    handle->data_size = 6;

    return MODBUS_NO_EXCEPTION;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Идентификатор удаленного устройства Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @return          Исключительное событие Modbus @ref enum modbus_exception
 */
static enum modbus_exception modbus_report_server_id(struct modbus_handle *handle)
{
    if (handle->data_size != 4)
        return MODBUS_ILLEGAL_FUNCTION;

    /* Заполнить ответ */
    handle->data[2]  = 8;
    handle->data[3]  = device_id;
    handle->data[4]  = 0xFF;
    handle->data[5]  = (uint8_t) (sw_version.major >> 8);
    handle->data[6]  = (uint8_t)  sw_version.major;
    handle->data[7]  = (uint8_t) (sw_version.minor >> 8);
    handle->data[8]  = (uint8_t)  sw_version.minor;
    handle->data[9]  = (uint8_t) (sw_version.build >> 8);
    handle->data[10] = (uint8_t)  sw_version.build;

    /* Установить размер ответа (без контрольной суммы) */
    handle->data_size = 11;

    return MODBUS_NO_EXCEPTION;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Запрос - Диагностика Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @param[in]       request: Указатель на структуру данных запроса Modbus
 */
static void modbus_request_diagnostic(struct modbus_handle *handle, struct modbus_request *request)
{
    handle->data[0] = request->server_address;
    handle->data[1] = request->function;
    handle->data[2] = 0;
    handle->data[3] = 0;
    handle->data[4] = 0;
    handle->data[5] = 0;

    /* Установить размер запроса (без контрольной суммы) */
    handle->data_size = 6;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Запрос - Чтение регистров Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @param[in]       request: Указатель на структуру данных запроса Modbus
 */
static void modbus_request_read_holding_registers(struct modbus_handle *handle, struct modbus_request *request)
{
    handle->data[0] = request->server_address;
    handle->data[1] = request->function;
    handle->data[2] = (uint8_t) (request->register_address >> 8);
    handle->data[3] = (uint8_t)  request->register_address;
    handle->data[4] = (uint8_t) (request->register_count >> 8);
    handle->data[5] = (uint8_t)  request->register_count;

    /* Установить размер запроса (без контрольной суммы) */
    handle->data_size = 6;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Запрос - Запись регистров Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @param[in]       request: Указатель на структуру данных запроса Modbus
 */
static void modbus_request_write_multiple_registers(struct modbus_handle *handle, struct modbus_request *request)
{
    handle->data[0] = request->server_address;
    handle->data[1] = request->function;
    handle->data[2] = (uint8_t) (request->register_address >> 8);
    handle->data[3] = (uint8_t)  request->register_address;
    handle->data[4] = (uint8_t) (request->register_count >> 8);
    handle->data[5] = (uint8_t)  request->register_count;
    handle->data[6] = request->register_count * 2;

    /* Скопировать данные */
    taskENTER_CRITICAL();
    {
        const uint8_t *src = (uint8_t *) (request->data + (request->data_size - 1));
        uint8_t *dest = (uint8_t *) &handle->data[7];

        for (register size_t i = 0; i < request->data_size; i++)
            *dest++ = *src--;

        if (request->data_size % 2)
            *dest++ = 0;
    }
    taskEXIT_CRITICAL();

    /* Установить размер запроса (без контрольной суммы) */
    handle->data_size = handle->data[6] + 7;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Ответ - Чтение регистров Modbus
 *
 * @param[in]       handle: Указатель на структуру данных обработчика Modbus
 * @param[in]       request: Указатель на структуру данных запроса Modbus
 */
static void modbus_response_read_holding_registers(struct modbus_handle *handle, struct modbus_request *request)
{
    /* Скопировать данные */
    taskENTER_CRITICAL();
    {
        const uint8_t *src = (uint8_t *) &handle->data[3];
        uint8_t *dest =  (uint8_t *) (request->data + (request->data_size - 1));

        for (register size_t i = 0; i < request->data_size; i++)
            *dest-- = *src++;
    }
    taskEXIT_CRITICAL();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Контрольная сумма Modbus
 *
 * @param[in]       data: Указатель на данные
 * @param[in]       data: Размер данных
 * @return          Контрольная сумма CRC16
 */
static uint16_t modbus_crc(const void *data, size_t size)
{
    static const uint8_t msb_table[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    };

    static const uint8_t lsb_table[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
        0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
        0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
        0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
        0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
        0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
        0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
        0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
        0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
        0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
        0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE,
        0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
        0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
        0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
        0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
        0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62,
        0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
        0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
        0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
        0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
        0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
        0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76,
        0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
        0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
        0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
        0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
        0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
        0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A,
        0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
        0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40,
    };

    /* Проверить наличие данных */
    if (data == NULL || size == 0)
        return UINT16_MAX;

    /* Указатель на данные */
    const uint8_t *pdata = (uint8_t *) data;

    /* Инициализировать начальное значение */
    uint8_t msb = 0xFF;
    uint8_t lsb = 0xFF;

    /* Рассчитать значение контрольной суммы */
    uint32_t index;

    while (size > 0) {
        index = msb ^ *pdata;

        msb = lsb ^ msb_table[index];
        lsb = lsb_table[index];

        pdata++;
        size--;
    }

    return (msb << 8) | lsb;
}
/* ------------------------------------------------------------------------- */
