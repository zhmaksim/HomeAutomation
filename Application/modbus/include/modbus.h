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

#ifndef MODBUS_H_
#define MODBUS_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "usart.h"
#include "led.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define MODBUS_COUNT                2

#define MODBUS_DATA_SIZE            256

#define MODBUS_QUEUE_SIZE           32

#define MODBUS_REQUEST_CPLT_Pos     0
#define MODBUS_REQUEST_CPLT_Msk     HAL_BITMASK(0x01, MODBUS_REQUEST_CPLT_Pos)
#define MODBUS_REQUEST_CPLT         MODBUS_REQUEST_CPLT_Msk

#define MODBUS_REQUEST_NRSP_Pos     1
#define MODBUS_REQUEST_NRSP_Msk     HAL_BITMASK(0x01, MODBUS_REQUEST_NRSP_Pos)
#define MODBUS_REQUEST_NRSP         MODBUS_REQUEST_NRSP_Msk

#define MODBUS_REQUEST_ERR_Pos      7
#define MODBUS_REQUEST_ERR_Msk      HAL_BITMASK(0x01, MODBUS_REQUEST_ERR_Pos)
#define MODBUS_REQUEST_ERR          MODBUS_REQUEST_ERR_Msk

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение перечисления идентификаторов Modbus
 */
enum modbus_id {
    MODBUS0,
    MODBUS1,
};


/**
 * @brief           Определение перечисления функций Modbus
 */
enum modbus_function {
    MODBUS_NO_FUNCTION,
    MODBUS_DIAGNOSTIC = 0x08,
    MODBUS_READ_HOLDING_REGISTERS = 0x03,
    MODBUS_WRITE_MULTIPLE_REGISTERS = 0x10,
    MODBUS_REPORT_SERVER_ID = 0x11,
};


/**
 * @brief           Определение перечисления исключительных событий Modbus
 */
enum modbus_exception {
    MODBUS_NO_EXCEPTION,
    MODBUS_ILLEGAL_FUNCTION,
    MODBUS_ILLEGAL_DATA_ADDRESS,
    MODBUS_ILLEGAL_DATA_VALUE,
    MODBUS_DEVICE_FAILURE,
};


/**
 * @brief           Определение перечисления режимов работы Modbus
 */
enum modbus_mode {
    MODBUS_SLAVE,
    MODBUS_MASTER,
};


/**
 * @brief           Определение перечисления состояний Modbus
 */
enum modbus_state {
    MODBUS_NOT_INIT,
    MODBUS_IDLE,
    MODBUS_RECEIVE_WAIT,
    MODBUS_RECEIVE,
    MODBUS_RECEIVE_COMPLETED,
    MODBUS_TRANSMIT,
    MODBUS_TIMEOUT_RESPONSE = 0x80,
};


/**
 * @brief           Определение структуры данных запроса Modbus
 */
struct modbus_request {
    uint8_t                 server_address;     /*!< Адрес подчиненного устройства */

    uint8_t                 server_id;          /*!< Идентификатор подчиненного устройства */

    enum modbus_function    function;           /*!< Функция */

    uint16_t                register_address;   /*!< Адрес регистра */

    uint8_t                 register_count;     /*!< Количество регистров */

    void                   *data;               /*!< Указатель на данные */

    size_t                  data_size;          /*!< Размер данных */

    void                   *status;             /*!< Статус */
};


/**
 * @brief           Определение структуры данных обработчика Modbus
 */
struct modbus_handle {
    struct usart_handle            *usart;                                  /*!< Указатель на структуру данных обработчика USART */

    SemaphoreHandle_t               usart_mutex;                            /*!< Mutex USART (FreeRTOS) */

    EventGroupHandle_t              usart_event_group;                      /*!< Event Group USART (FreeRTOS) */

    struct led_handle              *led_rx;                                 /*!< Указатель на структуру данных обработчика LED RX */

    struct led_handle              *led_tx;                                 /*!< Указатель на структуру данных обработчика LED TX */

    uint8_t                         server_address;                         /*!< Адрес удаленного устройства (в режиме Slave) */

    enum modbus_mode                mode;                                   /*!< Режим работы */

    enum modbus_state               state;                                  /*!< Состояние */

    uint8_t                         data[MODBUS_DATA_SIZE];                 /*!< Данные */

    size_t                          data_size;                              /*!< Размер данных */

    QueueHandle_t                   queue;                                  /*!< Очередь */

    uint16_t                        bus_message_count;                      /*!< Количество собощений */

    uint16_t                        bus_communication_error_count;          /*!< Количество ошибок CRC */

    uint16_t                        bus_exception_error_count;              /*!< Количество ответов об исключениях */

    uint16_t                        server_no_response_count;               /*!< Количество сообщений без ответа */

    uint32_t                        timer;                                  /*!< Таймер таймаута */

    uint32_t                        timeout35;                              /*!< Таймаут 3.5 символа */

    uint32_t                        timeout_response;                       /*!< Таймаут ответа от устройства */
};

/* Exported variables ------------------------------------------------------ */

extern struct modbus_handle modbus[MODBUS_COUNT];

/* Exported function prototypes -------------------------------------------- */

void modbus_init(struct modbus_handle *handle);

void modbus_usart_transmit_completed_it_handler(struct modbus_handle *handle);

void modbus_usart_receive_completed_it_handler(struct modbus_handle *handle);

void modbus_usart_error_it_handler(struct modbus_handle *handle);

void modbus_tim_it_handler(struct modbus_handle *handle);

hal_status_t modbus_request(struct modbus_handle *handle, const struct modbus_request *request);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MODBUS_H_ */
