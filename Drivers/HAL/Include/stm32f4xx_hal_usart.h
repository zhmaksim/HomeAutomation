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

#ifndef STM32F4XX_HAL_USART_H_
#define STM32F4XX_HAL_USART_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование USART1
 */
#define HAL_USART1_ENABLE_CLOCK() \
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN_Msk)

/**
 * @brief           Включить тактирование USART2
 */
#define HAL_USART2_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN_Msk)

/**
 * @brief           Включить тактирование USART3
 */
#define HAL_USART3_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN_Msk)

/**
 * @brief           Включить тактирование UART4
 */
#define HAL_UART4_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN_Msk)

/**
 * @brief           Включить тактирование UART5
 */
#define HAL_UART5_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN_Msk)

/**
 * @brief           Включить тактирование USART6
 */
#define HAL_USART6_ENABLE_CLOCK() \
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных USART
 */
typedef USART_TypeDef usart_t;


/**
 * @brief           Определение перечисления режимов передискретизации USART
 */
enum usart_oversampling_mode {
    USART_OVERSAMPLING16,
    USART_OVERSAMPLING8,
};


/**
 * @brief           Определение перечисления длины слова USART
 */
enum usart_word_length {
    USART_WORD_8BIT,
    USART_WORD_9BIT,
};


/**
 * @brief           Определение перечисления количества стоп-бит USART
 */
enum usart_stop_bit {
    USART_1STOP_BIT,
    USART_2STOP_BITS = 2,
};


/**
 * @brief           Определение перечисления контроля четности USART
 */
enum usart_parity {
    USART_PARITY_EVEN,
    USART_PARITY_ODD,
    USART_PARITY_NO,
};


/**
 * @brief           Определение перечисления идентификаторов функций обратного вызова USART
 */
enum usart_callback_id {
    USART_TRANSMIT_COMPLETED_CALLBACK,
    USART_RECEIVE_COMPLETED_CALLBACK,
    USART_ERROR_CALLBACK,
};


/**
 * @brief           Определение структуры данных инициализации USART
 */
struct usart_init {
    uint32_t                        frequency;          /*!< Частота тактирования (Гц) */

    uint32_t                        baudrate;           /*!< Скорость */

    enum usart_oversampling_mode    oversampling;       /*!< Режим передискретизации:
                                                            - USART_OVERSAMPLING16
                                                            - USART_OVERSAMPLING8 */

    enum usart_word_length          word_length;        /*!< Длина слова:
                                                            - USART_WORD_8BIT
                                                            - USART_WORD_9BIT */

    enum usart_stop_bit             stop_bit;           /*!< Количество стоп-бит:
                                                            - USART_1STOP_BIT
                                                            - USART_2STOP_BITS */

    enum usart_parity               parity;             /*!< Контроль четности:
                                                            - USART_PARITY_EVEN
                                                            - USART_PARITY_ODD
                                                            - USART_PARITY_NO */
};


/**
 * @brief           Определение структуры данных обработчика USART
 */
struct usart_handle {
    usart_t                *instance;           /*!< Указатель на структуру данных USART */

    void                   *dma_tx;             /*!< Указатель на структуру данных обработчика DMA TX */

    void                   *dma_rx;             /*!< Указатель на структуру данных обработчика DMA RX */

    void                   *gpio_rs485;         /*!< Указатель на структуру данных обработчика GPIO RS485 */

    struct usart_init       init;               /*!< Настройки USART */

    volatile void          *tx_data;            /*!< Указатель на передаваемые данные */

    volatile size_t         tx_data_size;       /*!< Размер передаваемых данных */

    volatile size_t         tx_counter;         /*!< Счетчик переданных данных */

    volatile void          *rx_data;            /*!< Указатель на принимаемые данные */

    volatile size_t         rx_data_size;       /*!< Размер принимаемых данных */

    volatile size_t         rx_counter;         /*!< Счетчик принятых данных */

    /* --- */

    void (*transmit_completed_callback)(struct usart_handle *handle);

    void (*receive_completed_callback)(struct usart_handle *handle);

    void (*error_callback)(struct usart_handle *handle);
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_usart_init(struct usart_handle *handle);

void hal_usart_it_handler(struct usart_handle *handle);

void hal_usart_transmit_completed_dma_it_handler(struct usart_handle *handle);

void hal_usart_receive_completed_dma_it_handler(struct usart_handle *handle);

void hal_usart_error_dma_it_handler(struct usart_handle *handle);

void hal_usart_enable(struct usart_handle *handle);

void hal_usart_disable(struct usart_handle *handle);

void hal_usart_enable_receiver(struct usart_handle *handle);

void hal_usart_disable_receiver(struct usart_handle *handle);

void hal_usart_enable_transiver(struct usart_handle *handle);

void hal_usart_disable_transiver(struct usart_handle *handle);

void hal_usart_enable_receiver_dma(struct usart_handle *handle);

void hal_usart_disable_receiver_dma(struct usart_handle *handle);

void hal_usart_enable_transiver_dma(struct usart_handle *handle);

void hal_usart_disable_transiver_dma(struct usart_handle *handle);

void hal_usart_change_baudrate(struct usart_handle *handle, uint32_t baudrate);

void hal_usart_change_word_length(struct usart_handle *handle, enum usart_word_length word_length);

void hal_usart_change_parity(struct usart_handle *handle, enum usart_parity parity);

void hal_usart_change_stop_bit(struct usart_handle *handle, enum usart_stop_bit stop_bit);

hal_status_t hal_usart_transmit_it(struct usart_handle *handle, const void *data, size_t size);

void hal_usart_abort_transmit_it(struct usart_handle *handle);

hal_status_t hal_usart_receive_it(struct usart_handle *handle, void *data, size_t size);

void hal_usart_abort_receive_it(struct usart_handle *handle);

hal_status_t hal_usart_transmit_dma(struct usart_handle *handle, const void *data, size_t size);

void hal_usart_abort_transmit_dma(struct usart_handle *handle);

hal_status_t hal_usart_receive_dma(struct usart_handle *handle, void *data, size_t size);

void hal_usart_abort_receive_dma(struct usart_handle *handle);

void hal_usart_register_callback(struct usart_handle *handle,
                                 enum usart_callback_id callback_id,
                                 void (*callback)(struct usart_handle *handle));

void hal_usart_unregister_callback(struct usart_handle *handle,
                                   enum usart_callback_id callback_id);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_USART_H_ */
