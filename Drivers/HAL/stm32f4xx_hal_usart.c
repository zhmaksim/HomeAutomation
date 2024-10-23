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

#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_usart_setup_baudrate(usart_t *instance, uint32_t frequency, uint32_t baudrate, enum usart_oversampling_mode over);

static void hal_usart_setup_word_length(usart_t *instance, enum usart_word_length word_length);

static void hal_usart_setup_stop_bit(usart_t *instance, enum usart_stop_bit stop_bit);

static void hal_usart_setup_parity(usart_t *instance, enum usart_parity parity);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_init(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_usart_setup_baudrate(handle->instance, handle->init.frequency, handle->init.baudrate, handle->init.oversampling);
    hal_usart_setup_word_length(handle->instance, handle->init.word_length);
    hal_usart_setup_stop_bit(handle->instance, handle->init.stop_bit);
    hal_usart_setup_parity(handle->instance, handle->init.parity);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить скорость работы USART
 *
 * @param[in]       instance: Указатель на структуру данных USART
 * @param[in]       frequency: Частота тактирования (Гц)
 * @param[in]       baudrate: Скорость
 * @param[in]       over: Режим передискретизации USART
 */
static void hal_usart_setup_baudrate(usart_t *instance, uint32_t frequency, uint32_t baudrate, enum usart_oversampling_mode over)
{
    uint32_t mantissa = (frequency / (8 * (2 - over) * baudrate));
    uint32_t fraction = ((frequency - (mantissa * (8 * (2 - over) * baudrate))) / baudrate);

    uint32_t BRR = mantissa << USART_BRR_DIV_Mantissa_Pos
                 | fraction << USART_BRR_DIV_Fraction_Pos;

    WRITE_REG(instance->BRR, BRR);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить длину слова USART
 *
 * @param[in]       instance: Указатель на структуру данных USART
 * @param[in]       word_length: Длина слова USART
 */
static void hal_usart_setup_word_length(usart_t *instance, enum usart_word_length word_length)
{
    MODIFY_REG(instance->CR1,
               USART_CR1_M_Msk,
               word_length << USART_CR1_M_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить количество стоп-бит USART
 *
 * @param[in]       instance: Указатель на структуру данных USART
 * @param[in]       stop_bit: Количество стоп-бит USART
 */
static void hal_usart_setup_stop_bit(usart_t *instance, enum usart_stop_bit stop_bit)
{
    MODIFY_REG(instance->CR2,
               USART_CR2_STOP_Msk,
               stop_bit << USART_CR2_STOP_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Установить контроль четности USART
 *
 * @param[in]       instance: Указатель на структуру данных USART
 * @param[in]       parity: Контроль четности USART
 */
static void hal_usart_setup_parity(usart_t *instance, enum usart_parity parity)
{
    if (parity == USART_PARITY_NO) {
        CLEAR_BIT(instance->CR1, USART_CR1_PCE_Msk);
    } else {
        MODIFY_REG(instance->CR1,
                   USART_CR1_PS_Msk,
                   USART_CR1_PCE_Msk
                 | parity << USART_CR1_PS_Pos);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывания USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_it_handler(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    uint32_t SR = READ_REG(handle->instance->SR);

    if (READ_BIT(SR, USART_SR_PE_Msk
                   | USART_SR_FE_Msk
                   | USART_SR_NE_Msk
                   | USART_SR_ORE_Msk)) {
        READ_REG(handle->instance->SR);
        READ_REG(handle->instance->DR);

        if (handle->error_callback != NULL)
            handle->error_callback(handle);
    } else if (READ_BIT(SR, USART_SR_RXNE_Msk) &&
               handle->rx_data != NULL &&
               handle->rx_data_size > 0) {
        volatile uint8_t *pdata = (uint8_t *) handle->rx_data;

        if (handle->init.word_length == USART_WORD_8BIT &&
            handle->init.parity != USART_PARITY_NO) {
            *pdata = READ_REG(handle->instance->DR) & 0x7F;
        } else {
            *pdata = READ_REG(handle->instance->DR);
        }

        handle->rx_data++;
        handle->rx_counter++;

        if (handle->rx_counter >= handle->rx_data_size) {
            hal_usart_abort_receive_it(handle);

            if (handle->receive_completed_callback != NULL)
                handle->receive_completed_callback(handle);
        }
    } else if (READ_BIT(SR, USART_SR_TXE_Msk | USART_SR_TC_Msk) &&
               handle->tx_data != NULL &&
               handle->tx_data_size > 0) {
        if (handle->tx_counter < handle->tx_data_size) {
            volatile const uint8_t *pdata = (uint8_t *) handle->tx_data;

            if (handle->init.word_length == USART_WORD_8BIT &&
                handle->init.parity != USART_PARITY_NO) {
                WRITE_REG(handle->instance->DR, *pdata & 0x7F);
            } else {
                WRITE_REG(handle->instance->DR, *pdata);
            }

            handle->tx_data++;
            handle->tx_counter++;
        } else if (READ_BIT(SR, USART_SR_TC_Msk)) {
            hal_usart_abort_transmit_it(handle);

            if (handle->transmit_completed_callback != NULL)
                handle->transmit_completed_callback(handle);
        }
    } else {
        READ_REG(handle->instance->SR);
        READ_REG(handle->instance->DR);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершения передачи данных DMA USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_transmit_completed_dma_it_handler(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Ожидание завершения передачи данных USART */
    while (!READ_BIT(handle->instance->SR, USART_SR_TC_Msk) ||
           !READ_BIT(handle->instance->SR, USART_SR_TXE_Msk))
        continue;

    if (handle->transmit_completed_callback != NULL)
        handle->transmit_completed_callback(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершения приема данных DMA USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_receive_completed_dma_it_handler(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить статусы USART */
    uint32_t SR = READ_REG(handle->instance->SR);

    if (READ_BIT(SR,
                 USART_SR_PE_Msk
               | USART_SR_FE_Msk
               | USART_SR_NE_Msk
               | USART_SR_ORE_Msk)) {
        READ_REG(handle->instance->SR);
        READ_REG(handle->instance->DR);
    }

    if (handle->receive_completed_callback != NULL)
        handle->receive_completed_callback(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание ошибки приема-передачи данных DMA USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_error_dma_it_handler(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Cбросить статусы USART */
    READ_REG(handle->instance->SR);
    READ_REG(handle->instance->DR);

    if (handle->error_callback != NULL)
        handle->error_callback(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_enable(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR1, USART_CR1_UE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_disable(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR1, USART_CR1_UE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить приемник USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_enable_receiver(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR1, USART_CR1_RE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить приемник USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_disable_receiver(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR1, USART_CR1_RE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить передатчик USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_enable_transiver(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR1, USART_CR1_TE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить передатчик USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_disable_transiver(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR1, USART_CR1_TE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить приемник DMA USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_enable_receiver_dma(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR3, USART_CR3_DMAR_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить приемник DMA USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_disable_receiver_dma(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR3, USART_CR3_DMAR_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить передатчик DMA USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_enable_transiver_dma(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR3, USART_CR3_DMAT_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить передатчик DMA USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_disable_transiver_dma(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR3, USART_CR3_DMAT_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Изменить скорость USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       baudrate: Скорость
 */
void hal_usart_change_baudrate(struct usart_handle *handle, uint32_t baudrate)
{
    assert(handle != NULL);

    handle->init.baudrate = baudrate;

    hal_usart_setup_baudrate(handle->instance,
                             handle->init.frequency,
                             handle->init.baudrate,
                             handle->init.oversampling);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Изменить длину слова USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       word_length: Длина слова USART
 */
void hal_usart_change_word_length(struct usart_handle *handle, enum usart_word_length word_length)
{
    assert(handle != NULL);

    handle->init.word_length = word_length;

    hal_usart_setup_word_length(handle->instance, handle->init.word_length);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Изменить контроль четности USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       parity: Контроль четности USART
 */
void hal_usart_change_parity(struct usart_handle *handle, enum usart_parity parity)
{
    assert(handle != NULL);

    handle->init.parity = parity;

    hal_usart_setup_parity(handle->instance, handle->init.parity);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Изменить количество стоп-бит USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       stop_bit: Количество стоп-бит USART
 */
void hal_usart_change_stop_bit(struct usart_handle *handle, enum usart_stop_bit stop_bit)
{
    assert(handle != NULL);

    handle->init.stop_bit = stop_bit;

    hal_usart_setup_stop_bit(handle->instance, handle->init.stop_bit);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передать данные USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_usart_transmit_it(struct usart_handle *handle, const void *data, size_t size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    if (data == NULL || size == 0)
        return HAL_ERROR;

    handle->tx_data = (void *) data;
    handle->tx_data_size = size;
    handle->tx_counter = 0;

    /* Установить состояние передачи данных микросхемы RS485 */
    if (handle->gpio_rs485 != NULL)
        hal_gpio_set_state(handle->gpio_rs485, GPIO_SET);

    SET_BIT(handle->instance->CR1,
            USART_CR1_TXEIE_Msk
          | USART_CR1_TCIE_Msk);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прервать передачу данных USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_abort_transmit_it(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    handle->tx_data = NULL;
    handle->tx_data_size = 0;
    handle->tx_counter = 0;

    CLEAR_BIT(handle->instance->CR1,
              USART_CR1_TXEIE_Msk
            | USART_CR1_TCIE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Принять данные USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_usart_receive_it(struct usart_handle *handle, void *data, size_t size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    if (data == NULL || size == 0)
        return HAL_ERROR;

    handle->rx_data = (void *) data;
    handle->rx_data_size = size;
    handle->rx_counter = 0;

    /* Установить состояние приема данных микросхемы RS485 */
    if (handle->gpio_rs485 != NULL)
        hal_gpio_set_state(handle->gpio_rs485, GPIO_RESET);

    SET_BIT(handle->instance->CR1,
            USART_CR1_RXNEIE_Msk
          | USART_CR1_PEIE_Msk);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прервать прием данных USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_abort_receive_it(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    handle->rx_data = NULL;
    handle->rx_data_size = 0;
    handle->rx_counter = 0;

    CLEAR_BIT(handle->instance->CR1,
              USART_CR1_RXNEIE_Msk
            | USART_CR1_PEIE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передать данные USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_usart_transmit_dma(struct usart_handle *handle, const void *data, size_t size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->dma_tx != NULL);

    /* Проверить наличие данных */
    if (data == NULL || size == 0)
        return HAL_ERROR;

    /* Сбросить статусы USART */
    CLEAR_BIT(handle->instance->SR, USART_SR_TC_Msk);

    /* Установить состояние передачи микросхемы RS485 */
    if (handle->gpio_rs485 != NULL)
        hal_gpio_set_state(handle->gpio_rs485, GPIO_SET);

    /* Указатель на данные периферии */
    void *peripheral_data = (void *) &handle->instance->DR;

    /* Передать данные DMA */
    return hal_dma_transfer_it(handle->dma_tx, peripheral_data, (void *) data, size);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прервать передачу данных USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_abort_transmit_dma(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->dma_tx != NULL);

    hal_dma_disable(handle->dma_tx);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Принять данные USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[out]      data: Указатель на данные
 * @param[in]       size: Размер данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_usart_receive_dma(struct usart_handle *handle, void *data, size_t size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->dma_rx != NULL);

    /* Проверить наличие данных */
    if (data == NULL || size == 0)
        return HAL_ERROR;

    /* Сбросить статусы USART */
    READ_REG(handle->instance->SR);
    READ_REG(handle->instance->DR);

    /* Установить состояние приема микросхемы RS485 */
    if (handle->gpio_rs485 != NULL)
        hal_gpio_set_state(handle->gpio_rs485, GPIO_RESET);

    /* Указатель на данные периферии */
    void *peripheral_data = (void *) &handle->instance->DR;

    /* Принять данные DMA */
    return hal_dma_transfer_it(handle->dma_rx, peripheral_data, (void *) data, size);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прервать прием данных USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 */
void hal_usart_abort_receive_dma(struct usart_handle *handle)
{
    assert(handle != NULL);
    assert(handle->dma_rx != NULL);

    hal_dma_disable(handle->dma_rx);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       callback_id: Идентификатор функции обратного вызова USART
 * @param[in]       callback: Указатель на функцию обратного вызова USART
 */
void hal_usart_register_callback(struct usart_handle *handle,
                                 enum usart_callback_id callback_id,
                                 void (*callback)(struct usart_handle *handle))
{
    assert(handle != NULL);

    switch (callback_id) {
        case USART_TRANSMIT_COMPLETED_CALLBACK:
            handle->transmit_completed_callback = callback;
            break;
        case USART_RECEIVE_COMPLETED_CALLBACK:
            handle->receive_completed_callback = callback;
            break;
        case USART_ERROR_CALLBACK:
            handle->error_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова USART
 *
 * @param[in]       handle: Указатель на структуру данных обработчика USART
 * @param[in]       callback_id: Идентификатор функции обратного вызова USART
 */
void hal_usart_unregister_callback(struct usart_handle *handle,
                                   enum usart_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case USART_TRANSMIT_COMPLETED_CALLBACK:
            handle->transmit_completed_callback = NULL;
            break;
        case USART_RECEIVE_COMPLETED_CALLBACK:
            handle->receive_completed_callback = NULL;
            break;
        case USART_ERROR_CALLBACK:
            handle->error_callback = NULL;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
