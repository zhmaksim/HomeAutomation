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

#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_dma.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_spi_setup_mode(spi_t *instance, enum spi_mode mode);

static void hal_spi_setup_div(spi_t *instance, enum spi_div div);

static void hal_spi_setup_frame(spi_t *instance, enum spi_frame_size size, enum spi_frame_format format);

static void hal_spi_setup_nss(spi_t *instance);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_init(struct spi_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_spi_setup_mode(handle->instance, handle->init.mode);
    hal_spi_setup_div(handle->instance, handle->init.div);
    hal_spi_setup_frame(handle->instance, handle->init.frame_size, handle->init.frame_format);
    hal_spi_setup_nss(handle->instance);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить режим работы SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 * @param[in]       mode: Режим работы SPI
 */
static void hal_spi_setup_mode(spi_t *instance, enum spi_mode mode)
{
    /* Установить MSRT */
    SET_BIT(instance->CR1, SPI_CR1_MSTR_Msk);

    /* Настроить CPOL и CPHA */
    if (mode == SPI_MODE0) {
        CLEAR_BIT(instance->CR1,
                  SPI_CR1_CPOL_Msk
                | SPI_CR1_CPHA_Msk);
    } else if (mode == SPI_MODE1) {
        MODIFY_REG(instance->CR1,
                   SPI_CR1_CPHA_Msk,
                   SPI_CR1_CPOL_Msk);
    } else if (mode == SPI_MODE2) {
        MODIFY_REG(instance->CR1,
                   SPI_CR1_CPOL_Msk,
                   SPI_CR1_CPHA_Msk);
    } else if (mode == SPI_MODE3) {
        SET_BIT(instance->CR1,
                SPI_CR1_CPOL_Msk
              | SPI_CR1_CPHA_Msk);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить делитель часов SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 * @param[in]       div: Делитель часов SPI
 */
static void hal_spi_setup_div(spi_t *instance, enum spi_div div)
{
    MODIFY_REG(instance->CR1,
               SPI_CR1_BR_Msk,
               div << SPI_CR1_BR_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить фрейм данных SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 * @param[in]       size: Размер фрейма SPI
 * @param[in]       format: Формат фрейма SPI
 */
static void hal_spi_setup_frame(spi_t *instance, enum spi_frame_size size, enum spi_frame_format format)
{
    MODIFY_REG(instance->CR1,
               SPI_CR1_DFF_Msk
             | SPI_CR1_LSBFIRST_Msk,
               size << SPI_CR1_DFF_Pos
             | format << SPI_CR1_LSBFIRST_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить сигнал NSS SPI
 *
 * @param[in]       instance: Указатель на структуру данных SPI
 */
static void hal_spi_setup_nss(spi_t *instance)
{
    /* Настроить NSS = Software */
    SET_BIT(instance->CR1,
            SPI_CR1_SSM_Msk
          | SPI_CR1_SSI_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывания SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_it_handler(struct spi_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    if ((handle->tx_data_size > 0) || (handle->rx_data_size > 0)) {
        uint32_t SR = READ_REG(handle->instance->SR);

        /* Передача данных */
        if (READ_BIT(SR, SPI_SR_TXE_Msk) && handle->is_tx_allowed) {
            if (handle->tx_data_size > 0) {
                volatile const uint8_t *pdata = handle->tx_data;

                WRITE_REG(handle->instance->DR, *pdata);

                handle->tx_data++;
                handle->tx_data_size--;
            } else if (handle->rx_data_size > 0) {
                WRITE_REG(handle->instance->DR, 0);
            }

            handle->is_tx_allowed = false;
        }
        /* Прием данных */
        if (READ_BIT(SR, SPI_SR_RXNE_Msk)) {
            if (handle->rx_data_size > 0) {
                volatile uint8_t *pdata = handle->rx_data;

                *pdata = READ_REG(handle->instance->DR);

                handle->rx_data++;
                handle->rx_data_size--;
            } else {
                READ_REG(handle->instance->DR);
            }

            handle->is_tx_allowed = true;
        }
        /* Проверка ошибки OVR и таймаута приема-передачи данных */
        if (READ_BIT(SR, SPI_SR_OVR_Msk)) {
            READ_REG(handle->instance->DR);
            READ_REG(handle->instance->SR);

            /* Выключить прерывания */
            CLEAR_BIT(handle->instance->CR2,
                      SPI_CR2_TXEIE_Msk
                    | SPI_CR2_RXNEIE_Msk
                    | SPI_CR2_ERRIE_Msk);

            /* Сбросить данные */
            handle->is_tx_allowed = false;
            handle->tx_data = NULL;
            handle->tx_data_size = 0;
            handle->rx_data = NULL;
            handle->rx_data_size = 0;

            if (handle->error_callback != NULL)
                handle->error_callback(handle);
        }
    } else if (!READ_BIT(handle->instance->SR, SPI_SR_BSY_Msk)) {
        /* Выключить прерывания */
        CLEAR_BIT(handle->instance->CR2,
                  SPI_CR2_TXEIE_Msk
                | SPI_CR2_RXNEIE_Msk
                | SPI_CR2_ERRIE_Msk);

        /* Сбросить данные */
        handle->is_tx_allowed = false;
        handle->tx_data = NULL;
        handle->tx_data_size = 0;
        handle->rx_data = NULL;
        handle->rx_data_size = 0;

        if (handle->transmit_receive_completed_callback != NULL)
            handle->transmit_receive_completed_callback(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание завершения передачи данных DMA SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_transmit_receive_completed_dma_it_handler(struct spi_handle *handle)
{
    /* Ожидание готовности SPI */
    while (READ_BIT(handle->instance->SR, SPI_SR_BSY_Msk))
        continue;

    /* Выключить прием-передачу данных DMA */
    CLEAR_BIT(handle->instance->CR2,
              SPI_CR2_TXDMAEN_Msk
            | SPI_CR2_RXDMAEN_Msk);

    if (handle->transmit_receive_completed_callback != NULL)
        handle->transmit_receive_completed_callback(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание ошибки передачи данных DMA SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_error_dma_it_handler(struct spi_handle *handle)
{
    /* Ожидание готовности SPI */
    while (READ_BIT(handle->instance->SR, SPI_SR_BSY_Msk))
        continue;

    /* Выключить прием-передачу данных DMA */
    CLEAR_BIT(handle->instance->CR2,
              SPI_CR2_TXDMAEN_Msk
            | SPI_CR2_RXDMAEN_Msk);

    if (handle->error_callback != NULL)
        handle->error_callback(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_enable(struct spi_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR1, SPI_CR1_SPE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 */
void hal_spi_disable(struct spi_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR1, SPI_CR1_SPE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием-передача данных SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 * @param[in]       tx_data Указатель на передаваемые данные
 * @param[in]       tx_data_size Размер передаваемых данных
 * @param[out]      rx_data Указатель на принимаемые данные
 * @param[in]       rx_data_size Размер принимаемых данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_spi_transmit_receive(struct spi_handle *handle,
                                      const void *tx_data,
                                      size_t tx_data_size,
                                      void *rx_data,
                                      size_t rx_data_size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    bool is_tx_allowed = true;

    while ((tx_data_size > 0) || (rx_data_size > 0)) {
        uint32_t SR = READ_REG(handle->instance->SR);

        /* Передача данных */
        if (READ_BIT(SR, SPI_SR_TXE_Msk) && is_tx_allowed) {
            if (tx_data_size > 0) {
                const uint8_t *pdata = tx_data;

                WRITE_REG(handle->instance->DR, *pdata);

                tx_data++;
                tx_data_size--;
            } else if (rx_data_size > 0) {
                WRITE_REG(handle->instance->DR, 0);
            }

            is_tx_allowed = false;
        }
        /* Прием данных */
        if (READ_BIT(SR, SPI_SR_RXNE_Msk)) {
            if (rx_data_size > 0) {
                uint8_t *pdata = rx_data;

                *pdata = READ_REG(handle->instance->DR);

                rx_data++;
                rx_data_size--;
            } else {
                READ_REG(handle->instance->DR);
            }

            is_tx_allowed = true;
        }
        /* Проверка ошибки OVR и таймаута приема-передачи данных */
        if (READ_BIT(SR, SPI_SR_OVR_Msk)) {
            READ_REG(handle->instance->DR);
            READ_REG(handle->instance->SR);

            return HAL_ERROR;
        }
    }

    /* Ожидание готовности SPI */
    while (READ_BIT(handle->instance->SR, SPI_SR_BSY_Msk))
        continue;

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием-передача данных SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 * @param[in]       tx_data: Указатель на передаваемые данные
 * @param[in]       tx_data_size: Размер передаваемых данных
 * @param[out]      rx_data: Указатель на принимаемые данные
 * @param[in]       rx_data_size: Размер принимаемых данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_spi_transmit_receive_it(struct spi_handle *handle,
                                         const void *tx_data,
                                         size_t tx_data_size,
                                         void *rx_data,
                                         size_t rx_data_size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить наличие данных */
    if ((tx_data == NULL || tx_data_size == 0) &&
        (rx_data == NULL || rx_data_size == 0)) return HAL_ERROR;

    /* Установить данные */
    handle->is_tx_allowed = true;
    handle->tx_data = (void *) tx_data;
    handle->tx_data_size = tx_data_size;
    handle->rx_data = (void *) rx_data;
    handle->rx_data_size = rx_data_size;

    /* Включить прерывания */
    SET_BIT(handle->instance->CR2,
            SPI_CR2_TXEIE_Msk
          | SPI_CR2_RXNEIE_Msk
          | SPI_CR2_ERRIE_Msk);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прием-передача данных DMA SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 * @param[in]       tx_data: Указатель на передаваемые данные
 * @param[in]       tx_data_size: Размер передаваемых данных
 * @param[out]      rx_data: Указатель на принимаемые данные
 * @param[in]       rx_data_size: Размер принимаемых данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_spi_transmit_receive_dma(struct spi_handle *handle,
                                          const void *tx_data,
                                          size_t tx_data_size,
                                          void *rx_data,
                                          size_t rx_data_size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->dma_tx != NULL);
    assert(handle->dma_rx != NULL);

    /* Проверить наличие данных */
    if ((tx_data == NULL || tx_data_size == 0) &&
        (rx_data == NULL || rx_data_size == 0)) return HAL_ERROR;

    /* Включить прием-передачу данных DMA */
    SET_BIT(handle->instance->CR2,
            SPI_CR2_TXDMAEN_Msk
          | SPI_CR2_RXDMAEN_Msk);

    /* Указатель на данные периферии */
    void *peripheral_data = (void *) &handle->instance->DR;

    /* Запустить прием данных DMA */
    if (hal_dma_transfer_it(handle->dma_rx,
                            peripheral_data,
                            (void *) rx_data,
                            rx_data_size) == HAL_OK) {
        /* Запустить передачу данных DMA */
        if (hal_dma_transfer_it(handle->dma_tx,
                                peripheral_data,
                                (void *) tx_data,
                                tx_data_size) == HAL_OK) {
            return HAL_OK;
        }
    }

    /* Выключить прием-передачу данных DMA */
    CLEAR_BIT(handle->instance->CR2,
              SPI_CR2_TXDMAEN_Msk
            | SPI_CR2_RXDMAEN_Msk);

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 * @param[in]       callback_id: Идентификатор функции обратного вызова SPI
 * @param[in]       callback: Указатель на функцию обратного вызова SPI
 */
void hal_spi_register_callback(struct spi_handle *handle,
                               enum spi_callback_id callback_id,
                               void (*callback)(struct spi_handle *handle))
{
    assert(handle != NULL);

    switch (callback_id) {
        case SPI_TRANSMIT_RECEIVE_COMPLETED_CALLBACK:
            handle->transmit_receive_completed_callback = callback;
            break;
        case SPI_ERROR_CALLBACK:
            handle->error_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова SPI
 *
 * @param[in]       handle: Указатель на структуру данных обработчика SPI
 * @param[in]       callback_id: Идентификатор функции обратного вызова SPI
 */
void hal_spi_unregister_callback(struct spi_handle *handle,
                                 enum spi_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case SPI_TRANSMIT_RECEIVE_COMPLETED_CALLBACK:
            handle->transmit_receive_completed_callback = NULL;
            break;
        case SPI_ERROR_CALLBACK:
            handle->error_callback = NULL;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
