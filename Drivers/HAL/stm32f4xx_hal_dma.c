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

#include "stm32f4xx_hal_dma.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

static const uint32_t DMA_TCIF_Msk[] = {
    DMA_LISR_TCIF0_Msk,
    DMA_LISR_TCIF1_Msk,
    DMA_LISR_TCIF2_Msk,
    DMA_LISR_TCIF3_Msk,
    DMA_HISR_TCIF4_Msk,
    DMA_HISR_TCIF5_Msk,
    DMA_HISR_TCIF6_Msk,
    DMA_HISR_TCIF7_Msk,
};

static const uint32_t DMA_HTIF_Msk[] = {
    DMA_LISR_HTIF0_Msk,
    DMA_LISR_HTIF1_Msk,
    DMA_LISR_HTIF2_Msk,
    DMA_LISR_HTIF3_Msk,
    DMA_HISR_HTIF4_Msk,
    DMA_HISR_HTIF5_Msk,
    DMA_HISR_HTIF6_Msk,
    DMA_HISR_HTIF7_Msk,
};

static const uint32_t DMA_TEIF_Msk[] = {
    DMA_LISR_TEIF0_Msk,
    DMA_LISR_TEIF1_Msk,
    DMA_LISR_TEIF2_Msk,
    DMA_LISR_TEIF3_Msk,
    DMA_HISR_TEIF4_Msk,
    DMA_HISR_TEIF5_Msk,
    DMA_HISR_TEIF6_Msk,
    DMA_HISR_TEIF7_Msk,
};

/* Private function prototypes --------------------------------------------- */

static void hal_dma_setup_channel(dma_stream_t *instance_stream, enum dma_channel channel);

static void hal_dma_setup_priority(dma_stream_t *instance_stream, enum dma_priority priority);

static void hal_dma_setup_transfer_direction(dma_stream_t *instance_stream, enum dma_transfer_direction dir);

static void hal_dma_setup_peripheral_data_size(dma_stream_t *instance_stream, enum dma_data_size data_size);

static void hal_dma_setup_peripheral_inc_mode(dma_stream_t *instance_stream, hal_state_t state);

static void hal_dma_setup_memory_data_size(dma_stream_t *instance_stream, enum dma_data_size data_size);

static void hal_dma_setup_memory_inc_mode(dma_stream_t *instance_stream, hal_state_t state);

static void hal_dma_setup_circ_mode(dma_stream_t *instance_stream, hal_state_t state);

static void hal_dma_setup_fifo(dma_stream_t *instance_stream);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_init(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->instance_stream != NULL);

    hal_dma_disable(handle);

    hal_dma_setup_channel(handle->instance_stream, handle->init.channel);
    hal_dma_setup_priority(handle->instance_stream, handle->init.priority);
    hal_dma_setup_transfer_direction(handle->instance_stream, handle->init.transfer_direction);
    hal_dma_setup_peripheral_data_size(handle->instance_stream, handle->init.peripheral_data_size);
    hal_dma_setup_peripheral_inc_mode(handle->instance_stream, handle->init.peripheral_inc_mode);
    hal_dma_setup_memory_data_size(handle->instance_stream, handle->init.memory_data_size);
    hal_dma_setup_memory_inc_mode(handle->instance_stream, handle->init.memory_inc_mode);
    hal_dma_setup_circ_mode(handle->instance_stream, handle->init.circ_mode);
    hal_dma_setup_fifo(handle->instance_stream);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить номер канала потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       channel: Номер канала DMA
 */
static void hal_dma_setup_channel(dma_stream_t *instance_stream, enum dma_channel channel)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_CHSEL_Msk,
               channel << DMA_SxCR_CHSEL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить приоритет потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       priority: Приоритет DMA
 */
static void hal_dma_setup_priority(dma_stream_t *instance_stream, enum dma_priority priority)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_PL_Msk,
               priority << DMA_SxCR_PL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить направление передачи данных потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       dir: Направление передачи данных DMA
 */
static void hal_dma_setup_transfer_direction(dma_stream_t *instance_stream, enum dma_transfer_direction dir)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_DIR_Msk,
               dir << DMA_SxCR_DIR_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить размер данных периферии потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       data_size: Размер данных DMA
 */
static void hal_dma_setup_peripheral_data_size(dma_stream_t *instance_stream, enum dma_data_size data_size)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_PSIZE_Msk,
               data_size << DMA_SxCR_PSIZE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить режим инкремента периферии потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       state: Состояние инкремента периферии DMA
 */
static void hal_dma_setup_peripheral_inc_mode(dma_stream_t *instance_stream, hal_state_t state)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_PINC_Msk,
               state << DMA_SxCR_PINC_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить размер данных памяти потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       data_size: Размер данных DMA
 */
static void hal_dma_setup_memory_data_size(dma_stream_t *instance_stream, enum dma_data_size data_size)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_MSIZE_Msk,
               data_size << DMA_SxCR_MSIZE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить режим инкремента памяти потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       state: Состояние инкремента памяти DMA
 */
static void hal_dma_setup_memory_inc_mode(dma_stream_t *instance_stream, hal_state_t state)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_MINC_Msk,
               state << DMA_SxCR_MINC_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить циклический режим потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 * @param[in]       state: Состояние циклического режима DMA
 */
static void hal_dma_setup_circ_mode(dma_stream_t *instance_stream, hal_state_t state)
{
    MODIFY_REG(instance_stream->CR,
               DMA_SxCR_CIRC_Msk,
               state << DMA_SxCR_CIRC_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить FIFO потока DMA
 *
 * @param[in]       instance_stream: Указатель на структуру данных потока DMA
 */
static void hal_dma_setup_fifo(dma_stream_t *instance_stream)
{
    CLEAR_BIT(instance_stream->FCR, DMA_SxFCR_DMDIS_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_it_handler(struct dma_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->instance_stream != NULL);

    /* Обработать статусы DMA */
    volatile uint32_t *ISR = NULL;
    volatile uint32_t *IFCR = NULL;

    if (handle->stream_nb < 4) {
        ISR = &handle->instance->LISR;
        IFCR = &handle->instance->LIFCR;
    } else {
        ISR = &handle->instance->HISR;
        IFCR = &handle->instance->HIFCR;
    }

    if (READ_BIT(*ISR, DMA_TEIF_Msk[handle->stream_nb])) {
        SET_BIT(*IFCR, DMA_TEIF_Msk[handle->stream_nb]);

        if (handle->transfer_error_callback != NULL) {
            handle->transfer_error_callback(handle);
        }
    } else if (READ_BIT(*ISR, DMA_TCIF_Msk[handle->stream_nb])) {
        SET_BIT(*IFCR, DMA_TCIF_Msk[handle->stream_nb]);

        if (handle->transfer_completed_callback != NULL) {
            handle->transfer_completed_callback(handle);
        }
    }

    /* Выключить прерывания DMA */
    CLEAR_BIT(handle->instance_stream->CR,
              DMA_SxCR_TCIE_Msk
            | DMA_SxCR_TEIE_Msk);

    /* Выключить DMA */
    hal_dma_disable(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_enable(struct dma_handle *handle)
{
    SET_BIT(handle->instance_stream->CR, DMA_SxCR_EN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_disable(struct dma_handle *handle)
{
    CLEAR_BIT(handle->instance_stream->CR, DMA_SxCR_EN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передать данные DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 * @param[in]       peripheral_data: Указатель на данные периферии
 * @param[in]       memory_data: Указатель на данные памяти
 * @param[in]       size: Размер данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_dma_transfer_it(struct dma_handle *handle,
                                 void *peripheral_data,
                                 void *memory_data,
                                 size_t size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);
    assert(handle->instance_stream != NULL);

    /* Проверить наличие данных */
    if (peripheral_data == NULL ||
        memory_data == NULL ||
        size == 0) return HAL_ERROR;

    /* Выключить DMA */
    hal_dma_disable(handle);
    /* Ожидание выключения DMA перед настройкой */
    while (READ_BIT(handle->instance_stream->CR, DMA_SxCR_EN_Msk))
        continue;

    /* Сбросить статусы DMA */
    volatile uint32_t *IFCR = NULL;

    if (handle->stream_nb < 4) {
        IFCR = &handle->instance->LIFCR;
    } else {
        IFCR = &handle->instance->HIFCR;
    }

    SET_BIT(*IFCR,
            DMA_TCIF_Msk[handle->stream_nb]
          | DMA_HTIF_Msk[handle->stream_nb]
          | DMA_TEIF_Msk[handle->stream_nb]);

    /* Включить прерывания DMA */
    SET_BIT(handle->instance_stream->CR,
            DMA_SxCR_TCIE_Msk
          | DMA_SxCR_TEIE_Msk);

    /* Установить адрес периферии */
    WRITE_REG(handle->instance_stream->PAR, (uint32_t) peripheral_data);
    /* Установить адрес памяти */
    WRITE_REG(handle->instance_stream->M0AR, (uint32_t) memory_data);
    /* Установить количество данных */
    WRITE_REG(handle->instance_stream->NDTR, size);

    /* Включить DMA */
    hal_dma_enable(handle);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прервать передачу данных DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 */
void hal_dma_abort_transfer_it(struct dma_handle *handle)
{
    /* Выключить прерывания DMA */
    CLEAR_BIT(handle->instance_stream->CR,
              DMA_SxCR_TCIE_Msk
            | DMA_SxCR_TEIE_Msk);

    /* Выключить DMA */
    hal_dma_disable(handle);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 * @param[in]       callback_id: Идентификатор функции обратного вызова DMA
 * @param[in]       callback: Указатель на функцию обратного вызова DMA
 */
void hal_dma_register_callback(struct dma_handle *handle,
                               enum dma_callback_id callback_id,
                               void (*callback)(struct dma_handle *handle))
{
    assert(handle != NULL);

    switch (callback_id) {
        case DMA_TRANSFER_COMPLETED_CALLBACK:
            handle->transfer_completed_callback = callback;
            break;
        case DMA_TRANSFER_ERROR_CALLBACK:
            handle->transfer_error_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова DMA
 *
 * @param[in]       handle: Указатель на структуру данных обработчика DMA
 * @param[in]       callback_id: Идентификатор функции обратного вызова DMA
 */
void hal_dma_unregister_callback(struct dma_handle *handle,
                                 enum dma_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case DMA_TRANSFER_COMPLETED_CALLBACK:
            handle->transfer_completed_callback = NULL;
            break;
        case DMA_TRANSFER_ERROR_CALLBACK:
            handle->transfer_error_callback = NULL;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
