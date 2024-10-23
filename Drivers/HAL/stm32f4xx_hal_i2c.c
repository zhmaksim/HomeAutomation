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

#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define I2C_WRITE       0
#define I2C_READ        1

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик SysTick */
extern struct systick_handle systick;

/* Private function prototypes --------------------------------------------- */

static void hal_i2c_sw_reset(i2c_t *instance);

static void hal_i2c_setup_frequency(i2c_t *instance, uint32_t frequency);

static void hal_i2c_setup_ccr(i2c_t *instance, uint32_t frequency, enum i2c_mode mode, enum i2c_duty_cycle duty);

static void hal_i2c_setup_trise(i2c_t *instance, uint32_t frequency, enum i2c_mode mode);

static hal_status_t hal_i2c_start(i2c_t *instance, uint32_t timeout);

static hal_status_t hal_i2c_stop(i2c_t *instance);

static hal_status_t hal_i2c_transmit_slave_address(i2c_t *instance,
                                                   uint8_t slave_address,
                                                   uint32_t timeout);

static hal_status_t hal_i2c_transmit_mem_address(i2c_t *instance,
                                                 uint32_t mem_address,
                                                 size_t mem_address_size,
                                                 uint32_t timeout);

static hal_status_t hal_i2c_transmit_data(i2c_t *instance,
                                          const void *data,
                                          size_t data_size,
                                          uint32_t timeout);

static hal_status_t hal_i2c_receive_data(i2c_t *instance,
                                         void *data,
                                         size_t data_size,
                                         uint32_t timeout);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 */
void hal_i2c_init(struct i2c_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_i2c_sw_reset(handle->instance);
    hal_i2c_setup_frequency(handle->instance, handle->init.frequency );
    hal_i2c_setup_ccr(handle->instance, handle->init.frequency, handle->init.mode, handle->init.fm_duty);
    hal_i2c_setup_trise(handle->instance, handle->init.frequency, handle->init.mode );
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Программный сброс I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 */
static void hal_i2c_sw_reset(i2c_t *instance)
{
    /* Сброс I2C */
    SET_BIT(instance->CR1, I2C_CR1_SWRST_Msk);

    /* Ожидание готовности I2C */
    while (READ_BIT(instance->SR2, I2C_SR2_BUSY_Msk))
        continue;

    /* Исходное состояние I2C */
    CLEAR_BIT(instance->CR1, I2C_CR1_SWRST_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить частоту тактирования I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @param[in]       frequency: Частота тактирования (Гц)
 */
static void hal_i2c_setup_frequency(i2c_t *instance, uint32_t frequency)
{
    MODIFY_REG(instance->CR2,
               I2C_CR2_FREQ_Msk,
               (frequency / 1000000) << I2C_CR2_FREQ_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить CCR I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @param[in]       frequency: Частота тактирования (Гц)
 * @param[in]       mode: Режим работы I2C
 * @param[in]       duty: Рабочий цикл для режима I2C_FM
 */
static void hal_i2c_setup_ccr(i2c_t *instance, uint32_t frequency, enum i2c_mode mode, enum i2c_duty_cycle duty)
{
    /* Настроить режим работы */
    MODIFY_REG(instance->CCR,
               I2C_CCR_FS_Msk,
               mode << I2C_CCR_FS_Pos);

    /* Настроить Duty Cycle */
    MODIFY_REG(instance->CCR,
               I2C_CCR_DUTY_Msk,
               duty << I2C_CCR_DUTY_Pos);

    /* Настроить значение CCR для SM */
    if (mode == I2C_SM) {
        /*
         * Рассчитать значение CCR
         * T = Thigh + Tlow
         * Fm = 100000 Hz
         * Tm = 1 / 100000 s
         * Fpclk = 42000000 Hz
         * Tpclk = 1 / 42000000 s
         *
         * Thigh = CCR * Tpclk
         * Tlow = CCR * Tpclk
         * T = 2 * CCR * Tpclk
         * CCR = Tm / 2 * Tpclk
         * CCR = 42000000 / 200000 = 210
         */
        uint32_t CCR = frequency / 200000;

        MODIFY_REG(instance->CCR,
                   I2C_CCR_CCR_Msk,
                   CCR << I2C_CCR_CCR_Pos);
    } else if (mode == I2C_FM && duty == I2C_DUTY_2) {
        /*
         * Рассчитать значение CCR
         * T = Thigh + Tlow
         * Fm = 400000 Hz
         * Tm = 1 / 400000 s
         * Fpclk = 42000000 Hz
         * Tpclk = 1 / 42000000 s
         *
         * Thigh = CCR * Tpclk
         * Tlow = 2 * CCR * Tpclk
         * T = 3 * CCR * Tpclk
         * CCR = Tm / 3 * Tpclk
         * CCR = 42000000 / 1200000 = 35
         */
        uint32_t CCR = frequency / 1200000;

        MODIFY_REG(instance->CCR,
                   I2C_CCR_CCR_Msk,
                   CCR << I2C_CCR_CCR_Pos);
    } else if (mode == I2C_FM && duty == I2C_DUTY_16_9) {
        /*
         * Рассчитать значение CCR
         * T = Thigh + Tlow
         * Fm = 400000 Hz
         * Tm = 1 / 400000 s
         * Fpclk = 42000000 Hz
         * Tpclk = 1 / 42000000 s
         *
         * Thigh = 9 * CCR * Tpclk
         * Tlow = 16 * CCR * Tpclk
         * T = 25 * CCR * Tpclk
         * CCR = Tm / 25 * Tpclk
         * CCR = 42000000 / 10000000 = 4,2
         */
        uint32_t CCR = frequency / 10000000;

        MODIFY_REG(instance->CCR,
                   I2C_CCR_CCR_Msk,
                   CCR << I2C_CCR_CCR_Pos);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить TRISE I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @param[in]       frequency: Частота тактирования (Гц)
 * @param[in]       mode: Режим работы I2C
 */
static void hal_i2c_setup_trise(i2c_t *instance, uint32_t frequency, enum i2c_mode mode)
{
    /* Настроить значение TRISE для SM */
    if (mode == I2C_SM) {
        /*
         * Рассчитать значение TRISE:
         * Rise Timer = 1000 ns
         * Tpclk = 1000000000 / 42000000 = 23,8 ns
         * TRISE = 1000 / 23,8 + 1 = 43
         */
        uint32_t TRISE = (1000 / (1000000000 / frequency)) + 1;

        MODIFY_REG(instance->TRISE,
                   I2C_TRISE_TRISE_Msk,
                   TRISE << I2C_TRISE_TRISE_Pos);
    } else if (mode == I2C_FM) {
        /*
         * Рассчитать значение TRISE:
         * Rise Timer = 300 ns
         * Tpclk = 1000000000 / 42000000 = 23,8 ns
         * TRISE = 300 / 23,8 + 1 = 14
         */
        uint32_t TRISE = (300 / (1000000000 / frequency)) + 1;

        MODIFY_REG(instance->TRISE,
                   I2C_TRISE_TRISE_Msk,
                   TRISE << I2C_TRISE_TRISE_Pos);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 */
void hal_i2c_it_handler(struct i2c_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    uint32_t SR1 = READ_REG(handle->instance->SR1);

    /* Проверить ошибки I2C */
    if (READ_BIT(SR1,
                 I2C_SR1_BERR_Msk
               | I2C_SR1_ARLO_Msk
               | I2C_SR1_AF_Msk
               | I2C_SR1_OVR_Msk
               | I2C_SR1_PECERR_Msk)) {
        /* Событие STOP */
        SET_BIT(handle->instance->CR1, I2C_CR1_STOP_Msk);

        /* Сбросить статусы I2C */
        CLEAR_BIT(handle->instance->SR1,
                  I2C_SR1_BERR_Msk
                | I2C_SR1_ARLO_Msk
                | I2C_SR1_AF_Msk
                | I2C_SR1_OVR_Msk
                | I2C_SR1_PECERR_Msk);

        /* Выключить прерывания I2C */
        CLEAR_BIT(handle->instance->CR2,
                  I2C_CR2_ITEVTEN_Msk
                | I2C_CR2_ITBUFEN_Msk
                | I2C_CR2_ITERREN_Msk);

        /* Сбросить данные */
        handle->instance_event = 0;
        handle->command = I2C_NO_COMMAND;
        handle->slave_address = 0;
        handle->mem_address = 0;
        handle->mem_address_size = 0;
        handle->data = NULL;
        handle->data_size = 0;

        if (handle->error_callback != NULL)
            handle->error_callback(handle);
    }
    /* Событие START */
    else if (READ_BIT(SR1, I2C_SR1_SB_Msk)) {
        READ_REG(handle->instance->SR1);

        /* Передать адрес подчиненного устройства */
        if (handle->instance_event == 0) {
            WRITE_REG(handle->instance->DR, handle->slave_address | I2C_WRITE);
        } else if (handle->instance_event == 1) {
            WRITE_REG(handle->instance->DR, handle->slave_address | I2C_READ);
        }
    }
    /* Событие передачи адреса подчиненного устройства */
    else if (READ_BIT(SR1, I2C_SR1_ADDR_Msk)) {
        READ_REG(handle->instance->SR1);
        READ_REG(handle->instance->SR2);
    }
    /* Событие приема данных */
    else if (READ_BIT(SR1, I2C_SR1_RXNE_Msk)) {
        if (handle->instance_event == 1) {
            /* Если команда I2C_MEM_READ и не все данные приняты - принять данные */
            if (handle->command == I2C_MEM_READ && handle->data_size > 0) {
                volatile uint8_t *pdata = (uint8_t *) handle->data;

                *pdata = READ_REG(handle->instance->DR);

                handle->data++;
                handle->data_size--;

                if (handle->data_size == 1) {
                    /* Выключить ACK */
                    CLEAR_BIT(handle->instance->CR1, I2C_CR1_ACK_Msk);
                    /* Событие STOP */
                    SET_BIT(handle->instance->CR1, I2C_CR1_STOP_Msk);
                } else if (handle->data_size == 0) {
                    /* Изменить событие I2C */
                    handle->instance_event = 2;
                }
            }
        }
    }
    /* Событие передачи данных */
    else if (READ_BIT(SR1, I2C_SR1_TXE_Msk)) {
        if (handle->instance_event == 0) {
            /* Передать адрес данных */
            if (handle->mem_address_size > 0) {
                volatile uint8_t mem_address_data = (handle->mem_address >> (8 * handle->mem_address_size - 8));

                WRITE_REG(handle->instance->DR, mem_address_data);

                handle->mem_address_size--;
            } else if (handle->command == I2C_MEM_READ) {
                /*
                 * Если команда I2C_MEM_READ
                 * необходимо после передачи адреса памяти
                 * произвести повторное событие START
                 * и ожидать приема данных
                 */

                /* Событие START */
                SET_BIT(handle->instance->CR1, I2C_CR1_START_Msk);

                /* Изменить событие I2C */
                handle->instance_event = 1;
            } else if (handle->command == I2C_MEM_WRITE && handle->data_size > 0) {
                /* Если команда I2C_MEM_WRITE и не все данные переданы - передать данные */
                volatile const uint8_t *pdata  = (uint8_t *) handle->data;

                WRITE_REG(handle->instance->DR, *pdata);

                handle->data++;
                handle->data_size--;
            } else {
                /* Событие STOP */
                SET_BIT(handle->instance->CR1, I2C_CR1_STOP_Msk);

                /* Изменить событие I2C */
                handle->instance_event = 2;
            }
        }
    }

    /* Завершение выполнение команды */
    if (handle->instance_event == 2) {
        /* Выключить прерывания I2C */
        CLEAR_BIT(handle->instance->CR2,
                  I2C_CR2_ITEVTEN_Msk
                | I2C_CR2_ITBUFEN_Msk
                | I2C_CR2_ITERREN_Msk);

        /* Сбросить данные */
        handle->instance_event = 0;
        handle->command = I2C_NO_COMMAND;
        handle->slave_address = 0;
        handle->mem_address = 0;
        handle->mem_address_size = 0;
        handle->data = NULL;
        handle->data_size = 0;

        if (handle->command_completed_callback != NULL)
            handle->command_completed_callback(handle);
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 */
void hal_i2c_enable(struct i2c_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    SET_BIT(handle->instance->CR1, I2C_CR1_PE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 */
void hal_i2c_disable(struct i2c_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    CLEAR_BIT(handle->instance->CR1, I2C_CR1_PE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Ожидание готовности I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_i2c_wait_ready(struct i2c_handle *handle, uint32_t timeout)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Ожидание готовности */
    uint32_t tickstart = hal_systick_tick(&systick);

    while (true) {
        uint32_t SR2 = READ_REG(handle->instance->SR2);

        if (!READ_BIT(SR2, I2C_SR2_BUSY_Msk)) {
            return HAL_OK;
        } else if (hal_systick_tick(&systick) - tickstart >= timeout) {
            return HAL_ERROR;
        }
    }

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Событие START I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_i2c_start(i2c_t *instance, uint32_t timeout)
{
    /* Сбросить статусы I2C */
    CLEAR_BIT(instance->SR1,
              I2C_SR1_BERR_Msk
            | I2C_SR1_ARLO_Msk
            | I2C_SR1_AF_Msk
            | I2C_SR1_OVR_Msk
            | I2C_SR1_PECERR_Msk);

    /* Настроить и включить ACK I2C */
    MODIFY_REG(instance->CR1,
               I2C_CR1_POS_Msk,
               I2C_CR1_ACK_Msk);

    /* Событие START */
    SET_BIT(instance->CR1, I2C_CR1_START_Msk);

    /* Ожидание события START */
    uint32_t tickstart = hal_systick_tick(&systick);

    while (true) {
        uint32_t SR1 = READ_REG(instance->SR1);

        if (READ_BIT(SR1, I2C_SR1_SB_Msk)) {
            READ_REG(instance->SR1);
            return HAL_OK;
        } else if (READ_BIT(SR1, I2C_SR1_AF_Msk)) {
            SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);
            return HAL_ERROR;
        } else if (hal_systick_tick(&systick) - tickstart >= timeout) {
            CLEAR_BIT(instance->CR1, I2C_CR1_START_Msk);
            return HAL_ERROR;
        }
    }

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Событие STOP I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_i2c_stop(i2c_t *instance)
{
    /* Событие STOP */
    SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передать адрес подчиненного устройства I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @param[in]       slave_address: Адрес подчиненного устройства
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_i2c_transmit_slave_address(i2c_t *instance,
                                                   uint8_t slave_address,
                                                   uint32_t timeout)
{
    /* Передать адрес подчиненного устройства */
    WRITE_REG(instance->DR, slave_address);

    /* Ожидание передачи адреса подчиненного устройства */
    uint32_t tickstart = hal_systick_tick(&systick);

    while (true) {
        uint32_t SR1 = READ_REG(instance->SR1);

        if (READ_BIT(SR1, I2C_SR1_ADDR_Msk)) {
            READ_REG(instance->SR1);
            READ_REG(instance->SR2);
            return HAL_OK;
        } else if (READ_BIT(SR1, I2C_SR1_AF_Msk)) {
            SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);
            return HAL_ERROR;
        } else if (hal_systick_tick(&systick) - tickstart >= timeout) {
            SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);
            return HAL_ERROR;
        }
    }

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передать адрес памяти I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @param[in]       mem_address: Адрес памяти
 * @param[in]       mem_address_size: Размер адреса памяти
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_i2c_transmit_mem_address(i2c_t *instance,
                                                 uint32_t mem_address,
                                                 size_t mem_address_size,
                                                 uint32_t timeout)
{
    while (mem_address_size > 0) {
        /* Выделить данные адреса памяти */
        uint8_t mem_address_data = (mem_address >> (8 * mem_address_size - 8));

        /* Передать данные */
        WRITE_REG(instance->DR, mem_address_data);

        /* Ожидание передачи данных */
        uint32_t tickstart = hal_systick_tick(&systick);

        while (true) {
            uint32_t SR1 = READ_REG(instance->SR1);

            if (READ_BIT(SR1, I2C_SR1_TXE_Msk)) {
                break;
            } else if (READ_BIT(SR1, I2C_SR1_AF_Msk)) {
                SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);
                return HAL_ERROR;
            } else if (hal_systick_tick(&systick) - tickstart >= timeout) {
                return HAL_ERROR;
            }
        }

        /* Изменить количество данных адреса */
        mem_address_size--;
    }

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Передать данные I2C
 *
 * @param[in]       instance: Указатель на структуру данных I2C
 * @param[in]       data: Указатель на данные
 * @param[in]       data_size: Размер данных
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_i2c_transmit_data(i2c_t *instance,
                                          const void *data,
                                          size_t data_size,
                                          uint32_t timeout)
{
    /* Проверить наличие данных */
    if (data == NULL || data_size == 0)
        return HAL_ERROR;

    /* Указатель на данные */
    const uint8_t *pdata = (uint8_t *) data;

    while (data_size > 0) {
        /* Передать данные */
        WRITE_REG(instance->DR, *pdata);

        /* Ожидание передачи данных */
        uint32_t tickstart = hal_systick_tick(&systick);

        while (true) {
            uint32_t SR1 = READ_REG(instance->SR1);

            if (READ_BIT(SR1, I2C_SR1_TXE_Msk)) {
                break;
            } else if (READ_BIT(SR1, I2C_SR1_AF_Msk)) {
                SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);
                return HAL_ERROR;
            } else if (hal_systick_tick(&systick) - tickstart >= timeout) {
                return HAL_ERROR;
            }
        }

        pdata++;
        data_size--;
    }

    /* Событие STOP */
    SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Принять данные I2C
 *
 * @param[in]       instance Указатель на структуру данных I2C
 * @param[in]       data: Указатель на данные
 * @param[in]       data_size: Размер данных
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_i2c_receive_data(i2c_t *instance,
                                         void *data,
                                         size_t data_size,
                                         uint32_t timeout)
{
    /* Проверить наличие данных */
    if (data == NULL || data_size == 0)
        return HAL_ERROR;

    /* Указатель на данные */
    uint8_t *pdata = (uint8_t *) data;

    while (data_size > 0) {
        /* Проверить завершение приема данных */
        if (data_size == 1) {
            /* Выключить ACK */
            CLEAR_BIT(instance->CR1, I2C_CR1_ACK_Msk);
            /* Событие STOP */
            SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);
        }

        /* Ожидание приема данных */
        uint32_t tickstart = hal_systick_tick(&systick);

        while (true) {
            uint32_t SR1 = READ_REG(instance->SR1);

            if (READ_BIT(SR1, I2C_SR1_RXNE_Msk)) {
                break;
            } else if (READ_BIT(SR1, I2C_SR1_AF_Msk)) {
                SET_BIT(instance->CR1, I2C_CR1_STOP_Msk);
                return HAL_ERROR;
            } else if (hal_systick_tick(&systick) - tickstart >= timeout) {
                return HAL_ERROR;
            }
        }

        /* Принять данные */
        *pdata = READ_REG(instance->DR);

        pdata++;
        data_size--;
    }

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Проверить готовность устройства I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       slave_address: Адрес подчиненного устройства
 * @param[in]       retry: Количество повторов
 * @param[in]       timeout: Таймаут
 * @return          Состояние
 */
bool hal_i2c_device_is_ready(struct i2c_handle *handle,
                             uint8_t slave_address,
                             uint8_t retry,
                             uint32_t timeout)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    for (uint8_t i = 0; i < retry; i++) {
        /* Выполнить последовательность действий поиска устройства */
        do {
            /* Ожидание готовности */
            if (hal_i2c_wait_ready(handle, timeout) != HAL_OK)
                break;

            /* Событие START */
            if (hal_i2c_start(handle->instance, timeout) != HAL_OK)
                break;

            /* Передать адрес подчиненного устройства */
            if (hal_i2c_transmit_slave_address(handle->instance,
                                               slave_address | I2C_WRITE,
                                               timeout) != HAL_OK)
                break;

            /* Событие STOP */
            if (hal_i2c_stop(handle->instance) != HAL_OK)
                break;

            /* Ожидание готовности */
            if (hal_i2c_wait_ready(handle, timeout) != HAL_OK)
                break;

            return true;
        } while (false);
    }

    return false;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать данные из памяти I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       slave_address: Адрес подчиненного устройства
 * @param[in]       mem_address: Адрес памяти
 * @param[in]       mem_address_size: Размер адреса памяти
 * @param[out]      data: Указатель на данные
 * @param[in]       data_size: Размер данных
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_i2c_mem_read(struct i2c_handle *handle,
                              uint8_t slave_address,
                              uint32_t mem_address,
                              size_t mem_address_size,
                              void *data,
                              size_t data_size,
                              uint32_t timeout)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить наличие данных */
    if (data == NULL || data_size == 0)
        return HAL_ERROR;

    /* Выполнить последовательность действий чтения данных */
    do {
        /* Ожидание готовности */
        if (hal_i2c_wait_ready(handle, timeout) != HAL_OK)
            break;

        /* Событие START */
        if (hal_i2c_start(handle->instance, timeout) != HAL_OK)
            break;

        /* Передать адрес подчиненного устройства */
        if (hal_i2c_transmit_slave_address(handle->instance,
                                           slave_address | I2C_WRITE,
                                           timeout) != HAL_OK)
            break;

        /* Передать адрес памяти */
        if (hal_i2c_transmit_mem_address(handle->instance,
                                         mem_address,
                                         mem_address_size,
                                         timeout) != HAL_OK)
            break;

        /* Повторное событие START */
        if (hal_i2c_start(handle->instance, timeout) != HAL_OK)
            break;

        /* Передать адрес подчиненного устройства */
        if (hal_i2c_transmit_slave_address(handle->instance,
                                           slave_address | I2C_READ,
                                           timeout) != HAL_OK)
            break;

        /* Принять данные */
        if (hal_i2c_receive_data(handle->instance,
                                 data,
                                 data_size,
                                 timeout) != HAL_OK)
            break;

        /* Ожидание готовности */
        if (hal_i2c_wait_ready(handle, timeout) != HAL_OK)
            break;

        return HAL_OK;
    } while (false);

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Записать данные в память I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       slave_address: Адрес подчиненного устройства
 * @param[in]       mem_address: Адрес памяти
 * @param[in]       mem_address_size: Размер адреса памяти
 * @param[in]       data: Указатель на данные
 * @param[in]       data_size: Размер данных
 * @param[in]       timeout: Таймаут
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_i2c_mem_write(struct i2c_handle *handle,
                               uint8_t slave_address,
                               uint32_t mem_address,
                               size_t mem_address_size,
                               const void *data,
                               size_t data_size,
                               uint32_t timeout)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить наличие данных */
    if (data == NULL || data_size == 0)
        return HAL_ERROR;

    /* Выполнить последовательность действий записи данных */
    do {
        /* Ожидание готовности */
        if (hal_i2c_wait_ready(handle, timeout) != HAL_OK)
            break;

        /* Событие START */
        if (hal_i2c_start(handle->instance, timeout) != HAL_OK)
            break;

        /* Передать адрес подчиненного устройства */
        if (hal_i2c_transmit_slave_address(handle->instance,
                                           slave_address | I2C_WRITE,
                                           timeout) != HAL_OK)
            break;

        /* Передать адрес памяти */
        if (hal_i2c_transmit_mem_address(handle->instance,
                                         mem_address,
                                         mem_address_size,
                                         timeout) != HAL_OK)
            break;

        /* Передать данные */
        if (hal_i2c_transmit_data(handle->instance,
                                  data,
                                  data_size,
                                  timeout) != HAL_OK)
            break;

        /* Ожидание готовности */
        if (hal_i2c_wait_ready(handle, timeout) != HAL_OK)
            break;

        return HAL_OK;
    } while (false);

    return HAL_ERROR;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Прочитать данные из памяти I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       slave_address: Адрес подчиненного устройства
 * @param[in]       mem_address: Адрес памяти
 * @param[in]       mem_address_size: Размер адреса памяти
 * @param[out]      data: Указатель на данные
 * @param[in]       data_size: Размер данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_i2c_mem_read_it(struct i2c_handle *handle,
                                 uint8_t slave_address,
                                 uint32_t mem_address,
                                 size_t mem_address_size,
                                 void *data,
                                 size_t data_size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить наличие данных */
    if (data == NULL || data_size == 0)
        return HAL_ERROR;

    /* Проверить готовность I2C */
    if (READ_BIT(handle->instance->SR2, I2C_SR2_BUSY_Msk))
        return HAL_ERROR;

    /* Заполнить параметры чтения данных */
    handle->instance_event = 0;
    handle->command = I2C_MEM_READ;
    handle->slave_address = slave_address;
    handle->mem_address = mem_address;
    handle->mem_address_size = mem_address_size;
    handle->data = (void *) data;
    handle->data_size = data_size;

    /* Сбросить статусы I2C */
    CLEAR_BIT(handle->instance->SR1,
              I2C_SR1_BERR_Msk
            | I2C_SR1_ARLO_Msk
            | I2C_SR1_AF_Msk
            | I2C_SR1_OVR_Msk
            | I2C_SR1_PECERR_Msk);

    /* Включить и настроить ACK I2C */
    MODIFY_REG(handle->instance->CR1,
               I2C_CR1_POS_Msk,
               I2C_CR1_ACK_Msk);

    /* Включить прерывания I2C */
    SET_BIT(handle->instance->CR2,
            I2C_CR2_ITEVTEN_Msk
          | I2C_CR2_ITBUFEN_Msk
          | I2C_CR2_ITERREN_Msk);

    /* Событие START */
    SET_BIT(handle->instance->CR1, I2C_CR1_START_Msk);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Записать данные в память I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       slave_address: Адрес подчиненного устройства
 * @param[in]       mem_address: Адрес памяти
 * @param[in]       mem_address_size: Размер адреса памяти
 * @param[in]       data: Указатель на данные
 * @param[in]       data_size: Размер данных
 * @return          Статус @ref hal_status_t
 */
hal_status_t hal_i2c_mem_write_it(struct i2c_handle *handle,
                                  uint8_t slave_address,
                                  uint32_t mem_address,
                                  size_t mem_address_size,
                                  const void *data,
                                  size_t data_size)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить наличие данных */
    if (data == NULL || data_size == 0)
        return HAL_ERROR;

    /* Проверить готовность I2C */
    if (READ_BIT(handle->instance->SR2, I2C_SR2_BUSY_Msk))
        return HAL_ERROR;

    /* Заполнить параметры записи данных */
    handle->instance_event = 0;
    handle->command = I2C_MEM_WRITE;
    handle->slave_address = slave_address;
    handle->mem_address = mem_address;
    handle->mem_address_size = mem_address_size;
    handle->data = (void *) data;
    handle->data_size = data_size;

    /* Сбросить статусы I2C */
    CLEAR_BIT(handle->instance->SR1,
              I2C_SR1_BERR_Msk
            | I2C_SR1_ARLO_Msk
            | I2C_SR1_AF_Msk
            | I2C_SR1_OVR_Msk
            | I2C_SR1_PECERR_Msk);

    /* Включить и настроить ACK I2C */
    MODIFY_REG(handle->instance->CR1,
               I2C_CR1_POS_Msk,
               I2C_CR1_ACK_Msk);

    /* Включить прерывания I2C */
    SET_BIT(handle->instance->CR2,
            I2C_CR2_ITEVTEN_Msk
          | I2C_CR2_ITBUFEN_Msk
          | I2C_CR2_ITERREN_Msk);

    /* Событие START */
    SET_BIT(handle->instance->CR1, I2C_CR1_START_Msk);

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       callback_id: Идентификатор функции обратного вызова I2C
 * @param[in]       callback: Указатель на функцию обратного вызова I2C
 */
void hal_i2c_register_callback(struct i2c_handle *handle,
                               enum i2c_callback_id callback_id,
                               void (*callback)(struct i2c_handle *handle))
{
    assert(handle != NULL);

    switch (callback_id) {
        case I2C_COMMAND_COMPLETED_CALLBACK:
            handle->command_completed_callback = callback;
            break;
        case I2C_ERROR_CALLBACK:
            handle->error_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова I2C
 *
 * @param[in]       handle: Указатель на структуру данных обработчика I2C
 * @param[in]       callback_id: Идентификатор функции обратного вызова I2C
 */
void hal_i2c_unregister_callback(struct i2c_handle *handle,
                                 enum i2c_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case I2C_COMMAND_COMPLETED_CALLBACK:
            handle->command_completed_callback = NULL;
            break;
        case I2C_ERROR_CALLBACK:
            handle->error_callback = NULL;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
