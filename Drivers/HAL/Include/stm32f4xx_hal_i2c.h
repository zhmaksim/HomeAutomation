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

#ifndef STM32F4XX_HAL_I2C_H_
#define STM32F4XX_HAL_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование I2C1
 */
#define HAL_I2C1_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных I2C
 */
typedef I2C_TypeDef i2c_t;


/**
 * @brief           Определение перечисления режимов работы I2C
 */
enum i2c_mode {
    I2C_SM,                                     /*!< Standart Mode = 100kHz */
    I2C_FM,                                     /*!< Fast Mode = 400kHz */
};


/**
 * @brief           Определение перечисления рабочих циклов I2C
 */
enum i2c_duty_cycle {
    I2C_DUTY_2,
    I2C_DUTY_16_9,
};


/**
 * @brief           Определение перечисления команд I2C
 */
enum i2c_command {
    I2C_NO_COMMAND,
    I2C_MEM_READ,
    I2C_MEM_WRITE,
};


/**
 * @brief           Определение перечисления идентификаторов функций обратного вызова I2C
 */
enum i2c_callback_id {
    I2C_COMMAND_COMPLETED_CALLBACK,
    I2C_ERROR_CALLBACK,
};


/**
 * @brief           Определение структуры данных настройки I2C
 */
struct i2c_init {
    uint32_t                frequency;          /*!< Частота тактирования (Гц) */

    enum i2c_mode           mode;               /*!< Режим работы:
                                                    - I2C_SM
                                                    - I2C_FM */

    enum i2c_duty_cycle     fm_duty;            /*!< Рабочий цикл для режима FM:
                                                    - I2C_DUTY_2
                                                    - I2C_DUTY_16_9 */
};


/**
 * @brief           Определение структуры данных обработчика I2C
 */
struct i2c_handle {
    i2c_t                  *instance;           /*!< Указатель на структуру данных I2C */

    volatile uint8_t        instance_event;     /*!< Событие приема-передачи I2C */

    struct i2c_init         init;               /*!< Настройки I2C */

    volatile uint8_t        command;            /*!< Команда:
                                                    - I2C_MEM_READ
                                                    - I2C_MEM_WRITE */

    volatile uint8_t        slave_address;      /*!< Адрес подчиненного устройства */

    volatile uint32_t       mem_address;        /*!< Адрес памяти */

    volatile size_t         mem_address_size;   /*!< Размер адреса памяти (0..4 байта) */

    volatile void          *data;               /*!< Указатель на данные */

    volatile size_t         data_size;          /*!< Размер данных */

    /* --- */

    void (*command_completed_callback)(struct i2c_handle *handle);

    void (*error_callback)(struct i2c_handle *handle);
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_i2c_init(struct i2c_handle *handle);

void hal_i2c_it_handler(struct i2c_handle *handle);

void hal_i2c_enable(struct i2c_handle *handle);

void hal_i2c_disable(struct i2c_handle *handle);

hal_status_t hal_i2c_wait_ready(struct i2c_handle *handle, uint32_t timeout);

bool hal_i2c_device_is_ready(struct i2c_handle *handle,
                             uint8_t slave_address,
                             uint8_t retry,
                             uint32_t timeout);

hal_status_t hal_i2c_mem_read(struct i2c_handle *handle,
                              uint8_t slave_address,
                              uint32_t mem_address,
                              size_t mem_address_size,
                              void *data,
                              size_t data_size,
                              uint32_t timeout);

hal_status_t hal_i2c_mem_write(struct i2c_handle *handle,
                               uint8_t slave_address,
                               uint32_t mem_address,
                               size_t mem_address_size,
                               const void *data,
                               size_t data_size,
                               uint32_t timeout);

hal_status_t hal_i2c_mem_read_it(struct i2c_handle *handle,
                                 uint8_t slave_address,
                                 uint32_t mem_address,
                                 size_t mem_address_size,
                                 void *data,
                                 size_t data_size);

hal_status_t hal_i2c_mem_write_it(struct i2c_handle *handle,
                                  uint8_t slave_address,
                                  uint32_t mem_address,
                                  size_t mem_address_size,
                                  const void *data,
                                  size_t data_size);

void hal_i2c_register_callback(struct i2c_handle *handle,
                               enum i2c_callback_id callback_id,
                               void (*callback)(struct i2c_handle *handle));

void hal_i2c_unregister_callback(struct i2c_handle *handle,
                                 enum i2c_callback_id callback_id);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_I2C_H_ */
