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

#include "i2c.h"
#include "rcc.h"
#include "stm32f446xx_it.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик RCC */
extern struct rcc_handle rcc;

/* Обработчик I2C1 */
struct i2c_handle i2c1 = {
    .instance = I2C1,
};

/* Mutex и Event Group I2C1 */
SemaphoreHandle_t i2c1_mutex;
EventGroupHandle_t i2c1_event_group;

/* Private function prototypes --------------------------------------------- */

static void i2c1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать I2C
 */
void i2c_init(void)
{
    HAL_I2C1_ENABLE_CLOCK();

    i2c1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать I2C1
 */
static void i2c1_init(void)
{
    i2c1.init.frequency = rcc.apb1_clock;
    i2c1.init.mode = I2C_FM;
    i2c1.init.fm_duty = I2C_DUTY_2;

    hal_i2c_register_callback(&i2c1,
                              I2C_COMMAND_COMPLETED_CALLBACK,
                              I2C_CommandCompletedCallback);
    hal_i2c_register_callback(&i2c1,
                              I2C_ERROR_CALLBACK,
                              I2C_ErrorCallback);
    hal_i2c_init(&i2c1);
    hal_i2c_enable(&i2c1);

    NVIC_SetPriority(I2C1_EV_IRQn, 10);
    NVIC_SetPriority(I2C1_ER_IRQn, 10);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    i2c1_mutex = xSemaphoreCreateMutex();
    if (i2c1_mutex == NULL)
        hal_error();

    i2c1_event_group = xEventGroupCreate();
    if (i2c1_event_group == NULL)
        hal_error();
}
/* ------------------------------------------------------------------------- */
