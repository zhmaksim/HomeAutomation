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

#ifndef I2C_H_
#define I2C_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "stm32f4xx_hal_i2c.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define I2C_EV_CMD_CPLT_Pos         0
#define I2C_EV_CMD_CPLT_Msk         HAL_BITMASK(0x01, I2C_EV_CMD_CPLT_Pos)
#define I2C_EV_CMD_CPLT             I2C_EV_CMD_CPLT_Msk

#define I2C_EV_ERR_Pos              7
#define I2C_EV_ERR_Msk              HAL_BITMASK(0x01, I2C_EV_ERR_Pos)
#define I2C_EV_ERR                  I2C_EV_ERR_Msk

/* Exported types ---------------------------------------------------------- */

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void i2c_init(void);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* I2C_H_ */
