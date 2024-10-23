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

#ifndef USART_H_
#define USART_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "stm32f4xx_hal_usart.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define USART_EV_TRANSMIT_CPLT_Pos          0
#define USART_EV_TRANSMIT_CPLT_Msk          HAL_BITMASK(0x01, USART_EV_TRANSMIT_CPLT_Pos)
#define USART_EV_TRANSMIT_CPLT              USART_EV_TRANSMIT_CPLT_Msk

#define USART_EV_RECEIVE_CPLT_Pos           1
#define USART_EV_RECEIVE_CPLT_Msk           HAL_BITMASK(0x01, USART_EV_RECEIVE_CPLT_Pos)
#define USART_EV_RECEIVE_CPLT               USART_EV_RECEIVE_CPLT_Msk

#define USART_EV_ERR_Pos                    7
#define USART_EV_ERR_Msk                    HAL_BITMASK(0x01, USART_EV_ERR_Pos)
#define USART_EV_ERR                        USART_EV_ERR_Msk

/* Exported types ---------------------------------------------------------- */

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void usart_init(void);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* USART_H_ */
