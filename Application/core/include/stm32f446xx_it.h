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

#ifndef STM32F446XX_IT_H_
#define STM32F446XX_IT_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "stm32f4xx_hal_systick.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_adc.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void NMI_Handler(void);

void HardFault_Handler(void);

void MemManage_Handler(void);

void BusFault_Handler(void);

void UsageFault_Handler(void);

void SysTick_Handler(void);

void SysTick_PeriodElapsedCallback(void);

void PVD_IRQHandler(void);

void PWR_PVD_StatusChangedCallback(void);

void ADC_IRQHandler(void);

void ADC_MeasureCompletedCallback(struct adc_handle *handle);

void ADC_ErrorCallback(struct adc_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F446XX_IT_H_ */
