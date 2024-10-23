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
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_tim.h"

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

void DMA1_Stream0_IRQHandler(void);

void DMA1_Stream3_IRQHandler(void);

void DMA1_Stream4_IRQHandler(void);

void DMA1_Stream7_IRQHandler(void);

void DMA2_Stream2_IRQHandler(void);

void DMA2_Stream3_IRQHandler(void);

void DMA_TransferCompletedCallback(struct dma_handle *handle);

void DMA_TransferErrorCallback(struct dma_handle *handle);

void SPI_TransmitReceiveCompletedCallback(struct spi_handle *handle);

void SPI_ErrorCallback(struct spi_handle *handle);

void I2C1_EV_IRQHandler(void);

void I2C1_ER_IRQHandler(void);

void I2C_CommandCompletedCallback(struct i2c_handle *handle);

void I2C_ErrorCallback(struct i2c_handle *handle);

void USART1_IRQHandler(void);

void USART2_IRQHandler(void);

void USART6_IRQHandler(void);

void USART_TransmitCompletedCallback(struct usart_handle *handle);

void USART_ReceiveCompletedCallback(struct usart_handle *handle);

void USART_ErrorCallback(struct usart_handle *handle);

void TIM7_IRQHandler(void);

void TIM_PeriodElapsedCallback(struct tim_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F446XX_IT_H_ */
