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

#include "stm32f446xx_it.h"
#include "systick.h"
#include "pwr.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "sensors.h"
#include "dio.h"
#include "eeprom.h"
#include "w25q.h"
#include "modbus.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик SysTick */
extern struct systick_handle systick;

/* Обработчик PWR */
extern struct pwr_handle pwr;

/* Состояние VDD */
extern bool vdd_is_lower;

/* Обработчик ADC1 */
extern struct adc_handle adc1;

/* Обработчики DMA */
extern struct dma_handle dma1_stream0;
extern struct dma_handle dma1_stream3;
extern struct dma_handle dma1_stream4;
extern struct dma_handle dma1_stream7;
extern struct dma_handle dma2_stream2;
extern struct dma_handle dma2_stream3;

/* Обработчики SPI */
extern struct spi_handle spi1;
extern struct spi_handle spi2;
extern struct spi_handle spi3;

/* Обработчик I2C1 */
extern struct i2c_handle i2c1;

/* Обработчики USART */
extern struct usart_handle usart1;
extern struct usart_handle usart2;
extern struct usart_handle usart6;

/* Обработчик TIM7 */
extern struct tim_handle tim7;

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

void NMI_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void HardFault_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void MemManage_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void BusFault_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void UsageFault_Handler(void)
{
    hal_error();
}
/* ------------------------------------------------------------------------- */

void SysTick_Handler(void)
{
    hal_systick_it_handler(&systick);
}
/* ------------------------------------------------------------------------- */

void SysTick_PeriodElapsedCallback(void)
{
    /* Обработать системный таймер FreeRTOS */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        xPortSysTickHandler();
}
/* ------------------------------------------------------------------------- */

void PVD_IRQHandler(void)
{
    hal_pwr_pvd_it_handler(&pwr);
}
/* ------------------------------------------------------------------------- */

void PWR_PVD_StatusChangedCallback(void)
{
    if (hal_pwr_pvd_status(&pwr) == PWR_VDD_LOWER_PVD) {
        vdd_is_lower = true;
    } else {
        vdd_is_lower = false;
    }
}
/* ------------------------------------------------------------------------- */

void ADC_IRQHandler(void)
{
    hal_adc_it_handler(&adc1);
}
/* ------------------------------------------------------------------------- */

void ADC_MeasureCompletedCallback(struct adc_handle *handle)
{
    if (handle == &adc1)
        sensors_adc_measure_completed_it_handler(&sensors);
}
/* ------------------------------------------------------------------------- */

void ADC_ErrorCallback(struct adc_handle *handle)
{
    if (handle == &adc1)
        sensors_adc_error_it_handler(&sensors);
}
/* ------------------------------------------------------------------------- */

void DMA1_Stream0_IRQHandler(void)
{
    hal_dma_it_handler(&dma1_stream0);
}
/* ------------------------------------------------------------------------- */

void DMA1_Stream3_IRQHandler(void)
{
    hal_dma_it_handler(&dma1_stream3);
}
/* ------------------------------------------------------------------------- */

void DMA1_Stream4_IRQHandler(void)
{
    hal_dma_it_handler(&dma1_stream4);
}
/* ------------------------------------------------------------------------- */

void DMA1_Stream7_IRQHandler(void)
{
    hal_dma_it_handler(&dma1_stream7);
}
/* ------------------------------------------------------------------------- */

void DMA2_Stream2_IRQHandler(void)
{
    hal_dma_it_handler(&dma2_stream2);
}
/* ------------------------------------------------------------------------- */

void DMA2_Stream3_IRQHandler(void)
{
    hal_dma_it_handler(&dma2_stream3);
}
/* ------------------------------------------------------------------------- */

void DMA_TransferCompletedCallback(struct dma_handle *handle)
{
    if (handle == &dma2_stream2) {
        hal_spi_transmit_receive_completed_dma_it_handler(&spi1);
    } else if (handle == &dma1_stream3) {
        hal_spi_transmit_receive_completed_dma_it_handler(&spi2);
    }
}
/* ------------------------------------------------------------------------- */

void DMA_TransferErrorCallback(struct dma_handle *handle)
{
    if (handle == &dma2_stream2 || handle == &dma2_stream3) {
        hal_spi_error_dma_it_handler(&spi1);
    } else if (handle == &dma1_stream3 || handle == &dma1_stream4) {
        hal_spi_error_dma_it_handler(&spi2);
    }
}
/* ------------------------------------------------------------------------- */

void SPI_TransmitReceiveCompletedCallback(struct spi_handle *handle)
{
    if (handle == &spi1) {
        w25q_spi_transmit_receive_completed_it_handler(&w25q);
    } else if (handle == &spi2) {
        dio_spi_transmit_receive_completed_it_handler(&dio);
    }
}
/* ------------------------------------------------------------------------- */

void SPI_ErrorCallback(struct spi_handle *handle)
{
    if (handle == &spi1) {
        w25q_spi_error_it_handler(&w25q);
    } else if (handle == &spi2) {
        dio_spi_error_it_handler(&dio);
    }
}
/* ------------------------------------------------------------------------- */

void I2C1_EV_IRQHandler(void)
{
    hal_i2c_it_handler(&i2c1);
}
/* ------------------------------------------------------------------------- */

void I2C1_ER_IRQHandler(void)
{
    hal_i2c_it_handler(&i2c1);
}
/* ------------------------------------------------------------------------- */

void I2C_CommandCompletedCallback(struct i2c_handle *handle)
{
    if (handle == &i2c1)
        eeprom_i2c_command_completed_it_handler(&eeprom);
}
/* ------------------------------------------------------------------------- */

void I2C_ErrorCallback(struct i2c_handle *handle)
{
    if (handle == &i2c1)
        eeprom_i2c_error_it_handler(&eeprom);
}
/* ------------------------------------------------------------------------- */

void USART1_IRQHandler(void)
{
    hal_usart_it_handler(&usart1);
}
/* ------------------------------------------------------------------------- */

void USART2_IRQHandler(void)
{
    hal_usart_it_handler(&usart2);
}
/* ------------------------------------------------------------------------- */

void USART6_IRQHandler(void)
{
    hal_usart_it_handler(&usart6);
}
/* ------------------------------------------------------------------------- */

void USART_TransmitCompletedCallback(struct usart_handle *handle)
{
    if (handle == &usart1) {
        modbus_usart_transmit_completed_it_handler(&modbus[MODBUS0]);
    } else if (handle == &usart2) {
        modbus_usart_transmit_completed_it_handler(&modbus[MODBUS1]);
    } else if (handle == &usart6) {

    }
}
/* ------------------------------------------------------------------------- */

void USART_ReceiveCompletedCallback(struct usart_handle *handle)
{
    if (handle == &usart1) {
        modbus_usart_receive_completed_it_handler(&modbus[MODBUS0]);
    } else if (handle == &usart2) {
        modbus_usart_receive_completed_it_handler(&modbus[MODBUS1]);
    } else if (handle == &usart6) {

    }
}
/* ------------------------------------------------------------------------- */

void USART_ErrorCallback(struct usart_handle *handle)
{
    if (handle == &usart1) {
        modbus_usart_error_it_handler(&modbus[MODBUS0]);
    } else if (handle == &usart2) {
        modbus_usart_error_it_handler(&modbus[MODBUS1]);
    } else if (handle == &usart6) {

    }
}
/* ------------------------------------------------------------------------- */

void TIM7_IRQHandler(void)
{
    hal_tim_it_handler(&tim7);
}
/* ------------------------------------------------------------------------- */

void TIM_PeriodElapsedCallback(struct tim_handle *handle)
{
    if (handle == &tim7) {
        modbus_tim_it_handler(&modbus[MODBUS0]);
        modbus_tim_it_handler(&modbus[MODBUS1]);
    }
}
/* ------------------------------------------------------------------------- */
