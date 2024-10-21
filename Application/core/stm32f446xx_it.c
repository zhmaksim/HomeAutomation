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
#include "sensors.h"

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
