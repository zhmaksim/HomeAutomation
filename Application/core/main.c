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

/*
 * TODO RTC
 * TODO WATCH
 * TODO ADC
 * TODO SENSORS
 * TODO SPI
 * TODO DIO
 */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "systick.h"
#include "pwr.h"
#include "flash.h"
#include "rcc.h"
#include "gpio.h"
#include "led.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define VTOR_ADDRESS    0x08000000

#define HSI_CLOCK       16000000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Параметры отслеживания работы FreeRTOS */
static size_t free_heap_size;
static size_t minimum_ever_free_heap_size;
static uint32_t appl_idle_hook_counter;

/* Private function prototypes --------------------------------------------- */

static void setup_hardware(void);

static void setup_vector_table(void);

static void setup_fpu(void);

static void app_main(void *arg);

/* Private user code ------------------------------------------------------- */

int main(void)
{
    setup_hardware();

    xTaskCreate(app_main,
                "app_main",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    vTaskStartScheduler();
}
/* ------------------------------------------------------------------------- */

void error(void)
{
    __disable_irq();

    /* Выключить все светодиоды */
    led_off(LED_ST);
    led_off(LED_TX);
    led_off(LED_RX);

    while (true) {
        /* Задержка */
        for (uint32_t i = 0; i < 5000; i++) {
            for (uint32_t j = 0; j < 1000; j++)
                __NOP();
        }

        /* Переключить светодиод состояния */
        led_toggle(LED_ST);
    }
}
/* ------------------------------------------------------------------------- */

static void app_main(void *arg)
{
    static const TickType_t frequency = pdMS_TO_TICKS(1000);

    /* INIT CODE BEGIN ----------------------------------------------------- */

    /* INIT CODE END ------------------------------------------------------- */

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, frequency);

        /* Включить светодиод состояния */
        led_on(LED_ST);

        /* Обновить информацию об используемой памяти FreeRTOS */
        free_heap_size = xPortGetFreeHeapSize();
        minimum_ever_free_heap_size = xPortGetMinimumEverFreeHeapSize();
    }
}
/* ------------------------------------------------------------------------- */

void vApplicationIdleHook(void)
{
    /* Отслеживание свободного времени FreeRTOS */
    appl_idle_hook_counter++;
}
/* ------------------------------------------------------------------------- */

static void setup_hardware(void)
{
    setup_vector_table();
    setup_fpu();

    systick_init(HSI_CLOCK);
    pwr_init();
    flash_init();
    rcc_init();
    systick_init(CPU_CLOCK);
    gpio_init();
}
/* ------------------------------------------------------------------------- */

static void setup_vector_table(void)
{
    __disable_irq();
    __set_PRIMASK(1);

    WRITE_REG(SCB->VTOR, VTOR_ADDRESS);

    __set_PRIMASK(0);
    __enable_irq();
}
/* ------------------------------------------------------------------------- */

static void setup_fpu(void)
{
    SET_BIT(SCB->CPACR, (0x03 << 20) | (0x03 << 22));
}
/* ------------------------------------------------------------------------- */
