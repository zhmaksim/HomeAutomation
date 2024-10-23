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

#include "tim.h"
#include "stm32f446xx_it.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик TIM7 */
struct tim_handle tim7 = {
    .instance = TIM7,
};

/* Private function prototypes --------------------------------------------- */

static void tim7_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать TIM
 */
void tim_init(void)
{
    HAL_TIM7_ENABLE_CLOCK();

    tim7_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать TIM7 (1 мс)
 */
static void tim7_init(void)
{
    tim7.init.prescaler = 840 - 1;
    tim7.init.reload = 100 - 1;

    hal_tim_register_callback(&tim7,
                              TIM_PERIOD_ELAPSED_CALLBACK,
                              TIM_PeriodElapsedCallback);
    hal_tim_init(&tim7);
    hal_tim_start_it(&tim7);

    NVIC_SetPriority(TIM7_IRQn, 15);
    NVIC_EnableIRQ(TIM7_IRQn);
}
/* ------------------------------------------------------------------------- */
