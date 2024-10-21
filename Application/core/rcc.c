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

#include "rcc.h"
#include "systick.h"
#include "pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик RCC */
struct rcc_handle rcc = {
    .instance = RCC,
};

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать RCC
 */
void rcc_init(void)
{
    rcc.init.hse_enable = HAL_ENABLE;
    rcc.init.css_enable = HAL_ENABLE;
    rcc.init.lse_enable = HAL_ENABLE;
    rcc.init.pll_init.enable = HAL_ENABLE;
    rcc.init.pll_init.clksource = RCC_PLL_CLOCK_SOURCE_HSE;
    rcc.init.pll_init.divm = 8;
    rcc.init.pll_init.divn = 168;
    rcc.init.pll_init.divp = 2;
    rcc.init.pll_init.divq = 7;
    rcc.init.pll_init.divr = 7;
    rcc.init.ahb_div = RCC_AHB_NOT_DIV;
    rcc.init.apb1_div = RCC_APB_DIV4;
    rcc.init.apb2_div = RCC_APB_DIV2;
    rcc.init.cpu_clksource = RCC_CPU_CLOCK_SOURCE_PLLP;
    rcc.init.mco1_clksource = RCC_MCO1_CLOCK_SOURCE_LSE;
    rcc.init.mco1_div = RCC_MCO1_NOT_DIV;

    rcc.ahb_clock = 168000000;
    rcc.apb1_clock = 42000000;
    rcc.apb2_clock = 84000000;
    rcc.cpu_clock = 168000000;

    hal_rcc_init(&rcc);

    HAL_RCC_ENABLE_CLOCK_BACKUP_SRAM();
}
/* ------------------------------------------------------------------------- */
