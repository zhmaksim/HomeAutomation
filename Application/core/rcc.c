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

#define RCC_HSERDY_TIMEOUT      100
#define RCC_LSERDY_TIMEOUT      5000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать RCC
 */
void rcc_init(void)
{
    uint32_t tickstart;

    /* Включить HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON_Msk);

    tickstart = systick_get_tick();

    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY_Msk)) {
        if (systick_get_tick() - tickstart >= RCC_HSERDY_TIMEOUT)
            error();
    }

    /* Включить CSS HSE */
    SET_BIT(RCC->CR, RCC_CR_CSSON_Msk);

    /* Включить LSE */
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEON_Msk);

    tickstart = systick_get_tick();

    while (!READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY_Msk)) {
        if (systick_get_tick() - tickstart >= RCC_LSERDY_TIMEOUT)
            error();
    }

    /* Настроить PLL */
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON_Msk);

    /* Настроить источник тактирования = HSE (16MHz) */
    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk);

    /* Настроить DIVM = /8 (16MHz / 8 = 2MHz) */
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLM_Msk,
               8 << RCC_PLLCFGR_PLLM_Pos);

    /* Настроить DIVN = x168 (2MHz * 168 = 336MHz) */
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLN_Msk,
               168 << RCC_PLLCFGR_PLLN_Pos);

    /* Настроить DIVP = /2 (336MHz / 2 = 168MHz) */
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk);

    /* Настроить DIVQ = /7 (336MHz / 7 = 48MHz) */
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLQ_Msk,
               7 << RCC_PLLCFGR_PLLQ_Pos);

    /* Настроить DIVR = /7 (336MHz / 7 = 48MHz) */
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLLR_Msk,
               7 << RCC_PLLCFGR_PLLR_Pos);

    /* Включить PLL */
    SET_BIT(RCC->CR, RCC_CR_PLLON_Msk);
    while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY_Msk))
        continue;

    /* Дождаться готовности PWR VOS после включения PLL */
    while (!pwr_vos_is_ready())
        continue;

    /* Настроить AHB = /1, APB1 = /4, APB2 = /2 */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_HPRE_Msk
             | RCC_CFGR_PPRE1_Msk
             | RCC_CFGR_PPRE2_Msk,
               0x05 << RCC_CFGR_PPRE1_Pos
             | 0x04 << RCC_CFGR_PPRE2_Pos);

    /* Настроить источник тактирования CPU = PLLP */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_SW_Msk,
               0x02 << RCC_CFGR_SW_Pos);
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) !=
            0x02 << RCC_CFGR_SWS_Pos)
        continue;

    /* Настроить MCO1 = LSE */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_MCO1_Msk
             | RCC_CFGR_MCO1PRE_Msk,
               0x01 << RCC_CFGR_MCO1_Pos);
}
/* ------------------------------------------------------------------------- */
