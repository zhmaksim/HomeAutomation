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

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_systick.h"
#include "stm32f4xx_hal_pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define HAL_RCC_HSERDY_TIMEOUT      100
#define HAL_RCC_LSERDY_TIMEOUT      5000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик SysTick */
extern struct systick_handle systick;

/* Обработчик PWR */
extern struct pwr_handle pwr;

/* Private function prototypes --------------------------------------------- */

static void hal_rcc_setup_hse(rcc_t *instance, hal_state_t state);

static void hal_rcc_setup_css_hse(rcc_t *instance, hal_state_t state);

static void hal_rcc_setup_lse(rcc_t *instance, hal_state_t state);

static void hal_rcc_setup_pll(rcc_t *instance, struct rcc_pll_init *init);

static void hal_rcc_setup_ahb(rcc_t *instance, enum rcc_ahb_div div);

static void hal_rcc_setup_apb1(rcc_t *instance, enum rcc_apb_div div);

static void hal_rcc_setup_apb2(rcc_t *instance, enum rcc_apb_div div);

static void hal_rcc_setup_cpu_clock_source(rcc_t *instance, enum rcc_cpu_clock_source clksource);

static void hal_rcc_setup_mco1(rcc_t *instance, enum rcc_mco1_clock_source clksource, enum rcc_mco1_div div);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать RCC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика RCC
 */
void hal_rcc_init(struct rcc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_rcc_setup_hse(handle->instance, handle->init.hse_enable);
    hal_rcc_setup_css_hse(handle->instance, handle->init.css_enable);
    hal_rcc_setup_lse(handle->instance, handle->init.lse_enable);
    hal_rcc_setup_pll(handle->instance, &handle->init.pll_init);
    hal_rcc_setup_ahb(handle->instance, handle->init.ahb_div);
    hal_rcc_setup_apb1(handle->instance, handle->init.apb1_div);
    hal_rcc_setup_apb2(handle->instance, handle->init.apb2_div);
    hal_rcc_setup_cpu_clock_source(handle->instance, handle->init.cpu_clksource);
    hal_rcc_setup_mco1(handle->instance, handle->init.mco1_clksource, handle->init.mco1_div);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить HSE
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       state: Состояние HSE
 */
static void hal_rcc_setup_hse(rcc_t *instance, hal_state_t state)
{
    MODIFY_REG(instance->CR,
               RCC_CR_HSEON_Msk,
               state << RCC_CR_HSEON_Pos);

    if (state == HAL_ENABLE) {
        uint32_t tickstart = hal_systick_tick(&systick);

        while (!READ_BIT(instance->CR, RCC_CR_HSERDY_Msk)) {
            if (hal_systick_tick(&systick) - tickstart >= HAL_RCC_HSERDY_TIMEOUT)
                hal_error();
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить CSS HSE
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       state: Состояние CSS
 */
static void hal_rcc_setup_css_hse(rcc_t *instance, hal_state_t state)
{
    MODIFY_REG(instance->CR,
               RCC_CR_CSSON_Msk,
               state << RCC_CR_CSSON_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить LSE
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       state: Состояние LSE
 */
static void hal_rcc_setup_lse(rcc_t *instance, hal_state_t state)
{
    MODIFY_REG(instance->BDCR,
               RCC_BDCR_LSEON_Msk,
               state << RCC_BDCR_LSEON_Pos);

    if (state == HAL_ENABLE) {
        uint32_t tickstart = hal_systick_tick(&systick);

        while (!READ_BIT(instance->BDCR, RCC_BDCR_LSERDY_Msk)) {
            if (hal_systick_tick(&systick) - tickstart >= HAL_RCC_LSERDY_TIMEOUT)
                hal_error();
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить PLL
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       init: Указатель на структуру данных настройки PLL
 */
static void hal_rcc_setup_pll(rcc_t *instance, struct rcc_pll_init *init)
{
    /* Выключить PLL перед настройкой */
    CLEAR_BIT(instance->CR, RCC_CR_PLLON_Msk);

    /* Настроить источник тактирования */
    MODIFY_REG(instance->PLLCFGR,
               RCC_PLLCFGR_PLLSRC_Msk,
               init->clksource << RCC_PLLCFGR_PLLSRC_Pos);

    /* Настроить DIVM */
    MODIFY_REG(instance->PLLCFGR,
               RCC_PLLCFGR_PLLM_Msk,
               init->divm << RCC_PLLCFGR_PLLM_Pos);

    /* Настроить DIVN */
    MODIFY_REG(instance->PLLCFGR,
               RCC_PLLCFGR_PLLN_Msk,
               init->divn << RCC_PLLCFGR_PLLN_Pos);

    /* Настроить DIVP */
    MODIFY_REG(instance->PLLCFGR,
               RCC_PLLCFGR_PLLP_Msk,
               ((init->divp / 2) - 1) << RCC_PLLCFGR_PLLP_Pos);

    /* Настроить DIVQ */
    MODIFY_REG(instance->PLLCFGR,
               RCC_PLLCFGR_PLLQ_Msk,
               init->divq << RCC_PLLCFGR_PLLQ_Pos);

    /* Настроить DIVR */
    MODIFY_REG(instance->PLLCFGR,
               RCC_PLLCFGR_PLLR_Msk,
               init->divr << RCC_PLLCFGR_PLLR_Pos);

    /* Включить PLL */
    if (init->enable == HAL_ENABLE) {
        SET_BIT(instance->CR, RCC_CR_PLLON_Msk);
        while (!READ_BIT(instance->CR, RCC_CR_PLLRDY_Msk))
            continue;

        /* Дождаться готовности PWR VOS после включения PLL */
        while (hal_pwr_vos_is_ready(&pwr) != true)
            continue;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить AHB
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       div: Делитель AHB
 */
static void hal_rcc_setup_ahb(rcc_t *instance, enum rcc_ahb_div div)
{
    MODIFY_REG(instance->CFGR,
               RCC_CFGR_HPRE_Msk,
               div << RCC_CFGR_HPRE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить APB1
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       div: Делитель APB
 */
static void hal_rcc_setup_apb1(rcc_t *instance, enum rcc_apb_div div)
{
    MODIFY_REG(instance->CFGR,
               RCC_CFGR_PPRE1_Msk,
               div << RCC_CFGR_PPRE1_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить APB2
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       div: Делитель APB
 */
static void hal_rcc_setup_apb2(rcc_t *instance, enum rcc_apb_div div)
{
    MODIFY_REG(instance->CFGR,
               RCC_CFGR_PPRE2_Msk,
               div << RCC_CFGR_PPRE2_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить источник тактирования CPU
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       clksource: Источник тактирования CPU
 */
static void hal_rcc_setup_cpu_clock_source(rcc_t *instance, enum rcc_cpu_clock_source clksource)
{
    MODIFY_REG(instance->CFGR,
               RCC_CFGR_SW_Msk,
               clksource << RCC_CFGR_SW_Pos);

    while (READ_BIT(instance->CFGR, RCC_CFGR_SWS_Msk) !=
            clksource << RCC_CFGR_SWS_Pos)
        continue;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить MCO1
 *
 * @param[in]       instance: Указатель на структуру данных RCC
 * @param[in]       clksource: Источник тактирования MCO1
 * @param[in]       div: Делитель MCO1
 */
static void hal_rcc_setup_mco1(rcc_t *instance, enum rcc_mco1_clock_source clksource, enum rcc_mco1_div div)
{
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_MCO1_Msk
             | RCC_CFGR_MCO1PRE_Msk,
               clksource << RCC_CFGR_MCO1_Pos
             | div << RCC_CFGR_MCO1PRE_Pos);
}
/* ------------------------------------------------------------------------- */
