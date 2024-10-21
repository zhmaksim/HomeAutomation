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

#ifndef STM32F4XX_HAL_RCC_H_
#define STM32F4XX_HAL_RCC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование Backup SRAM
 */
#define HAL_RCC_ENABLE_CLOCK_BACKUP_SRAM() \
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_BKPSRAMEN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных RCC
 */
typedef RCC_TypeDef rcc_t;


/**
 * @brief           Определение перечисления источников тактирования PLL
 */
enum rcc_pll_clock_source {
    RCC_PLL_CLOCK_SOURCE_HSI,
    RCC_PLL_CLOCK_SOURCE_HSE,
};


/**
 * @brief           Определение перечисления делителей AHB
 */
enum rcc_ahb_div {
    RCC_AHB_NOT_DIV,
    RCC_AHB_DIV2 = 8,
    RCC_AHB_DIV4,
    RCC_AHB_DIV8,
    RCC_AHB_DIV16,
    RCC_AHB_DIV64,
    RCC_AHB_DIV128,
    RCC_AHB_DIV256,
    RCC_AHB_DIV512,
};


/**
 * @brief           Определение перечисления делителей APB
 */
enum rcc_apb_div {
    RCC_APB_NOT_DIV,
    RCC_APB_DIV2 = 4,
    RCC_APB_DIV4,
    RCC_APB_DIV8,
    RCC_APB_DIV16,
};


/**
 * @brief           Определение перечисления источников тактирования CPU
 */
enum rcc_cpu_clock_source {
    RCC_CPU_CLOCK_SOURCE_HSI,
    RCC_CPU_CLOCK_SOURCE_HSE,
    RCC_CPU_CLOCK_SOURCE_PLLP,
    RCC_CPU_CLOCK_SOURCE_PLLR,
};


/**
 * @brief           Определение перечисления источников тактирования MCO1
 */
enum rcc_mco1_clock_source {
    RCC_MCO1_CLOCK_SOURCE_HSI,
    RCC_MCO1_CLOCK_SOURCE_LSE,
    RCC_MCO1_CLOCK_SOURCE_HSE,
    RCC_MCO1_CLOCK_SOURCE_PLL,
};


/**
 * @brief           Определение перечисления делителей MCO1
 */
enum rcc_mco1_div {
    RCC_MCO1_NOT_DIV,
    RCC_MCO1_DIV2,
    RCC_MCO1_DIV3,
    RCC_MCO1_DIV4,
    RCC_MCO1_DIV5,
};


/**
 * @brief           Определение структуры данных настройки PLL
 */
struct rcc_pll_init {
    hal_state_t                 enable;         /*!< Включить PLL:
                                                    - HAL_ENABLE
                                                    - HAL_DISABLE */

    enum rcc_pll_clock_source   clksource;      /*!< Источник тактирования:
                                                    - RCC_PLL_CLOCK_SOURCE_HSI
                                                    - RCC_PLL_CLOCK_SOURCE_HSE */

    uint16_t                    divm;           /*!< Делитель DIVM = 2 <= PLLM <= 63
                                                    Рекомендуется выбрать частоту 2 МГц */

    uint16_t                    divn;           /*!< Делитель DIVN = 50 <= PLLN <= 432
                                                    Выходная частота должна находится в диапазоне от 100 до 432 МГц */

    uint16_t                    divp;           /*!< Делитель DIVP = 2, 4, 6 или 8 */

    uint16_t                    divq;           /*!< Делитель DIVQ = 2 <= PLLQ <= 15 */

    uint16_t                    divr;           /*!< Делитель DIVR = 2 <= PLLQ <= 7 */
};


/**
 * @brief           Определение структуры данных настройки RCC
 */
struct rcc_init {
    hal_state_t                 hse_enable;     /*!< Включить HSE:
                                                    - HAL_ENABLE
                                                    - HAL_DISABLE */

    hal_state_t                 css_enable;     /*!< Включить CSS:
                                                    - HAL_ENABLE
                                                    - HAL_DISABLE */

    hal_state_t                 lse_enable;     /*!< Включить LSE:
                                                    - HAL_ENABLE
                                                    - HAL_DISABLE */

    struct rcc_pll_init         pll_init;       /*!< Структура данных настройки PLL */

    enum rcc_ahb_div            ahb_div;        /*!< Делитель AHB:
                                                    - RCC_AHB_NOT_DIV
                                                    - RCC_AHB_DIV2
                                                    ...
                                                    - RCC_AHB_DIV512 */

    enum rcc_apb_div            apb1_div;       /*!< Делитель APB1:
                                                    - RCC_APB_NOT_DIV
                                                    - RCC_APB_DIV2
                                                    ...
                                                    - RCC_APB_DIV16 */

    enum rcc_apb_div            apb2_div;       /*!< Делитель APB2:
                                                    - RCC_APB_NOT_DIV
                                                    - RCC_APB_DIV2
                                                    ...
                                                    - RCC_APB_DIV16 */

    enum rcc_cpu_clock_source   cpu_clksource;  /*!< Источник тактирования CPU:
                                                    - RCC_CPU_CLOCK_SOURCE_HSI
                                                    - RCC_CPU_CLOCK_SOURCE_HSE
                                                    - RCC_CPU_CLOCK_SOURCE_PLL */

    enum rcc_mco1_clock_source  mco1_clksource; /*!< Источник тактирования MCO1:
                                                    - RCC_MCO1_CLOCK_SOURCE_HSI
                                                    - RCC_MCO1_CLOCK_SOURCE_LSE
                                                    - RCC_MCO1_CLOCK_SOURCE_HSE
                                                    - RCC_MCO1_CLOCK_SOURCE_PLL */

    enum rcc_mco1_div           mco1_div;       /*!< Делитель MCO1:
                                                    - RCC_MCO1_NOT_DIV
                                                    - RCC_MCO1_DIV2
                                                    - RCC_MCO1_DIV3
                                                    - RCC_MCO1_DIV4
                                                    - RCC_MCO1_DIV5 */
};


/**
 * @brief           Определение структуры данных обработчика RCC
 */
struct rcc_handle {
    rcc_t              *instance;               /*!< Указатель на структуру данных RCC */

    struct rcc_init     init;                   /*!< Настройки RCC */

    uint32_t            ahb_clock;              /*!< Частота тактирования AHB (Гц) */

    uint32_t            apb1_clock;             /*!< Частота тактирования APB1 (Гц) */

    uint32_t            apb2_clock;             /*!< Частота тактирования APB2 (Гц) */

    uint32_t            cpu_clock;              /*!< Частота тактирования CPU (Гц) */
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_rcc_init(struct rcc_handle *handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_RCC_H_ */
