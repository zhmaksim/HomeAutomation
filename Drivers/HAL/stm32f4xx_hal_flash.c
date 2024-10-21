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

#include "stm32f4xx_hal_flash.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_flash_setup_latency(flash_t *instance, enum flash_latency latency);

static void hal_flash_setup_prefetch(flash_t *instance, hal_state_t state);

static void hal_flash_setup_icache(flash_t *instance, hal_state_t state);

static void hal_flash_setup_dcache(flash_t *instance, hal_state_t state);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать FLASH
 *
 * @param[in]       handle: Указатель на структуру данных обработчика FLASH
 */
void hal_flash_init(struct flash_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_flash_setup_latency(handle->instance, handle->init.latency);
    hal_flash_setup_prefetch(handle->instance, handle->init.prefetch_enable);
    hal_flash_setup_icache(handle->instance, handle->init.icache_enable);
    hal_flash_setup_dcache(handle->instance, handle->init.dcache_enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить задержку чтения FLASH
 *
 * @param[in]       instance: Указатель на структуру данных FLASH
 * @param[in]       latency: Задержка чтения
 */
static void hal_flash_setup_latency(flash_t *instance, enum flash_latency latency)
{
    MODIFY_REG(instance->ACR,
               FLASH_ACR_LATENCY_Msk,
               latency << FLASH_ACR_LATENCY_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить предварительную выборку данных
 *
 * @param[in]       instance: Указатель на структуру данных FLASH
 * @param[in]       state: Состояние предварительной выборки данных
 */
static void hal_flash_setup_prefetch(flash_t *instance, hal_state_t state)
{
    MODIFY_REG(instance->ACR,
               FLASH_ACR_PRFTEN_Msk,
               state << FLASH_ACR_PRFTEN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить кэш-инструкций
 *
 * @param[in]       instance: Указатель на структуру данных FLASH
 * @param[in]       state: Состояние кэша
 */
static void hal_flash_setup_icache(flash_t *instance, hal_state_t state)
{
    /* Выключить кэш-инструкций перед сбросом */
    CLEAR_BIT(instance->ACR, FLASH_ACR_ICEN_Msk);

    /* Сбросить кэш-инструкций */
    SET_BIT(instance->ACR, FLASH_ACR_ICRST_Msk);
    __NOP();
    CLEAR_BIT(instance->ACR, FLASH_ACR_ICRST_Msk);

    /* Установить значение состояния кэша-инструкций */
    MODIFY_REG(instance->ACR,
               FLASH_ACR_ICEN_Msk,
               state << FLASH_ACR_ICEN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить кэш-данных
 *
 * @param[in]       instance: Указатель на структуру данных FLASH
 * @param[in]       state Состояние кэша
 */
static void hal_flash_setup_dcache(flash_t *instance, hal_state_t state)
{
    /* Выключить кэш-данных перед сбросом */
    CLEAR_BIT(instance->ACR, FLASH_ACR_DCEN_Msk);

    /* Сбросить кэш-данных */
    SET_BIT(instance->ACR, FLASH_ACR_DCRST_Msk);
    __NOP();
    CLEAR_BIT(instance->ACR, FLASH_ACR_DCRST_Msk);

    /* Установить значение состояния кэша-данных */
    MODIFY_REG(instance->ACR,
               FLASH_ACR_DCEN_Msk,
               state << FLASH_ACR_DCEN_Pos);
}
/* ------------------------------------------------------------------------- */
