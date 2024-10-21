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

#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_systick.h"
#include "stm32f4xx_hal_rcc.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define HAL_RTC_TIMEOUT     100

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик SysTick */
extern struct systick_handle systick;

/* Обработчик RCC */
extern struct rcc_handle rcc;

/* Private function prototypes --------------------------------------------- */

static void hal_rtc_setup_clksource(enum rtc_clock_source clksource);

static void hal_rtc_enable(void);

static void hal_rtc_enable_protect(rtc_t *instance);

static void hal_rtc_disable_protect(rtc_t *instance);

static void hal_rtc_setup_div(rtc_t *instance, uint16_t sync, uint16_t async);

static void hal_rtc_setup_hour_format(rtc_t *instance, enum rtc_hour_format format);

static void hal_rtc_setup_ref_clock_detect(rtc_t *instance, hal_state_t state);

static void hal_rtc_setup_calib_output(rtc_t *instance, hal_state_t state, enum rtc_calib_output sel);

static hal_status_t hal_rtc_wait_synchro(rtc_t *instance);

static hal_status_t hal_rtc_enter_init_mode(rtc_t *instance);

static hal_status_t hal_rtc_exit_init_mode(rtc_t *instance);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать RTC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика RTC
 */
void hal_rtc_init(struct rtc_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_rtc_setup_clksource(handle->init.clksource);
    hal_rtc_enable();

    hal_rtc_disable_protect(handle->instance);
    while (hal_rtc_enter_init_mode(handle->instance) != HAL_OK)
        continue;

    hal_rtc_setup_div(handle->instance, handle->init.sync_div, handle->init.async_div);
    hal_rtc_setup_hour_format(handle->instance, handle->init.hour_format);
    hal_rtc_setup_ref_clock_detect(handle->instance, handle->init.ref_clock_detect_enable);
    hal_rtc_setup_calib_output(handle->instance, handle->init.calib_output_enable, handle->init.calib_output_sel);

    hal_rtc_exit_init_mode(handle->instance);
    hal_rtc_enable_protect(handle->instance);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить источник такстирования RTC
 *
 * @param[in]       clksource: Источник тактирования RTC
 */
static void hal_rtc_setup_clksource(enum rtc_clock_source clksource)
{
    SET_BIT(rcc.instance->BDCR, clksource << RCC_BDCR_RTCSEL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить RTC
 */
static void hal_rtc_enable(void)
{
    SET_BIT(rcc.instance->BDCR, RCC_BDCR_RTCEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Включить защиту RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 */
static void hal_rtc_enable_protect(rtc_t *instance)
{
    WRITE_REG(instance->WPR, 0xFF);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить защиту RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 */
static void hal_rtc_disable_protect(rtc_t *instance)
{
    WRITE_REG(instance->WPR, 0xCA);
    WRITE_REG(instance->WPR, 0x53);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить делители часов RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 * @param[in]       sync: Синхронный делитель часов RTC
 * @param[in]       async: Асинхронный делитель часов RTC
 */
static void hal_rtc_setup_div(rtc_t *instance, uint16_t sync, uint16_t async)
{
    MODIFY_REG(instance->PRER,
               RTC_PRER_PREDIV_S_Msk,
               sync << RTC_PRER_PREDIV_S_Pos);
    MODIFY_REG(instance->PRER,
               RTC_PRER_PREDIV_A_Msk,
               async << RTC_PRER_PREDIV_A_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить формат времени RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 * @param[in]       format: Формат времени RTC
 */
static void hal_rtc_setup_hour_format(rtc_t *instance, enum rtc_hour_format format)
{
    MODIFY_REG(instance->CR,
               RTC_CR_FMT_Msk,
               format << RTC_CR_FMT_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить обнаружение опорной частоты (50 или 60 Гц)
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 * @param[in]       state: Состояние Reference Clock Detect RTC
 */
static void hal_rtc_setup_ref_clock_detect(rtc_t *instance, hal_state_t state)
{
    MODIFY_REG(instance->CR,
               RTC_CR_REFCKON_Msk,
               state << RTC_CR_REFCKON_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить сигнал калибровки RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 * @param[in]       state: Состояние сигнала калибровки RTC
 * @param[in]       sel: Сигнал калибровки RTC
 */
static void hal_rtc_setup_calib_output(rtc_t *instance, hal_state_t state, enum rtc_calib_output sel)
{
    MODIFY_REG(instance->CR,
               RTC_CR_COE_Msk
             | RTC_CR_COSEL_Msk,
               state << RTC_CR_COE_Pos
             | sel << RTC_CR_COSEL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Текущие дата и время RTC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика RTC
 * @param[out]      datetime: Указатель на структуру данных даты и времени RTC
 */
void hal_rtc_now(struct rtc_handle *handle, struct rtc_datetime *datetime)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Прочитать значения даты и времени RTC */
    uint32_t TR = READ_REG(handle->instance->TR);
    uint32_t DR = READ_REG(handle->instance->DR);

    /* Часы */
    datetime->hours = ((READ_BIT(TR, RTC_TR_HT_Msk) >> RTC_TR_HT_Pos) * 10 +
                       (READ_BIT(TR, RTC_TR_HU_Msk) >> RTC_TR_HU_Pos));

    /* Минуты */
    datetime->minutes = ((READ_BIT(TR, RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos) * 10 +
                         (READ_BIT(TR, RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos));

    /* Секунды */
    datetime->seconds = ((READ_BIT(TR, RTC_TR_ST_Msk) >> RTC_TR_ST_Pos) * 10 +
                         (READ_BIT(TR, RTC_TR_SU_Msk) >> RTC_TR_SU_Pos));

    /* Год */
    datetime->year = ((READ_BIT(DR, RTC_DR_YT_Msk) >> RTC_DR_YT_Pos) * 10 +
                      (READ_BIT(DR, RTC_DR_YU_Msk) >> RTC_DR_YU_Pos));

    /* День недели */
    datetime->weekday = (READ_BIT(DR, RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos);

    /* Месяц */
    datetime->month = ((READ_BIT(DR, RTC_DR_MT_Msk) >> RTC_DR_MT_Pos) * 10 +
                       (READ_BIT(DR, RTC_DR_MU_Msk) >> RTC_DR_MU_Pos));

    /* День */
    datetime->day = ((READ_BIT(DR, RTC_DR_DT_Msk) >> RTC_DR_DT_Pos) * 10 +
                     (READ_BIT(DR, RTC_DR_DU_Msk) >> RTC_DR_DU_Pos));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить дату и время RTC
 *
 * @param[in]       handle: Указатель на структуру данных обработчика RTC
 * @param[in]       datetime: Указатель на структуру данных даты и времени RTC
 */
void hal_rtc_setup(struct rtc_handle *handle, const struct rtc_datetime *datetime)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    uint32_t DR = 0;
    uint32_t TR = 0;

    /* Год */
    SET_BIT(DR, ((datetime->year / 10) & 0x0F) << RTC_DR_YT_Pos);
    SET_BIT(DR, ((datetime->year % 10) & 0x0F) << RTC_DR_YU_Pos);

    /* День недели */
    SET_BIT(DR, (datetime->weekday & 0x07) << RTC_DR_WDU_Pos);

    /* Месяц */
    SET_BIT(DR, ((datetime->month / 10) & 0x01) << RTC_DR_MT_Pos);
    SET_BIT(DR, ((datetime->month % 10) & 0x0F) << RTC_DR_MU_Pos);

    /* День */
    SET_BIT(DR, ((datetime->day / 10) & 0x03) << RTC_DR_DT_Pos);
    SET_BIT(DR, ((datetime->day % 10) & 0x0F) << RTC_DR_DU_Pos);

    /* Часы */
    SET_BIT(TR, ((datetime->hours / 10) & 0x03) << RTC_TR_HT_Pos);
    SET_BIT(TR, ((datetime->hours % 10) & 0x0F) << RTC_TR_HU_Pos);

    /* Минуты */
    SET_BIT(TR, ((datetime->minutes / 10) & 0x07) << RTC_TR_MNT_Pos);
    SET_BIT(TR, ((datetime->minutes % 10) & 0x0F) << RTC_TR_MNU_Pos);

    /* Секунды */
    SET_BIT(TR, ((datetime->seconds / 10) & 0x07) << RTC_TR_ST_Pos);
    SET_BIT(TR, ((datetime->seconds % 10) & 0x0F) << RTC_TR_SU_Pos);

    hal_rtc_disable_protect(handle->instance);
    while (hal_rtc_enter_init_mode(handle->instance) != HAL_OK)
        continue;

    /* Записать значения даты и времени */
    WRITE_REG(handle->instance->DR, DR);
    WRITE_REG(handle->instance->TR, TR);

    hal_rtc_exit_init_mode(handle->instance);
    hal_rtc_enable_protect(handle->instance);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Ожидание синхронизации RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_rtc_wait_synchro(rtc_t *instance)
{
    CLEAR_BIT(instance->ISR, RTC_ISR_RSF_Msk);

    uint32_t tickstart = hal_systick_tick(&systick);

    while (!READ_BIT(instance->ISR, RTC_ISR_RSF_Msk)) {
        if (hal_systick_tick(&systick) - tickstart >= HAL_RTC_TIMEOUT)
            return HAL_ERROR;
    }

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Вход в режим инициализации RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_rtc_enter_init_mode(rtc_t *instance)
{
    if (!READ_BIT(instance->ISR, RTC_ISR_INITF_Msk)) {
        SET_BIT(instance->ISR, RTC_ISR_INIT_Msk);

        uint32_t tickstart = hal_systick_tick(&systick);

        while (!READ_BIT(instance->ISR, RTC_ISR_INITF_Msk)) {
            if (hal_systick_tick(&systick) - tickstart >= HAL_RTC_TIMEOUT)
                return HAL_ERROR;
        }
    }

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выход из режима инициализации RTC
 *
 * @param[in]       instance: Указатель на структуру данных RTC
 * @return          Статус @ref hal_status_t
 */
static hal_status_t hal_rtc_exit_init_mode(rtc_t *instance)
{
    CLEAR_BIT(instance->ISR, RTC_ISR_INIT_Msk);

    if (!READ_BIT(instance->CR, RTC_CR_BYPSHAD_Msk)) {
        if (hal_rtc_wait_synchro(instance) != HAL_OK)
            return HAL_ERROR;
    }

    return HAL_OK;
}
/* ------------------------------------------------------------------------- */
