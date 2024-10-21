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

#include "stm32f4xx_hal_pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_pwr_disable_backup_protect(pwr_t *instance);

static void hal_pwr_setup_vos(pwr_t *instance, enum pwr_vos vos);

static void hal_pwr_setup_pvd(pwr_t *instance, hal_state_t state, enum pwr_pvd_level level);

static void hal_pwr_setup_backup_regulator(pwr_t *instance, hal_state_t state);

static void hal_pwr_setup_wkup1(pwr_t *instance, hal_state_t state);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать PWR
 *
 * @param[in]       handle: Указатель на структуру данных обработчика PWR
 */
void hal_pwr_init(struct pwr_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    hal_pwr_disable_backup_protect(handle->instance);
    hal_pwr_setup_vos(handle->instance, handle->init.vos);
    hal_pwr_setup_pvd(handle->instance, handle->init.pvd_enable, handle->init.pvd_level);
    hal_pwr_setup_backup_regulator(handle->instance, handle->init.backup_regulator_enable);
    hal_pwr_setup_wkup1(handle->instance, handle->init.wkup1_enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Выключить защиту Backup домена
 *
 * @param[in]       instance: Указатель на структуру данных PWR
 */
static void hal_pwr_disable_backup_protect(pwr_t *instance)
{
    SET_BIT(instance->CR, PWR_CR_DBP_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить VOS
 *
 * @param[in]       instance: Указатель на структуру данных PWR
 * @param[in]       vos: Масштабирование напряжения
 */
static void hal_pwr_setup_vos(pwr_t *instance, enum pwr_vos vos)
{
    MODIFY_REG(instance->CR,
               PWR_CR_VOS_Msk,
               vos << PWR_CR_VOS_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить PVD
 *
 * @param[in]       instance: Указатель на структуру данных PWR
 * @param[in]       state: Состояние PVD
 * @param[in]       level: Уровень PVD
 */
static void hal_pwr_setup_pvd(pwr_t *instance, hal_state_t state, enum pwr_pvd_level level)
{
    MODIFY_REG(instance->CR,
               PWR_CR_PVDE_Msk
             | PWR_CR_PLS_Msk,
               state << PWR_CR_PVDE_Pos
             | level << PWR_CR_PLS_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить Backup Regulator
 *
 * @param[in]       instance: Указатель на структуру данных PWR
 * @param[in]       state: Состояние Backup Regulator
 */
static void hal_pwr_setup_backup_regulator(pwr_t *instance, hal_state_t state)
{
    MODIFY_REG(instance->CSR,
               PWR_CSR_BRE_Msk,
               state << PWR_CSR_BRE_Pos);

    if (state == HAL_ENABLE) {
        while (!READ_BIT(PWR->CSR, PWR_CSR_BRR_Msk))
            continue;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Настроить WKUP1
 *
 * @param[in]       instance: Указатель на структуру данных PWR
 * @param[in]       state: Состояние WKUP1
 */
static void hal_pwr_setup_wkup1(pwr_t *instance, hal_state_t state)
{
    MODIFY_REG(instance->CR,
               PWR_CSR_EWUP1_Msk,
               state << PWR_CSR_EWUP1_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Обработать прерывание PWR PVD
 *
 * @param[in]       handle: Указатель на структуру данных обработчика PWR
 */
void hal_pwr_pvd_it_handler(struct pwr_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    /* Проверить прерывание EXTI16 */
    if (READ_BIT(EXTI->PR, EXTI_PR_PR16_Msk)) {
        /* Сбросить прерывание EXTI16 */
        SET_BIT(EXTI->PR, EXTI_PR_PR16_Msk);

        if (handle->pvd_status_changed_callback != NULL)
            handle->pvd_status_changed_callback();
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Проверить готовность VOS
 *
 * @param[in]       handle: Указатель на структуру данных обработчика PWR
 * @return          Состояние готовности VOS
 */
bool hal_pwr_vos_is_ready(struct pwr_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    return READ_BIT(handle->instance->CSR, PWR_CSR_VOSRDY_Msk) ? true : false;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Получить статус PVD PWR
 *
 * @param[in]       handle: Указатель на структуру данных обработчика PWR
 * @return          Статус PVD
 */
enum pwr_pvd_status hal_pwr_pvd_status(struct pwr_handle *handle)
{
    assert(handle != NULL);
    assert(handle->instance != NULL);

    return READ_BIT(handle->instance->CSR, PWR_CSR_PVDO_Msk) ?
            PWR_VDD_LOWER_PVD : PWR_VDD_HIGHER_PVD;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Зарегистрировать функцию обратного вызова PWR
 *
 * @param[in]       handle: Указатель на структуру данных обработчика PWR
 * @param[in]       callback_id: Идентификатор функции обратного вызова
 * @param[in]       callback: Указатель на функцию обратного вызова
 */
void hal_pwr_register_callback(struct pwr_handle *handle,
                               enum pwr_callback_id callback_id,
                               void (*callback)(void))
{
    assert(handle != NULL);

    switch (callback_id) {
        case PWR_PVD_STATUS_CHANGED_CALLBACK:
            handle->pvd_status_changed_callback = callback;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Отменить регистрацию функции обратного вызова PWR
 *
 * @param[in]       handle: Указатель на структуру данных обработчика PWR
 * @param[in]       callback_id: Идентификатор функции обратного вызова
 */
void hal_pwr_unregister_callback(struct pwr_handle *handle,
                                 enum pwr_callback_id callback_id)
{
    assert(handle != NULL);

    switch (callback_id) {
        case PWR_PVD_STATUS_CHANGED_CALLBACK:
            handle->pvd_status_changed_callback = NULL;
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------------------- */
