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

#ifndef STM32F4XX_HAL_PWR_H_
#define STM32F4XX_HAL_PWR_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32f4xx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief           Включить тактирование PWR
 */
#define HAL_PWR_ENABLE_CLOCK() \
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief           Определение структуры данных PWR
 */
typedef PWR_TypeDef pwr_t;


/**
 * @brief           Определение перечисления масштабирования напряжения PWR
 */
enum pwr_vos {
    PWR_VOS3 = 1,
    PWR_VOS2,
    PWR_VOS1,
};


/**
 * @brief           Определение перечисления уровней PVD PWR
 */
enum pwr_pvd_level {
    PWR_PVD_2V0,
    PWR_PVD_2V1,
    PWR_PVD_2V3,
    PWR_PVD_2V5,
    PWR_PVD_2V6,
    PWR_PVD_2V7,
    PWR_PVD_2V8,
    PWR_PVD_2V9,
};


/**
 * @brief           Определение перечисления статусов PVD PWR
 */
enum pwr_pvd_status {
    PWR_VDD_HIGHER_PVD,
    PWR_VDD_LOWER_PVD,
};


/**
 * @brief           Определение перечисления идентификаторов функций обратного вызова PWR
 */
enum pwr_callback_id {
    PWR_PVD_STATUS_CHANGED_CALLBACK,
};


/**
 * @brief           Определение структуры данных инициализации PWR
 */
struct pwr_init {
    enum pwr_vos        vos;                        /*!< Значение масштабирования напряжения:
                                                        - PWR_VOS3
                                                        - PWR_VOS2
                                                        - PWR_VOS1 */

    hal_state_t         pvd_enable;                 /*!< Включить PVD:
                                                        - HAL_ENABLE
                                                        - HAL_DISABLE */

    enum pwr_pvd_level  pvd_level;                  /*!< Уровень PVD:
                                                        - PWR_PVD_2V0
                                                        - PWR_PVD_2V1
                                                        ...
                                                        - PWR_PVD_2V9 */

    hal_state_t         backup_regulator_enable;    /*!< Включить Backup Regulator:
                                                        - HAL_ENABLE
                                                        - HAL_DISABLE */

    hal_state_t         wkup1_enable;               /*!< Включить WKUP1:
                                                        - HAL_ENABLE
                                                        - HAL_DISABLE */
};


/**
 * @brief           Определение структуры данных обработчика PWR
 */
struct pwr_handle {
    pwr_t              *instance;               /*!< Указатель на структуру данных PWR */

    struct pwr_init     init;                   /*!< Настройки PWR */

    /* --- */

    void (*pvd_status_changed_callback)(void);
};

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_pwr_init(struct pwr_handle *handle);

void hal_pwr_pvd_it_handler(struct pwr_handle *handle);

bool hal_pwr_vos_is_ready(struct pwr_handle *handle);

enum pwr_pvd_status hal_pwr_pvd_status(struct pwr_handle *handle);

void hal_pwr_register_callback(struct pwr_handle *handle,
                               enum pwr_callback_id callback_id,
                               void (*callback)(void));

void hal_pwr_unregister_callback(struct pwr_handle *handle,
                                 enum pwr_callback_id callback_id);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32F4XX_HAL_PWR_H_ */
