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

#include "pwr.h"
#include "stm32f446xx_it.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик PWR */
struct pwr_handle pwr = {
    .instance = PWR,
};

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать PWR
 */
void pwr_init(void)
{
    HAL_PWR_ENABLE_CLOCK();

    pwr.init.vos = PWR_VOS1;
    pwr.init.pvd_enable = HAL_ENABLE;
    pwr.init.pvd_level = PWR_PVD_2V9;
    pwr.init.backup_regulator_enable = HAL_ENABLE;
    pwr.init.wkup1_enable = HAL_ENABLE;

    hal_pwr_register_callback(&pwr,
                              PWR_PVD_STATUS_CHANGED_CALLBACK,
                              PWR_PVD_StatusChangedCallback);
    hal_pwr_init(&pwr);
}
/* ------------------------------------------------------------------------- */
