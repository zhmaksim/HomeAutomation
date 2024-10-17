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

#include "flash.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать FLASH
 */
void flash_init(void)
{
    /* Настроить задержку чтения флэш-памяти = 5WS */
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_LATENCY_Msk,
               0x05 << FLASH_ACR_LATENCY_Pos);

    /* Включить предварительную выборку данных */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN_Msk);

    /* Включить кэш-инструкций и кэш-данных */
    SET_BIT(FLASH->ACR,
            FLASH_ACR_ICEN_Msk
          | FLASH_ACR_DCEN_Msk);
}
/* ------------------------------------------------------------------------- */
