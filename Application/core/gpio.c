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

#include "gpio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Обработчик GPIO LED ST */
struct gpio_handle gpio_led_st = {
    .instance = GPIOB,
    .pin = GPIO_PIN0,
};

/* Обработчик GPIO LED TX */
struct gpio_handle gpio_led_tx = {
    .instance = GPIOB,
    .pin = GPIO_PIN4,
};

/* Обработчик GPIO LED RX */
struct gpio_handle gpio_led_rx = {
    .instance = GPIOB,
    .pin = GPIO_PIN5,
};

/* Обработчик GPIO RS485 USART1 */
struct gpio_handle gpio_rs485_usart1 = {
    .instance = GPIOA,
    .pin = GPIO_PIN11,
};

/* Обработчик GPIO RS485 USART2 */
struct gpio_handle gpio_rs485_usart2 = {
    .instance = GPIOA,
    .pin = GPIO_PIN1,
};

/* Обработчик GPIO W25Q_CS */
struct gpio_handle gpio_w25q_cs = {
    .instance = GPIOA,
    .pin = GPIO_PIN4,
};

/* Обработчик GPIO 74HC595_EN */
struct gpio_handle gpio_74hc595_en = {
    .instance = GPIOB,
    .pin = GPIO_PIN2,
};

/* Обработчик GPIO 74HC595_CS */
struct gpio_handle gpio_74hc595_cs = {
    .instance = GPIOB,
    .pin = GPIO_PIN10,
};

/* Обработчик GPIO 74HC165_CS */
struct gpio_handle gpio_74hc165_cs = {
    .instance = GPIOB,
    .pin = GPIO_PIN12,
};

/* Обработчик GPIO W5500_RESET */
struct gpio_handle gpio_w5500_reset = {
    .instance = GPIOB,
    .pin = GPIO_PIN3,
};

/* Обработчик GPIO W5500_CS */
struct gpio_handle gpio_w5500_cs = {
    .instance = GPIOA,
    .pin = GPIO_PIN15,
};

/* Обработчик GPIO W5500_INT */
struct gpio_handle gpio_w5500_int = {
    .instance = GPIOD,
    .pin = GPIO_PIN2,
};

/* Private function prototypes --------------------------------------------- */

static void gpio_mco_init(void);

static void gpio_led_init(void);

static void gpio_74hc_init(void);

static void gpio_rs485_init(void);

static void gpio_w25q_init(void);

static void gpio_w5500_init(void);

static void gpio_usart_init(void);

static void gpio_usart1_init(void);

static void gpio_usart2_init(void);

static void gpio_usart6_init(void);

static void gpio_spi_init(void);

static void gpio_spi1_init(void);

static void gpio_spi2_init(void);

static void gpio_spi3_init(void);

static void gpio_i2c_init(void);

static void gpio_i2c1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO
 */
void gpio_init(void)
{
    /* Включить тактирование */
    HAL_GPIOA_ENABLE_CLOCK();
    HAL_GPIOB_ENABLE_CLOCK();
    HAL_GPIOC_ENABLE_CLOCK();
    HAL_GPIOD_ENABLE_CLOCK();

    gpio_mco_init();
    gpio_led_init();
    gpio_74hc_init();
    gpio_rs485_init();
    gpio_w25q_init();
    gpio_w5500_init();
    gpio_usart_init();
    gpio_spi_init();
    gpio_i2c_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO MCO
 */
static void gpio_mco_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_PUSH_PULL,
        .ospeed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 0,
    };

    struct gpio_handle gpio_mco1 = {
        .instance = GPIOA,
        .pin = GPIO_PIN8,
    };

    gpio_mco1.init = init;

    hal_gpio_init(&gpio_mco1);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO LED
 */
static void gpio_led_init(void)
{
    static const struct gpio_init init = {
         .mode = GPIO_OUTPUT,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_LOW_SPEED,
         .pupd = GPIO_PULL_UP,
         .af = 0,
    };

    gpio_led_st.init = init;
    gpio_led_tx.init = init;
    gpio_led_rx.init = init;

    hal_gpio_set_state(&gpio_led_st, GPIO_RESET);
    hal_gpio_set_state(&gpio_led_tx, GPIO_RESET);
    hal_gpio_set_state(&gpio_led_rx, GPIO_RESET);
    hal_gpio_init(&gpio_led_st);
    hal_gpio_init(&gpio_led_tx);
    hal_gpio_init(&gpio_led_rx);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO 74HC
 */
static void gpio_74hc_init(void)
{
    static const struct gpio_init init = {
         .mode = GPIO_OUTPUT,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_VERY_HIGH_SPEED,
         .pupd = GPIO_PULL_UP,
         .af = 0,
    };

    gpio_74hc595_en.init = init;
    gpio_74hc595_cs.init = init;
    gpio_74hc165_cs.init = init;

    hal_gpio_set_state(&gpio_74hc595_en, GPIO_SET);
    hal_gpio_set_state(&gpio_74hc595_cs, GPIO_SET);
    hal_gpio_set_state(&gpio_74hc165_cs, GPIO_RESET);
    hal_gpio_init(&gpio_74hc595_en);
    hal_gpio_init(&gpio_74hc595_cs);
    hal_gpio_init(&gpio_74hc165_cs);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO RS485
 */
static void gpio_rs485_init(void)
{
    static const struct gpio_init init = {
         .mode = GPIO_OUTPUT,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_VERY_HIGH_SPEED,
         .pupd = GPIO_PULL_DOWN,
         .af = 0,
    };

    gpio_rs485_usart1.init = init;
    gpio_rs485_usart2.init = init;

    hal_gpio_set_state(&gpio_rs485_usart1, GPIO_RESET);
    hal_gpio_set_state(&gpio_rs485_usart2, GPIO_RESET);
    hal_gpio_init(&gpio_rs485_usart1);
    hal_gpio_init(&gpio_rs485_usart2);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO W25Q
 */
static void gpio_w25q_init(void)
{
    static const struct gpio_init init = {
         .mode = GPIO_OUTPUT,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_VERY_HIGH_SPEED,
         .pupd = GPIO_PULL_UP,
         .af = 0,
    };

    gpio_w25q_cs.init = init;

    hal_gpio_set_state(&gpio_w25q_cs, GPIO_SET);
    hal_gpio_init(&gpio_w25q_cs);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO W5500
 */
static void gpio_w5500_init(void)
{
    static const struct gpio_init output_init = {
         .mode = GPIO_OUTPUT,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_VERY_HIGH_SPEED,
         .pupd = GPIO_PULL_UP,
         .af = 0,
    };

    static const struct gpio_init input_init = {
         .mode = GPIO_INPUT,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_VERY_HIGH_SPEED,
         .pupd = GPIO_PULL_UP,
         .af = 0,
    };

    gpio_w5500_reset.init = output_init;
    gpio_w5500_cs.init = output_init;
    gpio_w5500_int.init = input_init;

    hal_gpio_set_state(&gpio_w5500_reset, GPIO_SET);
    hal_gpio_set_state(&gpio_w5500_cs, GPIO_SET);
    hal_gpio_init(&gpio_w5500_reset);
    hal_gpio_init(&gpio_w5500_cs);
    hal_gpio_init(&gpio_w5500_int);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO USART
 */
static void gpio_usart_init(void)
{
    gpio_usart1_init();
    gpio_usart2_init();
    gpio_usart6_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO USART1
 */
static void gpio_usart1_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_PUSH_PULL,
        .ospeed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 7,
    };

    struct gpio_handle gpio_usart_tx = {
        .instance = GPIOA,
        .pin = GPIO_PIN9,
    };
    struct gpio_handle gpio_usart_rx = {
        .instance = GPIOA,
        .pin = GPIO_PIN10,
    };

    gpio_usart_tx.init = init;
    gpio_usart_rx.init = init;

    hal_gpio_init(&gpio_usart_tx);
    hal_gpio_init(&gpio_usart_rx);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO USART2
 */
static void gpio_usart2_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_PUSH_PULL,
        .ospeed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 7,
    };

    struct gpio_handle gpio_usart_tx = {
        .instance = GPIOA,
        .pin = GPIO_PIN2,
    };
    struct gpio_handle gpio_usart_rx = {
        .instance = GPIOA,
        .pin = GPIO_PIN3,
    };

    gpio_usart_tx.init = init;
    gpio_usart_rx.init = init;

    hal_gpio_init(&gpio_usart_tx);
    hal_gpio_init(&gpio_usart_rx);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO USART6
 */
static void gpio_usart6_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_PUSH_PULL,
        .ospeed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 8,
    };

    struct gpio_handle gpio_usart_tx = {
        .instance = GPIOC,
        .pin = GPIO_PIN6,
    };
    struct gpio_handle gpio_usart_rx = {
        .instance = GPIOC,
        .pin = GPIO_PIN7,
    };

    gpio_usart_tx.init = init;
    gpio_usart_rx.init = init;

    hal_gpio_init(&gpio_usart_tx);
    hal_gpio_init(&gpio_usart_rx);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI
 */
static void gpio_spi_init(void)
{
    gpio_spi1_init();
    gpio_spi2_init();
    gpio_spi3_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI1
 */
static void gpio_spi1_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_PUSH_PULL,
        .ospeed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 5,
    };

    struct gpio_handle gpio_spi_sck = {
        .instance = GPIOA,
        .pin = GPIO_PIN5,
    };
    struct gpio_handle gpio_spi_miso = {
        .instance = GPIOA,
        .pin = GPIO_PIN6,
    };
    struct gpio_handle gpio_spi_mosi = {
        .instance = GPIOA,
        .pin = GPIO_PIN7,
    };

    gpio_spi_sck.init = init;
    gpio_spi_miso.init = init;
    gpio_spi_mosi.init = init;

    hal_gpio_init(&gpio_spi_sck);
    hal_gpio_init(&gpio_spi_miso);
    hal_gpio_init(&gpio_spi_mosi);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI2
 */
static void gpio_spi2_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_PUSH_PULL,
        .ospeed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 5,
    };

    struct gpio_handle gpio_spi_sck = {
        .instance = GPIOB,
        .pin = GPIO_PIN13,
    };
    struct gpio_handle gpio_spi_miso = {
        .instance = GPIOB,
        .pin = GPIO_PIN14,
    };
    struct gpio_handle gpio_spi_mosi = {
        .instance = GPIOB,
        .pin = GPIO_PIN15,
    };

    gpio_spi_sck.init = init;
    gpio_spi_miso.init = init;
    gpio_spi_mosi.init = init;

    hal_gpio_init(&gpio_spi_sck);
    hal_gpio_init(&gpio_spi_miso);
    hal_gpio_init(&gpio_spi_mosi);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI3
 */
static void gpio_spi3_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_PUSH_PULL,
        .ospeed = GPIO_VERY_HIGH_SPEED,
        .pupd = GPIO_NO_PULL,
        .af = 6,
    };

    struct gpio_handle gpio_spi_sck = {
        .instance = GPIOC,
        .pin = GPIO_PIN10,
    };
    struct gpio_handle gpio_spi_miso = {
        .instance = GPIOC,
        .pin = GPIO_PIN11,
    };
    struct gpio_handle gpio_spi_mosi = {
        .instance = GPIOC,
        .pin = GPIO_PIN12,
    };

    gpio_spi_sck.init = init;
    gpio_spi_miso.init = init;
    gpio_spi_mosi.init = init;

    hal_gpio_init(&gpio_spi_sck);
    hal_gpio_init(&gpio_spi_miso);
    hal_gpio_init(&gpio_spi_mosi);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO I2C
 */
static void gpio_i2c_init(void)
{
    gpio_i2c1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO I2C1
 */
static void gpio_i2c1_init(void)
{
    static const struct gpio_init init = {
        .mode = GPIO_AF,
        .otype = GPIO_OPEN_DRAIN,
        .ospeed = GPIO_MEDIUM_SPEED,
        .pupd = GPIO_PULL_UP,
        .af = 4,
    };

    struct gpio_handle gpio_i2c_scl = {
        .instance = GPIOB,
        .pin = GPIO_PIN6,
    };
    struct gpio_handle gpio_i2c_sda = {
        .instance = GPIOB,
        .pin = GPIO_PIN7,
    };

    gpio_i2c_scl.init = init;
    gpio_i2c_sda.init = init;

    hal_gpio_init(&gpio_i2c_scl);
    hal_gpio_init(&gpio_i2c_sda);
}
/* ------------------------------------------------------------------------- */
