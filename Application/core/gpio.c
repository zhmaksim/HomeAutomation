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
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Msk);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Msk);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Msk);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Msk);

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
    /* GPIOA8 RCC_MCO1 */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOA->MODER,
               GPIO_MODER_MODE8_Msk,
               0x02 << GPIO_MODER_MODE8_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT8_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8_Msk);

    /* Настроить тип подтяжки сигнала = NoPull */
    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD8_Msk);

    /* Настроить номер альтернативной функции = 0 */
    CLEAR_BIT(GPIOA->AFR[1], GPIO_AFRH_AFSEL8_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO LED
 */
static void gpio_led_init(void)
{
    /*
     * GPIOB0 LED_ST
     * GPIOB4 LED_TX
     * GPIOB5 LED_RX
     */

    /* Установить начальный уровень = Low */
    CLEAR_BIT(GPIOB->ODR,
              GPIO_ODR_OD0_Msk
            | GPIO_ODR_OD4_Msk
            | GPIO_ODR_OD5_Msk);

    /* Настроить режим работы = Output */
    MODIFY_REG(GPIOB->MODER,
               GPIO_MODER_MODE0_Msk
             | GPIO_MODER_MODE4_Msk
             | GPIO_MODER_MODE5_Msk,
               0x01 << GPIO_MODER_MODE0_Pos
             | 0x01 << GPIO_MODER_MODE4_Pos
             | 0x01 << GPIO_MODER_MODE5_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOB->OTYPER,
              GPIO_OTYPER_OT0_Msk
            | GPIO_OTYPER_OT4_Msk
            | GPIO_OTYPER_OT5_Msk);

    /* Настроить скорость работы вывода = Low Speed */
    CLEAR_BIT(GPIOB->OSPEEDR,
              GPIO_OSPEEDR_OSPEED0_Msk
            | GPIO_OSPEEDR_OSPEED4_Msk
            | GPIO_OSPEEDR_OSPEED5_Msk);

    /* Настроить тип подтяжки сигнала = PullUp */
    MODIFY_REG(GPIOB->PUPDR,
               GPIO_PUPDR_PUPD0_Msk
             | GPIO_PUPDR_PUPD4_Msk
             | GPIO_PUPDR_PUPD5_Msk,
               0x01 << GPIO_PUPDR_PUPD0_Pos
             | 0x01 << GPIO_PUPDR_PUPD4_Pos
             | 0x01 << GPIO_PUPDR_PUPD5_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO 74HC
 */
static void gpio_74hc_init(void)
{
    /*
     * GPIOB2 74HC595_EN
     * GPIOB10 74HC595_CS
     * GPIOB12 74HC165_CS
     */

    /*
     * Установить начальный уровень:
     * 74HC595_EN = High
     * 74HC595_CS = High
     * 74HC165_CS = Low
     */
    MODIFY_REG(GPIOB->ODR,
               GPIO_ODR_OD12_Msk,
               GPIO_ODR_OD2_Msk
             | GPIO_ODR_OD10_Msk);

    /* Настроить режим работы = Output */
    MODIFY_REG(GPIOB->MODER,
               GPIO_MODER_MODE2_Msk
             | GPIO_MODER_MODE10_Msk
             | GPIO_MODER_MODE12_Msk,
               0x01 << GPIO_MODER_MODE2_Pos
             | 0x01 << GPIO_MODER_MODE10_Pos
             | 0x01 << GPIO_MODER_MODE12_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOB->OTYPER,
              GPIO_OTYPER_OT2_Msk
            | GPIO_OTYPER_OT10_Msk
            | GPIO_OTYPER_OT12_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOB->OSPEEDR,
            GPIO_OSPEEDR_OSPEED2_Msk
          | GPIO_OSPEEDR_OSPEED10_Msk
          | GPIO_OSPEEDR_OSPEED12_Msk);

    /* Настроить тип подтяжки сигнала = PullUp */
    MODIFY_REG(GPIOB->PUPDR,
               GPIO_PUPDR_PUPD2_Msk
             | GPIO_PUPDR_PUPD10_Msk
             | GPIO_PUPDR_PUPD12_Msk,
               0x01 << GPIO_PUPDR_PUPD2_Pos
             | 0x01 << GPIO_PUPDR_PUPD10_Pos
             | 0x01 << GPIO_PUPDR_PUPD12_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO RS485
 */
static void gpio_rs485_init(void)
{
    /*
     * GPIOA11 RS485_USART1
     * GPIOA1 RS485_USART2
     */

    /* Установить начальный уровень = Low */
    CLEAR_BIT(GPIOA->ODR,
              GPIO_ODR_OD11_Msk
            | GPIO_ODR_OD1_Msk);

    /* Настроить режим работы = Output */
    MODIFY_REG(GPIOA->MODER,
               GPIO_MODER_MODE11_Msk
             | GPIO_MODER_MODE1_Msk,
               0x01 << GPIO_MODER_MODE11_Pos
             | 0x01 << GPIO_MODER_MODE1_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOA->OTYPER,
              GPIO_OTYPER_OT11_Msk
            | GPIO_OTYPER_OT1_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOA->OSPEEDR,
            GPIO_OSPEEDR_OSPEED11_Msk
          | GPIO_OSPEEDR_OSPEED1_Msk);

    /* Настроить тип подтяжки сигнала = PullDown */
    MODIFY_REG(GPIOA->PUPDR,
               GPIO_PUPDR_PUPD11_Msk
             | GPIO_PUPDR_PUPD1_Msk,
               0x02 << GPIO_PUPDR_PUPD11_Pos
             | 0x02 << GPIO_PUPDR_PUPD1_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO W25Q
 */
static void gpio_w25q_init(void)
{
    /* GPIOA4 W25Q_CS */

    /* Установить начальный уровень = High */
    SET_BIT(GPIOA->ODR, GPIO_ODR_OD4_Msk);

    /* Настроить режим работы = Output */
    MODIFY_REG(GPIOA->MODER,
               GPIO_MODER_MODE4_Msk,
               0x01 << GPIO_MODER_MODE4_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT4_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED4_Msk);

    /* Настроить тип подтяжки сигнала = PullUp */
    MODIFY_REG(GPIOA->PUPDR,
               GPIO_PUPDR_PUPD4_Msk,
               0x01 << GPIO_PUPDR_PUPD4_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO W5500
 */
static void gpio_w5500_init(void)
{
    /*
     * GPIOB3 W5500_RESET
     * GPIOA15 W5500_CS
     * GPIOD2 W5500_INT
     */

    /* Установить начальный уровень = High */
    SET_BIT(GPIOB->ODR, GPIO_ODR_OD3_Msk);

    SET_BIT(GPIOA->ODR, GPIO_ODR_OD15_Msk);

    /* Настроить режим работы = Output */
    MODIFY_REG(GPIOB->MODER,
               GPIO_MODER_MODE3_Msk,
               0x01 << GPIO_MODER_MODE3_Pos);

    MODIFY_REG(GPIOA->MODER,
               GPIO_MODER_MODE15_Msk,
               0x01 << GPIO_MODER_MODE15_Pos);

    /* Настроить режим работы = Input */
    CLEAR_BIT(GPIOD->MODER, GPIO_MODER_MODE2_Msk);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT3_Msk);

    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT15_Msk);

    CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT2_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED3_Msk);

    SET_BIT(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED15_Msk);

    SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED2_Msk);

    /* Настроить тип подтяжки сигнала = PullUp */
    MODIFY_REG(GPIOB->PUPDR,
               GPIO_PUPDR_PUPD3_Msk,
               0x01 << GPIO_PUPDR_PUPD3_Pos);

    MODIFY_REG(GPIOA->PUPDR,
               GPIO_PUPDR_PUPD15_Msk,
               0x01 << GPIO_PUPDR_PUPD15_Pos);

    MODIFY_REG(GPIOD->PUPDR,
               GPIO_PUPDR_PUPD2_Msk,
               0x01 << GPIO_PUPDR_PUPD2_Pos);
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
    /*
     * GPIOA9 USART1_TX
     * GPIOA10 USART1_RX
     */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOA->MODER,
               GPIO_MODER_MODE9_Msk
             | GPIO_MODER_MODE10_Msk,
               0x02 << GPIO_MODER_MODE9_Pos
             | 0x02 << GPIO_MODER_MODE10_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOA->OTYPER,
              GPIO_OTYPER_OT9_Msk
            | GPIO_OTYPER_OT10_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOA->OSPEEDR,
            GPIO_OSPEEDR_OSPEED9_Msk
          | GPIO_OSPEEDR_OSPEED10_Msk);

    /* Настроить тип подтяжки сигнала = NoPull */
    CLEAR_BIT(GPIOA->PUPDR,
              GPIO_PUPDR_PUPD9_Msk
            | GPIO_PUPDR_PUPD10_Msk);

    /* Настроить номер альтернативной функции = 7 */
    MODIFY_REG(GPIOA->AFR[1],
               GPIO_AFRH_AFSEL9_Msk
             | GPIO_AFRH_AFSEL10_Msk,
               7 << GPIO_AFRH_AFSEL9_Pos
             | 7 << GPIO_AFRH_AFSEL10_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO USART2
 */
static void gpio_usart2_init(void)
{
    /*
     * GPIOA2 USART2_TX
     * GPIOA3 USART2_RX
     */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOA->MODER,
               GPIO_MODER_MODE2_Msk
             | GPIO_MODER_MODE3_Msk,
               0x02 << GPIO_MODER_MODE2_Pos
             | 0x02 << GPIO_MODER_MODE3_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOA->OTYPER,
              GPIO_OTYPER_OT2_Msk
            | GPIO_OTYPER_OT3_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOA->OSPEEDR,
            GPIO_OSPEEDR_OSPEED2_Msk
          | GPIO_OSPEEDR_OSPEED3_Msk);

    /* Настроить тип подтяжки сигнала = NoPull */
    CLEAR_BIT(GPIOA->PUPDR,
              GPIO_PUPDR_PUPD2_Msk
            | GPIO_PUPDR_PUPD3_Msk);

    /* Настроить номер альтернативной функции = 7 */
    MODIFY_REG(GPIOA->AFR[0],
               GPIO_AFRL_AFSEL2_Msk
             | GPIO_AFRL_AFSEL3_Msk,
               7 << GPIO_AFRL_AFSEL2_Pos
             | 7 << GPIO_AFRL_AFSEL3_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO USART6
 */
static void gpio_usart6_init(void)
{
    /*
     * GPIOC6 USART6_TX
     * GPIOC7 USART6_RX
     */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOC->MODER,
               GPIO_MODER_MODE6_Msk
             | GPIO_MODER_MODE7_Msk,
               0x02 << GPIO_MODER_MODE6_Pos
             | 0x02 << GPIO_MODER_MODE7_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOC->OTYPER,
              GPIO_OTYPER_OT6_Msk
            | GPIO_OTYPER_OT7_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOC->OSPEEDR,
            GPIO_OSPEEDR_OSPEED6_Msk
          | GPIO_OSPEEDR_OSPEED7_Msk);

    /* Настроить тип подтяжки сигнала = NoPull */
    CLEAR_BIT(GPIOC->PUPDR,
              GPIO_PUPDR_PUPD6_Msk
            | GPIO_PUPDR_PUPD7_Msk);

    /* Настроить номер альтернативной функции = 8 */
    MODIFY_REG(GPIOC->AFR[0],
               GPIO_AFRL_AFSEL6_Msk
             | GPIO_AFRL_AFSEL7_Msk,
               8 << GPIO_AFRL_AFSEL6_Pos
             | 8 << GPIO_AFRL_AFSEL7_Pos);
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
    /*
     * GPIOA5 SPI1_SCK
     * GPIOA6 SPI1_MISO
     * GPIOA7 SPI1_MOSI
     */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOA->MODER,
               GPIO_MODER_MODE5_Msk
             | GPIO_MODER_MODE6_Msk
             | GPIO_MODER_MODE7_Msk,
               0x02 << GPIO_MODER_MODE5_Pos
             | 0x02 << GPIO_MODER_MODE6_Pos
             | 0x02 << GPIO_MODER_MODE7_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOA->OTYPER,
              GPIO_OTYPER_OT5_Msk
            | GPIO_OTYPER_OT6_Msk
            | GPIO_OTYPER_OT7_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOA->OSPEEDR,
            GPIO_OSPEEDR_OSPEED5_Msk
          | GPIO_OSPEEDR_OSPEED6_Msk
          | GPIO_OSPEEDR_OSPEED7_Msk);

    /* Настроить тип подтяжки сигнала = NoPull */
    CLEAR_BIT(GPIOA->PUPDR,
              GPIO_PUPDR_PUPD5_Msk
            | GPIO_PUPDR_PUPD6_Msk
            | GPIO_PUPDR_PUPD7_Msk);

    /* Настроить номер альтернативной функции = 5 */
    MODIFY_REG(GPIOA->AFR[0],
               GPIO_AFRL_AFSEL5_Msk
             | GPIO_AFRL_AFSEL6_Msk
             | GPIO_AFRL_AFSEL7_Msk,
               5 << GPIO_AFRL_AFSEL5_Pos
             | 5 << GPIO_AFRL_AFSEL6_Pos
             | 5 << GPIO_AFRL_AFSEL7_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI2
 */
static void gpio_spi2_init(void)
{
    /*
     * GPIOB13 SPI2_SCK
     * GPIOB14 SPI2_MISO
     * GPIOB15 SPI2_MOSI
     */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOB->MODER,
               GPIO_MODER_MODE13_Msk
             | GPIO_MODER_MODE14_Msk
             | GPIO_MODER_MODE15_Msk,
               0x02 << GPIO_MODER_MODE13_Pos
             | 0x02 << GPIO_MODER_MODE14_Pos
             | 0x02 << GPIO_MODER_MODE15_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOB->OTYPER,
              GPIO_OTYPER_OT13_Msk
            | GPIO_OTYPER_OT14_Msk
            | GPIO_OTYPER_OT15_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOB->OSPEEDR,
            GPIO_OSPEEDR_OSPEED13_Msk
          | GPIO_OSPEEDR_OSPEED14_Msk
          | GPIO_OSPEEDR_OSPEED15_Msk);

    /* Настроить тип подтяжки сигнала = NoPull */
    CLEAR_BIT(GPIOB->PUPDR,
              GPIO_PUPDR_PUPD13_Msk
            | GPIO_PUPDR_PUPD14_Msk
            | GPIO_PUPDR_PUPD15_Msk);

    /* Настроить номер альтернативной функции = 5 */
    MODIFY_REG(GPIOB->AFR[1],
               GPIO_AFRH_AFSEL13_Msk
             | GPIO_AFRH_AFSEL14_Msk
             | GPIO_AFRH_AFSEL15_Msk,
               5 << GPIO_AFRH_AFSEL13_Pos
             | 5 << GPIO_AFRH_AFSEL14_Pos
             | 5 << GPIO_AFRH_AFSEL15_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief           Инициализировать GPIO SPI3
 */
static void gpio_spi3_init(void)
{
    /*
     * GPIOC10 SPI3_SCK
     * GPIOC11 SPI3_MISO
     * GPIOC12 SPI3_MOSI
     */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOC->MODER,
               GPIO_MODER_MODE10_Msk
             | GPIO_MODER_MODE11_Msk
             | GPIO_MODER_MODE12_Msk,
               0x02 << GPIO_MODER_MODE10_Pos
             | 0x02 << GPIO_MODER_MODE11_Pos
             | 0x02 << GPIO_MODER_MODE12_Pos);

    /* Настроить тип вывода = Push-Pull */
    CLEAR_BIT(GPIOC->OTYPER,
              GPIO_OTYPER_OT10_Msk
            | GPIO_OTYPER_OT11_Msk
            | GPIO_OTYPER_OT12_Msk);

    /* Настроить скорость работы вывода = Very High Speed */
    SET_BIT(GPIOC->OSPEEDR,
            GPIO_OSPEEDR_OSPEED10_Msk
          | GPIO_OSPEEDR_OSPEED11_Msk
          | GPIO_OSPEEDR_OSPEED12_Msk);

    /* Настроить тип подтяжки сигнала = NoPull */
    CLEAR_BIT(GPIOC->PUPDR,
              GPIO_PUPDR_PUPD10_Msk
            | GPIO_PUPDR_PUPD11_Msk
            | GPIO_PUPDR_PUPD12_Msk);

    /* Настроить номер альтернативной функции = 6 */
    MODIFY_REG(GPIOC->AFR[1],
               GPIO_AFRH_AFSEL10_Msk
             | GPIO_AFRH_AFSEL11_Msk
             | GPIO_AFRH_AFSEL12_Msk,
               6 << GPIO_AFRH_AFSEL10_Pos
             | 6 << GPIO_AFRH_AFSEL11_Pos
             | 6 << GPIO_AFRH_AFSEL12_Pos);
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
    /*
     * GPIOB6 I2C1_SCL
     * GPIOB7 I2C1_SDA
     */

    /* Настроить режим работы = AF */
    MODIFY_REG(GPIOB->MODER,
               GPIO_MODER_MODE6_Msk
             | GPIO_MODER_MODE7_Msk,
               0x02 << GPIO_MODER_MODE6_Pos
             | 0x02 << GPIO_MODER_MODE7_Pos);

    /* Настроить тип вывода = Open-Drain */
    SET_BIT(GPIOB->OTYPER,
            GPIO_OTYPER_OT6_Msk
          | GPIO_OTYPER_OT7_Msk);

    /* Настроить скорость работы вывода = Medium Speed */
    MODIFY_REG(GPIOB->OSPEEDR,
               GPIO_OSPEEDR_OSPEED6_Msk
             | GPIO_OSPEEDR_OSPEED7_Msk,
               0x01 << GPIO_OSPEEDR_OSPEED6_Pos
             | 0x01 << GPIO_OSPEEDR_OSPEED7_Pos);

    /* Настроить тип подтяжки сигнала = PullUp */
    MODIFY_REG(GPIOB->PUPDR,
               GPIO_PUPDR_PUPD6_Msk
             | GPIO_PUPDR_PUPD7_Msk,
               0x01 << GPIO_PUPDR_PUPD6_Pos
             | 0x01 << GPIO_PUPDR_PUPD7_Pos);

    /* Настроить номер альтернативной функции = 4 */
    MODIFY_REG(GPIOB->AFR[0],
               GPIO_AFRL_AFSEL6_Msk
             | GPIO_AFRL_AFSEL7_Msk,
               4 << GPIO_AFRL_AFSEL6_Pos
             | 4 << GPIO_AFRL_AFSEL7_Pos);
}
/* ------------------------------------------------------------------------- */
