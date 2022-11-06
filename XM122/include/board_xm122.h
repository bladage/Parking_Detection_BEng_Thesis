// Copyright (c) Acconeer AB, 2019-2021
// All rights reserved
#ifndef BOARD_XM122_H_
#define BOARD_XM122_H_

#include "nrf_gpio.h"

#if defined(BOARD_PCA10056)
#define UART_TX                   NRF_GPIO_PIN_MAP(0, 6)
#define UART_RX                   NRF_GPIO_PIN_MAP(0, 8)
#define UART_RTS                  NRF_GPIO_PIN_MAP(0, 5)
#define UART_CTS                  NRF_GPIO_PIN_MAP(0, 7)
#define A111_SENSOR_INTERRUPT_Pin NRF_GPIO_PIN_MAP(1, 4)
#define A111_SPI_MISO_Pin         NRF_GPIO_PIN_MAP(1, 2)
#define VIN_ADC_EN_Pin            NRF_GPIO_PIN_MAP(0, 11)
#define MS_IRQ_Pin                NRF_GPIO_PIN_MAP(0, 28)
#else
#define UART_TX                   NRF_GPIO_PIN_MAP(0, 16)
#define UART_RX                   NRF_GPIO_PIN_MAP(0, 6)
#define UART_RTS                  NRF_GPIO_PIN_MAP(0, 20)
#define UART_CTS                  NRF_GPIO_PIN_MAP(0, 19)
#define A111_SENSOR_INTERRUPT_Pin NRF_GPIO_PIN_MAP(0, 8)
#define A111_SPI_MISO_Pin         NRF_GPIO_PIN_MAP(0, 5)
#define VIN_ADC_EN_Pin            NRF_GPIO_PIN_MAP(0, 17)
#define MS_IRQ_Pin                NRF_GPIO_PIN_MAP(0, 24)
#define MS_WAKE_UP_Pin            NRF_GPIO_PIN_MAP(0, 22)
#endif

#define A111_ENABLE_Pin    NRF_GPIO_PIN_MAP(0, 15)
#define A111_PS_ENABLE_Pin NRF_GPIO_PIN_MAP(0, 14)
#define A111_CTRL_Pin      NRF_GPIO_PIN_MAP(0, 13)
#define A111_SPI_CLK_Pin   NRF_GPIO_PIN_MAP(0, 27)
#define A111_SPI_MOSI_Pin  NRF_GPIO_PIN_MAP(1, 8)
#define A111_SPI_CS_N_Pin  NRF_GPIO_PIN_MAP(0, 26)
#define VIN_ADC_Pin        NRF_GPIO_PIN_MAP(0, 31)

// TWI definitions (pins / address) for XM122
#define XM122_I2C_DEVICE_ID 0x52
#define TWI_DATA_Pin        NRF_GPIO_PIN_MAP(0, 21)
#define TWI_CLK_Pin         NRF_GPIO_PIN_MAP(0, 23)

// LED definitions for XM122

#define LEDS_NUMBER    2

#define LED_1          NRF_GPIO_PIN_MAP(1, 11)  // D2 on the XM122 board
#define LED_2          NRF_GPIO_PIN_MAP(0, 4)   // D2 on the XB122 board

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1, LED_2 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

// Button definitions for XM122

#define BUTTONS_NUMBER 1

#define BUTTON_1       NRF_GPIO_PIN_MAP(0, 25)  // SW2 on the XB122 board (DFU button)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

// UART pin definitions for XM122

#define TX_PIN_NUMBER  UART_TX
#define RX_PIN_NUMBER  UART_RX
#define RTS_PIN_NUMBER UART_RTS
#define CTS_PIN_NUMBER UART_CTS

#endif // BOARD_XM122_H_
