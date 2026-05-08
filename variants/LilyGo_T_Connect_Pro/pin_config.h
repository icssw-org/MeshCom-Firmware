/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2023-06-05 13:01:59
 * @LastEditTime: 2025-10-22 10:35:08
 */
#pragma once

#define T_Connect_Pro_V1_0
// #define T_Connect_Pro_V1_0_No_Screen
// #define T_Connect_Pro_V1_0_No_Lora_Add_Sd_Rtc

#define T_Connect_Pro_V1_0_SX1262
// #define T_Connect_Pro_V1_0_SX1276 // not use int pin
// #define T_Connect_Pro_V1_1_SX1276 // use int pin

#ifdef T_Connect_Pro_V1_0

#define IIC_SDA 39
#define IIC_SCL 40

// ST7796
#define SCREEN_WIDTH 272//222
#define SCREEN_HEIGHT 480
#define SCREEN_BL 46
#define SCREEN_MOSI 11
#define SCREEN_MISO 13
#define SCREEN_SCLK 12
#define SCREEN_CS 21
#define SCREEN_DC 41
#define SCREEN_RST -1

// CST226SE
#define TOUCH_SDA IIC_SDA
#define TOUCH_SCL IIC_SCL
#define TOUCH_RST 47
#define TOUCH_INT 3

// SX1262
#define SX1262_CS 14
#define SX1262_RST 42
#define SX1262_SCLK 12
#define SX1262_MOSI 11
#define SX1262_MISO 13
#define SX1262_BUSY 38
#define SX1262_INT 45
#define SX1262_DIO1 45

// SX1276
#define SX1276_CS 14
#define SX1276_RST 42
#define SX1276_SCLK 12
#define SX1276_MOSI 11
#define SX1276_MISO 13
#define SX1276_BUSY 38
#define SX1276_INT 45
#define SX1276_DIO1 45

#elif defined T_Connect_Pro_V1_0_No_Screen

#define WS2812_DATA_1 46
#define WS2812_DATA_2 3
#define WS2812_DATA_3 21
#define WS2812_DATA_4 41

#define KEY_A 39
#define KEY_B 40

// SX1262
#define SX1262_CS 14
#define SX1262_RST 42
#define SX1262_SCLK 12
#define SX1262_MOSI 11
#define SX1262_MISO 13
#define SX1262_BUSY 38
#define SX1262_INT 45
#define SX1262_DIO1 45

// SX1276
#define SX1276_CS 14
#define SX1276_RST 42
#define SX1276_SCLK 12
#define SX1276_MOSI 11
#define SX1276_MISO 13
#define SX1276_BUSY 38
#define SX1276_INT 45
#define SX1276_DIO1 45

#elif defined T_Connect_Pro_V1_0_No_Lora_Add_Sd_Rtc

#define IIC_SDA 39
#define IIC_SCL 40

#define IIC_SDA_2 42
#define IIC_SCL_2 38

#define SPI_MOSI 11
#define SPI_MISO 13
#define SPI_SCLK 12

// ST7796
#define SCREEN_WIDTH 222
#define SCREEN_HEIGHT 480
#define SCREEN_BL 46
#define SCREEN_MOSI SPI_MOSI
#define SCREEN_MISO SPI_MISO
#define SCREEN_SCLK SPI_SCLK
#define SCREEN_CS 21
#define SCREEN_DC 41
#define SCREEN_RST -1

// CST226SE
#define TOUCH_SDA IIC_SDA
#define TOUCH_SCL IIC_SCL
#define TOUCH_RST 47
#define TOUCH_INT 3

// SD
#define SD_MOSI SPI_MOSI
#define SD_MISO SPI_MISO
#define SD_SCLK SPI_SCLK
#define SD_CS 45

// PCF8563
#define PCF8563_SDA IIC_SDA_2
#define PCF8563_SCL IIC_SCL_2
#define PCF8563_INT 14

#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

// Relay
#define RELAY_1 8

// Ethernet
#define W5500_SCLK 12
#define W5500_MISO 13
#define W5500_MOSI 11
#define W5500_CS 10
#define W5500_RST 48
#define W5500_INT 9

// RS485
#define RS485_TX_1 4
#define RS485_RX_1 5
#define RS485_TX_2 17
#define RS485_RX_2 18

// RS232
#define RS232_TX_1 4
#define RS232_RX_1 5

// CAN
#define CAN_TX 6
#define CAN_RX 7

// ESPBOOT
#define ESP_BOOT 0