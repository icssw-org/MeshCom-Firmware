/**
 * definitions for T-Deck Board
 */

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// T-Deck specifig config
#define MODUL_HARDWARE T_DECK_PLUS
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz
#define SX1262X  // some functions differ from SX127x and SX126x in RadioLib based on Semtech Chip
#define ENABLE_GPS
#define GPS_L76K
//#define ENABLE_BMX280
//#define ENABLE_BMX680
//#define ENABLE_MCP23017
//#define ENABLE_INA226
//#define ENABLE_MC811
//#define ENABLE_RTC
#define ENABLE_SOFTSER
#define ENABLE_AUDIO

#define TX_POWER_MAX 22  // max 22 dBm
#define TX_POWER_MIN 2

#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

#define TDECK_TFT_TIMEOUT 30  // time until display turns dark in seconds

#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()
#define TX_OUTPUT_POWER 21  // SX1268 have up to +22dBm
#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 


/**
 * RadioLib Coding Rate: Allowed values range from 5 to 8.
 * case 5: CR_4_5;
    case 6: CR_4_6;
    case 7: CR_4_7;
    case 8: CR_4_8;
*/
#define LORA_CR 6

// RadioLib LoRa Bandwidth Setting in kHz
#define LORA_BANDWIDTH 250

/** RadioLib Spreading Factor
 * case 6: SF_6;
    case 7: SF_7;
    case 8: SF_8;
    case 9: SF_9;
    case 10: SF_10;
    case 11: SF_11;
    case 12: SF_12;
*/
#define LORA_SF 11


// OE3GJC: TODO check

//! The board peripheral power control pin needs to be set to HIGH when using the peripheral
#define TDECK_POWERON 10

#define TDECK_SDCARD_CS     39
#define TDECK_SPI_MOSI      41
#define TDECK_SPI_MISO      38
#define TDECK_SPI_SCK       40

#define TDECK_TFT_CS        12
#define TDECK_TFT_DC        11
#define TDECK_TFT_BACKLIGHT 42

#define GPS_RX_PIN        44
#define GPS_TX_PIN        43

#define LORA_RST  17
#define LORA_DIO0 13 // aka BUSY Pin
#define LORA_DIO1 45
#define LORA_DIO2 32 //?
#define LORA_CS 9
#define LED 2 // ?

// trackball
#define TDECK_BOOT_PIN      0
#define TDECK_TBOX_G02      2
#define TDECK_TBOX_G01      3
#define TDECK_TBOX_G04      1
#define TDECK_TBOX_G03      15

// touchscreen
#define TDECK_TOUCH_INT     16


#define I2C_SDA 18
#define I2C_SCL 8

// lora radio
#define SX1262X_CS LORA_CS
#define SX1262X_IRQ LORA_DIO1
#define SX1262X_RST LORA_RST
#define SX1262X_GPIO LORA_DIO0

#define OneWire_GPIO 25 // getestet OE5HWN

#define BATTERY_PIN       4
#define BUTTON_PIN TDECK_BOOT_PIN

#define I2S_BCLK            7
#define I2S_LRC             5
#define I2S_DOUT            6

// Audio input
#define TDECK_ES7210_MCLK   48
#define TDECK_ES7210_LRCK   21
#define TDECK_ES7210_SCK    47
#define TDECK_ES7210_DIN    14

#define BOARD_I2S_WS        I2S_LRC
#define BOARD_I2S_BCK       I2S_BCLK
#define BOARD_I2S_DOUT      I2S_DOUT

// T-Deck GUI configuration
#define MAX_MAP 5                          // max count of maps
#define MAX_POINTS 30                      // max count of points
#define MAX_POSROW 40                       // max numbers of rows in POS view

#define TDECK_KEYBOARD_INT  46
#define TDECK_BL_PIN        42
