#pragma once
/*
definitions for LilyGo T-BEAM 1W Board OE3WAS
*/

#include <Arduino.h>
#include <configuration_global.h>

//original LilyGo T-Beam-1W mit MesCom definitions ergänzt und tw. andere Namen

#define UNUSED_PIN                   (0)

#define BOARD_VARIANT_NAME          "LoRa 1W"
#define MODUL_HARDWARE TBEAM_1W
#define RADIO_TYPE_STR  "SX1262"
#ifndef USING_SX1262
#define USING_SX1262
#endif

#define I2C_SDA                     (8)
#define I2C_SCL                     (9)

#define HAS_GPS
#define GPS_SLEEP_HOLD_ON_LOW
#define GPS_BAUD_RATE               9600
#define GPS_RX_PIN                  (5)
#define GPS_TX_PIN                  (6)
#define GPS_PPS_PIN                 (7)
#define GPS_EN_PIN                  (16)

#define BOARD_LED                   18
#define LED_ON                      HIGH
#define LED_OFF                     LOW

#define BUTTON0_PIN                 (0)   /*BUTTON 0 = GPIO0 (BOOT)*/
#define BUTTON_PIN                  17    /*BUTTON 1 = GPIO17*/

// SPI GPIOs
#define SPI_MOSI                    (11)
#define SPI_SCK                     (13)
#define SPI_MISO                    (12)
#define SPI_CS                      (10)

#define SDCARD_CS                   SPI_CS
#define RADIO_SCLK_PIN              (SPI_SCK)
#define RADIO_MISO_PIN              (SPI_MISO)
#define RADIO_MOSI_PIN              (SPI_MOSI)

#define RADIO_CS_PIN                15
#define RADIO_RST_PIN               3
#define RADIO_LDO_EN                (40)
#define RADIO_CTRL                  (21)
#define RADIO_DIO1_PIN              (1)
#define RADIO_BUSY_PIN              (38)

// SX1262 alternative definitions - duplicates 433MHz/868MHz
#define SX1262_CS RADIO_CS_PIN
#define SX1262_CTL RADIO_CTRL
#define SX126x_BUSY RADIO_BUSY_PIN
#define SX1262_IRQ RADIO_DIO1_PIN
#define SX1262_RST RADIO_RST_PIN

#define NTC_PIN                     (14)
#define FAN_CTRL                    (41)

#define ADC_PIN                     4
#define BATTERY_PIN                 4 // A battery voltage measurement pin
// voltage divider connected here to measure battery voltage
#define BAT_ADC_PULLUP_RES          300000.0
#define BAT_ADC_PULLDOWN_RES        150000.0
#define BAT_MAX_VOLTAGE             7.4
#define BAT_VOL_COMPENSATION        0.25
#define S3_VREF                     (1.1)  // 1.0 - 1.2 V
#define ADC_MULTIPLIER (BAT_ADC_PULLUP_RES+BAT_ADC_PULLDOWN_RES)/BAT_ADC_PULLDOWN_RES

#define ANALOG_PIN ADC_PIN  //testweise
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

#define GPS_SLEEP_HOLD_ON_LOW
#define GPS_BAUD_RATE               9600
#define HAS_SDCARD
#define HAS_GPS
#define SDCARD_MOSI                 SPI_MOSI
#define SDCARD_MISO                 SPI_MISO
#define SDCARD_SCLK                 SPI_SCK
#define SDCARD_CS                   SPI_CS
#define SD_SHARE_SPI_BUS           // SD-CARD AND RADIO SHARE SPI BUS
#define __HAS_SPI1__

#define HAS_DISPLAY
#define DISPLAY_MODEL               U8G2_SH1106_128X64_NONAME_F_HW_I2C
#define DISPLAY_MODEL_SSD_LIB       SH1106Wire

#define __HAS_SPI1__

// end original LilyGo T-Beam-1W


#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz

#define BOARD_COUNTRY 8   // EU8

#define ENABLE_GPS
#define GPS_L76K
#define ENABLE_BMX280
#define ENABLE_BMP390
#define ENABLE_AHT20
#define ENABLE_SHT21
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_INA226
#define ENABLE_MC811
#define ENABLE_RTC
//#define ENABLE_SOFTSER


#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 
#define TX_POWER_MAX 22  // SX1262 max 22 dBm
#define TX_POWER_MIN 2
#define TX_OUTPUT_POWER 8

#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

//todo #define WAIT_TX x         // waiting after Lora TX in doTX() >800µs


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

// =============================================
// LilyGo T-BEAM 1W + ESP32-S3-WROOM-1-N16R8
// spezielle externe User GPIO noch nicht voll ausdefiniert
// ===== GPIOs =====
//#define OneWire_GPIO xxx
