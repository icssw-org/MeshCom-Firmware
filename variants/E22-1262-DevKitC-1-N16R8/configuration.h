/*
definitions for E22 + ESP32-S3_DevKitC-1_N16R8 Board
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// E22 specifig config
#define MODUL_HARDWARE ESP32_S3_EBYTE_E22
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz
#define ENABLE_GPS
#define ENABLE_BMX280
#define ENABLE_BMP390
#define ENABLE_AHT20
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_INA226
#define ENABLE_MC811
#define ENABLE_RTC
#define ENABLE_SOFTSER

#define SX126x_V3
#define SX1262_E22  // E22 900M30S/900M33S

#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 

#define TX_POWER_MAX 22  // max 22 dBm
#define TX_POWER_MIN 2
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define TX_OUTPUT_POWER 22

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
// Custom Board homemade E22-Ebyte Module + ESP32-S3 DevKitC-1-N16R8V
// ===== GPIOs =====
#define ANALOG_PIN 1
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

#define BATTERY_PIN 2 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_MULTIPLIER (10000 + 2200) / 2200    // default and can be overwritten with Flash variable node_analog_batt_faktor 

#define BUTTON_PIN  0
#define BUTTON_EXT  14  // TODO

#define LED_PIXEL 1     // NEOPIXEL
#define LED_PIN 48      // NEOPIXEL
#define BOARD_LED 38    // LED_BUILTIN

#define OneWire_GPIO 47

#define GPS_TX_PIN  15
#define GPS_RX_PIN  16

// I2C GPIOs
#define I2C_SDA  8
#define I2C_SCL  9

// SPI GPIOs
#define SCK 12
#define MISO 13
#define MOSI 11
#define SS 10

// E22 Module
#define E22_RXEN 4
#define E22_TXEN 5
#define E22_DIO1 6
#define E22_BUSY 7
#define E22_NRST 17
#define E22_SCK SCK
#define E22_NSS SS
#define LORA_RST E22_NRST // do be compatible with other config.h  

// SX126x_V3 alternative definitions - duplicates 433MHz/868MHz
#define SX126x_CS E22_NSS
#define SX126x_IRQ E22_DIO1
#define SX126x_RST E22_NRST
#define SX126x_GPIO E22_BUSY