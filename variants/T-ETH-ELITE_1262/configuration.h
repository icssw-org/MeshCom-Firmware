/*
definitions for LILYGO T-ETH-Elite S3 + SX1262 shield
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

#define HAS_ETHERNET

// Manteniamo compatibilità MeshCom
#define MODUL_HARDWARE T_ETH_ELITE_1262

// Frequenze
#define RF_FREQUENCY 433.175000
#define LORA_APRS_FREQUENCY 433.775000
#define BOARD_COUNTRY 8

// External Hardware
#define ENABLE_BMX280
//#define ENABLE_BMP390
//#define ENABLE_AHT20
//#define ENABLE_SHT21
//#define ENABLE_BMX680
//#define ENABLE_MCP23017
//#define ENABLE_INA226
//#define ENABLE_MC811
//#define ENABLE_RTC
//#define ENABLE_SOFTSER

#define CURRENT_LIMIT 140
#define TX_POWER_MAX 22
#define TX_POWER_MIN -9
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH
#define WAIT_TX 5
#define TX_OUTPUT_POWER 22
#define LORA_CR 6
#define LORA_BANDWIDTH 125
#define LORA_SF 11

// GPIO
#define ANALOG_PIN 7
#define ANALOG_REFRESH_INTERVAL 30

#define BATTERY_PIN 2
#define ADC_MULTIPLIER 1

#define BUTTON_PIN 0
#define BUTTON_EXT -1

#define LED_PIXEL 0
#define LED_PIN 38
#define BOARD_LED 1

#define OneWire_GPIO -1

// I2C
#define I2C_SDA 17
#define I2C_SCL 18

// =========================
// Ethernet W5500 (T-ETH-ELITE)
#define ETH_CS_PIN    45
#define ETH_INT_PIN   14
#define ETH_RST_PIN   16
#define ETH_SCLK_PIN  48
#define ETH_MISO_PIN  47
#define ETH_MOSI_PIN  21

// Radio
#define USING_SX1262

// SPI radio
#define RADIO_MISO    9
#define RADIO_MOSI    11
#define RADIO_SCK     10

#define RADIO_CS_PIN    40
#define RADIO_IRQ_PIN   8
#define RADIO_RST_PIN   46
#define RADIO_BUSY_PIN  16

// GPS
#define ENABLE_GPS
#define GPS_RESET_MODE 1
#define GPS_TX_PIN 42
#define GPS_RX_PIN 39
