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

// GPS
#define ENABLE_GPS
#define GPS_RESET_MODE 1

// Radio
#define SX126x_V3
#define SX1262_E22

#define CURRENT_LIMIT 140
#define TX_POWER_MAX 22
#define TX_POWER_MIN 2
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

#define GPS_TX_PIN 42
#define GPS_RX_PIN 39

// I2C
#define I2C_SDA 17
#define I2C_SCL 18

// SPI radio
#define SCK 10
#define MISO 9
#define MOSI 11
#define SS 40

// =========================
// Ethernet W5500 (T-ETH-ELITE)
#define ETH_CS_PIN    45
#define ETH_INT_PIN   14
#define ETH_RST_PIN   16
#define ETH_SCLK_PIN  48
#define ETH_MISO_PIN  47
#define ETH_MOSI_PIN  21

// E22 compatibility (required by MeshCom)
//#define E22_RXEN -1
//#define E22_TXEN -1

// SX126x mapping (required by MeshCom)
#define SX126x_IRQ   8      // ex E22_DIO1
#define SX126x_GPIO  16     // ex E22_BUSY
#define SX126x_RST   46
#define SX126x_CS    SS
#define LORA_RST     SX126x_RST

