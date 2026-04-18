/*
definitions for RAK4631
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// RAK4631 specific config

#define MODUL_HARDWARE T_ECHO
#define RF_FREQUENCY 433175000 // 432900000   // Hz
#define LORA_APRS_FREQUENCY 433775000 // 432900000   // Hz
#define TX_POWER_MAX 22  // max 22 dBm
#define TX_POWER_MIN -9

#define ENABLE_RTC

#define ENABLE_BMX280

//#define ENABLE_BMP390
//#define ENABLE_AHT20
//#define ENABLE_BMX680
//#define ENABLE_INA226
//#define ENABLE_MC811
//#define ENABLE_MCP23017

// #define ENABLE_SOFTSER

#define BUTTON_PIN 42

#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

#define USE_T_ECHO

// Define RAK LoRa parameters
#define TX_OUTPUT_POWER 22       // dBm

#define LORA_APRS_BANDWIDTH 0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_APRS_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_APRS_CODINGRATE 1        // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_APRS_PREAMBLE_LENGTH 8  // Same for Tx and Rx

#define LORA_BANDWIDTH 1         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SF 11 // [SF7..SF12]
#define LORA_CR 2        // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

#define LORA_SYMBOL_TIMEOUT 0    // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 0      // continous rx with 0
#define TX_TIMEOUT_VALUE 3000

// ETH Sield
//#define ETH_CS WB_IO5                   // Resoldered CS Pin to WB_IO5
#define ETH_CS SS                     // use this to try with pin 26 CS
#define MAX_DEVICE_ID 0xfFfFfFfFfFfF    // maximum mac address used to mask uint64_t from HW register

#define VEXT_CTRL   34  // To turn on GPS and TFT
#define ADC_CTRL    2   // ADC_CTRL = HIGH

#define RST_GPS     37
#define PIN_VEXT_CTL      34
#define VEXT_ENABLE       1

#define PIN_SPI1_MISO         (23)
#define PIN_SPI1_MOSI         (22)
#define PIN_SPI1_SCK          (19)

// LORA Chip
#define LORA_NRSET 25

#define HAS_EPAPER

#define ENABLE_GPS
#define GPS_RX_PIN 41
#define GPS_TX_PIN 40

// bei T114 bitte auf SOFTCHECK lassen .. Interrupt routine bei NRF52 nicht ok
#define GPS_BAUDRATE_SOFTCHECK        // GPS Baudratenermittlung wird mit Software Loop geprüft
//#define ENABLE_GPS_UBLOX_FIX          // UBLOX wird fix festgelegt und kein setup gemacht
//#define GPS_BAUDRATE_SETFIX 38400     // Die Baudrate für GPS wird auf FIXWERT gesetzt
