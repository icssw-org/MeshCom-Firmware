/*
definitions for RAK4631
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// RAK4631 specific config

#define MODUL_HARDWARE HELTEC_T114
#define RF_FREQUENCY 433175000 // 432900000   // Hz
#define LORA_APRS_FREQUENCY 433775000 // 432900000   // Hz

#define ENABLE_RTC_DISABLED
#define ENABLE_BMX280_DISABLED
#define ENABLE_BMP390_DISABLED
#define ENABLE_AHT20_DISABLED
#define ENABLE_BMX680_DISABLED
#define ENABLE_INA226_DISABLED
#define ENABLE_MC811_DISABLED
#define ENABLE_MCP23017_DISABLED
#define ENABLE_SHT21_DISABLED

// #define ENABLE_SOFTSER

#define BUTTON_PIN 42

#define USE_HELTEC_T114

// Define RAK LoRa parameters

#define LORA_APRS_BANDWIDTH 0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_APRS_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_APRS_CODINGRATE 1        // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_APRS_PREAMBLE_LENGTH 8  // Same for Tx and Rx

#define LORA_BANDWIDTH 1         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_CR 2        // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]

#define LORA_SYMBOL_TIMEOUT 0    // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 0      // continous rx with 0
#define TX_TIMEOUT_VALUE 3000

// ETH Sield
//#define ETH_CS WB_IO5                   // Resoldered CS Pin to WB_IO5
#define ETH_CS SS                     // use this to try with pin 26 CS
#define MAX_DEVICE_ID 0xfFfFfFfFfFfF    // maximum mac address used to mask uint64_t from HW register

#define VEXT_CTRL   3   // To turn on GPS and TFT
#define ADC_CTRL    2   // ADC_CTRL = HIGH

#define RST_GPS     38
#define PIN_VEXT_CTL      21
#define VEXT_ENABLE       1

#define PIN_SPI1_MISO         (43)
#define PIN_SPI1_MOSI         (41)
#define PIN_SPI1_SCK          (40)

// TFT Display
#define HAS_TFT_114

#define PIN_TFT_CS        11
#define PIN_TFT_RST       2 // Or set to -1 and connect to Arduino RESET pin
#define PIN_TFT_DC        12

#define PIN_TFT_VDD_CTL      3
#define TFT_VDD_ENABLE       0
#define PIN_TFT_LEDA_CTL     15
#define TFT_LEDA_ENABLE      0

// LORA Chip
#define LORA_NRSET 25

#define GPS_RX_PIN 37
#define GPS_TX_PIN 39

// bei T114 bitte auf SOFTCHECK lassen .. Interrupt routine bei NRF52 nicht ok
#define GPS_BAUDRATE_SOFTCHECK        // GPS Baudratenermittlung wird mit Software Loop geprüft
//#define ENABLE_GPS_UBLOX_FIX          // UBLOX wird fix festgelegt und kein setup gemacht
//#define GPS_BAUDRATE_SETFIX 38400     // Die Baudrate für GPS wird auf FIXWERT gesetzt

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
