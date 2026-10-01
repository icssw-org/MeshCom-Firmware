/*
definitions for RAK4631
*/


#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// RAK4631 specific config

#define MODUL_HARDWARE RAK4631
#define RF_FREQUENCY 433175000 // 432900000   // Hz
#define LORA_APRS_FREQUENCY 433775000 // 432900000   // Hz
#define TX_POWER_MIN 2
#define ENABLE_SHT21_DISABLED

// #define ENABLE_SOFTSER

#define OLED
#define USE_RAK4630

#define BUTTON_PIN WB_IO6       // only in combination with RAK13002
#define OneWire_GPIO WB_IO1     // only in combination with RAK13002

#define ENABLE_RAK_GPS
#define ENABLE_GPS_DISABLED   // this board uses ENABLE_RAK_GPS instead of ENABLE_GPS


#define LPS33     // Druckmesser
#define SHTC3     // Temperatur

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

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
