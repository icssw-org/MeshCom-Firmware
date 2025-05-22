/*
definitions for HELTEC_V3
*/

#pragma once

#include <Arduino.h>

#define ENABLE_SBUFFER

#include <configuration_global.h>

// HELTEC_V3 specific config
#define MODUL_HARDWARE HELTEC_V3
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz
#define SX1262_V3
#define RX_TIMEOUT_VALUE 0      // continous rx with 0
#define ENABLE_GPS
#define ENABLE_BMX280
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_MC811
//I2C fault #define ENABLE_INA226
#define ENABLE_RTC
#define ENABLE_SOFTSER
#define TX_POWER_MAX 22  // max 22dBm
#define TX_POWER_MIN 2
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 
#define TX_OUTPUT_POWER 22  // SX1262 have up to +22dBm

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

#define RESET_OLED RST_OLED
//issue #108 #define I2C_SDA 17 // I2C pins for this board
//issue #108 #define I2C_SCL 18

#define I2C_SDA 41 // I2C pins for this board
#define I2C_SCL 42

#define VEXT_ENABLE Vext // active low, powers the oled display and the lora antenna boost
#define BUTTON_PIN 0

#define BATTERY_PIN 1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_MULTIPLIER 4.9245

#define USE_SX1262

#define LORA_DIO0 -1 // a No connect on the SX1262 module
#define LORA_RESET 12
#define LORA_DIO1 14 // SX1262 IRQ
#define LORA_DIO2 13 // SX1262 BUSY
#define LORA_DIO3    // Not connected on PCB, but internally on the TTGO SX1262, if DIO3 is high the TXCO is enabled

#define RF95_SCK 9
#define RF95_MISO 11
#define RF95_MOSI 10
#define RF95_NSS 8

#define SX1262X_CS RF95_NSS
#define SX1262X_IRQ LORA_DIO1
#define SX1262X_RST LORA_RESET
#define SX1262X_GPIO LORA_DIO2

#define SDA_PIN 17
#define SCL_PIN 18

#define OneWire_GPIO 99 // getestet ???

#define GPS_RX_PIN 47
#define GPS_TX_PIN 48
