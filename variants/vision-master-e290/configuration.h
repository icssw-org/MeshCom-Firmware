/*
definitions for HELTEC E290
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// HELTEC_E290 specifig config
#define MODUL_HARDWARE HELTEC_E290
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz
#define ENABLE_GPS
#define ENABLE_BMX280
#define ENABLE_BMX680
//    #define ENABLE_MCP23017
//    #define ENABLE_INA226
//    #define ENABLE_MCU811
//    #define ENABLE_RTC
//    #define ENABLE_SOFTSER
#define TX_POWER_MAX 22  // max 22 dBm
#define TX_POWER_MIN 2

#define SX1262_E290

#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define GPS_RX_PIN 47
#define GPS_TX_PIN 48

// ESP32
/** 
 * Radiolib Object Module
 * Pins needed DIO0, DIO1, NSS(CS), RESET
 * Pin Definitions are found in the Board Files under: 
 * USERDIRECTORY/.platformio/packages/framework-arduinoespressif32/variants
 * Pin Definitions vary in the definitions
*/ 

// Board names are defined in platformio.ini so it is easy to distinguish them
// Chipselect SS definition is on all boards the same 

// !!sofern richtig wird DIO1 nur für LoRaWAN benötigt. Das TloraV2 hat keinen DIO1 definiert, 
// wird aber grundsätzlich in der Modem-Config benötigt!!!


#define TX_OUTPUT_POWER 22  // SX1268 have up to +22dBm

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

#define SDA_PIN 39
#define SCL_PIN 38

//#define RESET_OLED RST_OLED
#define I2C_SDA SDA_PIN // I2C pins for this board
#define I2C_SCL SCL_PIN

#define VEXT_ENABLE_1 18 // active high, powers the EPaper display
#define VEXT_ENABLE_2 46 // active high, powers the EPaper display

#define BUTTON_PIN 21

#define BATTERY_PIN 7 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_MULTIPLIER 4.9245

// PCB Wiring - LoRa - only used for prepareToSleep()
// Provided for use convenience, and examples
#define PIN_LORA_DIO_1          14
#define PIN_LORA_NSS            8
#define PIN_LORA_NRST           12
#define PIN_LORA_BUSY           13
#define PIN_LORA_SCK            9
#define PIN_LORA_MISO           11
#define PIN_LORA_MOSI           10