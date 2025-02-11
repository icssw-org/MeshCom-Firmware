/*
definitions for E22 Board
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// E22 specifig config
#define MODUL_HARDWARE EBYTE_E22
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz
#define SX126X  // some functions differ from SX127x and SX126x in RadioLib based on Semtech Chip
#define ENABLE_GPS
#define ENABLE_BMX280
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_INA226
#define ENABLE_RTC
#define ENABLE_SOFTSER
#define TX_POWER_MAX 22  // max 22dBm
#define TX_POWER_MIN 2
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()
#define TX_OUTPUT_POWER 22  // E22 / SX1262 have up to +22dBm
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

// Custom Board homemade E22-Ebyte Module + AZ Delivery ESP32 DevKitC v4

#define BUTTON_PIN 12

#define LORA_RST  27
#define LORA_DIO0 26 // aka BUSY Pin
#define LORA_DIO1 33
#define LORA_DIO2 32
#define LORA_CS 5
#define RXEN 14
#define TXEN 13
#define LED 2

#define I2C_SDA 21
#define I2C_SCL 22

#define SX1268_CS LORA_CS
#define SX1268_IRQ LORA_DIO1
#define SX1268_RST LORA_RST
#define SX1268_GPIO LORA_DIO0

#define OneWire_GPIO 25 // getestet OE5HWN
