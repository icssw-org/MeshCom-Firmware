/*
definitions for T-Beam
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// T-Beam specific config
#define MODUL_HARDWARE TBEAM
#define MODUL_FW_TBEAM TBEAM
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz
#define ENABLE_GPS
#define ENABLE_BMX280
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_INA226
#define ENABLE_MC811
#define ENABLE_RTC
#define ENABLE_SOFTSER
#define SX127X
#define TX_POWER_MAX 17  // max 17 dBm
#define TX_POWER_MIN 2

// Defined using AXP192
#define XPOWERS_CHIP_AXP192
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define TX_OUTPUT_POWER 17
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

//#define LORA_RST  //already defined
#define LORA_DIO0 LORA_IO0
#define LORA_DIO1 LORA_IO1
//#define LORA_CS  //already defined

#define SDA_PIN 21
#define SCL_PIN 22

#define I2C_SDA    SDA_PIN
#define I2C_SCL    SCL_PIN

#define BUTTON_PIN 38
