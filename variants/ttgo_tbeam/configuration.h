/*
definitions for T-Beam
*/

#pragma once

#include <Arduino.h>

#include <configuration_global.h>

// T-Beam specific config
// LoRa
#define MODUL_HARDWARE TBEAM
#define MODUL_FW_TBEAM TBEAM
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz

// LoRa Chip
#define SX127X
#define TX_POWER_MAX 20  // max 20 dBm
#define TX_POWER_MIN 2
#define TX_OUTPUT_POWER 20
#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 
#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

// LoRa SPI Bus
//#define LORA_RST  //already defined
#define LORA_DIO0 LORA_IO0
#define LORA_DIO1 LORA_IO1
//#define LORA_CS  //already defined

// I2C Bus
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_SDA    SDA_PIN
#define I2C_SCL    SCL_PIN

// Defined PowerManagement AXP192
#define XPOWERS_CHIP_AXP192
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

// Telemetry
#define ENABLE_BMX280
#define ENABLE_BMP390
#define ENABLE_AHT20
#define ENABLE_SHT21
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_INA226
#define ENABLE_MC811

// Extra Hardware
#define ENABLE_RTC

// Extra Project
//#define ENABLE_SOFTSER    //do not enable on TBEAM !!

// OnBoard LED
#define BOARD_LED 4    // LED_BUILTIN

// OnBoard Button
#define BUTTON_PIN      38
#define OneWire_GPIO    4

// OnBoard ADC
#define ANALOG_PIN 36
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall


/* LoRa Parameters without onther Settings
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

// GPS on Board
#define ENABLE_GPS      // GPS on OnBoard or External
#define GPS_RX_PIN 34   // Serial Pin RX
#define GPS_TX_PIN 12   // Serial Pin TX

#define ENABLE_GPS_BAUD_FIX
