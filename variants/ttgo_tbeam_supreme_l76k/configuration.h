/*
definitions for T-Beam-Supreme-L76K
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// T-Beam specific config
#define MODUL_HARDWARE TBEAM_SUPREME_L76K
#define MODUL_FW_TBEAM TBEAM_SUPREME_L76K
#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz
#define ENABLE_GPS
#define GPS_L76K
#define ENABLE_BMX280
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_INA226
#define ENABLE_MC811
#define ENABLE_RTC
#define ENABLE_SOFTSER

#define ANALOG_PIN 36
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

#define SX1262_V3

#define TX_POWER_MAX 22  // max 22 dBm
#define TX_POWER_MIN 2

// Defined using AXP192
#define XPOWERS_CHIP_AXP192
#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define TX_OUTPUT_POWER 22
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

#define RESET_OLED RST_OLED

#define VEXT_ENABLE Vext // active low, powers the oled display and the lora antenna boost
#define BUTTON_PIN 0

#define BATTERY_PIN 1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_MULTIPLIER 4.9245

#define LORA_DIO0       // a No connect on the SX1262 module
#define LORA_RESET 5    // RST SX1262 RST
#define LORA_DIO1 1     // DIO1 SX1268 IRQ
#define LORA_DIO2 4     // BUSY SX1268 BUSY
#define LORA_DIO3       // Not connected on PCB, but internally on the TTGO SX1262, if DIO3 is high the TXCO is enabled
#define LORA_NSS 10     // CS SX1262 CS

#define SX1262X_CS LORA_NSS
#define SX1262X_IRQ LORA_DIO1
#define SX1262X_RST LORA_RESET
#define SX1262X_GPIO LORA_DIO2

#define SDA_PIN 17
#define SCL_PIN 18

#define I2C_SDA  SDA_PIN
#define I2C_SCL  SCL_PIN

#define PMU_USE_WIRE1
#define I2C1_SDA 42
#define I2C1_SCL 41

#define GPS_RX_PIN 9
#define GPS_TX_PIN 8
#define GPS_WAKEUP 7
