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

// LoRa Chip
#define SX127X
#define TX_POWER_MAX 20  // max 20 dBm
#define TX_POWER_MIN -4
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

// Telemetry

// Extra Hardware

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

// RadioLib LoRa Bandwidth Setting in kHz

/** RadioLib Spreading Factor
 * case 6: SF_6;
    case 7: SF_7;
    case 8: SF_8;
    case 9: SF_9;
    case 10: SF_10;
    case 11: SF_11;
    case 12: SF_12;
*/

// GPS on Board
#define GPS_RX_PIN 34   // Serial Pin RX
#define GPS_TX_PIN 12   // Serial Pin TX

// bei TBEAM bitte auf SOFTCHECK lassen .. Interrupt routine benötigt zu viel vom IRAM
#define GPS_BAUDRATE_SOFTCHECK        // GPS Baudratenermittlung wird mit Software Loop geprüft
//#define ENABLE_GPS_UBLOX_FIX          // UBLOX wird fix festgelegt und kein setup gemacht
//#define GPS_BAUDRATE_SETFIX 38400     // Die Baudrate für GPS wird auf FIXWERT gesetzt

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
