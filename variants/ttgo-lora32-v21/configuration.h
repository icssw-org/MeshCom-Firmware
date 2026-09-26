/*
definitions for TLORA_V2_1_1p6
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>


// TLORA_V2_1_1p6 specific config
#define MODUL_HARDWARE TLORA_V2_1_1p6
#define SX127X


//#define ENABLE_SOFTSER

#define USE_NEW_BATT              // neu batt_functions.cpp nehmen (kommt wenn alle Nodes umgestellt sind raus)
#define USE_BATT
#ifdef USE_BATT
  #define BATTERY_PIN             35
  #define BAT_VOLT_PIN            BATTERY_PIN
  // voltage divider connected here to measure battery voltage
  #define BAT_ADC_PULLUP_RES      100000.0  //intern verbaut
  #define BAT_ADC_PULLDOWN_RES    100000.0  //intern verbaut
  #define BAT_MULTIPLIER (BAT_ADC_PULLUP_RES+BAT_ADC_PULLDOWN_RES)/BAT_ADC_PULLDOWN_RES
  #define ADC_MULTIPLIER          BAT_MULTIPLIER
  #define BAT_MAX_VOLTAGE         4.1  // [--maxv 4.1] Volt => Proz Umrechnung, def. Akku
  #define BAT_MIN_VOLTAGE         3.3  // für Volt => Proz Umrechnung, definiert durch LDO
  #define BAT_VOLT_OFFSET         0    // offset
  #define BAT_VOLT_FACTOR         1    //factor [--batt factor 1.000]
  #define BAT_ATTEN               ADC_11db  // Standard für ESP32 mit VREF = 1.1 V für 0 bis ≈3,9 V Messbereich
  #define BAT_WIDTH               12
#endif

#define TX_POWER_MAX 20  // max 20dBm
#define TX_POWER_MIN -4

#define BOARD_LED 25    // LED_BUILTIN

#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define TX_OUTPUT_POWER 20

#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 


/**
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


//#define LORA_RST  //already defined
#define LORA_DIO0 LORA_IRQ
#define LORA_DIO1 LORA_D1
//#define LORA_CS  //already defined

#define BUTTON_PIN 12
#define BATTERY_PIN 35 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage

#define I2C_SDA    21
#define I2C_SCL    22

#define OneWire_GPIO 4

#define GPS_RX_PIN 13
#define GPS_TX_PIN 15

//#define GPS_BAUDRATE_SOFTCHECK          // GPS Baudratenermittlung wird mit Software Loop geprüft
//#define ENABLE_GPS_UBLOX_FIX          // UBLOX wird fix festgelegt und kein setup gemacht
//#define GPS_BAUDRATE_SETFIX 38400     // Die Baudrate für GPS wird auf FIXWERT gesetzt

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
