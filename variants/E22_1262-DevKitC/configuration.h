/*
definitions for E22 Board
*/

#pragma once

#include <Arduino.h>

#include <configuration_global.h>

// E22 specifig config
#define MODUL_HARDWARE EBYTE_E22

#define BOARD_COUNTRY 5   // E22-900 -> 868

#define SX1262X  // some functions differ from SX127x and SX126x in RadioLib based on Semtech Chip

//#define ENABLE_SOFTSER

#define USE_NEW_BATT              // neu batt_functions.cpp nehmen (kommt wenn alle Nodes umgestellt sind raus)
#define USE_BATT
#ifdef USE_BATT
  #define BATTERY_PIN             32
  #define BAT_VOLT_PIN            BATTERY_PIN
  // voltage divider connected here to measure battery voltage
  #define BAT_ADC_PULLUP_RES      47000.0  //extern
  #define BAT_ADC_PULLDOWN_RES    47000.0  //extern
  #define BAT_MULTIPLIER (BAT_ADC_PULLUP_RES+BAT_ADC_PULLDOWN_RES)/BAT_ADC_PULLDOWN_RES
  #define BAT_MAX_VOLTAGE         4.1     //für Volt => Proz Umrechnung, definiert durch Akku
  #define BAT_MIN_VOLTAGE         3.3     //für Volt => Proz Umrechnung, definiert durch LDO
  #define BAT_VOLT_OFFSET         -0.28   //offset
  #define BAT_VOLT_FACTOR         1       //factor
  #define BAT_ATTEN               ADC_11db
  #define BAT_WIDTH               12
#endif

#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()
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

// Custom Board homemade E22-Ebyte Module + AZ Delivery ESP32 DevKitC v4

#define BUTTON_PIN 12

#define LORA_RST  27
#define LORA_DIO0 26 // aka BUSY Pin
#define LORA_DIO1 33
//#define LORA_DIO2 not used
#define LORA_CS 5
#define E22_RXEN 14
#define E22_TXEN 13
#define BOARD_LED 2

#define I2C_SDA 21
#define I2C_SCL 22

#define SX1262X_CS LORA_CS
#define SX1262X_IRQ LORA_DIO1
#define SX1262X_RST LORA_RST
#define SX1262X_GPIO LORA_DIO0

#define OneWire_GPIO 25 // getestet OE5HWN

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

//#define GPS_BAUDRATE_SOFTCHECK        // GPS Baudratenermittlung wird mit Software Loop geprüft
//#define ENABLE_GPS_UBLOX_FIX          // UBLOX wird fix festgelegt und kein setup gemacht
//#define GPS_BAUDRATE_SETFIX 38400     // Die Baudrate für GPS wird auf FIXWERT gesetzt

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
