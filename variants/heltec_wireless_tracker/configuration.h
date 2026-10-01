/*
definitions for HELTEC_V3
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// HELTEC_V3 specific config
#define MODUL_HARDWARE HELTEC_TRACKER
#define SX1262_V3
#define RX_TIMEOUT_VALUE 0      // continous rx with 0

#define ENABLE_INA226_DISABLED  // I2C fault

//#define ENABLE_SOFTSER


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

#define RESET_OLED RST_OLED

#define I2C_SDA 7 //* I2C pins for this board
#define I2C_SCL 6 //* I2C pins for this board

#define VEXT_ENABLE Vext // active low, powers the oled display and the lora antenna boost
#define BUTTON_PIN 0 //*

#define BATTERY_PIN 1 //* A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_MULTIPLIER 4.9245

#define USE_SX1262

#define RADIO_SCLK_PIN      9
#define RADIO_MISO_PIN      11
#define RADIO_MOSI_PIN      10
#define RADIO_CS_PIN        8   // NSS
#define RADIO_RST_PIN       12
#define RADIO_DIO1_PIN      14
#define RADIO_BUSY_PIN      13

#define SX1262X_CS  RADIO_CS_PIN
#define SX1262X_IRQ RADIO_DIO1_PIN
#define SX1262X_RST RADIO_RST_PIN
#define SX1262X_GPIO RADIO_BUSY_PIN

#define SDA_PIN 7 //*
#define SCL_PIN 6 //*

// R3-03 (2026-09-16): stand hier als 99. 99 ist auf diesem Board kein
// gueltiger GPIO -- OneWire hat hier also nie funktioniert. -1 ist die
// Schreibweise fuer "dieses Board hat keinen OneWire-Pin", wie sie
// T-ETH-ELITE_1262 schon benutzt: sie faellt durch den `> 0`-Test in
// onewire_functions.cpp, der Treiber startet nicht.
//
// Das Makro bleibt DEFINIERT und wird nicht geloescht: die gesamte
// OneWire-Implementierung steht in `#ifdef OneWire_GPIO`
// (onewire_functions.cpp:13-379). Ohne das Makro koennte man den Sensor
// auch mit `--owgpio <pin>` nicht mehr einschalten -- das waere eine
// Funktionsentfernung, keine Bereinigung.
#define OneWire_GPIO -1

#define VEXT_CTRL   3   // To turn on GPS and TFT
#define ADC_CTRL    2   // ADC_CTRL = HIGH

#define HAS_TFT 1

#define GPS_RX_PIN 33
#define GPS_TX_PIN 34

// bei WIRELESS_TRACKER bitte auf SOFTCHECK lassen
#define GPS_BAUDRATE_SOFTCHECK        // GPS Baudratenermittlung wird mit Software Loop geprüft
//#define ENABLE_UBLOX                  // UBLOX wird fix festgelegt und EIN setup gemacht
#define ENABLE_L76K                     // Chip Erkennung fix auf L86K
//#define ENABLE_GPS_UBLOX_FIX          // UBLOX wird fix festgelegt und KEIN setup gemacht
#define GPS_BAUDRATE_SETFIX 115200      // Die Baudrate für GPS wird auf FIXWERT gesetzt

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
