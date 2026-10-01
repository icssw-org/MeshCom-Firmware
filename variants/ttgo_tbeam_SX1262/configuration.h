/*
definitions for TBEAM 1262
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>


// TBEAM_1262 specific config
#define MODUL_HARDWARE TBEAM_1262
#define MODUL_FW_TBEAM  TBEAM_1262

//#define ENABLE_SOFTSER    do not enable onj TBEAM !!

#define ANALOG_PIN 36
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

#define SX1262X

// Defined using AXP192
#define XPOWERS_CHIP_AXP192

#define BOARD_LED 4    // LED_BUILTIN

#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

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

#define VEXT_ENABLE Vext // active low, powers the oled display and the lora antenna boost

#define BUTTON_PIN 38
#define OneWire_GPIO  4

#define BATTERY_PIN 1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_MULTIPLIER 4.9245

#define LORA_DIO0 26 // a No connect on the SX1262 module
#define LORA_RESET 23
#define LORA_DIO1 33 // SX1268 IRQ
#define LORA_DIO2 32 // SX1268 BUSY
#define LORA_DIO3    // Not connected on PCB, but internally on the TTGO SX1262, if DIO3 is high the TXCO is enabled
#define LORA_NSS 18

#define SX1262X_CS LORA_NSS
#define SX1262X_IRQ LORA_DIO1
#define SX1262X_RST LORA_RESET
#define SX1262X_GPIO LORA_DIO2

#define SDA_PIN 21
#define SCL_PIN 22

#define I2C_SDA    SDA_PIN
#define I2C_SCL    SCL_PIN

#define GPS_RX_PIN 34
#define GPS_TX_PIN 12

// bei TBEAM bitte auf SOFTCHECK lassen .. Interrupt routine benötigt zu viel vom IRAM
#define GPS_BAUDRATE_SOFTCHECK        // GPS Baudratenermittlung wird mit Software Loop geprüft
//#define ENABLE_GPS_UBLOX_FIX          // UBLOX wird fix festgelegt und kein setup gemacht
//#define GPS_BAUDRATE_SETFIX 38400     // Die Baudrate für GPS wird auf FIXWERT gesetzt

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
