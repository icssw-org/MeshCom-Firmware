/*
definitions for HELTEC E290
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// HELTEC_E290 specifig config
#define MODUL_HARDWARE HELTEC_E290

#define ENABLE_MCU811
// W7: ENABLE_MCU811 above is pre-existing and out of scope here; the source only tests
// ENABLE_MC811, so this board's MC811 sensor code was never actually reachable.
#define ENABLE_MC811_DISABLED

#define ENABLE_SOFTSER


#define SX1262_E290



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

#define SDA_PIN 39
#define SCL_PIN 38

//#define RESET_OLED RST_OLED
#define I2C_SDA SDA_PIN // I2C pins for this board
#define I2C_SCL SCL_PIN

#define VEXT_ENABLE_1 18 // active high, powers the EPaper display
#define VEXT_ENABLE_2 46 // active high, powers the EPaper display

// =============================================
// Custom Board homemade E22-Ebyte Module + ESP32-S3 DevKitC-1-N16R8V
// ===== GPIOs =====
#define ANALOG_PIN 0
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

#define BUTTON_PIN 21

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

//#define USE_NEW_BATT              // neu batt_functions.cpp nehmen (kommt wenn alle Nodes umgestellt sind raus)
#define BATTERY_PIN             7
#define ADC_MULTIPLIER 4.9245


/*
#define USE_BATT
#ifdef USE_BATT
  #define BATTERY_PIN             7
  #define BAT_VOLT_PIN            BATTERY_PIN
  // voltage divider connected here to measure battery voltage
  #define BAT_ADC_PULLUP_RES      100000.0  //intern verbaut
  #define BAT_ADC_PULLDOWN_RES    100000.0  //intern verbaut
  #define BAT_MULTIPLIER (BAT_ADC_PULLUP_RES+BAT_ADC_PULLDOWN_RES)/BAT_ADC_PULLDOWN_RES
  #define ADC_MULTIPLIER          BAT_MULTIPLIER
  #define BAT_MAX_VOLTAGE         4.1  // [--maxv 4.1] Volt => Proz Umrechnung, def. Akku
  #define BAT_MIN_VOLTAGE         3.3  // fuer Volt => Proz Umrechnung, definiert durch LDO
  #define BAT_VOLT_OFFSET         0    // offset
  #define BAT_VOLT_FACTOR         1    //factor [--batt factor 1.000]
  #define BAT_ATTEN               ADC_11db 
  #define BAT_WIDTH               12

  #define VEXT_ENABLE             45 // active high, powers the EPaper display
  
  #define ADC_CTRL_PIN            19
#endif
*/

// PCB Wiring - LoRa - only used for prepareToSleep()
// Provided for use convenience, and examples
#define PIN_LORA_DIO_1          14
#define PIN_LORA_NSS            8
#define PIN_LORA_NRST           12
#define PIN_LORA_BUSY           13
#define PIN_LORA_SCK            9
#define PIN_LORA_MISO           11
#define PIN_LORA_MOSI           10

#define HAS_EPAPER

#define GPS_RX_PIN 44
#define GPS_TX_PIN 43

#define GPS_SWITCH 42

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
