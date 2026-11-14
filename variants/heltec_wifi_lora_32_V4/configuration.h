/*
    definitions for HELTEC_V4
    Based on V3 configuration with PA control additions
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// HELTEC_V4 specific config
#define MODUL_HARDWARE HELTEC_V4
#define SX1262_V4
#define RX_TIMEOUT_VALUE 0      // continous rx with 0

#define ENABLE_INA226_DISABLED

//#define ENABLE_SOFTSER


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define CURRENT_LIMIT 140 // in mA

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

// GPSV4-01: moved off 41/42 (GNSS PPS / reset on V4, see below); 4/3 are free header pins.
#define I2C_SDA 4 // I2C pins for this board
#define I2C_SCL 3

#define VEXT_ENABLE Vext // active low, powers the oled display and the lora antenna boost
#define BUTTON_PIN 0

#define BATTERY_PIN 1 // A battery voltage measurement pin
#define ADC_MULTIPLIER 4.3270     // V4 has different ADC calibration
#define ADC_CTRL_PIN 37         // V4 has ADC control pin

#define USE_SX1262

#define LORA_DIO0 -1 // a No connect on the SX1262 module
#define LORA_RESET 12
#define LORA_DIO1 14 // SX1262 IRQ
#define LORA_DIO2 13 // SX1262 BUSY
#define LORA_DIO3    // Not connected on PCB, but internally on the TTGO SX1262

#define RF95_SCK 9
#define RF95_MISO 11
#define RF95_MOSI 10
#define RF95_NSS 8

#define SX1262X_CS RF95_NSS
#define SX1262X_IRQ LORA_DIO1
#define SX1262X_RST LORA_RESET
#define SX1262X_GPIO LORA_DIO2

#define SDA_PIN 17
#define SCL_PIN 18

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

// V4 has different GPS pins than V3 (source: Meshtastic variants/esp32s3/heltec_v4/variant.h)
#define GPS_RX_PIN 39 // ESP RX <- GNSS TX
#define GPS_TX_PIN 38 // ESP TX -> GNSS RX
#define HV4_GPS_EN_PIN 34 // GNSS power enable, active LOW
#define HV4_GPS_EN_ACTIVE LOW
#define HV4_GPS_WAKE_PIN 40
#define HV4_GPS_RST_PIN 42
#define HV4_GPS_PPS_PIN 41

// White user LED, GPIO35, active HIGH, as on V3: `--board led` heartbeat (off by default) and RM
// "led on|flash|off"; REMOTE_LED_PIN derives from BOARD_LED. V4R8 has its LED on GPIO46 (GPIO35 is
// octal PSRAM there); untested on V4 hardware until someone reports.
// V4R8 is not supported: GPIO34 (GNSS enable here) is an octal-PSRAM line on the S3R8 (PSRAM is not initialised by this
// env), and the GNSS enable on R8 is reportedly GPIO42. Target is the S3R2 V4 only, no runtime guard.
#define BOARD_LED 35

// PA Control Pins (defined in platformio.ini build flags)
// P_LORA_PA_POWER 7   - PA power enable
// P_LORA_PA_EN    2   - PA enable
// P_LORA_PA_TX_EN 46  - PA TX enable during transmission

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
