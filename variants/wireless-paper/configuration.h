/*
definitions for HELTEC Wireless Paper (ESP32-S3FN8 + SX1262, 2.13" E-Ink 250x122 / Panel 128x250).
Abgeleitet von vision-master-e290. Keine Onboard-Sensoren, kein GPS.

Der verbaute Panel-Controller variiert je Hardware-Version und wird zur Laufzeit per
Chip-ID erkannt (detectEinkChipId() in esp32_functions.cpp):
  - E0213A367      -> V1.0 / V1.1.1 / V1.2
  - LCMEN2R13EFC1  -> V1.1
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// Wireless Paper specific config
#define MODUL_HARDWARE HELTEC_WIRELESS_PAPER

#define RF_FREQUENCY 433.175000          // Hz  (AT 70cm, wie E290)
#define LORA_APRS_FREQUENCY 433.775000   // Hz

// --- Keine Onboard-Sensoren auf der Wireless Paper -> bewusst NICHT aktiviert ---
// (kein ENABLE_GPS / BMX280 / BMP390 / AHT20 / SHT21 / BMX680 / MCP23017 / INA226 / MCU811 / RTC / SOFTSER)

#define TX_POWER_MAX 22  // max 22 dBm
#define TX_POWER_MIN 2

// SX1262 mit identischer Pinbelegung wie E290 -> eigener Schalter, additiv in lora_setchip.cpp
#define SX1262_WIRELESS_PAPER
// Der Funkchip-Selektor SX1262_E290 ist rein radio-bezogen (radio-Objekt-Instanziierung,
// RX-Interrupt, Spektralscan, Kommando-/Web-Funktionen) und steuert KEIN Display/Board-
// Verhalten (das laeuft separat ueber BOARD_WIRELESS_PAPER + E0213A367-Panel). Da Chip
// und Pinbelegung identisch zur E290 sind, aktivieren wir dieselben Funkpfade mit.
// Das vermeidet das unvollstaendige Nachziehen von SX1262_WIRELESS_PAPER in
// esp32_main.cpp / lora_functions / spectral_scan / command_functions / web_functions.
#define SX1262_E290

// dev (>= v4.35p) nutzt das generische Makro HAS_EPAPER (statt board-spezifischer
// BOARD_E290-Pruefungen) und das gemeinsame Display-Objekt epaper_display. Damit der
// E-Paper-Pfad (Display-Init, sendDisplay*, Update) auch fuer die Wireless Paper greift,
// muss HAS_EPAPER hier gesetzt sein - genau wie in der vision-master-e290 configuration.h.
#define HAS_EPAPER

#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define TX_OUTPUT_POWER 22  // SX1262 up to +22dBm
#define CURRENT_LIMIT 140   // mA

// RadioLib Modem-Parameter (wie E290)
#define LORA_CR 6
#define LORA_BANDWIDTH 250
#define LORA_SF 11

// =============================================
// GPIOs
// ===== Analog / Batterie =====
#define ANALOG_PIN 0
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

// PRG-Taster der Wireless Paper liegt auf GPIO0
#define BUTTON_PIN 0

// --- Pins, die von NICHT board-spezifisch geschuetztem Code referenziert werden ---
// esp32_main.cpp ruft beim Start ungeschuetzt Wire.begin(I2C_SDA, I2C_SCL) auf.
// Die Wireless Paper hat keine Onboard-I2C-Sensoren; wir setzen die frei liegenden
// I2C-Pins (Heltec-Standard, kollidieren nicht mit E-Ink 4/5/6/7 oder LoRa 8-14).
#define I2C_SDA 41
#define I2C_SCL 42

// esp32_pmu.cpp legt ohne ENABLE_GPS ein SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN)
// an. Wireless Paper hat KEIN GPS -> Pins nur Platzhalter, das Objekt bleibt schlafend
// (wird nie .begin()'d), damit die Datei kompiliert.
#define GPS_RX_PIN 47  // ungenutzt (kein GPS)
#define GPS_TX_PIN 48  // ungenutzt (kein GPS)

#define OneWire_GPIO  99 // ungenutzt

// Batteriemessung Heltec Wireless Paper (laut offizieller Heltec/Meshtastic-Pinbelegung):
//  - VBAT liegt ueber einen 1:1-Spannungsteiler an GPIO20 (ADC)
//  - Die Messung wird ueber den Control-Pin GPIO19 freigegeben (ACTIVE LOW:
//    LOW = Teiler durchgeschaltet/messen, HIGH = getrennt/Strom sparen)
//  - 1:1-Teiler -> Faktor 2 (V_bat = V_pin * 2)
// Die eigentliche Mess-/Toggle-Logik steht in src/batt_functions.cpp (BOARD_WIRELESS_PAPER).
#define BATTERY_PIN 20
#define ADC_CTRL_WP 19          // Mess-Freigabe, active LOW
#define ADC_MULTIPLIER 2.0      // 1:1-Spannungsteiler

// E-Ink-Versorgung (VEXT=GPIO45, active LOW) wird von der Plattform-Schicht
// src/Platforms/WirelessPaper/power_controls.cpp (#ifdef WIRELESS_PAPER) gehandhabt.
// Daher hier KEINE VEXT_ENABLE-Defines noetig.

// PCB Wiring - LoRa - nur fuer prepareToSleep() / Referenz
#define PIN_LORA_DIO_1          14
#define PIN_LORA_NSS            8
#define PIN_LORA_NRST           12
#define PIN_LORA_BUSY           13
#define PIN_LORA_SCK            9
#define PIN_LORA_MISO           11
#define PIN_LORA_MOSI           10
