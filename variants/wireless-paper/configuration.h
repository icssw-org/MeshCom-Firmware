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


// --- Keine Onboard-Sensoren auf der Wireless Paper -> bewusst NICHT aktiviert ---
// (kein ENABLE_GPS / BMX280 / BMP390 / AHT20 / SHT21 / BMX680 / MCP23017 / INA226 / MCU811 / RTC / SOFTSER)
#define ENABLE_GPS_DISABLED
#define ENABLE_BMX280_DISABLED
#define ENABLE_BMX680_DISABLED
#define ENABLE_RTC_DISABLED
#define ENABLE_BMP390_DISABLED
#define ENABLE_AHT20_DISABLED
#define ENABLE_MCP23017_DISABLED
#define ENABLE_SHT21_DISABLED
#define ENABLE_MC811_DISABLED
#define ENABLE_INA226_DISABLED

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


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define CURRENT_LIMIT 140   // mA

// RadioLib Modem-Parameter (wie E290)

// =============================================
// GPIOs
// ===== Analog / Batterie =====
#define ANALOG_PIN 0
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

// PRG-Taster der Wireless Paper liegt auf GPIO0
// PRG-Taster der Wireless Paper E213 1.1.1 liegt auf GPIO21
#define BUTTON_PIN 0

// Onboard-LED der Wireless Paper auf GPIO18 (active HIGH, einfache LED - KEIN NeoPixel).
// MeshCom blinkt sie als Heartbeat ueber den BOARD_LED-Pfad; standardmaessig AUS
// (bUSER_BOARD_LED=false), per Terminal-Kommando "--board led on" einschaltbar. Gleiche Belegung wie T-Beam-1W.
#define BOARD_LED 18

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

// Batteriemessung Heltec Wireless Paper (laut offizieller Heltec/Meshtastic-Pinbelegung):
//  - Die Messung wird ueber den Control-Pin GPIO19 freigegeben (ACTIVE LOW:
//    LOW = Teiler durchgeschaltet/messen, HIGH = getrennt/Strom sparen)

// Die eigentliche Mess-/Toggle-Logik steht in src/batt_functions.cpp (BOARD_WIRELESS_PAPER).
#define ADC_CTRL_WP 19          // Mess-Freigabe, active LOW

#define USE_NEW_BATT              // neu batt_functions.cpp nehmen (kommt wenn alle Nodes umgestellt sind raus)
#define USE_BATT
#ifdef USE_BATT
  #define BATTERY_PIN             20
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

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
