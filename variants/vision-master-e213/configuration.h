/*
definitions for HELTEC Vision Master E213 (V1.1.1)
ESP32-S3R8 (16MB Flash, 8MB PSRAM) + SX1262 + 2.13" E-Ink 250x122.

Abgeleitet von vision-master-e290 (Funkpfad, nativer USB) und wireless-paper (2.13"-Panel,
keine Sensoren, kein GPS). Der Panel-Controller variiert je HW-Version und wird zur Laufzeit
per Chip-ID erkannt (detectEinkChipId() in esp32_functions.cpp):
  - E0213A367      -> V1.0 / V1.1.1 / V1.2
  - LCMEN2R13EFC1  -> V1.1

Pinbelegung verifiziert aus offiziellem Heltec-Schaltplan HT-VME213_V1.0:
  E-Ink: BUSY=1 DC=2 RST=3 CLK=4 CS=5 SDI/MOSI=6  (in src/Platforms/VisionMasterE213/VisionMasterE213.h)
  LoRa : NSS=8 SCK=9 MOSI=10 MISO=11 RST=12 BUSY=13 DIO1=14  (identisch zur E290)
  Akku : VBAT_Read=7  ADC_Ctrl=46 ;  LED=45 ; VEXT/Ve_Ctrl=18 (active HIGH) ; PRG=0 USER=21
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// Vision Master E213 specific config
#define MODUL_HARDWARE HELTEC_E213


// --- Keine Onboard-Sensoren auf dem E213 -> bewusst NICHT aktiviert ---
// (BMX280 / BMP390 / AHT20 / SHT21 / BMX680 / MCP23017 / INA226 / MCU811 / RTC / SOFTSER)
#define ENABLE_BMX280_DISABLED
#define ENABLE_BMX680_DISABLED
#define ENABLE_RTC_DISABLED
#define ENABLE_BMP390_DISABLED
#define ENABLE_AHT20_DISABLED
#define ENABLE_MCP23017_DISABLED
#define ENABLE_SHT21_DISABLED
#define ENABLE_MC811_DISABLED
#define ENABLE_INA226_DISABLED
#define GPS_RX_PIN 44
#define GPS_TX_PIN 43
#define GPS_MODULE_REFRESH_INTERVAL 20 // sec

#define GPS_SWITCH 42

#define TX_POWER_MIN 2

// SX1262 mit identischer Pinbelegung wie E290 -> bewaehrten E290-Funkpfad mitnutzen
// (radio-Objekt, RX-IRQ, Spektralscan, Kommando-/Web-Funktionen). Steuert KEIN Display.
#define SX1262_E290

// generischer E-Paper-Pfad (Display-Init, sendDisplay*, Update) wie E290/WP
#define HAS_EPAPER


#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define CURRENT_LIMIT 140   // mA

// RadioLib Modem-Parameter (wie E290)

// =============================================
// GPIOs
// ===== Analog / Batterie =====
#define ANALOG_PIN 0
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

// PRG-Taster (BUTTON_1) liegt auf GPIO0; USER-Taster (BUTTON_2) auf GPIO21.
#define BUTTON_PIN 0

// Onboard-LED des E213 auf GPIO45 (active HIGH). ACHTUNG: GPIO18 ist hier VEXT (NICHT LED
// wie bei der Wireless Paper). MeshCom blinkt die LED ueber den BOARD_LED-Pfad; per Default
// AUS (bUSER_BOARD_LED=false), per "--board led on" einschaltbar.
#define BOARD_LED 45

// I2C frei liegend (QL_SDA/QL_SCL der SH2.0-4P-Sensorports). esp32_main.cpp ruft beim Start
// ungeschuetzt Wire.begin(I2C_SDA, I2C_SCL) auf; kollidiert nicht mit E-Ink 1-6 oder LoRa 8-14.
#define I2C_SDA 39
#define I2C_SCL 38

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

// Batteriemessung Heltec Vision Master E213 (laut offiziellem Schaltplan):
//  - VBAT_Read auf GPIO7, interner Teiler 100k/100k (Faktor 2)
//  - Mess-Freigabe ueber ADC_Ctrl GPIO46 (Polaritaet am Geraet verifizieren)
#define ADC_CTRL_E213 46

#define USE_NEW_BATT
#define USE_BATT
#ifdef USE_BATT
  #define BATTERY_PIN             7
  #define BAT_VOLT_PIN            BATTERY_PIN
  // Teiler-Faktor AM GERAET ermittelt (2026-06-23): Pin misst ~840 mV bei vollem Akku -> Faktor
  // ~4.92, IDENTISCH zur Vision Master E290 (ADC_MULTIPLIER 4.9245). Der im Schaltplan vermutete
  // 100k/100k-Teiler (Faktor 2) war FALSCH (840 mV * 2 = 1.68 V; 840 mV * 4.9245 = 4.14 V = voll).
  // analogReadMilliVolts() ist kalibriert -> der Faktor ist rein der Teiler. Feinabgleich via --batt factor.
  #define BAT_MULTIPLIER          4.9245
  #define ADC_MULTIPLIER          BAT_MULTIPLIER
  #define BAT_MAX_VOLTAGE         4.1  // [--maxv 4.1] Volt => Proz Umrechnung, def. Akku
  #define BAT_MIN_VOLTAGE         3.3  // fuer Volt => Proz Umrechnung, definiert durch LDO
  #define BAT_VOLT_OFFSET         0    // offset
  #define BAT_VOLT_FACTOR         1    //factor [--batt factor 1.000]
  #define BAT_ATTEN               ADC_11db
  #define BAT_WIDTH               12

  #define ADC_CTRL_PIN            46
#endif

// E-Ink-Versorgung (VEXT=GPIO18, active HIGH) wird von der Plattform-Schicht
// src/Platforms/VisionMasterE213/power_controls.cpp (#ifdef Vision_Master_E213) gehandhabt.

// PCB Wiring - LoRa - nur fuer prepareToSleep() / Referenz (identisch zur E290)
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
