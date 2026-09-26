/*
definitions for HELTEC_V2_1
*/

#pragma once

#include <Arduino.h>
#include <configuration_global.h>

// HELTEC_V2_1 specific config
#define MODUL_HARDWARE HELTEC_V2_1
#define SX127X

//#define ENABLE_SOFTSER

#define TX_POWER_MAX 15
#define TX_POWER_MIN -4

#define WAIT_TX 5         // ticks waiting after Lora TX in doTX()

#define TX_OUTPUT_POWER 15
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

#define BUTTON_PIN 0

#define LORA_RST  RST_LoRa
#define LORA_DIO0 DIO0
#define LORA_DIO1 DIO1
#define LORA_CS SS

#define I2C_SDA 4       // getestet OE5HWN
#define I2C_SCL 15      // getestet OE5HWN

#define BATTERY_PIN 37  // Battery voltage via voltage divider on GPIO 37

#define OneWire_GPIO 17 // getestet OE5HWN

#define GPS_RX_PIN 13
#define GPS_TX_PIN 12

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
