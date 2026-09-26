#pragma once
/*
definitions for LilyGo T3_S3_V1_3
*/

#include <Arduino.h>
#include <configuration_global.h>

//original LilyGo T-Beam-1W mit MesCom definitions ergänzt und tw. andere Namen

#define UNUSED_PIN                   (0)

#define BOARD_VARIANT_NAME          "LoRa T3_S3_V1.3"
#define MODUL_HARDWARE T3_S3_V13

#define USING_SX1262

#define I2C_SDA                     (18)
#define I2C_SCL                     (17)


#define GPS_RX_PIN                  (44)
#define GPS_TX_PIN                  (43)

#define BUTTON_PIN                  (0)   /*BUTTON 0 = GPIO0 (BOOT)*/
#define BUTTON_PIN1                  38    /*BUTTON 1 = GPIO38*/
#define BUTTON_PIN1_MASK             GPIO_SEL_38

// SPI GPIOs
#define RADIO_SCLK_PIN              5
#define RADIO_MISO_PIN              3
#define RADIO_MOSI_PIN              6
#define RADIO_CS_PIN                7

#define RADIO_DIO1_PIN              33
#define RADIO_IRQ_PIN               33  // same as DIO1
#define RADIO_RST_PIN               8
#define RADIO_BUSY_PIN              34

#define BOARD_LED                   4
#define LED_ON                      LOW
#define LED_OFF                     HIGH

// SX1262 alternative definitions - duplicates 433MHz/868MHz
#define SX1262_CS RADIO_CS_PIN
#define SX1262_CTL RADIO_CTRL
#define SX126x_BUSY RADIO_BUSY_PIN
#define SX1262_IRQ RADIO_DIO1_PIN
#define SX1262_RST RADIO_RST_PIN

#define ADC_PIN                     (1)

#define USE_NEW_BATT              // neu batt_functions.cpp nehmen (kommt wenn alle Nodes umgestellt sind raus)
#define USE_BATT
#ifdef USE_BATT
  #define BATTERY_PIN             1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
  #define BAT_VOLT_PIN            BATTERY_PIN
  #define BAT_ADC_PULLUP_RES      10000.0  //extern
  #define BAT_ADC_PULLDOWN_RES    10000.0  //extern
  #define BAT_MULTIPLIER (BAT_ADC_PULLUP_RES+BAT_ADC_PULLDOWN_RES)/BAT_ADC_PULLDOWN_RES
  #define BAT_MAX_VOLTAGE         4.1     //für Volt => Proz Umrechnung, definiert durch Akku
  #define BAT_MIN_VOLTAGE         3.3       //für Volt => Proz Umrechnung, definiert durch LDO
  #define BAT_VOLT_OFFSET         0         //offset
  #define BAT_VOLT_FACTOR         1      //factor
  #define BAT_ATTEN               ADC_11db
  #define BAT_WIDTH               12
#endif

#define ANALOG_PIN ADC_PIN  //testweise
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

#define HAS_DISPLAY


#define BOARD_COUNTRY 8   // EU8


//#define ENABLE_SOFTSER


#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 
#define TX_OUTPUT_POWER 8


//todo #define WAIT_TX x         // waiting after Lora TX in doTX() >800µs

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

// =============================================
// LilyGo T-BEAM 1W + ESP32-S3-WROOM-1-N16R8
// spezielle externe User GPIO noch nicht voll ausdefiniert
// ===== GPIOs =====
//#define OneWire_GPIO xxx

// W7 (D6-01): die Flottenvorgaben stehen in src/configuration_default.h.
// Der Include gehoert ans ENDE: was diese Datei oben selbst setzt, hat es
// dann schon gesetzt, und die #ifndef-Waechter dort ueberspringen es.
#include <configuration_default.h>   // W7: Flottenvorgaben, #ifndef -- was oben steht, gewinnt
