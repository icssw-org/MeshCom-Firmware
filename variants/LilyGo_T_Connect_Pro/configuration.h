#pragma once
/*
definitions for LilyGo T-CONNECT
*/

#include <Arduino.h>
#include <configuration_global.h>

//original LilyGo T-Beam-1W mit MesCom definitions ergänzt und tw. andere Namen

//#define UNUSED_PIN                   (0)

#define BOARD_VARIANT_NAME          "LoRa T_Connect_Pro"
#define MODUL_HARDWARE T_CONNECT_PRO

#define USING_SX1262

#define HAS_ETHERNET

#define HAS_TFT_CONNECT

// Touch
#define I2C_SDA                     (39)
#define I2C_SCL                     (40)

// =========================
// Ethernet W5500
#define ETH_CS_PIN                  (10)
#define ETH_INT_PIN                 (9)
#define ETH_RST_PIN                 (48)
#define ETH_SCLK_PIN                (12)
#define ETH_MISO_PIN                (13)
#define ETH_MOSI_PIN                (11)

#define ENABLE_GPS

#define GPS_RX_PIN                  (5)
#define GPS_TX_PIN                  (4)

#define BUTTON_PIN                  (0) /*BUTTON 0 = GPIO0 (BOOT)*/
//+#define BUTTON_PIN1                  38    /*BUTTON 1 = GPIO38*/
//+#define BUTTON_PIN1_MASK             GPIO_SEL_38

// SPI GPIOs
#define RADIO_SCLK_PIN              12
#define RADIO_MISO_PIN              13
#define RADIO_MOSI_PIN              11
#define RADIO_CS_PIN                14

#define RADIO_DIO1_PIN              45
#define RADIO_IRQ_PIN               45  // same as DIO1
#define RADIO_RST_PIN               42
#define RADIO_BUSY_PIN              38

//+#define BOARD_LED                   4
//+#define LED_ON                      LOW
//+#define LED_OFF                     HIGH

#define PIN_TFT_VDD_CTL      3
#define TFT_VDD_ENABLE       0
#define PIN_TFT_LEDA_CTL     15
#define TFT_LEDA_ENABLE      0

// SX1262 alternative definitions - duplicates 433MHz/868MHz
/*
#define SX1262_CS RADIO_CS_PIN
#define SX1262_CTL RADIO_CTRL
#define SX126x_BUSY RADIO_BUSY_PIN
#define SX1262_IRQ RADIO_DIO1_PIN
#define SX1262_RST RADIO_RST_PIN
*/

#define ADC_PIN             4

#define RELAY_SWITCH        8


#define ANALOG_PIN ADC_PIN  //testweise
#define ANALOG_REFRESH_INTERVAL 30 // sec messure intervall

#define HAS_DISPLAY

#define RF_FREQUENCY 433.175000 // 432.900000   // Hz
#define LORA_APRS_FREQUENCY 433.775000 // 432.900000   // Hz

#define BOARD_COUNTRY 8   // EU8

#define ENABLE_BMX280
#define ENABLE_BMP390
#define ENABLE_AHT20
#define ENABLE_SHT21
#define ENABLE_BMX680
#define ENABLE_MCP23017
#define ENABLE_INA226
#define ENABLE_MC811
#define ENABLE_RTC

//#define ENABLE_SOFTSER


#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 
#define TX_POWER_MAX 22  // SX1262 max 22 dBm
#define TX_POWER_MIN -9
#define TX_OUTPUT_POWER 8

#define LORA_PREAMBLE_LENGTH DEFAULT_PREAMPLE_LENGTH  // Same for Tx and Rx

//todo #define WAIT_TX x         // waiting after Lora TX in doTX() >800µs

/**
 * RadioLib Coding Rate: Allowed values range from 5 to 8.
 * case 5: CR_4_5;
    case 6: CR_4_6;
    case 7: CR_4_7;
    case 8: CR_4_8;
*/
#define LORA_CR 6

// RadioLib LoRa Bandwidth Setting in kHz
#define LORA_BANDWIDTH 250

/** RadioLib Spreading Factor
 * case 6: SF_6;
    case 7: SF_7;
    case 8: SF_8;
    case 9: SF_9;
    case 10: SF_10;
    case 11: SF_11;
    case 12: SF_12;
*/
#define LORA_SF 11

// =============================================
// LilyGo T-BEAM 1W + ESP32-S3-WROOM-1-N16R8
// spezielle externe User GPIO noch nicht voll ausdefiniert
// ===== GPIOs =====
//#define OneWire_GPIO xxx
