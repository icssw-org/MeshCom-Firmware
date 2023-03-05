/*
    definitions
*/

#pragma once

#include <Arduino.h>

// Common LoRa Parameters
#define RF_FREQUENCY 432900000  // 433175000   // Hz
#define LORA_PREAMBLE_LENGTH 32  // Same for Tx and Rx

#ifdef RAK4630

// Define RAK LoRa parameters
#define TX_OUTPUT_POWER 22       // dBm
#define LORA_BANDWIDTH 1         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 11 // [SF7..SF12]
#define LORA_CODINGRATE 2        // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 32  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0    // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 0      // continous rx with 0
#define TX_TIMEOUT_VALUE 2000

// ETH Sield
//#define ETH_CS WB_IO5                   // Resoldered CS Pin to WB_IO5
#define ETH_CS SS                     // use this to try with pin 26 CS
#define MAX_DEVICE_ID 0xfFfFfFfFfFfF    // maximum mac address used to mask uint64_t from HW register

#endif

// Meshcom Params
#define LONGNAME_MAXLEN 20 // maximum length of the longname
#define TX_ENABLE 1        // switch to en/disable LoRa TX
#define MAX_CAD_WAIT 10    // maximum retries on channel free detection when we want to TX lora
#define WAIT_TX 30         // ticks waiting after Lora TX in doTX()
#define WAIT_AFTER_RX 15      // ticks we wait after Lora RX for Lora TX
#define WAIT_AFTER_TXDONE WAIT_TX // ticks waiting after Lora TxDone()
#define MAX_CALL_LEN 20     // max length of Callsign


// UDP
#define UDP_PORT 1880                      // Set the server port.
#define LOCAL_PORT UDP_PORT                // Set the local port we are listening to.
#define UDP_TX_BUF_SIZE 255                // BUffer size of outgoing buffer
#define UDP_CONF_BUFF_SIZE UDP_TX_BUF_SIZE // Buffer to hold incoming config messages
#define SEE_ALL_PACKETS 1                  // switch to filter multiple receives of same packets from neighbours rebroadcasted
#define UDP_MSG_INDICATOR_LEN 4            // the first n bytes to recognize which incoming message we have (GATE, CONF)
#define HEARTBEAT_INTERVAL 30              // HB interval in seconds
#define POSINFO_INTERVAL 600               // POSINFO interval in seconds
#define MAX_RING 30                        // max count of messages in ringbuffer
#define MAX_RING_UDP_OUT 30                // size of Ringbuffer for UDP TX messages received from LoRa
#define MAX_ZEROS 6                        // maximum number of zeros in a row in a received udp message
#define MAX_ERR_UDP_TX 10                   // maximum of errors on sending KEEP messages via UDP (derived from Udp.endPacket())
#define DHCP_REFRESH 1                     // Refreshtime of DHCP Address in minutes
#define MAX_HB_RX_TIME 300                 // time we wait for an HB from server in seconds

// NTP
#define NTP_UPDATE_TIME 240     // NTP update interval in Minutes
#define TIME_OFFSET 60          // utc timeoffset in minutes

// BLE Settings
#define MAX_MSG_LEN_PHONE 300


#ifdef ESP32

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

#define SYNC_WORD_SX127x 0x2b  // Meshtastic Sync Word!!
#define SYNC_WORD_SX126x 0x242b  // Meshtastic Sync Word!!

#define TX_OUTPUT_POWER 17
#define CURRENT_LIMIT 140 // in mA +20dBm are about 120mA -> check if enough headroom 

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


#ifdef BOARD_HELTEC
/** 
 * static const uint8_t Vext = 21;
static const uint8_t LED  = 25;
static const uint8_t RST_OLED = 16;
static const uint8_t SCL_OLED = 15;
static const uint8_t SDA_OLED = 4;
static const uint8_t RST_LoRa = 14;
static const uint8_t DIO0 = 26;
static const uint8_t DIO1 = 35;
static const uint8_t DIO2 = 34;
static const uint8_t KEY_BUILTIN = 0;

static const uint8_t TX = 1;
static const uint8_t RX = 3;

static const uint8_t SDA = 21;
static const uint8_t SCL = 22;

static const uint8_t SS    = 18;
static const uint8_t MOSI  = 27;
static const uint8_t MISO  = 19;
static const uint8_t SCK   = 5;

#define WIFI_LoRa_32_V2 true
#define DISPLAY_HEIGHT 64
#define DISPLAY_WIDTH  128
*/
#define LORA_RST  RST_LoRa
#define LORA_DIO0 DIO0
#define LORA_DIO1 DIO1
#define LORA_CS SS

#endif

#ifdef BOARD_TBEAM
/**
 * #define LED_BUILTIN LED_BUILTIN

static const uint8_t TX = 1;
static const uint8_t RX = 3;

static const uint8_t SDA = 21;
static const uint8_t SCL = 22;
 
static const uint8_t SS    = 18;
static const uint8_t MOSI  = 27;
static const uint8_t MISO  = 19;
static const uint8_t SCK   = 5;
static const uint8_t KEY_BUILTIN = 39;

// SPI LoRa Radio
#define LORA_SCK 5      // GPIO5  - SX1276 SCK
#define LORA_MISO 19    // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 - SX1276 MOSI
#define LORA_CS 18      // GPIO18 - SX1276 CS
#define LORA_RST 23     // GPIO23 - SX1276 RST
#define LORA_IRQ 26     // GPIO26 - SX1276 IO0
#define LORA_IO0 LORA_IRQ  // alias
#define LORA_IO1 33     // GPIO33 - SX1276 IO1 -> wired on pcb AND connected to header pin LORA1
#define LORA_IO2 32     // GPIO32 - SX1276 IO2 -> wired on pcb AND connected to header pin LORA2
*/
//#define LORA_RST  //already defined
#define LORA_DIO0 LORA_IO0
#define LORA_DIO1 LORA_IO1
//#define LORA_CS  //already defined

#endif

#ifdef BOARD_TLORA_OLV2
/**
 * #define LED_BUILTIN LED_BUILTIN

static const uint8_t KEY_BUILTIN =  0;

static const uint8_t TX =   1;
static const uint8_t RX =   3;

static const uint8_t SDA =  21;
static const uint8_t SCL =  22;

static const uint8_t SS =   18;
static const uint8_t MOSI = 27;
static const uint8_t MISO = 19;
static const uint8_t SCK =  5;
// I2C OLED Display works with SSD1306 driver
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    16

// SPI LoRa Radio
#define LORA_SCK    5   // GPIO5 - SX1276 SCK
#define LORA_MISO   19  // GPIO19 - SX1276 MISO
#define LORA_MOSI   27  // GPIO27 - SX1276 MOSI
#define LORA_CS     18  // GPIO18 - SX1276 CS
#define LORA_RST    12  // GPIO14 - SX1276 RST
#define LORA_IRQ    26  // GPIO26 - SX1276 IRQ (interrupt request)
*/
//#define LORA_RST  //already defined
#define LORA_DIO0 LORA_IRQ
//#define LORA_DIO1 LORA_IO1
//#define LORA_CS  //already defined

#endif

#ifdef BOARD_TLORA_OLV216
/**
 * static const uint8_t KEY_BUILTIN =  0;

static const uint8_t TX =   1;
static const uint8_t RX =   3;

static const uint8_t SDA =  21;
static const uint8_t SCL =  22;

static const uint8_t SS =   18;
static const uint8_t MOSI = 27;
static const uint8_t MISO = 19;
static const uint8_t SCK =  5;
// I2C OLED Display works with SSD1306 driver
#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    16

// SPI LoRa Radio
#define LORA_SCK    5   // GPIO5 - SX1276 SCK
#define LORA_MISO   19  // GPIO19 - SX1276 MISO
#define LORA_MOSI   27  // GPIO27 - SX1276 MOSI
#define LORA_CS     18  // GPIO18 - SX1276 CS
#define LORA_RST    23  // GPIO23 - SX1276 RST
#define LORA_IRQ    26  // GPIO26 - SX1276 IRQ (interrupt request)
#define LORA_D1     33  // GPIO33 - SX1276 IO1 (for LMIC Arduino library)
#define LORA_D2     32 // GPIO32 - SX1276 IO2

// SD card
#define SD_SCK  14
#define SD_MISO 2
#define SD_MOSI 15
#define SD_CS   13
*/
//#define LORA_RST  //already defined
#define LORA_DIO0 LORA_IRQ
#define LORA_DIO1 LORA_D1
//#define LORA_CS  //already defined
#endif

#endif

// common for all Boards:

// OLED
#define SSD1306_ADDRESS 0x3C