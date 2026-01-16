#define SOURCE_VERSION "4.35"
#define SOURCE_VERSION_SUB "k"

//Hardware Types
#define TLORA_V2 1
#define TLORA_V1 2
#define TLORA_V2_1_1p6 3
#define TBEAM 4
#define TBEAM_1268 5
#define TBEAM_0p7 6
#define T_ECHO 7
#define T_DECK 8
#define RAK4631 9
#define HELTEC_V2_1 10
#define HELTEC_V1 11
#define TBEAM_AXP2101 12
#define EBYTE_E22 39
#define T5_EPAPER 40
#define HELTEC_TRACKER 41
#define HELTEC_STICK_V3 42
#define HELTEC_V3 43
#define HELTEC_E290 44
#define TBEAM_1262 45
#define T_DECK_PLUS 46
#define TBEAM_SUPREME_L76K 47
#define ESP32_S3_EBYTE_E22 48
#define TLORA_PAGER 49
#define T_DECK_PRO 50

#define DEFAULT_PREAMPLE_LENGTH 32

// Meshcom Params
#define LONGNAME_MAXLEN 20 // maximum length of the longname
#define TX_ENABLE 1        // switch to en/disable LoRa TX
#define MAX_CAD_WAIT 10    // maximum retries on channel free detection when we want to TX lora
#define MAX_CALL_LEN 20     // max length of Callsign

// UDP
#define UDP_PORT 1990                      // Set the server port.
#define LOCAL_PORT UDP_PORT                // Set the local port we are listening to.
#define EXTERN_PORT 1799                   // Set the external server port.
#define EXTERN_RAW_PORT 1798    
#define UDP_TX_BUF_SIZE 255                // BUffer size of outgoing buffer
#define UDP_CONF_BUFF_SIZE UDP_TX_BUF_SIZE // Buffer to hold incoming config messages
//#define SEE_ALL_PACKETS 0                  // switch to filter multiple receives of same packets from neighbours rebroadcasted
#define UDP_MSG_INDICATOR_LEN 4            // the first n bytes to recognize which incoming message we have (GATE, CONF)
#define HEARTBEAT_INTERVAL 30              // HB interval in seconds
#define POSINFO_INTERVAL 30 * 60           // POSINFO interval in seconds default 30 minutes
#define HEYINFO_INTERVAL 15 * 60           // HEYINFO interval in seconds default 15 minutes
#define TELEMETRY_INTERVAL 30 * 60         // TELEMETRY interval in seconds default 30 minutes
#define GPS_REFRESH_INTERVAL 10            // GPS Refresh in seconds
#define TEMPHUM_INTERVAL 3000              // TEMPHUM interval in milliseconds
#define DRUCK_INTERVAL 3000                // DRUCK interval in milliseconds
#define ALIVEBLINK_INTERVAL 3000           // ALIVEBLINK interval in milliseconds
#define ALIVERESET_INTERVAL 2 * 10 * 30    // 1/2 Stunde
#define BLEBLINK_INTERVAL 3000             // BLEBLINK interval in milliseconds

#if defined(ENABLE_XML)
#define MAX_MHEARD 5                       // max count of messages in mheard ringbuffer
#define MAX_MHPATH 5                       // max count of messages in mhpath ringbuffer
#define MAX_RING 20                        // max count of messages in ringbuffer
#define MAX_LOG 20                         // max count of messages in ringbuffer
#define MAX_RING_UDP 20                    // size of Ringbuffer for UDP TX messages received from LoRa
#elif defined(ENABLE_SBUFFER)
#define MAX_MHEARD 5                       // max count of messages in mheard ringbuffer
#define MAX_MHPATH 5                       // max count of messages in mhpath ringbuffer
#define MAX_RING 20                        // max count of messages in ringbuffer
#define MAX_LOG 20                         // max count of messages in ringbuffer
#define MAX_RING_UDP 20                    // size of Ringbuffer for UDP TX messages received from LoRa
#else
#define MAX_MHEARD 20                      // max count of messages in mheard ringbuffer
#define MAX_MHPATH 30                      // max count of messages in mhpath ringbuffer
#define MAX_RING 30                        // max count of messages in ringbuffer
#define MAX_LOG 20                         // max count of messages in LOG-ringbuffer
#define MAX_RING_UDP 20                    // size of Ringbuffer for UDP TX messages received from LoRa
#endif

#define MAX_ZEROS 6                        // maximum number of zeros in a row in a received udp message
#define MAX_ERR_UDP_TX 10                  // maximum of errors on sending KEEP messages via UDP (derived from Udp.endPacket())
#define DHCP_REFRESH 1                     // Refreshtime of DHCP Address in minutes
#define MAX_HB_RX_TIME 30                  // time we wait for an HB from server in seconds

#define MAX_HOP_TEXT_DEFAULT 4             // max hop set on text-message
#define MAX_HOP_POS_DEFAULT 2              // max hop set on pos-message

#define RECEIVE_TIMEOUT 4500               // [SX126x] 4.5sec
#define RADIOLIB_SX126X_CAD 0x07           // 0x00...length off    0x07...32-bit detect
#define RADIOLIB_SX126X_DETMIN  10         // default 10
#define RADIOLIB_SX126X_DETPEAK 25         // default 25

// SOFTSERIEL
#define SOFTSER_REFRESH_INTERVAL 5         // SOFTSER Refresh alle 5 Minuten
#define SOFTSER_APP_ID 1                   // SOFTSER ID...0 -> no active APP,  1 -> Pegelmesser

// NTP
#define NTP_UPDATE_TIME 240     // NTP update interval in Minutes
#define TIME_OFFSET 60          // utc timeoffset in minutes

// BLE Settings
#define MAX_MSG_LEN_PHONE 300
#define PAIRING_PIN "000000"    // Pairing PIN for BLE Connection

#define BLE_TEST 0

#define SYNC_WORD_SX127x 0x2b  // MeshCom Sync Word!!
#define SYNC_WORD_SX127x_2BYTE 0x2b24 // MeshCom Sync Word!!


// common for all Boards:

// OLED
#define SSD1306_ADDRESS 0x3C
