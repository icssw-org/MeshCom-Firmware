#define SOURCE_VERSION "4.35"
#define SOURCE_VERSION_SUB "p"

#define FLASH_VERSION 20260401

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
#define TBEAM_SUPREME 47
#define ESP32_S3_EBYTE_E22 48
#define TLORA_PAGER 49
#define T_DECK_PRO 50
#define TBEAM_1W 51
#define HELTEC_V4 52
#define T_ETH_ELITE_1262 53
#define HELTEC_T114 54

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
#define POSINFO_INTERVAL 30 * 60           // POSINFO interval in minutes default 30 minutes
#define HEYINFO_INTERVAL 15 * 60           // HEYINFO interval in minutes default 15 minutes
#define TELEMETRY_INTERVAL 30 * 60         // TELEMETRY interval in minutes default 30 minutes
#define GPS_REFRESH_INTERVAL 5             // GPS Refresh in seconds
#define TEMPHUM_INTERVAL 3000              // TEMPHUM interval in milliseconds
#define DRUCK_INTERVAL 3000                // DRUCK interval in milliseconds
#define ALIVEBLINK_INTERVAL 3000           // ALIVEBLINK interval in milliseconds
#define ALIVERESET_INTERVAL 2 * 10 * 30    // 1/2 Stunde
#define BLEBLINK_INTERVAL 3000             // BLEBLINK interval in milliseconds

#if defined(ENABLE_XML)
#define MAX_MHEARD 50                      // max count of messages in mheard ringbuffer
#define MAX_MHPATH 50                      // max count of messages in mhpath ringbuffer
#define MAX_RING 20                        // max count of messages in ringbuffer
#define MAX_DEDUP_RING 60                  // dedup ring for received msg_ids (separate from TX ring)
#define MAX_LOG 20                         // max count of messages in ringbuffer
#define MAX_RING_UDP 20                    // size of Ringbuffer for UDP TX messages received from LoRa
#elif defined(ENABLE_SBUFFER)
#define MAX_MHEARD 50                      // max count of messages in mheard ringbuffer
#define MAX_MHPATH 50                      // max count of messages in mhpath ringbuffer
#define MAX_RING 20                        // max count of messages in ringbuffer
#define MAX_DEDUP_RING 60                  // dedup ring for received msg_ids (separate from TX ring)
#define MAX_LOG 20                         // max count of messages in ringbuffer
#define MAX_RING_UDP 20                    // size of Ringbuffer for UDP TX messages received from LoRa
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(BOARD_RAK4630)
// ESP32-S3 (320 KB SRAM) and nRF52840 (256 KB RAM) — full buffer sizes
#define MAX_MHEARD 120                     // max count of messages in mheard ringbuffer (was 20, 85-124 H00 nodes observed)
#define MAX_MHPATH 150                     // max count of messages in mhpath ringbuffer (was 30, multiple paths per node)
#define MAX_RING 30                        // max count of messages in ringbuffer
#define MAX_DEDUP_RING 100                 // dedup ring for received msg_ids (was 60, wraparounds observed)
#define MAX_LOG 20                         // max count of messages in LOG-ringbuffer
#define MAX_RING_UDP 30                    // size of Ringbuffer for UDP TX messages received from LoRa (was 20)
#elif defined(ENABLE_TBEAM)                // very smal version only for developer tests
#define MAX_MHEARD 10                      // max count of messages in mheard ringbuffer (was 20, limited by DRAM)
#define MAX_MHPATH 10                      // max count of messages in mhpath ringbuffer (was 30, limited by DRAM)
#define MAX_RING 10                        // max count of messages in ringbuffer
#define MAX_DEDUP_RING 10                  // dedup ring for received msg_ids (was 60)
#define MAX_LOG 10                         // max count of messages in LOG-ringbuffer
#define MAX_RING_UDP 10                    // size of Ringbuffer for UDP TX messages received from LoRa (was 20)
#else
// ESP32 original (~160 KB DRAM) — reduced buffer sizes due to RAM constraints
#define MAX_MHEARD 30                      // max count of messages in mheard ringbuffer (was 20, limited by DRAM)
#define MAX_MHPATH 40                      // max count of messages in mhpath ringbuffer (was 30, limited by DRAM)
#define MAX_RING 30                        // max count of messages in ringbuffer
#define MAX_DEDUP_RING 70                  // dedup ring for received msg_ids (was 60)
#define MAX_LOG 20                         // max count of messages in LOG-ringbuffer
#define MAX_RING_UDP 25                    // size of Ringbuffer for UDP TX messages received from LoRa (was 20)
#endif

#define MAX_ZEROS 6                        // maximum number of zeros in a row in a received udp message
#define MAX_ERR_UDP_TX 10                  // maximum of errors on sending KEEP messages via UDP (derived from Udp.endPacket())
#define DHCP_REFRESH 1                     // Refreshtime of DHCP Address in minutes
#define MAX_HB_RX_TIME 65                  // time we wait for an HB from server in seconds
#define HB_WARN_TIME 35                    // seconds before logging server-not-responding warning

#define MAX_HOP_TEXT_DEFAULT 4             // max hop set on text-message
#define MAX_HOP_POS_DEFAULT 2              // max hop set on pos-message

#define RECEIVE_TIMEOUT 4500               // [SX126x] 4.5sec
#define RADIOLIB_SX126X_CAD 0x07           // 0x00...length off    0x07...32-bit detect
#define RADIOLIB_SX126X_DETMIN  10         // default 10
#define RADIOLIB_SX126X_DETPEAK 25         // default 25

// CSMA/CA - Slot-basierter Backoff mit Hardware-CAD
#define CSMA_BASE_0         4500    // Basis-Timeout Versuch 0 (ms)
#define CSMA_BASE_1         3000    // Basis-Timeout Versuch 1 (ms)
#define CSMA_BASE_2         2000    // Basis-Timeout Versuch 2 (ms)
#define CSMA_SLOT_SIZE      35      // Slot-Groesse (ms): 28ms CAD + 2ms TX-Switch + 5ms Safety
#define CSMA_SLOTS_0        10      // Zufalls-Slots bei Versuch 0 (max 350ms Jitter)
#define CSMA_SLOTS_1_2      5       // Zufalls-Slots bei Versuch 1-2 (max 175ms Jitter)
#define CSMA_MAX_ATTEMPTS   3       // Ab hier: Rapid-fire CAD bis Kanal frei
#define CSMA_RAPID_RX_MS    100     // Preamble-Check Fenster im Rapid-fire Modus (ms)

// OnRxDone processing time monitoring
#define ONRXDONE_WARN_MS    50      // Warnung wenn OnRxDone laenger als X ms dauert

// LoRa Message Types
#define MSG_TYPE_ACK          0x41
#define MSG_TYPE_TEXT         0x3A
#define MSG_TYPE_POSITION     0x21
#define MSG_TYPE_HEY          0x40

// Ring Buffer Slot Status (ringBuffer[slot][1])
#define RING_STATUS_READY     0x00   // Ready to send
#define RING_STATUS_SENT      0x01   // Sent, waiting for ACK/timer
#define RING_STATUS_DONE      0xFF   // Final, no retransmission

// Message Priority Classes (lower = higher priority)
#define MSG_PRIO_CRITICAL   1   // ACK (0x41) + persoenliche DM
#define MSG_PRIO_HIGH       2   // Gruppen-Nachrichten + Broadcast "*"
#define MSG_PRIO_NORMAL     3   // Mesh-Relay (weitergeleitete Pakete)
#define MSG_PRIO_LOW        4   // Position (0x21)
#define MSG_PRIO_BACKGROUND 5   // HEY (0x40)

// Priority-dependent CSMA base timeouts (ms)
#define CSMA_PRIO_BASE_1    3000   // ACK/DM
#define CSMA_PRIO_BASE_2    3000   // Gruppen/Broadcast
#define CSMA_PRIO_BASE_3    4500   // Relay
#define CSMA_PRIO_BASE_4    5500   // Position
#define CSMA_PRIO_BASE_5    5500   // HEY

// Priority-dependent CSMA slot ranges
#define CSMA_PRIO_SLOTS_1   10   // max 350ms Jitter
#define CSMA_PRIO_SLOTS_2   10   // max 350ms Jitter
#define CSMA_PRIO_SLOTS_3   10   // max 350ms Jitter
#define CSMA_PRIO_SLOTS_4   10   // max 350ms Jitter
#define CSMA_PRIO_SLOTS_5   10   // max 350ms Jitter

// Trickle-HEY (RFC 6206 adaptiert)
#define TRICKLE_IMIN_S        30      // Schnellstes HEY-Intervall (30s nach Topologieaenderung)
#define TRICKLE_IMAX_S        (15*60) // Langsamstes HEY-Intervall (15min, wie bisher)
#define TRICKLE_K             2       // Redundanzschwelle: eigenen HEY unterdruecken wenn >=k konsistente gehoert

// Priority statistics interval
#define PRIO_STAT_INTERVAL_S  300   // 5 Minuten
#define PRIO_HWM_INTERVAL_S   1800  // 30 Minuten

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
