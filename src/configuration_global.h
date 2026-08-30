#define SOURCE_VERSION "4.35"
#define SOURCE_VERSION_SUB "p"
#define SOURCE_VERSION_WEB_SUB "p"

// Werkseinstellung des Rufzeichens und der zugehoerige "Node ist noch nicht
// konfiguriert"-Test. Beides stand bisher als Literal an fuenf Stellen in drei
// Images (esp32_main, nrf52_main, safeboot) und wurde dort unterschiedlich
// geprueft -- safeboot testete vier Formen, die Mains drei, und der Default wurde
// an einer weiteren Stelle erneut als Literal hingeschrieben. Aendert sich die
// Werkseinstellung, muss das genau einmal hier passieren. ALT-34.
#define DEFAULT_CALL "XX0XXX-00"
#define DEFAULT_CALL_PREFIX "XX0XXX"

// true, wenn das Rufzeichen noch die Werkseinstellung ist (oder leer/"none").
// Der 6-Zeichen-Praefixtest deckt "XX0XXX-00" mit ab -- die vier Formen, die
// safeboot einzeln geprueft hat, sind damit vollstaendig abgedeckt.
#ifdef __cplusplus
inline bool isNodeUnconfigured(const char *call)
{
    if (call == nullptr || call[0] == 0x00)
        return true;
    if (__builtin_memcmp(call, DEFAULT_CALL_PREFIX, 6) == 0)
        return true;
    if (__builtin_memcmp(call, "none", 4) == 0)
        return true;
    return false;
}
#endif

// ---------------------------------------------------------------------------
// Settings-Persistenz: Build-Kennung und Layout-Generation sind ZWEI Dinge
//
// FLASH_VERSION ist die Build-/Release-Kennung. Sie wird pro Release
// hochgezogen, wird in --info angezeigt und ist rein informativ.
//
// FLASH_STRUCT_VERSION benennt die Generation des Settings-Layouts, also den
// Aufbau von struct s_meshcom_settings. Sie wird NUR hochgezogen, wenn sich
// dieses Layout tatsaechlich aendert -- Feld hinzugefuegt, entfernt, Typ oder
// Reihenfolge geaendert.
//
// Nur FLASH_STRUCT_VERSION entscheidet ueber clear_flash(). Vorher wurde
// FLASH_VERSION verglichen: damit hat JEDES Release mit neuem Datum die
// Konfiguration jedes aktualisierenden Knotens geloescht -- Rufzeichen, WLAN,
// Sensoren --, auch wenn sich am Layout nichts geaendert hatte. Genau das ist
// beim Sprung 20260724 -> 20260821 passiert: dieser Commit hat esp32_flash.h
// nicht angefasst, die Einstellungen aller Knoten aber trotzdem verworfen.
//
// Letzte echte Layout-Aenderung: 6e7c012a (2026-07-24), node_pingmax,
// node_pingcount und node_pingduration kamen hinzu. Daher 20260724.
#define FLASH_VERSION 20260724
#define FLASH_STRUCT_VERSION 20260724

// Bestandsschutz. Diese Staende tragen dasselbe Layout wie
// FLASH_STRUCT_VERSION, haben aber wegen der alten Semantik noch ihr
// Build-Datum in node_fversion stehen. Sie duerfen nicht zurueckgesetzt
// werden, nur weil sich die Bedeutung des Feldes geaendert hat.
//
// Die Liste waechst NICHT weiter: ab dieser Version speichert der Knoten
// FLASH_STRUCT_VERSION in node_fversion, nicht mehr das Datum.
#define FLASH_STRUCT_LEGACY_COUNT 1
static const int FLASH_STRUCT_LEGACY[FLASH_STRUCT_LEGACY_COUNT] = { 20260821 };

/// @return true, wenn der gespeicherte Wert dasselbe Settings-Layout meint
///         wie dieser Build -- dann darf NICHT geloescht werden.
static inline bool flashLayoutCompatible(int stored)
{
    if (stored == FLASH_STRUCT_VERSION)
        return true;

    for (int i = 0; i < FLASH_STRUCT_LEGACY_COUNT; i++)
    {
        if (stored == FLASH_STRUCT_LEGACY[i])
            return true;
    }

    return false;
}

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
#define T3_S3_V13 55
#define T_CONNECT_PRO 56
#define HELTEC_WIRELESS_PAPER 57
#define HELTEC_E213 58
#define ESP32_LORAPRS_E22 59
#define ESP32_LORAPRS_RA01 60
#define T_WATCH_S3 61

// Boards mit dem 2.13"-E-Ink-Panel (E0213A367/SSD1680) UND dem gemeinsamen WP-Display-Pfad:
// Wireless Paper + Vision Master E213. Beide nutzen identisches Layout, fette 9pt-Schrift,
// wpApplyLayout/wpRefreshClock, Browse usw. -> der Display-Code prueft WP_DISP statt nur
// BOARD_WIRELESS_PAPER. (Battery/Pins/USB bleiben board-spezifisch ueber BOARD_E213/-config.)
#if defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213)
#define WP_DISP
#endif

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
#define KISS_TCP_PORT 8001                 // KISS-over-TCP interface (ESP32, opt-out -D DISABLE_KISS_TCP)
                                          // node_sset4: 0x0010 enable, 0x0020 allow-TX, 0x0040 RxMeta
#define UDP_TX_BUF_SIZE 255                // BUffer size of outgoing buffer
#define UDP_CONF_BUFF_SIZE UDP_TX_BUF_SIZE // Buffer to hold incoming config messages
//#define SEE_ALL_PACKETS 0                  // switch to filter multiple receives of same packets from neighbours rebroadcasted
#define UDP_MSG_INDICATOR_LEN 4            // the first n bytes to recognize which incoming message we have (GATE, CONF)
#define HEARTBEAT_INTERVAL 30              // HB interval in seconds
#define POSINFO_INTERVAL 30 * 60           // POSINFO interval in minutes default 30 minutes
#define HEYINFO_INTERVAL 15 * 60           // HEYINFO interval in minutes default 15 minutes
#define TELEMETRY_INTERVAL 30 * 60         // TELEMETRY interval in minutes default 30 minutes
#define PING_INTERVAL 60                   // PING interval in seconds default 60 seconds
#define PING_MAX 5                         // Maximum number of PING attempts
#define GPS_REFRESH_INTERVAL 3             // GPS Refresh in seconds
#define TEMPHUM_INTERVAL 3000              // TEMPHUM interval in milliseconds
#define DRUCK_INTERVAL 3000                // DRUCK interval in milliseconds
#define ALIVEBLINK_INTERVAL 3000           // ALIVEBLINK interval in milliseconds
#define ALIVERESET_INTERVAL 2 * 10 * 30    // 1/2 Stunde
#define BLEBLINK_INTERVAL 3000             // BLEBLINK interval in milliseconds

// Auf diesen Boards muss der I2C-Bus vor einem Sensorzugriff neu aufgesetzt
// werden (Wire.end() + Wire.begin()), sonst haengt der Bus. Die Bedingung stand
// bisher an neun Stellen in vier Sensordateien einzeln -- und war bereits
// auseinandergelaufen: bmx280.cpp fragte an zwei Stellen nur BOARD_TBEAM_V3 ab
// und liess BOARD_E22_S3 aus. Einmal zentral definiert, damit das nicht wieder
// driften kann. DRY-25.
#if defined(BOARD_TBEAM_V3) || defined(BOARD_E22_S3)
#define MC_I2C_NEEDS_BUS_RESET 1
#else
#define MC_I2C_NEEDS_BUS_RESET 0
#endif

// Eine Speicherklasse pro Zweig. Jeder Zweig MUSS alle sechs Konstanten setzen --
// wer eine vergisst, bekommt keinen stillen Fehlwert, sondern einen Compile-Fehler,
// weil die Konstanten Array-Groessen sind. ALT-33.
#if defined(ENABLE_XML) || defined(ENABLE_SBUFFER)
// ENABLE_XML und ENABLE_SBUFFER hatten bis 2026-08-18 zwei byte-identische Zweige
// nebeneinander; zusammengelegt, damit sie nicht auseinanderlaufen koennen.
#define MAX_MHEARD 50                      // max count of messages in mheard ringbuffer
#define MAX_MHPATH 50                      // max count of messages in mhpath ringbuffer
#define MAX_RING 20                        // max count of messages in ringbuffer
#define MAX_DEDUP_RING 60                  // dedup ring for received msg_ids (separate from TX ring)
#define MAX_LOG 20                         // max count of messages in ringbuffer
#define MAX_RING_UDP 20                    // size of Ringbuffer for UDP TX messages received from LoRa
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(BOARD_RAK4630)
// ESP32-S3 (320 KB SRAM) and nRF52840 (256 KB RAM) — full buffer sizes
#define MAX_MHEARD 80                      // max count of messages in mheard ringbuffer (was 20, 85-124 H00 nodes observed)
#define MAX_MHPATH 100                     // max count of messages in mhpath ringbuffer (was 30, multiple paths per node)
#define MAX_RING 20                        // max count of messages in ringbuffer
#define MAX_DEDUP_RING 100                 // dedup ring for received msg_ids (was 60, wraparounds observed)
#define MAX_LOG 10                         // max count of messages in LOG-ringbuffer (ram_opti)
#define MAX_RING_UDP 20                    // size of Ringbuffer for UDP TX messages received from LoRa (was 20)
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
#define MAX_HOP_LIMIT 7                    // obere Schranke fuer {SET} und ACK-Plausibilitaet
                                           // (Byte 5 einer ACK fuehrt max_hop in 7 Bit; im Feld
                                           //  beobachtet: gueltige ACKs 0..4, Textpakete bis 5)

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

// EXTERNAL_RADIO only: bounded channel-access (bridge CHANNEL_BUSY) attempts per
// message, counted SEPARATELY from the MeshCom delivery retransmission budget
// (MAX_RETRANSMIT). A bridge CHANNEL_BUSY = channel access denied, not a consumed
// delivery retry. Larger than MAX_RETRANSMIT so transient channel congestion does
// not prematurely drop a message, but still bounded (no unlimited busy retries).
#define EXT_BUSY_MAX_ATTEMPTS 8

// TX-IRQ Watchdog: Maximale Zeit (ms) fuer einen LoRa-TX-Vorgang bevor Zwangs-Recovery
// SF11 BW250 CR6 255 Bytes ToA ~6s -> 15s gibt ausreichend Sicherheitsabstand
#define TX_WATCHDOG_MS      15000

// OnRxDone processing time monitoring
#define ONRXDONE_WARN_MS    50      // Warnung wenn OnRxDone laenger als X ms dauert

// LoRa Message Types
#define MSG_TYPE_ACK          0x41
#define MSG_TYPE_TEXT         0x3A
#define MSG_TYPE_POSITION     0x21
#define MSG_TYPE_HEY          0x40

// Ring Buffer Slot Status (ringBuffer[slot][1])
#define RING_STATUS_READY     0x00   // Ready to send
#define RING_STATUS_SENT      0x01   // Sent, waiting for ACK/timer (0x01..0x14 = aging counter)
#define RING_STATUS_DONE      0xFF   // Final, no retransmission
// 0x80: slot owned by an in-flight external-radio TX awaiting an async bridge
// TX_RESULT (EXTERNAL_RADIO only). Deliberately outside READY(0x00), the SENT
// aging window (0x01..0x14) and DONE(0xFF) so normal selection skips it and
// retransmission maintenance must skip it explicitly. Used only when the optional
// EXTERNAL_RADIO backend is compiled in; otherwise no slot ever carries it.
#define RING_STATUS_EXT_PENDING 0x80

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

// --- Persistenter Display-Dreh-Offset (Terminalkommando --rotate 0/90/180/270) --------------
// g_dispRotOffset (Grad) wird board-uebergreifend ADDITIV auf die Werks-Basisrotation
// aufaddiert: setRotation((basis + g_dispRotOffset) % 360). 0 = Werksausrichtung. Persistiert
// als meshcom_settings.node_disp_rot (NVS "node_disrot"), beim Boot in initDisplay() gespiegelt.
// Definition in esp32_functions.cpp. Nur Wireless Paper + Vision Master E213 (WP_DISP).
#if defined(WP_DISP)
extern int g_dispRotOffset;
#endif
