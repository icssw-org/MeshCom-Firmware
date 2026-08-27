#pragma once

// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// 20230326: Version 4.00: START
/**
 *  @author      Ralph Weich (DD5RW)
 *  @date        2025-12-03
 */

#include <atomic>

extern bool gpsDetected;
extern bool gpsInitDone;

extern bool bnextread;

extern int ifalseping;

extern int BOARD_HARDWARE;
extern bool bUSER_BOARD_LED;

extern unsigned long rebootAuto;

extern bool bRadio;

extern bool bLED_RED;
extern bool bLED_BLUE;
extern bool bLED_GREEN;
extern bool bLED_ORANGE;

extern int iWlanWait;

extern volatile bool bSetLoRaAPRS;

extern bool bDEBUG;
extern bool bDEBUGCSV;
extern bool bDEBUGEN;
extern bool bDEBUGLNG;
extern bool bLORADEBUG;
extern bool bBLEDEBUG;
extern bool bWXDEBUG;
extern bool bIODEBUG;
extern bool bLED_CLEAR;
extern bool bLED_DELAY;
extern bool bTDECKDEBUG;
extern bool bCHECKMESH;

extern bool bPingSend;

extern bool bPosDisplay;
extern bool bDisplayOff;
extern bool bDisplayIsOff;
extern bool bDisplayVolt;
extern bool bDisplayInfo;
extern bool bDisplayVia;
extern bool bDisplayCont;

extern bool bDisplayLog;
extern char LogCallsign[10];

extern bool bDisplayRetx;
extern unsigned long DisplayOffWait;
extern int DisplayTimeWait;
extern unsigned long BattTimeWait;
extern unsigned long BattTimeAPP;
extern unsigned long BMXTimeWait;
extern unsigned long BMP3TimeWait;
extern unsigned long MCU811TimeWait;
extern unsigned long INA226TimeWait;
extern bool bButtonCheck;
extern uint8_t iButtonPin;

extern bool bAnalogCheck;
extern float fAnalogValue;
extern bool bAnalogFilter;

extern float fBattFaktor;

extern bool bDisplayTrack;
extern bool bOneButton;
extern bool bEnterDfu;       // --dfu (nur nRF52): UF2-Bootloader statt normalem Neustart
extern bool bDisplayDirty;   // ALT-35: Render-Anforderung, unabhaengig vom Tastendruck

extern bool bGPSON;
extern bool bGPSAutosymbol;
extern bool bGPSUBLOX;
extern bool bGPSL76K;
extern bool bBMPON;
extern bool bBMP3ON;
extern bool bAHT20ON;
extern bool bSHT21ON;
extern bool bBMEON;
extern bool bBME680ON;
extern bool bMCU811ON;
extern bool bINA226ON;
extern bool bRTCON;
extern bool bSMALLDISPLAY;
extern bool bSOFTSERON;
extern bool bSOFTSERREAD;
extern bool bNoMSGtoALL;

extern bool bTCA9548A;
extern bool bMCP23017;

extern bool bONEWIRE;

extern bool bLPS33;

extern bool bme680_found;
extern bool bmx_found;
extern int bmx_start;
extern bool bmp3_found;
extern bool aht20_found;
extern bool sht21_found;
extern bool mcu811_found;
extern bool one_found;
extern int one_start;
extern bool ina226_found;
extern bool shtc3_found;
extern bool dht_found;

extern bool bSHORTPATH;
//extern bool bGPSDEBUG;
extern int iGPSDEBUG;
extern bool bSOFTSERDEBUG;

extern bool bBLElong;

extern bool bBOOSTEDGAIN;

extern bool bGATEWAY;
extern bool bGATEWAY_NOPOS;
extern bool bMESH;
extern bool bVIA;
extern bool bWEBSERVER;
extern bool bWIFIAP;
extern bool bEXTUDP;
extern bool bNETCONSOLE;

extern float fBaseAltidude;
extern float fBasePress;

extern unsigned long onewireTimeWait;
extern unsigned long lastHeardTime;

extern unsigned long track_to_meshcom_timer;

// common variables
extern char msg_text[MAX_MSG_LEN_PHONE * 2];

extern unsigned int _GW_ID; // ID of our Node

extern unsigned int msg_counter;

extern uint8_t RcvBuffer[UDP_TX_BUF_SIZE * 2];

extern uint8_t own_msg_id[MAX_RING][5];

// TELEMTRY global variables
extern int iNextTelemetry;
extern String strTelemetry;

// ANALOG values
extern unsigned long analog_oversample_timer;
// ADC-filtering variables
extern uint16_t ADCraw;
extern float ADCalpha;
extern float ADCexp1;
extern float ADCexp1pre;
extern float ADCexp12;
extern float ADCexp12pre;
extern float ADCexp2;

// same set of variables for BATT
extern float BATTalpha;
extern float BATTexp1;
extern float BATTexp1pre;
extern float BATTexp12;
extern float BATexp12pre;
extern float BATexp2;

// ring_index_t liegt jetzt in ring_index.h (reine Verschiebung), damit
// einzelne Ringe in eigene Uebersetzungseinheiten wandern koennen.
#include "ring_index.h"

// RINGBUFFER for incoming UDP lora packets for lora TX
extern unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE+5];
extern ring_index_t iWrite;
extern ring_index_t iRead;
extern int iRetransmit;
extern uint8_t retryCount[MAX_RING];
extern uint8_t ringPriority[MAX_RING];         // Prio 1-5 pro Slot
extern uint32_t ringEnqueueTime[MAX_RING];     // millis() timestamp when enqueued

// N-14: kanonische Deklaration mit Default-Argumenten steht in loop_functions.h
// (ein Default darf pro Parameter nur einmal je Uebersetzungseinheit stehen);
// diese Zeile deckt nur TUs ab, die ausschliesslich dieses Extern-Header ziehen.
int addTxRingEntry(const uint8_t* frame, uint16_t len, uint8_t ring_status,
                    const char* source, int retryCountIn, bool clearSlotFirst);

extern unsigned char ringbufferRAWLoraRX[MAX_LOG][UDP_TX_BUF_SIZE+5];
extern int RAWLoRaWrite;
extern int RAWLoRaRead;

// RINGBUFFER for outgoing UDP lora packets for lora TX
extern uint8_t ringBufferUDPout[MAX_RING_UDP][UDP_TX_BUF_SIZE+20];
extern int udpWrite;
extern int udpRead;

extern bool hasMsgFromPhone;

// BLE Ringbuffer to phone
extern unsigned char BLEtoPhoneBuff[MAX_RING][MAX_MSG_LEN_PHONE+5];
extern int toPhoneWrite;
extern int toPhoneRead;

// BLE Commands Ringbuffer to phone
extern unsigned char BLEComToPhoneBuff[MAX_RING][MAX_MSG_LEN_PHONE+5];
extern int ComToPhoneWrite;
extern int ComToPhoneRead;

// ringBufferLoraRX/loraWrite werden jetzt in dedup_functions.h deklariert.

// is_receiving: same reasoning as ch_util_rx_start_t below -- on ESP32, OnRxDone()/OnTxDone()
// and friends run synchronously off the esp32loop() -> checkRX() call chain (no ISR, no
// second task touches this flag there), so the atomic is unnecessary overhead. nRF52
// registers these as real interrupt callbacks -- genuine concurrency, keep std::atomic. N-13.
#if defined(ESP32)
struct is_receiving_t {
    bool v = false;
    is_receiving_t() = default;
    is_receiving_t(bool nv) : v(nv) {}
    is_receiving_t &operator=(bool nv) { v = nv; return *this; }
    operator bool() const { return v; }
};
#else
using is_receiving_t = std::atomic<bool>;
#endif
extern is_receiving_t is_receiving;   // flag to store we are receiving a lora packet.
extern std::atomic<bool> tx_is_active;   // flag to store we are transmitting  a lora packet.

extern int cad_attempt;
extern unsigned long csma_timeout;
extern int rx_irq_defer_count;
extern std::atomic<bool> cad_in_progress;
extern std::atomic<bool> cad_done_flag;
extern std::atomic<bool> cad_double_check;


// RACE-01 fix: spinlock for deferred display update (ISR → main loop)
#if defined(ESP32)
extern portMUX_TYPE displayMux;
#endif

// Channel utilization tracking (10s window)
#if defined(ESP32)
// ESP32 never registers OnHeaderDetect as a radio callback (see esp32_main.cpp),
// so ch_util_rx_start has no writer reachable from an ISR or any concurrent task
// on this platform -- no atomic needed. N-13.
struct ch_util_rx_start_t {
    unsigned long v = 0;
    ch_util_rx_start_t() = default;
    ch_util_rx_start_t(unsigned long nv) : v(nv) {}
    ch_util_rx_start_t &operator=(unsigned long nv) { v = nv; return *this; }
    unsigned long exchange(unsigned long nv) { unsigned long old = v; v = nv; return old; }
};

// Same reasoning for the accumulators/timestamps below: on ESP32 they are only touched
// from OnRxDone()/OnTxDone()/checkRX(), all called synchronously from esp32loop() -- no
// ISR, no second task. nRF52 runs these as real interrupt callbacks, keep std::atomic.
struct ch_util_ulong_t {
    unsigned long v = 0;
    ch_util_ulong_t() = default;
    ch_util_ulong_t(unsigned long nv) : v(nv) {}
    ch_util_ulong_t &operator=(unsigned long nv) { v = nv; return *this; }
    operator unsigned long() const { return v; }
    unsigned long exchange(unsigned long nv) { unsigned long old = v; v = nv; return old; }
    unsigned long fetch_add(unsigned long nv) { unsigned long old = v; v += nv; return old; }
};
#else
using ch_util_rx_start_t = std::atomic<unsigned long>;
using ch_util_ulong_t = std::atomic<unsigned long>;
#endif
extern ch_util_rx_start_t ch_util_rx_start;
extern ch_util_ulong_t ch_util_tx_start;
extern ch_util_ulong_t ch_util_rx_accum;
extern ch_util_ulong_t ch_util_tx_accum;


// Trickle-HEY state
extern unsigned long trickle_interval_ms;       // Aktuelles Intervall (Imin..Imax)
extern uint8_t trickle_consistent_count;        // Konsistente HEYs seit letztem Reset
extern int trickle_last_neighbor_count;         // Nachbarzahl beim letzten Check

// Priority statistics (5-minute window, reset after output)
extern uint16_t stat_tx_count[6];               // Index 1-5 = Prio 1-5
extern uint16_t stat_drop_count[6];             // Drops per priority
extern uint16_t stat_preempt_count;             // Priority preemptions (out-of-order sends)
extern uint32_t stat_latency_sum[6];            // Sum of queue-to-TX latency per prio (ms)
extern uint16_t stat_latency_max[6];            // Max latency per prio (ms)
extern uint8_t  stat_queue_hwm;                 // Queue high-water mark since boot
extern uint16_t stat_csma_hwm_attempts;         // Max CAD attempts since boot
extern unsigned long stat_prio_timer;           // Timer for periodic stat output
extern unsigned long stat_hwm_timer;            // Timer for HWM output

extern int isPhoneReady;      // flag we receive from phone when itis ready to receive data
extern bool bPhoneTimeValid;
extern bool bNTPDateTimeValid;
extern bool ble_disconnect_requested; // set on auth failure to trigger platform BLE disconnect

// timers
extern double gps_refresh_intervall;   // sec
extern unsigned long posinfo_interval; // check interval
extern unsigned long posfixinterall;

extern unsigned long currentWiFiMillis;
extern unsigned long previousWiFiMillis;

// Timer variables for persitence to SD
extern unsigned long lastsavePOSPersistence;
extern unsigned long lastsaveMHEARDPersistence;
extern unsigned long lastsavePATHPersistence;

extern double posinfo_distance;
extern double posinfo_direction;
extern double posinfo_lat;
extern double posinfo_lon;
extern double posinfo_last_lat;
extern double posinfo_last_lon;
extern double posinfo_prev_lat;
extern double posinfo_prev_lon;
extern double posinfo_last_direction;
extern uint32_t posinfo_satcount;
extern float fposinfo_hdop;
extern bool posinfo_fix;
extern bool posinfo_shot;
extern uint32_t posinfo_age;
extern bool pos_shot;
extern bool wx_shot;
extern int no_gps_reset_counter;
extern int gps_refresh_track;

extern unsigned long posinfo_timer;      // we check periodically to send GPS
extern unsigned long posinfo_timer_min;
extern unsigned long heyinfo_timer;      // we check periodically to send HEY
extern int ncnt_hold;

extern unsigned long telemetry_timer;    // we check periodically to send TELEMETRY
extern unsigned long temphum_timer;      // we check periodically get TEMP/HUM
extern unsigned long druck_timer;        // we check periodically get AIRPRESURE
extern unsigned long hb_timer;           // GW Server-Heartbeat-Timer
extern unsigned long web_timer;          // Refreshtime WEbServer

// batt
extern float global_batt;
extern int global_proz;

extern unsigned char mheardBuffer[MAX_MHEARD][60]; //Ringbuffer for MHeard Lines
extern char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
extern unsigned long mheardEpoch[MAX_MHEARD];  //Ringbuffer for MHeard EPoch Update Time
extern int mheardNCount[MAX_MHEARD];

extern char mheardPathCalls[MAX_MHPATH][10]; //Ringbuffer for MHeard Key = Call
extern unsigned long mheardPathEpoch[MAX_MHPATH];  //Ringbuffer for MHeard EPoch Update Time
extern unsigned char mheardPathBuffer1[MAX_MHPATH][50]; //Ringbuffer for MHeard Sourcepath
extern uint8_t mheardPathLen[MAX_MHPATH];

extern char cTimeSource[10];

extern char cBLEName[60];

// SOFTSER global variables
extern String strSOFTSER_BUF;
extern bool bSOFTSER_APP;
extern int lastSOFTSER_MINUTE;
extern int softserFunktion;

extern String strSOFTSERAPP_ID;    // ID der Messstelle
extern String strSOFTSERAPP_NAME;  // Name der Messstelle

extern String strSOFTSERAPP_PEGEL;
extern String strSOFTSERAPP_PEGEL2;
extern String strSOFTSERAPP_TEMP;
extern String strSOFTSERAPP_BATT;

extern String strSOFTSERAPP_FIXPEGEL;
extern String strSOFTSERAPP_FIXPEGEL2;
extern String strSOFTSERAPP_FIXTEMP;

extern byte c1;

extern int iDisplayType;

#if defined(BOARD_T_ECHO)
#define maxdisplines 11
#define PAGE_MAX 6
#elif defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213)
#define maxdisplines 7
#define PAGE_MAX 10   // Wireless Paper / Vision Master E213: bis zu 9 gespeicherte Nachrichten durchblaettern
#else
#define maxdisplines 7
#define PAGE_MAX 6
#endif

extern int pageLine[maxdisplines][3];
extern char pageText[maxdisplines][25];
extern char pageTextLong1[25];
extern char pageTextLong2[200];
extern int pageLineAnz;


extern int pageLastLine[PAGE_MAX][maxdisplines][3];
extern char pageLastText[PAGE_MAX][maxdisplines][25];
extern char pageLastTextLong1[PAGE_MAX][25];
extern char pageLastTextLong2[PAGE_MAX][200];
extern int pageLastLineAnz[PAGE_MAX];
extern int pageLastPointer;
extern int pagePointer;
extern int pageHold;

extern bool bShowHead;

// ONRXDONE_TIME monitoring
extern unsigned long onrxdone_max_ms;
extern unsigned int  onrxdone_warn_count;

// Deferred display update from OnRxDone
extern volatile bool bPendingDisplayText;
extern volatile bool bPendingDisplayPos;

// SPI bus guard — prevent LoRa ISR from accessing SPI while Ethernet (W5100S) is active
extern volatile bool bSPI_ETH_Active;
extern volatile bool bPendingRadioRx;
extern struct aprsMessage pendingDisplayMsg;
extern int16_t pendingDisplayRssi;
extern int8_t  pendingDisplaySnr;

// special
extern int g_dispRotOffset;