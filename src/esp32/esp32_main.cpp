
// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// 20230326: Version 4.00: START
/**
 *  @author      Kurt Baumann (OE1KBC)
 *  @date        2023-03-26
 *  @author      Ralph Weich (DD5RW)
 *  @date        2025-12-03
 */
#include <Arduino.h>
#include "uptime_min.h"   // wrap-safe 16-bit uptime minutes (NBR stamps)
#include <atomic>
#include <configuration.h>
#include "capture_functions.h"
#include "dedup_functions.h"
#include "ntp_async.h"      // NTP-01: isPending() haelt das GPS-Gate fuer --ntpsync offen
#include "configuration_global.h"
#include "instrument.h"     // TEMPORARY -- measurement scaffolding, see src/instrument.h
#include "loop_breadcrumb.h" // F3: always-on LAST_LOOP_SECTION breadcrumb for TASK_WDT forensics
#include "track_warning.h" // TRK-01: Warnhinweis bei aktivem Track
#include <maxhop.h>         // CS-01: plausibility of the persisted text hop limit
#include <RadioLib.h>

#include <Wire.h>               
#include <SPI.h>
#include <WiFi.h>

#if defined(BOARD_T_ETH_ELITE) || defined(BOARD_T_CONNECT_PRO)
#include "esp32_eth.h"
SPIClass ethSPI(FSPI);
#endif

#include "esp32_pmu.h"
#include "charge_led_mode.h"
#include "esp32_flash.h"
#include "fw_update.h"       // AU-05 (#1187): update policy (fwTick and friends)
#include "wifi_start_gate.h" // D15: WiFi start rule for the auto update
#include "fw_update_net.h"   // AU-05: CHECK / DOWNLOAD worker (own task)
#include "rm_au_go.h"        // remote "upd go" state (rm_au_esp32.cpp)
#include "safeboot/safeboot_ver.h" // AU-12: AU_SAFEBOOT_MIN for the boot demotion
#include <esp_adc_cal.h>
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <esp_sleep.h>

#if not defined(BOARD_T_DECK_PRO)
//====== Timer for periodical events u.a.
#include "Timeout.h"
Timeout timerSerial;
#endif

#if defined(ARDUINO_ARCH_ESP32)
#include <FS.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
#include "hal/gpio_hal.h"
#endif
#include "driver/gpio.h"
#endif //ARDUINO_ARCH_ESP32

// Kompatibilitaets-Macro: DIO1-Pin hat je nach Board verschiedene Namen
#if defined(BOARD_T5_EPAPER)
  #include <t5-epaper/utilities.h>   // T5-01: BOARD_LORA_IRQ is the SX1262 DIO1 (pin defines only)
#endif
#ifndef LORA_DIO1
  #if defined(E22_DIO1)
    #define LORA_DIO1 E22_DIO1
  #elif defined(RADIO_DIO1_PIN)
    #define LORA_DIO1 RADIO_DIO1_PIN
  #elif defined(RADIO_IRQ_PIN)
    #define LORA_DIO1 RADIO_IRQ_PIN
  #elif defined(PIN_LORA_DIO_1)
    #define LORA_DIO1 PIN_LORA_DIO_1
  #elif defined(BOARD_LORA_IRQ)
    #define LORA_DIO1 BOARD_LORA_IRQ
  #else
    #warning "LORA_DIO1 not defined -- safety net digitalRead() disabled"
  #endif
#endif

#if defined(ENABLE_GPS)
    #include "gps_functions.h"
    extern GPSData gpsData;
#endif

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/tdeck_sdmap.h>
#endif

#if defined(HAS_TFT_CONNECT)
#include "tft_display_functions.h"

#include "Arduino_GFX_Library.h"

#include "pin_config.h"
/*
Wiring an ST7789 (SPI):
VCC: 3.3V
GND: Ground
SCL/SCK: SPI Clock Pin
SDA/MOSI: SPI MOSI Pin
RES/RST: Digital IO (Reset)
DC: Digital IO (Data/Command)
CS: Digital IO (Optional, if not present, the display is always active) 
*/

Arduino_DataBus *bus = new Arduino_HWSPI(
    SCREEN_DC /* DC */, SCREEN_CS /* CS */, SCREEN_SCLK /* SCK */,
    SCREEN_MOSI /* MOSI */, SCREEN_MISO /* MISO */);

Arduino_GFX *gfx = new Arduino_ST7796(
    bus, SCREEN_RST /* RST */, 1 /* rotation */, true /* IPS */,
    SCREEN_WIDTH /* width */, SCREEN_HEIGHT /* height */,
    49 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col_offset2 */, 0 /* row_offset2 */);

//SPIClass SPI_TFT(NRF_SPIM3, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI);
//Adafruit_ST7789 tft1 = Adafruit_ST7789(&SPI_TFT, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

#endif


// Sensors
#include "bmx280.h"
#include "bmp390.h"
#include "aht20.h"
#include "sht21.h"
#include "bme680.h"
#include "mcu811.h"
#include "io_functions.h"
#include "ina226_functions.h"
#ifdef ENABLE_RTC
    #include "rtc_functions.h"
    #include "rtc_offset.h"    // RTC-1..3: UTC<->Knotenzeit, Refresh-Gate (test_rtc_offset)
#endif
#include "softser_functions.h"

// MeshCom Common (ers32/nrf52) Funktions
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include "loop_scheduler.h" // D1-10: shared loop scheduler (see there)
#if defined(ENABLE_MSGSTORE)
#include "msgstore_api.h"
#include "msgstore_settings.h"
#endif
#include <regex_functions.h>
#include <test_inject.h>
#include <command_functions.h>
#include "nbr_matrix.h"    // --nbrdebug: nbrLogSnapshot()/nbrMatrix fuer den 15-Minuten-Takt
#include <phone_commands.h>
#include "ble_phone_drain.h"   // BLE-N1/N2: BlePhoneSend, g_blePhoneStats
#include "ble_session.h"       // BLC-01: per-session reset, disconnect classes, counters
#include <aprs_functions.h>
#include <batt_functions.h>
#include <lora_functions.h>
#include "rm_runtime.h" // RM-05 (#1189): rmInit(), rmDrain()
#include "blocked_calls.h" // BLK-01: blockListLoad()
#include "txring_functions.h" // BP-02: txRingDepth() declaration
#include <udp_functions.h>
#include <extudp_functions.h>
#include <kiss_functions.h>
#include <web_functions/web_functions.h>
#include <mh_phone.h>
#include <topo_ui.h>
#include <time_functions.h>
#include <clock.h>
#include <onrx_stats.h> // OR-STAT
#include <setlog_lines.h> // SL-04/SL-05: --setlog on line formatters (Welle 0)
#include <onewire_functions.h>
#include <onebutton_functions.h>
#include <adc_functions.h>
#include <lora_setchip.h>
#include "esp32_functions.h"
#include "tft_display_functions.h"
#include "net_console.h"
#if defined(EXTERNAL_RADIO)
#include "external_radio_glue.h"
#endif
#include "printfdeb_functions.h"

#ifdef BOARD_HELTEC_V4
    #include "pa_control.h"
#endif

#ifndef BOARD_TLORA_OLV216
    #include <lora_setchip.h>
#endif

#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
#include "XPowersAXP192.tpp"
#include "XPowersAXP2101.tpp"
#include "XPowersLibInterface.hpp"
extern XPowersLibInterface *PMU;
#endif

// NEOPIXEL
#ifdef LED_PIN
#include <Adafruit_NeoPixel.h> 

Adafruit_NeoPixel pixels(LED_PIXEL, LED_PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 1000

bool bLED_WEISS=false;
int iCount_weiss=0;

#endif

bool bDEEP_SLEEP = false;

#ifdef BOARD_LED
    #ifndef BOARD_LED_ACTIVE_LOW
        #define BOARD_LED_ACTIVE_LOW 0
    #endif
#endif

#if defined(ENABLE_AUDIO)
#include "esp32_audio.h"
#endif

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/tdeck_main.h>
#include <t-deck/tdeck_extern.h>
#include <t-deck/tdeck_helpers.h>   // TD-23: brightnessTick()
#include <t-deck/lv_obj_functions.h>
#include <t-deck/lv_obj_functions_extern.h>
#endif

#if defined(BOARD_T_DECK_PRO)
#include <t-deck-pro/tdeck_pro.h>
#endif

#if defined(BOARD_T5_EPAPER)
#include <t5-epaper/t5epaper_main.h>
#include <t5-epaper/t5epaper_extern.h>
//#include <t-deck/lv_obj_functions.h>
#endif

#include <OneButton.h>
OneButton btn;

/**
 * RadioLib Infos und Examples:
 * SX127x:
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Settings/SX127x_Settings.ino
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Receive/SX127x_Receive.ino
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Transmit/SX127x_Transmit.ino
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_PingPong/SX127x_PingPong.ino
*/

/** Flash Examples
 * https://microcontrollerslab.com/save-data-esp32-flash-permanently-preferences-library/
*/

// Wie mit fehlender DIO1 PinDefinition beim TLORA_OLED_V2 umgehen? Theoretisch wird DIO1 nur für LoRaWAN benötigt

/**
 * LoRa Sync Word für Meshcom definieren! 2b
 * SX127x Chips haben nur ein Byte als Syncowrd, SX126x haben zwei Byte
 * Alle Funktionen Setter, Getter, etc finden sich in den korrspondierenden Libraries
 * SX1278 zB: https://github.com/jgromes/RadioLib/blob/master/src/modules/SX127x/SX1278.h
*/

unsigned long lFreeHeap = 0;
unsigned long lFreePsram = 0;

bool bPosFirst = true;
bool bHeyFirst = true;
bool bTeleFirst = true;

bool bAllStarted = true;


#if defined(BOARD_T_ETH_ELITE) || defined(BOARD_T_CONNECT_PRO)
EspETH neth;
bool bETHERNET = false;
#endif

String strTime;
String strDate;
String str;

/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"
   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.
*/

#include <NimBLEDevice.h>
#include "nimble/porting/nimble/include/os/os_mbuf.h"   // os_msys_num_free() for BLE diagnostics

// Textmessage buffer from phone, hasMsgFromPhone flag indicates new message
extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;
extern unsigned long last_upd_timer;
extern bool hb_warn_logged;


// FreeRTOS Queue for BLE data from NimBLE task to Main Loop
#include "freertos/queue.h"
#include "freertos/semphr.h"   // AU-10: BLE pause guard
#include "esp_heap_caps.h"
#include "esp_timer.h"       // AU-20: esp_timer_dump for the bleguardcycle counters

struct BleQueueItem {
    uint8_t data[MAX_MSG_LEN_PHONE];
    size_t length;
};

static QueueHandle_t bleQueue = NULL;

NimBLEServer *pServer = NULL;
NimBLECharacteristic* pTxCharacteristic;
NimBLEService *pService;
// BLC-01: written by the NimBLE host task (callbacks), read by the loop task
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;
BleStats g_bleStats;               // BLC-01: link counters since boot (src/ble_session.h)
extern bool g_ble_uart_is_connected;   // defined below; the callbacks reset it
uint16_t g_ble_conn_handle = 0xFFFF;  // current BLE connection handle

uint32_t PIN = 000000;             // pairing password PIN Passwort PIN-Code Kennwort

// Queue for sending config jsons to phone
bool config_to_phone_prepare = false;
bool conffin_sent = false;
unsigned long config_to_phone_prepare_timer = 0;
unsigned long config_to_phone_datetime_timer = 0;
const uint8_t json_configs_cnt = 10;
const char config_cmds[json_configs_cnt][20] = {"--info", "--seset", "--wifiset", "--nodeset", "--wx", "--pos", "--aprsset", "--io", "--tel", "--analogset"};
uint8_t config_cmds_index = 0;
uint8_t iPhoneState=0;

// Bluetooth UUIDs are standardized. For more info: https://www.bluetooth.com/specifications/assigned-numbers/
// Nordic UUID DB is here: https://github.com/NordicSemiconductor/bluetooth-numbers-database


// BLE connection diagnostics (bBLEDEBUG): start of the current link, for the link duration on disconnect
static uint32_t g_ble_conn_start_ms = 0;

// BLC-01: the whole per-session reset runs inside the NimBLE callbacks, not on
// the loop-observed edge: a disconnect and reconnect between two loop passes
// shows the loop no edge, and the new central would inherit isPhoneReady == 1.
static inline void bleSessionResetAll()
{
    BleSessionFlags f = { &isPhoneReady, &g_ble_uart_is_connected, &bAckInfo,
                          &config_to_phone_prepare, &conffin_sent, &ble_disconnect_requested };
    bleSessionReset(f);
}

// Connection interval in units of 1.25 ms, printed as ms with two decimals
#define BLE_ITVL_MS_FMT "%u.%02u"
#define BLE_ITVL_MS_ARGS(itvl) (unsigned)((itvl) * 125U / 100U), (unsigned)((itvl) * 125U % 100U)

class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override 
    {
        // fresh session: isPhoneReady/prepare flags are set only after the
        // app-layer auth via hello of THIS central (BLC-01)
        bleSessionResetAll();
        bleStatsOnConnect(g_bleStats);
        deviceConnected = true;
        g_ble_conn_handle = connInfo.getConnHandle();
        g_ble_conn_start_ms = millis();

        printfdeb("BLE Connected with: %s\n", connInfo.getAddress().toString().c_str());

        // parameters the phone uses for connection setup, before our update request below
        if(bBLEDEBUG)
        {
            Serial.printf("[BLE ];connect;ms;%lu;addr;%s;itvl;" BLE_ITVL_MS_FMT ";lat;%u;tmo;%u;req;30-60/0/4000;wifi;%d;msys;%d\n",
                (unsigned long)g_ble_conn_start_ms, connInfo.getAddress().toString().c_str(),
                BLE_ITVL_MS_ARGS(connInfo.getConnInterval()), connInfo.getConnLatency(),
                (unsigned)connInfo.getConnTimeout() * 10U, (int)WiFi.status(), os_msys_num_free());
        }
        /**
         *  We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments.
         */
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 400);
    };

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override 
    {
        deviceConnected = false;
        bleSessionResetAll();
        bleStatsOnDisconnect(g_bleStats, reason);
        // print the reason for the disconnection in hex
        // https://github.com/apache/mynewt-nimble/blob/master/docs/ble_hs/ble_hs_return_codes.rst
        Serial.printf("BLE disconnected. Reason: 0x%04x\n", reason);

        if(bBLEDEBUG)
        {
            unsigned long now = millis();
            Serial.printf("[BLE ];disconnect;ms;%lu;reason;0x%04x;text;%s;dur;%lu;wifi;%d;msys;%d\n",
                now, reason, bleReasonText(reason), now - (unsigned long)g_ble_conn_start_ms,
                (int)WiFi.status(), os_msys_num_free());
        }
        //NimBLEDevice::startAdvertising();
    }

    // negotiated connection parameters after an update (shows whether the phone accepted the 4 s timeout)
    void onConnParamsUpdate(NimBLEConnInfo& connInfo) override
    {
        if(bBLEDEBUG)
        {
            Serial.printf("[BLE ];connparams;ms;%lu;itvl;" BLE_ITVL_MS_FMT ";lat;%u;tmo;%u\n",
                (unsigned long)millis(), BLE_ITVL_MS_ARGS(connInfo.getConnInterval()),
                connInfo.getConnLatency(), (unsigned)connInfo.getConnTimeout() * 10U);
        }
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override
    {
        // BLE-N2: remember the negotiated MTU. The drain reads the live value
        // on every send (esp32_ble_mtu()) and counts frames longer than MTU-3.
        g_blePhoneStats.last_mtu = MTU;
        if(bBLEDEBUG)
        {
            Serial.printf("[BLE ];mtu;ms;%lu;mtu;%u\n", (unsigned long)millis(), MTU);
        }
    }

	/********************* Security handled here *********************/
	uint32_t onPassKeyDisplay() override 
    {
        if(meshcom_settings.bt_code > 0 && meshcom_settings.bt_code <= 999999)
        {
    		Serial.printf("Server PassKeyRequest <%06i>\n", meshcom_settings.bt_code);
	    	return (uint32_t)meshcom_settings.bt_code;
        }

		Serial.printf("Server PassKeyRequest <%06i>\n", PIN);
		return PIN;
	}
	/*******************************************************************/

    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
        printfdeb("Client connected: %s (encrypted: %s)\n",
            connInfo.getAddress().toString().c_str(),
            connInfo.isEncrypted() ? "yes" : "no");
    }
} serverCallbacks;


/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    /*void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        printfdeb("%s : onRead(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }*/

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        BleQueueItem item = {};
        item.length = pCharacteristic->getLength();
        if (item.length <= 0 || item.length > MAX_MSG_LEN_PHONE)
            return;
        memcpy(item.data, pCharacteristic->getValue(), item.length);
        xQueueSend(bleQueue, &item, 0);  // non-blocking, drop on full queue
    }

} chrCallbacks;

// ---------------------------------------------------------------------------
// BLE stack bring-up (esp32setup() and the AU-10 resume) and the AU-10 pause.
//
// AU-10 (#1187, AU-D18): on the classic ESP32 the TLS handshake of the firmware update does
// not fit next to NimBLE. The update job task (src/esp32/fw_update_net.cpp) calls
// esp32BlePause() before CHECK / DOWNLOAD and esp32BleResume() when it is done. The pause is
// NimBLEDevice::deinit(true): deinit(false) would keep the NimBLEServer with its GATT objects,
// but the host forgets the registered services on deinit (ble_hs_deinit) while the library
// believes them started (m_gattsStarted, service defs built), so after a re-init the table
// would be empty. deinit(true) deletes server, advertising and services; the resume rebuilds
// them with bleStackStart(), the same function the boot uses.
//
// Concurrency: pServer / pService / pTxCharacteristic and every NimBLE call of the loop task are
// used only under bleLifeLock() (non-blocking try-lock, false while a pause or resume is in
// flight or BLE is down). The job task holds s_bleLife (blocking) across the deinit and the
// init. The NimBLE host-task callbacks (onConnect, onDisconnect, onWrite) never take it.
// ---------------------------------------------------------------------------
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static SemaphoreHandle_t s_bleLife = NULL;         // created in esp32setup(); NULL = no guarding (DISABLE_BLE)
static volatile bool s_bleDown = false;            // BLE stack stopped by esp32BlePause() (or a failed resume)
static volatile bool s_bleResumeFailed = false;    // the last esp32BleResume() failed; the loop retries
static volatile bool s_bleGuard = false;           // BLE guard (spec 7): set/cleared by the loop task only; esp32BleResume() refuses while set
static char s_bleManuf[60] = {0};                  // manufacturer data string, built once at boot

// Loop-task side. True: the BLE objects may be used now, the caller MUST bleLifeUnlock().
static inline bool bleLifeLock(void)
{
    if(s_bleLife == NULL)
        return true;
    if(xSemaphoreTake(s_bleLife, 0) != pdTRUE)
        return false;
    if(s_bleDown)
    {
        xSemaphoreGive(s_bleLife);
        return false;
    }
    return true;
}

static inline void bleLifeUnlock(void)
{
    if(s_bleLife != NULL)
        xSemaphoreGive(s_bleLife);
}

#if !defined(DISABLE_BLE)   // opt-out -D DISABLE_BLE: board without a usable BLE controller
// Brings the whole NimBLE stack up: stack, security, server, service, both characteristics,
// callbacks, advertising data. startAdv false = leave advertising off (BENCH_BLE_ADV_LATE boot).
// False if NimBLEDevice::init() failed (the objects are then not created, the pointers stay NULL).
static bool bleStackStart(bool startAdv)
{
    const std::string strBLEName = cBLEName;
    const std::string strBLEManufData = s_bleManuf;

    if(!NimBLEDevice::init(strBLEName))
    {
        printfdeb("[BLE ]...init failed\n");
        return false;
    }

    printfdeb("[BLE ]...Device-Address <%s>\n", NimBLEDevice::toString().c_str());

    //NimBLEDevice::setDeviceName(strBLEName);

    NimBLEDevice::setPower(9); // +9dbm
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityAuth(false, false, false);

    /*if(meshcom_settings.bt_code >= 100000 && meshcom_settings.bt_code <= 999999)
        NimBLEDevice::setSecurityPasskey(meshcom_settings.bt_code);
    else
        NimBLEDevice::setSecurityPasskey(PIN);*/

    // Create the BLE Server. serverCallbacks is a static object: deleteCallbacks = false, or
    // NimBLEDevice::deinit(true) (AU-10 pause) would delete it with the server.
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks, false);
    pServer->advertiseOnDisconnect(true);

    // Create the BLE Service
    pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        NIMBLE_PROPERTY::WRITE  |
                        // Require a secure connection for read and write access
                        //NIMBLE_PROPERTY::WRITE_AUTHEN |  // only allow writing if paired / encrypted
                        //NIMBLE_PROPERTY::WRITE_ENC |  // only allow writing if paired / encrypted
                        NIMBLE_PROPERTY::READ   |
                        //NIMBLE_PROPERTY::READ_ENC |  // only allow reading if paired / encrypted
                        //NIMBLE_PROPERTY::READ_AUTHEN |
                        NIMBLE_PROPERTY::NOTIFY );

    NimBLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        NIMBLE_PROPERTY::WRITE  |
                        // Require a secure connection for read and write access
                        //NIMBLE_PROPERTY::WRITE_AUTHEN |  // only allow writing if paired / encrypted
                        //NIMBLE_PROPERTY::WRITE_ENC |  // only allow writing if paired / encrypted
                        NIMBLE_PROPERTY::READ   |
                        //NIMBLE_PROPERTY::READ_ENC |  // only allow reading if paired / encrypted
                        //NIMBLE_PROPERTY::READ_AUTHEN |
                        NIMBLE_PROPERTY::NOTIFY );

    pRxCharacteristic->setCallbacks(&chrCallbacks);

    // Start the service
    pService->start();

    // Start advertising
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->reset();

    std::string advName = strBLEName;
    if(!bBLElong && advName.size() > 26)
      advName = advName.substr(0, 26); // keep first 26 chars for short adverts

    pAdvertising->setName(advName);  // BLE Local Name (possibly shortened)

    if (bBLElong)
    {
        pAdvertising->setManufacturerData(strBLEManufData);
        pAdvertising->addServiceUUID(SERVICE_UUID);
    }
    else
    {
        // For short adverts we skip adding the 128-bit service UUID which

    }

    if (bBLElong)
        pAdvertising->enableScanResponse(true);
    else
        pAdvertising->enableScanResponse(false);

    if(startAdv)
        pAdvertising->start();
    else
    {
        // TM-11 experiment (BENCH_BLE_ADV_LATE): advertising starts in esp32loop() once
        // [BOOT];ready fired (WiFi joined or given up) -- TD-01 hypothesis: BLE advertising
        // during the first association attempt makes it fail.
        printfdeb("[BLE ]...advertising deferred until the network phase settled (BENCH_BLE_ADV_LATE)\n");
    }

    printfdeb("[BLE ]...Waiting a client connection to notify...\n");
    return true;
}
#endif   // !DISABLE_BLE

bool esp32BlePause(void)
{
#if defined(DISABLE_BLE)
    return false;
#else
    if(s_bleLife == NULL || s_bleDown || pServer == NULL)
        return false;   // no guard, already down, or BLE never came up: nothing to pause
    if(deviceConnected || isPhoneReady != 0)
    {
        Serial.print("[AU];ble;pause;skip;phone\n");
        return false;
    }

    xSemaphoreTake(s_bleLife, portMAX_DELAY);

    // Advertising off first: no new central can connect. Re-check afterwards, a phone may
    // have connected between the check above and the stop.
    NimBLEDevice::stopAdvertising();
    if(deviceConnected || isPhoneReady != 0 || pServer->getConnectedCount() > 0)
    {
        NimBLEDevice::getAdvertising()->start();
        xSemaphoreGive(s_bleLife);
        Serial.print("[AU];ble;pause;skip;phone\n");
        return false;
    }

    // From here the loop sees BLE as down (bleLifeLock() fails) and the pointers are NULL
    s_bleDown = true;
    pTxCharacteristic = nullptr;
    pService = NULL;
    pServer = NULL;

    // clearAll = true: server, services, characteristics and the advertising object are
    // deleted and rebuilt by esp32BleResume() (see the block comment above). The controller
    // memory stays reserved for the resume (no esp_bt_controller_mem_release()).
    const bool stopped = NimBLEDevice::deinit(true);
    deviceConnected = false;
    g_ble_conn_handle = 0xFFFF;
    phoneRingsDetach();   // 5 KB back for the job; frames for the phone are dropped until the resume.
                          // Inside s_bleLife, so a concurrent resume cannot bring BLE back without rings.
    xSemaphoreGive(s_bleLife);

    if(!stopped)
        Serial.print("[AU];ble;pause;deinit;fail\n");   // the resume rebuilds the objects on the running stack
    Serial.printf("[AU];ble;pause;heap;%u;blk;%u\n", (unsigned)esp_get_free_heap_size(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    return true;
#endif
}

// BLE guard force pause (spec 7): like esp32BlePause(), but a phone may be linked. Order: no re-advertising
// after the disconnect, advertising off, deinit(true) with the link up (NimBLE terminates the link while
// stopping), then the session flags are cleared by hand (the terminate may time out and skip onDisconnect).
// Job task only. True if the stack was stopped; esp32BleResume() brings it back after the guard release.
// Auto-update uses it on classic ESP32 only (AU_BLE_PAUSE): on the ESP32-S3 the forced deinit/reinit panics in the
// NimBLE host task (bench). INSTRUMENT builds also compile it on other targets for the bleguardcycle bench (AU-20).
#if AU_BLE_PAUSE
bool esp32BleForcePause(void)
{
#if defined(DISABLE_BLE)
    return false;
#else
    if(s_bleLife == NULL || s_bleDown || pServer == NULL)
        return false;
    const bool phone = deviceConnected || isPhoneReady != 0;

    xSemaphoreTake(s_bleLife, portMAX_DELAY);
    NimBLEServer *srv = pServer;
    if(srv != NULL)
        srv->advertiseOnDisconnect(false);
    NimBLEDevice::stopAdvertising();

    s_bleDown = true;
    pTxCharacteristic = nullptr;
    pService = NULL;
    pServer = NULL;

    const bool stopped = NimBLEDevice::deinit(true);
    deviceConnected = false;
    g_ble_conn_handle = 0xFFFF;
    bleSessionResetAll();
    phoneRingsDetach();   // same as esp32BlePause(), inside s_bleLife
    xSemaphoreGive(s_bleLife);

    if(!stopped)
        Serial.print("[AU];ble;pause;deinit;fail\n");
    Serial.printf("[AU];ble;pause;force;phone;%u;heap;%u;blk;%u\n", (unsigned)phone, (unsigned)esp_get_free_heap_size(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    return true;
#endif
}
#endif   // AU_BLE_PAUSE

// True while the BLE stack is stopped (paused by an update job or a failed resume).
bool esp32BleDown(void)
{
    return s_bleDown;
}

bool esp32BleResume(void)
{
#if defined(DISABLE_BLE)
    return true;
#else
    if(s_bleLife == NULL || !s_bleDown)
        return true;   // never paused (or already back)
    if(s_bleGuard)
    {
        Serial.print("[AU];ble;resume;refused;guard\n");   // only the guard release brings BLE back
        return false;
    }

    xSemaphoreTake(s_bleLife, portMAX_DELAY);
    if(!s_bleDown)
    {
        xSemaphoreGive(s_bleLife);
        return true;   // a concurrent resume has won
    }
    // The phone rings come back before the stack, inside s_bleLife (two resumes must not both
    // allocate). No memory: BLE stays off (the loop retries a few times; the next boot is the sure
    // retry), logged once.
    if(!phoneRingsAttach())
    {
        static bool s_ringsFailLogged = false;
        if(!s_ringsFailLogged)
        {
            s_ringsFailLogged = true;
            Serial.print("[AU];ble;resume;fail;rings\n");
        }
        s_bleResumeFailed = true;
        xSemaphoreGive(s_bleLife);
        return false;
    }

    const bool ok = bleStackStart(true);
    if(ok)
    {
        s_bleDown = false;
        s_bleResumeFailed = false;
    }
    else
        s_bleResumeFailed = true;
    xSemaphoreGive(s_bleLife);

    Serial.print(ok ? "[AU];ble;resume;ok\n" : "[AU];ble;resume;fail\n");
    return ok;
#endif
}



#ifdef SX127X
// RadioModule SX1276 
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
// SyncWord - 1byte for SX127x, 2 bytes for SX126x
// Check which chip is used !!!
#endif

#ifdef BOARD_E220
// RadioModule derived from SX1262 
LLCC68 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
#endif

#ifdef SX1262X
    // RadioModule SX1262
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    //#if not defined(BOARD_T_DECK_PRO)
    SX1262 radio = new Module(SX1262X_CS, SX1262X_IRQ, SX1262X_RST, SX1262X_GPIO);
    //#endif

#endif

#ifdef SX126X
    // RadioModule SX1268 
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    SX1268 radio = new Module(SX1268_CS, SX1268_IRQ, SX1268_RST, SX1268_GPIO);

#endif

#ifdef SX1262_E22
    // RadioModule SX1262 
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    SX1262 radio = new Module(SX126x_CS, SX126x_IRQ, SX126x_RST, SX126x_GPIO);

#endif

#ifdef SX1268_E22
    // RadioModule SX1268 
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    SX1268 radio = new Module(SX126x_CS, SX126x_IRQ, SX126x_RST, SX126x_GPIO);

#endif

#ifdef SX1262_V3
    // RadioModule SX1262
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    //  begin(sck, miso, mosi, ss).
    SX1262 radio = new Module(SX1262X_CS, SX1262X_IRQ, SX1262X_RST, SX1262X_GPIO);

#endif


#ifdef USING_SX1262 // BOARD_TBEAM_1W
    SX1262 radio = new Module(RADIO_CS_PIN, RADIO_IRQ_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

#ifdef SX1262_E290
    // RadioModule SX1262
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle,
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    //  begin(sck, miso, mosi, ss).
    SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO_1, PIN_LORA_NRST, PIN_LORA_BUSY);

#endif

#ifdef SX1262_V4
    // RadioModule SX1262 for Heltec V4 (with external PA)
    // Same pins as V3, but requires PA control during transmit
    SX1262 radio = new Module(SX1262X_CS, SX1262X_IRQ, SX1262X_RST, SX1262X_GPIO);

#endif

// Lora callback Function declarations
int checkRX(bool bRadio);
#include "lora_rx_park.h"
#if !defined(BOARD_T5_EPAPER)
static void loraRxParkDrain(void);
#endif

// save transmission state between loops
volatile int transmissionState = RADIOLIB_ERR_UNKNOWN;

// flag to indicate that a preamble was not detected
std::atomic<bool> receiveFlag{false};
std::atomic<bool> bEnableInterruptReceive{true};

// flag to indicate if we are after receiving
unsigned long iReceiveTimeOutTime = 0;
unsigned long inoReceiveTimeOutTime = 0;

// flag to indicate if we are currently allowed to transmittig
std::atomic<bool> transmittedFlag{false};
std::atomic<bool> bEnableInterruptTransmit{false};

// flag to indicate one second
unsigned long retransmit_timer = 0;

// blink frequency for board_led
unsigned long led_timer = 0;

// flag to update NTP Time
unsigned long updateTimeClient = 0;

// resend Ping
unsigned long resendPing = 0;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlagReceive(void)
{
    if(bEnableInterruptReceive)
    {
        receiveFlag = true;
    }

    if(bEnableInterruptTransmit)
    {
        transmittedFlag = true;
    }
}

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlagSent(void)
{
    if(bEnableInterruptReceive)
    {
        receiveFlag = true;
#if INSTRUMENT_ENABLED
        INSTR_IRQ_STAMP();      // R1-F: DIO1 -> readData latency
#endif
    }

    if(bEnableInterruptTransmit)
    {
        transmittedFlag = true;
    }
}

asm(".global _scanf_float");
asm(".global _printf_float");

// LoRa Events and Buffers

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "C";

uint64_t timeStamp = 0;

/** Flag if BLE UART is connected */
bool g_ble_uart_is_connected = false;

// Client basic variables
uint8_t dmac[6] = {0};

unsigned long gps_refresh_timer = 0;
unsigned long mcp_refresh_timer = 0;
unsigned long softser_refresh_timer = 0;
unsigned long rtc_refresh_timer = 0;
unsigned long pixels_delay = 0;
unsigned long ble_wait = 0;
unsigned long wifi_active_timer = 0;

bool is_new_packet(uint8_t compBuffer[4]);     // switch if we have a packet received we never saw before RcvBuffer[12] changes, rest is same
// checkSerialCommand() now lives in its own TU (C3 carve-out)
#include "serial_command.h"
// gatewayService_*() likewise (C4 carve-out)
#include "gateway_service.h"
#include "gw_reg_runtime.h"

bool g_meshcom_initialized;
bool init_flash_done=false;

unsigned int getMacAddr(void)
{
    uint64_t dmac64 = ESP.getEfuseMac();

    dmac[0] = dmac64 >> 40;
    dmac[1] = dmac64 >> 32;
    dmac[2] = dmac64 >> 24;
    dmac[3] = dmac64 >> 16;
    dmac[4] = dmac64 >> 8;
    dmac[5] = dmac64;

    unsigned int gw_id = dmac[3] << 24 | dmac[2] << 16 | dmac[1] << 8 | dmac[0];

    return gw_id;
}

// BME680
#if defined(ENABLE_BMX680)
unsigned long bme680_timer = millis();
int delay_bme680 = 0;
#endif

//=======================================================================================
#ifdef NTC_PIN
    //NCP18XH103F03RB: https://item.szlcsc.com/14214.html
    #define SERIES_RESISTOR 10000 // Series resistance value (10kΩ)
    #define B_COEFFICIENT 3950 // B value, set according to the NTC specification
    #define ROOM_TEMP 298.15 // 25°C absolute temperature (K)
    #define ROOM_TEMP_RESISTANCE 10000 // Resistance of NTC at 25°C (10kΩ)

float getTempForNTC()
{
    static float temperature = 0.0f;
    static uint32_t check_temperature = 0;
    if ((int32_t)(millis() - check_temperature) > 0) {
        analogSetAttenuation(ADC_11db); // bis <2,2V
        float voltage = analogReadMilliVolts(NTC_PIN);
        uint16_t raw = analogReadRaw(NTC_PIN);
        // die fixen 3.3 V stimmen nicht, hier sollte diese Spannung auch gemessen werden (Jumper)
        float resistance = SERIES_RESISTOR * ((3.3 / voltage *1000.0) - 1); // Calculate the resistance of NTC

        // Calculate temperature using the Steinhart-Hart equation
        temperature = (1.0 / (log(resistance / ROOM_TEMP_RESISTANCE) / B_COEFFICIENT + 1.0 / ROOM_TEMP)) - 273.15;

        if(bWXDEBUG && bDisplayCont)
            printfdeb("NTC-Temp: %.3f_°C %u_raw %.3f_mV %.2f_Ohm\n", temperature, raw, voltage, resistance);

        check_temperature  = millis() + 1000;
    }
    return temperature;
}
#endif
//=======================================================================================


// TM-51: name of the last reset cause for the boot banner. A field reboot without
// this line is indistinguishable from a power cycle (see bug-GPS-uart-overflow §3.3).
static const char* resetReasonName(esp_reset_reason_t r)
{
    switch (r)
    {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// Name of the wakeup cause after a deep-sleep reset, printed next to RESET_REASON so a
// field log shows whether the button (EXT1), a timer or something else woke the node.
static const char* wakeCauseName(esp_sleep_source_t c)
{
    switch (c)
    {
        case ESP_SLEEP_WAKEUP_UNDEFINED: return "UNDEFINED";
        case ESP_SLEEP_WAKEUP_ALL:       return "ALL";
        case ESP_SLEEP_WAKEUP_EXT0:      return "EXT0";
        case ESP_SLEEP_WAKEUP_EXT1:      return "EXT1";
        case ESP_SLEEP_WAKEUP_TIMER:     return "TIMER";
        case ESP_SLEEP_WAKEUP_TOUCHPAD:  return "TOUCHPAD";
        case ESP_SLEEP_WAKEUP_ULP:       return "ULP";
        case ESP_SLEEP_WAKEUP_GPIO:      return "GPIO";
        case ESP_SLEEP_WAKEUP_UART:      return "UART";
        default:                         return "OTHER";
    }
}

uint32_t console_txbuf_bytes = 0;
int      console_txbuf_ok    = 0;

#if !ARDUINO_USB_CDC_ON_BOOT && (CONSOLE_UART_TXBUF > 0 || (defined(INSTR_UART_TXBUF) && INSTR_UART_TXBUF > 0))
#include <driver/uart.h>
static void console_txbuf_shutdown(void)
{
    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
}
#endif

void console_txbuf_apply(HardwareSerial &port)
{
    (void)port;
#if !ARDUINO_USB_CDC_ON_BOOT && (CONSOLE_UART_TXBUF > 0 || (defined(INSTR_UART_TXBUF) && INSTR_UART_TXBUF > 0))
    static bool s_done = false;
    if (s_done)
        return;
    s_done = true;
  #if defined(INSTR_UART_TXBUF) && INSTR_UART_TXBUF > 0
    const size_t want = INSTR_UART_TXBUF;   // INSTRUMENT experiment override wins
  #else
    const size_t want = CONSOLE_UART_TXBUF;
  #endif
    console_txbuf_bytes = (uint32_t)want;
    // arduino-esp32 2.0.17: must precede begin(); returns 0 once the port runs.
    // want <= 128 (SOC_UART_FIFO_LEN) returns 128 and installs no ring: ok stays 0.
    const size_t got = port.setTxBufferSize(want);
    console_txbuf_ok = (want > 128 && got >= want) ? 1 : 0;
    if (console_txbuf_ok)
    {
        // R2-F2/R1: bytes still queued in the driver ring would be lost on a deliberate
        // restart. esp_restart() runs the shutdown handlers (IDF 4.4.7 esp_system.c);
        // console is UART0 (Serial = HardwareSerial(0)). Bounded 100 ms.
        esp_register_shutdown_handler(console_txbuf_shutdown);
    }
#endif
}

void esp32setup()
{
    ///< Initialize T5-EPAPER GUI
    ///< delay for ESP32-S3 nativ USB [OE3WAS]
    ///< um Terminal verbinden zu können
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE && defined(DISABLE_NET_CONSOLE)
    // CDC-02: must run BEFORE Serial.begin(). HWCDC::begin() (arduino-esp32
    // cores/esp32/HWCDC.cpp) creates the 256 B tx ring buffer and enables the
    // TX ISR; setTxBufferSize() on an already-begun HWCDC deletes and NULLs
    // that buffer before recreating it, and the ISR dereferences it without a
    // NULL check, so a live resize races the ISR and can assert/crash. Sizing
    // before begin() avoids the race. Full rationale at
    // MeshSerialClass::begin() (src/net_console.cpp).
    Serial.setTxBufferSize(4096);
#endif
#if defined(DISABLE_NET_CONSOLE) && !ARDUINO_USB_CDC_ON_BOOT
    // UART console: TX ring before the first begin() (with the net console
    // compiled in, MeshSerialClass::begin() does this for s_hwSerial).
    console_txbuf_apply(Serial);
#endif
    Serial.begin(MONITOR_SPEED);
    Serial.setTimeout(50);
    if(console_txbuf_bytes > 0)   // silent on boards where no ring applies (CDC)
        printfdeb("[MC-INFO] CONSOLE_TXBUF bytes=%lu ok=%d\n", (unsigned long)console_txbuf_bytes, console_txbuf_ok);

#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE && defined(DISABLE_NET_CONSOLE)
    // CDC-01: without the net-console wrapper Serial is the HWCDC object
    // itself; the rationale sits at MeshSerialClass::begin() (net_console.cpp).
    Serial.setTxTimeoutMs(0);
#endif
    
// 1.1.1??   pinMode(45,OUTPUT);
// 1.1.1??    digitalWrite(45, LOW);

    #if defined(BOARD_LED) && BOARD_LED_ACTIVE_LOW
        // Active-low LED (common anode): a floating pad can glow until the gated init below (after the settings
        // load). A pull-up holds it dark and does not load a OneWire/DHT bus on a shared pad.
        pinMode(BOARD_LED, INPUT_PULLUP);
    #endif

    // Wartet maximal 5 Sekunden auf den seriellen Monitor.
    // Achtung: delay() füttert den Task-Watchdog NICHT -- nur
    // esp_task_wdt_reset() tut das. Diese Schleife ist nur deshalb
    // unkritisch, weil esp32setup() bewusst nicht überwacht wird (N-25/S1).
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 5000)) {
        delay(10);
    }

    // Board LED pin init: after the settings load (LED-04, needs the OneWire state), see bUSER_BOARD_LED below

    #if defined(HAS_TFT_CONNECT)

        ledcAttachPin(SCREEN_BL, 1);
        ledcSetup(1, 2000, 8);
        ledcWrite(1, 255);

        gfx->begin();
    #endif

    #if defined BOARD_T5_EPAPER
        if (psramInit()) {
            printlndeb("\nThe PSRAM is correctly initialized");
        } else {
            printlndeb("\nPSRAM does not work");
        }
    #endif

    // Heltec V2: Enable Vext (GPIO 21) Heltec V3: Enable Vext (GPIO 36) to power OLED and reset display
    #if defined(BOARD_HELTEC) || defined(BOARD_HELTEC_V3)
        printlndeb(F("[INIT]...Enabling Vext for OLED power"));
        pinMode(Vext, OUTPUT);
        digitalWrite(Vext, LOW);   // Vext ON (active low)
        delay(50);

        // Reset OLED 
        printlndeb(F("[INIT]...Resetting OLED display"));
        pinMode(RST_OLED, OUTPUT);
        digitalWrite(RST_OLED, LOW);   // Reset active
        delay(50);
        digitalWrite(RST_OLED, HIGH);  // Reset release
        delay(50);
    #endif
    
    #if defined(BOARD_T3S3_V13)
        printlndeb("[INIT]...Enabling ADC_PIN");
        pinMode(ADC_PIN, INPUT);
    #endif

    #if not defined(BOARD_T_DECK_PRO)
        #if not defined(BOARD_T5_EPAPER) && not defined(BOARD_T_CONNECT_PRO)
            Wire.begin(I2C_SDA, I2C_SCL);
        #endif
        
        #ifdef PMU_USE_WIRE1
            Wire1.begin(I2C1_SDA, I2C1_SCL);
        #endif
    #endif

    delay(500);
    //======================================================
    // hier mal alles reingepackt bez. der GPOIs für BOARD_TBEAM_1W [OE3WAS]
    #if defined(BOARD_TBEAM_1W)

        SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

        #if defined(HAS_SDCARD) && defined(SD_SHARE_SPI_BUS)
            // Share spi bus with lora , set lora cs,rst to high
            pinMode(RADIO_CS_PIN, OUTPUT);
            pinMode(RADIO_RST_PIN, OUTPUT);
            digitalWrite(RADIO_CS_PIN, HIGH);
            digitalWrite(RADIO_RST_PIN, HIGH);
        #endif

        #ifdef FAN_CTRL
            pinMode(FAN_CTRL, OUTPUT);
            digitalWrite(FAN_CTRL, LOW);
        #endif

        #ifdef GPS_PPS_PIN
            pinMode(GPS_PPS_PIN, INPUT);
        #endif

    #endif
    //======================================================

    printlndeb("");
    printlndeb("");
    printlndeb("============");
    printlndeb("CLIENT SETUP");
    printlndeb("============");

    // TM-51: reset cause, raw Serial.printf so it survives --debug off (nRF52 prints
    // [BOOT] RESETREAS=... at the same point).
    {
        esp_reset_reason_t rr = esp_reset_reason();
        Serial.printf("[BOOT] RESET_REASON=%d %s\n", (int)rr, resetReasonName(rr));
        if (rr == ESP_RST_DEEPSLEEP)
        {
            esp_sleep_source_t wc = esp_sleep_get_wakeup_cause();
            Serial.printf("[BOOT] WAKE_CAUSE=%d %s\n", (int)wc, wakeCauseName(wc));
        }
    }
    // F3: where was the loop task when the previous boot ended? Unconditional
    // (raw Serial, not behind --debug), printed once, record cleared by the read.
    {
        char crumb[64];
        char summary[LOOPCRUMB_SUMMARY_LEN];
        esp_reset_reason_t rr = esp_reset_reason();
        int n = snprintf(summary, sizeof(summary), "RESET_REASON=%d %s", (int)rr, resetReasonName(rr));
        if (loopCrumbTakeReport(crumb, sizeof(crumb)))
        {
            Serial.println(crumb);
            const char *c = (strncmp(crumb, "[BOOT] ", 7) == 0) ? crumb + 7 : crumb;
            if (n > 0 && (size_t)n < sizeof(summary))
                snprintf(summary + n, sizeof(summary) - (size_t)n, " %s", c);
        }
        loopCrumbSetBootSummary(summary);   // shown by --info (net console readers)
    }
#if INSTRUMENT_ENABLED
    instrument_report_prev_boot();   // CDC-01: loop gaps of the previous boot, from RTC memory
    instrument_onrx_boot();          // R1-F: [INS] UARTTXBUF / ONRX_READY
#endif

    lFreeHeap =  ESP.getFreeHeap();
    lFreePsram = ESP.getFreePsram();

    if(!bDisplayLog)
    {
        printfdeb("[HEAP];%s;%lu;%d;%d;(init)\n", getTimeString().c_str(),
            lFreeHeap, ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
        printfdeb("[PSRM];%s;%lu\n", getTimeString().c_str(), lFreePsram);
    }
    
    check_efuse();

    // init nach Reboot
    init_loop_function();

	// Get LoRa parameter
	init_flash();

    bool bClear = false;
    if(meshcom_settings.node_cleanflash == 1)
        bClear = true;

    printfdeb("[INIT]...build %s / %s\n", __DATE__, __TIME__);

    // Geloescht wird nur bei echter Layout-Aenderung, nicht bei jedem neuen
    // Build-Datum. Siehe configuration_global.h.
    if(!flashLayoutCompatible(meshcom_settings.node_fversion) || bClear)
    {
        printfdeb("[INIT]...FLASH cleared, Settings-Layout %i -> %i\n",
                  meshcom_settings.node_fversion, FLASH_STRUCT_VERSION);

        initTimePersistence();

        clear_flash();

        // TZ-01 (D2): nur ein frischer oder geloeschter ESP32-Knoten bekommt die
        // MEZ/MESZ-Regel. Nicht in init_flash(): dessen Vorbelegung laeuft bei
        // jedem Boot, ein aktualisierter Knoten ohne NVS-Key "node_tz" bekaeme
        // sie sonst auch. Der save_settings() weiter unten schreibt sie.
        snprintf(meshcom_settings.node_tz, sizeof(meshcom_settings.node_tz), "%s", "CET-1CEST,M3.5.0,M10.5.0/3");
    }
    else
    {
        printfdeb("[INIT]...FLASH layout %i ok, build %i\n",
                  meshcom_settings.node_fversion, FLASH_VERSION);
    }

    if(bClear)
        init_flash();

    meshcom_settings.node_fversion = FLASH_STRUCT_VERSION;
    meshcom_settings.node_mversion = MODUL_HARDWARE;
    meshcom_settings.node_cleanflash = 0;
    snprintf(meshcom_settings.node_fwversion, sizeof(meshcom_settings.node_fwversion), "%-4.4s%-1.1s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    gwRegBoot();   // gateway registration state from settings (after the settings load)

#if defined(ENABLE_MSGSTORE)
    // S3: store node -- glue first (installs the MsgStoreEnv), then the persisted
    // --store/--storecall/--storetime/--storeslots/--storenotice settings.
    msgstoreGlueInit();
    msgstoreSettingsLoad();
#endif

    // BLK-01: call blocklist from flash/NVS (independent of ENABLE_MSGSTORE).
    blockListLoad();

    // RM-05 (#1189): remote-management state (high-water mark from its own NVS key), after the
    // settings load so node_call/node_passwd are final.
    rmInit();

    // "-0" und "-01" sind nicht die kanonische Schreibweise der SSID. Was aus
    // dem Flash kommt, wird deshalb einmal beim Start geradegezogen -- das
    // save_settings() darunter schreibt es ohnehin. Ein Rufzeichen ohne SSID
    // bleibt, wie es ist. Die Werkseinstellung bleibt unberuehrt, sie soll weiter als
    // "noch nicht konfiguriert" erkennbar sein.
    if(!isNodeUnconfigured(meshcom_settings.node_call))
    {
        String sOwnCall = meshcom_settings.node_call;

        if(normalizeOwnCall(sOwnCall) && strcmp(sOwnCall.c_str(), meshcom_settings.node_call) != 0)
        {
            printfdeb("[INIT]...Call <%s> -> <%s> (SSID kanonisch)\n", meshcom_settings.node_call, sOwnCall.c_str());

            snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s", sOwnCall.c_str());
            snprintf(meshcom_settings.node_short, sizeof(meshcom_settings.node_short), "%s", convertCallToShort(meshcom_settings.node_call).c_str());
        }
    }

    save_settings();

    bDisplayVolt = meshcom_settings.node_sset & 0x0001;
    bDisplayOff = meshcom_settings.node_sset & 0x0002;
    #if defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213)
    // WP / E213: Display-Aus-Zustand NICHT ueber Reboots persistieren. Der User-Button-Long-Press
    // ("--display off") wuerde das Panel sonst dauerhaft aus lassen - und weil E-Ink sein
    // Bild behaelt, sieht das aus wie "Display haengt am Startschirm" (kein Crash, nur nicht
    // aktualisiert). Daher beim Boot immer mit Display AN starten: ein Reboot holt die Anzeige
    // garantiert zurueck. Long-Press schaltet weiterhin zur Laufzeit aus/ein.
    bDisplayOff = false;
    #endif
    bPosDisplay = meshcom_settings.node_sset & 0x0004;
    bDEBUG = meshcom_settings.node_sset & 0x0008;
    bButtonCheck = meshcom_settings.node_sset & 0x0010;
    bDisplayTrack = meshcom_settings.node_sset & 0x0020;
    bGPSON =  meshcom_settings.node_sset & 0x0040;
    bBMPON =  meshcom_settings.node_sset & 0x0080;
    bBMEON =  meshcom_settings.node_sset & 0x0100;
    bLORADEBUG = meshcom_settings.node_sset & 0x0200;
    bSHORTPATH = meshcom_settings.node_sset & 0x0400;
    bBLElong = meshcom_settings.node_sset & 0x0800;
    bGATEWAY =  meshcom_settings.node_sset & 0x1000;
    bEXTUDP =  meshcom_settings.node_sset & 0x2000;
    bDisplayCont = meshcom_settings.node_sset & 0x4000;

    // TRK-01: Warnhinweis einmal beim Boot, wenn Track aus den Settings aktiv geladen wurde
    if(bDisplayTrack)
        printfdeb("[INIT]..." TRACK_WARNING_SERIAL "\n");

    bONEWIRE =  meshcom_settings.node_sset2 & 0x0001;
    bLPS33 =  meshcom_settings.node_sset2 & 0x0002;
    bBME680ON =  meshcom_settings.node_sset2 & 0x0004;
    bMCU811ON =  meshcom_settings.node_sset2 & 0x0008;
    //bGPSDEBUG = meshcom_settings.node_sset2 & 0x0010; // new variale
    iGPSDEBUG = meshcom_settings.node_gpsdebug;

    bMESH = !(meshcom_settings.node_sset2 & 0x0020);
    bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
    bWIFIAP = meshcom_settings.node_sset2 & 0x0080;
    bGATEWAY_NOPOS =  meshcom_settings.node_sset2 & 0x0100;
    bSMALLDISPLAY =  false;
    bSOFTSERREAD = meshcom_settings.node_sset2 & 0x0200;
    bSOFTSERON =  meshcom_settings.node_sset2 & 0x0400;
    bBOOSTEDGAIN =  meshcom_settings.node_sset2 & 0x0800;
    bNETCONSOLE = meshcom_settings.node_sset2 & 0x1000;
    #ifndef DISABLE_NET_CONSOLE
    netConsoleSetPassword(meshcom_settings.node_passwd);
    #endif
    #ifndef DISABLE_KISS_TCP
    kissSetPassword(meshcom_settings.node_passwd);
    #endif
    // NOTE: externalRadioSetup() runs later, AFTER lora_setcountry() has applied
    // the effective freq/bw/sf/cr/preamble, so the bridge config snapshot is built
    // from the same effective radio settings the local radio path would use.
    //xxxxxxx   = meshcom_settings.node_sset2 & 0x2000;
    bVIA = meshcom_settings.node_sset2 & 0x4000;

    // nicht mehr notwendig bMHONLY =  bMHONLY =  meshcom_settings.node_sset3 & 0x0001;
    bCHARGE_LED = chargeLedAutoFromSettings(meshcom_settings.node_sset2, meshcom_settings.node_sset3);   // CHG-01/02: sset3 0x0001 counts only with the sset2 0x8000 marker
    bNoMSGtoALL =  meshcom_settings.node_sset3 & 0x0002;
    bBLEDEBUG = meshcom_settings.node_sset3 & 0x0004;
    bAnalogCheck = meshcom_settings.node_sset3 & 0x0008;
    bBMP3ON = meshcom_settings.node_sset3 & 0x0010;
    bAHT20ON = meshcom_settings.node_sset3 & 0x0020;
    bAnalogFilter = meshcom_settings.node_sset3 & 0x0040;
    bUSER_BOARD_LED = meshcom_settings.node_sset3 & 0x0080;

    #ifdef BOARD_LED
        // Safe Hardware Initalization. LED-04: skipped while OneWire/DHT owns the pad (T-Beam: GPIO4)
        if (boardLedPinUsable())
        {
            // Pre-load the safe execution state into the output register
            // This prevents the LED from briefly flashing/glitching
            // when changing the pin mode
            digitalWrite(BOARD_LED, BOARD_LED_ACTIVE_LOW);
            // configure the pin driver as active output:
            pinMode(BOARD_LED, OUTPUT);
        }
    #endif

    bSOFTSERDEBUG = meshcom_settings.node_sset3 & 0x0100;
    bWXDEBUG = meshcom_settings.node_sset3 & 0x0200;
    bSHT21ON = meshcom_settings.node_sset3 & 0x0400;
    bINA226ON = meshcom_settings.node_sset3 & 0x0800;
    bGPSAutosymbol = meshcom_settings.node_sset3 & 0x1000;
    bGPSUBLOX = meshcom_settings.node_sset3 & 0x2000;
    bGPSL76K = meshcom_settings.node_sset3 & 0x4000;

    bDEBUGCSV = meshcom_settings.node_sset4 & 0x0001;
    bDEBUGEN = meshcom_settings.node_sset4 & 0x0002;
    bDisplayLog = meshcom_settings.node_sset4 & 0x0004;
    bTXCAPTURE = meshcom_settings.node_sset4 & 0x0008;
    bNBRDEBUG = meshcom_settings.node_sset4 & 0x0400;
    nbrDebugApply();
    // --nbrrelay off|count|on (Stufe 2): 0x0800 rechnen/zaehlen, 0x1000 anwenden.
    bNBRRELAY  = (meshcom_settings.node_sset4 & 0x1800) != 0;
    bNBRCANCEL = (meshcom_settings.node_sset4 & 0x1000) != 0;
    // --nbrsym on|off (Stufe 2): 0x2000 invertiert gespeichert, siehe command_functions.cpp.
    bNBRSYM = (meshcom_settings.node_sset4 & 0x2000) == 0;
    // --nbrreport off|auto|on (Stufe 3, HN-Bericht): 0x0100 off, 0x0200 on, keines von beiden auto.
    bNBRRPTOFF = (meshcom_settings.node_sset4 & 0x0100) != 0;
    bNBRRPTON  = (meshcom_settings.node_sset4 & 0x0200) != 0;
    #ifndef DISABLE_KISS_TCP
    bKISS = meshcom_settings.node_sset4 & 0x0010;
    bKISSTX = meshcom_settings.node_sset4 & 0x0020;
    bKISSMETA = meshcom_settings.node_sset4 & 0x0040;
    bKISSAUTH = meshcom_settings.node_sset4 & 0x0080;
    #endif

    if(strlen(meshcom_settings.node_aprsmc) < 4)
    {
        strcpy(meshcom_settings.node_aprsmc, (char*)"APRSMC");  // default
    }

    // Time from Flash
    loadTimePersistence();
    snprintf(cTimeSource, sizeof(cTimeSource), (char*)"INIT");
    memset(meshcom_settings.node_update, 0x00, sizeof(meshcom_settings.node_update));

    #ifdef BUTTON_PIN
        iButtonPin = BUTTON_PIN;
    #else
        iButtonPin = 99;
    #endif

    if(meshcom_settings.node_button_pin > 0)
        iButtonPin = meshcom_settings.node_button_pin;

    #if defined(ENABLE_AUDIO)
        init_audio();
    #endif

    // Initialize T-Deck GUI
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        initTDeck();
        if(bGPSON)
            addMessage("GPS enabled");
        else
            addMessage("GPS disabled");
    #endif

    // Initialize T-Deck Pro GUI
    #if defined(BOARD_T_DECK_PRO)
        initTDeck_pro();
    #endif

    // MeshCom 5 (docs/meshcom5-campaign.md Welle 4): /topo.dat laden. Erst hier,
    // nach init_flash() (Rufzeichen, node_persist_to_sd) und nach initTDeck()/
    // initTDeck_pro(), die die SD-Karte einhaengen; auf allen anderen Boards ein
    // No-Op (src/topo_ui.h).
    topoUiBoot();

    #if defined(BOARD_T5_EPAPER)
        idf_setup();
    #endif

    // if Node not set --> WifiAP Mode on
    if(isNodeUnconfigured(meshcom_settings.node_call))
    {
        bWIFIAP = true;
        printlndeb("[INIT]...WIFIAP starting...");
    }

    if(bWIFIAP)
    {
        bWEBSERVER = true;
        printlndeb("[INIT]...WEBServer starting...");
    }

    #if defined(BOARD_T_DECK_PRO)
        bWEBSERVER=false;
        bDEBUG=true;
        bGPSON=false;
    #endif


    if(meshcom_settings.node_gwsrv[0] == 0x00)
        snprintf(meshcom_settings.node_gwsrv, sizeof(meshcom_settings.node_gwsrv), "%s", "OE");

    // if Node is in WifiAP Mode -> no Gateway posible
    if(bWIFIAP && bGATEWAY)
    {
        bGATEWAY=false;
        bEXTUDP=false;
    }

    // keine WIFI Connection
    if(strlen(meshcom_settings.node_pwd) == 0 && strlen(meshcom_settings.node_ssid) == 0)
    {
        bGATEWAY=false;

        if(!bWIFIAP)
            bWEBSERVER=false;
    }
       
    if(bBMPON)
    {
        bBMEON=false;
        bBME680ON=false;
        bmx_found=false;
    }
    else
    if(bBMEON)
    {
        bBMPON=false;
        bBME680ON=false;
        bmx_found=false;
    }

    bme680_found=false;

    bDisplayInfo = bLORADEBUG;

    // CS-01: max_hop_text is user-settable (--maxhop) and comes out of NVS now;
    // only a missing or out-of-range value falls back to the compile default.
    // max_hop_pos stays a compile-time constant on purpose (operator, 2026-08-30).
    meshcom_settings.max_hop_text = maxHopTextSanitize(meshcom_settings.max_hop_text);
    meshcom_settings.max_hop_pos = MAX_HOP_POS_DEFAULT;


    #if defined(BOARD_E22_S3) || defined(BOARD_TBEAM_1W)
        fBattFaktor = 1;   // default Korrekturfaktor
        if(meshcom_settings.node_analog_batt_faktor > 0.0)
            fBattFaktor = meshcom_settings.node_analog_batt_faktor;
    #endif

    // Initialize battery reading
    #if defined(DISABLE_BATTERY)   // opt-out -D DISABLE_BATTERY: board without a battery divider
    battProbeState = BATT_PROBE_NONE;   // no battery measurement on this board
    #elif not defined (BOARD_T_DECK_PRO)
	init_batt();
    #endif

    #ifdef VEXT_CTRL
        pinMode(VEXT_CTRL, OUTPUT);
        #if defined(BOARD_TRACKER)
            digitalWrite(VEXT_CTRL, HIGH);   // HWT needs this for GPS and TFT Screen
        #endif
    #endif

    #ifdef ADC_CTRL
        pinMode(ADC_CTRL, OUTPUT);
        #if defined(BOARD_TRACKER)
            digitalWrite(ADC_CTRL, HIGH);
        #endif
    #endif
        
    #ifdef LED_PIN
        pixels.begin();
        printlndeb("[INIT]...NEOPIXEL set");
    #endif

    #ifndef ENABLE_SOFTSER
        bSOFTSERON=false;
    #endif

    // Umstellung auf langes WIFI Passwort
    if(strlen(meshcom_settings.node_ossid) > 0 && (strlen(meshcom_settings.node_ssid) == 0 || is_equ(meshcom_settings.node_ssid, "none")))
    {
        strncpy(meshcom_settings.node_ssid, meshcom_settings.node_ossid, sizeof(meshcom_settings.node_ssid) - 1);
        meshcom_settings.node_ssid[sizeof(meshcom_settings.node_ssid) - 1] = '\0';
        strncpy(meshcom_settings.node_pwd, meshcom_settings.node_opwd, sizeof(meshcom_settings.node_pwd) - 1);
        meshcom_settings.node_pwd[sizeof(meshcom_settings.node_pwd) - 1] = '\0';

        memset(meshcom_settings.node_ossid, 0x00, sizeof(meshcom_settings.node_ossid));
        memset(meshcom_settings.node_opwd, 0x00, sizeof(meshcom_settings.node_opwd));

        save_settings();
    }

    global_batt = 4200.0;

    posinfo_interval = POSINFO_INTERVAL;

    if(meshcom_settings.node_postime > 0)
        posinfo_interval = meshcom_settings.node_postime;

    meshcom_settings.node_press = 0.0;
    meshcom_settings.node_hum = 0.0;
    meshcom_settings.node_temp = 0.0;
    meshcom_settings.node_temp2 = 0.0;
    meshcom_settings.node_gas_res = 0.0;
    meshcom_settings.node_press_alt = 0;
    meshcom_settings.node_press_asl = 0.0;

    if(meshcom_settings.node_maxv > 0)
    {
        setMaxBatt(meshcom_settings.node_maxv * 1000.0F);
        global_batt = meshcom_settings.node_maxv * 1000.0F;
    }
    else
    {
        #ifdef BAT_MAX_VOLTAGE
        setMaxBatt(BAT_MAX_VOLTAGE * 1000.0F);
        global_batt = BAT_MAX_VOLTAGE * 1000.0F;
        #endif
    }

    if(memcmp(meshcom_settings.node_ssid, DEFAULT_CALL_PREFIX, 6) == 0)
    {
        snprintf(meshcom_settings.node_ssid, sizeof(meshcom_settings.node_ssid), (char*)"none");
        snprintf(meshcom_settings.node_pwd, sizeof(meshcom_settings.node_pwd), (char*)"none");
    }

    #ifdef MODUL_FW_TBEAM
        if(meshcom_settings.node_sset == 0x0004)
        {
            bButtonCheck = true;
            bGPSON = true;          // TRK-03: GPS schon beim ersten Start, nicht erst nach dem Reboot
            bDisplayVolt = true;
            meshcom_settings.node_sset = meshcom_settings.node_sset | TBEAM_FIRST_BOOT_SSET;	// TRK-03
            save_settings();
        }
    #endif

    #ifdef SET_BUTTON_PIN
        if(meshcom_settings.node_sset == 0x0004)
        {
            bButtonCheck = true;
            meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0010;
            save_settings();
        }
    #endif

    _GW_ID = getMacAddr();

    #ifdef BOARD_HELTEC_V3
        SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
    #endif

    #ifdef BOARD_STICK_V3
        SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
    #endif

    #ifdef BOARD_HELTEC_V4
        SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
        initPAControl();
    #endif

    #if defined(USING_SX1262) && not defined(BOARD_TBEAM_1W) && not defined(BOARD_T3S3_V13) && not defined(BOARD_T_CONNECT_PRO)
        // SPI-Bus für das Shield initialisieren
        SPI.begin(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_CS_PIN);
    #endif

    #if defined(ENABLE_GPS)
        // TBEAM variants
        #if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
            setupPMU();
        #endif

    #else

        #if !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T5_EPAPER)
            setupPMU();
        #endif

    #endif

    #if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
       setChargeLedOn(!!bCHARGE_LED); // Initialize the charge indicator LED after boot and setupPMU()
    #endif

    #if defined(ENABLE_BMX280)
        setupBMX280(true);
    #endif

    #if defined(ENABLE_BMP390)
        setupBMP390(true);
    #endif

    #if defined(ENABLE_AHT20)
        setupAHT20(true);
    #endif

    #if defined(ENABLE_SHT21)
        setupSHT21(true);
    #endif

    #if defined(ENABLE_MC811)
        setupMCU811();
    #endif

    // BME680
    #if defined(ENABLE_BMX680)
        setupBME680();
    #endif

    // MCP23017
    #if defined(ENABLE_MCP23017)
        setupMCP23017();
    #endif

    // INA226
    #if defined(ENABLE_INA226)
        setupINA226();
    #endif

    // RTC
    #if defined(ENABLE_RTC)
        setupRTC();
    #endif

    // SOFTSER
    #if defined(ENABLE_SOFTSER)
        setupSOFTSER();
    #endif

    // ANALOG
    #if defined (ANALOG_PIN)
        initAnalogPin();
    #endif

    // Initialize temp sensor
    #ifdef OneWire_GPIO
    if(bONEWIRE)
    {
        init_onewire_ds18();
        init_onewire_dht();
    }
    #endif

    init_onebutton();

    printfdeb("[INIT].._GW_ID: %08X\n", _GW_ID);

 
    ////////////////////////////////////////////////////////////////////
    // Initialize time
	bool boResult;
	
	// initialize clock
	boResult = MyClock.Init();
	printfdeb("[INIT]...Initialize clock: %s\n", (boResult) ? "ok" : "FAILED");
    snprintf(cTimeSource, sizeof(cTimeSource), (char*)"INIT");

    DisplayTimeWait=0;
    //
    ////////////////////////////////////////////////////////////////////

    #if defined(BOARD_E22) || defined(BOARD_E220) || defined(BOARD_E22_S3)
        // if RESET Pin is connected
        pinMode(LORA_RST, PULLUP);
        digitalWrite(LORA_RST, LOW);
        delay(200);
        digitalWrite(LORA_RST, HIGH);
    #endif

    #if defined(BOARD_E22) || defined(BOARD_E220) || defined(BOARD_E22_S3)
        #if defined(E22_RXEN) && defined(E22_TXEN)
            radio.setRfSwitchPins(E22_RXEN, E22_TXEN);
        #endif
    #endif
    
    #if defined(BOARD_T5_EPAPER)
    //
    #elif defined(BOARD_T_DECK_PRO)

    #elif defined(BOARD_T_CONNECT_PRO)
    //
    #elif defined(HAS_TFT)
        initTFT();
        heapMarkTft();
    #else
        initDisplay();
    #endif

    #if defined(BOARD_HELTEC_V3) || defined(BOARD_STICK_V3)
    delay(500);
    #endif

    #if defined (BOARD_T5_EPAPER)
    //
    #elif defined (BOARD_T_DECK_PRO)
    //
    #elif defined(HAS_TFT) || defined(BOARD_T_CONNECT_PRO)
        char cvers[22];
        snprintf(cvers, sizeof(cvers), "FW %s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
        String  version = cvers;
        displayTFT("MeshCom 4.0", version, "@BY ICSSW.ORG", "OE1KBC, OE1KFR",  "...starting now", 5000);
    #else
        startDisplay((char*)"...starting now", (char*)"@BY ICSSW.ORG", (char*)"OE1KBC, OE1KFR");
    #endif

    //LORA CHIP present
    bRadio = false;

    //TEST #ifndef BOARD_E290

    #ifdef SX127X
    printdeb(F("[LoRa]...SX1276 Chip"));
    #endif
    
    #ifdef BOARD_E220
    printdeb(F("[LoRa]...LLCC68 chip"));
    #endif

    #ifdef SX1262X
    printdeb(F("[LoRa]...SX1262 chip"));
    #endif
    
    #ifdef SX126X
    printdeb(F("[LoRa]...SX1268 chip"));
    #endif
    
    #ifdef SX1262_V3
    printdeb(F("[LoRa]...SX1262 V3 chip"));
    #endif
    
    #if defined(SX1262_E22) || defined(USING_SX1262)
    printdeb(F("[LoRa]...SX1262 V3 chip"));
    #endif

    #ifdef SX1268_E22
    printdeb(F("[LoRa]...SX1268 V3 chip"));
    #endif

    #ifdef SX1262_E290
    printdeb(F("[LoRa]...SX1262 E290 chip"));
    #endif

    #ifdef SX1262_V4
    printdeb(F("[LoRa]...SX1262 V4 chip (with PA)"));
    #endif

    #if defined (BOARD_TRACKER) || defined(BOARD_TBEAM_1W) || defined(BOARD_T3S3_V13)
        SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    #endif

    #if defined(BOARD_T5_EPAPER)
    // extra source
    // #elif defined(BOARD_T_DECK_PRO)
    // extra source
    #elif defined(BOARD_E220)
        printdeb(F(" Initializing ... "));
        int state = radio.begin(434.0F, 125.0F, 9, 7, SYNC_WORD_SX127x, 10, LORA_PREAMBLE_LENGTH, /*float tcxoVoltage = 0*/ 1.6F, /*bool useRegulatorLDO = false*/ false);
    #else
        printdeb(F(" Initializing ... "));

        #if defined(BOARD_TBEAM_1W)
        #ifdef RADIO_LDO_EN
            // Issue 962 deepsleep (esp32_sleep.cpp): --deepsleep holds this
            // pin low with gpio_hold_en()/gpio_deep_sleep_hold_en() so it
            // doesn't float during sleep. That hold survives the wake reset
            // (esp_idf gpio.h), so the digitalWrite(HIGH) below would be
            // silently ignored without releasing it first.
            gpio_hold_dis((gpio_num_t) RADIO_LDO_EN);
            gpio_deep_sleep_hold_dis();

            // T-BEAM-1W Control SX1262, LNA, must set RADIO_LDO_EN to HIGH to power the Radio
            pinMode(RADIO_LDO_EN, OUTPUT);
            digitalWrite(RADIO_LDO_EN, HIGH);
            delay(200);
        #endif

        #ifdef RADIO_CTRL
            // T-BEAM-1W LoRa RX/TX Control. RADIO_CTRL controls the LNA, not the PA.
            // Only when RX DATA is on, set to 1 to turn on LNA.
            // When TX DATA is on, RADIO_CTRL is set to 0 and LNA is turned off.
            pinMode(RADIO_CTRL, OUTPUT);
            digitalWrite(RADIO_CTRL, HIGH);  // RX Mode
            delay(200);
        #endif
        #endif

        #if defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E213)
            // DS-02: prepareToSleep() in src/Platforms/<board>/power_controls.cpp
            // holds PIN_LORA_NSS HIGH with gpio_hold_en() before deep sleep.
            // ESP-IDF gpio.h: the hold survives the deep-sleep wake reset and is
            // released only by gpio_hold_dis(). Without this the SPI chip-select
            // can never go LOW after the first wake and the radio is dead until a
            // power cycle. GPIO8 is RTC-capable but the sleep side never arms
            // gpio_deep_sleep_hold_en(), so no gpio_deep_sleep_hold_dis() is needed.
            // No-op on a cold boot. Field-verified 2026-09-11 by OE3LCR on
            // E213 and Wireless Paper V1.2 (PR #1135 comment): EXT1 wake, radio
            // init and SPI traffic fine. Pre-fix 4.35p also woke with working
            // RX there, so in practice this is a guard rather than a repair.
            gpio_hold_dis((gpio_num_t) PIN_LORA_NSS);
        #endif

        #if defined(EXTERNAL_RADIO)
        // External radio: the bridge owns the RF chip. Do NOT initialize/begin the
        // local RadioLib transceiver. bRadio is forced false below so no local
        // RX/CAD/TX path runs.
        int state = RADIOLIB_ERR_NONE;
        (void)state;
        #elif defined(BOARD_T_ETH_ELITE)
        int state = radio.begin(433.175);
        radio.setDio2AsRfSwitch(true);
        radio.setTCXO(1.8);
        #else
        int state = radio.begin(433.175);
        #endif

    #endif
    
    #if defined(BOARD_T5_EPAPER)
    // extra source
    // #elif defined(BOARD_T_DECK_PRO)
    // extra source
    #else
    if (state == RADIOLIB_ERR_NONE)
    {
        printlndeb(F("success"));
        bRadio=true;

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_addMessage(true);
        #endif
    }
    else
    {
        printdeb(F("failed, code "));
        printlndeb(state);
        bRadio=false;

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_addMessage(false);
        #endif
    }
    #endif

    #if defined(BOARD_E220)
        bRadio = false; // no detailed setting
    #endif

    #if defined(EXTERNAL_RADIO)
        // External radio: no local RF chip is initialized or driven. Forcing
        // bRadio=false makes the local radio config block and the whole local
        // RX/CAD/TX loop section no-ops; the bridge transport (externalRadioLoop)
        // owns RX/TX instead.
        bRadio = false;
    #endif

    //#if not defined(BOARD_T_DECK_PRO)
    // extra source
    // > 4.34w we use EU8 instead of EU
    if(meshcom_settings.node_country == 0)
        meshcom_settings.node_country = 8;

    #ifdef BOARD_COUNTRY
        meshcom_settings.node_country = BOARD_COUNTRY;
    #endif

    lora_setcountry(meshcom_settings.node_country);

    #if defined(EXTERNAL_RADIO)
    // The local-radio path normalizes the -20 "use default" power sentinel to the
    // board default inside its bRadio block, which is skipped in external mode.
    // Apply the same default here so the bridge snapshot uses the intended default
    // power (getPower() then clamps to the valid range) instead of the clamped
    // sentinel. Mirrors the bRadio-block normalization; RAM-only (no extra flash
    // write — a later --txpower change persists normally).
    if(meshcom_settings.node_power == -20)
        meshcom_settings.node_power = TX_OUTPUT_POWER;

    // Build the bridge RadioConfig snapshot from the EFFECTIVE radio settings:
    // lora_setcountry() above has just normalized freq/bw/sf/cr/preamble to the
    // active country. Taking the snapshot here (rather than before lora_setcountry)
    // ensures the bridge is configured with the same effective values the local
    // radio path would use, including on first boot / country change / flash clear.
    // Runtime setting changes are still re-synced via
    // lora_setchip_meshcom() -> externalRadioConfigChanged().
    externalRadioSetup();
    #endif

    //#endif

    // you can also change the settings at runtime
    // and check if the configuration was changed successfully
    #if defined(BOARD_T5_EPAPER)
    // extra source
    // #elif defined(BOARD_T_DECK_PRO)
    // extra source
    #else
    if(bRadio)
    {
        #if defined(BOARD_WIRELESS_PAPER) || defined(BOARD_E290) || defined(BOARD_E213)
        if(radio.setTCXO(1.8) != RADIOLIB_ERR_NONE)
            printlndeb(F("[LoRa]...TCXO 1.8V konnte nicht gesetzt werden"));
        else
            printlndeb(F("[LoRa]...TCXO DIO3 = 1.8V (Wireless Paper / E213)"));
        #endif

        // set boosted gain
        #if defined(SX1262_V3) || defined(SX126x_V3) || defined(SX1262_E290) || defined(SX1262X) || defined(SX126X) || defined(USING_SX1262) || defined(SX1262_V4)
        printfdeb("[LoRa]...RX_BOOSTED_GAIN: %d\n", (meshcom_settings.node_sset2 &  0x0800) == 0x0800);
        if (radio.setRxBoostedGainMode(bBOOSTEDGAIN)  != RADIOLIB_ERR_NONE ) {
            printlndeb(F("Boosted Gain is not available for this module!"));
        }
        #endif

        // set carrier frequency
        printfdeb("[LoRa]...RF_FREQUENCY: %.3f MHz\n", meshcom_settings.node_freq);
        if (radio.setFrequency(meshcom_settings.node_freq) == RADIOLIB_ERR_INVALID_FREQUENCY) {
            printlndeb(F("Selected frequency is invalid for this module!"));
        }
        else
        {
            if(meshcom_settings.node_freq > 440)
                printlndeb(F("[LoRa]...SAT Only Frequency out of Band"));
        }

        // set bandwidth 
        printfdeb("[LoRa]...RF_BANDWIDTH: %.0f kHz\n", meshcom_settings.node_bw);
        if (radio.setBandwidth(meshcom_settings.node_bw) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
            printlndeb(F("Selected bandwidth is invalid for this module!"));
        }

        // set spreading factor 
        printfdeb("[LoRa]...RF_SF: %i\n", meshcom_settings.node_sf);
        if (radio.setSpreadingFactor(meshcom_settings.node_sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
            printlndeb(F("Selected spreading factor is invalid for this module!"));
        }

        // set coding rate 
        printfdeb("[LoRa]...RF_CR: 4/%i\n", meshcom_settings.node_cr);
        if (radio.setCodingRate(meshcom_settings.node_cr) == RADIOLIB_ERR_INVALID_CODING_RATE) {
            printlndeb(F("Selected coding rate is invalid for this module!"));
        }

        // set LoRa sync word 
        // NOTE: value 0x34 is reserved for LoRaWAN networks and should not be used
        if (radio.setSyncWord(SYNC_WORD_SX127x) != RADIOLIB_ERR_NONE) {
            printlndeb(F("Unable to set sync word!"));
        }

        // set output power to 10 dBm (accepted range is -3 - 17 dBm)
        // NOTE: 20 dBm value allows high power operation, but transmission
        //       duty cycle MUST NOT exceed 1%
        
        int8_t tx_power = TX_OUTPUT_POWER;
        
        if(meshcom_settings.node_power == -20)
            meshcom_settings.node_power = TX_OUTPUT_POWER;
        else
            tx_power=meshcom_settings.node_power;   //set by command

        if(tx_power > TX_POWER_MAX)
            tx_power= TX_POWER_MAX;

        if(tx_power < TX_POWER_MIN)
            tx_power= TX_POWER_MIN;

        meshcom_settings.node_power = tx_power;  // [OE3WAS] für den Ring der Sicherheit ;-) ??
        save_settings();
        printfdeb("[LoRa]...RF_POWER: %d dBm\n", tx_power);

        #if defined(BOARD_T_DECK_PRO)
        if (radio.setOutputPower(tx_power) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
            printlndeb(F("Selected output power is invalid for this module!"));
            Serial.printf("[LoRa] radio config failed, restarting\n");
            esp_restart();
        }
        #else
        if (radio.setOutputPower(tx_power, false) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
            printlndeb(F("Selected output power is invalid for this module!"));
            Serial.printf("[LoRa] radio config failed, restarting\n");
            esp_restart();
        }
        #endif

        // set over current protection limit (accepted range is 0 - 140 mA)
        // NOTE: set value to 0 to disable overcurrent protection
        if (radio.setCurrentLimit(CURRENT_LIMIT) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
            printlndeb(F("Selected current limit is invalid for this module!"));
            Serial.printf("[LoRa] radio config failed, restarting\n");
            esp_restart();
        }

        // set LoRa preamble length to 15 symbols (accepted range is 6 - 65535)
        printfdeb("[LoRa]...PREAMBLE: %i symbols\n", meshcom_settings.node_preamplebits);
        
        if (radio.setPreambleLength(meshcom_settings.node_preamplebits) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
            printlndeb(F("Selected preamble length is invalid for this module!"));
            Serial.printf("[LoRa] radio config failed, restarting\n");
            esp_restart();
        }

        #if defined(SX127X)
        // set amplifier gain  (accepted range is 1 - 6, where 1 is maximum gain)
        // NOTE: set value to 0 to enable automatic gain control
        //       leave at 0 unless you know what you're doing
        if (radio.setGain(0) == RADIOLIB_ERR_INVALID_GAIN)
        {
            printlndeb(F("Selected gain is invalid for this module!"));
            Serial.printf("[LoRa] radio config failed, restarting\n");
            esp_restart();
        }

        // set the function that will be called
        // when LoRa preamble is not detected within CAD timeout period
        // or when a packet is received
        
        // set Receive Interupt
        bEnableInterruptReceive = true; //KBC 0801
        radio.setPacketReceivedAction(setFlagReceive); //KBC 0801

        //KBC 0801 radio.setPacketSentAction(setFlagSent);

        //KBC 0801 radio.setDio0Action(setFlagReceive, RISING);
        //KBC 0801 radio.setDio1Action(setFlagSent, RISING);

        // set the function that will be called
        // when LoRa preamble is detected
        // radio.setDio1Action(setFlagDetected, RISING);

        // start scanning the channel
        printdeb(F("[LoRa]...Starting to listen ... "));
        state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        if (state == RADIOLIB_ERR_NONE)
        {
            printlndeb(F("success"));
        }
        else
        {
            printdeb(F("failed, code "));
            printlndeb(state);
        }        

        // enable CRC
        if (radio.setCRC(true) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
        {
            printlndeb(F("Selected CRC is invalid for this module!"));
            Serial.printf("[LoRa] radio config failed, restarting\n");
            esp_restart();
        }

        #endif

        // setup for SX126x, 1268_V3 Radios
        #if defined(SX126X) || defined(SX126x_V3) || defined(SX1262X) || defined(USING_SX1262)
        // set the function that will be called
        // when LoRa preamble is not detected within CAD timeout period
        // or when a packet is received
        radio.setPacketReceivedAction(setFlagReceive);
        radio.setPacketSentAction(setFlagSent);

        radio.setDio1Action(setFlagSent);

        // set the function that will be called
        // when LoRa preamble is detected
        // radio.setDio1Action(setFlagDetected, RISING);

        // start scanning the channel
        printdeb(F("[LoRa]...Starting to listen ... "));
        #ifdef RADIO_CTRL
            digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
            delay(2);
        #endif
        state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        if (state == RADIOLIB_ERR_NONE)
        {
            printlndeb("success");
        }
        else
        {
            printdeb("failed, code ");
            printdeb(state);
        }        

        // enable CRC
        if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
        {
            printlndeb(F("Selected CRC is invalid for this module!"));
            Serial.printf("[LoRa] radio config failed, restarting\n");
            esp_restart();
        }
        #endif

        #if defined(SX1262_V3) || defined(SX1262_E290) || defined(SX1262_V4)

            // set Receive Interupt
            bEnableInterruptReceive = true; //KBC 0801
            radio.setPacketReceivedAction(setFlagReceive); //KBC 0801
            
            //KBC 0801 radio.setPacketSentAction(setFlagSent);

            #if defined (BOARD_TRACKER)
                radio.setDio1Action(setFlagSent);
            #endif

            // start scanning the channel
            printdeb(F("[LoRa]...Starting to listen ... "));
            state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
            if (state == RADIOLIB_ERR_NONE)
            {
                printlndeb("success");
            }
            else
            {
                printdeb(F("failed, code "));
                printlndeb(state);
            }        
    
            // enable CRC
            if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
            {
                printlndeb("Selected CRC is invalid for this module!");
                Serial.printf("[LoRa] radio config failed, restarting\n");
                esp_restart();
            }
        #endif

        // setup for E220 Radios
        #if defined(BOARD_E220)

            // interrupt pin
            radio.setDio1Action(setFlag);

            // start scanning the channel
            printdeb(F("[E220] Starting to listen ... "));
            state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
            if (state == RADIOLIB_ERR_NONE)
            {
                printlndeb("success!");
            }
            else
            {
                    printdeb(F("failed, code "));
                    printlndeb(state);
            }        

            // enable CRC
            if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
                printlndeb(F("Selected CRC is invalid for this module!"));
                Serial.printf("[LoRa] radio config failed, restarting\n");
                esp_restart();
            }

        #endif
    }

    printlndeb(F("[LoRa]...All settings successfully changed"));
    #endif

    //#endif

    if(meshcom_settings.node_call[0] == 0x00)
    {
        snprintf(meshcom_settings.node_call, sizeof(meshcom_settings.node_call), "%s", (char*)DEFAULT_CALL);
    }

    // Create the BLE Device & WiFiAP
    snprintf(cBLEName, sizeof(cBLEName), "M%s-%02x%02x-%s", g_ble_dev_name, dmac[1], dmac[0], meshcom_settings.node_call);
    snprintf(s_bleManuf, sizeof(s_bleManuf), "MCM%s-%02x%02x-%s", g_ble_dev_name,  dmac[1], dmac[0], meshcom_settings.node_call);


    const std::__cxx11::string strBLEName = cBLEName;

    printfdeb("[BLE ]...Device started with BLE-Name <%s>\n", strBLEName.c_str());

    bleQueue = xQueueCreate(5, sizeof(BleQueueItem));

    // Phone rings on the heap (classic ESP32 with the AU BLE pause only; no-op elsewhere). Allocated
    // with BLE start, released by esp32BlePause(). A failure here leaves the rings empty (frames for
    // the phone are dropped); BLE itself still starts.
    if(!phoneRingsAttach())
        printfdeb("[BLE ]...phone rings: no memory, phone data dropped until next boot\n");

    #if !defined(DISABLE_BLE)   // opt-out -D DISABLE_BLE: board without a usable BLE controller
    s_bleLife = xSemaphoreCreateMutex();   // AU-10: guards the BLE objects against pause / resume
    #if defined(BENCH_BLE_ADV_LATE)
    const bool bleAdvNow = false;
    #else
    const bool bleAdvNow = true;
    #endif
    bleStackStart(bleAdvNow);   // the same function the AU-10 resume uses
    heapMarkBle();
    #else
    printfdeb("[BLE ]...disabled (DISABLE_BLE)\n");
    #endif
    
    // reset GPS-Time parameter
    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    // Start Audio on T-Deck
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    // Only start audio if not muted, to avoid unnecessary task creation
    if (!meshcom_settings.node_mute) {
        startAudio();
    }
    #endif

    printlndeb("==============");
    printlndeb("CLIENT STARTED");
    printlndeb("==============");
    delay(500);

    ///////////////////////////////////////////////////////
    // WIFI
    // AU-05 (#1187): the firmware auto update needs the STA link on its own, independent of the gateway flag.
    // D15: for AU alone only with an SSID set and not in AP mode (wifi_start_gate.h)
    if(wifiStartWanted(bGATEWAY, bEXTUDP, bWEBSERVER, bNETCONSOLE, meshcom_settings.node_autoupd, bWIFIAP, meshcom_settings.node_ssid))
    {
        bAllStarted=false;

        if(!startNetwork())
        {
            printlndeb("[WIFI]...no connection possible");
            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            addMessage("WiFi connection failed");
            #endif
        }
        else
        {
            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            addMessage("WiFi connected");
            #endif
        }
    }
    else
    {
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        addMessage("WiFi deactivated");
        #endif
    }
    //
    ///////////////////////////////////////////////////////


    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_clear_text_ta();
    #endif

    #if defined(RELAY_SWITCH)
        pinMode(RELAY_SWITCH, OUTPUT);
        if((meshcom_settings.node_relay & 0x0001) == 0x0001)
            digitalWrite(RELAY_SWITCH, LOW);
        else
            digitalWrite(RELAY_SWITCH, HIGH);
    #endif

    // N-25/S1: Der Task-Watchdog wird erst hier scharf geschaltet, nicht am
    // Anfang von esp32setup(). setup() führt legitim sekundenlange
    // Einmal-Initialisierung aus (USB-CDC-Wartezeit, Display-Timeouts,
    // MCU811, die while(true)-Fehlerpfade der Peripherie-Init). Diese zu
    // überwachen war nie Sinn der Maßnahme -- sie hätte diagnostizierbare
    // Hänger in einen anonymen Boot-Loop verwandelt. Bewacht wird ab jetzt
    // ausschließlich esp32loop(), das sich in :esp32loop() selbst füttert.
    esp_task_wdt_add(NULL);
}

// BLE TX Function -> Node to Client.
//
// BLE-N1: report what the stack did, so the drain can keep the frame when it
// was refused. The old code did pTxCharacteristic->setValue() + notify();
// notify() without arguments is NimBLE-Arduino's ble_gatts_chr_updated(), a
// void call that queues the value for every subscribed peer and swallows a
// failing send (out of mbufs) -- the result was always "true". The value
// overload notify(data, len, handle) is not better: it returns only rc == 0.
// So the frame goes out through ble_gatts_notify_custom() directly, which
// returns the NimBLE rc:
//   0                 the ATT PDU is queued for the controller      -> SENT
//   BLE_HS_ENOTCONN   the connection is gone                        -> DOWN
//   BLE_HS_ENOMEM ... no mbuf / out of resources (also our own
//                     ble_hs_mbuf_from_flat() failing)              -> BUSY
// One difference to the old path: notify_custom does not look at the CCCD, it
// sends to the connection handle regardless. The drain only runs after the
// app-layer hello (isPhoneReady), i.e. with a subscribed phone.
BlePhoneSend esp32_write_ble(const uint8_t *buf, uint16_t len)
{
    #if defined(DISABLE_BLE)
    (void)buf; (void)len;     // no BLE stack: pTxCharacteristic was never created
    return BLE_SEND_DOWN;
    #else
    if(bBLEDEBUG)
        printfdeb("[LOOP] <%lu> WRITE BLE\n", millis());

    if(!deviceConnected || g_ble_conn_handle == 0xFFFF)
        return BLE_SEND_DOWN;

    // AU-10: the stack and pTxCharacteristic are gone while the update job has BLE paused
    // (that only happens with no phone connected, so this is a race guard, not a normal path)
    if(!bleLifeLock())
        return BLE_SEND_DOWN;
    if(pTxCharacteristic == nullptr)
    {
        bleLifeUnlock();
        return BLE_SEND_DOWN;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    int rc = BLE_HS_ENOMEM;
    if(om != nullptr)
        rc = ble_gatts_notify_custom(g_ble_conn_handle, pTxCharacteristic->getHandle(), om);   // frees om
    bleLifeUnlock();

    if(rc == 0)
        return BLE_SEND_SENT;

    if(bBLEDEBUG)
        Serial.printf("[BLE ];tx_rc;%lu;rc%d;msys%d\n", (unsigned long)millis(), rc, (int)os_msys_num_free());

    return (rc == BLE_HS_ENOTCONN) ? BLE_SEND_DOWN : BLE_SEND_BUSY;
    #endif
}

// BLE-N2: the live ATT MTU of the current connection, 0 if there is none.
uint16_t esp32_ble_mtu()
{
    #if defined(DISABLE_BLE)
    return 0;
    #else
    if(!deviceConnected || g_ble_conn_handle == 0xFFFF)
        return 0;
    if(!bleLifeLock())   // AU-10: no host calls while the stack is stopped
        return 0;
    const uint16_t mtu = (uint16_t)ble_att_mtu(g_ble_conn_handle);
    bleLifeUnlock();
    return mtu;
    #endif
}



// Deferred display update from OnRxDone (avoid I2C inside the radio callback).
// RACE-01: snapshot first, display call outside. There is no lock here and none
// is needed -- OnRxDone runs inside esp32loop() on ESP32, so producer and
// consumer are one task. Factored so both
// the local-radio loop and the external-radio path flush pending RX displays.
// the T5 drives its e-paper from its own task; the only other caller is the external-radio drain
#if !defined(BOARD_T5_EPAPER) || defined(EXTERNAL_RADIO)
static void flushDeferredDisplayUpdates()
{
    bool _pendText = bPendingDisplayText;
    bool _pendPos = bPendingDisplayPos;
    struct aprsMessage _msg;
    int16_t _rssi = 0;
    int8_t _snr = 0;
    if(_pendText || _pendPos) {
        _msg = pendingDisplayMsg;
        _rssi = pendingDisplayRssi;
        _snr = pendingDisplaySnr;
        bPendingDisplayText = false;
        bPendingDisplayPos = false;
    }
    INSTR_SECTION("display_rx");
    if(_pendText) sendDisplayText(_msg, _rssi, _snr);
    if(_pendPos)  sendDisplayPosition(_msg, _rssi, _snr);
}
#endif

// ---------------------------------------------------------------------------
// AU-05 (#1187): firmware auto update, loop side. Once per second, from esp32loop().
// The decisions live in src/fw_update.h (fwTick, fwAttemptAllowed ...); the network
// work (CHECK, DOWNLOAD) runs in its own task behind src/esp32/fw_update_net.h. This
// function only feeds the policy and starts jobs; FW_HANDOVER reboots into Safeboot
// through auRebootToSafeboot() (command_functions.cpp, shared with --ota-update).
// ---------------------------------------------------------------------------
static FwTimer s_auTimer;
static bool s_auTimerInit = false;
static uint32_t s_auLastTickMs = 0;
static uint32_t s_auNextTryMs = 0;      // back-off after fwNetStart() refused (busy / no link)
static bool s_auBackoff = false;
static bool s_auSawBusy = false;        // a job was observed running and its result is not consumed yet
static char s_auNotifiedTag[24] = "";   // [AU];notify printed for this tag
static char s_auHandoverTag[24] = "";   // [AU];handover printed for this tag
#if AU_AUTO_INSTALL
static bool s_auBlockedPrinted = false; // [AU];handover;blocked;old_safeboot printed this boot
#endif
static bool s_auSkipLogged = false;     // D9: [AU];auto;skip / blocked printed for this check
// auRequestApplyReboot(): flag + request time; auTick performs the reboot >= 500 ms later
static volatile bool s_auApplyRebootReq = false;
static volatile uint32_t s_auApplyRebootAtMs = 0;

void auRequestApplyReboot(void)
{
    s_auApplyRebootAtMs = millis();   // time first, flag second: the loop reads them in the other order
    s_auApplyRebootReq = true;
}

// fwNetStart() refuses new jobs while the requested reboot is still waiting for its 500 ms.
bool auApplyRebootPending(void)
{
    return s_auApplyRebootReq;
}

static bool auTagSame(const char *a, const char *b)
{
    return strncmp(a, b, 23) == 0;
}

// ---------------------------------------------------------------------------
// BLE guard (spec 7) and service shedding (spec 8.1). Loop task only, except the reads noted.
// ---------------------------------------------------------------------------
static uint32_t s_guardSetMs = 0;
static bool s_guardChain = false;       // the CHECK was requested with a chain and no DOWNLOAD started yet
static bool s_guardSawBusy = false;     // a job of this guard was started (BUSY is set synchronously by fwNetStart)
static volatile uint8_t s_auShedMask = 0;   // spec 8.1 step 3: FW_SHED_* bits the loop keeps stopped (KISS TCP, net console)

bool auBleGuardActive(void)
{
#if AU_BLE_PAUSE
    return s_bleGuard;
#else
    return false;   // no guard off the classic ESP32 (forced NimBLE deinit is not safe on the S3)
#endif
}

// Spec 8.1 step 3 (job task). The services the node could stop now (FW_SHED_* bits): a connected net console
// session, KISS TCP when on. Read only.
uint8_t auShedAvail(void)
{
    uint8_t mask = 0;
    #ifndef DISABLE_NET_CONSOLE
    if(isNetConsoleConnected())
        mask |= FW_SHED_CONSOLE;
    #endif
    #ifndef DISABLE_KISS_TCP
    if(bKISS)
        mask |= FW_SHED_KISS;
    #endif
    #if defined(HAS_TFT)
    if(tftSpriteActive())
        mask |= FW_SHED_TFT;   // AU-21: the loop can release the 25.6 KB sprite
    #endif
    return mask;
}

// Keep the services of mask stopped through the same off paths as `--netconsole off` and `--kiss off` (the loop does
// the stopping: sockets are only touched by the loop task). mask 0 restores both (the loop reopens them).
void auShedHold(uint8_t mask)
{
    s_auShedMask = mask;
}

// True once the sockets of mask are closed (the loop has run the off paths).
bool auShedSettled(uint8_t mask)
{
    bool consoleUp = false;
    bool kissUp = false;
    #ifndef DISABLE_NET_CONSOLE
    consoleUp = isNetConsoleConnected();
    #endif
    #ifndef DISABLE_KISS_TCP
    kissUp = isKissClientConnected();
    #endif
    bool tftUp = false;
    #if defined(HAS_TFT)
    tftUp = tftSpriteActive();
    #endif
    return fwShedSettled(mask, consoleUp, kissUp, tftUp);
}


#if AU_BLE_PAUSE
static bool auBleGuardStart(bool chainDownload, bool dueReason)
{
    if(s_bleGuard)
        return false;
    FwNetStatus st;
    fwNetGetStatus(st);
    if(st.state == FWS_BUSY)
        return false;
    const unsigned phone = (deviceConnected || isPhoneReady != 0) ? 1u : 0u;
    // set before the job starts: the job task reads the flag when it begins
    s_bleGuard = true;
    s_guardSetMs = millis();
    s_guardChain = chainDownload;
    s_guardSawBusy = false;
    if(dueReason)
        Serial.printf("[AU];bleguard;on;phone;%u;reason;due\n", phone);
    else
        Serial.printf("[AU];bleguard;on;phone;%u\n", phone);
    if(!fwNetStart(FWJ_CHECK, (uint8_t)meshcom_settings.node_updchan))
    {
        s_bleGuard = false;
        s_guardChain = false;
        Serial.printf("[AU];bleguard;off;refused\n");
        return false;
    }
    s_guardSawBusy = true;
    s_auSawBusy = true;
    return true;
}
#endif   // AU_BLE_PAUSE

bool auBleGuardRequest(bool chainDownload)
{
#if AU_BLE_PAUSE
    return auBleGuardStart(chainDownload, false);
#else
    (void)chainDownload;
    return false;
#endif
}

static void auBleGuardRelease(FwGuardAct act)
{
    s_bleGuard = false;
    s_guardChain = false;
    s_guardSawBusy = false;
    switch(act)
    {
    case FWG_REL_DONE:      Serial.printf("[AU];bleguard;off;done\n"); break;
    case FWG_REL_FAIL:      Serial.printf("[AU];bleguard;off;fail\n"); break;
    case FWG_REL_REFUSED:   Serial.printf("[AU];bleguard;off;refused\n"); break;
    default:                Serial.printf("[AU];bleguard;off;watchdog\n"); break;
    }
    // the job task left BLE down while the guard was set; a failed resume is retried by the loop
    if(esp32BleDown())
        esp32BleResume();
}

// Effective update mode (D14: a stored 2 acts as 1).
static int auEffectiveMode(void)
{
    const int storedMode = meshcom_settings.node_autoupd < 0 ? 0 : (meshcom_settings.node_autoupd > 2 ? 2 : meshcom_settings.node_autoupd);
    return fwEffectiveMode(storedMode);
}

// "[AU];notify;<tag>" for a finished CHECK that found a newer release, once per tag. Called by the result block of
// auTick and by the guard chain before it starts the DOWNLOAD (that DOWNLOAD hides the CHECK result in the same tick).
static void auNotifyCheck(const FwNetStatus &st)
{
    if(!fwNotifyDue(st.job == FWJ_CHECK && st.state == FWS_DONE_OK, st.availNewer, auEffectiveMode(),
                    auTagSame(s_auNotifiedTag, st.availTag)))
        return;
    snprintf(s_auNotifiedTag, sizeof(s_auNotifiedTag), "%s", st.availTag);
    // Serial.printf, not printfdeb: the latter strips ';' outside --debug csv
    Serial.printf("[AU];notify;%s\n", st.availTag);
}

// 1 Hz, before every early return of auTick: chain / release the guard, drop a shed hold nobody owns.
static void auGuardTick(uint32_t now)
{
    FwNetStatus st;
    fwNetGetStatus(st);
    const bool busy = (st.state == FWS_BUSY);
    if(s_auShedMask != 0 && !busy)
        s_auShedMask = 0;   // the job task restores at its end; this is the backstop
    if(!s_bleGuard)
        return;

    const bool ended = !busy && s_guardSawBusy && (st.state == FWS_DONE_OK || st.state == FWS_DONE_FAIL);
    const bool failed = (st.state == FWS_DONE_FAIL);
    const bool chainPending = s_guardChain && st.job == FWJ_CHECK;
    const bool stagedSame = st.staged && st.stagedTag[0] != '\0' && strncmp(st.stagedTag, st.availTag, 23) == 0;
    const bool chainOk = fwGuardChainOk(st.availNewer, st.installable, st.availTag[0] != '\0', stagedSame);
    const FwGuardAct act = fwGuardStep(true, busy, ended, failed, failed && fwGuardRefusedErr(st.lastErr),
                                       chainPending, chainOk, now, s_guardSetMs);
    if(act == FWG_HOLD)
        return;
    if(act == FWG_CHAIN_DOWNLOAD)
    {
        s_guardChain = false;
        auNotifyCheck(st);   // the DOWNLOAD below turns BUSY before auTick reads the CHECK result
        if(fwNetStart(FWJ_DOWNLOAD, (uint8_t)meshcom_settings.node_updchan))
        {
            s_guardSawBusy = true;
            s_auSawBusy = true;
        }
        else
            auBleGuardRelease(FWG_REL_FAIL);
        return;
    }
    auBleGuardRelease(act);
}

// 30 s pause after fwNetStart() said no, so a refused start is not retried every second.
// Wrap-safe: the flag is cleared when the deadline is reached (1 Hz caller).
static bool auBackedOff(uint32_t now)
{
    if(s_auBackoff && (int32_t)(now - s_auNextTryMs) >= 0)
        s_auBackoff = false;
    return s_auBackoff;
}
static void auBackOff(uint32_t now)
{
    s_auNextTryMs = now + 30000u;
    s_auBackoff = true;
}

// True if a handover for `tag` was already attempted this boot (auHandoverRecord is a no-op then).
bool auHandoverAttempted(const char *tag)
{
    return tag != nullptr && tag[0] != '\0' && s_auHandoverTag[0] != '\0' && auTagSame(s_auHandoverTag, tag);
}

// The handover sequence, shared by the auto path (FW_HANDOVER) and the remote go. One attempt per tag
// and boot (s_auHandoverTag); does not return on success. Returns false if the attempt failed (nvs).
static bool auHandoverRecord(const FwStageRecord &rec)
{
    if(auTagSame(s_auHandoverTag, rec.tag))
        return true;
    snprintf(s_auHandoverTag, sizeof(s_auHandoverTag), "%s", rec.tag);
    // AU-16: a DIO bootloader never reaches the Safeboot; refuse before the marker is written
    if(!fwNetBootloaderQio())
    {
        Serial.printf("[AU];handover;blocked;bootloader_dio\n");
        return false;
    }
    // the "hand" marker goes in BEFORE the reboot: it is how the next boot tells an old
    // Safeboot (record untouched) from a new one (marker consumed)
    if(!fwNetMarkHandover(rec.tag))
    {
        Serial.printf("[AU];fail;nvs\n");
        return false;
    }
    Serial.printf("[AU];handover;%s\n", rec.tag);
    if(!auRebootToSafeboot())
    {
        // hopeless (no Safeboot partition, or it cannot be set as boot partition): drop the
        // record so the next boot does not try again, and the staged flag no longer fires.
        // AU-16: a DIO bootloader was already reported by the guard; the record is dropped the same way
        if(fwNetBootloaderQio())
            Serial.printf("[AU];fail;nosafeboot\n");
        fwNetClearRecord();
    }
    return true;
}

static void auTick(void)
{
    const uint32_t now = millis();

    // web "Install now": the answer is out, the node reboots into Safeboot (not throttled to 1 Hz)
    if(fwApplyRebootDue(s_auApplyRebootReq, s_auApplyRebootAtMs, now))
    {
        s_auApplyRebootReq = false;
        // a console / RM job may have started inside the 500 ms window (fwNetStart refuses new ones since the
        // request, this catches one that slipped in before it): do not reboot under a running job
        FwNetStatus auNow;
        fwNetGetStatus(auNow);
        if(fwApplyRebootAct(true, auNow.state == FWS_BUSY) == FWAPPLY_ABORT_BUSY)
        {
            fwNetUnmarkHandover();
            Serial.printf("[AU];apply;abort;busy\n");
            return;
        }
        Serial.printf("[AU];apply;reboot\n");
        if(!auRebootToSafeboot())   // returns only without a usable Safeboot partition
        {
            fwNetUnmarkHandover();
            if(fwNetBootloaderQio())   // AU-16: the DIO guard has reported itself
                Serial.printf("[ERR];update;no safeboot partition\n");
        }
        return;
    }

    if((uint32_t)(now - s_auLastTickMs) < 1000u)
        return;
    s_auLastTickMs = now;
    auGuardTick(now);

    // AU-12: boot demotion, once per boot at the first tick (settings are loaded long before). A node
    // that was in auto mode with a Safeboot too old to apply updates drops to notify and says so. Only
    // auto mode triggers the Safeboot partition scan here (cached by fwNetSafebootVersion; the web pages and
    // --update status trigger it too, once per boot). The setting is written once; there is no reboot and no retry.
    static bool s_auBootCapChecked = false;
    if(!s_auBootCapChecked)
    {
        s_auBootCapChecked = true;
        if(meshcom_settings.node_autoupd == 2 && !fwNetSafebootCapable())
        {
            meshcom_settings.node_autoupd = 1;
            save_settings();
            // Serial.printf, not printfdeb: the latter strips ';' outside --debug csv
            if(!fwNetBootloaderQio())
                Serial.printf("[AU];refuse;bootloader_dio;mode;notify;renew with the web flasher\n");
            else
                Serial.printf("[AU];refuse;safeboot_too_old;mode;notify;ver;%d;need;%d\n", fwNetSafebootVersion(), AU_SAFEBOOT_MIN);
        }
    }

    // WiFi STA with an address (the Ethernet path is not an AU transport)
    const bool wifiUp = (WiFi.status() == WL_CONNECTED) && ((uint32_t)WiFi.localIP() != 0u);

    if(wifiUp && !s_auTimerInit)
    {
        fwTimerInit(s_auTimer, now, (uint32_t)ESP.getEfuseMac());
        s_auTimerInit = true;
    }
    if(!s_auTimerInit && !rmAuGoArmed())   // a remote go needs no timer (works with auto-update off)
        return;

    FwPolicyIn in;
    const int storedMode = meshcom_settings.node_autoupd < 0 ? 0 : (meshcom_settings.node_autoupd > 2 ? 2 : meshcom_settings.node_autoupd);
    in.mode = (uint8_t)fwEffectiveMode(storedMode);   // D14: a stored 2 (auto) acts as 1 (notify)
    in.wifiUp = wifiUp;
    // clock valid for TLS and phone state: one definition, shared with rmAuGo (rm_au_esp32.cpp)
    rmAuAutoGates(&in.timeValid, &in.phoneConnected);
    in.phoneConnected = fwGuardPhoneConnected(in.phoneConnected, s_bleGuard);   // spec 7.5
    // global_batt is the filtered pack voltage in mV; "no reading" = 0 mV = mains / USB. The same
    // fail-safe as the /B= tag: battHardwarePresent() is false without a battery reading.
    in.onBattery = battHardwarePresent();
    in.battMv = (uint16_t)(global_batt <= 0.0f ? 0.0f : (global_batt > 65000.0f ? 65000.0f : global_batt));
    // Node time is UTC + node_utcoff; with a TZ-01 rule node_utcoff is re-derived from it, so this
    // is the rule's local time (UTC without a rule and offset 0).
    {
        int h = meshcom_settings.node_date_hour, m = meshcom_settings.node_date_minute;
        in.localMinuteOfDay = (uint16_t)((h < 0 || h > 23 || m < 0 || m > 59) ? 0 : h * 60 + m);
    }
    const bool battOk = !in.onBattery || in.battMv >= FW_BATT_FLOOR_MV;   // AU-D17

    FwNetStatus st;
    fwNetGetStatus(st);

    // ---- consume the result of a finished job (also one started by --update) ----
    if(st.state == FWS_BUSY)
        s_auSawBusy = true;
    else if(s_auSawBusy && (st.state == FWS_DONE_OK || st.state == FWS_DONE_FAIL))
    {
        s_auSawBusy = false;
        if(st.job == FWJ_DOWNLOAD && st.state == FWS_DONE_FAIL)
        {
            if(fwErrConsumesAttempt(st.lastErr))
                fwAttemptFailed(s_auTimer, st.availTag, now);
            else
            {
                // lowheap (classic ESP32 with a phone on BLE): transient, no attempt used; wait 10 min
                auBackOff(now);
                s_auNextTryMs = now + 600000u;
            }
        }
        auNotifyCheck(st);
    }
    if(st.state == FWS_BUSY)
        return;

    // ---- remote go (RM1 "upd go"): independent of node_autoupd, the update window and the check timer ----
    {
        char goTag[24];
        bool goDelayPassed = false;
        static char s_goWaitReason[16] = "";
        if(rmAuGoGet(goTag, &goDelayPassed, now))
        {
            const bool stagedMatch = st.staged && auTagSame(st.stagedTag, goTag);
            FwGoTickIn gt;
            memset(&gt, 0, sizeof(gt));
            gt.busy = (st.state == FWS_BUSY);
            gt.downloadFailed = (st.job == FWJ_DOWNLOAD && st.state == FWS_DONE_FAIL);
            gt.stagedMatch = stagedMatch;
            gt.attempted = auHandoverAttempted(goTag);
            gt.delayPassed = goDelayPassed;
            gt.ready = fwGoHandoverReady(in);
            gt.phoneConnected = in.phoneConnected;
            gt.safebootCapable = fwNetSafebootCapable();
            gt.recordState = FWGO_REC_UNCHECKED;
            FwStageRecord grec;
            FwGoStep step = fwGoTickStep(gt);
            if(step == FWGOS_NEED_RECORD)
            {
                // the record is read last: re-validated here, and it must carry the go tag
                gt.recordState = !fwNetLoadRecord(grec) ? FWGO_REC_MISSING : (auTagSame(grec.tag, goTag) ? FWGO_REC_OK : FWGO_REC_TAG_DIFFERS);
                step = fwGoTickStep(gt);
            }
            const char *wait = nullptr;
            bool failed = true;   // literal format strings: the log lines stay greppable
            switch(step)
            {
            case FWGOS_WAIT_DOWNLOAD:   wait = "download"; failed = false; break;
            case FWGOS_WAIT_REPLY:      wait = "reply"; failed = false; break;
            case FWGOS_WAIT_PHONE:      wait = "phone"; failed = false; break;
            case FWGOS_WAIT_BATTERY:    wait = "battery"; failed = false; break;
            case FWGOS_FAIL_DOWNLOAD:   Serial.printf("[AU];remote;fail;download\n"); break;
            case FWGOS_FAIL_NOSTAGE:    Serial.printf("[AU];remote;fail;nostage\n"); break;
            case FWGOS_FAIL_ATTEMPTED:  Serial.printf("[AU];remote;fail;attempted\n"); break;
            case FWGOS_FAIL_SAFEBOOT:   Serial.printf("[AU];remote;fail;safeboot\n"); break;
            case FWGOS_FAIL_NORECORD:   Serial.printf("[AU];remote;fail;norecord\n"); break;
            case FWGOS_HANDOVER:
                Serial.printf("[AU];remote;go;%s\n", goTag);
                rmAuGoDisarm();
                if(!auHandoverRecord(grec))
                    Serial.printf("[AU];remote;fail;nvs\n");
                return;
            default: failed = false; break;
            }
            if(failed)
                rmAuGoDisarm();
            else
            {
                if(wait != nullptr && strcmp(wait, s_goWaitReason) != 0)
                {
                    snprintf(s_goWaitReason, sizeof(s_goWaitReason), "%s", wait);
                    Serial.printf("[AU];remote;wait;%s\n", wait);
                }
                else if(wait == nullptr)
                    s_goWaitReason[0] = '\0';
            }
        }
        else
            s_goWaitReason[0] = '\0';
    }

    if(!s_auTimerInit)
        return;

    FwAction act = fwTick(s_auTimer, in, fwNetStagedPending(), now);

#if AU_AUTO_INSTALL
    if(act == FW_HANDOVER)
    {
        // AU-08: reboot into Safeboot, which applies the staged image. The record is re-read and
        // re-validated here (fwNetLoadRecord drops a stale one and clears the cached flag, so a
        // gone record ends the handovers). One attempt per tag and boot: if there is no Safeboot
        // partition the function returns and the tick must not retry every second.
        // old Safeboot (too old by version scan, or it ignored the last handover): no automatic handover, or every update window
        // would reboot the node into the same dead end. --update apply still tries.
        if(!fwNetSafebootCapable())   // AU-12: version scan + nocap backstop
        {
            if(!s_auBlockedPrinted)
            {
                s_auBlockedPrinted = true;
                if(fwNetBootloaderQio())
                    Serial.printf("[AU];handover;blocked;old_safeboot\n");
                else
                    Serial.printf("[AU];handover;blocked;bootloader_dio\n");
            }
            return;
        }
        if(st.staged && st.stagedTag[0] && auTagSame(s_auHandoverTag, st.stagedTag))
            return;   // already attempted this boot: no NVS read every second
        FwStageRecord rec;
        if(!fwNetLoadRecord(rec))
            return;
        auHandoverRecord(rec);
        return;
    }
#endif   // AU_AUTO_INSTALL

    if((act == FW_CHECK || act == FW_CHECK_GUARDED) && !auBackedOff(now))
    {
        // D16: the phone stayed connected 30 min past the due time: run this period's check under the BLE guard
        // (the job force-pauses BLE, the guard is released when the check is done). No automatic download (D14).
        // Without AU_BLE_PAUSE (ESP32-S3 etc.) there is no guard: the plain check below runs, as before W4.
        #if AU_BLE_PAUSE
        if(act == FW_CHECK_GUARDED)
        {
            if(auBleGuardStart(false, true))
            {
                fwCheckDone(s_auTimer, now);
                s_auSkipLogged = false;
            }
            else
                auBackOff(now);
            return;
        }
        #endif
        if(fwNetStart(FWJ_CHECK, (uint8_t)meshcom_settings.node_updchan))
        {
            fwCheckDone(s_auTimer, now);
            s_auSawBusy = true;
            s_auSkipLogged = false;
        }
        else
            auBackOff(now);
        return;
    }

    // stored mode 2 (auto): a CHECK found a newer, installable release (matching .autoupdate asset and digest)
    // that is not staged yet -> DOWNLOAD (AU-D17: battery floor). A release without the asset must not
    // consume the 3 attempts of AU-D5. Mode 1 never gets here: it only prints the notify line.
    // D14: the automatic download is not compiled (AU_AUTO_INSTALL 0); where it would be due, the node says
    // why it does not start (D9), once per check.
    if(storedMode == 2 && st.availNewer && st.installable && st.availTag[0] && wifiUp)
    {
        const bool alreadyStaged = st.staged && auTagSame(st.stagedTag, st.availTag);
        const char *skip = fwAutoSkipReason(AU_AUTO_INSTALL != 0, in.timeValid, in.phoneConnected, battOk,
                                            auBackedOff(now), fwAttemptAllowed(s_auTimer, st.availTag, now), alreadyStaged);
        if(skip == nullptr)
        {
#if AU_AUTO_INSTALL
            if(fwNetStart(FWJ_DOWNLOAD, (uint8_t)meshcom_settings.node_updchan))
                s_auSawBusy = true;
            else
                auBackOff(now);
#endif   // AU_AUTO_INSTALL
        }
        else if(!s_auSkipLogged)
        {
            s_auSkipLogged = true;
            // Serial.printf, not printfdeb: the latter strips ';' outside --debug csv
            Serial.printf("[AU];auto;skip;%s\n", skip);
            if(!AU_AUTO_INSTALL)
                Serial.printf("[AU];auto;blocked;notify only\n");
        }
    }
}

#if defined(HAS_TFT)
// AU-21: the loop owns the TFT sprite. Released while the update job holds FW_SHED_TFT or the stored display
// setting is off (bDisplayOff, not the screensaver); never re-acquired while an AU job runs. A failed acquire retries
// at most every 30 s, a transition (hold ends, display on again) retries at once. Nothing else creates the
// sprite: displayTFT is a no-op without it and initTFT runs once at boot.
static void tftSpriteReconcile(void)
{
    static bool lastHeld = false;
    static bool lastOff = false;
    static unsigned long lastTry = 0;
    static bool tried = false;

    const bool held = (s_auShedMask & FW_SHED_TFT) != 0;
    bool busy = false;
    if(!bDisplayOff && !held)   // only then can a running job change the verdict
    {
        FwNetStatus st;
        fwNetGetStatus(st);
        busy = (st.state == FWS_BUSY);
    }
    const bool wanted = fwTftSpriteWanted(bDisplayOff, held, busy);
    const bool transition = (lastHeld && !held) || (lastOff && !bDisplayOff);
    lastHeld = held;
    lastOff = bDisplayOff;

    if(!wanted && tftSpriteActive())
    {
        tftSpriteRelease();
        if(held && !bDisplayOff)
            tftShowUpdating();
    }
    else if(wanted && !tftSpriteActive())
    {
        if(transition || !tried || (millis() - lastTry) >= 30000UL)
        {
            tried = true;
            lastTry = millis();
            if(tftSpriteAcquire() && !bDisplayIsOff)
                sendDisplayHead(true);   // full redraw of the current page
        }
    }
}
#endif

void esp32loop()
{
    // Boot-Ende-Marke fuer den Bench-Harness: die Netzwerkphase ist vorbei,
    // sobald WLAN verbunden ist (node_hasIPaddress), der Verbindungsversuch
    // aufgegeben wurde (bAllStarted, nach Reset und Wiederholung) oder gar kein
    // Netzwerkdienst konfiguriert ist. Bis dahin blinken die Kopfzeilen-Icons
    // und Befehle werden verzoegert bearbeitet.
    // Nicht nur bAllStarted: das bleibt false, wenn WLAN schon in setup()
    // verbunden hat (iWlanWait == 0 und IP vorhanden -> der Zweig, der es auf
    // true setzt, laeuft nie).
    static bool s_bootReadyLogged = false;
    if (!s_bootReadyLogged)
    {
        bool network_wanted = bGATEWAY || bEXTUDP || bWEBSERVER || bNETCONSOLE;
        // hasIPaddress (global) kippt bei "now listening at IP"; die Kopie in
        // meshcom_settings wird erst spaeter nachgezogen -- beide pruefen.
        extern bool hasIPaddress;
        if (!network_wanted || bAllStarted || hasIPaddress || meshcom_settings.node_hasIPaddress)
        {
            s_bootReadyLogged = true;
            // Serial.printf, nicht printfdeb: das wuerde die Semikolons ausserhalb
            // von --debug csv entfernen und der Harness faende die Marke nicht.
            Serial.printf("[BOOT];ready;ms;%lu;ip;%d\n", (unsigned long)millis(),
                          (hasIPaddress || meshcom_settings.node_hasIPaddress) ? 1 : 0);
            #if defined(BENCH_BLE_ADV_LATE) && !defined(DISABLE_BLE)
            if(bleLifeLock())   // AU-10: BLE may be paused by an update job
            {
                NimBLEDevice::getAdvertising()->start();
                bleLifeUnlock();
                Serial.printf("[BLE ];advertising;started;ms;%lu\n", (unsigned long)millis());
            }
            #endif
        }
    }
    // TEMPORARY -- measures the period between successive loop entries, so a
    // stall is caught regardless of which call site blocked. See src/instrument.h
    INSTR_LOOPTICK();
    gwRegTick();   // gateway registration state: call-change check (rate-limited to 1 s)
#if INSTRUMENT_ENABLED
    INSTR_ONRX_DRAIN();      // R1-F: emit queued [INS] ONRX records
#endif

    // BP-01/BP-04: close a back-pressure episode with QRV once the TX ring
    // has drained, even when no further user message arrives to observe it.
    // Since BP-02 an occupied-slot scan (O(MAX_RING)), since BP-04 with the
    // water-band hold; it emits at most once per episode.
    bpPollDrain();

    esp_task_wdt_reset();

    #if not defined(BOARD_T_DECK_PRO)
    btn.tick();
    #endif

    if(bDEEP_SLEEP)
    {
        delay(1000);
        // only loop
    }
    else
    {

    static unsigned long last_time_save = 0;
    if(millis() - last_time_save > 900000) // Save every 15 minutes
    {
        saveTimePersistence();
        last_time_save = millis();
    }

    // loop T-Deck GUI
    #if defined(BOARD_T_DECK_PRO)
        loopTDeck_pro();
    #endif

    #ifdef LED_PIN
        if(bLED_GREEN || bLED_RED || bLED_BLUE || bLED_ORANGE || bLED_WEISS || bLED_CLEAR || bLED_DELAY)
        {
            if(pixels_delay == 0 || bLED_CLEAR)
            {
                pixels.clear();

                for(int i=0; i<LED_PIXEL; i++)
                {
                    if(bLED_GREEN)
                    {
                        pixels.setPixelColor(i, pixels.Color(0, 50, 0));
                        bLED_CLEAR=false;
                    }
                    else
                    if(bLED_RED)
                    {
                        pixels.setPixelColor(i, pixels.Color(50, 0, 0));
                        bLED_CLEAR=false;
                    }
                    else
                    if(bLED_BLUE)
                    {
                        pixels.setPixelColor(i, pixels.Color(0, 0, 50));
                        bLED_CLEAR=false;
                    }
                    else
                    if(bLED_ORANGE)
                    {
                        pixels.setPixelColor(i, pixels.Color(50, 25, 0));
                        bLED_CLEAR=false;
                    }
                    else
                    if(bLED_WEISS)
                    {
                        pixels.setPixelColor(i, pixels.Color(50, 50, 50));
                        bLED_CLEAR=false;
                    }
                    else
                    if(bLED_CLEAR)
                        pixels.setPixelColor(i, pixels.Color(0, 0, 0));

                    bLED_RED=false;
                    bLED_GREEN=false;
                    bLED_BLUE=false;
                    bLED_ORANGE=false;
                    bLED_WEISS=false;
                }

                pixels.setBrightness(25);

                pixels.show();

                if(!bLED_CLEAR)
                {
                    pixels_delay = millis();    // ms
                    bLED_DELAY=true;
                }

                bLED_CLEAR=false;
            }
            else
            {
                if((uint32_t)(millis() - pixels_delay) >= DELAYVAL)
                {
                    pixels_delay = 0;
                    bLED_CLEAR=true;
                    bLED_DELAY=false;
                }
            }
        }
    #endif

    #if defined(REMOTE_LED_PIN)
        remoteLedTick();   // RM "led flash": the pattern runs here, independent of --board led
    #endif

    boardLedTick();   // --board led heartbeat (30 ms / 2 s); yields to RM hold and OneWire, no-op without an LED pin

    // D1-10: the shared scheduler runs at function scope, BEFORE the
    // BOARD_T5_EPAPER / if(bRadio) block below. Six of its seven jobs (battery,
    // heap monitor, MCP23017, BMP390, MCU811, INA226) never depended on the
    // radio; only retransmit did, and loopEnabled_retransmit() keeps that gate
    // (bRadio) per entry. Inside the block they would stop on a node whose
    // radio failed to init, on env:esp32-external-radio (bRadio is false by
    // design there) and on the T5 e-paper build, which compiles the #else arm
    // away. BattTimeWait's zero-init is unconditional in the original (unlike
    // heapMonTimer's/INA226TimeWait's, which are nested inside their own
    // runtime guard and so live inside loopEnabled_heapMon()/
    // loopEnabled_ina226() instead -- see loop_actions_esp32.cpp), so it stays
    // a top-level statement here, right before the scheduler call.
    if (BattTimeWait == 0)
        BattTimeWait = millis() - 30000;

    loopSchedulerRun(millis());

    // WEB-RX-01: park counters, once per 5 minutes, only if the park was ever used
    {
        static uint32_t rxparkStatTimer = 0;
        if((uint32_t)(millis() - rxparkStatTimer) >= (5 * 60 * 1000))
        {
            rxparkStatTimer = millis();
            if(rxParkParked() > 0)
                printfdeb("[RXPARK];parked;%lu;overflow;%lu;clamped;%lu\n", (unsigned long)rxParkParked(),
                    (unsigned long)rxParkOverflow(), (unsigned long)rxParkClamped());
        }
    }

    // LoRa-Chip found
    #if defined(BOARD_T5_EPAPER)
        idf_loop();
    // #elif defined(BOARD_T_DECK_PRO)
    // extra source
    #else

    if(bRadio)
    {
        if(bLORADEBUG && receiveFlag)
            printlndeb("receiveflag");

        if(bLORADEBUG && transmittedFlag)
            printlndeb("transmittedFlag");

        if(inoReceiveTimeOutTime > 0)
        {
            // Timeout RECEIVE_TIMEOUT
            if((uint32_t)(millis() - inoReceiveTimeOutTime) >= (60 * 6 * 1000))  // 6 Minuten
            {
                inoReceiveTimeOutTime=0;

                if(bLORADEBUG)
                {
                    printdeb(getTimeString());
                    printlndeb(" [LoRa]...Receive Timeout > 6.5 min. just for info");
                }

            }
        }


        // FIX: Periodic ring buffer utilization report (every 30s)
        {
            static unsigned long ring_status_timer = 0;
            if(bLORADEBUG && (millis() - ring_status_timer) > 30000)
            {
                ring_status_timer = millis();
                int pending = 0, retrying = 0, done = 0;
                for(int i = 0; i < MAX_RING; i++)
                {
                    if(ringBuffer[i][0] == 0) continue;
                    if(ringBuffer[i][1] == RING_STATUS_DONE) done++;
                    else if(ringBuffer[i][1] == RING_STATUS_READY) pending++;
                    else retrying++;
                }
                // BP-02: queued is the honest occupied-slot count (the
                // pending/retrying/done buckets above already scan every
                // slot and partition it exactly). TXP W1b (slot pool): the
                // iW=/iR=/dist= fields are gone with the indices; retrying
                // includes INFLIGHT and parked SENT slots.
                int queued = pending + retrying + done;
                int dedup_used = 0;
                for(int i = 0; i < MAX_DEDUP_RING; i++)
                {
                    if(ringBufferLoraRX[i][0] != 0 || ringBufferLoraRX[i][1] != 0 ||
                       ringBufferLoraRX[i][2] != 0 || ringBufferLoraRX[i][3] != 0)
                        dedup_used++;
                }
                printfdeb("[MC-DBG] RING_STATUS queued=%d pending=%d retrying=%d "
                              "done=%d dedup=%d/%d\n",
                              queued, pending, retrying, done,
                              dedup_used, MAX_DEDUP_RING);
            }
        }

        // Deferred display update from OnRxDone (avoid I2C inside radio callback)
        flushDeferredDisplayUpdates();

        // Channel utilization report (every 10s). SL-05: the drain of
        // ch_util_rx_accum/tx_accum below feeds stat_util_rx_5m/tx_5m for the
        // 5-minute STAT line (--setlog on), so the 10-s tick itself must not
        // depend on bLORADEBUG any more -- only the diagnostic prints still
        // do, unchanged from before. The 10-s accumulation math is untouched.
        {
            static unsigned long ch_util_timer = 0;
            if((millis() - ch_util_timer) > 10000)
            {
                unsigned long window = millis() - ch_util_timer;
                ch_util_timer = millis();
                unsigned long rx_ms = ch_util_rx_accum.exchange(0);
                unsigned long tx_ms = ch_util_tx_accum.exchange(0);
                unsigned int util = (unsigned int)((rx_ms + tx_ms) * 100 / window);
                if(util > 100) util = 100;
                stat_util_rx_5m.fetch_add((uint32_t)rx_ms);
                stat_util_tx_5m.fetch_add((uint32_t)tx_ms);
                if(bLORADEBUG)
                {
                    printfdeb("[MC-DBG] CHANNEL_UTIL rx=%lums tx=%lums util=%u%%\n",
                        rx_ms, tx_ms, util);
                    // ONRXDONE stats: report max and warn count, then reset
                    printfdeb("[MC-DBG] ONRXDONE_STATS max=%lums warn=%u (>%dms)\n",
                        onrxdone_max_ms, onrxdone_warn_count, ONRXDONE_WARN_MS);
                    onrxdone_max_ms = 0;
                    onrxdone_warn_count = 0;
                }
            }
        }

        // Priority statistics output (every 5 minutes)
        if((millis() - stat_prio_timer) > (unsigned long)(PRIO_STAT_INTERVAL_S * 1000UL))
        {
            stat_prio_timer = millis();

            // SL-05: STAT line under --setlog on, independent of bLORADEBUG.
            // setlogFillStat() reads stat_drop_count[] before the existing
            // (unconditional) memset further below resets it -- no second
            // reset here. The interval counters are drained on every tick
            // (so they never grow unbounded); only the print depends on
            // bDisplayLog.
            {
                struct setlogStatFields f;
                setlogFillStat(&f, (uint32_t)ESP.getFreeHeap());

                // OR-STAT: OnRxDone time distribution (drains the window)
                {
                    struct onrxStatsOut o;
                    onrxStatsSnapshotReset(&o, ONRXDONE_WARN_MS);
                    f.onrx_n = o.n; f.onrx_avg_ms = o.avg_ms; f.onrx_p95_ms = o.p95_ms;
                    f.onrx_p99_ms = o.p99_ms; f.onrx_max_ms = o.max_ms; f.onrx_warn = o.warn;
                }

                if(bDisplayLog)
                {
                    char buf[400];
                    setlogFormatStat(buf, sizeof(buf), &f);
                    setlogPrint(buf);
                }
            }

            if(bLORADEBUG)
            {
                printfdeb("[MC-STAT] t=%ds qmax=%d/%d\n",
                    PRIO_STAT_INTERVAL_S, stat_queue_hwm, MAX_RING);
                printfdeb("  tx: p1=%d p2=%d p3=%d p4=%d p5=%d preempt=%d\n",
                    stat_tx_count[1], stat_tx_count[2], stat_tx_count[3],
                    stat_tx_count[4], stat_tx_count[5], stat_preempt_count);
                printfdeb("  drop: p1=%d p2=%d p3=%d p4=%d p5=%d\n",
                    stat_drop_count[1], stat_drop_count[2], stat_drop_count[3],
                    stat_drop_count[4], stat_drop_count[5]);
            }
            
            for(int p = 1; p <= 5; p++)
            {
                if(stat_tx_count[p] > 0)
                {
                    uint32_t avg = stat_latency_sum[p] / stat_tx_count[p];
                    if(bLORADEBUG)
                        printfdeb("[MC-PRIO] p%d_lat_avg=%ums p%d_lat_max=%lums p%d_cnt=%d\n",
                            p, avg, p, (unsigned long)stat_latency_max[p], p, stat_tx_count[p]);
                }
            }
            // Reset window counters
            memset(stat_tx_count, 0, sizeof(stat_tx_count));
            memset(stat_drop_count, 0, sizeof(stat_drop_count));
            memset(stat_latency_sum, 0, sizeof(stat_latency_sum));
            memset(stat_latency_max, 0, sizeof(stat_latency_max));
            stat_preempt_count = 0;
        }

        // High-water mark output (every 30 minutes)
        if((millis() - stat_hwm_timer) > (unsigned long)(PRIO_HWM_INTERVAL_S * 1000UL))
        {
            stat_hwm_timer = millis();
            if(!bDisplayLog)
            {
                printfdeb("[MC-HWM] uptime=%lus queue_hwm=%d/%d csma_hwm=%d trickle=%lums aged1=%lu aged2=%lu wait_hwm=%lu\n",
                    millis() / 1000, stat_queue_hwm, MAX_RING,
                    stat_csma_hwm_attempts, trickle_interval_ms,
                    (unsigned long)stat_csma_aged_boot[1], (unsigned long)stat_csma_aged_boot[2],
                    (unsigned long)stat_wait_hwm_ms);
            }
        }

        // TX-IRQ Watchdog: Wenn bEnableInterruptTransmit zu lange true bleibt
        // (TX-Done-IRQ wurde nie gefeuert), ist die State Machine eingefroren.
        // iReceiveTimeOutTime == 0 waehrend TX => normaler CSMA-Timeout greift nicht.
        // Fix: nach TX_WATCHDOG_MS Zwangs-Recovery zurueck in RX.
        {
            unsigned long _tx_s = (unsigned long)ch_util_tx_start;
            if(bEnableInterruptTransmit && _tx_s > 0 &&
               (millis() - _tx_s) > TX_WATCHDOG_MS)
            {
                printfdeb("[MC-WDT] TX_WATCHDOG fired after %lums — forcing RX recovery\n",
                    millis() - _tx_s);

                // SL-05: abort counter for the STAT block's txfail=.
                stat_txfail.fetch_add(1);

                ch_util_tx_start = 0;
                transmittedFlag   = false;
                bEnableInterruptTransmit = false;
                bEnableInterruptReceive  = false;

                radio.clearPacketSentAction();
                radio.clearPacketReceivedAction();
                radio.finishTransmit();

                #ifdef BOARD_HELTEC_V4
                disablePATransmit();
                #endif
                #ifdef RADIO_CTRL
                    digitalWrite(RADIO_CTRL, HIGH);  // RX Mode
                    delay(2);
                #endif

                radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF,
                    RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
                    RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
                radio.setPacketReceivedAction(setFlagReceive);
                bEnableInterruptReceive = true;

                iReceiveTimeOutTime = millis();
                csma_reset();
            }
        }

        if(iReceiveTimeOutTime > 0)
        {
            // Timeout csma_timeout (slot-basierter Backoff)
            if((uint32_t)(millis() - iReceiveTimeOutTime) >= csma_timeout)
            {
                // Debug A: RX_TIMEOUT_FIRE
                if(bLORADEBUG)
                    printfdeb("[MC-DBG] RX_TIMEOUT_FIRE ts=%lu wait=%lu delta=%lu\n",
                        millis(), csma_timeout, millis() - iReceiveTimeOutTime);

                // FIX BUG #1: Do not reset radio if a received packet is pending
                if(receiveFlag)
                {
                    // A packet arrived just before timeout — let the receiveFlag
                    // handler process it. Just reset the timer.
                    iReceiveTimeOutTime = millis();

                    if(bLORADEBUG)
                        printlndeb("[MC-DBG] RX_TIMEOUT_DEFERRED src=receiveFlag\n");
                }
                // FIX: Do not reset radio if a packet reception is in progress.
                // Poll IRQ register for PREAMBLE_DETECTED or HEADER_VALID —
                // if set, the radio is actively demodulating a packet and
                // calling startReceive() would abort it.
                // RSSI validation: stale IRQ flags with no signal are rejected early.
                // Header valid: strong indicator, up to 3 deferrals allowed.
                // Preamble only: weak indicator, max 1 deferral allowed.
                else
                {
                    bool shouldDefer = false;
                    uint32_t irqFlags = radio.getIrqFlags();
                    bool irq_active = irqFlags & (RADIOLIB_SX126X_IRQ_HEADER_VALID |
                                                   RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED);

                    if(irq_active)
                    {
                        bool header_valid = irqFlags & RADIOLIB_SX126X_IRQ_HEADER_VALID;
                        float currentRSSI = radio.getRSSI(false);

                        if(currentRSSI < -126.0f)
                        {
                            // No signal present — IRQ flags are stale
                            if(bLORADEBUG)
                                printfdeb("[MC-DBG] RX_IRQ_STALE_EARLY rssi=%.0f flags=%s cnt=%d\n",
                                    currentRSSI, header_valid ? "HDR" : "PRE", rx_irq_defer_count);
                        }
                        else if(header_valid && rx_irq_defer_count < 3)
                        {
                            shouldDefer = true;
                            if(bLORADEBUG)
                                printfdeb("[MC-DBG] RX_TIMEOUT_DEFERRED src=header_valid rssi=%.0f cnt=%d\n",
                                    currentRSSI, rx_irq_defer_count);
                        }
                        else if(!header_valid && rx_irq_defer_count < 1)
                        {
                            shouldDefer = true;
                            if(bLORADEBUG)
                                printfdeb("[MC-DBG] RX_TIMEOUT_DEFERRED src=preamble_only rssi=%.0f cnt=%d\n",
                                    currentRSSI, rx_irq_defer_count);
                        }
                        else
                        {
                            if(bLORADEBUG)
                                printfdeb("[MC-DBG] RX_IRQ_STALE rssi=%.0f flags=%s cnt=%d\n",
                                    currentRSSI, header_valid ? "HDR" : "PRE", rx_irq_defer_count);
                        }
                    }

                    if(shouldDefer)
                    {
                        rx_irq_defer_count++;
                        iReceiveTimeOutTime = millis();
                    }
                    else
                    {
                        if(rx_irq_defer_count > 0 && bLORADEBUG)
                            printfdeb("[MC-DBG] RX_IRQ_STALE forced restart after %d deferrals\n", rx_irq_defer_count);
                        rx_irq_defer_count = 0;
                        iReceiveTimeOutTime=0;

                        // RX-DEAF-01: no read is in flight here (the read path runs in this loop task
                        // and clears the flag itself), so a set is_receiving is stale: clear it with
                        // the radio restart, or the node stays deaf.
                        is_receiving = false;

                        // FIX BUG #1: Call startReceive FIRST, then re-wire interrupts.
                        // Disable interrupt gating while we reconfigure
                        bEnableInterruptReceive = false;
                        bEnableInterruptTransmit = false;

                        #ifdef RADIO_CTRL
                            digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
                            delay(2);
                        #endif
                        int state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);

                        // Now re-wire the interrupt callback
                        radio.clearPacketReceivedAction();
                        radio.clearPacketSentAction();
                        radio.setPacketReceivedAction(setFlagReceive);

                        bEnableInterruptReceive = true;

                        // Flanken-Recovery
                        #ifdef LORA_DIO1
                        if(digitalRead(LORA_DIO1) == HIGH)
                        {
                            receiveFlag = true;
                            if(bLORADEBUG)
                                printlndeb(F("[MC-DBG] RX_TIMEOUT missed_edge recovery"));
                        }
                        #endif

                        // Debug B: RX_RESTART after timeout
                        if(bLORADEBUG)
                        {
                            printfdeb("[MC-SM] IDLE -> RX_LISTEN rc=%d\n", state);
                            printfdeb("[MC-DBG] RX_RESTART src=timeout state=%d\n", state);
                        }

                        if(bLORADEBUG)
                        {
                            printdeb(getTimeString());
                            if (state == RADIOLIB_ERR_NONE)
                                printlndeb(F(" [LoRa]...Receive Timeout, startReceive again with sucess"));
                            else
                            {
                                printdeb(F(" [LoRa]...Receive Timeout, startReceive again with error = "));
                                printlndeb(state);
                            }
                        }
                    }
                }
            }
        }

        // WEB-RX-01: packets parked while the web send buffer blocked the loop go first (oldest first),
        // also when receiveFlag is not set.
        if(rxParkCount() > 0)
        { INSTR_SECTION("lora_rx"); LOOP_SECTION(LSEC_LORA_RX); loraRxParkDrain(); }

        if(receiveFlag || transmittedFlag)
        {
            // check ongoing reception
            if(receiveFlag)
            {
                // Debug C: RX_FLAG_PROCESS
                if(bLORADEBUG)
                {
                    printlndeb("[MC-SM] RX_LISTEN -> RX_PROCESS rc=0\n");
                    printfdeb("[MC-DBG] RX_FLAG_PROCESS ts=%lu\n", millis());
                }

                // reset flags first
                bEnableInterruptReceive = false;
                receiveFlag = false;
                rx_irq_defer_count = 0;

                // DIO triggered while reception is ongoing
                // that means we got a packet

                { INSTR_SECTION("lora_rx"); LOOP_SECTION(LSEC_LORA_RX); checkRX(bRadio); }

                // FIX BUG #2: checkRX() now restarts RX internally.
                // Remove redundant interrupt rewiring that would double-reconfigure.
                // Only reset the timeout timers here.
                inoReceiveTimeOutTime=millis();
                iReceiveTimeOutTime = millis();
                csma_timeout = csma_compute_timeout(cad_attempt);
            }
            else
            if(transmittedFlag)
            {
                // reset flags first
                bEnableInterruptTransmit = false;
                bEnableInterruptReceive = false;

                transmittedFlag = false;

                // Debug G: TX_DONE
                if(bLORADEBUG)
                {
                    printfdeb("[MC-SM] TX_ACTIVE -> TX_DONE rc=%d\n", transmissionState);
                    printfdeb("[MC-DBG] TX_DONE state=%d ts=%lu\n", transmissionState, millis());
                }

                if (transmissionState == RADIOLIB_ERR_NONE)
                {
                    // packet was successfully sent
                    if(bLORADEBUG)
                        printlndeb("transmission finished!");
                }
                else
                {
                    if(bLORADEBUG)
                    {
                        printdeb("failed, code <3> ");
                        printlndeb(transmissionState);
                    }
                }

                // clean up after transmission is finished
                // this will ensure transmitter is disabled,
                // RF switch is powered down etc.
                radio.finishTransmit();  // hat keine Wirkung, ist leere Definition [OE3WAS]

                #ifdef BOARD_HELTEC_V4
                disablePATransmit();
                #endif

                #ifndef BOARD_TLORA_OLV216
                // reset MeshCom now
                if(bSetLoRaAPRS)
                {
                    lora_setchip_meshcom();
                    bSetLoRaAPRS = false;
                }
                #endif

                OnTxDone();

                // Atomarer RX-Restart: Radio in RX BEVOR ISR aktiv
                bEnableInterruptTransmit = false;
                bEnableInterruptReceive = false;
                radio.clearPacketSentAction();
                radio.clearPacketReceivedAction();
                #ifdef RADIO_CTRL
                    digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
                    delay(2);
                #endif
                int state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);           // Radio in RX zuerst
                radio.setPacketReceivedAction(setFlagReceive);
                bEnableInterruptReceive = true;
                radio.setPacketReceivedAction(setFlagReceive); //KBC 0801

                // Verpasste Flanke erkennen: wenn DIO1 bereits HIGH ist, hat ein Paket
                // waehrend der Umschaltung ausgeloest
                #ifdef LORA_DIO1
                if(digitalRead(LORA_DIO1) == HIGH)
                {
                    receiveFlag = true;
                    if(bLORADEBUG)
                        printlndeb(F("[MC-DBG] RX_RESTART missed_edge recovery"));
                }
                #endif

                // Debug H: RX_RESTARTED after TX
                if(bLORADEBUG)
                {
                    printfdeb("[MC-SM] TX_DONE -> RX_LISTEN rc=%d\n", state);
                    printfdeb("[MC-DBG] RX_RESTARTED src=after_tx state=%d\n", state);
                }

                if (state != RADIOLIB_ERR_NONE)
                {
                    if(bLORADEBUG)
                    {
                        printdeb(F("[LoRa]...Starting to listen again (1)... "));
                        printdeb(F("failed, code "));
                        printlndeb(state);
                    }
                }

                inoReceiveTimeOutTime=millis();

                iReceiveTimeOutTime = millis(); // start to wait for next transmit
                csma_reset();
            }
        }

        // Check transmit now
        if(iReceiveTimeOutTime == 0 && !bEnableInterruptTransmit)
        {
            // channel is free
            // nothing was detected
            // do not print anything, it just spams the console
            // TXP W1b: "something sendable in the TX pool" (was iWrite != iRead); not
            // txRingDepth() > 0: a parked own text waiting for its ACK is not sendable
            if (getNextTxSlot() >= 0)
            {
                // Debug E: TX_GATE_ENTER
                // BP-02: qlen is txRingDepth() (occupied slots), not the raw
                // index distance -- see txRingDepth() doc comment.
                if(bLORADEBUG)
                {
                    printfdeb("[MC-SM] IDLE -> TX_PREPARE rc=0\n");
                    printfdeb("[MC-DBG] TX_GATE_ENTER qlen=%d cad_attempt=%d\n",
                        txRingDepth(),
                        cad_attempt);
                }

                // Header/Preamble-Check per IRQ-Register-Polling
                // getIrqFlags() liest das SX1262 IRQ-Status-Register per SPI
                // ohne Flags zu clearen. Wenn HEADER_VALID oder PREAMBLE_DETECTED
                // gesetzt ist, ist ein Paket im Anflug -> TX abbrechen.
                bool irq_rx_active = false;
                {
                    uint32_t irqStatus = radio.getIrqFlags();
                    if(irqStatus & (RADIOLIB_SX126X_IRQ_HEADER_VALID |
                                    RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED))
                    {
                        // RX-DEAF-01: the receive flag is not set here. irq_rx_active aborts the TX;
                        // the flag is owned by the read path (checkRXRead() returns while it is set),
                        // so setting it here kept every later read out until reboot.
                        irq_rx_active = true;
                        if(bLORADEBUG)
                            printfdeb("[MC-DBG] IRQ_POLL hdr/pre=0x%04X -> TX_ABORT\n",
                                          irqStatus);
                        iReceiveTimeOutTime = millis();
                        csma_timeout = csma_compute_timeout(cad_attempt);
                    }
                }

                if(!irq_rx_active)
                {
                // Disable RX interrupt before CAD scan
                bEnableInterruptReceive = false;
                radio.clearPacketReceivedAction();

                if(bLORADEBUG)
                    printfdeb("[CHECK] radio.scanChannel() / 1\n");

                // CAD Scan 1
                #if defined(BOARD_T_ETH_ELITE)
                int cad_result = RADIOLIB_CHANNEL_FREE; //!!!!!!!!!!!!!! ACHTUNG !!!!!!!!!!!!!!!! muss raus
                if(bLORADEBUG)
                    printfdeb("[CHECK] T_ETH_ELITE BUG please fixit\n");
                #else
                int cad_result = radio.scanChannel();
                #endif
                if(bLORADEBUG)
                    printfdeb("[MC-DBG] CAD_SCAN result=%d\n", cad_result);

                bool channel_free = false;
                if(cad_result == RADIOLIB_CHANNEL_FREE)
                {
                    channel_free = true;
                }
                else
                {
                    // Double-Check: second scan to filter false positives
                    if(bLORADEBUG)
                        printfdeb("[MC-DBG] CAD_BUSY_1 attempt=%d, double-check...\n", cad_attempt);

                    if(bLORADEBUG)
                        printfdeb("[CHECK] radio.scanChannel() / 2\n");

                    cad_result = radio.scanChannel();
                    if(bLORADEBUG)
                        printfdeb("[MC-DBG] CAD_SCAN result=%d\n", cad_result);
                    if(cad_result == RADIOLIB_CHANNEL_FREE)
                    {
                        if(bLORADEBUG)
                            printfdeb("[MC-DBG] CAD_FALSE_POSITIVE\n");
                        channel_free = true;
                    }
                }

                if(channel_free)
                {
                    if(bLORADEBUG)
                        printfdeb("[MC-DBG] CAD_FREE attempt=%d\n", cad_attempt);

                    tx_cad_attempt = cad_attempt; // honest cad= in the TX log (csma_reset zeroes it)
                    csma_reset();

                    // set Transmit Interrupt
                    bEnableInterruptTransmit = true;
                    radio.setPacketSentAction(setFlagSent);

                #ifdef BOARD_HELTEC_V4
                enablePATransmit();
                #endif

                    if(doTX())
                    {
                        ch_util_tx_start = millis();
                        // Debug F: TX_START
                        // BP-02: qlen is txRingDepth() (occupied slots), not
                        // the raw index distance -- see txRingDepth() doc.
                        if(bLORADEBUG)
                        {
                            printfdeb("[MC-SM] TX_PREPARE -> TX_ACTIVE rc=0\n");
                            printfdeb("[MC-DBG] TX_START qlen=%d\n",
                                txRingDepth());
                        }
                    }
                    else
                    {
                        #ifdef BOARD_HELTEC_V4
                        disablePATransmit();
                        #endif

                        // doTX() returned false (no ready slot) — restore RX
                        // and set timeout to prevent CAD spin loop
                        if(bLORADEBUG)
                            printfdeb("[MC-DBG] CAD_FREE_NO_TX restoring RX\n");
                        bEnableInterruptTransmit = false;
                        radio.clearPacketSentAction();

                        bEnableInterruptReceive = true;
                        radio.setPacketReceivedAction(setFlagReceive);
                        #ifdef RADIO_CTRL
                            digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
                            delay(2);
                        #endif
                        radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
                        iReceiveTimeOutTime = millis();
                    }
                }
                else
                {
                    // Channel busy confirmed — backoff
                    cad_attempt++;
                    csma_timeout = csma_compute_timeout_nofloor(cad_attempt); // CAD_BUSY: no age floor

                    if(bLORADEBUG)
                    {
                        printfdeb("[MC-SM] TX_PREPARE -> IDLE rc=-1\n");
                        printfdeb("[MC-DBG] CAD_BUSY attempt=%d next_timeout=%lu\n",
                            cad_attempt, csma_timeout);
                    }

                    // Restore RX
                    bEnableInterruptReceive = true;
                    radio.setPacketReceivedAction(setFlagReceive);
                    #ifdef RADIO_CTRL
                        digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
                        delay(2);
                    #endif
                    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);

                    iReceiveTimeOutTime = millis();
                }
                } // end if(!irq_rx_active)
            }
            else
            {
                // Nothing to send — restart timeout cycle
                iReceiveTimeOutTime = millis();
            }
        }
    } // bRadio active

    #endif

    // RM-05 (#1189): authenticated remote-management DMs queued by OnRxDone; crypto, execution,
    // flash write and the reply run here, in the loop task. Outside the T5 / local-radio #if above
    // on purpose: every board (and the external-radio path) feeds the same queue.
    rmDrain();

    // get RTC Now
    // RTC hat Vorrang zu Zeit via MeshCom-Server
    bool bMyClock = true;

    // !posinfo_fix && !bNTPDateTimeValid
    // Time NTP
    // NTP-01 Nachtrag (Bench-Regression): mit GPS-Fix lief dieser Block nie,
    // ein manuelles --ntpsync setzte also nur _nextDueMs und nichts sendete.
    // isPending() haelt den Pump offen, bis der manuelle Request ok/timeout
    // gemeldet hat; danach schliesst das GPS-Gate wieder wie gehabt.
    extern NtpAsync timeClient;
    if(meshcom_settings.node_hasIPaddress && (!posinfo_fix || timeClient.isPending()))
    {
        strTime = "none";

        // every 15 minutes
        if((uint32_t)(millis() - updateTimeClient) >= 1000 * 60 * 15 || updateTimeClient == 0)
        {
            strTime = udpUpdateTimeClient();

            updateTimeClient = millis();

        }
        else
        {
            strTime = udpGetTimeClient();
        }

        if(strTime.compareTo("none") != 0)
        {
            strDate = udpGetDateClient();

            uint16_t Year = (uint16_t)strDate.substring(0, 4).toInt();
            uint16_t Month = (uint16_t)strDate.substring(5, 7).toInt();
            uint16_t Day = (uint16_t)strDate.substring(8, 10).toInt();

            uint16_t Hour = (uint16_t)strTime.substring(0, 2).toInt();
            uint16_t Minute = (uint16_t)strTime.substring(3, 5).toInt();
            uint16_t Second = (uint16_t)strTime.substring(6, 8).toInt();
        
            // check valid Date & Time
            if(Year > 2023 && strTime.compareTo("none") != 0)
            {
                MyClock.setCurrentTime(meshcom_settings.node_utcoff, Year, Month, Day, Hour, Minute, Second);
                bNTPDateTimeValid = true;

                snprintf(cTimeSource, sizeof(cTimeSource), (char*)"NTP");
            }
            else
                bNTPDateTimeValid = false;
        }
        else
        {
            snprintf(cTimeSource, sizeof(cTimeSource), (char*)"CPU");
            bNTPDateTimeValid = false;
        }

    }
    #if defined(ENABLE_RTC)
    else
    if(bRTCON)
    {
        bMyClock = false;

        loopRTC();

        if(posinfo_fix) // GPS hat Vorang zur RTC und setzt RTC
        {
            if(meshcom_settings.node_date_year > 2023 && rtcRefreshDue(millis(), rtc_refresh_timer))
            {
                //only every minute
                // RTC haelt UTC, node_date_* sind Knotenzeit (RTC-1)
                DateTime wutc((uint32_t)rtcUtcFromLocalFields(meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, meshcom_settings.node_utcoff));
                setRTCNow(wutc.year(), wutc.month(), wutc.day(), wutc.hour(), wutc.minute(), wutc.second());
                rtc_refresh_timer = millis() | 1;   // 0 = noch nie geschrieben
            }
        }
        else
        {
            DateTime utc = getRTCNow();

            // RTC-UTC + Offset genau einmal, setCurrentTime() mit fUTC = 0
            DateTime now ((uint32_t)rtcNodeEpochFromUtc(utc.unixtime(), meshcom_settings.node_utcoff));

            uint16_t Year = now.year();
            uint16_t Month = now.month();
            uint16_t Day = now.day();

            uint16_t Hour = now.hour();
            uint16_t Minute = now.minute();
            uint16_t Second = now.second();

            //printfdeb("%04i.%02i.%02i %02i:%02i:%02i\n", Year, Month, Day, Hour, Minute, Second);

            // check valid Date & Time
            if(Year > 2013)
            {
                MyClock.setCurrentTime(0.0, Year, Month, Day, Hour, Minute, Second);
                snprintf(cTimeSource, sizeof(cTimeSource), (char*)"RTC");

                bMyClock = true;
            }
        }
    }
    #endif
    else
    {
        bNTPDateTimeValid = false;
    }
    
    if(bMyClock)
    {
        MyClock.CheckEvent();
        
        if(MyClock.Year() > 2023)
            meshcom_settings.node_date_year = MyClock.Year();
        else
            meshcom_settings.node_date_year = 2000;

        meshcom_settings.node_date_month = MyClock.Month();
        meshcom_settings.node_date_day = MyClock.Day();

        meshcom_settings.node_date_hour = MyClock.Hour();
        meshcom_settings.node_date_minute = MyClock.Minute();
        meshcom_settings.node_date_second = MyClock.Second();

        // Starttime setzen
        if(meshcom_settings.node_date_year > 2023 && meshcom_settings.node_update[0] == 0x00 && memcmp(cTimeSource, "INIT", 4) != 0)
        {
            char ctemp[80];
            snprintf(ctemp, sizeof(ctemp), "%04i-%02i-%02i %02i:%02i:%02i",
             meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);

            // This was `memcpy(..., ctemp, 21)` against a char[21] field, but
            // the snprintf above fills exactly 20 bytes (19-char timestamp +
            // NUL) -- so the 21st byte copied was UNINITIALISED STACK, and it
            // landed in a settings field. The field is char[20] now and the
            // copy is bounded by its own size with the terminator written
            // explicitly, so neither the length nor the NUL depends on what
            // snprintf happened to leave behind.
            //
            // Not formatted straight into node_update the way nrf52_main.cpp
            // does it: the date fields are plain `int`, so xtensa-gcc cannot
            // bound "%02i" to two characters and -Werror=format-truncation
            // rejects a 20-byte target. Formatting into the wide scratch and
            // copying a provable number of bytes keeps both the bound and the
            // warning honest.
            memcpy(meshcom_settings.node_update, ctemp, sizeof(meshcom_settings.node_update) - 1);
            meshcom_settings.node_update[sizeof(meshcom_settings.node_update) - 1] = 0x00;
        }

        #if defined(ENABLE_RTC)
        // Ausserhalb des Einmal-pro-Boot-Blocks: gueltige NTP-Zeit frischt den RTC
        // einmal pro Minute auf (RTC-3), und zwar in UTC (RTC-1).
        if(bRTCON && bNTPDateTimeValid && meshcom_settings.node_date_year > 2023) // NTP hat Vorang zur RTC und setzt RTC
        {
            if(rtcRefreshDue(millis(), rtc_refresh_timer))
            {
                //only every minute
                DateTime wutc((uint32_t)rtcUtcFromLocalFields(meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second, meshcom_settings.node_utcoff));
                setRTCNow(wutc.year(), wutc.month(), wutc.day(), wutc.hour(), wutc.minute(), wutc.second());

                rtc_refresh_timer = millis() | 1;   // 0 = noch nie geschrieben
            }
        }
        #endif
    }

    #if defined(ENABLE_GPS)

    #if defined(GPS_SWITCH)
        pinMode(GPS_SWITCH, OUTPUT);
    #endif

    if(bGPSON)
    {
        #if defined(GPS_SWITCH)
            digitalWrite(GPS_SWITCH, HIGH);
        #endif

        WZ_GPS_Init();
    }
    else
    {
        #if defined(GPS_SWITCH)
            digitalWrite(GPS_SWITCH, LOW);
        #endif
    }
    #endif

    // AU-05 (#1187): firmware auto update tick (self-gated to once per second)
    auTick();

    // check WiFI connected with Ping every 30 sec
    if(meshcom_settings.node_netmode == 0 && (uint32_t)(millis() - wifi_active_timer) >= 30000)
    {
        INSTR_SECTION("wifi_ping");
        LOOP_SECTION(LSEC_WIFI_PING);
        if(!checkWifiPing())
        {
            if(ifalseping > 0)
            {
               wifi_active_timer = millis() - 25000;   // next ping max. 5 sec
            }
            else
            {
                // restart WEB-Client
                if(bWEBSERVER)
                    stopWebserver();

                startNetwork();

                ifalseping = 5;
                
                wifi_active_timer = millis();
            }
        }
        else
        {
            ifalseping = 5;
            
            wifi_active_timer = millis();
        }
    }

    // SOFTSER
    #if defined(ENABLE_SOFTSER)
        if(bSOFTSERON)
        {
            // Reset Node on XML not working
            #if defined(ENABLE_XML)
            extern unsigned long lTELE_TIMER;
            if(lTELE_TIMER == 0)
            {
                lTELE_TIMER = millis();
            }

            // check every 15 minuten to check telemetry via serial interface is ok
            if ((int32_t)(millis() - (lTELE_TIMER + (15 * 60 * 1000))) > 0)
            {
                printfdeb("[SOFTSER] Reset Node, XML not working\n");
                delay(1000);
                ESP.restart();
            }
            #endif

            // check every 5 seconds to ready next telemetry via serial interface
            if ((uint32_t)(millis() - softser_refresh_timer) >= 5000 && softserFunktion == 0)
            {
                if(lastSOFTSER_MINUTE != meshcom_settings.node_date_minute && meshcom_settings.node_date_second > 20)
                {
                    if(meshcom_settings.node_date_minute % SOFTSER_REFRESH_INTERVAL  == 0)
                    {
                        lastSOFTSER_MINUTE = meshcom_settings.node_date_minute;

                        strSOFTSER_BUF="";

                        // start SOFTSER APP
                        loopSOFTSER(SOFTSER_APP_ID);
                    }
                }

                softser_refresh_timer = millis();
            }
            else
            if (bSOFTSER_APP)
            {
                bSOFTSER_APP = false;

                softserFunktion = 0;

                strSOFTSER_BUF="";
                
                lastSOFTSER_MINUTE = meshcom_settings.node_date_minute;
                
                // start SOFTSER APP
                loopSOFTSER(SOFTSER_APP_ID);
            }
            else
            {
                appSOFTSER(SOFTSER_APP_ID);
            }
        }
    #endif

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    tdeck_update_header_standby();
    #endif

    #if defined (ANALOG_PIN)
    loop_ADCFunctions();    // OE3WAS
    #endif

    // BLE
    if (deviceConnected)
    {
    	g_ble_uart_is_connected = true;

        // Disconnect if app-layer auth failed. AU-10: pServer is NULL and the stack stopped while
        // an update job has BLE paused; the request stays pending until the lock is free.
        if(ble_disconnect_requested && bleLifeLock())
        {
            ble_disconnect_requested = false;
            if(pServer != NULL)
                pServer->disconnect(g_ble_conn_handle);
            bleLifeUnlock();
        }
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        /* KBC/KFR TEST 2025-04-10
        delay(2000); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        */

        oldDeviceConnected = deviceConnected;

        // BLC-01: the session reset itself runs in onConnect/onDisconnect. Only the
        // loop-owned flag is cleared here (the loop sets it while connected), and
        // only if no new central connected in the meantime.
        if(!deviceConnected)
            g_ble_uart_is_connected = false;
        if(bBLEDEBUG)
            Serial.printf("[BLE ];edge;down;ms;%lu\n", (unsigned long)millis());

        // Update T-Deck header so BT icon reflects disconnected state
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_update_header_bt();
        #endif
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
 		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;

        // Update T-Deck header so BT icon reflects connected state
        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_update_header_bt();
        #endif
    }

    #if !defined(DISABLE_BLE)
    // BLC-01: advertising self-check. The library restarts advertising on
    // disconnect and on a failed connect; if that ever fails the node would stay
    // unconnectable until reboot. Every 5 s while no central is connected, make
    // sure advertising runs, else start it and count. Not before the late start
    // of BENCH_BLE_ADV_LATE builds.
    {
        static uint32_t s_bleAdvChk = 0;
        if((uint32_t)(millis() - s_bleAdvChk) >= 5000UL)
        {
            s_bleAdvChk = millis();

            // AU-10: a resume that failed after an update job is retried here (30 s apart, 5 times;
            // blocks the loop for the NimBLE bring-up, about a second). Not while the job runs:
            // it retries itself three times.
            static uint32_t s_bleRetryMs = 0;
            static uint8_t s_bleRetries = 0;
            if(s_bleDown && s_bleResumeFailed && !s_bleGuard && s_bleRetries < 5 &&
               (uint32_t)(millis() - s_bleRetryMs) >= 30000UL)
            {
                FwNetStatus _fst;
                fwNetGetStatus(_fst);
                if(_fst.state != FWS_BUSY)
                {
                    s_bleRetryMs = millis();
                    s_bleRetries++;
                    esp32BleResume();
                }
            }
            if(!s_bleDown)
                s_bleRetries = 0;

            // AU-10: the lock fails while BLE is paused or being resumed (and pServer is NULL then)
            if(!deviceConnected && pServer != NULL && pService != NULL
               #if defined(BENCH_BLE_ADV_LATE)
               && s_bootReadyLogged
               #endif
               && bleLifeLock()   // last: it takes the lock, the body releases it
              )
            {
                NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
                if(pAdv != NULL && !pAdv->isAdvertising())
                {
                    bool ok = pAdv->start();
                    bleStatsOnAdvRestart(g_bleStats);
                    if(bBLEDEBUG)
                        Serial.printf("[BLE ];adv_restart;ms;%lu;ok;%d\n", (unsigned long)millis(), (int)ok);
                }
                bleLifeUnlock();
            }
        }

        // --bledebug on: counters every 15 min (short line, see Print::printf malloc note)
        static uint32_t s_bleStatTimer = 0;
        if(bBLEDEBUG && (uint32_t)(millis() - s_bleStatTimer) >= 900000UL)
        {
            s_bleStatTimer = millis();
            char sb[128];
            bleStatsFormat(g_bleStats, sb, sizeof(sb));
            Serial.print("[BLE ];stat;");
            Serial.println(sb);
        }
    }
    #endif    // check if message from phone to send

    // BLE Queue: process data from NimBLE task in Main Loop context
    {
        BleQueueItem bleItem;
        while (xQueueReceive(bleQueue, &bleItem, 0) == pdTRUE) {
            { INSTR_SECTION("ble_cmd"); LOOP_SECTION(LSEC_BLE_CMD); readPhoneCommand(bleItem.data); }
        }
    }

    if(hasMsgFromPhone)
    {
        if(bBLEDEBUG)
            printfdeb("[LOOP] hasMsgFromPhone\n");
        
        if(memcmp(textbuff_phone, ":", 1) == 0)
        {
            // BP-01: tag the origin so the back-pressure notice goes back to
            // the phone that just typed, and never over the air.
            setMsgOrigin(ORIGIN_BLE);
            (void)sendMessage(textbuff_phone, txt_msg_len_phone);
            setMsgOrigin(ORIGIN_NONE);
        }

        if(memcmp(textbuff_phone, "-", 1) == 0)
            commandAction(textbuff_phone, isPhoneReady, true);

        hasMsgFromPhone = false;
    }

    if (isPhoneReady == 1)
    {
        if (config_to_phone_prepare)
        {
            for(int config_cmds_index=0; config_cmds_index < json_configs_cnt; config_cmds_index++)
            {
                commandAction((char*)config_cmds[config_cmds_index], isPhoneReady, true);

                //sendMessage((char*)config_cmds[config_cmds_index], strlen(config_cmds[config_cmds_index]));
            }

            mhPhoneListStart(); // MHeard-Liste erst, wenn der Kommando-Ring leer ist (siehe unten)

            config_to_phone_prepare_timer = millis();

            config_to_phone_prepare = false;
        }
        else
        {
            // wait after BLE Connect 3 sec.
            if((uint32_t)(millis() - config_to_phone_prepare_timer) < 3000)
                iPhoneState = 0;

            if (iPhoneState > 3)   // only every 3 times of mainloop send to phone - main loop has no additional delay anymore! 14.06.2025
            {
                // prepare JSON config to phone after BLE connection
                // send JSON config to phone after BLE connection
                // wait at least 300ms between sending messages
                if (!bf_empty(&phoneComRing))
                {
                    // check every 300 ms to send to phone
                    if ((uint32_t)(millis() - ble_wait) >= 300)
                    {
                        sendComToPhone();

                        ble_wait = millis();
                    }
                }
                else if (mhPhoneListPending())
                {
                    // Kommando-Ring leer: naechste Portion der MHeard-Liste nachlegen
                    mhPhoneListStep();
                }
                else if (!bf_empty(&phoneRing))
                {
                    // wait for each message to send to phone
                    if ((uint32_t)(millis() - ble_wait) >= 400)
                    {
                        sendToPhone();

                        ble_wait = millis();
                    }
                }
                else if (!conffin_sent)
                {
                    // both queues empty — send config finish once
                    commandAction((char*)"--conffin", isPhoneReady, true);
                    conffin_sent = true;
                }

                iPhoneState = 0;
            }
            else
                iPhoneState++;
        }

        // 5 minuten
        if((uint32_t)(millis() - config_to_phone_datetime_timer) >= (5 * 60 * 1000))
        {
            bNTPDateTimeValid=false;

            config_to_phone_datetime_timer = millis();
        }
    }


    // D1-10 loop scheduler: mcp_refresh_timer moved to the scheduler call above.

    // gps display refresh every 5 sec
    gps_refresh_intervall = GPS_REFRESH_INTERVAL;

    #if defined(GPS_MODULE_REFRESH_INTERVAL)
        gps_refresh_intervall = GPS_MODULE_REFRESH_INTERVAL;
    #endif

    // TRACK ON
    if(bDisplayTrack)
    {
        gps_refresh_intervall = 1.0;
    }

    #ifdef ENABLE_GPS
    if (bGPSON && gpsDetected) { INSTR_SECTION("gps_feed"); LOOP_SECTION(LSEC_GPS); WZ_GPS_Feed(); }
    #endif

    if ((uint32_t)(millis() - gps_refresh_timer) >= ((unsigned long)gps_refresh_intervall * 1000))
    {
        #ifdef ENABLE_GPS

        unsigned int igps=0;
            
        if(!bGPSON)
        {
            #if defined (BOARD_T_DECK_PRO)
            tdeck_set_gps(false);
            #endif

            if(meshcom_settings.node_postime > 0)
            {
                igps = (unsigned int)meshcom_settings.node_postime;
            }
            else
            {
                igps =  POSINFO_INTERVAL;
            }

            posinfo_fix = false;
            posinfo_satcount = 0;
            fposinfo_hdop = 0.0;
        }
        else
        {
            #if defined (ENABLE_GPS)
                if(gpsDetected)
                {
                    { INSTR_SECTION("gps"); LOOP_SECTION(LSEC_GPS); igps = WZ_GPS_Loop(); }
                    
                    if(iGPSDEBUG > 0)
                    {
                        printfdeb("[GPS ]...fix:%s sat:%i hdop:%.1lf\n", (posinfo_fix?"yes":"no"), gpsData.satellites, gpsData.hdop);

                        printdeb("[GPS ]...Time <UTC>: ");
                        if (gpsData.hour < 10) printdeb(F("0"));
                        printdeb(gpsData.hour);
                        printdeb(F(":"));
                        if (gpsData.minute < 10) printdeb(F("0"));
                        printdeb(gpsData.minute);
                        printdeb(F(":"));
                        if (gpsData.second < 10) printdeb(F("0"));
                        printdeb(gpsData.second);

                        printdeb(F(" / Date: "));
                        printdeb(gpsData.year);
                        printdeb(F("."));
                        if (gpsData.month < 10) printdeb(F("0"));
                        printdeb(gpsData.month);
                        printdeb(F("."));
                        if (gpsData.day < 10) printdeb(F("0"));
                        printlndeb(gpsData.day);

                        if(posinfo_fix)
                        {
                            printfdeb("[GPS ]...position  : lat:%.6lf lon:%.6lf alt:%.1lf\n", gpsData.latitude, gpsData.longitude, gpsData.altitude);
                        }

                    }
                }
                
            #else

                #if defined (BOARD_T_DECK_PRO)
                    tdeck_set_gps(true);
                    igps = tdeck_get_gps();
                #endif

            #endif // ENABLE_GPS
        }

        if(igps > 0)
            posinfo_interval = igps;
        else
        {
            no_gps_reset_counter++;
            if(no_gps_reset_counter > 10)
            {
                posinfo_interval = POSINFO_INTERVAL;
                no_gps_reset_counter = 0;
            }
        }


        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
            gps_refresh_track++;
            if(gps_refresh_track > 4)
            {
                tdeck_refresh_track_view();
                gps_refresh_track=0;
            }

            // SD-Kartenmodus: alle 30 Sekunden pruefen, ob die eigene Position die
            // aktuell geladene Kachel verlassen hat, und bei Bedarf nachladen -
            // nur solange der Kartenbildschirm auch tatsaechlich sichtbar ist.
            static unsigned long sdmap_boundary_timer = 0;
            if ((uint32_t)(millis() - sdmap_boundary_timer) >= 30000UL)   // N-08 idiom, rollover-safe
            {
                sdmap_boundary_timer = millis();

                // TD-07: kein Auto-Recenter, solange der Nutzer manuell gepannt hat
                if (lv_tabview_get_tab_act(tv) == 3 && !tdeck_map_user_panned() && (gpsData.latitude != 0.0 || gpsData.longitude != 0.0))
                {
                    if (!sdmap_in_current_tile(gpsData.latitude, gpsData.longitude))
                    {
                        sdmap_lastKnownLat = gpsData.latitude;
                        sdmap_lastKnownLon = gpsData.longitude;

                        if (sdmap_lastKnownLat == 0.0 && sdmap_lastKnownLon == 0.0)
                        {
                            sdmap_lastKnownLat = meshcom_settings.node_lat;
                            sdmap_lastKnownLon = meshcom_settings.node_lon;
                        }

                        sdmap_refresh(map_ta, sdmap_lastKnownLat, sdmap_lastKnownLon, "boundary");
                        refresh_map(meshcom_settings.node_map);
                    }
                }
            }
        #endif

        #endif

        gps_refresh_timer = millis();
    }

    // check NCNT modified -- MeshCom 5 (Welle 4, Konzept 4.8): lokaler
    // Vergleich, nicht die Sendefassung -- nbrNcnt(), nicht nbrNcntAir().
    int incnt = nbrNcnt(nbrMatrix, uptimeMin16());
    if(ncnt_hold != incnt)
    {
        // minimal alle 60 sec
        if((uint32_t)(millis() - posinfo_timer_min) >= 60000)
        {
            posinfo_shot = true;
            ncnt_hold = incnt;
        }
    }

    // posinfo_interval in Seconds
    if (((uint32_t)(millis() - posinfo_timer) >= (posinfo_interval * 1000)) || (millis() > 100000 && millis() < 130000 && bPosFirst) || posinfo_shot)
    {
        // minimal transmit time only max 30 sec
        if((uint32_t)(millis() - posinfo_timer_min) >= 30000)
        {
            if(bDisplayInfo)
            {
                printdeb(getTimeString());
                printfdeb(" [POS]...sendPostion initialized F:%i S:%i\n", bPosFirst, posinfo_shot);
            }

            bPosFirst = false;

            // posinfo_prev is SmartBeaconing's TRACK point; with TRACK off (or none yet) send the node position
            if(posinfo_shot && bDisplayTrack && (posinfo_prev_lat != 0.0 || posinfo_prev_lon != 0.0))
            {
                double slat = 0.0;
                double slon = 0.0;
                
                double slatr=60.0;
                double slonr=60.0;
                
                slat = (int)posinfo_prev_lat;
                slatr = (posinfo_prev_lat - slat) * slatr;
                slat = (slat * 100.) + slatr;
                
                slon = (int)posinfo_prev_lon;
                slonr = (posinfo_prev_lon - slon) * slonr;
                slon = (slon * 100.) + slonr;
            
                double node_lat = cround4(posinfo_prev_lat);
                double node_lon = cround4(posinfo_prev_lon);

                char node_lat_c = 'N';
                char node_lon_c = 'E';

                if(posinfo_prev_lat < 0.0)
                    node_lat_c='S';
                    
                if(posinfo_prev_lon < 0.0)
                    node_lon_c='W';

                sendPosition(posinfo_interval, node_lat, node_lat_c, node_lon, node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);
            }
            else
            {
                sendPosition(posinfo_interval, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);
            }

            posinfo_shot=false;

            posinfo_prev_lat = 0.0; // done
            posinfo_prev_lon = 0.0; // done

            posinfo_last_lat=posinfo_lat;
            posinfo_last_lon=posinfo_lon;

            posinfo_last_direction=posinfo_direction;
            posinfo_distance = 0.0;

            posinfo_timer = millis();

            if(pos_shot)
            {
                commandAction((char*)"--pos", isPhoneReady, false);
                pos_shot = false;
            }

            posinfo_timer_min = millis();
        }
    }
    else
    {
        posinfo_timer_min = millis();
    }


    // Trickle-HEY: adaptive interval (RFC 6206)
    unsigned long extra_hey_time = 0;
    #if defined(ENABLE_SOFTSER)
        extra_hey_time = 10 * 60 * 1000; // 10 minutes extra
    #endif
    if (((uint32_t)(millis() - heyinfo_timer) >= (trickle_interval_ms + extra_hey_time)) || (bHeyFirst && bAllStarted))
    {
        bHeyFirst = false;

        // Check for topology change (neighbor count changed) -- lokaler
        // Vergleich (Konzept 4.8: die Trickle-Ruecksetzung ist kein Sender).
        int current_neighbors = nbrNcnt(nbrMatrix, uptimeMin16());
        if(trickle_last_neighbor_count >= 0 && current_neighbors != trickle_last_neighbor_count)
        {
            // Topology changed — reset to fastest interval
            trickle_interval_ms = TRICKLE_IMIN_S * 1000UL;
            trickle_consistent_count = 0;
            if(bLORADEBUG)
                printfdeb("[MC-TRICKLE] TOPO_CHANGE neighbors=%d->%d interval_reset=%lums\n",
                    trickle_last_neighbor_count, current_neighbors, trickle_interval_ms);
        }
        trickle_last_neighbor_count = current_neighbors;

        // Trickle suppression: skip HEY if enough consistent HEYs heard
        // F6: a gateway keeps sending at least one HG per Imax (hey_policy.h).
        if(heyTrickleSuppress(trickle_consistent_count))
        {
            if(bLORADEBUG)
                printfdeb("[MC-TRICKLE] SUPPRESS consistent=%d>=k=%d interval=%lums neighbors=%d\n",
                    trickle_consistent_count, TRICKLE_K, trickle_interval_ms, current_neighbors);
        }
        else
        {
            sendHey();
            if(bLORADEBUG)
                printfdeb("[MC-TRICKLE] SEND consistent=%d<k=%d interval=%lums neighbors=%d\n",
                    trickle_consistent_count, TRICKLE_K, trickle_interval_ms, current_neighbors);
        }

        // Double interval (Trickle step), cap at Imax
        trickle_interval_ms = min(trickle_interval_ms * 2, (unsigned long)(TRICKLE_IMAX_S * 1000UL));
        trickle_consistent_count = 0;

        heyinfo_timer = millis();
    }

    // HN-Bericht (Nachbarschaftsmatrix Stufe 3, --nbrreport): eigener Takt,
    // unabhaengig vom Trickle-Intervall oben und nie unterdrueckt -- siehe
    // nbrReportTick() in loop_functions.cpp fuer Zeitplan und Modus-Auswertung.
    nbrReportTick();

    // --nbrdebug (24-h-Dauertest der Nachbarschaftsmatrix): 15-Minuten-Takt fuer
    // nbrLogSnapshot(). Nur im Loop, NICHT im Timer-Task -- laesst sich hier nicht
    // verletzen, esp32loop() ist der Loop-Task. Laeuft nur, wenn das Flag gesetzt
    // ist; nbrDebugApply() setzt nbrsnap_timer beim Einschalten zurueck, damit der
    // erste Schnappschuss nicht erst 15 Minuten nach dem Einschalten kommt.
    if(bNBRDEBUG && (uint32_t)(millis() - nbrsnap_timer) >= 900000UL)
    {
        nbrsnap_timer = millis();
        nbrLogSnapshot(nbrMatrix, uptimeMin16());
    }

    // Nachbarschaftsmatrix (Welle 2, edge pool): Minuten-Sweep (nbrSweep(),
    // CONTRACT in nbr_matrix.h) -- IMMER, unabhaengig von --nbrdebug (das
    // steuert nur den 15-Minuten-Schnappschuss oben). Loop-Task, nicht
    // Timer-Task, wie nbrLogSnapshot() oben. nbrSweep() selbst ist ein No-Op
    // bei einem zweiten Aufruf in derselben Minute; die Minuten-Waechter hier
    // spart trotzdem den Funktionsaufruf bei jedem Loop-Durchlauf.
    {
        static uint16_t s_nbr_sweep_min = 0xFFFF;
        uint16_t now_min_sweep = uptimeMin16();
        if(now_min_sweep != s_nbr_sweep_min)
        {
            s_nbr_sweep_min = now_min_sweep;
            nbrSweep(nbrMatrix, now_min_sweep);
            // Konsistenzpruefung Masken <-> Kantenpool (nbrLogCheck(), [NBR]|CHECK),
            // einmal je Minute, nur bei --nbrdebug: auf nRF52 das Instrument fuer
            // Task-Wechsel mitten in einer Aenderung (Konzept 5).
            if(bNBRDEBUG)
                nbrLogCheck(nbrMatrix, now_min_sweep);
        }
    }

    // TELEMETRY_INTERVAL in Minutes == 15 minutes default
    unsigned long akt_timer = meshcom_settings.node_parm_time;
    if(akt_timer < 5 || akt_timer > 120)
    {
        akt_timer = 15;   // default 15 minutes (TELEMETRY_INTERVAL is 30*60 and read as minutes = 30 h)
    }
    
    akt_timer = akt_timer * 1000 * 60; // convert to minutes

    if(bSOFTSERON)
    {
        akt_timer = 20 * 1000; // 20 Seconds PARM, UNIT, EQNS and 1st T-Message
    }
        
    if (((uint32_t)(millis() - telemetry_timer) >= akt_timer) || (bTeleFirst && bAllStarted))
    {
        bTeleFirst=false;

        // APP-ID , Header only flag
        sendTelemetry(SOFTSER_APP_ID);

        telemetry_timer = millis();
    }

    if(meshcom_settings.node_pingcall[0] == 0x00 || meshcom_settings.node_pingtime == 0 || meshcom_settings.node_pingcount == 0)
        { INSTR_SECTION("display_tick"); LOOP_SECTION(LSEC_DISPLAY); mainStartTimeLoop(); }

    #if not defined(BOARD_T_DECK_PRO)
    if(DisplayOffWait > 0)
    {
        if ((int32_t)(millis() - DisplayOffWait) > 0)
        {
            DisplayOffWait = 0;
            if(bDisplayOff)
            {
                bDisplayIsOff=true;

                #if defined(BOARD_HELTEC_T114) || defined(BOARD_T_CONNECT_PRO)
                digitalWrite(PIN_TFT_LEDA_CTL, HIGH);   // TFT OFF
                #endif

                if(meshcom_settings.node_pingcall[0] == 0x00 || meshcom_settings.node_pingtime == 0 || meshcom_settings.node_pingcount == 0)
                    sendDisplay1306(true, true, 0, 0, (char*)"#C");
            }
        }
    }
    #endif

    // rebootAuto
    if(rebootAuto > 0)
    {
        if ((int32_t)(millis() - rebootAuto) > 0)
        {
            rebootAuto = 0;

            #ifdef ESP32
                ESP.restart();
            #endif
        }
    }

    checkSerialCommand();

    // TM-06: service --loratx / --injectraw with the radio idle -- OnRxDone()'s
    // own exit points only fire on RX traffic, which would stall a staged
    // burst or injection on a quiet bench.
    if(tx_is_active == false && is_receiving == false)
        test_inject_service();

    // D1-10 loop scheduler: BattTimeWait (the battery sampler tick, 100 ms since the
    // battery consolidation; the sampler itself decides when to read) moved to
    // the scheduler call above; its zero-init stays here (see the comment
    // there).

    // [OE3WAS] Lüftersteuerung -- split off from the battery-read block above into its own
    // 0.5 sec timer (DRY unification, operator decision 2026-09-11): BattTimeWait was then
    // 30 sec (it is the 100 ms sampler tick since the battery consolidation), and this 1W T-Beam
    // fan/NTC control must not depend on the battery cadence to react to overtemp -- that would be a thermal regression. Only
    // LilyGo_T-Beam-1W defines NTC_PIN/FAN_CTRL, so this whole block compiles away elsewhere.
    #if defined(NTC_PIN) && defined(FAN_CTRL) // BOARD_TBEAM_1W
    static unsigned long FanTimeWait = 0;
    static int FanWaitCounter = 0;

    if(FanTimeWait == 0)
        FanTimeWait = millis() - 500;

    if ((uint32_t)(millis() - FanTimeWait) >= 500)  // 0.5 sec OE3WAS -- kept fast on purpose, see comment above
    {
        if (tx_is_active == false && is_receiving == false)
        {
            FanWaitCounter++;

            float NTCtemp = getTempForNTC();
            if (NTCtemp > 40.0)
            {
                 digitalWrite(FAN_CTRL, HIGH);
            }
            else
            {
                if (NTCtemp < 35.0)
                    digitalWrite(FAN_CTRL, LOW);
            }

            if(bWXDEBUG && FanWaitCounter > 20)
                printfdeb("%s;[TEMP];%.2f;%s\n", getTimeString().c_str(), NTCtemp, digitalRead(FAN_CTRL) ? "on" : "off");

            meshcom_settings.node_ntctemp = NTCtemp;

            if(digitalRead(FAN_CTRL) == HIGH)
                meshcom_settings.node_fanon = true;
            else
                meshcom_settings.node_fanon = false;

            FanTimeWait = millis();

            if(FanWaitCounter > 20)
            {
                FanWaitCounter = 0;
            }
        }
    }
    #endif

    // D1-10 loop scheduler: heapMonTimer (always-active 60 s heap monitor)
    // moved to the scheduler call above; its zero-init now lives inside
    // loopEnabled_heapMon() (loop_actions_esp32.cpp).

    #ifdef OneWire_GPIO
    if(bONEWIRE)
    {
        if ((uint32_t)(millis() - onewireTimeWait) >= 30000)  // 30 sec
        {
            unsigned long lreduction = 0;

            if (tx_is_active == false && is_receiving == false)
            {
                if(one_found)
                {
                    if(loop_onewire_ds18())
                    {
                        one_start = 0;
                    }
                    else
                    {
                        if(one_start > 0)
                            lreduction = 29000;
                    }
                }

                if(dht_found)
                    loop_onewire_dht();

                onewireTimeWait = millis() - lreduction;

                if(wx_shot)
                {
                    commandAction((char*)"--wx", isPhoneReady, false);
                    wx_shot = false;
                }
            }
        }
    }
    #endif

    // read BMP Sensor
    #if defined(ENABLE_BMX280) || defined(ENABLE_AHT20) || defined(ENABLE_SHT21)
    if(((bBMPON || bBMEON) && bmx_found) || (bAHT20ON && aht20_found) || (bSHT21ON && sht21_found))
    {
        unsigned long lreduction = 0;

        if ((uint32_t)(millis() - BMXTimeWait) >= 60000)   // 60 sec
        {
            #if defined(ENABLE_BMX280)
                if(loopBMX280())
                {
                    if(!aht20_found && !bmp3_found)
                    {
                        meshcom_settings.node_temp = getTemp();
                    }

                    if(!aht20_found)
                    {
                        meshcom_settings.node_hum = getHum();
                    }

                    if(!bmp3_found)
                    {
                        meshcom_settings.node_press = getPress();
                        meshcom_settings.node_press_alt = getPressALT();
                        meshcom_settings.node_press_asl = getPressASL(meshcom_settings.node_alt);
                    }

                    bmx_start = 0;
                }
                else
                {
                    if(bmx_start > 0)
                        lreduction = 58000;
                }
            #endif


            #if defined(ENABLE_AHT20)
                if(loopAHT20())
                {
                    meshcom_settings.node_temp = getAHT20Temp();
                    meshcom_settings.node_hum = getAHT20Hum();
                }
            #endif

            #if defined(ENABLE_SHT21)
                if(loopSHT21())
                {
                    meshcom_settings.node_temp2 = getSHT21Temp();
                    meshcom_settings.node_hum = getSHT21Hum();
                }
            #endif

            if(wx_shot)
            {
                commandAction((char*)"--wx", isPhoneReady, false);
                wx_shot = false;
            }

            BMXTimeWait = millis() - lreduction; // wait for next messurement
        }
    }
    #endif

    // D1-10 loop scheduler: BMP3TimeWait, MCU811TimeWait and INA226TimeWait
    // moved to the scheduler call above (bodies/guards in
    // loop_actions_esp32.cpp).

    // read every n seconds the bme680 sensor calculated from millis()
    #if defined(ENABLE_BMX680)
    if(bBME680ON && bme680_found)
    {
        if ((uint32_t)(millis() - bme680_timer) >= 60000 || delay_bme680 <= 0)
        {
            #if defined(ENABLE_BMX280)
                
                if (delay_bme680 <= 0)
                {
                    getBME680();

                }

                if(wx_shot)
                {
                    commandAction((char*)"--wx", isPhoneReady, false);
                    wx_shot = false;
                }

                // calculate delay
                delay_bme680 = bme680_get_endTime() - millis();
            #endif

            bme680_timer = millis();
        }
    }
    #endif
    
    // C4 carve-out: the gateway service block lives in gateway_service_esp32.cpp
    { LOOP_SECTION(LSEC_GATEWAY); gatewayService_esp32(); }

    if(bEXTUDP)
    {
        getExternUDP();
        flushExternQueue();
    }

    // KISS/TCP lifecycle — kept OUT of the "if(bWEBSERVER||bEXTUDP||bGATEWAY||
    // bNETCONSOLE||bKISS)" service block below: on a KISS-only node with all four
    // others off that block is never entered, so kissStop() would be unreachable
    // and "--kiss off" could not close the (unauthenticated) listener.
    #ifndef DISABLE_KISS_TCP
    if(bKISS && (s_auShedMask & FW_SHED_KISS) == 0)   // s_auShedMask: update job shedding (spec 8.1), restored by clearing it
    {
        flushKissQueue();
        kissLoop();   // self-guarding on WiFi.status(); opens the socket once WiFi is up
    }
    else
    {
        kissStop();
    }
    #endif

    // Net console stop, same reason as KISS above: with the console as the only
    // service the block below is skipped after "--netconsole off", and the
    // listener stayed open (P-0169). Stopping needs no network, so no
    // iWlanWait gate either.
    #ifndef DISABLE_NET_CONSOLE
    if(!bNETCONSOLE || (s_auShedMask & FW_SHED_CONSOLE) != 0)
        stopNetConsole();   // no-op unless started
    #endif

    if(bWEBSERVER || bEXTUDP || bGATEWAY || bNETCONSOLE || bKISS)
    {
        if (web_timer == 0 || (iWlanWait > 0 && ((uint32_t)(millis() - web_timer) >= 1000)) || ((uint32_t)(millis() - web_timer) >= (HEARTBEAT_INTERVAL * 1000 * 10)))   // repeat 5 minutes
        {

            web_timer = millis();

            #ifndef BOARD_RAK4630
                if(!meshcom_settings.node_hasIPaddress || iWlanWait > 0)
                {
                    if(iWlanWait == 0)
                    {
                        // 5-min path (F3): cycle the radio only when the driver
                        // is truly idle -- never while it is still reconnecting
                        // on its own (a driver-side join is harvested below).
                        if(wifiTrulyOffline())
                        {
                            // restart WEB-Client
                            if(bWEBSERVER)
                                stopWebserver();

                            startNetwork();
                        }
                    }
                    else
                    {
                        { INSTR_SECTION("wifi_connect"); LOOP_SECTION(LSEC_WIFI_CONNECT); doWiFiConnect(); }

                        if(iWlanWait > 20)
                        {
                            // F3: stop the 1-s polling; the driver keeps
                            // retrying (auto-reconnect) and got_ip is harvested
                            // from the loop. No radio reset at boot (TM-34 §8).
                            iWlanWait = 0;
                            printfdeb("[WIFI]...no join within 20 s, driver keeps retrying\n");
                            bAllStarted=true;
                        }
                    }
                }
                else
                {
                    iWlanWait = 0;
                    bAllStarted=true;
                }
            #endif
        }

        #ifndef BOARD_RAK4630
        // F3: harvest a driver-side (re)connect at any time (TM-17: bAllStarted)
        if(iWlanWait == 0 && wifiHarvestGotIp())
            bAllStarted=true;
        wifiDnsPoll();
        wifiLinkHeartbeat();
        #endif

        if(bWEBSERVER && iWlanWait == 0)
        {
            LOOP_SECTION(LSEC_WEB);   // F3: a blocked WiFiClient::write() lands here

            startWebserver();

            loopWebserver();
        }

        #ifndef DISABLE_NET_CONSOLE
        if(bNETCONSOLE && iWlanWait == 0 && (s_auShedMask & FW_SHED_CONSOLE) == 0)
        {
            startNetConsole();
            loopNetConsole();
        }
        #endif

        // KISS/TCP lifecycle is handled above, outside this service block (F3).

        if(bEXTUDP && iWlanWait == 0)
        {
        #if defined(BOARD_T_ETH_ELITE) || defined(BOARD_T_CONNECT_PRO)
            if(meshcom_settings.node_hasIPaddress == false)
            {
                neth.initethDHCP();
            }
        #endif

            startExternUDP();
        }
    }

    heapMarksPoll();   // [HEAP] boot marks, each once per boot
    #if defined(HAS_TFT)
    tftSpriteReconcile();
    #endif

    if (resendPing == 0)
        resendPing = millis();

    if(meshcom_settings.node_pingtime > 29 && meshcom_settings.node_pingcall[0] != 0x00 && meshcom_settings.node_pingcount > 0)
    {
        if((int32_t)(millis() - (resendPing + meshcom_settings.node_pingtime * 1000)) > 0)
        {
            if(bPingSend)
            {
                printfdeb("[PONG]...fail from %s\n", meshcom_settings.node_pingcall);
                PongFail(meshcom_settings.node_pingcall);
            }

            resendPing = millis();

            PingResult pingResult = sendPing(meshcom_settings.node_pingcall);

            printfdeb("[PING]...%s Ping to %s <%i>\n", (pingResult == PING_QUEUED) ? "send" : "FAILED", meshcom_settings.node_pingcall, meshcom_settings.node_pingcount);

            // TRACK ist ein statischer Zustand: würde man das Budget trotzdem verbrauchen,
            // liefe es in N x pingtime Sekunden auf null, ohne dass je etwas gesendet wurde,
            // und der Knoten verstummt dauerhaft -- wirkt dann wie ein Einstellungsfehler.
            // Ein abgelehnter Ring ist dagegen ein echter Sendeversuch und zählt mit.
            if(pingResult != PING_SUPPRESSED_TRACK)
                meshcom_settings.node_pingcount--;
        }
    }

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

    if ((uint32_t)(millis() - tdeck_tft_timer) >= (TDECK_TFT_TIMEOUT * 1000))
    {
        // printfdeb("Loop: Timeout reached. Timer: %lu, Millis: %lu\n", tdeck_tft_timer, millis());
        tft_off();
    }

    // TD-23: day/night backlight check and the debounced SYM+B save
    brightnessTick(false);

    { INSTR_SECTION("lvgl"); LOOP_SECTION(LSEC_LVGL); lv_task_handler(); }

    #endif

    #if defined(EXTERNAL_RADIO)
    // In external mode bRadio is false, so the local-radio loop section does not
    // run. Re-create here ONLY the RadioLib-free, message-level maintenance the
    // external lifecycle needs, at the same 2s cadence as the native path:
    //
    //  * retransmission maintenance — ages SENT entries (e.g. those entered by a
    //    confirmed bridge RF send awaiting a MeshCom ACK), retries or gives up per
    //    the normal budget, and skips RING_STATUS_EXT_PENDING. It touches only the
    //    ring (no CAD/TX/RX/RadioLib). A requeued retry becomes a READY slot that
    //    externalRadioTxStep() submits with a fresh ownership token.
    if((int32_t)(millis() - (retransmit_timer + (1000 * 2))) > 0)
    {
        updateRetransmissionStatus();
#if defined(ENABLE_MSGSTORE)
        msgstoreLoop();   // S3: all mailbox work runs here, in the loop task
#endif
        retransmit_timer = millis();
    }

    // Non-blocking poll of the optional external-radio TCP transport, AFTER the
    // normal Wi-Fi/network-readiness maintenance above. Self-gates on Wi-Fi state.
    // poll() delivers RX synchronously via glueRxSink -> OnRxDone (main-loop
    // context) and drives one queued external TX. Flush deferred RX display
    // updates here (the local-radio loop section that normally does so is off).
    externalRadioLoop();
    flushDeferredDisplayUpdates();
    #endif

    //
    ////////////////////////////////////////////////

    ////////////////////////////////////////////////
    }   // else from DEEPSLEEP
    ////////////////////////////////////////////////

    // WOR/KBC not necesary     delay(100);

    // yield();
    delay(5); // Save power by allowing IDLE task to run
}


// WEB-RX-01 (#1229): checkRX() is split in a READ half (chip -> RxCtx, RX restart) and a PROCESS
// half (accounting, logs, OnRxDone). The web send buffer's wait path may run the READ half via
// loraRxParkService() and park the packet; the loop drains it through the PROCESS half.
struct RxCtx
{
    uint8_t payload[UDP_TX_BUF_SIZE+10];
    size_t  ibytes;
    int     state;
    int     rssi;       // saved before the RX restart
    int     snr;
    float   ferr;       // not carried through the park (0.0 for parked packets)
};

// READ half. Returns false when a read is already in action (is_receiving). On true the caller owns
// is_receiving and must clear it after the process half / park push.
static bool checkRXRead(RxCtx &c)
{
    if(is_receiving)    // receive in action
        return false;

    is_receiving=true;

    c.ibytes = 0;
    c.rssi = 0;
    c.snr = 0;
    c.ferr = 0.0f;

    // Die echte Paketlaenge muss VOR readData() vom Chip gelesen werden.
    // RadioLib nimmt `len` per WERT und schreibt die tatsaechliche Laenge NICHT
    // zurueck -- es dient nur als obere Schranke (SX126x::readData, dokumentiert
    // in SX126x.h: "getPacketLength method must be called BEFORE calling
    // readData!"). Wer hier UDP_TX_BUF_SIZE uebergibt, bekommt fuer JEDES Paket
    // 255 heraus, egal wie kurz der Frame war: readData fuellt nur die echten
    // Bytes, der Rest von `payload` bleibt uninitialisierter Stackinhalt.
    //
    // Am Mitschnitt eines laufenden Knotens gemessen (DK5EN-98, 25.08.2026):
    // jedes RX_FRAME meldete len=255 bei tatsaechlich 48-133 Byte, gefolgt von
    // immer demselben Stackmuster. Zwei Folgen davon:
    //   - Kanalauslastung: getTimeOnAir(255) buchte fuer JEDEN Empfang 2476 ms
    //     statt der echten 608-1394 ms -- rx war um Faktor 1,8-4,1 zu hoch.
    //   - Korpus/Diagnose: CRC_PAYLOAD- und Mitschnitt-Dumps hingen ~190 Byte
    //     RAM-Inhalt an jeden Frame.
    // Auf nRF52 tritt das nicht auf, dort liefert der Radio-Callback die Laenge.
    size_t ibytes = radio.getPacketLength();

    if(ibytes > UDP_TX_BUF_SIZE)
        ibytes = UDP_TX_BUF_SIZE;

    c.ibytes = ibytes;

    int state = radio.readData(c.payload, ibytes);
    c.state = state;
#if INSTRUMENT_ENABLED
    INSTR_IRQ_READ();        // R1-F: irq_to_read_us, consumed by OnRxDone
#endif

    if (state == RADIOLIB_ERR_LORA_HEADER_DAMAGED || state == RADIOLIB_ERR_NONE)
    {
        // FIX BUG #2: Save RSSI/SNR/FreqError BEFORE restarting RX.
        // These read from SX1262 hardware registers via SPI and would be
        // invalidated once startReceive() transitions the radio to standby+RX.
        c.rssi = (int)radio.getRSSI();
        c.snr  = (int)radio.getSNR();
        c.ferr = radio.getFrequencyError();

        // FIX BUG #2: Restart RX immediately after reading FIFO, BEFORE processing.
        // The packet data is already in our local payload buffer.
        // The radio can now listen for the next packet while we process this one.
        {
            radio.clearPacketReceivedAction();
            radio.clearPacketSentAction();
            #ifdef RADIO_CTRL
                digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
                delay(2);
            #endif
            int rxstate = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);          // Radio in RX zuerst
            radio.setPacketReceivedAction(setFlagReceive);
            bEnableInterruptReceive = true;
            #ifdef LORA_DIO1
            if(digitalRead(LORA_DIO1) == HIGH)
            {
                receiveFlag = true;
                if(bLORADEBUG)
                    printlndeb("[MC-DBG] CHECKRX missed_edge recovery");
            }
            #endif

            // Debug D: RX_RESTARTED after readData
            if(bLORADEBUG)
                printfdeb("[MC-DBG] RX_RESTARTED src=after_readData state=%d\n", rxstate);
        }
    }
    else
    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        // RSSI/SNR/FreqError VOR RX-Restart sichern (Register werden ungueltig)
        c.rssi = (int16_t)radio.getRSSI();
        c.snr  = (int8_t)radio.getSNR();
        c.ferr = radio.getFrequencyError();

        // RX sofort wieder starten
        {
            radio.clearPacketReceivedAction();
            radio.clearPacketSentAction();
            bEnableInterruptReceive = true;
            radio.setPacketReceivedAction(setFlagReceive);
            #ifdef RADIO_CTRL
                digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
                delay(2);
            #endif
            int rxstate = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);

            if(bLORADEBUG)
                printfdeb("[MC-DBG] RX_RESTARTED src=after_crc_error state=%d\n", rxstate);
        }
    }
    else
    {
        // RX-Restart auch bei unbekannten Fehlern -- ohne dies bleibt
        // das Radio im Standby (BLINDSPOT fuer Empfang!)
        c.rssi = (int16_t)radio.getRSSI();
        c.snr  = (int8_t)radio.getSNR();

        {
            radio.clearPacketReceivedAction();
            radio.clearPacketSentAction();
            bEnableInterruptReceive = true;
            radio.setPacketReceivedAction(setFlagReceive);
            #ifdef RADIO_CTRL
                digitalWrite(RADIO_CTRL, HIGH);  // RX Mode [OE1KBC]
                delay(2);
            #endif
            int rxstate = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);

            if(bLORADEBUG)
                printfdeb("[MC-DBG] RX_RESTARTED src=after_other_error state=%d\n", rxstate);
        }
    }

    return true;
}

// PROCESS half: everything after the RX restart. Runs in the loop task only (direct or drained).
static void checkRXProcess(RxCtx &c)
{
    uint8_t *payload = c.payload;
    size_t ibytes = c.ibytes;
    int state = c.state;

    if (state == RADIOLIB_ERR_LORA_HEADER_DAMAGED || state == RADIOLIB_ERR_NONE)
    {
        int saved_rssi = c.rssi;
        int saved_snr  = c.snr;
        float saved_ferr = c.ferr;

        if(bLORADEBUG)
        {
            // packet was successfully received
            printdeb(F("[LoRa]...Received packet: "));

            // print RSSI (Received Signal Strength Indicator)
            printdeb(F("RSSI:\t\t"));
            printdeb(saved_rssi);
            printdeb(F(" dBm / "));

            // print SNR (Signal-to-Noise Ratio)
            printdeb(F("SNR:\t\t"));
            printdeb(saved_snr);
            printdeb(F(" dB / "));

            // print frequency error
            printdeb(F("Frequency error:\t"));
            printdeb(saved_ferr);
            printlndeb(" Hz");
        }

        // RX channel utilization: calculate airtime from packet length
        // (ESP32 has no OnHeaderDetect, so ch_util_rx_start is never set)
        if(ibytes > 0)
        {
            ch_util_rx_accum.fetch_add(radio.getTimeOnAir(ibytes) / 1000);  // us -> ms

            OnRxDone(payload, (uint16_t)ibytes, saved_rssi, saved_snr);
        }
        else
        {
#if INSTRUMENT_ENABLED
            INSTR_IRQ_DROP();   // R1-F: frame read, OnRxDone not reached
#endif
        }
        if(ibytes == 0 && bLORADEBUG)
        {
            // Laenge 0 heisst: der Chip hat nichts im Puffer. Frueher lief das
            // als 255-Byte-Frame aus uninitialisiertem Stack weiter.
            printlndeb("[MC-DBG] CHECKRX zero_length");
        }
    }
    else
    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
#if INSTRUMENT_ENABLED
        INSTR_IRQ_DROP();       // R1-F: frame read, OnRxDone not reached
#endif
        if(bLORADEBUG)
            printfdeb("OnRxError\n");

        int16_t saved_crc_rssi = (int16_t)c.rssi;
        int8_t  saved_crc_snr  = (int8_t)c.snr;
        float   saved_crc_ferr = c.ferr;

        // RX channel utilization: CRC-failed packet still occupied the channel
        ch_util_rx_accum.fetch_add(radio.getTimeOnAir(ibytes) / 1000);  // us -> ms

        // SL-04: CRC/RX error, platform-independent counter for the STAT
        // block (SL-05) and the ERR line under --setlog on. checkRX() runs
        // in loop() context here (called from esp32loop()), so a local
        // stack buffer is unproblematic.
        stat_rx_err.fetch_add(1);
        if(bDisplayLog)
        {
            char buf[300];
            setlogFormatErr(buf, sizeof(buf), saved_crc_rssi, saved_crc_snr,
                            (uint16_t)ibytes, (int32_t)saved_crc_ferr, millis());
            setlogPrint(buf);
        }

        // Diagnose-Output: RSSI/SNR + kompletter Payload-Hex-Dump
        if(bLORADEBUG)
        {
            printfdeb("[MC-DBG] CRC_ERROR rssi=%d snr=%d freq_err=%.1f size=%d ts=%lu\n",
                saved_crc_rssi, saved_crc_snr, saved_crc_ferr, (int)ibytes, millis());

            // Hex-Dump des beschaedigten Payloads (max 255 Bytes)
            int dump_len = (ibytes > 255) ? 255 : (int)ibytes;
            printfdeb("[MC-DBG] CRC_PAYLOAD[%d]: ", dump_len);
            for(int i = 0; i < dump_len; i++)
                printfdeb("%02X ", payload[i]);
            printfdeb("\n");   // F2: printfdeb appends no line end
        }
    }
    else
    {
#if INSTRUMENT_ENABLED
        INSTR_IRQ_DROP();       // R1-F: read error, OnRxDone not reached
#endif
        if(bLORADEBUG)
            printfdeb("OnRxError\n");

        int16_t saved_err_rssi = (int16_t)c.rssi;
        int8_t  saved_err_snr  = (int8_t)c.snr;

        // Immer loggen (nicht nur bLORADEBUG) -- unbekannte Fehler sind kritisch
        printfdeb("[MC-DBG] RX_OTHER_ERROR code=%d rssi=%d snr=%d size=%d ts=%lu\n",
            state, saved_err_rssi, saved_err_snr, (int)ibytes, millis());

        // Payload-Dump bei Debug aktiv
        if(bLORADEBUG && ibytes > 0)
        {
            int dump_len = (ibytes > 255) ? 255 : (int)ibytes;
            printfdeb("[MC-DBG] ERR_PAYLOAD[%d]: ", dump_len);
            for(int i = 0; i < dump_len; i++)
                printfdeb("%02X ", payload[i]);
            printfdeb("\n");   // F2: printfdeb appends no line end
        }
    }
}

int checkRX(bool bRadio)
{
    //#if not defined(BOARD_T_DECK_PRO)
    // extra source
    // you can receive data as an Arduino String
    // NOTE: receive() is a blocking method!
    //       See example ReceiveInterrupt for details
    //       on non-blocking reception method.

    if(!bRadio)
        return -1;

    RxCtx c;
    if(!checkRXRead(c))
        return -1;

    checkRXProcess(c);

    //#endif

    is_receiving=false;

    return c.state;
}

#if !defined(BOARD_T5_EPAPER)
// WEB-RX-01: drain parked packets (oldest first) through the PROCESS half. Loop task only.
// noinline: keeps the RxCtx payload (~265 B) out of the esp32loop() frame, the root of the deepest
// loop-task call chains (advisor note, WEB-RX-01 W2)
__attribute__((noinline)) static void loraRxParkDrain(void)
{
    while(rxParkCount() > 0)
    {
        RxCtx c;
        size_t len = 0;
        int16_t rssi = 0;
        int8_t snr = 0;
        int st = 0;
        if(!rxParkPop(c.payload, &len, &rssi, &snr, &st))
            break;
        c.ibytes = len;
        c.state = st;
        c.rssi = rssi;
        c.snr = snr;
        c.ferr = 0.0f;

        // same exclusion as the flag path: no hook read while a packet is being processed
        is_receiving = true;
        checkRXProcess(c);
        is_receiving = false;

        inoReceiveTimeOutTime = millis();
        iReceiveTimeOutTime = millis();
        csma_timeout = csma_compute_timeout(cad_attempt);
    }
}
#endif

// WEB-RX-01: called from the web send buffer's wait path (txFeed). Reads a pending packet off the
// chip and parks it; never processes (no OnRxDone). CRC/header-error/other branches are counted
// and logged exactly once, in checkRXProcess() when the packet is drained.
void loraRxParkService(void)
{
#if !defined(BOARD_T5_EPAPER)
    if(!bRadio)
        return;

    // TX running: bEnableInterruptTransmit is set before doTX() until TX done (or TX watchdog),
    // tx_is_active covers the lora_functions TX path, transmittedFlag a finished-but-unhandled TX.
    bool txBusy = bEnableInterruptTransmit || tx_is_active || transmittedFlag;

    if(!rxParkShouldService(receiveFlag, is_receiving, txBusy, rxParkCount() >= RX_PARK_SLOTS))
        return;

    // clear flags the same way the loop does before checkRX()
    bEnableInterruptReceive = false;
    receiveFlag = false;
    rx_irq_defer_count = 0;

    RxCtx c;
    if(!checkRXRead(c))     // sets is_receiving
        return;

    rxParkPush(c.payload, c.ibytes, (int16_t)c.rssi, (int8_t)c.snr, c.state);
    is_receiving = false;
#endif
}


