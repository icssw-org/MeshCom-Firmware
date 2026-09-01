// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
//
// 20230326: Version 4.00: START

#include "configuration.h"
#include "capture_functions.h"

#include <Arduino.h>
#include "instrument.h"
#include <maxhop.h>         // CS-01: plausibility of the persisted text hop limit
#include <SPI.h>

#include <debugconf.h>
#include <time.h>
#include <nrf52_functions.h>
#include <nrf52_radio.h>
#include <extudp_functions.h>

#include <TinyGPSPlus.h>

// MeshCom nrf52 functions
#include <RAK13800_W5100S.h>
#include <WisBlock-API.h>

#include <nrf_eth.h>

#include "onebutton_functions.h"

#include <SX126x-RAK4630.h>


#if defined(HAS_TFT) || defined(HAS_TFT_114)
#include "tft_display_functions.h"
#endif

bool bDEEP_SLEEP = false;

#ifdef HAS_TFT_114
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

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

SPIClass SPI_TFT(NRF_SPIM3, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI);
Adafruit_ST7789 tft1 = Adafruit_ST7789(&SPI_TFT, PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);


#endif

#include "Adafruit_LPS2X.h"
#include "Adafruit_SHTC3.h"

#if defined(ENABLE_GPS)
#include "gps_functions.h"
#endif

#include <OneButton.h>
#include <malloc.h>
OneButton btn;

// nRF52 heap monitoring using Adafruit framework APIs (debug.h)
// dbgHeapTotal() = __HeapLimit - __HeapBase (linker symbols)
// dbgHeapUsed()  = mallinfo().uordblks (allocated bytes)
extern int dbgHeapTotal(void);
extern int dbgHeapUsed(void);

static uint32_t nrf52_getFreeHeap(void)
{
    return (uint32_t)(dbgHeapTotal() - dbgHeapUsed());
}

// Largest contiguous free block — binary search probe (fragmentation indicator)
static uint32_t nrf52_getMaxFreeBlock(void)
{
    uint32_t lo = 0, hi = nrf52_getFreeHeap();
    while (lo + 64 < hi) {
        uint32_t mid = (lo + hi) / 2;
        void *p = malloc(mid);
        if (p) { free(p); lo = mid; }
        else   { hi = mid; }
    }
    return lo;
}

// Min-free watermark since boot
static uint32_t nrf52_heapMinFree = UINT32_MAX;
static uint32_t nrf52_heapFree = UINT32_MAX;

// Ethernet Object
NrfETH neth;

// flag to indicate one second 
unsigned long retransmit_timer = 0;

// flag to update NTP Time
unsigned long updateTimeClient = 0;

// resend Ping Timer
unsigned long resendPing = 0;

// timers
uint32_t dhcp_timer = 0;         // dhcp refresh timer

static uint8_t convBuffer[UDP_TX_BUF_SIZE+50]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)

// ETH Prototypes
void sendUDP();                                      // UDP tx routine
void sendHeartbeat();


// MeshCom Common (esp32/nrf52) functions
#include <lora_setchip.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include "dedup_functions.h"
#include <command_functions.h>
#include <aprs_functions.h>
#include <batt_functions.h>
#include <lora_functions.h>
#include "txring_functions.h" // BP-02: txRingDepth() declaration
#include <udp_functions.h>
#include <web_functions/web_functions.h>
#include <phone_commands.h>
#include <mheard_functions.h>
#include <clock.h>

#include <bmx280.h>
#include "bmp390.h"
#include "aht20.h"
#include "bme680.h"
#include "mcu811.h"
#include "io_functions.h"
#include "ina226_functions.h"
#include "rtc_functions.h"
#include "softser_functions.h"

#include <onewire_functions.h>

#include "printfdeb_functions.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGPS;

// UBLOX Modul auf Factory-Set zurück setzen
// derzeit nur im esp32_gps realisiert
// nrf52 fehlt diese Funktion noch
// Diese Variablen werden durch das Command --gps reset gesetzt und solten im GPS Modul abgehandelt werden
int state = 0; // only for gps reset
bool bMitHardReset = false;  // only for gps reset

String strTime;
String strDate;

/*
    RAK4631 PIN DEFINITIONS

    static const uint8_t WB_IO1 = 17;	   // SLOT_A SLOT_B
    static const uint8_t WB_IO2 = 34;	   // SLOT_A SLOT_B
    static const uint8_t WB_IO3 = 21;	   // SLOT_C
    static const uint8_t WB_IO4 = 4;	   // SLOT_C
    static const uint8_t WB_IO5 = 9;	   // SLOT_D
    static const uint8_t WB_IO6 = 10;	   // SLOT_D
    static const uint8_t WB_SW1 = 33;	   // IO_SLOT
    static const uint8_t WB_A0 = 5;		   // IO_SLOT
    static const uint8_t WB_A1 = 31;	   // IO_SLOT
    static const uint8_t WB_I2C1_SDA = 13; // SENSOR_SLOT IO_SLOT
    static const uint8_t WB_I2C1_SCL = 14; // SENSOR_SLOT IO_SLOT
    static const uint8_t WB_I2C2_SDA = 24; // IO_SLOT
    static const uint8_t WB_I2C2_SCL = 25; // IO_SLOT
    static const uint8_t WB_SPI_CS = 26;   // IO_SLOT
    static const uint8_t WB_SPI_CLK = 3;   // IO_SLOT
    static const uint8_t WB_SPI_MISO = 29; // IO_SLOT
    static const uint8_t WB_SPI_MOSI = 30; // IO_SLOT

    SPI
    static const uint8_t SS = 26;
    static const uint8_t MOSI = PIN_SPI_MOSI;
    static const uint8_t MISO = PIN_SPI_MISO;
    static const uint8_t SCK = PIN_SPI_SCK;

    // QSPI Pins
    #define PIN_QSPI_SCK 3	// 19
    #define PIN_QSPI_CS 26	// 17
    #define PIN_QSPI_IO0 30 // 20
    #define PIN_QSPI_IO1 29 // 21
    #define PIN_QSPI_IO2 28 // 22
    #define PIN_QSPI_IO3 2	// 23

    @note RAK5005-O GPIO mapping to RAK4631 GPIO ports
   RAK5005-O <->  nRF52840
   IO1       <->  P0.17 (Arduino GPIO number 17)
   IO2       <->  P1.02 (Arduino GPIO number 34)
   IO3       <->  P0.21 (Arduino GPIO number 21)
   IO4       <->  P0.04 (Arduino GPIO number 4)
   IO5       <->  P0.09 (Arduino GPIO number 9)
   IO6       <->  P0.10 (Arduino GPIO number 10)
   SW1       <->  P0.01 (Arduino GPIO number 1)
   A0        <->  P0.04/AIN2 (Arduino Analog A2
   A1        <->  P0.31/AIN7 (Arduino Analog A7
   SPI_CS    <->  P0.26 (Arduino GPIO number 26)

    */

/*
Sync Word Setting in MeshCom
    MeshCom Syc Word is 0x2b

    Output of the LoRa Sync Word Register 0x0740 in MeshCom:

    Set radio: power level=22
    Sync Word 1st byte = 24
    Sync Word 2nd byte = b4

    In our Library it gets set at sx126x.h / radio.cpp
    Define: sx126x.h line 109:
    #define LORA_MAC_PUBLIC_SYNCWORD 0x242b

    Radio.SetCustomSyncWord(0x242b); needs to be called, so syncword gets new set in radio.cpp
    Method
*/

//NVIC_SystemReset(); resets the device

asm(".global _scanf_float");
asm(".global _printf_float");

// LoRa Events and Buffers
static RadioEvents_t RadioEvents;

// flag to indicate if we are after receiving
unsigned long iReceiveTimeOutTime = 0;

// CSMA/CA async CAD state
std::atomic<bool> cad_done_flag{false};
std::atomic<bool> cad_channel_busy{false};
std::atomic<bool> cad_in_progress{false};
std::atomic<bool> cad_double_check{false};
unsigned long cad_start_time = 0;

bool g_meshcom_initialized;
bool init_flash_done=false;

// FreeRTOS queue of raw BLE payloads, filled by bleuart_rx_callback() in
// nrf52_ble.cpp (BLE task context) and drained below in nrf52loop() so
// readPhoneCommand()/commandAction()/save_settings() run in the Main Loop
// task instead of inline in the BLE callback (CONC-14). Struct layout must
// stay in sync with the definition in nrf52_ble.cpp.
struct BleQueueItem {
    uint8_t data[MAX_MSG_LEN_PHONE];
    size_t length;
};
extern QueueHandle_t bleQueue;

bool bPosFirst = true;
bool bHeyFirst = true;

// Queue for sending config jsons to phone
uint8_t iPhoneState = 0;
bool config_to_phone_prepare = false;
bool conffin_sent = false;
unsigned long config_to_phone_prepare_timer = 0;
unsigned long config_to_phone_datetime_timer = 0;
const uint8_t json_configs_cnt = 9;
const char config_cmds[json_configs_cnt][20] = {"--info", "--seset", "--wifiset", "--nodeset", "--wx", "--pos", "--aprsset", "--io", "--tel"};
uint8_t config_cmds_index = 0;

/**
 * BLE Spec
 * Messages to and from the phone need to have flag if it is a Text, Pos Msg or it is a configuration
 * Text/Pos flag: 0x40
 * Config Flag: 0x80
 * 
 * Config Message Parameters: Callsign, Lat, Lon, altitude
 * Text/Pos Msg Format:
 * LENGTH 2B - FLAG 1B - MSG xB
 * Config Format:
 * LENGTH 2B - FLAG 1B - LENCALL 1B - Callsign - LAT 4B(Float) - LON 4B(Float) - ALT 4B(INT)
 * 
 * Hello Message from Phone is: "XXaaYYzz"
 * Config of this device is sent to phone after hello msg from phone
*/

// Textmessage buffer from phone, hasMsgFromPhone flag indicates new message
extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;

extern bool ble_busy_flag;    // flag to signal bluetooth uart is active

//variables and helper functions
uint8_t err_cnt_udp_tx = 0;    // counter on errors sending message via UDP

// CheckSerialConsole
String strTextWork;
char strText[600] = {0};
int iTxtPos = 0;
int iTxtLen = 0;

// TinyGPS
TinyGPSPlus tinyGPSPlus;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

#if defined(ENABLE_RAK_GPS)
unsigned int getGPS(void);
void SetupUBLOX();
#endif

// TEMP/HUM
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

void getTEMP(void);

Adafruit_LPS22 g_lps22hb;

void getPRESSURE(void);

// BME680
#if defined(ENABLE_BMX680)
unsigned long bme680_timer = millis();
int delay_bme680 = 0;
#endif


#define POWER_ENABLE   WB_IO2

#define LEFT_BUTTON    WB_IO3
#define MIDDLE_BUTTON  WB_IO5
#define RIGHT_BUTTON   WB_IO6

uint8_t gKeyNum = 0; // which button is pressed
int iGPSCount=0;

// left button interrupt handle function
void interruptHandle1()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 1;
  }
}

// middle button interrupt handle function
void interruptHandle2()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 2;
  }
}

// right button interrupt handle function
void interruptHandle3()
{
  if(gKeyNum == 0)
  {
    gKeyNum = 3;
  }
}

#if not defined(BOARD_HELTEC_T114) and not defined(BOARD_T_ECHO)
#include <U8g2lib.h>

//extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
extern U8G2 *u8g2;
extern U8G2 u8g2_1;
extern U8G2 u8g2_2;
#endif

// Prototypes
void blinkLED();                                     // blink GREEN
void blinkLED2();                                    // blink BLUE
void blinkLED2();                                    // blink RED

void checkSerialCommand(void);


unsigned long gps_refresh_timer = 0;
unsigned long mcp_refresh_timer = 0;
unsigned long softser_refresh_timer = 0;

// Client basic variables
uint8_t dmac[6];

/** Set the device name, max length is 10 characters */
    char g_ble_dev_name[10] = "MC";

void getMacAddr(uint8_t *dmac)
{
    const uint8_t *src = (const uint8_t *)NRF_FICR->DEVICEADDR;
    dmac[5] = src[0];
    dmac[4] = src[1];
    dmac[3] = src[2];
    dmac[2] = src[3];
    dmac[1] = src[4];
    dmac[0] = src[5]; // | 0xc0; // MSB high two bits get set elsewhere in the bluetooth stack
}

void OnCadDone(bool channelActivityDetected)
{
    // RACE-05 fix: atomic flag update under critical section
    taskENTER_CRITICAL();
    cad_channel_busy = channelActivityDetected;
    cad_done_flag.store(true, std::memory_order_release);
    taskEXIT_CRITICAL();
}

void RadioInit()
{
    Radio.Init(&RadioEvents);
}

///////////////////////////////////////////
void nrf52setup()
{
#if defined NRF52_SERIES || defined ESP32
	// Create the task event semaphore
	g_task_sem = xSemaphoreCreateBinary();
	// Initialize semaphore
	xSemaphoreGive(g_task_sem);
#endif

#if defined(HAS_TFT_114)
    pinMode(PIN_TFT_VDD_CTL, OUTPUT);
    pinMode(PIN_TFT_LEDA_CTL, OUTPUT);
    digitalWrite(PIN_TFT_VDD_CTL, TFT_LEDA_ENABLE);  // TFT VDD
    digitalWrite(PIN_TFT_LEDA_CTL, TFT_VDD_ENABLE); // TFT LEDA

    digitalWrite(LORA_NRSET, HIGH);   // LORA

    SPI_TFT.begin();

    tft1.init(TFT_WIDTH, TFT_HEIGHT);                  // ST7789 240x135
    tft1.setRotation(TFT_ROTATION);
    tft1.setSPISpeed(SPI_FREQUENCY);

    char cvers1[22];
    snprintf(cvers1, sizeof(cvers1), "  FW %s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
    String  version1 = cvers1;
    displayTFT(" MeshCom 4.0 ", version1, "@BY ICSSW.ORG", "  OE1KBC, OE1KFR",  "  ...starting now", 5000);

#endif

     // LEDs
    #if not defined(BOARD_HELTEC_T114)
        pinMode(LED_GREEN, OUTPUT);
        #ifndef BOARD_RAK4630
        pinMode(LED_RED, OUTPUT);
        #endif
        pinMode(LED_BLUE, OUTPUT);
    #endif

    //  Initialize the Serial Port for debug output
    time_t timeout = millis();
    Serial.begin(MONITOR_SPEED);
    while (!Serial)
    {
        if ((millis() - timeout) < 2000)
        {
            delay(100);
        }
        else
        {
            break;
        }
    }

    Serial.println("=====================================");
    Serial.println("[INIT] START CLIENT");
    nrf52_heapMinFree = nrf52_getFreeHeap();
    nrf52_heapFree = nrf52_heapMinFree;
    Serial.printf("%s;[HEAP];%lu;%lu;%lu;(init)\n", getTimeString().c_str(),
        (unsigned long)nrf52_getFreeHeap(),
        (unsigned long)nrf52_heapMinFree,
        (unsigned long)nrf52_getMaxFreeBlock());

    // init nach Reboot
    init_loop_function();

    // clear the buffers
    for (int i = 0; i < uint8_t(sizeof(RcvBuffer)); i++)
    {
        RcvBuffer[i] = 0x00;
    }

    for(int ib=0; ib<MAX_RING; ib++)
    {
        memset(own_msg_id[ib], 0x00, 5);
    }

    //clear ringbuffer
    for(int i=0; i<MAX_RING; i++)
    {
        memset(ringBufferLoraRX[i], 0, 4);
    }

    // Initialize mheard list
    initMheard();

	// Initialize battery reading
	init_batt();

	// Get LoRa parameter
	init_flash();

    bool bClear = false;
    if(meshcom_settings.node_cleanflash == 1)
        bClear = true;

    // Geloescht wird nur bei echter Layout-Aenderung, nicht bei jedem neuen
    // Build-Datum. Siehe configuration_global.h.
    if(!flashLayoutCompatible(meshcom_settings.node_fversion) || bClear)
    {
        Serial.printf("[INIT]...FLASH cleared, Settings-Layout %i -> %i\n",
                      meshcom_settings.node_fversion, FLASH_STRUCT_VERSION);

        flash_reset();
    }
    else
    {
        Serial.printf("[INIT]...FLASH layout %i ok, build %i\n",
                      meshcom_settings.node_fversion, FLASH_VERSION);
    }

    if(bClear)
        init_flash();

    meshcom_settings.node_fversion = FLASH_STRUCT_VERSION;
    meshcom_settings.node_mversion = MODUL_HARDWARE;
    meshcom_settings.node_cleanflash = 0;
    snprintf(meshcom_settings.node_fwversion, sizeof(meshcom_settings.node_fwversion), "%-4.4s%-1.1s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    
    save_settings();

    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    meshcom_settings.node_date_year = 0;
    meshcom_settings.node_date_month = 0;
    meshcom_settings.node_date_day = 0;

    bDisplayVolt = meshcom_settings.node_sset & 0x0001;
    bDisplayOff = meshcom_settings.node_sset & 0x0002;
    bPosDisplay = meshcom_settings.node_sset & 0x0004;
    bDEBUG = meshcom_settings.node_sset & 0x0008;
    bButtonCheck = meshcom_settings.node_sset & 0x0010;
    bDisplayTrack = meshcom_settings.node_sset & 0x0020;
    bGPSON =  meshcom_settings.node_sset & 0x0040;
    bBMPON =  meshcom_settings.node_sset & 0x0080;
    bBMEON =  meshcom_settings.node_sset & 0x0100;
    bLORADEBUG = meshcom_settings.node_sset & 0x0200;
    bSHORTPATH = meshcom_settings.node_sset & 0x0400;
    bGATEWAY =  meshcom_settings.node_sset & 0x1000;
    bEXTUDP =  meshcom_settings.node_sset & 0x2000;
    bDisplayCont =  meshcom_settings.node_sset & 0x4000;

    bONEWIRE =  meshcom_settings.node_sset2 & 0x0001;
    bLPS33 =  meshcom_settings.node_sset2 & 0x0002;
    bBME680ON = meshcom_settings.node_sset2 & 0x0004;
    bMCU811ON =  meshcom_settings.node_sset2 & 0x0008;
    //bGPSDEBUG = meshcom_settings.node_sset2 & 0x0010;
    iGPSDEBUG = meshcom_settings.node_gpsdebug;
    bMESH = !(meshcom_settings.node_sset2 & 0x0020);
    bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
    bWIFIAP = meshcom_settings.node_sset2 & 0x0080;
    bGATEWAY_NOPOS =  meshcom_settings.node_sset2 & 0x0100;
    bSMALLDISPLAY =  false;
    //free !! meshcom_settings.node_sset2 & 0x0200;
    bSOFTSERON =  meshcom_settings.node_sset2 & 0x0400;
    bBOOSTEDGAIN =  meshcom_settings.node_sset2 & 0x0800;
    //xxxxx = meshcom_settings.node_sset2 & 0x2000;
    bVIA = meshcom_settings.node_sset2 & 0x4000;

    // nicht mehr notwendig bMHONLY =  meshcom_settings.node_sset3 & 0x0001;
    bNoMSGtoALL =  meshcom_settings.node_sset3 & 0x0002;
    bBLEDEBUG = meshcom_settings.node_sset3 & 0x0004;
    bAnalogCheck = meshcom_settings.node_sset3 & 0x0008;
    bBMP3ON = meshcom_settings.node_sset3 & 0x0010;
    bAHT20ON = meshcom_settings.node_sset3 & 0x0020;

    bDEBUGCSV = meshcom_settings.node_sset4 & 0x0001;
    bDEBUGEN = meshcom_settings.node_sset4 & 0x0002;
    bDisplayLog = meshcom_settings.node_sset4 & 0x0004;
    bTXCAPTURE = meshcom_settings.node_sset4 & 0x0008;

    bDisplayInfo = bLORADEBUG;

    if(strlen(meshcom_settings.node_aprsmc) < 4)
    {
        strcpy(meshcom_settings.node_aprsmc, (char*)"APRSMC");  // default
    }

    // CS-01: max_hop_text is user-settable (--maxhop) and survives in the settings
    // file now; only a missing or out-of-range value falls back to the compile
    // default. max_hop_pos stays a compile-time constant (operator, 2026-08-30).
    meshcom_settings.max_hop_text = maxHopTextSanitize(meshcom_settings.max_hop_text);
    meshcom_settings.max_hop_pos = MAX_HOP_POS_DEFAULT;

    iButtonPin = BUTTON_PIN;
    if(meshcom_settings.node_button_pin > 0)
        iButtonPin = meshcom_settings.node_button_pin;

    // user button init
    init_onebutton();

    // if Node not set --> WifiAP Mode on
    /* NRF52 no WiFi
    if(isNodeUnconfigured(meshcom_settings.node_call))
    {
        bWIFIAP = true;
        bWEBSERVER = true;

        Serial.println("WIFIAP starting...");
    }
    */

    if(meshcom_settings.node_gwsrv[0] == 0x00)
        snprintf(meshcom_settings.node_gwsrv, sizeof(meshcom_settings.node_gwsrv), "%s", "OE");

    // if Node is in WifiAP Mode -> no Gateway posible
    if(bWIFIAP && bGATEWAY)
    {
        bGATEWAY=false;
        bEXTUDP=false;
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

    memset(meshcom_settings.node_update, 0x00, sizeof(meshcom_settings.node_update));
    
    #ifndef ENABLE_SOFTSER
        bSOFTSERON=false;
    #endif

    // Umstekllung auf langes WIFI Passwort
    if(strlen(meshcom_settings.node_ossid) > 4 && strlen(meshcom_settings.node_ssid) < 5)
    {
        strncpy(meshcom_settings.node_ssid, meshcom_settings.node_ossid, sizeof(meshcom_settings.node_ssid) - 1);
        meshcom_settings.node_ssid[sizeof(meshcom_settings.node_ssid) - 1] = '\0';
        strncpy(meshcom_settings.node_pwd, meshcom_settings.node_opwd, sizeof(meshcom_settings.node_pwd) - 1);
        meshcom_settings.node_pwd[sizeof(meshcom_settings.node_pwd) - 1] = '\0';

        memset(meshcom_settings.node_ossid, 0x00, sizeof(meshcom_settings.node_ossid));
        memset(meshcom_settings.node_opwd, 0x00, sizeof(meshcom_settings.node_opwd));

        save_settings();
    }

    meshcom_settings.node_press = 0.0;
    meshcom_settings.node_hum = 0.0;
    meshcom_settings.node_temp = 0.0;
    meshcom_settings.node_temp2 = 0.0;
    meshcom_settings.node_press_alt = 0;
    meshcom_settings.node_press_asl = 0.0;

    global_batt = 4125.0;

    posinfo_interval = POSINFO_INTERVAL;

    if(meshcom_settings.node_postime > 0)
        posinfo_interval = meshcom_settings.node_postime;

    if(meshcom_settings.node_maxv > 0)
    {
        setMaxBatt(meshcom_settings.node_maxv * 1000.0F);
    
        global_batt = meshcom_settings.node_maxv * 1000.0F;
    }

	// Initialize onewire sensor
    // check pin number in flash
    if(meshcom_settings.node_owgpio > 7 || meshcom_settings.node_owgpio < 0){
        meshcom_settings.node_owgpio = 0;
        save_settings();
    }

#ifdef OneWire_GPIO
    if(bONEWIRE)
    {
        init_onewire_ds18();
        init_onewire_dht();
    }
#endif

//  Initialize the LoRa Module
    api_init_lora();

    getMacAddr(dmac);

    _GW_ID = dmac[0] | (dmac[1] << 8) | (dmac[2] << 16) | (dmac[3] << 24);

    // BLE ID
    meshcom_settings.node_device_eui[0]=0x4D;   // M
    meshcom_settings.node_device_eui[1]=0x43;   // C
    for(int ieui=0; ieui<6; ieui++)
    {
        meshcom_settings.node_device_eui[ieui+2] = dmac[ieui];
    }


    ////////////////////////////////////////////////////////////////////
    // Initialize time
	bool boResult;
	
	// initialize clock
	boResult = MyClock.Init();
	Serial.printf("[INIT]...Initialize clock: %s\n", (boResult) ? "ok" : "FAILED");
    snprintf(cTimeSource, sizeof(cTimeSource), (char*)"INIT");

    DisplayTimeWait=0;
    //
    ////////////////////////////////////////////////////////////////////

    // Hardware immer aktivieren
    //if(bGPSON)
    {
        //gps init
        #if defined(ENABLE_RAK_GPS)
        pinMode(WB_IO2, OUTPUT);
        digitalWrite(WB_IO2, 0);
        delay(1000);
        digitalWrite(WB_IO2, 1);
        delay(1000);
        
        Serial.println("=====================================");

        Serial.println("GPS: trying 9600 baud");
        
        Serial1.begin(9600);
        Serial1.setTimeout(500);
        while (!Serial1);

        if(Serial1)
        {
            if (myGPS.begin(Serial1))
            {
                Serial.println("GPS: connected at 9600 baud");
            }
            else
            {
                Serial1.end();

                delay(100);
                Serial.println("GPS: trying 38400 baud");

                Serial1.begin(38400);
                Serial1.setTimeout(500);
                while (!Serial1);

                if(Serial1)
                {
                    if (myGPS.begin(Serial1))
                    {
                        Serial.println("GPS: connected at 38400 baud");
                    }
                    else
                    {
                        Serial.println("GPS: speed not found");
                    }
                }
                else
                    Serial.println("GPS: not connected");
            }
        }
        else
            Serial.println("GPS: not connected");

        delay(100);

        SetupUBLOX();

        #endif
    }

    posinfo_fix = false;
    posinfo_satcount = 0;
    fposinfo_hdop = 0.0;

    // Try to initialize!
    #if defined(LPS33)

    if(bLPS33)
    {
        Serial.println("Adafruit LPS33 check");

        if (!g_lps22hb.begin_I2C(0x5d)) 
        {
            Serial.println("Failed to find LPS33 chip");
            //while (1) 
            { 
            //  delay(10); 
            }
        }
        else
        {
            Serial.println("LPS33 sensor found");

            g_lps22hb.setDataRate(LPS22_RATE_10_HZ);

            /*
            Serial.print("Data rate set to: ");

            switch (g_lps22hb.getDataRate()) 
            {
                case LPS22_RATE_ONE_SHOT: Serial.println("One Shot / Power Down"); 
                    break;
                case LPS22_RATE_1_HZ: Serial.println("1 Hz"); 
                    break;
                case LPS22_RATE_10_HZ: Serial.println("10 Hz"); 
                    break;
                case LPS22_RATE_25_HZ: Serial.println("25 Hz"); 
                    break;
                case LPS22_RATE_50_HZ: Serial.println("50 Hz"); 
                    break;
            }
            */
        }
    }
    #endif // LPS33

    shtc3_found = false;

    #if defined(SHTC3)

    Serial.println("[INIT]...Adafruit SHTC3 check");
    if (!shtc3.begin())
    {
        Serial.println("[INIT]...Couldn't find SHTC3");
    }
    else
    {
        Serial.println("[INIT]...SHTC3 sensor found");
        shtc3_found = true;
    }

    #endif // SHTC3

    //////////////////////////////////////////////////////
    // BLE INIT

    g_enable_ble=true;

#if defined NRF52_SERIES || defined ESP32
	if (g_enable_ble)
	{
		// Init BLE
		init_ble();

        Serial.println("[INIT]...BLE init");
	}
	else
	{
		// BLE is not activated, switch off blue LED
        #if not defined(BOARD_HELTEC_T114)
		digitalWrite(LED_BLUE, LOW);
        #endif
	}

	// Take the semaphore so the loop will go to sleep until an event happens
	xSemaphoreTake(g_task_sem, 10);
#endif

#ifdef ARDUINO_ARCH_RP2040
	// RAK11310 does not have BLE, switch off blue LED
	digitalWrite(LED_BLUE, LOW);
#endif

    // I2C init
    #if not defined(USE_HELTEC_T114)
        Wire.begin();
    #endif

    #if defined(ENABLE_BMX280)
        setupBMX280(true);
    #endif

    #if defined(ENABLE_AHT20)
        setupAHT20(true);
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

    #if defined(BOARD_T_ECHO)
        initDisplay();
        startDisplay((char*)"..starting now..", (char*)"@by icssw.org", (char*)"OE1KBC, OE1KFR");
    
    #elif defined(HAS_EPAPER)

    #elif defined(HAS_TFT_114)
        char cvers[22];
        snprintf(cvers, sizeof(cvers), "  FW %s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
        String  version = cvers;
        displayTFT(" MeshCom 4.0", version, "@BY ICSSW.ORG", "   OE1KBC, OE1KFR",  "   ...starting now", 5000);
    #elif defined(HAS_TFT)
        initTFT();
        char cvers[22];
        snprintf(cvers, sizeof(cvers), "  FW %s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
        String  version = cvers;
        displayTFT(" MeshCom 4.0 ", version, "  @BY ICSSW.ORG", "  OE1KBC, OE1KFR",  "  ...starting now", 5000);
    #else
        initDisplay();
        startDisplay((char*)"..starting now..", (char*)"@by icssw.org", (char*)"OE1KBC, OE1KFR");
    #endif

    // reset GPS-Time parameter
    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    Serial.println("[INIT]...CLIENT STARTED");

    // Reset-Ursache loggen (POWER->RESETREAS, vom Core beim Start gesichert):
    // 0x1 Reset-Pin, 0x2 Watchdog, 0x4 Soft-Reset (SREQ — auch der Weg, den
    // der SoftDevice-Fault-Handler nach einem App-Absturz nimmt), 0x8 Lockup.
    // Unterscheidet nach einem unerwarteten Reboot sofort Absturz von
    // Spannungsproblem — haette die N-22-Diagnose erheblich verkuerzt.
    {
        extern uint32_t readResetReason(void);
        Serial.printf("[BOOT] RESETREAS=0x%08lX\n", (unsigned long)readResetReason());
    }


    Radio.SetModem(MODEM_LORA);

    Serial.printf("[INIT]...Radio-Sync: %04X\n", Radio.GetSyncWord());

    //  Set the LoRa Callback Functions
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;
    //RadioEvents.PreAmpDetect = OnPreambleDetect;
    RadioEvents.PreAmpDetect = OnHeaderDetect;

    // 4.34w we use EU8 instead of EU
    if(meshcom_settings.node_country == 0)
        meshcom_settings.node_country = 8;

    lora_setcountry(meshcom_settings.node_country);

    //  Initialize the LoRa Transceiver
    RadioInit();

    // Sets the Syncword new that we can set the MESHCOM SWORD
    DEBUG_MSG("RADIO", "Setting new LoRa Sync Word");
    Radio.SetCustomSyncWord(0x242b);

    // set bandwidth 
    Serial.printf("[LoRa]...RF_BANDWIDTH: %.0f kHz\n", getBW());

    // set spreading factor 
    Serial.printf("[LoRa]...RF_SF: %i\n",  getSF());

    // coding rate
    Serial.printf("[LoRa]...RF_CR: 4/%i\n", getCR());

    // set carrier frequency
    uint32_t ifreq=(getFreq()*1000.)+0.5;
    ifreq = ifreq * 1000;

    Serial.printf("[LoRa]...RF_FREQUENCY: %.4f %ld MHz\n", getFreq(), ifreq);

    //  Set the LoRa Frequency
    Radio.SetChannel(ifreq);

    Serial.printf("[LoRa]...PREAMBLE: %i symbols\n", meshcom_settings.node_preamplebits);

    uint16_t preamble_length = meshcom_settings.node_preamplebits;

    //  Configure the LoRa Transceiver for receiving messages
    Radio.SetRxConfig(
        MODEM_LORA,
        (uint32_t)meshcom_settings.node_bw,
        (uint32_t)meshcom_settings.node_sf,
        (uint8_t)meshcom_settings.node_cr,
        0, //  AFC bandwidth: Unused with LoRa
        preamble_length,
        LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0,    //  Fixed payload length: N/A
        true, //  CRC enabled
        0,    //  Frequency hopping disabled
        0,    //  Hop period: N/A
        LORA_IQ_INVERSION_ON,
        true //  Continuous receive mode
    );

    // Set Radio TX configuration
    Serial.printf("[LoRa]...RF_POWER: %i dBm\n", getPower());

    Radio.SetTxConfig(
        MODEM_LORA,
        getPower(),
        0, // fsk only
        (uint32_t)meshcom_settings.node_bw,
        (uint32_t)meshcom_settings.node_sf,
        (uint8_t)meshcom_settings.node_cr,
        preamble_length,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true, // CRC ON
        0,    // fsk only frequ hop
        0,    // fsk only frequ hop period
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE);

    Serial.printf("[INIT]...Radio-Sync: %04X\n", Radio.GetSyncWord());

    //  Start receiving LoRa packets
    startRadioReceive();

    // Configure CAD parameters: 4 symbols, DETPEAK/DETMIN from config, CAD_ONLY mode
    Radio.SetCadParams(LORA_CAD_04_SYMBOL, RADIOLIB_SX126X_DETPEAK, RADIOLIB_SX126X_DETMIN, LORA_CAD_ONLY, 0);

    // set left button interrupt
    //pinMode(LEFT_BUTTON, INPUT);
    //attachInterrupt(LEFT_BUTTON, interruptHandle1, FALLING);

    // set middle button interrupt
    pinMode(MIDDLE_BUTTON, INPUT);
    attachInterrupt(MIDDLE_BUTTON, interruptHandle2, FALLING);

    // set right button interrupt
    //pinMode(RIGHT_BUTTON, INPUT);
    //attachInterrupt(RIGHT_BUTTON, interruptHandle3, FALLING);

    sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);

    Serial.println("=====================================");

    delay(100);

    if (bGATEWAY || bWEBSERVER)
    {
        //////////////////////////////////////////////////////
        // ETHERNET INIT
   
        if(strlen(meshcom_settings.node_ownip) > 6 && strlen(meshcom_settings.node_ownms) > 6 && strlen(meshcom_settings.node_owngw) > 6)
        {
            if(bDEBUG)
                Serial.println("[init] ETH FIX-IP init");

            neth.initethfixIP();
        }
        else
        {
            if(bDEBUG)
                Serial.println("[init] ETH DHCP init");

            neth.initethDHCP();
        }

        if(neth.hasETHHardware)
        {
            if(bGATEWAY)
            {
                sendHeartbeat();

                Serial.println("=====================================");
                Serial.printf("GATEWAY 4.0 RUNNING %s\n", neth.hasIPaddress?"ETH connect":"ETH no connect");
                Serial.println("=====================================");
            }

            if(bWEBSERVER)
            {
                startWebserver();
            }

            if(bEXTUDP)
            {
                startExternUDP();
            }
        }
        else
        {
            meshcom_settings.node_hasIPaddress = false;

            Serial.println("==================================");
            Serial.println("CLIENT 4.0 RUNNING no ETH-Hardware");
            Serial.println("==================================");
        }
    }
    else
    {
        meshcom_settings.node_hasIPaddress = false;
        neth.hasETHHardware = false;
    }
}   

void nrf52loop()
{
    INSTR_LOOPTICK();

    // BP-01/BP-04: close a back-pressure episode with QRV once the TX ring
    // has drained, even when no further user message arrives to observe it.
    // Since BP-02 an occupied-slot scan (O(MAX_RING)), since BP-04 with the
    // water-band hold; it emits at most once per episode.
    bpPollDrain();

    // TM-25: Boot-Marke fuer den Bench-Harness, wie [BOOT];ready auf dem ESP32.
    // Auf dem nRF52 ist die Netzwerkphase (W5100S/DHCP) am Ende von setup()
    // bereits synchron abgeschlossen -- die Marke faellt beim ersten Loop-Durchlauf.
    static bool s_bootReadyLogged = false;
    if (!s_bootReadyLogged)
    {
        s_bootReadyLogged = true;
        Serial.printf("[BOOT];ready;ms;%lu;ip;%d;eth;%d\n", (unsigned long)millis(),
                      neth.hasIPaddress ? 1 : 0, neth.hasETHHardware ? 1 : 0);
    }
    #if defined(HAS_TFT_114) or defined(BOARD_T_ECHO)
    if(bDEEP_SLEEP)
    {
        delay(60000);
        // only loop
    }
    else
    {
    #endif

    // get RTC Now
    // RTC hat Vorrang zu Zeit via MeshCom-Server
    bool bMyClock = true;

    #if defined(ENABLE_RTC)
    if(bRTCON)
    {
        bMyClock = false;

        { INSTR_SECTION("rtc"); loopRTC(); }

        if(!posinfo_fix) // GPS hat Vorang zur RTC
        {
            DateTime utc = getRTCNow();

            DateTime now (utc + TimeSpan(meshcom_settings.node_utcoff * 60 * 60));

            uint16_t Year = now.year();
            uint16_t Month = now.month();
            uint16_t Day = now.day();

            uint16_t Hour = now.hour();
            uint16_t Minute = now.minute();
            uint16_t Second = now.second();


            // check valid Date & Time
            if(Year > 2023)
            {
                MyClock.setCurrentTime(meshcom_settings.node_utcoff, Year, Month, Day, Hour, Minute, Second);
                snprintf(cTimeSource, sizeof(cTimeSource), (char*)"RTC");
                bMyClock = true;
            }
        }
    }
    #endif

    if(meshcom_settings.node_hasIPaddress)
    {
        INSTR_SECTION("eth_state");
        strTime = "none";

        extern bool btimeClient;

        // every 15 minutes
        if(btimeClient)
        {
            if((uint32_t)(millis() - updateTimeClient) >= (uint32_t)(1000 * 60 * 15) || updateTimeClient == 0)
            {
                strTime = neth.udpUpdateTimeClient();

                updateTimeClient = millis();
            }
            else
                strTime = neth.udpGetTimeClient();

            strDate = neth.udpGetDateClient();

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
    }
    else
    {
        bNTPDateTimeValid = false;
    }


    if(bMyClock)
    {
        INSTR_SECTION("clock");
        MyClock.CheckEvent();
        
        if(MyClock.Year() > 2023)
            meshcom_settings.node_date_year = MyClock.Year();
        else
            meshcom_settings.node_date_year = 0;

        meshcom_settings.node_date_month = MyClock.Month();
        meshcom_settings.node_date_day = MyClock.Day();

        meshcom_settings.node_date_hour = MyClock.Hour();
        meshcom_settings.node_date_minute = MyClock.Minute();
        meshcom_settings.node_date_second = MyClock.Second();

        // Starttime setzen
        if(meshcom_settings.node_date_year > 2023 && meshcom_settings.node_update[0] == 0x00)
        {
            snprintf(meshcom_settings.node_update, sizeof(meshcom_settings.node_update), "%04i-%02i-%02i %02i:%02i:%02i",
             meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);
        }
    }

    // Retransmission status must tick on ALL nodes (including gateways).
    // Without this, gateway text messages stay stuck at RING_STATUS_SENT
    // forever if no echo is received via LoRa (RING_ZOMBIE).
    if ((uint32_t)(millis() - retransmit_timer) >= (1000 * 2))
    {
        updateRetransmissionStatus();
        // BP-03 (DJ8MEH-RCA): age out stale BACKGROUND (HEY) ring entries
        // here, in the main-loop tick -- NOT in getNextTxSlot(), which also
        // runs on the nRF52 timer task itself (Advisor F1, the critical
        // finding this fix is named after).
        txRingAgeBackground(millis());

        retransmit_timer = millis();
    }

    // Periodischer Ringpuffer-Auslastungsbericht (alle 30s)
    {
        static unsigned long ring_status_timer = 0;
        if(bLORADEBUG && (millis() - ring_status_timer) > 30000)
        {
            ring_status_timer = millis();
            int pending = 0, retrying = 0, done = 0;
            for(int i = 0; i < MAX_RING; i++)
            {
                if(ringBuffer[i][0] == 0) continue;
                if(ringBuffer[i][1] == 0xFF) done++;
                else if(ringBuffer[i][1] == 0x00) pending++;
                else retrying++;
            }
            int w = iWrite;
            int r = iRead;
            // BP-02: queued is now the honest occupied-slot count (the
            // pending/retrying/done buckets above already scan every slot
            // and partition it exactly) instead of the raw index distance --
            // that distance counted freed holes left behind by a
            // priority-starved entry parked at iRead as still queued
            // (DJ8MEH-RCA: queued=19 vs. 3-4 real). The old arithmetic
            // survives as `dist` below for anyone comparing against
            // pre-BP-02 logs.
            int queued = pending + retrying + done;
            int dist = (w >= r) ? (w - r) : (MAX_RING - r + w);
            Serial.printf("[MC-DBG] RING_STATUS queued=%d pending=%d retrying=%d done=%d iW=%d iR=%d dist=%d\n",
                queued, pending, retrying, done, w, r, dist);
        }
    }

    // Deferred display update from OnRxDone (avoid I2C inside radio callback)
    // RACE-01 fix: snapshot under critical section, display call outside
    {
        taskENTER_CRITICAL();
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
        taskEXIT_CRITICAL();
        INSTR_SECTION("display_rx");
        if(_pendText) sendDisplayText(_msg, _rssi, _snr);
        if(_pendPos)  sendDisplayPosition(_msg, _rssi, _snr);
    }

    // Channel utilization report (every 10s)
    {
        static unsigned long ch_util_timer = 0;
        if(bLORADEBUG && (millis() - ch_util_timer) > 10000)
        {
            unsigned long window = millis() - ch_util_timer;
            ch_util_timer = millis();
            unsigned long rx_ms = ch_util_rx_accum.exchange(0);
            unsigned long tx_ms = ch_util_tx_accum.exchange(0);
            unsigned int util = (unsigned int)((rx_ms + tx_ms) * 100 / window);
            if(util > 100) util = 100;
            Serial.printf("[MC-DBG] CHANNEL_UTIL rx=%lums tx=%lums util=%u%%\n",
                rx_ms, tx_ms, util);
            // ONRXDONE stats: report max and warn count, then reset
            Serial.printf("[MC-DBG] ONRXDONE_STATS max=%lums warn=%u (>%dms)\n",
                onrxdone_max_ms, onrxdone_warn_count, ONRXDONE_WARN_MS);
            onrxdone_max_ms = 0;
            onrxdone_warn_count = 0;
        }
    }

    if(iReceiveTimeOutTime > 0)
    {
        INSTR_SECTION("lora_rx_timeout");
        if((uint32_t)(millis() - iReceiveTimeOutTime) >= (uint32_t)csma_timeout)
        {
            if(bLORADEBUG)
                Serial.printf("[MC-DBG] RX_TIMEOUT_FIRE ts=%lu wait=%lu delta=%lu\n",
                    millis(), csma_timeout, millis() - iReceiveTimeOutTime);

            // BUG #1 Aequivalent: Wenn Header erkannt, ist Empfang moeglicherweise
            // noch aktiv. Timer verlaengern statt zuruecksetzen.
            if(is_receiving && rx_irq_defer_count < 3)
            {
                rx_irq_defer_count++;
                iReceiveTimeOutTime = millis();
                if(bLORADEBUG)
                    Serial.printf("[MC-DBG] RX_TIMEOUT_DEFERRED src=is_receiving cnt=%d\n", rx_irq_defer_count);
            }
            else
            {
                if(rx_irq_defer_count >= 3 && bLORADEBUG)
                    Serial.printf("[MC-DBG] RX_IRQ_STALE forced restart after %d deferrals\n", rx_irq_defer_count);
                rx_irq_defer_count = 0;
                iReceiveTimeOutTime = 0;
                taskENTER_CRITICAL();
                startRadioReceive();
                taskEXIT_CRITICAL();
                if(bLORADEBUG)
                {
                    Serial.printf("[MC-SM] IDLE -> RX_LISTEN rc=0\n");
                    Serial.printf("[MC-DBG] RX_RESTART src=timeout\n");
                }
            }
        }
    }

    if(iReceiveTimeOutTime == 0 && is_receiving == false && tx_is_active == false)
    {
        INSTR_SECTION("lora_sm");
        int _w = iWrite;
        int _r = iRead;
        if (_w != _r)
        {
            // RACE-05 fix: snapshot CAD flags under critical section,
            // then act on snapshot values outside the lock
            taskENTER_CRITICAL();
            bool _cad_ip = cad_in_progress;
            bool _cad_df = cad_done_flag;
            bool _cad_cb = cad_channel_busy;
            bool _cad_dc = cad_double_check;
            taskEXIT_CRITICAL();

            if(!_cad_ip && !_cad_df)
            {
                // Start CAD scan
                // BP-02: qlen is txRingDepth() (occupied slots), not the raw
                // index distance -- see txRingDepth() doc comment.
                if(bLORADEBUG)
                {
                    Serial.printf("[MC-SM] IDLE -> TX_PREPARE rc=0\n");
                    Serial.printf("[MC-DBG] TX_GATE_ENTER qlen=%d cad_attempt=%d\n",
                        txRingDepth(),
                        cad_attempt);
                }

                taskENTER_CRITICAL();
                cad_in_progress = true;
                cad_done_flag = false;
                cad_double_check = false;
                cad_start_time = millis();
                taskEXIT_CRITICAL();
                taskENTER_CRITICAL();
                Radio.Standby();
                Radio.StartCad();
                taskEXIT_CRITICAL();
            }
            else if(_cad_df)
            {
                taskENTER_CRITICAL();
                cad_in_progress = false;
                cad_done_flag = false;
                taskEXIT_CRITICAL();
                if(bLORADEBUG)
                    Serial.printf("[MC-DBG] CAD_SCAN result=%d\n", _cad_cb ? -702 : 0);

                if(!_cad_cb)
                {
                    if(_cad_dc && bLORADEBUG)
                        Serial.printf("[MC-DBG] CAD_FALSE_POSITIVE\n");
                    // Channel free — transmit
                    csma_reset();
                    bool _tx_ok;
                    { INSTR_SECTION("lora_tx"); _tx_ok = doTX(); }
                    if(_tx_ok)
                    {
                        ch_util_tx_start = millis();
                        if(bLORADEBUG)
                        {
                            Serial.printf("[MC-SM] TX_PREPARE -> TX_ACTIVE rc=0\n");
                            Serial.printf("[MC-DBG] CAD_FREE attempt=%d\n", cad_attempt);
                        }
                    }
                    else
                    {
                        // doTX() returned false (no ready slot) — restore RX
                        // to prevent CAD spin loop where radio stays in standby
                        if(bLORADEBUG)
                            Serial.printf("[MC-DBG] CAD_FREE_NO_TX restoring RX\n");
                        taskENTER_CRITICAL();
                        startRadioReceive();
                        taskEXIT_CRITICAL();
                        iReceiveTimeOutTime = millis();
                    }
                }
                else if(!_cad_dc)
                {
                    // First scan busy — double-check
                    if(bLORADEBUG)
                        Serial.printf("[MC-DBG] CAD_BUSY_1 attempt=%d, double-check...\n", cad_attempt);

                    taskENTER_CRITICAL();
                    cad_double_check = true;
                    cad_in_progress = true;
                    cad_done_flag = false;
                    cad_start_time = millis();
                    taskEXIT_CRITICAL();
                    taskENTER_CRITICAL();
                    Radio.Standby();
                    Radio.StartCad();
                    taskEXIT_CRITICAL();
                }
                else
                {
                    // Double-check confirmed busy — backoff
                    cad_attempt++;
                    csma_timeout = csma_compute_timeout(cad_attempt);

                    if(bLORADEBUG)
                    {
                        Serial.printf("[MC-SM] TX_PREPARE -> IDLE rc=-1\n");
                        Serial.printf("[MC-DBG] CAD_BUSY attempt=%d next_timeout=%lu\n",
                            cad_attempt, csma_timeout);
                    }

                    taskENTER_CRITICAL();
                    startRadioReceive();
                    taskEXIT_CRITICAL();
                    iReceiveTimeOutTime = millis();
                }
            }
            else if(_cad_ip && (millis() - cad_start_time > 100))
            {
                // Safety timeout: CadDone never fired
                if(bLORADEBUG)
                    Serial.printf("[MC-DBG] CAD_SAFETY_TIMEOUT\n");

                taskENTER_CRITICAL();
                cad_in_progress = false;
                cad_done_flag = false;
                taskEXIT_CRITICAL();
                taskENTER_CRITICAL();
                startRadioReceive();
                taskEXIT_CRITICAL();
                iReceiveTimeOutTime = millis();
            }
        }
        else
        {
            // Nothing to send — restart timeout cycle (radio watchdog).
            // Matches ESP32 esp32_main.cpp behaviour: periodic Radio.Rx()
            // prevents silent SX1262 stall when queue is empty.
            iReceiveTimeOutTime = millis();
        }
    }

    #if defined(ENABLE_GPS)
    if(bGPSON)
    {
        INSTR_SECTION("gps_init");
        WZ_GPS_Init();
    }
    #endif

    // SOFTSER
    #if defined(ENABLE_SOFTSER)
    if(bSOFTSERON)
    {
        INSTR_SECTION("softser");
        // check every 5 seconds to ready next telemetry via serial interface
        if ((uint32_t)(millis() - softser_refresh_timer) >= 5000 && softserFunktion == 0)
        {
            if(lastSOFTSER_MINUTE != meshcom_settings.node_date_minute)
            {
                if(meshcom_settings.node_date_minute % SOFTSER_REFRESH_INTERVAL  == 0)
                {
                    lastSOFTSER_MINUTE = meshcom_settings.node_date_minute;

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

    btn.tick();

    // BLE Queue: process data from the BLE task in Main Loop context (CONC-14)
    {
        BleQueueItem bleItem;
        while (xQueueReceive(bleQueue, &bleItem, 0) == pdTRUE) {
            { INSTR_SECTION("ble_cmd"); readPhoneCommand(bleItem.data); }
        }
    }

    // Apply a settings write staged by settings_rx_callback(), if any (CONC-17)
    { INSTR_SECTION("ble_settings"); applyPendingBleSettings(); }

    // check if message from phone to send
    if(hasMsgFromPhone)
    {
        INSTR_SECTION("phone_msg");
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

    #if defined(ENABLE_MCP23017)
    // 5 sec
    if ((uint32_t)(millis() - mcp_refresh_timer) >= 5000)
    {
        // get i/o state
        if(loopMCP23017())
        {
        }

        mcp_refresh_timer = millis();
    }
    #endif

    #if defined(ENABLE_GPS)
        gKeyNum = 2;
    #endif


    if(gKeyNum == 1)
    {
        INSTR_SECTION("key1");
        Serial.println("gKeyNum == 1");

        //getTEMP();

        //getPRESSURE();

        gKeyNum = 0;
    }

    if(gKeyNum == 2)
    {
        INSTR_SECTION("key2");
        //TEST Serial.println("gKeyNum == 2");

        #if defined(ENABLE_RAK_GPS)
        if(bGPSON)
        {
            // gps refresh every 10 sec
            if ((uint32_t)(millis() - gps_refresh_timer) >= (uint32_t)(GPS_REFRESH_INTERVAL * 1000))
            {
                unsigned int igps = getGPS();

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

                gps_refresh_timer = millis();
            }
        }
        #endif

        #if defined(ENABLE_GPS)
        if(bGPSON)
        {
            // gps refresh every sec
            if ((uint32_t)(millis() - gps_refresh_timer) >= 1000)
            {
                unsigned int igps = POSINFO_INTERVAL;

                if(gpsDetected)
                {
                    { INSTR_SECTION("gps"); igps = WZ_GPS_Loop(); }

                    if(iGPSDEBUG > 0)
                    {
                        Serial.printf("[GPS ]...fix:%s sat:%i hdop:%.1lf\n", (posinfo_fix?"yes":"no"), gpsData.satellites, gpsData.hdop);

                        Serial.print("[GPS ]...Time <UTC>: ");
                        if (gpsData.hour < 10) Serial.print(F("0"));
                        Serial.print(gpsData.hour);
                        Serial.print(F(":"));
                        if (gpsData.minute < 10) Serial.print(F("0"));
                        Serial.print(gpsData.minute);
                        Serial.print(F(":"));
                        if (gpsData.second < 10) Serial.print(F("0"));
                        Serial.print(gpsData.second);

                        Serial.print(F(" / Date: "));
                        Serial.print(gpsData.year);
                        Serial.print(F("."));
                        if (gpsData.month < 10) Serial.print(F("0"));
                        Serial.print(gpsData.month);
                        Serial.print(F("."));
                        if (gpsData.day < 10) Serial.print(F("0"));
                        Serial.println(gpsData.day);

                        if(posinfo_fix)
                        {
                            Serial.printf("[GPS ]...position  : lat:%.6lf lon:%.6lf alt:%.1lf\n", gpsData.latitude, gpsData.longitude, gpsData.altitude);
                        }

                    }
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

                gps_refresh_timer = millis();
            }
        }
        #endif
        
        #if defined(ENABLE_RAK_GPS) || defined(ENABLE_GPS)
        if(bGPSON)
        {
            // check GPS ON and activ --> <gKeyNum == 2> the signal must be active
            if ((uint32_t)(millis() - gps_refresh_timer) >= (uint32_t)(5 * (GPS_REFRESH_INTERVAL * 1000)))
            {
                posinfo_fix = false;
                posinfo_satcount = 0;
                fposinfo_hdop = 0.0;
                posinfo_interval = POSINFO_INTERVAL;
            }
        }
        #endif
        
        gKeyNum = 0;
    }

    if(gKeyNum == 3)
    {
        INSTR_SECTION("key3");
        Serial.println("Right button pressed");

        gKeyNum = 0;
    }


    if (isPhoneReady == 1)
    {
        if (config_to_phone_prepare)
        {
            for(int config_cmds_index=0; config_cmds_index < json_configs_cnt; config_cmds_index++)
            {
                commandAction((char*)config_cmds[config_cmds_index], isPhoneReady, true);
            }

            sendMheard();

            config_to_phone_prepare_timer=millis();

            config_to_phone_prepare = false;
        }
        else
        {
            // wait after BLE Connect 3 sec.
            if((uint32_t)(millis() - config_to_phone_prepare_timer) < 3000)
                iPhoneState = 0;

            if (iPhoneState > 3)   // only every 6 times of mainloop send to phone  RAK 2 x ESP
            {
                // prepare JSON config to phone after BLE connection
                // send JSON config to phone after BLE connection
                if (ComToPhoneWrite != ComToPhoneRead)
                {
                    sendComToPhone();
                }
                else if (toPhoneWrite != toPhoneRead)
                {
                    sendToPhone();
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
            {
                iPhoneState++;
            }
        }

        // 5 minuten
        if((uint32_t)(millis() - config_to_phone_datetime_timer) >= (5 * 60 * 1000))
        {
            bNTPDateTimeValid=false;

            config_to_phone_datetime_timer = millis();
        }
    }

    // check NCNT modified
    int incnt = getMheardCount();
    if(ncnt_hold != incnt)
    {
        INSTR_SECTION("pos_timer");
        // minimal alle 60 sec
        if((uint32_t)(millis() - posinfo_timer_min) >= 60000)
        {
            posinfo_shot = true;
            ncnt_hold = incnt;
        }
    }

    // posinfo
    //Serial.print(getTimeString());
    //Serial.printf(" posinfo_timer:%ld posinfo_interval:%ld timer:%ld millis:%ld\n", posinfo_timer, posinfo_interval, (posinfo_timer + (posinfo_interval * 1000)), millis());

    // posinfo_interval in Seconds
    if (((uint32_t)(millis() - posinfo_timer) >= (uint32_t)(posinfo_interval * 1000)) || (millis() > 100000 && millis() < 130000 && bPosFirst) || posinfo_shot)
    {
        // minimal transmit time only max 30 sec
        if((uint32_t)(millis() - posinfo_timer_min) >= 30000)
        {
            if(bDisplayInfo)
            {
                Serial.print(getTimeString());
                Serial.printf(" [POS]...sendPostion initialized F:%i S:%i\n", bPosFirst, posinfo_shot);
            }

            bPosFirst = false;

            if(posinfo_shot)
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
                else
                    node_lat_c='N';

                if(posinfo_prev_lon < 0.0)
                    node_lon_c='W';
                else
                    node_lon_c='E';

                { INSTR_SECTION("pos_tx"); sendPosition(posinfo_interval, node_lat, node_lat_c, node_lon, node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl); }
            }
            else
            {
                { INSTR_SECTION("pos_tx"); sendPosition(posinfo_interval, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl); }
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
    if (((uint32_t)(millis() - heyinfo_timer) >= trickle_interval_ms) || bHeyFirst)
    {
        bHeyFirst = false;

        // Check for topology change
        int current_neighbors = getMheardCount();
        if(trickle_last_neighbor_count >= 0 && current_neighbors != trickle_last_neighbor_count)
        {
            trickle_interval_ms = TRICKLE_IMIN_S * 1000UL;
            trickle_consistent_count = 0;
        }
        trickle_last_neighbor_count = current_neighbors;

        // Trickle suppression
        if(trickle_consistent_count >= TRICKLE_K)
        {
            if(bDisplayInfo)
                Serial.printf("[MC-TRICKLE] SUPPRESS consistent=%d interval=%lums\n",
                    trickle_consistent_count, trickle_interval_ms);
        }
        else
        {
            { INSTR_SECTION("hey_tx"); sendHey(); }
        }

        trickle_interval_ms = min(trickle_interval_ms * 2, TRICKLE_IMAX_S * 1000UL);
        trickle_consistent_count = 0;

        heyinfo_timer = millis();
    }

    // TELEMETRY_INTERVAL in Minutes == 15 minutes default
    unsigned long akt_timer = meshcom_settings.node_parm_time;
    if(akt_timer < 5 || akt_timer > 120)
    {
        INSTR_SECTION("akt_timer");
        akt_timer = TELEMETRY_INTERVAL;
    }
    
    akt_timer = akt_timer * 1000 * 60; // in minutes

    if(iNextTelemetry < 5)
        akt_timer= 15 * 1000; // 15 Seconds PARM, UNIT, EQNS and 1st T-Message

    if (((uint32_t)(millis() - telemetry_timer) >= (uint32_t)akt_timer) || bHeyFirst)
    {
        bHeyFirst = false;
        
        { INSTR_SECTION("telemetry"); sendTelemetry(SOFTSER_APP_ID); }

        telemetry_timer = millis();
    }
    
    // ETH-01: link poll/heartbeat and the DHCP lease refresh must run
    // whenever Ethernet is up, not only for bGATEWAY -- a gateway-off /
    // webserver-on node (bWEBSERVER, see setup ~1086) has ETH hardware too
    // and was never renewing its lease nor logging link state. Both
    // functions already self-guard on neth.hasETHHardware (false when
    // neither bGATEWAY nor bWEBSERVER is set), so hoisting them here is a
    // no-op for nodes without ETH. Order: link edges/heartbeat first, then
    // the DHCP renew check, then (below) either the gateway UDP get/send or
    // the TM-45 NTP-only harvest -- exactly one of the latter two per pass.

    // TM-35 / N-20 instrumentation: link edges and the 60-s heartbeat
    ethLinkPoll();
    ethLinkHeartbeat();

    // DHCP refresh
    if(neth.hasETHHardware && (uint32_t)(millis() - dhcp_timer) >= (uint32_t)(DHCP_REFRESH * 60000))
    {
        // no need on static IPs
        if(!(strlen(meshcom_settings.node_ownip) > 6 && strlen(meshcom_settings.node_ownms) > 6 && strlen(meshcom_settings.node_owngw) > 6))
        {
            if(bDEBUG)
            {
                Serial.print(getTimeString());
                Serial.println(" [MAIN] checkDHCP");
            }

            { INSTR_SECTION("eth_dhcp"); neth.checkDHCP(); }
        }

        dhcp_timer = millis();
    }

    // get UDP & send UDP message from ringBufferOut if there is one to tx
    if(bGATEWAY)
    {
        INSTR_SECTION("gateway");
        int bUDPReceived = false;

        // check if we received a UDP packet
        if (neth.hasIPaddress)
        {
            bSPI_ETH_Active = true;   // SPI guard: Ethernet owns bus
            INSTR_SECTION("eth_udp");
            if(neth.getUDP() == 1)  // 1...no udp-paket received
            {
                { INSTR_SECTION("eth_udp_tx"); sendUDP(); }
            }
            else
            {
                bUDPReceived=true;

                if(bDEBUG)
                    Serial.println("LOOP GATEWAY actions UDP received");
            }
            bSPI_ETH_Active = false;  // SPI guard: release bus
            if(bPendingRadioRx) { bPendingRadioRx = false; startRadioReceive(); }
        }
        else
        {
            //neth.last_upd_timer = 0; // ETH new
        }

        // UDP Action for next loop
        if(!bUDPReceived)
        {
            meshcom_settings.node_hasIPaddress = neth.hasIPaddress;
            meshcom_settings.node_last_upd_timer = neth.last_upd_timer;
            
            // check HB response (we also check successful sending KEEP. check if they work together!)
            if((uint32_t)(millis() - neth.last_upd_timer) >= (uint32_t)(MAX_HB_RX_TIME * 1000))
            {
                if(bDEBUG)
                    Serial.println("LOOP GATEWAY last_upd_timer actions");

                neth.last_upd_timer = millis();

                // avoid TX and UDP
                if(!neth.hasIPaddress)
                {
                    neth.hasIPaddress = false;
                    iReceiveTimeOutTime = millis();

                    if(strlen(meshcom_settings.node_ownip) > 6 && strlen(meshcom_settings.node_ownms) > 6 && strlen(meshcom_settings.node_owngw) > 6)
                    {
                        if(bDEBUG)
                        {
                            Serial.print(getTimeString());
                            Serial.println(" [MAIN] initethETH fix-IP");
                        }

                        neth.initethfixIP();
                    }
                    else
                    {
                        Serial.print(getTimeString());
                        Serial.println(" [MAIN] resetDHCP (retry)");

                        // N-20: initethDHCP() wuerde den W5100S bei jedem
                        // Retry per initETH_HW() hardware-resetten — danach
                        // braucht die PHY-Aushandlung mehrere Sekunden und der
                        // Link-Check in startETH() sieht dauerhaft LinkOFF:
                        // ein einmal gezogenes Kabel verbindet nie wieder (auf
                        // Hardware beobachtet). Das volle HW-Init ist nur beim
                        // Boot noetig (Setup); hier reicht resetDHCP() ohne
                        // PHY-Reset — der Link-Zustand ist dann echt, und bei
                        // LinkOFF bricht startETH() sofort ab statt 10 s zu
                        // blocken.
                        neth.resetDHCP();
                    }
                }
            }
            // ETH-01: DHCP refresh moved above, ahead of this if(bGATEWAY)
            // block, so it also runs when bGATEWAY is off.
        }
    }
    else if(neth.hasIPaddress)
    {
        // TM-45: the block above never runs while bGATEWAY is off, so it
        // never reads the socket -- do only the NTP-reply harvest instead
        // of the full gateway receive path (no double read: exactly one of
        // the two branches runs per loop pass).
        INSTR_SECTION("udp"); neth.harvestNTP();
    }

    #if defined(SHTC3)

    // TEMP/HUM
    if (((uint32_t)(millis() - temphum_timer) >= TEMPHUM_INTERVAL))
    {
        if(shtc3_found)
        {
            INSTR_SECTION("shtc3");
            getTEMP();
        }

        temphum_timer = millis();
    }

    #endif

    #if defined(LPS33)

    if(bLPS33)
    {
        INSTR_SECTION("lps33");
        // DRUCK
        if (((uint32_t)(millis() - druck_timer) >= DRUCK_INTERVAL))
        {
            getPRESSURE();

            druck_timer = millis();

            if(wx_shot)
            {
                commandAction((char*)"--wx", isPhoneReady, true);
                wx_shot = false;
            }
        }
    }

    #endif

    { INSTR_SECTION("display_tick"); mainStartTimeLoop(); }

    if(DisplayOffWait > 0)
    {
        INSTR_SECTION("display_off");
        if ((int32_t)(millis() - DisplayOffWait) > 0)
        {
            DisplayOffWait = 0;
            if(bDisplayOff)
            {
                bDisplayIsOff=true;

                #if defined(BOARD_HELTEC_T114)
                digitalWrite(PIN_TFT_LEDA_CTL, HIGH);   // TFT OFF
                #endif

                sendDisplay1306(true, true, 0, 0, (char*)"#C");
            }
        }
    }
    
    // rebootAuto
    if(rebootAuto > 0)
    {
        INSTR_SECTION("reboot_auto");
        if ((int32_t)(millis() - rebootAuto) > 0)
        {
            rebootAuto = 0;

            #ifdef ESP32
                ESP.restart();
            #endif

            #if defined NRF52_SERIES
                // dfuAuto: --dfu will den UF2-Bootloader statt eines normalen
                // Neustarts. Beides laeuft ueber denselben verzoegerten Pfad, damit
                // die Quittung noch ueber BLE bzw. Seriell rausgeht, bevor der Reset
                // kommt -- ein Reset direkt in commandAction() verschluckt sie.
                //
                // N-19: enterUf2Dfu() (reset_mcu() in cores/nRF5/wiring.c) haengt
                // aus diesem Loop-Task -- auf Hardware reproduziert: --dfu liess das
                // Board mit stehender CPU zurueck (USB-Deskriptor blieb App-PID, kein
                // Reset). Derselbe reset_mcu()-Pfad funktioniert aus dem TinyUSB-Task
                // (1200-Baud-Touch), und NVIC_SystemReset() funktioniert aus genau
                // diesem Loop-Pfad (--reboot) -- verdaechtig ist sd_softdevice_disable()
                // im Loop-Kontext. Deshalb hier ohne SD-Disable: GPREGRET per
                // SoftDevice-SVC setzen (bei aktivem SoftDevice erlaubt, das BLE-Init
                // laeuft auf nRF52 immer) und den bewaehrten NVIC_SystemReset()
                // nehmen; der Bootloader liest GPREGRET beim Start und bleibt im
                // UF2-Modus (USB-Laufwerk).
                if(bEnterDfu)
                {
                    bEnterDfu = false;
                    sd_power_gpregret_clr(0, 0xFF);
                    sd_power_gpregret_set(0, 0x57);   // DFU_MAGIC_UF2_RESET (cores/nRF5/wiring.c)
                    uint32_t gpr = 0;
                    sd_power_gpregret_get(0, &gpr);
                    printfdeb("...GPREGRET=0x%02lX (0x57 -> UF2-Bootloader)\n", (unsigned long)gpr);
                    // Ack/Log noch ueber die CDC rausschreiben, bevor der Reset die
                    // USB-Verbindung kappt. Ohne flush+Wartezeit kam das Board in
                    // einem Hardware-Test trotz korrekt gesetztem GPREGRET als App
                    // statt als Bootloader zurueck.
                    Serial.flush();
                    delay(300);
                }

                NVIC_SystemReset();     // resets the device
            #endif
        }
    }

    { INSTR_SECTION("serial_cmd"); checkSerialCommand(); }

    if(BattTimeWait == 0)
        BattTimeWait = millis() - 31000;

    if ((uint32_t)(millis() - BattTimeWait) >= 30000)
    {
        if (tx_is_active == false && is_receiving == false)
        {
            global_batt = read_batt();
            global_proz = mv_to_percent(global_batt);

            BattTimeWait = millis();
        }
    }

    // Heap Monitor — always active, 60s interval
    if(!bDisplayLog)
    {
        static unsigned long heapMonTimer = 0;
        if (heapMonTimer == 0)
            heapMonTimer = millis();

        if ((uint32_t)(millis() - heapMonTimer) >= 60000)
        {
            uint32_t freeHeap = nrf52_getFreeHeap();

            if(nrf52_heapFree != freeHeap)
            {
                nrf52_heapFree = freeHeap;
                
                if (freeHeap < nrf52_heapMinFree) nrf52_heapMinFree = freeHeap;

                Serial.printf("%s;[HEAP];%lu;%lu;%lu;(mon)\n",
                    getTimeString().c_str(),
                    (unsigned long)freeHeap,
                    (unsigned long)nrf52_heapMinFree,
                    (unsigned long)nrf52_getMaxFreeBlock());
            }

            heapMonTimer = millis();
        }
    }

    #ifdef OneWire_GPIO
    if(bONEWIRE)
    {
        if(onewireTimeWait == 0)
            onewireTimeWait = millis() - 10000;


        if ((uint32_t)(millis() - onewireTimeWait) >= 30000)  // 30 sec
        {
            //if (tx_is_active == false && is_receiving == false)
            {
                if(one_found)
                    loop_onewire_ds18();

                if(dht_found)
                    loop_onewire_dht();

                onewireTimeWait = millis();

                if(wx_shot)
                {
                    commandAction((char*)"--wx", isPhoneReady, true);
                    wx_shot = false;
                }
            }
        }
    }
    #endif

    // read BMP Sensor
    #if defined(ENABLE_BMX280) || defined(ENABLE_AHT20)
    if(((bBMPON || bBMEON) && bmx_found) || (bAHT20ON && aht20_found))
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

            if(wx_shot)
            {
                commandAction((char*)"--wx", isPhoneReady, false);
                wx_shot = false;
            }

            BMXTimeWait = millis() - lreduction; // wait for next messurement
        }
    }
    #endif

    // read BMP390 Sensor
    #if defined(ENABLE_BMP390)
    if((bBMP3ON && bmp3_found))
    {
        if ((uint32_t)(millis() - BMP3TimeWait) >= 60000)   // 60 sec
        {
            if(loopBMP390())
            {
                meshcom_settings.node_press = getPress3();
                if(!aht20_found)
                {
                    meshcom_settings.node_temp = getTemp3();
                }
                meshcom_settings.node_press_asl = getPressASL3();
                meshcom_settings.node_press_alt = getAltitude3();
            }

            BMP3TimeWait = millis(); // wait for next messurement
        }
    }
    #endif

    #if defined(ENABLE_MC811)
    if(bMCU811ON && mcu811_found)
    {
        if(MCU811TimeWait == 0)
            MCU811TimeWait = millis() - 10000;

        if ((uint32_t)(millis() - MCU811TimeWait) >= 60000)   // 60 sec
        {
            // read MCU-811 Sensor
            if(loopMCU811())
            {
                meshcom_settings.node_co2 = geteCO2();
                
                if(wx_shot)
                {
                    commandAction((char*)"--wx", isPhoneReady, true);
                    wx_shot = false;
                }
            }

            MCU811TimeWait = millis(); // wait for next messurement
        }
    }
    #endif

    #if defined(ENABLE_INA226)
    if(bINA226ON)
    {
        if(INA226TimeWait == 0)
            INA226TimeWait = millis() - 10000;

        if ((uint32_t)(millis() - INA226TimeWait) >= 60000)   // 60 sec
        {
            // read INA Sensor
            if(loopINA226())
            {
                meshcom_settings.node_vbus = getvBUS();
                meshcom_settings.node_vshunt = getvSHUNT();
                meshcom_settings.node_vcurrent = getvCURRENT();
                meshcom_settings.node_vpower = getvPOWER();
            }

            INA226TimeWait = millis(); // wait for next messurement
        }
    }
    #endif

    // read every n seconds the bme680 sensor calculated from millis()
    #if defined(ENABLE_BMX680)
    if(bBME680ON && bme680_found)
    {
        if ((uint32_t)(millis() - bme680_timer) >= 60000 || delay_bme680 <= 0)
        {
            if (delay_bme680 <= 0)
            {
                getBME680();

            }

            if(wx_shot)
            {
                commandAction((char*)"--wx", isPhoneReady, true);
                wx_shot = false;
            }

            // calculate delay
            delay_bme680 = bme680_get_endTime() - millis();

            bme680_timer = millis();
        }
    }
    #endif

    // heartbeat
    if (bGATEWAY)
    {
        if ((uint32_t)(millis() - hb_timer) >= (uint32_t)(HEARTBEAT_INTERVAL * 1000))
        {
            if(bDisplayCont)
            {
                Serial.print(getTimeString());
                Serial.printf(" [UDP] sending Heartbeat\n");
            }

            { INSTR_SECTION("eth_heartbeat"); sendHeartbeat(); }

            hb_timer = millis();
        }
    }

    if(bEXTUDP)
    {
        bSPI_ETH_Active = true;   // SPI guard: Ethernet owns bus
        getExternUDP();
        flushExternQueue();
        bSPI_ETH_Active = false;  // SPI guard: release bus
        if(bPendingRadioRx) { bPendingRadioRx = false; startRadioReceive(); }
    }

    if(bWEBSERVER || bEXTUDP)
    {
        if (web_timer == 0 || ((uint32_t)(millis() - web_timer) >= (uint32_t)(HEARTBEAT_INTERVAL * 1000 * 30)))   // repeat 15 minutes
        {
            meshcom_settings.node_hasIPaddress = neth.hasIPaddress;

            web_timer = millis();

            #ifndef BOARD_RAK4630
                // restart WEB-Client
                stopWebserver();

                if(!meshcom_settings.node_hasIPaddress)
                    startWIFI();
            #endif

            // N-20-Falle: ohne initialisiertes Ethernet (Setup ueberspringt die
            // HW-Initialisierung, wenn weder Gateway noch Webserver aktiv sind,
            // oder es ist keine RAK13800 gesteckt) blockieren W5100S-Socket-Ops
            // (UdpExtern.begin()/sendExternHeartbeat()) den Loop-Task unbegrenzt
            // -- Node wirkt tot, Konsole ohne Echo, nur 1200-Baud-Touch hilft.
            // Reproduziert 2026-08-22: --extudp on bei Gateway/Webserver off
            // friert den Loop ab dem naechsten Durchlauf dauerhaft ein (auch
            // nach jedem Reboot, da gespeichert). Deshalb: Start nur mit IP.
            if(bWEBSERVER && neth.hasIPaddress)
            {
                bSPI_ETH_Active = true;
                startWebserver();
                bSPI_ETH_Active = false;
                if(bPendingRadioRx) { bPendingRadioRx = false; startRadioReceive(); }
            }

            if(bEXTUDP && neth.hasIPaddress)
            {
                bSPI_ETH_Active = true;
                startExternUDP();
                bSPI_ETH_Active = false;
                if(bPendingRadioRx) { bPendingRadioRx = false; startRadioReceive(); }
            }
        }

        if(bWEBSERVER)
        {
            bSPI_ETH_Active = true;   // SPI guard: Ethernet owns bus (web page delivery)
            loopWebserver();
            bSPI_ETH_Active = false;  // SPI guard: release bus
            if(bPendingRadioRx) { bPendingRadioRx = false; startRadioReceive(); }
        }

    }

    if (resendPing == 0)
        resendPing = millis();

    if(meshcom_settings.node_pingtime > 29)
    {
        if((int32_t)(millis() - (resendPing + meshcom_settings.node_pingtime * 1000)) > 0)
        {
            resendPing = millis();

            if(bDisplayInfo)
                printfdeb("[PING]...send Ping to %s\n", meshcom_settings.node_pingcall);

            if(meshcom_settings.node_pingcall[0] != 0x00)
                sendPing(meshcom_settings.node_pingcall);
        }
    }


    #if defined(HAS_TFT_114) or defined(BOARD_T_ECHO)
    }   // else from 
    #endif

    //  We are on FreeRTOS, give other tasks a chance to run
    delay(100);

    yield();
}

/** @brief Function to check if the modem detected a preamble
 */
void blinkLED()
{
    #if not defined(BOARD_HELTEC_T114)
    digitalWrite(LED_GREEN, HIGH);
    delay(20);
    digitalWrite(LED_GREEN, LOW);
    #endif
}

void blinkLED2()
{
    #if not defined(BOARD_HELTEC_T114)
    digitalWrite(LED_BLUE, HIGH);
    delay(20);
    digitalWrite(LED_BLUE, LOW);
    #endif
}


/**@brief Function for handling a LoRa tx timer timeout event.
 */
void getTEMP(void)
{
    sensors_event_t humidity, temp;

    shtc3.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data

    if(bWXDEBUG)
    {
        Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
        Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    }

    meshcom_settings.node_temp2 = temp.temperature;
    meshcom_settings.node_hum = humidity.relative_humidity;
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
void getPRESSURE(void)
{
    sensors_event_t temp;
    sensors_event_t pressure;
    
    g_lps22hb.getEvent(&pressure, &temp);

    if(bWXDEBUG)
    {
        Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");
        Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
        Serial.println("");
    }
    
    //double home_alt=meshcom_settings.node_alt;    // Höhe des Standorts
    //double temperature_gradient = 0.0065;           // Standard-Temperaturgradient
    //double temperatureK = temp.temperature + 273.15;     // Temperatur in Kelvin
    
    // barometrische Höhenformel
    //meshcom_settings.node_press = pressure.pressure * pow((temperatureK / (temperatureK + home_alt * temperature_gradient)) , -5.255);
    if(bLPS33)
    {
        meshcom_settings.node_press = pressure.pressure;
        meshcom_settings.node_temp = temp.temperature;
    }
    else
    {
        meshcom_settings.node_press = 0;
        meshcom_settings.node_temp = 0;
    }
}

/**@brief Function for analytical direction.
 */
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") < 0 && tmp.indexOf(",W,") > 0)
    {
        direction_E_W = 0;  // WEST
    }
    else
    {
        direction_E_W = 1;  // EAST
    }
    
    if (tmp.indexOf(",N,") < 0 && tmp.indexOf(",S,") > 0)
    {
        direction_S_N = 1;  // SOUTH
    }
    else
    {
        direction_S_N = 0;  // NORTH
    }
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
#if defined(ENABLE_RAK_GPS)
// Hilfsfunktion zum Senden
void sendUBXCommand(String cmd)
{
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  Serial1.println(cmd);
  #else
  Serial1.println(cmd);
  #endif
  delay(100); // Kurze Pause für das Modul
}

uint32_t startTimeout;
String ver = "";

void WaitPause() {
  startTimeout = millis() + 1000;
  #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
  while ((!Serial1.available()) && ((int32_t)(millis() - startTimeout) < 0)) { delay(5); } // auf Block von Zeichen warten
  #else
  while ((!Serial1.available()) && ((int32_t)(millis() - startTimeout) < 0)) { delay(5); } // auf Block von Zeichen warten
  #endif
  if(iGPSDEBUG >= 2)
    Serial.printf("[GPS ]...wait");
  startTimeout = millis() + 50;  // für Serial Sync Zeichenblock lesen und Pause von 50ms abwarten
  while ((int32_t)(millis() - startTimeout) < 0) {
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    if (Serial1.available()) {
      Serial1.read();
    #else
    if (Serial1.available()) {
      Serial1.read();
    #endif
      startTimeout = millis() + 50;  // retrigger timeout
    }
  }
}

const uint8_t UBX_MON_VER_RAK[] = {  // Size 8, swVersion, hwVersion
  0xB5, 0x62,             // Header (sync)
  0x0A, 0x04,             // Class, ID
  0x00, 0x00,             // Length (2 Bytes, Little Endian)
  0x0E, 0x34              // CK_A, CK_B
};

uint8_t ubx_cfg_gnss [] = {// GPS + GALILEO + GLONASS wo / SBAS
0xB5,0x62,0x06,0x3E, 0x3C, 0x00,
0x00,0x00,0x20,0x07,
0x00,0x00,0x10,0x00,0x01,0x00,0x01,0x01,
0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x02,0x00,0x08,0x00,0x01,0x00,0x01,0x01,
0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x06,0x00,0x10,0x00,0x01,0x00,0x01,0x01,
0xF5,0x8A};

void sendUBX_MON_VER() {  // Binäres Paket senden
  
  if(iGPSDEBUG >= 2)
    Serial.println("[GPS ]...Sende UBX_MON_VER");

  for (size_t i = 0; i < sizeof(UBX_MON_VER_RAK); i++)
  {
    Serial1.write(UBX_MON_VER_RAK[i]);
  }

  delay(100); // Kurze Pause für das Modul
}

void sendUBX_SET_GNSS() {  // Binäres Paket senden
  
  if(iGPSDEBUG >= 2)
    Serial.println("[GPS ]...Sende UBX_SET_GNSS");

  for (size_t i = 0; i < sizeof(ubx_cfg_gnss); i++)
  {
    Serial1.write(ubx_cfg_gnss[i]);
  }

  delay(100); // Kurze Pause für das Modul
}

String readUBXbin() {
  startTimeout = millis() + 500;
  ver = "";
    while ((int32_t)(millis() - startTimeout) < 0) {
    #if defined(USE_HELTEC_T114) or defined(BOARD_T_ECHO)
    while (Serial1.available()) {
      int c = Serial1.read();
    #else
    while (Serial1.available()) {
      int c = Serial1.read();
    #endif

      if ((ver.length() > 500) || (c == 0x0D)) { break; }  //TODO: mehr Zeichen für ganzen Versions-String
      if (c == 0xB5) { c = 0x75; }
      if ((c < 0x20) || (c > 0x7E)) { c = 0x2E; }
      ver = ver + char(c);
      startTimeout = millis() + 500;  // retrigger timeout
    }
  }
  return ver;
}

void SetupUBLOX()
{
  // 1. Alle Nachrichten (GSV) ausschalten, um Flut an Daten zu reduzieren
  sendUBXCommand("$PUBX,40,GSV,0,0,0,0,0,0*59");
  sendUBXCommand("$PUBX,40,VTG,0,0,0,0,0,0*5E");
  sendUBXCommand("$PUBX,40,GSA,0,0,0,0,0,0*4E");
  sendUBXCommand("$PUBX,40,GLL,0,0,0,0,0,0*5C");
  
  // 2. Nur GGA (Position) und RMC (Zeit/Datum/Speed) aktivieren
  sendUBXCommand("$PUBX,40,GGA,0,1,0,0,0,0*5B");
  sendUBXCommand("$PUBX,40,RMC,0,1,0,0,0,0*47");

  WaitPause(); // Pause zwischen Blöcken erreicht

  sendUBX_SET_GNSS();

  sendUBX_MON_VER();
  ver = readUBXbin();
  if(iGPSDEBUG >= 2)
    Serial.printf("[GPS_VER] %s\n", ver.c_str());

}

unsigned int getGPS(void)
{
    if(iGPSDEBUG > 1)
        Serial.println("-----------check GPS-----------");

    bool newData = false;
  
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        
        if(iGPSDEBUG > 2)
            Serial.write(c);

        if (tinyGPSPlus.encode(c))// Did a new valid sentence come in?
          newData = true;
      }
    }

    if(iGPSDEBUG > 1)
        Serial.printf("newData:%i SAT:%d Fix:%d UPD:%d VAL:%d HDOP:%i\n", newData, tinyGPSPlus.satellites.value(), tinyGPSPlus.sentencesWithFix(), tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.hdop.value());

    posinfo_satcount = tinyGPSPlus.satellites.value();
    fposinfo_hdop = tinyGPSPlus.hdop.value();

    bool has_gnss_location=false;

    if ((tinyGPSPlus.hdop.value() < 300) && (tinyGPSPlus.satellites.value() > 4))
    {
        has_gnss_location = true;
        posinfo_fix = true;
    }
    else
    {
        posinfo_fix = false;
    }
    
    if (newData && has_gnss_location)
    {
        double dlat, dlon;
        
        dlat = tinyGPSPlus.location.lat();
        dlon = tinyGPSPlus.location.lng();

        meshcom_settings.node_lat = cround4(dlat);
        meshcom_settings.node_lon = cround4(dlon);

        if(tinyGPSPlus.location.rawLat().negative)
            meshcom_settings.node_lat_c = 'S';
        else
            meshcom_settings.node_lat_c = 'N';

        if(tinyGPSPlus.location.rawLng().negative)
            meshcom_settings.node_lon_c = 'W';
        else
            meshcom_settings.node_lon_c = 'E';

        meshcom_settings.node_alt = ((meshcom_settings.node_alt * 10) + (int)tinyGPSPlus.altitude.meters()) / 11;

        MyClock.setCurrentTime(meshcom_settings.node_utcoff, tinyGPSPlus.date.year(), tinyGPSPlus.date.month(), tinyGPSPlus.date.day(), tinyGPSPlus.time.hour(), tinyGPSPlus.time.minute(), tinyGPSPlus.time.second());
        snprintf(cTimeSource, sizeof(cTimeSource), (char*)"GPS");

        if(iGPSDEBUG > 1)
        {
            Serial.printf("INT: LAT:%lf%c LON:%lf%c ALT:%i (%i-%02i-%02i %02i:%02i:%02i)\n", meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
            meshcom_settings.node_date_year, meshcom_settings.node_date_month,  meshcom_settings.node_date_day,
            meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second );
        }

        return setSMartBeaconing(dlat, dlon);
    }

    return POSINFO_INTERVAL;   // no GPS
}
#endif

void checkSerialCommand(void)
{
    // Serial available
    if(Serial)
    {
        // Check USB Serial input (Serial == MSerial after telnet_functions.h include)
        if(Serial.available() > 0)
        {
            char rd = (char)Serial.read();
            // Drop NUL bytes: UART RX noise (e.g. unpowered USB-UART bridge on battery
            // supply) delivers 0x00 which strlen() cannot see and wedges the parser
            // (DRY-22 — ported from the ESP32 copy of this function).
            if(rd != 0x00)
            {
                printdeb(rd);   // echo to USB + net console via MSerial
                strText[iTxtPos] = rd;
                if(iTxtPos < (int)sizeof(strText) - 1)
                {
                    iTxtPos++;
                }
            }
        }
    }

    iTxtLen = strlen(strText);

    // Self-healing: normally every stored byte is non-NUL, so strlen == iTxtPos.
    // A stray NUL in the buffer breaks that invariant and would block command
    // processing forever (early return below never reaches the memset). Discard.
    // (DRY-22 — ported from the ESP32 copy of this function.)
    if(iTxtLen != iTxtPos)
    {
        memset(strText, 0x00, sizeof(strText));
        iTxtPos = 0;
        return;
    }

    if(iTxtLen == 0)
        return;

    if(strText[0] == ':' || strText[0] == '-' || strText[0] == '{')
    {
        if(strText[iTxtLen-1] == '\n' || strText[iTxtLen-1] == '\r')
        {
            strTextWork = strText;
            strTextWork.trim();
            snprintf(strText, sizeof(strText), "%s", strTextWork.c_str());

            strncpy(msg_text, strText, sizeof(msg_text) - 1);
            msg_text[sizeof(msg_text) - 1] = '\0';

            int inext=0;
            // N-22: 600 B vom knappen 4-KB-Loop-Task-Stack in BSS verlagert —
            // checkSerialCommand() laeuft nur im Loop-Task, und der Pfad
            // ueber sendMessage() -> sendExtern() lief mit Watermark 0
            // (Details: STATUS-Box N-22 im Defektkatalog).
            static char msg_buffer[600];
            iTxtLen = strlen(strText);
            for(int itx=0; itx<iTxtLen; itx++)
            {
                if(msg_text[itx] == 0x08 || msg_text[itx] == 0x7F)
                {
                    inext--;
                    if(inext < 0)
                        inext=0;
                        
                    msg_buffer[inext+1]=0x00;
                }
                else
                {
                    msg_buffer[inext]=msg_text[itx];
                    msg_buffer[inext+1]=0x00;
                    inext++;

                    // buffer size reached
                    if(inext > (int)sizeof(msg_buffer)-2)
                        break;
                }
            }

            if(strText[0] == ':' && strText[1] == ':')
            {
                // BP-01: origin serial -- the notice comes back on the console.
                setMsgOrigin(ORIGIN_SERIAL);
                (void)sendMessage(msg_buffer, inext);
                setMsgOrigin(ORIGIN_NONE);
            }
            else
                if(strText[0] == '-' && strText[1] == '-')
                    commandAction(msg_buffer, isPhoneReady, false);
                else
                    printfdeb("\n...wrong command %s\n", strText);

            memset(strText, 0x00, sizeof(strText));
            iTxtPos = 0;
        }
    }
    else
    {
        if(bDEBUG)
        {
            if(strText[0] != '\n' && strText[0] != '\r')
            {
                printfdeb("MSG:%02X..not sent\n", (unsigned char)strText[0]);
            }
        }

        memset(strText, 0x00, sizeof(strText));
        iTxtPos = 0;
    }
}

/**@brief UDP tx Routine
 */
void sendUDP()
{
    if(udpWrite != udpRead)
    {
        if(bDisplayCont)
            Serial.printf("udpWrite:%i udpRead:%i neth.udp_is_busy:%i\n", udpWrite, udpRead, neth.udp_is_busy);

        if(!neth.udp_is_busy)
        {
            // CONC-16 (nRF52-Leser): der Schreiber addUdpOutBuffer() laeuft
            // ueber addNodeData() im Timer-Service-Task (OnRxDone, siehe
            // C-01) und kann diesen Slot per Ring-voll-Eviction ueberholen,
            // waehrend hier gesendet wird. Laenge und Payload deshalb als
            // Snapshot unter kurzem Lock lesen und den Index-Advance unten
            // gegen ein zwischenzeitliches Vorruecken sichern — gleiche
            // Behandlung wie sendMeshComUDP() in udp_functions.cpp (ESP32).
            // Snapshot bewusst groesser als der Quell-Slot und nullgefuellt
            // (siehe dortige Begruendung).
            static uint8_t udpSnapshot[UDP_TX_BUF_SIZE+64] = {0};
            int mySlot = udpRead;
            /*BISECT*/ memcpy(udpSnapshot, ringBufferUDPout[mySlot], sizeof(ringBufferUDPout[0]));

            uint16_t msg_len = udpSnapshot[0];

            // send it over UDP
            if (!neth.sendUDP(udpSnapshot + 1, msg_len))
            {
                Serial.printf("Sending UDP Packet failed <%i>!\n", msg_len);

                DEBUG_MSG("ERROR", "Sending UDP Packet failed!");

                err_cnt_udp_tx++;
                // if we have too much errors sending, reset UDP
                if (err_cnt_udp_tx >= MAX_ERR_UDP_TX)
                {
                    // avoid TX and UDP
                    neth.hasIPaddress = false;

                    Serial.print(getTimeString());
                    Serial.printf(" [MAIN] resetDHCP\n");

                    err_cnt_udp_tx = 0;
                    neth.resetDHCP();
                }
            }
            else
            {
                // UDP DATA Header 36 byte. Der Slot enthaelt msg_len Bytes ab
                // Offset 1 (Header + APRS-Frame); msg_len Bytes ab Offset 1+36
                // zu kopieren las 36 Bytes ueber das Geschriebene hinaus — bei
                // msg_len > 239 sogar ueber das Slot-Ende (Slot ist
                // UDP_TX_BUF_SIZE+20). Wahre APRS-Laenge ist msg_len-36.
                // (Nebenbefund aus dem CONC-16-Commit; auf nRF52-Gateways
                // aktiv — Schreiber ist addUdpOutBuffer() via addNodeData(),
                // auf Hardware am TX-UDP-Log verifiziert.)
                uint16_t aprs_len = (msg_len > 36) ? (uint16_t)(msg_len - 36) : 0;
                memcpy(convBuffer, udpSnapshot + 1 + 36, aprs_len);

                if(aprs_len > 0 && (convBuffer[0] == 0x3A || convBuffer[0] == 0x21 || convBuffer[0] == 0x40))
                {
                    struct aprsMessage aprsmsg;

                    // print which message type we got
                    decodeAPRS(convBuffer, aprs_len, aprsmsg);

                    // print aprs message
                    if(bDisplayVia)
                    {
                        printBuffer_aprs((char*)"[MESHu]...TX-UDP  ", aprsmsg);
                    }
                    else
                    {
                        if(bDisplayInfo)
                        {
                            printBuffer_aprs((char*)"TX-UDP  ", aprsmsg);
                        }
                    }
                }
            }

            // zero out sent buffer and advance the read pointer under the same
            // lock as the writer's addRingPointer() (CONC-16). Guard against a
            // writer having already force-advanced udpRead past us via the
            // ring-full eviction path while we were sending.
            /*BISECT*/ if (udpRead == mySlot)
            {
                memset(ringBufferUDPout[mySlot], 0, UDP_TX_BUF_SIZE);
                udpRead++;
                if (udpRead >= MAX_RING_UDP)
                    udpRead = 0;
            }

        }
        else
        {
            DEBUG_MSG("UDP", "UDP busy. Sending asap");
        }
    }
}

/**@brief Function to send our heartbeat
 * longanme0x000xAABBCCDDKEEPGW0110x00
 *               GW_ID
 */
void sendHeartbeat()
{
    if (!neth.hasIPaddress)
        return;

    sendKEEP();

}