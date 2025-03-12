// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
//
// 20230326: Version 4.00: START

#include <Arduino.h>
#include <SPI.h>
#include <SX126x-RAK4630.h>
#include <debugconf.h>
#include <time.h>

#include <TinyGPSPlus.h>

#include "Adafruit_SHTC3.h"

#include "Adafruit_LPS2X.h"

// MeshCom nrf52 functions
#include <RAK13800_W5100S.h>
#include <WisBlock-API.h>

#include <nrf_eth.h>

// Ethernet Object
NrfETH neth;

// flag to indicate one second 
unsigned long retransmit_timer = 0;

// flag to update NTP Time
unsigned long updateTimeClient = 0;

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
#include <command_functions.h>
#include <aprs_functions.h>
#include <batt_functions.h>
#include <lora_functions.h>
#include <udp_functions.h>
#include <web_functions.h>
#include <phone_commands.h>
#include <mheard_functions.h>
#include <clock.h>

#include <bmx280.h>
#include "bme680.h"
#include "mcu811.h"
#include "io_functions.h"
#include "ina226_functions.h"
#include "rtc_functions.h"
#include "softser_functions.h"

#include <onewire_functions.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGPS;

// UBLOX Modul auf Factory-Set zurück setzen
// derzeit nur im esp32_gps realisiert
// nrf52 fehlt diese Funktion noch
// Diese Variablen werden durch das Command --gps reset gesetzt und solten im GPS Modul abgehandelt werden
int state = 0; // only for gps reset
bool bMitHardReset = false;  // only for gps reset

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

bool g_meshcom_initialized;
bool init_flash_done=false;

bool bPosFirst = true;
bool bHeyFirst = true;

// Queue for sending config jsons to phone
uint8_t iPhoneState = 0;
bool config_to_phone_prepare = false;
unsigned long config_to_phone_prepare_timer = 0;
unsigned long config_to_phone_datetime_timer = 0;
const uint8_t json_configs_cnt = 7;
const char config_cmds[json_configs_cnt][20] = {"--info", "--seset", "--wifiset", "--nodeset", "--wx", "--pos", "--aprsset"};
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

String strText="";

// TinyGPS
TinyGPSPlus tinyGPSPlus;

int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

unsigned int getGPS(void);

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

#include <U8g2lib.h>

//extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
extern U8G2 *u8g2;
extern U8G2 u8g2_1;
extern U8G2 u8g2_2;

// Prototypes
void blinkLED();                                     // blink GREEN
void blinkLED2();                                    // blink BLUE

void checkSerialCommand(void);


unsigned long gps_refresh_timer = 0;
unsigned long softser_refresh_time = 0;

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

     // LEDs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

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

    // init nach Reboot
    init_loop_function();

    // user button init
    initButtonPin();

    //gps init
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, 0);
    delay(1000);
    digitalWrite(WB_IO2, 1);
    delay(1000);

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
    bEXTSER =  meshcom_settings.node_sset & 0x4000;

    bONEWIRE =  meshcom_settings.node_sset2 & 0x0001;
    bLPS33 =  meshcom_settings.node_sset2 & 0x0002;
    bBME680ON = meshcom_settings.node_sset2 & 0x0004;
    bMCU811ON =  meshcom_settings.node_sset2 & 0x0008;
    bGPSDEBUG = meshcom_settings.node_sset2 & 0x0010;
    bMESH = !(meshcom_settings.node_sset2 & 0x0020);
    bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
    bWIFIAP = meshcom_settings.node_sset2 & 0x0080;
    bGATEWAY_NOPOS =  meshcom_settings.node_sset2 & 0x0100;
    bSMALLDISPLAY =  meshcom_settings.node_sset2 & 0x0200;
    bSOFTSERON =  meshcom_settings.node_sset2 & 0x0400;
    bBOOSTEDGAIN =  meshcom_settings.node_sset2 & 0x0800;

    // nicht mehr notwendig bMHONLY =  meshcom_settings.node_sset3 & 0x0001;
    bNoMSGtoALL =  meshcom_settings.node_sset3 & 0x0002;
    bBLEDEBUG = meshcom_settings.node_sset3 & 0x0004;

    bDisplayInfo = bLORADEBUG;

    meshcom_settings.max_hop_text = MAX_HOP_TEXT_DEFAULT;
    meshcom_settings.max_hop_pos = MAX_HOP_POS_DEFAULT;

    iButtonPin = BUTTON_PIN;
    if(meshcom_settings.node_button_pin > 0)
        iButtonPin = meshcom_settings.node_button_pin;

    // if Node is in WifiAP Mode -> no Gateway posible
    if(bWIFIAP && bGATEWAY)
    {
        bGATEWAY=false;
        bEXTSER=false;
        bEXTUDP=false;
    }

    memset(meshcom_settings.node_update, 0x00, sizeof(meshcom_settings.node_update));
    
    #ifndef ENABLE_SOFTSER
        bSOFTSERON=false;
    #endif

    // Umstekllung auf langes WIFI Passwort
    if(strlen(meshcom_settings.node_ossid) > 4 && strlen(meshcom_settings.node_ssid) < 5)
    {
        strcpy(meshcom_settings.node_ssid, meshcom_settings.node_ossid);
        strcpy(meshcom_settings.node_pwd, meshcom_settings.node_opwd);

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

    if(bONEWIRE)
        init_onewire();
        

    //  Initialize the LoRa Module
    lora_rak4630_init();

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

    DisplayTimeWait=0;
    //
    ////////////////////////////////////////////////////////////////////

    if(bGPSON)
    {
        //gps init
        pinMode(WB_IO2, OUTPUT);
        digitalWrite(WB_IO2, 0);
        delay(1000);
        digitalWrite(WB_IO2, 1);
        delay(1000);
        
        Serial.println("=====================================");

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
                Serial1.end();

                delay(100);
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
    }
    else
    {
        posinfo_fix = false;
        posinfo_satcount = 0;
        posinfo_hdop = 0;
    }

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

    #if defined(SHTC3)

    Serial.println("Adafruit SHTC3 check");
    if (!shtc3.begin()) {
        Serial.println("Couldn't find SHTC3");
        while (1) delay(1);
    }
    Serial.println("SHTC3 sensor found");

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
		digitalWrite(LED_BLUE, LOW);
	}

	// Take the semaphore so the loop will go to sleep until an event happens
	xSemaphoreTake(g_task_sem, 10);
#endif

#ifdef ARDUINO_ARCH_RP2040
	// RAK11310 does not have BLE, switch off blue LED
	digitalWrite(LED_BLUE, LOW);
#endif

    // I2C init
    Wire.begin();

    #if defined(ENABLE_BMX280)
        setupBMX280(true);
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

    Serial.println(F("[INIT]...Auto detecting display:"));
    
    if (esp32_isSSD1306(0x3C))
    { //Address of the display to be checked
        Serial.println(F("[INIT]...OLED Display is SSD1306"));
        u8g2 = &u8g2_2;
    }
    else
    {
        Serial.println(F("[INIT]...OLED Display is SH1106"));
        u8g2 = &u8g2_1;
    }

    u8g2->begin();

    u8g2->clearDisplay();
    u8g2->setFont(u8g2_font_6x10_mf);
    u8g2->firstPage();
    do
    {
        u8g2->setFont(u8g2_font_10x20_mf);
        u8g2->drawStr(5, 15, "MeshCom 4.0");
        u8g2->setFont(u8g2_font_6x10_mf);
        char cvers[20];
        snprintf(cvers, sizeof(cvers), "FW %s/%s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
        u8g2->drawStr(5, 25, cvers);
        u8g2->drawStr(5, 35, "by icssw.org");
        u8g2->drawStr(5, 45, "OE1KFR, OE1KBC");
        u8g2->drawStr(5, 55, "...starting now");
    } while (u8g2->nextPage());

    // reset GPS-Time parameter
    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    Serial.println("[INIT]...CLIENT STARTED");

    //  Set the LoRa Callback Functions
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    //RadioEvents.PreAmpDetect = OnPreambleDetect;
    RadioEvents.PreAmpDetect = OnHeaderDetect;
    
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
        (uint16_t)preamble_length,
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
        (uint16_t)preamble_length,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true, // CRC ON
        0,    // fsk only frequ hop
        0,    // fsk only frequ hop period
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE);

    //  Start receiving LoRa packets
    Radio.Rx(RX_TIMEOUT_VALUE);

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
        }
        else
        {
            bGATEWAY=false;

            meshcom_settings.node_sset = meshcom_settings.node_sset & 0x6FFF;   // mask 0x1000

            addBLECommandBack((char*)"--gateway off");

            save_settings();
        }
    }
}

void nrf52loop()
{
    // get RTC Now
    // RTC hat Vorrang zu Zeit via MeshCom-Server
    bool bMyClock = true;

    if(bRTCON)
    {
        bMyClock = false;

        loopRTC();

        if(!bGPSON) // GPS hat Vorang zur RTC
        {
            DateTime utc = getRTCNow();

            DateTime now (utc + TimeSpan(meshcom_settings.node_utcoff * 60 * 60));

            meshcom_settings.node_date_year = now.year();
            meshcom_settings.node_date_month = now.month();
            meshcom_settings.node_date_day = now.day();

            meshcom_settings.node_date_hour = now.hour();
            meshcom_settings.node_date_minute = now.minute();
            meshcom_settings.node_date_second = now.second();
        }
    }
    else
    if(meshcom_settings.node_hasIPaddress)
    {
        String strTime = "none";

        // every five minutes
        if((updateTimeClient + 1000 * 60 * 5) < millis() || updateTimeClient == 0)
        {
            strTime = neth.udpUpdateTimeClient();

            updateTimeClient = millis();
        }
        else
            strTime = neth.udpGetTimeClient();

        String strDate = neth.udpGetDateClient();

        uint16_t Year = (uint16_t)strDate.substring(0, 4).toInt();
        uint16_t Month = (uint16_t)strDate.substring(5, 7).toInt();
        uint16_t Day = (uint16_t)strDate.substring(8, 10).toInt();

        uint16_t Hour = (uint16_t)strTime.substring(0, 2).toInt();
        uint16_t Minute = (uint16_t)strTime.substring(3, 5).toInt();
        uint16_t Second = (uint16_t)strTime.substring(6, 8).toInt();
    
        // check valid Date & Time
        if(Year > 2020 && strTime.compareTo("none") != 0)
        {
            MyClock.setCurrentTime(meshcom_settings.node_utcoff, Year, Month, Day, Hour, Minute, Second);
            bNTPDateTimeValid = true;
        }
        else
            bNTPDateTimeValid = false;

    }
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

    if(!bGATEWAY)
    {
        if ((retransmit_timer + (1000 * 2)) < millis())   // repeat 2 seconds
        {
            updateRetransmissionStatus();

            retransmit_timer = millis();
        }
    }

    if(iReceiveTimeOutTime > 0)
    {
        // Timeout RECEIVE_TIMEOUT
        if((iReceiveTimeOutTime + RECEIVE_TIMEOUT) < millis())
        {
            iReceiveTimeOutTime=0;

            // LoRa preamble was detected
            if(bLORADEBUG)
            {
                Serial.printf("[SX12xx] Receive Timeout, starting receiving again ... \n");
            }
        }
    }

    if(iReceiveTimeOutTime == 0 && is_receiving == false && tx_is_active == false)
    {
        // channel is free
        // nothing was detected
        // do not print anything, it just spams the console
        if (iWrite != iRead)
        {
            // save transmission state between loops
            doTX();
        }
    }

    // SOFTSER
    #if defined(ENABLE_SOFTSER)
        if(bSOFTSERON)
        {
            if (bSOFTSER_APP || ((softser_refresh_time + ((SOFTSER_REFRESH_INTERVAL * 1000) - 3000)) < millis()))
            {
                // start SOFTSER APP
                loopSOFTSER(SOFTSER_APP_ID, 0);

                softser_refresh_time = millis();

                bSOFTSER_APP = false;
            }
            else
            {
                appSOFTSER(SOFTSER_APP_ID);
            }
        }
    #endif

    checkButtonState();

    // check if message from phone to send
    if(hasMsgFromPhone)
    {
        //sendMessage(textbuff_phone, txt_msg_len_phone);

        if(memcmp(textbuff_phone, ":", 1) == 0)
            sendMessage(textbuff_phone, txt_msg_len_phone);

        if(memcmp(textbuff_phone, "-", 1) == 0)
            commandAction(textbuff_phone, isPhoneReady, true);

        hasMsgFromPhone = false;
    }

    if(gKeyNum == 1)
    {
        Serial.println("Left button pressed");

        //Serial.println("gKeyNum == 1");

        //getTEMP();

        //getPRESSURE();

        gKeyNum = 0;
    }

    if(gKeyNum == 2)
    {
        //Serial.println("gKeyNum == 2");

        #ifdef ENABLE_GPS

        if(bGPSON)
        {
            // gps refresh every 10 sec
            if ((gps_refresh_timer + (GPS_REFRESH_INTERVAL * 1000)) < millis())
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

        gKeyNum = 0;
    }

    if(gKeyNum == 3)
    {
        Serial.println("Right button pressed");

        gKeyNum = 0;
    }

    #ifdef ENABLE_GPS
    if(bGPSON)
    {
        // check GPS ON and activ --> <gKeyNum == 2> the signal must be active
        if ((gps_refresh_timer + (5 * (GPS_REFRESH_INTERVAL * 1000))) < millis())
        {
            posinfo_fix = false;
            posinfo_satcount = 0;
            posinfo_hdop = 0;
            posinfo_interval = POSINFO_INTERVAL;
        }
    }
    #endif

if (isPhoneReady == 1)
    {
        if (config_to_phone_prepare)
        {
            for(int config_cmds_index=0; config_cmds_index < json_configs_cnt; config_cmds_index++)
            {
                commandAction((char*)config_cmds[config_cmds_index], isPhoneReady, true);

//                sendMessage((char*)config_cmds[config_cmds_index], strlen(config_cmds[config_cmds_index]));
            }

            sendMheard();

            config_to_phone_prepare_timer=millis();

            config_to_phone_prepare = false;
        }
        else
        {
            // wait after BLE Connect 3 sec.
            if(millis() < config_to_phone_prepare_timer + 3000)
                iPhoneState = 0;

            if (iPhoneState > 3)   // only every 6 times of mainloop send to phone  RAK 2 x ESP
            {
                // prepare JSON config to phone after BLE connection
                // send JSON config to phone after BLE connection
                if (ComToPhoneWrite != ComToPhoneRead)
                {
                    sendComToPhone();   
                }
                else
                {
                    // check if we have messages for BLE to send
                    if (toPhoneWrite != toPhoneRead)
                    {
                        sendToPhone();   
                    }
                }

                iPhoneState = 0;
            }
            else
            {
                iPhoneState++;
            }
        }

        // 5 minuten
        if((config_to_phone_datetime_timer + (5 * 60 * 1000)) < millis())
        {
            bNTPDateTimeValid=false;

            config_to_phone_datetime_timer = millis();
        }
    }

    // posinfo
    //Serial.print(getTimeString());
    //Serial.printf(" posinfo_timer:%ld posinfo_interval:%ld timer:%ld millis:%ld\n", posinfo_timer, posinfo_interval, (posinfo_timer + (posinfo_interval * 1000)), millis());

    // posinfo_interval in Seconds
    if (((posinfo_timer + (posinfo_interval * 1000)) < millis()) || (millis() > 100000 && millis() < 130000 && bPosFirst) || posinfo_shot)
    {
        bPosFirst = false;
        posinfo_shot=false;
        posinfo_timer = millis();
        
        sendPosition(posinfo_interval, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

        posinfo_last_lat=posinfo_lat;
        posinfo_last_lon=posinfo_lon;
        posinfo_last_direction=posinfo_direction;

        if(pos_shot)
        {
            commandAction((char*)"--pos", isPhoneReady, true);
            pos_shot = false;
        }
    }

    // heysinfo_interval in Seconds == 15 minutes
    if (((heyinfo_timer + (HEYINFO_INTERVAL * 1000)) < millis()) || bHeyFirst)
    {
        bHeyFirst = false;
        
        if(!bGATEWAY)
            sendHey();

        heyinfo_timer = millis();
    }

    // get UDP & send UDP message from ringBufferOut if there is one to tx
    if(bGATEWAY)
    {
        int bUDPReceived = false;

        // check if we received a UDP packet
        if (neth.hasIPaddress)
        {
            if(neth.getUDP() == 1)  // 1...no udp-paket received
            {
                sendUDP(); 
            }
            else
            {
                bUDPReceived=true;

                if(bDEBUG)
                    Serial.println("LOOP GATEWAY actions UDP received");
            }
        }
        else
        {
            neth.last_upd_timer = 0; // ETH new
        }

        // UDP Action for next loop
        if(!bUDPReceived)
        {
            meshcom_settings.node_hasIPaddress = neth.hasIPaddress;
            meshcom_settings.node_last_upd_timer = neth.last_upd_timer;
            
            // check HB response (we also check successful sending KEEP. check if they work together!)
            if((neth.last_upd_timer + (MAX_HB_RX_TIME * 1000)) < millis())
            {
                if(bDEBUG)
                    Serial.println("LOOP GATEWAY last_upd_timer actions");

                neth.last_upd_timer = millis();

                // avoid TX and UDP
                if(!neth.hasIPaddress)
                {
                    neth.hasIPaddress = false;
                    cmd_counter = 50;

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
                        {
                            Serial.print(getTimeString());
                            Serial.println(" [MAIN] initethETH DHCP");
                        }

                        neth.initethDHCP();
                    }
                }
            }
            
            // DHCP refresh
            if ((dhcp_timer + (DHCP_REFRESH * 60000)) < millis())
            {
                // no need on static IPs
                if(!(strlen(meshcom_settings.node_ownip) > 6 && strlen(meshcom_settings.node_ownms) > 6 && strlen(meshcom_settings.node_owngw) > 6))
                {
                    if(bDEBUG)
                    {
                        Serial.print(getTimeString());
                        Serial.println(" [MAIN] checkDHCP");
                    }
                    
                    neth.checkDHCP();
                }

                dhcp_timer = millis();
            }
        }
    }

    #if defined(SHTC3)

    // TEMP/HUM
    if (((temphum_timer + TEMPHUM_INTERVAL) < millis()))
    {
        getTEMP();

        temphum_timer = millis();
    }

    #endif

    #if defined(LPS33)

    if(bLPS33)
    {
        // DRUCK
        if (((druck_timer + DRUCK_INTERVAL) < millis()))
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

    if(onewireTimeWait == 0)
        onewireTimeWait = millis() - 10000;


    if ((onewireTimeWait + 10000) < millis())  // 10 sec
    {
        //if (tx_is_active == false && is_receiving == false)
        {
            loop_onewire();

            onewireTimeWait = millis();

            if(wx_shot)
            {
                commandAction((char*)"--wx", isPhoneReady, true);
                wx_shot = false;
            }
        }
    }

    checkButtonState();

    mainStartTimeLoop();

    if(DisplayOffWait > 0)
    {
        if (millis() > DisplayOffWait)
        {
            DisplayOffWait = 0;
            bDisplayOff=true;
            commandAction((char*)"--display off", isPhoneReady, false);
            sendDisplay1306(true, true, 0, 0, (char*)"#C");
        }
    }
    
    // rebootAuto
    if(rebootAuto > 0)
    {
        if (millis() > rebootAuto)
        {
            rebootAuto = 0;

            #ifdef ESP32
                ESP.restart();
            #endif
            
            #if defined NRF52_SERIES
                NVIC_SystemReset();     // resets the device
            #endif
        }
    }

    checkSerialCommand();

    if(BattTimeWait == 0)
        BattTimeWait = millis() - 31000;

    if ((BattTimeWait + 30000) < millis())
    {
        if (tx_is_active == false && is_receiving == false)
        {
            global_batt = read_batt();
            global_proz = mv_to_percent(global_batt);

            BattTimeWait = millis();
        }
    }
    

    if(bONEWIRE)
    {
        if(onewireTimeWait == 0)
            onewireTimeWait = millis() - 10000;


        if ((onewireTimeWait + 10000) < millis())  // 10 sec
        {
            //if (tx_is_active == false && is_receiving == false)
            {
                loop_onewire();

                onewireTimeWait = millis();

                if(wx_shot)
                {
                    commandAction((char*)"--wx", isPhoneReady, true);
                    wx_shot = false;
                }
            }
        }
    }

    if(BMXTimeWait == 0)
        BMXTimeWait = millis() - 10000;

    if ((BMXTimeWait + 60000) < millis())   // 60 sec
    {
        // read BMX Sensor
        if(loopBMX280())
        {
            meshcom_settings.node_temp = getTemp();
            meshcom_settings.node_hum = getHum();  //BMP280 - not supported
            meshcom_settings.node_press = getPress();

            if(wx_shot)
            {
                commandAction((char*)"--wx", isPhoneReady, true);
                wx_shot = false;
            }
        }

        BMXTimeWait = millis();
    }

    // read every n seconds the bme680 sensor calculated from millis()
    #if defined(ENABLE_BMX680)
    if(bBME680ON && bme680_found)
    {
        if ((bme680_timer + 60000) < millis() || delay_bme680 <= 0)
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

    // read BMP Sensor
    #if defined(ENABLE_BMX280)
    if((BMPON || bBMEON) && bmx_found)
    {
        if(BMXTimeWait == 0)
            BMXTimeWait = millis() - 10000;

        if ((BMXTimeWait + 30000) < millis())   // 30 sec
        {
                if(loopBMX280())
                {
                    meshcom_settings.node_press = getPress();
                    meshcom_settings.node_temp = getTemp();
                    meshcom_settings.node_hum = getHum();
                    meshcom_settings.node_press_alt = getPressALT();
                    meshcom_settings.node_press_asl = getPressASL(meshcom_settings.node_alt);
                    
                    if(wx_shot)
                    {
                        commandAction((char*)"--wx", isPhoneReady, true);
                        wx_shot = false;
                    }
                }

            BMXTimeWait = millis(); // wait for next messurement
        }
    }
    #endif

    if(bMCU811ON)
    {
        if(MCU811TimeWait == 0)
            MCU811TimeWait = millis() - 10000;

        if ((MCU811TimeWait + 60000) < millis())   // 60 sec
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

    #if defined(ENABLE_INA226)
    if(bINA226ON)
    {
        if(INA226TimeWait == 0)
            INA226TimeWait = millis() - 10000;

        if ((INA226TimeWait + 60000) < millis())   // 60 sec
        {
            // read MCU-811 Sensor
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
        if ((bme680_timer + 60000) < millis() || delay_bme680 <= 0)
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
        if ((hb_timer + (HEARTBEAT_INTERVAL * 1000)) < millis())
        {
            if(bDisplayInfo)
            {
                Serial.print(getTimeString());
                Serial.printf(" [UDP] sending Heartbeat\n");
            }

            sendHeartbeat();

            hb_timer = millis();
        }
    }

    checkButtonState();

    if(bWEBSERVER)
    {
        if (web_timer == 0 || ((web_timer + (HEARTBEAT_INTERVAL * 1000 * 30)) < millis()))   // repeat 15 minutes
        {
            meshcom_settings.node_hasIPaddress = neth.hasIPaddress;

            web_timer = millis();

            #ifndef BOARD_RAK4630
                // restart WEB-Client
                stopWebserver();

                if(!meshcom_settings.node_hasIPaddress)
                    startWIFI();
            #endif

            startWebserver();
        }

        loopWebserver(); 

    }

    //  We are on FreeRTOS, give other tasks a chance to run
    delay(100);
    
    yield();
}

/** @brief Function to check if the modem detected a preamble
 */
void blinkLED()
{
    digitalWrite(LED_GREEN, HIGH);
    delay(5);
    digitalWrite(LED_GREEN, LOW);
}

void blinkLED2()
{
    digitalWrite(LED_BLUE, HIGH);
    delay(20);
    digitalWrite(LED_BLUE, LOW);
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

    meshcom_settings.node_temp = temp.temperature;
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
unsigned int getGPS(void)
{
    if(bGPSDEBUG)
        Serial.println("-----------check GPS-----------");

    bool newData = false;
  
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        
        if(bGPSDEBUG)
            Serial.write(c);

        if (tinyGPSPlus.encode(c))// Did a new valid sentence come in?
          newData = true;
      }
    }

    if(bGPSDEBUG)
        Serial.printf("newData:%i SAT:%d Fix:%d UPD:%d VAL:%d HDOP:%i\n", newData, tinyGPSPlus.satellites.value(), tinyGPSPlus.sentencesWithFix(), tinyGPSPlus.location.isUpdated(), tinyGPSPlus.location.isValid(), tinyGPSPlus.hdop.value());

    if (newData && tinyGPSPlus.location.isUpdated() && tinyGPSPlus.location.isValid() && tinyGPSPlus.hdop.isValid() && tinyGPSPlus.hdop.value() < 800)
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

        posinfo_satcount = tinyGPSPlus.satellites.value();
        posinfo_hdop = tinyGPSPlus.hdop.value();
        posinfo_fix = true;

        if(bGPSDEBUG)
        {
            Serial.printf("INT: LAT:%lf%c LON:%lf%c ALT:%i (%i-%02i-%02i %02i:%02i:%02i)\n", meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
            meshcom_settings.node_date_year, meshcom_settings.node_date_month,  meshcom_settings.node_date_day,
            meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second );
        }


        posinfo_satcount = tinyGPSPlus.satellites.value();
        posinfo_hdop = tinyGPSPlus.hdop.value();
        posinfo_fix = true;

        return setSMartBeaconing(dlat, dlon);
    }
    else
    {
        posinfo_fix = false;
        posinfo_satcount = 0;
        posinfo_hdop = 0;
    }

    return 0;   // no GPS
}

void checkSerialCommand(void)
{
    //  Check Serial connected
    if(!Serial)
    {
        DEBUG_MSG("SERIAL", "not connected");
        return;
    }

 	//check if we got from the serial input
    if(Serial.available() > 0)
    {
        char rd = Serial.read();

        Serial.print(rd);

        strText += rd;

        if(strText.startsWith(":") || strText.startsWith("-"))
        {
            if(strText.endsWith("\n") || strText.endsWith("\r"))
            {
                strText.trim();
                strcpy(msg_text, strText.c_str());

                int inext=0;
                char msg_buffer[300];
                for(int itx=0; itx<(int)strText.length(); itx++)
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
                        if(inext >= (int)sizeof(msg_buffer)-2)
                            inext=(int)sizeof(msg_buffer)-2;
                    }
                }

                if(strText.startsWith(":"))
                    sendMessage(msg_buffer+1, inext-1);
                else
                    if(strText.startsWith("-"))
                        commandAction(msg_buffer, isPhoneReady, false);

                strText="";
            }
        }
        else
        {
            if(bDEBUG)
            {
                if(!strText.startsWith("\n") && !strText.startsWith("\r"))
                {
                    printf("MSG:%02X", rd);
                    printf("..not sent\n");
                }
            }
            strText="";
        }
    }
}

/**@brief UDP tx Routine
 */
void sendUDP()
{
    if(udpWrite != udpRead)
    {
        if(bDEBUG)
            Serial.printf("udpWrite:%i udpRead:%i neth.udp_is_busy:%i\n", udpWrite, udpRead, neth.udp_is_busy);

        if(!neth.udp_is_busy)
        {
            uint8_t msg_len = ringBufferUDPout[udpRead][0];
            
            
            if(msg_len != 23)
            {
                //Serial.printf("UDP TX out:%i len:%i\n", udpRead, msg_len);
                //DEBUG_MSG_VAL("UDP", udpRead, "UDP TX out:");
                //printBuffer(ringBufferUDPout[udpRead] + 1, msg_len);
            }
            

            // send it over UDP
            if (!neth.sendUDP(ringBufferUDPout[udpRead] + 1, msg_len))
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
                memcpy(convBuffer, ringBufferUDPout[udpRead] + 1 + 18, msg_len);

                //Serial.printf("convBuffer[0] %02X\n", convBuffer[0]);

                if(convBuffer[0] == 0x3A || convBuffer[0] == 0x21 || convBuffer[0] == 0x40)
                {
                    struct aprsMessage aprsmsg;
                    
                    // print which message type we got
                    decodeAPRS(convBuffer, (uint8_t)msg_len, aprsmsg);

                    // print aprs message
                    if(bDisplayInfo)
                    {
                        printBuffer_aprs((char*)"TX-UDP ", aprsmsg);
                        Serial.println("");
                    }
                }
            }

            // zero out sent buffer
            memset(ringBufferUDPout[udpRead], 0, UDP_TX_BUF_SIZE);

            udpRead++;
            if (udpRead >= MAX_RING_UDP) 
                udpRead = 0;

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
