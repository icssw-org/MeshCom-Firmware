// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// This code is not for distribution and may not be published!
// Vervielfältigung und Veröffentlichung des Codes sind nicht gestattet!
// Lizenz: kein Open Source.
// zugelassen ausschließlich für OE1KBC, OE1KFR
// alle anderen Finger weg !
// 20230326: Version 4.00: START

#include <Arduino.h>
#include <configuration.h>
#include <RadioLib.h>

#include <Wire.h>               
#include <SPI.h>
#include <WiFi.h>

#include "esp32_gps.h"
#include "esp32_flash.h"

// Sensors
#include "bmx280.h"
#include "bme680.h"
#include "mcu811.h"
#include "io_functions.h"
#include "ina226_functions.h"
#include "rtc_functions.h"
#include "softser_functions.h"

// MeshCom Common (ers32/nrf52) Funktions
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <command_functions.h>
#include <phone_commands.h>
#include <aprs_functions.h>
#include <batt_functions.h>
#include <lora_functions.h>
#include <udp_functions.h>
#include <web_functions.h>
#include <mheard_functions.h>
#include <clock.h>
#include <onewire_functions.h>
#include <lora_setchip.h>

#ifdef BOARD_E290
#include "heltec-eink-modules.h"

extern EInkDisplay_VisionMasterE290 e290_display;

#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSans18pt7b.h"

#endif

#ifndef BOARD_TLORA_OLV216
    #include <lora_setchip.h>
#endif

#include <esp_adc_cal.h>

#if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)

#include "XPowersAXP192.tpp"
#include "XPowersAXP2101.tpp"
#include "XPowersLibInterface.hpp"
extern XPowersLibInterface *PMU;

#endif

#ifdef BOARD_E290
#else

#include <U8g2lib.h>

#if defined(BOARD_HELTEC)
    extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
#elif defined(BOARD_HELTEC_V3)
    extern U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2;
#elif defined(BOARD_TBEAM)
    extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
#elif defined(BOARD_RAK4630)
    extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
#elif defined(BOARD_E290)
    extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
#else
    extern U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2;
#endif

#endif

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

bool bPosFirst = true;
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

// Textmessage buffer from phone, hasMsgFromPhone flag indicates new message
extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;

NimBLEServer *pServer = NULL;
NimBLECharacteristic* pTxCharacteristic;
NimBLEService *pService;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint32_t PIN = 000000;             // pairing password PIN Passwort PIN-Code Kennwort

// Queue for sending config jsons to phone
bool config_to_phone_prepare = false;
unsigned long config_to_phone_prepare_timer = 0;
const uint8_t json_configs_cnt = 7;
const char config_cmds[json_configs_cnt][20] = {"--info", "--seset", "--wifiset", "--nodeset", "--wx", "--pos", "--aprsset"};
uint8_t config_cmds_index = 0;
uint8_t iPhoneState=0;

// Bluetooth UUIDs are standardized. For more info: https://www.bluetooth.com/specifications/assigned-numbers/
// Nordic UUID DB is here: https://github.com/NordicSemiconductor/bluetooth-numbers-database


class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer)
    {
        deviceConnected = true;
        config_to_phone_prepare = true;
        // set the config finish msg for phone at the end of the queue, so it comes after the offline TXT msgs
        commandAction((char*)"--conffin", true);
        Serial.println("BLE connected");
    };

    void onDisconnect(NimBLEServer* pServer)
    {
        deviceConnected = false;
        Serial.println("BLE disconnected");
    }

	/***************** New - Security handled here ********************
	****** Note: these are the same return values as defaults ********/
	uint32_t onPassKeyRequest() {
		Serial.printf("Server PassKeyRequest <%06i>\n", PIN);
		return PIN;
	}
	/*******************************************************************/
};

class MyCallbacks: public NimBLECharacteristicCallbacks 
{
    void onWrite(NimBLECharacteristic *pCharacteristic)
    {
        // Forward data from Mobile to our peripheral
        uint8_t conf_data[MAX_MSG_LEN_PHONE] = {0};
        size_t conf_length=0;

        conf_length = pCharacteristic->getDataLength(); // getLength();

        if (conf_length <= 0)
            return;

        memcpy(conf_data, pCharacteristic->getValue() , conf_length);

    	readPhoneCommand(conf_data);
    };
};

#ifdef SX127X
// RadioModule SX1278 
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
// SyncWord - 1byte for SX127x, 2 bytes for SX126x
// Check which chip is used !!!
#endif

#ifdef BOARD_E220
// RadioModule derived from SX1262 
LLCC68 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
#endif

#ifdef SX126X
    // RadioModule SX1268 
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    SX1268 radio = new Module(SX1268_CS, SX1268_IRQ, SX1268_RST, SX1268_GPIO);

#endif

#ifdef SX126X_V3
    // RadioModule SX1262
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    //  begin(sck, miso, mosi, ss).
    SX1262 radio = new Module(SX126X_CS, SX126X_IRQ, SX126X_RST, LORA_DIO2);

#endif

#ifdef SX1262_E290
    // RadioModule SX1262
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!

    //  begin(sck, miso, mosi, ss).
    SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO_1, PIN_LORA_NRST, PIN_LORA_BUSY);

#endif

// Lora callback Function declarations
int checkRX(void);

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_UNKNOWN;

// flag to indicate that a preamble was not detected
volatile bool receiveFlag = false;
volatile bool bEnableInterruptReceive = true;

// flag to indicate if we are after receiving
unsigned long iReceiveTimeOutTime = 0;

// flag to indicate if we are currently allowed to transmittig
volatile bool transmittedFlag = false;
volatile bool bEnableInterruptTransmit = false;

// flag to indicate that a packet was detected or CAD timed out
volatile bool scanFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlagReceive(void)
{
    if(bEnableInterruptReceive)
    {
        receiveFlag = true;

        if(bLORADEBUG)
            Serial.println("receiveFlag");
    }

    if(bEnableInterruptTransmit)
    {
        transmittedFlag = true;

        if(bLORADEBUG)
            Serial.println("transmittedFlag");
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

        if(bLORADEBUG)
            Serial.println("receiveFlag");
    }

    if(bEnableInterruptTransmit)
    {
        transmittedFlag = true;

        if(bLORADEBUG)
            Serial.println("transmittedFlag");
    }
}

void enableRX(void);    // for Modules with RXEN / TXEN Pin
void enableTX(void);    // for Modules with RXEN / TXEN Pin


asm(".global _scanf_float");
asm(".global _printf_float");

// LoRa Events and Buffers

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = SOURCE_TYPE;

uint64_t timeStamp = 0;

/** Flag if BLE UART is connected */
bool g_ble_uart_is_connected = false;

// Client basic variables
uint8_t dmac[6] = {0};

unsigned long gps_refresh_timer = 0;
unsigned long softser_refresh_timer = 0;

bool is_new_packet(uint8_t compBuffer[4]);     // switch if we have a packet received we never saw before RcvBuffer[12] changes, rest is same
void checkSerialCommand(void);

bool g_meshcom_initialized;
bool init_flash_done=false;

String strText="";

unsigned int  getMacAddr(void)
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


void esp32setup()
{
    Serial.begin(MONITOR_SPEED);
    //while(!Serial);

    delay(1500);

    Serial.println("");
    Serial.println("");
    Serial.println("============");
    Serial.println("CLIENT SETUP");
    Serial.println("============");

    // init nach Reboot
    init_loop_function();

    // Initialize mheard list
    initMheard();

	// Get LoRa parameter
	init_flash();

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
    bBLElong = meshcom_settings.node_sset & 0x0800;
    bGATEWAY =  meshcom_settings.node_sset & 0x1000;
    bEXTUDP =  meshcom_settings.node_sset & 0x2000;
    bEXTSER =  meshcom_settings.node_sset & 0x4000;

    bONEWIRE =  meshcom_settings.node_sset2 & 0x0001;
    bLPS33 =  meshcom_settings.node_sset2 & 0x0002;
    bBME680ON =  meshcom_settings.node_sset2 & 0x0004;
    bMCU811ON =  meshcom_settings.node_sset2 & 0x0008;
    bGPSDEBUG = meshcom_settings.node_sset2 & 0x0010;
    bMESH = !(meshcom_settings.node_sset2 & 0x0020);
    bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
    bWIFIAP = meshcom_settings.node_sset2 & 0x0080;
    bGATEWAY_NOPOS =  meshcom_settings.node_sset2 & 0x0100;
    bSMALLDISPLAY =  meshcom_settings.node_sset2 & 0x0200;
    bSOFTSERON =  meshcom_settings.node_sset2 & 0x0400;

    bMHONLY =  meshcom_settings.node_sset3 & 0x0001;

    // if Node not set --> WifiAP Mode on
    if(meshcom_settings.node_call[0] == 0x00 || memcmp(meshcom_settings.node_call, "none", 4) == 0)
    {
        bWIFIAP = true;
        bWEBSERVER = true;
    }

    // if Node is in WifiAP Mode -> no Gateway posible
    if(bWIFIAP && bGATEWAY)
    {
        bGATEWAY=false;
    }

    if(bBMPON)
    {
        bBMEON=false;
        bBME680ON=false;
    }
    else
    if(bBMEON)
    {
        bBMPON=false;
        bBME680ON=false;
    }

    bDisplayInfo = bLORADEBUG;

    meshcom_settings.max_hop_text = MAX_HOP_TEXT_DEFAULT;
    meshcom_settings.max_hop_pos = MAX_HOP_POS_DEFAULT;

    // Initialize battery reading
	init_batt();


    global_batt = 4125.0;

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

    if(strcmp(meshcom_settings.node_ssid, "XX0XXX") == 0)
    {
        sprintf(meshcom_settings.node_ssid, (char*)"none");
        sprintf(meshcom_settings.node_pwd, (char*)"none");
    }

    #ifdef MODUL_FW_TBEAM
        if(meshcom_settings.node_sset == 0x0000)
        {
            bButtonCheck = true;
            meshcom_settings.node_sset = meshcom_settings.node_sset | 0x0035;	// bDisplayPos = true, bButtonCheck = true, bGPSON = true
            save_settings();
        }
    #endif

    _GW_ID = getMacAddr();

    #ifdef BOARD_HELTEC_V3
        SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
    #endif

    bool bSETGPS_POWER=false;

    #if defined(ENABLE_GPS)
        bSETGPS_POWER=true;
    #endif

    #if defined(MODUL_FW_TBEAM)
        setupGPS(bSETGPS_POWER);
    #else
        Wire.begin(I2C_SDA, I2C_SCL);
    #endif

    #if defined(ENABLE_BMX280)
        setupBMX280();
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

	// Initialize temp sensor
    if(bONEWIRE)
        init_onewire();


    //#if defined(BOARD_HELTEC) || defined(BOARD_HELTEC_V3)  || defined(BOARD_E22)
    //    Wire.setPins(I2C_SDA, I2C_SCL);
    //#endif

    initButtonPin();
    
    Serial.printf("[INIT]..._GW_ID: %08X\n", _GW_ID);

    ////////////////////////////////////////////////////////////////////
    // Initialize time
	bool boResult;
	
	// initialize clock
	boResult = MyClock.Init();
	Serial.printf("[INIT]...Initialize clock: %s\n", (boResult) ? "ok" : "FAILED");

    DisplayTimeWait=0;
    //
    ////////////////////////////////////////////////////////////////////

#if defined(BOARD_E22) || defined(BOARD_E220)
    // if RESET Pin is connected
    pinMode(LORA_RST, PULLUP);
    digitalWrite(LORA_RST, LOW);
    delay(200);
    digitalWrite(LORA_RST, HIGH);

    // we have TXEN and RXEN Pin connected
    radio.setRfSwitchPins(RXEN, TXEN);
#endif

    char cvers[10];
    sprintf(cvers, "%s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());

    #ifdef BOARD_E290
    
    e290_display.clear();
    e290_display.fastmodeOn();

    e290_display.landscape();

    e290_display.setRotation(270);

    e290_display.fillCircle(10, 10,
        10,                             // Radius: 10px
        BLACK                           // Color: black
        );

    e290_display.setFont( &FreeSansBold12pt7b );
    e290_display.setCursor(20, 50);
    e290_display.printf("MeshCom %s\n", cvers);
    e290_display.setCursor(65, 80);
    e290_display.setFont( &FreeSans12pt7b );
    e290_display.println("HELTEC E290");

    e290_display.setFont( &FreeSans9pt7b );
    e290_display.setCursor(30, 18);
    e290_display.println("...starting now");
    e290_display.setCursor(80, 100);
    e290_display.println("@by icssw.org");
    e290_display.setCursor(65, 120);
    e290_display.println("OE1KBC, OE1KFR");

    e290_display.update();

    #else

    u8g2.begin();

    u8g2.clearDisplay();
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_10x20_mf);
        u8g2.drawStr(5, 20, "MeshCom 4.0");
        u8g2.setFont(u8g2_font_6x10_mf);
        u8g2.drawStr(5, 30, cvers);
        u8g2.drawStr(5, 40, "by icssw.org");
        u8g2.drawStr(5, 50, "OE1KFR, OE1KBC");
        u8g2.drawStr(5, 60, "...starting now");
    } while (u8g2.nextPage());

    #endif

    bool bRadio=false;

    //TEST #ifndef BOARD_E290

    // initialize SX12xx with default settings
    Serial.print(F("[INIT]...LoRa Modem Initializing ... "));

    int state = RADIOLIB_ERR_UNKNOWN;
    
    #if defined(BOARD_E220)
        state = radio.begin(434.0F, 125.0F, 9, 7, SYNC_WORD_SX127x, 10, LORA_PREAMBLE_LENGTH, /*float tcxoVoltage = 0*/ 0, /*bool useRegulatorLDO = false*/ false);
    #else
        state = radio.begin();
    #endif
    
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("LoRa-Chip success!"));
        bRadio=true;
    }
    else
    {
        Serial.print(F("LoRa-Chip failed, code "));
        Serial.println(state);
        bRadio=false;
    }

    #if defined(BOARD_E220)
        bRadio = false; // no detailed setting
    #endif

    // you can also change the settings at runtime
    // and check if the configuration was changed successfully
    if(bRadio)
    {
        lora_setcountry(meshcom_settings.node_country);

        // set carrier frequency
        Serial.printf("[LoRa]...RF_FREQUENCY: %.3f MHz\n", meshcom_settings.node_freq);
        if (radio.setFrequency(meshcom_settings.node_freq) == RADIOLIB_ERR_INVALID_FREQUENCY) {
            Serial.println(F("Selected frequency is invalid for this module!"));
            while (true);
        }

        // set bandwidth 
        Serial.printf("[LoRa]...RF_BANDWIDTH: %.0f kHz\n", meshcom_settings.node_bw);
        if (radio.setBandwidth(meshcom_settings.node_bw) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
            Serial.println(F("Selected bandwidth is invalid for this module!"));
            while (true);
        }


        // set spreading factor 
        Serial.printf("[LoRa]...RF_SF: %i\n", meshcom_settings.node_sf);
        if (radio.setSpreadingFactor(meshcom_settings.node_sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
            Serial.println(F("Selected spreading factor is invalid for this module!"));
            while (true);
        }

        // set coding rate 
        Serial.printf("[LoRa]...RF_CR: 4/%i\n", meshcom_settings.node_cr);
        if (radio.setCodingRate(meshcom_settings.node_cr) == RADIOLIB_ERR_INVALID_CODING_RATE) {
            Serial.println(F("Selected coding rate is invalid for this module!"));
            while (true);
        }

        // set LoRa sync word 
        // NOTE: value 0x34 is reserved for LoRaWAN networks and should not be used
        if (radio.setSyncWord(SYNC_WORD_SX127x) != RADIOLIB_ERR_NONE) {
            Serial.println(F("Unable to set sync word!"));
            while (true);
        }

        // set output power to 10 dBm (accepted range is -3 - 17 dBm)
        // NOTE: 20 dBm value allows high power operation, but transmission
        //       duty cycle MUST NOT exceed 1%
        
        int8_t tx_power = TX_OUTPUT_POWER;
        
        if(meshcom_settings.node_power <= 0)
            meshcom_settings.node_power = TX_OUTPUT_POWER;
        else
            tx_power=meshcom_settings.node_power;   //set by command

        if(tx_power > TX_POWER_MAX)
            tx_power= TX_POWER_MAX;

        if(tx_power < TX_POWER_MIN)
            tx_power= TX_POWER_MIN;

        Serial.printf("[LoRa]...RF_POWER: %d dBm\n", tx_power);

        if (radio.setOutputPower(tx_power) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
            Serial.println(F("Selected output power is invalid for this module!"));
            while (true);
        }

        // set over current protection limit (accepted range is 45 - 240 mA)
        // NOTE: set value to 0 to disable overcurrent protection
        if (radio.setCurrentLimit(CURRENT_LIMIT) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
            Serial.println(F("Selected current limit is invalid for this module!"));
            while (true);
        }

        // set LoRa preamble length to 15 symbols (accepted range is 6 - 65535)
        if (radio.setPreambleLength(LORA_PREAMBLE_LENGTH) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
            Serial.println(F("Selected preamble length is invalid for this module!"));
            while (true);
        }

        #if defined(SX127X)
        // set amplifier gain  (accepted range is 1 - 6, where 1 is maximum gain)
        // NOTE: set value to 0 to enable automatic gain control
        //       leave at 0 unless you know what you're doing
        if (radio.setGain(0) == RADIOLIB_ERR_INVALID_GAIN)
        {
            Serial.println(F("Selected gain is invalid for this module!"));
            while (true);
        }

        // set the function that will be called
        // when LoRa preamble is not detected within CAD timeout period
        // or when a packet is received
        
        radio.setPacketReceivedAction(setFlagReceive);
        radio.setPacketSentAction(setFlagSent);

        radio.setDio0Action(setFlagReceive, RISING);
        radio.setDio1Action(setFlagSent, RISING);

        // set the function that will be called
        // when LoRa preamble is detected
        // radio.setDio1Action(setFlagDetected, RISING);

        // start scanning the channel
        Serial.print(F("[SX127x] Starting to listen ... "));
        state = radio.startReceive();
        if (state == RADIOLIB_ERR_NONE)
        {
            Serial.println(F("success!"));
        }
        else
        {
                Serial.print(F("failed, code "));
                Serial.println(state);
        }        

        // enable CRC
        if (radio.setCRC(true) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
        {
            Serial.println(F("Selected CRC is invalid for this module!"));
            while (true);
        }

        #endif

        // setup for SX126x Radios
        #if defined(SX126X)
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
        Serial.print(F("[SX126x] Starting to listen ... "));
        state = radio.startReceive();
        if (state == RADIOLIB_ERR_NONE)
        {
            Serial.println(F("success!"));
        }
        else
        {
                Serial.print(F("failed, code "));
                Serial.println(state);
        }        

        // enable CRC
        if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION)
        {
            Serial.println(F("Selected CRC is invalid for this module!"));
            while (true);
        }
        #endif

        #if defined(SX126X_V3) || defined(SX1262_E290)
            // interrupt pin
            radio.setPacketReceivedAction(setFlagReceive);
            radio.setPacketSentAction(setFlagSent);

            radio.setDio1Action(setFlagSent);

            // start scanning the channel
            Serial.print(F("[SX126x] Starting to listen ... "));
            state = radio.startReceive();
            if (state == RADIOLIB_ERR_NONE)
            {
                Serial.println(F("success!"));
            }
            else
            {
                    Serial.print(F("failed, code "));
                    Serial.println(state);
            }        
    
            // enablee CRC
            if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
                Serial.println(F("Selected CRC is invalid for this module!"));
                while (true);
            }
        #endif
        
        // setup for SX126x Radios
        #if defined(BOARD_E220)

            // interrupt pin
            radio.setDio1Action(setFlag);

            // start scanning the channel
            Serial.print(F("[SX126x] Starting to listen ... "));
            state = radio.startReceive();
            if (state == RADIOLIB_ERR_NONE)
            {
                Serial.println(F("success!"));
            }
            else
            {
                    Serial.print(F("failed, code "));
                    Serial.println(state);
            }        

            // enablee CRC
            if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
                Serial.println(F("Selected CRC is invalid for this module!"));
                while (true);
            }
        
        #endif
    }

    Serial.println(F("[SX12xx] All settings successfully changed!"));

    //#endif

    if(meshcom_settings.node_call[0] == 0x00)
    {
        sprintf(meshcom_settings.node_call, "%s", (char*)"XX0XXX-00");
    }

    // Create the BLE Device & WiFiAP
    sprintf(cBLEName, "M%s-%02x%02x-%s", g_ble_dev_name, dmac[1], dmac[0], meshcom_settings.node_call);
    char cManufData[50]={0};
    sprintf(cManufData, "MCM%s-%02x%02x-%s", g_ble_dev_name,  dmac[1], dmac[0], meshcom_settings.node_call);
    
    
    const std::__cxx11::string strBLEName = cBLEName;
    const std::__cxx11::string strBLEManufData = cManufData;

    Serial.printf("[INIT]...BLE-Device started with BLE-Name <%s>\n", strBLEName.c_str());

    NimBLEDevice::init("NimBLE");

    Serial.printf("[INIT]...NIM<%s>\n", NimBLEDevice::toString().c_str());
    
    NimBLEDevice::setDeviceName(strBLEName);

    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // +9db ESP_PWR_LVL_P9

    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityPasskey(PIN);

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

    // Create the BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    pServer->advertiseOnDisconnect(false);

    // Create the BLE Service
    pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        NIMBLE_PROPERTY::WRITE  |
#if not defined(BOARD_HELTEC_V3)
                        // Require a secure connection for read and write access
                        NIMBLE_PROPERTY::WRITE_AUTHEN |  // only allow writing if paired / encrypted
                        NIMBLE_PROPERTY::WRITE_ENC |  // only allow writing if paired / encrypted
#endif
                        NIMBLE_PROPERTY::READ   |
#if not defined(BOARD_HELTEC_V3)
                        NIMBLE_PROPERTY::READ_ENC |  // only allow reading if paired / encrypted
                        NIMBLE_PROPERTY::READ_AUTHEN |
#endif
                        NIMBLE_PROPERTY::NOTIFY );

    NimBLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        NIMBLE_PROPERTY::WRITE  |
#if not defined(BOARD_HELTEC_V3)
                        // Require a secure connection for read and write access
                        NIMBLE_PROPERTY::WRITE_AUTHEN |  // only allow writing if paired / encrypted
                        NIMBLE_PROPERTY::WRITE_ENC |  // only allow writing if paired / encrypted
#endif
                        NIMBLE_PROPERTY::READ   |
#if not defined(BOARD_HELTEC_V3)
                        NIMBLE_PROPERTY::READ_ENC |  // only allow reading if paired / encrypted
                        NIMBLE_PROPERTY::READ_AUTHEN |
#endif
                        NIMBLE_PROPERTY::NOTIFY );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->reset();
    pAdvertising->setName(strBLEName);  //BLE Local Name
    pAdvertising->setManufacturerData(strBLEManufData);
    pAdvertising->addServiceUUID(SERVICE_UUID);

    if(bBLElong)
        pAdvertising->setScanResponse(true);    // true ANDROID  false IPhone ab 4.25 sollte true für beiden abgedeckt sein
    else
        pAdvertising->setScanResponse(false);    // true ANDROID  false IPhone ab 4.25 sollte true für beiden abgedeckt sein

    pAdvertising->start(0);
 
    Serial.println("[INIT]...Waiting a client connection to notify...");
    
    // reset GPS-Time parameter
    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    ///////////////////////////////////////////////////////
    // WIFI
    if(bGATEWAY || bEXTUDP || bWEBSERVER)
    {
        delay(500);

        if(startWIFI())
        {
            if(bGATEWAY || bWEBSERVER)
            {
                // get Wifi DHCP, start WIfI
                sendMeshComHeartbeat();
            }


            if(bWEBSERVER)
            {
                delay(500);
                
                startWebserver();
            }

            if(bEXTUDP)
                startExternUDP();
        }
    }
    //
    ///////////////////////////////////////////////////////

    Serial.println("==============");
    Serial.println("CLIENT STARTED");
    Serial.println("==============");

}

void esp32_write_ble(uint8_t confBuff[300], uint8_t conf_len)
{
    if(bBLEDEBUG)
        Serial.println("[LOOP] WRITE BLE");

    pTxCharacteristic->setValue(confBuff, conf_len);
    pTxCharacteristic->notify();
}

void esp32loop()
{
    bool bLoopActive = false;

    if(bLoopActive)
        Serial.println("loop 00");

    if(iReceiveTimeOutTime > 0)
    {
        // Timeout RECEIVE_TIMEOUT
        if((iReceiveTimeOutTime + RECEIVE_TIMEOUT) < millis())
        {
            iReceiveTimeOutTime=0;

            // LoRa preamble was detected
            if(bLORADEBUG)
            {
                Serial.printf("[SX12xx] Receive Timeout, starting sending again ... \n");
            }
        }
    }

    if(bLoopActive)
        Serial.println("loop 01");

    if(receiveFlag || transmittedFlag)
    {
        int state = RADIOLIB_ERR_NONE;

        // check ongoing reception
        if(receiveFlag)
        {
            // reset flags first
            bEnableInterruptReceive = false;
            receiveFlag = false;

            // DIO triggered while reception is ongoing
            // that means we got a packet

            checkRX();

            bEnableInterruptReceive = true;
        }
        else
        if(transmittedFlag)
        {
            // reset flags first
            bEnableInterruptTransmit = false;
            bEnableInterruptReceive = false;

            transmittedFlag = false;

            if (transmissionState == RADIOLIB_ERR_NONE)
            {
                // packet was successfully sent
                if(bLORADEBUG)
                    Serial.println(F("transmission finished!"));
            }
            else
            {
                if(bLORADEBUG)
                {
                    Serial.print(F("failed, code <3> "));
                    Serial.println(transmissionState);
                }
            }

            // clean up after transmission is finished
            // this will ensure transmitter is disabled,
            // RF switch is powered down etc.
            radio.finishTransmit();


            #ifndef BOARD_TLORA_OLV216
            // reset MeshCom now
            if(bSetLoRaAPRS)
            {
                lora_setchip_meshcom();
                bSetLoRaAPRS = false;
            }
            #endif

            OnTxDone();

            if(bLORADEBUG)
                Serial.print(F("[SX12xx] Starting to listen again... "));

            int state = radio.startReceive();
            if (state == RADIOLIB_ERR_NONE)
            {
                if(bLORADEBUG)
                    Serial.println(F("success!"));
            }
            else
            {
                if(bLORADEBUG)
                {
                    Serial.print(F("failed, code "));
                    Serial.println(state);
                }
            }        

            iReceiveTimeOutTime = millis(); // start to wait for next transmit

            bEnableInterruptReceive = true;
        }
    }

    // Check transmit now
    if(iReceiveTimeOutTime == 0 && !bEnableInterruptTransmit)
    {
        // channel is free
        // nothing was detected
        // do not print anything, it just spams the console
        if (iWrite != iRead)
        {
            // save transmission state between loops
            cmd_counter=0;
            tx_waiting=true;

            bEnableInterruptReceive = false;

            if(doTX())
            {
                bEnableInterruptTransmit = true;
            }
            else
            {
                if(bLORADEBUG)
                    Serial.print(F("[SX12xx] Starting to listen again... "));

                int state = radio.startReceive();
                if (state == RADIOLIB_ERR_NONE)
                {
                    if(bLORADEBUG)
                        Serial.println(F("success!"));
                }
                else
                {
                    if(bLORADEBUG)
                    {
                        Serial.print(F("failed, code "));
                        Serial.println(state);
                    }
                }        
            }
        }
    }
    
    if(bLoopActive)
        Serial.println("loop 02");

    // get RTC Now
    // RTC hat Vorrang zu Zeit via MeshCom-Server
    if(bRTCON)
    {
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
    {
        MyClock.CheckEvent();
        
        meshcom_settings.node_date_year = MyClock.Year();
        meshcom_settings.node_date_month = MyClock.Month();
        meshcom_settings.node_date_day = MyClock.Day();

        meshcom_settings.node_date_hour = MyClock.Hour();
        meshcom_settings.node_date_minute = MyClock.Minute();
        meshcom_settings.node_date_second = MyClock.Second();
    }

    // SOFTSER
    #if defined(ENABLE_SOFTSER)
        if(bSOFTSERON)
        {
            if (bSOFTSER_APP || ((softser_refresh_timer + ((SOFTSER_REFRESH_INTERVAL * 1000) - 3000)) < millis()))
            {
                // start SOFTSER APP
                loopSOFTSER(SOFTSER_APP_ID, 0);

                softser_refresh_timer = millis();

                bSOFTSER_APP = false;
            }
            else
            {
                appSOFTSER(SOFTSER_APP_ID);
            }
        }
    #endif

    if(bLoopActive)
        Serial.printf("[LOOP] 1\n");

    checkButtonState();

    // BLE
    if (deviceConnected)
    {
    	g_ble_uart_is_connected = true;
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(2000); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        oldDeviceConnected = deviceConnected;

        g_ble_uart_is_connected = false;
        isPhoneReady = 0;
    }

    if(bLoopActive)
        Serial.printf("[LOOP] 1-1\n");

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

    if(bLoopActive)
        Serial.printf("[LOOP] 1-2\n");

    // check if message from phone to send
    if(hasMsgFromPhone)
    {
        if(bBLEDEBUG)
            Serial.printf("[LOOP] hasMsgFromPhone\n");
        sendMessage(textbuff_phone, txt_msg_len_phone);

        hasMsgFromPhone = false;
    }

    if(bLoopActive)
        Serial.printf("[LOOP] 1-3\n");

    checkButtonState();

    if (isPhoneReady == 1)
    {
        if (config_to_phone_prepare)
        {
            for(int config_cmds_index=0; config_cmds_index < json_configs_cnt; config_cmds_index++)
            {
                sendMessage((char*)config_cmds[config_cmds_index], strlen(config_cmds[config_cmds_index]));
            }

            sendMheard();

            config_to_phone_prepare_timer = millis();
            
            config_to_phone_prepare = false;
        }
        else
        {
            // wait after BLE Connect 3 sec.
            if(millis() < config_to_phone_prepare_timer + 3000)
                iPhoneState = 0;

            if (iPhoneState > 3)   // only every 3 times of mainloop send to phone
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
                iPhoneState++;
        }
    }

    if(bLoopActive)
        Serial.printf("[LOOP] 2\n");

    // gps refresh every 10 sec
    if ((gps_refresh_timer + (GPS_REFRESH_INTERVAL * 1000)) < millis())
    {
        // get i/o state
        if(loopMCP23017())
        {
        }

        #ifdef ENABLE_GPS
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
        #endif

        gps_refresh_timer = millis();
    }

    if(bLoopActive)
        Serial.printf("[LOOP] 3\n");

    checkButtonState();

    // posinfo_interval in Seconds
    if (((posinfo_timer + (posinfo_interval * 1000)) < millis()) || (millis() > 100000 && millis() < 130000 && bPosFirst) || posinfo_shot)
    {
        bPosFirst = false;
        posinfo_shot=false;
        
        if(bSOFTSERON && SOFTSER_APP_ID == 1)
        {
            // no normal positons sent
        }
        else
        {
            sendPosition(posinfo_interval, meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, meshcom_settings.node_press, meshcom_settings.node_hum, meshcom_settings.node_temp, meshcom_settings.node_temp2, meshcom_settings.node_gas_res, meshcom_settings.node_co2, meshcom_settings.node_press_alt, meshcom_settings.node_press_asl);

            posinfo_last_lat=posinfo_lat;
            posinfo_last_lon=posinfo_lon;
            posinfo_last_direction=posinfo_direction;

            if(pos_shot)
            {
                commandAction((char*)"--pos", true);
                pos_shot = false;
            }
        }

        posinfo_timer = millis();
    }

    mainStartTimeLoop();

    if(DisplayOffWait > 0)
    {
        if (millis() > DisplayOffWait)
        {
            DisplayOffWait = 0;
            bDisplayOff=true;
        }
    }

    if(bLoopActive)
        Serial.printf("[LOOP] 4\n");

    // rebootAuto
    if(rebootAuto > 0)
    {
        if (millis() > rebootAuto)
        {
            rebootAuto = 0;

            #ifdef ESP32
                ESP.restart();
            #endif
        }
    }

    checkButtonState();

    if(bLoopActive)
        Serial.printf("[LOOP] 5\n");

    checkSerialCommand();

    if(bLoopActive)
        Serial.printf("[LOOP] 6\n");

    if(BattTimeWait == 0)
        BattTimeWait = millis() - 10000;


    //Serial.printf("BattTimeWait:%i millis():%i tx:%i rx:%i\n", BattTimeWait, millis(), tx_is_active, is_receiving);

    if ((BattTimeWait + 10000) < millis())  // 10 sec
    {
        if (tx_is_active == false && is_receiving == false)
        {
            #if defined(MODUL_FW_TBEAM)
                if(PMU != NULL)
                {
                    global_batt = (float)PMU->getBattVoltage();
                    global_proz = (int)PMU->getBatteryPercent();

                    // no BATT
                    if(global_proz <= 0)
                    {
                        global_batt = (float)PMU->getVbusVoltage();
                        global_proz=100.0;
                    }
                }
                else
                {
                    global_batt = 0;
                    global_proz = 0;
                }

                if(bDEBUG)
                    Serial.printf("PMU.volt %.1f PMU.proz %i\n", global_batt, global_proz);
            #else
                global_batt = read_batt();
                global_proz = mv_to_percent(global_batt);
                if(bDEBUG)
                    Serial.printf("volt %.1f proz %i\n", global_batt, global_proz);
            #endif

            BattTimeWait = millis();
        }
    }

    checkButtonState();

//#ifndef BOARD_TLORA_OLV216
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
                    commandAction((char*)"--wx", true);
                    wx_shot = false;
                }
            }
        }
    }
//#endif

    if(bLoopActive)
        Serial.printf("[LOOP] 7\n");

    // read BMP Sensor
    #if defined(ENABLE_BMX280)
    if(bBMPON || bBMEON)
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
                        commandAction((char*)"--wx", true);
                        wx_shot = false;
                    }
                }

            BMXTimeWait = millis(); // wait for next messurement
        }
    }
    #endif

    if(bLoopActive)
        Serial.printf("[LOOP] 8\n");

    checkButtonState();

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
                    commandAction((char*)"--wx", true);
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

        if ((INA226TimeWait + 15000) < millis())   // 15 sec
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

    if(bLoopActive)
        Serial.printf("[LOOP] 9\n");

    // read every n seconds the bme680 sensor calculated from millis()
    #if defined(ENABLE_BMX680)
    if(bBME680ON && bme680_found)
    {
        if ((bme680_timer + 60000) < millis() || delay_bme680 <= 0)
        {
            #if defined(ENABLE_BMX280)
                
                if (delay_bme680 <= 0)
                {
                    getBME680();

                }

                if(wx_shot)
                {
                    commandAction((char*)"--wx", true);
                    wx_shot = false;
                }

                // calculate delay
                delay_bme680 = bme680_get_endTime() - millis();
            #endif

            bme680_timer = millis();
        }
    }
    #endif
    
    if(bLoopActive)
        Serial.printf("[LOOP] A\n");

    checkButtonState();

    ////////////////////////////////////////////////
    // WIFI Gateway functions
    if(bGATEWAY)
    {
        getMeshComUDP();

        sendMeshComUDP();

        // heartbeat
        if ((hb_timer + (HEARTBEAT_INTERVAL * 1000)) < millis())
        {
            //DEBUG_MSG("UDP", "Sending Heartbeat");
            sendMeshComHeartbeat();

            hb_timer = millis();
        }

        meshcom_settings.node_last_upd_timer = hb_timer;

    }

    checkButtonState();

    if(bEXTUDP)
    {
        getExternUDP();
    }

    checkButtonState();

    if(bWEBSERVER)
    {
        if (web_timer == 0 || ((web_timer + (HEARTBEAT_INTERVAL * 1000 * 60)) < millis()))   // HEARTBEAT_INTERVAL to minutes
        {
            // restart WEB-Client
            stopWebserver();

            web_timer = millis();

            if(!meshcom_settings.node_hasIPaddress)
                startWIFI();
        }

        startWebserver();

        loopWebserver();
    }

    //
    ////////////////////////////////////////////////

    if(bLoopActive)
            Serial.printf("[LOOP] B\n");

    delay(100);

    yield();
}



int checkRX(void)
{
    // you can receive data as an Arduino String
    // NOTE: receive() is a blocking method!
    //       See example ReceiveInterrupt for details
    //       on non-blocking reception method.

    if(is_receiving)    // receive in action
        return -1;

    is_receiving=true;

    uint8_t payload[UDP_TX_BUF_SIZE+10];

    String str;
    
    size_t ibytes = UDP_TX_BUF_SIZE;

    int state = radio.readData(payload, ibytes);

    if (state == RADIOLIB_ERR_LORA_HEADER_DAMAGED || state == RADIOLIB_ERR_NONE)
    {
        if(bLORADEBUG)
        {
            // packet was successfully received
            Serial.print(F("[SX12xx] Received packet: "));

            // print RSSI (Received Signal Strength Indicator)
            Serial.print(F("RSSI:\t\t"));
            Serial.print(radio.getRSSI());
            Serial.print(F(" dBm / "));

            // print SNR (Signal-to-Noise Ratio)
            Serial.print(F("SNR:\t\t"));
            Serial.print(radio.getSNR());
            Serial.print(F(" dB / "));

            // print frequency error
            Serial.print(F("Frequency error:\t"));
            Serial.print(radio.getFrequencyError());
            Serial.println(F(" Hz"));
        }

        OnRxDone(payload, (uint16_t)ibytes, (int16_t)radio.getRSSI(), (int8_t)radio.getSNR());
    }
    else
    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        // packet was received, but is malformed
        if(bLORADEBUG)
            Serial.println(F("[SX12xx] CRC error!"));
    }
    else
    {
        // some other error occurred
        Serial.print(F("[SX12xx] Failed, code <2>"));
        Serial.println(state);
    }

    is_receiving=false;

    return state;
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

        if(strText.startsWith(":") || strText.startsWith("-") || strText.startsWith("{"))
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
                    }
                }

                if(strText.startsWith(":"))
                    sendMessage(msg_buffer, inext);

                if(strText.startsWith("-"))
                    commandAction(msg_buffer, false);

                if(strText.startsWith("{"))
                    commandAction(msg_buffer, false);

                strText="";
            }
        }
        else
        {
            if(!strText.startsWith("\n") && !strText.startsWith("\r"))
            {
                printf("MSG:%02X", rd);
                printf("..not sent\n");
            }
            strText="";
        }
    }
}
