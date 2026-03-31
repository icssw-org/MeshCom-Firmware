
// (C) 2023 OE1KBC Kurt Baumann, OE1KFR Rainer 
// (C) 2016, 2017, 2018, 2018, 2019, 2020 OE1KBC Kurt Baumann
//
// 20230326: Version 4.00: START
/**
 *  @author      Ralph Weich (DD5RW)
 *  @date        2025-12-03
 */
#include <Arduino.h>
#include <atomic>
#include <configuration.h>
#include <RadioLib.h>

#include <Wire.h>               
#include <SPI.h>
#include <WiFi.h>

#ifdef BOARD_T_ETH_ELITE
#include "esp32_eth.h"
SPIClass ethSPI(FSPI);
#endif

#include "esp32_pmu.h"
#include "esp32_flash.h"
#include <esp_adc_cal.h>

#if not defined(BOARD_T_DECK_PRO)
//====== Timer for periodical events u.a.
#include "Timeout.h"
Timeout timerSerial;
#endif

#ifdef HAS_SDCARD
#include <SD.h>
#endif

#if defined(ARDUINO_ARCH_ESP32)
#include <FS.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
#include "hal/gpio_hal.h"
#endif
#include "driver/gpio.h"
#endif //ARDUINO_ARCH_ESP32

// Kompatibilitaets-Macro: DIO1-Pin hat je nach Board verschiedene Namen
#ifndef LORA_DIO1
  #if defined(E22_DIO1)
    #define LORA_DIO1 E22_DIO1
  #elif defined(RADIO_DIO1_PIN)
    #define LORA_DIO1 RADIO_DIO1_PIN
  #elif defined(PIN_LORA_DIO_1)
    #define LORA_DIO1 PIN_LORA_DIO_1
  #else
    #warning "LORA_DIO1 not defined -- safety net digitalRead() disabled"
  #endif
#endif

#if defined(ENABLE_GPS)
    #include "gps_functions.h"
    extern GPSData gpsData;
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
#endif
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
#include <extudp_functions.h>
#include <web_functions/web_functions.h>
#include <mheard_functions.h>
#include <time_functions.h>
#include <clock.h>
#include <onewire_functions.h>
#include <onebutton_functions.h>
#include <adc_functions.h>
#include <lora_setchip.h>
#include "esp32_functions.h"
#include "tft_display_functions.h"

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

#ifdef BOARD_LED
bool bLED = true;
#endif

#if defined(ENABLE_AUDIO)
#include "esp32_audio.h"
#endif

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/tdeck_main.h>
#include <t-deck/tdeck_extern.h>
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

bool bPosFirst = true;
bool bHeyFirst = true;
bool bTeleFirst = true;

bool bAllStarted = true;

#ifdef BOARD_T_ETH_ELITE
EspETH neth;
bool bETHERNET = false;
#endif

String strTime;
String strDate;
String strText = "";
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

// Textmessage buffer from phone, hasMsgFromPhone flag indicates new message
extern char textbuff_phone [MAX_MSG_LEN_PHONE];
extern uint8_t txt_msg_len_phone;
extern unsigned long last_upd_timer;
extern bool hb_warn_logged;

// FreeRTOS Queue for BLE data from NimBLE task to Main Loop
#include "freertos/queue.h"

struct BleQueueItem {
    uint8_t data[MAX_MSG_LEN_PHONE];
    size_t length;
};

static QueueHandle_t bleQueue = NULL;

NimBLEServer *pServer = NULL;
NimBLECharacteristic* pTxCharacteristic;
NimBLEService *pService;
bool deviceConnected = false;
bool oldDeviceConnected = false;

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


class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override 
    {
        deviceConnected = true;
        config_to_phone_prepare = true;
        conffin_sent = false;

        Serial.printf("BLE Connected with: %s\n", connInfo.getAddress().toString().c_str());
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
    };

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override 
    {
        deviceConnected = false;
        // print the reason for the disconnection in hex
        // https://github.com/apache/mynewt-nimble/blob/master/docs/ble_hs/ble_hs_return_codes.rst
        Serial.printf("BLE disconnected. Reason: 0x%04x\n", reason);
        //NimBLEDevice::startAdvertising();
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
        Serial.printf("Client connected: %s (encrypted: %s)\n",
            connInfo.getAddress().toString().c_str(),
            connInfo.isEncrypted() ? "yes" : "no");
    }
} serverCallbacks;


/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    /*void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        Serial.printf("%s : onRead(), value: %s\n",
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
    // !!!!! es wird nur ein SX1261 erkannt !!!!!
    SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);    
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

// flag to indicate that a packet was detected or CAD timed out
volatile bool scanFlag = false;

// flag to indicate one second 
unsigned long retransmit_timer = 0;

// blink frequency for board_led
unsigned long led_timer = 0;

// flag to update NTP Time
unsigned long updateTimeClient = 0;

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
void checkSerialCommand(void);

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
    if (millis() > check_temperature) {
        analogSetAttenuation(ADC_11db); // bis <2,2V
        float voltage = analogReadMilliVolts(NTC_PIN);
        uint16_t raw = analogReadRaw(NTC_PIN);
        // die fixen 3.3 V stimmen nicht, hier sollte diese Spannung auch gemessen werden (Jumper)
        float resistance = SERIES_RESISTOR * ((3.3 / voltage *1000.0) - 1); // Calculate the resistance of NTC

        // Calculate temperature using the Steinhart-Hart equation
        temperature = (1.0 / (log(resistance / ROOM_TEMP_RESISTANCE) / B_COEFFICIENT + 1.0 / ROOM_TEMP)) - 273.15;

        if(bWXDEBUG)
            Serial.printf("NTC-Temp: %.3f_°C %u_raw %.3f_mV %.2f_Ohm\n", temperature, raw, voltage, resistance);

        check_temperature  = millis() + 1000;
    }
    return temperature;
}
#endif
//=======================================================================================


void esp32setup()
{
    ///< Initialize T5-EPAPER GUI
    ///< delay for ESP32-S3 nativ USB [OE3WAS]
    ///< um Terminal verbinden zu können
    #if not defined(BOARD_T_DECK_PRO)
    timerSerial.start(2000);  //timeout falls keine USB verbunden ist
    #endif
    Serial.begin(MONITOR_SPEED);
    
    #if defined(BOARD_T_DECK_PRO)
    while (!Serial) // && !timerSerial.time_over());
    #else
    while (!Serial && !timerSerial.time_over());
    #endif
    if (Serial) { for (int i=0;i<5;i++) { Serial.println("."); delay(1000); } } //delay for Terminal connect

    #if defined BOARD_T5_EPAPER
        if (psramInit()) {
            Serial.println("\nThe PSRAM is correctly initialized");
        } else {
            Serial.println("\nPSRAM does not work");
        }
    #endif

    // Heltec V2: Enable Vext (GPIO 21) Heltec V3: Enable Vext (GPIO 36) to power OLED and reset display
    #if defined(BOARD_HELTEC) || defined(BOARD_HELTEC_V3)
        Serial.println(F("[INIT]...Enabling Vext for OLED power"));
        pinMode(Vext, OUTPUT);
        digitalWrite(Vext, LOW);   // Vext ON (active low)
        delay(50);

        // Reset OLED 
        Serial.println(F("[INIT]...Resetting OLED display"));
        pinMode(RST_OLED, OUTPUT);
        digitalWrite(RST_OLED, LOW);   // Reset active
        delay(50);
        digitalWrite(RST_OLED, HIGH);  // Reset release
        delay(50);
    #endif
    
    #if not defined(BOARD_T_DECK_PRO)
        #ifndef BOARD_T5_EPAPER
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

        #if defined(HAS_GPS) && defined(GPS_EN_PIN)
            pinMode(GPS_EN_PIN, OUTPUT);
            digitalWrite(GPS_EN_PIN, HIGH);
        #endif /*GPS_EN_PIN*/

        #ifdef GPS_PPS_PIN
            pinMode(GPS_PPS_PIN, INPUT);
        #endif

        #ifdef FAN_CTRL
            pinMode(FAN_CTRL, OUTPUT);
            digitalWrite(FAN_CTRL,HIGH);
        #endif

    #endif
    //======================================================

    Serial.println("");
    Serial.println("");
    Serial.println("============");
    Serial.println("CLIENT SETUP");
    Serial.println("============");

    Serial.printf("%s;[HEAP];%d;%d;%d;(init)\n", getTimeString().c_str(),
        ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
    Serial.printf("%s;[PSRM];%d\n", getTimeString().c_str(), ESP.getFreePsram());
    
    check_efuse();

    // init nach Reboot
    init_loop_function();

    // Initialize mheard list
    initMheard();

	// Get LoRa parameter
	init_flash();

    bool bClear = false;
    if(meshcom_settings.node_cleanflash == 1)
        bClear = true;

    if(meshcom_settings.node_fversion != FLASH_VERSION || bClear)
    {
        Serial.printf("[INIT]...FLASH cleared new version %i\n", FLASH_VERSION);

        initTimePersistence();

        clear_flash();
    }
    else
    {
        Serial.printf("[INIT]...FLASH version %i\n", meshcom_settings.node_fversion);
    }

    if(bClear)
        init_flash();

    meshcom_settings.node_fversion = FLASH_VERSION;
    meshcom_settings.node_mversion = MODUL_HARDWARE;
    meshcom_settings.node_cleanflash = 0;
    snprintf(meshcom_settings.node_fwversion, sizeof(meshcom_settings.node_fwversion), "%-4.4s%-1.1s", SOURCE_VERSION, SOURCE_VERSION_SUB);

    save_settings();

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
    bDisplayCont = meshcom_settings.node_sset & 0x4000;

    bONEWIRE =  meshcom_settings.node_sset2 & 0x0001;
    bLPS33 =  meshcom_settings.node_sset2 & 0x0002;
    bBME680ON =  meshcom_settings.node_sset2 & 0x0004;
    bMCU811ON =  meshcom_settings.node_sset2 & 0x0008;
    bGPSDEBUG = meshcom_settings.node_sset2 & 0x0010;
    bMESH = !(meshcom_settings.node_sset2 & 0x0020);
    bWEBSERVER = meshcom_settings.node_sset2 & 0x0040;
    bWIFIAP = meshcom_settings.node_sset2 & 0x0080;
    bGATEWAY_NOPOS =  meshcom_settings.node_sset2 & 0x0100;
    bSMALLDISPLAY =  false;
    bSOFTSERREAD = meshcom_settings.node_sset2 & 0x0200;
    bSOFTSERON =  meshcom_settings.node_sset2 & 0x0400;
    bBOOSTEDGAIN =  meshcom_settings.node_sset2 & 0x0800;

    // nicht mehr notwendig bMHONLY =  bMHONLY =  meshcom_settings.node_sset3 & 0x0001;
    bNoMSGtoALL =  meshcom_settings.node_sset3 & 0x0002;
    bBLEDEBUG = meshcom_settings.node_sset3 & 0x0004;
    bAnalogCheck = meshcom_settings.node_sset3 & 0x0008;
    bBMP3ON = meshcom_settings.node_sset3 & 0x0010;
    bAHT20ON = meshcom_settings.node_sset3 & 0x0020;
    bAnalogFilter = meshcom_settings.node_sset3 & 0x0040;
    bUSER_BOARD_LED = meshcom_settings.node_sset3 & 0x0080;
    
    bSOFTSERDEBUG = meshcom_settings.node_sset3 & 0x0100;
    bWXDEBUG = meshcom_settings.node_sset3 & 0x0200;
    bSHT21ON = meshcom_settings.node_sset3 & 0x0400;
    bINA226ON = meshcom_settings.node_sset3 & 0x0800;

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

    #if defined(BOARD_T5_EPAPER)
        idf_setup();
    #endif

    // if Node not set --> WifiAP Mode on
    if(memcmp(meshcom_settings.node_call, "XX0XXX", 6) == 0 || meshcom_settings.node_call[0] == 0x00 || memcmp(meshcom_settings.node_call, "none", 4) == 0)
    {
        bWIFIAP = true;
        Serial.println("[INIT]...WIFIAP starting...");
    }

    if(bWIFIAP)
    {
        bWEBSERVER = true;
        Serial.println("[INIT]...WEBServer starting...");
    }

    #if defined(BOARD_T_DECK_PRO)
        bWEBSERVER=false;
        bDEBUG=true;
        bGPSON=false;
    #endif


    if(meshcom_settings.node_gwsrv[0] == 0x00)
        sprintf(meshcom_settings.node_gwsrv, "%s", "OE");

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

    meshcom_settings.max_hop_text = MAX_HOP_TEXT_DEFAULT;
    meshcom_settings.max_hop_pos = MAX_HOP_POS_DEFAULT;

    #if defined(BOARD_E22_S3) || defined(BOARD_TBEAM_1W)
        fBattFaktor = ADC_MULTIPLIER;   // default
        if(meshcom_settings.node_analog_batt_faktor > 0.0)
            fBattFaktor = meshcom_settings.node_analog_batt_faktor;
    #endif

    // Initialize battery reading
    #if not defined (BOARD_T_DECK_PRO)
	init_batt();
    #endif

    #ifdef VEXT_CTRL
        pinMode(VEXT_CTRL, OUTPUT);
    #endif

    #ifdef ADC_CTRL
        pinMode(ADC_CTRL, OUTPUT);
    #endif
        
    #if defined(BOARD_TRACKER)
        digitalWrite(VEXT_CTRL, HIGH);   // HWT needs this for GPS and TFT Screen
        digitalWrite(ADC_CTRL, HIGH);
    #endif

    #ifdef LED_PIN
        pixels.begin();
        Serial.println("[INIT]...NEOPIXEL set");
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

    if(memcmp(meshcom_settings.node_ssid, "XX0XXX", 6) == 0)
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

    #ifdef BOARD_STICK_V3
        SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
    #endif

    #ifdef BOARD_HELTEC_V4
        SPI.begin(RF95_SCK, RF95_MISO, RF95_MOSI, RF95_NSS);
        initPAControl();
    #endif

    #if defined(ENABLE_GPS)
        // TBEAM variants
        #if defined(XPOWERS_CHIP_AXP192) || defined(XPOWERS_CHIP_AXP2101)
            setupPMU();
        #endif
    #endif
    
    #if defined(ENABLE_GPS)
        GPS_Init();
    #else

    #if !defined(BOARD_T_DECK_PRO) && !defined(BOARD_T5_EPAPER)
        setupPMU();
    #endif

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

    Serial.printf("[INIT].._GW_ID: %08X\n", _GW_ID);

 
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
    
    #if defined (BOARD_T5_EPAPER)
    //
    #elif defined (BOARD_T_DECK_PRO)
    //
    #elif HAS_TFT
        initTFT();
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
    #elif HAS_TFT
        char cvers[22];
        sprintf(cvers, "  FW %s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB, getCountry(meshcom_settings.node_country).c_str());
        String  version = cvers;
        displayTFT(" MeshCom 4.0 ", version, "  @BY ICSSW.ORG", "  OE1KBC, OE1KFR",  "  ...starting now", 5000);
    #else
        startDisplay((char*)"...starting now", (char*)"@BY ICSSW.ORG", (char*)"OE1KBC, OE1KFR");
    #endif

    //LORA CHIP present
    bRadio = false;

    //TEST #ifndef BOARD_E290

    #ifdef SX127X
    Serial.print(F("[LoRa]...SX1276 Chip"));
    #endif
    
    #ifdef BOARD_E220
    Serial.print(F("[LoRa]...LLCC68 chip"));
    #endif

    #ifdef SX1262X
    Serial.print(F("[LoRa]...SX1262 chip"));
    #endif
    
    #ifdef SX126X
    Serial.print(F("[LoRa]...SX1268 chip"));
    #endif
    
    #ifdef SX1262_V3
    Serial.print(F("[LoRa]...SX1262 V3 chip"));
    #endif
    
    #if defined(SX1262_E22) || defined(USING_SX1262)
    Serial.print(F("[LoRa]...SX1262 V3 chip"));
    #endif

    #ifdef SX1268_E22
    Serial.print(F("[LoRa]...SX1268 V3 chip"));
    #endif

    #ifdef SX1262_E290
    Serial.print(F("[LoRa]...SX1262 E290 chip"));
    #endif

    #ifdef SX1262_V4
    Serial.print(F("[LoRa]...SX1262 V4 chip (with PA)"));
    #endif

    #if defined (BOARD_TRACKER) || defined(BOARD_TBEAM_1W)
        SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    #endif

    #if defined(BOARD_T5_EPAPER)
    // extra source
    // #elif defined(BOARD_T_DECK_PRO)
    // extra source
    #elif defined(BOARD_E220)
        Serial.print(F(" Initializing ... "));
        int state = radio.begin(434.0F, 125.0F, 9, 7, SYNC_WORD_SX127x, 10, LORA_PREAMBLE_LENGTH, /*float tcxoVoltage = 0*/ 1.6F, /*bool useRegulatorLDO = false*/ false);
    #elif defined(BOARD_T_ETH_ELITE)
    Serial.print(F(" Initializing ... "));
    int state = radio.begin(433.175F);
    radio.setDio2AsRfSwitch(true);
    #else
        Serial.print(F(" Initializing ... "));

        #if defined(BOARD_TBEAM_1W)
        #ifdef RADIO_LDO_EN
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

        int state = radio.begin(433.175F);

    #endif
    
    #if defined(BOARD_T5_EPAPER)
    // extra source
    // #elif defined(BOARD_T_DECK_PRO)
    // extra source
    #else
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success"));
        bRadio=true;

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_addMessage(true);
        #endif
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        bRadio=false;

        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_addMessage(false);
        #endif
    }
    #endif

    #if defined(BOARD_E220)
        bRadio = false; // no detailed setting
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
        // set boosted gain
        #if defined(SX1262_V3) || defined(SX126x_V3) || defined(SX1262_E290) || defined(SX1262X) || defined(SX126X) || defined(USING_SX1262) || defined(SX1262_V4)
        Serial.printf("[LoRa]...RX_BOOSTED_GAIN: %d\n", (meshcom_settings.node_sset2 &  0x0800) == 0x0800);
        if (radio.setRxBoostedGainMode(meshcom_settings.node_sset2 & 0x0800)  != RADIOLIB_ERR_NONE ) {
            Serial.println(F("Boosted Gain is not available for this module!"));
        }
        #endif

        // set carrier frequency
        Serial.printf("[LoRa]...RF_FREQUENCY: %.3f MHz\n", meshcom_settings.node_freq);
        if (radio.setFrequency(meshcom_settings.node_freq) == RADIOLIB_ERR_INVALID_FREQUENCY) {
            Serial.println(F("Selected frequency is invalid for this module!"));
        }

        // set bandwidth 
        Serial.printf("[LoRa]...RF_BANDWIDTH: %.0f kHz\n", meshcom_settings.node_bw);
        if (radio.setBandwidth(meshcom_settings.node_bw) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
            Serial.println(F("Selected bandwidth is invalid for this module!"));
        }

        // set spreading factor 
        Serial.printf("[LoRa]...RF_SF: %i\n", meshcom_settings.node_sf);
        if (radio.setSpreadingFactor(meshcom_settings.node_sf) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
            Serial.println(F("Selected spreading factor is invalid for this module!"));
        }

        // set coding rate 
        Serial.printf("[LoRa]...RF_CR: 4/%i\n", meshcom_settings.node_cr);
        if (radio.setCodingRate(meshcom_settings.node_cr) == RADIOLIB_ERR_INVALID_CODING_RATE) {
            Serial.println(F("Selected coding rate is invalid for this module!"));
        }

        // set LoRa sync word 
        // NOTE: value 0x34 is reserved for LoRaWAN networks and should not be used
        if (radio.setSyncWord(SYNC_WORD_SX127x) != RADIOLIB_ERR_NONE) {
            Serial.println(F("Unable to set sync word!"));
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

        meshcom_settings.node_power = tx_power;  // [OE3WAS] für den Ring der Sicherheit ;-) ??
        save_settings();
        Serial.printf("[LoRa]...RF_POWER: %d dBm\n", tx_power);

        if (radio.setOutputPower(tx_power) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
            Serial.println(F("Selected output power is invalid for this module!"));
            while (true);
        }

        // set over current protection limit (accepted range is 0 - 140 mA)
        // NOTE: set value to 0 to disable overcurrent protection
        if (radio.setCurrentLimit(CURRENT_LIMIT) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
            Serial.println(F("Selected current limit is invalid for this module!"));
            while (true);
        }

        // set LoRa preamble length to 15 symbols (accepted range is 6 - 65535)
        Serial.printf("[LoRa]...PREAMBLE: %i symbols\n", meshcom_settings.node_preamplebits);
        
        if (radio.setPreambleLength(meshcom_settings.node_preamplebits) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
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
        Serial.print(F("[LoRa]...Starting to listen ... "));
        state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        if (state == RADIOLIB_ERR_NONE)
        {
            Serial.println(F("success"));
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
        Serial.print(F("[LoRa]...Starting to listen ... "));
        state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
        if (state == RADIOLIB_ERR_NONE)
        {
            Serial.println(F("success"));
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

        #if defined(SX1262_V3) || defined(SX1262_E290) || defined(SX1262_V4)

            // set Receive Interupt
            bEnableInterruptReceive = true; //KBC 0801
            radio.setPacketReceivedAction(setFlagReceive); //KBC 0801
            
            //KBC 0801 radio.setPacketSentAction(setFlagSent);

            #if defined (BOARD_TRACKER)
                radio.setDio1Action(setFlagSent);
            #endif

            // start scanning the channel
            Serial.print(F("[LoRa]...Starting to listen ... "));
            state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
            if (state == RADIOLIB_ERR_NONE)
            {
                Serial.println(F("success"));
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
        
        // setup for E220 Radios
        #if defined(BOARD_E220)

            // interrupt pin
            radio.setDio1Action(setFlag);

            // start scanning the channel
            Serial.print(F("[E220] Starting to listen ... "));
            state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
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
            if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
                Serial.println(F("Selected CRC is invalid for this module!"));
                while (true);
            }
        
        #endif
    }

    Serial.println(F("[LoRa]...All settings successfully changed"));
    #endif

    //#endif

    if(meshcom_settings.node_call[0] == 0x00)
    {
        sprintf(meshcom_settings.node_call, "%s", (char*)"XX0XXX-00");
    }

    // Create the BLE Device & WiFiAP
    sprintf(cBLEName, "M%s-%02x%02x-%s", g_ble_dev_name, dmac[1], dmac[0], meshcom_settings.node_call);
    char cManufData[60]={0};
    sprintf(cManufData, "MCM%s-%02x%02x-%s", g_ble_dev_name,  dmac[1], dmac[0], meshcom_settings.node_call);
    
    
    const std::__cxx11::string strBLEName = cBLEName;
    const std::__cxx11::string strBLEManufData = cManufData;

    Serial.printf("[BLE ]...Device started with BLE-Name <%s>\n", strBLEName.c_str());

    bleQueue = xQueueCreate(5, sizeof(BleQueueItem));

    NimBLEDevice::init(strBLEName);

    Serial.printf("[BLE ]...Device-Address <%s>\n", NimBLEDevice::toString().c_str());
    
    //NimBLEDevice::setDeviceName(strBLEName);

    NimBLEDevice::setPower(9); // +9dbm
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityAuth(false, false, false);
    
    /*if(meshcom_settings.bt_code >= 100000 && meshcom_settings.bt_code <= 999999)
        NimBLEDevice::setSecurityPasskey(meshcom_settings.bt_code);
    else
        NimBLEDevice::setSecurityPasskey(PIN);*/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

    // Create the BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);
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
    
    pAdvertising->start();
 
    Serial.println("[BLE ]...Waiting a client connection to notify...");
    
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

    Serial.println("==============");
    Serial.println("CLIENT STARTED");
    Serial.println("==============");
    delay(500);

    ///////////////////////////////////////////////////////
    // WIFI
    if(bGATEWAY || bEXTUDP || bWEBSERVER)
    {
        bAllStarted=false;

        if(!startNetwork())
        {
            Serial.println("[WIFI]...no connection");
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

    #ifdef BOARD_LED
        pinMode(BOARD_LED, OUTPUT);
    #endif

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        tdeck_clear_text_ta();
    #endif
}

// BLE TX Function -> Node to Client
void esp32_write_ble(uint8_t confBuff[300], uint8_t conf_len)
{
    if(bBLEDEBUG)
        Serial.printf("[LOOP] <%lu> WRITE BLE\n", millis());

    pTxCharacteristic->setValue(confBuff, conf_len);
    pTxCharacteristic->notify();
}



void esp32loop()
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

    #if not defined(BOARD_T_DECK_PRO)
    btn.tick();
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
                if(pixels_delay + DELAYVAL < millis())
                {
                    pixels_delay = 0;
                    bLED_CLEAR=true;
                    bLED_DELAY=false;
                }
            }
        }
    #endif

    #ifdef BOARD_LED
        if(bUSER_BOARD_LED)
        {
            if ((led_timer + 1000) < millis())   // repeat 1 seconds
            {
                #ifdef LED_PIN
                    if(pixels_delay == 0 && !bLED_CLEAR)
                    {
                        if(!bLED_RED && !bLED_GREEN && !bLED_ORANGE && !bLED_BLUE)
                            bLED_WEISS = true;
                    }
                #endif

                if(bLED)
                    digitalWrite(BOARD_LED, HIGH);
                else
                    digitalWrite(BOARD_LED, LOW);
                bLED = !bLED;

                led_timer = millis();
            }

        }
    #endif

    // LoRa-Chip found
    #if defined(BOARD_T5_EPAPER)
        idf_loop();
    // #elif defined(BOARD_T_DECK_PRO)
    // extra source
    #else

    if(bRadio)
    {
        if(bLORADEBUG && receiveFlag)
            Serial.println("receiveflag");

        if(bLORADEBUG && transmittedFlag)
            Serial.println("transmittedFlag");

        if(inoReceiveTimeOutTime > 0)
        {
            // Timeout RECEIVE_TIMEOUT
            if((inoReceiveTimeOutTime + (60 * 6 * 1000)) < millis())  // 6 Minuten
            {
                inoReceiveTimeOutTime=0;

                if(bLORADEBUG)
                {
                    Serial.print(getTimeString());
                    Serial.println(" [LoRa]...Receive Timeout > 6.5 min. just for info");
                }

            }
        }

        // Retransmission status must tick on ALL nodes (including gateways).
        // Without this, gateway text messages stay stuck at RING_STATUS_SENT
        // forever if no echo is received via LoRa (RING_ZOMBIE).
        if ((retransmit_timer + (1000 * 2)) < millis())
        {
            updateRetransmissionStatus();

            retransmit_timer = millis();
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
                int w = iWrite;
                int r = iRead;
                int queued = (w >= r) ? (w - r) : (MAX_RING - r + w);
                int dedup_used = 0;
                for(int i = 0; i < MAX_DEDUP_RING; i++)
                {
                    if(ringBufferLoraRX[i][0] != 0 || ringBufferLoraRX[i][1] != 0 ||
                       ringBufferLoraRX[i][2] != 0 || ringBufferLoraRX[i][3] != 0)
                        dedup_used++;
                }
                Serial.printf("[MC-DBG] RING_STATUS queued=%d pending=%d retrying=%d "
                              "done=%d iW=%d iR=%d dedup=%d/%d\n",
                              queued, pending, retrying, done, w, r,
                              dedup_used, MAX_DEDUP_RING);
            }
        }

        // Deferred display update from OnRxDone (avoid I2C inside radio callback)
        // RACE-01 fix: snapshot under spinlock, display call outside
        {
            portENTER_CRITICAL(&displayMux);
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
            portEXIT_CRITICAL(&displayMux);
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

        // Priority statistics output (every 5 minutes)
        if((millis() - stat_prio_timer) > (unsigned long)(PRIO_STAT_INTERVAL_S * 1000UL))
        {
            stat_prio_timer = millis();
            if(bLORADEBUG)
            {
                Serial.printf("[MC-STAT] t=%ds qmax=%d/%d\n",
                    PRIO_STAT_INTERVAL_S, stat_queue_hwm, MAX_RING);
                Serial.printf("  tx: p1=%d p2=%d p3=%d p4=%d p5=%d preempt=%d\n",
                    stat_tx_count[1], stat_tx_count[2], stat_tx_count[3],
                    stat_tx_count[4], stat_tx_count[5], stat_preempt_count);
                Serial.printf("  drop: p1=%d p2=%d p3=%d p4=%d p5=%d\n",
                    stat_drop_count[1], stat_drop_count[2], stat_drop_count[3],
                    stat_drop_count[4], stat_drop_count[5]);
            }
            
            for(int p = 1; p <= 5; p++)
            {
                if(stat_tx_count[p] > 0)
                {
                    uint32_t avg = stat_latency_sum[p] / stat_tx_count[p];
                    if(bLORADEBUG)
                        Serial.printf("[MC-PRIO] p%d_lat_avg=%ums p%d_lat_max=%dms p%d_cnt=%d\n",
                            p, avg, p, stat_latency_max[p], p, stat_tx_count[p]);
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
            Serial.printf("[MC-HWM] uptime=%lus queue_hwm=%d/%d csma_hwm=%d trickle=%lums\n",
                millis() / 1000, stat_queue_hwm, MAX_RING,
                stat_csma_hwm_attempts, trickle_interval_ms);
        }

        if(iReceiveTimeOutTime > 0)
        {
            // Timeout csma_timeout (slot-basierter Backoff)
            if((iReceiveTimeOutTime + csma_timeout) < millis())
            {
                // Debug A: RX_TIMEOUT_FIRE
                if(bLORADEBUG)
                    Serial.printf("[MC-DBG] RX_TIMEOUT_FIRE ts=%lu wait=%lu delta=%lu\n",
                        millis(), csma_timeout, millis() - iReceiveTimeOutTime);

                // FIX BUG #1: Do not reset radio if a received packet is pending
                if(receiveFlag)
                {
                    // A packet arrived just before timeout — let the receiveFlag
                    // handler process it. Just reset the timer.
                    iReceiveTimeOutTime = millis();

                    if(bLORADEBUG)
                        Serial.printf("[MC-DBG] RX_TIMEOUT_DEFERRED src=receiveFlag\n");
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
                                Serial.printf("[MC-DBG] RX_IRQ_STALE_EARLY rssi=%.0f flags=%s cnt=%d\n",
                                    currentRSSI, header_valid ? "HDR" : "PRE", rx_irq_defer_count);
                        }
                        else if(header_valid && rx_irq_defer_count < 3)
                        {
                            shouldDefer = true;
                            if(bLORADEBUG)
                                Serial.printf("[MC-DBG] RX_TIMEOUT_DEFERRED src=header_valid rssi=%.0f cnt=%d\n",
                                    currentRSSI, rx_irq_defer_count);
                        }
                        else if(!header_valid && rx_irq_defer_count < 1)
                        {
                            shouldDefer = true;
                            if(bLORADEBUG)
                                Serial.printf("[MC-DBG] RX_TIMEOUT_DEFERRED src=preamble_only rssi=%.0f cnt=%d\n",
                                    currentRSSI, rx_irq_defer_count);
                        }
                        else
                        {
                            if(bLORADEBUG)
                                Serial.printf("[MC-DBG] RX_IRQ_STALE rssi=%.0f flags=%s cnt=%d\n",
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
                            Serial.printf("[MC-DBG] RX_IRQ_STALE forced restart after %d deferrals\n", rx_irq_defer_count);
                        rx_irq_defer_count = 0;
                        iReceiveTimeOutTime=0;

                        // FIX BUG #1: Call startReceive FIRST, then re-wire interrupts.
                        // Disable interrupt gating while we reconfigure
                        bEnableInterruptReceive = false;
                        bEnableInterruptTransmit = false;

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
                                Serial.println(F("[MC-DBG] RX_TIMEOUT missed_edge recovery"));
                        }
                        #endif

                        // Debug B: RX_RESTART after timeout
                        if(bLORADEBUG)
                        {
                            Serial.printf("[MC-SM] IDLE -> RX_LISTEN rc=%d\n", state);
                            Serial.printf("[MC-DBG] RX_RESTART src=timeout state=%d\n", state);
                        }

                        if(bLORADEBUG)
                        {
                            Serial.print(getTimeString());
                            if (state == RADIOLIB_ERR_NONE)
                                Serial.println(F(" [LoRa]...Receive Timeout, startReceive again with sucess"));
                            else
                            {
                                Serial.print(F(" [LoRa]...Receive Timeout, startReceive again with error = "));
                                Serial.println(state);
                            }
                        }
                    }
                }
            }
        }

        if(receiveFlag || transmittedFlag)
        {
            // check ongoing reception
            if(receiveFlag)
            {
                // Debug C: RX_FLAG_PROCESS
                if(bLORADEBUG)
                {
                    Serial.printf("[MC-SM] RX_LISTEN -> RX_PROCESS rc=0\n");
                    Serial.printf("[MC-DBG] RX_FLAG_PROCESS ts=%lu\n", millis());
                }

                // reset flags first
                bEnableInterruptReceive = false;
                receiveFlag = false;
                rx_irq_defer_count = 0;

                // DIO triggered while reception is ongoing
                // that means we got a packet

                checkRX(bRadio);

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
                    Serial.printf("[MC-SM] TX_ACTIVE -> TX_DONE rc=%d\n", transmissionState);
                    Serial.printf("[MC-DBG] TX_DONE state=%d ts=%lu\n", transmissionState, millis());
                }

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
                        Serial.println(F("[MC-DBG] RX_RESTART missed_edge recovery"));
                }
                #endif

                // Debug H: RX_RESTARTED after TX
                if(bLORADEBUG)
                {
                    Serial.printf("[MC-SM] TX_DONE -> RX_LISTEN rc=%d\n", state);
                    Serial.printf("[MC-DBG] RX_RESTARTED src=after_tx state=%d\n", state);
                }

                if (state != RADIOLIB_ERR_NONE)
                {
                    if(bLORADEBUG)
                    {
                        Serial.print(F("[LoRa]...Starting to listen again (1)... "));
                        Serial.print(F("failed, code "));
                        Serial.println(state);
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
            int _w = iWrite;
            int _r = iRead;
            if (_w != _r)
            {
                // Debug E: TX_GATE_ENTER
                if(bLORADEBUG)
                {
                    Serial.printf("[MC-SM] IDLE -> TX_PREPARE rc=0\n");
                    Serial.printf("[MC-DBG] TX_GATE_ENTER qlen=%d cad_attempt=%d\n",
                        (_w >= _r) ? (_w - _r) : (MAX_RING - _r + _w),
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
                        is_receiving = true;
                        irq_rx_active = true;
                        if(bLORADEBUG)
                            Serial.printf("[MC-DBG] IRQ_POLL hdr/pre=0x%04X -> TX_ABORT\n",
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

                // CAD Scan 1
                int cad_result = radio.scanChannel();
                if(bLORADEBUG)
                    Serial.printf("[MC-DBG] CAD_SCAN result=%d\n", cad_result);

                bool channel_free = false;
                if(cad_result == RADIOLIB_CHANNEL_FREE)
                {
                    channel_free = true;
                }
                else
                {
                    // Double-Check: second scan to filter false positives
                    if(bLORADEBUG)
                        Serial.printf("[MC-DBG] CAD_BUSY_1 attempt=%d, double-check...\n", cad_attempt);

                    cad_result = radio.scanChannel();
                    if(bLORADEBUG)
                        Serial.printf("[MC-DBG] CAD_SCAN result=%d\n", cad_result);
                    if(cad_result == RADIOLIB_CHANNEL_FREE)
                    {
                        if(bLORADEBUG)
                            Serial.printf("[MC-DBG] CAD_FALSE_POSITIVE\n");
                        channel_free = true;
                    }
                }

                if(channel_free)
                {
                    if(bLORADEBUG)
                        Serial.printf("[MC-DBG] CAD_FREE attempt=%d\n", cad_attempt);

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
                        if(bLORADEBUG)
                        {
                            Serial.printf("[MC-SM] TX_PREPARE -> TX_ACTIVE rc=0\n");
                            int __w = iWrite;
                            int __r = iRead;
                            Serial.printf("[MC-DBG] TX_START qlen=%d\n",
                                (__w >= __r) ? (__w - __r) : (MAX_RING - __r + __w));
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
                            Serial.printf("[MC-DBG] CAD_FREE_NO_TX restoring RX\n");
                        bEnableInterruptTransmit = false;
                        radio.clearPacketSentAction();

                        bEnableInterruptReceive = true;
                        radio.setPacketReceivedAction(setFlagReceive);
                        radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);
                        iReceiveTimeOutTime = millis();
                    }
                }
                else
                {
                    // Channel busy confirmed — backoff
                    cad_attempt++;
                    csma_timeout = csma_compute_timeout(cad_attempt);

                    if(bLORADEBUG)
                    {
                        Serial.printf("[MC-SM] TX_PREPARE -> IDLE rc=-1\n");
                        Serial.printf("[MC-DBG] CAD_BUSY attempt=%d next_timeout=%lu\n",
                            cad_attempt, csma_timeout);
                    }

                    // Restore RX
                    bEnableInterruptReceive = true;
                    radio.setPacketReceivedAction(setFlagReceive);
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

    // get RTC Now
    // RTC hat Vorrang zu Zeit via MeshCom-Server
    bool bMyClock = true;

    // !posinfo_fix && !bNTPDateTimeValid
    // Time NTP
    if(meshcom_settings.node_hasIPaddress && !posinfo_fix)
    {
        strTime = "none";

        // every 15 minutes
        if((updateTimeClient + 1000 * 60 * 15) < millis() || updateTimeClient == 0)
        {
            strTime = udpUpdateTimeClient();

            updateTimeClient = millis();
        }
        else
            strTime = udpGetTimeClient();

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
    #if defined(ENABLE_RTC)
    else
    if(bRTCON)
    {
        bMyClock = false;

        loopRTC();

        if(posinfo_fix) // GPS hat Vorang zur RTC und setzt RTC
        {
            if((rtc_refresh_timer + 60000) > millis())
            {
                //only every minute
                setRTCNow(meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);
                rtc_refresh_timer = millis();
            }
        }
        else
        {
            DateTime utc = getRTCNow();

            DateTime now (utc + TimeSpan(meshcom_settings.node_utcoff * 60 * 60));

            uint16_t Year = now.year();
            uint16_t Month = now.month();
            uint16_t Day = now.day();

            uint16_t Hour = now.hour();
            uint16_t Minute = now.minute();
            uint16_t Second = now.second();

            //Serial.printf("%04i.%02i.%02i %02i:%02i:%02i\n", Year, Month, Day, Hour, Minute, Second);

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
        if(meshcom_settings.node_date_year > 2023 && meshcom_settings.node_update[0] == 0x00)
        {
            char ctemp[80];
            snprintf(ctemp, sizeof(ctemp), "%04i-%02i-%02i %02i:%02i:%02i",
             meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);

            memcpy(meshcom_settings.node_update, ctemp, 21);

            #if defined(ENABLE_RTC)
            if(bRTCON && bNTPDateTimeValid) // NTP hat Vorang zur RTC und setzt RTC
            {
                if((rtc_refresh_timer + 60000) > millis())
                {
                    //only every minute
                    setRTCNow(meshcom_settings.node_date_year, meshcom_settings.node_date_month, meshcom_settings.node_date_day, meshcom_settings.node_date_hour, meshcom_settings.node_date_minute, meshcom_settings.node_date_second);

                    rtc_refresh_timer = millis();
                }
            }
            #endif
        }
    }

    // check WiFI connected with Ping every 30 sec
    if (meshcom_settings.node_netmode == 0 &&
    (wifi_active_timer + 30000) < millis())
    {
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
            // check every 5 seconds to ready next telemetry via serial interface
            if ((softser_refresh_timer + 5000) < millis() && softserFunktion == 0)
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
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        /* KBC/KFR TEST 2025-04-10
        delay(2000); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        */

        oldDeviceConnected = deviceConnected;

        g_ble_uart_is_connected = false;
        isPhoneReady = 0;

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
    }    // check if message from phone to send

    // BLE Queue: process data from NimBLE task in Main Loop context
    {
        BleQueueItem bleItem;
        while (xQueueReceive(bleQueue, &bleItem, 0) == pdTRUE) {
            readPhoneCommand(bleItem.data);
        }
    }

    if(hasMsgFromPhone)
    {
        if(bBLEDEBUG)
            Serial.printf("[LOOP] hasMsgFromPhone\n");
        
        if(memcmp(textbuff_phone, ":", 1) == 0)
            sendMessage(textbuff_phone, txt_msg_len_phone);

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

            sendMheard();

            config_to_phone_prepare_timer = millis();

            config_to_phone_prepare = false;
        }
        else
        {
            // wait after BLE Connect 3 sec.
            if(millis() < config_to_phone_prepare_timer + 3000)
                iPhoneState = 0;

            if (iPhoneState > 3)   // only every 3 times of mainloop send to phone - main loop has no additional delay anymore! 14.06.2025
            {
                // prepare JSON config to phone after BLE connection
                // send JSON config to phone after BLE connection
                // wait at least 300ms between sending messages
                if (ComToPhoneWrite != ComToPhoneRead)
                {
                    // check every 300 ms to send to phone
                    if ((ble_wait + 300) < millis())
                    {
                        sendComToPhone();

                        ble_wait = millis();
                    }
                }
                else if (toPhoneWrite != toPhoneRead)
                {
                    // wait for each message to send to phone
                    if ((ble_wait + 400) < millis())
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
        if((config_to_phone_datetime_timer + (5 * 60 * 1000)) < millis())
        {
            bNTPDateTimeValid=false;

            config_to_phone_datetime_timer = millis();
        }
    }


    #if defined(ENABLE_MCP23017)
    // 5 sec
    if ((mcp_refresh_timer + 5000) < millis())
    {
        // get i/o state
        if(loopMCP23017())
        {
        }

        mcp_refresh_timer = millis();
    }
    #endif

    // gps display refresh every 10 sec
    gps_refresh_intervall = GPS_REFRESH_INTERVAL;

    // TRACK ON
    if(bDisplayTrack)
        gps_refresh_intervall = 2.0;

    if ((gps_refresh_timer + ((unsigned long)gps_refresh_intervall * 1000)) < millis())
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
            posinfo_hdop = 0;
        }
        else
        {
            #if defined (ENABLE_GPS)
                igps = GPS_Loop();

                if(bGPSDEBUG)
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
            #else

            #if defined (BOARD_T_DECK_PRO)
                tdeck_set_gps(true);
            #endif

            #ifdef BOARD_T_DECK_PRO
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


        #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS) || defined(BOARD_T_DECK_PRO)
            gps_refresh_track++;
            if(gps_refresh_track > 4)
            {
                tdeck_refresh_track_view();
                gps_refresh_track=0;
            }
        #endif

        #endif

        gps_refresh_timer = millis();
    }

    // posinfo_interval in Seconds
    if (((posinfo_timer + (posinfo_interval * 1000)) < millis()) || (millis() > 100000 && millis() < 130000 && bPosFirst) || posinfo_shot)
    {
        // minimal transmit time only max 15 sec
        if((posinfo_timer_min + 15000) < millis())
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
    if (((heyinfo_timer + trickle_interval_ms) < millis()) || (bHeyFirst && bAllStarted))
    {
        bHeyFirst = false;

        // Check for topology change (neighbor count changed)
        int current_neighbors = getMheardCount();
        if(trickle_last_neighbor_count >= 0 && current_neighbors != trickle_last_neighbor_count)
        {
            // Topology changed — reset to fastest interval
            trickle_interval_ms = TRICKLE_IMIN_S * 1000UL;
            trickle_consistent_count = 0;
            if(bLORADEBUG)
                Serial.printf("[MC-TRICKLE] TOPO_CHANGE neighbors=%d->%d interval_reset=%lums\n",
                    trickle_last_neighbor_count, current_neighbors, trickle_interval_ms);
        }
        trickle_last_neighbor_count = current_neighbors;

        // Trickle suppression: skip HEY if enough consistent HEYs heard
        if(trickle_consistent_count >= TRICKLE_K)
        {
            if(bLORADEBUG)
                Serial.printf("[MC-TRICKLE] SUPPRESS consistent=%d>=k=%d interval=%lums neighbors=%d\n",
                    trickle_consistent_count, TRICKLE_K, trickle_interval_ms, current_neighbors);
        }
        else
        {
            sendHey();
            if(bLORADEBUG)
                Serial.printf("[MC-TRICKLE] SEND consistent=%d<k=%d interval=%lums neighbors=%d\n",
                    trickle_consistent_count, TRICKLE_K, trickle_interval_ms, current_neighbors);
        }

        // Double interval (Trickle step), cap at Imax
        trickle_interval_ms = min(trickle_interval_ms * 2, (unsigned long)(TRICKLE_IMAX_S * 1000UL));
        trickle_consistent_count = 0;

        heyinfo_timer = millis();
    }

    // TELEMETRY_INTERVAL in Minutes == 15 minutes default
    unsigned long akt_timer = meshcom_settings.node_parm_time;
    if(akt_timer < 5 || akt_timer > 120)
    {
        akt_timer = TELEMETRY_INTERVAL;
    }
    
    akt_timer = akt_timer * 1000 * 60; // convert to minutes

    if(bSOFTSERON)
    {
        akt_timer = 20 * 1000; // 20 Seconds PARM, UNIT, EQNS and 1st T-Message
    }
        
    if (((telemetry_timer + akt_timer) < millis()) || (bTeleFirst && bAllStarted))
    {
        bTeleFirst=false;

        // APP-ID , Header only flag
        sendTelemetry(SOFTSER_APP_ID);

        telemetry_timer = millis();
    }

    mainStartTimeLoop();

    #if not defined(BOARD_T_DECK_PRO)
    if(DisplayOffWait > 0)
    {
        if (millis() > DisplayOffWait)
        {
            DisplayOffWait = 0;
            if(bDisplayOff)
            {
                bDisplayIsOff=true;
                sendDisplay1306(true, true, 0, 0, (char*)"#C");
            }
        }
    }
    #endif

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

    checkSerialCommand();

    if(BattTimeWait == 0)
        BattTimeWait = millis() - 10000;


    //Serial.printf("BattTimeWait:%i millis():%i tx:%i rx:%i\n", BattTimeWait, millis(), tx_is_active, is_receiving);

    if ((BattTimeWait + 20000) < millis())  // 20 sec
    {
        if (tx_is_active == false && is_receiving == false)
        {
            #if defined(MODUL_FW_TBEAM)
                int pmu_proz=0;
                if(PMU != NULL)
                {
                    global_batt = (float)PMU->getBattVoltage();
                    global_proz = (int)PMU->getBatteryPercent();

                    // no BATT
                    if(global_proz < 0)
                    {
                        if(bDisplayCont)
                            Serial.println("[readBatteryVoltage]...no battery is connected");
                            
                        global_batt = (float)PMU->getVbusVoltage();
                        global_proz=100.0;
                    }
                    else
                    {
                        if(global_proz < 1.0 && global_batt < 3200.0)
                            global_proz = 2;
                    }
                }
                else
                {
                    global_batt = 0;
                    global_proz = 0;
                }

                if(bDisplayCont)
                    Serial.printf("[readBatteryVoltage]...PMU.volt %.1f PMU.proz %i %i\n", global_batt, global_proz, pmu_proz);
            #else
            
                global_batt = read_batt();
                global_proz = mv_to_percent(global_batt);
                
                if(bDisplayCont)
                {
                    #if not defined (BOARD_T_DECK_PRO)
                    Serial.printf("[readBatteryVoltage] %s ... %.2f V %i%% max_batt %.3f V\n", getTimeString().c_str(), global_batt/1000., global_proz, meshcom_settings.node_maxv);
                    #endif
                }

                #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
                tdeck_update_batt_label(global_batt/1000., global_proz);
                #endif 
            
            #endif

            if(bDisplayCont)
            {
                Serial.printf("%s;[HEAP];%d;(free)\n", getTimeString().c_str(), ESP.getFreeHeap());
                Serial.printf("%s;[PSRM];%d\n", getTimeString().c_str(), ESP.getFreePsram());
            }

            // [OE3WAS] Lüftersteuerung
            #if defined(NTC_PIN) && defined(FAN_CTRL) // BOARD_TBEAM_1W
            float NTCtemp = getTempForNTC();
            if (NTCtemp > 35.0) { digitalWrite(FAN_CTRL, HIGH); 
            } else if (NTCtemp < 28.0) { digitalWrite(FAN_CTRL, LOW); }

            if(bWXDEBUG)
                Serial.printf("%s;[TEMP];%.2f;%s\n", getTimeString().c_str(), NTCtemp, digitalRead(FAN_CTRL) ? "on" : "off");
                
            #endif

            BattTimeWait = millis();
        }
    }

    // Heap Monitor — always active, 60s interval
    {
        static unsigned long heapMonTimer = 0;
        if (heapMonTimer == 0)
            heapMonTimer = millis();

        if ((heapMonTimer + 60000) < millis())
        {
            Serial.printf("%s;[HEAP];%d;%d;%d;(mon)\n",
                getTimeString().c_str(),
                ESP.getFreeHeap(),
                ESP.getMinFreeHeap(),
                ESP.getMaxAllocHeap());
            #if defined(BOARD_HAS_PSRAM)
            Serial.printf("%s;[PSRM];%d;(mon)\n",
                getTimeString().c_str(),
                ESP.getFreePsram());
            #endif
            heapMonTimer = millis();
        }
    }

    #ifdef OneWire_GPIO
    if(bONEWIRE)
    {
        if ((onewireTimeWait + 30000) < millis())  // 30 sec
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

        if ((BMXTimeWait + 60000) < millis())   // 60 sec
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

    // read BMP390 Sensor
    #if defined(ENABLE_BMP390)
    if((bBMP3ON && bmp3_found))
    {
        if ((BMP3TimeWait + 60000) < millis())   // 60 sec
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
        if ((MCU811TimeWait + 60000) < millis())   // 60 sec
        {
            // read MCU-811 Sensor
            if(loopMCU811())
            {
                meshcom_settings.node_co2 = geteCO2();
                
                if(wx_shot)
                {
                    commandAction((char*)"--wx", isPhoneReady, false);
                    wx_shot = false;
                }
            }

            MCU811TimeWait = millis(); // wait for next messurement
        }
    }
    #endif

    #if defined(ENABLE_INA226)
    if(bINA226ON && ina226_found)
    {
        if(INA226TimeWait == 0)
            INA226TimeWait = millis() - 10000;

        if ((INA226TimeWait + 15000) < millis())   // 15 sec
        {
            // read INA226 Sensor
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
    
    ////////////////////////////////////////////////
    // WIFI Gateway functions
    if(bGATEWAY && meshcom_settings.node_hasIPaddress)
    {
        getMeshComUDP();

        sendMeshComUDP();

        // heartbeat
        if ((hb_timer + (HEARTBEAT_INTERVAL * 1000)) < millis())
        {
            sendMeshComHeartbeat();
            hb_timer = millis();

            if (last_upd_timer > 0)
            {
                unsigned long hb_age = millis() - last_upd_timer;

                // Stage 1: diagnostic warning at 35s
                if (hb_age > (HB_WARN_TIME * 1000) && !hb_warn_logged)
                {
                    bool wifi_ok = (WiFi.status() == WL_CONNECTED);
                    Serial.printf("[UDP] Server not responding for %lus — WiFi %s\n",
                                  hb_age / 1000, wifi_ok ? "CONNECTED" : "NOT_CONNECTED");
                    hb_warn_logged = true;

                    // WiFi actually down → reset immediately, don't wait
                    if (!wifi_ok)
                    {
                        Serial.println("[UDP] WiFi down — resetting");
                        resetMeshComUDP();
                        last_upd_timer = millis();
                        hb_warn_logged = false;
                    }
                }

                // Stage 2: timeout at 65s
                if (hb_age > (MAX_HB_RX_TIME * 1000))
                {
                    bool wifi_ok = (WiFi.status() == WL_CONNECTED);

                    if (!wifi_ok)
                    {
                        Serial.printf("[UDP] Heartbeat timeout %lus — WiFi NOT_CONNECTED, resetting\n",
                                      hb_age / 1000);
                        resetMeshComUDP();
                    }
                    else
                    {
                        Serial.printf("[UDP] Heartbeat timeout %lus — WiFi CONNECTED, server unresponsive, waiting\n",
                                      hb_age / 1000);
                    }

                    last_upd_timer = millis();
                    hb_warn_logged = false;
                }
            }
        }

        meshcom_settings.node_last_upd_timer = hb_timer;

    }

    if(bEXTUDP)
    {
        getExternUDP();
        flushExternQueue();
    }

    if(bWEBSERVER || bEXTUDP || bGATEWAY)
    {
        if (web_timer == 0 || (iWlanWait > 0 && ((web_timer + 1000) < millis())) || ((web_timer + (HEARTBEAT_INTERVAL * 1000 * 10)) < millis()))   // repeat 5 minutes
        {

            web_timer = millis();

            #ifndef BOARD_RAK4630
                if(!meshcom_settings.node_hasIPaddress || iWlanWait > 0)
                {
                    if(iWlanWait == 0)
                    {
                        // restart WEB-Client
                        if(bWEBSERVER)
                            stopWebserver();

                        startNetwork();
                    }
                    else
                    {
                        doWiFiConnect();

                        if(iWlanWait > 15)
                        {
                            iWlanWait = 0;

                            if (!bAllStarted)
                            {
                                // First boot failure — full radio power-cycle and retry
                                Serial.println("[WIFI]...no connection at boot — full radio reset and retrying");
                                WiFi.disconnect(true, true);
                                WiFi.mode(WIFI_OFF);
                                delay(1500);
                                startNetwork();  // sets iWlanWait = 1, triggers doWiFiConnect() polling
                            }
                            else
                            {
                                Serial.println("[WIFI]...SET but no Wifi connect ...please wait for next try (5 min)");
                            }

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

        if(bWEBSERVER && iWlanWait == 0)
        {
            startWebserver();

            loopWebserver();
        }

        if(bEXTUDP && iWlanWait == 0)
        {
        #ifdef BOARD_T_ETH_ELITE
            if(meshcom_settings.node_hasIPaddress == false)
            {
                neth.initethDHCP();
            }
        #endif

            startExternUDP();
        }
    }

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

    if ((tdeck_tft_timer + (TDECK_TFT_TIMEOUT * 1000)) < millis())
    {
        // Serial.printf("Loop: Timeout reached. Timer: %lu, Millis: %lu\n", tdeck_tft_timer, millis());
        tft_off();
    }

    lv_task_handler();

    #endif

    //
    ////////////////////////////////////////////////

    // WOR/KBC not necesary     delay(100);

    // yield();
    delay(5); // Save power by allowing IDLE task to run
}


int checkRX(bool bRadio)
{
    int state = -1;

    //#if not defined(BOARD_T_DECK_PRO)
    // extra source
    // you can receive data as an Arduino String
    // NOTE: receive() is a blocking method!
    //       See example ReceiveInterrupt for details
    //       on non-blocking reception method.

    if(!bRadio)
        return -1;
        
    if(is_receiving)    // receive in action
        return -1;

    is_receiving=true;

    uint8_t payload[UDP_TX_BUF_SIZE+10];
    
    size_t ibytes = UDP_TX_BUF_SIZE;

    state = radio.readData(payload, ibytes);

    if (state == RADIOLIB_ERR_LORA_HEADER_DAMAGED || state == RADIOLIB_ERR_NONE)
    {
        // FIX BUG #2: Save RSSI/SNR/FreqError BEFORE restarting RX.
        // These read from SX1262 hardware registers via SPI and would be
        // invalidated once startReceive() transitions the radio to standby+RX.
        int16_t saved_rssi = (int16_t)radio.getRSSI();
        int8_t  saved_snr  = (int8_t)radio.getSNR();
        float   saved_ferr = radio.getFrequencyError();

        // FIX BUG #2: Restart RX immediately after reading FIFO, BEFORE processing.
        // The packet data is already in our local payload buffer.
        // The radio can now listen for the next packet while we process this one.
        {
            radio.clearPacketReceivedAction();
            radio.clearPacketSentAction();
            int rxstate = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);          // Radio in RX zuerst
            radio.setPacketReceivedAction(setFlagReceive);
            bEnableInterruptReceive = true;
            #ifdef LORA_DIO1
            if(digitalRead(LORA_DIO1) == HIGH)
            {
                receiveFlag = true;
                if(bLORADEBUG)
                    Serial.println(F("[MC-DBG] CHECKRX missed_edge recovery"));
            }
            #endif

            // Debug D: RX_RESTARTED after readData
            if(bLORADEBUG)
                Serial.printf("[MC-DBG] RX_RESTARTED src=after_readData state=%d\n", rxstate);
        }

        if(bLORADEBUG)
        {
            // packet was successfully received
            Serial.print(F("[LoRa]...Received packet: "));

            // print RSSI (Received Signal Strength Indicator)
            Serial.print(F("RSSI:\t\t"));
            Serial.print(saved_rssi);
            Serial.print(F(" dBm / "));

            // print SNR (Signal-to-Noise Ratio)
            Serial.print(F("SNR:\t\t"));
            Serial.print(saved_snr);
            Serial.print(F(" dB / "));

            // print frequency error
            Serial.print(F("Frequency error:\t"));
            Serial.print(saved_ferr);
            Serial.println(F(" Hz"));
        }

        // RX channel utilization: calculate airtime from packet length
        // (ESP32 has no OnHeaderDetect, so ch_util_rx_start is never set)
        ch_util_rx_accum.fetch_add(radio.getTimeOnAir(ibytes) / 1000);  // us -> ms

        OnRxDone(payload, (uint16_t)ibytes, saved_rssi, saved_snr);
    }
    else
    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        if(bLORADEBUG)
            Serial.printf("OnRxError\n");

        // RSSI/SNR/FreqError VOR RX-Restart sichern (Register werden ungueltig)
        int16_t saved_crc_rssi = (int16_t)radio.getRSSI();
        int8_t  saved_crc_snr  = (int8_t)radio.getSNR();
        float   saved_crc_ferr = radio.getFrequencyError();

        // RX sofort wieder starten
        {
            radio.clearPacketReceivedAction();
            radio.clearPacketSentAction();
            bEnableInterruptReceive = true;
            radio.setPacketReceivedAction(setFlagReceive);
            int rxstate = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);

            if(bLORADEBUG)
                Serial.printf("[MC-DBG] RX_RESTARTED src=after_crc_error state=%d\n", rxstate);
        }

        // RX channel utilization: CRC-failed packet still occupied the channel
        ch_util_rx_accum.fetch_add(radio.getTimeOnAir(ibytes) / 1000);  // us -> ms

        // Diagnose-Output: RSSI/SNR + kompletter Payload-Hex-Dump
        if(bLORADEBUG)
        {
            Serial.printf("[MC-DBG] CRC_ERROR rssi=%d snr=%d freq_err=%.1f size=%d ts=%lu\n",
                saved_crc_rssi, saved_crc_snr, saved_crc_ferr, (int)ibytes, millis());

            // Hex-Dump des beschaedigten Payloads (max 255 Bytes)
            int dump_len = (ibytes > 255) ? 255 : (int)ibytes;
            Serial.printf("[MC-DBG] CRC_PAYLOAD[%d]: ", dump_len);
            for(int i = 0; i < dump_len; i++)
                Serial.printf("%02X ", payload[i]);
            Serial.println();
        }
    }
    else
    {
        if(bLORADEBUG)
            Serial.printf("OnRxError\n");

        // RX-Restart auch bei unbekannten Fehlern -- ohne dies bleibt
        // das Radio im Standby (BLINDSPOT fuer Empfang!)
        int16_t saved_err_rssi = (int16_t)radio.getRSSI();
        int8_t  saved_err_snr  = (int8_t)radio.getSNR();

        {
            radio.clearPacketReceivedAction();
            radio.clearPacketSentAction();
            bEnableInterruptReceive = true;
            radio.setPacketReceivedAction(setFlagReceive);
            int rxstate = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED), RADIOLIB_IRQ_RX_DEFAULT_MASK, 0);

            if(bLORADEBUG)
                Serial.printf("[MC-DBG] RX_RESTARTED src=after_other_error state=%d\n", rxstate);
        }

        // Immer loggen (nicht nur bLORADEBUG) -- unbekannte Fehler sind kritisch
        Serial.printf("[MC-DBG] RX_OTHER_ERROR code=%d rssi=%d snr=%d size=%d ts=%lu\n",
            state, saved_err_rssi, saved_err_snr, (int)ibytes, millis());

        // Payload-Dump bei Debug aktiv
        if(bLORADEBUG && ibytes > 0)
        {
            int dump_len = (ibytes > 255) ? 255 : (int)ibytes;
            Serial.printf("[MC-DBG] ERR_PAYLOAD[%d]: ", dump_len);
            for(int i = 0; i < dump_len; i++)
                Serial.printf("%02X ", payload[i]);
            Serial.println();
        }
    }

    //#endif

    is_receiving=false;

    return state;
}

void checkSerialCommand(void)
{
    //  Check Serial connected
    if(!Serial)
    {
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
                strncpy(msg_text, strText.c_str(), sizeof(msg_text) - 1);
                msg_text[sizeof(msg_text) - 1] = '\0';

                int inext=0;
                char msg_buffer[600];
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

                        // buffer size reached
                        if(inext > sizeof(msg_buffer)-2)
                            break;
                    }
                }

                if(strText.startsWith("::"))
                {
                    sendMessage(msg_buffer, inext);
                }
                else
                    if(strText.startsWith("--"))
                        commandAction(msg_buffer, isPhoneReady, false);
                    else
                        Serial.printf("\n...wrong command %s\n", strText.c_str());

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
