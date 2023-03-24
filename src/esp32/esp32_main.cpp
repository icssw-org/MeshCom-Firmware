#include <Arduino.h>
#include <RadioLib.h>
//#include <TinyGPSPlus.h>
#include <Wire.h>               
#include "SSD1306Wire.h"
#include <SPI.h>
#include <configuration.h>
#include <WiFi.h>

#include <U8g2lib.h> // Click to install library: http://librarymanager/All#u8g2
#include <esp_adc_cal.h>

#include "debugconf.h"
#include "esp_flash.h"
#include "TinyGPS.h"
#include "bat.h"

#define ADC_PIN             34  //ADC_PIN is the ADC pin the battery is connected to through a voltage divider

/**
 * RadioLib Infos und Examples:
 * SX127x:
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Settings/SX127x_Settings.ino
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Receive/SX127x_Receive.ino
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Transmit/SX127x_Transmit.ino
 * https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_PingPong/SX127x_PingPong.ino
*/

// Wie mit fehlender DIO1 PinDefinition beim TLORA_OLED_V2 umgehen? Theoretisch wird DIO1 nur für LoRaWAN benötigt

/**
 * LoRa Sync Word für Meshcom definieren! 2b = Meshtastic
 * SX127x Chips haben nur ein Byte als Syncowrd, SX126x haben zwei Byte
 * Alle Funktionen Setter, Getter, etc finden sich in den korrspondierenden Libraries
 * SX1278 zB: https://github.com/jgromes/RadioLib/blob/master/src/modules/SX127x/SX1278.h
*/

s_meshcom_settings g_meshcom_settings;

bool bInitDisplay = true;
bool ble_busy_flag = false;
uint8_t isPhoneReady = 0;      // flag we receive from phone when itis ready to receive data

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
   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


// BLE Ringbuffer to phone
unsigned char BLEtoPhoneBuff[MAX_RING][UDP_TX_BUF_SIZE];
int toPhoneWrite=0;
int toPhoneRead=0;
// Textmessage buffer from phone, hasMsgFromPhone flag indicates new message
bool hasMsgFromPhone = false;
char textbuff_phone [MAX_MSG_LEN_PHONE] = {0};
uint8_t txt_msg_len_phone = 0;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

void sendConfigToPhone();


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        // Forward data from Mobile to our peripheral
        uint8_t conf_data[MAX_MSG_LEN_PHONE] = {0};
        size_t conf_length=0;

        conf_length = pCharacteristic->getLength();

        if (conf_length <= 0)
            return;

        memcpy(conf_data, pCharacteristic->getData() , conf_length);

        /**
         * Config Messages
         * length 1B - Msg ID 1B - Data
         * Msg ID:
         * 0x10 - Hello Message (followed by 0x20, 0x30)
         * 0x50 - Callsign
         * 0x70 - Latitude
         * 0x80 - Longitude
         * 0x90 - Altitude
         * 0xA0 - Textmessage
         * Data:
         * Callsign: length Callsign 1B - Callsign
         * Latitude: 8B Double
         * Longitude: 8B Double
         * Altitude: 4B Integer
         * 
         * 
         *  */ 

        uint8_t msg_len = conf_data[0];
        uint8_t msg_type = conf_data[1];
        bool restart_ADV = false;

        //DEBUG_MSG_VAL("BLE", conf_length, "Msg from Device Length");

        switch (msg_type)
        {
            case 0x10: {

                if(conf_data[2] == 0x20 && conf_data[3] == 0x30)
                {
                    DEBUG_MSG("BLE", "Hello Msg from phone");
                    sendConfigToPhone();
                    isPhoneReady = 1;
                }

                break;
            }

            case 0x50: {

                DEBUG_MSG("BLE", "Callsing Setting from phone");

                char call_arr[(uint8_t)conf_data[2] + 1];
                call_arr[(uint8_t)conf_data[2]] = '\0';

                for(int i=0; i<(uint8_t)conf_data[2]; i++)
                    call_arr[i] = conf_data[i+3];
                
                String sVar = call_arr;
                sVar.toUpperCase();
                

                sprintf(g_meshcom_settings.node_call, "%s", sVar.c_str());

                save_settings();

                // send config back to phone
                sendConfigToPhone();

                //sprintf(helper_string, "%s-%s", g_ble_dev_name, g_meshcom_settings.node_call);	// Anzeige mit callsign
                //Bluefruit.setName(helper_string);

                bInitDisplay = false;

                //restart_ADV = true;	//nicht notwendig

                break;
            }

            case 0x70: {

                DEBUG_MSG("BLE", "Latitude Setting from phone");
                float latitude;
                memcpy(&latitude, conf_data + 2, sizeof(latitude));

                g_meshcom_settings.node_lat=latitude;

                save_settings();
                // send config back to phone
                sendConfigToPhone();

                break;
            }

            case 0x80: {

                DEBUG_MSG("BLE", "Longitude Setting from phone");
                float longitude;
                memcpy(&longitude, conf_data + 2, sizeof(longitude));

                g_meshcom_settings.node_lon=longitude;

                save_settings();
                // send config back to phone
                sendConfigToPhone();

                break;
            }

            case 0x90: {

                int altitude;
                memcpy(&altitude, conf_data + 2, sizeof(altitude));
                DEBUG_MSG_VAL("BLE", altitude, "Altitude from phone:");

                g_meshcom_settings.node_alt=altitude;

                save_settings();
                // send config back to phone
                sendConfigToPhone();

                break;
            }

            case 0xA0: {
                // length 1B - Msg ID 1B - Text

                uint8_t txt_len = msg_len - 1; // includes zero escape
                txt_msg_len_phone = txt_len - 1;	// now zero escape for lora TX

                char textbuff [txt_len] = {0};
                textbuff [txt_len] = '\0';
                memcpy(textbuff, conf_data + 2, txt_len);

                // kopieren der message in buffer fuer main
                memcpy(textbuff_phone, conf_data + 2, txt_msg_len_phone);

                DEBUG_MSG_TXT("BLE", textbuff, "Message from phone:");

                // flag für main neue msg von phone
                hasMsgFromPhone = true;

            }

        }
    }

    #if BLE_TEST > 0
        if(str[0] == ':')
        {
            if(str[strlen(str)-1] == 0x0a)
                str[strlen(str)-1]=0x00;

            sendMessage(str, strlen(str));
        }
        else
        if(str[0] == '-')
        {
            commandAction(str, strlen(str), true);
        }
    #endif

};

/**
 * @brief Method to send configuration to phone 
 * Config Format:
 * LENGTH 2B - FLAG 1B - LENCALL 1B - Callsign - LAT 8B(Double) - LON 8B(Double) - ALT 4B(INT)
*/
void sendConfigToPhone () {

    ble_busy_flag = true;

	#if BLE_TEST
		char bleBuff [100] = {0};
		sprintf(bleBuff, "Connected to %s\n", g_meshcom_settings.node_call);
		// send to phone
        site_t blelen=strlen(bleBuff);
        pTxCharacteristic->setValue(bleBuff, blelen);
        pTxCharacteristic->notify();
	#else
		// assemble conf message
		uint8_t call_len = sizeof(g_meshcom_settings.node_call);
		size_t conf_len = call_len + 22;	// currently fixed length - adapt if needed
		uint8_t confBuff [conf_len] = {0};
		uint8_t call_offset = 2;

		confBuff [0] = 0x80;
		confBuff [1] = call_len;
		memcpy(confBuff + call_offset, g_meshcom_settings.node_call, call_len);
		uint8_t latOffset = call_offset + call_len;
		memcpy(confBuff + latOffset, &g_meshcom_settings.node_lat, 8);
		memcpy(confBuff + latOffset + 8, &g_meshcom_settings.node_lon, 8);
		memcpy(confBuff + latOffset + 16, &g_meshcom_settings.node_alt, 4);

		// send to phone
        pTxCharacteristic->setValue(confBuff, conf_len);
        pTxCharacteristic->notify();
	#endif

	ble_busy_flag = false;
};


bool bDEBUG=false;

// RadioModule SX1278 
SX1278 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
// SyncWord - 1byte for SX127x, 2 bytes for SX126x
// Check which chip is used !!!

// Lora callback Function declarations
void checkRX(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void OnTxDone(void);
void OnTxTimeout(void);
void OnPreambleDetect(void);
void OnHeaderDetect(void);
void doTX();        // LoraTX function
void setInterruptFlag();     // LoRaRX Interrupt function


asm(".global _scanf_float");
asm(".global _printf_float");

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_UNKNOWN;

bool is_receiving = false;  // flag to store we are receiving a lora packet. triggered by header detect not preamble

int sendlng = 0;              // lora tx message length

// LoRa Events and Buffers

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "RAK-CL40";

int vref = 1100;                //Default value. We'll get a more accurate value from the efuses
uint64_t timeStamp = 0;

/** Flag if BLE UART is connected */
bool g_ble_uart_is_connected = false;

// Client basic variables
uint8_t dmac[6];
unsigned int _GW_ID = 0x12345678; // ID of our Node

char msg_text[MAX_MSG_LEN_PHONE];

unsigned int msg_counter = 0;
unsigned int posinfo_first = 0;
unsigned long posinfo_timer = 0;      // we check periodically to send GPS

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Display 128 x 64 px
void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text);
void sendDisplayHead();
void sendDisplayText(uint8_t *text, int size);

bool is_new_packet(uint8_t compBuffer[4]);     // switch if we have a packet received we never saw before RcvBuffer[12] changes, rest is same
bool tx_is_active = false;    // avoids calling doTX() on each main iteration when we are already in TX mode
void addNodeData(uint16_t size, int16_t rssi, int8_t snr); // add additional data we need and send the udp packet
void checkSerialCommand(void);
void commandAction(char *buffer, int len, bool ble);
void sendMessage(char *buffer, int len);
int CallToAPRS(char msg_type, uint8_t msg_buffer[MAX_MSG_LEN_PHONE]);
void sendWX(char *buffer, float temp, float hum, float press);
void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt);
void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press);
void addUdpOutBuffer(int msg_id, uint8_t *buffer, uint16_t len); // function adds outgoing udp messages in the udp_out_ringbuffer
void printBuffer(uint8_t *buffer, int len);
void printBuffer_ascii(uint8_t *buffer, int len);
void addBLEOutBuffer(uint8_t *buffer, uint16_t len);
void sendToPhone();

static uint8_t txBuffer[UDP_TX_BUF_SIZE]; // we need an extra buffer for udp tx, as we add other stuff (ID, RSSI, SNR, MODE)
uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE];  // lora tx buffer

static uint8_t RcvBuffer[UDP_TX_BUF_SIZE];
static uint8_t RcvBuffer_before[MAX_RING_UDP_OUT][4];

static uint8_t ringBufferUDPout[MAX_RING_UDP_OUT][UDP_TX_BUF_SIZE]; //Ringbuffer for UDP TX from LoRa RX, first byte is length
uint8_t udpWrite = 0;   // counter for ringbuffer
uint8_t udpRead = 0;    // counter for ringbuffer

// RINGBUFFER for incoming UDP lora packets for lora TX
unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE];
int iWrite=0;
int iRead=0;

// TinyGPS
TinyGPS gps;

String tmp_data = "";
int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

void getGPS(void);

bool g_meshcom_initialized;
bool init_flash_done=false;

String strText="";

unsigned int  getMacAddr(void)
{
  return ESP.getEfuseMac() & 0xFFFFFFFF;
}

void showVoltage()
{
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v2 = analogRead(ADC_PIN);
        float battery_voltage = ((float)v2 / 4095.0) * 2.0 * 3.3 * (1100 / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V\n";
        Serial.println(voltage);
    }
}

void esp32setup()
{
 	// Get LoRa parameter
	init_flash();

   _GW_ID = getMacAddr();

    dmac[0] = 0x00;
    dmac[1] = 0x00;
    dmac[2] = _GW_ID >> 24;
    dmac[3] = _GW_ID >> 16;
    dmac[4] = _GW_ID >> 8;
    dmac[5] = _GW_ID;

    // clear the buffers
    for (int i = 0; i < uint8_t(sizeof(RcvBuffer)); i++)
    {
        RcvBuffer[i] = 0x00;
    }

    for(int ib=0; ib<MAX_RING_UDP_OUT; ib++)
    {
        RcvBuffer_before[ib][0] = 0x00;
        RcvBuffer_before[ib][1] = 0x00;
        RcvBuffer_before[ib][2] = 0x00;
        RcvBuffer_before[ib][3] = 0x00;
    }

    //clear ringbuffer
    for(int i=0; i<MAX_RING_UDP_OUT; i++)
    {
        memset(ringBufferUDPout[i], 0, UDP_TX_BUF_SIZE);
    }

    Wire.begin();
    u8g2.begin();

    Serial.begin(115200);

    // initialize SX1278 with default settings
    Serial.print(F("[SX1278] Initializing ... "));
    int state = radio.begin();
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }


    // you can also change the settings at runtime
    // and check if the configuration was changed successfully

    // set carrier frequency 
    if (radio.setFrequency(RF_FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY) {
        Serial.println(F("Selected frequency is invalid for this module!"));
        while (true);
    }

    // set bandwidth 
    if (radio.setBandwidth(LORA_BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        Serial.println(F("Selected bandwidth is invalid for this module!"));
        while (true);
    }

    // set spreading factor 
    if (radio.setSpreadingFactor(LORA_SF) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        Serial.println(F("Selected spreading factor is invalid for this module!"));
        while (true);
    }

    // set coding rate 
    if (radio.setCodingRate(LORA_CR) == RADIOLIB_ERR_INVALID_CODING_RATE) {
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
    if (radio.setOutputPower(TX_OUTPUT_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
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

    // set amplifier gain  (accepted range is 1 - 6, where 1 is maximum gain)
    // NOTE: set value to 0 to enable automatic gain control
    //       leave at 0 unless you know what you're doing
    if (radio.setGain(0) == RADIOLIB_ERR_INVALID_GAIN)
    {
        Serial.println(F("Selected gain is invalid for this module!"));
        while (true);
    }

    // set the function that will be called
    // when new packet is received
    radio.setDio0Action(setInterruptFlag);

    // put module back to listen mode
    radio.startReceive();

    Serial.println(F("All settings successfully changed!"));

    // Create the BLE Device
    char strBLEName[50]={0};
    sprintf(strBLEName, "MeshCom-%02x%02x-%s", dmac[4], dmac[5], g_meshcom_settings.node_call);
    BLEDevice::init(strBLEName);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_TX,
                                            BLECharacteristic::PROPERTY_NOTIFY
                                        );
                        
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                                CHARACTERISTIC_UUID_RX,
                                                BLECharacteristic::PROPERTY_READ |
                                                BLECharacteristic::PROPERTY_WRITE
                                            );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->addServiceUUID(pService->getUUID());
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
    
    // BATT
    adcAttachPin(ADC_PIN);
    
    analogRead(ADC_PIN);
    
    analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    }
    else
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    }
    else
    {
        Serial.println("Default Vref: 1100mV");
    }
   
    Serial.println();

    // gps init
    // TBEAM PINS
    // RXD	34 
    // TXD	12 (WB_I02)
    // Baud	9600

    /*
    pinMode(12, OUTPUT);
    digitalWrite(12, 0);
    delay(1000);
    digitalWrite(12, 1);
    delay(1000);
    
    Serial1.begin(9600);
    Serial1.setTimeout(500);
    while (!Serial1);

    Serial.println("=====================================");
    Serial.println("GPS UART init ok!");
    */

    Serial.println("CLIENT STARTED");

}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setInterruptFlag(void)
{
    if(tx_is_active)
    {
        if(bDEBUG)
            Serial.println("transmit done");

        // we sent a packet, set the flag
        transmittedFlag = true;
    }
    else
    {
        if(bDEBUG)
            Serial.println("receive ready");

        // we got a packet, set the flag
        receivedFlag = true;
    }
}

void esp32loop()
{
    // check if the previous transmission finished
    if(transmittedFlag)
    {
        // reset flag
        transmittedFlag = false;

        if (transmissionState == RADIOLIB_ERR_NONE)
        {
            // packet was successfully sent
            if(bDEBUG)
                Serial.println(F("transmission finished!"));

            // NOTE: when using interrupt-driven transmit method,
            //       it is not possible to automatically measure
            //       transmission data rate using getDataRate()
            OnTxDone();
        }
        else
        {
                Serial.print(F("TX failed, code "));
                Serial.println(transmissionState);

                OnTxTimeout();
        }

        transmissionState = RADIOLIB_ERR_UNKNOWN;

        // clean up after transmission is finished
        // this will ensure transmitter is disabled,
        // RF switch is powered down etc.
        radio.finishTransmit();
    }

    // LORA RECEIVE
    if(receivedFlag)
    {
        checkRX();
    
        // reset flag
        receivedFlag = false;

        // put module back to listen mode
        radio.startReceive();
    }

    // LORA SEND
    // check if we have messages in ringbuffer to send
    if (iWrite != iRead)
    {
        if (tx_is_active == false && is_receiving == false)
            doTX();
    }

    // BLE
    if (deviceConnected)
    {
        pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->notify();
        txValue++;
    
    	g_ble_uart_is_connected = true;
	
    	delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;

        g_ble_uart_is_connected = false;
        isPhoneReady = 0;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

    if(bInitDisplay)
    {
      sendDisplayHead();

      bInitDisplay=false;
    }

    // check if message from phone to send
    if(hasMsgFromPhone)
    {
        sendMessage(textbuff_phone, txt_msg_len_phone);

        hasMsgFromPhone = false;
    }

    // posinfo
    if (((posinfo_timer + POSINFO_INTERVAL * 1000) < millis()) || posinfo_first == 1)
    {
        sendPosition(g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt, (int)mv_to_percent(read_batt()));
        posinfo_first=2;

        save_settings();    // Position ins Flash schreiben

        #if defined(LPS33)

        sendWeather(g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt,
         g_meshcom_settings.node_temp, g_meshcom_settings.node_hum, g_meshcom_settings.node_press);

        #endif

        posinfo_timer = millis();
    }

    // check if we have messages for BLE to send
    if (toPhoneWrite != toPhoneRead)
    {
        if(isPhoneReady == 1)
            sendToPhone();   
    }

    checkSerialCommand();

    delay(100);

    yield();
}

void checkRX(void)
{
    //Serial.println(F("[SX1278] Waiting for incoming transmission ... "));

    // you can receive data as an Arduino String
    // NOTE: receive() is a blocking method!
    //       See example ReceiveInterrupt for details
    //       on non-blocking reception method.

    is_receiving=true;

    uint8_t payload[UDP_TX_BUF_SIZE];

    String str;
    
    int ibytes = UDP_TX_BUF_SIZE;

    int state = radio.readData(payload, ibytes);

    for(int irx=6; irx<UDP_TX_BUF_SIZE; irx++)
    {
        if(payload[irx] == 0x00)
        {
            ibytes=irx+3;   // ikl. FCS
            break;
        }
    }

    if (state == RADIOLIB_ERR_NONE)
    {
        OnRxDone(payload, ibytes, (int16_t)radio.getRSSI(), (int8_t)radio.getSNR());
    }
    else
    if (state == RADIOLIB_ERR_RX_TIMEOUT)
    {
        if(bDEBUG)
            Serial.println("RX timeout");

        OnRxTimeout();
    }
    else
    if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        if(bDEBUG)
            Serial.println("RX CRC mismatch");

        OnRxError();
    }
    else
    {
        if(bDEBUG)
            Serial.println("RX Error");

        OnRxError();
    }
}

/** @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    memcpy(RcvBuffer, payload, size);

    int msg_id = (RcvBuffer[4]<<24) | (RcvBuffer[3]<<16) | (RcvBuffer[2]<<8) | RcvBuffer[1];

    if(is_new_packet(RcvBuffer+1) || (SEE_ALL_PACKETS == 1))
    {
		// :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

        // print which message type we got
        uint8_t msg_type_b_lora = payload[0];

        switch (msg_type_b_lora)
        {

            case 0x3A: DEBUG_MSG("RADIO", "Received Textmessage"); break;
            case 0x21: DEBUG_MSG("RADIO", "Received PosInfo"); break;
            case 0x40: DEBUG_MSG("RADIO", "Received Weather"); break;
            case 0x06: DEBUG_MSG("RADIO", "Admin APP"); break;
            case 0x20: DEBUG_MSG("RADIO", "Reply APP"); break;
            case 0x42: DEBUG_MSG("RADIO", "Rangetest APP"); break;
            case 0x43: DEBUG_MSG("RADIO", "Environmental APP"); break;
            default:
                DEBUG_MSG("RADIO", "Received unknown");
                if(bDEBUG)
                    printBuffer(RcvBuffer, size);
                break;
        }

        // txtmessage, position
        if(msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21 || msg_type_b_lora == 0x40)
        {
            // print ascii of message
            printBuffer_ascii(RcvBuffer, size);

            // we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
            // MODE BYTE: LongSlow = 1, MediumSlow = 3
            // and send the UDP packet (done in the method)

            // we only send the packet via UDP if we have no collision with UDP rx
            // und wenn MSG nicht von einem anderen Gateway empfangen wurde welches es bereits vopm Server bekommen hat
            int msg_hop = RcvBuffer[5] & 0x07;
            int msg_hop_pos = 5;
            bool msg_server =false;
            if(RcvBuffer[5] & 0x80)
                msg_server=true;

            int lora_msg_len = size; // size ist uint16_t !
            if (lora_msg_len > UDP_TX_BUF_SIZE)
            lora_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

            if(bDEBUG)
                printf("msg_id: %04X msg_len: %i payload[%i]=%i via=%d\n", msg_id, lora_msg_len, msg_hop_pos, msg_hop, msg_server);

            //if(!msg_server) // Message kommt von User
            {
                // Wiederaussendung via LORA
                // Ringbuffer filling

                bool bMsg=false;

                for(int iop=0;iop<MAX_RING_UDP_OUT;iop++)
                {
                    int ring_msg_id = (ringBufferUDPout[iop][3]<<24) | (ringBufferUDPout[iop][2]<<16) | (ringBufferUDPout[iop][1]<<8) | ringBufferUDPout[iop][0];

                    if(ring_msg_id != 0 && bDEBUG)
                        printf("ring_msg_id:%08X msg_id:%08X\n", ring_msg_id, msg_id);

                    if(ring_msg_id == msg_id)
                    {
                        bMsg=true;

                        break;
                    }
                }

                if(!bMsg)
                {
                    if(msg_hop > 0 && msg_hop_pos > 0)
                        payload[msg_hop_pos]=msg_hop-1;

                    if ((msg_type_b_lora == 0x3A || msg_type_b_lora == 0x21 || msg_type_b_lora == 0x40))
                    {
                        // add rcvMsg to forward to LoRa TX
                        addNodeData(size, rssi, snr);

                        // add rcvMsg to BLE out Buff
                        // size message is int -> uint16_t buffer size
                        if(isPhoneReady == 1)
                        {
                            addBLEOutBuffer(RcvBuffer, size);
                        }

                        if(msg_type_b_lora == 0x3A)
                        {
                            sendDisplayText(RcvBuffer+6, size-6-3);
                        }

                        DEBUG_MSG("RADIO", "Packet resend to mesh");
                    }
                } 
            }   
        }
        else
        {
            // print hex of message
            if(bDEBUG)
                printBuffer(RcvBuffer, size);
        }

        if(bDEBUG)
            Serial.println("");

        // set buffer to 0
        memset(RcvBuffer, 0, UDP_TX_BUF_SIZE);

        //blinkLED();
    }
    else
    {
        DEBUG_MSG_VAL("RADIO", msg_id, "Packet discarded, already seen it!");


        #if BLE_TEST > 0
            char print_buff[20];
            sprintf(print_buff, "MSG %08X ACK\n", msg_id);
       		g_ble_uart.write(print_buff, strlen(print_buff));
        #else
            // ACK MSG 0x41 | 0x01020304
            uint8_t print_buff[20];
            print_buff[0]=0x41;
            print_buff[1]=msg_id & 0xFF;
            print_buff[2]=(msg_id >> 8) & 0xFF;
            print_buff[3]=(msg_id >> 16) & 0xFF;
            print_buff[4]=(msg_id >> 24) & 0xFF;
            print_buff[5]=0x00;
            addBLEOutBuffer(print_buff, 6);

            Serial.printf("ACK sent to phone ");
            printBuffer(print_buff, 6);
        #endif
    }

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Error event
 */

void OnRxError(void)
{
    is_receiving = false;
}

/**@brief our Lora TX sequence
 */

void doTX()
{
    tx_is_active = true;

    if (iWrite != iRead && iWrite < MAX_RING)
    {
        //int irs=iRead;

        sendlng = ringBuffer[iRead][0];
        memcpy(lora_tx_buffer, ringBuffer[iRead] + 1, sendlng);

        // we can now tx the message
        if (TX_ENABLE == 1)
        {
            //digitalWrite(LED_BLUE, HIGH);

            // print tx buffer
            //printBuffer(neth.lora_tx_buffer_eth, neth.lora_tx_msg_len);

            iRead++;
            if (iRead >= MAX_RING)
                iRead = 0;

            // you can transmit C-string or Arduino string up to
            // 256 characters long
            // NOTE: transmit() is a blocking method!
            //       See example SX127x_Transmit_Interrupt for details
            //       on non-blocking transmission method.
            transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);

            if (iWrite == iRead)
            {
                DEBUG_MSG_VAL("RADIO", iRead,  "TX (LAST) :");
            }
            else
            {
                DEBUG_MSG_VAL("RADIO", iRead, "TX :");
            }
            
            if(bDEBUG)
                Serial.println("");
        }
        else
        {
            DEBUG_MSG("RADIO", "TX DISABLED");
        }
    }

    tx_is_active = false;

    // wait for a second before transmitting again
    // delay(1000);
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
    //Radio.Rx(RX_TIMEOUT_VALUE);
    tx_is_active = false;
    
    //digitalWrite(LED_BLUE, LOW);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
    // DEBUG_MSG("RADIO", "OnTxTimeout");
    tx_is_active = false;
    
    //digitalWrite(LED_BLUE, LOW);
    
    //Radio.Rx(RX_TIMEOUT_VALUE);
}


/* @brief Method to send incoming LoRa messages to BLE connected device
 */
void sendToPhone()
{
    if(ble_busy_flag)
        return;

    ble_busy_flag = true;

    // we need to insert the first byte text msg flag
    uint8_t toPhoneBuff [MAX_MSG_LEN_PHONE] = {0};

    size_t blelen = BLEtoPhoneBuff[toPhoneRead][0];   //len ist um ein byte zu kurz

    toPhoneBuff[0] = 0x40;

    memcpy(toPhoneBuff+1, BLEtoPhoneBuff[toPhoneRead]+1, blelen);

if(g_ble_uart_is_connected && isPhoneReady == 1)
{

#if BLE_TEST > 0
    size_t tlen=0;
    for(int i=9; i<blelen+3; i++)
    {
        if(toPhoneBuff[i] == 0x00)
            break;
        
        toPhoneBuff[i-9]=toPhoneBuff[i];
        toPhoneBuff[i-8]=0x0a;
        toPhoneBuff[i-7]=0x00;
        tlen++;
    }
    tlen++;
    tlen++;

	// send to phone
    pTxCharacteristic->setValue(toPhoneBuff, tlen);
    pTxCharacteristic->notify();
#else
    // send to phone
    blelen=blelen+2;
    pTxCharacteristic->setValue(toPhoneBuff, blelen);
    pTxCharacteristic->notify();
#endif

}
    toPhoneRead++;
    if (toPhoneRead >= MAX_RING)
        toPhoneRead = 0;

if(g_ble_uart_is_connected && isPhoneReady == 1)
{
    if (toPhoneWrite == toPhoneRead)
    {
        DEBUG_MSG_VAL("BLE", toPhoneRead,"TX (LAST) :");
    }
    else
    {
        DEBUG_MSG_VAL("BLE", toPhoneRead,"TX :");
    }

    Serial.println("");
}
    ble_busy_flag = false;
}

void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text)
{
	// display bme680 sensor data on OLED
	if(bClear)
    {
        u8g2.clearBuffer();					// clear the internal memory
    }
	
    u8g2.setFont(u8g2_font_6x10_mf);    // u8g2_font_ncenB10_tr); // choose a suitable font

	if(memcmp(text, "L", 1) == 0)
    {
        //Serial.println("line");
    	u8g2.drawHLine(3, 16, 120);
    }
    else
    {
        //Serial.println(text);
        u8g2.drawUTF8(x, y, text);
    }

    if(bTransfer)
    {
        //Serial.println("Transfer");
	    u8g2.sendBuffer(); // transfer internal memory to the display
    }
}

void sendDisplayHead()
{
    sprintf(msg_text, "MeshCom 4.0     %3d%%", 100);  // (int)mv_to_percent(read_batt()));

    sendDisplay1306(true, false, 3, 13, msg_text);
    sendDisplay1306(false, false, 3, 15, (char*)"L");

    sprintf(msg_text, "Call:  %s", g_meshcom_settings.node_call);
    sendDisplay1306(false, false, 3, 27, msg_text);

    sprintf(msg_text, "Short: %s", g_meshcom_settings.node_short);
    sendDisplay1306(false, false, 3, 39, msg_text);

    sprintf(msg_text, "MAC:   %08X", _GW_ID);
    sendDisplay1306(false, false, 3, 51, msg_text);

    sprintf(msg_text, "Modul: %i", MODUL_HARDWARE);
    sendDisplay1306(false, true, 3, 63, msg_text);
}

void sendDisplayText(uint8_t text[300], int size)
{
    int izeile=13;

    bool bClear=true;

    for(int itxt=0; itxt<size; itxt=itxt+20)
    {
        sprintf(msg_text, "%-20.20s", text+itxt);
        
        if(size-itxt < 20)
            sendDisplay1306(bClear, true, 3, izeile, msg_text);
        else
            sendDisplay1306(bClear, false, 3, izeile, msg_text);
        
        izeile=izeile+12;

        bClear=false;
    }
}

/** @brief Method to print our buffers
 */
void printBuffer(uint8_t *buffer, int len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.printf("%02X ", buffer[i]);
  }
  Serial.println("");
}

/** @brief Method to print our buffers
 */
void printBuffer_ascii(uint8_t *buffer, int len)
{
  int i=0;

  Serial.printf("%03i ", len);

  Serial.printf("%c x", buffer[0]);
  for (i = 4; i > 0; i--)
  {
    Serial.printf("%02X", buffer[i]);
  }
  Serial.printf(" %02X ", buffer[5]);

  int ineg=0;

  for (i = 6; i < len; i++)
  {
    if(buffer[i] == 0x00)
    {
        ineg=i;
        break;
    }

    Serial.printf("%c", buffer[i]);
  }

  Serial.printf(" %02X", buffer[ineg]);
  Serial.printf("%02X", buffer[ineg+1]);

  Serial.println("");
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
                    }
                }

                if(strText.startsWith(":"))
                    sendMessage(msg_buffer, inext);

                if(strText.startsWith("-"))
                    commandAction(msg_buffer, inext, false);

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

/**@brief Function for analytical direction.
 */
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") != -1)
    {
        direction_E_W = 0;
    }
    else
    {
        direction_E_W = 1;
    }
    
    if (tmp.indexOf(",S,") != -1)
    {
        direction_S_N = 0;
    }
    else
    {
        direction_S_N = 1;
    }
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
void getGPS(void)
{ 
    bool newData = false;
  
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        tmp_data += c;
        if (gps.encode(c))// Did a new valid sentence come in?
          newData = true;
      }
    }

    direction_parse(tmp_data);
    tmp_data = "";
    float flat, flon;
    
    if (newData)
    {
        unsigned long age;  
        gps.f_get_position(&flat, &flon, &age);
        //flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
        //flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;

        g_meshcom_settings.node_lat = flat;
        g_meshcom_settings.node_lon = flon;

        if(direction_S_N == 0)
        {
        g_meshcom_settings.node_lat_c = 'S';
        }
        else
        {
        g_meshcom_settings.node_lat_c = 'N';
        }

        if(direction_E_W == 0)
        {
        g_meshcom_settings.node_lon_c = 'E';
        }
        else
        {
        g_meshcom_settings.node_lon_c = 'W';
        }

        g_meshcom_settings.node_alt = ((g_meshcom_settings.node_alt * 10) + (int)gps.f_altitude()) / 11;

        unsigned long date, time;
        gps.get_datetime(&date, &time, &g_meshcom_settings.node_age);

        g_meshcom_settings.node_date_year = date % 100;
        g_meshcom_settings.node_date_year += g_meshcom_settings.node_date_year > 80 ? 1900 : 2000;
        g_meshcom_settings.node_date_month = (date / 100) % 100;        
        g_meshcom_settings.node_date_day = date / 10000;

        g_meshcom_settings.node_date_hour = time / 1000000;
        g_meshcom_settings.node_date_minute = (time / 10000) % 100;
        g_meshcom_settings.node_date_second = (time / 100) % 100;
        g_meshcom_settings.node_date_hundredths = time % 100;

        //printf("Time: %ld\n", time);
    }
}

/**@brief Function to check if we have a Lora packet already received
 */
bool is_new_packet(uint8_t compBuffer[4])
{
    //:|01020304x

    for(int ib=0; ib < MAX_RING_UDP_OUT; ib++)
    {
        for (int i = 0; i < 4; i++)
        {
            if (memcmp(compBuffer, RcvBuffer_before[ib], 4) == 0)
            {
                if(bDEBUG)
                    Serial.printf("MSG: old one\n");
                return false;
            }
        }
    }

    if(bDEBUG)
        Serial.printf("new one\n");

    return true;
}

/**@brief Function to write our additional data into the UDP tx buffer
 * we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
 * MODE BYTE: LongSlow = 1, MediumSlow = 3
 * 8 byte offset = ID+RSSI+SNR
 */
void addNodeData(uint16_t size, int16_t rssi, int8_t snr)
{
    // copying the contents of the
    // string to char array
    uint8_t offset = 8 + strlen(g_meshcom_settings.node_call) + 1; // offset for the payload written into tx udp buffer. We add 0x00 after Longanme

    if (strlen(g_meshcom_settings.node_call) <= LONGNAME_MAXLEN)
    {
        memcpy(txBuffer, g_meshcom_settings.node_call, strlen(g_meshcom_settings.node_call));
        txBuffer[strlen(g_meshcom_settings.node_call)] = 0x00; // we add a trailing 0x00 to mark the end of longname
    }
    else
    {
        DEBUG_MSG("ERROR", "LongName is too long!");
    }

    uint8_t offset_params = strlen(g_meshcom_settings.node_call) + 1;
    memcpy(&txBuffer[offset_params], &_GW_ID, sizeof(_GW_ID));
    memcpy(&txBuffer[offset_params + 4], &rssi, sizeof(rssi));
    txBuffer[offset_params + 6] = snr;
    txBuffer[offset_params + 7] = 0x03; // manually set to 0x03 because we are on MediumSlow per default

    // now copy the rcvbuffer into txbuffer
    if ((size + 8 + offset) < UDP_TX_BUF_SIZE)
    {
        for (int i = 0; i < size; i++)
        {
            txBuffer[i + offset] = RcvBuffer[i];
        }
        // add it to the outgoing udp buffer
        // TODO change txBuffer with rinbuffer
        //DEBUG_MSG("UDP", "UDP out Buffer");
        //neth.printBuffer(txBuffer, (size + offset));

        int msg_id = (RcvBuffer[4]<<24) | (RcvBuffer[3]<<16) | (RcvBuffer[2]<<8) | RcvBuffer[1];

        addUdpOutBuffer(msg_id, txBuffer, (size + offset));
    }
    else
    {
        DEBUG_MSG("ERROR", "Exceeding Buffer length!");
    }

}

/**@brief Function adding messages into outgoing UDP ringbuffer
 * 
 */
void addUdpOutBuffer(int msg_id, uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE; // just for safety

    // byte 0-3 msg_id
    ringBufferUDPout[udpWrite][3] = msg_id >> 24;
    ringBufferUDPout[udpWrite][2] = msg_id >> 16;
    ringBufferUDPout[udpWrite][1] = msg_id >> 8;
    ringBufferUDPout[udpWrite][0] = msg_id;

    // byte 4 one byte is always the message length
    ringBufferUDPout[udpWrite][4] = len;
    memcpy(ringBufferUDPout[udpWrite] + 4 + 1, buffer, len);

    if(bDEBUG)
    {
        Serial.printf("Out Ringbuffer added element: %u\n", udpWrite);
        //DEBUG_MSG_VAL("UDP", udpWrite, "UDP Ringbuf added El.:");
        printBuffer(ringBufferUDPout[udpWrite], len);
    }

    udpWrite++;
    if (udpWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        udpWrite = 0;
}

/** @brief Function adding messages into outgoing BLE ringbuffer
 * BLE to PHONE Buffer
 */
void addBLEOutBuffer(uint8_t *buffer, uint16_t len)
{
    if (len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE; // just for safety

    //first two bytes are always the message length
    BLEtoPhoneBuff[toPhoneWrite][0] = len;
    memcpy(BLEtoPhoneBuff[toPhoneWrite] + 1, buffer, len);

    if(bDEBUG)
        Serial.printf("BLEtoPhone RingBuff added element: %u\n", toPhoneWrite);

    if(bDEBUG)
    {
        printBuffer(BLEtoPhoneBuff[toPhoneWrite], len + 1);
    }

    toPhoneWrite++;
    if (toPhoneWrite >= MAX_RING_UDP_OUT) // if the buffer is full we start at index 0 -> take care of overwriting!
        toPhoneWrite = 0;
}

void sendMessage(char *msg_text, int len)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    int inext = CallToAPRS(':', msg_buffer);
        
    if(inext == 0)
        return;

    if(memcmp(msg_text, ":", 1) == 0)
    {
        memcpy(msg_buffer+inext, msg_text+1, len-1);
        inext=inext+len-1;
    }
    else
    {
        memcpy(msg_buffer+inext, msg_text, len);
        inext=inext+len;
    }
   
    msg_buffer[inext]=0x00;
    inext++;

    int FCS_SUMME=0;
    for(int ifcs=0; ifcs<inext; ifcs++)
    {
        FCS_SUMME += msg_buffer[ifcs];
    }
    
    // FCS
    msg_buffer[inext] = (FCS_SUMME >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = FCS_SUMME & 0xFF;
    inext++;

    // _GW_ID   nur für 2.0 -> 4.0
    msg_buffer[inext] = (_GW_ID) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 16) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 24) & 0xFF;
    inext++;

    msg_buffer[inext] = MODUL_HARDWARE;
    inext++;

    // An APP als Anzeige und damit die MSG_ID bekannt ist retour senden
    if(hasMsgFromPhone)
    {
        if(bDEBUG)
            printBuffer_ascii(msg_buffer, inext);

        addBLEOutBuffer(msg_buffer, inext);
    }

    if(bDEBUG)
    {
        printBuffer(msg_buffer, inext);
        Serial.println("");
    }

    if(inext > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE;

    ringBuffer[iWrite][0]=inext;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    // store last message to compare later on
    memcpy(RcvBuffer_before[iWrite], msg_buffer+1, 4);

    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;

}

int CallToAPRS(char msg_type, uint8_t msg_buffer[MAX_MSG_LEN_PHONE])
{
    char msg_start[100];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

    msg_buffer[0]=msg_type;
    
    msg_counter=millis();

    msg_buffer[1]=msg_counter & 0xff;
    msg_buffer[2]=(msg_counter >> 8) & 0xff;
    msg_buffer[3]=(msg_counter >> 16) & 0xff;
    msg_buffer[4]=(msg_counter >> 24) & 0xff;

    msg_buffer[5]=0x05; //max hop

    sprintf(msg_start, "%s>*", g_meshcom_settings.node_call);

    memcpy(msg_buffer+6, msg_start, strlen(g_meshcom_settings.node_call)+2);

    int inext=6+2+strlen(g_meshcom_settings.node_call);

    msg_buffer[inext] = msg_type;
    inext++;

    return inext;
}

int PositionToAPRS(uint8_t msg_buffer[MAX_MSG_LEN_PHONE], bool bConvPos, bool bWeather, double lat, char lat_c, double lon, char lon_c, int alt, int batt)
{
    if(lat == 0 or lon == 0)
    {
        DEBUG_MSG("APRS", "Error PositionToAPRS");
        return 0;
    }

    char msg_start[100];

    // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00

	double slat=lat*100.0;
	double slon=lon*100.0;
	
    if(bConvPos)
    {
        double slatr=0;
        double slonr=0;
        
        slat = (int)lat;
        slatr = (lat - slat)*60.;
        slat = (slat * 100.) + slatr;
        
        slon = (int)lon;
        slonr = (lon - slon)*60.;
        slon = (slon * 100.) + slonr;
    }

    if(bWeather)
        sprintf(msg_start, "%02i%02i%02iz%07.2lf%c%c%08.2lf%c_", g_meshcom_settings.node_date_day, g_meshcom_settings.node_date_hour, g_meshcom_settings.node_date_minute, slat, lat_c, g_meshcom_settings.node_symid, slon, lon_c);
    else
        sprintf(msg_start, "%07.2lf%c%c%08.2lf%c%c %i /A=%i", slat, lat_c, g_meshcom_settings.node_symid, slon, lon_c, g_meshcom_settings.node_symcd, batt, alt);

    memcpy(msg_buffer, msg_start, strlen(msg_start));

    int inext=strlen(msg_start);
    
    return inext;
}

void sendPosition(double lat, char lat_c, double lon, char lon_c, int alt, int batt)
{
    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    int inext = CallToAPRS('!', msg_buffer);

    int inext_pos = PositionToAPRS(msg_buffer+inext, true, false, lat, lat_c, lon, lon_c, alt, batt);
        
    if(inext_pos == 0)
        return;

    inext = inext + inext_pos;

    msg_buffer[inext] = 0x00;
    inext++;

    int FCS_SUMME=0;
    for(int ifcs=0; ifcs<inext; ifcs++)
    {
        FCS_SUMME += msg_buffer[ifcs];
    }
    
    // FCS
    msg_buffer[inext] = (FCS_SUMME >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = FCS_SUMME & 0xFF;
    inext++;

    // _GW_ID   nur für 2.0 -> 4.0
    msg_buffer[inext] = (_GW_ID) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 16) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 24) & 0xFF;
    inext++;

    msg_buffer[inext] = MODUL_HARDWARE;
    inext++;

    if(bDEBUG)
    {
        printBuffer_ascii(msg_buffer, inext);
    }

    if(inext > UDP_TX_BUF_SIZE)
        inext = UDP_TX_BUF_SIZE;

    ringBuffer[iWrite][0]=inext;

    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    // store last message to compare later on
    memcpy(RcvBuffer_before[iWrite], msg_buffer+1, 4);

    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;

}

void sendWeather(double lat, char lat_c, double lon, char lon_c, int alt, float temp, float hum, float press)
{
    // @141324z4812.06N/01555.87E_270/...g...t...r000p000P...h..b10257Weather in Neulengbach

    uint8_t msg_buffer[MAX_MSG_LEN_PHONE];

    int inext = CallToAPRS('@', msg_buffer);

    int inext_pos = PositionToAPRS(msg_buffer+inext, true, true, lat, lat_c, lon, lon_c, 0, 0);
        
    if(inext_pos == 0)
        return;

    inext = inext + inext_pos;

    char msg_start[100];

    sprintf(msg_start, ".../...g...t%03ir...p...P...h%02ib%05ixMESH", (int)((temp*1.8)+32.0), (int)(hum), (int)(press*10.0));

    memcpy(msg_buffer+inext, msg_start, strlen(msg_start));

    inext=inext+strlen(msg_start);

    msg_buffer[inext] = 0x00;
    inext++;

    int FCS_SUMME=0;
    for(int ifcs=0; ifcs<inext; ifcs++)
    {
        FCS_SUMME += msg_buffer[ifcs];
    }
    
    // FCS
    msg_buffer[inext] = (FCS_SUMME >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = FCS_SUMME & 0xFF;
    inext++;

    // _GW_ID   nur für 2.0 -> 4.0
    msg_buffer[inext] = (_GW_ID) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 8) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 16) & 0xFF;
    inext++;
    msg_buffer[inext] = (_GW_ID >> 24) & 0xFF;
    inext++;

    msg_buffer[inext] = MODUL_HARDWARE;
    inext++;

    if(bDEBUG)
    {
        printBuffer_ascii(msg_buffer, inext);
    }

    ringBuffer[iWrite][0]=inext;
    memcpy(ringBuffer[iWrite]+1, msg_buffer, inext);
    
    // store last message to compare later on
    memcpy(RcvBuffer_before[iWrite], msg_buffer+1, 4);

    iWrite++;
    if(iWrite >= MAX_RING)
        iWrite=0;
}

void sendWX(char* text, float temp, float hum, float press)
{
    char msg_wx[200];

    //sprintf(msg_wx, "%s, %.1f °C, %.1f hPa, %i %%", text, temp, press, (int)(hum));
    sprintf(msg_wx, "%s, %.1f hPa, hum %i %%", text, press, (int)(hum));

    msg_wx[0] = ':';

    sendMessage(msg_wx, strlen(msg_wx));
}


void commandAction(char *msg_text, int len, bool ble)
{
    char print_buff[500];
    uint8_t buffer[500];

    // -info
    // -set-owner

    char _owner_c[MAX_CALL_LEN];
    double fVar=0.0;
    int iVar;
    String sVar;

    // copying the contents of the
    // string to char array

    bool bInfo=false;
    bool bPos=false;
    bool bWeather=false;

    if(memcmp(msg_text, "-reset", 6) == 0)
    {
        //NVIC_SystemReset();     // resets the device
    }
    else
    if(memcmp(msg_text, "-help", 5) == 0)
    {
        sprintf(print_buff, "MeshCom 4.0 Client commands\n-info     show info\n-pos      show lat/lon/alt/time info\n-weather   show temp/hum/press\n-sendpos  send pos info now\n-sendweather send weather info now\n-setowner set callsign (OE0XXX-1)\n-setlat   set latitude (44.12345)\n-setlon   set logitude (016.12345)\n-setalt   set altidude (9999)\n");

        if(ble)
        {
            memcpy(buffer, print_buff,300);
    		//SerialBT.write(buffer, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }

        return;
    }
    else
    if(memcmp(msg_text, "-info", 5) == 0)
    {
        bInfo=true;
    }
    else
    if(memcmp(msg_text, "-pos", 4) == 0)
    {
        bPos=true;
    }
    else
    if(memcmp(msg_text, "-weather", 8) == 0)
    {
        bWeather=true;
    }
    else
    if(memcmp(msg_text, "-WX", 3) == 0)
    {
        sendWX(msg_text, g_meshcom_settings.node_temp, g_meshcom_settings.node_hum, g_meshcom_settings.node_press);
        return;
    }
    else
    if(memcmp(msg_text, "-gps", 4) == 0)
    {
        getGPS();
        return;
    }
    else
    if(memcmp(msg_text, "-sendpos", 8) == 0)
    {
        sendPosition(g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt, (int)mv_to_percent(read_batt()));
        return;
    }
    else
    if(memcmp(msg_text, "-sendweather", 12) == 0)
    {
        sendWeather(g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt,
         g_meshcom_settings.node_temp, g_meshcom_settings.node_hum, g_meshcom_settings.node_press);
        return;
    }
    else
    if(memcmp(msg_text, "-setowner ", 10) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+10);
        if(_owner_c[strlen(_owner_c)-1] == 0x0a)
            _owner_c[strlen(_owner_c)-1] = 0x00;
        sVar = _owner_c;
        sVar.toUpperCase();
        sprintf(g_meshcom_settings.node_call, "%s", sVar.c_str());

        sprintf(_owner_c, "XXX40");

        for(int its=11; its<21; its++)
        {
            if(msg_text[its] == '-')
            {
                memcpy(_owner_c, msg_text+its-3, 3);
                _owner_c[3]=0x34;
                _owner_c[4]=0x30;
                _owner_c[5]=0x00;
                break;
            }
        }

        sVar = _owner_c;
        sVar.toUpperCase();
        sprintf(g_meshcom_settings.node_short, "%s", sVar.c_str());

        save_settings();

        bInfo=true;
    }
    else
    if(memcmp(msg_text, "-setlat ", 8) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+8);
        sscanf(_owner_c, "%lf", &fVar);

        //printf("_owner_c:%s fVar:%f\n", _owner_c, fVar);

        g_meshcom_settings.node_lat_c='N';
        g_meshcom_settings.node_lat=fVar;

        if(fVar < 0)
        {
            g_meshcom_settings.node_lat_c='S';
            g_meshcom_settings.node_lat=fabs(fVar);
        }

        save_settings();

        bPos=true;
    }
    else
    if(memcmp(msg_text, "-setlon ", 8) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+8);
        sscanf(_owner_c, "%lf", &fVar);

        g_meshcom_settings.node_lon=fVar;

        g_meshcom_settings.node_lon_c='E';
        g_meshcom_settings.node_lon=fVar;

        if(fVar < 0)
        {
            g_meshcom_settings.node_lon_c='W';
            g_meshcom_settings.node_lon=fabs(fVar);
        }

        save_settings();

        bPos=true;
    }
    else
    if(memcmp(msg_text, "-setalt ", 8) == 0)
    {
        sprintf(_owner_c, "%s", msg_text+8);
        sscanf(_owner_c, "%d", &iVar);

        g_meshcom_settings.node_alt=iVar;

        save_settings();

        bPos=true;
    }

    if(bInfo)
    {
        sprintf(print_buff, "MeshCom 4.0 Client\n...Call:  <%s>\n...Short: <%s>\n...ID %08X\n...MODUL %i\n...MAC %02X %02X %02X %02X %02X %02X\n...BATT %.2f mV\n...PBATT %d %%\n...TIME %li ms\n",
                g_meshcom_settings.node_call, g_meshcom_settings.node_short, _GW_ID, MODUL_HARDWARE, dmac[0], dmac[1], dmac[2], dmac[3], dmac[4], dmac[5], read_batt(), mv_to_percent(read_batt()), millis());

        if(ble)
        {
            memcpy(buffer, print_buff,300);
    		//SerialBT.write(buffer, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }

        sendDisplayHead();
    }
    else
    if(bPos)
    {
        sprintf(print_buff, "MeshCom 4.0 Client\n...LAT: %.6lf %c\n...LON: %.6lf %c\n...ALT: %i\n...DATE: %i.%02i.%02i %02i:%02i:%02i UTC\n",
         g_meshcom_settings.node_lat, g_meshcom_settings.node_lat_c, g_meshcom_settings.node_lon, g_meshcom_settings.node_lon_c, g_meshcom_settings.node_alt,
         g_meshcom_settings.node_date_year, g_meshcom_settings.node_date_month, g_meshcom_settings.node_date_day,
         g_meshcom_settings.node_date_hour, g_meshcom_settings.node_date_minute, g_meshcom_settings.node_date_second);

        if(ble)
        {
            memcpy(buffer, print_buff,300);
    		//SerialBT.write(buffer, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }
    }
     else
    if(bWeather)
    {
        sprintf(print_buff, "MeshCom 4.0 Client\n...TEMP: %.2f °C\n...HUM: %.2f%% rH\n...PRESS: %.2f hPa\n",
         g_meshcom_settings.node_temp, g_meshcom_settings.node_hum, g_meshcom_settings.node_press);

        if(ble)
        {
            memcpy(buffer, print_buff,300);
    		//SerialBT.write(buffer, strlen(print_buff));
        }
        else
        {
            printf("\n%s", print_buff);
        }
    }
    else
        printf("\nMeshCom 4.0 Client\n...wrong command %s\n", msg_text);
}
