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
#include <RadioLib.h>

#include <Wire.h>               
#include "SSD1306Wire.h"
#include <SPI.h>
#include <WiFi.h>

#include "esp32_gps.h"

// MeshCom Common (ers32/nrf52) Funktions
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <command_functions.h>
#include <phone_commands.h>
#include <aprs_functions.h>
#include <batt_functions.h>
#include <lora_functions.h>

#include <esp_adc_cal.h>

#ifdef BOARD_HELTEC
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#endif

#define ADC_PIN 35  //ADC_PIN is the ADC pin the battery is connected to through a voltage divider

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
 * LoRa Sync Word für Meshcom definieren! 2b = Meshtastic
 * SX127x Chips haben nur ein Byte als Syncowrd, SX126x haben zwei Byte
 * Alle Funktionen Setter, Getter, etc finden sich in den korrspondierenden Libraries
 * SX1278 zB: https://github.com/jgromes/RadioLib/blob/master/src/modules/SX127x/SX1278.h
*/

bool bInitDisplay = true;

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
extern bool ble_busy_flag;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint32_t PIN = 000000;             // pairing password PIN Passwort PIN-Code Kennwort

// Bluetooth UUIDs are standardized. For more info: https://www.bluetooth.com/specifications/assigned-numbers/
// Nordic UUID DB is here: https://github.com/NordicSemiconductor/bluetooth-numbers-database


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE disconnected");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        // Forward data from Mobile to our peripheral
        uint8_t conf_data[MAX_MSG_LEN_PHONE] = {0};
        size_t conf_length=0;

        conf_length = pCharacteristic->getDataLength(); // getLength();

        if (conf_length <= 0)
            return;

        memcpy(conf_data, pCharacteristic->getValue() , conf_length);

    	readPhoneCommand(conf_data);
    }
};


#ifdef SX127X
// RadioModule SX1278 
SX1278 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
// SyncWord - 1byte for SX127x, 2 bytes for SX126x
// Check which chip is used !!!
#endif

#ifdef SX126X
    // RadioModule SX1268 
    // cs - irq - reset - interrupt gpio
    // If you have RESET of the E22 connected to a GPIO on the ESP you must initialize the GPIO as output and perform a LOW - HIGH cycle, 
    // otherwise your E22 is in an undefined state. RESET can be connected, but is not a must. IF so, make RESET before INIT!
    SX1268 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_DIO0);

#endif

// Lora callback Function declarations
void checkRX(void);

void setInterruptFlag();     // LoRaRX Interrupt function
void enableRX(void);    // for Modules with RXEN / TXEN Pin
void enableTX(void);    // for Modules with RXEN / TXEN Pin


asm(".global _scanf_float");
asm(".global _printf_float");

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_UNKNOWN;

// LoRa Events and Buffers

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "CL";

int vref = 1100;                //Default value. We'll get a more accurate value from the efuses
uint64_t timeStamp = 0;

/** Flag if BLE UART is connected */
bool g_ble_uart_is_connected = false;

// Client basic variables
uint8_t dmac[6] = {0};

unsigned long gps_refresh_timer = 0;

bool is_new_packet(uint8_t compBuffer[4]);     // switch if we have a packet received we never saw before RcvBuffer[12] changes, rest is same
void checkSerialCommand(void);

bool g_meshcom_initialized;
bool init_flash_done=false;

String strText="";

unsigned int  getMacAddr(void)
{
  return ESP.getEfuseMac() & 0xFFFFFFFF;
}

void showVoltage()
{
    #ifdef ADC_READ
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v2 = analogRead(ADC_PIN);
        float battery_voltage = ((float)v2 / 4095.0) * 2.0 * 3.3 * (1100 / 1000.0);
        String voltage = "Voltage :" + String(battery_voltage) + "V\n";
        Serial.println(voltage);
    }
    #endif
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

    for(int ib=0; ib<MAX_RING; ib++)
    {
        memset(own_msg_id[ib], 0x00, 5);
    }

    //clear ringbuffer
    for(int i=0; i<MAX_RING_UDP_OUT; i++)
    {
        memset(ringBufferLoraRX[i], 0, 4);
    }

    #ifdef BOARD_HELTEC
        Wire.setPins(SDA_PIN, SCL_PIN);
    #endif
    
    #ifdef BOARD_E22
        Wire.setPins(SDA_PIN, SCL_PIN);
    #endif

    Serial.begin(MONITOR_SPEED);
    while(!Serial);
    Serial.println("SERIAL open");

    Serial.println("============");
    Serial.println("CLIENT SETUP");
    Serial.println("============");

    #if defined(BOARD_TBEAM) || defined(BOARD_SX1268)
        setupGPS();
    #else
        Wire.begin();
    #endif

    u8g2.begin();

#ifdef BOARD_E22
    // if RESET Pin is connected
    pinMode(LORA_RST, PULLUP);
    digitalWrite(LORA_RST, LOW);
    delay(100);
    digitalWrite(LORA_RST, HIGH);

    // we have TXEN and RXEN Pin connected
    radio.setRfSwitchPins(RXEN, TXEN);
#endif

    // initialize SX12xx with default settings
    Serial.print(F("LoRa Modem Initializing ... "));
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

    #ifdef SX127X
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
    #endif

    // setup for SX126x Radios
    #ifdef SX126X
    // interrupt pin
    radio.setDio1Action(setInterruptFlag);
    // if DIO2 controls the RF Switch you need to set it
    // radio.setDio2AsRfSwitch(true);
    // Important! To enable receive you need to switch the SX126x rf switch to RECEIVE 
    
    #endif

    // put module back to listen mode
    radio.startReceive();
    Serial.println("startReceive Start");

    Serial.println(F("All settings successfully changed!"));

    // Create the BLE Device
    char strBLEName[50]={0};
    sprintf(strBLEName, "%s-%02x%02x-%s", g_ble_dev_name, dmac[4], dmac[5], meshcom_settings.node_call);
    NimBLEDevice::init(strBLEName);

#ifdef ESP_PLATFORM
    NimBLEDevice::setPower(ESP_PWR_LVL_N9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif

    // Create the BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    static BLEUUID serviceUUID((uint16_t)0xF0A0);
    static BLEUUID rxdataUUID((uint16_t)0xF0A1);
    static BLEUUID txdataUUID((uint16_t)0xF0A2);

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

//#define SERVICE_UUID            "6E41"
//#define CHARACTERISTIC_UUID_RX  "6E42"
//#define CHARACTERISTIC_UUID_TX  "6E43"

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID_TX,
                                            NIMBLE_PROPERTY::NOTIFY
                                        );
                        
    pTxCharacteristic->addDescriptor(new NimBLE2904());

    NimBLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                                                CHARACTERISTIC_UUID_RX,
                                                NIMBLE_PROPERTY::READ |
                                                NIMBLE_PROPERTY::WRITE
                                            );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->addServiceUUID(pService->getUUID());
    //pServer->getAdvertising()->setScanResponse(true);
    //pServer->getAdvertising()->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    //pServer->getAdvertising()->setMinPreferred(0x12);
    pServer->getAdvertising()->start();
    
    Serial.println("Waiting a client connection to notify...");
    
    // BATT
    #ifdef ADC_READ

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
    #endif

    // reset GPS-Time parameter
    meshcom_settings.node_date_hour = 0;
    meshcom_settings.node_date_minute = 0;
    meshcom_settings.node_date_second = 0;
    meshcom_settings.node_date_hundredths = 0;

    Serial.println("==============");
    Serial.println("CLIENT STARTED");
    Serial.println("==============");

}

void esp32_write_ble(uint8_t confBuff[300], uint8_t conf_len)
{
    pTxCharacteristic->setValue(confBuff, conf_len);
    pTxCharacteristic->notify();
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

        // put module back to listen mode
        radio.startReceive();
    }

    // LORA RECEIVE
    if(receivedFlag)
    {
        if (tx_is_active == false && is_receiving == false)
        {
            checkRX();

            // put module back to listen mode
            radio.startReceive();
        }
    
        // reset flag
        receivedFlag = false;
    }

    // LORA SEND
    // check if we have messages in ringbuffer to send
    if (iWrite != iRead)
    {
        if(cmd_counter <= 0)
        {
            if (tx_is_active == false && is_receiving == false)
                doTX();
        }
    }

    cmd_counter--;
    if(cmd_counter < 0)
        cmd_counter=0;

    // BLE
    if (deviceConnected)
    {
    	g_ble_uart_is_connected = true;
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
      sendDisplayHead((int)mv_to_percent(read_batt()));

      bInitDisplay=false;
    }

    // check if message from phone to send
    if(hasMsgFromPhone)
    {
        sendMessage(textbuff_phone, txt_msg_len_phone);

        hasMsgFromPhone = false;
    }

    // gps refresh
    if ((gps_refresh_timer + (GPS_REFRESH_INTERVAL * 1000)) < millis())
    {
        #ifdef ENABLE_GPS
            getGPS();
        #endif

        gps_refresh_timer = millis();
    }

    // POSINFO_INTERVAL in Minuten
    if ((posinfo_timer + (POSINFO_INTERVAL * 1000 * 60)) < millis())
    {
        #ifdef ENABLE_GPS
            getGPS();
        #endif

        sendPosition(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt, (int)mv_to_percent(read_batt()));

        #if defined(LPS33)
        sendWeather(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_alt,
         meshcom_settings.node_temp, meshcom_settings.node_hum, meshcom_settings.node_press);
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
    
    size_t ibytes = UDP_TX_BUF_SIZE;

    int state = radio.readData(payload, ibytes);

    if (state == RADIOLIB_ERR_NONE)
    {
        OnRxDone(payload, (uint16_t)ibytes, (int16_t)radio.getRSSI(), (int8_t)radio.getSNR());
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
