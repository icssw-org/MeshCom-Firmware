#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>               
#include "SSD1306Wire.h"
#include <SPI.h>
#include <configuration.h>
#include <WiFi.h>

#include <U8g2lib.h> // Click to install library: http://librarymanager/All#u8g2
#include "BluetoothSerial.h"
#include "esp_adc_cal.h"

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



// RadioModule SX1278 
SX1278 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
// SyncWord - 1byte for SX127x, 2 bytes for SX126x
// Check which chip is used !!!

/** Set the device name, max length is 10 characters */
char g_ble_dev_name[10] = "RAK-CL40";

int vref = 1100;                //Default value. We'll get a more accurate value from the efuses
uint64_t timeStamp = 0;

BluetoothSerial SerialBT;
const char *pin = "1234";

// BLE Ringbuffer to phone
unsigned char BLEtoPhoneBuff[MAX_RING][UDP_TX_BUF_SIZE];
int toPhoneWrite=0;
int toPhoneRead=0;

// Client basic variables
unsigned int _GW_ID = 0x12345678; // ID of our Node

char msg_text[MAX_MSG_LEN_PHONE];

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Display 128 x 64 px
void sendDisplay1306(bool bClear, bool bTransfer, int x, int y, char *text);
void sendDisplayHead();
void sendDisplayText(uint8_t *text, int size);

bool bInitDisplay = true;

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
/*
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
  if (radio.setGain(0) == RADIOLIB_ERR_INVALID_GAIN) {
    Serial.println(F("Selected gain is invalid for this module!"));
    while (true);
  }

  Serial.println(F("All settings successfully changed!"));
*/

    _GW_ID = getMacAddr();

    Wire.begin();
    u8g2.begin();

    SerialBT.begin("ESP32test"); //Bluetooth device name

    adcAttachPin(ADC_PIN);
    
    analogRead(ADC_PIN);
    
    analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        SerialBT.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        SerialBT.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        SerialBT.println("Default Vref: 1100mV");
    }
    SerialBT.println();


    Serial.println("CLIENT STARTED");

}

void esp32loop()
{
    if(bInitDisplay)
    {
      Serial.println(F("sendDisplayHead()"));
        //mv_to_percent(read_batt());

      sendDisplayHead();

      bInitDisplay=false;
    }

    showVoltage();

    delay(250);
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

    sprintf(msg_text, "Call:  %s", (char*)"OE1KBC-88"); //g_meshcom_settings.node_call);
    sendDisplay1306(false, false, 3, 27, msg_text);

    sprintf(msg_text, "Short: %s", (char*)"KBC88"); //g_meshcom_settings.node_short);
    sendDisplay1306(false, false, 3, 40, msg_text);

    sprintf(msg_text, "MAC:   %08X", _GW_ID);
    sendDisplay1306(false, true, 3, 53, msg_text);
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
        
        izeile=izeile+13;

        bClear=false;
    }
}
