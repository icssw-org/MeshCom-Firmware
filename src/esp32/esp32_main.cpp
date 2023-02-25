#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>               
#include "SSD1306Wire.h"
#include <SPI.h>
#include <configuration.h>
#include <WiFi.h>

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



void esp32setup(){

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
  if (radio.setGain(0) == RADIOLIB_ERR_INVALID_GAIN) {
    Serial.println(F("Selected gain is invalid for this module!"));
    while (true);
  }

  Serial.println(F("All settings successfully changed!"));


}

void esp32loop(){
    Serial.println("Hello from ESP32");
    delay(250);
}