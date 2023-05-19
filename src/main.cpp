#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Wire.h>               
#include "SSD1306Wire.h"
#include <SPI.h>
#include <configuration.h>

#ifdef ESP32
  #include <esp32/esp32_main.h>
#endif

#ifdef RAK4630
  #include <nrf52/nrf52_main.h>
#endif

void setup() {

  Serial.begin(MONITOR_SPEED);
  SPI.begin();

  #ifdef RAK4630
    Serial.println("=====================================");
    Serial.println("CLIENT STARTED");
    Serial.println("=====================================");

   nrf52setup();
  #endif

  #ifdef ESP32
    esp32setup();
  #endif

}

void loop() {

  #ifdef RAK4630
    nrf52loop();
  #endif

  #ifdef ESP32
    esp32loop();
  #endif
  
}