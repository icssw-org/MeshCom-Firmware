// Functions controlling power and reset for  "Wireless Paper" All-in-one board

#include "Platforms/platforms.h"

#ifdef WIRELESS_PAPER

namespace Platform {

    // Enable power to Wireless Paper's interfaces (Display + LoRa P/A)
    void VExtOn() {
        pinMode(PIN_PCB_VEXT, OUTPUT);          // OUTPUT, incase this is the first call
        
        if (digitalRead(PIN_PCB_VEXT) != VEXT_ACTIVE) {     // Read, to avoid waiting unnecessariy for power to stabilize
            digitalWrite(PIN_PCB_VEXT, VEXT_ACTIVE);        // Power on (Active LOW)

            uint32_t start = millis();              // Non-blocking wait
            while (millis() - start < 50)           // 50 ms
                yield();
        }
    }

    // Remove power from Wireless Paper's interfaces (Display + LoRa)
    void VExtOff() {
        pinMode(PIN_PCB_VEXT, OUTPUT);
        digitalWrite(PIN_PCB_VEXT, !VEXT_ACTIVE);   // ACTIVE LOW
    }

    void toggleResetPin() {
        pinMode(PIN_DISPLAY_RST, OUTPUT);
        digitalWrite(PIN_DISPLAY_RST, LOW);
        
        uint32_t start = millis();              // Non-blocking wait for reset
        while (millis() - start < 10)           // 10 ms
            yield();

        digitalWrite(PIN_DISPLAY_RST, HIGH);
    }

    void prepareToSleep() {

        // Set SX1262 to SLEEP mode - Software SPI so we can "trample" stuff
        // -----------------------------------------------------------------

        // Default pin states
        digitalWrite(PIN_LORA_NSS, HIGH);
        digitalWrite(PIN_LORA_SCK, LOW);        // Mode 0 - Idle Low
        digitalWrite(PIN_LORA_MOSI, LOW);

        // Set pin modes
        pinMode(PIN_LORA_NSS, OUTPUT);
        pinMode(PIN_LORA_SCK, OUTPUT);
        pinMode(PIN_LORA_MOSI, OUTPUT);

        // CS LOW
        digitalWrite(PIN_LORA_NSS, LOW); 

        // TX data and params
        shiftOut(PIN_LORA_MOSI, PIN_LORA_SCK, MSBFIRST, 0x84);  // Command: Enter SLEEP mode
        shiftOut(PIN_LORA_MOSI, PIN_LORA_SCK, MSBFIRST, 0x04);  // Parameter: sleepConfig - maintain chip config while sleeping

        // CS HIGH - all done
        digitalWrite(PIN_LORA_NSS, HIGH);


        // Set the GPIOs for sleep
        // ------------------------

        VExtOff();

        // LoRa pins to high-impedence
        pinMode(PIN_LORA_NRST, ANALOG);
        pinMode(PIN_LORA_BUSY, ANALOG);
        pinMode(PIN_LORA_SCK, ANALOG);
        pinMode(PIN_LORA_MISO, ANALOG);
        pinMode(PIN_LORA_MOSI, ANALOG);

        // LoRa CS (RADIO_NSS) needs to stay HIGH, even during deep sleep
        pinMode(PIN_LORA_NSS, OUTPUT);
        digitalWrite(PIN_LORA_NSS, HIGH);
        gpio_hold_en((gpio_num_t) PIN_LORA_NSS);    // "stay where you're told"


        #if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3   // Broken in Heltec ESP32 Dev Enviornment 3.0.0 (IDF5)
            // Set what exactly should be powered down
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
        #endif


        // User: please set the time and start sleep:
        // -----------------------------------------
        
        // const uint8_t sleep_seconds = 10;
        // esp_sleep_enable_timer_wakeup((uint64_t) 1000 * 1000 * sleep_seconds);
        // esp_deep_sleep_start()
    }

}   // End of namespace

#endif