// Functions controlling power and reset for  "Wireless Paper" All-in-one board

#include "Platforms/platforms.h"

#ifdef Vision_Master_E213

namespace Platform {

    // Enable power to Wireless Paper's interfaces (Display + LoRa)
    void VExtOn() {
        pinMode(PIN_PCB_VEXT, OUTPUT);          // OUTPUT, incase this is the first call
        
        if (digitalRead(PIN_PCB_VEXT) != VEXT_ACTIVE) {     // Read, to avoid waiting unnecessariy for power to stabilize
            digitalWrite(PIN_PCB_VEXT, VEXT_ACTIVE);        // Power on (Active HIGH)

            uint32_t start = millis();              // Non-blocking wait
            while (millis() - start < 50)           // 50 ms
                yield();
        }
    }

    // Remove power from Wireless Paper's interfaces (Display + I2C quick connector)
    void VExtOff() {
        pinMode(PIN_PCB_VEXT, OUTPUT);
        digitalWrite(PIN_PCB_VEXT, !VEXT_ACTIVE);   // ACTIVE HIGH
    }

    void toggleResetPin() {
        pinMode(PIN_DISPLAY_RST, OUTPUT);
        digitalWrite(PIN_DISPLAY_RST, LOW);

        uint32_t start = millis();              // Non-blocking wait for reset
        while (millis() - start < 10)           // 10 ms
            yield();

        digitalWrite(PIN_DISPLAY_RST, HIGH);
    }

    // SX1262 in den SLEEP-Modus (Software-SPI, damit wir die Pins "uebernehmen" koennen).
    // 1:1 von der Wireless Paper portiert - gleiche LoRa-Pins (HT-RA62-Modul). Idempotent
    // (prepareToSleep() ruft sie erneut auf). Spart vor dem Deepsleep den RX-Strom (~mehrere mA).
    void loraToSleep() {

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
        shiftOut(PIN_LORA_MOSI, PIN_LORA_SCK, MSBFIRST, 0x04);  // Parameter: maintain chip config while sleeping

        // CS HIGH - all done
        digitalWrite(PIN_LORA_NSS, HIGH);
    }

    // Board + Peripherie fuer den Deepsleep konfigurieren (Ziel ~18uA). 1:1 von der Wireless
    // Paper portiert - alle verwendeten Pin-Makros sind im E213-Header definiert.
    void prepareToSleep() {

        // SX1262 -> SLEEP (Software-SPI)
        loraToSleep();

        // Display-/Peripherie-Versorgung (VEXT, active HIGH) abschalten
        VExtOff();

        // LoRa-Pins hochohmig
        pinMode(PIN_LORA_NRST, ANALOG);
        pinMode(PIN_LORA_BUSY, ANALOG);
        pinMode(PIN_LORA_SCK, ANALOG);
        pinMode(PIN_LORA_MISO, ANALOG);
        pinMode(PIN_LORA_MOSI, ANALOG);

        // LoRa CS (NSS) muss HIGH bleiben, auch waehrend des Deepsleep
        pinMode(PIN_LORA_NSS, OUTPUT);
        digitalWrite(PIN_LORA_NSS, HIGH);
        gpio_hold_en((gpio_num_t) PIN_LORA_NSS);    // "stay where you're told"

        #if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
            esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
        #endif
    }

}   // End of namespace

#endif