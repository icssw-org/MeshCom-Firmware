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

}   // End of namespace

#endif