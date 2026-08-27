#include "configuration.h"

// ESP-IDF ADC calibration API (esp_adc_cal.h) and analogReadMilliVolts() are
// ESP32-Arduino-core-only. The previous `#ifndef BOARD_RAK4630` guard only
// worked for wiscore_rak4631 by coincidence of a platformio.ini section-name
// collision (see Wave 0.2 notes) that leaked BOARD_RAK4630 into every other
// nRF52 variant's build; on a correctly isolated build it left heltec_t114
// and t_echo trying to compile ESP-IDF headers. loop_ADCFunctions() is only
// ever called from src/esp32/esp32_main.cpp, so gating on ESP32 directly
// keeps every board's compiled behavior identical to before.
#if defined(ESP32)

#include "loop_functions_extern.h"

#include "loop_functions.h"
#include <command_functions.h>

#include <adc_functions.h>
#include <esp_adc_cal.h>

// ANALOG values
unsigned long analog_oversample_timer = 0;

// ADC-filtering variables
uint16_t ADCraw = 0;
uint32_t ADCmV = 0;
float raw = 0;
float ADCalpha = 0.1;
float ADCexp1 = 0.0;
float ADCexp1pre = 0.0;
float ADCexp12 = 0.0;
float ADCexp12pre = 0.0;
float ADCexp2 = 0.0;


// ADC general variables
unsigned long analog_show_timer = 0;
float ADCslope = 1.0;
float ADCoffset = 0;
uint16_t SampleCount = 0;

///
/**
 * @brief ### [OE3WAS] Smoothing of ADC values version 2
 * @brief #### --analogset = Abfrage der Paramter
 * @brief #### --analog check {on|off} = Serielle Ausgabe ON/OFF {"ACK"}
 * @brief #### --analog filter {on|off} {"AFL"}
 * @param node_analog_pin ADC-GPIO               [--analog GPIO {0..99} (aber nur bestimmte GPIO gültig!)] {"APN"}
 * @param ADCatten {0..3} Abschwächer intern     [--analog atten {0|1|2|3}] {"ADCAT"}
 * @param node_analog_faktor Kalibrierungsfaktor [--analog factor 99.999] {"AFC"}
 * @param ADCslope 9.999 (1.0 default)           [--analog slope 9.999] {"ADCSL"}
 * @param ADCoffset 999 [mV] (0.0 default)       [--analog offset 999 [mV]] {"ADCOF"}
 * @param node_analog_alpha float xxxx.xxx       [--analog alpha 9999.999 (Kombinationswert für mehrere Parameter)] {"AK"}
 * @param = ADCalpha     xxxx.999 [.001 .. .999] (=Filter-Koeffizient)
 * @param = ADCintervall xx99.xxx [02 .. 99 ms]  (=Sampling Intervall)
 * @param = ADCshowtime  99xx.xxx [01 .. 99 s]   (=Ausgabe Intervall)
 * @return Die Ausgabe ist vorbereitet für .csv Weiterverarbeitung:
 * @return - [ADC1]; GPIOx; Time; SampleCount; ADCalpha; ADCraw; ADCexp1; ADCexp2
 * @return - ADCalpha = 0.001 .. 0.999 [0.10 default]
 * @return - fAnalogValue {"ADC"}
 * @return - ADCraw = 0..4095 12bit-Messwert ohne Umrechnung {"ADCRAW"}
 * @return - ADCexp1 = Messwert exponentielle Glättung 1. Ordnung {"ADCE1"}
 * @return - ADCexp2 = Messwert exponentielle Glättung 2. Ordnung {"ADCE2"}
**/
void loop_ADCFunctions()
{    
    #if defined (ANALOG_PIN)
        if(bAnalogCheck)
        {
            // bAnalogFilter noch irgendwie sinnvoll?
            ADCslope = meshcom_settings.node_analog_slope;
            if (ADCslope == 0.0) ADCslope = 1.0;
            ADCoffset = meshcom_settings.node_analog_offset;
            ADCalpha = meshcom_settings.node_analog_alpha - (int)meshcom_settings.node_analog_alpha; // 0.001 .. 0.999
            if (ADCalpha == 0.0) { ADCalpha = 0.001; }

            int ADCintervall = (int)meshcom_settings.node_analog_alpha % 100;
            if ((uint32_t)(millis() - analog_oversample_timer) >= (uint32_t)std::max(2,ADCintervall))  //min. 2ms, max. 99ms
            {
                //digitalWrite(BOARD_LED, LOW);  // OE3WAS für TEST Timing
                //digitalWrite(BOARD_LED, HIGH);  // OE3WAS für TEST
                //digitalWrite(BOARD_LED, LOW);  // OE3WAS für TEST                

                ADCraw = analogReadRaw(meshcom_settings.node_analog_pin);
                ADCmV = analogReadMilliVolts(meshcom_settings.node_analog_pin);  //inkl. Vref, Atten, Characteristic
                SampleCount++;
                // 12bit value [0 .. 4095] als default angenommen, muss ggf angepasst werden
                raw = ADCslope * (float)ADCmV * meshcom_settings.node_analog_faktor + ADCoffset; // Faktor & Offset & Slope
                if (ADCexp1pre==0) {ADCexp1pre = raw;}  //langsamen Start beschleunigen
                if (ADCexp12pre==0) {ADCexp12pre = raw;}

                // Glättung berechnen
                ADCexp1 = ADCalpha * raw + (1.0-ADCalpha) * ADCexp1pre;
                ADCexp12 = ADCalpha * ADCexp1 + (1.0-ADCalpha) * ADCexp12pre;
                ADCexp2 = ((2.0-ADCalpha) * ADCexp1 - ADCexp12) / (1.0-ADCalpha);

                ADCexp1pre = ADCexp1;
                ADCexp12pre = ADCexp12;
                analog_oversample_timer = millis();

                //digitalWrite(BOARD_LED, HIGH);  // OE3WAS für TEST
            }

            int ADCshowtime = (int)meshcom_settings.node_analog_alpha / 100;
            if ((uint32_t)(millis() - analog_show_timer) >= (uint32_t)(1000 * std::max(1,ADCshowtime)))  // 1 .. 99s
            {
                Serial.printf("[ADC1]; GPIO%d; %s; %u; %.3f; %u; %u; %.1f; %.1f; %.1f\n",
                    meshcom_settings.node_analog_pin, getTimeString().c_str(),
                    SampleCount, ADCalpha, ADCraw, ADCmV, raw, ADCexp1, ADCexp2);
                analog_show_timer = millis();
                SampleCount = 0;

                fAnalogValue = ADCexp2 / 1000.0;

                commandAction((char*)"--analogset", isPhoneReady, true);  // BLE-Übertragung vom TYP = AN
            }
        }
    #endif
}

#endif