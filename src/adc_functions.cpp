#include "configuration.h"
#include "loop_functions_extern.h"

#include "loop_functions.h"
#include <command_functions.h>

#ifndef BOARD_RAK4630

#include <adc_functions.h>
#include <esp_adc_cal.h>

// ANALOG values
unsigned long analog_oversample_timer = 0;

// ADC-filtering variables
uint16_t ADCraw = 0;
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
 * @brief ### [OE3WAS] Smoothing of ADC values version 1
 * @brief #### --analogset = Abfrage der Paramter
 * @brief #### --analog check {on|off} = Serielle Ausgabe ON/OFF
 * @attention - noch keine Verareitung ev. verfügbarem Vref (konnte noch nicht gelesen werden)
 * @param node_analog_pin ADC-GPIO               [--analog GPIO {0..99} (aber nur bestimmte GPIO gültig!)]
 * @param ADCatten {0..3} Abschwächer intern     [--analog atten {0|1|2|3}]
 * @param node_analog_faktor Kalibrierungsfaktor [--analog factor 99.999]
 * @param ADCslope 9.999 (1.0 default)           [--analog slope 9.999]
 * @param ADCoffset 999 [mV] (0.0 default)       [--analog offset 999 [mV]]
 * @param node_analog_alpha float xxxx.xxx       [--analog alpha 9999.999 (Kombinationswert für mehrere Parameter)]
 * @param ADCalpha     xxxx.999 [.001 .. .999] (=Filter-Koeffizient)
 * @param ADCintervall xx99.xxx [02 .. 99 ms]  (=Sampling Intervall)
 * @param ADCshowtime  99xx.xxx [01 .. 99 s]   (=Ausgabe Intervall)
 * @return Die Ausgabe ist vorbereitet für .csv Weiterverarbeitung:
 * @return - [ADC1]; GPIOx; Time; SampleCount; ADCalpha; ADCraw; ADCexp1; ADCexp2
 * @return - ADCalpha = 0.001 .. 0.999 [0.10 default]
 * @return - ADCraw = 0..4095 12bit-Messwert ohne Umrechnung
 * @return - ADCexp1 = Messwert exponentielle Glättung 1. Ordnung
 * @return - ADCexp2 = Messwert exponentielle Glättung 2. Ordnung
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
            int ADCatten = (int)meshcom_settings.node_analog_atten;
            float Atten = 1.0;
            switch (ADCatten) {
                case ADC_ATTEN_DB_0:   { Atten = 1.0000; break; }
                case ADC_ATTEN_DB_2_5: { Atten = 1.3335; break; }
                case ADC_ATTEN_DB_6:   { Atten = 1.9953; break; }
                case ADC_ATTEN_DB_12:  { Atten = 3.9811; break; }
            }
            int ADCintervall = (int)meshcom_settings.node_analog_alpha % 100;
            if ((analog_oversample_timer + std::max(2,ADCintervall)) < millis())  //min. 2ms, max. 99ms
            {
                //digitalWrite(BOARD_LED, LOW);  // OE3WAS für TEST Timing
                //digitalWrite(BOARD_LED, HIGH);  // OE3WAS für TEST
                //digitalWrite(BOARD_LED, LOW);  // OE3WAS für TEST

                // optimal wäre es, den Wert von Vref aus der EFUSE auslesen zu können
                // aber nicht jeder ESP32/ESP32-S3 enthält so einen Wert
                //KORR DL1MX  float vref = 1100.0;  // bereich 1000 .. 1200 [mV]
                #if defined(BOARD_E22)
                    float vref = 3300.0; // ESP32 DevKitC v4 has 3.3 V
                #else
                    float vref = 1100.0; // bereich 1000 .. 1200 [mV]
                #endif
                
                ADCalpha = meshcom_settings.node_analog_alpha - (int)meshcom_settings.node_analog_alpha; // 0.001 .. 0.999

                ADCraw = analogReadRaw(meshcom_settings.node_analog_pin);
                SampleCount++;
                // 12bit value [0 .. 4095] als default angenommen, muss ggf angepasst werden
                float raw = (float)ADCraw * (vref / 4095.0) * meshcom_settings.node_analog_faktor;  // raw Wert [mV] nach Kalibrierung
                raw *= Atten;                           // Abschwächer berücksichtigen
                raw = ADCslope * raw + ADCoffset;       // hier jetzt Offset & Slope verarbeiten
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
            if ((analog_show_timer + (1000 * std::max(1,ADCshowtime))) < millis())  // 1 .. 99s
            {
                Serial.printf("[ADC1]; GPIO%d; %s; %u; %.3f; %u; %.1f; %.1f\n",
                    meshcom_settings.node_analog_pin, getTimeString().c_str(),
                    SampleCount, ADCalpha, ADCraw, ADCexp1, ADCexp2);
                analog_show_timer = millis();
                SampleCount = 0;

                fAnalogValue = ADCexp2 / 1000.0;

                commandAction((char*)"--analogset", isPhoneReady, true);  // BLE-Übertragung vom TYP = AN
            }
        }
    #endif
}

#endif