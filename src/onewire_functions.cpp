#include "loop_functions.h"
#include "loop_functions_extern.h"

#include "onewire_functions.h"

byte i;
byte present = 0;
//byte type_s;
byte data[9];
byte addr[8];
float celsius, fahrenheit;

#ifdef OneWire_GPIO
//--------------------------------------------------------------------------

#include <DHT.h>
#include <DHT_U.h>

// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

uint32_t delayMS;
DHT_Unified dht(OneWire_GPIO, DHTTYPE);

void init_onewire_dht()
{
    dht_found = false;
    
    if(!bONEWIRE)
        return;

    meshcom_settings.node_temp = 0;
    meshcom_settings.node_hum = 0;
    
    #ifdef BOARD_TBEAM
        if(meshcom_settings.node_owgpio == 16)
        {
            meshcom_settings.node_owgpio = 0;
        }
    #endif

    Serial.printf("[INIT]...ONEWIRE - DHT GPIO:%i\n", meshcom_settings.node_owgpio);

    if(meshcom_settings.node_owgpio > 0)
        DHT_Unified dht(meshcom_settings.node_owgpio, DHTTYPE);
    else
    {
        #ifdef OneWire_GPIO
            meshcom_settings.node_owgpio = OneWire_GPIO;
            DHT_Unified dht(meshcom_settings.node_owgpio, DHTTYPE);
        #else
            return;
        #endif
    }

    dht.begin();

    Serial.println(F("[INIT]...DHTxx Unified Sensor found"));
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);

    if(strcmp(sensor.name, "DHT11") == 0)
    {
        dht_found = true;

        /*
        Serial.println(F("------------------------------------"));
        Serial.println(F("Temperature Sensor"));
        Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
        Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
        Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
        Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
        Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
        Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
        Serial.println(F("------------------------------------"));
        // Print humidity sensor details.
        dht.humidity().getSensor(&sensor);
        Serial.println(F("Humidity Sensor"));
        Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
        Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
        Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
        Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
        Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
        Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
        Serial.println(F("------------------------------------"));
        */

        // Set delay between sensor readings based on sensor details.
        delayMS = sensor.min_delay / 1000;
    }
}

bool loop_onewire_dht()
{
    if(!dht_found)
    {
        return false;
    }

    // Delay between measurements.
    /* defined by mainloop
    delay(delayMS);
    
    if(bWXDEBUG)
        Serial.printf("delayMS: %u\n", delayMS);
    */

    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature))
    {
        if(bWXDEBUG)
            Serial.println(F("Error reading temperature (DHT)!"));

        dht_found=false;
    }
    else
    {
        meshcom_settings.node_temp = event.temperature;

        if(bWXDEBUG)
        {
            Serial.print(F("Temperature (DHT) = "));
            Serial.print(event.temperature);
            Serial.println(F("°C"));
        }
    }

    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity))
    {
        if(bWXDEBUG)
            Serial.println(F("Error reading humidity (DHT)!"));

        dht_found = false;
    }
    else
    {
        meshcom_settings.node_hum = event.relative_humidity;

        if(bWXDEBUG)
        {
            Serial.print(F("Humidity (DHT) = "));
            Serial.print(event.relative_humidity);
            Serial.println(F("%"));
        }
    }

    return true;
}

//--------------------------------------------------------------------------

#include <OneWire.h>

unsigned long onewireTimeWait = 0;

OneWire ds;

void PrintBytes(const uint8_t* addr, uint8_t count, bool newline=false)
{
  for (uint8_t i = 0; i < count; i++) {
    Serial.print(addr[i]>>4, HEX);
    Serial.print(addr[i]&0x0f, HEX);
  }
  if (newline)
    Serial.println();
}

void init_onewire_ds18(void)
{
    one_found = false;

    if(!bONEWIRE)
        return;

    #ifdef BOARD_TBEAM
        if(meshcom_settings.node_owgpio == 16)
        {
            meshcom_settings.node_owgpio = 0;
        }
    #endif

    Serial.printf("[INIT]...ONEWIRE - DS18 GPIO:%i\n", meshcom_settings.node_owgpio);

    if(meshcom_settings.node_owgpio > 0)
    {
        ds.begin(meshcom_settings.node_owgpio);
        one_found=true;
    }
    else
    {
        #ifdef OneWire_GPIO
            meshcom_settings.node_owgpio = OneWire_GPIO;
            ds.begin(meshcom_settings.node_owgpio);
            one_found=true;
        #endif
    }
}

bool loop_onewire_ds18()
{
    if(!one_found)
    {
        meshcom_settings.node_temp2 = 0;
        return false;
    }

    if(one_start == 0)
    {
        if (!ds.search(addr))
        {
            if(bWXDEBUG && bDisplayCont)
            {
                Serial.println("No more OneWire addresses.");
            }
            ds.reset_search();
            return false;
        }

        if(bWXDEBUG && bDisplayCont)
        {
            Serial.print("ROM =");
            for( i = 0; i < 8; i++)
            {
                Serial.write(' ');
                Serial.print(addr[i], HEX);
            }
        }

        if (OneWire::crc8(addr, 7) != addr[7])
        {
            Serial.println("OneWire CRC is not valid!");
            one_found=false;
            return false;
        }

        if(bWXDEBUG && bDisplayCont)
        {
            Serial.println();
        }

        // the first ROM byte indicates which chip
        switch (addr[0])
        {
            case 0x10:
                if(bWXDEBUG && bDisplayCont)
                    Serial.println("  Chip = DS18S20");  // or old DS1820
                //type_s = 1;
                break;
            case 0x28:
                if(bWXDEBUG && bDisplayCont)
                    Serial.println("  Chip = DS18B20");
                //type_s = 0;
                break;
            case 0x22:
                if(bWXDEBUG && bDisplayCont)
                    Serial.println("  Chip = DS1822");
                //type_s = 0;
                break;
            default:
                if(bWXDEBUG && bDisplayCont)
                    Serial.println("Device is not a DS18x20 family device.");
                one_found=false;
                return false;
        } 

        ds.reset();
        ds.select(addr);
        ds.write(0x44, 1);        // start conversion, with parasite power on at the end

        //delay(1000);     // maybe 750ms is enough, maybe not
        one_start++;

        return false;
    }
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

    if(bWXDEBUG && bDisplayCont)
    {
        Serial.print("  Data = ");
        Serial.print(present, HEX);
        Serial.print(" ");
    }

    for ( i = 0; i < 9; i++)
    {
        // we need 9 bytes
        data[i] = ds.read();
        if(bWXDEBUG && bDisplayCont)
        {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
    }

    ds.reset_search();

    if(bWXDEBUG && bDisplayCont)
    {
        Serial.print(" CRC=");
        Serial.print(OneWire::crc8(data, 8), HEX);
        Serial.println();
    }

    if(OneWire::crc8(data, 8) == (uint8_t)data[8])
    {
        // Convert the data to actual temperature
        // because the result is a 16 bit signed integer, it should
        // be stored to an "int16_t" type, which is always 16 bits
        // even when compiled on a 32 bit processor.
        
        byte MSB = data[1];
        byte LSB = data[0];

        float tempRead = (int16_t)((MSB << 8) | LSB); //using two's compliment
        float TemperatureSum = tempRead / 16.0;
    
    /*
        if (type_s)
        {
            raw = raw << 3; // 9 bit resolution default
            if (data[7] == 0x10)
            {
                // "count remain" gives full 12 bit resolution
                raw = (raw & 0xFFF0) + 12 - data[6];
            }
        }
        else
        {
            byte cfg = (data[4] & 0x60);
            // at lower res, the low bits are undefined, so let's zero them
            if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
            else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
            else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
            //// default is 12 bit resolution, 750 ms conversion time
        }
    */
        celsius = TemperatureSum + meshcom_settings.node_tempo_off;
        fahrenheit = celsius * 1.8 + 32.0;

        if(bWXDEBUG)
        {
            Serial.print("Temperature (DS18) = ");
            Serial.print(celsius);
            Serial.print(" Celsius, ");
            Serial.print(fahrenheit);
            Serial.println(" Fahrenheit");
        }

        meshcom_settings.node_temp2 = celsius;
    }
    else
    {
        if(bWXDEBUG)
        {
            Serial.println("Temperature (DS18) = CRC ERROR");
        }
    }

    return true;
}
//--------------------------------------------------------------------------
#endif