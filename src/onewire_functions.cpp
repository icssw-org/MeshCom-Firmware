//#ifndef BOARD_TLORA_OLV216

#include "loop_functions.h"
#include "loop_functions_extern.h"

#include "onewire_functions.h"

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

void init_onewire(void)
{
    if(!bONEWIRE)
        return;

    Serial.printf("[INIT]...init_onewire - GPIO:%i\n", meshcom_settings.node_owgpio);

    if(meshcom_settings.node_owgpio > 0)
        ds.begin(meshcom_settings.node_owgpio);  // default on pin 36
   
}

void loop_onewire()
{
    if(!bONEWIRE)
        return;

    if(meshcom_settings.node_owgpio == 0)
        return;

    byte i;
    byte present = 0;
    byte type_s;
    byte data[9];
    byte addr[8];
    float celsius, fahrenheit;

    if ( !ds.search(addr))
    {
        if(bWXDEBUG)
        {
            Serial.println("No more OneWire addresses.");
            Serial.println();
        }
        ds.reset_search();
        delay(250);
        return;
    }

    if(bWXDEBUG)
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
        return;
    }

    if(bWXDEBUG)
    {
        Serial.println();
    }

    // the first ROM byte indicates which chip
    switch (addr[0])
    {
        case 0x10:
            if(bWXDEBUG)
                Serial.println("  Chip = DS18S20");  // or old DS1820
            type_s = 1;
            break;
        case 0x28:
            if(bWXDEBUG)
                Serial.println("  Chip = DS18B20");
            type_s = 0;
            break;
        case 0x22:
            if(bWXDEBUG)
                Serial.println("  Chip = DS1822");
            type_s = 0;
            break;
        default:
            if(bWXDEBUG)
                Serial.println("Device is not a DS18x20 family device.");
            return;
    } 

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad

    if(bWXDEBUG)
    {
        Serial.print("  Data = ");
        Serial.print(present, HEX);
        Serial.print(" ");
    }
    for ( i = 0; i < 9; i++)
    {           // we need 9 bytes
        data[i] = ds.read();
        if(bWXDEBUG)
        {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
    }

    if(bWXDEBUG)
    {
        Serial.print(" CRC=");
        Serial.print(OneWire::crc8(data, 8), HEX);
        Serial.println();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
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

    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;

    if(bWXDEBUG)
    {
        Serial.print("  Temperature = ");
        Serial.print(celsius);
        Serial.print(" Celsius, ");
        Serial.print(fahrenheit);
        Serial.println(" Fahrenheit");
    }

    meshcom_settings.node_temp2 = celsius;
}
//#endif