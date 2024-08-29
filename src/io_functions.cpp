#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>

//#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>

// uncomment appropriate line
//Adafruit_MCP23X08 mcp;
Adafruit_MCP23X17 mcp;

#include "io_functions.h"

#include "io_functions.h"

uint8_t status_0 = 0xFF;

bool bMCP23017=false;

void setupMCP23017()
{
    if (!mcp.begin_I2C(0x20))
    {
        Serial.println("[INIT]...MCP23017 not found");
        bMCP23017=false;
        return;
    }

    // configure pin for input with pull up
    uint16_t t_io = meshcom_settings.node_mcp17io;

    for(int io=0;io<16;io++)
    {
        if((t_io & 0x0001) == 0x0001)
        {
            mcp.pinMode(io, OUTPUT);
        }
        else
        {
            mcp.pinMode(io, INPUT_PULLUP);
        }

        t_io >>= 1;
    }

    Serial.printf("[INIT]...MCP23017 set [%04X] out[%04X]\n", meshcom_settings.node_mcp17io, meshcom_settings.node_mcp17out);

    bMCP23017=true;
}

bool loopMCP23017()
{
    if(!bMCP23017)
        return false;

    uint16_t t_io = meshcom_settings.node_mcp17io;
    uint16_t t_in = 0;
    uint16_t t_out = meshcom_settings.node_mcp17out;
    uint16_t t_t = 0;

    //Serial.printf("[LOOP]...MCP23017 out [%04X] check [%04X]\n", meshcom_settings.node_mcp17out, meshcom_settings.node_mcp17io);

    for(int io=0;io<16;io++)
    {
        if((t_io & 0x0001) == 0x0001)
        {
            mcp.digitalWrite(io, t_out & 0x0001);

            //Serial.printf("oloop %i t_out:%04X\n", io, t_out);
        }
        else
        {
            t_t = mcp.digitalRead(io);
            if((t_t & 0x01) == 0x01)
                t_in = t_in | 0x8000;
            
            //Serial.printf("iloop %i t_in:%04X read:%02X\n", io, t_in, mcp.digitalRead(io));
        }

        if(io < 15)
        {
            t_io >>= 1;
            t_out >>= 1;
            t_in >>= 1;
        }

        delay(20);
    }

    if(t_in != meshcom_settings.node_mcp17in)
        Serial.printf("MCP23017 BITS: %04X\n", t_in);

    meshcom_settings.node_mcp17in = t_in;

    return true;
}
