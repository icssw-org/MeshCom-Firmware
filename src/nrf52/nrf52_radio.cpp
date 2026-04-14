#include "nrf52_radio.h"

#include <configuration.h>

#include <LoRaWan-Arduino.h> // includes radio/radio.h which defines Radio

extern bool bBOOSTEDGAIN;

void startRadioReceive()
{
    if (bBOOSTEDGAIN)
    {
        Radio.RxBoosted(RX_TIMEOUT_VALUE);
    }
    else
    {
        Radio.Rx(RX_TIMEOUT_VALUE);
    }
}