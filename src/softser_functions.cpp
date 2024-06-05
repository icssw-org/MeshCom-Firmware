#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>

#if defined(ENABLE_SOFTSER)

#include <softser_functions.h>

#include "SoftwareSerial.h"

SoftwareSerial SOFTSER;

bool setupSOFTSER()
{  
    if(!bSOFTSERON)
    {
        return false;
    }

    if(meshcom_settings.node_ss_baud == 0)
    {
        return false;
    }

    SOFTSER.begin((uint32_t)meshcom_settings.node_ss_baud, EspSoftwareSerial::SWSERIAL_8N1, (int8_t)meshcom_settings.node_ss_rx_pin, (int8_t)meshcom_settings.node_ss_tx_pin);

    Serial.printf("[INIT]...SOFTSER RX:%i TX:%i BAUD:%i\n", meshcom_settings.node_ss_rx_pin, meshcom_settings.node_ss_tx_pin, meshcom_settings.node_ss_baud);

    return true;
}

bool loopSOFTSER()
{
    if(!bSOFTSERON)
    {
        return false;
    }

    if(meshcom_settings.node_ss_baud == 0)
    {
        return false;
    }

    String tmp_data = "";

    bool newData = false;

    if(bSOFTSERDEBUG)
    {
        Serial.print("-----------check SOFTSER ");
        Serial.print(getTimeString());
        Serial.println(" -----------");
    }
  
    // For one second we parse SOFTSER data and report
    //for (unsigned long start = millis(); millis() - start < 1000;)
    bool bgrun=true;
    unsigned long start = millis();
    while(bgrun)
    {
      while (SOFTSER.available())
      {
        char c = SOFTSER.read();

        if(c == 0x00)
        {
            bgrun=false;
            break;
        }

        if(bSOFTSERDEBUG && bDEBUG)
          Serial.print(c);

        tmp_data += c;
        
      }
    
      if((millis() - start) > 1000)
          bgrun=false;
    }

    if(bSOFTSERDEBUG)
    {
        Serial.printf("%s\n", tmp_data.c_str());
    }

    return true;
}

bool sendSOFTSER(char cText[100])
{
    if(!bSOFTSERON)
    {
        return false;
    }

    if(meshcom_settings.node_ss_baud == 0)
    {
        return false;
    }

    Serial.printf("writeSOFTSER <%s>\n", cText);

    SOFTSER.write(cText);

    return true;
}

#endif