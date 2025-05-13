#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"
#include <clock.h>

#include <Arduino.h>

#if defined(ENABLE_SOFTSER)

#include <softser_functions.h>
#include <tinyxml_functions.h>

#ifdef ESP32
#include "SoftwareSerial.h"
SoftwareSerial SOFTSER;
#else
#include "Wire.h"
#endif

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

#ifdef ESP32
    SOFTSER.begin((uint32_t)meshcom_settings.node_ss_baud, EspSoftwareSerial::SWSERIAL_8N1, (int8_t)meshcom_settings.node_ss_rx_pin, (int8_t)meshcom_settings.node_ss_tx_pin);
    SOFTSER.setTimeout(50);
#else
    Serial1.begin((unsigned long)meshcom_settings.node_ss_baud);
    Serial1.setTimeout(50);
#endif

    Serial.printf("[INIT]...SOFTSER RX:%i TX:%i BAUD:%i\n", meshcom_settings.node_ss_rx_pin, meshcom_settings.node_ss_tx_pin, meshcom_settings.node_ss_baud);

    return true;
}

bool loopSOFTSER(int ID, int iFunction)
{
    if(!bSOFTSERON)
        return false;
        
    // last query running
    if(iFunction == 0 && strSOFTSER_BUF.length() > 0)
        return false;

    char cText[100] = {0};

    // Pegestandsmesser
    if(ID == 1)
    {
        if(iFunction == 0)
        {
            snprintf(cText, sizeof(cText), "%s", "/cl/time/get");
        }
        else
        if(iFunction == 1)
        {
            snprintf(cText, sizeof(cText), "/cl/data/get/%s", strSOFTSER_BUF.substring(6, 20).c_str());

            unsigned int year;
            unsigned int month;
            unsigned int day;
            unsigned int hour;
            unsigned int minute;
            unsigned int second;

            char ctime_buf[30];

            snprintf(ctime_buf, sizeof(ctime_buf), "%s", strSOFTSER_BUF.substring(6, 20).c_str());

            sscanf(ctime_buf, "%04u%02u%02u%02u%02u%02u", &year, &month, &day, &hour, &minute, &second);

            MyClock.setCurrentTime(meshcom_settings.node_utcoff, year, month, day, hour, minute, second);

        }

        strSOFTSER_BUF = "";

        Serial.println(cText);
        
        sendSOFTSER(cText);

    }

    return true;
}

String strSOFTSERAPP_ID;
String strSOFTSERAPP_NAME;
String strSOFTSERAPP_PEGEL;
String strSOFTSERAPP_PEGEL2;
String strSOFTSERAPP_TEMP;
String strSOFTSERAPP_BATT;

String strSOFTSERAPP_FIXPEGEL="";
String strSOFTSERAPP_FIXPEGEL2="";
String strSOFTSERAPP_FIXTEMP="";

bool appSOFTSER(int ID)
{
    if(!bSOFTSERON)
        return false;
        
    // no query running
    if(strSOFTSER_BUF.length() < 1)
        return false;

    ////////////////////////////////////////////////////////////////////////
    // Pegestandsmesser

    Serial.println(strSOFTSER_BUF.substring(0,24).c_str());

    // got time
    if(ID == 1) // Pegelmesser
    {
#if defined(ENABLE_XML)
        if(strSOFTSER_BUF.indexOf("<0x03>") > 0)
        {
            loopSOFTSER(ID, 1);
        }
        else
        // got Data
        if(strSOFTSER_BUF.indexOf("?xml") > 0)
        {
            int sindex = strSOFTSER_BUF.indexOf("<StationDataList>");

            if(sindex > 0)
            {
                char* decodexml;
                sprintf(decodexml, "%s", strSOFTSER_BUF.substring(sindex).c_str());
                decodeTinyXML(decodexml);
            }
        }

#endif
        strSOFTSER_BUF = "";
    }
    else
    {
        strSOFTSER_BUF = "";
    }

    return true;
}

bool getSOFTSER()
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
    char tmp_hex[7] = {0};

    strSOFTSER_BUF = "";

    // For one second we parse SOFTSER data and report
    //for (unsigned long start = millis(); millis() - start < 1000;)
    bool bgrun=true;
    unsigned long start = millis();

    while(bgrun)
    {
#ifdef ESP32
        while (SOFTSER.available())
        {
            char c = SOFTSER.read();
#else
        while (Serial1.available())
        {
            char c = Serial1.read();
#endif

            if(c == 0x00)
            {
                bgrun=false;
                break;
            }

            if(bSOFTSERDEBUG)
                Serial.print(c);

            if((c < 0x20 || c > 0x7f) && c != 0x0d && c != 0x0a)
            {
                snprintf(tmp_hex, sizeof(tmp_hex), "<0x%02x>", c);
                tmp_data += tmp_hex;
                
            }
            else
                tmp_data += c;
        }
        
        if((millis() - start) > 3000)
            bgrun=false;
    }

    strSOFTSER_BUF.concat(tmp_data);
    strSOFTSER_BUF.concat("\r\n");

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

    if(bSOFTSERDEBUG)
        Serial.printf("writeSOFTSER...<%s>\n", cText);

    char cSend[100];
    
    snprintf(cSend, sizeof(cSend), "\r");

#ifdef ESP32
    SOFTSER.write(cSend);
#else
    Serial1.write(cSend);
#endif

    delay(500);

    snprintf(cSend, sizeof(cSend), "%s\r", cText);

#ifdef ESP32
    SOFTSER.write(cSend);
#else
    Serial1.write(cSend);
#endif

    getSOFTSER();
    
    return true;
}

#endif