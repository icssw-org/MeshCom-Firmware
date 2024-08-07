#include "configuration.h"
#include "loop_functions.h"
#include "loop_functions_extern.h"

#include <Arduino.h>

#if defined(ENABLE_SOFTSER)

#include <softser_functions.h>

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
            sprintf(cText, "%s", "/cl/time/get");
        }
        else
        if(iFunction == 1)
        {
            sprintf(cText, "/cl/data/get/%s", strSOFTSER_BUF.substring(6, 20).c_str());
        }

        strSOFTSER_BUF = "";

        Serial.println(cText);
        
        sendSOFTSER(cText);

    }

    return true;
}

String strSOFTSERAPP_ID;
String strSOFTSERAPP_PEGEL;
String strSOFTSERAPP_TEMP;
String strSOFTSERAPP_BATT;

String strSOFTSERAPP_FIXPEGEL = "550";
String strSOFTSERAPP_FIXTEMP = "20.0";

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
        if(strSOFTSER_BUF.indexOf("<0x03>") > 0)
        {
            loopSOFTSER(ID, 1);
        }
        else
        // got Data
        if(strSOFTSER_BUF.indexOf("?xml") > 0)
        {
            String strtemp;
            String strtemp1;

            if(bSOFTSERDEBUG)
            {
                Serial.printf("SOFTSERDATA\n%s\n", strSOFTSER_BUF.substring(strSOFTSER_BUF.indexOf("?xml"), strSOFTSER_BUF.indexOf("firmware")).c_str());
            }

            if(strSOFTSER_BUF.indexOf("stationId=") > 0)
            {
            strtemp = strSOFTSER_BUF.substring(strSOFTSER_BUF.indexOf("stationId="));

                strSOFTSERAPP_ID = strtemp.substring(11, 21);
                
                Serial.printf("Station...%s\n", strSOFTSERAPP_ID.c_str());
            }

            if(strSOFTSER_BUF.indexOf("Wasserstand") > 0)
            {
                strtemp = strSOFTSER_BUF.substring(strSOFTSER_BUF.indexOf("Wasserstand"));

                if(strtemp.indexOf("<VT t=") > 0)
                {
                    strtemp1 = strtemp.substring(strtemp.indexOf("<VT t=")+1);

                    strSOFTSERAPP_PEGEL = strtemp1.substring(strtemp1.indexOf(">")+1, strtemp1.indexOf("<")).c_str();

                    Serial.printf("Pegel....%s cm\n", strSOFTSERAPP_PEGEL.c_str());
                }
                else
                    strSOFTSERAPP_PEGEL = "";
            }

            if(strSOFTSER_BUF.indexOf("Wassertemperatur") > 0)
            {
                strtemp = strSOFTSER_BUF.substring(strSOFTSER_BUF.indexOf("Wassertemperatur"));

                if(strtemp.indexOf("<VT t=") > 0)
                {
                    strtemp1 = strtemp.substring(strtemp.indexOf("<VT t=")+1);

                    strSOFTSERAPP_TEMP = strtemp1.substring(strtemp1.indexOf(">")+1, strtemp1.indexOf("<")).c_str();

                    Serial.printf("Temp.....%s °C\n", strSOFTSERAPP_TEMP.c_str());
                }
                else
                    strSOFTSERAPP_TEMP = "";
            }

            if(strSOFTSER_BUF.indexOf("Batteriespannung") > 0)
            {
                strtemp = strSOFTSER_BUF.substring(strSOFTSER_BUF.indexOf("Batteriespannung"));

                if(strtemp.indexOf("<VT t=") > 0)
                {
                    strtemp1 = strtemp.substring(strtemp.indexOf("<VT t=")+1);

                    strSOFTSERAPP_BATT = strtemp1.substring(strtemp1.indexOf(">")+1, strtemp1.indexOf("<")).c_str();

                    Serial.printf("BATT.....%s V\n", strSOFTSERAPP_BATT.c_str());
                }
                else
                    strSOFTSERAPP_BATT = "";
            }

            if(strSOFTSERAPP_FIXPEGEL.length() > 0)
                Serial.printf("PEGEL(F).%s cm\n", strSOFTSERAPP_FIXPEGEL.c_str());
            if(strSOFTSERAPP_FIXPEGEL.length() > 0)
                Serial.printf("TEMP(F)..%s °C\n", strSOFTSERAPP_FIXTEMP.c_str());

            sendAPPPosition(meshcom_settings.node_lat, meshcom_settings.node_lat_c, meshcom_settings.node_lon, meshcom_settings.node_lon_c, meshcom_settings.node_temp2);
        
            strSOFTSER_BUF = "";
        }
        else
        {
            strSOFTSER_BUF = "";
        }
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
                sprintf(tmp_hex, "<0x%02x>", c);
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
    
    sprintf(cSend, "\r");

#ifdef ESP32
    SOFTSER.write(cSend);
#else
    Serial1.write(cSend);
#endif

    delay(500);

    sprintf(cSend, "%s\r", cText);

#ifdef ESP32
    SOFTSER.write(cSend);
#else
    Serial1.write(cSend);
#endif

    getSOFTSER();
    
    return true;
}

#endif