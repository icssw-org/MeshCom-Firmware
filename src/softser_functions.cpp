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

String strSOFTSERAPP_FIXPEGEL="";
String strSOFTSERAPP_FIXPEGEL2="";
String strSOFTSERAPP_FIXTEMP="";

String strTELE_PARM="";
String strTELE_UNIT="";
String strTELE_VALUES="";
String strTELE_DATETIME="";
String strTELE_CH_ID="";

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
    // just for test
        if(strSOFTSER_BUF.indexOf("<0x03>") > 0)
        {
            loopSOFTSER(ID, 1);
        }
        else
        // got Data
        if(strSOFTSER_BUF.indexOf("<StationDataList>") >= 0)
        {
            int sindex = strSOFTSER_BUF.indexOf("<StationDataList>");

            if(sindex >= 0)
            {
                char* decodexml;
                sprintf(decodexml, "%s", strSOFTSER_BUF.substring(sindex).c_str());

                //decodeTinyXML(decodexml);
                testTinyXML();

                // fill Telemetry
                snprintf(meshcom_settings.node_parm_1, sizeof(meshcom_settings.node_parm_1), "%s", strTELE_PARM.c_str());
                snprintf(meshcom_settings.node_unit, sizeof(meshcom_settings.node_unit), "%s", strTELE_UNIT.c_str());
                snprintf(meshcom_settings.node_values, sizeof(meshcom_settings.node_values), "T:%s", strTELE_VALUES.c_str());

                // TEST
                sendTelemetry(ID);

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

/*
2025-05-15 10:05:37 CEST: OE3XIR-81>APRSMC,TCPIP*,qAR,OE1KBC-86::077234567:PARM.60 Wasserstand,65 Wassertemperatur,50 Batteriespannung
2025-05-15 08:08:35 CEST: OE3XIR-81>APRSMC,TCPIP*,qAR,OE1KBC-86::007723456:UNIT.cm,C,V
2025-05-15 10:05:57 CEST: OE3XIR-81>APRSMC,TCPIP*,qAR,OE1KBC-86::077234567:EQNS.0,1,0,0,1,0,0,1,0,0,1,0,0,1,0
2025-05-15 10:06:12 CEST: OE3XIR-81>APRSMC,TCPIP*,qAR,OE1KBC-86::077234567:BITS.00000000DemoStationNetDL500
2025-05-15 10:06:28 CEST: OE3XIR-81>APRSMC,TCPIP*,qAR,OE1KBC-86::077234567:T#249,28.5,25.0,13.4,0,0,00000000,2025-04-22T13:00:00,60,65,50
*/

#define MAX_ID 10

String strSID[MAX_ID];

String strSNAME[MAX_ID];
String strPARM[MAX_ID][5];
String strPARM_ID[MAX_ID][5];
String strUNIT[MAX_ID][5];

int next_id = 0;

int getSOFTSER_ID(String ID)
{
    for(int iid=0; iid<MAX_ID; iid++)
    {
        if(strSID[iid].compareTo(ID) == 0)
            return iid;
    }

    int retid = next_id;

    strSID[next_id] = ID;

    next_id++;
    if(next_id >= MAX_ID)
        next_id=0;

    return retid;
}

String getSOFTSER_PARM(int iID, int icd)
{
    return strPARM[iID][icd];
}

String getSOFTSER_UNIT(int iID, int icd)
{
    return strUNIT[iID][icd];
}

void setSOFTSER_PARM(int iID, int icd, String val, String valname)
{
    strPARM[iID][icd] = valname;
    strPARM_ID[iID][icd] = val;
}

void setSOFTSER_UNIT(int iID, int icd, String sDecode)
{
    strUNIT[iID][icd] = sDecode;
}

void setSOFTSER_SNAME(int iID, String sDecode)
{
    strSNAME[iID] = sDecode;
}

void displaySOFTSER(struct aprsMessage &aprsmsg)
{
    if(!bSOFTSERON)
    {
        return;
    }

    strSOFTSERAPP_ID = aprsmsg.msg_payload.substring(0, 9);
    strSOFTSERAPP_ID.trim();

    int iID = getSOFTSER_ID(strSOFTSERAPP_ID);

    if(iID < 0)
        return;

    int ipos=0;
    int impos=0;
    String sDecode="";

    if(aprsmsg.msg_payload.charAt(10) == 'P')
    {
        ipos=15;
        for(int icd=0; icd<5; icd++)
        {
            sDecode = aprsmsg.msg_payload.substring(ipos);
            impos = sDecode.indexOf(',');
            if(impos >= 0)
            {
                sDecode = sDecode.substring(0, impos);
            }

            if(sDecode.length() <= 0)
                break;
                
            setSOFTSER_PARM(iID, icd, sDecode.substring(0, 2), sDecode.substring(3));

            ipos = ipos + sDecode.length() + 1;
        }

        return;
    }
    else
    if(aprsmsg.msg_payload.charAt(10) == 'U')
    {
        ipos=15;
        for(int icd=0; icd<5; icd++)
        {
            sDecode = aprsmsg.msg_payload.substring(ipos);
            impos = sDecode.indexOf(',');
            if(impos >= 0)
            {
                sDecode = sDecode.substring(0, impos);
            }

            if(sDecode.length() <= 0)
                break;

            setSOFTSER_UNIT(iID, icd, sDecode);

            ipos = ipos + sDecode.length() + 1;
        }

        return;
    }
    else
    if(aprsmsg.msg_payload.charAt(10) == 'E')
    {
        return;
    }
    else
    if(aprsmsg.msg_payload.charAt(10) == 'B')
    {
        ipos=23;
        sDecode = aprsmsg.msg_payload.substring(ipos);

        setSOFTSER_SNAME(iID, sDecode);

        return;
    }
    else
    // VALUES receiced
    if(aprsmsg.msg_payload.charAt(10) == 'T')
    {
        String strValue[12] = {""};

        ipos=16;
        for(int icd=0; icd<12; icd++)
        {
            sDecode = aprsmsg.msg_payload.substring(ipos);
            impos = sDecode.indexOf(',');
            if(impos >= 0)
            {
                sDecode = sDecode.substring(0, impos);
            }

            if(sDecode.length() <= 0)
                break;
                
            strValue[icd] = sDecode;

            ipos = ipos + sDecode.length() + 1;
        }

        //2025-05-15 10:06:28 CEST: OE3XIR-81>APRSMC,TCPIP*,qAR,OE1KBC-86::077234567:T#249,28.5,25.0,13.4,0,0,00000000,2025-04-22T13:00:00,60,65,50

        Serial.printf("\n<SD Id=\"0%s\" name=\"%s\">\n", strSOFTSERAPP_ID.c_str(), strSNAME[iID].c_str());
        for(int icd=0;icd<5;icd++)
        {
            if(strValue[icd+7].length() > 0)
                Serial.printf("<CD id=\"00%s\" name=\"%s\" unit=\"%s\"><VT t=%s>%s</VT></CD>\n", strValue[icd+7].c_str(), getSOFTSER_PARM(iID, icd).c_str(), getSOFTSER_UNIT(iID, icd).c_str(), strValue[6].c_str(), strValue[icd].c_str());
        }
        Serial.printf("</SD>\n\n");
    }
}

#endif