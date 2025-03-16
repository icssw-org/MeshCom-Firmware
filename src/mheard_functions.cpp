#include <aprs_functions.h>
#include <loop_functions.h>
#include <debugconf.h>
#include <ArduinoJson.h>
#include <time_functions.h>
#include <mheard_functions.h>

char mheardBuffer[MAX_MHEARD][60]; //Ringbuffer for MHeard Lines
char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
char mheardPathCalls[MAX_MHPATH][10]; //Ringbuffer for MHeard Key = Call
char mheardPath[MAX_MHPATH][70]; //Ringbuffer for MHeard Sourcepath
double mheardLat[MAX_MHEARD];
double mheardLon[MAX_MHEARD];
unsigned long mheardEpoch[MAX_MHEARD];
unsigned long mheardPathEpoch[MAX_MHPATH];

uint8_t mheardWrite = 0;   // counter for ringbuffer
uint8_t mheardPathWrite = 0;   // counter for ringbuffer

#define max_hardware 18
String HardWare[max_hardware] = {"no info", "TLORA_V2", "TLORA_V1", "TLORA_V2_1_1p6", "TBEAM", "TBEAM_1268", "TBEAM_0p7", "T_ECHO", "TDECK", "RAK4631", "HELTEC_V2_1", "HELTEC_V1", "TBEAM_AXP2101", "EBYTE_E22", "HELTEC_V3", "HELTEC_E290", "TBEAM_1262", "TDECK_PLUS"};

void initMheard()
{
    Serial.println("[INIT]...initMheard");

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        memset(mheardBuffer[iset], 0x00, sizeof(mheardBuffer[iset]));
        memset(mheardCalls[iset], 0x00, sizeof(mheardCalls[iset]));
        mheardLat[iset]=0;
        mheardLon[iset]=0;
    }

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        memset(mheardPathCalls[iset], 0x00, sizeof(mheardPathCalls[iset]));
        memset(mheardPath[iset], 0x00, sizeof(mheardPath[iset]));
        mheardEpoch[iset]=0;
    }

    mheardWrite=0;
    mheardPathWrite = 0;

}

void initMheardLine(struct mheardLine &mheardLine)
{
    mheardLine.mh_callsign = "";
    mheardLine.mh_sourcecallsign = "";
    mheardLine.mh_sourcepath = "";
    mheardLine.mh_date = "";
    mheardLine.mh_time = "";
    mheardLine.mh_payload_type = 0x00;
    mheardLine.mh_hw = 0;
    mheardLine.mh_mod = 0;
    mheardLine.mh_rssi = 0;
    mheardLine.mh_snr = 0;
    mheardLine.mh_dist = 0.0;
    mheardLine.mh_path_len = 0;
    mheardLine.mh_mesh = 0;
}

void decodeMHeard(char mh_buffer[], struct mheardLine &mheardLine)
{
    initMheardLine(mheardLine);

    int itype=1;
    String strdec = "";
    for(int iset=0; iset<55; iset++)
    {
        if(mh_buffer[iset] == '|')
        {
            switch (itype)
            {
                case 1: break;
                case 2: break;
                case 3: break;
                case 4: mheardLine.mh_hw = strdec.toInt(); break;
                case 5: mheardLine.mh_mod = strdec.toInt(); break;
                case 6: mheardLine.mh_rssi = strdec.toInt(); break;
                case 7: mheardLine.mh_snr = strdec.toInt(); break;
                case 8: mheardLine.mh_dist = strdec.toFloat(); break;
                case 9: mheardLine.mh_path_len = strdec.toInt(); break;
                case 10: mheardLine.mh_mesh = strdec.toInt(); break;
                default: break;
            }

            strdec="";

            itype++;
        }
        else
        {
            switch (itype)
            {
                case 1: mheardLine.mh_date.concat(mh_buffer[iset]); break;
                case 2: mheardLine.mh_time.concat(mh_buffer[iset]); break;
                case 3: mheardLine.mh_payload_type = mh_buffer[iset]; break;
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
                    strdec.concat(mh_buffer[iset]);
                    break;
                default: break;
            }
        }
    }
}

void updateMheard(struct mheardLine &mheardLine, uint8_t isPhoneReady)
{
    int ipos=-1;
    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            if(strcmp(mheardCalls[iset], mheardLine.mh_callsign.c_str()) == 0)
            {
                ipos=iset;
                break;
            }

        }
    }

    if(ipos == -1)
    {
        ipos=mheardWrite;
        mheardWrite++;
        
        //Serial.printf("mheardWrite:%i\n", mheardWrite);

        if(mheardWrite >= MAX_MHEARD)
            mheardWrite=0;
    }

    strcpy(mheardCalls[ipos], mheardLine.mh_callsign.c_str());
    strcpy(mheardPath[ipos], mheardLine.mh_sourcepath.c_str());
    
    mheardEpoch[ipos] = getUnixClock();
    /*
    String mh_time;
    String mh_callsign;
    uint8_t mh_hw;
    uint8_t mh_mod;
    int16_t mh_rssi;
    int8_t mh_snr;
    */
    snprintf(mheardBuffer[ipos], sizeof(mheardBuffer[ipos]), "%s|%s|%c|%i|%u|%i|%i|%.1lf|%i|%i|", mheardLine.mh_date.c_str(), mheardLine.mh_time.c_str(), mheardLine.mh_payload_type, mheardLine.mh_hw,
     mheardLine.mh_mod, mheardLine.mh_rssi, mheardLine.mh_snr, mheardLine.mh_dist, mheardLine.mh_path_len, mheardLine.mh_mesh);

    // generate JSON
    JsonDocument mhdoc;

    mhdoc["TYP"] = "MH";
    mhdoc["CALL"] = mheardLine.mh_callsign.c_str();
    mhdoc["DATE"] = mheardLine.mh_date.c_str();
    mhdoc["TIME"] = mheardLine.mh_time.c_str();
    mhdoc["PLT"] = (uint8_t)mheardLine.mh_payload_type;
    mhdoc["HW"] = mheardLine.mh_hw;
    mhdoc["MOD"] = mheardLine.mh_mod;
    mhdoc["RSSI"] = mheardLine.mh_rssi;
    mhdoc["SNR"] = mheardLine.mh_snr;
    mhdoc["DIST"] = mheardLine.mh_dist;
    mhdoc["PL"] = mheardLine.mh_path_len;
    mhdoc["MESH"] = mheardLine.mh_mesh;

    // send to Phone
    uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
    bleBuffer[0] = 0x44;
    serializeJson(mhdoc, bleBuffer+1, measureJson(mhdoc)+1);

    if(isPhoneReady == 1)
        addBLEOutBuffer(bleBuffer, measureJson(mhdoc)+1);
}

void updateHeyPath(struct mheardLine &mheardLine)
{
    int ipos=-1;
    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            if(strcmp(mheardPathCalls[iset], mheardLine.mh_sourcecallsign.c_str()) == 0)
            {
                ipos=iset;
                break;
            }

        }
    }

    if(ipos == -1)
    {
        ipos=mheardPathWrite;
        mheardPathWrite++;
        
        //Serial.printf("mheardPathWrite:%i\n", mheardPathWrite);

        if(mheardPathWrite >= MAX_MHPATH)
            mheardPathWrite=0;
    }

    strcpy(mheardPathCalls[ipos], mheardLine.mh_sourcecallsign.c_str());
    strcpy(mheardPath[ipos], mheardLine.mh_sourcepath.c_str());
    
    mheardPathEpoch[ipos] = getUnixClock();
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void sendMheard()
{
    struct mheardLine mheardLine;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            if((mheardEpoch[iset]+60*60*6) > getUnixClock())
            {
                initMheardLine(mheardLine);

                mheardLine.mh_callsign = mheardCalls[iset];
                String mhstringdec = mheardBuffer[iset];

                mheardLine.mh_date = getValue(mhstringdec, '|', 0);
                mheardLine.mh_time = getValue(mhstringdec, '|', 1);

                String xval = getValue(mhstringdec, '|', 2);
                mheardLine.mh_payload_type = xval.charAt(0);

                xval = getValue(mhstringdec, '|', 3);
                mheardLine.mh_hw = xval.toInt();

                xval = getValue(mhstringdec, '|', 4);
                mheardLine.mh_mod = xval.toInt();

                xval = getValue(mhstringdec, '|', 5);
                mheardLine.mh_rssi = xval.toInt();

                xval = getValue(mhstringdec, '|', 6);
                mheardLine.mh_snr = xval.toInt();

                xval = getValue(mhstringdec, '|', 7);
                mheardLine.mh_dist = xval.toFloat();

                xval = getValue(mhstringdec, '|', 8);
                mheardLine.mh_path_len = xval.toInt();

                // generate JSON
                JsonDocument mhdoc;

                mhdoc["TYP"] = "MH";
                mhdoc["CALL"] = mheardLine.mh_callsign.c_str();
                mhdoc["DATE"] = mheardLine.mh_date.c_str();
                mhdoc["TIME"] = mheardLine.mh_time.c_str();
                mhdoc["PLT"] = (uint8_t)mheardLine.mh_payload_type;
                mhdoc["HW"] = mheardLine.mh_hw;
                mhdoc["MOD"] = mheardLine.mh_mod;
                mhdoc["RSSI"] = mheardLine.mh_rssi;
                mhdoc["SNR"] = mheardLine.mh_snr;
                mhdoc["DIST"] = mheardLine.mh_dist;
                mhdoc["DIST"] = mheardLine.mh_path_len;

                // send to Phone
                uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
                bleBuffer[0] = 0x44;
                serializeJson(mhdoc, bleBuffer+1, measureJson(mhdoc)+1);

                addBLEComToOutBuffer(bleBuffer, measureJson(mhdoc)+1);
            }
        }
    }
}

void showMHeard()
{
    Serial.printf("/------------------------------------------------------------------------------------------------\\\n");
    Serial.printf("|MHeard call |    date    |   time   | typ | source hardware | mod | rssi |  snr | dist | pl | m |\n");

    mheardLine mheardLine;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            if((mheardEpoch[iset]+60*60*6) > getUnixClock())
            {
                Serial.printf("|------------|------------|----------|-----|-----------------|-----|------|------|------|----|---|\n");

                Serial.printf("| %-10.10s | ", mheardCalls[iset]);
                
                decodeMHeard(mheardBuffer[iset], mheardLine);

                Serial.printf("%-10.10s | ", mheardLine.mh_date.c_str());
                Serial.printf("%-8.8s | ", mheardLine.mh_time.c_str());

                Serial.printf("%-3.3s | ", getPayloadType(mheardLine.mh_payload_type));

                Serial.printf("%-11.11s/%03i | ", getHardwareLong(mheardLine.mh_hw).c_str(), mheardLine.mh_hw);

                Serial.printf("%01X/%01i | ", (mheardLine.mh_mod>>4), (mheardLine.mh_mod & 0xf));
                Serial.printf("%4i | ", mheardLine.mh_rssi);
                Serial.printf("%4i |", mheardLine.mh_snr);
                Serial.printf("%5.1lf |", mheardLine.mh_dist);
                Serial.printf("%3i |", mheardLine.mh_path_len);
                Serial.printf("%2i |\n", mheardLine.mh_mesh);
            }
        }
    }

    Serial.printf("\\------------------------------------------------------------------------------------------------/\n");
}

void showPath()
{
    Serial.printf("/------------------------------------------------------------------------------------------------\\\n");
    Serial.printf("|MHeard call |       date          |                                                             |\n");

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            if((mheardPathEpoch[iset]+60*60*6) > getUnixClock())
            {
                Serial.printf("|------------|---------------------|-------------------------------------------------------------|\n");

                Serial.printf("| %-10.10s | ", mheardPathCalls[iset]);

                unsigned long lt = mheardPathEpoch[iset] + ((60 * 60 + 24) * (int)meshcom_settings.node_utcoff);
                
                Serial.printf("%-19.19s | ", convertUNIXtoString(lt).c_str()); // yyyy.mm.dd hh:mm:ss

                Serial.printf("%-60.60s|\n", mheardPath[iset]);
            }
        }
    }

    Serial.printf("\\------------------------------------------------------------------------------------------------/\n");
}

char* getPayloadType(char ptype)
{
    if(ptype == ':')
        return (char*)"TXT";
    else
    if(ptype == '!')
        return (char*)"POS";
    else
    if(ptype == '@')
        return (char*)"HEY";

    return (char*)"???";
}

String getHardwareLong(uint8_t hwid)
{
    int ihw=hwid;
    if(ihw == 39)
        ihw=13;
    if(ihw == 43)
        ihw=14;
    if(ihw == 44)
        ihw=15;
    if(ihw == 45)
        ihw=16;
    if(ihw == 46)
        ihw=17;
    if(ihw < 0 || ihw >= max_hardware)
        ihw=0;

    return HardWare[ihw];
}