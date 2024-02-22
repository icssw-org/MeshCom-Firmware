#include <aprs_functions.h>
#include <loop_functions.h>
#include <debugconf.h>
#include <ArduinoJson.h>

char mheardBuffer[MAX_MHEARD][60]; //Ringbuffer for MHeard Lines
char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
double mheardLat[MAX_MHEARD];
double mheardLon[MAX_MHEARD];

uint8_t mheardWrite = 0;   // counter for ringbuffer

#define max_hardware 15
String HardWare[max_hardware] = {"no info", "TLORA_V2", "TLORA_V1", "TLORA_V2_1_1p6", "TBEAM", "TBEAM_1268", "TBEAM_0p7", "T_ECHO", "T_DECK", "RAK4631", "HELTEC_V2_1", "HELTEC_V1", "TBEAM_AXP2101", "EBYTE_E22", "HELTEC_V3"};

void initMheard()
{
    Serial.println("[INIT]...initMheard");

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        memset(mheardBuffer[iset], 0x00, 60);
        memset(mheardCalls[iset], 0x00, 10);
    }

    mheardWrite=0;
}

void initMheardLine(struct mheardLine &mheardLine)
{
    mheardLine.mh_callsign = "";
    mheardLine.mh_date = "";
    mheardLine.mh_time = "";
    mheardLine.mh_payload_type = 0x00;
    mheardLine.mh_hw = 0;
    mheardLine.mh_mod = 0;
    mheardLine.mh_rssi = 0;
    mheardLine.mh_snr = 0;
    mheardLine.mh_dist = 0.0;
}

void decodeMHeard(char mh_buffer[], struct mheardLine &mheardLine)
{
    initMheardLine(mheardLine);

    int itype=1;
    String strdec = "";
    for(int iset=0; iset<55; iset++)
    {
        if(mh_buffer[iset] == '@')
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
    /*
    String mh_time;
    String mh_callsign;
    uint8_t mh_hw;
    uint8_t mh_mod;
    int16_t mh_rssi;
    int8_t mh_snr;
    */
    sprintf(mheardBuffer[ipos], "%s@%s@%c@%i@%i@%i@%i@%.0lf@", mheardLine.mh_date.c_str(), mheardLine.mh_time.c_str(), mheardLine.mh_payload_type, mheardLine.mh_hw, mheardLine.mh_mod, mheardLine.mh_rssi, mheardLine.mh_snr, mheardLine.mh_dist);

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

    // send to Phone
    uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
    bleBuffer[0] = 0x44;
    serializeJson(mhdoc, bleBuffer+1, measureJson(mhdoc)+1);

    //memcpy(bleBuffer+1, mheardCalls[ipos], 10);
    //memcpy(bleBuffer+11, mheardBuffer[ipos], strlen(mheardBuffer[ipos]));

    if(isPhoneReady == 1)
        //addBLEOutBuffer(bleBuffer, strlen(mheardBuffer[ipos])+1+10);
        addBLEOutBuffer(bleBuffer, measureJson(mhdoc)+1);
}

void showMHeard()
{
    Serial.printf("/---------------------------------------------------------------------------------------\\\n");
    Serial.printf("|MHeard call |    date    |   time   | typ | source hardware | mod | rssi |  snr | dist |\n");

    mheardLine mheardLine;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            Serial.printf("|------------|------------|----------|-----|-----------------|-----|------|------|------|\n");

            Serial.printf("| %-10.10s | ", mheardCalls[iset]);
            
            decodeMHeard(mheardBuffer[iset], mheardLine);

            Serial.printf("%-10.10s | ", mheardLine.mh_date.c_str());
            Serial.printf("%-8.8s | ", mheardLine.mh_time.c_str());
            if(mheardLine.mh_payload_type == ':')
                Serial.printf("TXT | ");
            else
            if(mheardLine.mh_payload_type == '!')
                Serial.printf("POS | ");
            else
            if(mheardLine.mh_payload_type == '@')
                Serial.printf("WX  | ");
            else
                Serial.printf("??? | ");

            int ihw=mheardLine.mh_hw;
            if(mheardLine.mh_hw == 39)
                ihw=13;
            if(mheardLine.mh_hw == 43)
                ihw=14;
            if(ihw < 0 || ihw >= max_hardware)
                ihw=0;

            if(ihw == 0)
                Serial.printf("%-11.11s/%03i | ", HardWare[ihw].c_str(), mheardLine.mh_hw);
            else
                Serial.printf("%-15.15s | ", HardWare[ihw].c_str());

            Serial.printf("%3i | ", mheardLine.mh_mod);
            Serial.printf("%4i | ", mheardLine.mh_rssi);
            Serial.printf("%4i |", mheardLine.mh_snr);
            //Serial.printf("%5.0lf | %.4lf %.4lf\n", mheardLine.mh_dist, mheardLat[iset], mheardLon[iset]);
            Serial.printf("%5.0lf |\n", mheardLine.mh_dist);
        }
    }

    Serial.printf("\\---------------------------------------------------------------------------------------/\n");
}