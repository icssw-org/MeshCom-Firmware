#include <aprs_functions.h>
#include <loop_functions.h>
#include <debugconf.h>

char mheardBuffer[MAX_MHEARD][60]; //Ringbuffer for MHeard Lines
char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
uint8_t mheardWrite = 0;   // counter for ringbuffer

String HardWare[13] = {"no info", "TLORA_V2", "TLORA_V1", "TLORA_V2_1_1p6", "TBEAM", "TBEAM_1268", "TBEAM_0p7", "T_ECHO", "-", "RAK4631", "HELTEC_V2_1", "HELTEC_V1", "EBYTE_E22"};

void initMheard()
{
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
                    strdec.concat(mh_buffer[iset]);
                    break;
                default: break;
            }
        }
    }
}

void updateMheard(struct mheardLine &mheardLine)
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
        if(mheardWrite > MAX_MHEARD)
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
    sprintf(mheardBuffer[ipos], "%s@%s@%c@%i@%i@%i@%i@", mheardLine.mh_date.c_str(), mheardLine.mh_time.c_str(), mheardLine.mh_payload_type, mheardLine.mh_hw, mheardLine.mh_mod, mheardLine.mh_rssi, mheardLine.mh_snr);

    // send to Phone
    uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
    bleBuffer[0] = 0x91;
    memcpy(bleBuffer+1, mheardCalls[ipos], 10);
    memcpy(bleBuffer+11, mheardBuffer[ipos], strlen(mheardBuffer[ipos]));

    addBLEOutBuffer(bleBuffer, strlen(mheardBuffer[ipos])+1+10);
}

void showMHeard()
{
    Serial.printf("/--------------------------------------------------------------------------------\\\n");
    Serial.printf("|MHeard call |    date    |   time   | typ | source hardware | mod | rssi |  snr |\n");

    mheardLine mheardLine;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            Serial.printf("|------------|------------|----------|-----|-----------------|-----|------|------|\n");

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
                ihw=12;
            if(ihw < 0 || ihw > 12)
                ihw=0;

            Serial.printf("%-15.15s | ", HardWare[ihw].c_str());

            Serial.printf("%3i | ", mheardLine.mh_mod);
            Serial.printf("%4i | ", mheardLine.mh_rssi);
            Serial.printf("%4i |\n", mheardLine.mh_snr);
        }
    }

    Serial.printf("\\--------------------------------------------------------------------------------/\n");
}