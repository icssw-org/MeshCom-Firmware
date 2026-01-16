#include <aprs_functions.h>
#include <loop_functions.h>
#include <debugconf.h>
#include <ArduinoJson.h>
#include <time_functions.h>
#include <mheard_functions.h>

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/lv_obj_functions_extern.h>
#endif

#if defined(BOARD_T_DECK_PRO)
#include <t-deck-pro/tdeck_pro.h>
#endif

unsigned char mheardBuffer[MAX_MHEARD][60]; //Ringbuffer for MHeard Lines
char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
double mheardLat[MAX_MHEARD];
double mheardLon[MAX_MHEARD];
int mheardAlt[MAX_MHEARD];
unsigned long mheardEpoch[MAX_MHEARD];

unsigned char mheardPathBuffer1[MAX_MHPATH][38]; //Ringbuffer for MHeard Sourcepath
char mheardPathCalls[MAX_MHPATH][10]; //Ringbuffer for MHeard Key = Call
unsigned long mheardPathEpoch[MAX_MHPATH];
uint8_t mheardPathLen[MAX_MHPATH];

uint8_t mheardWrite = 0;   // counter for ringbuffer
uint8_t mheardPathWrite = 0;   // counter for ringbuffer

#define max_hardware 25

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    String HardWare[max_hardware] = {"no info", "TLO_V2", "TLO_V1", "TLV2_1p6", "TBEAM", "TB_1268", "TB_0p7", "TECHO", "TDECK", "RAK4631", "HELTV21", "HELTV1", "TB_2101", "EB_E22", "HELTV3", "HELT_E290", "TB_1262", "TDECK+", "TB_SL76K", "ES3_E22", "TRACKER_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro"};
#else
    String HardWare[max_hardware] = {"no info", "TLORA_V2", "TLORA_V1", "TLORA_V2_1_1p6", "TBEAM", "TBEAM_1268", "TBEAM_0p7", "T_ECHO", "TDECK", "RAK4631", "HELTEC_V2_1", "HELTEC_V1", "TBEAM_AXP2101", "EBYTE_E22", "HELTEC_V3", "HELTEC_E290", "TBEAM_1262", "TDECK_PLUS", "TBEAM_SL76K", "ESP_S3_E22", "TRACK_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro"};
#endif

void initMheard()
{
    Serial.println("[INIT]...initMheard");

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        memset(mheardBuffer[iset], 0x00, sizeof(mheardBuffer[iset]));
        memset(mheardCalls[iset], 0x00, sizeof(mheardCalls[iset]));
        mheardLat[iset]=0;
        mheardLon[iset]=0;
        mheardAlt[iset]=0;
        mheardEpoch[iset]=0;
    }

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        memset(mheardPathBuffer1[iset], 0x00, sizeof(mheardPathBuffer1[iset]));
        memset(mheardPathCalls[iset], 0x00, sizeof(mheardPathCalls[iset]));
        mheardPathEpoch[iset]=0;
        mheardPathLen[iset]=0;
    }

    mheardWrite=0;
    mheardPathWrite = 0;

}

void initMheardLine(struct mheardLine &mheardLine)
{
    mheardLine.mh_callsign = "";
    mheardLine.mh_sourcecallsign = "";
    mheardLine.mh_sourcepath = "";
    mheardLine.mh_destinationpath = "";
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

void decodeMHeard(unsigned char u_mh_buffer[sizeof(mheardBuffer[0])], struct mheardLine &mheardLine)
{
    char mh_buffer[sizeof(mheardBuffer[0])];
    memcpy(mh_buffer, u_mh_buffer, sizeof(mheardBuffer[0]));
    
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
    String strYear = mheardLine.mh_date.substring(0, 4);
    if(strYear.toInt() < 2025)
        return;

    int ipos=-1;
    int inext=-1;
    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            if((mheardEpoch[iset]+60*60*12) < getUnixClock())    // 12h
            {
                mheardCalls[iset][0] = 0x00;
            }
            else
            {
                int ivgll= mheardLine.mh_callsign.length();
                if(strlen(mheardCalls[iset]) > (size_t)ivgll)
                    ivgll=strlen(mheardCalls[iset]);

                if(memcmp(mheardCalls[iset], mheardLine.mh_callsign.c_str(), ivgll) == 0)
                {
                    ipos=iset;
                }
            }
        }
        else
        {
            if(inext < 0)
                inext=iset;
        }
    }

    //Serial.printf("inext:%i ipos:%i\n", inext, ipos);

    if(inext >= 0 && ipos == -1)
    {
        ipos=inext;
    }
    else
    if(inext == -1 && ipos == -1)
    {
        ipos=mheardWrite;
        
        mheardWrite++;

        if(mheardWrite >= MAX_MHEARD)
            mheardWrite=0;
    }

    memset(mheardCalls[ipos], 0x00, sizeof(mheardCalls[ipos]));
    memcpy(mheardCalls[ipos], mheardLine.mh_callsign.c_str(), mheardLine.mh_callsign.length());
    
    mheardEpoch[ipos] = getUnixClock();
    /*
    String mh_time;
    String mh_callsign;
    uint8_t mh_hw;
    uint8_t mh_mod;
    int16_t mh_rssi;
    int8_t mh_snr;
    */
    char cBuffer[60];
    snprintf(cBuffer, sizeof(cBuffer), "%s|%s|%c|%i|%u|%i|%i|%.1lf|%i|%i|", mheardLine.mh_date.c_str(), mheardLine.mh_time.c_str(), mheardLine.mh_payload_type, mheardLine.mh_hw,
     mheardLine.mh_mod, mheardLine.mh_rssi, mheardLine.mh_snr, mheardLine.mh_dist, mheardLine.mh_path_len, mheardLine.mh_mesh); 
    memcpy(mheardBuffer[ipos], cBuffer, sizeof(cBuffer));

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

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    showMHeardTDECK();
    #endif

    #if defined(BOARD_T_DECK_PRO)
    TDeck_pro_mheard_disp();
    #endif
}

void updateHeyPath(struct mheardLine &mheardLine)
{
    String strYear = mheardLine.mh_date.substring(0, 4);
    if(strYear.toInt() < 2025)
        return;

    // exclude the owncall
    if(mheardLine.mh_sourcecallsign == meshcom_settings.node_call)
        return;

    // check MHEARD exists
    for(int imh=0; imh<MAX_MHEARD; imh++)
    {
        if(mheardCalls[imh][0] != 0x00)
        {
            int ivgll= mheardLine.mh_sourcecallsign.length();
            if(strlen(mheardCalls[imh]) > (size_t)ivgll)
                ivgll=strlen(mheardCalls[imh]);

            if(memcmp(mheardCalls[imh], mheardLine.mh_sourcecallsign.c_str(), ivgll) == 0)
                return; // call heard direct
        }
    }

    int ipos=-1;
    int inext=-1;
    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            // PATH DELETE after 12 Hours
            if((mheardPathEpoch[iset]+60*60*12) < getUnixClock())
            {
                mheardPathCalls[iset][0] = 0x00;
            }
            else
            {
                int ivgll= mheardLine.mh_sourcecallsign.length();
                if(strlen(mheardPathCalls[iset]) > (size_t)ivgll)
                    ivgll=strlen(mheardPathCalls[iset]);

                if(memcmp(mheardPathCalls[iset], mheardLine.mh_sourcecallsign.c_str(), ivgll) == 0)
                {
                    ipos=iset;
                }
            }
        }
        else
        {
            if(inext < 0)
                inext=iset;
        }
    }

    if(inext >= 0 && ipos == -1)
    {
        ipos=inext;
        mheardPathLen[ipos] = 0x7F;
    }
    else
    if(inext == -1 && ipos == -1)
    {
        ipos=mheardPathWrite;
        
        mheardPathWrite++;

        if(mheardPathWrite >= MAX_MHPATH)
            mheardPathWrite=0;
    }

    // check Path-Count
    if((mheardPathLen[ipos] & 0x7F) < mheardLine.mh_path_len)
    {
        // leave old record active
        return;
    }

    // OE3YCB-15,OE3XOC-12,OE3SPR-1>
    int ips = mheardLine.mh_sourcepath.indexOf(',') + 1;
    int ipc = mheardLine.mh_sourcepath.length() - ips;
    if(ipc > 37)
        ipc = 37;

    
    // only MHEARD HEY
    if(ips <= 0)
        return;

    memset(mheardPathCalls[ipos], 0x00, sizeof(mheardPathCalls[ipos]));
    memcpy(mheardPathCalls[ipos], mheardLine.mh_sourcecallsign.c_str(), mheardLine.mh_sourcecallsign.length());
    mheardPathCalls[ipos][9]=0x00;

    //Serial.printf("PATH:%i <%s> <%s> %i %i\n", ipos,  mheardLine.mh_sourcepath.c_str(), mheardLine.mh_sourcepath.substring(ips).c_str(), ips, ipc);

    memset(mheardPathBuffer1[ipos], 0x00, sizeof(mheardPathBuffer1[ipos]));
    memcpy(mheardPathBuffer1[ipos], mheardLine.mh_sourcepath.substring(ips).c_str(), ipc);
    mheardPathBuffer1[ipos][37] = 0x00;
    // TODO second 30 chars

    // check HEY! comming from gateway
    if(mheardLine.mh_destinationpath == "HG")
        mheardPathLen[ipos] = mheardLine.mh_path_len | 0x80;
    else
        mheardPathLen[ipos] = mheardLine.mh_path_len;
    
    mheardPathEpoch[ipos] = getUnixClock();

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    showPathTDECK();
    #endif
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
            // PATH DELETE after 12 Hours
            if((mheardEpoch[iset]+60*60*12) > getUnixClock()) // 12h
            {
                initMheardLine(mheardLine);

                mheardLine.mh_callsign = (char *)mheardCalls[iset];
                String mhstringdec = (char *)mheardBuffer[iset];

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
            if((mheardEpoch[iset]+60*60*12) > getUnixClock())
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
    Serial.printf("/---------------------------------------------------------------------------\\\n");
    Serial.printf("|       date          | lng/Gate/Path                                       |\n");

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            if((mheardPathEpoch[iset]+60*60*12) > getUnixClock())    // 12h
            {
                Serial.printf("|---------------------|-----------------------------------------------------|\n");

                //Serial.printf("| %-10.10s | ", mheardPathCalls[iset]);

                unsigned long lt = mheardPathEpoch[iset] + ((60 * 60 + 24) * (int)meshcom_settings.node_utcoff);
                
                Serial.printf("| %-19.19s | ", convertUNIXtoString(lt).c_str()); // yyyy.mm.dd hh:mm:ss

                Serial.printf("%01u%s/%-10.10s %-37.37s |\n", (mheardPathLen[iset] & 0x7F), ((mheardPathLen[iset] & 0x80)?"G":" "), mheardPathCalls[iset], mheardPathBuffer1[iset]);
            }
            else
            {
                mheardPathCalls[iset][0] = 0x00;
            }
        }
    }

    Serial.printf("\\---------------------------------------------------------------------------/\n");
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
    if(ihw == 40)
        ihw=22;
    if(ihw == 41)
        ihw=20;
    if(ihw == 42)
        ihw=21;
    if(ihw == 43)
        ihw=14;
    if(ihw == 44)
        ihw=15;
    if(ihw == 45)
        ihw=16;
    if(ihw == 46)
        ihw=17;
    if(ihw == 47)
        ihw=18;
    if(ihw == 48)
        ihw=19;
    if(ihw == 49)
        ihw=23;
    if(ihw == 50)
        ihw=24;
    if(ihw < 0 || ihw >= max_hardware)
        ihw=0;

    return HardWare[ihw];
}

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
/**
 * displays MHeard on T-Deck
 */
void showMHeardTDECK()
{
    char buf[200];

    //snprintf(buf, 200, "|   MHeard  | time  | typ |    HW   | rssi | snr |\n");

    mheardLine mheardLine;

    uint16_t row=0;

    lv_table_set_cell_value(mheard_ta, row, 0, (char*)"Call");
    lv_table_set_cell_value(mheard_ta, row, 1, (char*)"Time");
    lv_table_set_cell_value(mheard_ta, row, 2, (char*)"Type");
    lv_table_set_cell_value(mheard_ta, row, 3, (char*)"HW");
    lv_table_set_cell_value(mheard_ta, row, 4, (char*)"RSSI");
    lv_table_set_cell_value(mheard_ta, row, 5, (char*)"SNR");

    row++;

    int anzrow=1;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
            anzrow++;
    }

    lv_table_set_row_cnt(mheard_ta, anzrow);

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            snprintf(buf, 10, "%s", mheardCalls[iset]);
            lv_table_set_cell_value(mheard_ta, row, 0, buf);
            
            decodeMHeard(mheardBuffer[iset], mheardLine);

            snprintf(buf, 6, "%s", mheardLine.mh_time.substring(0, 5).c_str());
            lv_table_set_cell_value(mheard_ta, row, 1, buf);

            if(mheardLine.mh_payload_type == ':')
            {
                snprintf(buf, 4, "TXT");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }
            else
            if(mheardLine.mh_payload_type == '!')
            {
                snprintf(buf, 4, "POS");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }
            else
            if(mheardLine.mh_payload_type == '@')
            {
                snprintf(buf, 4, "HY");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }
            else
            {
                snprintf(buf, 4, "???");
                lv_table_set_cell_value(mheard_ta, row, 2, buf);
            }

            snprintf(buf, 8, "%s", getHardwareLong(mheardLine.mh_hw).c_str());
            lv_table_set_cell_value(mheard_ta, row, 3, buf);

            //snprintf(buf, 200, "%3i | ", mheardLine.mh_mod);
            //strRet.concat(buf);

            snprintf(buf, 7, "%4i", mheardLine.mh_rssi);
            lv_table_set_cell_value(mheard_ta, row, 4, buf);

            snprintf(buf, 7, "%4i", mheardLine.mh_snr);
            lv_table_set_cell_value(mheard_ta, row, 5, buf);

            row++;
        }
    }
}

/**
 * displays MHeard on T-Deck
 */
void showPathTDECK()
{
    char buf[200];

    uint16_t row=0;

    lv_table_set_cell_value(path_ta, row, 0, (char*)"Call");
    lv_table_set_cell_value(path_ta, row, 1, (char*)"Time");
    lv_table_set_cell_value(path_ta, row, 2, (char*)"Path");

    row++;

    int anzrow=1;

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
            anzrow++;
    }

    lv_table_set_row_cnt(path_ta, anzrow);

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            if((mheardPathEpoch[iset]+60*60*12) > getUnixClock())    // 12h
            {
                snprintf(buf, 11, "%s", mheardPathCalls[iset]);
                lv_table_set_cell_value(path_ta, row, 0, buf);
            
                unsigned long lt = mheardPathEpoch[iset] + ((60 * 60 + 24) * (int)meshcom_settings.node_utcoff);

                snprintf(buf, 20, "%s", convertUNIXtoString(lt).substring(11, 16).c_str());
                lv_table_set_cell_value(path_ta, row, 1, buf);

                snprintf(buf, 40, "%01u%s/%s", (mheardPathLen[iset] & 0x7F), ((mheardPathLen[iset] & 0x80)?"G":" "), mheardPathBuffer1[iset]);
                lv_table_set_cell_value(path_ta, row, 2, buf);
            }
            else
            {
                mheardPathCalls[iset][0] = 0x00;
            }

            row++;
        }
    }
}
#endif
