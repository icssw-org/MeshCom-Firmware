#include <aprs_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <debugconf.h>
#include <ArduinoJson.h>
#include <time_functions.h>
#include <mheard_functions.h>

#include "printfdeb_functions.h"

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <SD.h>
#include <SPI.h>
#include <t-deck/lv_obj_functions_extern.h>
#include <t-deck/tdeck_helpers.h>
#endif

#if defined(BOARD_T_DECK_PRO)
#include <t-deck-pro/tdeck_pro.h>
#endif

extern bool bDEBUG;

unsigned char mheardBuffer[MAX_MHEARD][60]; //Ringbuffer for MHeard Lines
char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
double mheardLat[MAX_MHEARD];
double mheardLon[MAX_MHEARD];
int mheardAlt[MAX_MHEARD];
unsigned long mheardEpoch[MAX_MHEARD];
int mheardNCount[MAX_MHEARD];

unsigned char mheardPathBuffer1[MAX_MHPATH][50]; //Ringbuffer for MHeard Sourcepath
char mheardPathCalls[MAX_MHPATH][10]; //Ringbuffer for MHeard Key = Call
unsigned long mheardPathEpoch[MAX_MHPATH];
uint8_t mheardPathLen[MAX_MHPATH];

uint8_t mheardWrite = 0;   // counter for ringbuffer
uint8_t mheardPathWrite = 0;   // counter for ringbuffer

#define max_hardware 36

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    String HardWare[max_hardware] = {"no info", "TLO_V2", "TLO_V1", "TLV2_1p6", "TBEAM", "TB_1268", "TB_0p7", "TECHO", "TDECK", "RAK4631", "HELTV21", "HELTV1", "TB_2101", "EB_E22", "HELTV3", "HELT_E290", "TB_1262", "TDECK+", "TB_SUPR", "ES3_E22", "TRACKER_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro", "TBEAM_1W", "HETLV4", "T_ETH_EL", "HETL_T114", "T3S3V13", "TCONPRO", "WLPAPER", "HELT_E213", "ESP32_LORAPRS_E22", "ESP32_LORAPRS_RA01", "T_WATCH_S3"};
#else
    String HardWare[max_hardware] = {"no info", "TLORA_V2", "TLORA_V1", "TLORA_V2_1_1p6", "TBEAM", "TBEAM_1268", "TBEAM_0p7", "T_ECHO", "TDECK", "RAK4631", "HELTEC_V2_1", "HELTEC_V1", "TBEAM_AXP2101", "EBYTE_E22", "HELTEC_V3", "HELTEC_E290", "TBEAM_1262", "TDECK_PLUS", "TBEAM_SUPREME", "ESP_S3_E22", "TRACK_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro", "TBEAM_1W", "HELTEC_V4", "T_ETH_ELITE", "HELTEC_T114", "T3_S3_V13", "T_CON_PRO", "WIRELESS_PAPER", "HELTEC_E213", "ESP32_LORAPRS_E22", "ESP32_LORAPRS_RA01", "T_WATCH_S3"};
#endif

void initMheard()
{
    printlndeb("[INIT]...initMheard");

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        memset(mheardBuffer[iset], 0x00, sizeof(mheardBuffer[iset]));
        memset(mheardCalls[iset], 0x00, sizeof(mheardCalls[iset]));
        mheardLat[iset]=0;
        mheardLon[iset]=0;
        mheardAlt[iset]=0;
        mheardEpoch[iset]=0;
        mheardNCount[iset]=0;
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
    mheardLine.mh_ncount = 0;
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
                case 11: mheardLine.mh_ncount = strdec.toInt(); break;
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
                case 11:
                    strdec.concat(mh_buffer[iset]);
                    break;
                default: break;
            }
        }
    }
}

void saveMHeardPersistence()
{
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        if (!meshcom_settings.node_persist_to_sd)
        {
            if (bDEBUG)
                printlndeb("[TDECK]...MHEARD not persisting to SD");
            return;
        }

        // check to save to SD only every 30 sec
        if(lastsaveMHEARDPersistence + 30000 > millis())
            return;

        lastsaveMHEARDPersistence = millis();

        if(bDisplayCont)
            printlndeb("[TDECK]...MHEARD persisting to SD");

        if(SD.exists("/mheard.dat")) SD.remove("/mheard.dat");
        File file = SD.open("/mheard.dat", FILE_WRITE);
        if(!file) return;
        file.write((uint8_t*)mheardCalls, sizeof(mheardCalls));
        file.write((uint8_t*)mheardBuffer, sizeof(mheardBuffer));
        file.write((uint8_t*)mheardLat, sizeof(mheardLat));
        file.write((uint8_t*)mheardLon, sizeof(mheardLon));
        file.write((uint8_t*)mheardEpoch, sizeof(mheardEpoch));
        file.write((uint8_t*)mheardNCount, sizeof(mheardNCount));
        file.close();
    #endif
}

void savePathPersistence()
{
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        if (!meshcom_settings.node_persist_to_sd)
        {
            if (bDEBUG)
                printlndeb("[TDECK]...PATH not persisting to SD");
            return;
        }

        // check to save to SD only every 30 sec
        if(lastsavePATHPersistence + 30000 > millis())
            return;

        lastsavePATHPersistence = millis();
        
        if(bDisplayCont)
            printlndeb("[TDECK]...PATH persisting to SD");

        if(SD.exists("/mhpath.dat")) SD.remove("/mhpath.dat");
        File file = SD.open("/mhpath.dat", FILE_WRITE);
        if(!file) return;
        file.write((uint8_t*)mheardPathCalls, sizeof(mheardPathCalls));
        file.write((uint8_t*)mheardPathBuffer1, sizeof(mheardPathBuffer1));
        file.write((uint8_t*)mheardPathEpoch, sizeof(mheardPathEpoch));
        file.write((uint8_t*)mheardPathLen, sizeof(mheardPathLen));
        file.close();
    #endif
}


void updateMheard(struct mheardLine &mheardLine, uint8_t isPhoneReady)
{
    struct mheardLine mheardLine_save;

    String strYear = mheardLine.mh_date.substring(0, 4);
    
    //printfdeb("strYear:%s int:%i\n", strYear.c_str(), strYear.toInt());

    if(strYear.toInt() < 2025)
        return;

    //printfdeb("mh_callsign:%s\n", mheardLine.mh_callsign.c_str());

    int ipos=-1;
    int inext=-1;
    
    unsigned long ulmin = getUnixClock();
    int imin = -1;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            // DELETE after 12h
            if((mheardEpoch[iset]+(60*60*12)) < getUnixClock())   // mheard last 12 hours
            {
                mheardCalls[iset][0] = 0x00;
                inext = iset;   // gerade frei geworden
            }
            else
            {
                if(is_equ(mheardCalls[iset], mheardLine.mh_callsign.c_str()))
                {
                    ipos=iset;

                    if(inext >= 0)
                        break;
                }
                else
                {
                    if(mheardEpoch[iset] < ulmin)
                    {
                        ulmin = mheardEpoch[iset];
                    }
                }
            }
        }
        else
        {
            inext=iset; // diese position ist frei
        }
    }

    //printfdeb("inext:%i ipos:%i\n", inext, ipos);

    bool bOld=true;

    if(ipos == -1)
    {
        if(inext >= 0)
        {
            ipos=inext;
        }
        else
        {
            if(imin >= 0)
            {
                ipos=imin;
            }
            else
            {
                ipos=mheardWrite;
                
                mheardWrite++;

                if(mheardWrite >= MAX_MHEARD)
                    mheardWrite=0;
            }
        }

        bOld=false;
    }

    memset(mheardCalls[ipos], 0x00, sizeof(mheardCalls[ipos]));
    int icsize=mheardLine.mh_callsign.length();
    if(icsize > (int)sizeof(mheardCalls[ipos])-1)
        icsize=sizeof(mheardCalls[ipos])-1;
    memcpy(mheardCalls[ipos], mheardLine.mh_callsign.c_str(), icsize);
    
    mheardEpoch[ipos] = getUnixClock();

    if(bOld)
    {
        // REP action
        decodeMHeard(mheardBuffer[ipos], mheardLine_save);

        // da bei dem eintreffen von updateMHeard kein NCOUNT dabei ist
        // wird dieser aus dem bestehenden Tabellen-Wert  mheardNCount[]; ergänzt
        
        if(mheardLine.mh_ncount == 0)
            mheardLine.mh_ncount = mheardNCount[ipos]; // 8immer aus array nehmen

        // Distance only on new calculation
        if(mheardLine.mh_dist < 0)
            mheardLine.mh_dist = mheardLine_save.mh_dist;
    }

    mheardNCount[ipos] = mheardLine.mh_ncount;

    char cBuffer[sizeof(mheardBuffer[ipos])];
    snprintf(cBuffer, sizeof(cBuffer), "%s|%s|%c|%i|%u|%i|%i|%.1lf|%i|%i|%i|", mheardLine.mh_date.c_str(), mheardLine.mh_time.c_str(), mheardLine.mh_payload_type, mheardLine.mh_hw,
     mheardLine.mh_mod, mheardLine.mh_rssi, mheardLine.mh_snr, mheardLine.mh_dist, mheardLine.mh_path_len, mheardLine.mh_mesh, mheardLine.mh_ncount); 
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
    mhdoc["NCNT"] = mheardLine.mh_ncount;
    // Link-Kette der HEY-Bake, RSSI/SNR je Hop. Nur wenn belegt - ein leeres Feld in jeden
    // Datensatz zu haengen kostet Platz im BLE-Puffer, ohne etwas zu sagen.
    if(mheardLine.mh_path_payload.length() > 0)
        mhdoc["PP"] = mheardLine.mh_path_payload.c_str();
    // Ursprung des Pakets: CALL ist der letzte Hop, SRC der Absender. Beides liegt hier
    // bereits im Struct (lora_functions.cpp fuellt es aus aprsmsg.msg_source_call).
    mhdoc["SRC"] = mheardLine.mh_sourcecallsign.c_str();
    // HEY-Baken eines Gateways tragen "HG" statt "H" als Ziel - dieselbe Auskunft, die
    // updateMheardPath() unten schon fuer die eigene Pfadtabelle auswertet (| 0x80).
    mhdoc["GW"] = (mheardLine.mh_destinationpath == "HG") ? 1 : 0;

    // send to Phone
    uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
    bleBuffer[0] = 0x44;
    // Puffergroesse begrenzen, nicht die gemessene JSON-Laenge: serializeJson schreibt
    // hoechstens so viele Bytes wie erlaubt und liefert die tatsaechlich geschriebene
    // Anzahl zurueck. 1 Byte Header + Null-Terminator bleiben reserviert.
    size_t json_len = serializeJson(mhdoc, bleBuffer+1, sizeof(bleBuffer)-1);

    if(isPhoneReady == 1)
        addBLEOutBuffer(bleBuffer, json_len+1);

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

    showMHeardTDECK();
    
    #ifdef HEAP_TEST    // log not used
        // Log MHeard to SD
        String json = "{";
        json += "\"call\":\"" + escape_json(mheardLine.mh_callsign) + "\",";
        json += "\"date\":\"" + escape_json(mheardLine.mh_date) + "\",";
        json += "\"time\":\"" + escape_json(mheardLine.mh_time) + "\",";
        json += "\"hw\":" + String(mheardLine.mh_hw) + ",";
        json += "\"mod\":" + String(mheardLine.mh_mod) + ",";
        json += "\"rssi\":" + String(mheardLine.mh_rssi) + ",";
        json += "\"snr\":" + String(mheardLine.mh_snr) + ",";
        json += "\"dist\":" + String(mheardLine.mh_dist + ",", 1);
        json += "\"ncount\":" + String(mheardLine.mh_ncount);
        json += "}";
        log_json_to_sd("/mheard.json", json);
    #endif

    saveMHeardPersistence();
    
    #endif

    #if defined(BOARD_T_DECK_PRO)
    TDeck_pro_mheard_disp();
    #endif
}

void updateHeyPath(struct mheardLine &mheardLine)
{
    struct mheardLine mheardLine_save;

    String strYear = mheardLine.mh_date.substring(0, 4);
    if(strYear.toInt() < 2025)
        return;

    // exclude the owncall
    if(mheardLine.mh_sourcecallsign == meshcom_settings.node_call)
        return;

    for(int imh=0; imh<MAX_MHEARD; imh++)
    {
        if(mheardCalls[imh][0] != 0x00)
        {
            if(is_equ(mheardCalls[imh], mheardLine.mh_sourcecallsign.c_str()))
            {
                if(bDisplayCont)
                {
                    printdeb("Path_Payload:");
                    printdeb(mheardLine.mh_sourcecallsign);
                    printdeb(" ");
                    printdeb(mheardLine.mh_path_payload);
                    printdeb(" ");
                }

                // NeighborCount einfügen
                // check new/old format
                // new R99; R99;77,7 ...
                // old R99,99,99;77,7 ... oder R99,77  ... oder R99
                // old R99,99;.... kein NCount

                // correct old format
                mheardLine.mh_path_payload.concat(";");

                int ipos=mheardLine.mh_path_payload.indexOf(";");

                if(ipos > 0 && mheardLine.mh_path_payload.startsWith("R"))
                {
                    // count comma
                    int icomma = 0;
                    for(int i=1; i<ipos; i++)
                    {
                        if(mheardLine.mh_path_payload.charAt(i) == ',')
                            icomma++;
                    }

                    // gültig
                    // R99;
                    // R99,99,99;

                    // ungültig
                    // R99,99;
                    
                    if(icomma == 0 || icomma == 2)
                    {
                        if(bDisplayCont)
                        {
                            printdeb(mheardLine.mh_path_payload.substring(1, ipos));
                            printdeb(" count:");
                        }

                        mheardLine.mh_ncount = mheardLine.mh_path_payload.substring(1, ipos).toInt();
                        mheardNCount[imh] = mheardLine.mh_ncount;

                        // REP action
                        decodeMHeard(mheardBuffer[imh], mheardLine_save);

                        char cBuffer[sizeof(mheardBuffer[imh])];
                        snprintf(cBuffer, sizeof(cBuffer), "%s|%s|%c|%i|%u|%i|%i|%.1lf|%i|%i|%i|", mheardLine.mh_date.c_str(), mheardLine.mh_time.c_str(), mheardLine.mh_payload_type, mheardLine_save.mh_hw,
                        mheardLine_save.mh_mod, mheardLine_save.mh_rssi, mheardLine_save.mh_snr, mheardLine_save.mh_dist, mheardLine.mh_path_len, mheardLine.mh_mesh, mheardLine.mh_ncount);
                        memcpy(mheardBuffer[imh], cBuffer, sizeof(cBuffer));
                    }
                }

                if(bDisplayCont)
                    printlndeb("");
            }
        }
    }

    int ipos=-1;
    int inext=-1;
    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            // PATH DELETE after 1 Hours
            if((mheardPathEpoch[iset]+(60*60*12)) < getUnixClock())  // mheard last 12 hours
            {
                mheardPathCalls[iset][0] = 0x00;
            }
            else
            {
                if(is_equ(mheardPathCalls[iset], mheardLine.mh_sourcecallsign.c_str()))
                {
                    ipos=iset;
                    if(inext >= 0)
                        break;
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
    memcpy(mheardPathCalls[ipos], mheardLine.mh_sourcecallsign.c_str(), sizeof(mheardPathCalls[ipos]));

    //printfdeb("PATH:%i <%s> <%s> %i %i\n", ipos,  mheardLine.mh_sourcepath.c_str(), mheardLine.mh_sourcepath.substring(ips).c_str(), ips, ipc);

    memset(mheardPathBuffer1[ipos], 0x00, sizeof(mheardPathBuffer1[ipos]));
    memcpy(mheardPathBuffer1[ipos], mheardLine.mh_sourcepath.substring(ips).c_str(), sizeof(mheardPathBuffer1[ipos]));
    mheardPathBuffer1[ipos][49] = 0x00;
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

    savePathPersistence();
}

int getMheardCount()
{
    int imhcount=0;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            if((mheardEpoch[iset]+(60*60)) > getUnixClock())  // mheard count only last hour
            {
                imhcount++;
            }
        }
    }

    return imhcount;
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
            if((mheardEpoch[iset]+(60*60*12)) > getUnixClock())  // mheard last 12 hours
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

                xval = getValue(mhstringdec, '|', 9);
                mheardLine.mh_mesh = xval.toInt();

                xval = getValue(mhstringdec, '|', 10);
                mheardLine.mh_ncount = xval.toInt();

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
                mheardLine.mh_ncount = mheardNCount[iset];
                mhdoc["NCNT"] = mheardNCount[iset]; // 8immer aus array nehmen

                // send to Phone
                uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
                bleBuffer[0] = 0x44;
                // wie oben: Puffergroesse begrenzen statt der gemessenen JSON-Laenge
                size_t json_len = serializeJson(mhdoc, bleBuffer+1, sizeof(bleBuffer)-1);

                addBLEComToOutBuffer(bleBuffer, json_len+1);
            }
        }
    }
}

void showMHeard()
{
    printlndeb("\n/-----------------------------------------------------------------------------------------------------\\");
    printlndeb("|MHeard call |    date    |   time   | typ | source hardware | mod | rssi |  snr | dist | pl | m | nc |");

    mheardLine mheardLine;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            if((mheardEpoch[iset]+(60*60*12)) > getUnixClock())  // mheard last 12 hours
            {
                printlndeb("|------------|------------|----------|-----|-----------------|-----|------|------|------|----|---|----|");

                printfdeb("| %-10.10s | ", mheardCalls[iset]);
                
                decodeMHeard(mheardBuffer[iset], mheardLine);

                printfdeb("%-10.10s | ", mheardLine.mh_date.c_str());
                printfdeb("%-8.8s | ", mheardLine.mh_time.c_str());

                printfdeb("%-3.3s | ", getPayloadType(mheardLine.mh_payload_type));

                printfdeb("%-11.11s/%03i | ", getHardwareLong(mheardLine.mh_hw).c_str(), mheardLine.mh_hw);

                printfdeb("%01X/%01i | ", (mheardLine.mh_mod>>4), (mheardLine.mh_mod & 0xf));
                printfdeb("%4i | ", mheardLine.mh_rssi);
                printfdeb("%4i |", mheardLine.mh_snr);
                printfdeb("%5.1lf |", mheardLine.mh_dist);
                printfdeb("%3i |", mheardLine.mh_path_len);
                printfdeb("%2i |", mheardLine.mh_mesh);
                printfdeb("%3i |\n", mheardNCount[iset]); // 8immer aus array nehmen
            }
        }
    }

    printlndeb("\\-----------------------------------------------------------------------------------------------------/\n");
}

void showPath()
{
    printlndeb("\n/---------------------------------------------------------------------------------------\\");
    printlndeb("|       date          | lng/Gate/Path                                                   |");

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            if((mheardPathEpoch[iset]+(60*60*12)) > getUnixClock())  // path last 12 hours
            {
                printlndeb("|---------------------|-----------------------------------------------------------------|");

                //printfdeb("| %-10.10s | ", mheardPathCalls[iset]);

                unsigned long lt = mheardPathEpoch[iset] + (long)(meshcom_settings.node_utcoff * 3600.0);
                
                printfdeb("| %-19.19s | ", convertUNIXtoString(lt).c_str()); // yyyy.mm.dd hh:mm:ss

                printfdeb("%01u%s/%-10.10s %-49.49s |\n", (mheardPathLen[iset] & 0x7F), ((mheardPathLen[iset] & 0x80)?"G":" "), mheardPathCalls[iset], mheardPathBuffer1[iset]);
            }
            else
            {
                mheardPathCalls[iset][0] = 0x00;
            }
        }
    }

    printlndeb("\\---------------------------------------------------------------------------------------/\n");
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
    if(ihw == 51)
        ihw=25;
    if(ihw == 52)
        ihw=26;
    if(ihw == 53)
        ihw=27;
    if(ihw == 54)
        ihw=28;
    if(ihw == 55)
        ihw=29;
    if(ihw == 56)
        ihw=30;
    if(ihw == 57)
        ihw=31;
    if(ihw == 58)
        ihw=32;   // HELTEC_E213 (Vision Master E213) -> Array-Index 32
    if(ihw == 59)
        ihw=33;   // ESP32_LORAPRS_E22 -> Array-Index 33
    if(ihw == 60)
        ihw=34;   // ESP32_LORAPRS_RA01 -> Array-Index 34
    if(ihw == 61)
        ihw=35;   // T_WACH_s3
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
    lv_table_set_cell_value(mheard_ta, row, 4, (char*)"SSI");
    lv_table_set_cell_value(mheard_ta, row, 5, (char*)"SNR");
    lv_table_set_cell_value(mheard_ta, row, 6, (char*)"NC");

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

            snprintf(buf, 7, "%4i", mheardNCount[iset]); // 8immer aus array nehmen
            lv_table_set_cell_value(mheard_ta, row, 6, buf);

            row++;
        }
    }
}

/**
 * displays MHeard on T-Deck
 */
void showPathTDECK()
{
    char buf[60];

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
            snprintf(buf, 11, "%s", mheardPathCalls[iset]);
            lv_table_set_cell_value(path_ta, row, 0, buf);
        
            unsigned long lt = mheardPathEpoch[iset] + (long)(meshcom_settings.node_utcoff * 3600.0);

            snprintf(buf, 20, "%s", convertUNIXtoString(lt).substring(11, 16).c_str());
            lv_table_set_cell_value(path_ta, row, 1, buf);

            snprintf(buf, 50, "%01u%s/%s", (mheardPathLen[iset] & 0x7F), ((mheardPathLen[iset] & 0x80)?"G":" "), mheardPathBuffer1[iset]);
            lv_table_set_cell_value(path_ta, row, 2, buf);

            row++;
        }
    }
}
#endif

void loadMHeardPersistence()
{
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        if (!meshcom_settings.node_persist_to_sd)
        {
            if (bDEBUG)
                printlndeb("[TDECK]...MHEARD not persisting from SD");
            return;
        }

        if(!SD.exists("/mheard.dat")) return;
        File file = SD.open("/mheard.dat", FILE_READ);
        if(!file) return;

// FIX — vor den file.read() Aufrufen einfuegen:
        size_t expected_mh = sizeof(mheardCalls) + sizeof(mheardBuffer) + sizeof(mheardLat)
                           + sizeof(mheardLon) + sizeof(mheardEpoch) + sizeof(mheardNCount);
        if(file.size() != expected_mh) {
            printfdeb("[TDECK]...mheard.dat size mismatch (%u != %u), deleting\n", file.size(), expected_mh);
            file.close();
            SD.remove("/mheard.dat");
            return;
        }

        file.read((uint8_t*)mheardCalls, sizeof(mheardCalls));
        file.read((uint8_t*)mheardBuffer, sizeof(mheardBuffer));
        file.read((uint8_t*)mheardLat, sizeof(mheardLat));
        file.read((uint8_t*)mheardLon, sizeof(mheardLon));
        file.read((uint8_t*)mheardEpoch, sizeof(mheardEpoch));
        file.read((uint8_t*)mheardNCount, sizeof(mheardNCount));
        file.close();
        showMHeardTDECK();
    #endif
}

void loadPathPersistence()
{
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
        if (!meshcom_settings.node_persist_to_sd)
        {
            if (bDEBUG)
                printlndeb("[TDECK]...PATH not persisting from SD");
            return;
        }

        if(!SD.exists("/mhpath.dat")) return;
        File file = SD.open("/mhpath.dat", FILE_READ);
        if(!file) return;

// FIX — vor den file.read() Aufrufen einfuegen:
        size_t expected_path = sizeof(mheardPathCalls) + sizeof(mheardPathBuffer1)
                             + sizeof(mheardPathEpoch) + sizeof(mheardPathLen);
        if(file.size() != expected_path) {
            printfdeb("[TDECK]...mhpath.dat size mismatch (%u != %u), deleting\n", file.size(), expected_path);
            file.close();
            SD.remove("/mhpath.dat");
            return;
        }

        file.read((uint8_t*)mheardPathCalls, sizeof(mheardPathCalls));
        file.read((uint8_t*)mheardPathBuffer1, sizeof(mheardPathBuffer1));
        file.read((uint8_t*)mheardPathEpoch, sizeof(mheardPathEpoch));
        file.read((uint8_t*)mheardPathLen, sizeof(mheardPathLen));
        file.close();
        showPathTDECK();
    #endif
}

unsigned long getLatestMHeardTimestamp()
{
    unsigned long max_ts = 0;
    for(int i=0; i<MAX_MHEARD; i++)
    {
        if(mheardEpoch[i] > max_ts) max_ts = mheardEpoch[i];
    }
    for(int i=0; i<MAX_MHPATH; i++)
    {
        if(mheardPathEpoch[i] > max_ts) max_ts = mheardPathEpoch[i];
    }
    return max_ts;
}