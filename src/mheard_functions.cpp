#include "mc_text.h"
#include <aprs_functions.h>
#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <byte_fifo.h>
#include <mheard_throttle.h>
#include <debugconf.h>
#include <ArduinoJson.h>
#include <ble_json_frame.h>
#include <time_functions.h>
#include <mheard_functions.h>
#include <mheard_record.h>

#include "printfdeb_functions.h"

// NATIVE_BUILD (pio test -e native_parsers, PT-01): these headers pull in
// SD/SPI and the T-Deck LVGL UI chain. The BOARD_T_DECK*/BOARD_T_DECK_PRO
// guards below already keep them out of any non-T-Deck firmware build, but
// PlatformIO's Library Dependency Finder text-scans #include lines without
// evaluating those macros, so it still tries (and fails) to pull lvgl into
// the native test build. NATIVE_BUILD is never defined for a firmware
// build, so this changes nothing there -- it only keeps the native test
// binary, which never needs these headers, from tripping over the LDF scan.
#ifndef NATIVE_BUILD
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <SD.h>
#include <SPI.h>
#include <t-deck/lv_obj_functions_extern.h>
#include <t-deck/tdeck_helpers.h>
#endif

#if defined(BOARD_T_DECK_PRO)
#include <t-deck-pro/tdeck_pro.h>

// R2-04/F4: mheardFormatDate()/mheardFormatTime() (mheard_record.h) scheitern
// LEISE, wenn ihr Ziel zu klein ist -- sie schreiben dann "" und kehren
// zurueck. mheardLine.mh_date/mh_time (aprs_structures.h) sind ihre einzigen
// Ziele. Ein zu kurzes MC_DATE_LEN gaebe also keinen Compile-Fehler, sondern
// jeden MHeard-Eintrag mit leerem Datum -- und updateMheard()/updateHeyPath()
// pruefen mcSliceToLong(mh_date, 0, 4) < 2025 und wuerfen dann JEDEN Eintrag
// weg. Beide Header sind in dieser Uebersetzungseinheit sichtbar, also wird
// die Kopplung hier geprueft, genau wie aprs_functions.cpp:18-20 es fuer
// MC_CALL_LEN_Z/MC_PAYLOAD_LEN tut.
static_assert(MC_DATE_LEN >= 11,
              "MC_DATE_LEN zu klein fuer mheardFormatDate() -- mh_date bliebe "
              "leer, ohne Compile-Fehler, und jeder MHeard-Eintrag fiele weg");
static_assert(MC_TIME_LEN >= 9,
              "MC_TIME_LEN zu klein fuer mheardFormatTime() -- mh_time bliebe "
              "leer, ohne Compile-Fehler");

#endif
#endif // NATIVE_BUILD

extern bool bDEBUG;

// R2-01: war `unsigned char mheardBuffer[MAX_MHEARD][60]` -- jeder Eintrag
// eine pipe-getrennte Zeichenkette, die bei jedem Lesen zeichenweise
// zurueckzerlegt wurde. Jetzt der Datensatz selbst, 20 statt 60 Byte.
// Begruendung und Rundungsfrage in src/mheard_record.h.
MheardRecord mheardRecords[MAX_MHEARD];
char mheardCalls[MAX_MHEARD][10]; //Ringbuffer for MHeard Key = Call
// R3-12: float statt double. Ein float haelt ~7 signifikante Dezimalstellen;
// gebraucht werden 4 Nachkommastellen bei zweistelligem Grad (48.1234 ->
// 6 Stellen), also bleibt eine Stelle Reserve. Das entspricht rund 1 m,
// und die MHeard-Liste ist eine Nachbarschaftsanzeige, keine Navigation.
//
// ACHTUNG: diese beiden Felder werden auf dem T-Deck ROH in /mheard.dat
// geschrieben (saveMHeardPersistence, file.write((uint8_t*)..., sizeof)).
// Die Typaenderung aendert die Dateigroesse. Das ist abgefangen: der
// Ladeweg vergleicht file.size() gegen die Summe der sizeof() und
// loescht die Datei bei Abweichung (siehe readMHeardPersistence). Eine
// vorhandene Datei geht also einmalig verloren und wird neu aufgebaut --
// kein Fehlverhalten, aber es passiert und gehoert hierher geschrieben.
//
// Drei Dateien deklarieren diese Felder von Hand als extern
// (lora_functions.cpp, web_functions.cpp). Ein vergessenes davon linkt
// STILL und liest in der falschen Breite; seit 2026-09-16 deckt
// test/golden/carve_extern_lint.py auch diese Dateien ab.
float mheardLat[MAX_MHEARD];
float mheardLon[MAX_MHEARD];
int mheardAlt[MAX_MHEARD];
unsigned long mheardEpoch[MAX_MHEARD];

// NC-01 (BACKLOG SS3.8o): monotonic heard-time, parallel to mheardEpoch[].
// mheardEpoch[] stays wall-clock-based (getUnixClock()) and unchanged for
// display/JSON, but a node with no NTP/GPS has no valid wall clock --
// getUnixClock() runs mktime() on unset date fields, which yields garbage
// (up to (unsigned long)-1). Aging entries by mheardEpoch[]+window vs.
// getUnixClock() then wraps and every entry looks stale, even ones just
// heard (loop_functions.cpp NCNT tag stays 0 forever). mheardMillis[] ages
// entries by millis() instead, which needs no wall clock. Declared uint32_t
// (not unsigned long): millis() is a 32-bit counter on every real target
// (nRF52840/ESP32 `unsigned long` is 4 bytes there), and the rollover-safe
// subtraction below relies on that width -- on the native/x86_64 test host
// `unsigned long` is 8 bytes, so an explicit uint32_t keeps the same
// wraparound behaviour there too instead of silently losing it.
uint32_t mheardMillis[MAX_MHEARD];
int mheardNCount[MAX_MHEARD];

// NC-01: aging windows in milliseconds, mirroring the epoch-second windows
// they replace (1 h / 12 h) at every mheardMillis[] comparison below. Both
// are far below the ~49.7 day uint32_t rollover, so plain rollover-safe
// unsigned subtraction (see comparisons below) is correct with no extra
// rollover handling.
#define MHEARD_AGE_WINDOW_MS   (60UL*60UL*1000UL)         // 1 h -- getMheardCount()
#define MHEARD_PRUNE_WINDOW_MS (60UL*60UL*12UL*1000UL)    // 12 h -- updateMheard()/sendMheard()/showMHeard()

unsigned char mheardPathBuffer1[MAX_MHPATH][52]; //Ringbuffer for MHeard Sourcepath
char mheardPathCalls[MAX_MHPATH][10]; //Ringbuffer for MHeard Key = Call
unsigned long mheardPathEpoch[MAX_MHPATH];

// NC-02 (BACKLOG SS3.8o): mheardMillis[]'s (NC-01, above) exact counterpart
// for the path ringbuffer -- mheardPathEpoch[] has the identical
// clockless-node hazard at updateHeyPath()'s PATH DELETE check and at
// showPath(). Same uint32_t rationale as mheardMillis[] (rollover-safe
// millis() width on native/x86_64).
uint32_t mheardPathMillis[MAX_MHPATH];
uint8_t mheardPathLen[MAX_MHPATH];

uint8_t mheardWrite = 0;   // counter for ringbuffer
uint8_t mheardPathWrite = 0;   // counter for ringbuffer

#define max_hardware 36

// R2-07 (DRY audit): pure constant lookup table, converted from String[] to
// const char*[] -- was 36 heap-backed String objects, now 36 pointers into
// .rodata plus zero allocations. getHardwareLong() below still returns
// String (unchanged call contract); the const char* -> String conversion
// happens once, at that single return, not once per table entry at startup.
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    static const char* const HardWare[max_hardware] = {"no info", "TLO_V2", "TLO_V1", "TLV2_1p6", "TBEAM", "TB_1268", "TB_0p7", "TECHO", "TDECK", "RAK4631", "HELTV21", "HELTV1", "TB_2101", "EB_E22", "HELTV3", "HELT_E290", "TB_1262", "TDECK+", "TB_SUPR", "ES3_E22", "TRACKER_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro", "TBEAM_1W", "HETLV4", "T_ETH_EL", "HETL_T114", "T3S3V13", "TCONPRO", "WLPAPER", "HELT_E213", "ESP32_LORAPRS_E22", "ESP32_LORAPRS_RA01", "T_WATCH_S3"};
#else
    static const char* const HardWare[max_hardware] = {"no info", "TLORA_V2", "TLORA_V1", "TLORA_V2_1_1p6", "TBEAM", "TBEAM_1268", "TBEAM_0p7", "T_ECHO", "TDECK", "RAK4631", "HELTEC_V2_1", "HELTEC_V1", "TBEAM_AXP2101", "EBYTE_E22", "HELTEC_V3", "HELTEC_E290", "TBEAM_1262", "TDECK_PLUS", "TBEAM_SUPREME", "ESP_S3_E22", "TRACK_V3", "STICK_V3", "T5_EPAPER", "TPAGER", "TDECKpro", "TBEAM_1W", "HELTEC_V4", "T_ETH_ELITE", "HELTEC_T114", "T3_S3_V13", "T_CON_PRO", "WIRELESS_PAPER", "HELTEC_E213", "ESP32_LORAPRS_E22", "ESP32_LORAPRS_RA01", "T_WATCH_S3"};
#endif

void initMheard()
{
    printlndeb("[INIT]...initMheard");

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        memset(&mheardRecords[iset], 0x00, sizeof(mheardRecords[iset]));
        memset(mheardCalls[iset], 0x00, sizeof(mheardCalls[iset]));
        mheardLat[iset]=0;
        mheardLon[iset]=0;
        mheardAlt[iset]=0;
        mheardEpoch[iset]=0;
        mheardMillis[iset]=0;
        mheardNCount[iset]=0;
    }

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        memset(mheardPathBuffer1[iset], 0x00, sizeof(mheardPathBuffer1[iset]));
        memset(mheardPathCalls[iset], 0x00, sizeof(mheardPathCalls[iset]));
        mheardPathEpoch[iset]=0;
        mheardPathMillis[iset]=0;
        mheardPathLen[iset]=0;
    }

    mheardWrite=0;
    mheardPathWrite = 0;

}

void initMheardLine(struct mheardLine &mheardLine)
{
    mheardLine.mh_callsign[0] = 0;
    mheardLine.mh_sourcecallsign[0] = 0;
    mheardLine.mh_sourcepath[0] = 0;
    mheardLine.mh_destinationpath[0] = 0;
    mheardLine.mh_date[0] = 0;
    mheardLine.mh_time[0] = 0;
    mheardLine.mh_path_payload[0] = 0;
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

// R2-01: hiess frueher decodeMHeard() und zerlegte eine pipe-getrennte
// Zeichenkette zeichenweise mit Arduino-String-Anhaengen -- rund 55 Durchlaeufe
// und mehrere Heap-Anforderungen je gelesenem Eintrag, auf den Pfaden von
// `--mheard`, des JSON-Registers und der Web-Oberflaeche. Jetzt Feldkopien.
void mheardLineFromRecord(const MheardRecord &rec, struct mheardLine &mheardLine)
{
    initMheardLine(mheardLine);

    // mh_date/mh_time sind exakt MC_DATE_LEN/MC_TIME_LEN breit (siehe
    // aprs_structures.h) -- derselbe Vertrag, den mheardFormatDate()/
    // mheardFormatTime() verlangen, also kein Zwischenpuffer mehr noetig.
    mheardFormatDate(rec, mheardLine.mh_date, sizeof(mheardLine.mh_date));
    mheardFormatTime(rec, mheardLine.mh_time, sizeof(mheardLine.mh_time));

    mheardLine.mh_payload_type = rec.mr_type;
    mheardLine.mh_hw           = rec.mr_hw;
    mheardLine.mh_mod          = rec.mr_mod;
    mheardLine.mh_rssi         = rec.mr_rssi;
    mheardLine.mh_snr          = rec.mr_snr;
    mheardLine.mh_dist         = rec.mr_dist;
    mheardLine.mh_path_len     = rec.mr_path_len;
    mheardLine.mh_mesh         = rec.mr_mesh;
    mheardLine.mh_ncount       = rec.mr_ncount;
}

// Gegenstueck zum Obigen. Datum und Uhrzeit kommen als feste Breite aus
// getDateString()/getTimeString(); schlaegt das Format fehl, bleiben die
// Felder auf ihrem Nullwert stehen, statt halb gefuellt zu werden.
void mheardRecordFromLine(const struct mheardLine &mheardLine, MheardRecord &rec)
{
    memset(&rec, 0x00, sizeof(rec));

    mheardSetDate(rec, mheardLine.mh_date);
    mheardSetTime(rec, mheardLine.mh_time);

    rec.mr_type     = mheardLine.mh_payload_type;
    rec.mr_hw       = mheardLine.mh_hw;
    rec.mr_mod      = mheardLine.mh_mod;
    rec.mr_rssi     = mheardLine.mh_rssi;
    rec.mr_snr      = mheardLine.mh_snr;
    rec.mr_dist     = mheardRoundDist(mheardLine.mh_dist);
    rec.mr_path_len = mheardLine.mh_path_len;
    rec.mr_mesh     = mheardLine.mh_mesh;
    rec.mr_ncount   = mheardLine.mh_ncount;
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
        if((uint32_t)(millis() - lastsaveMHEARDPersistence) < 30000)
            return;

        lastsaveMHEARDPersistence = millis();

        if(bDisplayCont)
            printlndeb("[TDECK]...MHEARD persisting to SD");

        if(SD.exists("/mheard.dat")) SD.remove("/mheard.dat");
        File file = SD.open("/mheard.dat", FILE_WRITE);
        if(!file) return;
        file.write((uint8_t*)mheardCalls, sizeof(mheardCalls));
        file.write((uint8_t*)mheardRecords, sizeof(mheardRecords));
        file.write((uint8_t*)mheardLat, sizeof(mheardLat));
        file.write((uint8_t*)mheardLon, sizeof(mheardLon));
        file.write((uint8_t*)mheardEpoch, sizeof(mheardEpoch));
        // mheardMillis[] (NC-01) is intentionally NOT persisted: millis()
        // resets to 0 every boot, so a saved monotonic stamp would be
        // meaningless after a reboot. loadMHeardPersistence() re-derives it.
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
        if((uint32_t)(millis() - lastsavePATHPersistence) < 30000)
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
        // mheardPathMillis[] (NC-02) is intentionally NOT persisted, same
        // reasoning as mheardMillis[] in saveMHeardPersistence() above:
        // millis() resets to 0 every boot. loadPathPersistence() re-derives it.
        file.write((uint8_t*)mheardPathLen, sizeof(mheardPathLen));
        file.close();
    #endif
}


void updateMheard(struct mheardLine &mheardLine, uint8_t isPhoneReady)
{
    struct mheardLine mheardLine_save;

    if(mcSliceToLong(mheardLine.mh_date, 0, 4) < 2025)
        return;

    //printfdeb("mh_callsign:%s\n", mheardLine.mh_callsign);

    int ipos=-1;
    int inext=-1;

    // MH-02 (BACKLOG SS3.8o): eviction candidate when the table is full and
    // this callsign is new -- the entry with the largest monotonic age
    // (millis() elapsed since last heard) is the oldest, so it is evicted
    // first. Was tracked by mheardEpoch[] (wall clock) via `ulmin`, but
    // `imin` was never assigned in the loop below, so this branch was dead
    // (fell through to the sequential mheardWrite ring, see below). Ages by
    // mheardMillis[] instead of mheardEpoch[], same clockless-node reason as
    // NC-01: mheardEpoch[] can be garbage on a node with no valid wall clock.
    uint32_t agemax = 0;
    int imin = -1;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            // DELETE after 12h -- aged by millis(), not by the (possibly
            // clockless) wall clock, see mheardMillis[] above (NC-01).
            if((uint32_t)(millis() - mheardMillis[iset]) >= MHEARD_PRUNE_WINDOW_MS)
            {
                mheardCalls[iset][0] = 0x00;
                inext = iset;   // gerade frei geworden
            }
            else
            {
                if(is_equ(mheardCalls[iset], mheardLine.mh_callsign))
                {
                    ipos=iset;

                    if(inext >= 0)
                        break;
                }
                else
                {
                    uint32_t age = (uint32_t)(millis() - mheardMillis[iset]);
                    if(age >= agemax)
                    {
                        agemax = age;
                        imin = iset;   // MH-02: oldest-so-far, evict this one if needed
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
    int icsize=(int)strlen(mheardLine.mh_callsign);
    if(icsize > (int)sizeof(mheardCalls[ipos])-1)
        icsize=sizeof(mheardCalls[ipos])-1;
    memcpy(mheardCalls[ipos], mheardLine.mh_callsign, icsize);
    
    mheardEpoch[ipos] = getUnixClock();
    mheardMillis[ipos] = (uint32_t)millis();   // NC-01: monotonic heard-time

    if(bOld)
    {
        // REP action
        mheardLineFromRecord(mheardRecords[ipos], mheardLine_save);

        // da bei dem eintreffen von updateMHeard kein NCOUNT dabei ist
        // wird dieser aus dem bestehenden Tabellen-Wert  mheardNCount[]; ergänzt
        
        if(mheardLine.mh_ncount == 0)
            mheardLine.mh_ncount = mheardNCount[ipos]; // 8immer aus array nehmen

        // Distance only on new calculation
        if(mheardLine.mh_dist < 0)
            mheardLine.mh_dist = mheardLine_save.mh_dist;
    }

    mheardNCount[ipos] = mheardLine.mh_ncount;

    mheardRecordFromLine(mheardLine, mheardRecords[ipos]);

    // generate JSON
    JsonDocument mhdoc;

    mhdoc["TYP"] = "MH";
    mhdoc["CALL"] = mheardLine.mh_callsign;
    mhdoc["DATE"] = mheardLine.mh_date;
    mhdoc["TIME"] = mheardLine.mh_time;
    mhdoc["PLT"] = (uint8_t)mheardLine.mh_payload_type;
    mhdoc["HW"] = mheardLine.mh_hw;
    mhdoc["MOD"] = mheardLine.mh_mod;
    mhdoc["RSSI"] = mheardLine.mh_rssi;
    mhdoc["SNR"] = mheardLine.mh_snr;
    mhdoc["DIST"] = mheardLine.mh_dist;
    mhdoc["PL"] = mheardLine.mh_path_len;
    mhdoc["MESH"] = mheardLine.mh_mesh;
    mhdoc["NCNT"] = mheardLine.mh_ncount;

    // send to Phone
    uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
    bleBuffer[0] = 0x44;
    // Schranke ist der Puffer, nicht die JSON-Laenge (UP-01, BND-03)
    uint16_t frame_len = bleJsonFrame(mhdoc, bleBuffer, sizeof(bleBuffer));

    if(isPhoneReady == 1)
        addBLEOutBuffer(bleBuffer, frame_len);

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
        // Stand hier als `String(mheardLine.mh_dist + ",", 1)`. Das ist
        // `double + const char*` und laesst sich GAR NICHT uebersetzen -- die
        // Zeile steht seit jeher in `#ifdef HEAP_TEST`, und HEAP_TEST ist im
        // ganzen Baum nirgends definiert, also hat sie nie ein Compiler
        // gesehen. Gemeint war die Klammer eine Stelle weiter rechts.
        json += "\"dist\":" + String(mheardLine.mh_dist, 1) + ",";
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

    if(mcSliceToLong(mheardLine.mh_date, 0, 4) < 2025)
        return;

    // exclude the owncall
    if(is_equ(mheardLine.mh_sourcecallsign, meshcom_settings.node_call))
        return;

    for(int imh=0; imh<MAX_MHEARD; imh++)
    {
        if(mheardCalls[imh][0] != 0x00)
        {
            if(is_equ(mheardCalls[imh], mheardLine.mh_sourcecallsign))
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
                mcAppend(mheardLine.mh_path_payload, sizeof(mheardLine.mh_path_payload), ";");

                int ipos=mcIndexOf(mheardLine.mh_path_payload, ';');

                if(ipos > 0 && mcStartsWith(mheardLine.mh_path_payload, "R"))
                {
                    // count comma
                    int icomma = 0;
                    for(int i=1; i<ipos; i++)
                    {
                        if(mheardLine.mh_path_payload[i] == ',')
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
                            // Debug-Ausschnitt mh_path_payload[1..ipos) -- reine
                            // Anzeige, deshalb ein lokaler Puffer statt eines
                            // neuen mc_text.h-Helfers nur dafuer.
                            char dbgbuf[16];
                            size_t dbglen = (ipos > 1) ? (size_t)(ipos - 1) : 0;
                            if(dbglen >= sizeof(dbgbuf))
                                dbglen = sizeof(dbgbuf) - 1;
                            memcpy(dbgbuf, mheardLine.mh_path_payload + 1, dbglen);
                            dbgbuf[dbglen] = 0;
                            printdeb(dbgbuf);
                            printdeb(" count:");
                        }

                        mheardLine.mh_ncount = (uint8_t)mcSliceToLong(mheardLine.mh_path_payload, 1, (size_t)ipos);
                        mheardNCount[imh] = mheardLine.mh_ncount;

                        // REP action
                        mheardLineFromRecord(mheardRecords[imh], mheardLine_save);

                        // ACHTUNG, gemischte Herkunft: das alte snprintf nahm
                        // Datum/Zeit/Typ/Pfad/Mesh/NCount aus mheardLine, aber
                        // hw/mod/rssi/snr/dist aus mheardLine_save -- also die
                        // Funkwerte des BESTEHENDEN Eintrags, nicht die des
                        // neuen. Das bleibt Feld fuer Feld so.
                        mheardRecordFromLine(mheardLine, mheardRecords[imh]);
                        mheardRecords[imh].mr_hw   = mheardLine_save.mh_hw;
                        mheardRecords[imh].mr_mod  = mheardLine_save.mh_mod;
                        mheardRecords[imh].mr_rssi = mheardLine_save.mh_rssi;
                        mheardRecords[imh].mr_snr  = mheardLine_save.mh_snr;
                        mheardRecords[imh].mr_dist = mheardRoundDist(mheardLine_save.mh_dist);
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
            // PATH DELETE after 12 Hours -- aged by millis(), not by the
            // (possibly clockless) wall clock (NC-02, mirrors mheardMillis[]
            // above / NC-01).
            if(!mheardPathFreshMs(iset, MHEARD_PRUNE_WINDOW_MS))
            {
                mheardPathCalls[iset][0] = 0x00;
            }
            else
            {
                if(is_equ(mheardPathCalls[iset], mheardLine.mh_sourcecallsign))
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
    int ips = mcIndexOf(mheardLine.mh_sourcepath, ',') + 1;
    int ipc = (int)strlen(mheardLine.mh_sourcepath) - ips;
    if(ipc > 51)
        ipc = 51;
    if(ipc < 0)
        ipc = 0;


    // only MHEARD HEY
    if(ips <= 0)
        return;

    memset(mheardPathCalls[ipos], 0x00, sizeof(mheardPathCalls[ipos]));
    int icallsize = (int)strlen(mheardLine.mh_sourcecallsign);
    if(icallsize > (int)sizeof(mheardPathCalls[ipos])-1)
        icallsize = (int)sizeof(mheardPathCalls[ipos])-1;
    memcpy(mheardPathCalls[ipos], mheardLine.mh_sourcecallsign, icallsize);

    //printfdeb("PATH:%i <%s> <%s> %i %i\n", ipos, mheardLine.mh_sourcepath, mheardLine.mh_sourcepath + ips, ips, ipc);

    memset(mheardPathBuffer1[ipos], 0x00, sizeof(mheardPathBuffer1[ipos]));
    memcpy(mheardPathBuffer1[ipos], mheardLine.mh_sourcepath + ips, ipc);
    mheardPathBuffer1[ipos][51] = 0x00;
    // TODO second 30 chars

    // check HEY! comming from gateway
    if(is_equ(mheardLine.mh_destinationpath, "HG"))
        mheardPathLen[ipos] = mheardLine.mh_path_len | 0x80;
    else
        mheardPathLen[ipos] = mheardLine.mh_path_len;
    
    mheardPathEpoch[ipos] = getUnixClock();
    mheardPathMillis[ipos] = (uint32_t)millis();   // NC-02: monotonic heard-time

    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
    showPathTDECK();
    #endif

    savePathPersistence();
}

// NC-02: exported freshness helpers (see mheard_functions.h) -- give callers
// outside this file (via_functions.cpp, web_functions.cpp) the same
// rollover-safe millis()-age comparison used internally throughout this
// file (NC-01), without externing mheardMillis[]/mheardPathMillis[] and
// re-implementing the comparison at each call site.
bool mheardFreshMs(int iset, uint32_t window_ms)
{
    if(iset < 0 || iset >= MAX_MHEARD)
        return false;

    return (uint32_t)(millis() - mheardMillis[iset]) < window_ms;
}

bool mheardPathFreshMs(int iset, uint32_t window_ms)
{
    if(iset < 0 || iset >= MAX_MHPATH)
        return false;

    return (uint32_t)(millis() - mheardPathMillis[iset]) < window_ms;
}

int getMheardCount()
{
    int imhcount=0;

    for(int iset=0; iset<MAX_MHEARD; iset++)
    {
        if(mheardCalls[iset][0] != 0x00)
        {
            // NC-01: aged by millis(), see mheardMillis[] above -- a node
            // with no valid wall clock still counts its recently-heard
            // neighbours (this is the /N tag in the position beacon).
            if((uint32_t)(millis() - mheardMillis[iset]) < MHEARD_AGE_WINDOW_MS)  // mheard count only last hour
            {
                imhcount++;
            }
        }
    }

    return imhcount;
}

// DR-28 (BACKLOG OPT-D16, decided 2026-09-12): the mHeard renderers must
// list entries most-recent-first. The slot-parallel storage arrays
// (mheardRecords, mheardCalls, mheardLat/Lon/Alt, mheardEpoch, mheardMillis,
// mheardNCount) are written by updateMheard() from the LORA task and are
// NEVER permuted -- this fills a caller-owned idx[] with the occupied slots
// in most-recently-heard-first order instead, so every renderer iterates
// idx[0..n) rather than the physical slot range 0..MAX_MHEARD.
//
// Age = (uint32_t)(now - mheardMillis[i]), same rollover-safe subtraction
// as the existing aging filters throughout this file (NC-01). Stable
// insertion sort ascending by age: MAX_MHEARD is <=80, so O(n^2) is cheap,
// and a stable sort keeps ties (equal age, e.g. two entries heard in the
// same millis() tick) in ascending slot order -- the scan below already
// visits slots ascending, and an element is only shifted past entries with
// a STRICTLY greater age, so equal-age entries never swap past each other.
uint8_t mheardSortedIndex(uint8_t *idx, uint32_t now)
{
    uint8_t n = 0;

    for(uint8_t i = 0; i < MAX_MHEARD; i++)
    {
        if(mheardCalls[i][0] == 0x00)
            continue;

        uint32_t age = (uint32_t)(now - mheardMillis[i]);

        uint8_t pos = n;
        while(pos > 0)
        {
            uint32_t prevAge = (uint32_t)(now - mheardMillis[idx[pos - 1]]);
            if(prevAge <= age)
                break;
            idx[pos] = idx[pos - 1];
            pos--;
        }
        idx[pos] = i;
        n++;
    }

    return n;
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

// MHeard-Liste zum Telefon, wiederaufnehmbar.
//
// Bis 4.35t wurde die komplette Liste in einem Loop-Durchlauf direkt hinter
// die Config-Frames in den Kommando-Ring (damals MAX_RING Slots) geschrieben. Ab
// MAX_RING - json_configs_cnt gehoerten Stationen (11 bzw. 12) ueberschrieb
// die Liste die Config-Frames, bevor der Drain sie senden konnte; die App sah
// CONFFIN ohne jemals das I-Frame bekommen zu haben (leere Node Settings,
// erst ein Reboot der Node half, weil er die MHeard-Liste leert).
//
// Jetzt setzt der Connect nur den Cursor; die Main-Loop ruft sendMheard()
// erst, wenn der Kommando-Ring leer ist, und jeder Aufruf legt nur so viele
// Eintraege nach, wie der Ring frei hat. Cursor -1 = nichts anstehend.
//
// Abweichung von upstream/dev: dort laeuft der Cursor ueber die physischen
// Slots 0..MAX_MHEARD, hier ueber die nach DR-28 sortierte Sicht. Diese
// Reihenfolge wird deshalb EINMAL beim Connect eingefroren. mheardSortedIndex()
// liefert bei jedem Aufruf eine frische Sortierung, und updateMheard() schreibt
// zwischen zwei Loop-Durchlaeufen aus dem LORA-Task weiter; ein Cursor gegen
// eine jedes Mal neu sortierte Liste wuerde Stationen doppelt senden oder
// ueberspringen, sobald sich waehrend der Uebertragung die Reihenfolge
// verschiebt. Der Schnappschuss kostet MAX_MHEARD Byte BSS und nimmt sie
// zugleich dem Loop-Stack ab -- auf nRF52 ist das die Richtung, in die dieses
// Modul ohnehin schon ausweicht (siehe N-22 in sendMheard()).
//
// Eingefroren wird ausschliesslich die REIHENFOLGE. Der Sortierzeitpunkt wird
// bewusst NICHT mit eingefroren: DR-28 verlangt denselben Augenblick nur fuer
// Renderer, die eine Altersspalte ausgeben, und sendMheard() gibt DATE/TIME
// aus dem Datensatz aus, kein gerechnetes Alter. Wuerde hier gegen den
// eingefrorenen Augenblick gealtert, verschwaende eine Station, die WAEHREND
// der Uebertragung neu gehoert wird: mheardMillis[] waere dann groesser als
// der Augenblick, die vorzeichenlose Differenz liefe unter und der Eintrag
// fiele aus dem 12-h-Fenster heraus. Deshalb millis() in der Schleife.
static uint8_t mheard_send_idx[MAX_MHEARD];
static uint8_t mheard_send_n = 0;
static int mheard_send_cursor = -1;

void startMheardToPhone()
{
    mheard_send_n = mheardSortedIndex(mheard_send_idx, (uint32_t)millis());
    mheard_send_cursor = 0;
}

bool mheardToPhonePending()
{
    return mheard_send_cursor >= 0;
}

static bool comRingWouldEvictUnread()
{
    // Byte-Ring statt Schlitzfeld: der Kommando-Ring ist nie "voll" --
    // bf_push() verdraengt notfalls die aeltesten Frames. Zu vermeiden ist
    // nur, dass dabei ein UNGELESENER weggeworfen wird, denn das sind genau
    // die, die dieser sendMheard()-Aufruf gerade selbst geschrieben hat: die
    // Liste verloere still Eintraege.
    //
    // bf_unread() zaehlt FRAMES, nicht Bytes -- byte_fifo.cpp erhoeht es je
    // push und senkt es je pop. Es direkt gegen cap zu stellen waere still
    // wirkungslos, die Drossel griffe nie.
    //
    // Die Bytes der ungelesenen Frames fuehrt der Ring nicht einzeln, nach
    // oben sind sie aber durch unread * 256 beschraenkt (bf_push2() laesst
    // hoechstens 255 Byte Nutzlast plus Laengenbyte zu). Dazu der groesste
    // Frame, der hier hineinkommt: addBLEComToOutBuffer() klemmt auf 245,
    // plus Laengenbyte.
    //
    // Bewusst NICHT bf_used(): das enthaelt den gelesenen Verlauf, faellt nie
    // (bf_pop() ruehrt es nicht an, nur die Verdraengung tut es) und stuende
    // nach dem ersten vollen Ringumlauf dauerhaft dicht an cap -- die Drossel
    // bliebe dann fuer immer haengen und die MHeard-Liste erreichte das
    // Telefon nie wieder.
    const uint32_t worst_unread_bytes = (uint32_t)bf_unread(&phoneComRing) * 256u;
    return worst_unread_bytes + 1u + 245u > (uint32_t)phoneComRing.cap;
}

void sendMheard()
{
    // N-22 (BACKLOG SS3.8m, Fix 9ce62aa0): der Loop-Task auf nRF52 hat 4 KB
    // Stack -- LOOP_STACK_SZ = 256*4 Woerter, hart im Adafruit-Core, nicht per
    // Build-Flag zu erhoehen. Seit R2-04 ist struct mheardLine 584 Byte statt
    // ~112 (sieben String-Handles a 12 B wurden feste char[]), und sendMheard() liegt
    // damit 1976 B tief auf einem Stack, auf dem N-22 schon einmal
    // uxTaskGetStackHighWaterMark(NULL) == 0 gemessen hat.
    // Gemessen mit -fstack-usage auf wiscore_rak4631: nrf52loop 792 + sendMheard 1184.
    // Auf dem Loop-Task und nur dort aufgerufen (nrf52_main.cpp:1839, im Loop), nicht reentrant --
    // also nach BSS statt auf den Stack. ESP32 behaelt den Stack-Puffer:
    // 8 KB Loop-Task, und dort ist der Frame kein Thema.
    // Bewusst DREI getrennte Statics statt eines gemeinsamen: ein gemeinsamer
    // muesste ueber zwei Uebersetzungseinheiten hinweg extern sein und
    // koppelte mheard_functions.cpp und web_functions.cpp an die Annahme,
    // dass keine der drei Funktionen je auf einen anderen Task wandert.
#if defined(NRF52_SERIES)
    static struct mheardLine mheardLine;
#else
    struct mheardLine mheardLine;
#endif

    // DR-28: most-recent-first, via mheardSortedIndex() -- die Speicher-
    // arrays selbst bleiben in physischer Slot-Reihenfolge, siehe den
    // Kommentar dieser Funktion und mheard_functions.h. Die Sortierung
    // stammt aus startMheardToPhone(); hier wird sie nur abgelaufen.
    if(mheard_send_cursor < 0)
        return;

    for(; mheard_send_cursor < mheard_send_n; mheard_send_cursor++)
    {
        uint8_t iset = mheard_send_idx[mheard_send_cursor];

        // Der Schnappschuss ist aelter als der Augenblick, in dem er
        // abgelaufen wird: updateMheard() (LORA-Task) kann einen Slot
        // zwischendurch geraeumt haben -- es setzt nur mheardCalls[x][0]
        // auf 0x00 und laesst mheardMillis[] stehen. Ohne diese Pruefung
        // ginge ein MH-Frame mit leerem CALL zum Telefon.
        if(mheardCalls[iset][0] == 0x00)
            continue;

        if((uint32_t)(millis() - mheardMillis[iset]) < MHEARD_PRUNE_WINDOW_MS)  // mheard last 12 hours (NC-01: millis(), not wall clock)
        {
            // Kein Platz mehr, ohne etwas Ungelesenes zu verdraengen: ohne
            // den Cursor weiterzuschalten zurueck, der naechste Aufruf nimmt
            // genau diesen Eintrag noch einmal.
            if(comRingWouldEvictUnread())
                return;

            // R2-01: hier stand die DRITTE Kopie derselben Zerlegung --
            // elf getValue()-Aufrufe, jeder mit einer eigenen String-
            // Anforderung, und jeder scannte die Zeichenkette erneut. Der
            // Datensatz braucht keine davon.
            //
            // Reihenfolge: mheardLineFromRecord() ruft initMheardLine()
            // auf und wuerde ein vorher gesetztes Rufzeichen wieder
            // loeschen. Deshalb erst der Datensatz, dann das Rufzeichen.
            mheardLineFromRecord(mheardRecords[iset], mheardLine);
            mcSet(mheardLine.mh_callsign, sizeof(mheardLine.mh_callsign), mheardCalls[iset]);

            // generate JSON
            JsonDocument mhdoc;

            mhdoc["TYP"] = "MH";
            mhdoc["CALL"] = mheardLine.mh_callsign;
            mhdoc["DATE"] = mheardLine.mh_date;
            mhdoc["TIME"] = mheardLine.mh_time;
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
            // Schranke ist der Puffer, nicht die JSON-Laenge (UP-01, BND-03)
            uint16_t frame_len = bleJsonFrame(mhdoc, bleBuffer, sizeof(bleBuffer));

            addBLEComToOutBuffer(bleBuffer, frame_len);
        }
    }

    mheard_send_cursor = -1;
}

void showMHeard()
{
    printlndeb("\n/-----------------------------------------------------------------------------------------------------\\");
    printlndeb("|MHeard call |    date    |   time   | typ | source hardware | mod | rssi |  snr | dist | pl | m | nc |");

    // N-22 (BACKLOG SS3.8m, Fix 9ce62aa0): der Loop-Task auf nRF52 hat 4 KB
    // Stack -- LOOP_STACK_SZ = 256*4 Woerter, hart im Adafruit-Core, nicht per
    // Build-Flag zu erhoehen. Seit R2-04 ist struct mheardLine 584 Byte statt
    // ~112 (sieben String-Handles a 12 B wurden feste char[]), und dieser Pfad
    // liegt damit 2568 B tief auf einem Stack, auf dem N-22 schon einmal
    // uxTaskGetStackHighWaterMark(NULL) == 0 gemessen hat.
    // Gemessen mit -fstack-usage auf wiscore_rak4631: nrf52loop 792 + commandAction 1120 + showMHeard 656.
    // Nur auf dem Loop-Task aufgerufen (command_functions.cpp:4798 ueber commandAction; BLE-Eingaben laufen ueber bleQueue in den Loop, siehe nrf52_ble.cpp:44), nicht reentrant --
    // also nach BSS statt auf den Stack. ESP32 behaelt den Stack-Puffer:
    // 8 KB Loop-Task, dort ist der Frame kein Thema.
    // Bewusst DREI getrennte Statics statt eines gemeinsamen: ein gemeinsamer
    // muesste ueber zwei Uebersetzungseinheiten hinweg extern sein und koppelte
    // mheard_functions.cpp an web_functions.cpp ueber die Annahme, dass keine
    // der drei Funktionen je auf einen anderen Task wandert.
#if defined(NRF52_SERIES)
    static mheardLine mheardLine;
#else
    mheardLine mheardLine;
#endif

    // DR-28: most-recent-first, via mheardSortedIndex() -- the storage
    // arrays themselves stay in physical slot order, see that function's
    // comment and mheard_functions.h.
    uint8_t idx[MAX_MHEARD];
    uint32_t now = (uint32_t)millis();
    uint8_t n = mheardSortedIndex(idx, now);

    for(uint8_t k=0; k<n; k++)
    {
        uint8_t iset = idx[k];

        if((uint32_t)(now - mheardMillis[iset]) < MHEARD_PRUNE_WINDOW_MS)  // mheard last 12 hours (NC-01: millis(), not wall clock)
        {
            printlndeb("|------------|------------|----------|-----|-----------------|-----|------|------|------|----|---|----|");

            printfdeb("| %-10.10s | ", mheardCalls[iset]);

            mheardLineFromRecord(mheardRecords[iset], mheardLine);

            printfdeb("%-10.10s | ", mheardLine.mh_date);
            printfdeb("%-8.8s | ", mheardLine.mh_time);

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

    printlndeb("\\-----------------------------------------------------------------------------------------------------/\n");
}

void showPath()
{
    printlndeb("\n/-----------------------------------------------------------------------------------------\\");
    printlndeb("|       date          | lng/Gate/Path                                                     |");

    for(int iset=0; iset<MAX_MHPATH; iset++)
    {
        if(mheardPathCalls[iset][0] != 0x00)
        {
            if(mheardPathFreshMs(iset, MHEARD_PRUNE_WINDOW_MS))  // path last 12 hours (NC-02: millis(), not wall clock)
            {
                printlndeb("|---------------------|-------------------------------------------------------------------|");

                //printfdeb("| %-10.10s | ", mheardPathCalls[iset]);

                unsigned long lt = mheardPathEpoch[iset] + (long)(meshcom_settings.node_utcoff * 3600.0);
                
                printfdeb("| %-19.19s | ", convertUNIXtoString(lt).c_str()); // yyyy.mm.dd hh:mm:ss

                printfdeb("%01u%s/%-10.10s %-51.51s |\n", (mheardPathLen[iset] & 0x7F), ((mheardPathLen[iset] & 0x80)?"G":" "), mheardPathCalls[iset], mheardPathBuffer1[iset]);
            }
            else
            {
                mheardPathCalls[iset][0] = 0x00;
            }
        }
    }

    printlndeb("\\-----------------------------------------------------------------------------------------/\n");
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

    // DR-28: most-recent-first, via mheardSortedIndex() -- the storage
    // arrays themselves stay in physical slot order, see that function's
    // comment and mheard_functions.h.
    uint8_t idx[MAX_MHEARD];
    uint32_t now = (uint32_t)millis();
    uint8_t n = mheardSortedIndex(idx, now);

    // n (from mheardSortedIndex() above) is already the occupied-slot count
    // that the old "for(iset=0..MAX_MHEARD) if(occupied) anzrow++" loop
    // computed by hand -- no separate counting pass needed.
    int anzrow = 1 + n;

    lv_table_set_row_cnt(mheard_ta, anzrow);

    for(uint8_t k=0; k<n; k++)
    {
        uint8_t iset = idx[k];

        snprintf(buf, 10, "%s", mheardCalls[iset]);
        lv_table_set_cell_value(mheard_ta, row, 0, buf);

        mheardLineFromRecord(mheardRecords[iset], mheardLine);

        snprintf(buf, 6, "%.5s", mheardLine.mh_time);
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

            snprintf(buf, 52, "%01u%s/%s", (mheardPathLen[iset] & 0x7F), ((mheardPathLen[iset] & 0x80)?"G":" "), mheardPathBuffer1[iset]);
            lv_table_set_cell_value(path_ta, row, 2, buf);

            row++;
        }
    }
}
#endif

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
// NC-01: is the wall clock (meshcom_settings.node_date_*) trustworthy right
// now? Mirrors updateMheard()'s own "strYear.toInt() < 2025" guard -- a
// node with no NTP/GPS boots with node_date_year unset (0, see
// nrf52_main.cpp/esp32_main.cpp), never a real year >= 2025. Only needed at
// SD-persistence load time (below), to decide how to seed mheardMillis[]
// for entries restored from a previous boot; the millis()-based aging in
// updateMheard()/getMheardCount()/sendMheard()/showMHeard() never calls
// this. File-local: not a replacement for getUnixClock() callers elsewhere.
static bool isWallClockValid()
{
    return meshcom_settings.node_date_year >= 2025;
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
        size_t expected_mh = sizeof(mheardCalls) + sizeof(mheardRecords) + sizeof(mheardLat)
                           + sizeof(mheardLon) + sizeof(mheardEpoch) + sizeof(mheardNCount);
        if(file.size() != expected_mh) {
            printfdeb("[TDECK]...mheard.dat size mismatch (%u != %u), deleting\n", file.size(), expected_mh);
            file.close();
            SD.remove("/mheard.dat");
            return;
        }

        file.read((uint8_t*)mheardCalls, sizeof(mheardCalls));
        file.read((uint8_t*)mheardRecords, sizeof(mheardRecords));
        file.read((uint8_t*)mheardLat, sizeof(mheardLat));
        file.read((uint8_t*)mheardLon, sizeof(mheardLon));
        file.read((uint8_t*)mheardEpoch, sizeof(mheardEpoch));
        file.read((uint8_t*)mheardNCount, sizeof(mheardNCount));
        file.close();

        // NC-01: mheardMillis[] was not in the file (see saveMHeardPersistence()) --
        // millis() restarts at 0 every boot, so re-derive an initial age instead
        // of leaving it at its zero-init value (which would read every restored
        // entry as heard 0 ms ago AND as heard `now`, wrongly extending its
        // 12h/1h lifetime by a full boot's worth of elapsed wall-clock time).
        // If the wall clock is valid already (RTC battery, or a fast NTP/GPS
        // fix before this runs), convert the saved epoch age into an
        // equivalent millis() age. Otherwise -- no valid wall clock yet,
        // exactly the NC-01 scenario -- treat every restored entry as just
        // heard: an unverifiable age is safer treated as fresh (still shown,
        // still counted) than silently dropped.
        uint32_t loadMillis = (uint32_t)millis();
        bool bWallClockValid = isWallClockValid();
        unsigned long nowEpoch = bWallClockValid ? getUnixClock() : 0;
        for(int i=0; i<MAX_MHEARD; i++)
        {
            if(mheardCalls[i][0] == 0x00)
            {
                mheardMillis[i] = 0;
                continue;
            }

            if(bWallClockValid && nowEpoch > mheardEpoch[i])
            {
                unsigned long ageMs = (nowEpoch - mheardEpoch[i]) * 1000UL;
                mheardMillis[i] = (ageMs < loadMillis) ? (loadMillis - (uint32_t)ageMs) : 0;
            }
            else
            {
                mheardMillis[i] = loadMillis;   // treat as just heard
            }
        }

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

        // NC-02: mheardPathMillis[] was not in the file (see
        // savePathPersistence()) -- re-derive it exactly the way
        // loadMHeardPersistence() re-derives mheardMillis[] above.
        uint32_t loadMillisPath = (uint32_t)millis();
        bool bWallClockValidPath = isWallClockValid();
        unsigned long nowEpochPath = bWallClockValidPath ? getUnixClock() : 0;
        for(int i=0; i<MAX_MHPATH; i++)
        {
            if(mheardPathCalls[i][0] == 0x00)
            {
                mheardPathMillis[i] = 0;
                continue;
            }

            if(bWallClockValidPath && nowEpochPath > mheardPathEpoch[i])
            {
                unsigned long ageMs = (nowEpochPath - mheardPathEpoch[i]) * 1000UL;
                mheardPathMillis[i] = (ageMs < loadMillisPath) ? (loadMillisPath - (uint32_t)ageMs) : 0;
            }
            else
            {
                mheardPathMillis[i] = loadMillisPath;   // treat as just heard
            }
        }

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