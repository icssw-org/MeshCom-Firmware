// MeshCom-5-Topologie, MH-Rahmen an die Telefon-App -- Umsetzung von
// src/mh_phone.h (docs/meshcom5-topologie/ 4.9, docs/meshcom5-campaign.md
// Welle 4). Ersetzt den MH-JSON-Teil der frueheren updateMheard() (live) und
// sendMheard()/startMheardToPhone()/mheardToPhonePending() (Liste beim
// Verbinden); jene Datei ist inzwischen entfernt (W4a).
//
// mhJsonBuild() ist Arduino-frei bis auf ArduinoJson selbst (Host-Test env
// native_mh_phone) und haengt an nichts, was diese Welle woanders loescht.
// Die Rundung auf 0,1 (mhRoundDist()) ist absichtlich eine EIGENE, kleine
// Kopie des alten mheardRoundDist()-Algorithmus (snprintf("%.1lf") + atof --
// eine binaere Skalierung kann die alte dezimale Rundung nicht nachbilden,
// siehe die Herleitung, die frueher in mheard_record.h stand), statt eines
// Includes auf eine Datei, die dieselbe Welle an anderer Stelle abraeumt.
//
// mhPhoneLive()/mhPhoneList*() brauchen Hardware (BLE-Ringe, meshcom_settings,
// die Wanduhr) und sind deshalb unter #ifndef NATIVE_BUILD; der Host-Test
// prueft nur mhJsonBuild().

#include "mh_phone.h"
#include "uptime_min.h"   // wrap-safe 16-bit uptime minutes (NBR stamps)
#include "ble_json_frame.h"
#include "configuration_global.h"   // BLE_JSON_PAYLOAD_MAX (unbedingt, board-unabhaengig)

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef NATIVE_BUILD
#include <new>                       // std::nothrow (mhPhoneListStart())
#include "Arduino.h"
#include "loop_functions.h"          // getUnixClock(), addBLEOutBuffer(), addBLEComToOutBuffer()
#include "loop_functions_extern.h"   // isPhoneReady, phoneComRing
#include "meshcom_settings.h"        // meshcom_settings.node_lat/node_lon
#endif

// Rundet wie das alte mheardRoundDist() (mheard_record.h, jetzt entfernt):
// ueber printf statt binaerer Skalierung, weil nur das die dezimale Rundung
// von frueher exakt nachbildet (0,05 rundet ueber printf zu "0.1", nicht zu
// "0.0" -- siehe die ausfuehrliche Herleitung, die dort stand). -1 heisst
// "nicht bekannt/nicht berechnet".
static float mhRoundDist(double dist_km)
{
    if (dist_km < 0.0)
        return -1.0f;
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%.1lf", dist_km);
    return (float)atof(tmp);
}

#ifndef NATIVE_BUILD
// Knoten-Epoche (Ortszeit als Epoche): getUnixClock() liefert rohe UTC
// (node_date_* minus node_utcoff), die App bekam DATE/TIME frueher in
// Ortszeit (getDateString()/getTimeString() aus node_date_*). mhJsonBuild()
// formatiert mit gmtime_r(), also den Offset hier wieder dazu.
//
// Advisor R1 (Welle 4, Gate): ohne gestellte Uhr ist node_date_year 0 (Default),
// aber getUnixClock() rechnet trotzdem ueber mktime() -- auf einem 32-Bit
// time_t (nRF52) kommt dabei ein Ueberlauf-Wert heraus, auf ESP32 (64-Bit
// time_t, auf 32 Bit gecastet) ein scheinbar gueltiges, weit in der Zukunft
// liegendes Jahr (>= 2025, besteht also den Uhr-Test in mhJsonBuild()
// faelschlich). Deshalb hier selbst geprueft, VOR jeder Rechnung: ohne
// gestellte Uhr 0 zurueck, mhJsonBuild(0) sendet dann nichts (der
// Listen-Cursor laeuft trotzdem weiter, wie es sendMheard() frueher tat).
static uint32_t mhNodeEpoch(void)
{
    if (meshcom_settings.node_date_year < 2025)
        return 0;
    return (uint32_t)getUnixClock() + (uint32_t)(int32_t)(meshcom_settings.node_utcoff * 3600.0);
}
#endif

uint16_t mhJsonBuild(const NbrMhView &v, uint32_t now_epoch, double own_lat, double own_lon,
                     uint8_t *buf, size_t len)
{
    // Puffer zuerst pruefen -- buf[0] wird gleich beschrieben, bevor
    // bleJsonFrame() seine eigene (dann zu spaete) Schranke sehen wuerde.
    if (buf == nullptr || len < 2)
        return 0;

    // Uhr-Test ZUERST auf now_epoch selbst, nicht auf das zurueckgerechnete
    // Ereignis: eine ungestellte Uhr (now_epoch == 0 oder sonst zu klein)
    // wuerde now_epoch - age_min*60 sonst vorzeichenlos unterlaufen und ein
    // beliebiges, womoeglich >= 2025 aussehendes Datum ergeben (Advisor R1 --
    // mhNodeEpoch() liefert seit dem Gate 0 fuer diesen Fall, aber
    // mhJsonBuild() darf sich darauf nicht allein verlassen: jeder Aufrufer
    // mit v.age_min > 0 waere sonst genauso betroffen). Ein Ereignis kurz vor
    // einem Jahreswechsel bei gueltiger Uhr bleibt davon unberuehrt, anders
    // als eine Pruefung auf das zurueckgerechnete Datum es haette.
    time_t now_t = (time_t)now_epoch;
    struct tm now_tm;
    gmtime_r(&now_t, &now_tm);
    if (now_tm.tm_year + 1900 < 2025)
        return 0;   // keine gueltige Uhr, wie heute (mheard_functions.cpp)

    // DATE/TIME: now_epoch - age_min*60 liefert Jahr/Monat/Tag/Stunde/Minute,
    // die Sekunde kommt aus v.sec (0xFF: 0), siehe src/mh_phone.h. now_epoch
    // ist ab hier als gueltig bestaetigt und v.age_min durch NBR_WINDOW_MIN
    // (12 h, nbrMhGet()) beschraenkt, die Subtraktion unterlaeuft also nicht.
    uint32_t event_epoch = now_epoch - (uint32_t)v.age_min * 60u;
    time_t   t = (time_t)event_epoch;
    struct tm tmv;
    gmtime_r(&t, &tmv);
    int year = tmv.tm_year + 1900;

    uint8_t use_sec = (v.sec != 0xFF) ? v.sec : 0;

    // Ziffern von Hand statt snprintf("%d"): struct tm's Felder sind `int`
    // ohne compile-zeitige Schranke, GCC kann bei "%02d" also nicht
    // ausschliessen, dass mehr als 2 Stellen kommen (-Werror=format-truncation
    // auf den ESP32/nRF52-Envs). year ist durch den Uhr-Test oben schon >=
    // 2025, hier nur defensiv auf 4 Stellen geklemmt (bei einem gueltigen
    // uint32_t-Epochenwert nie noetig, siehe getUnixClock()).
    char date_s[11];   // "YYYY-MM-DD" + NUL
    char time_s[9];    // "HH:MM:SS" + NUL
    {
        int y = year;
        if (y > 9999) y = 9999;
        int mo = tmv.tm_mon + 1;
        if (mo < 0) mo = 0;
        if (mo > 99) mo = 99;
        int dd = tmv.tm_mday;
        if (dd < 0) dd = 0;
        if (dd > 99) dd = 99;
        date_s[0] = (char)('0' + (y / 1000) % 10);
        date_s[1] = (char)('0' + (y / 100) % 10);
        date_s[2] = (char)('0' + (y / 10) % 10);
        date_s[3] = (char)('0' + y % 10);
        date_s[4] = '-';
        date_s[5] = (char)('0' + (mo / 10) % 10);
        date_s[6] = (char)('0' + mo % 10);
        date_s[7] = '-';
        date_s[8] = (char)('0' + (dd / 10) % 10);
        date_s[9] = (char)('0' + dd % 10);
        date_s[10] = 0;

        int hh = tmv.tm_hour;
        if (hh < 0) hh = 0;
        if (hh > 99) hh = 99;
        int mi = tmv.tm_min;
        if (mi < 0) mi = 0;
        if (mi > 99) mi = 99;
        int ss = use_sec;
        time_s[0] = (char)('0' + (hh / 10) % 10);
        time_s[1] = (char)('0' + hh % 10);
        time_s[2] = ':';
        time_s[3] = (char)('0' + (mi / 10) % 10);
        time_s[4] = (char)('0' + mi % 10);
        time_s[5] = ':';
        time_s[6] = (char)('0' + (ss / 10) % 10);
        time_s[7] = (char)('0' + ss % 10);
        time_s[8] = 0;
    }

    // Unbekannte Werte gehen NICHT als Rohplatzhalter (-32768 dBm, -128 dB, 0)
    // zur App, sondern fehlen im Rahmen (die App zeigt dann nichts statt einer
    // erfundenen Zahl). Ohne Detail-Slot (Direktstation verdraengt, siehe
    // nbr_views.h) liefert nbrMhGet() rssi == NBR_MH_RSSI_UNKNOWN (INT16_MIN)
    // und plt/mod/pl/mesh 0. Ein Slot kann diesen RSSI nie ergeben (8 Bit
    // minus 160, also >= -160), plt/mod/pl/mesh == 0 sind dagegen echte Werte
    // und taugen nicht als Marker. NbrMhView hat kein eigenes has_detail-Feld;
    // rssi ist der Marker, den auch die anderen Leser (Web, T-Deck, --mheard)
    // benutzen. SNR stammt aus der Kante, nicht aus dem Slot: er fehlt nur bei
    // NBR_SNR_UNKNOWN. DIST bleibt immer drin und numerisch (die App ruft
    // mheard.DIST.toFixed(); ohne DIST ginge der ganze Eintrag verloren).
    const bool has_detail = (v.rssi != NBR_MH_RSSI_UNKNOWN);

    JsonDocument doc;

    // Bis zu 13 alte Felder, alte Reihenfolge (CONTRACT, src/mh_phone.h).
    doc["TYP"]  = "MH";
    doc["CALL"] = v.call;
    doc["DATE"] = date_s;
    doc["TIME"] = time_s;
    if (has_detail)
        doc["PLT"] = (uint8_t)v.plt;
    doc["HW"]   = v.hw;
    if (has_detail)
    {
        doc["MOD"]  = v.mod;
        doc["RSSI"] = v.rssi;
    }
    if (v.snr != NBR_SNR_UNKNOWN)
        doc["SNR"] = v.snr;

    // DIST auf 0,1 km gerundet, -1 wenn die eigene oder die fremde Position
    // unbekannt ist (own 0/0, fremd NAN).
    double dist_out = -1.0;
    bool own_known = !(own_lat == 0.0 && own_lon == 0.0);
    bool nb_known  = nbrPosKnown(v.lat, v.lon);
    if (own_known && nb_known)
        dist_out = (double)mhRoundDist((double)nbrDistKm((float)own_lat, (float)own_lon, v.lat, v.lon));
    doc["DIST"] = dist_out;

    if (has_detail)
    {
        doc["PL"]   = v.pl;
        doc["MESH"] = v.mesh;
    }
    doc["NCNT"] = v.ncnt;

    // 7 neue Felder (Konzept 4.9), ans Ende angehaengt -- bleJsonFrameFailSoft()
    // streicht bei Ueberlaenge die zuletzt eingefuegten Schluessel zuerst, also
    // genau diese, nie die 13 alten oben.
    doc["AGE"] = v.age_min;
    if (v.hm_snr != NBR_SNR_UNKNOWN)
        doc["HM"] = v.hm_snr;   // fehlt, wenn unbekannt (Konzept 4.9)
    if (v.role != 0)
    {
        char rolebuf[2] = { v.role, 0 };
        doc["ROLE"] = rolebuf;  // fehlt, wenn NA (kein #X fuer diese Zeile)
    }
    doc["EX"]  = v.ex;
    doc["NB"]  = v.nb;
    doc["GW"]  = v.gw;
    doc["VIA"] = v.via;

    buf[0] = 0x44;
    return bleJsonFrameFailSoft(doc, buf, len, BLE_JSON_PAYLOAD_MAX);
}

#ifndef NATIVE_BUILD

// Live-Rahmen: hoechstens einmal je Nachbar und Minute, ausgeloest vom
// Aufrufer (OnRxDone, W4a) genau dann, wenn sich die Minute der Kante
// (row, 0) durch diesen Empfang geaendert hat. Derselbe BLE-Ausgang wie die
// alte updateMheard() (addBLEOutBuffer(), nicht der Kommando-Ring).
void mhPhoneLive(int row, uint16_t now_min)
{
    if (isPhoneReady != 1)
        return;

    NbrMhView v;
    if (!nbrMhGet(nbrMatrix, row, now_min, &v))
        return;

    uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
    uint16_t frame_len = mhJsonBuild(v, mhNodeEpoch(),
                                      meshcom_settings.node_lat, meshcom_settings.node_lon,
                                      bleBuffer, sizeof(bleBuffer));
    if (frame_len > 0)
        addBLEOutBuffer(bleBuffer, frame_len);
}

// --- Liste beim Verbinden ---------------------------------------------------
//
// Schnappschuss der Zeilenindizes (neueste zuerst, 12 h) auf dem Heap statt
// in einem zeilengrossen static-Feld aus struct-Werten: die Liste ist nur ein
// uint8_t je Zeile (hoechstens NBR_MAX_ROWS-1 Byte), gebaut EINMAL bei
// mhPhoneListStart() und beim Ablaufen (mhPhoneListStep()) freigegeben. Nur
// EIN NbrMhView existiert je Aufruf, auf dem Stack der Sendeschleife.
//
// Der eingefrorene Schnappschuss ist bewusst so gewaehlt wie beim alten
// sendMheard() (siehe dessen Kommentar, jetzt entfernt): ein Cursor gegen
// eine bei jedem Schritt neu sortierte Liste wuerde Stationen doppelt senden
// oder ueberspringen, sobald sich waehrend der Uebertragung die Reihenfolge
// verschiebt.
static uint8_t *s_mh_idx = nullptr;
static int      s_mh_n = 0;
static int      s_mh_cursor = -1;

// Wie die alte comRingWouldEvictUnread() (mheard_functions.cpp, jetzt
// entfernt): vermeidet nur, dass ein UNGELESENER Frame verdraengt wird --
// das waeren genau die, die dieser Lauf selbst gerade geschrieben hat.
static bool mhComRingWouldEvictUnread(void)
{
    const uint32_t worst_unread_bytes = (uint32_t)bf_unread(&phoneComRing) * 256u;
    return worst_unread_bytes + 1u + 245u > (uint32_t)phoneComRing.cap;
}

void mhPhoneListStart(void)
{
    if (s_mh_idx != nullptr)
    {
        delete[] s_mh_idx;
        s_mh_idx = nullptr;
    }
    s_mh_n = 0;
    s_mh_cursor = -1;

    uint16_t now_min = uptimeMin16();

    int total = nbrMhRows(nbrMatrix, now_min, NBR_WINDOW_MIN, nullptr, 0);
    if (total <= 0)
        return;

    // L4 (Advisor, Welle 4 Gate): nothrow statt eines ungefangenen new[] --
    // -fno-exceptions liesse ein werfendes new[] auf dieser Plattform nur
    // abbrechen (std::terminate/abort), statt fail-soft nichts zu senden.
    s_mh_idx = new (std::nothrow) uint8_t[total];
    if (s_mh_idx == nullptr)
        return;   // s_mh_cursor bleibt -1 (siehe oben) -- kein Frame, kein Absturz

    s_mh_n = nbrMhRows(nbrMatrix, now_min, NBR_WINDOW_MIN, s_mh_idx, total);
    if (s_mh_n > total)
        s_mh_n = total;   // Sicherung gegen ein Wachstum zwischen den zwei Aufrufen

    s_mh_cursor = (s_mh_n > 0) ? 0 : -1;
    if (s_mh_cursor < 0)
    {
        delete[] s_mh_idx;
        s_mh_idx = nullptr;
    }
}

bool mhPhoneListPending(void)
{
    return s_mh_cursor >= 0;
}

void mhPhoneListStep(void)
{
    if (s_mh_cursor < 0)
        return;

    uint16_t now_min = uptimeMin16();

    for (; s_mh_cursor < s_mh_n; s_mh_cursor++)
    {
        if (mhComRingWouldEvictUnread())
            return;   // Cursor bleibt stehen, naechster Aufruf holt denselben Eintrag nach

        NbrMhView v;
        if (!nbrMhGet(nbrMatrix, s_mh_idx[s_mh_cursor], now_min, &v))
            continue;   // Kante (x, 0) ist zwischen Schnappschuss und Versand verfallen

        uint8_t bleBuffer[MAX_MSG_LEN_PHONE] = {0};
        uint16_t frame_len = mhJsonBuild(v, mhNodeEpoch(),
                                          meshcom_settings.node_lat, meshcom_settings.node_lon,
                                          bleBuffer, sizeof(bleBuffer));
        if (frame_len > 0)
            addBLEComToOutBuffer(bleBuffer, frame_len);
    }

    delete[] s_mh_idx;
    s_mh_idx = nullptr;
    s_mh_n = 0;
    s_mh_cursor = -1;
}

#endif // NATIVE_BUILD
