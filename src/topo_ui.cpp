// MeshCom-5-Topologie, Geraeteanzeigen und Sicherung -- Implementierung
// (docs/meshcom5-topologie/ 4.12, docs/meshcom5-campaign.md Welle 4, Brief
// W4d). Siehe src/topo_ui.h fuer den eingefrorenen Vertrag.
//
// Die eigentliche Tabellen-Anzeige lebt weiter je Board (T-Deck:
// tdeck_refresh_mh_view()/tdeck_refresh_path_view() in
// src/t-deck/lv_obj_functions.cpp; T-Deck Pro: TDeck_pro_mheard_disp() ->
// ui_mheard_disp() in src/t-deck-pro/), genau wie die alten
// showMHeardTDECK()/showPathTDECK()/ui_mheard_disp() aus dem entfallenen
// mheard_functions.cpp. Diese Datei ist nur der zentrale Einstiegspunkt, den
// OnRxDone (topoUiChanged) und der Start (topoUiBoot) je einmal aufrufen --
// Takt und /topo.dat leben hier, weil beide Boards dasselbe Format und
// dieselbe 10-Minuten-Regel teilen (Konzept 4.12).

#include "topo_ui.h"
#include "uptime_min.h"   // wrap-safe 16-bit uptime minutes (NBR stamps)

#include <Arduino.h>
#include <configuration.h>
#include <debugconf.h>
#include <stdlib.h>

#include "meshcom_settings.h"
#include "nbr_views.h"
#include "loop_functions.h"        // getUnixClock()
#include "clock.h"                 // MyClock, Clock::SetClock()
#include "printfdeb_functions.h"

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <SD.h>
#include "t-deck/lv_obj_functions.h"   // tdeck_refresh_mh_view()/tdeck_refresh_path_view()
#elif defined(BOARD_T_DECK_PRO)
#include <SD.h>
#include "t-deck-pro/tdeck_pro.h"      // TDeck_pro_mheard_disp()
#endif

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS) || defined(BOARD_T_DECK_PRO)

// Wie oft ueberhaupt neu gebaut wird, waehrend eine Seite sichtbar bleibt --
// die Sichtbarkeitspruefung selbst sitzt beim jeweiligen Board (Tab-Index
// bzw. bMhShown), das hier ist nur der Deckel gegen "eine Kante trifft, also
// zehn lv_table_set_cell_value()-Aufrufe pro empfangenem Rahmen". 1 s ist
// schnell genug, um sich lebendig anzufuehlen, und billig genug, um bei
// einem Empfangsschwall (mehrere Rahmen dieselbe Sekunde) nicht jedes Mal
// neu zu zeichnen.
#define TOPO_UI_MIN_INTERVAL_MS 1000u

// /topo.dat wird hoechstens alle 10 Minuten neu geschrieben (Konzept 4.12).
#define TOPO_SAVE_INTERVAL_MIN 10u

// Epoche liegt zwischen dem 1.1.2025 und dem 1.1.2100 UTC (Advisor R2a):
// grobe Plausibilitaet gegen ein kaputtes oder fremdes Abbild, das sonst die
// Uhr auf einen Nonsens-Wert vorstellen wuerde (z. B. ~2106 aus einer ohne
// gueltige Uhr gesicherten, ueberlaufenen Epoche).
static bool vEpochPlausible(uint32_t epoch)
{
    time_t t = (time_t)epoch;
    struct tm tmv;
    gmtime_r(&t, &tmv);
    int year = tmv.tm_year + 1900;
    return year >= 2025 && year <= 2100;
}

static void vSaveTopoIfDue(uint16_t now_min)
{
    if (!meshcom_settings.node_persist_to_sd)
        return;

    static uint16_t vLastSaveMin = 0;
    if ((uint16_t)(now_min - vLastSaveMin) < TOPO_SAVE_INTERVAL_MIN)
        return;
    // R2b (Advisor): sofort stempeln, nicht erst nach einem erfolgreichen
    // Schreiben -- sonst mallociert/oeffnet diese Funktion bei fehlender oder
    // kaputter SD-Karte ab Minute 10 bei jedem einzelnen Aufruf auf Dauer neu
    // (wie saveMHeardPersistence() es frueher schon richtig machte).
    vLastSaveMin = now_min;

    // R2a (Advisor): ohne gueltige Uhr waere die gesicherte Epoche Muell
    // (getUnixClock() auf ungesetzte node_date_* Felder) -- lieber gar nicht
    // sichern, als beim naechsten unruhigen Start eine Nonsens-Epoche zu
    // liefern.
    if (meshcom_settings.node_date_year < 2025)
        return;

    size_t need = nbrSaveSize();
    uint8_t *buf = (uint8_t *)malloc(need);
    if (buf == NULL)
        return;

    // R7 (Advisor, docs/meshcom5-campaign.md Welle 4): rohe UTC-Epoche
    // sichern (getUnixClock() zieht node_utcoff bereits ab), damit ein
    // --utcoff-Wechsel zwischen Sichern und Laden das Alter der Zeilen nicht
    // verschiebt.
    uint32_t raw_utc = getUnixClock();

    size_t n = nbrSave(nbrMatrix, raw_utc, now_min, buf, need);
    if (n == need)
    {
        if (SD.exists("/topo.dat"))
            SD.remove("/topo.dat");

        File f = SD.open("/topo.dat", FILE_WRITE);
        if (f)
        {
            f.write(buf, n);
            f.close();
        }
    }

    free(buf);
}

#endif // T_DECK || T_DECK_PLUS || T_DECK_PRO

void topoUiChanged(uint16_t now_min)
{
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

    static unsigned long vLastMhMs = 0;
    static unsigned long vLastPathMs = 0;
    unsigned long ms = millis();

    if ((unsigned long)(ms - vLastMhMs) >= TOPO_UI_MIN_INTERVAL_MS)
    {
        vLastMhMs = ms;
        tdeck_refresh_mh_view();
    }

    if ((unsigned long)(ms - vLastPathMs) >= TOPO_UI_MIN_INTERVAL_MS)
    {
        vLastPathMs = ms;
        tdeck_refresh_path_view();
    }

    vSaveTopoIfDue(now_min);

#elif defined(BOARD_T_DECK_PRO)

    static unsigned long vLastMhMs = 0;
    unsigned long ms = millis();

    // TDeck_pro_mheard_disp() -> ui_mheard_disp() gated bereits selbst auf
    // bMhShown (nur die MHeard-Seite hat auf diesem Board eine Live-Ansicht,
    // kein separates Pfad-Screen). Der Ruf hier ist also immer billig, wenn
    // die Seite gerade nicht offen ist.
    if ((unsigned long)(ms - vLastMhMs) >= TOPO_UI_MIN_INTERVAL_MS)
    {
        vLastMhMs = ms;
        TDeck_pro_mheard_disp();
    }

    vSaveTopoIfDue(now_min);

#else
    (void)now_min;
#endif
}

void topoUiBoot(void)
{
#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS) || defined(BOARD_T_DECK_PRO)

    // Zwei mains (T-Deck-Familie kann ihr eigenes SD-Init vor dem
    // gemeinsamen Aufrufer aus W4c durchlaufen haben) koennten hier
    // theoretisch beide landen -- ein zweiter Aufruf ist ein billiges No-Op,
    // kein zweites Laden auf eine schon gefuellte Matrix.
    static bool vBooted = false;
    if (vBooted)
        return;
    vBooted = true;

    uint16_t now_min = uptimeMin16();

    if (SD.exists("/topo.dat"))
    {
        File f = SD.open("/topo.dat", FILE_READ);
        if (f)
        {
            size_t want = nbrSaveSize();
            uint8_t *buf = (uint8_t *)malloc(want);
            if (buf != NULL)
            {
                size_t got = (size_t)f.read(buf, want);
                f.close();

                // R7: die gesicherte Epoche ist roh-UTC (siehe vSaveTopoIfDue()).
                uint32_t saved_epoch = nbrSavedEpoch(buf, got);

                if (saved_epoch == 0 || !vEpochPlausible(saved_epoch))
                {
                    // Fremder oder kaputter Kopf (anderer Formatversion/-Board,
                    // abgeschnittene Datei, oder eine ohne gueltige Uhr
                    // gesicherte Nonsens-Epoche, Advisor R2a) -- weg damit,
                    // statt auf Treffer im naechsten Boot zu hoffen.
                    printfdeb("[TOPO]...topo.dat fremd/kaputt/unplausibel, geloescht\n");
                    SD.remove("/topo.dat");
                }
                else
                {
                    if (meshcom_settings.node_date_year < 2025)
                    {
                        // Uhr vorstellen, genau wie loadTimePersistence() es
                        // frueher aus getLatestMHeardTimestamp() tat: die
                        // gesicherte Epoche ist roh-UTC, Clock::SetClock()
                        // erwartet UTC + utcoff (siehe dessen Aufrufer
                        // setCurrentTime()). L2 (Advisor): float->uint32_t bei
                        // negativem utcoff ist UB, darum der Umweg ueber int32_t.
                        time_t tsNow = (time_t)(saved_epoch +
                                       (uint32_t)(int32_t)(meshcom_settings.node_utcoff * 3600.0));
                        MyClock.SetClock(tsNow, false);

                        meshcom_settings.node_date_year   = MyClock.Year();
                        meshcom_settings.node_date_month  = MyClock.Month();
                        meshcom_settings.node_date_day    = MyClock.Day();
                        meshcom_settings.node_date_hour    = MyClock.Hour();
                        meshcom_settings.node_date_minute = MyClock.Minute();
                        meshcom_settings.node_date_second = MyClock.Second();
                    }

                    // Nur laden, wenn die Uhr jetzt gueltig ist (entweder schon
                    // vorher, oder gerade eben aus derselben Datei vorgestellt)
                    // -- nbrLoad() selbst lehnt now_epoch==0 zwar auch ab, aber
                    // getUnixClock() auf eine ungesetzte Uhr liefert Muell statt
                    // sauber 0 (mktime() auf tm_year==-1900). nbrLoad() rechnet
                    // Zeilen/Kanten auf die neue Bootminute um und verwirft, was
                    // ueber NBR_WINDOW_MIN hinauswaechst.
                    if (meshcom_settings.node_date_year >= 2025)
                    {
                        // R3 (Advisor): nbrInit() lief noch nicht -- die
                        // Matrix ist lazy, das erste OnRxDone tut es sonst.
                        // Ohne diesen Aufruf wuerde nbrLoad() JEDES Abbild
                        // annehmen und dessen gesichertes Rufzeichen als
                        // Zeile 0 ("ich") uebernehmen (--setcall seither
                        // geaendert, oder eine fremde SD-Karte). nbrInit()
                        // setzt Zeile 0 auf das echte, gerade aus dem Flash
                        // geladene Rufzeichen und erhaelt boot_epoch dabei
                        // (nbrIInitAll() sichert/restauriert es um den
                        // memset() herum) -- an der Stelle noch 0, das setzt
                        // nbrLoad() unten ohnehin neu.
                        nbrInit(nbrMatrix, meshcom_settings.node_call, now_min);

                        bool ok = nbrLoad(nbrMatrix, buf, got, getUnixClock(), now_min);
                        if (ok)
                        {
                            // nbrLoad() setzt die Bootepoche aus der rohen
                            // UTC-Epoche; Clock::SetClock() fuettert sonst die
                            // Knoten-Epoche (UTC + utcoff). Wieder auf
                            // dieselbe Konvention bringen.
                            nbrSetClock(nbrMatrix,
                                        (uint32_t)getUnixClock() +
                                            (uint32_t)(int32_t)(meshcom_settings.node_utcoff * 3600.0),
                                        now_min);
                        }
                        else
                        {
                            // Eigenes Rufzeichen passt nicht zum gesicherten
                            // Abbild (R3) -- die Matrix bleibt bei nbrInit()s
                            // leerer Zeile 0 stehen, die Datei ist ohnehin
                            // wertlos fuer dieses Geraet.
                            printfdeb("[TOPO]...topo.dat gehoert zu einem anderen Rufzeichen, geloescht\n");
                            SD.remove("/topo.dat");
                        }
                    }
                }

                free(buf);
            }
            else
            {
                f.close();
            }
        }
    }

    // Einmalig: die alten, nun ersetzten Persistenzdateien loswerden. Beide
    // Namen bleiben absichtlich nur hier als Literal stehen (Gate-Stringscan,
    // docs/meshcom5-campaign.md Welle 4 Brief W4d).
    if (SD.exists("/mheard.dat"))
        SD.remove("/mheard.dat");
    if (SD.exists("/mhpath.dat"))
        SD.remove("/mhpath.dat");

#endif
}
