#pragma once

// R2-01: eine MHeard-Zeile als Datensatz statt als Text.
//
// Bis 2026-09-16 lag jeder Eintrag als pipe-getrennte ZEICHENKETTE in
// `mheardBuffer[MAX_MHEARD][60]`:
//
//     snprintf(cBuffer, 60, "%s|%s|%c|%i|%u|%i|%i|%.1lf|%i|%i|%i|", ...)
//
// und wurde bei JEDEM Lesen von `decodeMHeard()` Zeichen fuer Zeichen wieder
// zerlegt -- mit Arduino-`String`-Anhaengen pro Zeichen. Das ist eine
// Serialisierung mit anschliessender Deserialisierung fuer Daten, die das
// Geraet nie verlassen. Sie kostet 60 Byte je Eintrag statt 20, und sie
// erzeugt Heap-Verkehr auf genau den Pfaden, die `--mheard`, das
// JSON-Register und die Web-Oberflaeche bedienen. Auf dieser Plattform ist
// das nicht gratis: Heap-Verkehr pro Ausgabezeile ist der dokumentierte Grund,
// aus dem der BLE-Verbindungsaufbau scheitert.
//
// WARUM DATUM UND UHRZEIT HIER MITLIEGEN und nicht aus `mheardEpoch[]`
// kommen: `mheardEpoch[]` gibt es zwar parallel, aber es in Datum/Uhrzeit
// zurueckzurechnen braeuchte einen Formatierer fuer einen BELIEBIGEN
// Zeitstempel. `getDateString()`/`getTimeString()` formatieren die AKTUELLE
// Uhr, nicht eine vergangene. Sechs gepackte Bytes hier kosten weniger als
// diese Abhaengigkeit, und sie halten exakt das fest, was heute angezeigt
// wird.
//
// Arduino-frei, damit das Packen auf dem Host pruefbar ist
// (test/test_mheard_record, env native_mheard_record).

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>    // snprintf -- siehe mheardRoundDist()
#include <stdlib.h>   // atof

// 20 Byte je Eintrag (4-Byte-ausgerichtet) gegen 60 vorher.
struct MheardRecord
{
    float   mr_dist;       // Meter. Siehe die Rundungsnotiz unten.
    int16_t mr_rssi;
    uint8_t mr_year;       // Jahr - 2000, also 0..255 fuer 2000..2255
    uint8_t mr_month;      // 1..12
    uint8_t mr_day;        // 1..31
    uint8_t mr_hour;       // 0..23
    uint8_t mr_minute;     // 0..59
    uint8_t mr_second;     // 0..59
    char    mr_type;       // Nutzlasttyp, ein Zeichen
    uint8_t mr_hw;
    int8_t  mr_snr;
    uint8_t mr_mod;
    uint8_t mr_path_len;
    uint8_t mr_mesh;
    uint8_t mr_ncount;
};

/**
 * @brief Rundet eine Entfernung auf die Genauigkeit, die der Textsatz hatte.
 *
 * Der alte Satz schrieb `%.1lf`, speicherte also EINE Nachkommastelle, und
 * `decodeMHeard()` las genau diesen gerundeten Wert zurueck. Das JSON-Register
 * gibt `mh_dist` ROH aus (`mhdoc["DIST"] = mheardLine.mh_dist`), nicht
 * formatiert -- wer aus dem Puffer liest, bekam also bisher den gerundeten
 * Wert auf die Leitung.
 *
 * Ein `float` im Datensatz koennte die volle Genauigkeit halten. Das waere
 * eine stille Aenderung an einem Format, das die App liest, und dieser Umbau
 * geht ueber die SPEICHERUNG, nicht ueber die Genauigkeit. Deshalb wird beim
 * Ablegen gerundet: die Ausgabe bleibt Byte fuer Byte dieselbe.
 *
 * (Nebenbefund, hier nicht angefasst: der frische Pfad in `updateMheard()`
 * gibt `mh_dist` UNGERUNDET ins JSON, der Puffer-Pfad gerundet. Die beiden
 * Wege widersprechen sich also schon heute. Das ist eine eigene Entscheidung.)
 */
static inline float mheardRoundDist(double dist)
{
    if (dist < 0.0)
        return -1.0f;                      // "noch nicht berechnet"
    // Dies MUSS ueber printf laufen. Zwei Anlaeufe davor waren falsch, und
    // beide hat der Test gefangen:
    //
    //   (long)(x*10 + 0.5)        rundet die Haelfte immer nach oben.
    //                             printf nicht: %.1lf von 1.25 ist "1.2",
    //                             weil IEEE zur GERADEN Ziffer rundet.
    //   nearbyint(x*10) / 10      rundet zwar zur geraden Ziffer, aber die
    //                             MULTIPLIKATION zerstoert vorher die
    //                             Information, die entscheidet: 0.05 liegt
    //                             als double knapp UEBER 0.05, doch 0.05*10
    //                             rundet auf exakt 0.5, und 0.5 geht zur
    //                             geraden Null. printf sieht den
    //                             urspruenglichen Wert und liefert "0.1".
    //
    // Binaeres Skalieren kann eine DEZIMALE Rundung grundsaetzlich nicht
    // nachbilden. Der einzige Weg, exakt denselben Wert wie bisher zu
    // bekommen, ist derselbe Weg wie bisher: formatieren und zurueckparsen --
    // genau das tat der alte Satz mit snprintf("%.1lf") und toFloat(). Neu
    // ist nur, dass es EINMAL beim Ablegen passiert statt bei jedem Lesen,
    // und nur fuer dieses eine Feld statt fuer den ganzen Datensatz.
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%.1lf", dist);
    return (float)atof(tmp);
}

/**
 * @brief Zerlegt "YYYY-MM-DD" in den Datensatz. true bei genau diesem Format.
 *
 * Streng absichtlich: `getDateString()` ist der einzige Erzeuger und liefert
 * feste Breite. Alles andere ist ein Fehler und darf nicht halb uebernommen
 * werden -- ein halb gefuellter Datensatz sieht wie ein gueltiger aus.
 */
static inline bool mheardSetDate(MheardRecord &rec, const char *date)
{
    if (date == 0)
        return false;
    for (int i = 0; i < 10; i++)
    {
        if (date[i] == 0)
            return false;
        bool want_dash = (i == 4 || i == 7);
        bool is_dash = (date[i] == '-');
        bool is_digit = (date[i] >= '0' && date[i] <= '9');
        if (want_dash != is_dash || (!want_dash && !is_digit))
            return false;
    }
    int year = (date[0]-'0')*1000 + (date[1]-'0')*100 + (date[2]-'0')*10 + (date[3]-'0');
    int mon  = (date[5]-'0')*10 + (date[6]-'0');
    int day  = (date[8]-'0')*10 + (date[9]-'0');
    if (year < 2000 || year > 2255)
        return false;
    rec.mr_year  = (uint8_t)(year - 2000);
    rec.mr_month = (uint8_t)mon;
    rec.mr_day   = (uint8_t)day;
    return true;
}

/** @brief Zerlegt "HH:MM:SS". Gleiche Strenge wie mheardSetDate(). */
static inline bool mheardSetTime(MheardRecord &rec, const char *tm)
{
    if (tm == 0)
        return false;
    for (int i = 0; i < 8; i++)
    {
        if (tm[i] == 0)
            return false;
        bool want_colon = (i == 2 || i == 5);
        bool is_colon = (tm[i] == ':');
        bool is_digit = (tm[i] >= '0' && tm[i] <= '9');
        if (want_colon != is_colon || (!want_colon && !is_digit))
            return false;
    }
    rec.mr_hour   = (uint8_t)((tm[0]-'0')*10 + (tm[1]-'0'));
    rec.mr_minute = (uint8_t)((tm[3]-'0')*10 + (tm[4]-'0'));
    rec.mr_second = (uint8_t)((tm[6]-'0')*10 + (tm[7]-'0'));
    return true;
}

/** @brief Schreibt "YYYY-MM-DD" (10 Zeichen + NUL). out_size >= 11. */
static inline void mheardFormatDate(const MheardRecord &rec, char *out, size_t out_size)
{
    if (out == 0 || out_size < 11)
    {
        if (out && out_size)
            out[0] = 0;
        return;
    }
    int year = 2000 + rec.mr_year;
    out[0] = (char)('0' + (year / 1000) % 10);
    out[1] = (char)('0' + (year / 100) % 10);
    out[2] = (char)('0' + (year / 10) % 10);
    out[3] = (char)('0' + year % 10);
    out[4] = '-';
    out[5] = (char)('0' + (rec.mr_month / 10) % 10);
    out[6] = (char)('0' + rec.mr_month % 10);
    out[7] = '-';
    out[8] = (char)('0' + (rec.mr_day / 10) % 10);
    out[9] = (char)('0' + rec.mr_day % 10);
    out[10] = 0;
}

/** @brief Schreibt "HH:MM:SS" (8 Zeichen + NUL). out_size >= 9. */
static inline void mheardFormatTime(const MheardRecord &rec, char *out, size_t out_size)
{
    if (out == 0 || out_size < 9)
    {
        if (out && out_size)
            out[0] = 0;
        return;
    }
    out[0] = (char)('0' + (rec.mr_hour / 10) % 10);
    out[1] = (char)('0' + rec.mr_hour % 10);
    out[2] = ':';
    out[3] = (char)('0' + (rec.mr_minute / 10) % 10);
    out[4] = (char)('0' + rec.mr_minute % 10);
    out[5] = ':';
    out[6] = (char)('0' + (rec.mr_second / 10) % 10);
    out[7] = (char)('0' + rec.mr_second % 10);
    out[8] = 0;
}
