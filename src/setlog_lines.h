#ifndef _SETLOG_LINES_H_
#define _SETLOG_LINES_H_

// SL-00 -- reine Formatierer fuer die zusaetzlichen `--setlog on`-Zeilen
// (Implementierungsplan SL-01..SL-07, 2026-09-02).
//
// Regeln, die diese Datei bindend halten muss:
//
//  * Keine Arduino-Abhaengigkeit. Die Uebersetzungseinheit baut unter
//    `env:native` mit `-D NATIVE_BUILD=1` und nur <stdio.h>/<stdint.h>/
//    <stdbool.h>/<string.h>. Damit sind die Zeilen ohne Hardware pruefbar.
//  * Keine statischen Puffer. Jeder Formatierer schreibt in den vom Aufrufer
//    gestellten `char buf[]` und schreibt nie mehr als `n` Byte (inkl. NUL).
//  * Kein Semikolon als Feldtrenner. printfdeb() schreibt `;` ausserhalb des
//    CSV-Modus um (src/printfdeb_format.h). Trenner ist immer das Leerzeichen,
//    Felder sind `key=value` bzw. `KEY:value` wie in der bestehenden
//    [LOG]-Zeile.
//  * Die Formatierer liefern den Zeilenrumpf OHNE `HH:MM:SS [LOG] `-Praefix
//    und OHNE `\n`. Beides setzt der Aufrufer, so wie printBuffer_aprs() es
//    heute schon tut.
//
// Rueckgabewert ist immer die Zahl der tatsaechlich geschriebenen Zeichen
// (ohne NUL) -- also bereits auf `n-1` geklemmt, nicht der Wunschwert von
// snprintf(). Bei `n == 0` wird `buf` nicht angefasst und 0 geliefert.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////
// SL-01 -- Anhang an die bestehende RX-Zeile.
//
// Ergebnis (fuehrendes Leerzeichen ist Teil des Anhangs):
//
//     " RSSI:<int> SNR:<int> DUP:<n|d> OWN:<-|e> t=<ms>"
//
// dup      = true  -> `DUP:d` (msg_id lag schon im Dedup-Ring), sonst `DUP:n`
// own_echo = true  -> `OWN:e` (eigenes Rufzeichen steht im Pfad), sonst `OWN:-`
// t_ms     = millis() beim Empfang
//
// Laenge im schlimmsten Fall (RSSI -32768, SNR -128, t = 2^32-1): 46 Zeichen;
// mit realen Pegeln rund 38. Der Aufrufer braucht also 47 Byte Puffer --
// OnRxDone() in lora_functions.cpp stellt 56.
int setlogFormatRxTail(char *buf, size_t n, int16_t rssi, int8_t snr,
                       bool dup, bool own_echo, uint32_t t_ms);

//////////////////////////////////////////////////////////////////////////////
// SL-02 -- Relay-Entscheidung mit Grund, eine Zeile je NEUEM Frame.
//
//     "RLY x<id> <typ> H<hop> q=<code> prio=<n> slot=<n>"
//
// reason ist entweder "tx" (eingereiht, dann sind prio/slot gueltig) oder
// genau einer der Ablehnungsgruende aus SL-02 (unconf, self, aprs, nomesh,
// gwfilter, gwcap, ping, hop0, loop, full). Wurde nichts eingereiht, geben
// die Aufrufer prio = 0 und slot = -1 mit.
int setlogFormatRly(char *buf, size_t n, uint32_t msg_id, char typ,
                    uint8_t hop, const char *reason, int prio, int slot);

//////////////////////////////////////////////////////////////////////////////
// SL-03 -- jede eigene Sendung, beim erfolgreichen Start (nicht bei TX_DONE).
//
//     "TX x<id> <typ> H<hop> prio=<n> src=<o|r|g> wait=<ms> q=<depth>
//      cad=<n> len=<bytes> t=<ms>"   (eine Zeile, hier nur umbrochen)
//
// src stammt aus ringSource[] (SL-06): 'o' eigene Nachricht, 'r' Relay,
// 'g' vom Server eingespeist.
int setlogFormatTx(char *buf, size_t n, uint32_t msg_id, char typ, uint8_t hop,
                   int prio, char src, uint32_t wait_ms, int q_depth,
                   int cad, uint16_t len, uint32_t t_ms);

//////////////////////////////////////////////////////////////////////////////
// SL-04 -- verlorener Frame (CRC/RX-Error), plattformunabhaengig.
//
//     "ERR rssi=<int> snr=<int> len=<n> ferr=<hz> t=<ms>"
//
// Wo eine Plattform einen Wert nicht liefert (nRF52 kennt keine Laenge),
// geben die Aufrufer 0 mit.
int setlogFormatErr(char *buf, size_t n, int16_t rssi, int8_t snr,
                    uint16_t len, int32_t ferr_hz, uint32_t t_ms);

//////////////////////////////////////////////////////////////////////////////
// SL-05 -- Statuszeile alle fuenf Minuten (PRIO_STAT_INTERVAL_S).
//
// Viele Werte, deshalb ein Struct statt einer Parameterlawine. Alle
// Intervallzaehler setzt der Aufrufer nach dem Druck auf null (exchange(0)).
struct setlogStatFields
{
    uint8_t  util_pct;        // Kanalauslastung ueber das 5-min-Fenster in %
    uint32_t rx_ms;           // aufsummierte RX-Luftzeit im Fenster
    uint32_t tx_ms;           // aufsummierte TX-Luftzeit im Fenster
    uint32_t newid;           // neue msg_id im Dedup-Ring (SL-01)
    uint32_t dup;             // erkannte Kopien (SL-01)
    uint32_t err;             // RX-/CRC-Fehler (SL-04)
    uint32_t txn;             // eigene Sendungen (SL-03)
    uint32_t txfail;          // vom TX-Watchdog abgebrochene Sendungen
    uint8_t  ringmax;         // Hochwasser von txRingDepth() im Fenster
    uint8_t  ring_size;       // MAX_RING des Boards
    uint16_t drop[5];         // stat_drop_count[1..5], Prio 1..5
    uint16_t mh;              // getMheardCount()
    uint32_t heap;            // freier Heap in Byte
    uint32_t trk_interval_s;  // Trickle-Intervall in Sekunden
    uint16_t trk_consistent;  // konsistente HEYs seit letztem Reset
    uint8_t  fw_major;        // shortVERSION()
    char     fw_sub;          // shortSUBVERSION()
    uint32_t flash;           // FLASH_VERSION
    uint32_t up_s;            // Uptime in Sekunden
    uint32_t t_ms;            // millis()
};

//     "STAT util=<pct> rx=<ms> tx=<ms> newid=<n> dup=<n> err=<n> txn=<n>
//      txfail=<n> ringmax=<n>/<MAX_RING> drop=<p1>/<p2>/<p3>/<p4>/<p5>
//      mh=<n> heap=<bytes> trk=<interval_s>/<consistent>
//      fw=<major><sub>/<flash> up=<s> t=<ms>"   (eine Zeile)
int setlogFormatStat(char *buf, size_t n, const struct setlogStatFields *f);

//////////////////////////////////////////////////////////////////////////////
// SL-06 -- Gateway-Einspeisung und -Upload.
//
//     "GWI x<id> <typ> H<hop> from=<call> t=<ms>"
//     "GWU x<id> <typ> H<hop> t=<ms>"
int setlogFormatGwi(char *buf, size_t n, uint32_t msg_id, char typ,
                    uint8_t hop, const char *from_call, uint32_t t_ms);

int setlogFormatGwu(char *buf, size_t n, uint32_t msg_id, char typ,
                    uint8_t hop, uint32_t t_ms);

//////////////////////////////////////////////////////////////////////////////
// SL-06/SL-03 -- Herkunftskennung fuer ringSource[].
//
// Bewusst `inline` im Header und nicht in setlog_lines.cpp: txring_functions.cpp
// braucht die Abbildung, wird aber in `env:native_aprs` gebaut, wo
// setlog_lines.cpp NICHT im build_src_filter steht (dort laeuft test_txring).
// Als Header-Inline entsteht keine Link-Abhaengigkeit.
//
// Abbildung der `source`-Labels, die addTxRingEntry() heute nur in der
// `RING_WRITE ... src=`-Zeile druckt:
//
//   "rx_relay", "rx_ack_fwd", "rx_dm_ack_gw", "rx_dm_ack_new"  -> 'r'
//        alles, was aus einem Empfang heraus weitergereicht wird
//   "udp_rx"                                                    -> 'g'
//        vom Server eingespeist (WiFi-UDP und RAK-Ethernet nutzen dasselbe
//        Label)
//   alles andere ("user_msg", "beacon", "user_pos", "phone_msg", ...) -> 'o'
static inline uint8_t setlogRingSourceCode(const char *source)
{
    if(source == NULL)
        return (uint8_t)'o';

    if(strncmp(source, "rx_", 3) == 0)
        return (uint8_t)'r';

    if(strcmp(source, "udp_rx") == 0)
        return (uint8_t)'g';

    return (uint8_t)'o';
}

#endif // _SETLOG_LINES_H_
