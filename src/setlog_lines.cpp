// SL-00 -- Implementierung der `--setlog on`-Formatierer. Siehe setlog_lines.h
// fuer die bindenden Regeln (kein Arduino, kein statischer Puffer, kein
// Semikolon als Trenner, kein Praefix, kein Zeilenumbruch).

#include "setlog_lines.h"

#include <stdio.h>

// snprintf() liefert die Laenge, die OHNE Kuerzung noetig gewesen waere, und
// bei Encoding-Fehlern einen negativen Wert. Beides ist als Rueckgabewert
// unbrauchbar: der Aufrufer haengt das Ergebnis an eine Ausgabe an und muss
// wissen, wieviel wirklich im Puffer steht.
static int setlogClamp(int written, size_t n)
{
    if(n == 0)
        return 0;

    if(written < 0)
        return 0;

    if((size_t)written >= n)
        return (int)(n - 1);

    return written;
}

int setlogFormatRxTail(char *buf, size_t n, int16_t rssi, int8_t snr,
                       bool dup, bool own_echo, uint32_t t_ms)
{
    if(buf == NULL || n == 0)
        return 0;

    return setlogClamp(snprintf(buf, n, " RSSI:%d SNR:%d DUP:%c OWN:%c t=%lu",
                                (int)rssi, (int)snr,
                                dup ? 'd' : 'n',
                                own_echo ? 'e' : '-',
                                (unsigned long)t_ms), n);
}

int setlogFormatRly(char *buf, size_t n, uint32_t msg_id, char typ,
                    uint8_t hop, const char *reason, int prio, int slot)
{
    if(buf == NULL || n == 0)
        return 0;

    return setlogClamp(snprintf(buf, n, "RLY x%08lX %c H%02X q=%s prio=%d slot=%d",
                                (unsigned long)msg_id, typ, (unsigned)hop,
                                (reason == NULL) ? "?" : reason,
                                prio, slot), n);
}

int setlogFormatTx(char *buf, size_t n, uint32_t msg_id, char typ, uint8_t hop,
                   int prio, char src, uint32_t wait_ms, int q_depth,
                   int cad, uint16_t len, uint32_t t_ms)
{
    if(buf == NULL || n == 0)
        return 0;

    return setlogClamp(snprintf(buf, n,
                                "TX x%08lX %c H%02X prio=%d src=%c wait=%lu q=%d cad=%d len=%u t=%lu",
                                (unsigned long)msg_id, typ, (unsigned)hop,
                                prio, src, (unsigned long)wait_ms, q_depth,
                                cad, (unsigned)len, (unsigned long)t_ms), n);
}

int setlogFormatErr(char *buf, size_t n, int16_t rssi, int8_t snr,
                    uint16_t len, int32_t ferr_hz, uint32_t t_ms)
{
    if(buf == NULL || n == 0)
        return 0;

    return setlogClamp(snprintf(buf, n, "ERR rssi=%d snr=%d len=%u ferr=%ld t=%lu",
                                (int)rssi, (int)snr, (unsigned)len,
                                (long)ferr_hz, (unsigned long)t_ms), n);
}

int setlogFormatStat(char *buf, size_t n, const struct setlogStatFields *f)
{
    if(buf == NULL || n == 0 || f == NULL)
        return 0;

    // Der Formatstring bleibt deutlich unter den 300 Byte, bei denen
    // printfdeb_functions.cpp (char nformat[300]) den FORMATSTRING kappt --
    // nicht nur die Ausgabe. Die native Testsuite haelt diese Schranke fest.
    return setlogClamp(snprintf(buf, n,
                                "STAT util=%u rx=%lu tx=%lu newid=%lu dup=%lu err=%lu txn=%lu txfail=%lu "
                                "ringmax=%u/%u drop=%u/%u/%u/%u/%u mh=%u heap=%lu trk=%lu/%u "
                                "fw=%u%c/%lu up=%lu t=%lu",
                                (unsigned)f->util_pct,
                                (unsigned long)f->rx_ms, (unsigned long)f->tx_ms,
                                (unsigned long)f->newid, (unsigned long)f->dup,
                                (unsigned long)f->err, (unsigned long)f->txn,
                                (unsigned long)f->txfail,
                                (unsigned)f->ringmax, (unsigned)f->ring_size,
                                (unsigned)f->drop[0], (unsigned)f->drop[1],
                                (unsigned)f->drop[2], (unsigned)f->drop[3],
                                (unsigned)f->drop[4],
                                (unsigned)f->mh, (unsigned long)f->heap,
                                (unsigned long)f->trk_interval_s,
                                (unsigned)f->trk_consistent,
                                (unsigned)f->fw_major, f->fw_sub,
                                (unsigned long)f->flash,
                                (unsigned long)f->up_s,
                                (unsigned long)f->t_ms), n);
}

int setlogFormatGwi(char *buf, size_t n, uint32_t msg_id, char typ,
                    uint8_t hop, const char *from_call, uint32_t t_ms)
{
    if(buf == NULL || n == 0)
        return 0;

    return setlogClamp(snprintf(buf, n, "GWI x%08lX %c H%02X from=%s t=%lu",
                                (unsigned long)msg_id, typ, (unsigned)hop,
                                (from_call == NULL) ? "?" : from_call,
                                (unsigned long)t_ms), n);
}

int setlogFormatGwu(char *buf, size_t n, uint32_t msg_id, char typ,
                    uint8_t hop, uint32_t t_ms)
{
    if(buf == NULL || n == 0)
        return 0;

    return setlogClamp(snprintf(buf, n, "GWU x%08lX %c H%02X t=%lu",
                                (unsigned long)msg_id, typ, (unsigned)hop,
                                (unsigned long)t_ms), n);
}
