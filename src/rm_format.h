// RM extended commands, contract C3: pure reply formatters (header-only, no Arduino, no allocation,
// no globals, no printf family, so nothing depends on newlib-nano's %lld/%zu gaps).
// Host test: pio test -e native_rm_format
//
// Every function builds ONE reply BODY (the text after "ok ", the dispatcher adds the prefix) from a
// plain input struct into the caller buffer `char *out, size_t n` and returns the length written.
// It returns 0 and leaves out[0] == '\0' when the body does not fit n (never a truncated reply) or,
// for the free-text formatters, when the text is longer than its documented maximum. Values are
// clamped to their documented domain, so strlen(out) <= RM_FMT_BODY_MAX (105) for ALL inputs.
// Non-finite floats (NaN, +-inf) are detected on the IEEE exponent bits (no isnan(), no x != x: the
// nRF52 builds used -ffast-math, which folds those away); absent sensors/fields print `-`.
// Non-finite rules: pos with a non-finite lat or lon, radio with a non-finite freq or bw -> return 0
// with out[0] == '\0' (the dispatcher answers an error, nothing is published); mh direct with hasPos
// but a non-finite lat or lon prints `la=- lo=-` (as absent); a non-finite sensor value, distKm or
// rcSnr prints `-`.
// Decimals are rounded half away from zero on the STORED BINARY value, not truncated: 433.1745f is
// stored as 433.17449951..., so it prints 433.174. Values on the real channel grid (433.175, 868.0,
// 62.5 kHz ...) print exactly: the stored value lies far inside the rounding band of the printed
// decimal. A value that rounds to zero prints without a sign.
// Calls (mh rows, rc call, via chain): UPPER CASE A-Z 0-9 and `-` only; a-z is folded to upper case,
// every other byte becomes `?`. A call is never cut: a row call or rc call longer than 9 chars is
// emitted as `?`; a via call longer than 9 chars ends the via chain there (the calls before it stay).
// Output bytes: space, A-Z a-z 0-9 and - . / = + _ @ ? ( ) , * # only (free text is printed as is).
//
// Reply contract (also the contract for a second sender, e.g. McApp). "-" = absent.
//
//  cmd     reply body                                                   keys
//  ------  -----------------------------------------------------------  -------------------------------
//  radio   f=433.175 sf=11 cr=5 bw=250 p=10/22                          f frequency MHz, 3 decimals,
//                                                                       rounded; sf spreading factor;
//                                                                       cr coding rate denominator (5 =
//                                                                       4/5); bw kHz, no trailing zeros
//                                                                       (250, 62.5, 31.25); p current/max
//                                                                       TX power dBm. Never absent.
//  name    n=Martin                                                     n node name, last field, as is;
//                                                                       `n=-` when empty. Max 19 chars.
//  atxt    a=MeshCom Garten                                             a APRS comment, last field, as
//                                                                       is; `a=-` when empty. Max 39.
//  pos     48.40812 11.73812 492 gps                                    lat lon signed degrees, 5
//                                                                       decimals; alt metres (integer);
//                                                                       src gps | nofix | set
//  sens    t=21.4 h=45 p=1013.2 t2=-                                    t temp degC 1 dec; h humidity %
//                                                                       integer; p pressure hPa 1 dec;
//                                                                       t2 second temp degC 1 dec; `-`
//                                                                       when the sensor is absent
//  mh      <total> <next> CALL min CALL min ...                         total entries of the list; next
//          (page)                                                       list index of the first row NOT
//                                                                       in this reply, `-` when the list
//                                                                       is exhausted; then pairs of
//                                                                       call (upper case) and minutes
//                                                                       since last direct reception
//                                                                       (count format). As many rows as
//                                                                       fit 105 chars; empty list `0 -`;
//                                                                       first >= total is `<total> -`
//                                                                       with no rows (the dispatcher
//                                                                       answers `err end` itself).
//  mh      d g=1 m=1 r=-95 s=8 la=48.4231 lo=11.7871 di=4.0 a=499       d direct node; g gateway 0/1;
//          (direct) n=15 x=14 h=18 t=0                                  m mesh 0/1; r RSSI dBm; s SNR dB;
//                                                                       la lo 4 decimals; di distance km
//                                                                       1 dec (max 9999.9); a altitude m
//                                                                       (-999..99999); n its neighbour
//                                                                       count; x neighbours only it
//                                                                       hears; h neighbours it hears;
//                                                                       t minutes since heard (count)
//  mh      r h=3 k=2 g=0 m=- rc=17.8@DL2JA-2 t=3 v=DK5EN-98,DL2JA-2     r routed node; h hops of the
//          (routed)                                                     shortest route; k number of
//                                                                       routes; g gateway 0/1; m mesh
//                                                                       0/1 (`-` unknown); rc relay
//                                                                       SNR dB 1 dec @ relay call (`-`
//                                                                       absent or empty relay call); t
//                                                                       minutes since heard
//                                                                       (count); v via chain joined by
//                                                                       `,` (leading calls that fit,
//                                                                       never half a call; `-` if none)
//  txq     q=3/20 bp=quiet tx=<c> rt=<c> dr=<c> u=12                    q queued/capacity frames; bp
//                                                                       quiet | qrs | qrt (`?` unknown);
//                                                                       tx frames sent; rt retransmissions;
//                                                                       dr dropped; u channel utilisation
//                                                                       percent (0..100)
//                                                                       Firmware note: tx and dr are the
//                                                                       counters of the running statistics
//                                                                       interval, not totals since boot; rt
//                                                                       and u are 0 where the node keeps no
//                                                                       such value.
//  mbox    m=heard u=12/50 b=1834 a=3/20 st=<c> dl=<c> ak=<c>           m off | own | list | heard; u
//          dr=<c> bl=<c> nt=<c>                                         used/slots; b stored bytes; a
//                                                                       mailbox actions in the last hour
//                                                                       / hourly ceiling; st stored; dl
//                                                                       delivered; ak purged on ack; dr
//                                                                       dropped (storetime+cap+slots);
//                                                                       bl blocked by backpressure; nt
//                                                                       :sto notices sent
//  maxhop  t=4 p=2                                                      t max hops of text messages; p
//                                                                       max hops of position beacons
//
// <c> / count format (rmFmtCount): at most 5 chars: 0..99999, then NNNNk (thousands, rounded down, up
// to 9999k), then NNNNM (millions, up to 9999M).
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "rm_commands.h" // RM_NAME_MAX / RM_ATXT_MAX

#define RM_FMT_BODY_MAX 105 // 140 - 4 - 10 - 1 - 1 - 16 - 3 ("ok ")

// Window in minutes for the "heard" lists: the web page and the RM mh reply use the same 3 h (decision D11).
#define RM_HEARD_WINDOW_MIN (3 * 60)

// ---- low-level appender into a local buffer that holds a full body ------------------------------------

struct RmFmtBuf
{
    char s[RM_FMT_BODY_MAX + 1];
    size_t len;
    bool ok;
};

inline void rmfInit(RmFmtBuf &b)
{
    b.len = 0;
    b.ok = true;
    b.s[0] = '\0';
}

inline void rmfCh(RmFmtBuf &b, char c)
{
    if (!b.ok || b.len >= RM_FMT_BODY_MAX)
    {
        b.ok = false;
        return;
    }
    b.s[b.len++] = c;
    b.s[b.len] = '\0';
}

inline void rmfStr(RmFmtBuf &b, const char *s)
{
    while (s != nullptr && *s != '\0' && b.ok)
        rmfCh(b, *s++);
}

inline void rmfU(RmFmtBuf &b, uint32_t v)
{
    char t[11];
    uint8_t i = 0;
    do
    {
        t[i++] = (char)('0' + v % 10);
        v /= 10;
    } while (v != 0 && i < sizeof(t));
    while (i > 0)
        rmfCh(b, t[--i]);
}

inline void rmfI(RmFmtBuf &b, int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo)
        v = lo;
    if (v > hi)
        v = hi;
    if (v < 0)
    {
        rmfCh(b, '-');
        rmfU(b, (uint32_t)(-(int64_t)v));
    }
    else
        rmfU(b, (uint32_t)v);
}

// IEEE-754 finiteness from the exponent bits. The bits are read through a volatile local, so an
// optimiser (clang -Ofast / -ffinite-math-only) cannot fold the test away by assuming the value is
// finite. (isnan() and x != x are not used: -ffast-math folds those away.)
inline bool rmFmtFiniteF(float x)
{
    uint32_t t;
    memcpy(&t, &x, sizeof(t));
    volatile uint32_t u = t;
    return ((u >> 23) & 0xFFu) != 0xFFu;
}

inline bool rmFmtFiniteD(double x)
{
    uint64_t t;
    memcpy(&t, &x, sizeof(t));
    volatile uint64_t u = t;
    return ((u >> 52) & 0x7FFu) != 0x7FFu;
}

// Non-finite -> 0, then clamp to [lo, hi].
inline double rmfClampD(double v, double lo, double hi)
{
    if (!rmFmtFiniteD(v))
        return 0.0;
    return v < lo ? lo : (v > hi ? hi : v);
}

// Fixed-point with `dec` (0..5) decimals, rounded half up; no "-0.0". v must already be clamped.
inline void rmfFixed(RmFmtBuf &b, double v, uint8_t dec)
{
    uint64_t p10 = 1;
    for (uint8_t i = 0; i < dec; i++)
        p10 *= 10;
    double a = v < 0 ? -v : v;
    uint64_t sc = (uint64_t)(a * (double)p10 + 0.5);
    if (v < 0 && sc != 0)
        rmfCh(b, '-');
    rmfU(b, (uint32_t)(sc / p10));
    if (dec > 0)
    {
        rmfCh(b, '.');
        uint32_t fr = (uint32_t)(sc % p10);
        for (uint32_t d = (uint32_t)(p10 / 10); d > 0; d /= 10)
            rmfCh(b, (char)('0' + (fr / d) % 10));
    }
}

// Sensor value: absent / non-finite -> '-'.
inline void rmfOptFixed(RmFmtBuf &b, bool has, float v, double lo, double hi, uint8_t dec)
{
    if (!has || !rmFmtFiniteF(v))
        rmfCh(b, '-');
    else
        rmfFixed(b, rmfClampD((double)v, lo, hi), dec);
}

// Copies the finished body to the caller buffer; 0 + empty string when it does not fit.
inline size_t rmfEmit(char *out, size_t n, const RmFmtBuf &b)
{
    if (out == nullptr || n == 0)
        return 0;
    if (!b.ok || b.len >= n)
    {
        out[0] = '\0';
        return 0;
    }
    memcpy(out, b.s, b.len + 1);
    return b.len;
}

// ---- counts ---------------------------------------------------------------------------------------------

// At most 5 characters + NUL: 0..99999, NNNNk, NNNNM. Returns the length.
inline size_t rmFmtCount(uint32_t v, char out[6])
{
    uint32_t num = v;
    char suf = '\0';
    if (v > 99999u)
    {
        if (v < 10000000u)
        {
            num = v / 1000u;
            suf = 'k';
        }
        else
        {
            num = v / 1000000u;
            if (num > 9999u)
                num = 9999u;
            suf = 'M';
        }
    }
    RmFmtBuf b;
    rmfInit(b);
    rmfU(b, num);
    if (suf != '\0')
        rmfCh(b, suf);
    memcpy(out, b.s, b.len + 1);
    return b.len;
}

inline void rmfCount(RmFmtBuf &b, uint32_t v)
{
    char c[6];
    rmFmtCount(v, c);
    rmfStr(b, c);
}

// ---- radio ----------------------------------------------------------------------------------------------

struct RmRadioIn
{
    float freqMHz; // clamped 0..999.999
    int sf;        // clamped 0..99
    int cr;        // clamped 0..99
    float bwKHz;   // clamped 0..999.99
    int pCur;      // dBm, clamped -99..99
    int pMin;      // dBm, clamped -99..99 (remote floor)
    int pMax;      // dBm, clamped -99..99
};

inline size_t rmFmtRadio(char *out, size_t n, const RmRadioIn &in)
{
    RmFmtBuf b;
    rmfInit(b);
    if (!rmFmtFiniteF(in.freqMHz) || !rmFmtFiniteF(in.bwKHz))
        b.ok = false; // non-finite frequency or bandwidth: no reply body (return 0)
    rmfStr(b, "f=");
    rmfFixed(b, rmfClampD((double)in.freqMHz, 0.0, 999.999), 3);
    rmfStr(b, " sf=");
    rmfI(b, in.sf, 0, 99);
    rmfStr(b, " cr=");
    rmfI(b, in.cr, 0, 99);
    rmfStr(b, " bw=");
    {
        double bw = rmfClampD((double)in.bwKHz, 0.0, 999.99);
        uint32_t sc = (uint32_t)(bw * 100.0 + 0.5);
        rmfU(b, sc / 100u);
        uint32_t fr = sc % 100u;
        if (fr != 0)
        {
            rmfCh(b, '.');
            rmfCh(b, (char)('0' + fr / 10u));
            if (fr % 10u != 0)
                rmfCh(b, (char)('0' + fr % 10u));
        }
    }
    rmfStr(b, " p=");
    rmfI(b, in.pCur, -99, 99);
    rmfCh(b, '/');
    rmfI(b, in.pMin, -99, 99);
    rmfCh(b, '/');
    rmfI(b, in.pMax, -99, 99);
    return rmfEmit(out, n, b);
}

// ---- name / atxt ----------------------------------------------------------------------------------------

inline size_t rmfText(char *out, size_t n, const char *key, const char *text, size_t maxLen)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, key);
    if (text == nullptr || text[0] == '\0')
        rmfCh(b, '-');
    else if (strlen(text) > maxLen)
        b.ok = false; // never a truncated text
    else
        rmfStr(b, text);
    return rmfEmit(out, n, b);
}

inline size_t rmFmtName(char *out, size_t n, const char *name)
{
    return rmfText(out, n, "n=", name, RM_NAME_MAX);
}

inline size_t rmFmtAtxt(char *out, size_t n, const char *atxt)
{
    return rmfText(out, n, "a=", atxt, RM_ATXT_MAX);
}

// ---- blockcall --------------------------------------------------------------------------------------------
// "b=<csv>" ("b=-" when empty). A csv that does not fit the body is cut at an entry boundary and closed with
// ",+N" (N = entries left out); an entry is never cut.
inline size_t rmFmtBlock(char *out, size_t n, const char *csv)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, "b=");
    if (csv == nullptr || csv[0] == '\0')
    {
        rmfCh(b, '-');
        return rmfEmit(out, n, b);
    }
    const size_t room = RM_FMT_BODY_MAX - 2;
    const size_t total = strlen(csv);
    if (total <= room)
    {
        rmfStr(b, csv);
        return rmfEmit(out, n, b);
    }
    unsigned entries = 1;
    for (const char *q = csv; *q; q++)
        if (*q == ',')
            entries++;
    // largest k whose first k entries plus ",+N" fit
    size_t keepLen = 0;
    unsigned keep = 0;
    size_t pos = 0;
    for (unsigned k = 1; k <= entries; k++)
    {
        size_t end = pos;
        while (csv[end] != '\0' && csv[end] != ',')
            end++;
        const unsigned left = entries - k;
        const size_t suffix = left == 0 ? 0 : (left < 10 ? 3 : 4); // ",+N"
        if (end + suffix > room)
            break;
        keep = k;
        keepLen = end;
        if (csv[end] == '\0')
            break;
        pos = end + 1;
    }
    for (size_t i = 0; i < keepLen; i++)
        rmfCh(b, csv[i]);
    if (keep < entries)
    {
        if (keep > 0)
            rmfCh(b, ',');
        rmfCh(b, '+');
        rmfU(b, entries - keep);
    }
    return rmfEmit(out, n, b);
}

// ---- pos ------------------------------------------------------------------------------------------------

enum RmPosSrc
{
    RM_POS_GPS,
    RM_POS_NOFIX,
    RM_POS_SET
};

struct RmPosIn
{
    double lat; // signed degrees, clamped +-90
    double lon; // signed degrees, clamped +-180
    int alt;    // metres, clamped -999..99999
    RmPosSrc src;
};

inline size_t rmFmtPos(char *out, size_t n, const RmPosIn &in)
{
    RmFmtBuf b;
    rmfInit(b);
    if (!rmFmtFiniteD(in.lat) || !rmFmtFiniteD(in.lon))
        b.ok = false; // non-finite position: no reply body (return 0)
    rmfFixed(b, rmfClampD(in.lat, -90.0, 90.0), 5);
    rmfCh(b, ' ');
    rmfFixed(b, rmfClampD(in.lon, -180.0, 180.0), 5);
    rmfCh(b, ' ');
    rmfI(b, in.alt, -999, 99999);
    rmfCh(b, ' ');
    rmfStr(b, in.src == RM_POS_GPS ? "gps" : (in.src == RM_POS_SET ? "set" : "nofix"));
    return rmfEmit(out, n, b);
}

// ---- sens -----------------------------------------------------------------------------------------------

struct RmSensIn
{
    bool hasT, hasH, hasP, hasT2;
    float t, h, p, t2; // t, t2 degC (-999.9..999.9); h percent (0..999); p hPa (0..9999.9)
};

// Presence inputs of the sens reply, taken from the driver globals by the executor. *On is the
// config flag, *Found the driver's detection result. The BMP/BME flags only pick the I2C address;
// the chip (bmxIsBme: ID 0x60 = BME280, has humidity) comes from the driver.
struct RmSensFlags
{
    bool bmpOn, bmeOn, bmxFound, bmxIsBme;
    bool bme680On, bme680Found;
    bool aht20On, aht20Found;
    bool sht21On, sht21Found;
    bool bmp3On, bmp3Found;
    bool oneWireOn; // t2 only (a DHT on that path is not claimed, SENS-03)
};

// Fixed rule: a field is claimed only when its sensor is enabled AND detected.
inline void rmSensPresence(const RmSensFlags &f, RmSensIn &in)
{
    const bool bmx = (f.bmpOn || f.bmeOn) && f.bmxFound;
    const bool b680 = f.bme680On && f.bme680Found;
    const bool aht = f.aht20On && f.aht20Found;
    const bool sht = f.sht21On && f.sht21Found;
    const bool bmp3 = f.bmp3On && f.bmp3Found;
    in.hasT = bmx || b680 || aht || bmp3;
    in.hasH = (bmx && f.bmxIsBme) || b680 || aht || sht;
    in.hasP = bmx || b680 || bmp3;
    in.hasT2 = sht || f.oneWireOn;
}

inline size_t rmFmtSens(char *out, size_t n, const RmSensIn &in)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, "t=");
    rmfOptFixed(b, in.hasT, in.t, -999.9, 999.9, 1);
    rmfStr(b, " h=");
    rmfOptFixed(b, in.hasH, in.h, 0.0, 999.0, 0);
    rmfStr(b, " p=");
    rmfOptFixed(b, in.hasP, in.p, 0.0, 9999.9, 1);
    rmfStr(b, " t2=");
    rmfOptFixed(b, in.hasT2, in.t2, -999.9, 999.9, 1);
    return rmfEmit(out, n, b);
}

// ---- mh page --------------------------------------------------------------------------------------------

struct RmMhRow
{
    char call[10]; // upper case, NUL-terminated (at most 9 chars)
    uint16_t ageMin;
};

struct RmMhPageIn
{
    uint16_t total;       // entries in the whole list
    uint16_t first;       // list index of rows[0]
    const RmMhRow *rows;  // rows[0] is list index `first`
    uint8_t count;        // rows offered
};

// Length of a NUL-terminated call, counting at most 10 chars (10 = longer than 9; reads <= 10 bytes).
inline uint8_t rmfCallLen(const char *c)
{
    uint8_t i = 0;
    while (c != nullptr && i < 10 && c[i] != '\0')
        i++;
    return i;
}

// One call: a-z folded to upper case, bytes outside A-Z 0-9 `-` become `?`; empty -> `-`; longer than 9
// chars -> `?` (never cut). A caller that must not emit a long call checks rmfCallLen() first.
inline void rmfCall(RmFmtBuf &b, const char *c)
{
    uint8_t len = rmfCallLen(c);
    if (len == 0)
        rmfCh(b, '-');
    else if (len > 9)
        rmfCh(b, '?');
    else
        for (uint8_t i = 0; i < len; i++)
        {
            char ch = c[i];
            if (ch >= 'a' && ch <= 'z')
                ch = (char)(ch - 'a' + 'A');
            if (!((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '-'))
                ch = '?';
            rmfCh(b, ch);
        }
}

// Emits as many of the offered rows as fit; *emitted (optional) reports how many went out. A first row
// that does not fit returns 0 (cannot happen with 9-char calls).
inline size_t rmFmtMhPage(char *out, size_t n, const RmMhPageIn &in, uint8_t *emitted)
{
    if (emitted != nullptr)
        *emitted = 0;
    RmFmtBuf head;
    rmfInit(head);
    rmfU(head, in.total); // total is a plain number, not a count
    size_t lt = head.len;
    RmFmtBuf rows;
    rmfInit(rows);
    uint8_t k = 0;
    bool listed = in.total != 0 && in.first < in.total && in.rows != nullptr && in.count > 0;
    if (listed)
    {
        for (; k < in.count; k++)
        {
            RmFmtBuf row;
            rmfInit(row);
            rmfCh(row, ' ');
            rmfCall(row, in.rows[k].call);
            rmfCh(row, ' ');
            rmfCount(row, in.rows[k].ageMin);
            uint32_t nx = (uint32_t)in.first + k + 1u;
            size_t ln = 1;
            if (nx < in.total)
            {
                RmFmtBuf t;
                rmfInit(t);
                rmfU(t, nx);
                ln = t.len;
            }
            if (!row.ok || lt + 1 + ln + rows.len + row.len > RM_FMT_BODY_MAX)
                break;
            rmfStr(rows, row.s);
        }
        if (k == 0)
        {
            RmFmtBuf bad;
            rmfInit(bad);
            bad.ok = false;
            return rmfEmit(out, n, bad);
        }
    }
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, head.s);
    rmfCh(b, ' ');
    uint32_t nx = (uint32_t)in.first + k;
    if (listed && nx < in.total)
        rmfU(b, nx);
    else if (!listed && in.total != 0 && in.first < in.total)
        rmfU(b, in.first); // no rows offered (count 0 or rows NULL): nothing emitted, list not exhausted
    else
        rmfCh(b, '-');
    rmfStr(b, rows.s);
    size_t r = rmfEmit(out, n, b);
    if (r != 0 && emitted != nullptr)
        *emitted = k;
    return r;
}

// ---- mh direct ------------------------------------------------------------------------------------------

struct RmMhDirectIn
{
    bool gw;
    bool mesh;
    bool hasRssi;
    int rssi; // dBm, clamped -999..999
    bool hasSnr;
    int snr; // dB, clamped -999..999
    bool hasPos;
    double lat, lon; // signed degrees
    bool hasDist;
    float distKm; // clamped 0..9999.9
    bool hasAlt;
    int alt; // metres, clamped -999..99999
    uint8_t ncnt; // n= its neighbour count
    uint8_t ex;   // x= neighbours only it hears
    uint8_t nb;   // h= neighbours it hears
    uint16_t ageMin;
};

inline void rmfOptInt(RmFmtBuf &b, bool has, int v, int32_t lo, int32_t hi)
{
    if (!has)
        rmfCh(b, '-');
    else
        rmfI(b, v, lo, hi);
}

inline size_t rmFmtMhDirect(char *out, size_t n, const RmMhDirectIn &in)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, "d g=");
    rmfCh(b, in.gw ? '1' : '0');
    rmfStr(b, " m=");
    rmfCh(b, in.mesh ? '1' : '0');
    rmfStr(b, " r=");
    rmfOptInt(b, in.hasRssi, in.rssi, -999, 999);
    rmfStr(b, " s=");
    rmfOptInt(b, in.hasSnr, in.snr, -999, 999);
    rmfStr(b, " la=");
    bool posOk = in.hasPos && rmFmtFiniteD(in.lat) && rmFmtFiniteD(in.lon);
    if (posOk)
        rmfFixed(b, rmfClampD(in.lat, -90.0, 90.0), 4);
    else
        rmfCh(b, '-');
    rmfStr(b, " lo=");
    if (posOk)
        rmfFixed(b, rmfClampD(in.lon, -180.0, 180.0), 4);
    else
        rmfCh(b, '-');
    rmfStr(b, " di=");
    rmfOptFixed(b, in.hasDist, in.distKm, 0.0, 9999.9, 1);
    rmfStr(b, " a=");
    rmfOptInt(b, in.hasAlt, in.alt, -999, 99999);
    rmfStr(b, " n=");
    rmfU(b, in.ncnt);
    rmfStr(b, " x=");
    rmfU(b, in.ex);
    rmfStr(b, " h=");
    rmfU(b, in.nb);
    rmfStr(b, " t=");
    rmfCount(b, in.ageMin);
    return rmfEmit(out, n, b);
}

// ---- mh route -------------------------------------------------------------------------------------------

struct RmMhRouteIn
{
    uint8_t hops;
    uint8_t routes;
    bool gw;
    bool hasMesh;
    bool mesh;
    bool hasRc;
    float rcSnr; // dB, clamped -99.9..99.9
    char rcCall[10];
    uint16_t ageMin;
    const char *const *via; // via chain, nearest first
    uint8_t viaCount;
};

inline size_t rmFmtMhRoute(char *out, size_t n, const RmMhRouteIn &in)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, "r h=");
    rmfU(b, in.hops);
    rmfStr(b, " k=");
    rmfU(b, in.routes);
    rmfStr(b, " g=");
    rmfCh(b, in.gw ? '1' : '0');
    rmfStr(b, " m=");
    rmfCh(b, in.hasMesh ? (in.mesh ? '1' : '0') : '-');
    rmfStr(b, " rc=");
    if (in.hasRc && rmFmtFiniteF(in.rcSnr) && in.rcCall[0] != '\0')
    {
        rmfFixed(b, rmfClampD((double)in.rcSnr, -99.9, 99.9), 1);
        rmfCh(b, '@');
        rmfCall(b, in.rcCall); // reads at most 10 bytes: an unterminated array prints `?`
    }
    else
        rmfCh(b, '-');
    rmfStr(b, " t=");
    rmfCount(b, in.ageMin);
    rmfStr(b, " v=");
    uint8_t put = 0;
    for (uint8_t i = 0; in.via != nullptr && i < in.viaCount && in.via[i] != nullptr; i++)
    {
        RmFmtBuf t;
        rmfInit(t);
        if (rmfCallLen(in.via[i]) > 9)
            break; // a call is never cut: the chain ends before it
        if (put > 0)
            rmfCh(t, ',');
        rmfCall(t, in.via[i]);
        if (b.len + t.len > RM_FMT_BODY_MAX)
            break;
        rmfStr(b, t.s);
        put++;
    }
    if (put == 0)
        rmfCh(b, '-');
    return rmfEmit(out, n, b);
}

// ---- txq ------------------------------------------------------------------------------------------------

struct RmTxqIn
{
    uint16_t queued;
    uint16_t cap;
    uint8_t bp; // 0 quiet, 1 qrs, 2 qrt
    uint32_t tx; // frames sent
    uint32_t rt; // retransmissions
    uint32_t dr; // dropped
    uint8_t util; // channel utilisation percent, clamped 0..100
};

// D6: txRingPrioCounts() returns the total in prio[0] and the per-priority counts in prio[1..5].
inline uint16_t rmTxqQueued(const uint8_t prio[6])
{
    return prio[0];
}

inline size_t rmFmtTxq(char *out, size_t n, const RmTxqIn &in)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, "q=");
    rmfU(b, in.queued);
    rmfCh(b, '/');
    rmfU(b, in.cap);
    rmfStr(b, " bp=");
    rmfStr(b, in.bp == 0 ? "quiet" : (in.bp == 1 ? "qrs" : (in.bp == 2 ? "qrt" : "?")));
    rmfStr(b, " tx=");
    rmfCount(b, in.tx);
    rmfStr(b, " rt=");
    rmfCount(b, in.rt);
    rmfStr(b, " dr=");
    rmfCount(b, in.dr);
    rmfStr(b, " u=");
    rmfU(b, in.util > 100 ? 100u : in.util);
    return rmfEmit(out, n, b);
}

// ---- mbox -----------------------------------------------------------------------------------------------

struct RmMboxIn
{
    uint8_t mode; // MsgStoreMode: 0 off, 1 own, 2 list, 3 heard
    uint16_t used;
    uint16_t slots;
    uint32_t bytes; // clamped 999999
    uint16_t aUsed; // a=: mailbox actions in the last hour (msgstoreActionsLastHour())
    uint16_t aCap;  // a=: hourly ceiling (MSGSTORE_ACTIONS_PER_HOUR, 20)
    uint32_t stored, delivered, purgedAck;
    uint32_t droppedStoretime, droppedCap, droppedSlots; // dr= is their sum
    uint32_t blockedBp, notified;
};

inline size_t rmFmtMbox(char *out, size_t n, const RmMboxIn &in)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, "m=");
    rmfStr(b, in.mode == 0 ? "off" : (in.mode == 1 ? "own" : (in.mode == 2 ? "list" : (in.mode == 3 ? "heard" : "?"))));
    rmfStr(b, " u=");
    rmfU(b, in.used);
    rmfCh(b, '/');
    rmfU(b, in.slots);
    rmfStr(b, " b=");
    rmfU(b, in.bytes > 999999u ? 999999u : in.bytes);
    rmfStr(b, " a=");
    rmfU(b, in.aUsed);
    rmfCh(b, '/');
    rmfU(b, in.aCap);
    rmfStr(b, " st=");
    rmfCount(b, in.stored);
    rmfStr(b, " dl=");
    rmfCount(b, in.delivered);
    rmfStr(b, " ak=");
    rmfCount(b, in.purgedAck);
    rmfStr(b, " dr=");
    uint64_t dr = (uint64_t)in.droppedStoretime + in.droppedCap + in.droppedSlots;
    rmfCount(b, dr > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)dr);
    rmfStr(b, " bl=");
    rmfCount(b, in.blockedBp);
    rmfStr(b, " nt=");
    rmfCount(b, in.notified);
    return rmfEmit(out, n, b);
}

// ---- maxhop ---------------------------------------------------------------------------------------------

inline size_t rmFmtMaxhop(char *out, size_t n, int text, int pos)
{
    RmFmtBuf b;
    rmfInit(b);
    rmfStr(b, "t=");
    rmfI(b, text, 0, 99);
    rmfStr(b, " p=");
    rmfI(b, pos, 0, 99);
    return rmfEmit(out, n, b);
}
