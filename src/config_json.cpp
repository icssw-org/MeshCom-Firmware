/**
 * CS-03: config export/import as one JSON object. Format, canonical CRC form
 * and the list of deliberately excluded fields are documented in config_json.h
 * -- read that first.
 *
 * The one thing worth repeating here: export and import share ONE field table
 * (CFG_FIELD_LIST below). There is no second list anywhere; adding a field is
 * one line and both directions follow.
 */
#include "config_json.h"

#include <Arduino.h>
#include <configuration.h>
#include <crc32_util.h>
#include <maxhop.h>

#ifdef ESP32
    #include <esp32/esp32_flash.h>
#else
    #include <nrf52/WisBlock-API.h>
#endif

#include <ArduinoJson.h>

#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* BOARD_HARDWARE lives in loop_functions.cpp; declared here instead of pulling
 * in loop_functions_extern.h, which drags the whole firmware in and does not
 * build natively. Informational field only. */
extern int BOARD_HARDWARE;

/* The variant's configuration.h supplies these; the native test build has no
 * variant. The fallback is deliberately wide -- it must never be narrower than
 * any real board, or a legitimate export would be refused. */
#ifndef TX_POWER_MIN
    #define TX_POWER_MIN (-20)
#endif
#ifndef TX_POWER_MAX
    #define TX_POWER_MAX 30
#endif

/* Same sentinel settings_sanitize.cpp uses: "no TX power stored yet". It is
 * outside TX_POWER_MIN..MAX on some boards (RAK4631: 2..22) and must stay
 * importable, otherwise a factory-fresh node cannot restore its own export. */
#define CFG_POWER_NOT_SET (-20)

// ---------------------------------------------------------------------------
// Field table -- the single source of truth for both directions
// ---------------------------------------------------------------------------

enum CfgType
{
    CFG_STR = 0,  /* char[]   */
    CFG_CHR,      /* single char */
    CFG_INT,      /* int      */
    CFG_U32,      /* uint32_t / unsigned long */
    CFG_BOOL,     /* bool     */
    CFG_FLT,      /* float    */
    CFG_DBL       /* double   */
};

struct CfgField
{
    const char *key;      /* NVS key name (esp32_flash.cpp save_settings())   */
    uint8_t     type;
    void       *ptr;
    uint16_t    size;     /* buffer size, CFG_STR only                        */
    double      lo;       /* range check; lo > hi means "not range checked"   */
    double      hi;
    double      esc;      /* value always accepted regardless of lo/hi ...    */
    uint8_t     has_esc;  /* ... if has_esc                                   */
};

#define CFG_NORANGE   1.0, 0.0
#define CFG_NOESC     0.0, 0
#define CFG_ESC(v)    (double)(v), 1

/* X(key, type, member, lo, hi, esc, has_esc)
 *
 * Order matters: it is the order of the JSON members AND the order of the
 * canonical CRC form (config_json.h). Append new fields at the end of the
 * platform block they belong to; reordering invalidates every file in the
 * field. */
#define CFG_FIELD_LIST(X)                                                                     \
    X("node_call",     CFG_STR,  node_call,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_short",    CFG_STR,  node_short,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_symid",    CFG_CHR,  node_symid,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_symcd",    CFG_CHR,  node_symcd,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_lat",      CFG_DBL,  node_lat,              -90.0, 90.0,        CFG_NOESC)        \
    X("node_lon",      CFG_DBL,  node_lon,              -180.0, 180.0,      CFG_NOESC)        \
    X("node_alt",      CFG_INT,  node_alt,              -1000.0, 20000.0,   CFG_NOESC)        \
    X("node_lat_c",    CFG_CHR,  node_lat_c,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_lon_c",    CFG_CHR,  node_lon_c,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_ssid",     CFG_STR,  node_ossid,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_pwd",      CFG_STR,  node_opwd,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_honly",    CFG_INT,  node_hamnet_only,      0.0, 1.0,           CFG_NOESC)        \
    X("node_sset",     CFG_INT,  node_sset,             0.0, 65535.0,       CFG_NOESC)        \
    X("node_maxv",     CFG_FLT,  node_maxv,             0.0, 20.0,          CFG_NOESC)        \
    X("node_extern",   CFG_STR,  node_extern,           CFG_NORANGE,        CFG_NOESC)        \
    X("max_hop_text",  CFG_INT,  max_hop_text,          (double)MAXHOP_TEXT_MIN, (double)MAXHOP_TEXT_MAX, CFG_NOESC) \
    X("node_power",    CFG_INT,  node_power,            (double)TX_POWER_MIN, (double)TX_POWER_MAX, CFG_ESC(CFG_POWER_NOT_SET)) \
    X("node_freq",     CFG_FLT,  node_freq,             0.0, 1.0e9,         CFG_NOESC)        \
    X("node_bw",       CFG_FLT,  node_bw,               0.0, 500.0,         CFG_NOESC)        \
    X("node_sf",       CFG_INT,  node_sf,               0.0, 12.0,          CFG_NOESC)        \
    X("node_cr",       CFG_INT,  node_cr,               0.0, 8.0,           CFG_NOESC)        \
    X("node_atxt",     CFG_STR,  node_atxt,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_sset2",    CFG_INT,  node_sset2,            0.0, 65535.0,       CFG_NOESC)        \
    X("node_owgpio",   CFG_INT,  node_owgpio,           0.0, 99.0,          CFG_NOESC)        \
    X("node_utcof",    CFG_FLT,  node_utcoff,           -12.0, 14.0,        CFG_NOESC)        \
    X("node_mcp17",    CFG_INT,  node_mcp17io,          CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp17o",   CFG_INT,  node_mcp17out,         CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp17i",   CFG_INT,  node_mcp17in,          CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp170",   CFG_STR,  node_mcp17t[0],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp171",   CFG_STR,  node_mcp17t[1],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp172",   CFG_STR,  node_mcp17t[2],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp173",   CFG_STR,  node_mcp17t[3],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp174",   CFG_STR,  node_mcp17t[4],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp175",   CFG_STR,  node_mcp17t[5],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp176",   CFG_STR,  node_mcp17t[6],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp177",   CFG_STR,  node_mcp17t[7],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp178",   CFG_STR,  node_mcp17t[8],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp179",   CFG_STR,  node_mcp17t[9],        CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp1710",  CFG_STR,  node_mcp17t[10],       CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp1711",  CFG_STR,  node_mcp17t[11],       CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp1712",  CFG_STR,  node_mcp17t[12],       CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp1713",  CFG_STR,  node_mcp17t[13],       CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp1714",  CFG_STR,  node_mcp17t[14],       CFG_NORANGE,        CFG_NOESC)        \
    X("node_mcp1715",  CFG_STR,  node_mcp17t[15],       CFG_NORANGE,        CFG_NOESC)        \
    X("node_gcb0",     CFG_INT,  node_gcb[0],           CFG_NORANGE,        CFG_NOESC)        \
    X("node_gcb1",     CFG_INT,  node_gcb[1],           CFG_NORANGE,        CFG_NOESC)        \
    X("node_gcb2",     CFG_INT,  node_gcb[2],           CFG_NORANGE,        CFG_NOESC)        \
    X("node_gcb3",     CFG_INT,  node_gcb[3],           CFG_NORANGE,        CFG_NOESC)        \
    X("node_gcb4",     CFG_INT,  node_gcb[4],           CFG_NORANGE,        CFG_NOESC)        \
    X("node_gcb5",     CFG_INT,  node_gcb[5],           CFG_NORANGE,        CFG_NOESC)        \
    X("node_ctry",     CFG_INT,  node_country,          0.0, 20.0,          CFG_NOESC)        \
    X("node_track",    CFG_FLT,  node_track_freq,       0.0, 1.0e9,         CFG_NOESC)        \
    X("node_pream",    CFG_INT,  node_preamplebits,     0.0, 1024.0,        CFG_NOESC)        \
    X("node_ss_rx",    CFG_INT,  node_ss_rx_pin,        0.0, 99.0,          CFG_NOESC)        \
    X("node_ss_tx",    CFG_INT,  node_ss_tx_pin,        0.0, 99.0,          CFG_NOESC)        \
    X("node_ss_bd",    CFG_INT,  node_ss_baud,          0.0, 1000000.0,     CFG_NOESC)        \
    X("node_postime",  CFG_INT,  node_postime,          0.0, 1440.0,        CFG_NOESC)        \
    X("node_passwd",   CFG_STR,  node_passwd,           CFG_NORANGE,        CFG_NOESC)        \
    X("node_sset3",    CFG_INT,  node_sset3,            0.0, 65535.0,       CFG_NOESC)        \
    X("bt_code",       CFG_INT,  bt_code,               0.0, 999999.0,      CFG_NOESC)        \
    X("node_bpin",     CFG_INT,  node_button_pin,       0.0, 99.0,          CFG_NOESC)        \
    X("node_ownip",    CFG_STR,  node_ownip,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_owngw",    CFG_STR,  node_owngw,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_ownms",    CFG_STR,  node_ownms,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_name",     CFG_STR,  node_name,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_webpwd",   CFG_STR,  node_webpwd,           CFG_NORANGE,        CFG_NOESC)        \
    X("node_lssid",    CFG_STR,  node_ssid,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_lpwd",     CFG_STR,  node_pwd,              CFG_NORANGE,        CFG_NOESC)        \
    X("node_apin",     CFG_INT,  node_analog_pin,       0.0, 99.0,          CFG_NOESC)        \
    X("node_afakt",    CFG_FLT,  node_analog_faktor,    CFG_NORANGE,        CFG_NOESC)        \
    X("node_parm",     CFG_STR,  node_parm,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_unit",     CFG_STR,  node_unit,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_format",   CFG_STR,  node_format,           CFG_NORANGE,        CFG_NOESC)        \
    X("node_eqns",     CFG_STR,  node_eqns,             CFG_NORANGE,        CFG_NOESC)        \
    X("node_values",   CFG_STR,  node_values,           CFG_NORANGE,        CFG_NOESC)        \
    X("node_ptime",    CFG_INT,  node_parm_time,        0.0, 1440.0,        CFG_NOESC)        \
    X("node_wifip",    CFG_INT,  node_wifi_power,       0.0, 100.0,         CFG_NOESC)        \
    X("node_ucall",    CFG_STR,  node_lora_call,        CFG_NORANGE,        CFG_NOESC)        \
    X("node_aak",      CFG_FLT,  node_analog_alpha,     CFG_NORANGE,        CFG_NOESC)        \
    X("node_aslo",     CFG_FLT,  node_analog_slope,     CFG_NORANGE,        CFG_NOESC)        \
    X("node_aoff",     CFG_FLT,  node_analog_offset,    CFG_NORANGE,        CFG_NOESC)        \
    X("node_atten",    CFG_FLT,  node_analog_atten,     CFG_NORANGE,        CFG_NOESC)        \
    X("node_gwsrv",    CFG_STR,  node_gwsrv,            CFG_NORANGE,        CFG_NOESC)        \
    X("node_tmpiof",   CFG_FLT,  node_tempi_off,        -50.0, 50.0,        CFG_NOESC)        \
    X("node_tmpoof",   CFG_FLT,  node_tempo_off,        -50.0, 50.0,        CFG_NOESC)        \
    X("node_shunt",    CFG_FLT,  node_shunt,            0.0, 10.0,          CFG_NOESC)        \
    X("node_imax",     CFG_FLT,  node_imax,             0.0, 1000.0,        CFG_NOESC)        \
    X("node_isamp",    CFG_INT,  node_isamp,            0.0, 15.0,          CFG_NOESC)        \
    X("node_owndns",   CFG_STR,  node_owndns,           CFG_NORANGE,        CFG_NOESC)        \
    X("node_contrast", CFG_INT,  node_contrast,         0.0, 255.0,         CFG_NOESC)        \
    X("node_ownntp",   CFG_STR,  node_ownntp,           CFG_NORANGE,        CFG_NOESC)        \
    X("node_gpsbaud",  CFG_U32,  node_gpsbaud,          1200.0, 921600.0,   CFG_NOESC)        \
    X("node_netmode",  CFG_INT,  node_netmode,          0.0, 1.0,           CFG_NOESC)        \
    X("node_gpsdebug", CFG_INT,  node_gpsdebug,         0.0, 2.0,           CFG_NOESC)        \
    X("node_relay",    CFG_INT,  node_relay,            0.0, 65535.0,       CFG_NOESC)        \
    X("node_via",      CFG_STR,  node_via,              CFG_NORANGE,        CFG_NOESC)        \
    X("node_sset4",    CFG_INT,  node_sset4,            0.0, 65535.0,       CFG_NOESC)        \
    X("node_aprsmc",   CFG_STR,  node_aprsmc,           CFG_NORANGE,        CFG_NOESC)        \
    X("node_pingtime", CFG_INT,  node_pingtime,         0.0, 86400.0,       CFG_NOESC)        \
    X("node_pingcall", CFG_STR,  node_pingcall,         CFG_NORANGE,        CFG_NOESC)        \
    X("node_pingmax",  CFG_INT,  node_pingmax,          0.0, 100.0,         CFG_NOESC)        \
    CFG_FIELD_LIST_PLATFORM(X)

/* Platform-only persisted fields.
 *
 * ESP32: node_disp_rot, the spectrum-scan window and node_analog_batt_faktor
 * have NVS keys; on the nRF52 the same members exist but sit below the
 * "nicht im Flash" line of the struct and are not part of the stored config.
 *
 * nRF52: send_repeat_time and auto_join exist only there and have no NVS key,
 * so they get their member name as key. */
#ifdef ESP32
    #define CFG_FIELD_LIST_PLATFORM(X)                                                        \
        X("node_disrot",  CFG_INT, node_disp_rot,          0.0, 270.0,      CFG_NOESC)        \
        X("node_spstart", CFG_FLT, node_specstart,         0.0, 1.0e9,      CFG_NOESC)        \
        X("node_spend",   CFG_FLT, node_specend,           0.0, 1.0e9,      CFG_NOESC)        \
        X("node_spstep",  CFG_FLT, node_specstep,          0.0, 1000.0,     CFG_NOESC)        \
        X("node_spsamp",  CFG_INT, node_specsamples,       0.0, 65535.0,    CFG_NOESC)        \
        X("node_bfakt",   CFG_FLT, node_analog_batt_faktor, CFG_NORANGE,    CFG_NOESC)
#else
    #define CFG_FIELD_LIST_PLATFORM(X)                                                        \
        X("send_repeat_time", CFG_U32,  send_repeat_time,  CFG_NORANGE,     CFG_NOESC)        \
        X("auto_join",        CFG_BOOL, auto_join,         CFG_NORANGE,     CFG_NOESC)
#endif

/* Deliberately NOT exported although they have NVS keys (gate decision,
 * 2026-08-30, from the first live round trip on DK5EN-93): node_msgid /
 * node_ackid are the running message-id counters -- restoring a backup would
 * rewind them and make fresh messages collide with the dedup ring of every
 * neighbour; node_temp/hum/press/temp2/gas/co2 are the last sensor readings,
 * not configuration, and made two exports of an unchanged node differ. */

#define CFG_MEMBER_SIZE(m) ((uint16_t)sizeof(((s_meshcom_settings *)0)->m))

/* X is variadic so that CFG_NORANGE / CFG_NOESC / CFG_ESC() are expanded
 * BEFORE the row macro counts its arguments -- a non-variadic X would see
 * "CFG_NORANGE" as one argument and fail to match its parameter list. */
#define CFG_ROW(k, t, m, lo, hi, esc, has_esc) \
    { k, (uint8_t)(t), (void *)&meshcom_settings.m, CFG_MEMBER_SIZE(m), lo, hi, esc, (uint8_t)(has_esc) },

static const CfgField cfg_fields[] = {
#define X(...) CFG_ROW(__VA_ARGS__)
    CFG_FIELD_LIST(X)
#undef X
};

#undef CFG_ROW

static const size_t CFG_FIELD_COUNT = sizeof(cfg_fields) / sizeof(cfg_fields[0]);

/* CFG_U32 members are read and written as a plain 32-bit word (the two
 * platforms spell the type differently: unsigned long on the ESP32,
 * unsigned int / uint32_t on the nRF52). If that ever stops being 4 bytes
 * wide, the memcpy below would silently corrupt the neighbouring field. */
static_assert(sizeof(meshcom_settings.node_gpsbaud) == 4, "CFG_U32 assumes a 32-bit member");

// ---------------------------------------------------------------------------
// Canonical value formatting (config_json.h, "CANONICAL FORM")
// ---------------------------------------------------------------------------

#define CFG_VALBUF 96

static void fmt_int(int v, char *buf, size_t cap) { snprintf(buf, cap, "%d", v); }
static void fmt_u32(uint32_t v, char *buf, size_t cap) { snprintf(buf, cap, "%lu", (unsigned long)v); }
static void fmt_bool(bool v, char *buf, size_t cap) { snprintf(buf, cap, "%d", v ? 1 : 0); }

/* Shortest %g precision that strtod() turns back into the very same value --
 * 6..9 significant digits for a float, 15..17 for a double. Both bounds are
 * the textbook ones: 9 and 17 digits always round-trip, and starting lower
 * keeps the file readable (4.24 instead of 4.23999977).
 *
 * Fixed-point ("%.6f") was the first cut and is wrong twice over: it drops
 * everything below 1e-6, and its trailing zeros are exactly what makes
 * ArduinoJson's number parser miss by one ulp ("0.002000" parses to
 * 0.0019999998, "0.002" does not). Numbers therefore travel as JSON strings
 * and come back through strtod(); see the header. */
static void fmt_real(double v, char *buf, size_t cap, bool is_float)
{
    const int lo = is_float ? 6 : 15;
    const int hi = is_float ? 9 : 17;

    for (int prec = lo; prec <= hi; prec++)
    {
        snprintf(buf, cap, "%.*g", prec, v);

        double back = strtod(buf, NULL);

        if (is_float ? ((float)back == (float)v) : (back == v))
            return;
    }
    /* hi digits always round-trip; the loop only exits early for readability */
}

static void fmt_flt(double v, char *buf, size_t cap) { fmt_real(v, buf, cap, true); }
static void fmt_dbl(double v, char *buf, size_t cap) { fmt_real(v, buf, cap, false); }

/* The field's value as it currently stands in meshcom_settings. */
static void cfg_value_from_member(const CfgField &f, char *buf, size_t cap)
{
    switch (f.type)
    {
    case CFG_STR:
    {
        const char *s = (const char *)f.ptr;
        size_t n = 0;
        /* the member may have lost its terminator in a corrupt image */
        while (n < (size_t)f.size && n + 1 < cap && s[n] != '\0')
        {
            buf[n] = s[n];
            n++;
        }
        buf[n] = '\0';
        break;
    }
    case CFG_CHR:
    {
        char c = *(const char *)f.ptr;
        if (c == '\0' || cap < 2)
        {
            buf[0] = '\0';
        }
        else
        {
            buf[0] = c;
            buf[1] = '\0';
        }
        break;
    }
    case CFG_INT:  fmt_int(*(const int *)f.ptr, buf, cap); break;
    case CFG_U32:
    {
        uint32_t u;
        memcpy(&u, f.ptr, sizeof(u));
        fmt_u32(u, buf, cap);
        break;
    }
    case CFG_BOOL: fmt_bool(*(const bool *)f.ptr, buf, cap); break;
    case CFG_FLT:  fmt_flt((double)(*(const float *)f.ptr), buf, cap); break;
    case CFG_DBL:  fmt_dbl(*(const double *)f.ptr, buf, cap); break;
    default:       buf[0] = '\0'; break;
    }
}

/* A scalar JSON member as a number.
 *
 * Our own export writes every value as a JSON STRING, and those go through
 * strtod() -- exact, and independent of the JSON library's own decimal
 * parser, which is off by an ulp on plenty of inputs. A hand-written file
 * that uses real JSON numbers still works; it just inherits that parser's
 * precision. */
static double cfg_num_from_json(JsonVariantConst v)
{
    if (v.is<const char *>())
    {
        const char *s = v.as<const char *>();
        return strtod(s ? s : "", NULL);
    }

    if (v.is<bool>())
        return v.as<bool>() ? 1.0 : 0.0;

    return v.as<double>();
}

/* The same value, but taken from a parsed JSON member. Must produce exactly
 * the same text for the same value -- that is what makes the CRC survive a
 * reformatting editor. */
static void cfg_value_from_json(const CfgField &f, JsonVariantConst v, char *buf, size_t cap)
{
    switch (f.type)
    {
    case CFG_STR:
    case CFG_CHR:
    {
        const char *s = v.as<const char *>();
        snprintf(buf, cap, "%s", s ? s : "");
        break;
    }
    case CFG_INT:  fmt_int((int)llround(cfg_num_from_json(v)), buf, cap); break;
    case CFG_U32:  fmt_u32((uint32_t)llround(cfg_num_from_json(v)), buf, cap); break;
    case CFG_BOOL: fmt_bool(cfg_num_from_json(v) != 0.0, buf, cap); break;
    case CFG_FLT:  fmt_flt((double)(float)cfg_num_from_json(v), buf, cap); break;
    case CFG_DBL:  fmt_dbl(cfg_num_from_json(v), buf, cap); break;
    default:       buf[0] = '\0'; break;
    }
}

/* Streaming CRC over the canonical text; see config_json.h. */
struct CanonCrc
{
    uint32_t crc;
};

static void canon_begin(CanonCrc &c)
{
    c.crc = 0xFFFFFFFFUL;
    c.crc = crc32_update(c.crc, "MC-CFG-1\n", 9);
}

static void canon_kv(CanonCrc &c, const char *key, const char *val)
{
    c.crc = crc32_update(c.crc, key, strlen(key));
    c.crc = crc32_update(c.crc, "=", 1);
    c.crc = crc32_update(c.crc, val, strlen(val));
    c.crc = crc32_update(c.crc, "\n", 1);
}

static uint32_t canon_end(CanonCrc &c)
{
    return c.crc ^ 0xFFFFFFFFUL;
}

/* layout / fw / hw, identical on both sides. */
static void canon_head(CanonCrc &c, int layout, const char *fw, int hw)
{
    char buf[CFG_VALBUF];

    canon_begin(c);
    fmt_int(layout, buf, sizeof(buf));
    canon_kv(c, "layout", buf);
    canon_kv(c, "fw", fw ? fw : "");
    fmt_int(hw, buf, sizeof(buf));
    canon_kv(c, "hw", buf);
}

static const char *cfg_fw_string(void)
{
    static char fw[24];
    snprintf(fw, sizeof(fw), "%s%s", SOURCE_VERSION, SOURCE_VERSION_SUB);
    return fw;
}

// ---------------------------------------------------------------------------
// Export
// ---------------------------------------------------------------------------

/* Appends to out/len, keeping the buffer NUL-terminated. Returns false once it
 * no longer fits -- the caller then aborts the whole export. */
static bool out_add(char *out, size_t cap, size_t &len, const char *s)
{
    size_t n = strlen(s);
    if (len + n + 1 > cap)
        return false;
    memcpy(out + len, s, n);
    len += n;
    out[len] = '\0';
    return true;
}

/* JSON string literal including the quotes. Escapes what RFC 8259 requires;
 * config values are operator text, so control characters are the only exotic
 * case and they go out as \u00XX. */
static bool out_add_jsonstr(char *out, size_t cap, size_t &len, const char *s)
{
    if (!out_add(out, cap, len, "\""))
        return false;

    for (const char *p = s; *p; p++)
    {
        char esc[8];
        const char *chunk = esc;
        unsigned char c = (unsigned char)*p;

        if (c == '"')       chunk = "\\\"";
        else if (c == '\\') chunk = "\\\\";
        else if (c == '\n') chunk = "\\n";
        else if (c == '\r') chunk = "\\r";
        else if (c == '\t') chunk = "\\t";
        else if (c < 0x20)  { snprintf(esc, sizeof(esc), "\\u%04x", c); }
        else                { esc[0] = (char)c; esc[1] = '\0'; }

        if (!out_add(out, cap, len, chunk))
            return false;
    }

    return out_add(out, cap, len, "\"");
}

size_t configExportJson(char *out, size_t cap)
{
    if (out == NULL || cap == 0)
        return 0;

    out[0] = '\0';

    size_t len = 0;
    char   val[CFG_VALBUF];
    char   num[CFG_VALBUF];
    bool   ok = true;

    const int   layout = FLASH_STRUCT_VERSION;
    const char *fw     = cfg_fw_string();
    const int   hw     = BOARD_HARDWARE;

    CanonCrc canon;
    canon_head(canon, layout, fw, hw);

    ok = ok && out_add(out, cap, len, "{\"meshcom_config\":{\"layout\":");
    fmt_int(layout, num, sizeof(num));
    ok = ok && out_add(out, cap, len, num);
    ok = ok && out_add(out, cap, len, ",\"fw\":");
    ok = ok && out_add_jsonstr(out, cap, len, fw);
    ok = ok && out_add(out, cap, len, ",\"hw\":");
    fmt_int(hw, num, sizeof(num));
    ok = ok && out_add(out, cap, len, num);
    ok = ok && out_add(out, cap, len, ",\"settings\":{");

    for (size_t i = 0; ok && i < CFG_FIELD_COUNT; i++)
    {
        const CfgField &f = cfg_fields[i];

        cfg_value_from_member(f, val, sizeof(val));
        canon_kv(canon, f.key, val);

        if (i > 0)
            ok = ok && out_add(out, cap, len, ",");

        ok = ok && out_add_jsonstr(out, cap, len, f.key);
        ok = ok && out_add(out, cap, len, ":");

        /* Every value goes out as a JSON string, numbers included: `val` is
         * already the exact canonical text, and quoting it means the importer
         * reads it back with strtod() instead of through the JSON library's
         * decimal parser. That is what makes the round trip bit-exact. */
        ok = ok && out_add_jsonstr(out, cap, len, val);
    }

    const uint32_t crc = canon_end(canon);

    ok = ok && out_add(out, cap, len, "},\"crc32\":\"");
    snprintf(num, sizeof(num), "%08x", (unsigned int)crc);
    ok = ok && out_add(out, cap, len, num);
    ok = ok && out_add(out, cap, len, "\"}}");

    if (!ok)
    {
        out[0] = '\0';
        Serial.printf("[CONFIG];export;len;0;crc;00000000\n");
        return 0;
    }

    Serial.printf("[CONFIG];export;len;%u;crc;%08x\n", (unsigned int)len, (unsigned int)crc);

    return len;
}

// ---------------------------------------------------------------------------
// Import
// ---------------------------------------------------------------------------

static void cfg_err(char *err, size_t errcap, const char *fmt, ...)
{
    if (err == NULL || errcap == 0)
        return;

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(err, errcap, fmt, ap);
    va_end(ap);
}

static const CfgField *cfg_find(const char *key)
{
    for (size_t i = 0; i < CFG_FIELD_COUNT; i++)
    {
        if (strcmp(cfg_fields[i].key, key) == 0)
            return &cfg_fields[i];
    }
    return NULL;
}

/* Type check only -- the value must be usable at all before it can be
 * canonicalised. Strings are what our own export writes; a real JSON number
 * or boolean from a hand-written file is accepted too. */
static bool cfg_type_ok(const CfgField &f, JsonVariantConst v)
{
    if (f.type == CFG_STR || f.type == CFG_CHR)
        return v.is<const char *>();

    return v.is<const char *>() || v.is<double>() || v.is<int>() || v.is<bool>();
}

int configImportJson(const char *in, size_t len, char *err, size_t errcap)
{
    int rc      = CFG_IMP_OK;
    int applied = 0;
    int unknown = 0;

    if (in == NULL || len == 0)
    {
        cfg_err(err, errcap, "empty upload");
        rc = CFG_IMP_EARG;
    }
    else if (len > CONFIG_JSON_MAX)
    {
        cfg_err(err, errcap, "too large (%u > %u bytes)", (unsigned int)len, (unsigned int)CONFIG_JSON_MAX);
        rc = CFG_IMP_EARG;
    }

    JsonDocument doc;

    if (rc == CFG_IMP_OK)
    {
        DeserializationError jerr = deserializeJson(doc, in, len);
        if (jerr)
        {
            cfg_err(err, errcap, "not valid JSON (%s)", jerr.c_str());
            rc = CFG_IMP_EPARSE;
        }
    }

    JsonObjectConst root;
    JsonObjectConst set;

    if (rc == CFG_IMP_OK)
    {
        root = doc["meshcom_config"].as<JsonObjectConst>();
        set  = root["settings"].as<JsonObjectConst>();

        if (root.isNull() || set.isNull() || !root["crc32"].is<const char *>())
        {
            cfg_err(err, errcap, "not a meshcom config file");
            rc = CFG_IMP_ESHAPE;
        }
    }

    /* 1. layout. A file from another settings-layout generation is refused
     *    before anything else is looked at. */
    if (rc == CFG_IMP_OK)
    {
        if (!root["layout"].is<int>() || root["layout"].as<int>() != FLASH_STRUCT_VERSION)
        {
            cfg_err(err, errcap, "layout %d, this firmware needs %d",
                    root["layout"].is<int>() ? root["layout"].as<int>() : -1,
                    (int)FLASH_STRUCT_VERSION);
            rc = CFG_IMP_ELAYOUT;
        }
    }

    /* 2. every known value is type- and range-checked. Before the CRC on
     *    purpose: a hand-edited file then reports the value that is wrong
     *    instead of the far less useful "checksum mismatch". Nothing is
     *    written either way. */
    if (rc == CFG_IMP_OK)
    {
        for (JsonPairConst kv : set)
        {
            const CfgField *f = cfg_find(kv.key().c_str());
            if (f == NULL)
            {
                unknown++;
                continue;
            }

            if (!cfg_type_ok(*f, kv.value()))
            {
                cfg_err(err, errcap, "%s: wrong type", f->key);
                rc = CFG_IMP_EVALUE;
                break;
            }

            if (f->type == CFG_STR)
            {
                const char *s = kv.value().as<const char *>();
                if (strlen(s) > (size_t)(f->size - 1))
                {
                    cfg_err(err, errcap, "%s: %u chars, max %u", f->key,
                            (unsigned int)strlen(s), (unsigned int)(f->size - 1));
                    rc = CFG_IMP_EVALUE;
                    break;
                }
            }
            else if (f->type == CFG_CHR)
            {
                const char *s = kv.value().as<const char *>();
                if (strlen(s) > 1)
                {
                    cfg_err(err, errcap, "%s: single character expected", f->key);
                    rc = CFG_IMP_EVALUE;
                    break;
                }
            }
            else
            {
                double d = cfg_num_from_json(kv.value());

                /* NaN/inf compare false against every bound, so they would
                 * slip through the range test below unnoticed. */
                if (!isfinite(d))
                {
                    cfg_err(err, errcap, "%s: not a finite number", f->key);
                    rc = CFG_IMP_EVALUE;
                    break;
                }

                bool escaped = (f->has_esc != 0) && (d == f->esc);

                if (f->lo <= f->hi && !escaped && (d < f->lo || d > f->hi))
                {
                    cfg_err(err, errcap, "%s: %g out of range %g..%g", f->key, d, f->lo, f->hi);
                    rc = CFG_IMP_EVALUE;
                    break;
                }
            }
        }
    }

    /* 3. the CRC over the canonical form. */
    if (rc == CFG_IMP_OK)
    {
        CanonCrc canon;
        char     val[CFG_VALBUF];

        canon_head(canon, root["layout"].as<int>(), root["fw"].as<const char *>(), root["hw"].as<int>());

        for (size_t i = 0; i < CFG_FIELD_COUNT; i++)
        {
            const CfgField &f = cfg_fields[i];
            JsonVariantConst v = set[f.key];

            if (v.isNull())
                continue;   /* missing key: keeps the node's current value */

            cfg_value_from_json(f, v, val, sizeof(val));
            canon_kv(canon, f.key, val);
        }

        char have[16];
        snprintf(have, sizeof(have), "%08x", (unsigned int)canon_end(canon));

        const char *want = root["crc32"].as<const char *>();

        if (want == NULL || strcmp(have, want) != 0)
        {
            cfg_err(err, errcap, "crc32 %s, computed %s", want ? want : "(none)", have);
            rc = CFG_IMP_ECRC;
        }
    }

    /* 4. everything passed -- now, and only now, write. */
    if (rc == CFG_IMP_OK)
    {
        for (size_t i = 0; i < CFG_FIELD_COUNT; i++)
        {
            const CfgField &f = cfg_fields[i];
            JsonVariantConst v = set[f.key];

            if (v.isNull())
                continue;

            switch (f.type)
            {
            case CFG_STR:
            {
                char *dst = (char *)f.ptr;
                memset(dst, 0, f.size);
                snprintf(dst, f.size, "%s", v.as<const char *>());
                break;
            }
            case CFG_CHR:
            {
                const char *s = v.as<const char *>();
                *(char *)f.ptr = (s && s[0]) ? s[0] : '\0';
                break;
            }
            case CFG_INT:  *(int *)f.ptr = (int)llround(cfg_num_from_json(v)); break;
            case CFG_U32:
            {
                uint32_t u = (uint32_t)llround(cfg_num_from_json(v));
                memcpy(f.ptr, &u, sizeof(u));
                break;
            }
            case CFG_BOOL: *(bool *)f.ptr = (cfg_num_from_json(v) != 0.0); break;
            case CFG_FLT:  *(float *)f.ptr = (float)cfg_num_from_json(v); break;
            case CFG_DBL:  *(double *)f.ptr = cfg_num_from_json(v); break;
            default: break;
            }

            applied++;
        }

        cfg_err(err, errcap, "%d fields applied, %d unknown key(s) ignored", applied, unknown);
    }

    Serial.printf("[CONFIG];import;rc;%d;fields;%d;unknown;%d\n", rc, applied, unknown);

    return rc;
}
