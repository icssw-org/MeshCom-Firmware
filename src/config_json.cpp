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

#include <meshcom_settings.h> // the one settings struct, both platforms (D1-04)

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
