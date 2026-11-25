#ifndef _EXT_REG_H_
#define _EXT_REG_H_

// EXTUDP register query (read-only): {"type":"reg","q":"SN"} asks for one of
// the BLE config registers (TM, W, IO, I, ... CONFFIN), {"q":"all"} for all of
// them. The registers are built by the existing BLE builders
// (sendBleJsonRegister() in command_functions.cpp); this header holds the
// parts that need no hardware and are tested on the host
// (host-side):
//
//   - the table of valid register names and the command that makes the
//     firmware build each one,
//   - the secret filter (a denylist by key name) that every register frame
//     passes before it may leave on EXTUDP,
//   - the reply/error framing, the 1-per-second request limiter and the state
//     of a running query / "all" run (ExtRegRun).
//
// No ArduinoJson, no Arduino: plain C++ so the native env links nothing.
//
// Reply shape: the register JSON as the phone gets it, minus secret members,
// with "type":"reg" as the first member:
//   {"type":"reg","TYP":"SN","GW":false,...}
// Errors: {"type":"reg","err":"unknown"}  ("unavailable": this build or state
// has no such register). End of an "all" run: {"type":"reg","q":"all","done":true}.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// BLE_JSON_PAYLOAD_MAX (configuration_global.h) is 244; command_functions.cpp
// / extudp_functions.cpp static_assert that it does not exceed this.
#define EXT_REG_PAYLOAD_MAX 244
// "type":"reg", (13 bytes) + register JSON + NUL, with slack
#define EXT_REG_REPLY_MAX (EXT_REG_PAYLOAD_MAX + 20)
// longest register name (CONFFIN) + NUL
#define EXT_REG_NAME_MAX 8

struct ExtRegDef
{
    const char *name; // value of TYP in the frame, and the "q" the peer sends
    const char *cmd;  // console command that makes the firmware build the frame
};

// One entry per register frame the BLE builders can emit. Several names share
// a command (--info emits I, IS1, IS2); the sink keeps only the frame whose
// TYP equals the requested name.
static const ExtRegDef EXT_REG_TABLE[] = {
    {"TM", "--tel"},       {"W", "--weather"},    {"IO", "--io"},       {"I", "--info"},
    {"IS1", "--info"},     {"IS2", "--info"},     {"G", "--pos"},       {"SE", "--seset"},
    {"S1", "--seset"},     {"SW", "--wifiset"},   {"S2", "--wifiset"},  {"SN", "--nodeset"},
    {"SN1", "--nodeset"},  {"AN", "--analogset"}, {"SA", "--aprsset"},  {"CONFFIN", "--conffin"},
};
#define EXT_REG_COUNT ((int)(sizeof(EXT_REG_TABLE) / sizeof(EXT_REG_TABLE[0])))

#define EXT_REG_UNKNOWN (-1)
#define EXT_REG_ALL (-2)

static inline char extRegUpper(char c)
{
    return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

static inline bool extRegEqNoCase(const char *a, size_t alen, const char *b)
{
    size_t blen = strlen(b);
    if (alen != blen)
        return false;
    for (size_t i = 0; i < alen; i++)
        if (extRegUpper(a[i]) != extRegUpper(b[i]))
            return false;
    return true;
}

// q -> table index, EXT_REG_ALL for "all", EXT_REG_UNKNOWN otherwise (also for
// nullptr / empty / over-long input). Case-insensitive, exact, no prefixes.
static inline int extRegLookup(const char *q)
{
    if (q == nullptr)
        return EXT_REG_UNKNOWN;
    size_t n = strnlen(q, EXT_REG_NAME_MAX + 2);
    if (n == 0 || n > EXT_REG_NAME_MAX)
        return EXT_REG_UNKNOWN;
    if (extRegEqNoCase(q, n, "all"))
        return EXT_REG_ALL;
    for (int i = 0; i < EXT_REG_COUNT; i++)
        if (extRegEqNoCase(q, n, EXT_REG_TABLE[i].name))
            return i;
    return EXT_REG_UNKNOWN;
}

// TYP of a register frame: the frames start {"TYP":"<name>" (TYP is always
// the first member and survives bleJsonFrameFailSoft()). Copies the name into
// out; false if the frame has another shape.
static inline bool extRegFrameTyp(const char *json, size_t len, char *out, size_t cap)
{
    static const char head[] = "{\"TYP\":\"";
    const size_t hl = sizeof(head) - 1;
    if (json == nullptr || out == nullptr || cap < 2 || len <= hl || memcmp(json, head, hl) != 0)
        return false;
    size_t n = 0;
    while (hl + n < len && json[hl + n] != '"')
    {
        if (n + 1 >= cap)
            return false;
        out[n] = json[hl + n];
        n++;
    }
    if (hl + n >= len || n == 0)
        return false;
    out[n] = 0;
    return true;
}

// ---------------------------------------------------------------------------
// Secret filter
// ---------------------------------------------------------------------------

// A DENYLIST by key name, not an allowlist: a member is removed when its key
// (JSON escapes decoded, compared case-insensitively) is BPIN or WSPWD, or
// contains PWD, PASS, PW, KEY, SECRET, TOKEN, AUTH, HMAC or CRED, or ends in
// CODE, or is longer than the key buffer (cannot be judged, so removed). Every
// other key goes out. So a field added to a register later with a secret-like
// name is held back by the rule, but one with a harmless-looking name is
// ALLOWED until someone looks at it: review every key a register frame
// gains in command_functions.cpp against this list.
static inline bool extRegKeyIsSecret(const char *key)
{
    static const char *const bad[] = {"BPIN", "WSPWD", "PWD", "PASS", "PW", "KEY", "SECRET", "TOKEN", "AUTH", "HMAC", "CRED"};
    char up[40];
    size_t n = 0;
    for (; key[n] != 0 && n + 1 < sizeof(up); n++)
        up[n] = extRegUpper(key[n]);
    up[n] = 0;
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
        if (strstr(up, bad[i]) != nullptr)
            return true;
    if (n >= 4 && memcmp(up + n - 4, "CODE", 4) == 0)   // a CODE is a credential more often than not
        return true;
    return false;
}

#define EXT_REG_MAX_DEPTH 8

struct ExtRegCur
{
    const char *s;
    size_t n;
    size_t i;
    char *out;
    size_t cap;
    size_t o;
    bool ok;
    bool emit; // false while skipping the value of a removed member
};

static inline void extRegPut(ExtRegCur &c, char ch)
{
    if (!c.emit)
        return;
    if (c.o + 1 >= c.cap) // keep room for the NUL
    {
        c.ok = false;
        return;
    }
    c.out[c.o++] = ch;
}

static inline void extRegWs(ExtRegCur &c)
{
    while (c.i < c.n && (c.s[c.i] == ' ' || c.s[c.i] == '\t' || c.s[c.i] == '\r' || c.s[c.i] == '\n'))
        c.i++;
}

// Copies one string (c.s[c.i] == '"') to the output and, if key != nullptr,
// stores the decoded text (up to keycap-1 chars) there; *ktrunc is set if it
// did not fit.
static inline void extRegString(ExtRegCur &c, char *key, size_t keycap, bool *ktrunc)
{
    size_t k = 0;
    extRegPut(c, '"');
    c.i++;
    while (c.ok)
    {
        if (c.i >= c.n)
        {
            c.ok = false;
            return;
        }
        char ch = c.s[c.i++];
        if (ch == '"')
        {
            extRegPut(c, '"');
            break;
        }
        char dec = ch;
        if (ch == '\\')
        {
            if (c.i >= c.n)
            {
                c.ok = false;
                return;
            }
            char e = c.s[c.i++];
            extRegPut(c, '\\');
            extRegPut(c, e);
            if (e == 'u')
            {
                unsigned v = 0;
                for (int d = 0; d < 4; d++)
                {
                    if (c.i >= c.n)
                    {
                        c.ok = false;
                        return;
                    }
                    char h = c.s[c.i++];
                    extRegPut(c, h);
                    v <<= 4;
                    if (h >= '0' && h <= '9')
                        v |= (unsigned)(h - '0');
                    else if (h >= 'a' && h <= 'f')
                        v |= (unsigned)(h - 'a' + 10);
                    else if (h >= 'A' && h <= 'F')
                        v |= (unsigned)(h - 'A' + 10);
                    else
                    {
                        c.ok = false;
                        return;
                    }
                }
                dec = (v >= 0x20 && v < 0x80) ? (char)v : '?';
            }
            else if (e == 'n' || e == 't' || e == 'r' || e == 'b' || e == 'f')
                dec = '?';
            else
                dec = e;
        }
        else
            extRegPut(c, ch);
        if (key != nullptr)
        {
            if (k + 1 < keycap)
                key[k++] = dec;
            else if (ktrunc != nullptr)
                *ktrunc = true;
        }
    }
    if (key != nullptr)
        key[k] = 0;
}

static inline void extRegValue(ExtRegCur &c, int depth, const char *prefix);

static inline void extRegObject(ExtRegCur &c, int depth, const char *prefix)
{
    if (depth >= EXT_REG_MAX_DEPTH)
    {
        c.ok = false;
        return;
    }
    extRegPut(c, '{');
    c.i++;
    bool any = false;
    if (prefix != nullptr)
    {
        for (const char *p = prefix; *p; p++)
            extRegPut(c, *p);
        any = true;
    }
    bool first = true;
    while (c.ok)
    {
        extRegWs(c);
        if (c.i >= c.n)
        {
            c.ok = false;
            return;
        }
        if (c.s[c.i] == '}')
        {
            c.i++;
            break;
        }
        if (!first)
        {
            if (c.s[c.i] != ',')
            {
                c.ok = false;
                return;
            }
            c.i++;
            extRegWs(c);
        }
        first = false;
        if (c.i >= c.n || c.s[c.i] != '"')
        {
            c.ok = false;
            return;
        }
        const size_t o0 = c.o;
        if (any)
            extRegPut(c, ',');
        char key[40];
        bool ktrunc = false;
        extRegString(c, key, sizeof(key), &ktrunc);
        extRegWs(c);
        if (!c.ok || c.i >= c.n || c.s[c.i] != ':')
        {
            c.ok = false;
            return;
        }
        c.i++;
        extRegPut(c, ':');
        const bool secret = ktrunc || extRegKeyIsSecret(key);
        const bool saved = c.emit;
        if (secret)
            c.emit = false;
        extRegValue(c, depth + 1, nullptr);
        c.emit = saved;
        if (secret)
            c.o = o0; // roll back the comma and the key written above
        else
            any = true;
    }
    extRegPut(c, '}');
}

static inline void extRegValue(ExtRegCur &c, int depth, const char *prefix)
{
    extRegWs(c);
    if (c.i >= c.n)
    {
        c.ok = false;
        return;
    }
    char ch = c.s[c.i];
    if (ch == '{')
    {
        extRegObject(c, depth, prefix);
    }
    else if (ch == '[')
    {
        if (depth >= EXT_REG_MAX_DEPTH)
        {
            c.ok = false;
            return;
        }
        extRegPut(c, '[');
        c.i++;
        bool first = true;
        while (c.ok)
        {
            extRegWs(c);
            if (c.i >= c.n)
            {
                c.ok = false;
                return;
            }
            if (c.s[c.i] == ']')
            {
                c.i++;
                break;
            }
            if (!first)
            {
                if (c.s[c.i] != ',')
                {
                    c.ok = false;
                    return;
                }
                c.i++;
                extRegPut(c, ',');
            }
            first = false;
            extRegValue(c, depth + 1, nullptr);
        }
        extRegPut(c, ']');
    }
    else if (ch == '"')
    {
        extRegString(c, nullptr, 0, nullptr);
    }
    else
    {
        // number, true, false, null
        size_t start = c.i;
        while (c.i < c.n)
        {
            char t = c.s[c.i];
            if ((t >= '0' && t <= '9') || (t >= 'a' && t <= 'z') || (t >= 'A' && t <= 'Z') || t == '-' || t == '+' || t == '.')
            {
                extRegPut(c, t);
                c.i++;
            }
            else
                break;
        }
        if (c.i == start)
            c.ok = false;
    }
}

// Filters a serialised register object src[0..n) into out (NUL-terminated,
// compact) with the secret members removed. withType != 0 inserts
// "type":"reg" as the first member. Returns the length, 0 on any problem (not
// an object, malformed, too deep, does not fit): the caller then sends
// nothing, so a failure never lets the unfiltered text through.
static inline size_t extRegFilter(const char *src, size_t n, char *out, size_t cap, bool withType)
{
    if (src == nullptr || out == nullptr || cap < 3)
        return 0;
    ExtRegCur c = {src, n, 0, out, cap, 0, true, true};
    extRegWs(c);
    if (c.i >= c.n || c.s[c.i] != '{')
        return 0;
    extRegObject(c, 0, withType ? "\"type\":\"reg\"" : nullptr);
    if (c.ok)
    {
        extRegWs(c);
        if (c.i != c.n)
            c.ok = false;
    }
    if (!c.ok)
    {
        out[0] = 0;
        return 0;
    }
    out[c.o] = 0;
    return c.o;
}

// {"type":"reg","err":"<what>"} -- what is a fixed token ("unknown", "unavailable"),
// never peer input.
static inline size_t extRegErrorJson(char *out, size_t cap, const char *what)
{
    static const char head[] = "{\"type\":\"reg\",\"err\":\"";
    const size_t hl = sizeof(head) - 1;
    const size_t wl = strlen(what);
    if (out == nullptr || cap < hl + wl + 3)
        return 0;
    memcpy(out, head, hl);
    memcpy(out + hl, what, wl);
    out[hl + wl] = '"';
    out[hl + wl + 1] = '}';
    out[hl + wl + 2] = 0;
    return hl + wl + 2;
}

// {"type":"reg","q":"all","done":true} closes an "all" run.
static inline size_t extRegDoneJson(char *out, size_t cap)
{
    static const char s[] = "{\"type\":\"reg\",\"q\":\"all\",\"done\":true}";
    if (out == nullptr || cap < sizeof(s))
        return 0;
    memcpy(out, s, sizeof(s));
    return sizeof(s) - 1;
}

// ---------------------------------------------------------------------------
// Request limiter: the input is unauthenticated, so at most one query per
// second is accepted; the rest is dropped silently. Wrap-safe on millis().
// ---------------------------------------------------------------------------
#define EXT_REG_MIN_INTERVAL_MS 1000u
// gap between two registers of an "all" run
#define EXT_REG_ALL_STEP_MS 100u

struct ExtRegLimiter
{
    uint32_t last;
    bool have;
};

static inline bool extRegAllow(ExtRegLimiter &l, uint32_t now_ms)
{
    if (l.have && (uint32_t)(now_ms - l.last) < EXT_REG_MIN_INTERVAL_MS)
        return false;
    l.have = true;
    l.last = now_ms;
    return true;
}

// ---------------------------------------------------------------------------
// State of a running query: a queued single query (pending) and an "all" run
// (allIdx >= 0 = next table index, allNext = millis() of the next step).
// extRegRunReset() must run whenever the peer goes away or changes
// (resetExternUDP()): the tick is only called while EXTUDP is on, so a run
// left in this state would otherwise continue towards the NEW peer, and
// after > 24.8 days the signed millis() compare below would stall for good.
// ---------------------------------------------------------------------------
struct ExtRegRun
{
    bool pending;
    int allIdx;
    uint32_t allNext;
};

static inline void extRegRunReset(ExtRegRun &r)
{
    r.pending = false;
    r.allIdx = -1;
    r.allNext = 0;
}

static inline bool extRegRunBusy(const ExtRegRun &r)
{
    return r.pending || r.allIdx >= 0;
}

static inline void extRegRunStartAll(ExtRegRun &r, uint32_t now_ms)
{
    r.allIdx = 0;
    r.allNext = now_ms;
}

#define EXT_REG_STEP_NONE 0
#define EXT_REG_STEP_RUN 1  // run register *idx now
#define EXT_REG_STEP_DONE 2 // send the "done" frame now (the run is over)

static inline int extRegAllStep(ExtRegRun &r, uint32_t now_ms, int *idx)
{
    if (r.allIdx < 0 || (int32_t)(now_ms - r.allNext) < 0)
        return EXT_REG_STEP_NONE;
    if (r.allIdx < EXT_REG_COUNT)
    {
        if (idx != nullptr)
            *idx = r.allIdx;
        r.allIdx++;
        r.allNext = now_ms + EXT_REG_ALL_STEP_MS;
        return EXT_REG_STEP_RUN;
    }
    r.allIdx = -1;
    return EXT_REG_STEP_DONE;
}

#endif // _EXT_REG_H_
