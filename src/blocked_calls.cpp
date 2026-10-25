// BLK-01 -- call blocklist core, see blocked_calls.h.
#include "blocked_calls.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <strings.h>

#if !defined(NATIVE_BUILD)
#include <Arduino.h>
#if defined(ESP32)
#include <Preferences.h>
#else
#include <FreeRTOS.h>
#include <task.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#endif
#endif

// ---- lock --------------------------------------------------------------
// The list is only copied under the lock (<= 160 B); matching runs outside.
#if defined(NATIVE_BUILD)
#define BLK_LOCK()
#define BLK_UNLOCK()
#elif defined(ESP32)
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
#define BLK_LOCK() portENTER_CRITICAL(&s_mux)
#define BLK_UNLOCK() portEXIT_CRITICAL(&s_mux)
#else
#define BLK_LOCK() taskENTER_CRITICAL()
#define BLK_UNLOCK() taskEXIT_CRITICAL()
#endif

static char s_csv[BLOCKLIST_CSV_MAX];   // canonical "A,B,C", empty = no blocking
static volatile uint32_t s_drops = 0;
static bool s_last_save_ok = true;

// ---- normalisation -----------------------------------------------------

static bool isSep(char c) { return c == ',' || c == ';'; }

bool blockCallNormalize(const char *in, char *out, size_t cap)
{
    if (!in || !out || cap < BLOCKLIST_CALL_MAX + 1)
        return false;
    out[0] = 0;

    while (*in == ' ' || *in == '\t')
        in++;
    size_t len = strlen(in);
    while (len > 0 && (in[len - 1] == ' ' || in[len - 1] == '\t' || in[len - 1] == '\r' || in[len - 1] == '\n'))
        len--;

    // base: 3..6 of [A-Z0-9] with at least one letter and one digit
    size_t i = 0;
    bool letter = false, digit = false;
    char tmp[BLOCKLIST_CALL_MAX + 1];
    while (i < len && in[i] != '-')
    {
        char c = (char)toupper((unsigned char)in[i]);
        if (c >= 'A' && c <= 'Z')
            letter = true;
        else if (c >= '0' && c <= '9')
            digit = true;
        else
            return false;
        if (i >= 6)
            return false;
        tmp[i] = c;
        i++;
    }
    if (i < 3 || !letter || !digit)
        return false;
    size_t n = i;

    int ssid = 0;
    if (i < len)   // at '-'
    {
        i++;
        size_t nd = len - i;
        if (nd < 1 || nd > 2)
            return false;
        for (; i < len; i++)
        {
            if (in[i] < '0' || in[i] > '9')
                return false;
            ssid = ssid * 10 + (in[i] - '0');
        }
    }

    // built by hand: base <= 6 + '-' + 2 digits always fits (cap >= 10)
    memcpy(out, tmp, n);
    if (ssid > 0)
    {
        out[n++] = '-';
        if (ssid >= 10)
            out[n++] = (char)('0' + ssid / 10);
        out[n++] = (char)('0' + ssid % 10);
    }
    out[n] = 0;
    return true;
}

// ---- list parsing ------------------------------------------------------

// Next raw token (trimmed, clipped to cap-1) from *p; returns false at the end.
static bool nextToken(const char **p, char *tok, size_t cap)
{
    const char *s = *p;
    if (!*s)
        return false;
    const char *e = s;
    while (*e && !isSep(*e))
        e++;
    *p = *e ? e + 1 : e;

    while (s < e && (*s == ' ' || *s == '\t'))
        s++;
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n'))
        e--;
    size_t n = (size_t)(e - s);
    bool clipped = n > cap - 1;
    if (clipped)
        n = cap - 1;
    memcpy(tok, s, n);
    tok[n] = 0;
    // a clipped token must stay invalid ("DK5EN" + 30 spaces + "X" would
    // otherwise clip to a valid "DK5EN")
    if (clipped && n > 0)
        tok[n - 1] = '?';
    return true;
}

static bool isNone(const char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    if (strncasecmp(s, "none", 4) != 0)
        return false;
    s += 4;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')
        s++;
    return *s == 0;
}

// Shared by strict and lenient parsing. Strict: first problem -> false.
static bool listBuild(const char *in, char *out, size_t cap, char *bad, size_t badcap,
                      bool lenient, void (*dropped)(const char *), uint8_t *kept)
{
    if (kept)
        *kept = 0;
    if (!out || cap == 0)
        return false;
    out[0] = 0;
    if (bad && badcap)
        bad[0] = 0;
    if (!in || isNone(in))
        return true;

    size_t used = 0;
    uint8_t count = 0;
    const char *p = in;
    char tok[24];
    while (nextToken(&p, tok, sizeof(tok)))
    {
        if (!tok[0])
            continue;   // empty token

        char norm[BLOCKLIST_CALL_MAX + 1];
        bool ok = blockCallNormalize(tok, norm, sizeof(norm));
        if (ok)
        {
            // duplicate after normalisation: keep the first position
            bool dup = false;
            const char *q = out;
            char e[BLOCKLIST_CALL_MAX + 1];
            while (nextToken(&q, e, sizeof(e)))
                if (strcmp(e, norm) == 0)
                    dup = true;
            if (dup)
                continue;
        }

        const bool full = (count >= BLOCKLIST_MAX);
        size_t nl = ok ? strlen(norm) : 0;
        if (!ok || full || used + (used ? 1 : 0) + nl + 1 > cap)
        {
            if (lenient)
            {
                if (dropped)
                    dropped(tok);
                continue;
            }
            if (bad && badcap)
                snprintf(bad, badcap, "%s", ok ? "too many" : tok);
            out[0] = 0;
            return false;
        }
        if (used)
            out[used++] = ',';
        memcpy(out + used, norm, nl);
        used += nl;
        out[used] = 0;
        count++;
    }
    if (kept)
        *kept = count;
    return true;
}

bool blockListParse(const char *in, char *out, size_t cap, char *bad, size_t badcap)
{
    return listBuild(in, out, cap, bad, badcap, false, nullptr, nullptr);
}

uint8_t blockListLoadLenient(const char *in, char *out, size_t cap, void (*dropped)(const char *entry))
{
    uint8_t kept = 0;
    listBuild(in, out, cap, nullptr, 0, true, dropped, &kept);
    return kept;
}

// ---- matching ----------------------------------------------------------

// csv entries are canonical. Entry without SSID matches any SSID of the call.
static bool matchNorm(const char *csv, const char *norm)
{
    const char *dash = strchr(norm, '-');
    size_t baselen = dash ? (size_t)(dash - norm) : strlen(norm);

    const char *p = csv;
    char e[BLOCKLIST_CALL_MAX + 1];
    while (nextToken(&p, e, sizeof(e)))
    {
        if (strchr(e, '-'))
        {
            if (strcmp(e, norm) == 0)
                return true;
        }
        else if (strlen(e) == baselen && strncmp(e, norm, baselen) == 0)
            return true;
    }
    return false;
}

bool blockListMatchCall(const char *csv, const char *call)
{
    if (!csv || !csv[0] || !call)
        return false;
    char norm[BLOCKLIST_CALL_MAX + 1];
    if (!blockCallNormalize(call, norm, sizeof(norm)))
        return false;   // "*", "H", "WLNK-1", ... are never calls
    return matchNorm(csv, norm);
}

bool blockListMatchPath(const char *csv, const char *path)
{
    if (!csv || !csv[0] || !path)
        return false;
    const char *p = path;
    char tok[24];
    while (nextToken(&p, tok, sizeof(tok)))
    {
        // relay marker '*' (and spaces around it) is not part of the call
        size_t n = strlen(tok);
        while (n > 0 && (tok[n - 1] == '*' || tok[n - 1] == ' '))
            tok[--n] = 0;
        if (blockListMatchCall(csv, tok))
            return true;
    }
    return false;
}

// ---- runtime list ------------------------------------------------------

static void copyList(char *dst)
{
    BLK_LOCK();
    memcpy(dst, s_csv, sizeof(s_csv));
    BLK_UNLOCK();
}

bool blockListSet(const char *in, char *bad, size_t badcap)
{
    char tmp[BLOCKLIST_CSV_MAX];
    if (!blockListParse(in, tmp, sizeof(tmp), bad, badcap))
        return false;
    BLK_LOCK();
    memcpy(s_csv, tmp, sizeof(s_csv));
    BLK_UNLOCK();
    return true;
}

void blockListGet(char *out, size_t cap)
{
    if (!out || cap == 0)
        return;
    char tmp[BLOCKLIST_CSV_MAX];
    copyList(tmp);
    snprintf(out, cap, "%s", tmp);
}

uint8_t blockListCount()
{
    char tmp[BLOCKLIST_CSV_MAX];
    copyList(tmp);
    if (!tmp[0])
        return 0;
    uint8_t n = 1;
    for (const char *c = tmp; *c; c++)
        if (*c == ',')
            n++;
    return n;
}

bool isBlockedCall(const char *call)
{
    if (!s_csv[0])   // fast path; a torn read only costs one frame
        return false;
    char tmp[BLOCKLIST_CSV_MAX];
    copyList(tmp);
    return blockListMatchCall(tmp, call);
}

bool isBlockedInPath(const char *path)
{
    if (!s_csv[0])
        return false;
    char tmp[BLOCKLIST_CSV_MAX];
    copyList(tmp);
    return blockListMatchPath(tmp, path);
}

void blockListNoteDrop() { s_drops = s_drops + 1; }
bool blockListLastSaveOk() { return s_last_save_ok; }
uint32_t blockListDrops() { return s_drops; }

// ---- persistence -------------------------------------------------------
// Key and file format are fixed: nodes coming from a fork that wrote the same
// ones keep their list.

#if defined(NATIVE_BUILD)

void blockListLoad() {}
static bool s_host_save_fail = false;   // host tests: simulate a full NVS
void blockListTestFailSave(bool fail) { s_host_save_fail = fail; }
static bool blockListSaveImpl() { return !s_host_save_fail; }

#else

static void printDropped(const char *entry)
{
    Serial.printf("[BLOCK];load;dropped;%s\n", entry);
}

static void applyLoaded(const char *raw)
{
    char tmp[BLOCKLIST_CSV_MAX];
    blockListLoadLenient(raw, tmp, sizeof(tmp), printDropped);
    BLK_LOCK();
    memcpy(s_csv, tmp, sizeof(s_csv));
    BLK_UNLOCK();
}

#if defined(ESP32)

// Own namespace: clear_flash() wipes "Credentials" on --reset and on a settings
// layout bump; the blocklist survives both, like the nRF52 file does. The fork
// build stored it as "Credentials"/"block_list"; that key is migrated once.
static const char kBlockNs[] = "blocklist";
static const char kBlockKey[] = "block_list";

void blockListLoad()
{
    Preferences prefs;
    prefs.begin(kBlockNs, false);   // read-write: a read-only open of a missing namespace logs an error
    bool have = prefs.isKey(kBlockKey);
    String v = have ? prefs.getString(kBlockKey, "") : String("");
    prefs.end();

    if (!have)
    {
        // legacy key: migrate, then remove it so a later cleared list cannot
        // be resurrected from it
        prefs.begin("Credentials", false);
        bool legacy = prefs.isKey(kBlockKey);
        v = legacy ? prefs.getString(kBlockKey, "") : String("");
        applyLoaded(v.c_str());
        // remove it only once the new copy is stored: a failed save keeps the
        // legacy key, and the next boot migrates again
        if (legacy && blockListSave())
            prefs.remove(kBlockKey);
        prefs.end();
        return;
    }
    applyLoaded(v.c_str());
}

static bool blockListSaveImpl()
{
    char tmp[BLOCKLIST_CSV_MAX];
    blockListGet(tmp, sizeof(tmp));
    Preferences prefs;
    if (!prefs.begin(kBlockNs, false))
        return false;
    // putString() returns the bytes written, 0 on failure (full NVS); an empty
    // list is stored as a removed key so it never looks like a write failure
    bool ok = tmp[0] ? prefs.putString(kBlockKey, tmp) == strlen(tmp)
                     : (!prefs.isKey(kBlockKey) || prefs.remove(kBlockKey));
    prefs.end();
    return ok;
}

#else // nRF52

static const char kBlockFile[] = "/blocklist.cfg";
static const char kBlockMagic[] = "BLK1:";

void blockListLoad()
{
    InternalFS.begin();   // idempotent -- init_flash() mounts it at boot
    File f(InternalFS);
    char buf[sizeof(kBlockMagic) + BLOCKLIST_CSV_MAX + 64];
    size_t n = 0;
    if (f.open(kBlockFile, FILE_O_READ))
    {
        n = f.read((uint8_t *)buf, sizeof(buf) - 1);
        f.close();
    }
    buf[n] = 0;
    const size_t ml = sizeof(kBlockMagic) - 1;
    if (n < ml || strncmp(buf, kBlockMagic, ml) != 0)
        return;   // absent or foreign file: stay empty
    applyLoaded(buf + ml);
}

static bool blockListSaveImpl()
{
    char csv[BLOCKLIST_CSV_MAX];
    blockListGet(csv, sizeof(csv));
    char buf[sizeof(kBlockMagic) + BLOCKLIST_CSV_MAX];
    int len = snprintf(buf, sizeof(buf), "%s%s", kBlockMagic, csv);
    if (len < 0)
        return false;

    InternalFS.begin();
    InternalFS.remove(kBlockFile);
    File f(InternalFS);
    if (!f.open(kBlockFile, FILE_O_WRITE))
        return false;
    size_t w = f.write((const uint8_t *)buf, (size_t)len);
    f.flush();
    f.close();
    return w == (size_t)len;
}

#endif // ESP32 / nRF52
#endif // NATIVE_BUILD

bool blockListSave()
{
    s_last_save_ok = blockListSaveImpl();
    return s_last_save_ok;
}
