/**
 * settings_schema: builds the settings_store::FieldDescriptor table for
 * s_meshcom_settings by expanding CFG_FIELD_LIST(X) (config_json.h) and then
 * SETTINGS_PERSIST_ONLY_LIST(X) (settings_schema.h) into one array. See
 * settings_schema.h for what each list covers and why.
 *
 * This file wires config_json.h's field table into settings_store.h's
 * schema type; it does not itself call settings_store::encode()/decode() --
 * that wiring (real device persistence) is later wave work per
 * settings_store.h's own top comment.
 *
 * ---------------------------------------------------------------------------
 * CFG_CHR -> U8, not STRING (deviation from a naive type table)
 * ---------------------------------------------------------------------------
 *
 * CFG_CHR members (node_symid, node_symcd, node_lat_c, node_lon_c -- all
 * `char`, sizeof == 1) are mapped to settings_store::FieldType::U8, not
 * ::STRING. A STRING descriptor's `size` is documented (settings_store.h) as
 * "N including the NUL terminator slot", so a size-1 STRING can never hold a
 * NUL-terminated one-character value -- there is no room for the terminator.
 * Mapping CFG_CHR to STRING would either violate this file's own
 * static_assert(size >= 2) below (a build break) or -- had that assertion
 * been narrowed to dodge exactly these four fields -- silently truncate
 * every value to empty, which is the "silent truncation" a settings-schema
 * table must not ship.
 *
 * settings_store.h's own "TYPE COVERAGE" section already resolves this
 * without inventing a new FieldType: "I8, U8 -- ... (covers CFG_CHR's single
 * `char` too -- a bare `char`'s signedness is platform-defined, so callers
 * with an unsigned single-char field describe it as U8)". All four CFG_CHR
 * fields here hold printable ASCII (symbol table id/code, or a lat/lon
 * hemisphere letter) -- values in 0..127, so U8 vs. I8 makes no practical
 * difference; U8 is the one the header comment names explicitly.
 */
#include "settings_schema.h"

#include <config_json.h>
#include <maxhop.h>
#include <settings_store.h>

#include <meshcom_settings.h> // the one settings struct, both platforms (D1-04)

// The variant's configuration.h supplies TX_POWER_MIN/TX_POWER_MAX for the
// node_power row's range. Guarded because this translation unit is now also
// compiled into the two safeboot images (platformio.ini), which build no
// variant and therefore have no configuration.h on the include path -- there
// the fallbacks immediately below apply. Safeboot only ever LOADS settings to
// reach the OTA path; the range bounds clamp on decode, never on save, so a
// wider fallback range cannot write an out-of-range value to NVS.
#if defined(__has_include)
    #if __has_include(<configuration.h>)
        #include <configuration.h>
    #endif
#else
    #include <configuration.h>
#endif

#include <stddef.h>

/* Same fallback config_json.cpp carries for the native/no-variant build --
 * see the comment there. Kept here too because this is a second translation
 * unit expanding the same CFG_FIELD_LIST(X), which references these macros
 * in its node_power row. */
#ifndef TX_POWER_MIN
    #define TX_POWER_MIN (-20)
#endif
#ifndef TX_POWER_MAX
    #define TX_POWER_MAX 30
#endif

namespace settings_schema {

namespace {

// See the file header comment: resolves CfgType -> settings_store::FieldType
// for every row of both field lists, CFG_CHR -> U8 deliberately (not the
// naive CFG_CHR -> STRING a first pass at this table would reach for).
constexpr settings_store::FieldType CfgTypeToFieldType(CfgType t) {
    return t == CFG_STR    ? settings_store::FieldType::STRING
         : t == CFG_CHR    ? settings_store::FieldType::U8
         : t == CFG_INT    ? settings_store::FieldType::I32
         : t == CFG_U32    ? settings_store::FieldType::U32
         : t == CFG_BOOL   ? settings_store::FieldType::BOOL
         : t == CFG_FLT    ? settings_store::FieldType::FLOAT
         : /* CFG_DBL */     settings_store::FieldType::DOUBLE;
}

} // namespace

// ---------------------------------------------------------------------------
// Correctness machinery: one static_assert set per included row.
//
//   1. the member fits inside s_meshcom_settings (catches a typo'd member
//      name resolving to the wrong offset/size combination, or a struct
//      whose layout shrank under this table).
//   2. a STRING row's buffer is at least 2 bytes (room for one content byte
//      plus the NUL terminator) -- see the CFG_CHR -> U8 note above for why
//      this is the rule that forced that mapping choice.
//   3. ESP32 ONLY (D1-04 W3 Task 1(e)): the NVS key is <= 15 characters.
//      ESP32's Preferences/NVS silently rejects (or truncates, depending on
//      library version) any key longer than that -- a fail-CLOSED compile-
//      time gate here means a too-long key can never ship, instead of
//      surfacing as a field that mysteriously never persists on real
//      hardware. Guarded to ESP32 on purpose: NVS is an ESP32 concept and
//      the nRF52 keyed store has no such limit (the 16-character
//      "send_repeat_time" row that once forced this guard is gone with the
//      D1-04 struct merge). `sizeof(key) - 1` is the string length (sizeof
//      a string literal includes its NUL terminator; `key` is always a
//      literal here, one per X() row).
//
// (esc, has_esc) are accepted per CFG_FIELD_LIST's row shape. FieldDescriptor
// has no escape-sentinel concept, so has_esc is folded into has_range instead
// -- see the CFG_ESC note on the row macro below.
// ---------------------------------------------------------------------------
#ifdef ESP32
#define SETTINGS_SCHEMA_ASSERT_KEY_LENGTH(key) \
    static_assert(sizeof(key) - 1 <= 15, key);
#else
#define SETTINGS_SCHEMA_ASSERT_KEY_LENGTH(key)
#endif

#define SETTINGS_SCHEMA_ASSERT_ROW(key, cfgtype, member, lo, hi, esc, has_esc)                   \
    static_assert(offsetof(s_meshcom_settings, member) +                                         \
                      sizeof(((s_meshcom_settings *)0)->member) <=                                \
                      sizeof(s_meshcom_settings),                                                 \
                  key);                                                                           \
    static_assert(CfgTypeToFieldType(cfgtype) != settings_store::FieldType::STRING ||             \
                      sizeof(((s_meshcom_settings *)0)->member) >= 2,                             \
                  key);                                                                           \
    SETTINGS_SCHEMA_ASSERT_KEY_LENGTH(key)

#define X(...) SETTINGS_SCHEMA_ASSERT_ROW(__VA_ARGS__)
CFG_FIELD_LIST(X)
SETTINGS_PERSIST_ONLY_LIST(X)
#undef X

#undef SETTINGS_SCHEMA_ASSERT_ROW
#undef SETTINGS_SCHEMA_ASSERT_KEY_LENGTH

// Cross-check performed by hand against
// docs/d1-04-settings-field-triage-20260912.md S3 (recorded here, not
// machine-checked -- the doc is the source of truth): every row CFG_FIELD_LIST
// expands (all of config_json.h's common list plus CFG_FIELD_LIST_PLATFORM)
// is classified PERSIST there. None is classified RUNTIME, so no
// CFG_FIELD_LIST row needed excluding from this table.

// ---------------------------------------------------------------------------
// The table itself.
// ---------------------------------------------------------------------------
// CFG_ESC rows are NOT range-checked, which is why has_range folds in
// !has_esc rather than reading (lo <= hi) alone.
//
// Two rows carry CFG_ESC today. node_power, whose escape value is
// CFG_POWER_NOT_SET (-20) meaning "no TX power stored yet". That sentinel is
// deliberately OUTSIDE TX_POWER_MIN..MAX on several boards (RAK4631: 2..22).
// settings_store::decode() CLAMPS an out-of-envelope numeric into the range
// when has_range is set, so describing this row as range-checked would turn a
// factory-fresh node's -20 into the board minimum on the very first load --
// destroying the sentinel silently. settings_sanitize.cpp would then see a
// legitimate in-range value and leave it alone (:48), so the node would pin
// itself to minimum TX power instead of taking its board default (:123).
//
// And max_hop_text, added 2026-09-12 after the same failure mode was found a
// second time: range 1..6, struct default 0 meaning "nothing stored yet"
// (maxhop.h:30-37), whose intended resolution is MAXHOP_TEXT_FALLBACK (4).
// Without the escape, decode() clamped the 0 to 1 before maxHopTextSanitize()
// could see it, and 1 is a valid value, so it was kept -- pinning the node to
// one text hop instead of four.
//
// The other sentinel-bearing radio rows do not need the escape: node_freq,
// node_bw, node_sf and node_cr all use 0 as their "not set" value
// (settings_sanitize.cpp:8, FLOAT_NOT_SET) and 0 already lies inside each of
// their declared ranges.
//
// The rule that finds this class, which is the one to apply to any new row:
// a row needs CFG_ESC when its STRUCT DEFAULT lies outside its own declared
// [lo, hi]. Sweeping every row on that rule finds exactly these two. Checking
// only the fields that settings_sanitize.cpp names is NOT the same sweep and
// misses max_hop_text -- that is how it was missed the first time.
#define SETTINGS_SCHEMA_ROW(key, cfgtype, member, lo, hi, esc, has_esc)                          \
    {key, CfgTypeToFieldType(cfgtype), offsetof(s_meshcom_settings, member),                     \
     sizeof(((s_meshcom_settings *)0)->member),                                                   \
     ((double)(lo) <= (double)(hi)) && !(has_esc), (double)(lo), (double)(hi)},

static const settings_store::FieldDescriptor kFields[] = {
#define X(...) SETTINGS_SCHEMA_ROW(__VA_ARGS__)
    CFG_FIELD_LIST(X) SETTINGS_PERSIST_ONLY_LIST(X)
#undef X
};

#undef SETTINGS_SCHEMA_ROW

static const size_t kFieldCount = sizeof(kFields) / sizeof(kFields[0]);

const settings_store::FieldDescriptor *fields() { return kFields; }

size_t fieldCount() { return kFieldCount; }

} // namespace settings_schema
