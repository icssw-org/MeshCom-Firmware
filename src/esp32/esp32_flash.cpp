#include <Arduino.h>

#include "esp32_flash.h"
#if !defined(MC_SAFEBOOT)
// TM-32: der Sanitize-Pfad braucht Varianten-Konstanten (configuration.h via
// lora_setchip.h), die der minimale Safeboot-Build nicht hat -- und kein Radio,
// dessen Parameter zu plausibilisieren waeren.
#include <settings_sanitize.h>
#include <msgid_counter.h>
#include <lora_setchip.h>
#else
#include <maxhop.h>   // MAXHOP_TEXT_FALLBACK, dependency-frei (siehe unten)
#endif

#include <settings_schema.h>
#include <settings_store.h>
#include <counters_store.h>

#include <cstring>

void save_settings(void);

#include <Preferences.h>

#include "printfdeb_functions.h"

Preferences preferences;

s_meshcom_settings meshcom_settings;

// ---------------------------------------------------------------------------
// D1-04 W3 Task 2: fail-closed guard against the class of defect this same
// wave found and fixed (sanitize_loaded_settings() calling save_settings()
// from inside init_flash()'s own open load handle -- see
// test/test_esp32_flash_lifecycle for the full writeup and the mutation that
// reproduces it). 245 save_settings() call sites exist in src/; any future
// one reached while init_flash() still holds its read handle open repeats
// the defect silently. This flag is the one thing standing between that and
// a loud, cheap refusal: false on every call except the one window it
// actually needs to be true, so the normal (non-init_flash) path pays one
// bool read.
static bool g_flash_load_in_progress = false;

#if defined(NATIVE_BUILD) || defined(UNIT_TEST)
// Test-only hook: lets a native test drive save_settings()'s refusal path
// directly (flag on, call it, flag off) without reproducing the historical
// call-ordering bug inside init_flash() itself, which would mean editing the
// real control flow just to exercise the guard. Compiled out of every real
// firmware build.
extern "C" void mc_test_set_flash_load_in_progress(bool v) { g_flash_load_in_progress = v; }
#endif

// ---------------------------------------------------------------------------
// D1-04 W3 Task 1: the ESP32 NVS backend for settings_schema.
//
// One dispatch per settings_store::FieldType, straight into/out of
// Preferences' own typed get/put calls, keyed by FieldDescriptor::offset into
// `meshcom_settings`. This replaces the old hand-written list of 134
// preferences.get*()/132 preferences.put*() calls: adding a field to
// CFG_FIELD_LIST/SETTINGS_PERSIST_ONLY_LIST (settings_schema.h) is now the
// only edit needed to persist it on ESP32 too.
//
// CFG_CHR -> FieldType::U8 (settings_schema.cpp's own mapping) is dispatched
// via preferences.getChar()/putChar() here, not getUChar()/putUChar(): every
// U8 row in this table today is one of the four CFG_CHR fields (node_symid,
// node_symcd, node_lat_c, node_lon_c), and the ORIGINAL hand-written code
// already used getChar()/putChar() for exactly these -- matching that API
// keeps the NVS value TYPE (not just the key spelling) byte-identical to
// what every already-provisioned node has on flash today (Preferences' NVS
// types are read back strictly by the type they were written with).
//
// node_gpsbaud/node_cleanflash (the two FieldType::U32 rows) were previously
// written with putULong()/read with getUInt() -- an existing asymmetry in
// the hand-written code that round-trips fine because `unsigned long` and
// `uint32_t` are the same 4-byte width on this architecture, so both API
// names produce the identical NVS type tag (PT_U32). Dispatched here via
// getUInt()/putUInt() uniformly: same on-flash type, one call pair instead
// of two spellings for the same thing.
// ---------------------------------------------------------------------------
namespace
{

void loadFieldFromPreferences(const settings_store::FieldDescriptor &d, void *base)
{
    void *field = (uint8_t *)base + d.offset;
    switch (d.type)
    {
    case settings_store::FieldType::STRING:
    {
        char *buf = (char *)field;
        // "Missing key -> keep the field's current value" (settings_store.h's
        // own decode() contract, replicated here): the current buffer
        // content -- already NUL-terminated within `d.size` by construction
        // -- is passed as Preferences' own default, so an absent key leaves
        // the field exactly as it was, and a present key overwrites it,
        // bounded to the buffer.
        String cur(buf);
        String v = preferences.getString(d.key, cur);
        snprintf(buf, d.size, "%s", v.c_str());
        break;
    }
    case settings_store::FieldType::I8:
    case settings_store::FieldType::U8:
    {
        int8_t *p = (int8_t *)field;
        *p = preferences.getChar(d.key, *p);
        break;
    }
    case settings_store::FieldType::I32:
    {
        int32_t *p = (int32_t *)field;
        *p = preferences.getInt(d.key, *p);
        break;
    }
    case settings_store::FieldType::U32:
    {
        uint32_t *p = (uint32_t *)field;
        *p = preferences.getUInt(d.key, *p);
        break;
    }
    case settings_store::FieldType::FLOAT:
    {
        float *p = (float *)field;
        *p = preferences.getFloat(d.key, *p);
        break;
    }
    case settings_store::FieldType::DOUBLE:
    {
        double *p = (double *)field;
        *p = preferences.getDouble(d.key, *p);
        break;
    }
    case settings_store::FieldType::BOOL:
    {
        bool *p = (bool *)field;
        *p = preferences.getBool(d.key, *p);
        break;
    }
    }
}

void saveFieldToPreferences(const settings_store::FieldDescriptor &d, void *base)
{
    void *field = (uint8_t *)base + d.offset;
    switch (d.type)
    {
    case settings_store::FieldType::STRING:
    {
        String v = (char *)field;
        preferences.putString(d.key, v);
        break;
    }
    case settings_store::FieldType::I8:
    case settings_store::FieldType::U8:
        preferences.putChar(d.key, *(int8_t *)field);
        break;
    case settings_store::FieldType::I32:
        preferences.putInt(d.key, *(int32_t *)field);
        break;
    case settings_store::FieldType::U32:
        preferences.putUInt(d.key, *(uint32_t *)field);
        break;
    case settings_store::FieldType::FLOAT:
        preferences.putFloat(d.key, *(float *)field);
        break;
    case settings_store::FieldType::DOUBLE:
        preferences.putDouble(d.key, *(double *)field);
        break;
    case settings_store::FieldType::BOOL:
        preferences.putBool(d.key, *(bool *)field);
        break;
    }
}

// Keys the LOAD walk skips -- both for reasons that only apply to loading;
// SAVE (see save_settings() below) has no exclusions at all and writes every
// row in settings_schema::fields() generically.
//
//   - "max_hop_text": its load-time default is a build-variant literal
//     (MC_SAFEBOOT vs not, see the two-line block below) -- both variants are
//     asserted numerically equal (command_functions.cpp's
//     static_assert(MAXHOP_TEXT_FALLBACK == MAX_HOP_TEXT_DEFAULT, ...)), but
//     kept as its own hand-written call rather than folded into the walk's
//     generic "missing key -> keep the field's current value" default, to
//     avoid this file depending on maxhop.h being reachable from both
//     branches of the #if at the top of this file.
//
// "node_msgid" USED to be a second entry here (it advanced via
// msgIdAfterLoad() right after the walk, so the walk itself had to skip it
// or re-reading it a second time from NVS would undo the advance). It is no
// longer a settings_schema row at all (D1-04 W3 step 4, operator decision
// 2026-09-13): the walk never sees it, and its own load/advance/write-back
// lives in counters_store.h's countersLoad(), called from init_flash()
// separately -- see that call site's comment.
bool isLoadSpecialCased(const char *key)
{
    return strcmp(key, "max_hop_text") == 0;
}

} // namespace

// Get LoRa parameter
// TM-32: Plausibilitaet der geladenen Radio-Parameter (Marker und Groesse
// prueft N-12, den Inhalt bisher niemand). Korrekturen werden geloggt.
#if !defined(MC_SAFEBOOT)
static void sanitize_log(const char *field, const char *oldv, const char *newv)
{
    Serial.printf("[FLASH]...sanitized %s: %s -> %s\n", field, oldv, newv);
}

// Returns true when this call actually changed something in
// `meshcom_settings` (a radio param or max_hop_text out of range and reset).
// init_flash() (below) uses this to decide whether it needs to call
// save_settings() at all -- see that call site's comment. D1-04 W3 step 4:
// this function used to also advance and report on the message-id
// high-water mark (msgid_counter.h); that responsibility, write-back
// included, now lives entirely in counters_store.h's countersLoad(), called
// separately by init_flash() -- a correction here and the counter's own
// advance are unrelated events and no longer need to share one return path.
bool sanitize_loaded_settings(void)
{
    RadioLimits lim = { TX_POWER_MIN, TX_POWER_MAX, 400.0f, 960.0f, 0, 0, max_country };
    RadioParams p = { meshcom_settings.node_power, meshcom_settings.node_freq, meshcom_settings.node_bw,
                      meshcom_settings.node_sf, meshcom_settings.node_cr, meshcom_settings.node_country };
    int fixed = sanitize_radio_params(p, lim, sanitize_log);
    if(fixed > 0)
    {
        meshcom_settings.node_power = p.power;
        meshcom_settings.node_freq = p.freq;
        meshcom_settings.node_bw = p.bw;
        meshcom_settings.node_sf = p.sf;
        meshcom_settings.node_cr = p.cr;
        meshcom_settings.node_country = p.country;
    }

    // CS-01: derselbe Plausibilitaetspfad fuer das persistente Hop-Limit
    if(sanitize_max_hop_text(meshcom_settings.max_hop_text, sanitize_log))
        fixed++;

    // Akku-Maximalspannung muss ueber der Board-Leerspannung liegen (2S TBEAM_1W: nach
    // Wipe 4.2 V < 6.5 V -> BAT-01 meldet "kein Akku", Anzeige bleibt auf USB)
    #if defined(BAT_MIN_VOLTAGE) && defined(BAT_MAX_VOLTAGE)
    if(sanitize_max_voltage(meshcom_settings.node_maxv, BAT_MIN_VOLTAGE, BAT_MAX_VOLTAGE, sanitize_log))
        fixed++;
    #endif

    if(fixed > 0)
        Serial.printf("[FLASH]...%d setting(s) out of range, reset to default\n", fixed);

    return fixed > 0;
}

// ---------------------------------------------------------------------------
// D1-04 W3 step 4: counters_store.h's ESP32 backend. Contract is that
// header's own top comment -- read it first. Storage is a SEPARATE
// Preferences namespace ("Counters") through a SEPARATE static Preferences
// instance, deliberately never the global `preferences` object init_flash()/
// save_settings() use for "Credentials": sharing one handle across two
// unrelated call paths is exactly what bce95db5 got bitten by (a nested
// begin()/end() pair silently closing the handle a caller further up the
// stack was still using -- see g_flash_load_in_progress's comment above).
// Keeping the counter on its own handle makes that class of bug structurally
// unreachable here, not just guarded against.
static Preferences counters_preferences;

void countersLoad(void)
{
    if (!counters_preferences.begin("Counters", false))
        Serial.printf("[SETST];counters;namespace_open;failed\n");
    if (counters_preferences.isKey("node_msgid"))
    {
        meshcom_settings.node_msgid = counters_preferences.getInt("node_msgid", meshcom_settings.node_msgid);
    }
    else
    {
        // Upgrade path: nothing in "Counters" yet -- fall back to the legacy
        // location this value lived in before this cutover. Opened
        // read-only, on its own short-lived Preferences instance, and NOT
        // deleted afterwards: a downgrade back to a pre-cutover firmware
        // must still find it in "Credentials".
        counters_preferences.end();
        Preferences legacy;
        legacy.begin("Credentials", true);
        meshcom_settings.node_msgid = legacy.getInt("node_msgid", meshcom_settings.node_msgid);
        legacy.end();
        counters_preferences.begin("Counters", false);
    }

    // Message-id high-water mark (msgid_counter.h). The counter is no longer
    // persisted on every originated frame -- it reaches flash once per
    // kMsgIdPersistStep frames -- so the stored value can be up to one step
    // behind what the node actually used before it went down. Stepping past
    // that whole block here, and writing the result back BEFORE returning,
    // is what keeps an id from being handed out twice after an unclean
    // shutdown -- see msgid_counter.h's own top comment for why the
    // write-back is not optional.
    meshcom_settings.node_msgid = msgIdAfterLoad(meshcom_settings.node_msgid);
    // msgid_counter.h: the advanced block MUST reach flash before the first frame goes out. NVS can
    // be full or the namespace can fail to open; neither is silent any more (advisor finding 6).
    if (counters_preferences.putInt("node_msgid", meshcom_settings.node_msgid) == 0)
        Serial.printf("[SETST];counters;load_writeback;failed\n");
    counters_preferences.end();
}

bool countersSave(void)
{
    if (!counters_preferences.begin("Counters", false))
        return false;
    bool ok = counters_preferences.putInt("node_msgid", meshcom_settings.node_msgid) > 0;
    counters_preferences.end();
    return ok;
}
#endif // !MC_SAFEBOOT

void init_flash(void)
{
    Serial.println("[INIT]...init_flash");

    preferences.begin("Credentials", false);
    g_flash_load_in_progress = true;

    // -------------------------------------------------------------------
    // Pre-load defaults.
    //
    // settings_store::FieldDescriptor carries no "default value" field, only
    // a clamp range (settings_store.h) -- so the walk below treats a missing
    // NVS key the same way settings_store::decode() documents for the nRF52
    // keyed store: the field is left exactly as it already is in
    // `meshcom_settings` when the walk reaches it, and only overwritten when
    // the key is actually present. For most of the ~130 persisted fields
    // that is already correct with NO extra code: s_meshcom_settings' own
    // field initialiser (esp32_flash.h) already matches the literal 2nd-arg
    // default the old hand-written preferences.getX(key, literal) call used
    // for that field.
    //
    // The fields below do NOT match -- historically because every one of the
    // 132 old load call sites carried its OWN literal instead of relying on
    // the struct's default, and the two silently drifted apart over time
    // (e.g. node_maxv: struct default 4.24, old load default 4.200;
    // node_owgpio: struct default 36, old load default 0). Seeded here, once,
    // so a truly first-ever boot (empty NVS) lands on the exact value it did
    // before this refactor -- do NOT "fix" these to match the struct's own
    // compiled default in esp32_flash.h instead: that header value was never
    // what a device actually booted with, only what the walk's generic
    // fallback rule would use if left unseeded.
    meshcom_settings.node_call[0] = '\0';
    meshcom_settings.node_short[0] = '\0';
    meshcom_settings.node_lat_c = 'N';
    meshcom_settings.node_lon_c = 'E';
    snprintf(meshcom_settings.node_ossid, sizeof(meshcom_settings.node_ossid), "none");
    snprintf(meshcom_settings.node_opwd, sizeof(meshcom_settings.node_opwd), "none");
    snprintf(meshcom_settings.node_extern, sizeof(meshcom_settings.node_extern), "none");
    meshcom_settings.node_maxv = 4.200f;
    // -20 == CFG_POWER_NOT_SET (config_json.h): "no TX power stored yet",
    // deliberately outside TX_POWER_MIN..MAX on several boards so
    // sanitize_loaded_settings() (below) can tell "never configured" apart
    // from a legitimately stored value.
    meshcom_settings.node_power = -20;
    meshcom_settings.node_owgpio = 0;
    meshcom_settings.node_utcoff = 1.0f;
    meshcom_settings.node_preamplebits = 32;
    snprintf(meshcom_settings.node_ssid, sizeof(meshcom_settings.node_ssid), "none");
    snprintf(meshcom_settings.node_pwd, sizeof(meshcom_settings.node_pwd), "none");
    snprintf(meshcom_settings.node_parm, sizeof(meshcom_settings.node_parm), "none");
    snprintf(meshcom_settings.node_unit, sizeof(meshcom_settings.node_unit), "none");
    snprintf(meshcom_settings.node_format, sizeof(meshcom_settings.node_format), "none");
    snprintf(meshcom_settings.node_eqns, sizeof(meshcom_settings.node_eqns), "none");
    snprintf(meshcom_settings.node_values, sizeof(meshcom_settings.node_values), "none");
    snprintf(meshcom_settings.node_lora_call, sizeof(meshcom_settings.node_lora_call), "none");
    snprintf(meshcom_settings.node_gwsrv, sizeof(meshcom_settings.node_gwsrv), "OE");
    meshcom_settings.node_fversion = 0;
    #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS) || defined(BOARD_T_DECK_PRO)
    meshcom_settings.node_kbl_sync = false;
    #endif

    // Explicit, non-generic load -- see isLoadSpecialCased()'s comment for
    // why this key is not part of the walk below.
    #if defined(MC_SAFEBOOT)
    meshcom_settings.max_hop_text = preferences.getInt("max_hop_text", MAXHOP_TEXT_FALLBACK);
    #else
    meshcom_settings.max_hop_text = preferences.getInt("max_hop_text", MAX_HOP_TEXT_DEFAULT);
    #endif

    // The walk: every remaining row of settings_schema::fields() -- CFG_FIELD_LIST
    // + CFG_FIELD_LIST_PLATFORM's ESP32 branch + SETTINGS_PERSIST_ONLY_LIST[_PLATFORM]
    // -- loaded generically, T-Deck-only rows included (the schema only
    // contains those on a T-Deck build; see settings_schema.h).
    for (size_t i = 0; i < settings_schema::fieldCount(); i++)
    {
        const settings_store::FieldDescriptor &d = settings_schema::fields()[i];
        if (isLoadSpecialCased(d.key))
            continue;
        loadFieldFromPreferences(d, &meshcom_settings);
    }

    // TM-32 (upstream #661/#57): Radio-Parameter auf Plausibilitaet pruefen,
    // bevor sie in radio.setOutputPower() & Co. landen. Sentinels bleiben.
    // Called at exactly this point (radio params and max_hop_text already
    // loaded; nothing else has been reached yet by anything OUTSIDE this
    // function) -- same relative position this call has held since TM-32
    // landed it. `settings_corrected` records whether it actually changed
    // anything, for the conditional save_settings() call below.
    #if !defined(MC_SAFEBOOT)
    bool settings_corrected = sanitize_loaded_settings();
    #endif

    g_flash_load_in_progress = false;
    preferences.end();

    // save_settings() runs here -- after the walk above has finished and
    // after this function's own preferences.end(), never from inside
    // sanitize_loaded_settings() itself, where an earlier commit this same
    // day (bce95db5, "W3: node_msgid reaches flash once per 100 frames") had
    // put an equivalent call. Found while wiring this walk, not by that
    // commit's own testing: calling save_settings() before every field had
    // been loaded persists a HALF-loaded struct, so every field not yet
    // reached at that point gets clobbered in NVS with whatever
    // s_meshcom_settings' compiled default (or this function's own pre-load
    // seed above) happened to still hold. Worse, save_settings() opens its
    // OWN preferences.begin()/end() pair (Preferences::begin() is a
    // documented no-op re-entering an already-open handle, harmless) -- but
    // its preferences.end() is UNCONDITIONAL once the handle is open
    // (Preferences.cpp), so it closes the handle THIS function is still
    // using. Every following preferences.getX() call in this function (there
    // would have been ~70 of them, everything from node_track_freq onward in
    // the old hand-written order) would then see Preferences::_started ==
    // false and silently return its own literal default argument, NEVER
    // reading NVS again for the rest of this boot -- indistinguishable, from
    // the node's point of view, from every one of those settings resetting
    // to factory default on every single reboot. Neither half of this (the
    // clobber, or the closed-handle short-circuit) is specific to the schema
    // walk above; both apply identically to the original hand-written call
    // list. This is a separate, independent defect from the one docs/...
    // already describes as "the boot-2 settings loss was newlib-nano's
    // printf, not a schema gap" -- flagged for its own follow-up, not fixed
    // further than moving this one call site back out of the load path.
    //
    // Unlike the original one-write-per-boot shape, this call is now
    // CONDITIONAL on sanitize_loaded_settings() having actually corrected
    // something: with node_msgid no longer a settings_schema row (D1-04 W3
    // step 4), nothing else in this struct changes on an ordinary boot, so
    // an unconditional write here would just re-persist byte-identical data
    // every single time. The message-id write-back guarantee that used to
    // ride along with this call (bce95db5) is now countersLoad()'s own job,
    // called separately below, through its own Preferences handle -- see
    // that function's comment for why it is safe to call only after this
    // preferences.end() and does not need save_settings() at all.
    #if !defined(MC_SAFEBOOT)
    if (settings_corrected)
        save_settings();

    countersLoad();
    #endif
}

void clear_flash(void)
{
    // DECISION (W3c, advisor finding 4): only "Credentials" is cleared. The "Counters" namespace
    // (node_msgid, counters_store.h) survives a settings reset on purpose -- a reset is not a reason
    // to replay message ids into every neighbour's dedup ring.
    preferences.begin("Credentials", false);

    printfdeb("[INIT]...FLASH #entries %i bevor clear\n", (int)preferences.freeEntries());

    preferences.freeEntries();

    preferences.clear();

    printfdeb("[INIT]...FLASH #entries %i after clear\n", preferences.freeEntries());
    preferences.end();
}

void save_settings(void)
{
    if (g_flash_load_in_progress)
    {
        // D1-04 W3 Task 2 guard: refuse rather than corrupt. See the
        // g_flash_load_in_progress comment above init_flash() for what this
        // is standing in for.
        Serial.printf("[FLASH]...save_settings() REFUSED: init_flash() load still in progress -- NVS not touched\n");
        return;
    }

    preferences.begin("Credentials", false);

    for (size_t i = 0; i < settings_schema::fieldCount(); i++)
    {
        saveFieldToPreferences(settings_schema::fields()[i], &meshcom_settings);
    }

    //printfdeb("[INIT]...FLASH #entries %i after write\n", (int)preferences.freeEntries());
    preferences.end();

    //Test only Serial.println("flash save...");
}
