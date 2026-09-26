/**
 * CS-03 (BACKLOG.md #3.8h): the node configuration as ONE JSON object --
 * downloadable from the web Config page and uploadable back into a wiped or
 * replaced node, so a node can be provisioned without a human at the display.
 *
 * ---------------------------------------------------------------------------
 * FILE FORMAT
 * ---------------------------------------------------------------------------
 *
 *   {"meshcom_config":{
 *      "layout":  <int>,      settings-layout generation, FLASH_STRUCT_VERSION
 *      "fw":      "<string>", firmware version, SOURCE_VERSION SOURCE_VERSION_SUB
 *      "hw":      <int>,      BOARD_HARDWARE (informational, never checked)
 *      "settings":{ "<nvs key>": "<value>", ... },
 *      "crc32":   "<8 hex digits>"
 *   }}
 *
 * Every value inside "settings" is a JSON STRING, numbers and flags included.
 * That is not decoration: it takes the JSON library's decimal parser out of
 * the loop. ArduinoJson's parser is off by one ulp on plenty of inputs
 * ("0.002000" comes back as 0.0019999998), which would make a restored node
 * differ from the one that was backed up. Quoted values are read back with
 * strtod() instead, which is exact. A hand-written file that uses real JSON
 * numbers or true/false is still accepted -- it just inherits the JSON
 * parser's precision.
 *
 * The keys inside "settings" are the NVS key names the ESP32 uses in
 * save_settings() (esp32_flash.cpp), NOT the C member names -- so an ESP32
 * export and an nRF52 export of the same configuration are comparable
 * key-by-key even though the two platforms persist completely differently
 * (individual NVS entries vs. one struct in a LittleFS file).
 *
 * Beware two historical key/member mismatches that are preserved here on
 * purpose, because changing them would break every existing NVS entry:
 *   "node_ssid"  is node_ossid   (the MeshCom server SSID)
 *   "node_pwd"   is node_opwd
 *   "node_lssid" is node_ssid    (the local WLAN)
 *   "node_lpwd"  is node_pwd
 *
 * SECRETS TRAVEL IN THE FILE (operator decision, 2026-08-30): "node_lpwd" is
 * the WLAN password in clear, "node_webpwd" the web GUI password, "bt_code"
 * the BLE pairing code. That is what makes a restore complete. The download
 * page says so; treat the file like a password store.
 *
 * ---------------------------------------------------------------------------
 * WHAT THE CRC IS AND IS NOT
 * ---------------------------------------------------------------------------
 *
 * CRC-32 (see crc32_util.h) -- tamper-EVIDENT only, deliberately not an HMAC.
 * It catches truncation, a mangled copy/paste and a careless hand edit. It
 * does not and cannot stop anybody who edits the file on purpose: the
 * checksum is recomputable by anyone. Real forgery protection would need a
 * key, and which key (web password? BT code? a fleet key?) is undecided; a
 * per-node key would also make a config non-transferable, which is the whole
 * point of the feature.
 *
 * ---------------------------------------------------------------------------
 * CANONICAL FORM -- exactly what the CRC is computed over
 * ---------------------------------------------------------------------------
 *
 * NOT the raw file bytes. The CRC is computed over a canonical text rebuilt
 * from the *values*, so that JSON whitespace, member order and a reserializing
 * editor do not change it, and so that the exporter and the importer compute
 * it through the same code path from data that is provably the same.
 *
 * The canonical text is the concatenation of these lines, each terminated by
 * a single '\n' (0x0A), in exactly this order, with no other separator, no
 * trailing padding and no BOM:
 *
 *   1. "MC-CFG-1"                 format tag of the canonical form itself
 *   2. "layout=" <layout, %d>
 *   3. "fw=" <fw string, verbatim>
 *   4. "hw=" <hw, %d>
 *   5. for every field of the export table, IN TABLE ORDER, that is present
 *      in "settings":  "<key>" "=" <value, formatted as below>
 *
 * Value formatting (identical on export and on import; the importer parses
 * the JSON value and re-formats it with the same conversion, so a number
 * written as 2, 2.0 or 2e0 all canonicalise the same way):
 *
 *   string / char   the bytes themselves, no quoting, no escaping
 *   int             "%d"
 *   uint32          "%lu"      (value cast to unsigned long)
 *   bool            "1" / "0"
 *   float           the SHORTEST of "%.6g".."%.9g" that strtod() turns back
 *                   into the identical float
 *   double          the SHORTEST of "%.15g".."%.17g" that strtod() turns back
 *                   into the identical double
 *
 * The shortest-round-trip rule is what keeps the file readable (4.24, not
 * 4.23999977) while staying bit-exact; 9 and 17 significant digits are the
 * standard round-trip bounds for float and double.
 *
 * Keys that are absent from "settings" contribute nothing. Keys that this
 * firmware does not know contribute nothing either -- they are ignored and
 * counted, so a file written by a newer firmware still imports.
 *
 * The exporter puts exactly this canonical text into the JSON as the member's
 * string value, so parsing the file back and re-formatting reproduces the
 * canonical text byte for byte -- including after an editor has reindented
 * the file or reordered its members.
 *
 * ---------------------------------------------------------------------------
 * IMPORT RULES
 * ---------------------------------------------------------------------------
 *
 * Checked in this order -- the value check deliberately runs before the
 * checksum, so a hand-edited file names the field that is wrong instead of
 * answering the far less useful "checksum mismatch". Nothing is written
 * until every check has passed, so the order changes only the message:
 *
 *  - "layout" must equal FLASH_STRUCT_VERSION. A file from a different
 *    settings-layout generation is refused, never half-applied. (Note this is
 *    FLASH_STRUCT_VERSION, the layout generation, not FLASH_VERSION, the
 *    build/release stamp -- see configuration_global.h. Comparing the build
 *    stamp would refuse every file as soon as the next release ships, even
 *    though the layout is unchanged.)
 *  - "crc32" must match the canonical form above.
 *  - every value is range-checked BEFORE anything is written. A single bad
 *    value refuses the whole file; meshcom_settings is never left half-applied.
 *  - missing keys keep the node's current value.
 *  - unknown keys are ignored and counted in the notice text.
 *
 * Neither function touches flash. The caller writes the result
 * (save_settings()) and reboots -- see web_functions.cpp.
 *
 * Fields deliberately NOT in the file, although they are persisted:
 *   node_fversion / node_mversion / node_fwversion  flash+firmware bookkeeping;
 *       importing a foreign node_fversion would make the next boot believe the
 *       stored layout is incompatible and wipe the flash.
 *   node_cflash                                     one-shot "wipe at next boot"
 *       trigger, not a setting -- importing a 1 would erase the restore.
 *   the T-Deck-only block (node_map ... node_wifion) per-device display,
 *       keyboard and audio preferences, not portable configuration.
 *   node_date_* / node_age / node_device_eui / valid_mark_*  clock and flash
 *       markers, no NVS key on the ESP32.
 *   everything below the "nicht im Flash" line of the settings struct.
 *
 * Pure C++ apart from the Serial marker; unit-tested natively
 * (test/test_config_json, `pio test -e native_config`).
 */
#ifndef _CONFIG_JSON_H_
#define _CONFIG_JSON_H_

#include <stddef.h>
#include <stdint.h>

/* Big enough for the whole document plus slack; also the upload cap in
 * web_functions.cpp. A full export is roughly 3.5-4 kB. */
#define CONFIG_JSON_MAX 6144

/* Error codes of configImportJson(). 0 == imported. */
#define CFG_IMP_OK              0
#define CFG_IMP_EPARSE        (-1)   /* not JSON, or truncated                 */
#define CFG_IMP_ESHAPE        (-2)   /* no meshcom_config / settings object    */
#define CFG_IMP_ECRC          (-3)   /* crc32 missing or mismatching           */
#define CFG_IMP_ELAYOUT       (-4)   /* layout != FLASH_STRUCT_VERSION         */
#define CFG_IMP_EVALUE        (-5)   /* a value is out of range or too long    */
#define CFG_IMP_EARG          (-6)   /* no input, or larger than CONFIG_JSON_MAX */

/**
 * Serialises the current meshcom_settings into `out` (NUL-terminated).
 *
 * @return number of bytes written excluding the NUL, or 0 if `cap` was too
 *         small (in that case out[0] is set to '\0' when cap > 0).
 */
size_t configExportJson(char *out, size_t cap);

/**
 * Verifies and applies a config file. Writes nothing unless every check
 * passed. Does NOT persist -- the caller calls save_settings().
 *
 * @param in     the JSON document, need not be NUL-terminated
 * @param len    its length in bytes
 * @param err    short human-readable result; on success a notice
 *               ("42 fields, 3 unknown keys ignored"), on failure the reason.
 *               May be NULL.
 * @param errcap size of err
 * @return CFG_IMP_OK (0) or one of the negative CFG_IMP_* codes.
 */
int configImportJson(const char *in, size_t len, char *err, size_t errcap);


/* Moved here from config_json.cpp 2026-09-12 (W3 step 2). The list itself is
 * unchanged and is still the SINGLE source of truth; it lives in the header
 * only so that a second translation unit -- settings_schema.cpp, which builds
 * the persistence descriptors -- can expand the same list instead of keeping
 * a copy. Row semantics, order and CRC form are exactly as before. */
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

/* Same sentinel settings_sanitize.cpp uses (POWER_NOT_SET, :7): "no TX power
 * stored yet". It is outside TX_POWER_MIN..MAX on some boards (RAK4631:
 * 2..22) and must stay importable, otherwise a factory-fresh node cannot
 * restore its own export. Lives here rather than in config_json.cpp because
 * CFG_FIELD_LIST's node_power row references it and that list is now expanded
 * by a second translation unit (settings_schema.cpp) as well -- a local
 * fallback #define there would be a second source of truth for a sentinel
 * value, which is exactly what this campaign is removing. */
#define CFG_POWER_NOT_SET (-20)

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
    X("max_hop_text",  CFG_INT,  max_hop_text,          (double)MAXHOP_TEXT_MIN, (double)MAXHOP_TEXT_MAX, CFG_ESC(0)) \
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
 * have NVS keys; on the nRF52 the spectrum/battery members exist in the
 * shared struct (src/meshcom_settings.h) but are runtime-only there
 * (src/meshcom_settings_runtime.h), and node_disp_rot does not exist at all.
 *
 * nRF52: nothing. send_repeat_time and auto_join (LoRaWAN OTAA leftovers)
 * were removed from the struct in the D1-04 merge. */
#ifdef ESP32
    #define CFG_FIELD_LIST_PLATFORM(X)                                                        \
        X("node_disrot",  CFG_INT, node_disp_rot,          0.0, 270.0,      CFG_NOESC)        \
        X("node_spstart", CFG_FLT, node_specstart,         0.0, 1.0e9,      CFG_NOESC)        \
        X("node_spend",   CFG_FLT, node_specend,           0.0, 1.0e9,      CFG_NOESC)        \
        X("node_spstep",  CFG_FLT, node_specstep,          0.0, 1000.0,     CFG_NOESC)        \
        X("node_spsamp",  CFG_INT, node_specsamples,       0.0, 65535.0,    CFG_NOESC)        \
        X("node_bfakt",   CFG_FLT, node_analog_batt_faktor, CFG_NORANGE,    CFG_NOESC)
#else
    #define CFG_FIELD_LIST_PLATFORM(X)
#endif

/* Deliberately NOT exported (gate decision, 2026-08-30, from the first live
 * round trip on DK5EN-93): node_msgid is the running message-id counter --
 * restoring a backup would rewind it and make fresh messages collide with
 * the dedup ring of every neighbour (it is persisted on its own, see
 * counters_store.h); node_temp/hum/press/temp2/gas/co2 are the last sensor
 * readings, not configuration, and made two exports of an unchanged node
 * differ (since 2026-09-13 they are not persisted at all). */


#endif
