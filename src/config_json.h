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

#endif
