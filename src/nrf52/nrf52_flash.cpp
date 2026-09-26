/**
 * @file flash-nrf52.cpp
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief Initialize, read and write parameters from/to internal flash memory
 * @version 0.1
 * @date 2021-01-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifdef NRF52_SERIES

#include <debugconf.h>
#include <settings_sanitize.h>
#include <msgid_counter.h>
#include <lora_setchip.h>
#include <counters_store.h> // countersLoad()/countersSave() -- node_msgid's own file (D1-04 W3 step 4)
#include <settings_store.h> // settings_store::FieldDescriptor/encode()/decode(), for the counters file below
#include <crc32_util.h>	  // crc32_buf() -- legacy-blob-rewritten detection (D1-04 W3 step 4 addendum, task 7)

#include "WisBlock-API.h"
#include "settings_store_nrf52.h" // settingsStoreLoad()/settingsStoreSave()/writeFileAtomic(), the keyed store (W3/C1 cutover)
#include "ble_settings_v1.h"	  // s_ble_settings_v1, bleSettingsFromV1()/bleSettingsToV1() -- the frozen legacy blob layout

#include <cstddef> // offsetof, for the counters descriptor table below
#include <cstdio>  // snprintf, for the legacy-blob CRC and counters file encodings below
#include <cstdlib> // strtoul, for reading the recorded legacy-blob CRC back

s_meshcom_settings meshcom_settings;

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

const char settings_name[] = "MeshCom-RAK";

File lora_file(InternalFS);

// ---------------------------------------------------------------------------
// Counters store (src/counters_store.h, D1-04 W3 step 4): node_msgid's OWN
// file, so a settings rewrite/restore/BLE write can never rewind it. One
// descriptor, built by hand rather than through settings_schema.h -- that
// table is deliberately schema-driven for the *settings* record and node_msgid
// is deliberately NOT one of its rows (see settings_schema.h's own comment on
// this); this is a second, much smaller keyed record with its own file.
// ---------------------------------------------------------------------------
namespace
{
const char kCountersPath[] = "/counters.txt";
const char kCountersTmpPath[] = "/counters.txt.tmp";

const settings_store::FieldDescriptor kCounterFields[] = {
	{"node_msgid", settings_store::FieldType::I32, offsetof(s_meshcom_settings, node_msgid), 0, false, 0.0, 0.0},
};
constexpr size_t kCounterFieldCount = sizeof(kCounterFields) / sizeof(kCounterFields[0]);

File counters_file(InternalFS);

// The ONE v1-sized scratch image this file uses for the legacy blob: init_flash() reads the blob
// into it, flash_reset() renders its defaults image into it. File-scope static on purpose: the
// nRF52 loop task (which runs setup() and every command handler) has a 4 kB stack
// (LOOP_STACK_SZ, cores/nRF5/main.cpp), and flash_reset() already keeps a 2 kB s_meshcom_settings
// on that stack -- a second 2 kB object there would overflow it. Both users run in the same task
// and never overlap, so one buffer is enough.
s_ble_settings_v1 s_legacyV1Image;

// ---------------------------------------------------------------------------
// Legacy-blob-rewritten detection (addendum to D1-04 W3 step 4, task 7): a
// node downgraded to official firmware -- which only ever writes the raw v1
// blob -- reconfigured there, then upgraded back to this firmware must not
// have that reconfiguration hidden behind an untouched, stale keyed store.
// init_flash() records the CRC32 of the legacy blob's bytes here every time
// it leaves the blob in a KNOWN state (a successful migration, or a
// flash_reset() rewrite); on the next boot, a legacy blob whose CRC no
// longer matches what was recorded means "changed since we last looked",
// which forces the legacy path even when the keyed store would otherwise
// pass its own sanity gate.
// ---------------------------------------------------------------------------
const char kLegacyBlobCrcPath[] = "/legacy_blob.crc";
const char kLegacyBlobCrcTmpPath[] = "/legacy_blob.crc.tmp";

// Absent file and an unparsable one are treated the same way by the caller:
// both mean "no CRC can be trusted", which is the fail-safe direction here --
// the worst outcome is one extra legacy-path boot, never a hidden data loss.
bool readLegacyBlobCrc(uint32_t &out_crc)
{
	File f(InternalFS);
	if (!f.open(kLegacyBlobCrcPath, FILE_O_READ))
	{
		f.close();
		return false;
	}
	char buf[16] = {0};
	int got = f.read(buf, sizeof(buf) - 1);
	f.close();
	if (got < 8)
	{
		return false;
	}
	buf[got] = '\0';
	char *end = nullptr;
	unsigned long v = strtoul(buf, &end, 16);
	if (end == buf)
	{
		return false;
	}
	out_crc = (uint32_t)v;
	return true;
}

// Same temp-file-then-rename atomicity as the settings/counters files
// (writeFileAtomic(), settings_store_nrf52.cpp) -- a torn write here only
// costs one extra legacy-path boot next time, never data, so no return
// value is checked by callers.
void recordLegacyBlobCrc(uint32_t crc)
{
	char buf[16];
	int n = snprintf(buf, sizeof(buf), "%08lX\n", (unsigned long)crc);
	if (n > 0)
	{
		writeFileAtomic(kLegacyBlobCrcPath, kLegacyBlobCrcTmpPath, buf, (size_t)n);
	}
}
} // namespace

// Removes the counters file (and its temp file) so the next countersLoad() starts from whatever
// meshcom_settings.node_msgid already holds. Used by the legacy load path only -- see the call site.
// A missing file is not an error.
static void discardCountersFile()
{
	File f(InternalFS);
	if (f.open(kCountersPath, FILE_O_READ))
	{
		f.close();
		InternalFS.remove(kCountersPath);
	}
	else
	{
		f.close();
	}
	if (f.open(kCountersTmpPath, FILE_O_READ))
	{
		f.close();
		InternalFS.remove(kCountersTmpPath);
	}
	else
	{
		f.close();
	}
}

// countersLoad()/countersSave() -- src/counters_store.h contract, nRF52 half.
void countersLoad()
{
	counters_file.open(kCountersPath, FILE_O_READ);
	if (counters_file)
	{
		uint32_t stored_size = counters_file.size();
		char buf[64];
		size_t to_read = (stored_size < sizeof(buf)) ? (size_t)stored_size : sizeof(buf);
		int got = counters_file.read(buf, (uint16_t)to_read);
		counters_file.close();
		if (got > 0)
		{
			// Absent file: meshcom_settings.node_msgid already holds whatever init_flash()'s settings
			// load left there before this is called -- the legacy v1 blob still carries node_msgid at
			// its frozen offset, so that path leaves a real value here. A pre-merge dev image's keyed
			// settings-store file may still have a stray "node_msgid=NNN" line from when node_msgid was
			// itself a settings_schema row (before this wave); settings_schema.h no longer describes
			// that key, so settingsStoreLoad()'s decode() now reports that line as an unknown key and
			// ignores it -- acceptable: that node's counter starts fresh from whatever the keyed load
			// left in meshcom_settings.node_msgid (0, the struct default), a one-time cost of this
			// migration, not a correctness bug.
			settings_store::decode(kCounterFields, kCounterFieldCount, &meshcom_settings, buf, (size_t)got);
		}
	}
	else
	{
		counters_file.close();
	}

	meshcom_settings.node_msgid = msgIdAfterLoad(meshcom_settings.node_msgid);
	countersSave();
}

bool countersSave()
{
	char buf[64];
	long written =
		settings_store::encode(kCounterFields, kCounterFieldCount, &meshcom_settings, buf, sizeof(buf));
	if (written < 0)
	{
		// Cannot happen for one I32 field in a 64 B buffer today, but handled rather than assumed --
		// see settings_store_nrf52.cpp's own overflow branch for the same discipline.
		Serial.printf("[SETST];counters;save;overflow\n");
		return false;
	}
	return writeFileAtomic(kCountersPath, kCountersTmpPath, buf, (size_t)written);
}

void flash_int_reset(void);

/**
 * @brief Initialize access to nRF52 internal file system
 *
 */
// TM-32 (upstream #661: Brown-out korrumpiert die Einstellungen, Firmware
// stuerzt ab): Radio-Parameter auf Plausibilitaet, Zeichenketten auf
// Terminierung pruefen, bevor irgendetwas damit rechnet oder druckt.
static void sanitize_log(const char *field, const char *oldv, const char *newv)
{
	Serial.printf("[FLASH]...sanitized %s: %s -> %s\n", field, oldv, newv);
}

void sanitize_loaded_settings(void)
{
	RadioLimits lim = { TX_POWER_MIN, TX_POWER_MAX, 400.0e6f, 960.0e6f, 1, 1, max_country };
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

	// CS-01: max_hop_text liegt bereits in der Struktur (die ganze Struktur wird
	// in die Datei geschrieben), aber eine alte Datei traegt dort eine 0, weil das
	// Feld frueher bei jedem Boot ueberschrieben wurde. Plausibilitaet wie beim
	// ESP32; max_hop_pos bleibt bewusst beim Compile-Default (nrf52_main.cpp).
	if(sanitize_max_hop_text(meshcom_settings.max_hop_text, sanitize_log))
		fixed++;

	// Die Struktur wird roh aus der Datei gelesen -- ein fehlender Terminator
	// laesst strlen()/printf ueber das Feld hinauslesen.
	int strings = 0;
	#define SANITIZE_STR(f) do { if(sanitize_cstring(meshcom_settings.f, sizeof(meshcom_settings.f))) { strings++; Serial.printf("[FLASH]...sanitized %s: terminator missing\n", #f); } } while(0)
	SANITIZE_STR(node_call); SANITIZE_STR(node_short); SANITIZE_STR(node_ossid); SANITIZE_STR(node_opwd);
	SANITIZE_STR(node_extern); SANITIZE_STR(node_atxt); SANITIZE_STR(node_passwd); SANITIZE_STR(node_ownip);
	SANITIZE_STR(node_owngw); SANITIZE_STR(node_ownms); SANITIZE_STR(node_name); SANITIZE_STR(node_webpwd);
	SANITIZE_STR(node_ssid); SANITIZE_STR(node_pwd); SANITIZE_STR(node_parm); SANITIZE_STR(node_unit);
	SANITIZE_STR(node_format); SANITIZE_STR(node_eqns); SANITIZE_STR(node_values); SANITIZE_STR(node_lora_call);
	SANITIZE_STR(node_gwsrv); SANITIZE_STR(node_owndns); SANITIZE_STR(node_ownntp); SANITIZE_STR(node_fwversion);
	SANITIZE_STR(node_via); SANITIZE_STR(node_aprsmc); SANITIZE_STR(node_pingcall); SANITIZE_STR(node_ip);
	SANITIZE_STR(node_dns); SANITIZE_STR(node_gw); SANITIZE_STR(node_subnet);
	SANITIZE_STR(node_update); SANITIZE_STR(node_parm_1); SANITIZE_STR(node_parm_t); SANITIZE_STR(node_parm_id);
	#undef SANITIZE_STR

	// D1-04 W3 step 4: the message-id high-water mark used to be advanced and written back right
	// here, piggy-backing on this function's own unconditional save (that is why the save below used
	// to be unconditional). It now has its own file and its own load/save pair (src/counters_store.h,
	// implemented in this file above) -- init_flash() calls countersLoad() right after this function
	// returns, on every load path, and that call does the msgIdAfterLoad() step and its own
	// write-back. Nothing here touches node_msgid any more.

	if(fixed > 0 || strings > 0)
	{
		Serial.printf("[FLASH]...%d setting(s), %d string(s) corrected\n", fixed, strings);
		// One write only when there was something to correct: this file's settings save is no longer
		// unconditional on every boot now that the message-id write-back (the other half of the old
		// "always" reasoning) moved to countersSave() above, which does its own unconditional
		// per-boot write to its own file instead.
		save_settings();
	}
}

void init_flash(void)
{
	if (init_flash_done)
	{
		return;
	}

	// Initialize Internal File System
	InternalFS.begin();

	// -------------------------------------------------------------------------------------------
	// Task 7 (addendum): does the legacy blob's CURRENT content match the CRC recorded the last
	// time this firmware left it in a known state (a migration, or a flash_reset() rewrite)? If not
	// -- including "no CRC was ever recorded" -- the blob changed since, most plausibly a downgrade
	// to official firmware (which only ever writes this blob) followed by reconfiguration there, and
	// the keyed store below must not be allowed to hide that behind its own, now-stale, fast path.
	//
	// Read here, once, into the same function-scope static buffer step (b)/(c) below reuses for the
	// real load: a legacy blob is always exactly sizeof(s_ble_settings_v1) bytes when it is valid
	// (both this firmware's own flash_reset() and official firmware write exactly that many), so
	// this read and the later one see identical bytes whenever nothing in between rewrites the file
	// -- the only overhead is one extra small flash read at boot.
	//
	// DECISION (W3c, advisor finding 3): "no CRC ever recorded" is treated like "changed", i.e. the
	// blob wins. That is right for every node in the field (no released image ever wrote a keyed
	// store, so the blob is always the newer truth on the first boot of this firmware) and for
	// RAK-90's bench state. It is wrong only for a dry-unification dev image older than W3c that was
	// never re-flashed with a release image: its keyed store is newer than its blob and gets
	// overwritten by the blob once, on the first W3c boot. Accepted: a dev-bench-only, one-time
	// cost, versus silently hiding a user's reconfiguration on the fleet path.
	// -------------------------------------------------------------------------------------------
	s_ble_settings_v1 &legacy_v1_image = s_legacyV1Image; // file-scope static: kept off the stack (see Task 1)
	bool legacy_file_present = lora_file.open(settings_name, FILE_O_READ);
	uint32_t legacy_stored_size = legacy_file_present ? lora_file.size() : 0;
	if (legacy_file_present)
	{
		lora_file.read((uint8_t *)&legacy_v1_image, sizeof(s_ble_settings_v1));
	}
	lora_file.close();

	bool legacy_rewritten = false;
	if (legacy_file_present && legacy_stored_size == sizeof(s_ble_settings_v1))
	{
		uint32_t blob_crc = crc32_buf(&legacy_v1_image, sizeof(s_ble_settings_v1));
		uint32_t recorded_crc = 0;
		legacy_rewritten = !readLegacyBlobCrc(recorded_crc) || (recorded_crc != blob_crc);
	}
	else if (legacy_file_present)
	{
		// A present-but-wrong-sized blob is not something the CRC gate can meaningfully compare --
		// step (b)/(c) below already resets it to defaults regardless of which path is taken here, so
		// forcing the legacy path (fail-safe direction) costs nothing beyond one extra boot there.
		legacy_rewritten = true;
	}

	// -------------------------------------------------------------------------------------------
	// Step (a): try the keyed store first (W3/C1 cutover, docs/BACKLOG.md OPT-07/D1-04).
	//
	// meshcom_settings is a file-scope global (declared above with its member-initialisers), so by
	// the time init_flash() ever runs it already holds full struct defaults -- exactly the
	// precondition settings_store.h's decode() contract requires of the caller: decode() only
	// OVERWRITES the fields whose key it actually finds a valid line for, it never zeroes or
	// resets `state` first.
	// -------------------------------------------------------------------------------------------
	SettingsLoadResult keyed = settingsStoreLoad();

	// Sanity gate -- THIS is what protects the fleet's configuration on this cutover.
	// settings_store::decode() deliberately never fails as a whole (settings_store.h, "no
	// whole-decode failure"): a truncated or corrupted keyed-store file would leave almost every
	// field exactly at the default meshcom_settings already held, and decode() would still report
	// that as a normal, completed read. Accepting that blindly would look like a successful boot,
	// and the RIGHT thing to do after a successful load is to never touch the legacy blob again --
	// so a false accept here would go undetected until someone notices the node forgot its call
	// sign, radio settings, everything. Two independent, cheap checks must BOTH hold before the
	// keyed store is trusted:
	//   - stats.fields_set > 0: at least one line in the file matched a known key and actually
	//     decoded into meshcom_settings. An empty file, or one that is 100% unknown/malformed
	//     lines, reports fields_set == 0 and is indistinguishable from "nothing was read" -- this
	//     is exactly that case, not a corrupted-but-real file.
	//   - node_call is non-empty: the call sign is the one field every real, ever-configured node
	//     has non-blank (it is set on first join and never cleared), so its absence after a decode
	//     that claims to have set fields is a strong, single-field-cheap signal that the store is
	//     not what it appears to be.
	// Either check failing falls through to the legacy path below rather than straight to
	// flash_reset() -- the legacy blob (if present) is still the best source of truth available,
	// and step (b)/(c) below already knows how to fall back further to defaults on its own.
	//
	// Task 7 (addendum): `!legacy_rewritten` gates the fast path too -- a legacy blob whose CRC no
	// longer matches what was last recorded must never be shadowed by an otherwise-healthy keyed
	// store; see the CRC pre-check block above.
	if (!legacy_rewritten && keyed.file_found && keyed.read_ok && keyed.stats.fields_set > 0 &&
		meshcom_settings.node_call[0] != '\0')
	{
		DEBUG_MSG("FLASH", "Loaded settings from keyed store (%u fields set, %u unknown keys, %u malformed lines)",
				  (unsigned)keyed.stats.fields_set, (unsigned)keyed.stats.unknown_keys,
				  (unsigned)keyed.stats.malformed_lines);
		// Permanent diagnostic (not gated by DO_DEBUG, see debugconf.h): which load path this boot
		// took, plus the DecodeStats a keyed load produced -- see docs/w3-settings-verdict.md's
		// "sharpest open lead", which this line exists to make reproducible on the console.
		Serial.printf("[SETST];path;keyed;fields_set=%u;unknown_keys=%u;malformed_lines=%u;lines_total=%u\n",
					  (unsigned)keyed.stats.fields_set, (unsigned)keyed.stats.unknown_keys,
					  (unsigned)keyed.stats.malformed_lines, (unsigned)keyed.stats.lines_total);
		sanitize_loaded_settings();
		countersLoad();
		log_settings();
		init_flash_done = true;
		return;
	}

	if (legacy_rewritten)
	{
		// Printed whether or not the keyed store would otherwise have passed its own sanity gate --
		// it is the reason the fast path above was refused, distinct from (and possibly alongside)
		// the sanity-gate-rejected / keyed_absent lines just below.
		Serial.printf("[SETST];path;legacy_rewritten\n");
	}

	if (keyed.file_found)
	{
		DEBUG_MSG("FLASH",
				  "*** Keyed settings store present but failed the sanity gate (read_ok=%d fields_set=%u "
				  "node_call=\"%s\") -- falling back to legacy load ***",
				  keyed.read_ok ? 1 : 0, (unsigned)keyed.stats.fields_set, meshcom_settings.node_call);
		Serial.printf("[SETST];path;sanity_gate_rejected;read_ok=%d;fields_set=%u;unknown_keys=%u;"
					  "malformed_lines=%u;node_call_empty=%d\n",
					  keyed.read_ok ? 1 : 0, (unsigned)keyed.stats.fields_set, (unsigned)keyed.stats.unknown_keys,
					  (unsigned)keyed.stats.malformed_lines, meshcom_settings.node_call[0] == '\0' ? 1 : 0);
	}
	else
	{
		Serial.printf("[SETST];path;keyed_absent\n");
	}

	// -------------------------------------------------------------------------------------------
	// Step (b)/(c): legacy load path -- UNCHANGED IN BEHAVIOUR from before this wave, except that
	// the pre-compat (marker byte 0x57) branch that used to map the old compat struct field-by-field
	// is gone (see below, the compat struct and its marker macro are retired along with it) and a
	// one-time migration into the keyed store runs once meshcom_settings is populated.
	// -------------------------------------------------------------------------------------------

	// Check if file exists
	lora_file.open(settings_name, FILE_O_READ);
	if (!lora_file)
	{
		DEBUG_MSG("FLASH", "File doesn't exist, force format");
		Serial.printf("[SETST];flash_reset_reason;legacy_file_missing\n");
		delay(1000);
		flash_reset();
		lora_file.open(settings_name, FILE_O_READ);
	}

	uint8_t markers[2] = {0};
	lora_file.read(markers, 2);
	if ((markers[0] == 0xAA) && (markers[1] == 0x57))
	{
		// 0x57 was the marker byte of a settings file written by a firmware generation old enough to
		// predate even the compat struct that used to map this exact on-disk layout field-by-field
		// into meshcom_settings -- that compat struct and its marker-byte macro are retired (this
		// wave) along with the whole code path they served, so nothing left in this firmware can
		// decode that layout any more. Unlike the size-checked blit path below (which still validates
		// the current marker/stored_size before trusting the bytes it just read),
		// there is no struct left here to read this file INTO, so there is nothing safe to do with
		// its bytes except discard them: reset to defaults, exactly like the "file doesn't exist"
		// case above. A node still carrying this pre-compat format after all this time losing its
		// settings on this one firmware upgrade is a visible, one-time, acceptable outcome; silently
		// reinterpreting its bytes as the current struct layout -- the thing this comment exists to
		// rule out -- would not be.
		DEBUG_MSG("FLASH",
				  "*** File has pre-compat structure (marker 0x57); no firmware can read this layout any "
				  "more, resetting to defaults ***");
		Serial.printf("[SETST];flash_reset_reason;precompat_marker\n");
		lora_file.close();
		flash_reset();
		lora_file.open(settings_name, FILE_O_READ);
	}

	// Found current-format structure (either the file as it was, or freshly-written defaults from
	// a flash_reset() call just above). The legacy blob on every field node is a v1 image
	// (s_ble_settings_v1, src/nrf52/ble_settings_v1.h) -- the orchestrator verified it byte-identical
	// to what official firmware v4.35t still writes as its raw settings blob -- NOT a raw blit of the
	// live s_meshcom_settings any more, since that struct's layout is now free to change (D1-04).
	// Read into the frozen v1 type, validate IT, then translate member-by-member into
	// meshcom_settings via bleSettingsFromV1() (ble_settings_v1.cpp).
	lora_file.close();
	lora_file.open(settings_name, FILE_O_READ);
	// Groesse VOR dem Lesen sichern -- close() unten macht sie nicht mehr abfragbar.
	uint32_t stored_size = lora_file.size();
	lora_file.read((uint8_t *)&legacy_v1_image, sizeof(s_ble_settings_v1));
	lora_file.close();

	// Check if it is LPWAN settings^
	// Groessen-Check (N-12): eine Struktur-Layout-Aenderung faellt hier auf, wenn die abgelegte
	// Datei nicht mehr sizeof(s_ble_settings_v1) gross ist -- sonst wuerde stillschweigend mit
	// verschobenen/fehlinterpretierten Feldern weitergearbeitet. s_ble_settings_v1 ist FROZEN (siehe
	// ble_settings_v1.h), also ist dieser Vergleichswert selbst ein Fixpunkt, kein bewegliches Ziel
	// mehr wie es sizeof(s_meshcom_settings) vor D1-04 war. Der 0x57-Pfad oben verlaesst sich bewusst
	// NICHT auf diese Pruefung (er resettet direkt), aber valid_mark_2 sitzt im alten wie im neuen
	// Layout am selben Offset (Byte 1), sodass ein 0x57-Datensatz, der die obige Erkennung je
	// verfehlen sollte, hier ueber den Marker-Vergleich ebenfalls als ungueltig erkannt wuerde --
	// Verteidigung in der Tiefe, kein Ersatz fuer die explizite Erkennung oben.
	if (!bleSettingsV1MarkersOk(legacy_v1_image) || (stored_size != sizeof(s_ble_settings_v1)))
	{
		// Daten sind ungueltig oder die Dateigroesse passt nicht mehr zum frozen v1 Layout:
		// sauberer Reset auf Defaults statt stillem Weiterlaufen mit Datenmuell (N-12).
		// flash_reset() invalidiert dabei init_flash_done, ein erneuter Aufruf hier ist daher
		// nicht noetig und wuerde auch keine Rekursion ausloesen -- wir lesen direkt die frisch
		// geschriebene Default-Datei zurueck (einmaliger Reset, kein Retry-Loop).
		DEBUG_MSG("FLASH", "Invalid data set or size mismatch, resetting to defaults");
		Serial.printf("[SETST];flash_reset_reason;invalid_marker_or_size;stored_size=%lu;expected=%u\n",
					  (unsigned long)stored_size, (unsigned)sizeof(s_ble_settings_v1));
		flash_reset();
		lora_file.open(settings_name, FILE_O_READ);
		lora_file.read((uint8_t *)&legacy_v1_image, sizeof(s_ble_settings_v1));
		lora_file.close();
	}
	bleSettingsFromV1(legacy_v1_image, meshcom_settings);
	// bleSettingsFromV1() skips node_msgid on purpose (a BLE write must not rewind the counter --
	// ble_settings_v1.cpp). Here the blob IS the only source of the counter, so copy it explicitly.
	meshcom_settings.node_msgid = legacy_v1_image.node_msgid;
	sanitize_loaded_settings();
	// Legacy path == the blob is the newest truth (absent keyed store, or a blob rewritten since we
	// last looked -- official firmware writes it on every save and advances node_msgid in it). A
	// counters file left behind by an earlier run of this firmware is therefore STALE and would
	// rewind the counter below the value the blob carries (advisor finding 2, W3c). Discard it so
	// countersLoad() starts from the blob's value and writes that back.
	discardCountersFile();
	countersLoad();
	log_settings();

	// One-time migration into the keyed store: meshcom_settings now holds real settings, sourced
	// either from the legacy blob just read above or from flash_reset()'s freshly-written defaults
	// -- both step (b) and step (c) from the brief converge here. This only runs when the keyed
	// store did NOT already satisfy the sanity gate above (that path already returned). Deliberately
	// NOT deleting or blanking the legacy file afterwards -- see save_settings() below for why the
	// legacy blob is left on the filesystem untouched from here on.
	if (settingsStoreSave())
	{
		DEBUG_MSG("FLASH", "*** Migrated legacy settings into the keyed store ***");
		Serial.printf("[SETST];path;legacy_migrated\n");
	}
	else
	{
		DEBUG_MSG("FLASH",
				  "*** Migration of legacy settings into the keyed store FAILED -- still running on the "
				  "legacy struct blob this boot ***");
		Serial.printf("[SETST];path;legacy_migration_failed\n");
	}

	// Task 7 (addendum): legacy_v1_image at this point is exactly what is now on disk at
	// settings_name (the blob as it was, or a fresh defaults rewrite by flash_reset() above) --
	// record its CRC so the NEXT boot's pre-check (top of this function) can tell whether the blob
	// changed in between, regardless of whether the migration just above actually succeeded.
	recordLegacyBlobCrc(crc32_buf(&legacy_v1_image, sizeof(s_ble_settings_v1)));

	init_flash_done = true;
}

/**
 * @brief Save changed settings if required
 *
 * @return boolean
 * 			result of saving
 */
boolean save_settings(void)
{
	// Keyed store is now the persisted format (see init_flash()'s step (a)/migration above).
	// settingsStoreSave() is all-or-nothing and atomic (temp file, verified write, then an
	// InternalFS.rename() onto the live path -- see settings_store_nrf52.cpp for exactly what
	// guarantee that does and does not provide), AND it keeps the old raw blit's
	// skip-if-unchanged behaviour: it compares the freshly encoded record against the stored file
	// and returns without writing when they match. That guard is not optional here. This function
	// has 150+ call sites (command_functions.cpp, phone_commands.cpp, loop_functions.cpp,
	// event_functions.cpp) and nothing constrains them to call only on an actual change -- which
	// is presumably why the blit version read its own file back and memcmp'd it before writing.
	//
	// Deliberately NOT touching the legacy blob (settings_name / lora_file) here any more, and
	// deliberately NOT deleting it either. Silently keeping both formats in sync from here on would
	// require writing the legacy blob on every save forever, which is exactly the raw-struct
	// fragility (N-12: any layout change wipes the file) this cutover exists to get away from --
	// that part of the original reasoning stands.
	//
	// WITHDRAWN CLAIM (Fable verdict, docs/w3-settings-verdict.md, Finding 2): an earlier version of
	// this comment claimed that leaving the legacy blob in place lets a downgrade to an older
	// firmware recover the node's PRE-migration settings. That is false and must not be
	// reintroduced: flash_reset() below overwrites the legacy blob with a FRESH DEFAULTS record every
	// time it runs, including on ordinary reset paths that run AFTER this file's migration has
	// already happened (e.g. nrf52_main.cpp's layout-mismatch check, reachable on a boot that
	// loaded the keyed store fine). Nothing after that point ever writes real settings back into the
	// legacy blob, so a downgrade reads defaults, not the node's prior configuration -- the blob
	// surviving is an artifact of not bothering to delete it, not a safety net.
	//
	// The real recovery story for a bad migration or a bad save is an operator-triggered
	// configExportJson() backup (src/config_json.cpp) taken BEFORE an upgrade that changes this
	// store's format -- see docs/bench/w3-baseline/ for what that looks like in practice. There is
	// currently no on-device fallback that recovers pre-migration settings automatically.
	bool result = settingsStoreSave();

	log_settings();

	return result;
}

/**
 * @brief Reset content of the filesystem
 *
 */
// Callers (in and outside this file): nrf52_main.cpp on a FLASH_STRUCT_VERSION layout mismatch
// (reachable AFTER init_flash() already returned successfully via the keyed path -- W3 verdict
// Finding 2), init_flash() itself on a missing/pre-compat/invalid legacy file, and command
// handlers. flash_reset()'s own signature (WisBlock-API.h, out of this wave's file set) carries no
// reason argument, so the reason is logged by each call site immediately before calling this, in
// the same [SETST] marker style; this function logs only that it was entered and what it did.
void flash_reset(void)
{
	Serial.printf("[SETST];flash_reset;enter\n");

	// Targeted removal instead of InternalFS.format() (Fable verdict, docs/w3-settings-verdict.md,
	// Finding 2): format() erases EVERY file on the internal flash filesystem, not just this
	// function's own two.
	//
	// The worst of that is NOT the settings, and it is worth naming because nothing else in the
	// tree records it: BLE bonds live on this same filesystem, under BOND_DIR_PRPH
	// ("/adafruit/bond_prph/", the Adafruit nRF52 core's bonding.cpp), so every format() silently
	// unpaired every phone that had ever bonded with the node. That damage is invisible from the
	// settings console, survives no backup, and can only be undone by re-pairing each device by
	// hand. A settings reset has no business doing it.
	//
	// It also erased the keyed store (settings_store_nrf52.cpp) that init_flash()'s
	// step (a) may already have trusted and returned through on THIS boot before something later
	// (e.g. the layout-mismatch check in nrf52_main.cpp) calls flash_reset(). Wiping that store just
	// to rewrite one legacy defaults blob destroyed data this function has no business touching, and
	// left `save_settings()`'s downgrade-safety-net comment describing a guarantee the code did not
	// keep (withdrawn above). Remove exactly the two files this function is responsible for instead.
	// Existence check via open()-then-close, not InternalFS.exists(): both the real Adafruit_LittleFS
	// and the native test double (test/test_nrf52_settings_paths/stubs/Adafruit_LittleFS.h) already
	// implement File::open()'s falsy-on-missing-file behaviour -- the same thing `if (!lora_file)`
	// above already relies on -- so this needs no test-fixture change.
	bool legacy_existed = lora_file.open(settings_name, FILE_O_READ);
	lora_file.close();
	bool legacy_removed = !legacy_existed || InternalFS.remove(settings_name);
	bool keyed_removed = settingsStoreRemove(); // also removes the keyed store's temp file, if any
	// DECISION (W3c, advisor finding 4): the counters file is deliberately NOT removed here. A
	// settings reset is not a reason to replay message ids into every neighbour's dedup ring; the
	// counter keeps running across the reset (before W3c it restarted at 0 -> 100, which was a
	// side effect of the counter living inside the settings blob, not a feature).

	bool wrote_defaults = false;
	if (lora_file.open(settings_name, FILE_O_WRITE))
	{
		// default_settings ist default-konstruiert -> die Member-Initializer in s_meshcom_settings
		// setzen gueltige Werte. Geschrieben wird aber das FROZEN v1 Abbild davon (bleSettingsToV1(),
		// ble_settings_v1.cpp), nicht die Live-Struktur selbst -- so bleibt die Datei fuer ein
		// Downgrade auf offizielle Firmware groessenkorrekt lesbar (Task 7 s.u.: die CRC dieses frisch
		// geschriebenen Abbilds wird gleich danach festgehalten, sobald meshcom_settings selbst wieder
		// gefuellt ist -- siehe init_flash()).
		s_meshcom_settings default_settings;
		s_ble_settings_v1 &default_image = s_legacyV1Image; // shared static, see its definition above
		bleSettingsToV1(default_settings, default_image);
		size_t put = lora_file.write((uint8_t *)&default_image, sizeof(s_ble_settings_v1));
		lora_file.flush();
		lora_file.close();
		wrote_defaults = (put == sizeof(s_ble_settings_v1));
		if (wrote_defaults)
		{
			recordLegacyBlobCrc(crc32_buf(&default_image, sizeof(s_ble_settings_v1)));
		}
	}

	// InternalFS.format() stays as a genuine fallback, not the normal path: it is the one operation
	// guaranteed to leave the filesystem in a known, mountable state, and is worth reaching for only
	// when the targeted removal above or the rewrite that must follow it could not proceed -- a
	// filesystem that cannot delete its own files or take a fresh write of a few hundred bytes is
	// suspect enough that starting over is safer than continuing to poke at it file-by-file.
	if (!legacy_removed || !keyed_removed || !wrote_defaults)
	{
		Serial.printf("[SETST];flash_reset;fallback_format;legacy_removed=%d;keyed_removed=%d;wrote_defaults=%d\n",
					  legacy_removed ? 1 : 0, keyed_removed ? 1 : 0, wrote_defaults ? 1 : 0);
		InternalFS.format();
		if (lora_file.open(settings_name, FILE_O_WRITE))
		{
			s_meshcom_settings default_settings;
			s_ble_settings_v1 &default_image = s_legacyV1Image; // shared static, see its definition above
			bleSettingsToV1(default_settings, default_image);
			size_t put = lora_file.write((uint8_t *)&default_image, sizeof(s_ble_settings_v1));
			lora_file.flush();
			lora_file.close();
			wrote_defaults = (put == sizeof(s_ble_settings_v1));
			if (wrote_defaults)
			{
				recordLegacyBlobCrc(crc32_buf(&default_image, sizeof(s_ble_settings_v1)));
			}
		}
	}

	Serial.printf("[SETST];flash_reset;done;wrote_defaults=%d\n", wrote_defaults ? 1 : 0);

	// Cache invalidieren: ohne dies ist der naechste init_flash()-Aufruf ein No-Op (Guard oben),
	// und save_settings() wuerde die alte RAM-Kopie zurueckschreiben, obwohl das Log einen
	// sauberen Reset meldet (N-12). flash_reset() muss also erzwingen, dass frisch von Flash
	// gelesen wird.
	init_flash_done = false;
}

/**
 * @brief Printout of all settings
 *
 */
void ble_log_settings(void)
{
	g_ble_uart.printf("Saved settings:");
	delay(50);
	g_ble_uart.printf("Marks: %02X %02X", meshcom_settings.valid_mark_1, meshcom_settings.valid_mark_2);
	delay(50);
}

#endif
