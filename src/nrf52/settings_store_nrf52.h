/**
 * @file settings_store_nrf52.h
 * @brief nRF52 file backend for the keyed settings store (D1-04 target
 *        architecture, docs/BACKLOG.md OPT-07 sizing).
 *
 * THIS WAVE BUILDS THE MECHANISM ONLY -- it is wired into nothing. No call
 * from init_flash(), no call from any main loop, no call from any command
 * handler. Cutover (replacing the sizeof()-struct raw read/write in
 * nrf52_flash.cpp with calls to settingsStoreSave()/settingsStoreLoad()) is
 * later, bench-hardware-gated wave work: a mistake there costs every node in
 * the fleet its settings, so it does not happen in the same wave that first
 * writes this code.
 *
 * settings_store.h (the codec) and settings_schema.h (the field table, a
 * sibling deliverable of this same campaign) are the two things this file
 * depends on. See settings_store_nrf52.cpp for exactly how the write path
 * gets its atomicity and why the config_json.cpp layout-version gate must
 * NOT appear here.
 */
#pragma once

#ifdef NRF52_SERIES

#include "settings_store.h"

// Outcome of settingsStoreLoad(). A plain bool return can't distinguish "no
// settings file exists yet" (expected on first boot / after a format) from
// "a file exists but nothing in it decoded" -- both would otherwise look
// like the same false. DecodeStats already carries everything interesting
// about a decode that did run (settings_store.h), so this just adds the two
// bits DecodeStats itself has no way to express: whether a file was even
// there, and whether the read that would produce those stats completed.
struct SettingsLoadResult
{
	// false: no settings file existed to open (e.g. first boot after
	// InternalFS.format(), or the live path was never created). stats is
	// default-constructed (all zero) in this case -- decode() never ran.
	bool file_found = false;

	// true: the file existed, was opened and read without a filesystem
	// error, and settings_store::decode() ran over its content. False means
	// the file existed but could not be read (open/read failure, or a
	// malloc failure for the decode buffer) -- stats is meaningless then.
	bool read_ok = false;

	// Only meaningful when read_ok is true. See settings_store.h's
	// DecodeStats for what each count means; in particular a non-zero
	// malformed_lines/unknown_keys count is NOT itself a failure (that is
	// the whole point of the keyed store), only something a caller may want
	// to log.
	settings_store::DecodeStats stats;
};

// Encodes meshcom_settings through settings_schema::fields()/fieldCount()
// and writes the result to the settings file, replacing whatever was there
// before via a temp-file-then-rename sequence (see the .cpp for exactly
// what atomicity that does and does not provide). Returns false, leaving
// the previously-persisted file completely untouched, on any failure:
// allocation failure, settings_store::encode() overflowing the buffer, or a
// filesystem error opening/writing the temp file or renaming it into place.
bool settingsStoreSave();

// Reads the settings file (if any) and decodes it into meshcom_settings via
// settings_schema::fields()/fieldCount(). See SettingsLoadResult above for
// how to read what happened; meshcom_settings fields whose key was missing
// or malformed in the file are left at whatever value the caller already
// had in meshcom_settings before calling this (settings_store.h's decode()
// contract) -- callers that want struct-default fallback behaviour must
// ensure meshcom_settings already holds those defaults before calling this,
// exactly as today's sanitize/default path does.
SettingsLoadResult settingsStoreLoad();

// Removes the keyed store's live file and its temp file (if either exists),
// for flash_reset()'s targeted reset (nrf52_flash.cpp) -- an alternative to
// InternalFS.format(), which would also erase every OTHER file on the
// filesystem. "The file did not exist" is not a failure (expected on a
// first-ever reset); this returns false only when a file that DID exist
// could not be removed, which is flash_reset()'s signal to fall back to
// format().
bool settingsStoreRemove();

// Prints the raw contents of the keyed store to Serial, for bench diagnosis.
// Returns false if the file does not exist or cannot be read.
bool settingsStoreDump(void);

// Generic atomic write: replaces `path`'s content with exactly `len` bytes
// from `data`, via the same temp-file-then-rename sequence settingsStoreSave()
// itself uses -- write the FULL new content to `tmp_path`, verify every byte
// landed, then InternalFS.rename() it onto `path` (one retry on a failed
// rename, with a filesystem inventory logged in between; see
// settingsStoreSave()'s own top comment in the .cpp for exactly what
// guarantee this sequence does and does not provide). Returns false, leaving
// `path` completely untouched, on any failure -- open/write/rename error.
// Used by settingsStoreSave() itself and by src/counters_store.h's nRF52
// implementation (nrf52_flash.cpp) and its legacy-blob-CRC bookkeeping, so
// neither has to duplicate this sequence. NOT reentrant with itself or with
// settingsStoreSave()/settingsStoreLoad(): all three share the one
// `settings_store_file` object this translation unit keeps open only for the
// duration of a single call, matching Adafruit_LittleFS::open()'s own
// documented "only one file open at a time" constraint -- callers on the
// same task (the only place any of this runs) already serialise through it.
bool writeFileAtomic(const char *path, const char *tmp_path, const void *data, size_t len);

// Prints an inventory of the internal filesystem to Serial -- one line per
// file with its size, then a total -- tagged with `reason`.
//
// This exists for one specific open question: a save can fail at the rename
// step (`[SETST];save;rename_failed`), it failed twice on one boot on
// DK5EN-90 on 2026-09-12, and it has not been reproduced since. The leading
// hypothesis is space (the legacy blob, the live store and the temp file all
// present at once on a 28 672 B filesystem in 128 B blocks), and the
// competing one is a transient flash error while the SoftDevice is busy.
// A byte total plus the actual file list separates those two: out of space
// shows up as a total near the ceiling, a transient IO error does not.
// Adafruit_LittleFS exposes no free-space call, so this walks the tree
// (root plus one level, which reaches /adafruit/bond_prph/) using only the
// public File API and its own mutex.
//
// Called on a save failure and from `--dumpsettings`. Not cheap enough to
// call on a healthy save path, and it is not called there.
void settingsStoreReportFilesystem(const char *reason);

#endif // NRF52_SERIES
