/**
 * @file settings_store_nrf52.cpp
 * @brief nRF52 file backend for the keyed settings store. See
 *        settings_store_nrf52.h for scope (mechanism only, wired into
 *        nothing yet) and settings_store.h for the codec contract this
 *        wraps.
 */
#ifdef NRF52_SERIES

#include "settings_store_nrf52.h"

#include <cstdlib>
#include <cstdio>    // snprintf, for the filesystem inventory's path names
#include <cstring>   // memcmp, for the skip-if-unchanged compare below

#include <debugconf.h>

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

#include "WisBlock-API.h" // extern s_meshcom_settings meshcom_settings;
#include "settings_schema.h" // settings_schema::fields() / fieldCount()

namespace
{

// Worst-case encoded size for the full persist set, string-keyed. The
// docs/opt07-nrf52-settings-store-sizing-20260912.md figure of 2 937 B (cited
// against a 4096 B cap as "50x margin" in docs/BACKLOG.md OPT-07) is
// SUPERSEDED: it maximised only the STRING fields and left numerics at their
// struct defaults. The Fable verdict (docs/w3-settings-verdict.md, Finding 1)
// re-measured with BOTH axes at their worst case together -- every STRING
// field filled to capacity with backslashes (each escapes to two bytes on
// the wire, settings_store.h's ENCODE CONTRACT) AND every numeric field at
// its maximum-width representation (a `double` at "%.17g" is ~24 characters,
// e.g. "-1.2345678901234567e+308") -- and measured 4 036 B, 60 B under the
// OLD 4096 B cap.
//
// That 4 036 B figure is itself already stale, and the reason matters: the
// same wave then restored node_msgid and node_ackid to the schema, taking the
// table from 107 to 109 rows and the worst case to **4 082 B** -- 14 B under
// the old cap. (W3c later took both rows out again -- node_ackid is gone from
// the struct, node_msgid lives in its own counters file -- so the live worst
// case is back near 4 036 B; the 8192 B cap below was sized against 4 082 B
// and is not re-measured on every row change, which is the point of the
// margin.) Re-derived independently twice (orchestrator and advisor) and
// in agreement. Two routine rows consumed three quarters of what looked like
// headroom, which is the whole argument for not sizing this to just clear the
// measurement. Re-measure on any schema change; the sweep lives in the Fable
// verdict. That is not a margin, it is a live overflow risk
// for any node whose strings and numerics are both long: encode() would
// return -1, settingsStoreSave() would return false, and 234 save_settings()
// call sites across the tree ignore that return value (DO_DEBUG 0 compiles
// out the DEBUG_MSG that would have said why) -- the setting takes in RAM
// and silently reverts on the next reboot. Raised to 8192 B: a round power
// of two giving ~2x margin over the 4 082 B measured worst case, i.e. real
// headroom rather than a number chosen to just clear it. This buffer is
// heap-allocated per call (settingsStoreSave()/settingsStoreLoad()), never
// held for the file's lifetime, and free RAM on wiscore_rak4631 is ~165 kB
// (docs/bench/w3-baseline/), so the extra 4096 B here costs nothing that
// matters.
constexpr size_t kSettingsBufferCap = 8192;

const char kSettingsPath[] = "/MeshCom-Settings-Store";
const char kSettingsTmpPath[] = "/MeshCom-Settings-Store.tmp";

// Adafruit_LittleFS::open()'s own doc comment: "Note that currently only
// one file can be open at a time." One File object, opened and closed
// around each single use, mirrors the `lora_file` idiom nrf52_flash.cpp
// already uses for the very same reason.
File settings_store_file(InternalFS);

// True when `path` exists and holds exactly `len` bytes equal to `data`.
//
// Compared in 64 B chunks against a stack buffer rather than by slurping the
// file into a second heap allocation: the encoded record is already
// kSettingsBufferCap bytes of heap, and both callers run on tasks with a 4 KB
// stack.
//
// Two callers, deliberately sharing one implementation: the unchanged-guard in
// settingsStoreSave() (is a write needed at all?) and the false-negative check
// in writeFileAtomic() (did the write land despite the error?). Both are asking
// the same question of the filesystem -- "does this path already hold exactly
// these bytes" -- and only the conclusion drawn differs.
bool fileHasExactContent(const char *path, const void *data, size_t len)
{
	if (!settings_store_file.open(path, FILE_O_READ))
		return false;

	bool identical = (settings_store_file.size() == (uint32_t)len);
	if (identical)
	{
		const uint8_t *want_bytes = (const uint8_t *)data;
		uint8_t chunk[64];
		size_t off = 0;
		while (off < len)
		{
			size_t want = (len - off) < sizeof(chunk) ? (len - off) : sizeof(chunk);
			int got = settings_store_file.read(chunk, (int)want);
			if (got != (int)want || memcmp(chunk, want_bytes + off, want) != 0)
			{
				identical = false;
				break;
			}
			off += (size_t)got;
		}
	}
	settings_store_file.close();
	return identical;
}

} // namespace

bool settingsStoreSave()
{
	char *buf = (char *)malloc(kSettingsBufferCap);
	if (buf == nullptr)
	{
		DEBUG_MSG("SETST", "save: malloc(%u) failed", (unsigned)kSettingsBufferCap);
		Serial.printf("[SETST];save;malloc_failed;cap=%u\n", (unsigned)kSettingsBufferCap);
		return false;
	}

	long written = settings_store::encode(settings_schema::fields(), settings_schema::fieldCount(),
										   &meshcom_settings, buf, kSettingsBufferCap);
	if (written < 0)
	{
		// encode() is all-or-nothing (settings_store.h): a negative return means the full record
		// did not fit kSettingsBufferCap and NOTHING was written to buf. Growing kSettingsBufferCap
		// is the fix if this ever fires for real; it means the schema grew past the OPT-07 sizing
		// headroom, not that this call site is doing anything wrong.
		DEBUG_MSG("SETST", "save: encode() overflowed the %u B buffer", (unsigned)kSettingsBufferCap);
		// Permanent diagnostic, not gated by DO_DEBUG (see debugconf.h): this is exactly the silent
		// failure mode of Fable verdict Finding 1 -- settingsStoreSave() is about to return false and
		// none of save_settings()'s 234 call sites check it, so this line is the only trace this
		// overflow leaves anywhere.
		Serial.printf("[SETST];save;overflow;cap=%u\n", (unsigned)kSettingsBufferCap);
		free(buf);
		return false;
	}
	Serial.printf("[SETST];save;encoded;bytes=%ld\n", written);

	// ---------------------------------------------------------------------
	// Skip the write entirely when the encoded content is byte-identical to
	// what is already on the filesystem.
	//
	// This is NOT an optimisation bolted on: the raw-blit save_settings()
	// this store replaces read the stored struct back and did a memcmp
	// before writing ("Flash content changed, writing new data"), and that
	// guard has to survive the cutover. Its existence is the evidence for
	// why: save_settings() has 150+ call sites across command_functions.cpp,
	// phone_commands.cpp, loop_functions.cpp and event_functions.cpp, and
	// nothing makes them call only on an actual change. Dropping the guard
	// would turn every one of those calls into a temp-file write plus a
	// rename on the nRF52's internal flash -- a wear increase that would
	// show up in the field, months later, as a filesystem that stopped
	// taking writes.
	//
	// Compared in chunks against a small stack buffer rather than by slurping
	// the file into a second heap allocation: the encoded record is already
	// kSettingsBufferCap bytes of heap, and this runs on the main-loop task
	// whose stack is 4 KB.
	// ---------------------------------------------------------------------
	if (fileHasExactContent(kSettingsPath, buf, (size_t)written))
	{
		Serial.printf("[SETST];save;skipped_unchanged;bytes=%ld\n", written);
		free(buf);
		return true;
	}

	// Atomicity from here on is writeFileAtomic()'s job (factored out so
	// src/counters_store.h's nRF52 implementation and the Task 7 legacy-blob
	// CRC file can share it instead of duplicating the sequence) -- see that
	// function for exactly what guarantee it does and does not provide.
	bool ok = writeFileAtomic(kSettingsPath, kSettingsTmpPath, buf, (size_t)written);
	free(buf);
	return ok;
}

bool writeFileAtomic(const char *path, const char *tmp_path, const void *data, size_t len)
{
	// ---------------------------------------------------------------------
	// Write the FULL new content to a temp path, verify every byte of it
	// landed, and only then replace the live path -- the live path itself is
	// never opened for writing. Adafruit_LittleFS exposes a real rename()
	// (Adafruit_LittleFS.h), and littlefs's own lfs_rename() (littlefs/lfs.h)
	// both replaces an existing destination of matching type and is a single
	// atomic metadata update -- littlefs is a power-loss-safe filesystem by
	// design, so that swap step has no partial-write window: after any
	// reset, `path` is either the old content or the new content, never a
	// mix.
	//
	// The temp-file WRITE ahead of the rename is NOT covered by that
	// guarantee -- a power loss while writing `tmp_path` can leave a
	// partial/corrupt temp file. That is the window this sequence leaves
	// open, and it is the safe one to leave open: `path` (the file a caller's
	// own load function actually reads) is untouched throughout, so the
	// worst case is one lost save attempt, never a corrupted live file. The
	// stale temp file left behind is silently overwritten by the next call
	// to this function for the same paths (removed below, then reopened for
	// write).
	// ---------------------------------------------------------------------
	InternalFS.remove(tmp_path);

	bool write_ok = false;
	if (settings_store_file.open(tmp_path, FILE_O_WRITE))
	{
		size_t put = settings_store_file.write((const uint8_t *)data, len);
		settings_store_file.flush();
		settings_store_file.close();
		write_ok = (put == len);
		if (!write_ok)
		{
			DEBUG_MSG("SETST", "save: short write to temp file (%u of %u bytes)", (unsigned)put, (unsigned)len);
		}
	}
	else
	{
		DEBUG_MSG("SETST", "save: could not open temp file for write");
	}

	if (!write_ok)
	{
		Serial.printf("[SETST];save;write_failed;bytes=%u\n", (unsigned)len);
		InternalFS.remove(tmp_path);
		return false;
	}

	if (!InternalFS.rename(tmp_path, path))
	{
		DEBUG_MSG("SETST", "save: rename of temp file onto live path failed");
		Serial.printf("[SETST];save;rename_failed;bytes=%u\n", (unsigned)len);

		// Inventory BEFORE the temp file is removed: on the space hypothesis
		// the temp file is exactly what pushed the filesystem over, so a
		// report taken after the cleanup would describe a state that never
		// existed. (Measured on DK5EN-90 2026-09-13 with a healthy store:
		// 29 of 224 blocks in content, so space is not the explanation there.)
		settingsStoreReportFilesystem("rename_failed");

		// ---------------------------------------------------------------
		// Believe the filesystem, not the return value.
		//
		// DK5EN-90, 2026-09-16 (docs/bench/w3-baseline/README.md §7, capture
		// rak90-migration-boot-20260916.txt): on the W3 migration boot
		// lfs_rename() reported failure having ACTUALLY PERFORMED THE MOVE.
		// The inventory printed immediately above -- taken before any
		// cleanup -- listed the destination at its new size with no temp
		// file left anywhere on the volume, and a later save in the same
		// boot found the destination byte-identical to what encode()
		// produces ("save;skipped_unchanged"). The store had not existed at
		// all at the start of that boot ("path;keyed_absent"), so the only
		// thing that can have created it is this rename.
		//
		// The retry below then failed for a SECOND, different reason -- the
		// source it wanted was already gone -- and that pair of failures is
		// what reported `legacy_migration_failed` on a migration that had in
		// fact written every byte correctly.
		//
		// A no-op failure and a false negative are indistinguishable from
		// the return value alone and call for opposite actions (retry vs.
		// stop), so the destination is asked directly. This is strictly
		// stronger than trusting the return code: it verifies the actual
		// post-condition the caller cares about.
		// ---------------------------------------------------------------
		if (fileHasExactContent(path, data, len))
		{
			Serial.printf("[SETST];save;rename_false_negative;bytes=%u\n", (unsigned)len);
			// No-op when the rename consumed the temp file, which is the
			// case this branch exists for; harmless if some other path left
			// one behind.
			InternalFS.remove(tmp_path);
			return true;
		}

		// One retry, which is a diagnostic as much as a repair. The failure
		// this exists for (DK5EN-90, 2026-09-12, twice on one boot, never
		// since) has two live explanations left now that space is measured
		// out: a transient flash error while the SoftDevice owns the radio,
		// or something persistent about the destination. The retry separates
		// them on the console the next time it happens, and it cannot make
		// anything worse: the temp file is intact and the live path still
		// holds its old content either way.
		if (InternalFS.rename(tmp_path, path))
		{
			Serial.printf("[SETST];save;rename_retry_ok;bytes=%u\n", (unsigned)len);
			return true;
		}

		Serial.printf("[SETST];save;rename_failed_twice;bytes=%u\n", (unsigned)len);
		InternalFS.remove(tmp_path);
		return false;
	}

	Serial.printf("[SETST];save;ok;bytes=%u\n", (unsigned)len);
	return true;
}

SettingsLoadResult settingsStoreLoad()
{
	SettingsLoadResult result;

	settings_store_file.open(kSettingsPath, FILE_O_READ);
	if (!settings_store_file)
	{
		// No file at kSettingsPath -- expected on first boot / right after a format. Not an error:
		// the caller is expected to already hold struct defaults in meshcom_settings (the same
		// discipline nrf52_flash.cpp's flash_reset() / default-construction path already follows),
		// so "no file" and "decode() found nothing to apply" leave the caller in the same state.
		result.file_found = false;
		return result;
	}
	result.file_found = true;

	// Size before the read, same reasoning as nrf52_flash.cpp's init_flash(): captured before
	// close() so it is still queryable, and here also before the read so the read length can be
	// capped to it rather than to the buffer size (avoids feeding decode() stale/garbage bytes
	// from beyond EOF if a short file were ever misread as short().
	uint32_t stored_size = settings_store_file.size();

	char *buf = (char *)malloc(kSettingsBufferCap);
	if (buf == nullptr)
	{
		DEBUG_MSG("SETST", "load: malloc(%u) failed", (unsigned)kSettingsBufferCap);
		Serial.printf("[SETST];load;malloc_failed;cap=%u\n", (unsigned)kSettingsBufferCap);
		settings_store_file.close();
		return result; // file_found=true, read_ok=false, stats default
	}

	// settingsStoreSave() never writes more than kSettingsBufferCap bytes, so a file bigger than
	// that is foreign or corrupt by construction; read at most kSettingsBufferCap and let decode()
	// make what sense it can of the prefix rather than grow the allocation to match an untrusted
	// on-disk size.
	size_t to_read = (stored_size < (uint32_t)kSettingsBufferCap) ? (size_t)stored_size : kSettingsBufferCap;

	int got = settings_store_file.read(buf, (uint16_t)to_read);
	settings_store_file.close();

	if (got < 0)
	{
		DEBUG_MSG("SETST", "load: read() failed");
		Serial.printf("[SETST];load;read_failed\n");
		free(buf);
		return result; // file_found=true, read_ok=false, stats default
	}

	result.read_ok = true;

	// Deliberately NO layout/version check here (e.g. nothing resembling config_json.cpp's
	// CFG_IMP_ELAYOUT gate against FLASH_STRUCT_VERSION, config_json.h:157). That gate is correct
	// for configImportJson() importing a FOREIGN config file, and exactly wrong here: this is a
	// node reading its OWN settings back, which is precisely the case the D1-04 keyed-store
	// architecture (docs/BACKLOG.md, OPT-07) exists to make version-gate-free in the first place --
	// a missing key keeps meshcom_settings' existing value, an unknown key is ignored, so a field
	// reorder or an added/removed field no longer needs a layout bump to read safely. Reinstating a
	// version check on this path would silently restore the very "any struct change wipes the
	// file" failure mode (nrf52_flash.cpp:327-334, N-12) this store was built to replace. Do not
	// "fix" this comment's absence of a check.
	result.stats = settings_store::decode(settings_schema::fields(), settings_schema::fieldCount(),
										   &meshcom_settings, buf, (size_t)got);

	free(buf);
	return result;
}

namespace
{
// Existence check via open()-then-close rather than InternalFS.exists(): both the real
// Adafruit_LittleFS and the native test double (test/test_nrf52_settings_paths/stubs/
// Adafruit_LittleFS.h) implement File::open()'s falsy-on-missing-file behaviour (it is exactly what
// `if (!lora_file)` / `if (!settings_store_file)` already rely on elsewhere in this cutover), so
// this needs no test-fixture change to work in both the real firmware and the native suite.
bool fileExists(Adafruit_LittleFS_Namespace::File &file, const char *path)
{
	bool found = file.open(path, FILE_O_READ);
	file.close();
	return found;
}
} // namespace

bool settingsStoreRemove()
{
	// "Did not exist" is success, not failure -- checked explicitly (rather than trusting remove()'s
	// bool return alone) so flash_reset() gets a real signal to fall back to InternalFS.format() only
	// when a file that DID exist could not be removed, not on the ordinary first-ever-reset case where
	// neither file has ever been written.
	bool ok = true;

	if (fileExists(settings_store_file, kSettingsPath))
	{
		if (!InternalFS.remove(kSettingsPath))
		{
			Serial.printf("[SETST];reset;remove_failed;file=store\n");
			ok = false;
		}
	}

	if (fileExists(settings_store_file, kSettingsTmpPath))
	{
		if (!InternalFS.remove(kSettingsTmpPath))
		{
			Serial.printf("[SETST];reset;remove_failed;file=store_tmp\n");
			ok = false;
		}
	}

	return ok;
}

bool settingsStoreDump(void)
{
	if (!settings_store_file.open(kSettingsPath, FILE_O_READ))
	{
		Serial.printf("[SETST];dump;no_file\n");
		return false;
	}

	uint32_t stored_size = settings_store_file.size();
	Serial.printf("[SETST];dump;begin;bytes=%lu\n", (unsigned long)stored_size);

	// Printed via printf("%.*s", ...) rather than Serial.write(buf, len): the keyed store's content
	// is text ("key=value\n" lines, settings_store.h), and printf is the one Serial primitive every
	// build of this codebase already relies on (including the native test double, which stubs only
	// printf -- see test/test_nrf52_settings_paths/stubs).
	char chunk[64];
	uint32_t remaining = stored_size;
	bool read_error = false;
	while (remaining > 0)
	{
		int want = (int)(remaining < sizeof(chunk) ? remaining : sizeof(chunk));
		int got = settings_store_file.read(chunk, want);
		if (got <= 0)
		{
			read_error = true;
			break;
		}
		Serial.printf("%.*s", got, chunk);
		remaining -= (uint32_t)got;
	}
	settings_store_file.close();

	Serial.printf("\n[SETST];dump;end;read_error=%d\n", read_error ? 1 : 0);
	settingsStoreReportFilesystem("dump");
	return !read_error;
}

namespace
{

// Running totals for settingsStoreReportFilesystem()'s walk. Kept in one
// struct so the recursion carries a single reference rather than four
// out-parameters.
struct FsInventory
{
	const char *reason = "";
	uint32_t block_bytes = 128;
	uint32_t total_bytes = 0;
	uint32_t total_blocks = 0;
	uint32_t file_count = 0;
	uint32_t dir_count = 0;
	uint32_t depth_truncated = 0; // directories not entered because of the depth bound
};

// Prints one line per file below `dir` and accumulates into `inv`.
// `prefix` is the path already walked (no trailing slash), used only for the
// printed name; `depth_left` bounds the recursion.
void walk_directory(File &dir, const char *prefix, int depth_left, FsInventory &inv)
{
	for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile())
	{
		// 64 covers /adafruit/bond_prph/<16-hex-digit peer id> with room to
		// spare; a longer path is printed truncated rather than skipped, since
		// this line is a diagnostic and the SIZE is what carries the answer.
		char path[64];
		snprintf(path, sizeof(path), "%s/%s", prefix, entry.name());

		if (entry.isDirectory())
		{
			inv.dir_count++;
			if (depth_left > 1)
			{
				walk_directory(entry, path, depth_left - 1, inv);
			}
			else
			{
				inv.depth_truncated++;
			}
		}
		else
		{
			const uint32_t size = entry.size();
			Serial.printf("[SETST];fs;%s;file;%s;%lu\n", inv.reason, path, (unsigned long)size);
			inv.total_bytes += size;
			inv.total_blocks += (size + inv.block_bytes - 1) / inv.block_bytes;
			inv.file_count++;
		}
		entry.close();
	}
}

} // namespace

void settingsStoreReportFilesystem(const char *reason)
{
	// Geometry is compile-time constant on this platform and printed with the
	// inventory so a log line is self-contained: InternalFileSystem.cpp sizes
	// the filesystem at 7 flash pages (7 x 4096 = 28 672 B) in 128 B blocks.
	// It is repeated here rather than included because those macros are
	// private to that .cpp.
	constexpr uint32_t kFsTotalBytes = 7u * 4096u;
	constexpr uint32_t kFsBlockBytes = 128u;

	FsInventory inv;
	inv.reason = reason;
	inv.block_bytes = kFsBlockBytes;

	// Depth 3, not an unbounded walk: the deepest path this filesystem holds
	// is /adafruit/bond_prph/<peer> (bonding.cpp), and a diagnostic that runs
	// on a failure path has no business recursing without a bound on a 4 KB
	// task. Anything deeper is counted as a directory that was not entered,
	// which the total line reports rather than hides.
	File dir = InternalFS.open("/");
	if (!dir || !dir.isDirectory())
	{
		Serial.printf("[SETST];fs;%s;root_unreadable\n", reason);
		return;
	}
	walk_directory(dir, "", 3, inv);
	dir.close();

	const uint32_t total_bytes = inv.total_bytes;
	const uint32_t total_blocks = inv.total_blocks;
	const uint32_t file_count = inv.file_count;

	// content_blocks is file data only: littlefs also spends blocks on
	// directory metadata pairs and keeps free blocks for its copy-on-write
	// updates, so this is a floor on usage, never the free-space figure.
	Serial.printf("[SETST];fs;%s;total;files;%lu;dirs;%lu;bytes;%lu;content_blocks;%lu;of;%lu;not_entered;%lu\n",
				  reason, (unsigned long)file_count, (unsigned long)inv.dir_count, (unsigned long)total_bytes,
				  (unsigned long)total_blocks, (unsigned long)(kFsTotalBytes / kFsBlockBytes),
				  (unsigned long)inv.depth_truncated);
}

#endif // NRF52_SERIES
