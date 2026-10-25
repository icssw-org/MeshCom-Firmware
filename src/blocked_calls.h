// BLK-01 -- call blocklist (--blockcall).
//
// Frames from a blocked callsign are dropped on RX and never relayed. This
// module is the core only: validation, normalisation, the runtime list,
// matching and persistence. The wiring (RX path, console, web, RM) lives
// elsewhere.
//
// RAM: only the canonical CSV string is kept, no per-entry array. Entries
// are re-split on every match; a list is <= 160 B, so that is cheap.
#ifndef BLOCKED_CALLS_H
#define BLOCKED_CALLS_H

#include <stddef.h>
#include <stdint.h>

#define BLOCKLIST_MAX 16
#define BLOCKLIST_CALL_MAX 9   // call incl. SSID, chars ("DK5EN-12" is 8; 6+"-12" is 9)
#define BLOCKLIST_CSV_MAX (BLOCKLIST_MAX * (BLOCKLIST_CALL_MAX + 1))   // incl. NUL

// ---- pure functions (host-testable) ------------------------------------

// Validate and normalise one callsign token (surrounding spaces ignored).
// out needs >= BLOCKLIST_CALL_MAX + 1 bytes. Returns false if invalid.
// "dk5en-01" -> "DK5EN-1", "DK5EN-0" -> "DK5EN".
bool blockCallNormalize(const char *in, char *out, size_t cap);

// Parse a user list ("A,B;C", "none", "") into the canonical "A,B,C".
// On failure out is empty and bad holds the offending raw token or "too many".
bool blockListParse(const char *in, char *out, size_t cap, char *bad, size_t badcap);

// Lenient variant for stored data: keep every valid entry (up to the cap),
// call dropped(entry) for each invalid or surplus one. Returns entries kept.
uint8_t blockListLoadLenient(const char *in, char *out, size_t cap, void (*dropped)(const char *entry));

// Match against an explicit canonical CSV (no lock, no global state).
bool blockListMatchCall(const char *csv, const char *call);
bool blockListMatchPath(const char *csv, const char *path);

// ---- runtime list ------------------------------------------------------

// Replace the list; on failure it stays unchanged. Does NOT persist.
bool blockListSet(const char *in, char *bad = nullptr, size_t badcap = 0);
void blockListGet(char *out, size_t cap);
uint8_t blockListCount();

// Safe from any task (nRF52: OnRxDone runs in the LoRa task).
bool isBlockedCall(const char *call);
bool isBlockedInPath(const char *path);   // "SRC,RELAY1*,RELAY2"

inline bool isBlockedFrame(const char *src, const char *path)
{
    return isBlockedCall(src) || isBlockedInPath(path);
}

void blockListNoteDrop();
uint32_t blockListDrops();

// ---- persistence (host: no-ops) ----------------------------------------
void blockListLoad();
bool blockListSave();
// false after a save that did not reach flash (full NVS); the RAM list is set anyway
bool blockListLastSaveOk();
#if defined(NATIVE_BUILD)
void blockListTestFailSave(bool fail);   // host tests only
#endif

#endif
