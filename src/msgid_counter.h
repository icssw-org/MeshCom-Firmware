/**
 * The message-id counter and how often it reaches flash.
 *
 * `node_msgid` is the low 10 bits of every message id this node originates
 * (`msg_id = (_GW_ID << 10) | (node_msgid & 0x3FF)`, loop_functions.cpp). It
 * has to survive a reboot: a node that restarts its counter at 0 replays ids
 * its neighbours still hold in their dedup rings, and those frames are then
 * dropped as duplicates.
 *
 * Persisting it on EVERY originated frame is what that requirement used to
 * cost. On the nRF52 one save is a full ~1.5 kB file write plus a rename on a
 * 28 KB internal filesystem; on ESP32 it is an NVS commit. Eight call sites in
 * loop_functions.cpp did exactly that, once per frame.
 *
 * The high-water-mark scheme here keeps the guarantee and pays for it once per
 * `kMsgIdPersistStep` frames instead:
 *
 *   - the counter is persisted only when it is a multiple of the step, so the
 *     stored value is always a value the node really used, and never more than
 *     one step behind the live one;
 *   - on load, the stored value is advanced by one whole step before the first
 *     frame goes out, **and that advanced value is written back once**. Every
 *     id between the stored value and that point is treated as spent, which is
 *     precisely the range an unclean shutdown could have used without
 *     recording it.
 *
 * The write-back at load is not optional, and leaving it out reintroduces the
 * bug in a subtler form: without it flash still names the PREVIOUS block while
 * the node is already handing out ids from the new one, so a crash in the
 * first `step` frames after a boot makes the next boot start where this one
 * did. The one write per boot is what makes the whole scheme safe; it is paid
 * by the same `sanitize_loaded_settings()` that every load path already calls.
 *
 * So an id is never reused after a crash, and the cost drops from one write
 * per originated frame to one per `kMsgIdPersistStep` frames plus one per
 * boot. The price is that up to one step of ids is skipped per boot --
 * invisible on the wire (ids are mod 1000 anyway and the dedup window is
 * minutes, not hundreds of frames).
 *
 * Pure C++, no Arduino dependency -- unit-tested natively (test_msgid_counter),
 * the same shape as settings_sanitize.h. The persisting itself stays at the
 * call sites, which are the only places that know what else changed.
 */
#pragma once

// The counter is emitted as a 3-digit decimal in several frame types
// (`snprintf(cId, sizeof(cId), "%03i", ...)`), so it wraps at 999, not at
// 1023, even though 10 bits would hold more.
constexpr int kMsgIdMax = 999;

// One flash write per this many originated frames. 100 gives ten writes per
// full 0..999 cycle and skips at most 10 % of the id space per boot.
constexpr int kMsgIdPersistStep = 100;

// The next value of the counter: increment, wrapping 999 -> 0. Out-of-range
// input (a corrupt or uninitialised value) is folded back into the range
// rather than propagated -- this feeds a wire field.
int msgIdAdvance(int current);

// True when `msgid` is a value that has to reach flash. Call AFTER
// msgIdAdvance(), with the value that will be used next.
bool msgIdNeedsPersist(int msgid);

// The value the counter must start from after loading `stored` from flash:
// one whole step ahead, so the ids an unclean shutdown may have used without
// recording them are never handed out twice.
//
// THE CALLER MUST PERSIST THE RESULT before the first frame goes out -- see
// the write-back paragraph above for what breaks otherwise.
int msgIdAfterLoad(int stored);
