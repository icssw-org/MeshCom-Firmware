// msgstore.cpp -- store node (last-hop mailbox) core, stage 3
// (docs/dm-stage3-wave-plan-20260914.md). See msgstore_api.h for the
// contract and the full state machine this implements.
//
// Platform-neutral on purpose (stdint/string/stdio and the header-only
// crc32_util.h only, no Arduino) so it links into the native test build
// (test/test_msgstore) as well as both firmware platforms. Every clock,
// table lookup, random source and transmit goes through the MsgStoreEnv
// installed by msgstoreInit() -- see src/msgstore_glue.cpp for the
// firmware-side wiring.
#include "msgstore_api.h"

#include <string.h>
#include <stdio.h>

#include "crc32_util.h"
#include "msgstore_lock.h"

#define MSGSTORE_ACTION_RING_SIZE MSGSTORE_ACTIONS_PER_HOUR   // 20: exactly the ceiling we test against
#define MSGSTORE_HOUR_MS           3600000UL

static const struct MsgStoreEnv *s_env = NULL;

static struct MsgStoreEntry s_entries[MSGSTORE_SLOTS_MAX];

// Per-slot flag, not part of the api contract: set when an entry was the
// chosen candidate in msgstoreLoop() but the node-wide gate refused the
// action. Read back at storetime expiry to tell "never got a chance"
// (dropped_cap) apart from "got at least one attempt, still timed out"
// (dropped_storetime, attempt > 0 makes the flag irrelevant either way).
static bool s_capblocked[MSGSTORE_SLOTS_MAX];

static enum MsgStoreMode s_mode   = MSGSTORE_OFF;
static uint8_t           s_slots  = MSGSTORE_SLOTS_DEFAULT;
static uint16_t          s_hold_h = MSGSTORE_HOLD_DEFAULT_H;
static char              s_list[MSGSTORE_LIST_MAX * MSGSTORE_CALL_MAX] = {0};
static bool              s_notice_on = true;   // stage 4: --storenotice, default on

static struct MsgStoreCounters s_cnt = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// SNF-ACK-01: acked tombstones. An ack heard for a store-set call is
// remembered for MSGSTORE_TOMB_TTL_MS so a late copy of the same DM
// (src, dst, nnn) is not stored again. Fixed ring, oldest overwritten.
struct MsgStoreTomb
{
    char     sender[MSGSTORE_CALL_MAX];   // the DM's src
    char     acker[MSGSTORE_CALL_MAX];    // the DM's dst
    uint16_t nnn;
    uint32_t stamp_ms;
    bool     used;
};
static struct MsgStoreTomb s_tomb[MSGSTORE_TOMB_SLOTS];
static uint8_t             s_tomb_next = 0;   // next ring slot to overwrite

// Ring of the last MSGSTORE_ACTION_RING_SIZE (== 20) mailbox action
// timestamps, so "actions in the trailing hour" never needs an unbounded
// history: once the ring is full of timestamps all within the last hour,
// the 20/h ceiling is hit by construction.
static uint32_t s_action_ring[MSGSTORE_ACTION_RING_SIZE];
static uint8_t  s_action_ring_count = 0;
static uint8_t  s_action_ring_head  = 0;

static uint32_t s_last_action_ms   = 0;
static bool     s_have_last_action = false;

// F7: blocked_bp counts once per blocked *episode* (a run of due-but-refused
// ticks), not once per tick. Set true the first time a due candidate is
// refused, cleared as soon as the gate lets an action through or nothing is
// due any more.
static bool s_blocked_episode_active = false;

// ---------------------------------------------------------- lock counters
// Host build only: msgstore_lock.h maps MSGSTORE_LOCK()/UNLOCK() onto these so
// the host test can prove pairing, no nesting and no env callback inside
// the critical section. Firmware builds never compile this.
#if defined(NATIVE_BUILD)
static uint32_t s_lock_enters    = 0;
static uint32_t s_lock_exits     = 0;
static uint32_t s_lock_depth     = 0;
static uint32_t s_lock_depth_max = 0;

void msgstoreLockEnterCounted(void)
{
    s_lock_enters++;
    s_lock_depth++;
    if(s_lock_depth > s_lock_depth_max)
        s_lock_depth_max = s_lock_depth;
}

void msgstoreLockExitCounted(void)
{
    s_lock_exits++;
    if(s_lock_depth > 0)
        s_lock_depth--;
}

void msgstoreTestLockStats(uint32_t *enters, uint32_t *exits, uint32_t *depth_max)
{
    if(enters != NULL)    *enters    = s_lock_enters;
    if(exits != NULL)     *exits     = s_lock_exits;
    if(depth_max != NULL) *depth_max = s_lock_depth_max;
}

uint32_t msgstoreTestLockDepth(void) { return s_lock_depth; }

void msgstoreTestLockStatsReset(void)
{
    s_lock_enters = s_lock_exits = s_lock_depth = s_lock_depth_max = 0;
}
#endif

// ---------------------------------------------------------------- helpers

static uint32_t nowMs(void)
{
    return (s_env != NULL && s_env->now_ms != NULL) ? s_env->now_ms() : 0;
}

static const char *ownCall(void)
{
    return (s_env != NULL && s_env->own_call != NULL) ? s_env->own_call() : "";
}

static void copyCall(char *dst, size_t dstsz, const char *src)
{
    if(dstsz == 0)
        return;
    if(src == NULL)
    {
        dst[0] = 0;
        return;
    }
    strncpy(dst, src, dstsz - 1);
    dst[dstsz - 1] = 0;
}

// Everything before the first '-' (or the whole string when there is none).
static void baseCall(const char *call, char *out, size_t outsz)
{
    out[0] = 0;
    if(call == NULL || outsz == 0)
        return;

    size_t i = 0;
    while(call[i] != 0 && call[i] != '-' && i + 1 < outsz)
    {
        out[i] = call[i];
        i++;
    }
    out[i] = 0;
}

static bool sameBaseCall(const char *a, const char *b)
{
    char ba[MSGSTORE_CALL_MAX];
    char bb[MSGSTORE_CALL_MAX];
    baseCall(a, ba, sizeof(ba));
    baseCall(b, bb, sizeof(bb));
    return ba[0] != 0 && strcmp(ba, bb) == 0;
}

bool msgstoreSameBaseCall(const char *a, const char *b)
{
    return sameBaseCall(a, b);
}

// LIST mode match: a csv entry WITH an SSID ("OE1XYZ-5") must match dst
// exactly; an entry WITHOUT one ("OE1XYZ") matches any SSID of that base
// call, same as a bare base call would.
static bool listContains(const char *csv, const char *dst)
{
    if(csv == NULL || dst == NULL || dst[0] == 0)
        return false;

    char buf[MSGSTORE_LIST_MAX * MSGSTORE_CALL_MAX];
    copyCall(buf, sizeof(buf), csv);

    char *tok  = buf;
    bool  more = true;

    while(more && tok != NULL)
    {
        char *comma = strchr(tok, ',');
        if(comma != NULL)
            *comma = 0;
        else
            more = false;

        if(tok[0] != 0)
        {
            if(strchr(tok, '-') != NULL)
            {
                if(strcmp(tok, dst) == 0)
                    return true;
            }
            else
            {
                char dstBase[MSGSTORE_CALL_MAX];
                baseCall(dst, dstBase, sizeof(dstBase));
                if(strcmp(tok, dstBase) == 0)
                    return true;
            }
        }

        tok = more ? comma + 1 : NULL;
    }

    return false;
}

static uint8_t actionsInLastHour(uint32_t now)
{
    uint8_t n = 0;
    for(uint8_t i = 0; i < s_action_ring_count; i++)
    {
        if((uint32_t)(now - s_action_ring[i]) < MSGSTORE_HOUR_MS)
            n++;
    }
    return n;
}

static void recordAction(uint32_t now)
{
    s_action_ring[s_action_ring_head] = now;
    s_action_ring_head = (uint8_t)((s_action_ring_head + 1) % MSGSTORE_ACTION_RING_SIZE);
    if(s_action_ring_count < MSGSTORE_ACTION_RING_SIZE)
        s_action_ring_count++;

    s_last_action_ms   = now;
    s_have_last_action = true;
}

// F4: mark slot i as hook-touched. Called from every receive-path event and
// operator action that mutates a live slot (refresh/replace in msgstoreStore,
// msgstoreOnAck, msgstorePresence, msgstoreOnPeerDelivery, msgstorePurge(All),
// storetime expiry) so msgstoreLoop() can tell, after env->deliver() returns,
// whether the candidate slot it is about to stamp a ladder step onto is still
// the same logical entry it handed to deliver() a moment ago.
static void entryTouch(int i)
{
    s_entries[i].gen++;
}

// ---------------------------------------------------- lifecycle / config

void msgstoreInit(const struct MsgStoreEnv *env)
{
    s_env = env;
}

void msgstoreConfigure(enum MsgStoreMode mode, uint8_t slots, uint16_t hold_hours)
{
    s_mode = mode;

    uint8_t new_slots = (slots < 1) ? 1 : (slots > MSGSTORE_SLOTS_MAX ? MSGSTORE_SLOTS_MAX : slots);

    // F8: shrinking below the used range must not strand live entries --
    // every loop below iterates only `< s_slots`, so a slot >= new_slots
    // would otherwise sit forever with no expiry, no delivery, no ack purge
    // and no counter ever telling the operator it is gone.
    // SNF-GW-02: under the lock, a hook scans `< s_slots` in the LORA task.
    MSGSTORE_LOCK();
    if(new_slots < s_slots)
    {
        for(int i = new_slots; i < s_slots; i++)
        {
            if(s_entries[i].state != MSGSTORE_FREE)
            {
                s_entries[i].state  = MSGSTORE_FREE;
                s_entries[i].notice = 0;   // stage 4: purge/free clears notice
                s_capblocked[i]     = false;
                entryTouch(i);
                s_cnt.dropped_slots++;
            }
        }
    }

    s_slots  = new_slots;
    MSGSTORE_UNLOCK();
    s_hold_h = (hold_hours < 1) ? 1 : (hold_hours > MSGSTORE_HOLD_MAX_H ? MSGSTORE_HOLD_MAX_H : hold_hours);
}

void msgstoreSetList(const char *csv)
{
    copyCall(s_list, sizeof(s_list), csv);
}

enum MsgStoreMode msgstoreMode(void)     { return s_mode; }
uint8_t           msgstoreSlots(void)    { return s_slots; }
uint16_t          msgstoreHoldHours(void){ return s_hold_h; }
const char       *msgstoreListCsv(void)  { return s_list; }

const char *msgstoreModeName(enum MsgStoreMode mode)
{
    switch(mode)
    {
        case MSGSTORE_OWN:   return "own";
        case MSGSTORE_LIST:  return "list";
        case MSGSTORE_HEARD: return "heard";
        default:             return "off";
    }
}

const char *msgstoreStateName(uint8_t state)
{
    switch(state)
    {
        case MSGSTORE_HELD:     return "HELD";
        case MSGSTORE_ARMED:    return "ARMED";
        case MSGSTORE_LADDER:   return "LADDER";
        case MSGSTORE_COOLDOWN: return "COOLDOWN";
        default:                return "FREE";
    }
}

// Test-only entry point (same convention as dmDedupReset()/reackLimiterReset()):
// zero the table, counters, action ring and configuration back to boot
// defaults. Not part of the B/C-facing contract above this line.
void msgstoreReset(void)
{
    memset(s_entries, 0, sizeof(s_entries));
    memset(s_capblocked, 0, sizeof(s_capblocked));
    memset(&s_cnt, 0, sizeof(s_cnt));
    memset(s_tomb, 0, sizeof(s_tomb));   // SNF-ACK-01
    s_tomb_next = 0;
    memset(s_action_ring, 0, sizeof(s_action_ring));

    s_action_ring_count = 0;
    s_action_ring_head  = 0;
    s_last_action_ms    = 0;
    s_have_last_action  = false;

    s_blocked_episode_active = false;

    s_mode      = MSGSTORE_OFF;
    s_slots     = MSGSTORE_SLOTS_DEFAULT;
    s_hold_h    = MSGSTORE_HOLD_DEFAULT_H;
    s_list[0]   = 0;
    s_notice_on = true;   // stage 4: default on
}

// ---------------------------------------------------- receive-path events

bool msgstoreEligible(const char *dst)
{
    if(dst == NULL || dst[0] == 0)
        return false;

    switch(s_mode)
    {
        case MSGSTORE_OWN:
            return sameBaseCall(dst, ownCall());

        case MSGSTORE_LIST:
            return listContains(s_list, dst);

        case MSGSTORE_HEARD:
        {
            if(s_env == NULL || s_env->heard_age_ms == NULL)
                return false;
            int32_t age = s_env->heard_age_ms(dst);
            return age >= 0 && (uint32_t)age < MSGSTORE_HEARD_WINDOW_MS;
        }

        default:
            return false;
    }
}

// SNF-GW-02: every function from here to msgstoreLoop() may run in a task
// other than msgstoreLoop() (nRF52: OnRxDone in the LORA task, UDP GATE
// handlers in the loop task). Each reads the clock / own call / random source
// and hashes the payload BEFORE MSGSTORE_LOCK() and touches only the slot
// table, the counters and s_capblocked between lock and unlock: no printf, no
// malloc, no env callback inside the section (src/msgstore_lock.h).
int msgstoreStore(const char *src, const char *dst, uint16_t nnn,
                   const char *payload, size_t len)
{
    if(src == NULL || dst == NULL || payload == NULL)
        return -1;

    if(len == 0 || len > MSGSTORE_PAYLOAD_MAX)
        return -1;

    const char *own = ownCall();   // env callback: before the lock
    if(own[0] != 0 && strcmp(src, own) == 0)
        return -1;
    if(own[0] != 0 && strcmp(dst, own) == 0)
        return -1;

    uint16_t pcrc = (uint16_t)(crc32_buf(payload, len) & 0xFFFFU);
    uint32_t now  = nowMs();       // env callback: before the lock
    bool     notice_on = msgstoreNotice();   // plain bool read, taken outside to keep the section minimal

    int result = -1;

    MSGSTORE_LOCK();

    // Existing (src, dst, nnn)?
    for(int i = 0; i < s_slots; i++)
    {
        if(s_entries[i].state == MSGSTORE_FREE)
            continue;
        if(s_entries[i].nnn != nnn)
            continue;
        if(strcmp(s_entries[i].src, src) != 0 || strcmp(s_entries[i].dst, dst) != 0)
            continue;

        if(s_entries[i].plen == (uint16_t)len && s_entries[i].pcrc == pcrc)
        {
            // Sender re-flood: same message, still in flight on its end.
            // Refresh the clock and step back from wherever we were.
            s_entries[i].stored_ms  = now;
            s_entries[i].state      = MSGSTORE_HELD;
            s_capblocked[i]         = false;
            entryTouch(i);
            s_cnt.refreshed++;
            result = i;
            goto done;
        }

        // Same (src, dst, nnn), different payload: the 0..999 NNN counter
        // wrapped onto an unrelated message. Replace in place, fresh cycle.
        memset(s_entries[i].payload, 0, sizeof(s_entries[i].payload));
        memcpy(s_entries[i].payload, payload, len);
        s_entries[i].plen      = (uint16_t)len;
        s_entries[i].pcrc      = pcrc;
        s_entries[i].stored_ms = now;
        s_entries[i].next_ms   = now;
        s_entries[i].attempt   = 0;
        s_entries[i].cycles    = 0;
        s_entries[i].state     = MSGSTORE_HELD;
        s_capblocked[i]        = false;
        entryTouch(i);
        s_cnt.stored++;
        result = i;
        goto done;
    }

    // SNF-ACK-01: already acked? A live tombstone refuses a new slot.
    for(int t = 0; t < MSGSTORE_TOMB_SLOTS; t++)
    {
        if(!s_tomb[t].used || s_tomb[t].nnn != nnn)
            continue;
        if((uint32_t)(now - s_tomb[t].stamp_ms) >= MSGSTORE_TOMB_TTL_MS)
            continue;
        if(strcmp(s_tomb[t].sender, src) != 0 || strcmp(s_tomb[t].acker, dst) != 0)
            continue;

        s_cnt.dropped_acked++;
        goto done;   // result stays -1
    }

    // New entry: first free slot within the first msgstoreSlots().
    for(int i = 0; i < s_slots; i++)
    {
        if(s_entries[i].state != MSGSTORE_FREE)
            continue;

        copyCall(s_entries[i].src, sizeof(s_entries[i].src), src);
        copyCall(s_entries[i].dst, sizeof(s_entries[i].dst), dst);
        s_entries[i].nnn = nnn;
        memset(s_entries[i].payload, 0, sizeof(s_entries[i].payload));
        memcpy(s_entries[i].payload, payload, len);
        s_entries[i].plen      = (uint16_t)len;
        s_entries[i].pcrc      = pcrc;
        s_entries[i].stored_ms = now;
        s_entries[i].next_ms   = now;
        s_entries[i].attempt   = 0;
        s_entries[i].cycles    = 0;
        s_entries[i].state     = MSGSTORE_HELD;
        s_entries[i].notice    = notice_on ? 1 : 0;   // stage 4: new slot only
        s_capblocked[i]        = false;
        s_cnt.stored++;
        result = i;
        goto done;
    }

    // Full: never evict a held message for a new one.
    s_cnt.dropped_slots++;

done:
    MSGSTORE_UNLOCK();
    return result;
}

void msgstoreOnAck(const char *acker, const char *sender, uint16_t nnn)
{
    if(acker == NULL || sender == NULL)
        return;

    // SNF-ACK-01: env callbacks before the lock; tombstones only for calls we store for.
    bool     tomb_ok  = msgstoreEligible(acker);
    uint32_t tomb_now = nowMs();

    MSGSTORE_LOCK();

    if(tomb_ok)
    {
        int slot = -1;
        for(int t = 0; t < MSGSTORE_TOMB_SLOTS; t++)
        {
            if(s_tomb[t].used && s_tomb[t].nnn == nnn &&
               strcmp(s_tomb[t].sender, sender) == 0 && strcmp(s_tomb[t].acker, acker) == 0)
            {
                slot = t;   // same key: refresh the stamp, no second slot
                break;
            }
        }
        if(slot < 0)
        {
            slot = s_tomb_next;
            s_tomb_next = (uint8_t)((s_tomb_next + 1) % MSGSTORE_TOMB_SLOTS);
            copyCall(s_tomb[slot].sender, sizeof(s_tomb[slot].sender), sender);
            copyCall(s_tomb[slot].acker, sizeof(s_tomb[slot].acker), acker);
            s_tomb[slot].nnn  = nnn;
            s_tomb[slot].used = true;
        }
        s_tomb[slot].stamp_ms = tomb_now;
    }

    for(int i = 0; i < s_slots; i++)
    {
        if(s_entries[i].state == MSGSTORE_FREE)
            continue;
        if(s_entries[i].nnn != nnn)
            continue;
        if(strcmp(s_entries[i].dst, acker) != 0)
            continue;
        if(strcmp(s_entries[i].src, sender) != 0)
            continue;

        s_entries[i].state  = MSGSTORE_FREE;
        s_entries[i].notice = 0;   // stage 4: purge/free clears notice
        s_capblocked[i]     = false;
        entryTouch(i);
        s_cnt.purged_ack++;
        break;   // (src, dst, nnn) is unique among live entries
    }

    MSGSTORE_UNLOCK();
}

void msgstorePresence(const char *call)
{
    if(call == NULL || call[0] == 0)
        return;

    uint32_t now = nowMs();   // env callback: before the lock

    // One locked scan per matching slot (plus one final scan that finds
    // none): the random draw between match and arm is an env callback and must
    // stay outside the lock. Usually nothing is held for `call`, so the common
    // cost is a single short critical section.
    int next = 0;
    while(next < MSGSTORE_SLOTS_MAX)
    {
        int     idx = -1;
        uint8_t gen = 0;

        // Phase 1 (locked): first matching slot at or after `next`.
        MSGSTORE_LOCK();
        for(int i = next; i < s_slots; i++)
        {
            if(s_entries[i].state != MSGSTORE_HELD)
                continue;
            if(strcmp(s_entries[i].dst, call) != 0)
                continue;
            // One round trip since storing, so the sender's own attempt (still
            // in flight) gets its chance before we start competing with it.
            if((uint32_t)(now - s_entries[i].stored_ms) < 60000UL)
                continue;

            idx = i;
            gen = s_entries[i].gen;
            break;
        }
        MSGSTORE_UNLOCK();

        if(idx < 0)
            return;

        uint32_t jitter = (s_env != NULL && s_env->random_between != NULL)
                               ? s_env->random_between(MSGSTORE_JITTER_MIN_MS, MSGSTORE_JITTER_MAX_MS)
                               : MSGSTORE_JITTER_MIN_MS;

        // Phase 2 (locked): arm the slot and stamp next_ms in one step, unless
        // another hook touched it meanwhile (gen) -- then that hook wins.
        MSGSTORE_LOCK();
        if(idx < s_slots && s_entries[idx].state == MSGSTORE_HELD && s_entries[idx].gen == gen)
        {
            s_entries[idx].state   = MSGSTORE_ARMED;
            s_capblocked[idx]      = false;
            entryTouch(idx);
            s_entries[idx].next_ms = now + jitter;
        }
        MSGSTORE_UNLOCK();

        next = idx + 1;
    }
}

void msgstoreOnPeerDelivery(const char *src, uint16_t nnn)
{
    if(src == NULL)
        return;

    MSGSTORE_LOCK();

    for(int i = 0; i < s_slots; i++)
    {
        if(s_entries[i].state != MSGSTORE_ARMED && s_entries[i].state != MSGSTORE_LADDER)
            continue;
        if(s_entries[i].nnn != nnn)
            continue;
        if(strcmp(s_entries[i].src, src) != 0)
            continue;

        s_entries[i].state = MSGSTORE_HELD;
        s_capblocked[i]     = false;
        entryTouch(i);
        s_cnt.cancelled_peer++;
    }

    MSGSTORE_UNLOCK();
}

// -------------------------------------------------------------- loop task

void msgstoreLoop(void)
{
    if(s_env == NULL)
        return;

    // SNF-GW-02: the loop runs in the loop task while hooks may run in the
    // LORA task (nRF52) or the GATE handlers. Its slot-table access is cut
    // into short locked sections: (A) expiry + choose notice/candidate,
    // (B) snapshot for deliver()/notify(), (C) stamp the result. Everything
    // between them -- the node-wide gates (env callbacks), deliver(), notify()
    // -- runs unlocked, so a hook can land in between; (B) re-validates the
    // choice and (C) re-checks `gen`, exactly as the F4 guard always did.
    uint32_t now     = nowMs();   // env callback: before the lock
    uint32_t hold_ms = (uint32_t)s_hold_h * 3600000UL;

    int      notice_slot    = -1;
    int      candidate      = -1;
    uint32_t candidate_next = 0;

    MSGSTORE_LOCK();

    // (1) hold expiry (storetime/cap) and cooldown release. One pass over
    // every live entry; independent of the one-action-per-call gate below.
    for(int i = 0; i < s_slots; i++)
    {
        if(s_entries[i].state == MSGSTORE_FREE)
            continue;

        if((uint32_t)(now - s_entries[i].stored_ms) >= hold_ms)
        {
            if(s_entries[i].attempt == 0 && s_capblocked[i])
                s_cnt.dropped_cap++;
            else
                s_cnt.dropped_storetime++;

            s_entries[i].state  = MSGSTORE_FREE;
            s_entries[i].notice = 0;   // stage 4: purge/free clears notice
            s_capblocked[i]     = false;
            entryTouch(i);
            continue;
        }

        // F3: millis-wrap safe "cooldown is over" -- (int32_t)(now - next_ms)
        // >= 0 reads correctly across the 49.7-day wrap, plain >= does not.
        if(s_entries[i].state == MSGSTORE_COOLDOWN && (int32_t)(now - s_entries[i].next_ms) >= 0)
            s_entries[i].state = MSGSTORE_HELD;
    }

    // (2) stage 4: a pending notice pre-empts a ladder step this tick -- it
    // competes with deliveries under the very same node gates below (§5:
    // "one action; it competes with deliveries on purpose so the node
    // ceiling stays the single bound"). Picked ahead of the ARMED/LADDER
    // scan so a due delivery never starves a pending notice on a busy node.
    if(s_env->notify != NULL)
    {
        for(int i = 0; i < s_slots; i++)
        {
            if(s_entries[i].state != MSGSTORE_FREE && s_entries[i].notice == 1)
            {
                notice_slot = i;
                break;
            }
        }
    }

    // (3) at most one mailbox action this call: pick the ARMED/LADDER entry
    // with the smallest due next_ms -- skipped when a notice already claimed
    // this tick's action.
    if(notice_slot < 0)
    {
        for(int i = 0; i < s_slots; i++)
        {
            if(s_entries[i].state != MSGSTORE_ARMED && s_entries[i].state != MSGSTORE_LADDER)
                continue;
            // F3: due test, wrap-safe.
            if((int32_t)(now - s_entries[i].next_ms) < 0)
                continue;

            // F3: "earlier than the current candidate", wrap-safe.
            if(candidate < 0 || (int32_t)(s_entries[i].next_ms - candidate_next) < 0)
            {
                candidate      = i;
                candidate_next = s_entries[i].next_ms;
            }
        }
    }

    MSGSTORE_UNLOCK();

    if(notice_slot < 0 && candidate < 0)
    {
        s_blocked_episode_active = false;   // F7: nothing due, no episode in progress
        return;   // nothing due -- no gate check, nothing was blocked
    }

    // Node-wide gates: env callbacks (bp_state, util_pct), so unlocked.
    // s_have_last_action / s_action_ring / s_blocked_episode_active are
    // loop-task state, no hook touches them.
    bool gate_ok = true;

    if(s_have_last_action && (uint32_t)(now - s_last_action_ms) < MSGSTORE_ACTION_GAP_MS)
        gate_ok = false;
    else if(actionsInLastHour(now) >= MSGSTORE_ACTIONS_PER_HOUR)
        gate_ok = false;
    else if(s_env->bp_state == NULL || s_env->bp_state() != 0)
        gate_ok = false;
    else if(s_env->util_pct == NULL || s_env->util_pct() > MSGSTORE_UTIL_MAX_PCT)
        gate_ok = false;

    if(!gate_ok)
    {
        // F7: count once per blocked episode (the run of due-but-refused
        // ticks), not once per 2 s tick -- a util-only block held for
        // minutes must not read as "the 20/h ceiling ate a hold time".
        // Stage 4: a blocked pending notice counts into notice_blocked
        // instead, same one-per-episode rule (reusing the same flag).
        if(!s_blocked_episode_active)
        {
            if(notice_slot >= 0)
                s_cnt.notice_blocked++;
            else
                s_cnt.blocked_bp++;
            s_blocked_episode_active = true;
        }
        if(notice_slot < 0)
        {
            MSGSTORE_LOCK();
            // only if a hook did not free the candidate since section A
            if(s_entries[candidate].state == MSGSTORE_ARMED || s_entries[candidate].state == MSGSTORE_LADDER)
                s_capblocked[candidate] = true;
            MSGSTORE_UNLOCK();
        }
        return;
    }

    s_blocked_episode_active = false;   // F7: the gate let this tick through

    if(notice_slot >= 0)
    {
        // (B) F4-style snapshot/gen guard, same reasoning as the delivery
        // path below: notify() (String building + ring enqueue) can race a
        // hook that purges/refreshes this very slot from the LORA task. The
        // choice was made before the gates ran, so re-validate it: a hook may
        // have freed the slot or cleared the notice in between.
        uint8_t gen_before = 0;
        struct MsgStoreEntry snapshot;
        bool    still_valid;

        MSGSTORE_LOCK();
        still_valid = (s_entries[notice_slot].state != MSGSTORE_FREE && s_entries[notice_slot].notice == 1);
        if(still_valid)
        {
            gen_before = s_entries[notice_slot].gen;
            snapshot   = s_entries[notice_slot];
        }
        MSGSTORE_UNLOCK();

        if(!still_valid)
            return;   // a hook got there first; next tick decides afresh

        if(!s_env->notify(&snapshot))
            return;   // ring refused -- retry next tick, do not advance

        recordAction(now);
        s_cnt.notified++;

        // (C)
        MSGSTORE_LOCK();
        if(s_entries[notice_slot].state != MSGSTORE_FREE && s_entries[notice_slot].gen == gen_before)
            s_entries[notice_slot].notice = 2;   // sent -- do not re-notify
        MSGSTORE_UNLOCK();

        return;
    }

    if(s_env->deliver == NULL)
        return;

    // F4: snapshot the generation before handing the slot to deliver() --
    // on nRF52 deliver() (String building + encodeAPRS + ring enqueue,
    // hundreds of microseconds) runs from the loop task while the LORA task
    // can concurrently purge/refresh/peer-cancel/re-arm this very slot
    // through the receive-path hooks. deliver() itself must stay outside the
    // lock (the alternative would put the LORA task behind the loop task's TX
    // enqueue), so the guard stays: if the slot the hook left behind does not
    // match what we started with, the frame is still on the ring (it counts
    // as delivered) but the ladder bookkeeping below must not stamp
    // attempt/next_ms/COOLDOWN onto storage the hook has since repurposed --
    // that would resurrect a purged entry or silently promote a peer-cancelled
    // one.
    //
    // SNF-GW-02: the gen re-check and the stamps that follow are now ONE
    // locked section (C), so the former check-then-act window (a hook landing
    // between the re-check and the stores) is closed; what remains is the
    // inherent "hook during deliver()" case that gen detects.
    uint8_t gen_before = 0;

    // Same race, other half: deliver() reads src/dst/payload while a hook
    // may replace them (same NNN, wrapped counter). Hand it a stack copy so
    // the frame it builds is one consistent message, never a torn one.
    struct MsgStoreEntry snapshot;
    bool still_valid;

    // (B) the candidate was chosen before the gates ran: re-validate it.
    MSGSTORE_LOCK();
    still_valid = ((s_entries[candidate].state == MSGSTORE_ARMED || s_entries[candidate].state == MSGSTORE_LADDER)
                   && (int32_t)(now - s_entries[candidate].next_ms) >= 0);
    if(still_valid)
    {
        gen_before = s_entries[candidate].gen;
        snapshot   = s_entries[candidate];
    }
    MSGSTORE_UNLOCK();

    if(!still_valid)
        return;   // a hook got there first; next tick decides afresh

    if(!s_env->deliver(&snapshot))
        return;   // ring refused -- retry next tick, do not advance

    recordAction(now);
    s_cnt.delivered++;

    // (C)
    MSGSTORE_LOCK();
    if(s_entries[candidate].state != MSGSTORE_FREE && s_entries[candidate].gen == gen_before)
    {
        // (a hook that touched the slot during deliver() leaves it as it
        // left it -- F4)
        if(s_entries[candidate].state == MSGSTORE_ARMED)
            s_entries[candidate].state = MSGSTORE_LADDER;   // jitter over, ladder starts

        s_entries[candidate].attempt++;

        if(s_entries[candidate].attempt >= MSGSTORE_LADDER_STEPS)
        {
            s_entries[candidate].cycles++;
            s_entries[candidate].state   = MSGSTORE_COOLDOWN;
            s_entries[candidate].next_ms = now + MSGSTORE_COOLDOWN_MS;
        }
        else
        {
            uint32_t gap = MSGSTORE_STEP_MS;
            if(s_entries[candidate].attempt == 3 || s_entries[candidate].attempt == 6)
                gap += MSGSTORE_BLOCK_GAP_MS;
            s_entries[candidate].next_ms = now + gap;
        }
    }
    MSGSTORE_UNLOCK();
}

// --------------------------------------------------------- operator actions

// Operator actions run in the loop task (web page, serial/BLE commands) but
// mutate the same table the hooks write, so they take the lock too.
bool msgstorePurge(int slot)
{
    bool purged = false;

    MSGSTORE_LOCK();
    if(slot >= 0 && slot < s_slots && s_entries[slot].state != MSGSTORE_FREE)
    {
        s_entries[slot].state  = MSGSTORE_FREE;
        s_entries[slot].notice = 0;   // stage 4: purge/free clears notice
        s_capblocked[slot]     = false;
        entryTouch(slot);
        purged = true;
    }
    MSGSTORE_UNLOCK();

    return purged;
}

void msgstorePurgeAll(void)
{
    MSGSTORE_LOCK();
    for(int i = 0; i < MSGSTORE_SLOTS_MAX; i++)
    {
        s_entries[i].state  = MSGSTORE_FREE;
        s_entries[i].notice = 0;   // stage 4: purge/free clears notice
        s_capblocked[i]     = false;
        entryTouch(i);
    }
    MSGSTORE_UNLOCK();
}

bool msgstoreDeliverNow(int slot)
{
    uint32_t now    = nowMs();   // env callback: before the lock
    bool     armed  = false;

    MSGSTORE_LOCK();
    if(slot >= 0 && slot < s_slots
       && (s_entries[slot].state == MSGSTORE_HELD || s_entries[slot].state == MSGSTORE_COOLDOWN))
    {
        s_entries[slot].state   = MSGSTORE_ARMED;
        s_capblocked[slot]      = false;
        s_entries[slot].next_ms = now;
        armed = true;
    }
    MSGSTORE_UNLOCK();

    return armed;
}

// --------------------------------------------------------------- readers

int msgstoreUsed(void)
{
    int n = 0;
    for(int i = 0; i < s_slots; i++)
        if(s_entries[i].state != MSGSTORE_FREE)
            n++;
    return n;
}

// Same discipline as msgstoreUsed(): lock-free read of s_entries from the reader's context.
int msgstoreBytes(void)
{
    int n = 0;
    for(int i = 0; i < s_slots; i++)
        if(s_entries[i].state != MSGSTORE_FREE)
            n += (int)s_entries[i].plen;
    return n;
}

const struct MsgStoreEntry *msgstoreEntry(int slot)
{
    if(slot < 0 || slot >= MSGSTORE_SLOTS_MAX)
        return NULL;
    if(s_entries[slot].state == MSGSTORE_FREE)
        return NULL;
    return &s_entries[slot];
}

const struct MsgStoreCounters *msgstoreCounters(void)
{
    return &s_cnt;
}

uint8_t msgstoreActionsLastHour(void)
{
    return actionsInLastHour(nowMs());
}

uint32_t msgstoreNextActionInMs(void)
{
    uint32_t now   = nowMs();
    bool     found = false;
    uint32_t earliest = 0;

    for(int i = 0; i < s_slots; i++)
    {
        if(s_entries[i].state != MSGSTORE_ARMED && s_entries[i].state != MSGSTORE_LADDER)
            continue;
        // F3: "earlier than the current earliest", wrap-safe.
        if(!found || (int32_t)(s_entries[i].next_ms - earliest) < 0)
        {
            earliest = s_entries[i].next_ms;
            found    = true;
        }
    }

    // F3: "already due", wrap-safe -- and the remaining distance is then
    // just the unsigned difference, which is correctly wrap-safe on its own.
    if(!found || (int32_t)(earliest - now) <= 0)
        return 0;
    return (uint32_t)(earliest - now);
}

int msgstoreFormatLine(char *buf, size_t n)
{
    if(buf == NULL || n == 0)
        return 0;

    // F7: "blk" (blocked by caps), not "bp" -- the counter covers all four
    // node-wide gates (30 s gap, 20/h ceiling, back-pressure, utilisation),
    // not only back-pressure.
    int len = snprintf(buf, n,
        "MBOX mode=%s used=%u/%u act=%u/20 stored=%u refr=%u deliv=%u ack=%u dropt=%u dropc=%u drops=%u peer=%u blk=%u sto=%u/%u tomb=%u",
        msgstoreModeName(s_mode),
        (unsigned)msgstoreUsed(), (unsigned)s_slots,
        (unsigned)msgstoreActionsLastHour(),
        (unsigned)s_cnt.stored, (unsigned)s_cnt.refreshed, (unsigned)s_cnt.delivered,
        (unsigned)s_cnt.purged_ack, (unsigned)s_cnt.dropped_storetime, (unsigned)s_cnt.dropped_cap,
        (unsigned)s_cnt.dropped_slots, (unsigned)s_cnt.cancelled_peer, (unsigned)s_cnt.blocked_bp,
        (unsigned)s_cnt.notified, (unsigned)s_cnt.notice_blocked,
        (unsigned)s_cnt.dropped_acked);

    if(len < 0)
        return 0;
    return ((size_t)len >= n) ? (int)(n - 1) : len;
}

// ---- stage 4: --storenotice on|off, default on ----
void msgstoreSetNotice(bool on) { s_notice_on = on; }
bool msgstoreNotice(void)       { return s_notice_on; }
