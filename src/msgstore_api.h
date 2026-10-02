// msgstore_api.h -- store node (last-hop mailbox) core interface, stage 3.
//
// Contract between three writers (docs/dm-stage3-wave-plan-20260914.md):
//   A implements msgstore.cpp behind this header and hooks the receive path,
//   B renders /?page=mailbox from the readers below,
//   C sets the configuration from settings/commands.
// The core is platform-neutral (no Arduino): every clock, table lookup,
// random source and transmit goes through MsgStoreEnv, set once by the
// firmware glue and replaced by fakes in test/test_msgstore.
//
// Everything here is compiled only where ENABLE_MSGSTORE is defined
// (configuration_global.h: ESP32-S3 and nRF52840). Callers on other boards
// must guard their includes and calls with #if defined(ENABLE_MSGSTORE).
#pragma once

#include <stdint.h>
#include <stddef.h>

#define MSGSTORE_SLOTS_MAX     50    // build-time maximum, --storeslots 1..MSGSTORE_SLOTS_MAX (static table, ~190 B/slot)
#define MSGSTORE_SLOTS_DEFAULT 50
#define MSGSTORE_PAYLOAD_MAX   160   // stripped DM text, the on-air text limit
#define MSGSTORE_CALL_MAX      10
#define MSGSTORE_LIST_MAX      16    // --storecall entries (list mode)
#define MSGSTORE_HOLD_DEFAULT_H 24
#define MSGSTORE_HOLD_MAX_H    168

// D5 schedule and §3.5 caps (verdict-amended). Milliseconds.
#define MSGSTORE_LADDER_STEPS      9
#define MSGSTORE_STEP_MS           40000UL      // 3x40 s ...
#define MSGSTORE_BLOCK_GAP_MS      60000UL      // ... +1 min between the three blocks
#define MSGSTORE_COOLDOWN_MS       3600000UL    // 1 h after a ladder without ack
#define MSGSTORE_JITTER_MIN_MS     5000UL
#define MSGSTORE_JITTER_MAX_MS     60000UL
#define MSGSTORE_ACTION_GAP_MS     30000UL      // one mailbox action per 30 s per node
#define MSGSTORE_ACTIONS_PER_HOUR  20           // load-bearing node ceiling
#define MSGSTORE_UTIL_MAX_PCT      25           // no action above this 5-min utilisation
#define MSGSTORE_HEARD_WINDOW_MS   43200000UL   // 12 h, the mheard window (D4)

enum MsgStoreMode  { MSGSTORE_OFF = 0, MSGSTORE_OWN = 1, MSGSTORE_LIST = 2, MSGSTORE_HEARD = 3 };
enum MsgStoreState { MSGSTORE_FREE = 0, MSGSTORE_HELD = 1, MSGSTORE_ARMED = 2, MSGSTORE_LADDER = 3, MSGSTORE_COOLDOWN = 4 };

struct MsgStoreEntry
{
    char     src[MSGSTORE_CALL_MAX];   // original sender
    char     dst[MSGSTORE_CALL_MAX];   // destination (in the store set at store time)
    uint16_t nnn;                      // {NNN transport sequence number
    uint16_t plen;                     // stripped payload length
    uint16_t pcrc;                     // low 16 bits of crc32 over the stripped payload (T12)
    char     payload[MSGSTORE_PAYLOAD_MAX + 1];
    uint32_t stored_ms;                // first sighting (monotonic)
    uint32_t next_ms;                  // next allowed action for this entry
    uint8_t  attempt;                  // 0..9 within the current ladder cycle
    uint8_t  cycles;                   // completed ladder cycles
    uint8_t  state;                    // MsgStoreState
    uint8_t  notice;                   // stage 4: 0 none, 1 pending (send :sto to the sender), 2 sent
    uint8_t  gen;                      // bumped on every hook-driven mutation of this
                                        // slot (purge, refresh/replace, peer cancel,
                                        // presence, any transition to FREE); msgstoreLoop()
                                        // reads it back after env->deliver() to tell a slot
                                        // the nRF52 LORA task rewrote mid-delivery from one
                                        // it can still safely stamp a ladder step onto
                                        // (verdict F4, docs/review/fable-dm-stage3-verdict-20260914.md).
};

struct MsgStoreCounters
{
    uint32_t stored, refreshed, delivered, purged_ack,
             dropped_storetime, dropped_cap, dropped_slots,
             cancelled_peer, blocked_bp,
             notified, notice_blocked;   // stage 4: :sto notices sent / refused by the caps
};

// Environment the core runs against. All pointers required except log.
struct MsgStoreEnv
{
    uint32_t    (*now_ms)(void);
    const char *(*own_call)(void);
    int32_t     (*heard_age_ms)(const char *call);          // -1 = not in mheard
    int         (*bp_state)(void);                          // 0 QUIET, 1 QRS, 2 QRT
    uint8_t     (*util_pct)(void);                          // last 5-min window
    uint32_t    (*random_between)(uint32_t lo, uint32_t hi);
    bool        (*deliver)(const struct MsgStoreEntry *e);  // build + enqueue one frame at hop 0; true if enqueued
    void        (*log)(const char *line);                   // may be NULL
    // Stage 4, LAST on purpose so positional initialisers of older glue/tests
    // leave it NULL: build + enqueue the :sto text to e->src at max_hop_text.
    bool        (*notify)(const struct MsgStoreEntry *e);   // stage 4: build + enqueue the :sto text to e->src at max_hop_text; may be NULL
};

// ---- lifecycle / configuration (C writes, A reads) ----
void          msgstoreInit(const struct MsgStoreEnv *env);
void          msgstoreConfigure(enum MsgStoreMode mode, uint8_t slots, uint16_t hold_hours);
void          msgstoreSetList(const char *csv);          // list mode, "CALL1,CALL2,..."
void          msgstoreSetNotice(bool on);                // stage 4: --storenotice on|off (default on)
bool          msgstoreNotice(void);
enum MsgStoreMode msgstoreMode(void);
uint8_t       msgstoreSlots(void);
uint16_t      msgstoreHoldHours(void);
const char   *msgstoreListCsv(void);
const char   *msgstoreModeName(enum MsgStoreMode mode);
const char   *msgstoreStateName(uint8_t state);

// ---- receive-path events (A hooks; never from msg_server frames) ----
bool msgstoreEligible(const char *dst);                     // store-set test, false when mode is OFF
int  msgstoreStore(const char *src, const char *dst, uint16_t nnn,
                   const char *payload, size_t len);        // slot, -1 dropped; same (src,nnn,payload) refreshes
void msgstoreOnAck(const char *acker, const char *sender, uint16_t nnn);   // :ackNNN from acker to sender
void msgstorePresence(const char *call);                    // frame heard DIRECTLY from call
void msgstoreOnPeerDelivery(const char *src, uint16_t nnn); // another store node's hop-0 delivery heard

// ---- loop task ----
void msgstoreLoop(void);

// ---- operator actions (B page, C commands) ----
bool msgstorePurge(int slot);
void msgstorePurgeAll(void);
bool msgstoreDeliverNow(int slot);                          // starts a ladder now, still subject to node caps

// ---- readers ----
int                            msgstoreUsed(void);
const struct MsgStoreEntry    *msgstoreEntry(int slot);     // NULL if out of range or FREE
const struct MsgStoreCounters *msgstoreCounters(void);
uint8_t                        msgstoreActionsLastHour(void);
uint32_t                       msgstoreNextActionInMs(void); // 0 when nothing is due
int                            msgstoreFormatLine(char *buf, size_t n); // "MBOX mode=.. used=../.. ..." setlog line

// ---- test-only ----
void msgstoreReset(void);

// Firmware glue (src/msgstore_glue.cpp): installs the Arduino-side MsgStoreEnv.
// Called once at boot from the platform main before msgstoreSettingsLoad().
void msgstoreGlueInit(void);    // zero table, counters, action ring and config back to boot defaults
