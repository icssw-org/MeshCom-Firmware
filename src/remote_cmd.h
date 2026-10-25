// RM-02: protocol core of the authenticated remote management DM ("RM1").
// Issue #1189.
//
// Arduino-free, no printf, no malloc: parse, canonical string, key, tag
// verification, allowlist, counter / replay window, rate limit and lockout,
// reply build. Nothing here executes a command or touches flash; the caller
// (RM-03, loop task only) runs the command after RM_OK and persists the
// returned high-water mark.
//
// Wire format:  RM1 <ctr> <cmd>[ <args>] <tag>
//   tag = first 8 bytes (16 hex chars) of HMAC-SHA256(K, canonical)
//   K   = SHA-256(node_passwd with trailing spaces stripped)
//   canonical = "RM1|" dst "|" src "|" ctr "|" cmd [" " args]
// Reply:        RM1 <ctr> ok <status> <rtag>  /  RM1 <ctr> err <reason> <rtag>
//   rtag over "RM1R|" dst "|" src "|" ctr "|" result

#ifndef REMOTE_CMD_H
#define REMOTE_CMD_H

#include <stddef.h>
#include <stdint.h>

enum RmVerdict : uint8_t
{
    RM_OK = 0,        // execute, then rmAccept() and reply
    RM_CACHED,        // same ctr + valid tag within RM_CACHE_MS: re-send reply, do not execute
    RM_SYNC,          // valid "sync": reply "ok ctr=<hwm> v=<version>", no rmAccept()
    RM_REJ_FORMAT,    // silent
    RM_REJ_TAG,       // silent
    RM_REJ_REPLAY,    // silent
    RM_REJ_BLOCKED,   // a valid-tag sync with arguments (everything else valid-tag is RM_REFUSED); silent
    RM_REJ_RATE,      // silent
    RM_REJ_LOCKOUT,   // silent
    RM_REJ_DISABLED,  // empty password; silent
    RM_REFUSED        // valid tag, but not on the allowlist / bad args / range: answered (rmRefusalText), no strike
};

// "ok","cached","sync","format","tag","replay","blocked","rate","lockout","disabled","refused"
const char *rmVerdictName(RmVerdict v);

#define RM_RATE_MS 10000u        // minimum spacing of accepted commands
#define RM_SYNC_RATE_MS 60000u   // minimum spacing of accepted syncs (own limiter, independent of RM_RATE_MS)
#define RM_CACHE_MS 600000u      // lost-reply recovery window (10 min)
#define RM_REJ_WINDOW_MS 90000u  // 3 rejects inside this window ...
#define RM_REJ_LIMIT 3
#define RM_LOCKOUT_MS 300000u    // ... lock RM1 for 5 min
#define RM_MAX_RESULT 108         // longest result text a reply can carry; "RM1 <10 digits> <108> <16 hex>" = 140 chars
#define RM_MAX_ARGS 39            // longest args text of a command (RmCmd::args holds this + NUL)

struct RmCmd
{
    uint32_t ctr;
    char cmd[16];
    char args[RM_MAX_ARGS + 1];
    char tag[17]; // 16 lower-case hex + NUL
};

// Syntax only: "RM1 " prefix, single spaces, no leading/trailing space,
// decimal ctr 1..4294967295 (no leading zeros; 0 only with cmd "sync"),
// lower-case cmd/args, 16 lower-case hex tag. Allowlist is rmCheck()'s job.
bool rmParse(const char *text, RmCmd &out);
// True for a REPLY text "RM1 <ctr> ok ..." / "RM1 <ctr> err ..." (never a command: no allowlisted
// command is named ok/err). The receive hook shows replies to the operator instead of queueing them.
bool rmIsReply(const char *text);

// K = SHA-256(passwd with trailing spaces stripped); all-zero key for empty.
void rmDeriveKey(const char *passwd, uint8_t key[32]);

// Canonical string into out (NUL-terminated); returns its length, 0 if it
// does not fit or an argument is null.
size_t rmCanonical(const RmCmd &c, const char *dst, const char *src, char *out, size_t n);

// Reply canonical "RM1R|<dst>|<src>|<ctr>|<result>": the ONE builder behind the reply signer (rmReply) and
// the verifier (rmVerifyReply). Returns its length, 0 if it does not fit or an argument is null.
size_t rmReplyCanonical(uint32_t ctr, const char *dst, const char *src, const char *result, char *out, size_t n);

// Reject counter and lockout are kept per sender (callsign-SSID as passed to rmCheck), whatever path
// the frame took: one sender's wrong frames never lock another sender.
// "Same guess = one strike": the frames of the counted strikes of the current window are remembered per sender.
// A window holds at most RM_REJ_LIMIT - 1 counted strikes (the last one locks and forgets them), so this ring
// holds all of them. Identity = ctr + 16 tag chars + FNV-1a of "cmd args" (the tag covers cmd and args: holding
// ctr + tag fixed while they vary is a new HMAC guess each time). No per-frame time: the window starts with the
// first remembered strike, and a row whose window is over is swept, so every remembered frame is inside it.
#define RM_REJ_FRAMES (RM_REJ_LIMIT - 1)
struct RmRejFrame
{
    uint32_t ctr;
    uint32_t cmdArgsHash; // FNV-1a over cmd, one space, args (each bounded by its field size)
    char tag[16];         // raw, not NUL-terminated
};

#define RM_REJ_SOURCES 6 // senders whose rejects are tracked at once (least recently used is replaced)
struct RmRejSrc
{
    char call[10];        // sender call incl. SSID, as passed to rmCheck (exact compare)
    bool used;
    bool lockActive;
    uint8_t rejCount;     // rejects inside the current window
    uint32_t rejWindowMs; // start of the reject window
    uint32_t lockUntilMs;
    uint32_t lastMs;      // time of the last counted reject (replacement order)
    uint8_t nFrames;      // counted strikes of the current window whose frame is remembered (<= RM_REJ_FRAMES)
    uint8_t nextFrame;    // ring slot the next counted strike writes
    RmRejFrame frame[RM_REJ_FRAMES]; // the identical frame again is the same guess, no new strike
};

struct RmState
{
    uint32_t hwm;           // persisted high-water mark
    uint32_t lastCtr;       // last accepted command
    char lastReply[RM_MAX_RESULT + 1]; // RESULT text of the last accepted command ("ok rebooting");
                            // the caller re-sends rmReply(cmd, lastReply, ...) on RM_CACHED
    uint32_t lastAcceptMs;  // acceptance time of lastCtr (cache window)
    RmRejSrc rej[RM_REJ_SOURCES]; // per-sender reject counter and lockout
    char lastTag[17];       // tag of the last accepted command
    bool haveLast;
    // appended by RM-02 (rmStateInit() sets them, callers never touch them):
    uint32_t lastRateMs;    // time of the last accepted command (rate limit; sync does not touch it)
    bool haveRate;
    // appended by W0a: sync has its own limiter so a replayed sync can neither starve commands nor
    // be answered more than once per RM_SYNC_RATE_MS:
    uint32_t lastSyncMs;    // time of the last accepted sync
    bool haveSync;
    // appended by BF-01: strict security (node_rmstrict). 0 (the default, rmStateInit) = nothing is punished:
    // no strike is recorded, no sender locks, the rate limiters RM_RATE_MS / RM_SYNC_RATE_MS are skipped.
    // Correctness checks (format, allowlist, tag, replay counter) and the cached-reply path are unaffected.
    // The caller (rm_runtime.cpp) sets it from the setting before every rmRejSweep()/rmCheck().
    bool strict;
};

void rmStateInit(RmState &s, uint32_t hwm);

// Clears the lockout and the reject counter of EVERY sender (the whole table), nothing else: hwm,
// lastCtr, the reply cache, the rate limiter and the sync limiter stay. The operator standing at the node (own password change,
// console or web) may always unlock it; it is never reachable from the air.
void rmReceiverUnlock(RmState &s);

// True while `src` is locked out at nowMs (read-only, never changes state). A call longer than 9
// characters is compared and stored as its first 9; nullptr is the empty call "".
bool rmSenderLocked(const RmState &s, const char *src, uint32_t nowMs);
// Number of senders locked at nowMs; *maxRemainMs (may be nullptr) = longest remaining lock time in
// ms, 0 if none.
uint8_t rmLockedSenders(const RmState &s, uint32_t nowMs, uint32_t *maxRemainMs);

// Ages the reject table: an expired lock is cleared and an entry without a lock whose window is over
// is freed. rmCheck() calls it first; the firmware also calls it from its loop (rmDrain), because an
// entry nobody touches for 2^31 ms (24.8 days) would otherwise read as locked again.
void rmRejSweep(RmState &s, uint32_t nowMs);

// Full check incl. allowlist, counter, rate limit, lockout. Does NOT execute. Lockout and both rate
// limits apply only while s.strict is set (BF-01); without it only format, allowlist, tag and the
// replay counter can reject.
// minTxPower..maxTxPower bounds "txpower <n>". Order: disabled, lockout, format, canonical + tag, then (not for
// sync) cache, replay, rate, and the allowlist LAST: a command that fails it with a valid tag is RM_REFUSED (answered,
// no strike; the caller treats it like RM_OK without executing). Every reject counts
// towards the lockout, except RM_REJ_DISABLED, RM_REJ_LOCKOUT, RM_REJ_RATE and
// RM_REJ_REPLAY (each of the last two requires a VALID tag: the lockout answers wrong
// keys, a valid tag is not one, and a replayed or overtaken frame reveals and
// executes nothing; counting it let a jittery legitimate path lock the node). A strike
// exactly RM_REJ_WINDOW_MS after the window start opens a new window. The lockout is reachable without the
// key (3 junk DMs per 5 min keep RM unavailable): accepted by design, RM fails
// closed, never open (ADR). RM_SYNC
// has its own limiter (RM_SYNC_RATE_MS, a second authenticated sync inside it is RM_REJ_RATE, silent):
// it neither stamps nor obeys the command limiter, and it never moves hwm.
RmVerdict rmCheck(RmState &s, const RmCmd &c, const char *dst, const char *src, const char *passwd,
                  int minTxPower, int maxTxPower, uint32_t nowMs);

// Reply text for an RM_REFUSED command: "err range <min>..<max>" for a txpower whose args fail the range or shape,
// "err blocked" otherwise. Returns the length (0 if out is null or n is 0).
size_t rmRefusalText(const RmCmd &c, int minTxPower, int maxTxPower, char *out, size_t n);

// After the caller executed an RM_OK command (or answered an RM_REFUSED one without executing): records hwm,
// result, time.
// CONTRACT: call it for EVERY RM_OK and EVERY RM_REFUSED, also when execution failed (pass the
// "err <reason>" result) -- otherwise the same ctr stays executable and the
// rate limiter is not stamped.
// Returns the new hwm to persist. result is truncated to RM_MAX_RESULT.
uint32_t rmAccept(RmState &s, const RmCmd &c, const char *result, uint32_t nowMs);

// RM-09 (sender side, web "remote management"): the allowlist of rmCheck() as a predicate on plain
// strings (cmd and args as they appear on the wire, lower case; "" for no args). Includes the
// forbidden-character rule; txpower is bounded by minTxPower..maxTxPower.
bool rmCommandAllowed(const char *cmd, const char *args, int minTxPower, int maxTxPower);

// RM-09: builds "RM1 <ctr> <cmd>[ <args>] <tag>" (what rmParse() accepts) with the 16-hex tag of
// HMAC-SHA256(key, canonical). dst = managed node, src = this node. key = rmDeriveKey() of the
// TARGET's password. Pure builder: the allowlist is rmCommandAllowed()'s job. Returns the length,
// 0 if it does not fit, an argument is null/oversized or cmd is empty.
size_t rmBuildCommand(const char *dst, const char *src, uint32_t ctr, const char *cmd, const char *args,
                      const uint8_t key[32], char *out, size_t n);

// RM-09: checks a REPLY text "RM1 <ctr> <result> <rtag>" against dst (managed node), src (this node),
// the expected ctr and the key; the tag covers "RM1R|dst|src|ctr|result". On success copies the result
// ("ok ..." / "err ...") to result and returns true; false on any mismatch (result untouched then),
// also when result does not fit n.
bool rmVerifyReply(const char *text, const char *dst, const char *src, uint32_t ctr, const uint8_t key[32],
                   char *result, size_t n);

// "RM1 <ctr> <result> <rtag>" into out (NUL-terminated); returns the length,
// 0 if it does not fit, result is longer than RM_MAX_RESULT or the password is empty.
size_t rmReply(const RmCmd &c, const char *result, const char *dst, const char *src, const char *passwd,
               char *out, size_t n);

// Pure result sanitiser: allowed bytes are space, A-Z a-z 0-9 and "- . / = + _ @ ? ( ) , * #";
// every other byte becomes '?'. In place, returns the length.
size_t rmSanitizeResult(char *result);

#endif // REMOTE_CMD_H
