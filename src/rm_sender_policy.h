// rm_sender_policy.h -- pure sender-side policy of the RM web GUI.
//
// Header-only, no Arduino, no allocation, no clock access: every function takes "now" and all time
// math is unsigned 32-bit subtraction, so a millis() wrap at 2^32 is harmless.
//
//  1. Send policy per target: a rejected RM1 frame (wrong key, stale counter, even a wrong-key sync)
//     is SILENT on air and counts toward the target's lockout of THIS sender's call (3 within 90 s =
//     5 min locked, per sender; the
//     sender can not see any of it). So the sender allows at most N unanswered sends to one target
//     inside RM_POLICY_WINDOW_MS (RM_POLICY_WINDOW_MS = ladder 126 s + 90 s reject window + 30 s path delay + 30 s margin) and
//     spaces sends to one target by 10 s (the target rate limits to one per 10 s). N is 2 until a
//     reply under the current key verified; then N is 10 (the target does not count authenticated
//     replays toward its lockout). After a re-key (key fingerprint changed) the proof is gone and
//     ONE forced attempt beyond the limit is allowed. Two sends in a row without a verified reply
//     suspend the proof (rmPolicyLimit): the target may have been re-keyed.
//     BF-01: this whole policy is active only with strict security on (node_rmstrict, default off):
//     rmPolicyMaySend(..., strict = false) allows every send and spends no one-shot.
//  2. Entry state: queued / waiting / noanswer / ok / err / unverified as a function of timestamps.
//  3. Plain sentences for every state and every RM error token.
//  4. The compact `status` reply: `s=<letters> p=<cur>/<min>/<max>` encode/decode and the whole formatter,
//     so the 108-char reply limit (RM_MAX_RESULT) is asserted on the real code in a native test.
#ifndef RM_SENDER_POLICY_H
#define RM_SENDER_POLICY_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ---- constants (RM_POLICY_COOLDOWN_MS must equal RM_RATE_MS of remote_cmd.h; rm_runtime.cpp asserts it)
#define RM_POLICY_COOLDOWN_MS 10000u   // minimum spacing of two sends to one target
// RM1 repeats (wave 3): one send = copy 1 at 0 s plus RM_REPEAT_COPIES-1 ring repeats on the PN ladder
// (0x15 ticks of 2 s = 42 s, lora_functions.cpp). All copies carry the same ctr and tag, so they are ONE send in the
// budget and the sent book; the reply to the last copy can still be in flight RM_REPLY_ALLOWANCE_MS after it.
#define RM_REPEAT_COPIES 4u            // copy 1 + 3 repeats (MAX_RETRANSMIT 3)
#define RM_REPEAT_GAP_MS 42000u        // ladder step: 0x15 ticks x 2 s
#define RM_REPEAT_LAST_MS ((RM_REPEAT_COPIES - 1u) * RM_REPEAT_GAP_MS)   // last copy leaves 126 s after the send
#define RM_REPLY_ALLOWANCE_MS 45000u   // reply allowance after the last copy (the old single-copy order: ~75 s - 30 s)
#define RM_POLICY_WINDOW_MS (RM_REPEAT_LAST_MS + 150000u)
                                       // window of the unanswered-send budget = last copy 126 s + (target reject window
                                       // 90 s + 30 s path delay + 30 s margin for TX-queue latency; equal windows leak at
                                       // the edge: path delay jitter, verdict finding 4; a 120 s window still leaked at
                                       // 30 s delay, D2) = 276 s, so a send is counted until its LAST copy has aged out
#define RM_POLICY_MAX_UNANSWERED 2u    // budget while the key is UNPROVEN: a 3rd unanswered send would be the target's 3rd strike
#define RM_POLICY_LIMIT_UNPROVEN RM_POLICY_MAX_UNANSWERED
#define RM_POLICY_LIMIT_PROVEN 10u     // budget once a reply under the current key fingerprint verified
#define RM_NOANSWER_MS (RM_REPEAT_LAST_MS + RM_REPLY_ALLOWANCE_MS)   // 171 s: end of the ladder + reply allowance = "no answer"
static_assert(RM_NOANSWER_MS >= RM_REPEAT_LAST_MS + RM_REPLY_ALLOWANCE_MS, "no answer is declared only after the last copy had its chance");
static_assert(RM_POLICY_WINDOW_MS >= RM_NOANSWER_MS + 60000u, "the unanswered-send budget must outlive the no-answer verdict by the path delay + margin (60 s), else a send leaves the budget before it is declared unanswered");
#define RM_QUEUED_MS 3000u             // first seconds after the send: still in the local TX queue
#define RM_POLICY_MAX_ENTRIES 12u      // entries per target the policy looks at (= the sent book, kSentN)
static_assert(RM_POLICY_MAX_ENTRIES >= RM_POLICY_LIMIT_PROVEN + 2, "book covers the proven budget plus answered entries");

#define RM_PEND_SEND_DELAY_MS 10500u   // queued command goes out this long after the sync reply verified

// ---- entry state -------------------------------------------------------------------------------------
enum RmEntryState : uint8_t
{
    RM_ST_QUEUED = 0,  // handed to the TX ring, probably not on air yet
    RM_ST_WAITING,     // sent, no reply yet, inside the answer window
    RM_ST_NOANSWER,    // no reply after RM_NOANSWER_MS (or the entry outlived the key window)
    RM_ST_OK,          // verified reply "ok ..."
    RM_ST_ERR,         // verified reply "err ..."
    RM_ST_UNVERIFIED   // a reply arrived but its tag did not verify: never shown as success
};

// What the policy needs to know about one send. The caller fills it from its sent book.
struct RmPolEntry
{
    uint32_t sentMs;  // millis() at the send
    bool replied;     // some reply arrived
    bool verified;    // ... and its tag matched
    bool replyErr;    // verified reply starts with "err"
    bool expired;     // older than the reply window the caller tracks (aged out; guards the 2^32 wrap)
    bool isSync;      // a counter sync (ctr 0), not a command; only rmPeerMarkMisses() reads it (set it, memset 0 = command)
};

inline uint32_t rmPolicyAgeMs(uint32_t nowMs, uint32_t thenMs)
{
    return (uint32_t)(nowMs - thenMs);
}

inline RmEntryState rmPolicyState(const RmPolEntry &e, uint32_t nowMs)
{
    if (e.replied)
    {
        if (!e.verified)
            return RM_ST_UNVERIFIED;
        return e.replyErr ? RM_ST_ERR : RM_ST_OK;
    }
    if (e.expired)
        return RM_ST_NOANSWER;
    const uint32_t age = rmPolicyAgeMs(nowMs, e.sentMs);
    if (age >= RM_NOANSWER_MS)
        return RM_ST_NOANSWER;
    if (age < RM_QUEUED_MS)
        return RM_ST_QUEUED;
    return RM_ST_WAITING;
}

inline const char *rmStateName(RmEntryState s)
{
    switch (s)
    {
    case RM_ST_QUEUED:     return "queued";
    case RM_ST_WAITING:    return "waiting";
    case RM_ST_NOANSWER:   return "noanswer";
    case RM_ST_OK:         return "ok";
    case RM_ST_ERR:        return "err";
    case RM_ST_UNVERIFIED: return "unverified";
    }
    return "waiting";
}

// ---- send policy -------------------------------------------------------------------------------------
enum RmPolReason : uint8_t
{
    RM_POL_OK = 0,
    RM_POL_COOLDOWN,  // inside the 10 s spacing after the last send to this target
    RM_POL_LIMIT      // unanswered budget used up (2 unproven / 10 proven) inside the window
};

struct RmPolDecision
{
    bool allowed;
    RmPolReason reason;
    uint32_t retryS;  // seconds until a send is allowed again (0 when allowed); cooldown included
    uint8_t unanswered;  // unanswered sends inside the window
    bool canForce;       // refused with LIMIT and the one-shot after a re-key is armed (send with force=1)
    bool usedForce;      // allowed only because the one-shot was spent: the caller must disarm it
};

inline uint32_t rmPolicyCeilS(uint32_t ms)
{
    return (ms + 999u) / 1000u;
}

// e[0..n) = ALL entries of ONE target (any order). Unanswered = not verified, not expired, younger
// than the window. A verified err reply counts as answered: the target is alive and the key is right.
// Boundaries: age 10000 ms is out of the cooldown; age RM_POLICY_WINDOW_MS is out of the window.
inline RmPolDecision rmPolicyMaySend(const RmPolEntry *e, uint8_t n, uint32_t nowMs,
                                      uint8_t limit = RM_POLICY_LIMIT_UNPROVEN, bool oneShot = false, bool force = false,
                                      bool strict = true)
{
    RmPolDecision d = {true, RM_POL_OK, 0, 0, false, false};
    // BF-01 (D3): without strict security the whole policy (budget, cooldown, one-shot force) is off
    if (e == nullptr || !strict)
        return d;
    if (n > RM_POLICY_MAX_ENTRIES)
        n = (uint8_t)RM_POLICY_MAX_ENTRIES;

    uint32_t cooldownLeft = 0;
    uint32_t unans[RM_POLICY_MAX_ENTRIES];  // ages of the unanswered ones, sorted descending (oldest first)
    uint8_t k = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        const uint32_t age = rmPolicyAgeMs(nowMs, e[i].sentMs);
        if (!e[i].expired && age < RM_POLICY_COOLDOWN_MS && RM_POLICY_COOLDOWN_MS - age > cooldownLeft)
            cooldownLeft = RM_POLICY_COOLDOWN_MS - age;
        if (e[i].verified || e[i].expired || age >= RM_POLICY_WINDOW_MS)
            continue;
        uint8_t j = k++;
        while (j > 0 && unans[j - 1] < age)
        {
            unans[j] = unans[j - 1];
            j--;
        }
        unans[j] = age;
    }
    d.unanswered = k;

    uint32_t limitLeft = 0;
    if (k >= limit && oneShot && force && cooldownLeft == 0)
    {
        d.usedForce = true;  // the one-shot after a re-key: one send beyond the limit, the spacing still applies
        return d;
    }
    if (k >= limit)
    {
        d.canForce = oneShot;
        // the count must fall below the limit: the (k - limit + 1) oldest have to leave the window
        const uint8_t idx = (uint8_t)(k - limit);  // 0-based among oldest-first
        limitLeft = RM_POLICY_WINDOW_MS - unans[idx];
        d.reason = RM_POL_LIMIT;
    }
    if (d.reason != RM_POL_LIMIT && cooldownLeft > 0)
        d.reason = RM_POL_COOLDOWN;

    const uint32_t left = limitLeft > cooldownLeft ? limitLeft : cooldownLeft;
    if (left > 0)
    {
        d.allowed = false;
        d.retryS = rmPolicyCeilS(left);
    }
    else
        d.reason = RM_POL_OK;
    return d;
}

// Automatic counter sync: only a sender WITHOUT a trusted clock (ctr = time would be valid otherwise)
// that has NO learnt counter mark for the target needs one. It is a condition, never a button, and
// the sync command itself never triggers another.
inline bool rmPolicyNeedSync(bool clockTrusted, bool haveTargetMark, bool cmdIsSync)
{
    return !cmdIsSync && !clockTrusted && !haveTargetMark;
}

// CR-02: a learnt counter mark (the target's HWM as of the last verified reply) goes stale when the
// target's HWM moves on without this sender, e.g. a clock-based sender (ctr about 1.79e9) manages the same
// node. The mark-based sender then sends ctr = mark + 1, which the target rejects as a replay: silent on
// air, and rmPolicyNeedSync() stays false because a mark exists. Recovery: after RM_MARK_STALE_UNANSWERED
// dead commands in a row the mark is dropped, so the next send runs the automatic sync again.
// Flag independent: this is correctness, not a penalty. N = 2 = RM_POLICY_MAX_UNANSWERED: one lost frame
// is no evidence, two commands in a row that each waited out RM_NOANSWER_MS are; a false positive costs one
// sync frame; and under strict security the budget of 2 refuses the third send exactly until the first
// dead one leaves the window, which is when this rule fires.
#define RM_MARK_STALE_UNANSWERED RM_POLICY_MAX_UNANSWERED

// e[0..n) = ALL entries of ONE target (any order). Counts the commands (not syncs) that got no reply at all
// and are dead (expired, or older than RM_NOANSWER_MS) and newer than the newest verified entry of any kind
// (a verified reply, a sync included, refreshed the mark). A reply with a bad tag is no answer but is also
// not silence, so it is not counted. Expired entries have no usable age: they count unless a LIVE (not
// expired) verified entry exists, which is surely newer than any expired one. With only an expired verified
// entry the order is unknown and they count anyway: a user retrying slower than RM_CACHE_MS (10 min) must
// still recover, and a false positive costs one sync frame.
inline uint8_t rmPeerMarkMisses(const RmPolEntry *e, uint8_t n, uint32_t nowMs)
{
    if (e == nullptr)
        return 0;
    if (n > RM_POLICY_MAX_ENTRIES)
        n = (uint8_t)RM_POLICY_MAX_ENTRIES;
    bool haveLiveVer = false;
    uint32_t verAge = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        if (!e[i].verified)
            continue;
        if (e[i].expired)
            continue;
        const uint32_t age = rmPolicyAgeMs(nowMs, e[i].sentMs);
        if (!haveLiveVer || age < verAge)
        {
            haveLiveVer = true;
            verAge = age;
        }
    }
    uint8_t k = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        if (e[i].isSync || e[i].verified || e[i].replied)
            continue;
        if (e[i].expired)
        {
            if (!haveLiveVer)
                k++;
            continue;
        }
        const uint32_t age = rmPolicyAgeMs(nowMs, e[i].sentMs);
        if (age >= RM_NOANSWER_MS && (!haveLiveVer || age < verAge))
            k++;
    }
    return k;
}

inline bool rmPeerMarkStale(const RmPolEntry *e, uint8_t n, uint32_t nowMs)
{
    return rmPeerMarkMisses(e, n, nowMs) >= RM_MARK_STALE_UNANSWERED;
}

// Airtime guard for replies a node gives to frames ANYONE can replay (RM_CACHED, RM_SYNC): at most one
// per spacingMs, whatever the strict flag says. true = suppress this one. haveLast = a reply was sent
// before; unsigned subtraction, millis() wrap safe. Receiver side; the caller keeps the two statics.
inline bool rmReplyTooSoon(bool haveLast, uint32_t lastMs, uint32_t nowMs, uint32_t spacingMs)
{
    return haveLast && (uint32_t)(nowMs - lastMs) < spacingMs;
}

// Chain of an automatic sync: the command waits for its sync. Pure decision, called every loop pass.
//   syncGone     the sync entry left the sent book
//   syncExpired  the sync entry is older than the key window (aged out)
//   syncVerified a reply to the sync arrived AND its tag verified
//   syncReplied  some reply arrived (verified or not); IGNORED on purpose: an unverified reply is not
//                an answer, so it must not keep the chain alive (hang found by the advisor pass)
//   ageMs        now - sync send time
//   sinceVerifiedMs  now - time the verified reply was booked (only read when syncVerified)
// SEND only RM_PEND_SEND_DELAY_MS after a VERIFIED sync reply (the sender's own RM_POLICY_COOLDOWN_MS
// spacing after the sync send; the target's sync limiter is separate since W0a). DROP when the sync is gone or expired, or when it is not verified and RM_NOANSWER_MS have
// passed, whatever replied says. WAIT otherwise. All ages are unsigned differences (millis() wrap safe).
enum RmPendAction : uint8_t
{
    RM_PEND_WAIT = 0,
    RM_PEND_SEND,
    RM_PEND_DROP
};

inline RmPendAction rmPendingDecide(bool syncVerified, bool syncReplied, bool syncExpired, bool syncGone,
                                    uint32_t ageMs, uint32_t sinceVerifiedMs)
{
    (void)syncReplied;
    if (syncGone || syncExpired)
        return RM_PEND_DROP;
    if (syncVerified)
        return sinceVerifiedMs >= RM_PEND_SEND_DELAY_MS ? RM_PEND_SEND : RM_PEND_WAIT;
    return ageMs >= RM_NOANSWER_MS ? RM_PEND_DROP : RM_PEND_WAIT;
}

// ---- RM1 repeats: reply matching and copy cancel (wave 3) -----------------------------------------------
// A reply may be booked against a sent entry only while the entry still holds its key, is not verified yet and
// ctr and target match. A second reply for the same ctr (RF and server path, or the cached reply to a later copy)
// finds the entry verified (key wiped) and matches nothing: no second verification, no second count, no second row.
inline bool rmReplyMatchesEntry(bool keyLive, bool verified, bool ctrEq, bool dstEq)
{
    return keyLive && !verified && ctrEq && dstEq;
}

// The ring copies of one send are stopped through the ORIGINAL msg_id, exactly once (verified reply, forget of the
// target, key expiry, eviction from the book). take() hands the id out the first time only.
struct RmCopyCancel
{
    uint32_t msgId;
    bool live;
};
inline void rmCopyCancelArm(RmCopyCancel &c, uint32_t msgId)
{
    c.msgId = msgId;
    c.live = true;
}
inline bool rmCopyCancelTake(RmCopyCancel &c, uint32_t *msgId)
{
    if (!c.live)
        return false;
    c.live = false;
    *msgId = c.msgId;
    return true;
}

// ---- reply text helpers ------------------------------------------------------------------------------
inline bool rmReplyIsErr(const char *reply)
{
    return reply != nullptr && strncmp(reply, "err", 3) == 0 && (reply[3] == '\0' || reply[3] == ' ');
}

// ---- plain-English messages --------------------------------------------------------------------------
inline const char *rmStateMessage(RmEntryState s)
{
    switch (s)
    {
    case RM_ST_QUEUED:
        return "Handed to the radio. It goes on air in a moment.";
    case RM_ST_WAITING:
        return "Sent. Waiting for the node to answer, this can take up to a minute.";
    case RM_ST_NOANSWER:
        return "No answer after 75 seconds. Check the password, the distance and that remote "
               "management is on at the other node. With strict security on, a third failed try locks you "
               "out for 5 minutes.";
    case RM_ST_OK:
        return "Done. The node confirmed the command.";
    case RM_ST_ERR:
        return "The node received the command but could not do it.";
    case RM_ST_UNVERIFIED:
        return "An answer arrived but it could not be verified. Do not trust it.";
    }
    return "";
}

struct RmTokenMsg
{
    const char *token;
    const char *msg;
};

// Every RM error token. The first group is what a reply can carry ("err <token>") or a target counts
// silently (verdict names); the second group is what the sender itself refuses with.
inline const RmTokenMsg *rmTokenTable(size_t *n)
{
    static const RmTokenMsg t[] = {
        // reply / verdict tokens
        {"failed", "The node tried, but the setting did not change."},
        {"not output", "That pin is not set as an output on the node."},
        {"unsupported", "This board does not have that feature."},
        {"storage", "The node could not save its counter, so it did not run the command."},
        {"blocked", "The node does not accept this command or value."},
        {"rate", "Too fast. The node accepts one command every 10 seconds."},
        {"lockout", "The node has locked you out for 5 minutes after too many wrong tries."},
        {"replay", "The node has seen this counter already. Try again."},
        {"tag", "The password does not match the one on the node."},
        {"sync", "The counters are out of step. Check the connection first, then try again."},
        {"format", "The node could not read the message."},
        {"disabled", "Remote management is off on the node, or it has no password."},
        {"cached", "The node repeated its earlier answer."},
        // reply tokens of the extended commands (rm_exec_read.cpp / rm_exec_write.cpp)
        {"range", "That value is outside the range the node accepts."},
        {"text", "The text contains characters the node will not accept."},
        {"unknown", "The node does not know that one."},
        {"end", "There are no more rows."},
        {"gps", "The position is controlled by GPS on that node."},
        {"hidden", "That node does not send its position, so it is not shown."},
        // reply tokens of upd (rm_exec_write.cpp)
        {"aubusy", "The node is busy with an update. Try again in a moment."},
        {"nowifi", "The node has no WiFi connection, so it cannot reach the update server."},
        {"stale", "The update offered on the node has changed. Read the update state again."},
        {"current", "The node knows no newer release. Check for updates first."},
        {"noasset", "The update server has no firmware file for this node."},
        {"safeboot", "The node cannot apply an update: its Safeboot is missing or too old, or it already tried this one."},
        {"notready", "The node is not ready to download: its clock is not set, a phone is connected, or its battery is too low."},
        // sender-side refusals
        {"limit", "Two tries to this node are still unanswered. Wait before trying again, a third "
                  "wrong try could lock you out for 5 minutes."},
        {"busy", "Wait a few seconds. This node takes one command every 10 seconds."},
        {"passwd", "The password is not valid: 1 to 14 plain characters, no space at the start or end."},
        {"dst", "That call sign is not valid, or it is this node."},
        {"cmd", "That command is not allowed."},
        {"ctr", "This node ran out of counter values."},
        {"store", "This node could not save that. Try again."},
        {"send", "The radio queue is full. Try again in a moment."},
        {"size", "The request was too large."},
        {"short", "The request was incomplete."},
        {"form", "The request could not be read."},
        // web request refusals (web_rm_parse.h, web_rm_handlers.cpp)
        {"pw", "The password is not valid: 1 to 14 plain characters, no space at the start or end."},
        {"call", "That call sign is not valid."},
        {"act", "The request was not understood."},
        {"slot", "The saved node changed. Reload the page."},
        {"dup", "This node is already saved in another place."},
        // chain outcomes (a command queued behind an automatic sync that was then not sent)
        {"nosync", "The connection check got no answer, so your command was not sent. Check the "
                   "password, the distance and that remote management is on at the other node."},
        {"lost", "The connection check did not finish, so your command was not sent. Try again."},
    };
    if (n != nullptr)
        *n = sizeof(t) / sizeof(t[0]);
    return t;
}

// token = exact token ("not output") or a text that starts with it followed by a space or the end
// ("failed", "err failed", "not output pin"). Unknown token: a generic sentence, never empty.
inline const char *rmErrTokenMessage(const char *token)
{
    if (token == nullptr)
        return "The node reported an error.";
    if (strncmp(token, "err ", 4) == 0)
        token += 4;
    size_t n = 0;
    const RmTokenMsg *t = rmTokenTable(&n);
    for (size_t i = 0; i < n; i++)
    {
        const size_t l = strlen(t[i].token);
        if (strncmp(token, t[i].token, l) == 0 && (token[l] == '\0' || token[l] == ' '))
            return t[i].msg;
    }
    return "The node reported an error.";
}

// The table's own (static) token string for a token text, or `fallback` when it is unknown. Lets a
// caller keep a token beyond the lifetime of a stack buffer.
inline const char *rmTokenStatic(const char *token, const char *fallback)
{
    if (token == nullptr)
        return fallback;
    size_t n = 0;
    const RmTokenMsg *t = rmTokenTable(&n);
    for (size_t i = 0; i < n; i++)
        if (strcmp(token, t[i].token) == 0)
            return t[i].token;
    return fallback;
}

// The sentence for one sent entry: the state sentence, or for err the sentence of the reply's token.
inline const char *rmEntryMessage(RmEntryState s, const char *reply)
{
    if (s == RM_ST_ERR)
        return rmErrTokenMessage(reply);
    return rmStateMessage(s);
}

// ---- compact status token ----------------------------------------------------------------------------
// Fixed order, one letter per switch: upper case = on, lower case = off.
//   G gps   T track   D display   M mesh   W gateway   L led (omitted on boards without an LED)
// plus p=<cur>/<min>/<max> for the TX power in dBm (min/max = the range the node accepts remotely).
// Example: "s=GtDMwL p=17/2/22".
#define RM_STATUS_SWITCHES 6u

struct RmSwitches
{
    bool gps, track, display, mesh, gateway, led;
    bool ledSupported;  // false: the led letter is left out (board without an LED)
};

struct RmStatusInfo
{
    bool haveS;                 // s= token present
    int8_t sw[RM_STATUS_SWITCHES];  // gps, track, display, mesh, gateway, led: -1 unknown, 0 off, 1 on
    bool ledSupported;          // led letter present
    bool haveP;
    int cur, min, max;
};

// "s=GtDMwL" into out. Returns the length, 0 if it does not fit.
inline size_t rmStatusToken(char *out, size_t n, const RmSwitches &s)
{
    if (out == nullptr)
        return 0;
    const bool v[RM_STATUS_SWITCHES] = {s.gps, s.track, s.display, s.mesh, s.gateway, s.led};
    static const char up[RM_STATUS_SWITCHES + 1] = "GTDMWL";
    const size_t cnt = s.ledSupported ? RM_STATUS_SWITCHES : RM_STATUS_SWITCHES - 1u;
    if (n < 2 + cnt + 1)
        return 0;
    out[0] = 's';
    out[1] = '=';
    for (size_t i = 0; i < cnt; i++)
        out[2 + i] = v[i] ? up[i] : (char)(up[i] - 'A' + 'a');
    out[2 + cnt] = '\0';
    return 2 + cnt;
}

inline int rmClampDbm(int v)
{
    return v < -99 ? -99 : (v > 99 ? 99 : v);
}

// The whole `status` result (without the "RM1 <ctr> " frame and tag):
//   ok v=<ver> up=<min> bat=<%> heap=<kB> s=<letters> p=<cur>/<min>/<max> rs=<0|1>
// rs= is node_rmstrict (strict RM security); the parser ignores it, the s= letters are unchanged.
// s= carries the switches (gateway and mesh are the W and M letters; the L letter only on boards with
// an LED). Returns the length written (always NUL-terminated; truncated, never overrun, when it does
// not fit n). Worst case is far below the wire cap RM_MAX_RESULT, see test_rm_sender_policy.
inline size_t rmFormatStatus(char *out, size_t n, const char *ver, uint32_t upMin, int bat, uint32_t heapKb,
                             const RmSwitches &sw, int txCur, int txMin, int txMax, bool strict)
{
    if (out == nullptr || n == 0)
        return 0;
    char tok[RM_STATUS_SWITCHES + 3];
    if (rmStatusToken(tok, sizeof(tok), sw) == 0)
        tok[0] = '\0';
    const int w = snprintf(out, n, "ok v=%s up=%lu bat=%d heap=%lu %s p=%d/%d/%d rs=%d", ver != nullptr ? ver : "?",
                           (unsigned long)upMin, bat, (unsigned long)heapKb, tok, rmClampDbm(txCur), rmClampDbm(txMin), rmClampDbm(txMax), strict ? 1 : 0);
    return w < 0 ? 0 : ((size_t)w >= n ? n - 1 : (size_t)w);
}

// Parses a status result (the compact form only). Returns false when it is not a status result ("ok v="),
// when s= is missing, or when its letter count is not 5/6 or a letter is wrong.
inline bool rmStatusParse(const char *res, RmStatusInfo &o)
{
    memset(&o, 0, sizeof(o));
    for (uint8_t i = 0; i < RM_STATUS_SWITCHES; i++)
        o.sw[i] = -1;
    if (res == nullptr || strncmp(res, "ok v=", 5) != 0)
        return false;

    const char *p = res;
    while (*p != '\0')
    {
        while (*p == ' ')
            p++;
        const char *t = p;
        while (*p != '\0' && *p != ' ')
            p++;
        const size_t l = (size_t)(p - t);
        if (l >= 3 && t[0] == 's' && t[1] == '=')
        {
            static const char up[RM_STATUS_SWITCHES + 1] = "GTDMWL";
            const size_t cnt = l - 2;
            if (cnt != RM_STATUS_SWITCHES && cnt != RM_STATUS_SWITCHES - 1u)
                return false;
            for (size_t i = 0; i < cnt; i++)
            {
                const char ch = t[2 + i];
                if (ch == up[i])
                    o.sw[i] = 1;
                else if (ch == (char)(up[i] - 'A' + 'a'))
                    o.sw[i] = 0;
                else
                    return false;
            }
            o.haveS = true;
            if (cnt == RM_STATUS_SWITCHES)
                o.ledSupported = true;
        }
        else if (l >= 7 && strncmp(t, "p=", 2) == 0)
        {
            int cur = 0, mn = 0, mx = 0, cn = 0;
            if (sscanf(t, "p=%d/%d/%d%n", &cur, &mn, &mx, &cn) == 3 && (size_t)cn == l)
            {
                o.haveP = true;
                o.cur = cur;
                o.min = mn;
                o.max = mx;
            }
        }
    }
    return o.haveS;
}


// ---- per-target TX power range (sender side, RAM only) ---------------------------------------------------
// The sender learns what a target accepts for `txpower` from VERIFIED replies only: `status` and `radio`
// carry p=<cur>/<min>/<max>, and the target answers an out-of-range value with `err range <min>..<max>`.
// Until a range is known, 0..RM_TXPOWER_UNKNOWN_MAX goes out and the target's own `err range` decides.
#define RM_TXRANGE_MAX 8
#define RM_TXPOWER_UNKNOWN_MAX 22
struct RmTxRange
{
    char dst[10];
    int8_t min, max;
    bool used;
    uint32_t lastUse;  // caller's clock, stamped on learn only; the least recently learnt row is evicted
};

inline bool rmTxRangeValid(int mn, int mx)
{
    return mn >= -99 && mx <= 99 && mn <= mx;
}

// Parses the p=<cur>/<min>/<max> token of a `radio` result ("ok f=... p=10/2/22"). Only the 3-part form.
inline bool rmRadioPParse(const char *res, int *cur, int *mn, int *mx)
{
    if (res == nullptr || strncmp(res, "ok ", 3) != 0)
        return false;
    const char *p = res + 2;
    while (*p != '\0')
    {
        while (*p == ' ')
            p++;
        const char *t = p;
        while (*p != '\0' && *p != ' ')
            p++;
        const size_t l = (size_t)(p - t);
        if (l >= 7 && strncmp(t, "p=", 2) == 0)
        {
            int c = 0, a = 0, b = 0, cn = 0;
            if (sscanf(t, "p=%d/%d/%d%n", &c, &a, &b, &cn) == 3 && (size_t)cn == l)
            {
                if (cur != nullptr)
                    *cur = c;
                if (mn != nullptr)
                    *mn = a;
                if (mx != nullptr)
                    *mx = b;
                return true;
            }
        }
    }
    return false;
}

// Parses "err range <min>..<max>" (the target's answer to an out-of-range txpower).
inline bool rmErrRangeParse(const char *res, int *mn, int *mx)
{
    if (res == nullptr || strncmp(res, "err range ", 10) != 0)
        return false;
    int a = 0, b = 0, cn = 0;
    if (sscanf(res + 10, "%d..%d%n", &a, &b, &cn) != 2 || res[10 + cn] != '\0')
        return false;
    if (mn != nullptr)
        *mn = a;
    if (mx != nullptr)
        *mx = b;
    return true;
}

// True when the sent command line "<cmd>[ <args>]" starts with the whole word `name`.
inline bool rmSentCmdIs(const char *line, const char *name)
{
    if (line == nullptr || name == nullptr)
        return false;
    const size_t l = strlen(name);
    return strncmp(line, name, l) == 0 && (line[l] == '\0' || line[l] == ' ');
}

// Which verified reply teaches the sender a target's txpower range: p=<cur>/<min>/<max> in the reply to `status`
// or `radio`, "err range <min>..<max>" in the reply to `txpower`. Every other command (`sens`: p is pressure,
// `maxhop`: p is hops, ...) is never read. Returns true with a valid range in *mn / *mx.
inline bool rmTxRangeFromReply(const char *sentCmd, const char *result, int *mn, int *mx)
{
    if (sentCmd == nullptr || result == nullptr || mn == nullptr || mx == nullptr)
        return false;
    int a = 0, b = 0;
    bool got = false;
    if (rmSentCmdIs(sentCmd, "status"))
    {
        RmStatusInfo st;
        if (rmStatusParse(result, st) && st.haveP)
        {
            a = st.min;
            b = st.max;
            got = true;
        }
    }
    else if (rmSentCmdIs(sentCmd, "radio"))
        got = rmRadioPParse(result, nullptr, &a, &b);
    else if (rmSentCmdIs(sentCmd, "txpower"))
        got = rmErrRangeParse(result, &a, &b);
    if (!got || !rmTxRangeValid(a, b))
        return false;
    *mn = a;
    *mx = b;
    return true;
}

inline bool rmTxRangeGet(const RmTxRange *t, uint8_t n, const char *dst, int *mn, int *mx)
{
    if (t == nullptr || dst == nullptr)
        return false;
    for (uint8_t i = 0; i < n; i++)
        if (t[i].used && strcmp(t[i].dst, dst) == 0)
        {
            if (mn != nullptr)
                *mn = t[i].min;
            if (mx != nullptr)
                *mx = t[i].max;
            return true;
        }
    return false;
}

// Stores/overwrites dst's range. Returns false (table unchanged) for a malformed range or call.
inline bool rmTxRangeLearn(RmTxRange *t, uint8_t n, const char *dst, int mn, int mx, uint32_t now)
{
    if (t == nullptr || n == 0 || dst == nullptr || dst[0] == '\0' || strlen(dst) >= sizeof(t[0].dst) ||
        !rmTxRangeValid(mn, mx))
        return false;
    RmTxRange *r = nullptr;
    for (uint8_t i = 0; i < n && r == nullptr; i++)
        if (t[i].used && strcmp(t[i].dst, dst) == 0)
            r = &t[i];
    for (uint8_t i = 0; i < n && r == nullptr; i++)
        if (!t[i].used)
            r = &t[i];
    if (r == nullptr)  // full: evict the least recently learnt
    {
        r = &t[0];
        for (uint8_t i = 1; i < n; i++)
            if ((int32_t)(t[i].lastUse - r->lastUse) < 0)
                r = &t[i];
    }
    memset(r, 0, sizeof(*r));
    snprintf(r->dst, sizeof(r->dst), "%s", dst);
    r->min = (int8_t)mn;
    r->max = (int8_t)mx;
    r->used = true;
    r->lastUse = now;
    return true;
}

inline void rmTxRangeForget(RmTxRange *t, uint8_t n, const char *dst)
{
    if (t == nullptr || dst == nullptr)
        return;
    for (uint8_t i = 0; i < n; i++)
        if (t[i].used && strcmp(t[i].dst, dst) == 0)
            memset(&t[i], 0, sizeof(t[i]));
}

// Sender clamp for `txpower <v>`: a learned range decides; without one, 0..RM_TXPOWER_UNKNOWN_MAX goes out.
enum RmTxSend
{
    RM_TXSEND_OK = 0,
    RM_TXSEND_RANGE = 1  // refuse locally with err token `range`
};
inline int rmTxPowerSendAllowed(bool learned, int mn, int mx, int value)
{
    if (!learned)
    {
        mn = 0;
        mx = RM_TXPOWER_UNKNOWN_MAX;
    }
    return (value >= mn && value <= mx) ? RM_TXSEND_OK : RM_TXSEND_RANGE;
}

// --- sent book: slot choice (pure; rm_runtime.cpp keeps the storage) -------------------------------
// An entry COUNTS while it is unanswered (not verified), not aged out and inside the policy window:
// evicting it would let the sender exceed the target's budget.
inline bool rmBookCounts(const RmPolEntry &e, uint32_t nowMs)
{
    return !e.verified && !e.expired && rmPolicyAgeMs(nowMs, e.sentMs) < RM_POLICY_WINDOW_MS;
}

// e[0..n) newest first, cap = slots. Returns the index to free for a new entry (the OLDEST entry that no
// longer counts), -2 when a slot is free (n < cap), -1 when every entry still counts: refuse with "busy"
// BEFORE a counter is consumed.
inline int rmBookVictim(const RmPolEntry *e, uint8_t n, uint8_t cap, uint32_t nowMs)
{
    if (n < cap)
        return -2;
    for (int i = (int)n - 1; i >= 0; i--)
        if (!rmBookCounts(e[i], nowMs))
            return i;
    return -1;
}

// slots a new entry can take right now: free slots plus entries that no longer count (same rule as
// rmBookVictim). A chain (sync, then the queued command) needs 2 BEFORE the sync leaves, so the
// command can never fail with "busy" after the sync went out.
inline uint8_t rmBookRoom(const RmPolEntry *e, uint8_t n, uint8_t cap, uint32_t nowMs)
{
    uint8_t room = (n < cap) ? (uint8_t)(cap - n) : 0;
    for (uint8_t i = 0; i < n && i < cap; i++)
        if (!rmBookCounts(e[i], nowMs))
            room++;
    return room;
}

// --- per-target key proof, RAM only -----------------------------------------------------------------
// fp = first 4 bytes of a hash of the key (the caller derives it; never the key itself). proven: a reply
// verified under exactly that fp. oneShot: armed by a CHANGE of fp for a known target, spent by the first
// send that would be refused with LIMIT. A first key for a target arms nothing; the same fp never re-arms.
// Rule: a proven key lifts the unanswered-send budget from 2 to 10 (the receiver does not count
// authenticated replays, valid tag with stale counter, toward its lockout); an unproven key stays at 2.
#define RM_PROOF_N 8
struct RmProof
{
    char dst[10];
    uint8_t fp[4];
    bool used;
    bool proven;
    bool oneShot;
    uint32_t lastUse;  // LRU stamp (rmProofTouch): bigger = more recent; no clock, so no millis() wrap
};

inline RmProof *rmProofFind(RmProof *b, const char *dst)
{
    for (uint8_t i = 0; i < RM_PROOF_N; i++)
        if (b[i].used && strcmp(b[i].dst, dst) == 0)
            return &b[i];
    return nullptr;
}

// CR-04: the book is a cache, not a register. Every use stamps the entry with the next tick (the book's
// highest stamp + 1); a new target takes a free slot or evicts the least recently used one. Before, the
// 9th distinct target got nullptr for good, so a regularly managed node typed after eight one-off calls had
// no proof and no one-shot for the rest of the session (budget stuck at 2).
inline void rmProofTouch(RmProof *b, RmProof *p)
{
    uint32_t top = 0;
    for (uint8_t i = 0; i < RM_PROOF_N; i++)
        if (b[i].used && b[i].lastUse > top)
            top = b[i].lastUse;
    p->lastUse = top + 1u;
}

// the saved key of dst is now fp (save, or first use). Never nullptr. A target is recorded at SEND time
// (not after a verified reply) on purpose: the re-key one-shot and the unproven budget are about targets
// that never answered, so a verified-only book could not detect a changed key for them. Eviction is what
// keeps one-off targets from pushing out the managed ones.
inline RmProof *rmProofSetKey(RmProof *b, const char *dst, const uint8_t fp[4])
{
    RmProof *p = rmProofFind(b, dst);
    if (p == nullptr)
    {
        for (uint8_t i = 0; i < RM_PROOF_N && p == nullptr; i++)
            if (!b[i].used)
                p = &b[i];
        if (p == nullptr)  // full: evict the least recently used (oldest stamp; stamps are distinct per use)
        {
            p = &b[0];
            for (uint8_t i = 1; i < RM_PROOF_N; i++)
                if (b[i].lastUse < p->lastUse)
                    p = &b[i];
        }
        memset(p, 0, sizeof(*p));
        p->used = true;
        snprintf(p->dst, sizeof(p->dst), "%s", dst);
        memcpy(p->fp, fp, 4);
        rmProofTouch(b, p);
        return p;
    }
    rmProofTouch(b, p);
    if (memcmp(p->fp, fp, 4) != 0)
    {
        memcpy(p->fp, fp, 4);
        p->proven = false;
        p->oneShot = true;
    }
    return p;
}

// a reply verified under the key with fingerprint fp
inline void rmProofVerified(RmProof *b, const char *dst, const uint8_t fp[4])
{
    RmProof *p = rmProofFind(b, dst);
    if (p != nullptr && memcmp(p->fp, fp, 4) == 0)
    {
        p->proven = true;
        rmProofTouch(b, p);
    }
}

// the target is forgotten/deleted: proof and one-shot go (a re-added node starts fresh, without one-shot)
inline void rmProofForget(RmProof *b, const char *dst)
{
    RmProof *p = rmProofFind(b, dst);
    if (p != nullptr)
        memset(p, 0, sizeof(*p));
}

inline uint8_t rmProofLimit(const RmProof *p)
{
    return (p != nullptr && p->proven) ? (uint8_t)RM_POLICY_LIMIT_PROVEN : (uint8_t)RM_POLICY_LIMIT_UNPROVEN;
}

// RM-PROOF (bench 2026-10-06): a proof is a statement about the past. When the TARGET gets a new password
// the sender's key stays "proven" although every frame is now a counted wrong-key reject there; with the
// budget of 10 five such frames locked the target. So the proof only holds while replies keep verifying:
// RM_POLICY_PROOF_DROP sends in a row without a verified reply (still waiting ones included, a wrong key
// is never answered) put the target back on the unproven budget until the next reply verifies.
#define RM_POLICY_PROOF_DROP 2u

// Sends to ONE target that are newer than its newest verified send (any order in, expired entries ignored).
inline uint8_t rmPolicyUnverifiedStreak(const RmPolEntry *e, uint8_t n, uint32_t nowMs)
{
    if (e == nullptr)
        return 0;
    bool haveVer = false;
    uint32_t verAge = 0;
    for (uint8_t i = 0; i < n; i++)
    {
        if (e[i].expired || !e[i].verified)
            continue;
        const uint32_t age = rmPolicyAgeMs(nowMs, e[i].sentMs);
        if (!haveVer || age < verAge)
        {
            haveVer = true;
            verAge = age;
        }
    }
    uint8_t k = 0;
    for (uint8_t i = 0; i < n; i++)
        if (!e[i].expired && !e[i].verified && (!haveVer || rmPolicyAgeMs(nowMs, e[i].sentMs) < verAge))
            k++;
    return k;
}

// The unanswered budget for one target: rmProofLimit(), cut back to the unproven budget by the streak rule.
inline uint8_t rmPolicyLimit(const RmProof *p, const RmPolEntry *e, uint8_t n, uint32_t nowMs)
{
    const uint8_t lim = rmProofLimit(p);
    if (lim > RM_POLICY_LIMIT_UNPROVEN && rmPolicyUnverifiedStreak(e, n, nowMs) >= RM_POLICY_PROOF_DROP)
        return (uint8_t)RM_POLICY_LIMIT_UNPROVEN;
    return lim;
}

#endif // RM_SENDER_POLICY_H
