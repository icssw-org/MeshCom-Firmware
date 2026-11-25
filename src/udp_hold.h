// Server-outage hold for text messages (gateway, UDP out).
//
// The central server has no per-frame ACK. When it is down, every DATA frame
// sent to it is lost. This header keeps a bounded RAM copy of the TEXT frames
// that stay worth delivering late and replays them, oldest first and paced,
// once the server shows a sign of life again. The server drops duplicate
// message IDs, so a replay is safe. A replay sends the identical frame bytes.
//
// What is held (isHoldable): a DATA frame whose LoRa part is a text (type 0x3A)
// "SRC>DEST:payload". Never held, because a late arrival is wrong or harmful:
//   - positions, HEY, KEEP, STOR (DATA frames carry no timestamp: a replayed
//     position would look current)
//   - telemetry (DEST 100001, "CALL:T#..." and PARM frames)
//   - RM1 commands and replies (payload "RM1 "): a replay reaches the target
//     through server GATE, and rmCheck only rejects ctr <= high-water mark, so
//     it would run after the operator gave up
//   - text ACKs (":ackNNN", txRingFrameIsAck), {ping} and {pong} (wrong RTT),
//     APRS2SOTA (a stale spot)
//   - anything that does not parse as SRC>DEST:payload
// Kept on purpose: WLNK-1 and BBS text, plain DMs and group text; a late
// delivery is what they want.
//
// Pure and host-testable: the class works on a caller-supplied byte buffer and
// takes the time as a parameter. Everything platform-specific (lock, sender,
// log) is injected through the function pointers of UdpHoldRt at the bottom;
// the drains and gateway services only call inline functions from this header,
// so the native twin tests keep linking without new stubs.
//
// Sign of life = a change of last_upd_timer (set by any parsed server frame:
// BEAT, CONF, GATE). The server answers each KEEP (every HEARTBEAT_INTERVAL,
// 30 s) with a BEAT, so in normal operation a sign arrives every ~30 s. The
// synthetic writers of last_upd_timer (the gateway watchdogs on both
// platforms, the nRF52 ETH bring-up in nrf_eth.cpp: startUDP, static re-apply,
// hardware-reset re-apply) call udpHoldNoteSynthetic() so the next tick does
// not read their write as a server frame.
//
// Thresholds (mirroring configuration_global.h, kept here so the host test does
// not depend on it):
//   outage            no sign of life for more than 65 s     (MAX_HB_RX_TIME)
//   suspect gap       two signs more than 35 s apart         (HB_WARN_TIME)
//   release           a frame is dropped from the hold once a sign arrives at
//                     least 30 s after it (HEARTBEAT_INTERVAL), the previous
//                     sign was no more than 35 s before that one, and a sign
//                     came before the frame: server alive on both sides.
//                     A frame held without an IP never went out, so it is
//                     never released this way: the next sign starts a replay.
//   replay start      first sign after an outage / suspect gap / boot, AND a
//                     second sign at least 5 s later: one sign proves nothing
//                     about a link that just came back (the server answers a
//                     KEEP every 30 s, GATE and CONF frames can arrive in
//                     between), so the replay waits for a second one. Not the
//                     first sign alone.
//   replay pacing     one frame every 2 s through addUdpOutBuffer()
//   max age           20 min, older entries are dropped and counted. The local
//                     dedup window on the classic ESP32 is about 23-27 min: a
//                     replay that comes back as GATE must not retransmit on RF
//                     what the area already heard.
// Frames stamped in the last 65 s before the outage is detected are covered:
// they are still in the hold, because the release rule needs a sign after them.
// On a replay ALL held frames go out, including some already delivered before
// the outage began (the overlap); the server drops those duplicates.
//
// SUSPECT_GAP_MS = 35 s is a decision, not an oversight: a single lost BEAT
// (gap above 35 s) starts a replay of the few frames still held. Measured on the
// field BEAT stream (11434 gaps): p50 30.00 s, p99 30.2 s, p99.9 41 s, and 23
// gaps in the 35-65 s band, i.e. about one replay per 4 h. The server drops the
// duplicates; raising the threshold would lose text in real 35-65 s outages.

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "txring_ttl.h"   // txRingFrameIsAck()

class UdpHold
{
public:
    static const uint32_t MAX_AGE_MS       = 20UL * 60UL * 1000UL;
    static const uint32_t OUTAGE_MS        = 65UL * 1000UL;
    static const uint32_t SUSPECT_GAP_MS   = 35UL * 1000UL;
    static const uint32_t RELEASE_AGE_MS   = 30UL * 1000UL;
    static const uint32_t CONFIRM_MS       = 5UL * 1000UL;
    static const uint32_t REPLAY_GAP_MS    = 2UL * 1000UL;
    static const uint16_t DATA_HDR         = 36;     // "DATA"+ID+call+ver+rssi+snr+"03"
    static const uint8_t  TYPE_TEXT_BYTE    = 0x3A;   // first byte of the raw LoRa frame
    static const uint16_t LORA_TEXT_HDR    = 6;      // type(1)+msg id(4)+hop/flags(1), then "SRC>DEST:payload"
    static const uint16_t MAX_FRAME        = 255;    // UDP_TX_BUF_SIZE
    static const uint16_t ENTRY_OVERHEAD   = 5;      // stamp(4) + len(1)

    enum { EV_OUTAGE_START = 1, EV_OUTAGE_END = 2, EV_REPLAY_START = 4,
           EV_REPLAY_DONE = 8, EV_DROP_AGE = 16 };

    // text = DATA frame whose raw LoRa message type byte (right behind the
    // 36-byte header) is ':' . Position '!', HEY '@', ACK 'A', KEEP, STOR and
    // BEAT-style datagrams do not qualify. Type test only; push() and
    // udpHoldCapture() use isHoldable().
    static bool isText(const uint8_t *f, uint16_t len)
    {
        return f && len > DATA_HDR && f[0] == 'D' && f[1] == 'A' && f[2] == 'T' &&
               f[3] == 'A' && f[DATA_HDR] == TYPE_TEXT_BYTE;
    }

    // The raw LoRa text frame as encodeStartAPRS() builds it:
    //   0x3A | msg id (4) | hop+flags (1) | "SRC[,via..]>DEST:" | payload | 0x00 | hw mod fcs ...
    // true = worth delivering late (see the list at the top of this file).
    // Anything that does not parse as SRC>DEST:payload is refused.
    static bool loraHoldable(const uint8_t *l, uint16_t len)
    {
        if (!l || len <= LORA_TEXT_HDR + 3 || l[0] != TYPE_TEXT_BYTE)
            return false;
        const uint8_t *p = l + LORA_TEXT_HDR;
        const uint8_t *end = l + len;
        const uint8_t *src = p;
        while (p < end && *p != '>')                 // source path: no ':' and no NUL before '>'
        {
            if (*p == ':' || *p == 0x00)
                return false;
            p++;
        }
        if (p >= end || p == src)
            return false;                            // no '>' or empty source
        p++;
        const uint8_t *dest = p;
        while (p < end && *p != ':')                 // destination up to the payload separator
        {
            if (*p == 0x00 || *p == '>')
                return false;
            p++;
        }
        if (p >= end || p == dest)
            return false;                            // no ':' or empty destination
        const size_t dlen = (size_t)(p - dest);
        p++;
        const uint8_t *pay = p;
        while (p < end && *p != 0x00)
            p++;
        const size_t plen = (size_t)(p - pay);

        if (dlen == 6 && memcmp(dest, "100001", 6) == 0)
            return false;                            // telemetry / PARM
        if (dlen == 9 && memcmp(dest, "APRS2SOTA", 9) == 0)
            return false;                            // stale spot
        if (plen >= 4 && memcmp(pay, "RM1 ", 4) == 0)
            return false;                            // remote management command / reply
        if (plen >= 5 && (memcmp(pay, "{MCP}", 5) == 0 || memcmp(pay, "{mcp}", 5) == 0 ||
                          memcmp(pay, "{SET}", 5) == 0 || memcmp(pay, "{CET}", 5) == 0))
            return false;                            // switches a pin, hop settings, clock: never late
        if (plen >= 6 && (memcmp(pay, "{ping}", 6) == 0 || memcmp(pay, "{pong}", 6) == 0))
            return false;                            // RTT probe: a late answer is a wrong RTT
        if (txRingFrameIsAck(l, len))
            return false;                            // ":ackNNN" / WLNK-1 "ackNNNN"
        return true;
    }

    static bool isHoldable(const uint8_t *f, uint16_t len)
    {
        return isText(f, len) && loraHoldable(f + DATA_HDR, (uint16_t)(len - DATA_HDR));
    }

    void init(uint8_t *buf, uint16_t cap)
    {
        memset(this, 0, sizeof(*this));
        buf_ = buf;
        cap_ = buf ? cap : 0;
    }

    // Copy a frame in. Returns true if held. Refused: not text, empty,
    // larger than 255, or bigger than the whole buffer. Drops the oldest
    // entries until it fits.
    bool push(const uint8_t *f, uint16_t len, uint32_t now, bool sent = true)
    {
        if (!buf_ || len == 0 || len > MAX_FRAME || !isHoldable(f, len) ||
            (uint32_t)len + ENTRY_OVERHEAD > cap_)
        {
            refused_++;
            return false;
        }
        while ((uint32_t)used_ + len + ENTRY_OVERHEAD > cap_)
        {
            popFront();
            droppedFull_++;
            if (replayLeft_) replayLeft_--;
        }
        uint8_t hdr[5] = { (uint8_t)now, (uint8_t)(now >> 8), (uint8_t)(now >> 16),
                           (uint8_t)(now >> 24), (uint8_t)len };
        wr(hdr, 5);
        wr(f, len);
        used_ += len + ENTRY_OVERHEAD;
        count_++;
        pushed_++;
        if (!sent) unsent_ = true;
        return true;
    }

    // Feed a sign of life (the server was heard at `now`).
    void signOfLife(uint32_t now)
    {
        // unsent_: a frame went in without an IP, so "server alive on both sides"
        // says nothing about it. Replay instead of releasing; the server drops the overlap.
        const bool suspect = !haveAlive_ || outage_ || unsent_ || (uint32_t)(now - lastAlive_) > SUSPECT_GAP_MS;
        unsent_ = false;
        if (outage_)
        {
            outage_ = false;
            events_ |= EV_OUTAGE_END;
            lastOutageSec_ = (now - outageStart_) / 1000UL;
        }
        if (suspect)
        {
            if (replaying_) replayLeft_ = count_;          // widen to everything held
            else if (!recovering_ && count_ > 0)
            {
                recovering_ = true;
                recoverFirst_ = now;
            }
        }
        lastAlive_ = now;
        haveAlive_ = true;
        if (recovering_)
        {
            if ((uint32_t)(now - recoverFirst_) >= CONFIRM_MS)
            {
                recovering_ = false;
                if (count_ > 0)
                {
                    replaying_ = true;
                    replayLeft_ = count_;
                    nextAt_ = now;
                    events_ |= EV_REPLAY_START;
                }
            }
        }
        else if (!replaying_)
        {
            // server alive on both sides: release what is old enough. A stamp
            // ahead of `now` (captured between the tick's millis() and the lock)
            // would read as ~49 days old: leave it.
            while (count_ && (uint32_t)(now - frontStamp()) >= RELEASE_AGE_MS &&
                   (uint32_t)(now - frontStamp()) < 0x80000000UL)
            {
                popFront();
                released_++;
            }
        }
    }

    // Periodic step: age expiry and outage detection.
    void step(uint32_t now)
    {
        uint32_t aged = 0;
        while (count_ && (uint32_t)(now - frontStamp()) > MAX_AGE_MS &&
               (uint32_t)(now - frontStamp()) < 0x80000000UL)
        {
            popFront();
            droppedAge_++;
            aged++;
            if (replayLeft_) replayLeft_--;
        }
        if (aged) events_ |= EV_DROP_AGE;
        if (haveAlive_ && !outage_ && (uint32_t)(now - lastAlive_) > OUTAGE_MS)
        {
            outage_ = true;
            outageStart_ = lastAlive_;
            outages_++;
            recovering_ = false;
            replaying_ = false;
            replayLeft_ = 0;
            events_ |= EV_OUTAGE_START;
        }
    }

    // Next frame due for replay: copies it to out (outmax >= 255 for all),
    // removes it from the hold. false if none is due now.
    bool nextReplay(uint32_t now, uint8_t *out, uint16_t outmax, uint16_t *len)
    {
        if (!replaying_ || (int32_t)(now - nextAt_) < 0)
            return false;
        if (replayLeft_ == 0 || count_ == 0)
        {
            finishReplay();
            return false;
        }
        const uint16_t flen = frontLen();
        if (flen > outmax)
            return false;
        rdFront(out, flen);
        popFront();
        replayed_++;
        replayLeft_--;
        nextAt_ = now + REPLAY_GAP_MS;
        *len = flen;
        if (replayLeft_ == 0 || count_ == 0)
            finishReplay();
        return true;
    }

    bool serverAlive(uint32_t now) const
    {
        return haveAlive_ && !outage_ && (uint32_t)(now - lastAlive_) <= OUTAGE_MS;
    }

    uint8_t takeEvents() { uint8_t e = events_; events_ = 0; return e; }

    bool outage() const { return outage_; }
    bool replaying() const { return replaying_; }
    bool recovering() const { return recovering_; }
    uint16_t held() const { return count_; }
    uint16_t usedBytes() const { return used_; }
    uint16_t capacity() const { return cap_; }
    uint16_t replayLeft() const { return replayLeft_; }
    uint32_t replayed() const { return replayed_; }
    uint32_t released() const { return released_; }
    uint32_t pushed() const { return pushed_; }
    uint32_t droppedFull() const { return droppedFull_; }
    uint32_t droppedAge() const { return droppedAge_; }
    uint32_t refused() const { return refused_; }
    uint32_t outages() const { return outages_; }
    uint32_t lastOutageSec() const { return lastOutageSec_; }

private:
    uint8_t *buf_;
    uint16_t cap_, head_, tail_, used_, count_, replayLeft_;
    uint32_t lastAlive_, recoverFirst_, nextAt_, outageStart_, lastOutageSec_;
    uint32_t pushed_, released_, replayed_, droppedFull_, droppedAge_, refused_, outages_;
    bool haveAlive_, outage_, recovering_, replaying_, unsent_;
    uint8_t events_;

    void finishReplay()
    {
        replaying_ = false;
        replayLeft_ = 0;
        events_ |= EV_REPLAY_DONE;
    }
    uint16_t adv(uint16_t p, uint16_t n) const
    {
        const uint32_t v = (uint32_t)p + n;       // n <= cap_: one subtraction wraps
        return (uint16_t)(v >= cap_ ? v - cap_ : v);
    }
    void wr(const uint8_t *s, uint16_t n)
    {
        for (uint16_t i = 0; i < n; i++)
        {
            buf_[head_] = s[i];
            head_ = adv(head_, 1);
        }
    }
    uint8_t rdAt(uint16_t off) const { return buf_[adv(tail_, off)]; }
    uint32_t frontStamp() const
    {
        return (uint32_t)rdAt(0) | ((uint32_t)rdAt(1) << 8) | ((uint32_t)rdAt(2) << 16) |
               ((uint32_t)rdAt(3) << 24);
    }
    uint16_t frontLen() const { return rdAt(4); }
    void rdFront(uint8_t *out, uint16_t n) const
    {
        for (uint16_t i = 0; i < n; i++)
            out[i] = rdAt(ENTRY_OVERHEAD + i);
    }
    void popFront()
    {
        const uint16_t total = frontLen() + ENTRY_OVERHEAD;
        tail_ = adv(tail_, total);
        used_ -= total;
        count_--;
    }
};

///////////////////////////////////////////////////////////////////////////////
// Runtime glue. Inline so that every TU sees the same instance and no link
// symbol is added; the platform hooks are injected by udp_functions.cpp. When
// not attached (gateway off, board without RAM for it, native twin tests) every
// function is a no-op.

struct UdpHoldRt
{
    UdpHold hold;
    bool enabled;
    bool seenInit;
    uint32_t seenTimer;
    void (*sink)(uint8_t *, uint16_t);
    void (*logfn)(const char *);
    void (*lockfn)(void);
    void (*unlockfn)(void);
};

inline UdpHoldRt &udpHoldRt()
{
    static UdpHoldRt r;     // zero-initialised: not enabled
    return r;
}

inline void udpHoldAttach(uint8_t *buf, uint16_t cap, void (*sink)(uint8_t *, uint16_t),
                          void (*logfn)(const char *), void (*lockfn)(void), void (*unlockfn)(void))
{
    UdpHoldRt &r = udpHoldRt();
    r.hold.init(buf, cap);
    r.sink = sink;
    r.logfn = logfn;
    r.lockfn = lockfn;
    r.unlockfn = unlockfn;
    r.enabled = buf && cap > 0;
}

// Enqueue site (addNodeData): hold a copy of a DATA frame if it is a holdable
// TEXT (UdpHold::isHoldable). Caller checks bGATEWAY. Returns true if the frame
// is of the holdable class (held or evicted by size), false if the hold is off
// or the frame is not one; the caller uses it to decide whether a frame without
// an IP address is dropped (held only) or handled as before.
// sent = false: the caller has no IP, this frame is held but not enqueued.
inline bool udpHoldCapture(const uint8_t *f, uint16_t len, uint32_t now, bool sent = true)
{
    UdpHoldRt &r = udpHoldRt();
    if (!r.enabled || !UdpHold::isHoldable(f, len))
        return false;
    if (r.lockfn) r.lockfn();
    r.hold.push(f, len, now, sent);
    if (r.unlockfn) r.unlockfn();
    return true;
}

// Raw LoRa frame (no DATA header) variant for callers deciding whether to call
// addNodeData() although there is no IP address: only a frame that the hold
// would keep is worth the call. False when the hold is not attached.
inline bool udpHoldLoraHoldable(const uint8_t *lora, uint16_t len)
{
    return udpHoldRt().enabled && UdpHold::loraHoldable(lora, len);
}

// The watchdogs and the ETH bring-up rewrite last_upd_timer without a server
// frame: call after such a write so the next tick does not read a sign of life.
inline void udpHoldNoteSynthetic(uint32_t timerVal)
{
    UdpHoldRt &r = udpHoldRt();
    if (r.enabled) r.seenTimer = timerVal;
}

inline void udpHoldStatusLine(char *out, size_t n)
{
    if (!out || n == 0) return;
    UdpHoldRt &r = udpHoldRt();
    if (!r.enabled)
    {
        snprintf(out, n, "[GW];hold;state;off");
        return;
    }
    // Copy the counters under the lock, format outside (snprintf is slow and
    // must not run in a critical section). UdpHold is plain data; the copy
    // shares buf_, which the accessors below never touch.
    if (r.lockfn) r.lockfn();
    const UdpHold h = r.hold;
    if (r.unlockfn) r.unlockfn();

    const char *st = h.outage() ? "outage" : h.replaying() ? "replay" :
                     h.recovering() ? "recover" : "ok";
    snprintf(out, n, "[GW];hold;state;%s;held;%u;bytes;%u;cap;%u;pushed;%lu;released;%lu;replayed;%lu;"
             "drop_full;%lu;drop_age;%lu;refused;%lu;outages;%lu",
             st, (unsigned)h.held(), (unsigned)h.usedBytes(), (unsigned)h.capacity(),
             (unsigned long)h.pushed(), (unsigned long)h.released(), (unsigned long)h.replayed(),
             (unsigned long)h.droppedFull(), (unsigned long)h.droppedAge(),
             (unsigned long)h.refused(), (unsigned long)h.outages());
}

// Main-loop tick, gateway on only. timerVal = last_upd_timer (ESP32) or
// neth.last_upd_timer (nRF52), now = millis(). ipUp = the node has an IP
// address; without one nothing is replayed (frames stay held). The 256-byte
// replay buffer is on the stack: the tick runs in the Arduino loop task only
// (ESP32 8 kB+, nRF52 8 kB "mcloop"), shallow, and a static buffer would cost
// .bss on every board, gateway or not.
inline void udpHoldTick(uint32_t timerVal, uint32_t now, bool ipUp = true)
{
    UdpHoldRt &r = udpHoldRt();
    if (!r.enabled)
        return;
    uint8_t fr[UdpHold::MAX_FRAME + 1];
    uint16_t flen = 0;
    bool have = false;
    uint8_t ev = 0;
    char line[160];
    unsigned held = 0;
    unsigned long replayed = 0, dfull = 0, dage = 0, dur = 0;

    if (r.lockfn) r.lockfn();
    if (!r.seenInit)
    {
        r.seenInit = true;
        r.seenTimer = timerVal;
    }
    else if (timerVal != r.seenTimer)
    {
        r.seenTimer = timerVal;
        r.hold.signOfLife(now);
    }
    r.hold.step(now);
    if (ipUp)
        have = r.hold.nextReplay(now, fr, sizeof(fr), &flen);
    ev = r.hold.takeEvents();
    held = r.hold.held();
    replayed = r.hold.replayed();
    dfull = r.hold.droppedFull();
    dage = r.hold.droppedAge();
    dur = r.hold.lastOutageSec();
    if (r.unlockfn) r.unlockfn();

    if (have && r.sink)
        r.sink(fr, flen);       // addUdpOutBuffer(): normal out path, not re-held

    if (!ev || !r.logfn)
        return;
    if (ev & UdpHold::EV_OUTAGE_START)
    {
        snprintf(line, sizeof(line), "[GW];hold;outage;start;held;%u;ms;%lu\n", held, (unsigned long)now);
        r.logfn(line);
    }
    if (ev & UdpHold::EV_OUTAGE_END)
    {
        snprintf(line, sizeof(line), "[GW];hold;outage;end;dur_s;%lu;held;%u;drop_full;%lu;drop_age;%lu\n",
                 dur, held, dfull, dage);
        r.logfn(line);
    }
    if (ev & UdpHold::EV_REPLAY_START)
    {
        snprintf(line, sizeof(line), "[GW];hold;replay;start;n;%u\n", held + (have ? 1u : 0u));
        r.logfn(line);
    }
    if (ev & UdpHold::EV_REPLAY_DONE)
    {
        snprintf(line, sizeof(line), "[GW];hold;replay;done;replayed;%lu;held;%u;drop_full;%lu;drop_age;%lu\n",
                 replayed, held, dfull, dage);
        r.logfn(line);
    }
    if (ev & UdpHold::EV_DROP_AGE)
    {
        snprintf(line, sizeof(line), "[GW];hold;drop;age;total;%lu\n", dage);
        r.logfn(line);
    }
}
