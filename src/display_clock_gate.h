// Display clock -- redraw gate of
// sendDisplayTime() as a pure decision, for host tests.
// W2 contract: the clock line is redrawn every second (no 15 s gate, F2) on every page that HAS a
// clock line. The message page (iDisplayType == 0) has none: pageText[0] / the TFT header bar carry
// the sender header ("GM<grp> <CALL> <rssi>"), so the clock stays frozen there (SkipType0, F1
// operator decision 2026-11-08) and is drawn again when the next clock page is built (every
// sendDisplayHead/Position/Track/WX call sendDisplayMainline(), which draws the current time).
// Returned: Draw, SkipNoDisplay, SkipOff, SkipHold, SkipType0, SkipSetDisplay. SkipGate15 is
// no longer returned here; the enumerator and reason string stay for log lines. CLK-PING-01: a drawn held full-screen frame (holdActive = dispClkHeldFrame(), e.g.
// the ping/pong page from DisplayPong) has no clock line: SkipHold, no clock draw, no overwrite.
// A merely configured ping does NOT skip the clock. The hold is decided here (one predicate), not
// by gating the sendDisplayTime() call: that function also runs the button page-hold countdown.
// Order: off, hold, type0, setdisplay (hold before type0: the reason "hold" is the more specific).
#pragma once
#include <stdint.h>

enum class DispClkDecision
{
    Draw,
    SkipNoDisplay,   // no u8g2 / board without this clock line
    SkipOff,         // bDisplayIsOff
    SkipType0,       // iDisplayType == 0 (message page, F1): no clock line there, content stays
    SkipHold,        // drawn held full-screen frame (DispClkInputs::holdActive, dispClkHeldFrame())
    SkipSetDisplay,  // bSetDisplay: a draw is in progress
    SkipGate15       // legacy (F2): no longer returned by dispClkDecide
};

struct DispClkInputs
{
    bool hasDisplay;     // false -> SkipNoDisplay (u8g2 NULL or e-paper/T-Deck board)
    bool displayIsOff;
    int  displayType;
    int  pageHold;       // reserved, not consulted (current behaviour)
    bool setDisplay;
    int  second;         // node_date_second
    bool oneButton;
    bool displayDirty;
    bool holdActive;     // dispClkHeldFrame(): drawn held frame (ping/pong page, hold text), no clock line
};

static inline DispClkDecision dispClkDecide(const DispClkInputs &in)
{
    if (!in.hasDisplay)        return DispClkDecision::SkipNoDisplay;
    if (in.displayIsOff)       return DispClkDecision::SkipOff;
    if (in.holdActive)         return DispClkDecision::SkipHold;
    if (in.displayType == 0)   return DispClkDecision::SkipType0;
    if (in.setDisplay)         return DispClkDecision::SkipSetDisplay;
    return DispClkDecision::Draw;
}

// CLK-PING-01: the clock line is skipped only while a held full-screen frame was really drawn
// (DisplayOffWait running AND heldFrameDrawn, set by DisplayPong's u8g2 branch and the hold-text
// frame). DisplayOffWait alone is not enough: it is also set on boards whose DisplayPong draws
// nothing, and on the T-Deck Pro it never resets. An expired hold clears the flag.
static inline bool dispClkHeldFrame(unsigned long displayOffWait, bool &heldFrameDrawn)
{
    if (displayOffWait == 0)
        heldFrameDrawn = false;
    return displayOffWait > 0 && heldFrameDrawn;
}

// PING-HOLD-01: a ping/pong/fail frame is shown for PING_FRAME_HOLD_MS (or node_pingtime*1000 if that
// is shorter), then the clock pass redraws the page. DisplayOffWait keeps its full length (display-off
// timer, "display stays ON while pinging"); only the clock-skip window is shortened, by the deadline
// heldUntilMs (0 = none: the setdisplay hold text keeps its own duration).
#define PING_FRAME_HOLD_MS 10000UL

static inline unsigned long dispClkPingHoldMs(unsigned long pingtimeS)
{
    const unsigned long full = pingtimeS * 1000UL;
    return full < PING_FRAME_HOLD_MS ? full : PING_FRAME_HOLD_MS;
}

// deadline for a frame drawn at now (never 0: 0 means "no deadline")
static inline unsigned long dispClkFrameDeadline(unsigned long now, unsigned long holdMs)
{
    const unsigned long d = (unsigned long)(uint32_t)(now + holdMs);   // millis() is 32 bit
    return d == 0 ? 1UL : d;
}

static inline bool dispClkHeldFrame(unsigned long displayOffWait, bool &heldFrameDrawn,
                                    unsigned long now, unsigned long &heldUntilMs)
{
    if (heldUntilMs != 0 && (int32_t)(uint32_t)(now - heldUntilMs) >= 0)   // wrap-safe; deadline reached
    {
        heldFrameDrawn = false;
        heldUntilMs = 0;
    }
    return dispClkHeldFrame(displayOffWait, heldFrameDrawn);
}

// PING-HOLD-02: true exactly on the pass where a drawn held frame reaches its deadline (call BEFORE the
// 4-arg dispClkHeldFrame(), which clears the state). On the message page (SkipType0) nothing else redraws,
// so the clock pass re-transfers the cached page buffer once on this edge.
static inline bool dispClkFrameExpired(bool heldFrameDrawn, unsigned long heldUntilMs, unsigned long now)
{
    return heldFrameDrawn && heldUntilMs != 0 && (int32_t)(uint32_t)(now - heldUntilMs) >= 0;
}

static inline const char *dispClkReason(DispClkDecision d)
{
    switch (d)
    {
    case DispClkDecision::Draw:           return "draw";
    case DispClkDecision::SkipNoDisplay:  return "nodisplay";
    case DispClkDecision::SkipOff:        return "off";
    case DispClkDecision::SkipType0:      return "type0";
    case DispClkDecision::SkipHold:       return "hold";
    case DispClkDecision::SkipSetDisplay: return "setdisplay";
    case DispClkDecision::SkipGate15:     return "gate15";
    }
    return "?";
}

// The displayed time never runs back by a few seconds. Two time sources (GPS, NTP) can disagree by
// a second; when the clock is set to the earlier one, the display holds the second it already
// showed until the clock has caught up. A step back of more than DISP_CLK_HOLD_MAX_S (summer/winter
// time, a real correction) is shown at once. Times are seconds of the day; midnight wraps.
#ifndef DISP_CLK_HOLD_MAX_S
#define DISP_CLK_HOLD_MAX_S 10L
#endif

struct DispClkShown
{
    bool have;    // a time was shown before
    long sod;     // seconds of the day last shown
};

static inline long dispClkShownSod(DispClkShown &st, long now_sod)
{
    long diff = now_sod - st.sod;
    if (diff > 43200L)  diff -= 86400L;   // shortest way round midnight
    if (diff < -43200L) diff += 86400L;
    if (st.have && diff < 0 && diff >= -DISP_CLK_HOLD_MAX_S)
        return st.sod;                     // small step back: keep what is shown
    st.have = true;
    st.sod = now_sod;
    return now_sod;
}
