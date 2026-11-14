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
