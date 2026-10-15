// ETH-04: decision logic for the RAK4631 + W5100S Ethernet recovery paths.
// Pure C++ (only <stdint.h>), no Arduino, no W5100S; unit-tested natively
// (pio test -e native_eth_recovery). Wiring lives in the nRF52 ETH code.
//
// Problem: every recovery path (UDP send-error run, bad UDP frame, gateway
// heartbeat timeout, --ethdrop, DHCP retry) used to call Ethernet.begin(),
// which zeroes the chip IP and sends a fresh DISCOVER even if the router still
// holds a valid lease. A router that unicasts the OFFER to the known IP (e.g.
// MikroTik, 1 week lease) is then ignored by the W5100S (IP 0.0.0.0) and the
// node stays offline for hours. See docs/issue-rak-dhcp-after-reboot-db0ais.md.
//
// F1 (ReopenUdp): lease held -> keep the IP, only reopen the UDP socket.
//     Ethernet.maintain() keeps owning renew/rebind. A link flap does not
//     invalidate the lease (a DISCOVER right after a flap is the observed
//     failure). If a reopen got no UDP RX before the next trigger (reopen_stalled)
//     the lease or the chip is suspect -> escalate to full DHCP, then F2.
//     A link down edge disarms the stall check: time without link is no
//     evidence, so a link that keeps flapping keeps reopening (F2 needs link up
//     anyway).
// F2 (HwResetThenDhcp): repeated DHCP failures with link up -> one W5100S
//     hardware reset before the next attempt, rate-limited.
#ifndef ETH_RECOVERY_H
#define ETH_RECOVERY_H

#include <stdint.h>

namespace eth_recovery {

enum class Action : uint8_t {
    ReopenUdp,        // F1: keep chip IP + lease, Udp.stop()+startUDP(), no Ethernet.begin()
    FullDhcp,         // legacy path: startETH() -> Ethernet.begin() DISCOVER
    HwResetThenDhcp,  // F2: initETH_HW() then startETH()
    ReapplyStatic     // ETH-05: static node, re-apply the configured address, never DHCP
};

struct Inputs {
    bool fixed_ip;          // static IP configured (node_ownip/ownms/owngw set)
    bool link_up;           // Ethernet.linkStatus() == LinkON right now
    bool lease_held;        // last address came from a successful DHCP, not invalidated since
    bool reopen_stalled;    // a previous ReopenUdp got no UDP RX since
    int  last_maintain_rc;  // Ethernet.maintain(): 0 none,1 renew fail,2 renew ok,3 rebind fail,4 rebind ok
};

struct State {
    uint8_t  dhcp_fail_streak;   // consecutive DHCP failures with link up (saturating)
    bool     hw_reset_done;      // at least one F2 reset happened
    uint32_t last_hw_reset_ms;
};

constexpr uint8_t  kHwResetAfterFails = 2;
constexpr uint32_t kHwResetMinGapMs   = 300000UL;  // 5 min

// Same 5 min gap as F2 for the hardware reset of both paths (state shared: State::last_hw_reset_ms).
// Unsigned subtraction survives millis() wraparound.
inline bool hwResetAllowed(const State& st, uint32_t now_ms)
{
    return !st.hw_reset_done || (uint32_t)(now_ms - st.last_hw_reset_ms) >= kHwResetMinGapMs;
}

inline Action decide(const Inputs& in, const State& st, uint32_t now_ms)
{
    // Rule 1 (ETH-05): fixed IP has no lease and must never see a DHCP exchange;
    // the caller re-applies the configured static address.
    if (in.fixed_ip)
        return Action::ReapplyStatic;

    // Rule 2 (F1): a held lease is normally still valid on the router, so
    // re-DISCOVERing only loses it. A failed renew/rebind means the lease may be
    // gone; a stalled earlier reopen means reopening does not help (wedged chip,
    // no ARP) -> fall through to DHCP and, after repeated failures, F2.
    if (in.lease_held && in.link_up && !in.reopen_stalled &&
        in.last_maintain_rc != 1 && in.last_maintain_rc != 3)
        return Action::ReopenUdp;

    // Rule 3 (F2): hardware reset only with link up (N-20: a replugged cable must
    // reconnect, never reset the chip while the link is down) and at most once
    // per kHwResetMinGapMs. Unsigned subtraction survives millis() wraparound.
    if (in.link_up && st.dhcp_fail_streak >= kHwResetAfterFails && hwResetAllowed(st, now_ms))
        return Action::HwResetThenDhcp;

    return Action::FullDhcp;
}

// ETH-05 follow-up: how a static node recovers. The legacy behaviour was a cheap
// Udp.stop + startUDP; a hardware reset (initethfixIP -> initETH_HW) costs seconds of
// PHY bring-up and is reserved for a dead link or a failed re-apply.
//   Reopen:  link up and the chip already holds the configured address -> only reopen UDP.
//   Reapply: link up, address lost/different -> Ethernet.begin(static), no hardware reset.
//   HwReset: link down -> full initethfixIP() (also the fallback when Reapply fails).
// Stall guard (static analogue of F1 -> F2): if an earlier Reopen got no UDP RX before
// the next trigger (reopen_stalled), reopening again does not help (wedged sockets with
// a correct SIPR) -> HwReset when the address still matches, Reapply when it does not.
enum class StaticAction : uint8_t { Reopen, Reapply, HwReset };

struct StaticInputs {
    bool link_up;     // Ethernet.linkStatus() == LinkON right now
    bool ip_matches;  // Ethernet.localIP() equals the configured node_ownip
    bool reopen_stalled;  // a previous static Reopen got no UDP RX since
    bool hw_reset_allowed;  // hwResetAllowed(): no reset yet, or kHwResetMinGapMs elapsed
};

// A hardware reset is chosen only when hw_reset_allowed; otherwise every case that would reset
// falls back to Reapply (Ethernet.begin(static), no hardware reset).
inline StaticAction decideStatic(const StaticInputs& in)
{
    if (!in.link_up)
        return in.hw_reset_allowed ? StaticAction::HwReset : StaticAction::Reapply;
    if (in.reopen_stalled) {
        if (!in.ip_matches || !in.hw_reset_allowed)
            return StaticAction::Reapply;
        return StaticAction::HwReset;
    }
    return in.ip_matches ? StaticAction::Reopen : StaticAction::Reapply;
}

// Success clears the streak. A failure counts only with link up: without link
// DHCP cannot work and says nothing about the chip.
inline void onDhcpResult(State& st, bool ok, bool link_up)
{
    if (ok) {
        st.dhcp_fail_streak = 0;
    } else if (link_up && st.dhcp_fail_streak < 255) {
        st.dhcp_fail_streak++;
    }
}

// The streak stays: only a successful DHCP clears it, so a reset that does not
// help is followed by the 5 min back-off, not by an immediate second reset.
inline void onHwReset(State& st, uint32_t now_ms)
{
    st.hw_reset_done = true;
    st.last_hw_reset_ms = now_ms;
}

}  // namespace eth_recovery

#endif  // ETH_RECOVERY_H
