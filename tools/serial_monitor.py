#!/usr/bin/env python3
"""MeshCom serial monitor — tracks LoRa state machine, packet counts, and alerts.

Connects to a Heltec WiFi LoRa 32 V3 (or similar) via serial with DTR/RTS
disabled to avoid triggering a hardware reset. Logs all output to a timestamped
file in /tmp/meshcom_monitor/ and prints alerts + periodic summaries to console.

Dk5EN / Martin 
1. März 2026

Usage:
    python3 tools/serial_monitor.py                          # NRF52/RAK (CDC-ACM)
    python3 tools/serial_monitor.py --no-dtr                 # ESP32/CP2102 (no reset)
    python3 tools/serial_monitor.py --port /dev/cu.usbserial-0001 --interval 300
"""

import argparse
import os
import re
import signal
import sys
import termios
import threading
import time
from collections import defaultdict
from datetime import datetime, timedelta

import serial


# ---------------------------------------------------------------------------
# Pattern definitions
# ---------------------------------------------------------------------------

RE_STATE_TRANSITION = re.compile(r"\[MC-SM\]\s+(\w+)\s*->\s*(\w+)\s+rc=(-?\d+)")
RE_BUFFER_DROP = re.compile(r"\[MC-DBG\]\s+(\w+_DROPPED)\s+buffer_full")
RE_CAD_SCAN = re.compile(r"\[MC-DBG\]\s+CAD_SCAN\s+result=(-?\d+)")
RE_CAD_GIVEUP = re.compile(r"\[MC-DBG\]\s+CAD_GIVEUP")
RE_RETRANSMIT_GIVEUP = re.compile(r"\[MC-DBG\]\s+RETRANSMIT_GIVEUP")
RE_WIFI_DBG = re.compile(r"\[WIFI-DBG\]\s+(.*)")
RE_UDP_RESET = re.compile(r"resetMeshComUDP")
RE_RING_STATUS = re.compile(
    r"\[MC-DBG\]\s+RING_STATUS\s+queued=(\d+)\s+pending=(\d+)\s+retrying=(\d+)\s+done=(\d+)"
)
# RING_OVERFLOW buf=raw_rx is a normal cyclical buffer for the web UI — not a
# real overflow.  The firmware reuses the oldest slot when the ring is full,
# so no data is lost that matters for RF operation.  We silently count these
# but never alert on them.  Other buffer names (if they ever appear) still
# trigger an alert.
RE_RING_OVERFLOW = re.compile(r"\[MC-DBG\]\s+RING_OVERFLOW\s+buf=(\w+)")
RE_CHANNEL_UTIL = re.compile(
    r"\[MC-DBG\]\s+CHANNEL_UTIL\s+rx=(\d+)ms\s+tx=(\d+)ms\s+util=(\d+)%"
)
RE_RX_TIMEOUT_FIRE = re.compile(
    r"RX_TIMEOUT_FIRE.*?wait=(\d+(?:\.\d+)?)"
)
RE_CAD_FALSE_POSITIVE = re.compile(r"\[MC-DBG\]\s+CAD_FALSE_POSITIVE")
RE_RX_TIMEOUT_DEFERRED = re.compile(r"\[MC-DBG\]\s+RX_TIMEOUT_DEFERRED")
RE_RELAY_LOOP_BLOCKED = re.compile(r"\[MC-DBG\]\s+RELAY_LOOP_BLOCKED")
RE_NTP_FAIL = re.compile(r"TimeClient no (?:update|force update) possible")
RE_NTP_OK = re.compile(r"TimeClient now \(UTC\)")
RE_HB_TIMEOUT = re.compile(r"\[UDP\] Heartbeat timeout")

# ACK deduplication and tracking patterns
RE_ACK_RX_CANCEL = re.compile(
    r"\[MC-DBG\]\s+ACK_RX_CANCEL\s+msg_id=([0-9A-Fa-f]+)\s+cancelled=(\d+)"
)
RE_ACK_FWD_DEDUP = re.compile(
    r"\[MC-DBG\]\s+ACK_FWD_DEDUP\s+msg_id=([0-9A-Fa-f]+)\s+already_pending"
)
RE_GW_ACK_DEDUP = re.compile(
    r"\[MC-DBG\]\s+GW_ACK_DEDUP\s+msg_id=([0-9A-Fa-f]+)\s+already_pending"
)
RE_ACK_SLOT_SKIP = re.compile(
    r"\[MC-DBG\]\s+ACK_SLOT_SKIP\s+iAckRead=(\d+)\s+\(cancelled\)"
)
RE_ACK_FAST_QUEUED = re.compile(
    r"\[MC-DBG\]\s+ACK_FAST_QUEUED\s+msg_ref=([0-9A-Fa-f]+)\s+ack_qlen=(\d+)"
)
RE_ACK_FAST_TX = re.compile(
    r"\[MC-DBG\]\s+ACK_FAST_TX\s+len=(\d+)\s+ack_qlen=(\d+)"
)
RE_ACK_FWD_DROPPED = re.compile(
    r"\[MC-DBG\]\s+ACK_FWD_DROPPED\s+ack_buf_full\s+pending=(\d+)"
)
RE_GW_ACK_DROPPED = re.compile(
    r"\[MC-DBG\]\s+GW_ACK_DROPPED\s+ack_buf_full\s+pending=(\d+)"
)
RE_ACK_CANCEL_RETRANSMIT = re.compile(
    r"\[MC-DBG\]\s+ACK_CANCEL_RETRANSMIT\s+slot=(\d+)\s+msg_id=([0-9A-Fa-f]+)"
)
RE_ACK_RECEIVED = re.compile(
    r"\[MC-DBG\]\s+ACK_RECEIVED\s+retid=(\d+)\s+msg_id=([0-9A-Fa-f]+)"
)
RE_ACK_FAST_CAD_BUSY = re.compile(r"\[MC-DBG\]\s+ACK_FAST_CAD_BUSY")

SIMPLE_COUNTERS = {
    "OnRxDone": "rx_packets",
    "OnRxTimeout": "rx_timeouts",
    "OnRxError": "rx_errors",
    "OnTXDone": "tx_packets",
    "OnTXTimeout": "tx_timeouts",
}

# Alert thresholds
STUCK_STATE_SECONDS = 30
NO_TRANSITION_SECONDS = 30
RX_TIMEOUT_ALERT_THRESHOLD = 10  # per summary interval
CAD_BUSY_STREAK = 6
RX_RESTART_PER_MIN_THRESHOLD = 70  # only counts RX_TIMEOUT_FIRE, not post-RX restarts
RADIO_SILENT_THRESHOLD = 20  # adaptive wait max ~16s at 95% util; 20s = real trouble
RING_ZOMBIE_CONSECUTIVE = 5  # consecutive RING_STATUS with retrying>0, queued==0 (150s)
HB_TIMEOUT_CYCLE_THRESHOLD = 3  # consecutive heartbeat timeouts = server unreachable

LORA_STATES = [
    "IDLE", "RX_LISTEN", "RX_PROCESS", "TX_PREPARE", "TX_ACTIVE", "TX_DONE",
]

STATE_CHAR = {
    "IDLE": ".",
    "RX_LISTEN": "R",
    "RX_PROCESS": "r",
    "TX_PREPARE": "C",
    "TX_ACTIVE": "T",
    "TX_DONE": "t",
}


class Monitor:
    def __init__(self, summary_interval: int) -> None:
        self.summary_interval = summary_interval
        self.start_time = time.monotonic()
        self.interval_start = time.monotonic()

        # Counters (reset each interval)
        self.counters: dict[str, int] = defaultdict(int)
        # Lifetime counters
        self.total: dict[str, int] = defaultdict(int)

        # State tracking
        self.current_state: str | None = None
        self.state_since = time.monotonic()
        self.last_transition = time.monotonic()
        self.state_time: dict[str, float] = defaultdict(float)  # per interval

        # CAD false-positive streak
        self.cad_streak = 0

        # RX restart tracking (per-minute ring)
        self.rx_restart_times: list[float] = []

        # WiFi/UDP status
        self.wifi_events_interval: list[str] = []
        self.udp_resets_interval = 0

        # Radio silent tracking (RX_TIMEOUT_FIRE gaps)
        self.last_rx_timeout_fire: float | None = None
        self.max_radio_gap: float = 0.0  # per interval
        self.max_radio_gap_total: float = 0.0  # lifetime
        self.radio_silent_events_interval: int = 0
        self.rx_process_since_last_timeout: int = 0  # RX_PROCESS events since last RX_TIMEOUT_FIRE

        # Ring buffer zombie tracking
        self.ring_last_queued: int = 0
        self.ring_last_pending: int = 0
        self.ring_last_retrying: int = 0
        self.ring_last_done: int = 0
        self.ring_zombie_streak: int = 0  # consecutive reports with retrying>0, queued==0

        # CSMA / adaptive wait tracking
        self.last_channel_util: int | None = None  # last reported util%
        self.channel_util_samples: list[int] = []  # all util% values this interval
        self.channel_rx_ms_total: int = 0  # cumulative rx airtime this interval
        self.channel_tx_ms_total: int = 0  # cumulative tx airtime this interval
        self.adaptive_wait_min: float | None = None  # per interval
        self.adaptive_wait_max: float | None = None  # per interval

        # No-transition silence tracking (for resolved alerts)
        self.radio_was_silent = False

        # NTP tracking
        self.ntp_fails_interval: int = 0
        self.ntp_last_ok: float | None = None

        # Heartbeat timeout cycle tracking
        self.hb_timeout_streak: int = 0
        self.hb_timeout_cycle_alerted: bool = False

        # Priority/Trickle tracking
        self.last_mc_stat: str | None = None
        self.last_mc_hwm: str | None = None

        # Alerts collected this interval
        self.alerts: list[str] = []

        # Inline activity indicator
        self.activity_indicator = True
        self._indicator_dirty = False

        self.lock = threading.Lock()

    def _indicator(self, ch: str) -> None:
        if self.activity_indicator:
            sys.stdout.write(ch)
            sys.stdout.flush()
            self._indicator_dirty = True

    def _indicator_newline(self) -> None:
        if self._indicator_dirty:
            sys.stdout.write("\n")
            sys.stdout.flush()
            self._indicator_dirty = False

    # -- event processing ---------------------------------------------------

    def process_line(self, line: str) -> None:
        with self.lock:
            self._process(line)

    def _process(self, line: str) -> None:
        now = time.monotonic()

        # State machine transitions
        m = RE_STATE_TRANSITION.search(line)
        if m:
            from_st, to_st, rc = m.group(1), m.group(2), int(m.group(3))
            # Resolved: first transition after radio silence
            if self.radio_was_silent:
                silence_dur = now - self.last_transition
                self._resolved(
                    f"Radio alive after {silence_dur:.0f}s silence — "
                    f"woke up via {from_st}->{to_st}"
                )
                self.radio_was_silent = False
            # accumulate time in previous state
            if self.current_state:
                self.state_time[self.current_state] += now - self.state_since
            self.current_state = to_st
            self.state_since = now
            self.last_transition = now
            self.counters["transitions"] += 1
            self.total["transitions"] += 1
            if to_st == "RX_PROCESS":
                self.rx_process_since_last_timeout += 1
            ch = "!" if rc != 0 else STATE_CHAR.get(to_st, "?")
            self._indicator(ch)
            return

        # Simple counters
        for keyword, counter_name in SIMPLE_COUNTERS.items():
            if keyword in line:
                self.counters[counter_name] += 1
                self.total[counter_name] += 1
                if counter_name == "rx_errors":
                    self._indicator("x")
                    self._alert(f"RX ERROR detected: {line.strip()}")
                if counter_name == "tx_timeouts":
                    self._indicator("!")
                    self._alert(f"TX TIMEOUT: {line.strip()}")
                return

        # Buffer drops
        m = RE_BUFFER_DROP.search(line)
        if m:
            drop_type = m.group(1)
            self.counters[f"drop_{drop_type}"] += 1
            self.total[f"drop_{drop_type}"] += 1
            self._alert(f"BUFFER DROP: {drop_type}")
            return

        # CAD scan results
        m = RE_CAD_SCAN.search(line)
        if m:
            result = int(m.group(1))
            self.counters["cad_scans"] += 1
            self.total["cad_scans"] += 1
            if result == -702:
                self.cad_streak += 1
                self.counters["cad_busy"] += 1
                self.total["cad_busy"] += 1
                if self.cad_streak == CAD_BUSY_STREAK:
                    self._alert(
                        f"CAD BUSY streak: {self.cad_streak}+ consecutive LORA_DETECTED (-702)"
                    )
            else:
                self.cad_streak = 0
            return

        # CAD giveup
        if RE_CAD_GIVEUP.search(line):
            self.counters["cad_giveups"] += 1
            self.total["cad_giveups"] += 1
            self._alert("CAD GIVEUP — forced TX after max retries")
            return

        # Retransmit giveup
        if RE_RETRANSMIT_GIVEUP.search(line):
            self.counters["retransmit_fails"] += 1
            self.total["retransmit_fails"] += 1
            self._alert(f"RETRANSMIT GIVEUP: {line.strip()}")
            return

        # WiFi debug
        m = RE_WIFI_DBG.search(line)
        if m:
            detail = m.group(1).strip()
            self.wifi_events_interval.append(detail)
            self.counters["wifi_events"] += 1
            self.total["wifi_events"] += 1
            self._alert(f"WIFI EVENT: {detail}")
            return

        # UDP reset
        if RE_UDP_RESET.search(line):
            self.udp_resets_interval += 1
            self.counters["udp_resets"] += 1
            self.total["udp_resets"] += 1
            self._alert("UDP RESET detected")
            return

        # Heartbeat timeout from server — track consecutive cycles
        if RE_HB_TIMEOUT.search(line):
            self.hb_timeout_streak += 1
            self.counters["hb_timeouts"] += 1
            self.total["hb_timeouts"] += 1
            if (self.hb_timeout_streak >= HB_TIMEOUT_CYCLE_THRESHOLD
                    and not self.hb_timeout_cycle_alerted):
                self._alert(
                    f"SERVER UNREACHABLE: {self.hb_timeout_streak} consecutive "
                    f"heartbeat timeouts (~{self.hb_timeout_streak * 90}s)"
                )
                self.hb_timeout_cycle_alerted = True
            return

        # Server responded (RX-UDP = GATE packet from server) — reset HB streak
        if "RX-UDP" in line:
            if self.hb_timeout_streak > 0:
                if self.hb_timeout_cycle_alerted:
                    self._resolved(
                        f"Server back after {self.hb_timeout_streak} "
                        f"heartbeat timeouts (~{self.hb_timeout_streak * 90}s)"
                    )
                self.hb_timeout_streak = 0
                self.hb_timeout_cycle_alerted = False
            return

        # NTP failure detection
        if RE_NTP_FAIL.search(line):
            self.ntp_fails_interval += 1
            self.counters["ntp_fails"] += 1
            self.total["ntp_fails"] += 1
            self._alert(f"NTP FAILURE: {line.strip()}")
            return

        if RE_NTP_OK.search(line):
            if self.ntp_fails_interval > 0:
                self._resolved("NTP recovered")
            self.ntp_last_ok = now
            return

        # RING_STATUS parsing and zombie tracking
        m = RE_RING_STATUS.search(line)
        if m:
            self.ring_last_queued = int(m.group(1))
            self.ring_last_pending = int(m.group(2))
            self.ring_last_retrying = int(m.group(3))
            self.ring_last_done = int(m.group(4))
            if self.ring_last_retrying > 0 and self.ring_last_queued == 0:
                self.ring_zombie_streak += 1
                if self.ring_zombie_streak >= RING_ZOMBIE_CONSECUTIVE:
                    self._alert(
                        f"RING_ZOMBIE retrying={self.ring_last_retrying} "
                        f"stuck for {self.ring_zombie_streak * 30}s+"
                    )
            else:
                self.ring_zombie_streak = 0
            return

        # RING_OVERFLOW — silence raw_rx (web-UI cyclical buffer), alert others
        m = RE_RING_OVERFLOW.search(line)
        if m:
            buf_name = m.group(1)
            self.counters[f"ring_overflow_{buf_name}"] += 1
            self.total[f"ring_overflow_{buf_name}"] += 1
            if buf_name != "raw_rx":
                self._alert(f"RING_OVERFLOW buf={buf_name}")
            return

        # CHANNEL_UTIL periodic report (every 10s from firmware)
        m = RE_CHANNEL_UTIL.search(line)
        if m:
            rx_ms, tx_ms, util_pct = int(m.group(1)), int(m.group(2)), int(m.group(3))
            self.last_channel_util = util_pct
            self.channel_util_samples.append(util_pct)
            self.channel_rx_ms_total += rx_ms
            self.channel_tx_ms_total += tx_ms
            self.counters["channel_util_reports"] += 1
            self.total["channel_util_reports"] += 1
            return

        # CAD_FALSE_POSITIVE (double-scan filter catching stale SPI reads)
        if RE_CAD_FALSE_POSITIVE.search(line):
            self.counters["cad_false_pos_filtered"] += 1
            self.total["cad_false_pos_filtered"] += 1
            return

        # RX_TIMEOUT_DEFERRED (IRQ guard saves a packet)
        if RE_RX_TIMEOUT_DEFERRED.search(line):
            self.counters["rx_timeout_deferred"] += 1
            self.total["rx_timeout_deferred"] += 1
            return

        # RELAY_LOOP_BLOCKED (loop detection prevented a relay)
        if RE_RELAY_LOOP_BLOCKED.search(line):
            self.counters["relay_loop_blocked"] += 1
            self.total["relay_loop_blocked"] += 1
            return

        # -- ACK deduplication tracking -----------------------------------------

        # ACK_RX_CANCEL: heard another ACK, cancelled N pending
        m = RE_ACK_RX_CANCEL.search(line)
        if m:
            cancelled = int(m.group(2))
            self.counters["ack_rx_cancel"] += 1
            self.total["ack_rx_cancel"] += 1
            self.counters["ack_rx_cancel_saved"] += cancelled
            self.total["ack_rx_cancel_saved"] += cancelled
            return

        # ACK_FWD_DEDUP: prevented duplicate re-forward
        if RE_ACK_FWD_DEDUP.search(line):
            self.counters["ack_fwd_dedup"] += 1
            self.total["ack_fwd_dedup"] += 1
            return

        # GW_ACK_DEDUP: prevented duplicate gateway ACK
        if RE_GW_ACK_DEDUP.search(line):
            self.counters["gw_ack_dedup"] += 1
            self.total["gw_ack_dedup"] += 1
            return

        # ACK_SLOT_SKIP: TX skipping cancelled slot
        if RE_ACK_SLOT_SKIP.search(line):
            self.counters["ack_slot_skip"] += 1
            self.total["ack_slot_skip"] += 1
            return

        # ACK_FAST_QUEUED: ACK queued for TX
        if RE_ACK_FAST_QUEUED.search(line):
            self.counters["ack_fast_queued"] += 1
            self.total["ack_fast_queued"] += 1
            return

        # ACK_FAST_TX: ACK actually transmitted
        if RE_ACK_FAST_TX.search(line):
            self.counters["ack_fast_tx"] += 1
            self.total["ack_fast_tx"] += 1
            return

        # ACK_FWD_DROPPED: ACK dropped, buffer full
        m = RE_ACK_FWD_DROPPED.search(line)
        if m:
            self.counters["ack_fwd_dropped"] += 1
            self.total["ack_fwd_dropped"] += 1
            self._alert(f"ACK_FWD_DROPPED: buffer full, pending={m.group(1)}")
            return

        # GW_ACK_DROPPED: GW ACK dropped, buffer full
        m = RE_GW_ACK_DROPPED.search(line)
        if m:
            self.counters["gw_ack_dropped"] += 1
            self.total["gw_ack_dropped"] += 1
            self._alert(f"GW_ACK_DROPPED: buffer full, pending={m.group(1)}")
            return

        # ACK_CANCEL_RETRANSMIT: cancelled a retransmit slot
        if RE_ACK_CANCEL_RETRANSMIT.search(line):
            self.counters["ack_cancel_retransmit"] += 1
            self.total["ack_cancel_retransmit"] += 1
            return

        # ACK_RECEIVED: received ACK for own message
        if RE_ACK_RECEIVED.search(line):
            self.counters["ack_received"] += 1
            self.total["ack_received"] += 1
            return

        # ACK_FAST_CAD_BUSY: CAD busy during ACK TX
        if RE_ACK_FAST_CAD_BUSY.search(line):
            self.counters["ack_cad_busy"] += 1
            self.total["ack_cad_busy"] += 1
            return

        # RX_TIMEOUT_FIRE: adaptive wait tracking, radio-silent gap, flood detection
        if "RX_TIMEOUT_FIRE" in line:
            m_fire = RE_RX_TIMEOUT_FIRE.search(line)
            if m_fire:
                wait_val = float(m_fire.group(1))
                if self.adaptive_wait_min is None or wait_val < self.adaptive_wait_min:
                    self.adaptive_wait_min = wait_val
                if self.adaptive_wait_max is None or wait_val > self.adaptive_wait_max:
                    self.adaptive_wait_max = wait_val

            # Radio silent gap detection
            if self.last_rx_timeout_fire is not None:
                gap = now - self.last_rx_timeout_fire
                if gap > self.max_radio_gap:
                    self.max_radio_gap = gap
                if gap > self.max_radio_gap_total:
                    self.max_radio_gap_total = gap
                if gap > RADIO_SILENT_THRESHOLD:
                    if self.rx_process_since_last_timeout > 0:
                        # Channel was busy — not a radio problem
                        self._indicator("B")
                        self.counters["channel_busy"] += 1
                        self.total["channel_busy"] += 1
                    else:
                        # No RX activity either — real radio silence
                        self.radio_silent_events_interval += 1
                        self.total["radio_silent"] += 1
                        self._alert(f"RADIO_SILENT gap={gap:.0f}s (no RX_TIMEOUT_FIRE)")
            self.rx_process_since_last_timeout = 0
            self.last_rx_timeout_fire = now

            # Flood detection — only timeout-driven restarts count
            prev_count = len(self.rx_restart_times)
            self.rx_restart_times.append(now)
            cutoff = now - 60
            self.rx_restart_times = [t for t in self.rx_restart_times if t > cutoff]
            cur_count = len(self.rx_restart_times)
            if (cur_count > RX_RESTART_PER_MIN_THRESHOLD
                    and prev_count <= RX_RESTART_PER_MIN_THRESHOLD):
                self._alert(
                    f"RX RESTART flood: {cur_count}/min"
                )

        # Priority statistics from firmware
        if "[MC-STAT]" in line:
            self.counters["mc_stat_lines"] = self.counters.get("mc_stat_lines", 0) + 1
            self.last_mc_stat = line.strip()
            return

        if "[MC-PRIO]" in line:
            self.counters["mc_prio_lines"] = self.counters.get("mc_prio_lines", 0) + 1
            # Check for Prio-1 starvation (latency > 10000ms)
            m = re.search(r"p1_lat_max=(\d+)ms", line)
            if m and int(m.group(1)) > 10000:
                self._alert(f"PRIO1_STARVED: p1_lat_max={m.group(1)}ms (>10s)")
            return

        if "[MC-TRICKLE]" in line:
            self.counters["trickle_events"] = self.counters.get("trickle_events", 0) + 1
            if "SUPPRESS" in line:
                self.counters["trickle_suppress"] = self.counters.get("trickle_suppress", 0) + 1
            elif "TOPO_CHANGE" in line:
                self.counters["trickle_topo_change"] = self.counters.get("trickle_topo_change", 0) + 1
            return

        if "[MC-HWM]" in line:
            self.last_mc_hwm = line.strip()
            return

        # Priority drop alert
        if "RING_DROP_PRIO" in line:
            self.counters["ring_drop_prio"] = self.counters.get("ring_drop_prio", 0) + 1
            self.total["ring_drop_prio"] = self.total.get("ring_drop_prio", 0) + 1
            return

    def _alert(self, msg: str) -> None:
        self._indicator_newline()
        ts = datetime.now().strftime("%H:%M:%S")
        alert_line = f"[ALERT {ts}] {msg}"
        self.alerts.append(alert_line)
        print(f"\033[91m{alert_line}\033[0m", flush=True)

    def _resolved(self, msg: str) -> None:
        self._indicator_newline()
        ts = datetime.now().strftime("%H:%M:%S")
        resolved_line = f"[RESOLVED {ts}] {msg}"
        self.alerts.append(resolved_line)
        print(f"\033[92m{resolved_line}\033[0m", flush=True)

    # -- periodic checks (called from main thread) --------------------------

    def check_stuck_state(self) -> None:
        with self.lock:
            now = time.monotonic()
            if self.current_state and (now - self.state_since) > STUCK_STATE_SECONDS:
                if self.current_state not in ("RX_LISTEN",):
                    self._alert(
                        f"STUCK in {self.current_state} for "
                        f"{now - self.state_since:.0f}s"
                    )
            if (now - self.last_transition) > NO_TRANSITION_SECONDS:
                self.radio_was_silent = True
                self._alert(
                    f"No state transitions for {now - self.last_transition:.0f}s"
                )

    # -- summary ------------------------------------------------------------

    def print_summary(self) -> None:
        with self.lock:
            self._indicator_newline()
            now = time.monotonic()
            elapsed = now - self.interval_start
            uptime = now - self.start_time

            # finalize state time for current state
            if self.current_state:
                self.state_time[self.current_state] += now - self.state_since
                self.state_since = now

            t_start = datetime.now() - timedelta(seconds=elapsed)
            t_end = datetime.now()
            h, rem = divmod(int(uptime), 3600)
            m, _ = divmod(rem, 60)

            # state distribution
            total_state_time = sum(self.state_time.values()) or 1.0
            state_pcts = {
                s: self.state_time.get(s, 0) / total_state_time * 100
                for s in LORA_STATES
                if self.state_time.get(s, 0) > 0
            }
            state_str = ", ".join(
                f"{s} {p:.1f}%" for s, p in sorted(state_pcts.items(), key=lambda x: -x[1])
            )

            # drop summary
            drops = sum(v for k, v in self.counters.items() if k.startswith("drop_"))

            # wifi/udp/server status
            wifi_str = "stable" if not self.wifi_events_interval else "EVENTS"
            udp_str = "ok" if self.udp_resets_interval == 0 else f"{self.udp_resets_interval} resets"
            hb_to = self.counters.get("hb_timeouts", 0)
            ntp_f = self.ntp_fails_interval
            server_str = "ok" if hb_to == 0 else f"{hb_to} HB timeouts"
            ntp_str = "ok" if ntp_f == 0 else f"{ntp_f} failures"

            alert_str = "none" if not self.alerts else f"{len(self.alerts)} (see above)"

            # channel utilization stats
            if self.channel_util_samples:
                samples = self.channel_util_samples
                util_min = min(samples)
                util_max = max(samples)
                util_avg = sum(samples) / len(samples)
                util_str = f"min={util_min}% avg={util_avg:.0f}% max={util_max}%"
                airtime_str = (
                    f"rx={self.channel_rx_ms_total}ms "
                    f"tx={self.channel_tx_ms_total}ms "
                    f"({len(samples)} samples)"
                )
            else:
                util_str = "no data yet"
                airtime_str = ""

            # adaptive wait info
            if self.adaptive_wait_min is not None:
                wait_str = f"{self.adaptive_wait_min:.0f}-{self.adaptive_wait_max:.0f}ms"
            else:
                wait_str = "no data yet"
            deferred = self.counters.get("rx_timeout_deferred", 0)
            cad_fp_filtered = self.counters.get("cad_false_pos_filtered", 0)

            # ACK dedup stats
            ack_saved = (
                self.counters.get("ack_rx_cancel_saved", 0)
                + self.counters.get("ack_fwd_dedup", 0)
                + self.counters.get("gw_ack_dedup", 0)
                + self.counters.get("ack_slot_skip", 0)
            )
            ack_tx = self.counters.get("ack_fast_tx", 0)
            ack_queued = self.counters.get("ack_fast_queued", 0)
            ack_drops = (
                self.counters.get("ack_fwd_dropped", 0)
                + self.counters.get("gw_ack_dropped", 0)
            )
            ack_total_possible = ack_saved + ack_tx
            if ack_total_possible > 0:
                ack_eff_str = f"{ack_saved / ack_total_possible * 100:.0f}%"
            else:
                ack_eff_str = "n/a"

            # -- build summary: quiet-by-default, loud-when-interesting --
            SEP = "=" * 60
            lines: list[str] = [
                "",
                SEP,
                f"SUMMARY {t_start:%H:%M}-{t_end:%H:%M} "
                f"(uptime: {h}h{m:02d}m)",
                SEP,
            ]

            # always visible: core metrics
            lines.append(
                f"RX: {self.counters['rx_packets']} packets | "
                f"TX: {self.counters['tx_packets']} | "
                f"Errors: {self.counters['rx_errors']} | "
                f"Drops: {drops}"
            )
            lines.append(
                f"State: {state_str or 'no transitions'} | "
                f"Channel: {util_str}"
            )
            if airtime_str:
                lines.append(
                    f"Airtime: {airtime_str} | Adaptive wait: {wait_str}"
                )
            else:
                lines.append(f"Adaptive wait: {wait_str}")
            lines.append(
                f"Ring: queued={self.ring_last_queued} "
                f"retrying={self.ring_last_retrying} "
                f"done={self.ring_last_done} | Deferred: {deferred}"
            )
            lines.append(
                f"WiFi: {wifi_str} | UDP: {udp_str} | "
                f"Server: {server_str} | NTP: {ntp_str}"
            )

            # conditional: only when non-zero
            loop_blocked = self.counters.get("relay_loop_blocked", 0)
            cad_busy = self.counters.get("cad_busy", 0)
            retransmit_fails = self.counters.get("retransmit_fails", 0)
            if drops or loop_blocked or cad_busy or cad_fp_filtered or retransmit_fails:
                lines.append(
                    f"Drops: {drops} | Loop blocked: {loop_blocked} | "
                    f"CAD busy: {cad_busy} | CAD filtered: {cad_fp_filtered} | "
                    f"Retransmit fails: {retransmit_fails}"
                )

            channel_busy = self.counters.get("channel_busy", 0)
            if self.radio_silent_events_interval or channel_busy:
                lines.append(
                    f"Radio: {self.radio_silent_events_interval} silent events "
                    f"(max gap: {self.max_radio_gap:.0f}s) | "
                    f"Channel busy: {channel_busy}"
                )

            ack_received = self.counters.get("ack_received", 0)
            ack_cad_busy = self.counters.get("ack_cad_busy", 0)
            ack_cancel_retx = self.counters.get("ack_cancel_retransmit", 0)
            if ack_saved or ack_tx or ack_queued or ack_received or ack_drops:
                lines.append(
                    f"ACK: saved={ack_saved} "
                    f"(cancel={self.counters.get('ack_rx_cancel_saved', 0)} "
                    f"fwd_dedup={self.counters.get('ack_fwd_dedup', 0)} "
                    f"gw_dedup={self.counters.get('gw_ack_dedup', 0)} "
                    f"skip={self.counters.get('ack_slot_skip', 0)}) "
                    f"tx={ack_tx} eff={ack_eff_str} | "
                    f"queued={ack_queued} received={ack_received} "
                    f"cad_busy={ack_cad_busy} cancel_retx={ack_cancel_retx} "
                    f"drops={ack_drops}"
                )

            trickle_events = self.counters.get("trickle_events", 0)
            trickle_suppress = self.counters.get("trickle_suppress", 0)
            trickle_topo = self.counters.get("trickle_topo_change", 0)
            if trickle_events or trickle_suppress or trickle_topo:
                lines.append(
                    f"Trickle: events={trickle_events} "
                    f"suppress={trickle_suppress} "
                    f"topo_change={trickle_topo}"
                )

            prio_drops = self.counters.get("ring_drop_prio", 0)
            mc_stat_lines = self.counters.get("mc_stat_lines", 0)
            if prio_drops or mc_stat_lines > 1:
                lines.append(
                    f"Prio: stat_lines={mc_stat_lines} "
                    f"prio_drops={prio_drops} preempt=see [MC-STAT]"
                )

            if self.alerts:
                lines.append(f"Alerts: {len(self.alerts)} (see above)")

            # totals: always RX/TX, rest only if non-zero
            lines.append(SEP)
            totals_parts = [
                f"RX={self.total['rx_packets']}",
                f"TX={self.total['tx_packets']}",
            ]
            total_errors = self.total["rx_errors"]
            total_drops = sum(
                v for k, v in self.total.items() if k.startswith("drop_")
            )
            total_loop = self.total["relay_loop_blocked"]
            total_silent = self.total["radio_silent"]
            total_deferred = self.total["rx_timeout_deferred"]
            total_hb_to = self.total["hb_timeouts"]
            total_ntp = self.total["ntp_fails"]
            total_ack_saved = (
                self.total["ack_rx_cancel_saved"]
                + self.total["ack_fwd_dedup"]
                + self.total["gw_ack_dedup"]
                + self.total["ack_slot_skip"]
            )
            total_ack_tx = self.total["ack_fast_tx"]
            total_ack_drops = (
                self.total["ack_fwd_dropped"] + self.total["gw_ack_dropped"]
            )
            total_ack_rx = self.total["ack_received"]

            if total_errors:
                totals_parts.append(f"Errors={total_errors}")
            if total_drops:
                totals_parts.append(f"Drops={total_drops}")
            if total_loop:
                totals_parts.append(f"LoopBlocked={total_loop}")
            if total_silent:
                totals_parts.append(f"RadioSilent={total_silent}")
            totals_parts.append(
                f"(max gap: {self.max_radio_gap_total:.0f}s)"
            )
            if total_deferred:
                totals_parts.append(f"Deferred={total_deferred}")
            if total_hb_to:
                totals_parts.append(f"HB_Timeouts={total_hb_to}")
            if total_ntp:
                totals_parts.append(f"NTP_Fails={total_ntp}")
            if total_ack_saved or total_ack_tx or total_ack_drops or total_ack_rx:
                totals_parts.extend([
                    f"ACK_Saved={total_ack_saved}",
                    f"ACK_TX={total_ack_tx}",
                    f"ACK_Drops={total_ack_drops}",
                    f"ACK_Received={total_ack_rx}",
                ])
            lines.append(f"  TOTALS: {' '.join(totals_parts)}")
            lines.append(SEP)

            print("\n".join(lines), flush=True)

            # reset interval counters
            self.counters = defaultdict(int)
            self.state_time = defaultdict(float)
            self.alerts = []
            self.wifi_events_interval = []
            self.udp_resets_interval = 0
            self.radio_silent_events_interval = 0
            self.max_radio_gap = 0.0
            self.channel_util_samples = []
            self.channel_rx_ms_total = 0
            self.channel_tx_ms_total = 0
            self.adaptive_wait_min = None
            self.adaptive_wait_max = None
            self.ntp_fails_interval = 0
            self.interval_start = now


def reader_thread(
    ser: serial.Serial,
    monitor: Monitor,
    log_file,
    stop_event: threading.Event,
) -> None:
    """Read serial lines, log to file, feed to monitor."""
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            if not stop_event.is_set():
                print("\033[91m[SERIAL] Connection lost, retrying...\033[0m", flush=True)
                time.sleep(2)
            continue
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace").rstrip()
        except Exception:
            continue
        if not line:
            continue
        # Replace binary artefacts: keep printable ASCII, newline, tab;
        # replace everything else with a dot placeholder.
        line = "".join(
            ch if (ch in "\t\n" or (ch.isprintable() and ord(ch) < 0xFFFD)) else "."
            for ch in line
        )

        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"{ts}  {line}\n"
        log_file.write(log_line)
        log_file.flush()

        monitor.process_line(line)


def main() -> None:
    parser = argparse.ArgumentParser(description="MeshCom serial monitor")
    parser.add_argument(
        "--port",
        default="/dev/cu.usbserial-0001",
        help="Serial port (default: /dev/cu.usbserial-0001)",
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=300,
        help="Summary interval in seconds (default: 300)",
    )
    parser.add_argument(
        "--no-dtr",
        action="store_true",
        help="Suppress DTR/RTS to prevent hardware reset (needed for CP2102/ESP32, "
             "NOT for CDC-ACM/NRF52 where DTR=True is required)",
    )
    args = parser.parse_args()

    # Create log directory and file
    log_dir = "./meshcom_monitor"
    os.makedirs(log_dir, exist_ok=True)
    log_name = f"meshcom_{datetime.now():%Y-%m-%d_%H%M%S}.log"
    log_path = os.path.join(log_dir, log_name)

    print("MeshCom Serial Monitor")
    dtr_mode = "suppressed (--no-dtr)" if args.no_dtr else "enabled (default)"
    print(f"Port: {args.port} @ {args.baud}  DTR: {dtr_mode}")
    print(f"Log:  {log_path}")
    print(f"Summary every {args.interval}s")
    print("Press Ctrl+C to stop\n")

    # Open serial port.
    # --no-dtr: Suppress DTR/RTS to prevent hardware reset on CP2102 (ESP32).
    #   CP2102 on macOS: pyserial's open() asserts DTR/RTS before we can clear
    #   them, so we pre-configure the TTY via termios.
    #   See: https://github.com/pyserial/pyserial/issues/124
    # Default (DTR=True): Required for CDC-ACM (NRF52/RAK4630) where the device
    #   only considers a terminal connected when DTR is asserted.
    if args.no_dtr:
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL  # no DTR drop on close
        attrs[2] |= termios.CLOCAL  # ignore modem control lines
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        os.close(fd)
        ser = serial.Serial()
        ser.port = args.port
        ser.baudrate = args.baud
        ser.timeout = 1
        ser.dtr = False
        ser.rts = False
        ser.open()
    else:
        ser = serial.Serial(args.port, args.baud, timeout=1)

    monitor = Monitor(summary_interval=args.interval)
    stop_event = threading.Event()

    log_file = open(log_path, "w", encoding="utf-8")

    # Start reader thread
    reader = threading.Thread(
        target=reader_thread,
        args=(ser, monitor, log_file, stop_event),
        daemon=True,
    )
    reader.start()

    # Handle Ctrl+C
    def on_signal(sig, frame):
        stop_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Main loop: periodic summaries + stuck-state checks
    try:
        while not stop_event.wait(timeout=1):
            monitor.check_stuck_state()

            elapsed = time.monotonic() - monitor.interval_start
            if elapsed >= args.interval:
                monitor.print_summary()
    finally:
        stop_event.set()
        reader.join(timeout=3)
        print("\n--- Final Summary ---")
        monitor.print_summary()
        log_file.close()
        ser.close()
        print(f"Log saved: {log_path}")


if __name__ == "__main__":
    main()