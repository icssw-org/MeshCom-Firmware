# External-radio companion protocol (v1, draft)

Optional, compile-time backend that lets MeshCom firmware use an **external
radio** reached over TCP instead of a local RadioLib chip. This document
specifies the wire protocol implemented by the hardware-independent module
`lib/external_radio_protocol`. It is a draft for issue #1015.

## Scope and non-goals

**In scope (v1):** a generic, framed, transport-agnostic protocol between the
firmware and a bridge; a bounded fail-closed parser; a session state machine;
optional one-way connection authentication.

**Non-goals (v1):** payload confidentiality/encryption; mutual authentication of
the bridge to the firmware; per-frame integrity tags; serial transport; mDNS;
TLS; a web configuration UI; multiple bridge backends; any LoRaHAM-specific
names, commands, sockets, paths, or assumptions. The Raspberry Pi bridge and the
LoRaHAM daemon are **out of tree** — they are not part of MeshCom firmware.

## Reference topology

```
MeshCom firmware process            (ESP32 hardware, or an ESP32 emulator)
        |  external-radio TCP connection  (firmware = client, bridge = server)
        v
Raspberry Pi bridge process
        |  local-only daemon connection   (Unix socket or loopback TCP; no auth)
        v
LoRaHAM daemon  ->  LoRaHAM Pi-HAT radio
```

Firmware and bridge are separate processes; they may run on separate hardware or
together on one Raspberry Pi for emulation/testing. The daemon link is local to
the Pi and outside this protocol.

## Trust model and deployment requirements

v1 provides **connection authentication only** (the bridge may authenticate the
firmware). It is **not** encryption and does **not** authenticate the bridge to
the firmware. The firmware makes no cryptographic claims beyond this.

Because the bridge controls a transmitter, the following deployment controls are
**required**, independent of the optional password:

- the bridge binds only to an intended private interface (not the Internet);
- restrict clients by firewall and/or source-IP allowlist;
- accept one active client;
- keep the daemon link local (Unix-domain socket with restrictive permissions,
  or a loopback-only TCP listener).

## Frame format

All multi-byte integers are **big-endian**.

```
offset  size  field
0       2     magic = 0x58 0x52  ('X','R')
2       1     version = 0x01
3       1     type
4       2     length   (payload length, 0..300)
6       2     seq
8       len   payload
```

Fixed limits: max payload `300` bytes; max raw LoRa packet `255` bytes; header
`8` bytes. The parser bounds `length` before any copy and **fails closed** on
bad magic, version, type, or length (no byte-level resynchronization). After any
parser or session error the transport must disconnect and reset the parser.

### TCP stream handling

TCP is a byte stream: a single read may deliver a partial frame, exactly one
frame, or several coalesced frames. The parser owns a **fixed buffer of one
maximum frame** and is never grown to hold "N frames" — TCP can coalesce any
number.

The ingest API is therefore prefix-based: `parserPush()` copies only as many
leading bytes as currently fit, reports how many it consumed, and returns a
status — `PARSER_PUSH_OK` (all bytes accepted), `PARSER_PUSH_NEED_DRAIN` (a
prefix, possibly none, was accepted: pop frames and retry the remainder), or
`PARSER_PUSH_INVALID_INPUT` (null data with nonzero length). The caller drains
all complete frames with `parserPop()` between pushes, so at most one *incomplete*
frame is ever buffered. No input byte is silently discarded — bytes that did not
fit remain the caller's to re-offer after popping. Required caller loop, per
socket read of `n` bytes:

```
size_t off = 0;
for (;;) {
  PopResult r = parserPop(p, f, err);
  if (r == POP_GOT_FRAME) { handle(f); continue; }   // payload is owned by f
  if (r == POP_ERROR)     { disconnect(); parserReset(p); break; }  // fail closed
  if (off >= n) break;                               // need more bytes from socket
  size_t took; ParserPushStatus st = parserPush(p, data + off, n - off, took);
  off += took;
  if (st == PARSER_PUSH_INVALID_INPUT) { disconnect(); break; }                 // caller bug
  if (st == PARSER_PUSH_NEED_DRAIN && took == 0) { disconnect(); parserReset(p); break; }
}
```

Because the buffer holds exactly one maximum frame and every legal frame fits in
it, a full buffer always either yields a frame or fails closed — the loop cannot
deadlock and uses bounded memory regardless of how TCP chunks the stream.

## Message types

| type | name | direction | payload |
|------|------|-----------|---------|
| 0x01 | HELLO | fw→bridge | version byte (== header version) |
| 0x02 | HELLO_ACK | bridge→fw | version byte (== header version) |
| 0x03 | AUTH_CHALLENGE | bridge→fw | 16-byte nonce |
| 0x04 | AUTH_RESPONSE | fw→bridge | 32-byte HMAC-SHA256 result |
| 0x05 | AUTH_RESULT | bridge→fw | 1 byte: 0=AUTH_OK, 1=AUTH_FAIL |
| 0x06 | CONFIGURE | fw→bridge | RadioConfig (17 bytes) |
| 0x07 | CONFIG_RESULT | bridge→fw | success: `0x00` + RadioConfig echo (18 bytes); failure: one nonzero byte |
| 0x08 | RX_PACKET | bridge→fw | rssi[2] snr[2] len[2] data[len], len ≤ 255 |
| 0x09 | TX_REQUEST | fw→bridge | raw MeshCom packet bytes, ≤ 255 |
| 0x0A | TX_RESULT | bridge→fw | 1 byte: 0=SUCCESS,1=CHANNEL_BUSY,2=TIMEOUT,3=RADIO_ERROR |
| 0x0B | PING | bridge→fw | empty |
| 0x0C | PONG | fw→bridge | empty |
| 0x0D | ERROR | either | 1 byte error code (0..3) |

### RX metadata (RX_PACKET)

`rssi` and `snr` are signed **big-endian `int16_t`**: `rssi` in **centi-dBm**
(e.g. `-12050` = −120.50 dBm) and `snr` in **centi-dB** (e.g. `-275` = −2.75 dB),
followed by a 2-byte data length and `len` raw bytes (`len ≤ 255`). The firmware
carries these raw; conversion to MeshCom internal units is a later TX/RX
integration concern.

### Keepalive (PING/PONG)

One-way in v1: the **bridge sends PING**, the **firmware replies PONG**. Both are
`seq = 0` with empty payload, and are valid only while operational (`READY_RX` or
`TX_PENDING`). The firmware never originates PING and **rejects an incoming
PONG**, a PING before it is operational, or any nonzero sequence/payload (fail
closed).

### Sequence rules

- `seq = 0` for every control message (HELLO, HELLO_ACK, AUTH_*, CONFIGURE,
  CONFIG_RESULT, RX_PACKET, PING, PONG, ERROR).
- `seq = 1..65535` for TX_REQUEST and TX_RESULT. The firmware's TX sequence
  generator never allocates `0` and wraps `65535 → 1`.

A frame violating these rules is rejected (fail closed).

## RadioConfig (normalized, 17 bytes)

```
freq_hz[4] bw_hz[4] sf[1] cr_denom[1] sync_word[2] preamble[2] tx_power_dbm[1] crc[1] ldro[1]
```

Values are normalized (Hz, integers, on-air sync word, dBm), **not** RadioLib
enums. `crc` and `ldro` are booleans and accept only `0` or `1`; any other value
is rejected. The bridge translates these to its own hardware and must not
silently approximate unsupported settings.

## Connection authentication (one-way, optional)

Mirrors the existing MeshCom **NetConsole** model: an optional one-way HMAC
challenge/response that lets the **bridge** authenticate the **firmware**.

```
firmware                         bridge
  --- HELLO ----------------------->
  <-- HELLO_ACK --------------------
  <-- AUTH_CHALLENGE (16-byte nonce)        (only if a password is configured)
  --- AUTH_RESPONSE (HMAC-SHA256(password, nonce)) -->
  <-- AUTH_RESULT (AUTH_OK | AUTH_FAIL) ----
  ... CONFIGURE only after AUTH_OK ...
```

- A fresh random 16-byte nonce is required for **every** TCP connection.
- The HMAC is raw `HMAC-SHA256(password_bytes, nonce)`, 32 bytes, exactly.
- **Open local mode:** if the bridge has no password, it may send
  `AUTH_RESULT(AUTH_OK)` immediately after `HELLO_ACK`. This matches existing
  NetConsole behavior and is only for an operator-controlled private link.
- AUTH_FAIL, a malformed auth frame, an invalid state, or an auth timeout closes
  and resets the connection.

**The generic protocol codec computes no HMAC and stores no password.** The TCP
transport (or a small crypto adapter) computes the HMAC over the nonce and the
configured password. The password is provisioned at runtime through the existing
MeshCom settings/NVS mechanism (the same place as the NetConsole password); it
never appears in source, tests, build configuration, or logs. CONFIGURE,
TX_REQUEST, RX_PACKET, and TX_RESULT are invalid before AUTH_OK.

## Session state machine

```
DISCONNECTED → CONNECTING → HANDSHAKE → AUTHENTICATING → CONFIGURING → READY_RX ⇄ TX_PENDING
                                                                            ↘  DEGRADED (→ disconnect)
```

- No RX or TX before HELLO/HELLO_ACK, AUTH_OK, and an **exact** configuration
  echo.
- RX_PACKET is accepted only while operational (READY_RX or TX_PENDING).
- Configuration only enters READY_RX when the bridge's echoed `RadioConfig`
  **exactly** equals the requested one. A missing echo, a mismatch, or a failure
  status fails closed — never a silent mismatch.
- Any malformed frame, illegal state transition, remote ERROR, or timeout sets
  DEGRADED and requests disconnect. The transport then disconnects and resets.

### Timeouts

Wall-clock timing belongs to the transport, which calls `onTimeout(kind)` for
`HANDSHAKE`, `AUTH`, `CONFIG`, or `PENDING_TX`. Every timeout fails closed and
requests disconnect, reporting the distinct reason `ERR_TIMEOUT` (so a timeout is
diagnosable separately from a protocol/state violation); a pending-TX timeout
additionally resolves the in-flight TX as `UNKNOWN`. No timeout ever becomes TX
success or triggers a resend.

## TX result semantics and retry safety

- A TCP write is **never** TX success. Only a matching, well-formed `TX_RESULT`
  resolves the one in-flight TX.
- Exactly one TX may be in flight. A stale or mismatched-sequence `TX_RESULT` is
  ignored. A malformed *matching* `TX_RESULT`, a link loss, or a timeout while a
  TX is pending resolves the outcome as **UNKNOWN** — never a false success.
- **Retry rule:** only `CHANNEL_BUSY` is later eligible for the existing MeshCom
  backoff/retry path. `TIMEOUT`, `RADIO_ERROR`, and `UNKNOWN` are **not**
  immediate-retry signals and must not trigger an automatic local resend (a
  resend could duplicate a transmission that actually went out). No local retry
  logic lives in this milestone.

## Asynchronous TX queue ownership

Because a final `TX_RESULT` arrives asynchronously, the firmware must hold the
selected outbound packet until the bridge resolves it. A submitted packet is
therefore marked as **externally pending** in the local TX ring and is **owned**
(retained, not consumed at submission) until the final bridge outcome:

- a pending slot is excluded from normal TX selection and from retransmission
  maintenance — it cannot be re-sent, aged, or dropped while owned;
- at most one external TX is pending at a time;
- the pending slot has a stable identity (a monotonic token plus the message id),
  so a late or duplicate result can never complete, requeue, or alter a slot that
  was meanwhile resolved by an incoming ACK and reused for another packet;
- **confirmed success** (a `SUCCESS` `TX_RESULT`) completes the slot exactly once
  and is distinct from request submission / socket acceptance;
- `CHANNEL_BUSY` returns the frame to the existing delayed-retry path (subject to
  the normal retry limit) and is distinct from an uncertain/failure outcome;
- `TIMEOUT`, `RADIO_ERROR`, `UNKNOWN`, disconnect, or reconfigure-while-pending
  release the slot through a deliberate, observable non-success terminal path —
  never a silent disappearance, never a false success, never an automatic resend.

## Channel access

In external-radio mode the **bridge is the single channel-access authority**.
The firmware does not run native CAD/CSMA for a packet handed to the bridge.
`CHANNEL_BUSY` returns the frame to the existing MeshCom retry path with a
**bounded delay** before the next external submission (derived from the normal
CSMA backoff timing); it is not re-sent in the same scheduling pass and is bounded
by the normal retry limit.

## Firmware integration (external-radio mode)

When the firmware is built with the external-radio backend and a bridge is
provisioned:

- the **local RadioLib RF path is bypassed** — the local transceiver is not
  begun, configured, or driven, and no local RX/CAD/TX runs; the bridge owns the
  RF chip. Runtime radio-setting changes are still validated/accepted and trigger
  a bridge config re-sync (whose readiness is gated by the exact `CONFIG_RESULT`
  echo, never by a socket write);
- **received packets** are delivered synchronously in normal main-loop context
  (never an ISR), one validated `RX_PACKET` at a time and in arrival order, into
  the existing MeshCom receive ingress (`OnRxDone`). Zero-length and oversized
  frames are dropped before ingress. Wire metadata is converted to MeshCom units
  with integer truncation toward zero: RSSI `centi-dBm / 100`, SNR `centi-dB / 100`
  clamped to the signed-8-bit range;
- **transmission** routes a selected MeshCom TX-ring entry through `TX_REQUEST`;
  the final outcome is resolved exactly once from the terminal `TX_RESULT` (or a
  single `UNKNOWN` on disconnect/timeout/send-failure/reconfigure). A socket write
  is never success. The in-flight entry is held in the ring as `EXT_PENDING` and is
  protected from priority-overflow eviction, and each terminal result re-verifies
  the slot still carries the owned message (status + message id) before mutating
  it — so a result can never disturb a slot that was meanwhile freed or reused.

### Bridge RF success vs MeshCom acknowledgment

A bridge `SUCCESS` `TX_RESULT` means only that **the RF send completed** — it is
**not** a MeshCom delivery acknowledgment. The two are kept distinct, exactly as
in local-radio mode:

- a **retransmittable** MeshCom entry (a user text message) is **not** completed
  by RF success. It re-enters the normal post-send waiting state, keeping its
  payload, message identity and retry count, until either a later incoming
  **MeshCom ACK** clears it through the existing ACK handling or normal
  **retransmission maintenance** exhausts the retry budget. Each retry is a fresh
  bridge `TX_REQUEST` with its own local ownership token; only one external TX is
  ever in flight;
- a genuine **one-shot** entry (relay/ACK/position/HEY) is completed by RF success
  exactly as the local path completes it;
- retransmission maintenance runs in external mode at the normal cadence but is
  message-level only — it never drives local CAD/TX/RX or RadioLib, and it skips
  the in-flight `EXT_PENDING` entry;
- `CHANNEL_BUSY` means **channel access was not granted** (no RF send). It is paced
  by a real bounded delay and uses its **own bounded channel-access attempt budget,
  separate from the MeshCom delivery retransmission accounting** — a busy outcome
  never consumes the delivery `retryCount`. The budget is **independently bounded
  per queued channel-access episode** (tracked per ring slot): selecting and
  attempting other queued messages between two of a message's busy outcomes does
  **not** reset its count, so a message exhausts at its real configured cap. A
  different message reusing a slot starts fresh; a confirmed RF send resets that
  slot's episode before MeshCom ACK waiting begins. Exhausting the channel-access
  budget is a deliberate non-success terminal. The pacing delay itself stays
  global — any busy result delays the next external submission;
- `TIMEOUT`/`RADIO_ERROR`/`UNKNOWN`/disconnect/reconfigure are terminal
  non-success — never an ACK-wait, never an immediate resend, never a false
  success.

## Testing

The protocol module is pure C++17 with no Arduino, sockets, RadioLib, or
cryptography, so it is unit-tested on the host:

```
pio test -e native_extradio
```

The HMAC itself is a transport concern and is tested there, not in the codec.

## Platform network readiness

The TCP transport connects only when the node's IP network is usable. That check
is a small generic predicate injected into the transport, kept platform-agnostic:

- the normal ESP32 build uses Wi-Fi connectivity as the predicate;
- an alternative platform integration may supply its own IP-network readiness
  predicate by providing a strong override of the weak `externalRadioNetworkReady()`
  symbol at link time, without modifying any tracked firmware source.

When readiness becomes false the transport stops the link safely — a pending TX
resolves as `UNKNOWN`, never a false success — and reconnects once readiness
returns.

## Configuration snapshot

`CONFIGURE` carries a snapshot of the active MeshCom radio settings, captured once
during external-radio setup **after the effective radio settings are finalized**
(country/region normalization applied and power defaulted/clamped), so the bridge
is configured with the same effective frequency/bandwidth/SF/coding-rate/preamble/
power the local radio path would use — including on first boot, a country change,
or a flash clear. The snapshot is normalized to the protocol-v1 units above
(frequency/bandwidth converted to integer Hz by finite-checked nearest rounding;
the coding-rate denominator and on-air sync word taken directly; CRC from the
active setting; **LDRO** derived as the effective value from the standard LoRa
low-data-rate rule, symbol time > 16 ms). If the active settings cannot be mapped
to a valid `RadioConfig`, the transport does not start and no placeholder is used.

Runtime radio-setting changes that flow through the normal MeshCom
reconfiguration path trigger a **controlled external transport reset**: the link
is closed, the fresh snapshot becomes the desired config, and the bridge must
receive and **exactly echo** a new `CONFIGURE` before the link is ready again. A
pending external TX interrupted by reconfiguration resolves as `UNKNOWN` and is
**not** immediately retried. An unchanged configuration is a no-op.

## Status

Draft for issue #1015. Implemented and host-tested: the generic protocol module
(`lib/external_radio_protocol`), the ESP32-only compile-time-optional TCP transport
(`lib/external_radio_tcp` + `src/esp32/external_radio_glue.cpp`, `-D EXTERNAL_RADIO`),
the active-config snapshot above, and controlled re-sync on runtime radio
changes. Still deferred: the RadioLib bypass, and injecting RX / driving the
MeshCom TX queue. LoRaHAM integration remains outside MeshCom firmware.
