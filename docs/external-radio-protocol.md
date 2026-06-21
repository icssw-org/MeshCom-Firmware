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

## Channel access

In external-radio mode the **bridge is the single channel-access authority**.
The firmware does not run native CAD/CSMA for a packet handed to the bridge;
`CHANNEL_BUSY` maps into the existing MeshCom backoff path. (The firmware-side
integration lands in a later milestone.)

## Testing

The protocol module is pure C++17 with no Arduino, sockets, RadioLib, or
cryptography, so it is unit-tested on the host:

```
pio test -e native_extradio
```

The HMAC itself is a transport concern and is tested there, not in the codec.

## Status

Draft for issue #1015. This milestone delivers the protocol module and its
tests only — no TCP transport, no MeshCom TX/RX integration, no RadioLib bypass.
LoRaHAM integration remains outside MeshCom firmware.
