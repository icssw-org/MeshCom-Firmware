# KISS/TCP interface for MeshCom — feasibility analysis

Status: **historical feasibility notes.** The interface has since shipped
(`kiss_functions.cpp` + `lib/kiss_ax25/`, ESP32, opt-out `-D DISABLE_KISS_TCP`);
the wire contract lives in `kiss_tcp_protocol.md`. Some pre-implementation
details below (task context, buffer sizes, exact hook lines) drifted from the
final code and are kept only for the reasoning trail.

**Goal:** provide a second, standards-based machine interface **alongside
ext-udp** — same idea (an IP client reads mesh traffic and injects into it), but
over an **industry standard** (KISS framing over TCP) instead of MeshCom's own
JSON.

**This is explicitly not "APRS for MeshCom".** The on-air protocol stays
unchanged, MeshCom stays MeshCom. That the interface additionally converts frames
to AX.25 is only the technical means for existing standard software to read them
without any MeshCom knowledge — a means, not an end, and not the name of the
feature.

Transport: **TCP** (not the serial console).

---

## 1. Constraints

- **On-air unchanged.** What goes into `radio.startTransmit()` and comes out of
  `OnRxDone()` stays bit-for-bit identical. Other MeshCom nodes notice nothing.
  The node stays fully active in the mesh meanwhile (RX, relay, ACK, phone,
  ext-udp).
- **No new radio protocol**, no AX.25 frames on air.
- The interface is **optional** and can be disabled by command / build flag.

---

## 2. Motivation / benefits

### 2.1 Primary benefit: the standard

KISS is the established host↔TNC standard in amateur radio. A KISS/TCP socket on
the node means **any third-party software can access MeshCom** without
reimplementing the MeshCom JSON schema — Direwolf, YAAC, Xastir, APRSIS32, the
Linux `ax25` stack (`kissattach`), plus any KISS library in any language (Python
`kiss` / `aioax25`, Node, Go, Perl …). If the payload is AX.25/APRS, any APRS
parser (`aprslib` etc.) decodes it directly.

ext-udp cannot do this: there, every client must implement the proprietary JSON
schema.

### 2.2 WebDesk: monitor across all frame types

ext-udp only delivers `msg` / `pos` / `tele` — what the firmware chose to expose
as JSON fields. Over KISS/TCP, WebDesk receives **every** received frame type
raw (text, position with the full APRS comment, HEY/path beacons, ACK), including
path / hop count and — via an optional extra frame — RSSI/SNR. That allows a
**complete monitor window** showing radio traffic as it actually is.

In the WebDesk UI, "connection: ext-udp | KISS/TCP" would be selectable; both
deliver the same messages, KISS additionally the rest.

### 2.3 Transport benefits (TCP, independent of format — see §9)

Reliable/ordered delivery, visible connection state, client connects to the node
(no fixed peer IP needed).

---

## 3. What KISS is / example Direwolf

KISS is SLIP-style framing for the host↔TNC channel:

| Byte | Meaning |
|------|---------|
| `0xC0` | FEND — frame start/end |
| `0xDB` | FESC — escape |
| `0xDB 0xDC` | represents `0xC0` in the data stream |
| `0xDB 0xDD` | represents `0xDB` in the data stream |

Frame: `FEND | type byte | data (escaped) | FEND`.
Type byte = port (bits 7–4) + command (bits 3–0).

Commands host→TNC: `0x00` Data, `0x01` TXDELAY, `0x02` P-persistence,
`0x03` SlotTime, `0x04` TXtail, `0x05` FullDuplex, `0x06` SetHardware,
`0x0F` Return. TNC→host: `0x00` Data (received packet).

**No length field, no checksum field.** The AX.25 FCS is **not** part of the KISS
data frame. The AX.25 UI frames inside the KISS frame are carried **without FCS**.

**Example Direwolf** (one of many standard programs): listens by default on
**TCP port 8001** (`KISSPORT 8001`), payload = AX.25 frame without FCS, control
`0x03`, PID `0xF0`. Its APRS functions work as long as the node emits **valid
AX.25 UI frames**.

---

## 4. Reference: how MeshCore does it

From `meshcore-dev/MeshCore`, `examples/kiss_modem/` (`KissModem.cpp`, ~850 LOC):

- **Separate firmware, not a runtime toggle.** Flashing the KISS firmware gives
  a pure TNC; **the node is then no longer in the mesh.** `main.cpp` instantiates
  only `KissModem + RadioDriver + board + sensors` and leaves out the entire
  mesh/routing layer.
- **This works only because MeshCore's radio layer is cleanly abstracted:**
  public primitives `recvRaw()`, `startSendRaw()`, `isReceiving()`,
  `isSendComplete()`, `onSendFinished()`.
- **Transport: serial only** (USB / UART `Serial1`), 115200 8N1.
- **Frame content = raw LoRa PHY payload**, treated by the modem as **opaque**.
  **No AX.25 conversion** — the host gets MeshCore packets in KISS framing.
  Payload ≤ 255 bytes (`MAX_TRANS_UNIT`), KISS frame ≤ 512 bytes.
- **CSMA is reimplemented by the modem itself**
  (`TX_IDLE→TX_WAIT_CLEAR→TX_SLOT_WAIT→TX_DELAY→TX_SENDING→TX_DONE_PENDING`),
  carrier sense via `radio.isReceiving()`. Only **one** TX at a time, otherwise
  `Error(0xF1)+TxBusy(0x07)`.
- **RX metadata:** optional `HW_RESP_RX_META` frame after each packet with
  `{snr, rssi}` (1 byte each), enabled via SetHardware.
- **SetHardware (0x06) as a side-channel API:** GetIdentity, crypto (Ed25519 /
  X25519 / AES-128+HMAC / SHA-256), SetRadio (freq/bw/sf/cr, LE), SetTxPower,
  GetStats/GetBattery/GetMCUTemp, GetVersion, Ping, Reboot.
  Response = `command | 0x80`.
- **Memory:** `_rx_buf[512]`, `_tx_frame_buf[2][1536]`, `_pending_tx[255]` →
  ~3.8 KB of buffers alone.

### What follows for MeshCom

| Aspect | MeshCore | MeshCom reality |
|---|---|---|
| Radio layer | cleanly abstracted | **no such separation** — `OnRxDone()` is entangled with dedup, MHeard, ring-buffer retx, ACK, phone, ext-udp queue, display |
| KISS as separate firmware | trivial (drop the mesh layer) | only via a parallel minimal RX/TX path on the same SX126x driver → refactor + concurrency |
| CSMA | modem builds its own state machine | **not needed** if you go through the existing ring buffer — CSMA/CAD applies automatically |
| Node in the mesh meanwhile | no | **yes** (the interface is just an extra tap / injector) |

MeshCore's "separate firmware, PHY bypass" is **not cheap to reproduce** in
MeshCom. The pragmatic path sits one layer higher — at the MeshCom frame level,
as an extra tap like ext-udp.

---

## 5. Four variants

### Variant A — KISS framing, MeshCom frame as opaque payload

The KISS data frame carries the complete MeshCom on-air frame. No AX.25
conversion.

- On-air unchanged, node stays in the mesh. Effort ~450–550 LOC.
- Standard software sees KISS *framing*, but the content is a MeshCom frame —
  only software with MeshCom knowledge can do anything with it. Matches
  MeshCore's approach.
- Use: raw frame channel for sniffer/logger, all frame types.

### Variant B — AX.25 on air

AX.25 UI frames additionally on the LoRa channel. **Violates the constraint (new
on-air format), rejected.**

### Variant C — KISS/TCP with AX.25 conversion in the interface path  ← chosen

MeshCom frames stay on air. The firmware converts **only in the interface path**,
bidirectionally, MeshCom frame ⇄ AX.25 UI frame, so standard KISS software
understands the frames.

- On-air **100% unchanged**, node fully in the mesh.
- On the host: valid AX.25 → any KISS/APRS standard software docks in.
- Effort ~650–850 LOC, ~1.5–2 weeks incl. interop testing.

### Variant D — ext-tcp: JSON over TCP, all frame types

Rebuild ext-udp as a **second layer over TCP**: same JSON principle, but
connection-oriented **and** extended to all frame types (today only
`msg`/`pos`/`tele`).

- On-air unchanged. Effort ~300–400 LOC (no AX.25 codec, no semantic mappings).
- **Only** WebDesk / clients with the MeshCom schema can dock in — **no**
  standard tooling.
- Fully covers the WebDesk monitor wish (§2.2) and the transport benefits (§9),
  but **not** the standard benefit (§2.1).

### Comparison

| | A | C | D |
|---|---|---|---|
| Transport | TCP | TCP | TCP |
| On-air changed | no | no | no |
| Node in mesh | yes | yes | yes |
| Format | MeshCom frame raw | AX.25/APRS | MeshCom JSON |
| Standard software | framing yes, content no | **yes** | no |
| WebDesk monitor all types | yes | yes | yes |
| AX.25 codec / mappings needed | no | **yes** | no |
| Effort LOC | ~450–550 | ~650–850 | ~300–400 |

**Variant C is the chosen direction** — the stated purpose is that arbitrary
standard KISS software can use MeshCom, not just WebDesk. WebDesk is one client
among several (and may itself re-serve KISS to further apps, see §6.1).

**D is an optional add-on, not an alternative** — a JSON convenience layer for
WebDesk's own use. C and D can share one port (first byte `0xC0` → KISS, `{` →
JSON) or use two ports. Whether D is worth building at all is an open question
(§16), not a fork in the plan.

Remainder of the document: detailed design for **C**.

---

## 6. Variant C — architecture

```
  Standard software (Direwolf/YAAC/Xastir) OR WebDesk   (host)
        |  TCP, KISS framing, AX.25 UI frames without FCS
        v
  ┌─────────────────────────────────────────────┐
  │  MeshCom node                                │
  │  ┌───────────────┐   ┌───────────────────┐   │
  │  │ KISS TCP server│   │ AX.25 ⇄ APRS ⇄     │   │
  │  │ (1 client)     │──►│ MeshCom translator │   │
  │  └───────────────┘   └─────────┬─────────┘   │
  │                                │             │
  │   RX:  OnRxDone → dedup → queue │ TX: encodeAPRS → ring buffer
  │                                v             │
  │             existing MeshCom LoRa stack      │  ← unchanged
  └─────────────────────────────────────────────┘
        |  LoRa, MeshCom frames  (as before)
        v
  rest of the MeshCom mesh
```

Two data paths, both **decoupled** via a deferred queue (pattern: `externQueue`
in `extudp_functions.cpp`). `queueKiss()` does a bounded `memcpy` only and never
touches sockets. (As shipped, the RX hook runs synchronously in the loop task on
mainstream ESP32 targets — `esp32loop → checkRX`; only on `BOARD_T5_EPAPER` does
the radio path run in `lora_task`, so `flushKissQueue()` snapshots each queue
slot before use.)

### RX path (mesh → host)

1. Hook in `lora_functions.cpp` where `queueExtern("lora", RcvBuffer, size,
   rssi, snr)` sits today (`lora_functions.cpp:771`, inside the
   `is_new_packet()` branch — **dedup already done**).
2. `if(bKISS) queueKiss(RcvBuffer, size, rssi, snr);` — copy only.
3. In the loop: drain queue → `decodeAPRS()` → build AX.25 UI frame → KISS-escape
   → send over TCP to the client. Optional RxMeta frame for RSSI/SNR.

### TX path (host → mesh)

1. TCP RX → KISS deframer (state machine, one assembly buffer).
2. Parse AX.25 address field + info field → `SRC`, `DST`, path, APRS payload.
3. Determine `payload_type` from `info[0]` (`!`,`=`,`@`,`/` → position; `:` →
   message; …).
4. Build a regular MeshCom frame via `initAPRS()` + `encodeAPRS()`. **`msg_id`,
   hop byte, FCS, HW/MOD, path bytes, firmware version are set by the
   firmware**, not the host.
5. Into the ring buffer → normal transmission with the existing CSMA/CAD/retx.

### 6.1 Multiple consumers: single client + hub, not node multi-client

The purpose (§5) is that several programs can use the node — but the node's KISS
server stays **single-client**. Reasons: it is simpler, and it spares the tight
nRF52/W5100S socket budget (4 total, see §10).

Multiplexing is done **one level up** by whichever client connects:

| Approach | Meaning | Cost |
|---|---|---|
| **node multi-client** | KISS server accepts N connections, every RX frame fans out to all, TX from any goes into the ring | +~60 LOC, +N×~600 B RAM; **critical on nRF52/W5100S** |
| **hub / relay pattern** ← preferred | one client (WebDesk **or** Direwolf) connects to the node and itself serves KISS to further apps — the AGWPE/KISS-server role Direwolf already fills | 0 firmware LOC, logic lives in the hub |

So WebDesk connects to the node as one KISS client, uses the frames for its own
display **and** can forward them to further applications (KISS-server side in
WebDesk). WebDesk is then both consumer and distributor, without the node needing
multi-client support.

Node multi-client stays a possible **ESP32-only** option for later if direct
multi-connect is really needed.

---

## 7. Frame mapping MeshCom ⇄ AX.25 (Variant C only)

The MeshCom payload is already APRS-shaped: after the 6-byte binary header comes
ASCII `SRC>DST:payload`, and `payload_type` (`:` `!` `@` `<`) is the APRS
data-type identifier. `decodeAPRS()` / `decodeAPRSPOS()` already parse it — the
conversion is essentially "wrap an AX.25 frame around it", not re-encoding.

| MeshCom | → AX.25 (toward host) | cleanliness |
|---|---|---|
| `msg_source_path` / `msg_destination_path` (`CALL-SSID`) | AX.25 address field: call (≤6, ASCII<<1) + SSID byte | direct, as long as call ≤6 chars |
| hop / relay list | digipeater path with H bits | honestly mappable |
| `!` position + `/A=` `/T=` `/H=` `/P=` | APRS position | direct (`decodeAPRSPOS`) |
| `:` text message | APRS message `:ADDRESSEE :text{nn` | roughly direct (ACK see §8) |
| `<` LoRa-APRS (0x3C) | already TNC2 → AX.25 1:1 | trivial |
| control / PID | fixed `0x03` / `0xF0` (UI) | — |
| FCS | none in the KISS frame | — |

Reverse direction analogous: parse AX.25 → fields → `encodeAPRS()`.

---

## 8. Lossy points / semantics to decide (Variant C)

| Topic | Problem | Proposal |
|---|---|---|
| **ACK layer** | MeshCom: 32-bit `msg_id` + binary ACK frame (`A`). APRS: `{nn` / `ackNN` in text. | Interface speaks pure APRS messaging; MeshCom ACK stays firmware-internal. Optional: an incoming APRS `ack` triggers a local MeshCom ACK. |
| **Mesh flooding vs. WIDEn-N** | Host sends paths like `WIDE1-1`. MeshCom floods with a hop counter. | Ignore the incoming APRS path, MeshCom uses its own `max_hop`. Outgoing: show the traversed node calls as the digipeater path. |
| **Dedup toward host** | every mesh hop retransmits the same packet | dedup by `msg_id` before forwarding — the hook is in the `is_new_packet()` branch, so free |
| **`@` HEY / path beacons** | MeshCom-specific, no APRS equivalent | drop, or optionally as APRS status (but make visible in the monitor) |
| **Long / tactical callsigns** | `node_call[10]`; AX.25 only 6 + SSID 0–15 | truncate or don't convert the frame (log hint) |
| **TX direction / abuse** | node transmits on the mesh on the host's behalf → loop/flood risk | default **RX-only**, TX via `--kiss tx on` opt-in, optional rate limit |
| **RSSI/SNR** | doesn't fit into the AX.25 UI frame | separate RxMeta frame (MeshCore style) after each RX data frame, switchable |
| **Source call on TX** | host may send for a foreign call | policy: pass the host call through or force the node call |

Variant D avoids **all** of these points (JSON transports the MeshCom fields
directly).

---

## 9. Transport: why TCP instead of UDP

Applies to Variant C **and** D — the format question is independent of it.

| Property | UDP (ext-udp today) | TCP (ext-tcp / KISS) |
|---|---|---|
| **Delivery** | best effort, silent losses (small lwIP UDP buffers on the ESP32, WiFi congestion, node busy in the radio callback) | reliable, retransmitted, ordered |
| **Connection state** | none — the node "goes quiet" on crash/network loss; the client cannot tell "no frames" from "node dead" | socket close / RST / keepalive → the client sees immediately that the node is gone; the node sees whether anyone is listening |
| **Who connects** | node → fixed configured peer IP (`node_extern`) | client → node; node may be on DHCP, changing/multiple clients without reconfiguring the node |
| **NAT / port forwarding** | fiddly (both ends must know IPs) | one connection, one forwarded port |
| **Backpressure** | slow client → frames are dropped | TCP throttles the sender — on the device this needs a **bounded queue with drop-oldest**, otherwise a blocking risk |
| **Framing** | message boundary for free (`parsePacket()`) | byte stream → explicit framing needed (KISS `0xC0` resp. NL-delimited JSON) |
| **Resources** | no per-connection state | accept loop, lifecycle, keepalive |

**For the WebDesk monitor** (long-lived, wants completeness, one consumer, low
volume) TCP is clearly the right choice — lost frames = gaps in the window, and
the connection state is useful in the UI. UDP fit the model "gateway pushes to a
known server", not "client wants a gapless live view".

---

## 10. Memory & platform risks

| Target | Flash | RAM |
|---|---|---|
| **ESP32 (4 MB)** | app partition 3324 KB, `firmware.bin` ~1.4 MB → ~1.9 MB free. C: +8–15 KB, D: +3–5 KB. Non-issue. | +~2 KB (C) / +~1 KB (D). Non-issue. |
| **nRF52 / RAK4631** | ~816 KB app region, historically 85–95% used → **behind build flag `KISS_MODE`** (pattern `EXTERNAL_RADIO`). | loop-task stack 4 KB → all buffers **static/BSS**. C: AX.25 frame ≤330 B + RX assembly ≤512 B + 2-slot queue ~560 B ≈ **~1.7 KB BSS**. |

### Risks

- **nRF52 / W5100S: only 4 hardware sockets total.** Already used today: UDP to
  the server (1990), UDP ext (1799), NTP. A TCP listener + accepted socket = up
  to 2 more → **may run tight / be exhausted**. Must be checked before an nRF52
  implementation; possibly KISS on nRF52 only with ext-udp disabled.
- **SPI sharing nRF52:** radio and W5100S share the SPI bus (`bSPI_ETH_Active`
  guard). An additional TCP server increases SPI contention in the RX path —
  check timing.
- **Blocking risk on TCP send** in loop context → mandatory non-blocking socket
  + bounded queue.
- **ESP32 only** is usable without extra hardware (WiFi). nRF52 needs the
  W5100S, like ext-udp today.

---

## 11. Effort estimate

### Variant C

| Building block | LOC | Note |
|---|---:|---|
| SLIP/KISS codec (encode/deframe) | ~120 | platform-independent |
| deferred RX queue + hook | ~60 | copy of `externQueue` |
| AX.25 UI encode / decode | ~240 | address field, path, control/PID |
| APRS-type ⇄ MeshCom assembly glue | ~80 | uses `initAPRS` / `encodeAPRS` |
| dedup hook toward host | ~20 | in the `is_new_packet()` branch |
| TCP server (ESP32 lwIP; nRF52 EthernetServer) | ~150 | model `net_console.cpp` |
| commands + persist | ~50 | `--kiss on/off`, `--kiss tx on/off`, `--kiss port` |
| RxMeta channel | ~40 | optional |
| **Total** | **~650–850** | + docs |

**Time:** ~1.5–2 weeks, much of it interop testing against the matrix in §13.4.

### Variant D

As above **without** the AX.25 codec, without APRS glue, without the semantic
mappings: **~300–400 LOC**, ~4–6 days. Extends the existing
`extudp_functions.cpp` with a TCP path and with serializers for the remaining
frame types.

### Files touched (C)

- **new:** `src/kiss_functions.cpp` / `.h`, `docs/kiss_tcp_protocol.md`
- **new optional:** `lib/ax25_aprs/` (platform-independent codec, fail-closed
  parser)
- `src/lora_functions.cpp` — 1–2 lines at the RX fan-out (`:771`)
- main loop (`esp32_main.cpp` / `nrf52_main.cpp`) — `kissLoop()`
- `src/command_functions.cpp` — commands (pattern `extudp on/off` at `:2404`)
- `src/esp32/esp32_flash.cpp`, `src/nrf52/nrf52_flash.cpp` — load flag
- `src/configuration_global.h` — port define, `node_sset*` bit
- `variants/*/platformio.ini` — `-D KISS_MODE` where wanted

---

## 12. Configuration / settings

Modest, and it mirrors what **ext-udp + net_console** already have. No new flash
struct field, no peer-IP setting (the client connects *to* the node).

### 12.1 Console commands (`command_functions.cpp`)

Following the `--extudp` / `--netconsole` pattern
(`command_functions.cpp:2404` / `:2231`):

| Command | Purpose | Analogue |
|---|---|---|
| `--kiss on/off` | interface on/off | `--extudp on/off` |
| `--kiss tx on/off` | allow TX injection (**default off**, RX-only) | — (KISS-specific safety) |
| `--kiss meta on/off` | RxMeta frames (RSSI/SNR) | — optional |

Plus: help text (`:729` / `:766`) and the `--info` status line (`:5025`, where
`EXTUDP … EXT IP` is printed today).

**No `--kissip`** needed — with KISS the client connects to the node, not the
other way round. The node only needs its own (already configured) IP.

### 12.2 Web UI (`web_functions.cpp` + `web_setup.cpp`)

Next to the existing switches (`web_functions.cpp:1123-1126`):

```cpp
_create_setup_switch_element("kiss",   "KISS/TCP", "enable KISS interface (port 1798)", bKISS);
_create_setup_switch_element("kisstx", "KISS TX",  "allow transmit from KISS clients",  bKISSTX);
```

Plus the `paramName.equals("kiss")` / `"kisstx"` handlers in
`web_setup.cpp:468-482` and `:885-891` (set + get), same scheme as
`extudp` / `netconsole`. Guard with `#ifndef BOARD_RAK4630` resp.
`#if defined(HAS_ETHERNET)` like net_console — on nRF52 only useful with the
W5100S.

### 12.3 Persistence (`esp32_flash.cpp` / `nrf52_flash.cpp`)

- **2–3 bits in `node_sset2` / `node_sset3`** (enable, tx-enable, meta) — no
  struct field, **no flash-format change**.
- **Port fixed at `1798`** for v1 (already reserved in
  `configuration_global.h` as `EXTERN_RAW_PORT`). A configurable port would need
  an `int node_kiss_port` → struct bump → avoid for v1.

### 12.4 Optional / later

- **Phone-app S2 block** (`command_functions.cpp:5310`, `swdoc2["EUDP"]`): only
  needed if the MeshCom app should show the toggle. Defer for a niche feature —
  console + web UI is enough.
- **`Web-API_documentation.txt`**: document `/setparam/?kiss=on|off` and
  `/setparam/?kisstx=on|off`.
- **Auth:** none (LAN-only like ext-udp) needs nothing. HMAC like net_console
  can reuse the existing `node_passwd[15]`.

### 12.5 Total

~2 commands + 2 web switches + 2–3 sset bits + API doc. Already included in the
"commands + persist" line (~50 LOC) of the effort estimate (§11).

---

## 13. Compatible software and use cases

All of the following speak KISS and assume **AX.25 UI frames** (Variant C).
Suitability is filtered by what MeshCom actually is: a low-bandwidth LoRa mesh,
~255-byte frames, APRS-shaped payloads (position + text messaging), slow duty
cycle, mesh flooding rather than AX.25 digipeating.

### 13.1 Clearly useful

| Software | Platform | Use with MeshCom | Verdict |
|---|---|---|---|
| **aprx** | Linux (headless) | **RX-only iGate** — mesh positions/messages reach aprs.fi / APRS-IS. Lightweight, native KISS over TCP. | ★ best entry point, no TX risk |
| **Direwolf** | Linux/Win/Mac | as a KISS *client* over TCP: iGate to APRS-IS, plus an AGWPE/KISS server for further apps. `kissutil` for testing ships with it. | ★ versatile; leave the digipeat function **off** |
| **YAAC** | Java (all) | "NetworkKISS" → node on the LAN. Full APRS client: map, messaging, optional iGate. | ★ desktop operator view of the mesh |
| **APRSdroid** | Android | connection mode "TCP/IP" speaks KISS to `host:port`. Phone on the same WiFi as the node → APRS messaging UI without BLE pairing. | ★ very convenient for messages |
| **PinPoint APRS** | Windows | "KISS over TCP", good maps, messaging. | good for Windows users |
| **`axlisten` / ax25-apps** | Linux | after `kissattach` the node appears as `ax0` → live frame monitor, logging. | ★ exactly the monitor use case |
| **Wireshark** | all | KISS/AX.25 dissector — offline analysis of captures. | debugging |
| **custom scripts** | Python `kiss` + `aioax25` + `aprslib` | for WebDesk itself or automation (MQTT bridge, logger, alerting). | ★ this is also the WebDesk path |

### 13.2 Conditional / with caveats

| Software | Why cautious |
|---|---|
| **Xastir** | KISS-over-TCP not cleanly native — usually via PTY / `socat`. Works, but clumsier than YAAC. |
| **LinBPQ / BPQ32** (G8BPQ) | Can wire MeshCom into a packet-node network (chat nodes, mail). But: BBS mail forwarding fragments badly at 255-byte frames → experiments only. |
| **APRSIS32** | Windows, KISS-TCP possible; overlaps with YAAC/PinPoint, no extra advantage. |

### 13.3 Deliberately not used

- **AX.25 connected mode** (`axcall`, BBS connects, keyboard-to-keyboard): needs
  timely ACKs/retries — LoRa latency + duty cycle make it unusable. Only **UI
  frames** (APRS) are realistic.
- **Digipeating in Direwolf/aprx**: MeshCom already floods within the mesh
  itself. APRS WIDEn-N digipeating on top would be redundant and would soften
  the "on-air stays MeshCom" rule → leave it off.
- **TX iGate (APRS-IS → mesh)**: possible, but raises the abuse/loop/duty-cycle
  topics from §8 — opt-in only.

### 13.4 Interop test matrix

| Tool | Direction | Checks |
|---|---|---|
| `kissutil` (Direwolf) + Python `kiss` | RX + TX | raw framing, escaping, TX injection into the ring |
| **aprx**, RX-only | RX | dedup toward host, position/message → APRS-IS, path representation |
| **Direwolf** iGate | RX (+ opt. TX) | AX.25 UI validity, APRS-IS gating, `kissutil` cross-check |
| **YAAC** NetworkKISS | RX + TX | message round-trip, map display, SSID/callsign handling |
| **APRSdroid** TCP | RX + TX | message round-trip from a phone on the LAN |
| `kissattach` + `axlisten` | RX | Linux `ax25` stack accepts the frames, `ax0` interface |

### 13.5 Most valuable use case

**RX-only iGate with aprx or Direwolf**: one MeshCom node on the LAN that mirrors
local mesh traffic to APRS-IS — without the central MeshCom server, with standard
tooling, immediately visible on aprs.fi. This is where the AX.25 effort (Variant
C over D) pays for itself.

---

## 14. Relation to existing interfaces

### 14.1 ext-udp — same category, shared infrastructure

`extudp_functions.cpp` (port 1799, JSON/UDP) is the direct relative: read mesh
traffic / inject on the IP side next to the radio. The new interface shares with
ext-udp:

- **the same RX tap** (`lora_functions.cpp:771`, `queueExtern` next to it)
- **the same deferred-queue pattern** (`externQueue`, `extudp_functions.cpp:47`)
- **the same platform limitation** (ESP32 WiFi / nRF52 W5100S)

The only difference is **transport + format**: ext-udp = UDP + MeshCom JSON, only
`msg`/`pos`/`tele`. New = TCP + (KISS/AX.25 **or** extended JSON), all frame
types. It **does not replace ext-udp** — it sits next to it, WebDesk chooses.

### 14.2 MeshCom server — not the comparison point

A gateway node pushes mesh traffic via UDP to the central server
(`udp_functions.cpp:449`, port 1990). That is the **ecosystem connection**
(server routing, web maps), not a local machine interface and not the topic
here. The new interface is a **local** client connection like ext-udp —
independent of whether the node is a gateway at all.

### 14.3 external-radio — build template only

`external_radio_glue.cpp` (`-D EXTERNAL_RADIO`, TCP) goes the opposite direction
(node = protocol brain, remote box = PHY). Functionally unrelated, but provides
the **pattern**: "frames over a TCP socket" with a platform-independent,
fail-closed protocol lib in `lib/` and a compile flag. That is exactly how the
new interface should be built.

---

## 15. Recommendation

1. **Transport TCP** (§9) — connection-oriented, reliable, client connects.
2. **Variant C** (KISS/AX.25). The stated purpose is that arbitrary standard
   KISS software can use MeshCom — that is only reachable via C (§2.1). Variant D
   (JSON/TCP) is an optional convenience layer for WebDesk, not an alternative
   (§16).
3. **Single-client KISS server + hub pattern** for multiple consumers (§6.1) —
   no node multi-client on nRF52.
4. **Default RX-only**, TX opt-in.
5. Codec platform-independent in `lib/`, fail-closed — like external-radio.
6. Behind build flag `KISS_MODE`.
7. **Coordinate with icssw-org before implementing** — port assignment, auth,
   nRF52 socket budget, the semantic decisions in §8, and whether the feature
   belongs in the firmware or as a host tool.

---

## 16. Open questions for the upstream discussion

- Build Variant D (JSON/TCP) additionally, or not at all? If yes: same port as
  C (`0xC0` vs. `{`) or a separate one?
- Default port: `1798` (already reserved in `configuration_global.h` as
  `EXTERN_RAW_PORT`) or `8001` (Direwolf convention)?
- Auth on the TCP port: none (LAN-only, like ext-udp) / peer-IP binding / HMAC
  like `net_console.cpp`?
- Is the nRF52/W5100S socket budget (4 total) enough next to ext-udp + server
  UDP for a single-client KISS listener?
- RxMeta (RSSI/SNR): separate frame or drop it?
- For C: source call on host TX — pass through or force the node call?
- Feature in the firmware, or a thin firmware raw path (Variant A) plus the
  AX.25 translation in a host-side hub?
