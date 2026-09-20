# KISS/TCP interface — client protocol (for WebDesk & co.)

Exact wire contract of the shipped ESP32 v1.1 KISS interface. Use this to add
KISS as an optional transport next to ext-udp.

## 1. Connection

- **TCP** to `<node-ip>:8001` (fixed — `KISS_TCP_PORT`).
- **Single client.** While WebDesk is connected, Direwolf/YAAC/etc. cannot
  connect, and vice versa. ext-udp (UDP 1799) is a separate socket — KISS and
  ext-udp can run at the same time.
- **Authentication:** off by default (LAN only). Optional opt-in HMAC-SHA256
  challenge-response — `--kiss auth on` + `--passwd <secret>`. With auth on, a
  standard KISS client (Dire Wolf / YAAC) can no longer connect — see §9.
- Node prerequisites: `--kiss on` (server up — TCP connect succeeds),
  `--kiss tx on` to accept sends, `--kiss meta on` for RSSI/SNR frames,
  `--kiss auth on` for the handshake. All four are also switches on the web
  Settings page and `/setparam/?kiss=on|off`, `?kisstx=`, `?kissmeta=`,
  `?kissauth=`.
- **Single-client hygiene:** the node probes the connected socket every loop and
  drops a half-open peer; on WiFi loss the whole server is torn down and rebuilt.
  A live but idle *authenticated* client still holds the slot until it
  disconnects.
- **Reconnect:** on socket close / RST / read error the node is gone
  (reboot / network loss) — retry with backoff.

## 2. KISS framing (both directions)

TCP is a byte stream — reassemble across segments.

```
frame  =  FEND  type  data(escaped)  FEND
FEND   =  0xC0     FESC = 0xDB
escape :  0xDB 0xDC  ->  0xC0        0xDB 0xDD  ->  0xDB
```

- `type` = `(port << 4) | command`. Command is always `0` here.
  - `0x00` — **data** frame (AX.25 UI), port 0 — both directions
  - `0x10` — **RxMeta** frame, port 1, node→client (only with `--kiss meta on`)
  - `0x20` — **SrcInfo** frame, port 2, node→client (only for a clamped
    high-SSID origin — see §4a)
  - `0xF0` — **TX result** frame, port 15, node→client (reply to a client
    `0x00` frame — see §5)
- Consecutive `FEND`s / empty frames → ignore.
- Ports 1, 2 and 15 carry command `0` frames on a non-zero port. A strict KISS
  client routes them to a (non-existent) channel — Dire Wolf logs a channel
  error, a naive monitor may render the bytes as junk. A monitor should filter
  by port and only decode port 0. (RxMeta / TX-result content is otherwise as
  documented in §4 / §5.)

## 3. RX — data frame (`type 0x00`)

Payload = an **AX.25 UI frame, without FCS**:

```
[ dest 7 ][ src 7 ][ 0..8 digipeaters 7 each ][ 0x03 ][ 0xF0 ][ info ... ]
```

**Address field** (7 bytes each):
- bytes 0..5: callsign, each char shifted **left 1 bit** on the wire → decode
  with `>> 1`, strip trailing spaces.
- byte 6: `C/H (0x80) | reserved (0x60) | SSID<<1 (0x1E) | ext (0x01)`.
  `ssid = (b >> 1) & 0x0F`. `ext == 1` marks the **last** address of the field.
- Walk 7-byte blocks until `ext == 1`; then come control (`0x03`) and PID (`0xF0`).

| Field | Meaning |
|---|---|
| dest | the node's APRS-MC tocall (`--aprsmc`, default `APRSMC`) — **not meaningful**, ignore |
| src | **origin callsign** of the mesh packet (incl. SSID). SSIDs > 15 are clamped to 15 on the wire — AX.25 has only 4 SSID bits, so MeshCom's two-digit SSIDs (`-16`…`-99`) cannot be represented losslessly; distinct high-SSID stations all show as `-15`. When that clamp happened, a **SrcInfo frame (`0x20`, §4a)** with the real call follows — use that as the source (for display and replies). Do **not** auto-derive a reply addressee from this field when a `0x20` frame is present. |
| digipeaters | the mesh **relay nodes** that forwarded it (H-bit set) — the path |
| info | the APRS information field, verbatim |

**Info field content** — only two kinds are emitted:

- **Position** — `info` starts with `!` (MeshCom `!` frame):
  `!DDMM.mmN/DDDMM.mmE<symbol><comment>`
  The comment carries the full MeshCom extension set, e.g.
  `Ralf, F34, AVSK#MeshComWebDesk/B=100/A=000827/N9/P=1016.7/H=56.0/T=23.3/R=262;2626;26269;9;`
  (`/B=` battery %, `/A=` altitude ft, `/N` neighbour count, `/P /H /T` press/hum/temp,
  `/R=` relay-node list). `aprslib.parse()` decodes lat/lon/alt/symbol/comment.
- **Message** — `info` starts with `:`  →  `:ADDRESSEE :text`
  (addressee is 9 chars space-padded, then `:`). A group/`*` addressee = broadcast.
  A MeshCom message ACK arrives as an ordinary message: `:YOURCALL  :ackNN`
  (`ackNN` rewritten to the `nn` you sent on the original message — see §5).

MeshCom binary HEY (`@`) / ACK (`0x41`) frames and telemetry-only frames are
**not** sent over KISS. (The text ACK above *is* forwarded.)

### Reconstruct a TNC2 string for a standard APRS parser

```
tnc2 = f"{src}>{dest}" + ("," + ",".join(digis) if digis else "") + ":" + info
aprslib.parse(tnc2)
```

## 4. RX — RxMeta frame (`type 0x10`)

Sent **immediately after** the data frame it belongs to, only when
`--kiss meta on`. 3 bytes:

```
[ snr : int8  (dB)  ]
[ rssi: int16 little-endian (dBm) ]
```

Example bytes `06 D1 FF` → snr `+6` dB, rssi `0xFFD1` = `-47` dBm.
Associate it with the preceding `type 0x00` frame.

## 4a. RX — SrcInfo frame (`type 0x20`)

AX.25 addresses hold only a 4-bit SSID, so an origin call with SSID `-16`…`-99`
(routine in MeshCom) is **clamped to `-15`** in the `src` field of the `0x00`
frame. When — and only when — that happened, the node sends one extra frame
**immediately before** the `0x00` frame:

```
[ full origin callsign, ASCII, incl. real SSID ]   e.g.  "OE1XYZ-99"
```

No length byte or terminator — the FEND delimits it (normal SLIP escaping
applies). Not gated on `--kiss meta`. Buffer it and apply it as the **true
source** of the next `0x00` frame — for the monitor display and, crucially, as
the reply addressee (replying to the `-15` from the AX.25 frame would reach the
wrong station, or none).

If no `0x20` frame precedes a `0x00` frame, the AX.25 `src` is exact.
A standard KISS client ignores port 2.

## 5. TX — client → mesh (needs `--kiss tx on`)

Send a `type 0x00` KISS frame containing an AX.25 UI frame:

```
[ dest 7 ][ src 7 ][ 0x03 ][ 0xF0 ][ info ]     (no digipeaters, no FCS)
```

- **src = the operator's callsign.** The node accepts the frame **only if the
  base callsign (without SSID) matches the node's own call**, case-insensitive.
  Any other callsign → frame is **rejected with a `0x02` TX-result** (and logged
  as `[KISS] TX rejected: src ...` with `--loradebug on`).
  SSID is free: node `DH1FR-2`, send as `DH1FR-7` → goes out on the mesh as
  `DH1FR-7` (own object, no collision with the node). The **source** SSID must be
  0…15 (AX.25 address‑field limit) — your own call cannot be `…-99`. The message
  **addressee** has no such limit: it lives in the APRS info field as 9‑char
  text, so `:OE1XYZ-99:hi` reaches a two‑digit‑SSID station fine (see the table).
- **dest**: any tocall — the node ignores it. Use e.g. `APRS` or `APMESH`.
- `control = 0x03`, `pid = 0xF0`.
- The node sets msg_id, hop byte, FCS, and HW-type/FW-version bytes. Any
  digipeater path in your frame is ignored (the node uses its own `max_hop`).

**info field:**

| Kind | info | Result |
|---|---|---|
| Message | `:ADDRESSEE :text{nn` (**exactly** 9-char addressee, left-justified, space-padded, then `:`) | MeshCom DM / group message. The addressee is free APRS text, so a two-digit SSID works: `OE1XYZ-99` is already 9 chars → `:OE1XYZ-99:hi{01`. The client's `{nn` is remembered: MeshCom renumbers on the air, but the `:ackNN` coming back is rewritten to the client's original `nn` before it reaches the client, so APRS-message ACK matching works for a directly-connected client (no hub needed). `{nn` must be a well-formed APRS message number — `{` + 1…5 chars at the **end** of the info field (optionally `{nn}rr` reply-ack); a `{` elsewhere in the body is literal text. Numeric `nn` recommended (MeshCom's ack number is 3-digit decimal). |
| Message ACK | `:ADDRESSEE :ackNN` / `:rejNN` (info text is exactly `ack`/`rej` + 1…5 digits) | Relayed as a real MeshCom ACK to `ADDRESSEE` under your call — the original sender stops retransmitting. `rej` is delivered as an `ack` (MeshCom has no reject). Not injected as message text. |
| Message to all | `:*        :text` or `::text` | broadcast (both forms produce the same on-air payload) |
| Position | `!DDMM.mmN/DDDMM.mmE<sym>comment` (or `=…`) | MeshCom `!` beacon under your call |
| Position + timestamp | `@DDHHMMz…` or `/DDHHMMz…` | 7-char timestamp stripped, sent as `!` |

Rate: the node caps injections at 8 per second (a burst over that is rejected
with result `0x05`); and LoRa airtime is ~1–2 s per frame — don't burst.

### TX result (`type 0xF0`, node → client)

For **every** inbound `type 0x00` frame the node replies with one `0xF0`
frame — so a KISS-only client knows the outcome without needing the ext-udp
node-echo:

```
[ status : 1 byte ]  [ msg_id : 4 bytes LE ]   (msg_id only when status = 0x01)

0x01  accepted — queued for LoRa TX (msg_id follows; matches the msg_id the
      node will use on the air and, if ext-udp is also connected, in the
      "node" echo)
0x02  rejected — src base callsign != node call
0x03  rejected — --kiss tx off
0x04  rejected — unparseable frame / unsupported payload type / empty text
      (also: address field not terminated, or control/PID != 0x03/0xF0 —
       connected-mode AX.25 is not accepted, only UI frames)
0x05  rejected — injection rate cap (8/s) exceeded
```

`0x01` means *handed to the LoRa TX ring*, not *delivered* — end-to-end
delivery is still only known via a MeshCom ACK (DMs) or the APRS message
`ack` round-trip. Standard KISS clients ignore port 15.

## 6. What KISS does not give you (still use ext-udp for)

| | ext-udp | KISS |
|---|---|---|
| structured telemetry object (`temp1/hum/qfe/...` as numbers) | ✅ `tele` JSON | only raw `/T= /H=` in the comment |
| sender `hw_id` / `firmware` / `fw_sub` | ✅ | ✗ (no APRS field) |
| RSSI/SNR always | ✅ in every JSON | only with `--kiss meta on` (port 1) |
| HEY / path frames, ACK frames | partly | ✗ |

KISS gives, ext-udp doesn't: the **full position comment**, the **`/R=` relay
list** and **`/N` neighbour count**, the **digipeater path**, and a
**standard format** any APRS tool can read.

Also note:
- The client sees **only newly received RF frames** — not the node's own
  transmissions or anything injected locally (phone / web / this client). To see
  your own digipeated traffic, watch it via another node or ext-udp.
- A message text longer than the LoRa MTU (≈160 chars minus the addressee
  overhead) is rejected with `0x04`.
- The server needs an active **WiFi STA** connection (`WiFi.status() ==
  WL_CONNECTED`). It does **not** start on Ethernet-only boards or in WiFi-AP
  mode (ext-udp does).

## 7. Suggested WebDesk integration

- Settings: `Transport: ext-udp | KISS/TCP`, host + port (default `8001`).
- RX: KISS deframer + AX.25 decoder (a few dozen lines each, or a JS KISS/AX.25
  lib) → feed the reconstructed TNC2 line to an APRS parser. Buffer the last
  RxMeta and attach it to the next data frame. Likewise buffer a `0x20` SrcInfo
  frame and use it as the source of the next data frame (overrides the clamped
  AX.25 `src`) — needed so replies to a `-16`…`-99` station are addressed right.
- Monitor view: `src`, path (`digis`), info field, RSSI/SNR — one row per frame,
  all frame types visible.
- TX: build the AX.25 UI frame with the operator call as source; message or
  position in the info field; one KISS frame per send. Read the `0xF0` reply
  for a per-send outcome (accepted + msg_id / rejected + reason) — this is the
  read-only "KISS TX: ok / rejected" status, no config needed. Fall back to the
  ACK / echo-timeout heuristic for actual end-to-end delivery.
- KISS and ext-udp can run together — e.g. KISS for the raw monitor, ext-udp
  for the telemetry panel. The ext-udp `src_type:"node"` echo also fires for
  KISS-injected sends, so if you keep ext-udp connected you get that
  confirmation too (msg_id matches the `0xF0` frame).

## 8. Reference: minimal Python

```python
import socket
FEND=0xC0; FESC=0xDB; TFEND=0xDC; TFESC=0xDD

def frames(sock):
    buf=bytearray()
    while True:
        d=sock.recv(4096)
        if not d: return
        for b in d:
            if b==FEND:
                if buf: yield bytes(buf); buf.clear()
            else: buf.append(b)

def deslip(b):
    o=bytearray(); esc=False
    for x in b:
        if esc: o.append(FEND if x==TFEND else FESC if x==TFESC else x); esc=False
        elif x==FESC: esc=True
        else: o.append(x)
    return bytes(o)

def decode_call(a):
    c="".join(chr(x>>1) for x in a[:6]).strip()
    s=(a[6]>>1)&0x0F
    return f"{c}-{s}" if s else c

s=socket.create_connection(("192.168.1.102",8001))
real_src=None                             # set by the last 0x20 SrcInfo frame
for raw in frames(s):
    f=deslip(raw)
    if not f: continue
    if f[0]==0x10:                       # RxMeta
        snr=f[1]-256 if f[1]>127 else f[1]
        rssi=int.from_bytes(f[2:4],"little",signed=True)
        print("meta", snr, rssi); continue
    if f[0]==0x20:                       # SrcInfo — true origin of the next 0x00
        real_src=f[1:].decode("latin1","replace"); continue
    if f[0]==0xF0:                       # TX result of one of our sends
        st=f[1]
        if st==0x01:
            print("tx accepted, msg_id", int.from_bytes(f[2:6],"little"))
        else:
            print("tx rejected", {2:"bad call",3:"tx off",4:"bad frame",5:"rate limit"}.get(st,st))
        continue
    b=f[1:]; p=0; addrs=[]
    while p+7<=len(b):
        addrs.append(b[p:p+7]); last=b[p+6]&1; p+=7
        if last: break
    dst,src=decode_call(addrs[0]),(real_src or decode_call(addrs[1]))
    real_src=None
    digis=[decode_call(a) for a in addrs[2:]]
    info=b[p+2:].decode("latin1","replace")
    print(f"{src}>{dst}{(','+','.join(digis)) if digis else ''}:{info}")
```

(The `0x20` frame arrives **immediately before** the `0x00` it annotates —
buffer it and apply it to the next data frame, as this snippet does.)

To send (message):

```python
def enc(call, top, last):
    base,ssid=(call.split("-")+["0"])[:2]
    base=(base.upper()+"      ")[:6]
    return bytes((ord(c)<<1)&0xFF for c in base)+bytes([top|0x60|((int(ssid)&15)<<1)|(1 if last else 0)])

def kiss(payload):
    out=bytearray([FEND,0x00])
    for x in payload:
        out+= b"\xDB\xDC" if x==FEND else b"\xDB\xDD" if x==FESC else bytes([x])
    out.append(FEND); return bytes(out)

ax = enc("APRS",0x80,False)+enc("DH1FR-7",0x00,True)+bytes([0x03,0xF0])
ax += b":DH1FR-1  :hello from webdesk"
s.sendall(kiss(ax))
```

## 9. Optional authentication (`--kiss auth on`)

Off by default. When enabled **and** `--passwd` is set, the node runs a
plain-text (pre-KISS) HMAC-SHA256 challenge-response the moment a client
connects — the same scheme as the net console, keyed on `--passwd`:

```
node -> client :  "NONCE: <32 hex chars>\r\n"
client -> node :  "<64 hex = HMAC-SHA256(passwd, nonce)>\r\n"
node -> client :  "OK\r\n"     then normal KISS framing begins
             or:  "FAIL\r\n"   then the node closes the socket
```

- 15-second deadline: no valid response → `FAIL` + disconnect. This also stops a
  connection from squatting the single client slot.
- `--kiss auth on` with an empty `--passwd` logs a warning and does **not**
  enforce (open, as if auth were off).
- A standard KISS client (Dire Wolf, YAAC) does not speak this handshake — use
  auth **off** for those, on a trusted LAN.

```python
import hmac, hashlib
line = s.makefile().readline()                      # "NONCE: abcd...\r\n"
nonce = bytes.fromhex(line.split()[1].strip())
mac = hmac.new(b"<passwd>", nonce, hashlib.sha256).hexdigest()
s.sendall((mac + "\r\n").encode())
assert s.makefile().readline().startswith("OK")     # then proceed as §8
```
