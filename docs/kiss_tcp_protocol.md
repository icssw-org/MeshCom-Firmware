# KISS/TCP interface — client protocol (for WebDesk & co.)

Exact wire contract of the shipped ESP32 v1.1 KISS interface. Use this to add
KISS as an optional transport next to ext-udp.

## 1. Connection

- **TCP** to `<node-ip>:8001` (fixed — `KISS_TCP_PORT`).
- **Single client.** While WebDesk is connected, Direwolf/YAAC/etc. cannot
  connect, and vice versa. ext-udp (UDP 1799) is a separate socket — KISS and
  ext-udp can run at the same time.
- **No authentication.** LAN only.
- Node prerequisites: `--kiss on` (server up — TCP connect succeeds),
  `--kiss tx on` to accept sends, `--kiss meta on` for RSSI/SNR frames.
  All three are also switches on the web Settings page and
  `/setparam/?kiss=on|off`, `?kisstx=`, `?kissmeta=`.
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
  - `0x00` — **data** frame (AX.25 UI), port 0
  - `0x10` — **RxMeta** frame, port 1 (only with `--kiss meta on`)
- Consecutive `FEND`s / empty frames → ignore.
- A client that only understands standard KISS ignores port 1 automatically.

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
| src | **origin callsign** of the mesh packet (incl. SSID) |
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

HEY (`@`), ACK, and telemetry-only frames are **not** sent over KISS.

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

## 5. TX — client → mesh (needs `--kiss tx on`)

Send a `type 0x00` KISS frame containing an AX.25 UI frame:

```
[ dest 7 ][ src 7 ][ 0x03 ][ 0xF0 ][ info ]     (no digipeaters, no FCS)
```

- **src = the operator's callsign.** The node accepts the frame **only if the
  base callsign (without SSID) matches the node's own call**, case-insensitive.
  Any other callsign → frame is **silently dropped** (logged as
  `[KISS] TX rejected: src ...` with `--loradebug on`).
  SSID is free: node `DH1FR-2`, send as `DH1FR-7` → goes out on the mesh as
  `DH1FR-7` (own object, no collision with the node).
- **dest**: any tocall — the node ignores it. Use e.g. `APRS` or `APMESH`.
- `control = 0x03`, `pid = 0xF0`.
- The node sets msg_id, hop byte, FCS, and HW-type/FW-version bytes. Any
  digipeater path in your frame is ignored (the node uses its own `max_hop`).

**info field:**

| Kind | info | Result |
|---|---|---|
| Message | `:ADDRESSEE :text` (9-char addressee, then `:`) | MeshCom DM / group message. A trailing `{nn` APRS msg-number is stripped. |
| Message to all | `:*        :text` or `::text` | broadcast |
| Position | `!DDMM.mmN/DDDMM.mmE<sym>comment` (or `=…`) | MeshCom `!` beacon under your call |
| Position + timestamp | `@DDHHMMz…` or `/DDHHMMz…` | 7-char timestamp stripped, sent as `!` |

Rate: no limit enforced, but LoRa airtime is ~1–2 s per frame — don't burst.

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

## 7. Suggested WebDesk integration

- Settings: `Transport: ext-udp | KISS/TCP`, host + port (default `8001`).
- RX: KISS deframer + AX.25 decoder (a few dozen lines each, or a JS KISS/AX.25
  lib) → feed the reconstructed TNC2 line to an APRS parser. Buffer the last
  RxMeta and attach it to the next data frame.
- Monitor view: `src`, path (`digis`), info field, RSSI/SNR — one row per frame,
  all frame types visible.
- TX: build the AX.25 UI frame with the operator call as source; message or
  position in the info field; one KISS frame per send.
- KISS and ext-udp can run together — e.g. KISS for the raw monitor, ext-udp
  for the telemetry panel.

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
for raw in frames(s):
    f=deslip(raw)
    if not f: continue
    if f[0]==0x10:                       # RxMeta
        snr=f[1]-256 if f[1]>127 else f[1]
        rssi=int.from_bytes(f[2:4],"little",signed=True)
        print("meta", snr, rssi); continue
    b=f[1:]; p=0; addrs=[]
    while p+7<=len(b):
        addrs.append(b[p:p+7]); last=b[p+6]&1; p+=7
        if last: break
    dst,src=decode_call(addrs[0]),decode_call(addrs[1])
    digis=[decode_call(a) for a in addrs[2:]]
    info=b[p+2:].decode("latin1","replace")
    print(f"{src}>{dst}{(','+','.join(digis)) if digis else ''}:{info}")
```

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
