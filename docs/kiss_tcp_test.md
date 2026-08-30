# KISS/TCP interface — v1 (ESP32) usage & test guide

Implementation of Variant C from `docs/kiss_mode_analysis.md`. ESP32 only,
compiled in by default (opt-out `-D DISABLE_KISS_TCP`).

## What v1 does

- TCP server on **port 8001**, **single client**, no authentication (LAN-only).
- **RX (mesh → client):** received MeshCom **text** and **position** frames are
  converted to **AX.25 UI frames** (no FCS) and sent as KISS data frames.
  Dedup is already done, so each mesh packet is delivered once.
  HEY / ACK frames are not converted.
- **TX (client → mesh, opt-in):** an AX.25 UI frame whose info field is an APRS
  **message** (`:ADDRESSEE :text`) or **position** (`!` `=` `@` `/`) is
  injected into the mesh. `msg_id`, hop byte, FCS are set by the firmware.
  The frame goes out **under the client's callsign** (so e.g. a phone as
  `DH1FR-7` tracks separately from the node `DH1FR-2`) — but **only if the
  client's base callsign matches this node's call**; frames from any other
  callsign are rejected (no spoofing). Timestamped position formats have the
  7-char timestamp stripped; the position is sent as a MeshCom `!` beacon.
  HEY / ACK / telemetry-only payloads are ignored.
- **RxMeta (opt-in):** after each RX data frame, a second KISS frame on
  **KISS port 1** (type byte `0x10`) with 3 bytes: `snr` (int8, dB),
  `rssi` (int16 little-endian, dBm). Standard KISS clients ignore it.

The on-air MeshCom protocol is unchanged; the node stays fully in the mesh.

## Enable

Serial / net console, or web UI (`Settings` — switches **KISS/TCP**,
**KISS TX**, **KISS RxMeta**):

```
--kiss on            # start the server (needs WiFi + IP)
--kiss tx on         # allow transmit from the client   (default off)
--kiss meta on       # send RxMeta frames               (default off)
--kiss                # show status
--info               # KISS state is listed here too
```

Persisted in `node_sset4` (bits `0x0010` / `0x0020` / `0x0040`).

## Verified (2026-08-30, Heltec V3, live mesh)

- RX text / position / message → AX.25 UI; `aprslib` parses position
  (lat/lon/alt/symbol) and message (addressee/text).
- TX: APRS message from a KISS client injected into the mesh, delivered
  to the target node.
- RxMeta: `META` frame (snr int8 = 6 dB, rssi int16 LE = -47 dBm) after
  each data frame when `--kiss meta on`.
- Builds: `heltec_wifi_lora_32_V3`, `ttgo-lora32-v21`, `wiscore_rak4631`.

## Wire format

```
KISS frame :  C0 <type> <data, SLIP-escaped> C0
              escape:  DB DC = C0 ,  DB DD = DB
type 0x00  :  AX.25 UI frame  (dest7, src7, 0..8 digi7, 0x03, 0xF0, info)
type 0x10  :  RxMeta          (snr:int8, rssi:int16 LE)   [only with --kiss meta on]
```

AX.25 addresses: 6 chars each `<< 1` (space-padded), then SSID byte
`C/H<<7 | 0x60 | SSID<<1 | ext`. Destination = the node's APRS-MC tocall
(`--aprsmc <call>`, default `APRSMC`). Digipeaters = the mesh relays, H-bit set.

## Testing on Windows

### 1. Raw frames — PowerShell, no install

```powershell
$ip = "192.168.x.x"; $port = 8001
$c = [Net.Sockets.TcpClient]::new($ip, $port)
$s = $c.GetStream(); $buf = [Collections.Generic.List[byte]]::new()
$rx = [byte[]]::new(4096)
function Show-Frame([byte[]]$f) {
  if ($f.Count -lt 1) { return }
  $o = [Collections.Generic.List[byte]]::new()
  for ($i = 0; $i -lt $f.Count; $i++) {
    if ($f[$i] -eq 0xDB -and $i+1 -lt $f.Count) {
      $i++; if ($f[$i] -eq 0xDC) {$o.Add(0xC0)} elseif ($f[$i] -eq 0xDD) {$o.Add(0xDB)}
    } else { $o.Add($f[$i]) }
  }
  $t = $o[0]; $d = $o[1..($o.Count-1)]
  $k = if ($t -eq 0) {"DATA"} elseif ($t -eq 0x10) {"META"} else {"0x{0:x2}" -f $t}
  $asc = -join ($d | % { if ($_ -ge 32 -and $_ -lt 127) {[char]$_} else {"."} })
  Write-Host "$k  $asc" -ForegroundColor Cyan
}
while ($true) {
  $n = $s.Read($rx,0,$rx.Length); if ($n -le 0) { break }
  for ($i=0; $i -lt $n; $i++) {
    if ($rx[$i] -eq 0xC0) { if ($buf.Count) { Show-Frame $buf.ToArray(); $buf.Clear() } }
    else { $buf.Add($rx[$i]) }
  }
}
```

Expect `DATA` lines whose ASCII column reads like
`<call>  APRSMC ... !DDMM.mmN/DDDMM.mmE#...` (position) or
`... ::ADDRESSEE :text` (message), and — with `--kiss meta on` — a `META`
line after each.

### 2. Decode APRS — Python

```
py -m pip install kiss3 aprslib
```
```python
import aprslib, kiss
def f(fr):
    raw = bytes(fr)
    try:    print(aprslib.parse(raw.decode("latin1")))
    except Exception: print("raw:", raw[1:].decode("latin1", "replace"))
k = kiss.TCPKISS(host="192.168.x.x", port=8001); k.start()
k.read(callback=f)
```

### 3. APRS software

| Software | Setup |
|---|---|
| **YAAC** (Windows/Java) | Configure → Ports → Add → **NetworkKISS**, host = node IP, port `8001` |
| **APRSdroid** (Android, same WiFi) | Connd. Protocol **TCP/IP**, `IP:8001`, connection type **KISS** |
| **PocketPacket** (iOS) | TNC → KISS over TCP, `IP:8001` |
| **PinPoint APRS** (Windows) | TNC type "KISS over TCP", `IP:8001` |

**Set the client's callsign to your own call** (any SSID, e.g. `-7` for a
phone). TX (`--kiss tx on`): send an APRS message or position from the client →
it appears on the mesh **under that callsign**. A client using a different
callsign is silently rejected (`--loradebug on` shows `TX rejected: src ...`).

### 4. Linux / WSL (optional — iGate)

```
socat PTY,link=/tmp/kt,raw TCP:192.168.x.x:8001 &
sudo kissattach /tmp/kt radio && axlisten -a
```
or point **aprx** at `/tmp/kt` as a KISS serial interface for an RX-only iGate.

## Limitations

- ESP32 only. nRF52 (RAK) compiles the feature out.
- Single client. For multiple consumers use a host-side hub (e.g. Direwolf's
  KISS server, or WebDesk re-serving).
- No auth — keep it on a trusted LAN. The callsign gate (base call must match
  the node call) is the only TX restriction; there is no per-SSID whitelist
  and no "gateway mode" that relays arbitrary calls yet.
- TX: APRS message + position payloads. No APRS-ack bridging (MeshCom ACKs stay
  firmware-internal). Injected positions are sent as a MeshCom `!` beacon
  (no timestamp, no telemetry extension).
- Injected frames carry the HW-type / firmware-version bytes of the KISS node
  (set by `encodeAPRS()`), so a phone injecting as `DH1FR-7` shows up with the
  node's hardware (e.g. "HELTEC-V3") in other nodes' MHeard lists.
- RX: HEY / ACK / telemetry-only frames are not surfaced.
