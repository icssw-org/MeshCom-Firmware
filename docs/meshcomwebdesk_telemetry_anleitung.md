# Guide: Sending external sensor values from MeshComWebDesk

Practical step-by-step guide for using the `"type":"tele"` UDP telegram from
MeshComWebDesk. The full protocol reference (field names, limits, error codes)
is in [`ext_udp_telemetry.md`](ext_udp_telemetry.md) — this guide only covers
the practical steps up to the first working test.

**Important to understand:** The values are **not** sent as a standalone
telemetry packet — they are written directly into the node's sensor variables
and flow automatically into the next position beacon, exactly like a node
with real sensor hardware. They are therefore **indistinguishable from real
sensor data** to any monitor/dashboard.

---

## 1. Prerequisites

- The node runs firmware that includes `handleExternTelemetry()` in
  `src/extudp_functions.cpp`.
- The node and the computer running MeshComWebDesk are on **the same
  WiFi/LAN**.
- UDP port `1799` is not blocked by a firewall between the two.
- The node has **no real sensor hardware installed** (BME280/BMP3xx/AHT20/
  SHT21) — if it does, the node deliberately ignores the external values to
  avoid disturbing the real sensor hardware.

---

## 2. One-time configuration on the node

The extern-UDP interface is **off** by default. It must be enabled once via
the node's console (serial, BLE, or the corresponding fields in the official
MeshCom app):

```
--extudpip <IP-address-of-MeshComWebDesk>
--extudp on
```

Example, if the computer running MeshComWebDesk has the IP `192.168.1.43`:

```
--extudpip 192.168.1.43
--extudp on
```

**Important:** `--extudp on` fails ("Please set EXPUDP IP first") if no valid
IP was set beforehand with `--extudpip`. The IP also must not be identical to
the node's own IP.

> This configuration is **independent** of the actual `"tele"` feature — it's
> the same extudp setup that MeshComWebDesk presumably already uses for
> sending messages (`"type":"msg"`) and receiving the node's own `"type":"pos"`/
> `"type":"tele"` output. If that's already set up on your end, you can skip
> step 2.

Unlike an earlier version of this extension, `--gateway`/`--track` play **no**
role here — the position beacon always goes out over LoRa regardless of those
settings.

---

## 3. Sending the telegram from MeshComWebDesk

A simple UDP packet (no TCP, no connection needed) to `<node-IP>:1799` with
the following JSON payload — all fields optional, only the ones present are
applied:

```json
{"type":"tele","temp":23.3,"hum":60,"press":1018.5}
```

| Field    | Content                                |
|----------|------------------------------------------|
| `type`   | always `"tele"`                           |
| `temp`   | temperature in °C                         |
| `hum`    | relative humidity in %                    |
| `press`  | air pressure (QFE) in hPa                 |
| `temp2`  | 2nd temperature sensor (optional)         |
| `qnh`    | sea-level air pressure (optional)         |
| `gasres` | gas resistance (optional)                 |
| `co2`    | CO2 reading (optional)                    |

There is currently no field for rainfall or similar — only these 7 fixed
fields, because only these have a matching APRS comment field (see protocol
reference).

**Important:** values must be sent as JSON **numbers** (`"temp":23.3`), not
as strings (`"temp":"23.3"`) — ArduinoJson on the node side does not
automatically convert strings to numbers; a string value would silently
become `0.0`.

### Quick test without MeshComWebDesk (PowerShell)

```powershell
$client = New-Object System.Net.Sockets.UdpClient
$json = '{"type":"tele","temp":23.3,"hum":60,"press":1018.5}'
$bytes = [System.Text.Encoding]::ASCII.GetBytes($json)
$client.Send($bytes, $bytes.Length, "<node-IP>", 1799) | Out-Null
$client.Close()
```

Replace `<node-IP>` with the node's actual IP address.

### Example in C# (if MeshComWebDesk uses .NET)

```csharp
using System.Net.Sockets;
using System.Text;

var payload = "{\"type\":\"tele\",\"temp\":23.3,\"hum\":60,\"press\":1018.5}";
var bytes = Encoding.ASCII.GetBytes(payload);

using var client = new UdpClient();
client.Send(bytes, bytes.Length, nodeIpAddress, 1799);
```

---

## 4. Verifying it arrived

On the node (watching the serial console):

```
[EXT] tele accepted: temp=23.3 hum=60.0 press=1018.5 temp2=0.0 qnh=0.0 gasres=0.0 co2=0.0
```

Right after that, the node triggers an immediate position beacon. Shortly
after, the station should appear in the monitor/dashboard with the new
values — exactly like a station with real sensor hardware (e.g.
`/T=23.3/H=60.0/P=1018.5` in the position comment).

If instead this message appears in the log, see
[`ext_udp_telemetry.md`, section 4](ext_udp_telemetry.md#4-error-cases-log-messages-on-the-node)
for the exact cause:

- `[EXT] tele ignored: real sensor hardware detected on this node`
- `[EXT] tele missing recognized fields (temp/hum/press/temp2/qnh/gasres/co2)`

---

## 5. Quick summary

1. One-time: set `--extudpip <IP>` + `--extudp on` on the node.
2. From MeshComWebDesk: send a UDP packet with the `"type":"tele"` JSON to
   `<node-IP>:1799` (only the 7 known fields: `temp`, `hum`, `press`, `temp2`,
   `qnh`, `gasres`, `co2`, as JSON numbers, not strings).
3. The node logs `[EXT] tele accepted: ...`, sets the sensor variables, and
   immediately sends a position beacon with the new values.
4. The values stay set permanently (not just once) and flow into every
   subsequent beacon until overwritten or the node restarts.
5. Only works as long as the node has **no** real sensor hardware installed.

---

## 6. WebDesk-side configuration (Settings → Telemetry)

In the measurement mapping table, each row has a **Role** column. This one
column now drives two things at once:

- The map popup (roles `temp`/`humidity`/`pressure`, as before)
- The native extudp telegram (all 7 roles: `temp`/`humidity`/`pressure`/
  `temp2`/`qnh`/`gasres`/`co2`) — when "Extudp enabled" is turned on

There is no separate "slot" column anymore. Only one row may be assigned to a
given role (if assigned to multiple rows: the first row in the list wins,
further ones are ignored when sending).
