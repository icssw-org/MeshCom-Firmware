# Extern-UDP interface extension: external sensor values in the position beacon

Extends the existing JSON UDP interface (port `1799`, `src/extudp_functions.cpp`)
with a new incoming message type `"tele"`, which lets an external client
supply sensor values for a node that has **no physical sensor hardware of
its own**. The values are written directly into the node's
sensor variables (`meshcom_settings.node_temp`, `node_hum`, `node_press`, ...)
and therefore flow automatically into the next position beacon — exactly like
a node with real sensor hardware (e.g. `/T=23.9/H=34.3/Q=1013.6` in the APRS
comment). To any receiving station (and any monitor/dashboard), they are
indistinguishable from genuine sensor data.

Implemented in `src/extudp_functions.cpp`, function `handleExternTelemetry()`
(called from `getExtern()`).

> **History note:** An earlier version of this extension sent the values as a
> separate APRS telemetry packet (`T#...`, destination address `100001`). That
> was dropped because common monitors/dashboards (demonstrably including the
> MeshCom firmware itself via `sendExtern()`) only ever evaluate the sensor
> fields embedded in the position beacon, never standalone `T#` packets. This
> version therefore writes directly into the sensor variables instead.

---

## 1. Message format (client → node)

UDP packet to the node, port `1799`, JSON payload — all fields optional, only
the fields present are applied:

```json
{"type":"tele","temp":23.3,"hum":60,"press":1018.5,"temp2":0,"qnh":1018.5,"gasres":0,"co2":0}
```

| Field    | Target variable                  | APRS comment field in `PositionToAPRS()` |
|----------|-----------------------------------|-------------------------------------------|
| `temp`   | `meshcom_settings.node_temp`      | `/T=`                                      |
| `hum`    | `meshcom_settings.node_hum`       | `/H=`                                      |
| `press`  | `meshcom_settings.node_press`     | `/P=`                                      |
| `temp2`  | `meshcom_settings.node_temp2`     | `/O=`                                      |
| `qnh`    | `meshcom_settings.node_press_asl` | `/Q=`                                      |
| `gasres` | `meshcom_settings.node_gas_res`   | `/G=`                                      |
| `co2`    | `meshcom_settings.node_co2`       | `/C=`                                      |

There is no channel-name/unit field anymore — the meaning of each value is
fixed by its APRS comment field (identical to real sensor hardware).

### Example

```json
{"type":"tele","temp":23.3,"hum":60,"press":1018.5}
```

Sets `node_temp=23.3`, `node_hum=60`, `node_press=1018.5`. `temp2`/`qnh`/`gasres`/`co2`
remain unchanged (no value in the JSON = no change to that variable).

---

## 2. What the node does afterwards

1. **Immediate position beacon:** The node immediately triggers a position
   beacon (`sendPosition(0x9999, ...)`, the same mechanism as the `--sendpos`
   console command), instead of waiting for the next scheduled beacon
   (default every 30 minutes, `POSINFO_INTERVAL`).
2. **On-air format:** The values appear as part of the normal `!` position
   packet, e.g.
   `DH1FR-2>*!5051.09N/00906.45E#.../B=100/A=000827/P=1018.5/H=60.0/T=23.3/N5/R=...`
   — no separate telemetry packet, no `PARM.`/`UNIT.` header lines needed.
3. **Persistent, not one-shot:** Unlike the earlier `"T:"` version, the values
   are **not** reset after being sent — they stay set and flow into **every**
   subsequent position beacon (including the regular ones every 30 minutes)
   until overwritten by a new `"tele"` message or the node restarts.

---

## 3. Limitations (please note in client implementations)

- **Protects real sensor hardware.** If the node detects physically installed
  sensor hardware (`bmx_found`/`bmp3_found`/`aht20_found`/`sht21_found` —
  BME280/BMP3xx/AHT20/SHT21), the external message is **completely ignored**
  and logged (`"tele ignored: real sensor hardware detected on this node"`).
  This prevents an injection from colliding with the values of real, onboard
  sensor hardware — on a node with no sensor hardware at all (the intended
  use case for this extension), this guard does not trigger.
- **No acknowledgement over UDP.** The protocol is "fire and forget" — there
  is no response/ACK on port 1799. Success or rejection is only visible in
  the node's serial log (`[EXT] tele accepted: ...` resp.
  `[EXT] tele ignored: ...` / `[EXT] tele missing recognized fields ...`).
- **No sender validation.** Like the existing `dst`/`msg` message, this
  channel is reachable by any device on the same LAN (no IP allowlist). No
  new security risk compared to the status quo, but no additional protection
  either.
- **No free-form channel-name field anymore.** Values that don't correspond
  to one of the 7 fixed fields (e.g. rainfall) currently cannot be
  transmitted via this interface — `PositionToAPRS()` has no matching comment
  field for them.

---

## 4. Error cases (log messages on the node)

| Log message                                                                  | Cause                                                              |
|-------------------------------------------------------------------------------|---------------------------------------------------------------------|
| `[EXT] tele ignored: real sensor hardware detected on this node`             | Node detected real sensor hardware — external values are rejected |
| `[EXT] tele missing recognized fields (temp/hum/press/temp2/qnh/gasres/co2)` | None of the known fields were present in the JSON                 |

---

## 5. Unchanged behavior

- The existing `{"type":"msg","dst":"...","msg":"..."}` message keeps working
  unchanged (separate code path, untouched).
- The outgoing `sendExtern()` telemetry (`"type":"tele"` from the node to the
  client, on received position packets) is a separate, unchanged mechanism.
  After a successful injection it will automatically reflect the new values
  (since it now reads the real `node_temp`/`node_hum`/... variables).
- Nodes that never receive a `"tele"` message behave exactly as before — the
  new code only becomes active when a matching UDP message actually arrives.
- `--gateway`/`--track` settings are **not** relevant to this mechanism
  (unlike the earlier `"T:"` version) — the position-beacon code path
  (`sendPosition()`) always places packets in the local LoRa TX buffer
  regardless of those settings.
