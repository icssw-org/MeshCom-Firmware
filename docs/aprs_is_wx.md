# APRS-IS weather over Wi-Fi (ESP32)

An ESP32 node with a BME280 (or any sensor that fills `node_temp` / `node_hum` / `node_press`)
can report its own weather to APRS-IS directly over Wi-Fi, so that aprs.fi shows the station as
a weather station (symbol `_`) instead of telemetry. Off by default.

## Commands

| Command | Meaning |
|---------|---------|
| `--aprswx on` / `off` | enable / disable (default off; `on` also brings Wi-Fi up, STA mode with an SSID) |
| `--aprswx` | status (passcode shown as `***`) |
| `--aprswxint N` | minutes between reports, 5..1440 (default 10) |
| `--aprswxcall X` | source call/SSID for APRS-IS; `-` = use the node call |
| `--aprsis server H` | APRS-IS server (default `euro.aprs2.net`) |
| `--aprsis port P` | port (default 14580) |
| `--aprsis pass N` | numeric APRS-IS passcode (empty = `-1`, receive only: the server drops the packet) |
| `--sendaprswx` | send one report now (needs `--aprswx on`) |

All settings are persisted but not part of the JSON export. The passcode is never printed,
logged or exported.

## Frame

```
SO2MK-13>APRS,TCPIP*:!5213.78N/02100.73E_.../...g...t071h45b10132
```

- position from the node position (must be non-zero), symbol `/` `_` (weather)
- wind is always `.../...g...` (no anemometer data, no rain)
- `tTTT` temperature in degrees F, three digits (`t-05` for below zero); `t...` if there is no reading
- `hHH` humidity %, `h00` means 100 %; omitted if 0
- `bBBBBB` sea level pressure in tenths of hPa (`node_press_asl`, falling back to `node_press`); omitted if 0
- login: `user CALL pass PASS vers MeshCom <firmware version>`

A report is sent only when there is an IP address, a position and at least one reading.
A temperature of exactly 0.0 C is treated as "no reading".

## Behaviour and limits

- ESP32 only; nRF52 builds answer "not supported".
- Non-blocking state machine in the main loop (DNS, connect, banner, login, send, close), whole
  cycle limited to 15 s, **one attempt per cycle** (no retries until the next interval).
- Log lines: `[APRS-IS] ok`, `[APRS-IS] fail:<dns|connect|banner|login|send|timeout>`,
  and `[APRS-IS] skip:<noip|nopos-or-wx|busy|nocall>` when a cycle could not start.
  A `logresp ... unverified` reply (wrong passcode) is reported as `fail:login`.
- No wind, rain or other weather fields; nothing is sent over LoRa.
- With `--aprswx off` neither the timer nor the client executes any step.
