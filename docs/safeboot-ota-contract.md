# Safeboot OTA: interface contract (2026-09-13)

Binding for the safeboot campaign: firmware (`src/safeboot/`), the OTA page (`src/safeboot/ota.html`),
and the OTA client and abort bench (`tools/webflash.py`, `tools/bench/ota_abort.py` in the DK5EN
fork) are built against this document in parallel. Change it here first, then everywhere.

## Decisions (operator, 2026-09-13)

- WLAN: STA join as today (driver-selected AP, PMF-off config, auto-reconnect). **AP is not
  started at boot.** If no join after 25 s, `WIFI_AP_STA` is entered: the open AP (SSID = call sign,
  192.168.4.1) comes up and the STA keeps retrying for the rest of the window. A join that
  succeeds later leaves an already running AP on (no client is dropped); a join within 25 s means
  the AP never appears.
- AP stays open (no password). Fallback-to-app window stays 180 s.
- Scan results: RSSI, channel, auth mode. There is no SNR in the ESP32 scan API.
- Page language: English.
- **Single app slot (bench finding 2026-09-13, Heltec V3 kill50):** both partition tables
  (`partitions-4MB-safeboot.csv`, `partitions-16MB-safeboot.csv`) have exactly one app partition
  (`ota_0`). An upload writes into that slot from the first chunk on, so once an upload has
  started there is **no way back into the previous firmware**; `esp_ota_set_boot_partition(ota_0)`
  refuses the half-written image (`ESP_ERR_OTA_VALIDATE_FAILED`) and the bootloader boots
  safeboot again. Therefore: the safeboot checks the app image at boot and after every abort
  (`app_valid`), the fallback-to-app timer runs only while `app_valid` is true, and with
  `app_valid` false the node stays in safeboot (no reboot loop) until a complete upload succeeds.
  The page says so; `/ota/cancel` answers `409 app_invalid`.

## `GET /ota/info` -> `application/json`

```json
{
  "call": "DK5EN-93",
  "hostname": "DK5EN-93",
  "mdns": "DK5EN-93.local",
  "mode": "sta",
  "uptime_ms": 12345,
  "sta": {
    "configured": true,
    "ssid": "ORBI63",
    "connected": true,
    "bssid": "5A:AF:97:2E:2B:8B",
    "ip": "192.168.1.98",
    "rssi": -43,
    "channel": 6,
    "auth": "WPA2_WPA3_PSK",
    "last_disconnect_reason": 0,
    "join_attempts": 1
  },
  "ap": { "active": false, "ssid": "DK5EN-93", "ip": "192.168.4.1" },
  "scan": {
    "age_ms": 4200,
    "in_progress": false,
    "aps": [
      {
        "ssid": "ORBI63",
        "bssid": "5A:AF:97:2E:2B:8B",
        "rssi": -43,
        "channel": 6,
        "auth": "WPA2_WPA3_PSK",
        "connected": true
      }
    ]
  }
}
```

- `mode`: `"sta"` (STA only, AP not started), `"ap_sta"` (AP started after the 25 s), `"ap"`
  (no SSID configured or `--wifiap on`: AP only, as today).
- `sta.configured` false when the SSID is `none`; then the whole `sta` block carries empty strings / 0.
- `auth` strings follow `wifi_auth_mode_t` names without the `WIFI_AUTH_` prefix:
  `OPEN`, `WEP`, `WPA_PSK`, `WPA2_PSK`, `WPA_WPA2_PSK`, `WPA2_ENTERPRISE`, `WPA3_PSK`,
  `WPA2_WPA3_PSK`, `WAPI_PSK`, `OWE`, `WPA3_ENT_192`, `UNKNOWN`.
- `last_disconnect_reason`: the numeric `wifi_err_reason_t` of the most recent
  `ARDUINO_EVENT_WIFI_STA_DISCONNECTED`, 0 when none.
- `scan.aps` sorted by RSSI descending, at most 20 entries, only the entries with the configured
  SSID first is NOT required (the page sorts/marks). `connected` is true for the BSSID the STA is
  associated with. `age_ms` is the time since the scan finished; `in_progress` true while a
  rescan runs (then `aps` is the previous result).

## `GET /ota/state` -> `application/json`

```json
{
  "state": "aborted",
  "reason": "incomplete_upload",
  "generation": 3,
  "received": 1048576,
  "total": 2182465,
  "image_valid": false,
  "app_valid": false,
  "fallback_in_ms": -1,
  "uptime_ms": 98000
}
```

- `state`: `idle` | `receiving` | `verifying` | `done` | `aborted`.
- `reason`: empty in `idle`/`receiving`/`done`; on `aborted` one of `stale_session`,
  `write_failed`, `client_disconnected`, `stalled`, `incomplete_upload`, `md5_mismatch`,
  `begin_failed`; `verifying` is the window between the last chunk and the `Update.end()` verdict.
- `received`/`total`: bytes of the current or last session; `total` 0 when unknown.
- `app_valid`: true when the app partition holds a complete, verified image (checked at boot and
  re-checked after every abort). False after an aborted upload has written into the single app
  slot: the node then stays in safeboot until a full upload succeeds.
- `fallback_in_ms`: remaining time of the 180 s fallback-to-app window; `-1` while an upload is
  in progress (the window is suspended, the stall watchdog applies instead) and `-1` while
  `app_valid` is false (no fallback possible). `done` means the
  reboot into the app is scheduled.
- The last `aborted`/`done` record stays visible until the next `/ota/start`.

## `GET /ota/scan` -> `text/plain`

Starts an async rescan. `200 OK` when started, `409 busy` while an upload is in progress or a
scan is already running, `409 no_sta` when the STA is not configured. Results appear in
`/ota/info.scan` when `in_progress` returns to false.

## Existing endpoints, unchanged

`GET /update` (page), `GET /ota/start?mode=fr|fs&hash=<md5>`, `POST /ota/upload` (multipart),
`GET /ota/cancel` (`409 app_invalid` when `app_valid` is false). `/ota/upload` answers `200 OK` only when the image was verified, otherwise
`400` with the abort reason text; `/ota/cancel` answers `400` while an upload is in progress.

## Serial markers (unchanged names, complete list)

`[SAFEBOOT];wifi;pmf_off;rc;<n>`, `[SAFEBOOT];wifi;retry;reason;no_connect_12s`,
`[SAFEBOOT];wifi;ap_sta;reason;join_timeout_25s` (replaces `fallback_ap`),
`[SAFEBOOT];wifi;event;<connected|disconnected|got_ip>;reason;<n>`,
`[SAFEBOOT];ota;start`, `[SAFEBOOT];ota;abort;reason;<r>`, `[SAFEBOOT];ota;rearm;reason;<r>`,
`[SAFEBOOT];ota;verify;result;ok`, `[SAFEBOOT];ota;end;result;<success|error>`,
`[SAFEBOOT];fallback;reason;<timeout|cancel>`, `[SAFEBOOT];app;image;<valid|invalid>;rc;<n>` (at boot
and after every abort).

## State machine (host-testable, `src/safeboot/ota_state.h`)

Pure C++ (no Arduino), driven by events with an injected clock in ms:
`onStart(gen, total)`, `onChunk(len, now)`, `onFinal(verified_ok, now)`, `onDisconnect(gen, now)`,
`onCancel(now)`, `tick(now)`, `setAppValid(bool)` (false: tick never emits the timeout reboot,
`fallback_in_ms` is -1, cancel is refused). Outputs: current `/ota/state` record, and actions the caller
must perform (`abort(reason)`, `switch_partition`, `reboot_to_app`). Constants: stall 30 000 ms,
fallback 180 000 ms. Signed deltas everywhere (the TM-46 cross-task race).
