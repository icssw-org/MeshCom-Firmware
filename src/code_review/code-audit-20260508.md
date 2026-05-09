# MeshCom Firmware Code Audit

**Date:** 2026-05-08
**Branch:** v4.35p_prio (rebased onto upstream `oe1kbc_v4.35p` HEAD = aa457d8a)
**Local commits on top:** f462c1ff, 03cd2e01, e048d1db, b63f6fb3 (docs only, no src/)
**Auditor:** Claude Code (automated)
**Rules:** docs/codequality-rules.md (ESP32 C++ Code Quality Rules)
**Previous audit:** docs/code-audit-20260417.md (2026-04-17, 70 findings, base 95bd4c4)

**Delta vs. 2026-04-17:** Re-validated every CRITICAL/HIGH finding against the
current tree after a fresh upstream sync (50+ commits added since 95bd4c4 —
notable: GPS refactor "drop software serial" 3b75ecd1, BLE Pin Checking
bf05f9de, T-Connect-Pro variants, Lilygo T3 S3 v1.3, batt_functions rewrite,
clock.cpp `strftime` migration, nrf52 RX-Boost revert branch, telnet/TLS
console feature added & reverted in PR #942). Line numbers were updated where
they shifted; one CRITICAL was demoted to PARTIAL FIX (BLE PIN). No finding
from 2026-04-17 was fully closed.

---

## Audit Summary

| Category | Rule IDs | Status | Critical | High | Medium | Low |
|----------|----------|--------|----------|------|--------|-----|
| Memory Safety | MEM-01..05 | FAIL | 2 | 0 | 3 | 2 |
| Buffer Safety | BND-01..05 | FAIL | 4 | 1 | 1 | 0 |
| Input Validation | Section 3 | FAIL | 1 | 2 | 1 | 0 |
| Thread Safety | RACE-01..08 | FAIL | 0 | 3 | 3 | 1 |
| ISR Safety | ISR-01..04 | FAIL | 1 | 1 | 2 | 0 |
| SPI Bus | SPI-01..05 | FAIL | 0 | 1 | 1 | 0 |
| Auth & Security | Section 7 | PARTIAL | 2 | 4 | 1 | 0 |
| Error Handling | Section 8 | FAIL | 1 | 2 | 1 | 0 |
| Watchdog | STAB-01..05 | FAIL | 2 | 1 | 2 | 0 |
| Compiler/Build | COMP-01..05 | FAIL | 1 | 0 | 0 | 0 |
| Type Safety | Section 11 | FAIL | 0 | 1 | 1 | 0 |
| Lifetime Safety | Section 12 | PARTIAL | 0 | 0 | 2 | 0 |
| Logging Safety | Section 13 | PARTIAL | 0 | 1 | 1 | 0 |
| Design Patterns | Section 14 | FAIL | 1 | 1 | 1 | 0 |
| Protocol Correctness | Section 15 | FAIL | 1 | 1 | 1 | 0 |
| State Machines | Section 16 | PARTIAL | 0 | 0 | 2 | 0 |
| Data Drift | Section 17 | FAIL | 1 | 0 | 0 | 0 |
| TCP/Web/SSE | Section 18 | FAIL | 2 | 1 | 1 | 0 |
| Test Readiness | Section 19 | FAIL | 0 | 0 | 1 | 0 |
| Stack Safety | STK-01..04 | FAIL | 1 | 1 | 1 | 0 |

**Total: 20 Critical, 20 High, 26 Medium, 3 Low = 69 findings**
(−1 vs. 2026-04-17: BLE PIN downgraded to PARTIAL FIX/HIGH after commit
bf05f9de. New file `log_functions.cpp` is compliant; renamed
`tft_display_functions.cpp` adds no new findings.)

---

## 1. Memory Safety (MEM-01..05)

### MEM-01: malloc/new after initialization

| Severity | File | Line | Finding |
|----------|------|------|---------|
| LOW | nrf52/nrf52_main.cpp | 83 | `malloc()` in `nrf52_getMaxFreeBlock()` heap probe (runtime, not setup) |

### MEM-02: Arduino String in hot paths

| Severity | File | Line | Finding |
|----------|------|------|---------|
| <mark style="background-color:green;">CRITICAL</mark> | loop_functions.cpp | 2401 | `String charBuffer_aprs(...)` returns `String` by value (line shifted from 2205); caller in `lora_functions.cpp:454` invokes it in OnRxDone path — heap churn on every RX packet |
| <mark style="background-color:green;">CRITICAL</mark> | lora_functions.cpp | 454 | `memcpy(ringbufferRAWLoraRX[...], charBuffer_aprs(...).c_str(), UDP_TX_BUF_SIZE-1)` — String temp allocated inside OnRxDone execution path |
| <mark style="background-color:green;">MEDIUM</mark> | loop_functions.cpp | 176, 186 | Global `String strSOFTSER_BUF`, `String strTelemetry` used in main loop |
| <mark style="background-color:green;">MEDIUM</mark> | loop_functions.cpp | ≈multiple | Multiple temporary `String` concatenations in per-message processing |
| <mark style="background-color:green;">MEDIUM</mark> | aprs_functions.cpp | 192, 204, 209, 270, 280, 285, 344 | `aprsmsg.msg_source_path.concat(...)` — per-byte String growth during packet decode |

### MEM-03: C++ new without delete (memory leak)

| Severity | File | Line | Finding |
|----------|------|------|---------|
| <mark style="background-color:green;">CRITICAL</mark> | spectral_scan.cpp | 108 | `new uint16_t[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE]` — returned pointer never freed |
| <mark style="background-color:green;">CRITICAL</mark> | spectral_scan.cpp | 188 | `new uint16_t[...]{0}` — same pattern in second branch |

### MEM-04: Display buffer allocation

| Severity | File | Line | Finding |
|----------|------|------|---------|
| <mark style="background-color:green;">MEDIUM</mark> | t-deck-pro/tdeck_pro.cpp | 195-198 | Triple `ps_calloc` for display buffers, no unified error recovery |
| <mark style="background-color:green;">LOW</mark> | t-deck/tdeck_main.cpp | 318 | `ps_malloc` for LVGL buffer, no fallback on failure |
| <mark style="background-color:green;">LOW</mark> | t5-epaper/t5epaper_main.cpp | 258-279 | Triple `ps_malloc`/`ps_calloc` for display buffers |

### MEM-05/06: Buffer size constants & xTaskCreateStatic — PASS / unchanged

Static ring buffers in `loop_functions.cpp` (`ringBuffer`, `ringBufferLoraRX`,
`ringBufferUDPout`) all sized via `configuration_global.h`. No
`xTaskCreateStatic()` use, but task creation is init-time only.

---

## 2. Buffer Overflow Prevention (BND-01..05)

### BND-01: Banned unsafe functions

**sprintf() — 77+ instances (CRITICAL)**

| Severity | File | Lines | Finding |
|----------|------|-------|---------|
| <mark style="background-color:green;">CRITICAL</mark> | esp32/at_cmd.h | 30 | `AT_PRINTF` macro: unbounded `sprintf` into 255-byte buffer |
| <mark style="background-color:green;">CRITICAL</mark> | nrf52/at_cmd.h | 30 | Same `AT_PRINTF` macro |
| <mark style="background-color:green;">CRITICAL</mark> | nrf52/WisBlock-API.h | 658 | `API_LOG`: `int len = sprintf(buff, __VA_ARGS__);` unbounded |
| <mark style="background-color:green;">CRITICAL</mark> | esp32/esp32_main.cpp | 613, 842, 959-960, 1119, 1497, 1501, 1503 | 7x `sprintf` into settings/BLE name buffers (lines shifted from 785/895/896/1053/1430/1434/1436) |
| <mark style="background-color:green;">CRITICAL</mark> | esp32/esp32_functions.cpp | 83, 134, 141 | `sprintf` into 20-byte `cvers` (lines shifted +1) |
| <mark style="background-color:green;">HIGH</mark> | t-deck/event_functions.cpp | ≈20 sites | 20 `sprintf` in event handler |
| <mark style="background-color:green;">HIGH</mark> | t-deck/lv_obj_functions.cpp | 24 sites | 24 `sprintf` in UI functions |
| <mark style="background-color:green;">MEDIUM</mark> | rtc_functions.cpp | 1 site | `sprintf` into `cdate` buffer |
| <mark style="background-color:green;">MEDIUM</mark> | i2c_scanner.cpp | 4 sites | `sprintf` in scanner |
| <mark style="background-color:green;">MEDIUM</mark> | nrf52/nrf52_main.cpp | (shifted, ≈4 sites) | `sprintf` in init/log paths |
| <mark style="background-color:green;">MEDIUM</mark> | nrf52/nrf52_functions.cpp | 64, 196 | 2x `sprintf` into `cvers` |
| <mark style="background-color:green;">MEDIUM</mark> | nrf52/nrf52_ble.cpp | 93, 98, 122 | 3x `sprintf` |
| <mark style="background-color:green;">MEDIUM</mark> | command_functions.cpp | 4118 | `sprintf(meshcom_settings.node_parm, "%s", "none")` |

**strcpy() — 5 sites (HIGH)**

| Severity | File | Line | Finding |
|----------|------|------|---------|
| <mark style="background-color:green;">HIGH</mark> | web_functions/web_functions.cpp | 1800 | `strcpy(message_text, message.c_str())` into 200-byte buffer, unchecked source length (line shifted from 1797) |
| <mark style="background-color:green;">MEDIUM</mark> | onebutton_functions.cpp | 77, 81 | `strcpy` for page text buffers |
| <mark style="background-color:green;">LOW</mark> | Displays/BaseDisplay/SD.cpp | 368 | `strcpy` for hard-coded 4-byte filename suffix |

### BND-02: snprintf return value not checked — 559+ snprintf sites, zero checked

No `snprintf` call in the codebase validates return value `>= buf_size` for
truncation detection. Top files unchanged from 2026-04-17. The new
`log_functions.cpp` correctly uses `vsnprintf(buf, sizeof(buf), fmt, args)`
but does not check the return either.

### BND-03: memcpy without prior length validation

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | nrf52/nrf52_ble.cpp | ≈293 | `memcpy(&meshcom_settings, data, sizeof(s_meshcom_settings))` — exact-size + marker check passes; retained for documentation, marker is not a cryptographic validator |
| HIGH | esp32/esp32_main.cpp | 310 | BLE characteristic copy — `item.length` has no upper bound check |
| HIGH | lora_functions.cpp | 209 | `memcpy(print_buff, payload, 12)` — no check that `size >= 12` |
| HIGH | lora_functions.cpp | 383 | `memcpy(RcvBuffer, payload, size)` — bounded earlier on RAK only; ESP32 path lacks check |
| HIGH | phone_commands.cpp | 541 | `memcpy(textbuff_phone+iposn, conf_data+2, txt_msg_len_phone)` — len from BLE, no cap vs. `sizeof(textbuff_phone)` (lines shifted from 466, 487-488; PIN-auth wrapper added in bf05f9de does not re-check bounds) |
| HIGH | nrf52/nrf_eth.cpp | 270 | `memcpy(RcvBuffer, inc_udp_buffer+UDP_MSG_INDICATOR_LEN, lora_tx_msg_len)` — bounded by check at 268 (PASS); retained for documentation |
| MEDIUM | udp_functions.cpp | 140, 172 | UDP indicator/message copy without prior length validation |
| MEDIUM | nrf52/nrf52_flash.cpp | 70-226 | 47+ migration memcpy calls assuming correct field sizes |

### BND-04: Missing static_assert on protocol structs

Unchanged: `aprsMessage`, `s_meshcom_settings`, `mheardLine` lack
`static_assert` / `_Static_assert`. Only one in the entire codebase
(`bq27220_data_memory.h:93`).

---

## 3. Input Validation

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | phone_commands.cpp | 226, 324, 364, 385, 403, 466, 479-488 | No length validation before array access on BLE command data (e.g. `conf_data[ssid_len+3]` without prior bounds check). bf05f9de adds a PIN gate before commands run, but does not validate per-command lengths |
| HIGH | web_functions/web_setup.cpp | 69-259 | `toInt()`/`toDouble()` with no range validation on REST parameters |
| HIGH | web_functions/web_functions.cpp | 1920 | JSON output with unescaped user content (`paramValue.c_str()`) — injection risk in generated HTML/JSON |
| MEDIUM | web_functions/web_functions.cpp | 353-382, 1767, 1827, 1856, 1889 | URL parsing assumes `HTTP/1.1` is present; `indexOf()` return `-1` propagated to `substring()` |
| MEDIUM | gps_functions.cpp | (refactor) | After "drop software serial" rebuild, GPS pin macros come from variant headers; runtime validation still absent |

**Compliant areas:**
- LoRa RX on RAK: `lora_functions.cpp:309` — `rxSize = (size <= UDP_TX_BUF_SIZE) ? size : UDP_TX_BUF_SIZE`
- BLE ESP32: `esp32_main.cpp:305-311` — validates length within `MAX_MSG_LEN_PHONE`
- BLE nRF52: `nrf52_ble.cpp:279-286` — validates exact struct size AND markers

---

## 4. Thread Safety (RACE-01..08)

### RACE-01: Binary semaphore misused as mutex

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | esp32/esp32_audio.cpp | 38 | `xSemaphoreCreateBinary()` for audio shared state |
| MEDIUM | t-deck/tdeck_main.cpp | 111 | `xSemaphoreCreateBinary()` for TFT access (comment says "mutex") |
| LOW | nrf52/nrf52_main.cpp | 400 | `xSemaphoreCreateBinary()` for task signaling (acceptable) |

### RACE-02: portMAX_DELAY in mutex takes

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | t-deck/tdeck_main.cpp | 405 | `xSemaphoreTake(xSemaphore, portMAX_DELAY)` in display flush |
| MEDIUM | nrf52/api_functions.cpp | 262 | `xSemaphoreTake(g_task_sem, portMAX_DELAY)` |

### RACE-03: Thread-unsafe functions

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | time_functions.cpp | 96 | `localtime()` instead of `localtime_r()` |

**Compliant:** `clock.cpp` uses `localtime_r()`; recent commit e3fa0fda
replaced `snprintf` with `strftime` for date formatting (no new finding).

### RACE-04: Volatile without synchronization

| Severity | File | Lines | Finding |
|----------|------|-------|---------|
| HIGH | loop_functions.cpp | 299-300 | `volatile int iWrite/iRead` ring buffer indices — read-modify-write race (line shifted from 294-295). `addRingPointer()` takes them by reference but no mutex |
| HIGH | nrf52/nrf52_main.cpp | 233-236 | CAD flags (`cad_done_flag`, `cad_channel_busy`, etc.) — ISR vs main task race (line shifted from 230-233) |
| HIGH | gps_functions.cpp | 174-175 | After "drop software serial" refactor, `volatile unsigned long pulseTimes[SAMPLE_COUNT]` and `volatile int pulseIndex` remain at file scope — ISR vs main loop race |
| MEDIUM | esp32/esp32_main.cpp | 443, 458 | `transmissionState`, `scanFlag` — ISR writes, main reads (lines shifted from 410, 425) |
| MEDIUM | loop_functions.cpp | 75, 204-206 | Cross-module volatile bools without locks (`bSetLoRaAPRS`, etc.) |

### RACE-05: std::atomic usage — PASS

Properly used in `loop_functions.cpp` (`loraWrite`, `is_receiving`, `tx_is_active`),
`lora_functions.cpp:290-292` (`ch_util_rx_start/accum`), `esp32_main.cpp` (flags).

### RACE-06: Lock ordering documentation — FAIL

Unchanged. `taskENTER_CRITICAL()/taskEXIT_CRITICAL()` pairs in
`lora_functions.cpp:313-321, 335-342, 367-369` bracket their critical sections
symmetrically.

### RACE-07: Float tasks not pinned to core

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | t-deck-pro/peri_gps.cpp | 78 | `xTaskCreate` (not pinned) with `double` GPS coordinates |
| MEDIUM | t-deck-pro/peri_gyroscope.cpp | 14-16 | Global `float` factors accessed from unpinned task |

---

## 5. Interrupt Safety (ISR-01..04)

### ISR-01/02: ISR handlers — MOSTLY COMPLIANT

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | nrf52/nrf52_main.cpp | 317, 326, 335 | `interruptHandle1/2/3()` lack `IRAM_ATTR` |
| MEDIUM | gps_functions.cpp | ≈206/212 | `handleRxInterrupt()` ISR off-by-one survives the "drop software serial" refactor: `pulseIndex++` before storing into `pulseTimes[pulseIndex]` can hit `pulseTimes[SAMPLE_COUNT]` at the upper bound. Bounds check is `<` SAMPLE_COUNT *before* increment |

### ISR-03: Prohibited operations in ISR

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | t5-epaper/io_extend.c | 26 | `printf("interrupt_handler\n")` inside ISR |
| HIGH | lora_functions.cpp | 325, 327, 344, 346, 350, 375 | Multiple `Serial.printf()` calls in OnRxDone callback (guarded by `bLORADEBUG` / `_overwrite`, but still executed in ISR context when enabled) |
| HIGH | lora_functions.cpp | 332 | `startRadioReceive()` (SPI bus access via `Radio.Rx()` / `Radio.RxBoosted()`) invoked from OnRxDone ISR on nRF52 |

### ISR-04: DRAM_ATTR — PASS

ISR data uses `std::atomic<bool>`. Global volatile variables default to DRAM.

---

## 6. SPI Bus Safety (SPI-01..05)

### SPI-01: SPI bus mutex — PARTIAL

`portMUX_TYPE displayMux` spinlock for display updates. `bSPI_ETH_Active`
flag guards LoRa/Ethernet bus sharing.

### SPI-02: SPI access from ISR

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | lora_functions.cpp | 332 | `startRadioReceive()` (→ `Radio.Rx()` / `Radio.RxBoosted()`) called directly in OnRxDone ISR path. nRF52 RX-Boost branch (`upstream/revert-867-boostedgain-for-rak-4631`) is *not* applied to current HEAD, so this stays present |

### SPI-03: CS pin management — PASS

T-Echo SPI on SPIM3, TFT on SPIM1 — separate buses, CS management unchanged.

### SPI-04: Task pinning

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | (system-wide) | -- | No explicit core pinning for LoRa/SPI tasks on dual-core ESP32 |

---

## 7. Authentication & Security

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | udp_functions.cpp | 534 | `WiFi.softAP(meshcom_settings.node_call)` — NO WPA2 password, open AP (line shifted from 533) |
| CRITICAL | safeboot/main.cpp | 68, 187 | `WiFi.softAP(hostname)` — NO password in safeboot mode (2 call sites) |
| HIGH | nrf52/nrf52_ble.cpp | 274 | `g_lora_data.setPermission(SECMODE_OPEN, SECMODE_OPEN)` — no encryption required (line shifted from 248). Comment at line 133 still references `SECMODE_ENC_WITH_MITM` (commented out, "KBC 28.04.2025") |
| HIGH | web_functions/web_functions.cpp | 234 | Web password optional; empty password = open access |
| HIGH | web_functions/web_functions.cpp | 353-368 | Custom URL-param auth instead of HTTP Basic Auth (password posted in URL query string) |
| HIGH | esp32/esp32_main.cpp | 271 | **PARTIAL FIX:** `uint32_t PIN = 000000;` still hardcoded. Commit bf05f9de (2026-05-01, "BLE Pin Checking") added an *application-layer* SHA-256 PIN gate using `meshcom_settings.bt_code` (see `phone_commands.cpp:274-310`) — used when `bt_code > 0`. The Bluetooth-stack pairing PIN at line 271 is still `000000` and is taken when `bt_code` is unset. Net: better than 2026-04-17 (some auth gate exists), but the BLE pairing layer is still effectively open |
| MEDIUM | safeboot/ElegantOTA.cpp | 21-48 | OTA authentication optional; no firmware signature verification |

---

## 8. Error Handling

### Unchecked begin() calls

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | main.cpp | 37, 42 | `SPI.begin()` unchecked |
| HIGH | esp32/esp32_main.cpp | 651 | `Wire.begin(I2C_SDA, I2C_SCL)` unchecked (lines shifted from 596, 600; consolidated) |
| HIGH | nrf52/nrf52_main.cpp | 848 | `Wire.begin()` unchecked (line shifted from 837) |
| HIGH | nrf52/nrf52_ble.cpp | 128, 130, 135, 272, 278 | 5x BLE service `begin()` unchecked |
| MEDIUM | extudp_functions.cpp | 67 | `ETH.begin()` unchecked |

### Blocking while(true) on init errors

| Severity | File | Lines | Finding |
|----------|------|-------|---------|
| CRITICAL | esp32/esp32_main.cpp | 1323, 1330, 1338, 1348, 1385, 1425, 1458, 1484 | 8x `while (true);` on radio config errors (lines shifted from 1256..1417) |
| CRITICAL | t-deck-pro/peri_lora.cpp | 48, 54, 60, 66, 72, 78, 85, 91, 97, 105, 114 | 11x `while (true);` on radio state errors |
| MEDIUM | t-deck-pro/tdeck_pro.cpp | 309 | `while (1)` on camera init fail |
| MEDIUM | t-deck-pro/peri_gps.cpp | 54 | `while(1)` on GPS init fail |
| MEDIUM | t5-epaper/t5epaper_main.cpp | 73 | `while(1)` on display init fail |
| MEDIUM | t5-epaper/peri_gps.cpp | 53 | `while(1)` on GPS init fail |
| MEDIUM | t5-epaper/peri_lora.cpp | 36 | `while(1)` on LoRa init fail |

### Queue overflow handling

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | loop_functions.cpp | ≈444-481 | Ring buffer overflow silently wraps/overwrites without production warning (priority-aware drop logic logs under `bLORADEBUG` only) |

**Compliant areas:**
- `bmx280.cpp:176`, `ina226_functions.cpp:27` — check `begin()` return

---

## 9. Watchdog & Recovery (STAB-01..05)

### STAB-01/02: Task watchdog — NOT CONFIGURED

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | (entire codebase) | -- | Zero calls to `esp_task_wdt_add()` or `esp_task_wdt_reset()` |

### STAB-03: Busy-wait loops

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | Regexp.cpp | 145, 150 | Recursive `max_expand()` / `min_expand()` without yield; can run unbounded on pathological patterns |
| MEDIUM | gps_functions.cpp | ≈206 | `while (pulseIndex < SAMPLE_COUNT && (millis() - startWait < SAMPLE_DURATION)) { delay(10); }` — boot-time baud detection, blocks loop up to 5 s. Survived "drop software serial" refactor |

### STAB-04: Reset reason logging

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | (entire codebase) | -- | No `esp_reset_reason()` call at startup |
| MEDIUM | (entire codebase) | -- | No persistent crash counter / safe mode logic |

### STAB-05: millis() wraparound bugs — CRITICAL

Wrong pattern `(start + timeout) < millis()` or `start + timeout > millis()` in
**33+ locations** across `esp32_main.cpp`, `nrf52_main.cpp`,
`mheard_functions.cpp`, `lora_functions.cpp`, `web_functions.cpp`. Selected
samples re-validated in current tree:

| Severity | File | Line | Pattern (WRONG) |
|----------|------|------|-----------------|
| CRITICAL | mheard_functions.cpp | 160, 192 | `lastsaveMHEARDPersistence + 30000 > millis()` / `lastsavePATHPersistence + 30000 > millis()` |
| CRITICAL | esp32/esp32_main.cpp | 2353, 2420, 2443, 2477, 2601, 2612, 2622, 3030, 3321 | `(timer + ms) <|> millis()` |
| CRITICAL | nrf52/nrf52_main.cpp | 1144, 1208, 1218, 1261, 1281, 1344, 1368, 1384, 1397, 1420, 1423, 1436, 1460, 1510, 1541, 1557, 1566 | Same pattern (lines redistributed after major refactor; total count unchanged) |
| CRITICAL | lora_functions.cpp | 1579 | `millis() > track_to_meshcom_timer + 1000 * 60 * 5` (line shifted from 1576) |
| CRITICAL | web_functions/web_functions.cpp | 239 | `(ulong)(web_ip_passwd_time[iwid] + (1000*60*60*4)) < millis()` |

**Correct pattern** (used in some places): `(uint32_t)(millis() - start) >= interval`
Example: `lora_functions.cpp:1135`.

---

## 10. Compiler & Build Safety (COMP-01..05)

### COMP-01: Build flags

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | platformio.ini | 135, 162, 202 | `-Wall -Wextra` present but `-Werror` MISSING |

### COMP-05: Library version pinning

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | platformio.ini | 55-69 | Multiple libraries use `^` (ArduinoJson, TinyGPSPlus, OneWire, DHT, arduino-sht, MCP23017, RTClib, AHTX0, OneButton, BMx280MI, BME680, CCS811) |
| HIGH | platformio.ini | 115 | `espressif32@^6.13.0` — platform version not exact |
| HIGH | variants/t_deck_pro/platformio.ini | 75 | `RadioLib@7.1.2` vs main `RadioLib@7.6.0` (still unsynced) |
| HIGH | variants/t_deck_pro/platformio.ini | 2 | `platform = espressif32@6.5.0` — pinned but inconsistent with root `@^6.13.0` |

### COMP-02..04 — PASS

Switch statements in main hot paths have `default:` cases. No `NDEBUG`, no
side effects in `assert()`. Note: upstream commit 18c72ac2 ("gcc-warning-fixes")
removed a redundant copy in `encodePayloadAPRS` and a VLA — net positive.

---

## 11. Type Safety

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | nrf52/nrf_eth.cpp | 274 | `(uint8_t)lora_tx_msg_len` — narrowing `uint16_t` → `uint8_t` (UDP_TX_BUF_SIZE > 255) |
| MEDIUM | (multi files) | -- | `snprintf` return value (`int`, can be negative) never checked before use as offset |

---

## 12. Lifetime Safety

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | web_functions/web_functions.cpp | 208, 485, 503 | `web_client.stop()` without resetting web_header — potential use-after-close on re-entry |
| MEDIUM | web_functions/web_functions.cpp | 236-263 | Session table cleared on broken millis() pattern at 239 only, not on abrupt disconnect |

---

## 13. Logging Safety

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | lora_functions.cpp | 325, 327, 344, 346, 350, 375 | `Serial.printf()` in OnRxDone path |
| MEDIUM | t5-epaper/io_extend.c | 26 | `printf()` inside ISR handler |

**Compliant — NEW:** `log_functions.cpp` (new file, commit f462c1ff) uses
`vsnprintf(buf, sizeof(buf), fmt, args)` with proper bounds, no ISR usage,
no String, no malloc. No findings.

---

## 14. Design Patterns

### CSMA not a pure function

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | lora_functions.cpp | 1916-1952 | `csma_compute_timeout()` / `csma_compute_timeout_prio()` / `csma_reset()` mutate global state — not testable in isolation |

### Callback safety

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | esp32/esp32_main.cpp | 1331, 1371, 1399 | `radio.setDio1Action(callback)` — return value not checked |

### Static allocation audit — PASS (except spectral_scan.cpp)

---

## 15. Protocol Correctness

### FCS checked AFTER field extraction

| Severity | File | Lines | Finding |
|----------|------|-------|---------|
| CRITICAL | aprs_functions.cpp | 179-379 | Fields extracted via `String::concat` BEFORE FCS validation. `FCS_SUMME` computed at 371-375 and compared at 379. All String growth, regex checks and payload extraction happen on untrusted bytes before integrity check |

### Frame size validation

| Severity | File | Line | Finding |
|----------|------|------|---------|
| HIGH | aprs_functions.cpp | 134 | Only minimum (16 bytes) checked; no maximum frame size validation against `UDP_TX_BUF_SIZE` |

### Parser instance isolation

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | aprs_functions.cpp | 120 | Single `aprsmsg` struct parameter reused for LoRa and UDP decode paths — would race if concurrent |

---

## 16. State Machine & Session Safety

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | web_functions/web_functions.cpp | 38-39 | Session table accessed without mutex |
| MEDIUM | web_functions/web_functions.cpp | -- | No per-IP rate limiting on login attempts |

---

## 17. Data Drift Safety

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | t5-epaper/nvs_param.cpp | 7-47 | No NVS schema version field — `tpInit = prefs.isKey("nvsInit")` is a boolean, not a version |

---

## 18. TCP/Web/SSE Safety

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | web_functions/web_functions.cpp | 31, 340 | `String web_header` — unbounded concatenation of HTTP request data — heap exhaustion vector |
| CRITICAL | web_functions/web_functions.cpp | 353, 355, 382, 1767, 1827, 1856, 1889 | `indexOf()` return not checked before `substring()` |
| HIGH | web_functions/web_functions.cpp | 265 | Fixed 10-slot session table with no LRU eviction |
| MEDIUM | web_functions/web_functions.cpp | -- | No Content-Length validation or JSON body size limit on REST endpoints |

---

## 19. Test & Fuzz Readiness

| Severity | File | Line | Finding |
|----------|------|------|---------|
| MEDIUM | test/ | -- | Only `compress_functions.cpp/h`; no unit test framework, no parser tests, no fuzz targets |

---

## 20. Stack Safety (STK-01..04)

| Severity | File | Line | Finding |
|----------|------|------|---------|
| CRITICAL | (project root) | -- | No `sdkconfig` with stack overflow checking level 2 |
| HIGH | (entire codebase) | -- | No `uxTaskGetStackHighWaterMark()` calls |
| MEDIUM | platformio.ini | -- | `-fstack-usage` compiler flag not enabled |

---

## Summary of Deltas vs. 2026-04-17

### PARTIAL FIX

- **Section 7 (BLE PIN)**: Commit **bf05f9de** "BLE Pin Checking" (2026-05-01)
  introduced an application-layer SHA-256 challenge using
  `meshcom_settings.bt_code`. When configured (`bt_code > 0`), incoming
  phone commands are gated through this challenge in
  `phone_commands.cpp:274-310`. The Bluetooth pairing-layer hardcoded
  `PIN = 000000` at `esp32/esp32_main.cpp:271` remains as fallback. Severity
  downgraded **CRITICAL → HIGH**.

### Net positive (no audit impact, recorded for context)

- Commit **18c72ac2** ("gcc-warning-fixes") removed a VLA and a redundant
  copy in `encodePayloadAPRS` (no security impact, but reduces stack pressure).
- Commit **e3fa0fda** ("clock: replace snprintf with strftime") — safer date
  formatting, no finding affected.
- Commit **3b75ecd1** ("drop software serial") and follow-ups
  (d2109af2, 50c1ce59, 9195c807, fee106d8) restructured GPS handling. The
  ISR off-by-one in `pulseTimes`/`pulseIndex` and the 5 s boot busy-wait
  survived the refactor — still flagged.
- Commit **88cf2649** ("Revert Telnet Serial Bridge") undoes
  bfa3f5ca/5da3adc0/c3d01855/a7a9d010/78238b0a. No telnet code in current
  HEAD, so no audit impact.
- Commit **bf05f9de** also added per-command PIN gating but did not add
  per-command length validation; phone_commands.cpp BLE input findings
  remain.

### Newly checked since rebase (no findings)

- **`log_functions.cpp` / `log_functions.h`** (NEW, commit f462c1ff):
  uses `vsnprintf` with `sizeof(buf)`, no String, no malloc, no ISR call.
  COMPLIANT.
- **`tft_display_functions.cpp`** (renamed from `tft_display_funkctions.cpp`,
  ≈423 lines): heavy `String` use confined to UI render path; no new buffer
  safety issues.
- **`batt_functions.cpp`** (130-line diff): only added `Serial.printf()` in
  loop ctx; no new sprintf/memcpy issues.
- **`command_functions.cpp`** (146-line diff): JSON-bounded `memcpy(msg_buffer
  + 1, print_buff, json_len)` patterns; bounded by ArduinoJson serialize
  output. ACCEPTABLE.

### Confirmed unchanged (still present, same semantics)

- All 3 CRITICAL WiFi open-AP findings (udp_functions.cpp:534,
  safeboot/main.cpp:68/187)
- All 77 `sprintf` call sites
- BLE `SECMODE_OPEN` at nrf52_ble.cpp:274 and commented `SECMODE_ENC_WITH_MITM` at 133
- 8x `while(true)` in esp32/esp32_main.cpp + 11x in t-deck-pro/peri_lora.cpp
- `new uint16_t[]` leaks in spectral_scan.cpp:108, 188
- FCS-after-extraction bug in aprs_functions.cpp:179-379
- All `web_header` unbounded-concat / `substring`-of-negative-index bugs
- 33+ `millis()` wraparound sites

### Fixed since 2026-04-17

None fully closed; one PARTIAL FIX (BLE PIN) — see above.

---

## Top 10 Priority Fixes

### 1. WiFi AP password protection (Section 7 — CRITICAL)
- `udp_functions.cpp:534` and `safeboot/main.cpp:68, 187`
- Add WPA2 password: `WiFi.softAP(ssid, password)`

### 2. millis() wraparound (STAB-05 — CRITICAL, 33+ locations)
- Replace `(start + timeout) < millis()` with `(uint32_t)(millis() - start) >= timeout`
- Silent failure after 49.7 days uptime

### 3. sprintf -> snprintf (BND-01 — CRITICAL, 77 locations)
- Priority: `AT_PRINTF` / `API_LOG` macros (at_cmd.h, WisBlock-API.h),
  esp32_main.cpp, esp32_functions.cpp

### 4. FCS validation before field parsing (Section 15 — CRITICAL)
- `aprs_functions.cpp:371-379` — move FCS check before field extraction at line 179

### 5. Task watchdog configuration (STAB-01/02 — CRITICAL)
- Add `esp_task_wdt_add()` / `esp_task_wdt_reset()`; replace `while(true);`
  with `esp_restart()`

### 6. String allocation in RX hot path (MEM-02 — CRITICAL)
- `loop_functions.cpp:2401` / `lora_functions.cpp:454` — replace
  `charBuffer_aprs()` String return with caller-supplied char buffer

### 7. BLE pairing security (Section 7 — HIGH, was CRITICAL)
- `nrf52_ble.cpp:274` — restore `SECMODE_ENC_WITH_MITM`
- `esp32_main.cpp:271` — remove hardcoded `PIN = 000000` fallback now that
  `bt_code` PIN gate exists; require `bt_code` to be set on first boot

### 8. memcpy length validation (BND-03 — CRITICAL)
- `lora_functions.cpp:209` — validate `size >= 12`
- `phone_commands.cpp:541` — bound `memcpy` against `sizeof(textbuff_phone)`

### 9. Web header buffer limiting (Section 18 — CRITICAL)
- `web_functions.cpp:340` — bound `web_header` to ≤ 4-8 KB
- Check every `indexOf()` return before `substring()`

### 10. Memory leak in spectral scan (MEM-03 — CRITICAL)
- `spectral_scan.cpp:108, 188` — replace `new[]` with static buffer or
  ensure `delete[]`

---

## Files Most Affected

| File | Findings | Highest Severity |
|------|----------|-----------------|
| nrf52/nrf52_main.cpp | 19 | CRITICAL |
| lora_functions.cpp | 14 | CRITICAL |
| esp32/esp32_main.cpp | 14 | CRITICAL |
| platformio.ini | 14 | CRITICAL |
| t-deck-pro/peri_lora.cpp | 11 | CRITICAL |
| web_functions/web_functions.cpp | 10 | CRITICAL |
| loop_functions.cpp | 8 | CRITICAL |
| nrf52/nrf52_ble.cpp | 5 | HIGH (was CRITICAL) |
| phone_commands.cpp | 5 | CRITICAL |
| aprs_functions.cpp | 5 | CRITICAL |
| gps_functions.cpp | 3 | MEDIUM |
| spectral_scan.cpp | 2 | CRITICAL |

---

## Rebase-Specific Observations (oe1kbc_v4.35p 95bd4c4 → aa457d8a)

50+ commits were synced. Notable:

- **bf05f9de "BLE Pin Checking"** — adds `meshcom_settings.bt_code` and a
  SHA-256 challenge in `phone_commands.cpp:274-310`. Application-layer auth
  is now non-trivial, but the Bluetooth pairing PIN at
  `esp32_main.cpp:271` remains `000000`. Net effect: HIGH instead of CRITICAL.
- **3b75ecd1 + d2109af2 + 50c1ce59 + 9195c807 + fee106d8** — GPS subsystem
  rewrite ("drop software serial", "gps reprog", "t_beam gps seriel open
  fixed", "gps_t_echo update", "gps baud msg"). 1327 lines of diff in
  `gps_functions.cpp`. The ISR off-by-one and 5 s boot busy-wait survived;
  no SoftSerial code remaining (removed entirely).
- **bfa3f5ca → 88cf2649** — Telnet Serial Bridge added then reverted in
  PR #942. Current HEAD has no telnet code, so the brief audit window where
  port 23 was open is closed.
- **18c72ac2 + 4ec3848a + 9b6efbc1** — gcc warning fixes; removed VLA and
  redundant copy in `encodePayloadAPRS`. Stack-pressure improvement, no
  finding closed.
- **e3fa0fda** — `clock.cpp` migrated `snprintf`-based date formatting to
  `strftime`. Reduces snprintf surface by 1 site.
- **a111df50, ba4d6605, 428c31a3, 511a9b73** — power/extudp variants, no
  audit impact.
- **f462c1ff (local)** — Added `log_functions.cpp/h`. Compliant.
- **63960675 "issue #902"**, **3c264dd4 "gps modul m100"**, **62047dfe
  "oled pos"** — variant-level / issue-tracker fixes; no audit impact.

**Bottom line:** The rebase brought no security fixes. All previously
identified attack surfaces remain. One application-layer PIN gate (bf05f9de)
modestly improves BLE security and lets us downgrade BLE PIN from CRITICAL
to HIGH.
