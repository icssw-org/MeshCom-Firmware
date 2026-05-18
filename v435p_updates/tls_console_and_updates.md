## Zusammenfassung / Summary

Ersatz der deaktivierten TLS-Console durch eine RAM-effiziente HMAC-SHA256-Console sowie vier unabhängige Bugfixes.

Replaces the disabled TLS console with a RAM-efficient HMAC-SHA256 console, and fixes four independent bugs.

---

## 1. `src/net_console.cpp` — HMAC-Console ersetzt TLS-Console / HMAC Console replaces TLS Console

**Problem (DE):** Die TLS-Console benötigte mbedTLS SSL-Setup, X.509-Zertifikate und NVS-Schlüsselspeicherung (~36 KB RAM pro aktiver Session). Auf RAM-knappen Boards führte dies zu OOM-Fehlern.

**Problem (EN):** The TLS console required mbedTLS SSL setup, X.509 certificates and NVS key storage (~36 KB RAM per active session). On RAM-constrained boards this caused OOM crashes.

**Änderung / Change:** TLS wird durch ein HMAC-SHA256 Challenge-Response-Protokoll über Raw TCP ersetzt. `mbedtls/md.h` ist bereits im ESP-IDF enthalten — keine zusätzliche Bibliothek notwendig. Protokoll: Node sendet 16-Byte-NONCE → Client antwortet mit HMAC-SHA256(Passwort, NONCE) → `OK`/`FAIL`. RAM-Verbrauch aktive Session: ~0 KB. Die öffentliche API (`net_console.h`) ist zur bisherigen `tls_console.h` kompatibel.

TLS is replaced by an HMAC-SHA256 challenge-response protocol over raw TCP. `mbedtls/md.h` is already part of ESP-IDF — no additional library required. Protocol: node sends a 16-byte NONCE → client responds with HMAC-SHA256(password, NONCE) → `OK`/`FAIL`. RAM per active session: ~0 KB. Public API (`net_console.h`) is compatible with the previous `tls_console.h`.

Neue Client-Tools / New client tools: `tools/hmac_connect.py` (Linux/macOS/Windows, keine Abhängigkeiten / no dependencies) und / and `tools/hmac_connect.ps1` (Windows PowerShell 5.1+).

---

## 2. `src/lora_functions.cpp` — `startTransmit()` Fehlerbehandlung / Error Recovery

**Problem (DE):** Wenn `radio.startTransmit()` im Track→MeshCom- oder APRS-TX-Pfad fehlschlug, wurde `doTX()` trotzdem mit `return true` beendet. Kein TX-done-IRQ folgte → State-Machine blockierte für `TX_WATCHDOG_MS` (15 Sekunden).

**Problem (EN):** When `radio.startTransmit()` failed in the Track→MeshCom or APRS TX path, `doTX()` still returned `true`. No TX-done IRQ followed → state machine stalled for `TX_WATCHDOG_MS` (15 seconds).

**Änderung / Change:** Bei `startTransmit()`-Fehler: `tx_is_active` zurücksetzen, Ring-Slot wiederherstellen, `iRead` zurückrollen, `false` zurückgeben → sofortiger Retry ohne 15s-Wartezeit. / On failure: reset `tx_is_active`, restore the ring slot, roll back `iRead`, return `false` → immediate retry without the 15 s stall.

---

## 3. `src/lora_functions.cpp` — `updateRetransmissionStatus()`: Retransmit-Slot `len=0`

**Problem (DE):** Bei einem DM-Retry wurde `ringBuffer[ircheck][0] = 0` **vor** `memcpy(ringBuffer[iWrite], ...)` gesetzt → `len=0` in der Kopie → leere Payload beim Empfänger. Im Log sichtbar als `RING_WRITE ... len=0 src=retransmit`.

**Problem (EN):** During a DM retry `ringBuffer[ircheck][0] = 0` was set **before** `memcpy(ringBuffer[iWrite], ...)` → `len=0` in the copy → empty payload at the receiver. Visible in the log as `RING_WRITE ... len=0 src=retransmit`.

**Änderung / Change:** `memcpy` wird vor dem Löschen des Original-Slots ausgeführt. / `memcpy` is now executed before the original slot is cleared.

---

## 4. `src/loop_functions.cpp` — `SendAckMessage()`: APRS-ACK Priorität prio=3 statt prio=1

**Problem (DE):** `ringBuffer[iWrite][1] = 0xFF` wurde **vor** `addTxRingEntry()` gesetzt. `getMessagePriority()` interpretiert `RING_STATUS_DONE (0xFF)` als Relay-Nachricht → `MSG_PRIO_NORMAL` (prio=3) statt `MSG_PRIO_CRITICAL` (prio=1). Im Log sichtbar als `RING_TX_READ ... prio=3`.

**Problem (EN):** `ringBuffer[iWrite][1] = 0xFF` was set **before** `addTxRingEntry()`. `getMessagePriority()` interprets `RING_STATUS_DONE (0xFF)` as a relay message → `MSG_PRIO_NORMAL` (prio=3) instead of `MSG_PRIO_CRITICAL` (prio=1). Visible in the log as `RING_TX_READ ... prio=3`.

**Änderung / Change:** Status=`0x00` beim Einreihen, `addTxRingEntry()` aufrufen (Priorität korrekt als prio=1 bewertet), danach `status=0xFF` auf dem gesicherten Slot-Index setzen. / Status is set to `0x00` when enqueuing, `addTxRingEntry()` is called (priority correctly evaluated as prio=1), then `status=0xFF` is applied to the saved slot index.

---

## 5. `src/extudp_functions.cpp` — `resetExternUDP()`: UDP-Socket nach Fehler nie neu gestartet

**Problem (DE):** Nach einem UDP-TX-Fehler: `hasExternIPaddress = false` → sofort `if(bEXTUDP && hasExternIPaddress && ...)` — **immer false**. `startExternUDP()` wurde nie aufgerufen. Symptom: UDP-Empfang dauerhaft tot; Reboot nötig.

**Problem (EN):** After a UDP TX error: `hasExternIPaddress = false` → immediately `if(bEXTUDP && hasExternIPaddress && ...)` — **always false**. `startExternUDP()` was never called. Symptom: UDP reception permanently dead; reboot required.

**Änderung / Change:** `&& hasExternIPaddress` aus der Bedingung entfernt. / `&& hasExternIPaddress` removed from the condition.

---

## Getestete Boards / Tested Boards

- `ttgo_tbeam` ✓
- `heltec_wifi_lora_32_V3` ✓

## Ziel-Branch / Target Branch

`DEV` — `icssw-org/MeshCom-Firmware`
