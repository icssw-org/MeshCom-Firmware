# Release Notes -- MeshCom Firmware v4.35* (2026-03-21)

Nachrichtenprioritaet, Trickle-HEY, erweiterte Statistik,
APRS-Parser Hardening und diverse Bugfixes auf Basis von `oe1kbc_v4.35p`.

Kein On-Air-Change — alte Firmware empfaengt alle Pakete korrekt.

---

## Upstream-Sync 2026-03-21 (oe1kbc_v4.35p)

Rebase auf aktuellen upstream. Alle lokalen Commits wurden upstream uebernommen
(inkl. MHeard RSSI/SNR Fix via PR #785, mit zusaetzlichen Korrekturen vom Maintainer).

Neue Aenderungen aus upstream:
- BLE connect confirmation nach Nachrichten-Empfang
- BLE connect confirmation allgemein
- T-Deck Fonts und Symbole ueberarbeitet
- Neuer Befehl `cleanflash` / `clearflash`
- GPS HDOP Anpassung
- MHeard: Distanz nur bei Berechnung setzen (Maintainer-Fix zu unserem PR)
- MHeard: HEY-Distanz aus MHeard-Buffer (Maintainer-Fix zu unserem PR)
- Merge PR #785 (unser MHeard RSSI/SNR Fix)

Unsere uebernommenen Commits: alle (Branch ist jetzt identisch mit upstream)

---

## BLE-Ringpuffer Offline-Nachrichten Bugfix (2026-03-19)

Nachrichten wurden nicht im BLE-Ringpuffer gespeichert wenn kein Telefon verbunden war.
Beim spaeteten Verbinden eines Telefons war der Puffer daher leer — keine Offline-Nachrichten.

**Root Cause**: Guard in `addBLEOutBuffer()` (`!g_ble_uart_is_connected && !bWEBSERVER`)
verwarf alle Nachrichten wenn weder BLE-Telefon noch Webserver aktiv war.
E22 zeigte 7 Nachrichten weil dort der Webserver aktiv war (Guard griff nicht).
Heltec V2, RAK (GW und Non-GW) zeigten 0 Nachrichten.

**Fix** (2 Aenderungen in `src/loop_functions.cpp`):
- Guard entfernt — Nachrichten werden immer im Ringpuffer gespeichert
- Overflow-Debug-Log fuer "phone"-Puffer unterdrueckt (Ueberlauf gewuenscht,
  aelteste Eintraege werden ueberschrieben)

**Betroffene Datei**: `src/loop_functions.cpp`

---

## TX-Loop und UDP-Dedup Bugfix (2026-03-19)

Zwei Bugs behoben, die gemeinsam massive Duplikat-Sendungen verursachten.
Gateway oe1kbc zeigte TX/RX-Ratio von 8.5:1 (2573 TX bei 302 RX),
eine einzelne msg_id wurde 179x wiederholt, Queue-Wasserstand erreichte 29/30.

### Bug 1: TX-Loop im Ring-Buffer (KRITISCH)

**Root Cause**: `getNextTxSlot()` pruefte nur `ringBuffer[pos][0] > 0`, nicht den
Status-Byte. Durch Priority-Queue konnte ein Slot gesendet werden, der nicht an
der iRead-Position lag. Nach dem Senden stellte `doTX()` die Slot-Laenge fuer die
Retransmission wieder her (Zeile 1642). Da `advanceIReadPastEmpty()` iRead nicht
vorruecken konnte (fruehere belegte Slots), blieb der wiederhergestellte Slot im
iRead..iWrite-Fenster sichtbar — `getNextTxSlot()` waehlte ihn erneut aus.
Ergebnis: ~20 Sends derselben Nachricht in 40 Sekunden.

**Fix** (3 Aenderungen in `src/lora_functions.cpp`):
- `getNextTxSlot()`: Status-Check — nur READY oder DONE Slots auswaehlen,
  SENT-Slots (Retransmit-Timer laeuft) ueberspringen
- `updateRetransmissionStatus()`: Original-Slot `[0]` auf 0 setzen nach DONE-Markierung
  (sowohl bei Retransmit-Kopie als auch bei Giveup)
- Retransmit-Kopie: Status READY statt SENT, damit `doTX` sie normal aufgreift

### Bug 2: Fehlender Dedup-Ring-Check im UDP-Empfangspfad (MITTEL)

**Root Cause**: `udp_functions.cpp:321` pruefte nur `checkOwnTx()` (30-Entry-Buffer).
Der Dedup-Ring `ringBufferLoraRX` (100 Entries) wurde nicht abgefragt — im Gegensatz
zum LoRa-RX-Pfad, der beide Checks korrekt durchfuehrt. Nach 30 Nachrichten wrappte
`own_msg_id`, und der Gateway akzeptierte dieselbe Nachricht erneut vom Server.

**Fix** (1 Aenderung in `src/udp_functions.cpp`):
- `is_new_packet()` Check vor dem bestehenden `checkOwnTx()` eingefuegt,
  analog zum LoRa-RX-Pfad
- `#include <lora_functions.h>` fuer Deklaration

**Betroffene Dateien**: `src/lora_functions.cpp`, `src/udp_functions.cpp`

---

## Taeglich aktuell mit upstream

Ab sofort wird der Branch taeglich per `/rebase-upstream` mit den neuesten Aenderungen aus
dem upstream  synchronisiert. 

---

## Upstream-Sync 2026-03-18 (oe1kbc_v4.35p, fünfte Runde)

Race Conditions sind nun in Upstream über PR enthalten. Weiterhin Übernahme von Bugfixes und Verbesserungen.

## Race Condition Fixes 2026-03-17

Systematisches Security-Audit hat 10 Race Conditions zwischen ISR- und Main-Loop-Kontext
identifiziert (siehe `docs/audit-buffer-race-2026-03-17.md`). 5 davon hier behoben:

### RACE-01: ISR Display Queue Spinlock
- `pendingDisplayMsg` (aprsMessage mit String-Objekten) wurde aus dem OnRxDone-ISR ohne
  Synchronisation in eine globale Variable kopiert. Main-Loop konnte halb geschriebene
  Struct lesen → Memory Corruption.
- **Fix:** portMUX-Spinlock (ESP32) / taskENTER_CRITICAL (NRF52) um Schreib- und Lese-Seite.
  Display-I2C-Calls bleiben ausserhalb der Critical Section.
- **Dateien:** `lora_functions.cpp`, `esp32_main.cpp`, `nrf52_main.cpp`, `loop_functions_extern.h`

### RACE-07: ISR Flags auf std::atomic
- `receiveFlag`, `bEnableInterruptReceive`, `transmittedFlag`, `bEnableInterruptTransmit`
  waren `volatile bool` — reicht nicht fuer atomare Lese-/Schreiboperationen zwischen ISR
  und Main-Loop auf Dual-Core ESP32.
- **Fix:** `std::atomic<bool>` (lock-free auf Xtensa).
- **Datei:** `esp32_main.cpp`

### RACE-08: transmissionState volatile
- `transmissionState` war plain `int`, wird aber in ISR gesetzt und im Main-Loop gelesen.
  Compiler konnte den Wert cachen.
- **Fix:** `volatile int` + alle 8 extern-Deklarationen angepasst.
- **Dateien:** `esp32_main.cpp`, `lora_functions.cpp`

### RACE-02: Ring-Buffer-Indizes atomar
- `iWrite`/`iRead` (TX Ring Buffer) waren plain `int`, von ISR (OnRxDone → addTxRingEntry)
  und Main-Loop (doTX, advanceIReadPastEmpty) ohne Synchronisation gelesen/geschrieben.
- **Fix:** `std::atomic<int>` mit Local-Copy-Pattern fuer `addRingPointer`-Kompatibilitaet.
- **Dateien:** `loop_functions.cpp`, `loop_functions_extern.h`, `lora_functions.cpp`

### RACE-04: RX Double-Buffer Critical Section (NRF52)
- `rxBufIndex` und `rxBufInUse[]` wurden in OnRxDone ohne Schutz vor re-entrantem ISR
  modifiziert. Nach `Radio.Rx()` konnte ein neuer OnRxDone den aktuellen Buffer ueberschreiben.
- **Fix:** `taskENTER_CRITICAL`/`taskEXIT_CRITICAL` um Buffer-Swap und beide Release-Punkte.
- **Datei:** `lora_functions.cpp`

### RACE-05: CAD State Machine Critical Sections (NRF52)
- 4 CAD-Flags wurden von 3 verschiedenen ISRs (OnCadDone, OnRxDone, OnRxTimeout/OnRxError)
  und dem Main-Loop gleichzeitig gelesen/geschrieben. `volatile` allein reicht nicht fuer
  Multi-Flag-Atomizitaet → Lost-Wakeup-Bugs, inkonsistente Zustandsuebergaenge.
- **Fix:** Snapshot-Pattern im Main-Loop (alle Flags unter Critical Section lesen, dann
  ausserhalb handeln). OnCadDone und alle ISR-CAD-Aborts unter Critical Section.
  `cad_start_time` auf `volatile` geaendert.
- **Dateien:** `nrf52_main.cpp`, `lora_functions.cpp`

### RACE-03: Dedup-Buffer atomar
- `loraWrite` war plain `uint8_t`, wird in `addLoraRxBuffer()` geschrieben und indirekt
  von `is_new_packet()` gelesen (scannt alle Slots per `memcmp`). Torn Writes moeglich.
- **Fix:** `std::atomic<uint8_t>` mit Local-Copy-Pattern — Buffer-Content wird zuerst
  geschrieben, dann Index atomar aktualisiert.
- **Dateien:** `loop_functions.cpp`, `loop_functions_extern.h`

---

## Upstream-Sync 2026-03-17 (oe1kbc_v4.35p, vierte Runde)

Buffer-Overflow-Haertung, Display/UI-Fixes, Audio-Threading, MHeard-Logik-Fixes
und weitere Korrekturen aus dem upstream race-fix Commit vom 17.03.2026 uebernommen.

### command_functions: JSON Buffer-Overflow-Schutz
- `json_len` Typ von `uint16_t` auf `size_t` geaendert.
- An ~15 Stellen (sendGpsJson, sendNodeSetting, sendAnalogSetting, sendAPRSset,
  sendConfigFinish, commandAction fuer Telemetrie/Wetter/IO/Status) wird `json_len`
  nun auf `MAX_MSG_LEN_PHONE - 2` begrenzt bevor `memcpy` in `msg_buffer` erfolgt.
  Verhindert Overflow wenn serialisiertes JSON die BLE-MTU-Grenze ueberschreitet.
- `commandAction()`: `msg_detail` memcpy mit expliziter Laengenpruefung und Null-Terminator.
- `sprintf(node_gwsrv)` auf `snprintf()` gehaertet.
- `sendConfigFinish()`: Fehlerhaften `addBLEOutBuffer`-Aufruf durch `addBLEComToOutBuffer` ersetzt.
- **Betroffene Datei**: `src/command_functions.cpp`

### mheard_functions: Logik-Fixes und Bounds-Haertung
- `strcmp` in `updateMheard()` und `updateHeyPath()` durch `memcmp` mit expliziter
  Laengenbegrenzung ersetzt (verhindert Lesen ueber Puffer-Ende bei kuerzeren Strings).
- `updateHeyPath()`: Neues/altes Path-Format (`R99;` vs. `R99,99,99;`) korrekt erkannt —
  NCount wird nur noch bei neuem Format ohne Komma vor dem Semikolon eingetragen.
- `getMheardCount()`: Zeitfenster von 1 Stunde auf 12 Stunden erweitert (repraesentativerer
  Nachbar-Zaehler fuer Trickle-Algorithmus).
- UTC-Offset-Berechnung: `(60*60+24) * (int)off` durch `(long)(off * 3600.0)` ersetzt
  (war fehlerhafte Integer-Multiplikation, fuehrte zu falschen Zeitanzeigen).
- Persistence: Groessen-Check vor `file.read()` — bei Size-Mismatch wird die Datei
  geloescht statt falschen Speicher zu laden (Schutz nach Struct-Layout-Aenderungen).
- `memcpy` fuer `mheardPathCalls` auf `min(..., sizeof() - 1)` begrenzt.
- **Betroffene Dateien**: `src/mheard_functions.cpp`, `src/lora_functions.cpp`

### aprs_functions: Parser Hardening
- Loops fuer source_path und destination_path auf 120 Zeichen begrenzt (verhindert
  exzessive Iterationen bei malformed Paketen).
- Non-printable Character Checks (`< 0x20 || > 0x7E`) als Abbruchbedingung in
  Source-Path- und Destination-Path-Parsing eingefuegt.
- `msg_source_fw_sub_version`: Bounds-Check `inext < rsize` vor Zugriff ergaenzt.
- Alle `<=` bei `.length()`-Vergleichen in `decodeAPRSPOS()` auf `<` korrigiert
  (Off-by-one, ~20 Stellen, verhindert Out-of-Bounds charAt).
- **Betroffene Datei**: `src/aprs_functions.cpp`

### extudp_functions: NULL-Checks und Return-Path
- `dst`/`msg` JSON-Felder auf NULL geprueft bevor Zugriff.
- Laengenvalidierung: `dst` 1-9 Zeichen, `msg` 1-150 Zeichen.
- `return` nach `resetExternUDP()` ergaenzt — verhindert weiteren Zugriff auf
  geschlossenen UDP-Socket.
- **Betroffene Datei**: `src/extudp_functions.cpp`

### Display / UI: Buffer-Overflow-Fixes
- `t-deck/tdeck_main.cpp`: `assert(buf)` bei LVGL-Buffer-Fehler durch `ESP.restart()`
  mit aussagekraeftigem Log ersetzt.
- `t-deck/lv_obj_functions.cpp`: NULL-Checks nach `new HeaderEventData()` und
  `new DeleteEventData()` ergaenzt (Graceful bail-out statt Null-Dereference).
- `t-deck-pro/tdeck_pro.cpp`: `ps_calloc`-Ergebnisse fuer Display-Buffer geprueft,
  `ESP.restart()` bei Fehlschlag.
- `t-deck-pro/ui_deckpro.cpp`, `t5-epaper/ui.cpp`: `line_full_format()` mit
  explizitem `buf_size`-Check und sicherem `strncpy`/Null-Terminierung neugeschrieben.
- `t5-epaper/ui.cpp`: `scr3_add_img_btn()` Buffer-Fix — `text_len - 4` wurde nicht
  gegen `sizeof(buf)` geprueft.
- **Betroffene Dateien**: `src/t-deck/tdeck_main.cpp`, `src/t-deck/lv_obj_functions.cpp`,
  `src/t-deck-pro/tdeck_pro.cpp`, `src/t-deck-pro/ui_deckpro.cpp`, `src/t5-epaper/ui.cpp`

### esp32_audio: Semaphore-Schutz und Log-Tags
- `play_file_from_sd_blocking()`: `audio.connecttoFS()` jetzt unter `audioSemaphore`-Schutz
  (verhindert Konkurrenz mit Play-Function-Task).
- Log-Tag `[audi]` einheitlich auf `[audio]` korrigiert (alle 14 Vorkommen).
- **Betroffene Datei**: `src/esp32/esp32_audio.cpp`

### esp32_main / nrf52_main: conffin-Timing Fix
- `--conffin`-Kommando wird nicht mehr im BLE-Connect-Callback aufgerufen (falscher
  Kontext), sondern im Main-Loop bei `config_to_phone_prepare`. Verhindert Race zwischen
  BLE-Stack und Kommandoverarbeitung.
- `connect_pending` Flag entfernt.
- **Betroffene Dateien**: `src/esp32/esp32_main.cpp`, `src/nrf52/nrf52_main.cpp`

### loop_functions: printAsciiBuffer und POSINFO-Logs
- `printAsciiBuffer()`: Laengencheck (`len < 4`), Begrenzung auf 255 Bytes,
  Non-printable Chars (`< 0x20 || > 0x7E`) werden als `#` ausgegeben.
- `sendDisplayPosition()`: Off-by-one in `<= .length()` auf `<` korrigiert.
- POSINFO Debug-Logs: `[POSI]` auf `[POSINFO]` mit vorangestelltem Timestamp
  (`getTimeString()`) umgestellt.
- **Betroffene Datei**: `src/loop_functions.cpp`

### Neue Audio-Dateien
- `data/bling.mp3`, `data/flash.mp3`, `data/pop.mp3` aus upstream uebernommen.

---

## Upstream-Sync 2026-03-17 (oe1kbc_v4.35p, dritte Runde)

### GPS-Konsolidierung: esp32_gps → esp32_pmu, GPS_FUNCTIONS entfernt
- `esp32_gps.cpp` in `esp32_pmu.cpp` umbenannt — nur noch `setupPMU()` enthalten.
  Gesamter Legacy-GPS-Code entfernt (~700 Zeilen): u-blox State-Machine (`readGPS()`,
  `getGPS()`, `checkGPS()`), Auto-Baud-Erkennung, `SFE_UBLOX_GNSS`, HardwareSerial/SoftwareSerial.
- `esp32_gps.h` durch minimales `esp32_pmu.h` ersetzt (nur `setupPMU()`-Deklaration).
- `gps_functions.cpp`: `GPS_FUNCTIONS`-Block (~370 Zeilen) geloescht. `TinyGPSPlus gps`
  ist jetzt die globale Instanz (nicht mehr `extern`). Alle Plattformen nutzen denselben Pfad.
- `lora_functions.cpp` und `loop_functions.cpp`: `tinyGPSPLus`/`tinyGPSPlus`-Externs
  durch einheitliches `extern TinyGPSPlus gps` ersetzt.
- `t-deck-pro/peri_gps.cpp` und `tdeck_pro.cpp`: Eigene `TinyGPSPlus`-Instanzen durch
  `extern TinyGPSPlus gps` ersetzt (verhindert Duplicate-Symbol-Fehler).
- `command_functions.cpp`: Tote Variablen `extern int state` / `extern bool bMitHardReset` entfernt.
- `esp32_main.cpp`: Include auf `esp32_pmu.h` aktualisiert, `GPS_FUNCTIONS`-Bloecke in Setup
  und Loop entfernt, Serial-Startup-Delay von 10s auf 5s reduziert.
- Varianten: `//#define GPS_FUNCTIONS` (T-Beam-1W) und `#define GPS_L76K` (T-Deck-Pro) entfernt.
- **Betroffene Dateien**: `src/esp32/esp32_pmu.cpp` (umbenannt), `src/esp32/esp32_pmu.h` (umbenannt),
  `src/gps_functions.cpp`, `src/gps_functions.h`, `src/lora_functions.cpp`, `src/loop_functions.cpp`,
  `src/command_functions.cpp`, `src/esp32/esp32_main.cpp`, `src/t-deck-pro/peri_gps.cpp`,
  `src/t-deck-pro/tdeck_pro.cpp`, `variants/LilyGo_T-Beam-1W/configuration.h`,
  `variants/t_deck_pro/configuration.h`

### mh_hw Masking entfernt
- `mheardLine.mh_hw = aprsmsg.msg_last_hw & 0x7F` entfernte das MSB, das "Last-Sending"
  Information traegt. Downstream-Konsumenten verloren diese Information.
- **Fix**: `& 0x7F` entfernt — voller 8-Bit HW-Wert wird gespeichert.
- **Betroffene Datei**: `src/lora_functions.cpp`

### E-Paper Display-Treiber reduziert
- 8 unbenutzte Display-Treiber-Verzeichnisse geloescht (DEPG0150, DEPG0154, DEPG0213,
  DEPG0290BNS75A, GDE029A1, GDEP015OC1, LCMEN2R13EFC1, QYEG0213RWS800).
- Wireless Paper Includes in `heltec-eink-modules.h` auskommentiert.
- `DISABLE_SDCARD` Define fuer VisionMasterE290 hinzugefuegt (kein SD-Kartenleser auf diesen Boards).
- `BaseDisplay/base.h`: SDWrapper-Include und Klassenmember mit `#ifndef DISABLE_SDCARD` geschuetzt.
- **Betroffene Dateien**: `src/Displays/` (8 Verzeichnisse geloescht), `src/heltec-eink-modules.h`,
  `src/Platforms/VisionMasterE290/VisionMasterE290.h`, `src/Displays/BaseDisplay/base.h`

### T-Deck / T-Deck Plus: sdcard_esp32 Library entfernt
- Die upstream hinzugefuegte `sdcard_esp32` Library (glucee) wurde nicht vom Code referenziert und
  verursachte Build-Fehler (fehlende Framework-Includes fuer FS.h). T-Deck nutzt Standard `<SD.h>`.
- **Fix**: `sdcard_esp32` aus `lib_deps` entfernt, `-<SDWrapper/*>` Build-Filter entfernt (Verzeichnis existiert nicht mehr).
- **Betroffene Dateien**: `variants/t_deck/platformio.ini`, `variants/t_deck_plus/platformio.ini`

---

## Upstream-Sync 2026-03-16 (oe1kbc_v4.35p, zweite Runde)

### SDWrapper entfernt — externe SD-Bibliothek
- Gesamtes `src/SDWrapper/` Verzeichnis (122 Dateien, ~27.000 Zeilen) entfernt.
  Upstream verwendet jetzt die externe Library `sdcard_esp32` (GitHub: glucee/sdcard_esp32).
- T-Deck und T-Deck Plus: Library in `lib_deps` aufgenommen, SDWrapper aus Build-Filter ausgeschlossen.
- **Betroffene Dateien**: `src/SDWrapper/*` (geloescht), `platformio.ini`, `variants/t_deck/platformio.ini`, `variants/t_deck_plus/platformio.ini`

### BaseDisplay: virtual Destruktor
- `~BaseDisplay()` auf `virtual ~BaseDisplay()` geaendert — korrekte C++-Praxis fuer polymorphe Klassen.
- **Betroffene Datei**: `src/Displays/BaseDisplay/base.h`

### APRS: node_atxt direkt verwenden
- `encodeLoRaAPRS()` verwendete eine temporaere `String`-Variable mit Fallback `"(via MeshCom)"`. Upstream nutzt `meshcom_settings.node_atxt` direkt — spart String-Allokation.
- **Betroffene Datei**: `src/aprs_functions.cpp`

### ExtUDP: "none" Payload-Check
- Neue Validierung: JSON-Payload `"none"` wird abgefangen und mit Fehlermeldung abgebrochen.
- **Betroffene Datei**: `src/extudp_functions.cpp`

### GPS_Init() immer aufrufen
- `GPS_Init()` wird jetzt ohne `bGPSON`-Check aufgerufen (wie upstream). Unbenutzte Variable `connect_pending` entfernt.
- **Betroffene Datei**: `src/esp32/esp32_main.cpp`

### serial_monitor.py: Log-Verzeichnis
- Log-Verzeichnis von `/tmp/meshcom_monitor` auf `./meshcom_monitor` geaendert (upstream-konform).
- **Betroffene Datei**: `tools/serial_monitor.py`

---

## strcpy/strcat Buffer-Overflow Hardening (2026-03-16)

Alle unsicheren `strcpy()` und `strcat()` Aufrufe durch groessenbegrenzte Varianten ersetzt.

### SSID-Migration: Buffer-Overflow behoben (ESP32 + NRF52)
- `node_ossid[40]` wurde via `strcpy()` in `node_ssid[33]` kopiert — 7 Byte Overflow in angrenzende Struct-Felder.
- **Fix**: `strncpy()` mit `sizeof(node_ssid) - 1`.
- **Betroffene Dateien**: `src/esp32/esp32_main.cpp`, `src/nrf52/nrf52_main.cpp`

### commandCheck(): unbegrenzter Parameter in 100-Byte Buffer
- `strcpy(vmsg[100], msg)` ohne Laengenpruefung — `msg` kann beliebig lang sein.
- **Fix**: `strncpy()` mit `sizeof(vmsg) - 1`.
- **Betroffene Datei**: `src/command_functions.cpp`

### Display-Text und Serial-Input: Bounds-Checks ergaenzt
- `pageLastTextLong1[25]` und `pageLastTextLong2[200]`: Source konnte laenger als Buffer sein (3 Plattformen: T-Deck Pro, E290, Tracker).
- `line_text[21]`: `strcat()`-Schleife ohne Bounds-Check beim ersten Concat.
- `msg_text[600]`: Serial-Input ohne Laengenpruefung (ESP32 + NRF52).
- **Fix**: `strncpy()`/`strncat()` mit `sizeof()`.
- **Betroffene Dateien**: `src/loop_functions.cpp`, `src/esp32/esp32_main.cpp`, `src/nrf52/nrf52_main.cpp`

---

## Upstream-Sync 2026-03-16 (oe1kbc_v4.35p)

### GPS: Log-Ausgaben nur bei aktiviertem GPS
- Baudrate-Erkennung und "not found"-Meldungen werden nur noch bei `bGPSON == true` ausgegeben. Reduziert Log-Spam wenn GPS deaktiviert ist.
- **Betroffene Datei**: `src/gps_functions.cpp`

### MHeard: ncount-Logik korrigiert
- Bei Nicht-HEY-Paketen (Position, Text) ist kein NCOUNT im Paket enthalten. `updateMheard()` uebernimmt jetzt den bestehenden Tabellenwert `mheardNCount[ipos]` statt den (leeren) Paket-Wert.
- `updateHeyPath()` wird nach `updateMheard()` aufgerufen, damit der ncount beim Path-Update bereits aktuell ist.
- **Betroffene Dateien**: `src/mheard_functions.cpp`, `src/lora_functions.cpp`

### NRF52: sendPosition mit vorheriger Position bei Richtungsaenderung
- Bei `posinfo_shot` (Richtungs-/Distanz-Trigger) wird die Position von `posinfo_prev_lat/lon` gesendet statt der aktuellen GPS-Position — damit wird die Kurve auf der Karte korrekt abgebildet.
- 15s Mindestabstand zwischen Positions-Sendungen (`posinfo_timer_min`) verhindert Spam.
- **Betroffene Datei**: `src/nrf52/nrf52_main.cpp`

---

## Neue Features

### Priority-Queue: 5-stufige Nachrichtenprioritaet
- **Problem**: DMs und ACKs standen hinter 5-10 Relay/HEY-Paketen in der FIFO-Queue.
  Bei 50-80% CAD-Busy-Rate fuehrte das zu 10-50s unnoetige Verzoegerung fuer menschliche Nachrichten.
- **Loesung**: 5 Prioritaetsstufen mit differenziertem CSMA-Backoff:
  - Prio 1 (Kritisch): ACK + persoenliche DM — CSMA-Base 3000ms + 0-350ms Jitter
  - Prio 2 (Hoch): Gruppen + Broadcast "*" — CSMA-Base 3000ms + 0-350ms Jitter
  - Prio 3 (Normal): Mesh-Relay — CSMA-Base 4500ms + 0-350ms Jitter
  - Prio 4 (Niedrig): Position — CSMA-Base 5500ms + 0-350ms Jitter
  - Prio 5 (Hintergrund): HEY — CSMA-Base 5500ms + 0-350ms Jitter
  - Retry-Reduktion: 2. Versuch -17%, 3. Versuch -33% auf Base-Wert
- **Prio-Erkennung**: `getMessagePriority()` erkennt Typ via msg_type Byte,
  Relay via `RING_STATUS_DONE`, DM vs Gruppe via `CheckGroup()`.
- **Prio-Entnahme**: `getNextTxSlot()` scannt den Ring-Buffer nach hoechster Prioritaet.
  Bei gleicher Prio: FIFO-Reihenfolge (aeltester zuerst).
- **Prio-Drop**: Bei voller Queue wird der aelteste Eintrag der niedrigsten Prioritaet
  verworfen. ACKs und DMs werden nie zugunsten niedrigerer Pakete verworfen.
- **RAM-Aufwand**: 30+120 Bytes (`ringPriority[MAX_RING]` + `ringEnqueueTime[MAX_RING]`)
- **Betroffene Dateien**: `src/configuration_global.h`, `src/loop_functions_extern.h`,
  `src/loop_functions.cpp`, `src/lora_functions.h`, `src/lora_functions.cpp`,
  `src/esp32/esp32_main.cpp`

### Trickle-HEY: Adaptive HEY-Frequenz (ADR-001 Vorschlag C, RFC 6206)
- **Problem**: Bei 100 Nodes mit HEY alle 15 Min: ~7 HEY/Min = ein HEY alle 8.5s.
  Signifikanter Overhead bei 50-80% CAD-Busy.
- **Loesung**: Trickle-Algorithmus (RFC 6206 adaptiert) passt HEY-Intervall dynamisch an.
  Erwartete Einsparung: 60-70% weniger HEY-Traffic in stabilen Netzen.
- **Parameter**: Imin=30s, Imax=15min (wie bisher), k=2 (Redundanzschwelle)
- **Verhalten**:
  - Intervall verdoppelt sich bei Stabilitaet (30s -> 1m -> 2m -> 4m -> 8m -> 15m)
  - Reset auf 30s bei Topologieaenderung (neuer/verlorener Nachbar via getMheardCount())
  - Suppression: HEY wird unterdrueckt wenn >=2 konsistente Nachbar-HEYs gehoert
- **Betroffene Dateien**: `src/configuration_global.h`, `src/loop_functions_extern.h`,
  `src/loop_functions.cpp`, `src/lora_functions.cpp`,
  `src/esp32/esp32_main.cpp`, `src/nrf52/nrf52_main.cpp`

### Erweiterte Statistik und Logging
- **[MC-STAT]** alle 5 Minuten: TX-Zaehler pro Prioritaet, Drops pro Prio, Preemption-Zaehler
- **[MC-PRIO]** alle 5 Minuten: Latenz avg/max pro Prioritaetsstufe (Queue-to-TX)
- **[MC-HWM]** alle 30 Minuten: Queue/CSMA High-Water-Marks, aktuelles Trickle-Intervall
- **serial_monitor.py**: Neue Alerts (PRIO1_STARVED >10s), Trickle-Summary in Intervall-Report
- **loganalyse.sh**: Neue Sektionen PRIORITY_DISTRIBUTION, TRICKLE_HEY, HIGH_WATER_MARKS
- **Betroffene Dateien**: `src/esp32/esp32_main.cpp`, `tools/serial_monitor.py`, `tools/loganalyse.sh`

---

## Bugfixes

### RAK4630: Silent Freeze nach ~14h Betrieb behoben (2026-03-17)

**Race Condition in CSMA/CA State Machine (P1)**
- Wenn `OnRxDone` waehrend einer laufenden CAD-Operation feuert, ruft der Callback
  `Radio.Rx()` auf, was die CAD abbricht. Die Flags `cad_in_progress`/`cad_done_flag`
  werden aber nicht zurueckgesetzt — die State Machine wartet auf ein `OnCadDone` das
  nie kommt. Ohne Watchdog friert der Node permanent ein (~14h MTBF).
- **Fix**: CAD-State-Flags in OnRxDone, OnRxTimeout und OnRxError zuruecksetzen.
  `cad_in_progress` und `cad_double_check` als `volatile` deklariert.

**SPI-Bus-Schutz zwischen Main Loop und Radio Task (P2)**
- Die SX126x-Arduino Library hat keinen Mutex fuer SPI-Zugriffe. Main Loop
  (`Radio.Standby/StartCad/Rx`) und Background Task (`RadioBgIrqProcess`)
  koennen gleichzeitig auf den SPI-Bus zugreifen — SPI-Korruption moeglich.
- **Fix**: `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()` um alle Radio.*-Aufrufe
  im Main Loop und `Radio.Send()` in doTX.

**Radio-Watchdog bei leerer TX-Queue (P3)**
- Wenn die TX-Queue leer ist und keine RX-Aktivitaet vorliegt, wird
  `iReceiveTimeOutTime` nicht gesetzt. Ohne Timer-Zyklus erkennt der
  Main Loop nicht, wenn der SX1262 keine Interrupts mehr liefert —
  unbegrenzte Funkstille moeglich (~46s beobachtet auf OE1KBC-12).
- **Fix**: `else`-Branch analog zu ESP32 (`esp32_main.cpp:2203-2207`)
  eingefuegt. Periodischer `Radio.Rx()`-Restart ueber normalen
  CSMA-Timeout-Zyklus (~5s).

- **Betroffene Dateien**: `src/nrf52/nrf52_main.cpp`, `src/lora_functions.cpp`,
  `src/loop_functions_extern.h`

### APRS-Parser Hardening

**Source/Destination-Path-Schleifen gegen korrupte Pakete abgesichert**
- Ein korruptes RF-Paket ohne `>` Trennzeichen liess die Source-Path-Schleife in `decodeAPRS()` den gesamten Restbuffer durchlaufen. Fuer jedes Byte wurde `String::concat()` aufgerufen — bei ~250 Bytes Muell fuehrte das zu 488ms Verarbeitungszeit (normal: 1-9ms) und Heap-Fragmentierung.
- Crash-Ursache fuer OE1KBC-12 und OE3MAG-12 am 15.03.2026.
- **Fix**: Beide Path-Parsing-Schleifen begrenzt auf max 120 Bytes. Non-Printable-Bytes (< 0x20 oder > 0x7E) brechen die Schleife sofort ab.

**Buffer-Overread bei FW-Sub-Version behoben**
- Nach dem Parsen von FW-Version und Hardware-Byte wurde `RcvBuffer[inext]` ohne Bounds-Check gelesen.
- **Fix**: `inext < rsize` Guard eingefuegt, analog zu den bestehenden Guards.

**Off-by-One in APRS-Telemetrie-Parsing**
- 15 Parsing-Schleifen in `decodeAPRSPOS()` und der Display-Funktion verwendeten `id <= PayloadBuffer.length()`, was Out-of-Bounds-Zugriff erlaubte.
- **Fix**: `id <= X.length()` durch `id < X.length()` ersetzt in allen 15 Schleifen.

- **Betroffene Dateien**: `src/aprs_functions.cpp`, `src/loop_functions.cpp`

### Heltec V2: GPS-Pin-Konflikt mit UART0

**GPS-Pins auf GPIO 12/13 umgelegt**
- GPIO 3 (GPS_TX_PIN = UART0 RX) belegte den USB-Serial-Empfang. Auch GPIO 23 (GPS_RX_PIN) war auf manchen Revisionen anderweitig belegt.
- **Fix**: GPS-Pins auf freie GPIOs umgelegt: `GPS_RX_PIN 13`, `GPS_TX_PIN 12`. UART0 (GPIO 3) bleibt frei fuer serielle Kommandos.
- **Betroffene Datei**: `variants/heltec_wifi_lora_32_V2/configuration.h`

**GPS-Init blockierte serielle Kommandoeingabe** (revertiert)
- Dieser Fix wurde rueckgaengig gemacht: upstream ruft `GPS_Init()` jetzt wieder bedingungslos auf. Siehe Upstream-Sync oben.

### T-Deck / T-Deck Plus: Boot-Hang durch ungueltige Current-Limit

- `CURRENT_LIMIT 240` ueberschritt den gueltigen Bereich (0-140 mA). Nach dem Uncomment des `setCurrentLimit()`-Aufrufs loeste der ueberhoethe Wert `RADIOLIB_ERR_INVALID_CURRENT_LIMIT` aus — Endlosschleife beim Boot.
- **Fix**: `CURRENT_LIMIT` von 240 auf 140 mA korrigiert (Maximum des SX1262/SX1268 OCP-Bereichs).
- **Betroffene Dateien**: `variants/t_deck/configuration.h`, `variants/t_deck_plus/configuration.h`

### MHeard-Datenkorruption

**memcpy Bounds-Check und strcmp→memcmp**
- `memcpy` in `updateMheard()` kopierte ohne Bounds-Check in den 10-Byte `mheardCalls`-Slot. Bei Callsigns >= 10 Zeichen wurde der Nachbar-Eintrag korrumpiert. `strcmp()` verglich ohne Laengenbegrenzung.
- Bug-Report Rainer: Gespeicherte MHeard-Eintraege zeigten falsche HW-Typen und gemixte Callsigns.
- **Fix**: `memcpy` mit `min(length, sizeof-1)` begrenzt. `strcmp` durch `memcmp` mit expliziter max-length ersetzt.

**ncount-Quelle, Comma-Parsing und Zeitfenster**
- `mheardNCount[ipos]` nutzte den stale Array-Wert statt des aktuellen Paket-Werts. Comma-Check blockierte gueltige Path-Payloads. `getMheardCount()` zaehlte nur Nodes der letzten Stunde statt 12h.
- **Fix**: Paket-Wert statt Array-Wert. Comma-Check entfernt. Zeitfenster auf 12h erweitert.

**Persistence Size-Check gegen Datenkorruption**
- Nach der MAX_MHEARD-Aenderung (20→120) hatten alte `mheard.dat`-Dateien eine andere Groesse. `loadMHeardPersistence()` las ohne Groessencheck — die gesamte MHeard-Tabelle wurde korrumpiert.
- **Fix**: File-Size-Check vor dem Laden. Bei Mismatch wird die alte Datei geloescht.

- **Betroffene Datei**: `src/mheard_functions.cpp`

### UTC-Offset-Rechenfehler in mheard Path-Anzeige

- Die Zeitberechnung verwendete `(60 * 60 + 24)` = 3624 statt `3600` Sekunden pro Stunde. `node_utcoff` (float) wurde auf `(int)` gecastet, was die Halbstunden-Praezision verlor.
- **Fix**: Formel an `(long)(meshcom_settings.node_utcoff * 3600.0)` angeglichen.
- **Betroffene Dateien**: `src/mheard_functions.cpp`, `src/web_functions/web_functions.cpp`

### Race Conditions: volatile durch std::atomic ersetzt

- Channel-Utilization-Counter und ISR-Flags waren als `volatile` deklariert. Auf dem nRF52 ist das `+=` Read-Modify-Write-Pattern nicht atomar — Lost Updates moeglich.
- **Fix**: `volatile` durch `std::atomic` ersetzt. `+=` durch `fetch_add()` + `exchange()` atomar ausgefuehrt.
- **Betroffene Dateien**: `src/loop_functions_extern.h`, `src/loop_functions.cpp`, `src/lora_functions.cpp`, `src/esp32/esp32_main.cpp`, `src/nrf52/nrf52_main.cpp`

### ExtUDP Null-Pointer-Schutz und Socket-Reset

- `inputJson["dst"]` gibt `nullptr` zurueck wenn das JSON-Feld fehlt — undefiniertes Verhalten bei Zuweisung an `String`. Nach `resetExternUDP()` fiel die Ausfuehrung durch zu `endPacket()` auf dem geschlossenen Socket.
- **Fix**: Null-Checks und Laengenvalidierung. `return` nach `resetExternUDP()`.
- **Betroffene Datei**: `src/extudp_functions.cpp`

### CONFFIN nach Config-Commands im gleichen Buffer senden

- CONFFIN wurde ueber den regulaeren toPhone-Buffer gesendet, waehrend Config-JSONs ueber den priorisierten ComToPhone-Buffer gingen. CONFFIN kam daher verzoegert oder gar nicht an.
- **Fix**: `sendConfigFinish()` nutzt jetzt `addBLEComToOutBuffer()`. Reihenfolge garantiert: Config-JSONs → MHeard → CONFFIN.
- **Betroffene Dateien**: `src/nrf52/nrf52_ble.cpp`, `src/nrf52/nrf52_main.cpp`, `src/esp32/esp32_main.cpp`, `src/command_functions.cpp`

### pow(2,n) durch Bit-Shift ersetzt

- ADC-Maximalwert mit `pow(2, resolution) - 1` berechnet. `pow()` gibt `double` zurueck — implizite Konversion zu `int` kann Rundungsfehler verursachen.
- **Fix**: `(1 << resolution) - 1`.
- **Betroffene Datei**: `src/batt_functions.cpp`

### printAsciiBuffer: Bounds-Check bei kurzen Buffern

- Die Debug-Funktion griff auf `buffer[0..3]` zu ohne vorher `len >= 4` zu pruefen.
- **Fix**: `if(len < 4) return;` als erstes Statement.
- **Betroffene Datei**: `src/loop_functions.cpp`

---

## Code-Alignment mit Upstream

Folgende Aenderungen wurden an `upstream/oe1kbc_v4.35p` angeglichen, um keine unnoetige Differenz zu erzeugen:

- **APRS IGate-Text**: Upstream verwendet `node_atxt` direkt im komprimierten LoRa-APRS Paket — unseren Kommentar-Block entfernt.
- **sendPosition Magic-Values**: `unsigned long` und `0xEEEE`/`0x9999` beibehalten wie Upstream.
- **GPS_REFRESH_INTERVAL**: Zurueck auf 5s (Upstream-konform).
- **sendMheard MESH/NCNT**: Kurts Weiterentwicklung uebernommen — MESH- und NCNT-Felder im BLE-JSON.

**Betroffene Dateien**: `src/aprs_functions.cpp`, `src/loop_functions.h`, `src/loop_functions.cpp`, `src/command_functions.cpp`, `src/mheard_functions.cpp`, `src/configuration_global.h`

---

## Kleinere Verbesserungen

- **POSINFO-Logs**: Tag von `[POSI]` auf `[POSINFO]` vereinheitlicht, alle 7 `printf`-Stellen um `getTimeString()` erweitert. (`src/loop_functions.cpp`)
- **Audio Log-Tag**: `[audi]` → `[audio]` in allen 16 Log-Ausgaben. (`src/esp32/esp32_audio.cpp`)

---

## Basis: MeshCom Firmware v4.35n (2026-03-11)

Diese Version baut auf v4.35n auf. Die vollstaendigen Release Notes der Basis-Version
(CSMA/CA, WiFi/BLE/UDP-Stabilitaet, Retransmit-Fixes, Refactoring, Tools)
sind unter dem Tag `v4.35n_20260311` dokumentiert.

---

## Supported Hardware

E22-DevKitC.bin (433 MHz)
E22_XML-DevKitC.bin (433 MHz)
E22_1268_S3-DevKitC.bin (433 MHz)
E22_1262-DevKitC.bin (868 MHz)
E22_1262_S3-DevKitC.bin (868 MHz)
heltec_wifi_lora_32_V2.bin
heltec_wifi_lora_32_V3.bin
heltec_wireless_stick_v3.bin
heltec_wireless_tracker.bin
ttgo-lora32-v21.bin
ttgo_tbeam.bin
ttgo_tbeam_SX1262.bin
ttgo_tbeam_SX1268.bin
ttgo_tbeam_supreme_l76k.bin
vision-master-e290.bin
wiscore_rak4631.bin
t_deck.bin
t_deck_plus.bin

Please use webflasher https://esptool.oevsv.at for upgrade from 4.30q:
t_deck.bin
t_deck_plus.bin

Newer version able to upgrade via OTA-Flasher.

[MeshCom Changelog](https://icssw.org/meshcom-versionen/)

[MeshCom@ICSSW Projektseite](https://icssw.org/meshcom/)
