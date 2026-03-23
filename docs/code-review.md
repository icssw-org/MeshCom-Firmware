# MeshCom Firmware — Stability Code Review

**Datum:** 2026-03-18
**Branch:** v4.35p_prio
**Scope:** Buffer Overflow, Type Safety, Race Conditions, RTOS Best Practices
**Plattformen:** ESP32 (Heltec V3, T-Beam, T-Deck), nRF52840 (RAK4631)
**Hinweis:** PR #781 (RACE-01..05) ist bereits gemerged. Dieser Report listet nur **offene** Findings.

---

## Bereits behoben (PR #781 / RACE-01..05)

Die folgenden Punkte wurden durch [PR #781](https://github.com/icssw-org/MeshCom-Firmware/pull/781) adressiert und sind **nicht mehr offen**:

| Fix | Beschreibung | Begründung |
|-----|-------------|------------|
| ~~RC-01~~ | Ring-Buffer-Indizes `iWrite`/`iRead` | Bewusst `volatile int` statt `std::atomic<int>` — auf 32-bit Xtensa/ARM sind aligned int-Zugriffe hardware-atomar. `std::atomic` brach den Build (PR #779: deleted copy-constructor in variadic functions + `addRingPointer` Signatur-Inkompatibilität). |
| ~~RC-02~~ | Priority-Queue Index-Snapshot | Lokale Kopien via `int w = iWrite` sind auf diesen Plattformen sicher (single-instruction load). |
| ~~RC-03~~ | addLoraRxBuffer Write-before-Index | `loraWrite` korrekt als `std::atomic<uint8_t>` mit write-before-index Pattern. |
| ~~VOL-01~~ | CAD-State-Flags | RACE-05: Alle CAD-Flag-Zugriffe unter `taskENTER_CRITICAL`. `volatile` reicht wenn Critical Section schützt. |
| ~~ISR-01~~ | Display-Queue ISR Race | RACE-01: `portMUX_TYPE displayMux` Spinlock (ESP32) / `taskENTER_CRITICAL` (nRF52). |
| ~~RACE-04~~ | RX Double-Buffer-Swap | `taskENTER_CRITICAL` um Buffer-Swap in `OnRxDone()` auf nRF52. |

---

## Qualitätskriterien (Shortcut-Referenz)

| Kürzel | Qualitätsmerkmal | Beschreibung |
|--------|-----------------|--------------|
| **BOF** | Buffer Overflow | Unbegrenzte memcpy/sprintf/strcpy, fehlende Bounds-Checks bei Array-Zugriffen |
| **TS** | Type Safety | C-Style Casts, implizite Narrowing-Conversions, void*-Missbrauch |
| **MEM** | Memory Management | Heap-Fragmentierung durch String-Allokation, fehlende Freigabe, Stack Overflow |
| **RTOS** | FreeRTOS Misuse | Semaphore statt Mutex, portMAX_DELAY Deadlock, fehlende Yields |
| **DMA** | DMA/Peripheral | Buffer-Alignment, SPI-Bus-Contention, Radio-State-Machine |

---

## Offene Findings

### KRITISCH (Sofortiger Handlungsbedarf)

---

#### BOF-01: Unbounded memcpy in Ring-Buffer-Schreiboperationen

**Datei:** `src/loop_functions.cpp`
**Zeilen:** 2438, 2814, 2938, 3010, 3085, 3158, 3425

```cpp
memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);
```

**Problem:** `aprsmsg.msg_len` wird nicht gegen `UDP_TX_BUF_SIZE` geprüft. Ein fehlerhaftes oder manipuliertes Paket mit überhöhtem `msg_len` überschreibt den 255+5 Byte großen Buffer.

**Empfehlung:**
```cpp
if (aprsmsg.msg_len > UDP_TX_BUF_SIZE) {
    aprsmsg.msg_len = UDP_TX_BUF_SIZE;  // oder: return / drop packet
}
memcpy(ringBuffer[iWrite]+2, msg_buffer, aprsmsg.msg_len);
```

---

#### BOF-02: Unbounded Payload-Parsing via String::concat()

**Datei:** `src/aprs_functions.cpp`
**Zeilen:** 335-345

```cpp
for(ib=inext; ib < rsize; ib++)
{
    if(RcvBuffer[ib] == 0x00) break;
    aprsmsg.msg_payload.concat((char)RcvBuffer[ib]);  // Keine Maximalgröße
}
```

**Problem:** Arduino `String` wächst unbegrenzt. Ein manipuliertes Paket mit großem `rsize` und ohne Null-Terminator verursacht Heap-Exhaustion oder Fragmentierung.

**Empfehlung:** Maximum-Length-Check einbauen:
```cpp
int max_payload = UDP_TX_BUF_SIZE - HEADER_SIZE;
for(ib=inext; ib < rsize && (ib - inext) < max_payload; ib++) { ... }
```

---

#### BOF-03: sprintf in festen Buffer ohne Größenprüfung

**Datei:** `src/esp32/esp32_functions.cpp`
**Zeilen:** 81, 132, 139

```cpp
char cvers[20];
sprintf(cvers, "%s/%-1.1s <%s>", SOURCE_VERSION, SOURCE_VERSION_SUB,
        getCountry(meshcom_settings.node_country).c_str());
```

**Problem:** `cvers` ist nur 20 Bytes. Die kombinierte Länge von Version + Country-String kann 20 Bytes überschreiten.

**Empfehlung:** `snprintf(cvers, sizeof(cvers), ...)` verwenden.

---

#### BOF-04: Unbounded memcpy in Command Processing

**Datei:** `src/command_functions.cpp`
**Zeilen:** 136-137

```cpp
memset(msg_detail, 0x00, sizeof(msg_detail));
memcpy(msg_detail, msg_text+inext, ipos-inext);
```

**Problem:** `ipos-inext` wird nicht gegen `sizeof(msg_detail)` geprüft. Kann Buffer überschreiben, wenn der Input-String unerwartet lang ist.

**Empfehlung:**
```cpp
size_t copy_len = min((size_t)(ipos - inext), sizeof(msg_detail) - 1);
memcpy(msg_detail, msg_text + inext, copy_len);
```

---

### HOCH (Sollte zeitnah behoben werden)

---

#### MEM-01: Große Stack-Arrays in verschachtelten Funktionen

**Dateien und Größen:**

| Datei | Zeile | Array | Größe |
|-------|-------|-------|-------|
| `src/lora_functions.cpp` | 728, 787 | `tempRcvBuffer[255]` | 255 B |
| `src/loop_functions.cpp` | 1677 | `words[100][21]` | 2100 B |
| `src/extudp_functions.cpp` | 231-232 | `c_json[500]` + `c_tjson[500]` | 1000 B |
| `src/loop_functions.cpp` | 610 | `pageTextLong2[200]` | 200 B |

**Problem:** Bei typischen FreeRTOS Task-Stack-Größen von 4-8 KB können verschachtelte Aufrufe mit großen lokalen Arrays zum Stack Overflow führen.

**Empfehlung:**
- Große Arrays als `static` deklarieren (wenn Thread-Safety gewährleistet) oder
- In globale/statische Buffers auslagern
- Task-Stack-Größen mit `uxTaskGetStackHighWaterMark()` validieren

---

#### DMA-01: SPI-Bus-Contention bei geteiltem Bus

**Kontext:** SX1262 LoRa-Modul teilt sich den SPI-Bus mit Display (Heltec V3) oder Flash.

**Problem:** Wenn mehrere Tasks (LoRa-Task, Display-Task) gleichzeitig auf den SPI-Bus zugreifen, können Daten korrumpiert werden. Kein expliziter SPI-Bus-Mutex im Code sichtbar.

**Empfehlung:** `spi_device_acquire_bus()` / `spi_device_release_bus()` oder expliziter Mutex um alle SPI-Zugriffe.

---

### MITTEL (Verbesserungspotential)

---

#### MEM-02: String-Allokation in Message-Processing-Loops

**Datei:** `src/loop_functions.cpp`
**Zeilen:** 1538-1569

```cpp
String strMsgT5 = getTimeString() + " " + "<" + aprsmsg.msg_source_call + ">" + ...;
String strPath = "M * <" + aprsmsg.msg_source_call + ">";
```

**Problem:** Jede String-Verkettung allokiert neuen Heap-Speicher. In Message-Processing-Loops (pro empfangene Nachricht) führt dies über Stunden/Tage zu Heap-Fragmentierung.

**Empfehlung:** Feste `char`-Buffer mit `snprintf()` oder String-Objekte per `reserve()` vorallokieren.

---

#### RTOS-01: portMAX_DELAY Deadlock-Risiko

**Datei:** `src/t-deck/tdeck_main.cpp`
**Zeile:** 405

```cpp
xSemaphoreTake(semaphore, portMAX_DELAY);
```

**Problem:** Unendliches Warten ohne Timeout. Falls das Semaphore nie gegeben wird, hängt der Task permanent.

**Empfehlung:** Endlichen Timeout verwenden:
```cpp
if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(5000)) != pdTRUE) {
    // Error handling / recovery
}
```

---

#### BOF-05: Heartbeat-Parsing unvollständig

**Dateien:** `src/udp_functions.cpp:373`, `src/nrf52/nrf_eth.cpp:591`

```cpp
// TODO check HB accordingly to format not only BEAT at beginning
```

**Problem:** Heartbeat-Validation prüft nur den Anfang des Pakets. Malformed Heartbeats könnten akzeptiert werden.

---

#### MEM-03: Fehlende Freigabe bei SD-Card-Wrapper

**Datei:** `src/Displays/BaseDisplay/SD.cpp`
**Zeilen:** 58, 164, 177, ...

**Problem:** Mehrfache `new SDWrapper()` Aufrufe. Lifecycle-Management sollte verifiziert werden — potentielles Memory Leak bei wiederholtem Mount/Unmount.

---

#### TS-01: Endianness-Annahme bei Byte-zu-Integer-Konvertierung

**Datei:** `src/lora_functions.cpp`
**Zeilen:** 161-164

```cpp
return ((uint32_t)ringBuffer[slot][6] << 24) |
       ((uint32_t)ringBuffer[slot][5] << 16) |
       ((uint32_t)ringBuffer[slot][4] << 8)  |
        (uint32_t)ringBuffer[slot][3];
```

**Problem:** Implizite Annahme über Byte-Reihenfolge. Funktioniert, solange Sender und Empfänger dieselbe Konvention verwenden, ist aber nicht dokumentiert.

**Empfehlung:** Dokumentieren oder `htonl`/`ntohl` verwenden.

---

### NIEDRIG (Kosmetisch / Informativ)

---

#### INFO-01: Offene TODOs mit Stabilitätsbezug

| Datei | Zeile | TODO |
|-------|-------|------|
| `src/command_functions.cpp` | 253 | `adapt node_time accordingly!` |
| `src/udp_functions.cpp` | 373 | `check HB accordingly to format` |
| `src/nrf52/nrf_eth.cpp` | 591 | `check HB accordingly to format` |
| `src/bme680.cpp` | 80 | `fix this` (Sensor-Detection Fallback) |

---

## Übersicht nach Datei

| Datei | Findings |
|-------|----------|
| `src/loop_functions.cpp` | BOF-01, MEM-01, MEM-02 |
| `src/aprs_functions.cpp` | BOF-02 |
| `src/command_functions.cpp` | BOF-04, INFO-01 |
| `src/esp32/esp32_functions.cpp` | BOF-03 |
| `src/lora_functions.cpp` | MEM-01, TS-01, DMA-01 |
| `src/extudp_functions.cpp` | MEM-01 |
| `src/udp_functions.cpp` | BOF-05 |
| `src/nrf52/nrf_eth.cpp` | BOF-05 |
| `src/t-deck/tdeck_main.cpp` | RTOS-01 |
| `src/Displays/BaseDisplay/SD.cpp` | MEM-03 |

---

## Empfohlene Prioritätsreihenfolge

1. **BOF-01** — Ring-Buffer memcpy Bounds-Check (einfacher Fix, hoher Impact)
2. **BOF-02** — Payload-Parsing Maximallänge einführen
3. **BOF-03** — sprintf -> snprintf in esp32_functions.cpp
4. **BOF-04** — memcpy Bounds-Check in command_functions.cpp
5. **MEM-01** — Stack-Arrays reduzieren oder statisch machen
6. **DMA-01** — SPI-Bus-Contention klären
7. **MEM-02** — String-Allokation in Loops durch snprintf ersetzen
8. **RTOS-01** — portMAX_DELAY durch endlichen Timeout ersetzen
9. **BOF-05** — Heartbeat-Parsing vervollständigen
10. **MEM-03** — SD-Card-Wrapper Lifecycle prüfen

---

## Statische Analyse — cppcheck 2.20.0

**Datum:** 2026-03-18
**Tools:**
1. cppcheck 2.20.0 (Homebrew) via `cppcheck --enable=all --suppress=missingIncludeSystem --suppress=unusedFunction --inconclusive --force --language=c++ -I src/ src/`
2. `pio check -e <env> --skip-packages` (PlatformIO cppcheck 1.211, build-umgebungsaware mit korrekten Defines/Includes pro Target)

**Scope:** Alle 147 Dateien unter `src/` plus `lib/` (plattformübergreifend). `pio check` vollständig für T-Deck und T-Deck Plus, partielle Ergebnisse für Heltec V3, E22, T-Beam, T-Beam Supreme, RAK4631 (abgebrochen nach >27k Zeilen Output pro Target, alle `src/`-Findings bereits erfasst).

**Hinweis:** Findings in `src/Displays/`, `src/GFX_Root/`, Font-Dateien und Macro-Parsing-Fehler (`PROGMEM`, `LV_VERSION_CHECK`, `ESP_IDF_VERSION_VAL`) sind herausgefiltert, da sie Third-Party-Code oder cppcheck-Limitierungen betreffen.

### Zusammenfassung

| Severity | Anzahl | Beschreibung |
|----------|--------|--------------|
| **error** | 8 | Buffer Out-of-Bounds (3+1), Uninitialized Variable (1+2), Missing Return (1), ODR Violation (2) |
| **warning** | ~70 | Argument Size Mismatch (2), Uninitialized Members, printf Format-Mismatches, Dangerous Casts, Null-Pointer-Checks, Identical Conditions |
| **style** | ~1460 | C-style Casts (415), Unused Variables (49), Known Conditions (43), Variable Scope (34), Argument Name Mismatches (29), Shadow Variables (15), ... |
| **performance** | 1 | (niedrig, vernachlässigbar) |

---

### FEHLER (error)

#### CPP-E01: Buffer Out-of-Bounds — `clfd` in loop_functions.cpp

**Datei:** `src/loop_functions.cpp:1385`
**cppcheck-ID:** `bufferAccessOutOfBounds`

Buffer `clfd` wird über seine deklarierte Größe hinaus zugegriffen.

---

#### CPP-E02: Buffer Out-of-Bounds — `node_passwd` in event_functions.cpp

**Datei:** `src/t-deck/event_functions.cpp:593`
**cppcheck-ID:** `bufferAccessOutOfBounds`

`meshcom_settings.node_passwd` wird über seine Arraygrenzen hinaus zugegriffen.

---

#### CPP-E03: Buffer Out-of-Bounds — `value` in web_functions.cpp

**Datei:** `src/web_functions/web_functions.cpp:1599`
**cppcheck-ID:** `bufferAccessOutOfBounds`

```cpp
snprintf(value, 100, "%s", meshcom_settings.node_mcp17t[io]);
```

Buffer `value` ist kleiner als die angegebene Größe 100, oder `io`-Index kann out-of-bounds gehen.

---

#### CPP-E04: Uninitialized Variable `nodetype`

**Datei:** `src/loop_functions.cpp:1266`
**cppcheck-ID:** `uninitvar`

Variable `nodetype` wird verwendet bevor sie initialisiert wird.

---

#### CPP-E05: Missing Return Statement

**Datei:** `src/nrf52/api_functions.cpp:154`
**cppcheck-ID:** `missingReturn`

Funktion mit non-void Rückgabetyp hat einen Pfad ohne `return`-Statement. Kann undefiniertes Verhalten auslösen.

---

#### CPP-E06: One Definition Rule Violation — `s_meshcom_settings`

**Dateien:** `src/nrf52/WisBlock-API.h:173` vs. `src/esp32/esp32_flash.h:8`
**cppcheck-ID:** `ctuOneDefinitionRuleViolation`

Struct `s_meshcom_settings` ist in ESP32- und nRF52-Header unterschiedlich definiert. Gleiches gilt für `_ui_setting` in `src/t-deck-pro/ui_deckpro.h:97` vs. `src/t5-epaper/ui.h:75`. Beide werden durch `#ifdef` getrennt kompiliert, daher in der Praxis kein Problem — aber ein Wartungsrisiko bei Refactoring.

#### CPP-E07: Array Out-of-Bounds — `strMaps[5]` *(nur pio check)*

**Datei:** `src/t-deck/lv_obj_functions.cpp:1681`
**cppcheck-ID:** `arrayIndexOutOfBounds`

Array `strMaps[5]` wird an Index 5 zugegriffen (0-basiert, max erlaubt = 4).

---

#### CPP-E08: Uninitialized Variable `j` *(nur pio check)*

**Dateien:** `src/t-deck-pro/ui_deckpro.cpp:83`, `src/t5-epaper/ui.cpp:78`
**cppcheck-ID:** `uninitvar`

Variable `j` wird uninitalisiert in einer Schleife verwendet.

---

### WARNUNGEN (warning) — Auswahl der relevantesten

#### CPP-W01: Uninitialized Variables — `clat`, `clon`

**Datei:** `src/aprs_functions.cpp:1175`
**cppcheck-ID:** `uninitvar`

Variablen `clat` und `clon` werden möglicherweise uninitalisiert an printf übergeben.

---

#### CPP-W02: Array Index Out-of-Bounds — `dzeile[6]`

**Datei:** `src/loop_functions.cpp:1760`
**cppcheck-ID:** `arrayIndexOutOfBoundsCond`

```
Either the condition 'izeile>5' is redundant or the array 'dzeile[6]' is accessed at index 6.
```

---

#### CPP-W03: printf Format-Type-Mismatches

| Datei | Zeile | Problem |
|-------|-------|---------|
| `src/loop_functions.cpp` | 2160 | `%u` erwartet `unsigned int`, bekommt `signed int` |
| `src/loop_functions.cpp` | 3052, 3054 | `%i` erwartet `int`, bekommt `unsigned int` |
| `src/t-deck/lv_obj_functions.cpp` | 3464, 3475 | `%i` erwartet `int`, bekommt `unsigned int` |
| `src/t-deck/lv_obj_functions.cpp` | 4039, 4056 | `%li` erwartet `long`, bekommt `unsigned long` |
| `src/t5-epaper/ui_port.cpp` | 190 | `%llu` erwartet `unsigned long long`, bekommt `unsigned long` |
| `src/t-deck-pro/ui_deckpro_port.cpp` | 157 | `%llu` erwartet `unsigned long long`, bekommt `unsigned long` |

---

#### CPP-W04: Null-Pointer-Dereference nach Redundant-Check

**Dateien:** `src/t5-epaper/ui_port.cpp:190`, `src/t-deck-pro/ui_deckpro_port.cpp:157`
**cppcheck-ID:** `nullPointerRedundantCheck`

Pointer `used` wird nach `if(used)` Check dereferenziert, aber `*total` wird vorher ohne Check genutzt.

---

#### CPP-W05: Identical Inner Condition

**Datei:** `src/tinyxml_functions.cpp:119`
**cppcheck-ID:** `identicalInnerCondition`

```cpp
if(bSOFTSERDEBUG)if(bSOFTSERDEBUG)Serial.printf(...)  // doppelte Prüfung
```

---

#### CPP-W06: Uninitialized Member Variables

| Klasse | Header | Fehlende Member |
|--------|--------|-----------------|
| `Clock` | `src/clock.cpp:56` | `boAlarmValid_m` |
| `MatchState` | `src/Regexp.h:79-82` | `capture`, `level`, `MatchLength`, `MatchStart`, `src_end`, `src_len` |
| `BQ27220` | `src/t-deck-pro/bq27220.h:109` | `bat_st` |

---

#### CPP-W07: Buffer `msg_buffer` zu klein für `sendExtern()` *(nur pio check)*

**Datei:** `src/loop_functions.cpp:2477, 2967`
**cppcheck-ID:** `argumentSize`

`msg_buffer` wird an `sendExtern()` übergeben, aber die Funktion erwartet einen größeren Buffer im 3. Argument. Kann zu Buffer-Overread führen.

---

#### CPP-W08: printf Format-Mismatches in T-Deck Pro *(nur pio check)*

| Datei | Zeile | Problem |
|-------|-------|---------|
| `src/t-deck-pro/ui_deckpro.cpp` | 2128, 2145 | `%li` erwartet `long`, bekommt `unsigned long` |

---

### STIL (style) — Statistische Übersicht

| Kategorie | Anzahl | Beschreibung |
|-----------|--------|--------------|
| `cstyleCast` | 415 | C-style Casts `(char*)`, `(uint8_t*)` etc. — Überwiegend in `command_functions.cpp` und `udp_functions.cpp` |
| `unreadVariable` | 49 | Zugewiesene Werte werden nie gelesen |
| `knownConditionTrueFalse` | 43 | Bedingungen, die immer true/false sind |
| `variableScope` | 34 | Variable könnte in engerem Scope deklariert werden |
| `funcArgNamesDifferent` | 29 | Parameternamen in Declaration vs. Definition unterschiedlich |
| `dangerousTypeCast` | 23 | Potenziell unsichere Old-style Casts (bereits als warning gezählt) |
| `redundantAssignment` | 20 | Wert wird vor Nutzung überschrieben |
| `constParameterPointer` | 19 | Parameter könnte `const` sein |
| `shadowVariable` | 15 | Lokale Variable verdeckt äußere Variable |
| `constParameter` | 13 | Parameter könnte `const` sein |
| `unreachableCode` | 12 | Unerreichbarer Code nach return/break |
| `redundantInitialization` | 10 | Initialisierung wird sofort überschrieben |

**Top-Dateien nach Findings:**

| Datei | error | warning | style |
|-------|-------|---------|-------|
| `src/command_functions.cpp` | — | — | ~180 (fast ausschließlich `cstyleCast`) |
| `src/loop_functions.cpp` | 2 | 3 | ~80 |
| `src/udp_functions.cpp` | — | — | ~40 |
| `src/web_functions/web_functions.cpp` | 1 | — | ~25 |
| `src/t-deck/lv_obj_functions.cpp` | 1 | 4 | ~20 |
| `src/t-deck/event_functions.cpp` | 1 | — | ~15 |
| `src/t-deck-pro/ui_deckpro.cpp` | 1 | 2 | ~10 |
| `src/t5-epaper/ui.cpp` | 1 | — | ~5 |
| `src/mheard_functions.cpp` | — | 10 | ~10 |
| `src/nrf52/api_functions.cpp` | 1 | — | ~5 |
| `src/aprs_functions.cpp` | — | 2 | ~10 |

---

### Empfohlene Prioritätsreihenfolge (cppcheck-Findings)

1. **CPP-E01..E03** — Buffer Out-of-Bounds (3 Stellen) — direkt crash-/exploit-relevant
2. **CPP-E07** — Array `strMaps[5]` OOB in lv_obj_functions — Crash auf T-Deck
3. **CPP-E04, E08** — Uninitialized Variables (`nodetype`, `j`) — undefiniertes Verhalten
4. **CPP-E05** — Missing Return — undefiniertes Verhalten auf nRF52
5. **CPP-W07** — `msg_buffer` zu klein für `sendExtern()` — Buffer-Overread
6. **CPP-W01** — Uninitialized `clat`/`clon` — falscher APRS-Output möglich
7. **CPP-W02** — Array-Index `dzeile[6]` — potentieller Crash
8. **CPP-W03, W08** — printf Format-Mismatches — Portabilitätsrisiko
9. **CPP-W06** — Uninitialized Members — Regexp/BQ27220/Clock Klassen

---

### Korrelation mit manuellen Findings

| cppcheck | Manuell | Status |
|----------|---------|--------|
| CPP-E01 (`clfd` OOB) | BOF-01 (Ring-Buffer memcpy) | **Gleiche Datei, ergänzende Findings** |
| CPP-E03 (`value` OOB) | — | **Neu** — nur von cppcheck gefunden |
| CPP-E02 (`node_passwd` OOB) | — | **Neu** — nur von cppcheck gefunden |
| CPP-E04 (`nodetype` uninit) | — | **Neu** |
| CPP-E05 (missing return) | — | **Neu** |
| CPP-E07 (`strMaps[5]` OOB) | — | **Neu** — nur von pio check gefunden |
| CPP-E08 (`j` uninit) | — | **Neu** — nur von pio check gefunden |
| CPP-W02 (`dzeile` index) | — | **Neu** |
| CPP-W07 (`msg_buffer` argumentSize) | — | **Neu** — nur von pio check gefunden |
| — | BOF-02..04 | Nicht von cppcheck erkannt (dynamische String-Probleme) |
| — | MEM-01, DMA-01, RTOS-01 | Architektur-Issues — außerhalb cppcheck-Scope |

---

## Methodik

- **Web-Recherche:** ESP-IDF Docs, Nordic DevZone, FreeRTOS API Reference, MISRA C++ Guidelines, SX1262 Datasheet
- **Statische Analyse (manuell):** Manuelle Durchsicht aller .cpp/.h Dateien unter `src/`
- **Statische Analyse (automatisiert):** cppcheck 2.20.0 mit `--enable=all --inconclusive --force` über alle 147 Source-Dateien
- **Pattern-Matching:** Grep nach `memcpy`, `sprintf`, `strcpy`, `volatile`, `atomic`, `IRAM_ATTR`, `xSemaphore`, `portENTER_CRITICAL`, `String`, `malloc`, `new`
- **Kontextanalyse:** Cross-Referenz zwischen ISR-Handlern und aufgerufenen Funktionen
- **PR-Abgleich:** Findings gegen PR #781 (RACE-01..05) und PR #779 validiert, bereits behobene Issues entfernt
