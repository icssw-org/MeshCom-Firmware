# nRF52/RAK4630: LoRa Nachrichtenverlust-Fixes

Port der ESP32-Fixes aus PR #710 und PR #714 auf die nRF52/RAK4630-Plattform.

## Hintergrund

PR #710 und #714 haben 12+ Bugs behoben, die ~50% bidirektionalen Nachrichtenverlust auf ESP32 (Heltec V3) verursachten. Die meisten Fixes befinden sich im gemeinsamen Code (`lora_functions.cpp`, `loop_functions.cpp/h`, `udp_functions.cpp`) und wirken bereits auf beiden Plattformen.

Was fehlte, waren die **plattformspezifischen Main-Loop-Verbesserungen** aus `esp32_main.cpp`, die hier fuer `nrf52_main.cpp` angepasst wurden.

## Architekturunterschied ESP32 vs. nRF52

- **ESP32**: Nutzt RadioLib mit Interrupt-Flags und Main-Loop-Polling
- **nRF52**: Nutzt die RAK SX126x-Bibliothek mit direkten Callbacks aus dem Radio-IRQ

Daher koennen nicht alle ESP32-Fixes 1:1 uebernommen werden — sie muessen an das RAK-Callback-Modell angepasst werden.

## Bereits im Shared-Code vorhandene Fixes

Diese Fixes aus PR #710/#714 sind bereits fuer nRF52 aktiv:

| Bug | Beschreibung | Datei |
|-----|-------------|-------|
| BUG #3 | OnHeaderDetect setzt cmd_counter/tx_waiting nicht mehr zurueck | `lora_functions.cpp` |
| BUG #4 | cmd_counter von 7 auf 3 reduziert | `lora_functions.cpp` |
| BUG #5 | Retransmit-Schwelle angepasst | `lora_functions.cpp` |
| BUG #6 | ACK msg_id Fix | `lora_functions.cpp` |
| BUG #8 | DM-ACK loescht Ringpuffer | `loop_functions.cpp` |
| BUG #9-12 | Ringpuffer-Deadlock-Fixes | `lora_functions.cpp` |
| PR #714 BUG-01 | Retransmit-Mechanismus (Slot-Erhaltung, Full-Range-Scan, ACK-Slot-Freigabe) | `lora_functions.cpp` |
| PR #714 BUG-02 | RING_OVERFLOW mit Puffernamen | `loop_functions.cpp/h` |

## nRF52-spezifische Aenderungen

### 1. Radio.Rx() an den Anfang von OnRxDone verschoben (BUG #2 Aequivalent)

**Datei:** `src/lora_functions.cpp`

**Problem:** `Radio.Rx(RX_TIMEOUT_VALUE)` wurde am Ende von `OnRxDone()` aufgerufen, nach ~800 Zeilen Paketverarbeitung. Das erzeugt ein RX-Blindfenster in dem eingehende Pakete verloren gehen.

**Fix:** `Radio.Rx()` wird jetzt am Anfang von `OnRxDone()` aufgerufen (nach Sicherheitskopie des Payloads). Ein `#if defined BOARD_RAK4630` Guard stellt sicher, dass ESP32 nicht betroffen ist.

**Warum Payload-Kopie:** Der `payload`-Zeiger kann auf den internen Radiopuffer zeigen, der durch `Radio.Rx()` ueberschrieben wird. Ein statischer Puffer (`rxPayloadCopy`) sichert die Daten vor dem Radio-Neustart.

### 2. RING_STATUS periodischer Debug-Report

**Datei:** `src/nrf52/nrf52_main.cpp`

Alle 30 Sekunden wird (bei aktivem `bLORADEBUG`) ein Statusbericht des Ringpuffers ausgegeben: Anzahl wartender, wiederholender und abgeschlossener Eintraege sowie die aktuelle Queue-Laenge.

### 3. Timeout-Handler mit is_receiving-Guard (BUG #1 Aequivalent)

**Datei:** `src/nrf52/nrf52_main.cpp`

**Problem:** Der bisherige Timeout-Handler setzte den Timer blind zurueck, was zu einer Race-Condition fuehren konnte: Header erkannt (`is_receiving=true`) aber Timeout feuert und unterbricht den laufenden Empfang.

**Fix:** Wenn `is_receiving` aktiv ist, wird der Timer verlaengert statt zurueckgesetzt. Zusaetzlich wird bei tatsaechlichem Timeout `Radio.Rx()` neu gestartet als Sicherheitsmassnahme.

### 4. TX_GATE_ENTER Debug-Ausgabe

**Datei:** `src/nrf52/nrf52_main.cpp`

Debug-Ausgabe vor `doTX()` zeigt Queue-Laenge, `cmd_counter` und `tx_waiting` Status.

**Hinweis:** Anders als beim ESP32-Fix werden `cmd_counter` und `tx_waiting` hier NICHT zurueckgesetzt. nRF52 ruft `doTX()` bei jeder Loop-Iteration auf, daher funktioniert der CAD-Delay-Mechanismus bereits korrekt. Ausserdem wuerde das nRF52-spezifische `cmd_counter=50` fuer die Ethernet-DHCP-Pause kaputt gehen.

## Geaenderte Dateien

| Datei | Aenderung |
|-------|-----------|
| `src/lora_functions.cpp` | `Radio.Rx()` von Ende an Anfang von `OnRxDone()` verschoben |
| `src/nrf52/nrf52_main.cpp` | RING_STATUS, Timeout-Handler, TX-Debug hinzugefuegt |
| `nRF-lora-doku.md` | Diese Dokumentation |

## Verifikation

1. **Build RAK4631:** Kompilieren — keine Compile-Fehler
2. **Build ESP32:** Fuer Heltec V3 kompilieren — keine Regression (Guards)
3. **Serial-Debug** (`bLORADEBUG` aktivieren):
   - `RX_RESTART_EARLY` bei jedem empfangenen Paket
   - `RING_STATUS` alle 30s
   - `RX_TIMEOUT_FIRE` / `RX_RESTART` bei Timeout-Events
   - `TX_GATE_ENTER` vor Sendungen
4. **Feldtest:** 2+ Node Mesh mit RAK4631, bidirektionale Nachrichtenzustellung pruefen

## Referenzen

- PR #710: Erste Runde der LoRa-Fixes (BUG #1-#12)
- PR #714: Retransmit-Mechanismus-Reparatur und RING_OVERFLOW-Fix
