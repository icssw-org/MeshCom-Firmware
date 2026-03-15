# MeshCom Firmware Optimierung -- Implementierungsdokument

**Erstellt:** 2026-03-07

**Branch:** pr-dev-bugfix-csma (Commit ea13b72)

**Strategie:** Jeder Abschnitt = 1 Commit. Minimale, isolierte Aenderungen.

**Status:** Alle Aenderungen implementiert und kompiliert (heltec_wifi_lora_32_V3, E22-DevKitC).

---

## Aenderung 0: RadioLib Update 7.1.2 -> 7.6.0

### Ausgangslage

Das Projekt nutzt RadioLib 7.1.2. Version 7.6.0 enthaelt wichtige Bugfixes:
- v7.2.0: CAD-Optimierung fuer bessere Erkennungsrate
- v7.3.0: Fix finishReceive cleared IRQ flags zu frueh (SX126x)
- v7.5.0: Optimierte PA-Konfigurationstabelle fuer SX1262/SX1268
- v7.6.0: Fix incorrect LoRa header IRQ check logic

Breaking-Change-Analyse: **Keine Auswirkungen auf MeshCom** (alle Breaking Changes
betreffen LoRaWAN oder nicht genutzte APIs).

### Zielzustand

RadioLib auf 7.6.0 anheben. Alle bestehenden APIs sind stabil geblieben.

### Aenderungen

**Datei: `platformio.ini`**
```
Zeile 106: jgromes/RadioLib@7.1.2
        -> jgromes/RadioLib@7.6.0
```

**Datei: `variants/t_deck_pro/platformio.ini`**
```
Zeile 75:  jgromes/RadioLib@7.1.2
        -> jgromes/RadioLib@7.6.0
```

**Datei: `variants/t5_epaper/platformio.ini`**
```
Zeile 66:  jgromes/RadioLib@7.1.2
        -> jgromes/RadioLib@7.6.0
```

### Commit-Message
```
deps: update RadioLib 7.1.2 -> 7.6.0

Includes CAD detection rate optimization (7.2.0), IRQ flag clearing
fix in finishReceive (7.3.0), SX1262/SX1268 PA optimization (7.5.0),
and LoRa header IRQ check fix (7.6.0). No breaking changes affect
MeshCom (all breaking changes are LoRaWAN-only or unused APIs).
```

---

## Aenderung 1: Volatile-Deklaration fuer ISR-geteilte Flags

### Ausgangslage

Die globalen Flags `is_receiving`, `tx_is_active` und `bSetLoRaAPRS` werden zwischen
ISR-Callbacks (OnHeaderDetect, OnTxDone) und der Main Loop geteilt, aber NICHT als
`volatile` deklariert. Ohne `volatile` kann der Compiler Reads innerhalb einer
Loop-Iteration wegoptimieren -- z.B. koennte `while(!is_receiving)` zu einer
Endlosschleife kompiliert werden.

Zusaetzlich werden die Channel-Utilization-Variablen `ch_util_rx_start`,
`ch_util_tx_start`, `ch_util_rx_accum` und `ch_util_tx_accum` in ISR-Callbacks
(OnRxDone, OnRxTimeout, OnRxError, OnTxDone, OnTxTimeout) geschrieben und in der
Main Loop gelesen -- ebenfalls ohne `volatile`.

### Zielzustand

Alle 7 Flags als `volatile` deklarieren. Kein funktionaler Impact, kein Risiko.

### Aenderungen

**Datei: `src/loop_functions.cpp`**
```
Zeile 66:  bool bSetLoRaAPRS = false;
        -> volatile bool bSetLoRaAPRS = false;

Zeile 288: bool is_receiving = false;
        -> volatile bool is_receiving = false;

Zeile 289: bool tx_is_active = false;
        -> volatile bool tx_is_active = false;

Zeile 295: unsigned long ch_util_rx_start = 0;
        -> volatile unsigned long ch_util_rx_start = 0;

Zeile 296: unsigned long ch_util_tx_start = 0;
        -> volatile unsigned long ch_util_tx_start = 0;

Zeile 297: unsigned long ch_util_rx_accum = 0;
        -> volatile unsigned long ch_util_rx_accum = 0;

Zeile 298: unsigned long ch_util_tx_accum = 0;
        -> volatile unsigned long ch_util_tx_accum = 0;
```

**Datei: `src/loop_functions_extern.h`**
```
Zeile 27:  extern bool bSetLoRaAPRS;
        -> extern volatile bool bSetLoRaAPRS;

Zeile 189: extern bool is_receiving;
        -> extern volatile bool is_receiving;

Zeile 190: extern bool tx_is_active;
        -> extern volatile bool tx_is_active;

Zeile 196: extern unsigned long ch_util_rx_start;
        -> extern volatile unsigned long ch_util_rx_start;

Zeile 197: extern unsigned long ch_util_tx_start;
        -> extern volatile unsigned long ch_util_tx_start;

Zeile 198: extern unsigned long ch_util_rx_accum;
        -> extern volatile unsigned long ch_util_rx_accum;

Zeile 199: extern unsigned long ch_util_tx_accum;
        -> extern volatile unsigned long ch_util_tx_accum;
```

**Datei: `src/batt_functions.cpp` (zusaetzlich, im Plan nicht erfasst)**

`batt_functions.cpp` enthaelt eine eigene `extern bool is_receiving;` Deklaration
(Zeile 17), die NICHT ueber `loop_functions_extern.h` eingebunden wird. Ohne
Anpassung verursacht dies einen Compile-Fehler wegen Typ-Konflikt.

```
Zeile 17:  extern bool is_receiving;
        -> extern volatile bool is_receiving;
```

### Commit-Message
```
fix: add volatile qualifier to ISR-shared flags

is_receiving, tx_is_active, bSetLoRaAPRS and ch_util_rx/tx_start/accum
are shared between ISR callbacks (OnRxDone, OnTxDone, OnRxTimeout,
OnTxTimeout, OnRxError, OnHeaderDetect) and main loop. Without volatile,
compiler may optimize away reads in tight loops, causing stale values.

Also fix duplicate extern declaration in batt_functions.cpp.
```

---

## Aenderung 2: OnHeaderDetect fuer ESP32 (IRQ-Polling mit Safety Net)

### Ausgangslage

Auf NRF52 (RAK4630) existiert ein `OnHeaderDetect()`-Callback, der `is_receiving=true`
setzt sobald ein LoRa-Header erkannt wird. Auf ESP32 (RadioLib) fehlt diese
Frueh-Erkennung -- ein eingehendes Paket wird erst nach vollstaendigem Empfang
ueber `receiveFlag` erkannt.

**Warum kein ISR-basierter Ansatz (HEADER_VALID auf DIO1):**
Der SX1262 DIO1-Pin ist **level-sensitiv** -- bleibt HIGH solange IRQ-Flags gesetzt
sind. ESP32 GPIO-Interrupt ist **edge-triggered** (RISING). Wenn HEADER_VALID auf DIO1
gemappt wird, kann die ISR `setFlagReceive()` nicht zwischen HEADER_VALID und RX_DONE
unterscheiden. Bei `bEnableInterruptReceive = true` setzt die ISR `receiveFlag = true`,
die Main Loop ruft `readData()` auf einem unvollstaendigen Paket auf -- Empfang
abgebrochen, Paket verloren. Bei SF12/BW125 dauert ein Paket 200ms-2s, die Main Loop
laeuft alle paar ms -- `receiveFlag` wird fast immer VOR `RX_DONE` verarbeitet.

**Loesung: IRQ-Register-Polling statt DIO1-Mapping.**
`radio.getIrqFlags()` (SX126x.h:1050) liest das SX1262 IRQ-Status-Register per SPI
ohne Flags zu clearen. Die Main Loop kann vor dem CAD-Scan pruefen, ob HEADER_VALID
oder PREAMBLE_DETECTED gesetzt ist -- ohne den ISR-Pfad zu beruehren.

### Zielzustand

1. `PREAMBLE_DETECTED` zusaetzlich als IRQ-Flag aktivieren (HEADER_VALID ist bereits Standard)
2. DIO1-Maske **unveraendert** lassen: nur `RX_DONE` auf DIO1 (ISR bleibt wie bisher)
3. **Vor dem CAD-Scan:** IRQ-Register per `radio.getIrqFlags()` pollen. Wenn
   HEADER_VALID oder PREAMBLE_DETECTED gesetzt -> `is_receiving = true`, TX abbrechen
4. **Safety Net:** Nach jedem RX-Restart pruefen, ob DIO1 noch HIGH ist (verpasste
   RX_DONE-Flanke). Wenn ja, `receiveFlag` manuell setzen.

### Aenderungen

**Schritt 1: startReceive() mit PREAMBLE_DETECTED in IRQ-Flags**

Alle `radio.startReceive()`-Stellen (10 Vorkommen in esp32_main.cpp) ersetzen:

```cpp
// --- AKTUELL: ---
state = radio.startReceive();

// --- NEU: ---
// IRQ-Flags: Default + PREAMBLE_DETECTED (HEADER_VALID ist bereits im Default)
// DIO1-Maske: Nur RX_DONE auf DIO1 (unveraendert zum bisherigen Verhalten)
state = radio.startReceive(
    RADIOLIB_SX126X_RX_TIMEOUT_INF,
    RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
    RADIOLIB_IRQ_RX_DEFAULT_MASK,   // nur RX_DONE auf DIO1
    0);
```

**Implementierungsnotiz:** Alle 10 Vorkommen wurden per `replace_all` ersetzt
(Zeilen 1240, 1276, 1309, 1336, 1756, 1878, 1989, 2008, 3075, 3118).

**Schritt 2: IRQ-Polling vor dem CAD-Scan**

**Datei: `src/esp32/esp32_main.cpp` -- vor CAD-Scan:**

**Implementierungsabweichung:** Das Dokument sah `break;` vor um den TX-Block zu
verlassen. Da der Code jedoch NICHT in einer Schleife oder einem Switch liegt,
wuerde `break;` nicht kompilieren. Stattdessen wurde eine `bool irq_rx_active`
Flag-Variable verwendet, die den restlichen TX-Block in ein `if(!irq_rx_active)`
einschliesst.

```cpp
// Header/Preamble-Check per IRQ-Register-Polling
bool irq_rx_active = false;
{
    uint32_t irqStatus = radio.getIrqFlags();
    if(irqStatus & (RADIOLIB_SX126X_IRQ_HEADER_VALID |
                    RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED))
    {
        is_receiving = true;
        irq_rx_active = true;
        if(bLORADEBUG)
            Serial.printf("[MC-DBG] IRQ_POLL hdr/pre=0x%04X -> TX_ABORT\n",
                          irqStatus);
        iReceiveTimeOutTime = millis();
        csma_timeout = csma_compute_timeout(cad_attempt);
    }
}

if(!irq_rx_active)
{
    // Bestehender Code: CAD-Scan
    bEnableInterruptReceive = false;
    radio.clearPacketReceivedAction();
    int cad_result = radio.scanChannel();
    // ... restlicher TX-Block ...
} // end if(!irq_rx_active)
```

**Schritt 3: Kompatibilitaets-Macro fuer LORA_DIO1**

Nicht alle Boards definieren `LORA_DIO1`. Von 21 Board-Varianten nutzen 4
alternative Macro-Namen:

| Board-Variante | Macro | Pin |
|----------------|-------|-----|
| E22_1262_S3-DevKitC, E22_1268_S3-DevKitC | `E22_DIO1` | 6 |
| heltec_wireless_tracker | `RADIO_DIO1_PIN` | 14 |
| LilyGo_T-Beam-1W | `RADIO_DIO1_PIN` | 1 |
| vision-master-e290 | `PIN_LORA_DIO_1` | 14 |
| wiscore_rak4631 | (keins -- nRF52, anderer Code-Pfad) | n/a |

**Datei: `src/esp32/esp32_main.cpp` (nach den Includes, vor GPS-Includes):**
```cpp
// Kompatibilitaets-Macro: DIO1-Pin hat je nach Board verschiedene Namen
#ifndef LORA_DIO1
  #if defined(E22_DIO1)
    #define LORA_DIO1 E22_DIO1
  #elif defined(RADIO_DIO1_PIN)
    #define LORA_DIO1 RADIO_DIO1_PIN
  #elif defined(PIN_LORA_DIO_1)
    #define LORA_DIO1 PIN_LORA_DIO_1
  #else
    #warning "LORA_DIO1 not defined -- safety net digitalRead() disabled"
  #endif
#endif
```

**Schritt 4: Safety Net -- verpasste RX_DONE-Flanke erkennen**

Safety Net wird nach jedem RX-Restart eingefuegt (in Aenderung 3 integriert):
```cpp
#ifdef LORA_DIO1
if(digitalRead(LORA_DIO1) == HIGH && !receiveFlag)
{
    receiveFlag = true;
    if(bLORADEBUG)
        Serial.println(F("[MC-DBG] SAFETY_NET missed_edge DIO1=HIGH"));
}
#endif
```

### Risiken und Gegenmassnahmen

| Risiko | Gegenmassnahme |
|--------|----------------|
| Verpasste RX_DONE-Flanke (DIO1 bleibt HIGH) | Safety Net: digitalRead(LORA_DIO1) prueft DIO1-Level |
| SPI-Read Latenz bei getIrqFlags() vor CAD | Einmal pro TX-Versuch (~50us), nicht in einer Schleife |
| HEADER_VALID false positive durch Stoerung | is_receiving wird beim naechsten Timeout zurueckgesetzt |
| LORA_DIO1 nicht bei allen Boards definiert | Kompatibilitaets-Macro (Schritt 3) + `#ifdef`-Guard |
| `break;` im Original-Dokument nicht kompilierbar | Stattdessen `bool irq_rx_active` Flag mit `if(!irq_rx_active)` Block |

### Commit-Message
```
feat: add ESP32 header detection via IRQ register polling + safety net

Before CAD scan, poll SX1262 IRQ status register via radio.getIrqFlags()
to check for HEADER_VALID or PREAMBLE_DETECTED. If set, a packet is
being received — abort TX attempt and set is_receiving = true.

Add PREAMBLE_DETECTED to IRQ flags in startReceive() (HEADER_VALID is
already in RADIOLIB_IRQ_RX_DEFAULT_FLAGS). DIO1 mask stays unchanged
(only RX_DONE triggers ISR), avoiding the problem where HEADER_VALID
on DIO1 would cause premature readData() on incomplete packets.

Safety net: after every RX restart, check DIO1 level with
digitalRead(LORA_DIO1). If HIGH but receiveFlag not set, a rising
edge was missed. Manually set receiveFlag and log the event.

Uses bool flag + if-guard instead of break (no enclosing loop).

This brings ESP32 to parity with NRF52's OnHeaderDetect callback.
```

---

## Aenderung 3: Atomarer RX-Restart nach TX

### Ausgangslage

Nach TX-Completion wird der RX-Modus in mehreren nicht-atomaren Schritten
wiederhergestellt (esp32_main.cpp:1867-1878):

```cpp
bEnableInterruptTransmit = false;           // Zeile 1867
radio.clearPacketSentAction();              // Zeile 1868
bEnableInterruptReceive = false;            // Zeile 1871
radio.clearPacketReceivedAction();          // Zeile 1872
bEnableInterruptReceive = true;             // Zeile 1875
radio.setPacketReceivedAction(setFlagReceive);  // Zeile 1876
int state = radio.startReceive();           // Zeile 1878
```

**Problem:** Zwischen `bEnableInterruptReceive = true` (Zeile 1875) und
`startReceive()` (Zeile 1878) koennte ein ISR feuern und `receiveFlag` setzen,
obwohl der Radio noch nicht im RX-Modus ist. Ausserdem: DIO1 ist level-sensitiv --
bleibt HIGH solange IRQ-Flags gesetzt sind. Wenn nach `clearPacketReceivedAction()`
ein neues Paket kommt und DIO1 hochgeht, wird nach `setPacketReceivedAction()` die
RISING Edge moeglicherweise nicht erkannt, weil DIO1 bereits HIGH war.

**Loesung:** Reihenfolge aendern: **Erst startReceive(), dann Callback setzen, dann
Gate oeffnen.** So ist der Radio bereits im RX-Modus bevor der ISR-Pfad aktiviert
wird. Zusaetzlich: nach `startReceive()` DIO1-Level pruefen um verpasste Flanken
zu erkennen.

### Aenderungen

**Datei: `src/esp32/esp32_main.cpp`**

**TX-Done Handler (Zeilen 1867-1878):**
```cpp
// --- AKTUELL: ---
bEnableInterruptTransmit = false;           // Zeile 1867
radio.clearPacketSentAction();              // Zeile 1868
bEnableInterruptReceive = false;            // Zeile 1871
radio.clearPacketReceivedAction();          // Zeile 1872
bEnableInterruptReceive = true;             // Zeile 1875
radio.setPacketReceivedAction(setFlagReceive);  // Zeile 1876
int state = radio.startReceive();           // Zeile 1878

// --- NEU: ---
// Atomarer RX-Restart: Radio in RX BEVOR ISR aktiv
bEnableInterruptTransmit = false;
bEnableInterruptReceive = false;
radio.clearPacketSentAction();
radio.clearPacketReceivedAction();
int state = radio.startReceive();           // Radio in RX zuerst
radio.setPacketReceivedAction(setFlagReceive);
bEnableInterruptReceive = true;

// Verpasste Flanke erkennen
#ifdef LORA_DIO1
if(digitalRead(LORA_DIO1) == HIGH)
{
    receiveFlag = true;
    if(bLORADEBUG)
        Serial.println(F("[MC-DBG] RX_RESTART missed_edge recovery"));
}
#endif
```

**RX-Timeout Handler (Zeilen 1753-1763) -- bereits korrekte Reihenfolge:**
```cpp
// startReceive() VOR setPacketReceivedAction() VOR bEnableInterruptReceive = true
// Nur DIO1-Check nachgeruestet:
bEnableInterruptReceive = true;              // (bestehend)
// NEU: Flanken-Recovery
#ifdef LORA_DIO1
if(digitalRead(LORA_DIO1) == HIGH)
{
    receiveFlag = true;
    if(bLORADEBUG)
        Serial.println(F("[MC-DBG] RX_TIMEOUT missed_edge recovery"));
}
#endif
```

**checkRX() Funktion (Zeilen 3071-3075) -- gleiches Pattern anwenden:**
```cpp
// --- AKTUELL: ---
radio.clearPacketReceivedAction();          // Zeile 3071
radio.clearPacketSentAction();              // Zeile 3072
bEnableInterruptReceive = true;             // Zeile 3073
radio.setPacketReceivedAction(setFlagReceive);  // Zeile 3074
int rxstate = radio.startReceive();         // Zeile 3075

// --- NEU: ---
radio.clearPacketReceivedAction();
radio.clearPacketSentAction();
int rxstate = radio.startReceive();          // Radio in RX zuerst
radio.setPacketReceivedAction(setFlagReceive);
bEnableInterruptReceive = true;
#ifdef LORA_DIO1
if(digitalRead(LORA_DIO1) == HIGH)
{
    receiveFlag = true;
    if(bLORADEBUG)
        Serial.println(F("[MC-DBG] CHECKRX missed_edge recovery"));
}
#endif
```

### Commit-Message
```
fix: reorder RX restart sequence for atomic mode transition

Ensure radio.startReceive() is called BEFORE re-enabling the ISR
callback, preventing a window where ISR could fire before radio
is in RX mode. Add DIO1 level check after restart to recover from
missed rising edges (SX1262 DIO1 is level-sensitive).
```

---

## Aenderung 4: RX-Payload-Buffer nicht-ueberschreibend mit Debug-Logging

### Ausgangslage

Auf NRF52 (RAK4630) wird ein statischer Buffer fuer den RX-Payload verwendet
(lora_functions.cpp:115):

```cpp
static uint8_t rxPayloadCopy[UDP_TX_BUF_SIZE];
```

Dieser wird bei **jedem** Empfang ueberschrieben. Wenn zwei Pakete schnell
hintereinander eintreffen, koennte das erste Paket ueberschrieben werden bevor
`OnRxDone()` es vollstaendig verarbeitet hat. Vermutung: Hier gehen Empfangspakete
verloren, besonders bei hoher Netzlast.

### Zielzustand

Double-Buffer mit Logging: Zwei statische Buffer im Wechsel, mit Debug-Output
wenn ein Ueberschreiben verhindert oder erkannt wird.

### Aenderungen

**Datei: `src/lora_functions.cpp`**

**Innerhalb `#if defined BOARD_RAK4630` (OnRxDone-Anfang):**
```cpp
// --- AKTUELL: ---
static uint8_t rxPayloadCopy[UDP_TX_BUF_SIZE];
uint16_t rxSize = (size <= UDP_TX_BUF_SIZE) ? size : UDP_TX_BUF_SIZE;
memcpy(rxPayloadCopy, payload, rxSize);
payload = rxPayloadCopy;
size = rxSize;
Radio.Rx(RX_TIMEOUT_VALUE);

// --- NEU: ---
static uint8_t rxPayloadCopy[2][UDP_TX_BUF_SIZE];  // Double-Buffer
static uint8_t rxBufIndex = 0;                       // Aktueller Buffer-Index
static bool rxBufInUse[2] = {false, false};          // Buffer-Belegung

uint16_t rxSize = (size <= UDP_TX_BUF_SIZE) ? size : UDP_TX_BUF_SIZE;

// Naechsten freien Buffer waehlen
uint8_t nextBuf = (rxBufIndex + 1) % 2;
if(rxBufInUse[nextBuf])
{
    // Beide Buffer belegt -- muss ueberschreiben
    Serial.printf("[MC-DBG] RX_BUF_OVERWRITE buf=%d (still in use)\n", nextBuf);
}
else if(bLORADEBUG)
{
    Serial.printf("[MC-DBG] RX_BUF_SWITCH buf=%d->%d\n", rxBufIndex, nextBuf);
}

rxBufIndex = nextBuf;
rxBufInUse[rxBufIndex] = true;
memcpy(rxPayloadCopy[rxBufIndex], payload, rxSize);
payload = rxPayloadCopy[rxBufIndex];
size = rxSize;
Radio.Rx(RX_TIMEOUT_VALUE);
```

**Am Ende von OnRxDone() (vor `is_receiving = false`):**
```cpp
#if defined BOARD_RAK4630
    rxBufInUse[rxBufIndex] = false;
    if(bLORADEBUG)
        Serial.printf("[MC-DBG] RX_BUF_RELEASE buf=%d\n", rxBufIndex);
#endif
```

**Integration mit Aenderung 8 (OnRxDone-Split):**

Da Aenderung 8 den ACK-Pfad in `handleACK()` extrahiert hat, wird die
Buffer-Release-Logik an ZWEI Stellen benoetigt:

1. Im `handleACK()`-Early-Return-Pfad in OnRxDone:
```cpp
if(handleACK(payload, size, rssi, snr))
{
#if defined BOARD_RAK4630
    rxBufInUse[rxBufIndex] = false;
#endif
    is_receiving = false;
    // ...
    return;
}
```

2. Am Ende von OnRxDone (nach handleRegularMessage-Verarbeitung):
```cpp
#if defined BOARD_RAK4630
    rxBufInUse[rxBufIndex] = false;
    if(bLORADEBUG)
        Serial.printf("[MC-DBG] RX_BUF_RELEASE buf=%d\n", rxBufIndex);
#endif
    is_receiving = false;
```

### Commit-Message
```
feat: double-buffer RX payload with debug logging (RAK4630)

Replace single static rxPayloadCopy with double-buffer to prevent
overwrite of unprocessed packets. Add debug logging to track buffer
usage and detect overwrite conditions. This helps diagnose suspected
packet loss under high network load.
```

---

## Aenderung 5: Magic Numbers durch #define ersetzen

### Ausgangslage

Message-Type-Werte (0x41, 0x3A, 0x21, 0x40) und Ringpuffer-Status-Bytes
(0x00, 0x01, 0xFF) werden als rohe Hex-Werte quer durch den Code verwendet.
Das erschwert die Lesbarkeit und erhoht die Fehleranfaelligkeit (26+ Vorkommen
Message-Type, 15+ Vorkommen Ringpuffer-Status in `lora_functions.cpp`).

### Zielzustand

Benannte Konstanten in `configuration_global.h`, Verwendung ueberall im Code.

### Aenderungen

**Datei: `src/configuration_global.h` (nach CSMA_MAX_ATTEMPTS):**
```cpp
// LoRa Message Types
#define MSG_TYPE_ACK          0x41
#define MSG_TYPE_TEXT         0x3A
#define MSG_TYPE_POSITION     0x21
#define MSG_TYPE_HEY          0x40

// Ring Buffer Slot Status (ringBuffer[slot][1])
#define RING_STATUS_READY     0x00   // Ready to send
#define RING_STATUS_SENT      0x01   // Sent, waiting for ACK/timer
#define RING_STATUS_DONE      0xFF   // Final, no retransmission
```

**Datei: `src/lora_functions.cpp` -- Message-Type Ersetzungen:**

| Kontext | Alt | Neu |
|---------|-----|-----|
| OnRxDone ACK-Check | `payload[0] == 0x41` | `payload[0] == MSG_TYPE_ACK` |
| ACK to Phone | `print_buff[5] = 0x41` | `print_buff[5] = MSG_TYPE_ACK` |
| Position-Check | `msg_type_b_lora == 0x21` | `msg_type_b_lora == MSG_TYPE_POSITION` |
| Text-Check | `msg_type_b_lora == 0x3A` | `msg_type_b_lora == MSG_TYPE_TEXT` |
| DM-ACK print_buff | `print_buff[0]=0x41` | `print_buff[0] = MSG_TYPE_ACK` |
| GW-ACK print_buff | `print_buff[5]=0x41` / `print_buff[0]=0x41` | `= MSG_TYPE_ACK` |
| switch cases | `case 0x3A:` / `case 0x21:` / `case 0x40:` | `case MSG_TYPE_TEXT:` etc. |
| Typ-Pruefungen | `== 0x3A \|\| == 0x21 \|\| == 0x40` | `== MSG_TYPE_TEXT \|\| ...` |
| TX-Buffer ACK-Check | `lora_tx_buffer[0] == 0x41` | `lora_tx_buffer[0] == MSG_TYPE_ACK` |
| Slot-Clearing Typ-Check | `ringBuffer[...][2] != 0x3A` | `!= MSG_TYPE_TEXT` |
| Retransmit Typ-Check | `ringBuffer[...][2] != 0x3A` / `== 0x3A` | `!= MSG_TYPE_TEXT` / `== MSG_TYPE_TEXT` |

**Datei: `src/lora_functions.cpp` -- Ringpuffer-Status Ersetzungen:**

| Kontext | Alt | Neu |
|---------|-----|-----|
| Alle `ringBuffer[...][1] = 0xFF` | `= 0xFF` | `= RING_STATUS_DONE` |
| Alle `ringBuffer[...][1] != 0xFF` | `!= 0xFF` | `!= RING_STATUS_DONE` |
| doTX Mark-as-sent | `ringBuffer[iRead][1] == 0x00` → `= 0x01` | `== RING_STATUS_READY` → `= RING_STATUS_SENT` |
| Retransmit timer check | `!= 0x00 && != 0xFF` | `!= RING_STATUS_READY && != RING_STATUS_DONE` |
| Retransmit copy | `= 0x01` / `= 0xFF` | `= RING_STATUS_SENT` / `= RING_STATUS_DONE` |
| save_ring_status Check | `== 0xFF` | `== (char)RING_STATUS_DONE` |

**Implementierungsnotiz:** Bei `save_ring_status` (Typ `char`) wird ein expliziter
Cast `(char)RING_STATUS_DONE` verwendet, da `RING_STATUS_DONE` (0xFF) als `int`
einen Sign-Extension-Vergleich verursachen koennte.

**Datei: `src/esp32/esp32_main.cpp` -- RING_STATUS Report:**

```
ringBuffer[i][1] == 0xFF  -> == RING_STATUS_DONE
ringBuffer[i][1] == 0x00  -> == RING_STATUS_READY
```

### Commit-Message
```
refactor: replace magic numbers with named constants

Introduce MSG_TYPE_ACK/TEXT/POSITION/HEY and RING_STATUS_READY/
SENT/DONE constants in configuration_global.h. Replace all raw
hex values (0x41, 0x3A, 0x21, 0x40, 0xFF, 0x01) in
lora_functions.cpp and esp32_main.cpp. Pure refactoring, no
functional change.
```

---

## Aenderung 6: Dreifach duplizierte Ringpuffer-Suche extrahieren

### Ausgangslage

Drei Schleifen in `lora_functions.cpp` suchen im Ringpuffer nach einer
Message-ID durch manuelles Zusammenbauen einer 4-Byte-ID:

```cpp
unsigned int ring_msg_id = (ringBuffer[ircheck][6]<<24) |
                           (ringBuffer[ircheck][5]<<16) |
                           (ringBuffer[ircheck][4]<<8)  |
                            ringBuffer[ircheck][3];
```

**Vorkommen:**
- Zeilen 180-201 (ACK stoppt Retransmission) -- nutzt `==`-Vergleich
- Zeilen 242-265 (RX-Deduplizierung) -- nutzt `memcmp`, released zusaetzlich den Slot
- Zeilen 560-581 (DM-ACK Handling) -- nutzt `==`-Vergleich

Loop 1 und 3 sind strukturell identisch (uint32_t-Vergleich, Status setzen, retryCount
zuruecksetzen). Loop 2 ist anders: nutzt `memcmp`, released den Slot zusaetzlich, und
Debug-Werte muessen VOR dem Release gelesen werden.

### Zielzustand

Zwei Helper-Funktionen: `extractRingMsgId(int slot)` fuer die ID-Extraktion und
`findAndStopRingSlot(uint32_t msgId)` fuer die Suche per uint32_t-Vergleich.
Loop 1 und Loop 3 werden durch `findAndStopRingSlot()` ersetzt.
Loop 2 bleibt inline (Debug-Werte vor Release lesen), nutzt aber `extractRingMsgId()`
und die benannten Konstanten aus Aenderung 5.

### Aenderungen

**Datei: `src/lora_functions.cpp` (neue Funktionen, vor OnRxDone):**
```cpp
/**
 * Extract the 4-byte message ID from a ring buffer slot.
 * ringBuffer layout: [0]=len, [1]=status, [2]=msg_type, [3..6]=msg_id (LE)
 */
static uint32_t extractRingMsgId(int slot)
{
    return ((uint32_t)ringBuffer[slot][6] << 24) |
           ((uint32_t)ringBuffer[slot][5] << 16) |
           ((uint32_t)ringBuffer[slot][4] << 8)  |
            (uint32_t)ringBuffer[slot][3];
}

/**
 * Find and stop retransmission of a message by uint32_t msg_id.
 * Sets slot status to RING_STATUS_DONE and clears retryCount.
 * Returns slot index, or -1 if not found.
 */
static int findAndStopRingSlot(uint32_t msgId)
{
    for(int i = 0; i < MAX_RING; i++)
    {
        if(ringBuffer[i][0] > 0 && ringBuffer[i][1] != RING_STATUS_DONE)
        {
            if(extractRingMsgId(i) == msgId)
            {
                ringBuffer[i][1] = RING_STATUS_DONE;
                retryCount[i] = 0;
                return i;
            }
        }
    }
    return -1;
}
```

**Loop 1 (ACK -> Stop Retransmit):**
```cpp
// --- AKTUELL: 22-Zeilen Loop ---
// --- NEU: ---
int ackSlot = findAndStopRingSlot(msg_id);
if(ackSlot >= 0 && bDisplayRetx)
    Serial.printf("\n[RETX] binary ACK for retid:%i stop retransmit msg-id:%08X\n",
                  ackSlot, msg_id);
```

**Loop 2 (RX Dedup) -- bleibt inline, nutzt extractRingMsgId():**

**Implementierungsabweichung:** Debug-Werte (`dbg_status`, `dbg_lng`, `dbg_type`)
werden VOR dem Release gelesen, und der Slot wird erst danach freigegeben.
Ausserdem wurde der `ring_msg_id`-Vergleich von `unsigned int` auf `uint32_t`
(via `extractRingMsgId()`) geaendert.

```cpp
uint32_t dbg_msg_id = 0;
int rxSlot = -1;
for(int i = 0; i < MAX_RING; i++)
{
    if(ringBuffer[i][0] > 0 && ringBuffer[i][1] != RING_STATUS_DONE)
    {
        if(memcmp(ringBuffer[i]+3, RcvBuffer+1, 4) == 0)
        {
            rxSlot = i;
            dbg_msg_id = extractRingMsgId(i);
            break;
        }
    }
}
if(rxSlot >= 0)
{
    uint8_t dbg_status = ringBuffer[rxSlot][1];
    uint8_t dbg_lng = ringBuffer[rxSlot][0];
    uint8_t dbg_type = ringBuffer[rxSlot][2];

    ringBuffer[rxSlot][1] = RING_STATUS_DONE;
    retryCount[rxSlot] = 0;
    ringBuffer[rxSlot][0] = 0;

    if(bDisplayRetx)
        Serial.printf("\n[RETX] got lora rx for retid:%i no need status:%02X lng;%i msg-id:%c-%08X\n",
                      rxSlot, dbg_status, dbg_lng, dbg_type, dbg_msg_id);
    if(bLORADEBUG)
        Serial.printf("[MC-DBG] ACK_RECEIVED retid=%d msg_id=%08X\n",
                      rxSlot, dbg_msg_id);
}
```

**Loop 3 (DM ACK -> Stop Retransmit):**
```cpp
// --- AKTUELL: 22-Zeilen Loop ---
// --- NEU: ---
int dmSlot = findAndStopRingSlot(msg_counter);
if(dmSlot >= 0 && bDisplayRetx)
    Serial.printf("\n[RETX] DM-ACK for retid:%i stop retransmit msg-id:%08X\n",
                  dmSlot, msg_counter);
```

### Commit-Message
```
refactor: extract ring buffer search into helper functions

Add extractRingMsgId() and findAndStopRingSlot() helpers.
Replace Loop 1 (binary ACK) and Loop 3 (DM-ACK) with
findAndStopRingSlot(). Loop 2 (RX Dedup) stays inline because
debug values must be read before slot release, but uses
extractRingMsgId() and named constants. Reduces duplication,
no functional change.
```

---

## Aenderung 7: Doppelter APRS2SOTA-Check entfernen

### Ausgangslage

`lora_functions.cpp:832-836` enthaelt eine exakte Duplikation:

```cpp
832:  if(strcmp(destination_call, "APRS2SOTA") == 0)
833:      bMeshDestination = false;
834:  else
835:  if(strcmp(destination_call, "APRS2SOTA") == 0)
836:      bMeshDestination = false;
```

Der `else`-Zweig prueft exakt die gleiche Bedingung wie der `if`-Zweig.
Der `else`-Block ist toter Code -- er wird nie erreicht.

### Aenderungen

**Datei: `src/lora_functions.cpp`**
```
Zeile 834-836 entfernen:
  else
  if(strcmp(destination_call, "APRS2SOTA") == 0)
      bMeshDestination = false;
```

### Commit-Message
```
fix: remove duplicate APRS2SOTA destination check

The else-if branch at line 835 checks the exact same condition as
the if at line 832, making it unreachable dead code. Removed.
```

---

## Aenderung 8: OnRxDone() aufteilen

### Ausgangslage

`OnRxDone()` in `lora_functions.cpp:100-987` ist 888 Zeilen lang mit 6+
Verschachtelungsebenen -- der groesste Wartbarkeits-Blocker im Projekt. Die Funktion
laesst sich klar in logische Bloecke aufteilen:

1. **Preamble/Setup** (Zeilen 100-135): Buffer-Copy, RX-Restart, Variablen-Init
2. **ACK-Handling** (Zeilen 136-235): ACK-Pakete (0x41) verarbeiten
3. **Regulaere Nachrichten** (Zeilen 237-987)

Sektion B (ACK) und C (regulaer) sind **exklusiv** (if/else) -- kein Datenfluss
zwischen ihnen. Alle lokalen Variablen sind sektionslokal. Beide Sektionen koennen
als eigenstaendige Funktionen mit nur den OnRxDone()-Parametern extrahiert werden.

### Zielzustand (Phase 1 -- implementiert)

```
OnRxDone()           (~30 Zeilen Dispatcher)
  +-- handleACK()           ~90 Zeilen (extrahiert)
  +-- regulaere Nachrichten  ~700 Zeilen (inline, unveraendert)
  +-- cleanup
```

**Phase 2 (noch nicht umgesetzt):** handleRegularMessage() + handleTextMessage()
extrahieren. Aufgrund der Groesse (~700 Zeilen mit tiefer Verschachtelung) bewusst
auf einen separaten Commit verschoben.

### Aenderungen

**Datei: `src/lora_functions.cpp`**

**Schritt 1: handleACK() extrahieren**

Neue `static` Funktion VOR OnRxDone (und vor den Helper-Funktionen aus Aenderung 6):

```cpp
/**
 * Handle incoming ACK packet (msg_type 0x41).
 * Returns true if packet was processed as ACK, false otherwise.
 */
static bool handleACK(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    if(payload[0] != MSG_TYPE_ACK)
        return false;

    uint8_t print_buff[30];

    // ... kompletter Inhalt des ehemaligen if(payload[0] == 0x41) Blocks ...

    return true;
}
```

**Implementierungsabweichung gegenueber dem Plan:**

1. **`print_buff` wurde in `handleACK()` lokal deklariert.** Im Original war
   `print_buff[30]` in OnRxDone deklariert und wurde sowohl im ACK- als auch im
   regulaeren Pfad verwendet. Da die Sektionen exklusiv sind, hat `handleACK()`
   seine eigene `print_buff`-Deklaration bekommen. Die bestehende Deklaration in
   OnRxDone bleibt fuer den regulaeren Pfad erhalten.

2. **`bNewLine`-Handling vereinfacht.** Der Original-ACK-Block endete mit
   `if(bDisplayInfo && !bNewLine) { Serial.println(""); bNewLine = true; }`.
   In `handleACK()` wurde dies zu `if(bDisplayInfo) { Serial.println(""); }`
   vereinfacht, da `bNewLine` eine OnRxDone-lokale Variable ist und nach dem
   Return nicht mehr benoetigt wird.

3. **Kein `handleRegularMessage()` extrahiert.** Der Plan sah vor, auch den
   regulaeren Nachrichtenpfad (~700 Zeilen) in eine separate Funktion zu
   verschieben. Dies wurde bewusst auf Phase 2 verschoben, da die tiefe
   Verschachtelung und die vielen lokalen Variablen eine sorgfaeltige manuelle
   Pruefung erfordern.

4. **Kommentierter Code (`iWrite++` Bloecke) wurde in `handleACK()` nicht
   mitkopiert.** Die auskommentierten `iWrite++/if(iWrite >= MAX_RING)` Bloecke
   sind historische Ueberbleibsel und wurden beim Extrahieren bewusst weggelassen.

**In OnRxDone() (nach Buffer-Copy/RX-Restart):**
```cpp
bLED_GREEN = true;

if(handleACK(payload, size, rssi, snr))
{
#if defined BOARD_RAK4630
    rxBufInUse[rxBufIndex] = false;
#endif
    is_receiving = false;

    // Debug I: ONRXDONE_TIME
    if(bLORADEBUG)
        Serial.printf("[MC-DBG] ONRXDONE_TIME ms=%lu\n", millis() - _onrxdone_start);

    iReceiveTimeOutTime = millis();
    csma_timeout = csma_compute_timeout(cad_attempt);
    return;
}

{
    // regulaerer Nachrichtenpfad (unveraendert)
    memcpy(RcvBuffer, payload, size);
    // ...
}
```

**Implementierungsnotiz:** Der Early-Return im ACK-Pfad dupliziert die Cleanup-
Logik (Buffer-Release, is_receiving, Timer-Reset). Dies ist bewusst so, um den
regulaeren Pfad am Ende von OnRxDone nicht zu veraendern. Bei Phase 2 kann dies
konsolidiert werden.

### Commit-Message
```
refactor: extract handleACK() from OnRxDone() (Phase 1)

Extract ACK handling (~90 lines) into a separate static function.
OnRxDone() calls handleACK() first and returns early on ACK packets.
Regular message processing remains inline (Phase 2).

No functional change, pure structural refactoring. Reduces nesting
in the ACK path from 6+ to 3-4 levels.
```

---

## Aenderung 9: Ring Buffer Diagnostic Logging (Paketverlust-Diagnose)

### Ausgangslage

Es treten Paketverluste auf, deren Ursache nicht lokalisierbar ist. Luecken im
aktuellen Debug-Output:

1. **Kein Log beim Ueberschreiben aktiver Slots** -- msg_id und Status gehen verloren
2. **Kein Log beim Schreiben in den TX-Ringpuffer** -- msg_id, Slot-Nr, Fuellstand fehlen
3. **Kein Log im RX-Dedup-Puffer** -- Duplikat-Entscheidungen nicht nachvollziehbar
4. **Kein Log beim Senden** -- doTX() loggt Laenge aber nicht msg_id
5. **Dedup-Fuellstand fehlt im periodischen Report**
6. **CRC-Fehler: RSSI/SNR werden NACH startReceive() gelesen** -- Register bereits ungueltig
7. **"Other error" in checkRX() hat keinen RX-Restart** -- Radio bleibt im Standby (Blindspot!)

### Zielzustand

Lueckenloses Tracing einer Nachricht durch alle Puffer:
```
RX_DEDUP_ADD -> RING_WRITE -> RING_TX_READ -> RADIO_TX
                                              -> RING_DROP (bei Overflow)
RX_DEDUP_DUP (bei Duplikat-Erkennung)
```

Alles hinter `bLORADEBUG` geschuetzt (ausser RING_DROP und RX_OTHER_ERROR -- immer
aktiv, da kritisch). Kein Impact auf Normalbetrieb.

### Aenderungen

**Schritt 1: Zentraler TX-Ring-Write-Helper**

**Datei: `src/lora_functions.cpp` (neue Funktion, vor doTX()):**
```cpp
/**
 * Log + advance TX ring buffer write pointer.
 * Replaces direct addRingPointer(iWrite, iRead, MAX_RING, "tx") calls.
 * @param source  Short label for the calling code path (e.g. "rx_relay", "udp")
 */
void addTxRingEntry(const char* source)
{
    if(bLORADEBUG)
    {
        uint32_t mid = ((uint32_t)ringBuffer[iWrite][6] << 24) |
                       ((uint32_t)ringBuffer[iWrite][5] << 16) |
                       ((uint32_t)ringBuffer[iWrite][4] << 8)  |
                        (uint32_t)ringBuffer[iWrite][3];
        int queued = (iWrite >= iRead) ? (iWrite - iRead)
                                       : (MAX_RING - iRead + iWrite);
        Serial.printf("[MC-DBG] RING_WRITE slot=%d type=%02X status=%02X "
                      "len=%d msg_id=%08X queued=%d/%d src=%s\n",
                      iWrite, ringBuffer[iWrite][2], ringBuffer[iWrite][1],
                      ringBuffer[iWrite][0], mid, queued, MAX_RING, source);
    }

    // Overflow-Vorwarnung
    int nextWrite = iWrite + 1;
    if(nextWrite >= MAX_RING) nextWrite = 0;
    if(nextWrite == iRead && ringBuffer[iRead][0] > 0)
    {
        uint32_t lost_id = ((uint32_t)ringBuffer[iRead][6] << 24) |
                           ((uint32_t)ringBuffer[iRead][5] << 16) |
                           ((uint32_t)ringBuffer[iRead][4] << 8)  |
                            (uint32_t)ringBuffer[iRead][3];
        // IMMER loggen -- Paketverlust ist kritisch
        Serial.printf("[MC-DBG] RING_DROP slot=%d type=%02X status=%02X "
                      "msg_id=%08X retry=%d (overwritten by %s)\n",
                      iRead, ringBuffer[iRead][2], ringBuffer[iRead][1],
                      lost_id, retryCount[iRead], source);
    }

    addRingPointer(iWrite, iRead, MAX_RING, "tx");
}
```

**Datei: `src/loop_functions.h` -- Deklaration hinzugefuegt:**
```cpp
void addTxRingEntry(const char* source);
```

**Alle 14 `addRingPointer(iWrite, iRead, MAX_RING, "tx")` Aufrufe ersetzt:**

| Datei | Source-Label |
|-------|--------------|
| `lora_functions.cpp` (ACK forward) | `"rx_ack_fwd"` |
| `lora_functions.cpp` (DM ACK GW) | `"rx_dm_ack_gw"` |
| `lora_functions.cpp` (DM ACK new) | `"rx_dm_ack_new"` |
| `lora_functions.cpp` (relay) | `"rx_relay"` |
| `lora_functions.cpp` (retransmit) | `"retransmit"` |
| `udp_functions.cpp` | `"udp_rx"` |
| `loop_functions.cpp` (user msg) | `"user_msg"` |
| `loop_functions.cpp` (user pos) | `"user_pos"` |
| `loop_functions.cpp` (user wx) | `"user_wx"` |
| `loop_functions.cpp` (user hey) | `"user_hey"` |
| `loop_functions.cpp` (beacon) | `"beacon"` |
| `loop_functions.cpp` (auto pos) | `"auto_pos"` |
| `loop_functions.cpp` (phone msg) | `"phone_msg"` |
| `loop_functions.cpp` (phone raw) | `"phone_raw"` |

**Schritt 2: doTX() -- msg_id beim Senden loggen**

**Datei: `src/lora_functions.cpp` -- in doTX(), nach `save_ring_status`:**
```cpp
if(bLORADEBUG)
{
    uint32_t tx_mid = ((uint32_t)ringBuffer[iRead][6] << 24) |
                      ((uint32_t)ringBuffer[iRead][5] << 16) |
                      ((uint32_t)ringBuffer[iRead][4] << 8)  |
                       (uint32_t)ringBuffer[iRead][3];
    int queued = (iWrite >= iRead) ? (iWrite - iRead)
                                   : (MAX_RING - iRead + iWrite);
    Serial.printf("[MC-DBG] RING_TX_READ slot=%d type=%02X status=%02X "
                  "len=%d msg_id=%08X retry=%d queued=%d/%d\n",
                  iRead, ringBuffer[iRead][2], ringBuffer[iRead][1],
                  sendlng, tx_mid, retryCount[iRead], queued, MAX_RING);
}
```

**Schritt 3: RX-Dedup-Puffer Logging**

**Datei: `src/loop_functions.cpp` -- `addLoraRxBuffer()`:**
```cpp
void addLoraRxBuffer(unsigned int msg_id, bool bserver)
{
    if(bLORADEBUG)
        Serial.printf("[MC-DBG] RX_DEDUP_ADD msg_id=%08X srv=%d slot=%d/%d\n",
                      msg_id, bserver, loraWrite, MAX_DEDUP_RING);
    // ... bestehender Code ...
```

**Datei: `src/lora_functions.cpp` -- `is_new_packet()`:**
```cpp
bool is_new_packet(uint8_t compBuffer[4])
{
    for(int ib=0; ib<MAX_DEDUP_RING; ib++)
    {
            if (memcmp(compBuffer, ringBufferLoraRX[ib], 4) == 0)
            {
                if(bLORADEBUG)
                {
                    uint32_t dup_id = (uint32_t)compBuffer[0] |
                                      ((uint32_t)compBuffer[1] << 8) |
                                      ((uint32_t)compBuffer[2] << 16) |
                                      ((uint32_t)compBuffer[3] << 24);
                    Serial.printf("[MC-DBG] RX_DEDUP_DUP msg_id=%08X slot=%d\n",
                                  dup_id, ib);
                }
                return false;
            }
    }

    if(bLORADEBUG)
    {
        uint32_t new_id = (uint32_t)compBuffer[0] |
                          ((uint32_t)compBuffer[1] << 8) |
                          ((uint32_t)compBuffer[2] << 16) |
                          ((uint32_t)compBuffer[3] << 24);
        Serial.printf("[MC-DBG] RX_DEDUP_NEW msg_id=%08X\n", new_id);
    }
    return true;
}
```

**Schritt 4: Periodischen RING_STATUS um Dedup-Fuellstand erweitern**

**Datei: `src/esp32/esp32_main.cpp` -- RING_STATUS Block:**
```cpp
int dedup_used = 0;
for(int i = 0; i < MAX_DEDUP_RING; i++)
{
    if(ringBufferLoraRX[i][0] != 0 || ringBufferLoraRX[i][1] != 0 ||
       ringBufferLoraRX[i][2] != 0 || ringBufferLoraRX[i][3] != 0)
        dedup_used++;
}
Serial.printf("[MC-DBG] RING_STATUS queued=%d pending=%d retrying=%d "
              "done=%d iW=%d iR=%d dedup=%d/%d\n",
              queued, pending, retrying, done, iWrite, iRead,
              dedup_used, MAX_DEDUP_RING);
```

**Schritt 5: CRC-Fehler -- RSSI/SNR vor RX-Restart sichern + Payload-Hex-Dump**

**Datei: `src/esp32/esp32_main.cpp` -- checkRX(), CRC-Fehler-Pfad:**

**Implementierungsabweichung:** Das Dokument sah vor, den bestehenden CRC-Error-Block
komplett zu ersetzen. Tatsaechlich wurde der Block umstrukturiert:
- RSSI/SNR/FreqError werden jetzt VOR `startReceive()` gesichert
  (vorher wurden sie NACH dem Restart gelesen -- Register ungueltig)
- `startReceive()`-Rueckgabewert wird jetzt geloggt
- Hex-Dump des beschaedigten Payloads hinzugefuegt
- Variablen umbenannt zu `saved_crc_rssi`/`saved_crc_snr`/`saved_crc_ferr`
  um Namenskonflikte mit dem OK-Pfad zu vermeiden

```cpp
else if (state == RADIOLIB_ERR_CRC_MISMATCH)
{
    // RSSI/SNR/FreqError VOR RX-Restart sichern
    int16_t saved_crc_rssi = (int16_t)radio.getRSSI();
    int8_t  saved_crc_snr  = (int8_t)radio.getSNR();
    float   saved_crc_ferr = radio.getFrequencyError();

    // RX sofort wieder starten
    {
        radio.clearPacketReceivedAction();
        radio.clearPacketSentAction();
        bEnableInterruptReceive = true;
        radio.setPacketReceivedAction(setFlagReceive);
        int rxstate = radio.startReceive(...);
        if(bLORADEBUG)
            Serial.printf("[MC-DBG] RX_RESTARTED src=after_crc_error state=%d\n", rxstate);
    }

    ch_util_rx_accum += radio.getTimeOnAir(ibytes) / 1000;

    if(bLORADEBUG)
    {
        Serial.printf("[MC-DBG] CRC_ERROR rssi=%d snr=%d freq_err=%.1f size=%d ts=%lu\n",
            saved_crc_rssi, saved_crc_snr, saved_crc_ferr, (int)ibytes, millis());
        int dump_len = (ibytes > 255) ? 255 : (int)ibytes;
        Serial.printf("[MC-DBG] CRC_PAYLOAD[%d]: ", dump_len);
        for(int i = 0; i < dump_len; i++)
            Serial.printf("%02X ", payload[i]);
        Serial.println();
    }
}
```

**Schritt 6: "Other error" Blindspot -- RX-Restart + Diagnose-Logging**

Im "other error"-Pfad von checkRX() **fehlte der RX-Restart komplett!**
Nach diesem Pfad blieb das Radio im Standby -- es empfing nichts mehr bis zum
naechsten TX-Zyklus. Dies ist ein **echter RX-Blindspot** und potentielle Ursache
fuer unerklaeliche Paketverluste.

**Implementierungsabweichung:** Variablen umbenannt zu `saved_err_rssi`/
`saved_err_snr` um Namenskonflikte zu vermeiden.

```cpp
else
{
    // RX-Restart auch bei unbekannten Fehlern
    int16_t saved_err_rssi = (int16_t)radio.getRSSI();
    int8_t  saved_err_snr  = (int8_t)radio.getSNR();

    {
        radio.clearPacketReceivedAction();
        radio.clearPacketSentAction();
        bEnableInterruptReceive = true;
        radio.setPacketReceivedAction(setFlagReceive);
        int rxstate = radio.startReceive(...);
        if(bLORADEBUG)
            Serial.printf("[MC-DBG] RX_RESTARTED src=after_other_error state=%d\n", rxstate);
    }

    // Immer loggen -- unbekannte Fehler sind kritisch
    Serial.printf("[MC-DBG] RX_OTHER_ERROR code=%d rssi=%d snr=%d size=%d ts=%lu\n",
        state, saved_err_rssi, saved_err_snr, (int)ibytes, millis());

    if(bLORADEBUG && ibytes > 0)
    {
        int dump_len = (ibytes > 255) ? 255 : (int)ibytes;
        Serial.printf("[MC-DBG] ERR_PAYLOAD[%d]: ", dump_len);
        for(int i = 0; i < dump_len; i++)
            Serial.printf("%02X ", payload[i]);
        Serial.println();
    }
}
```

### Commit-Message
```
feat: add ring buffer diagnostic logging and fix RX blindspots

Add addTxRingEntry() wrapper that logs msg_id, slot, type, status,
and fill level on every TX ring buffer write. Log RING_DROP with
full details when an active slot is about to be overwritten.

Add RING_TX_READ logging in doTX() to trace which msg_id is sent.

Add RX_DEDUP_ADD/DUP/NEW logging in addLoraRxBuffer() and
is_new_packet() to trace deduplication decisions.

Extend periodic RING_STATUS with dedup ring fill level.

Fix CRC error path: save RSSI/SNR/FreqError BEFORE startReceive()
(registers become invalid after RX restart). Add hex dump of
CRC-failed payload for diagnosis.

Fix "other error" path: add missing startReceive() call -- without
this the radio stays in standby after unknown errors (RX blindspot).
Add error code + payload dump logging.

All logging behind bLORADEBUG except RING_DROP and RX_OTHER_ERROR
(always logged -- packet loss and unknown errors are critical).
```

---

## Reihenfolge der Commits

Die Aenderungen sind so geordnet, dass sie aufeinander aufbauen und
einzeln rebasebar sind:

| # | Aenderung | Abhaengigkeit | Risiko |
|---|-----------|---------------|--------|
| 9 | Ring Buffer Diagnostic Logging | Keine | Minimal |
| 0 | RadioLib 7.1.2 -> 7.6.0 | Keine | Niedrig |
| 1 | volatile Flags | Keine (+ batt_functions.cpp Fix) | Minimal |
| 7 | APRS2SOTA Duplikat | Keine | Minimal |
| 5 | Magic Numbers -> #define | Keine | Minimal |
| 3 | Atomarer RX-Restart | Keine | Niedrig |
| 4 | RX-Payload Double-Buffer | Keine | Niedrig |
| 6 | Ringpuffer-Helper | Setzt #5 voraus (nutzt RING_STATUS_DONE) | Niedrig |
| 2 | ESP32 Header Detection (IRQ-Polling) | Setzt #0 voraus (RadioLib 7.6.0 fuer Bugfixes) | Niedrig |
| 8 | OnRxDone() Split (Phase 1 only) | Setzt #5 und #6 voraus | Mittel |

**Empfohlene Reihenfolge:** 9 -> 0 -> 1 -> 7 -> 5 -> 3 -> 4 -> 6 -> 2 -> 8

**Implementierungsnotiz:** Alle Aenderungen wurden in einem Durchgang implementiert
(nicht einzeln committet). Build-Verifikation erfolgte fuer `heltec_wifi_lora_32_V3`
und `E22-DevKitC`.

---

## Implementierungsabweichungen (Zusammenfassung)

| Aenderung | Abweichung | Grund |
|-----------|-----------|-------|
| 1 | `batt_functions.cpp` zusaetzlich geaendert | Eigene `extern bool is_receiving;` Deklaration verursachte Compile-Fehler |
| 2 | `break;` -> `bool irq_rx_active` Flag | Kein umschliessender Loop/Switch vorhanden, `break;` waere nicht kompilierbar |
| 5 | `save_ring_status == (char)RING_STATUS_DONE` | Cast noetig wegen `char` vs. `int` Vergleich (0xFF als signed char) |
| 5 | Auch `esp32_main.cpp` RING_STATUS Report geaendert | War im Original-Dokument nicht als Ersetzungsziel aufgefuehrt |
| 8 | Nur Phase 1 (handleACK) implementiert | handleRegularMessage/handleTextMessage bewusst auf Phase 2 verschoben |
| 8 | Auskommentierter Code nicht mitkopiert | Historische `iWrite++`-Bloecke beim Extrahieren weggelassen |
| 8 | bNewLine-Handling vereinfacht in handleACK | `bNewLine` ist OnRxDone-lokal, nicht in handleACK relevant |
| 9 | CRC/Error-Variablen umbenannt | `saved_crc_rssi`/`saved_err_rssi` statt generisch `saved_rssi` um Konflikte zu vermeiden |
