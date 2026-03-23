# nRF52 BLE: Falsche Bandwidth / Coding Rate im WebApp

**Datum:** 2026-03-21
**Branch:** v4.35p_prio
**Plattform:** nRF52840 (RAK4631)
**Symptom:** WebApp zeigt Bandwidth = 1 kHz (statt 250) und Coding Rate = 2 (statt 6) bei EU8

---

## Problem

Die Funktion `sendNodeSetting()` in `src/command_functions.cpp` (Zeile 4729–4730) sendet die
rohen `meshcom_settings`-Werte per BLE-JSON an die WebApp:

```c
nsetdoc["MCR"] = meshcom_settings.node_cr;   // RAK: encoded 2
nsetdoc["MBW"] = meshcom_settings.node_bw;   // RAK: encoded 1
```

Auf **ESP32** werden Bandwidth und Coding Rate als physikalische Werte gespeichert
(`node_bw = 250.0`, `node_cr = 6`), daher stimmt die Anzeige.

Auf **nRF52 (RAK4631)** verwendet die SX126x-Library jedoch einen **Index-basierten Encoding**:

| Setting      | RAK-Rohwert | Bedeutung       | WebApp zeigt |
|--------------|-------------|-----------------|--------------|
| `node_bw`    | `0`         | 125.0 kHz       | 0 kHz        |
| `node_bw`    | `1`         | 250.0 kHz       | **1 kHz**    |
| `node_bw`    | `2`         | 500.0 kHz       | 2 kHz        |
| `node_cr`    | `1`         | CR 4/5          | 1            |
| `node_cr`    | `2`         | CR 4/6          | **2**        |
| `node_cr`    | `3`         | CR 4/7          | 3            |
| `node_cr`    | `4`         | CR 4/8          | 4            |

Die Konvertierung existiert bereits in `src/lora_setchip.cpp`:

- **`getBW()`** (Zeile 98–117) — wandelt RAK-Index → kHz-Wert
- **`getCR()`** (Zeile 128–149) — wandelt RAK-Index → CR-Wert (5–8)

Diese Funktionen werden aber beim BLE-Versand nicht verwendet.

## Ursache

Country-Profile (z.B. EU8) in `lora_setchip.cpp` setzen plattformspezifische Werte:

```c
case 8:  // EU8
    #if defined BOARD_RAK4630
        meshcom_settings.node_bw = 1;      // Index für 250 kHz
        meshcom_settings.node_cr = 2;      // Index für CR 4/6
    #else
        meshcom_settings.node_bw = 250.0;  // Direkt kHz
        meshcom_settings.node_cr = 6;      // Direkt CR
    #endif
```

Die `sendNodeSetting()`-Funktion gibt diese Rohwerte ohne Dekodierung weiter.

## Fix

In `src/command_functions.cpp`, `sendNodeSetting()`:

```diff
- nsetdoc["MCR"] = meshcom_settings.node_cr;
- nsetdoc["MBW"] = meshcom_settings.node_bw;
+ nsetdoc["MCR"] = getCR();
+ nsetdoc["MBW"] = getBW();
```

Damit werden auf allen Plattformen die physikalischen Werte gesendet.
