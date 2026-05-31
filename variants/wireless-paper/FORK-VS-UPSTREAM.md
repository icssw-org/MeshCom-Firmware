# Wireless Paper – Fork vs. Upstream-PR

> **Stand aktuell: Boot-Screen vereinheitlicht → beide Branches sind code-identisch.**
> Der Start-Screen nutzt in **beiden** Branches das **unveränderte Original-E290-Layout**
> (kein eigenes Layout, kein „Heltec Wireless Paper", **kein OE3LCR** im Code). Die zur
> Laufzeit erkannte Display-Version wird ausschließlich als **DEBUG-Zeile am Terminal**
> ausgegeben (`initDisplay()`), nicht am Display. Damit ist die Maintainer-Vorgabe
> „Boot-Screen nicht verändern" erfüllt, und das persönliche Rufzeichen ist nirgends
> mehr im Code.

Maintainer-Vorgabe für die Aufnahme in den **icssw-org `dev`**-Branch:

> **Bilddarstellungen nur an die Hardware anpassen, nicht inhaltlich verändern** –
> insbesondere den **Start-Screen nicht** ändern.

Die display-bezogenen Lesbarkeits-Anpassungen (FreeSans-Schrift, kompaktes Layout,
149-Zeichen-Anzeige, laufende Uhr, Voll-Refresh gegen Ghosting) gelten als
**Hardware-Anpassung an das kleinere 2.13"-Panel** und sind in **beiden** Branches
enthalten. Der Fork-Branch existiert weiter für künftige rein persönliche Anpassungen.

## Aufteilung

| Änderung | Kategorie | Fork | Upstream-PR |
|---|---|:--:|:--:|
| Display-Treiber E0213A367 + LCMEN2R13EFC1 | Hardware | ✅ | ✅ |
| Laufzeit-Erkennung (Chip-ID) + Polymorphie (`epaper_display`-Zeiger) | Hardware | ✅ | ✅ |
| Partial-Waveform-LUT (kein Ghosting) | Hardware | ✅ | ✅ |
| Variante (Pins, Frequenz, HAS_EPAPER), Build-Config, CI-Release | Hardware | ✅ | ✅ |
| HW-ID 57 + mheard-Tabelle (Kollision 45 behoben) | Hardware | ✅ | ✅ |
| Terminal-Fix (`ARDUINO_USB_CDC_ON_BOOT` raus → UART/CP2102) | Hardware | ✅ | ✅ |
| additive `BOARD_WIRELESS_PAPER`-Guards + `d_dir_to`-Bugfix | Hardware | ✅ | ✅ |
| | | | |
| Boot-Screen: **unverändertes Original-E290-Layout** (kein OE3LCR) | Hardware | ✅ | ✅ |
| Kompaktes Layout / 149-Zeichen / `wpApplyLayout` (seiten-abhängig) | Display-HW | ✅ | ✅ |
| Rundliche FreeSans- statt FreeMono-Schrift | Display-HW | ✅ | ✅ |
| Laufende Uhr (`wpRefreshClock`, 10-s-Teilrefresh) | Display-HW | ✅ | ✅ |
| Trennlinien-Position | Display-HW | ✅ | ✅ |
| Voll-Refresh bei neuer Nachricht / GPS-Seite (kein Ghosting) | Display-HW | ✅ | ✅ |
| Batterie-GPIOs (GPIO20 + ADC_CTRL 19) + Onboard-LED GPIO18 | Hardware | ✅ | ✅ |

## Technische Zuordnung der `#if defined(BOARD_WIRELESS_PAPER)`-Blöcke

Alle `#if defined(BOARD_WIRELESS_PAPER)`-Blöcke gelten als **Hardware-/Panel-Anpassung**
und sind in **beiden** Branches gleich:

- `esp32_functions.cpp`: `detectEinkChipId()`, Treiberwahl in `initDisplay()`,
  `extern`-Zeiger + Makro, gemeinsamer `startDisplay`-Rahmen. Der Boot-Screen selbst ist
  **unverändert** (Original-E290-Pfad, kein eigener WP-Block mehr).
- `loop_functions.cpp`: `extern gps`-Guard, Display-Include/Instanziierung (Zeiger),
  negative OLED-Ausschluss-Guards, `dzeile`/`wpApplyLayout`, FreeSans-Schrift,
  Trennlinien-`y`, `wpRefreshClock` (10-s-Teilrefresh), kompakte Nachrichtenschrift,
  Voll-Refresh in `sendDisplayText`/`sendDisplayPosition`/`sendDisplayTrack`.
- `variants/wireless-paper/configuration.h`: Pins (inkl. Batterie GPIO20/ADC_CTRL 19,
  LED GPIO18), Frequenz, `HAS_EPAPER`.

Die zur Laufzeit erkannte Display-Version (E0213A367 vs LCMEN2R13EFC1) erscheint nur als
**DEBUG-Zeile am Terminal**. Damit ist der `wireless-paper-upstream`-Branch direkt als
DEV-PR einreichbar; `wireless-paper` bleibt als persönlicher Branch bestehen.
