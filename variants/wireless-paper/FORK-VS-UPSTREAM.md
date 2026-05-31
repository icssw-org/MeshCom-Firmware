# Wireless Paper – Fork vs. Upstream-PR

Dieser Fork-Branch (`wireless-paper`) enthält **alle** Anpassungen inkl. eigener
Bilddarstellung und persönlichem Rufzeichen (OE3LCR). Für die Aufnahme in den
**icssw-org `dev`**-Branch gilt die Maintainer-Vorgabe:

> **Bilddarstellungen nur an die Hardware anpassen, nicht inhaltlich verändern.**

Deshalb existiert ein zweiter Branch (`wireless-paper-upstream`), der **nur die
Hardware-Anpassungen** enthält und die Bilddarstellung dem bestehenden E290-Pfad
(`HAS_EPAPER`) überlässt. Diese Datei dokumentiert die Aufteilung.

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
| Boot-Screen-Layout + „Heltec Wireless Paper" + **OE3LCR** | inhaltlich | ✅ | ❌ |
| Kompaktes Layout / 160-Zeichen / `wpApplyLayout` (seiten-abhängig) | inhaltlich | ✅ | ❌ |
| Rundliche FreeSans- statt FreeMono-Schrift | inhaltlich | ✅ | ❌ |
| Laufende Uhr (`wpRefreshClock`, 10-s-Teilrefresh) | inhaltlich | ✅ | ❌ |
| Trennlinien-Position | inhaltlich | ✅ | ❌ |
| Voll-Refresh bei neuer Nachricht | inhaltlich | ✅ | ❌ |
| Positions-/GPS-Anzeige-Anpassung (Voll-Refresh) | inhaltlich | ✅ | ❌ |

## Technische Zuordnung der `#if defined(BOARD_WIRELESS_PAPER)`-Blöcke

**Bleibt (Hardware):**
- `loop_functions.cpp`: `extern gps`-Guard, Display-Include/Instanziierung (Zeiger),
  negative OLED-Ausschluss-Guards
- `esp32_functions.cpp`: `detectEinkChipId()`, Treiberwahl in `initDisplay()`,
  `extern`-Zeiger + Makro, gemeinsamer `startDisplay`-Rahmen

**Raus für Upstream (inhaltlich, Fork behält sie):**
- `loop_functions.cpp`: `dzeile`/`wpApplyLayout`, Statuszeilen-Schrift (FreeSans),
  Trennlinien-`y`, `wpApplyLayout`-Aufruf in `sendDisplayMainline`, `wpRefreshClock`
  + 10-s-Aufruf, kompakte Nachrichtenschrift + Voll-Refresh in `sendDisplayText`,
  Voll-Refresh in `sendDisplayPosition`
- `esp32_functions.cpp`: der `#if defined(BOARD_WIRELESS_PAPER)`-Boot-Screen-Block in
  `startDisplay()` (Upstream nutzt den bestehenden E290-Boot-Screen)

Im Upstream läuft die Wireless Paper damit komplett über den bestehenden
`HAS_EPAPER`-Darstellungspfad – nur Treiber, Erkennung und Plattform sind neu.
