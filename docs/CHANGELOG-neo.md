# MeshCom neo -- Changelog

## Worum es hier geht

Dieser Branch ist `upstream/dev` plus besserer Code. Nichts sonst.

Er traegt keine neuen Protokollfunktionen und keine neuen Betriebsarten. Was er
traegt, ist das Ergebnis einer Aufraeum- und Vereinheitlichungskampagne: Code,
der an zwei oder an achtundzwanzig Stellen gleichzeitig stand, steht jetzt
einmal; tote Dateien sind weg; und an den Stellen, an denen die Doppelung
auseinandergelaufen war, sind dabei echte Fehler sichtbar geworden und behoben
worden.

**Das Versprechen: auf dem Draht verhaelt sich diese Firmware wie das offizielle
4.35t. Besser wird sie nur unter Last.** Es gibt keine gewollte Verhaltens-
aenderung gegenueber dem offiziellen Stand. Wo sich das Verhalten dennoch
unterscheidet, ist das eine Fehlerbehebung -- und sie ist hier benannt.

## Wie dieses Dokument zu lesen ist

Jeder Eintrag sagt, ob er eine **reine Restrukturierung** ist (gleiches
Verhalten, besser wartbarer Code) oder eine **Fehlerbehebung** (geaendertes
Verhalten, und zwar zum Besseren). Wo ein Nachweis existiert -- ein nativer
Test, ein Bench-Lauf auf einem echten Knoten, eine Messung -- steht er dabei,
mit dem Commit, der ihn belegt. Wo keiner existiert, steht das ebenfalls dort.
Ein duenner ehrlicher Eintrag ist uns lieber als ein gepolsterter.

Die Nummerierung laeuft durchgehend durch das ganze Dokument und ist eine
Nummerierung dieses Dokuments, nicht die Fortsetzung einer frueheren Zaehlung.

## Kapitel und Commits

Die siebzehn Kapitel erzaehlen die Arbeit. Der Code kommt in fuenf Commits, und
das ist kein Schnittfehler, sondern eine Eigenschaft des Codes: dreizehn der
Kapitel sind ueber Header, Typen und verschobene Funktionsdefinitionen so
verflochten, dass sie sich nicht einzeln uebersetzen lassen. Fuenf Projektions-
versuche mit jeweils anderer Reihenfolge haben das gezeigt, bevor es gerechnet
wurde; ueber alle Header des Deltas bilden diese dreizehn Kapitel eine einzige
stark zusammenhaengende Komponente.

Drei Beispiele, die das Muster tragen:

- `udp_drain_esp32.cpp` ruft `udpBeginRaw_esp32()` aus `udp_functions.cpp`, und
  dieselbe Datei braucht den Typwechsel von `aprsMessage`. Die Abhaengigkeit
  laeuft in beide Richtungen.
- Der C3-Carve-out hat `checkSerialCommand()` aus `esp32_main.cpp` in die neue
  `serial_command_esp32.cpp` verschoben. Liegen die beiden in verschiedenen
  Commits, ist die Funktion doppelt definiert und der Link bricht.
- `event_functions.cpp` liest `node_audio_start` als `String`, waehrend der
  Settings-Struct ihn schon als `char[128]` fuehrt.

| Commit       | Kapitel                                                              |
| ------------ | -------------------------------------------------------------------- |
| `neo K01`    | K01                                                                  |
| `neo` (Kern) | K02, K03, K04, K05, K06, K07, K08, K09, K10, K11, K12, K13, K14, K18 |
| `neo K15`    | K15                                                                  |
| `neo K16`    | K16                                                                  |
| `neo K17`    | K17                                                                  |

Jeder dieser fuenf Commits uebersetzt fuer sich auf allen acht Leitzielen.

Einzelne Aenderungen laufen quer zu den Kapiteln -- eine Typumstellung etwa
beruehrt fast jedes Subsystem. Solche Aenderungen werden in dem Kapitel
erzaehlt, in das ihre Substanz gehoert, und aus den uebrigen nur mit einem
Querverweis benannt, statt sie mehrfach zu wiederholen.

## Basis

Erstellt auf `upstream/dev`, Stand `4058b25b` (Merge von PR #1145). Die
Instrumentierung, die Kapitel K17 beschreibt, ist dort bereits enthalten; dieser
Branch aendert an ihr achtzehn Zeilen.

Seitdem dreimal nachgezogen: auf `80b85a5a` (PRs #1147 bis #1150, 2026-09-20),
auf `e4a2393f` (PRs #1151 bis #1153, #1155, #1156, 2026-09-25) und auf
`6cc8b552` (PRs #1157 bis #1161, 2026-09-26). Was upstream dabei gebracht hat,
verhaelt sich hier wie upstream; wo dieser Branch davon abweicht, steht es in
den beiden folgenden Abschnitten.

## Nachgezogen aus upstream/dev (2026-09-26)

Der Merge `43760732` (Basis-Merge `6cc8b552`, in der `dev`-Historie fuenf
Merge-Commits fuer die PRs #1157 bis #1161) bringt vier Bloecke: #1157 ist die
Portierung dieses Branches eigener RAM-Arbeit (Byte-FIFO-Ringe, MHeard-
Drosselfix, Web-Header als String, `int16_t`-Zeilenkoordinaten) ueber
`fork-main` zurueck nach upstream, zusammen mit eigenstaendigen `fork-main`-
Fixen; die meisten Hunks sind dieselbe Aenderung zweimal. #1158 (OE1KFR) ist
bereits seit dem 25. September als `f73cbf9e` auf diesem Branch und landet
jetzt unveraendert auch upstream. #1159 bis #1161 (OE1KBC) sind
Kommentaraenderungen fuer `4.35t`, die zwei MSB-Wiederholungsbits im `msg_id`
vorbereiten; die Maske selbst bleibt auskommentiert ("discussion ongoing").
Aufgeloest wurde ueberwiegend zugunsten der eigenen Struktur, mit vier
gezielten Uebernahmen aus upstream. Nachweis fuer den ganzen Merge: 36
Host-Umgebungen 1070/1070 Faelle, `test/golden/selftest.sh` gruen. Kein Board
hat dieses Zusammenfuehrungsergebnis gesehen.

**124. `sendPing()` meldet eine verweigerte TX-Ring-Eintragung jetzt laut statt
sie zu verschlucken.** (`43760732`, PR #1157). Upstreams Aenderung gibt
`sendPing()` einen `PingResult`-Rueckgabewert mit `PING_RING_REFUSED`, statt
den Aufruf wortlos ins Leere laufen zu lassen, wenn der TX-Ring das Ping nicht
annimmt; die Zeile geht als `[PING];...;not queued` auf die Konsole. Die
eigentliche Ring-Eintragung bleibt der neo-Pfad `addTxRingEntryOnce()` (P15,
`b55fe5d7`) -- nur die Fehlermeldung ist von upstream uebernommen.
Fehlerbehebung: aus stillem Verlust wird ein sichtbarer Fehler. Kein
Unterschied auf dem Draht, da hier nichts gesendet wird, das vorher gesendet
wurde. Nachweis: Host-Suite; kein Hardware-Nachweis fuer die Logzeile selbst.

**125. Der BLE-Kommandoring `RING_BYTES_PHONECOM` waechst auf 3072 B, auf
jeder Boardklasse gleich.** (`43760732`, PR #1157). Ziel ist, dass der
komplette Konfigurations-Burst (alle `SN`/`SN1`/`IS`/`IS1`-JSON-Antworten
zusammen) in den Ring passt, ohne dass eine Boardklasse mit einer kleineren
Ringgroesse Teile davon verwirft. Fehlerbehebung dem Zweck nach (verhindert
abgeschnittene Konfigurationsantworten ueber BLE), kostet aber Speicher: rund
+1 kB RAM je Boardklasse, auf keinem Board nachgemessen (siehe bekannte
Luecken). Nachweis: Host-Suite; kein Hardware-Nachweis.

**126. Neuer Diagnosemarker `[MC-DBG] RING_OVERFLOW buf=phone`.** (`43760732`,
PR #1157). Meldet einen Ueberlauf des Telefon-Kommandorings auf der Konsole,
so wie es fuer andere Ringe bereits existiert. Reine Restrukturierung
(Sichtbarkeit), kein Verhaltensunterschied auf dem Draht. Kein
Hardware-Nachweis, da der Marker nur bei einem tatsaechlichen Ueberlauf
feuert und ein solcher auf der Bank nicht provoziert wurde.

**127. Der Byte-Ring-Iterator fuer die Web-Nachrichtenseite nimmt seinen
Schnappschuss jetzt unter `BF_LOCK`.** (`43760732`, PR #1157). Vorher konnte
die Web-Historie waehrend des Iterierens von einem gleichzeitigen Schreiber
veraendert werden; der Snapshot wird jetzt unter dem bestehenden Lock
gezogen. Fehlerbehebung (verhindert eine inkonsistente Anzeige), betrifft nur
die Web-GUI, nichts auf dem Draht. Nachweis: Host-Suite; kein
Hardware-Nachweis fuer die Web-Anzeige selbst.

**128. `WSPWD` und `ASYM` stehen jetzt im zweiten Knoten-JSON `SN1` statt in
`SN`.** (`f73cbf9e`, OE1KFR, upstream als #1158; mit dem Merge vom
2026-09-26 neu auf diesem Branch). `SN` lag bei rund 250 Zeichen schon
ohne gesetztes Web-Passwort ueber `BLE_JSON_PAYLOAD_MAX` (244);
`bleJsonFrameFailSoft()` hat dann still Felder vom Ende her verworfen --
`ASYM` immer, `GWS` (das die App fuer die Gateway-Server-Auswahl braucht) ab
rund acht Passwortzeichen, `BLED` ab neunzehn. Mit `WSPWD` und `ASYM` in `SN1`
bleibt `SN` bei rund 228 (worst case 235) Zeichen, deutlich unter dem Limit.
Nach `--webpwd` per BLE sendet der Knoten jetzt `SN`/`SN1` erneut, damit die
App das neue Web-Passwort zeigt. Fehlerbehebung, betrifft nur die
BLE-Konfigurationsuebertragung zur App, nichts auf dem Draht. Nachweis:
Host-Suite; kein dedizierter Test und kein Hardware-Nachweis fuer die
App-Anzeige.

**129. Kommentare fuer zwei MSB-Wiederholungsbits im `msg_id`, ohne
Maskenaenderung.** (PRs #1159 bis #1161, OE1KBC, upstream `6cc8b552`). Die
Kommentare in `4.35t` bereiten vor, dass ein kommendes Protokoll zwei
hoechstwertige Bits der Nachrichten-ID fuer eine Wiederholungszaehlung
reserviert; die dazugehoerige Maske ist im upstream-Quelltext selbst
auskommentiert, laut Commit-Text "discussion ongoing". Reine
Restrukturierung ohne jede Codewirkung -- kein Verhaltensunterschied, daher
kein Nachweis noetig.

## Nachgezogen aus upstream/dev (2026-09-25)

Der Merge `def2dc7e` bringt die KISS/TCP-Schnittstelle (#1151, DH1FR), die
Via-Einstellungen als zweites Knoten-JSON `SN1` (#1155) und das Build-Datum als
zweites Info-JSON `IS1` (#1156); #1152 und #1153 sind eigene PRs und lagen hier
schon. Upstreams Code passt an vier Stellen nicht wortgleich auf diesen Branch
(`aprsMessage` traegt `char[]` statt `String`, die Ein/Aus-Kommandos stehen in
einer Tabelle, der UDP-Rahmenkoerper liegt in `udp_frame_esp32.cpp`); dort ist
er nach Absicht portiert, nicht nach Diff. Nachweis fuer den ganzen Merge: 36
Host-Umgebungen 1067/1067, 30 Board- und 2 Safeboot-Umgebungen gruen, KISS kostet
auf jedem ESP32 1224 bis 1240 B DRAM und 0 B IRAM.

**118. KISS/TCP laeuft auch auf E22_XML-DevKitC.** (`def2dc7e`). Upstream nimmt
das Board mit `-D DISABLE_KISS_TCP` aus, weil `dram0_0_seg` dort mit KISS um 32 B
ueberlaeuft (1160 B Reserve). Nach der RAM-Rueckgewinnung dieses Branches hat
das Board mit KISS noch 27 776 B DRAM- und unveraendert 4028 B IRAM-Reserve,
gemessen am sauberen Build vor und nach dem Merge. Abweichung von upstream:
ein Board mehr mit der Funktion.

**119. Eine per KISS eingespeiste Position schreibt nicht mehr jedes Mal den
Flash.** (`def2dc7e`). Upstreams `sendInjectedPosition()` zaehlt die
Nachrichten-ID von Hand hoch und ruft danach `save_settings()`. Hier laeuft sie
wie `SendAckMessage()` ueber `msgIdAdvance()` und schreibt nur an der
Hochwassermarke (`msgid_counter.h`, siehe Eintrag zu W3 in K08). Kein
Unterschied auf dem Draht; kein Hardware-Nachweis.

**120. Die Ack-Umschreibung fuer KISS-Clients kappt statt zu wachsen.**
(`def2dc7e`, Test `71502a94`). Upstream setzt die Antwort `:ackNN` per
`String`-Verkettung neu zusammen; hier ist die Nutzlast ein festes Feld mit
`MC_PAYLOAD_LEN`, und die neue Funktion `kissAckRewrite()` (`src/kiss_frame.cpp`)
kappt am Feldende. Im Betrieb nie erreicht (die Nummer hat hoechstens sieben
Zeichen). Die beiden reinen Funktionen `kissBuildAx25()` und `kissAckRewrite()`
sind dafuer aus `kiss_functions.cpp` herausgeloest und haben einen Host-Test mit
18 Faellen (`native_kiss_frame`), der die AX.25-Rahmen Byte fuer Byte prueft.

**121. `--via on` / `--via off` antworten dem Telefon mit `SN` und `SN1`.**
(`def2dc7e`, `0923e037`, Lint `71502a94`). Das ist upstreams Fix (die
Text-Antwort erschien in der App als Chatzeile); hier stehen die beiden Kommandos
in `COMMAND_TOGGLES[]`, deshalb waere der Fix beim Merge stumm verloren gegangen.
Die beiden Tabellenzeilen tragen jetzt `TG_DIRTY_NODE` statt `TG_BLE_ECHO`, und
der Aufrufer schickt `sendNodeSetting()` selbst. Der erste Anlauf ging ueber
`TG_BRETURN`; weil die Kommandoleiter keine durchgehende if/else-Kette ist, lief
die Eingabe dann weiter bis zur Argument-Stufe `via ` und speicherte "ON" als
Via-Rufzeichen -- auf der Bank an DK5EN-1 gefunden und dort nach der Korrektur
nachgemessen (`VIA` wechselt, `VIACALL` bleibt leer). `toggle_table_lint.py`
prueft beides seitdem (Pruefungen 8 und 9). Kein Unterschied zu upstream.

**122. Der KISS-Abgriff fuer Rahmen vom MeshCom-Server sitzt in
`handleUdpFrame_esp32()`.** (`def2dc7e`). Upstream hat ihn in
`getMeshComUDPpacket()`; dessen Koerper liegt hier seit dem C1/U1-Carve in
`udp_frame_esp32.cpp`, und der Merge hat den Abgriff ohne Konflikt verworfen.
Wieder eingesetzt direkt nach dem Dedup-Gatter, mit upstreams Bedingungen. Der
Zwillingstest `test_drift_kiss_server_relay_tap_is_esp32_only` schlaegt ohne
den Abgriff fehl und besteht mit ihm. Kein Unterschied zu upstream.

**123. Die zwei CS-01-`static_assert`s sind wieder da.** (`def2dc7e`). Sie
binden `MAXHOP_TEXT_FALLBACK` an `MAX_HOP_TEXT_DEFAULT` und das Kommandofenster
an `MAX_HOP_LIMIT`; der Kern-Commit dieses Branches hatte sie beim Verschieben
von `casecmp()` verloren, obwohl `maxhop.h` sie weiter nennt. Nur Uebersetzungszeit.

## Auf der Bank gemessen

Am 2026-09-19 lief ein Differenzlauf gegen `upstream/dev` auf zwei Knoten:
DK5EN-1 (Heltec V3, SX1262) und DK5EN-92 (T-Beam, SX1276), beide auf
433.175 MHz mit 1 bzw. 2 dBm, Gateway und Mesh aus. Mitgeschnitten wurde von
drei Seiten: beide Knoten seriell und DK5EN-98 als unabhaengiger Zeuge auf dem
Draht. Dasselbe Skript -- zwei Gruppennachrichten je Knoten und je eine
Direktnachricht in beide Richtungen -- lief zweimal je Firmware.

**Auf dem Draht ist kein Unterschied messbar.** Beide Firmwares erzeugen
dieselben acht Nutzlasten mit denselben Feldern: Rahmentyp, Hop-Flags, Quelle,
Ziel, Text, Hardware-Kennung, Modulation, Firmware-Kennung, letzter Hop. Die
Quittungen auf die Direktnachrichten kommen in beiden Faellen. Nur die
Reihenfolge, in der Quittungen relativ zu den Nachrichten eintreffen,
schwankt -- das sind Sekunden und haengt am Kanal, nicht an der Firmware; es
schwankt innerhalb einer Firmware genauso wie zwischen beiden.

**Im Knoten ist der Unterschied deutlich.** Freier Heap unmittelbar nach dem
Start, gemittelt ueber je zwei Laeufe:

| Knoten              | upstream/dev | dieser Stand | Gewinn    |
| ------------------- | ------------ | ------------ | --------- |
| DK5EN-1 (Heltec V3) | 244 900 B    | 256 196 B    | +11 296 B |
| DK5EN-92 (T-Beam)   | 146 868 B    | 162 428 B    | +15 560 B |

Die Werte sind innerhalb einer Firmware bitgenau stabil (beide neo-Laeufe
melden 256 196), zwischen den Firmwares klar getrennt. Dazu die
Flash-Ersparnis aus demselben Build: 10,5 kB auf dem Heltec, 45,5 kB auf dem
T-Beam.

### Der Dauerlauf

Vom 2026-09-19 19:41 bis 2026-09-20 08:00, zwoelf Stunden, auf drei Knoten:
DK5EN-1 (Heltec V3), DK5EN-92 (T-Beam) und DK5EN-98 (Heltec V3) -- letzterer
mit Mesh und Gateway EIN und Hop 2, also unter echter Netzlast. Alle drei auf
diesem Stand mit `-DINSTRUMENT_ENABLED=1`.

| Pruefpunkt                | Ergebnis                     |
| ------------------------- | ---------------------------- |
| Neustarts                 | keiner (nur der Start-Reset) |
| Abstuerze, Backtraces     | keine                        |
| Ringueberlaeufe           | keine                        |
| Gateway-Fehler (DK5EN-98) | keine                        |
| Verbindungsabbrueche      | keine                        |
| Laufzeit                  | 12,00 h, streng monoton      |

**Der Heap liegt flach, auf allen drei Knoten.** Die Knoten melden ihn ueber
zwei Kanaele, je nach `--setlog`: DK5EN-92 (`off`) ueber die `[HEAP]`-Zeile,
DK5EN-1 und DK5EN-98 (`on`) im Feld `heap=` der STAT-Zeile, einmal je
Fuenf-Minuten-Fenster. Drei unabhaengige Verlaeufe:

| Knoten   | Kanal    | Punkte | Mittel erste Haelfte | Mittel zweite Haelfte |
| -------- | -------- | -----: | -------------------: | --------------------: |
| DK5EN-92 | `[HEAP]` |     99 |             79 017 B |              78 119 B |
| DK5EN-1  | `STAT`   |    147 |            139 925 B |             139 839 B |
| DK5EN-98 | `STAT`   |    133 |            132 088 B |             132 234 B |

Die groesste Bewegung sind rund 900 Byte ueber neun Stunden; der Gateway-Knoten
unter Last steigt sogar leicht. Das ist Rauschen -- ein Leck saehe anders aus.
Die 161 228 B, die DK5EN-92 als ersten Wert meldet, sind der Zustand vor dem
Hochlauf: WLAN, GPS und Puffer belegen danach ihren Arbeitssatz, und die Zahl
faellt einmalig auf rund 79 kB. Alles danach ist die Gerade in der Tabelle.

**Die Schleife stockt nicht.** Die Instrumentierung meldete ueber zwoelf Stunden
fuenf Luecken, alle zwischen 19:41 und 19:42, also in der ersten Minute nach dem
Boot, waehrend WLAN-Anmeldung, NTP und GPS-Erfassung zusammenfallen: zweimal
`wifi_connect` (2569 und 2663 ms) und dreimal `udp` (2007, 939 und 632 ms). Im
Dauerbetrieb danach keine einzige.

Material: 8,2 MB, 13 910 Zeilen allein von der Netz-Konsole des Gateway-Knotens.

## K01 Vendor-Ballast entfernen

**1. 43 unbenutzte GFX-Fonts entfernt, die zehn tatsaechlich eingebundenen bleiben**
(`a50c5614`, W1 hygiene). `src/Fonts/` enthielt 53 `Free*`-Header aus der
Adafruit-GFX-Font-Sammlung; nur zehn davon werden irgendwo inkludiert:
`FreeMonoBold9/12/18/24pt7b.h` (`src/nrf52/nrf52_functions.cpp:88-91`,
`src/t-deck-pro/tdeck_pro.cpp:19`, `src/loop_functions.cpp:262-263,349-350`),
`FreeSans9/12/18pt7b.h` und `FreeSansBold9/10/12pt7b.h`
(`src/esp32/esp32_functions.cpp:39-43`, `src/loop_functions.cpp:265-267,352-353`).
Die Menge stammt nicht aus einer Schaetzung, sondern aus einem Referenz-Sweep
gegen den Baum; die vorige Zaehlung ("47 von 48") hatte sich verschoben. Dazu
zwei kleine, mit denselben Fonts verschraenkte Dateien: `Org_01.h`,
`Picopixel.h`, `Tiny3x3a2pt7b.h` und `TomThumb.h` (kleine Bitmap-Fonts,
ebenfalls ohne Include) sowie `src/nrf52/glcdfont.c` (143 Zeilen, ein zweites
`glcdfont.c` -- `src/GFX_Root/GFX.cpp:35` bindet `"glcdfont.c"` ueber den
Include-Pfad ein und trifft dabei auf GFX_Roots eigene Kopie, nie auf die in
`nrf52/`). Verifiziert: `grep -rn "Fonts/Free" src` und ein Sweep auf
`nrf52/glcdfont` liefern nach der Loeschung nur noch die zehn genannten
Include-Zeilen bzw. die eine GFX_Root-Stelle; `upstream/dev`s
`loop_functions.cpp` bindet dieselben zehn Fonts, keine der geloeschten.

**2. Fuenf tote Plattform-Shims entfernt: ESP8266, M1280, M2560, M328P,
SAMD21G18A** (`a50c5614`, W1 hygiene). `src/Platforms/platforms.h` band alle
fuenf MCU-Header bedingungslos ein, obwohl jeder von ihnen seinen gesamten
Inhalt selbst hinter einem Plattform-Makro versteckt (z. B.
`#ifdef ESP8266` in `ESP8266.h`, `#ifdef __AVR_ATmega328P__` in `M328P.h`) --
auf ESP32 und nRF52, den einzigen Zielen dieser Firmware, waren sie beim
Praeprozessor immer leer. `platforms.h` behaelt die vier tatsaechlich
verwendeten Zweige (`WirelessPaper`, `VisionMasterE213`, `VisionMasterE290`,
`ESP32`) und den `fallback`-Header. Verifiziert: ein Sweep auf
`Platforms/ESP8266`, `Platforms/M1280`, `Platforms/M2560`, `Platforms/M328P`
und `Platforms/SAMD21G18A` ueber `*.cpp`/`*.h`/`*.ini` findet nach der
Loeschung keine Stelle mehr. `upstream/dev` bindet alle fuenf weiterhin ein
(inklusive `M1280.h` unter dem eigenen Kommentar "Untested, but _should_
work") -- das ist upstream-Ballast, keiner dieser Fehlbetrag ist hier neu
entstanden.

**3. Zwei Restdateien ohne Buildrolle entfernt**: `src/idf_component.yml.orig`
(2 Zeilen, eine Backup-Kopie neben dem echten `src/idf_component.yml`) und die
zugehoerige Reduktion von `src/Platforms/platforms.h` um die fuenf jetzt
toten Include-Zeilen (`a50c5614`, W1 hygiene).

### Kampagnen-Commit

- `a50c5614` -- W1: defects 1, 2, 11 and the hygiene deletions

## K18 Build-Konfiguration

Die Zahlen zuerst, weil sie den Rest des Kapitels erklaeren: 64 Dateien,
+6987/-619 Zeilen, 28 Kampagnen-Commits im Diff dieses Dateisatzes. Der
groesste Teil dieser Zeilen ist kein neuer Build-Zustand, sondern das
Gegenteil davon -- Duplikat-Zeilen, die aus 31 fast identischen
`variants/<board>/platformio.ini` und `configuration.h` in eine gemeinsame
Quelle wandern. Vier Geschichten stehen dahinter; sie sind unten gruppiert,
nicht 28-fach aufgezaehlt.

### W7: Die Flottenvorgaben stehen einmal, nicht 28-mal

**4. Neun Radiowert-Makros aus 31 Variant-Headern in `src/configuration_default.h`
gehoben, jedes einzeln `#ifndef`-bewacht** (`233257e3`, W7 Schritt 1).
`RF_FREQUENCY 433.175000` stand woertlich 28-mal in
`variants/*/configuration.h`, `LORA_SF 11` 31-mal; 238 von 279 Zeilen sagten
dasselbe. Eine Frequenzkorrektur musste bisher 28 Dateien einzeln treffen,
oder sie traf sie inkonsistent, und nichts haette den Unterschied bemerkt.
`src/configuration_default.h:70-105` traegt jetzt `RF_FREQUENCY`,
`LORA_APRS_FREQUENCY`, `LORA_BANDWIDTH`, `LORA_SF`, `LORA_CR`,
`LORA_PREAMBLE_LENGTH`, `TX_OUTPUT_POWER`, `TX_POWER_MAX` und `TX_POWER_MIN`
mit `#ifndef`-Wache; der Include steht als LETZTE Zeile jeder Variante, sodass
ein Board, das einen eigenen Wert setzt, ihn bereits gesetzt hat, wenn die
Vorgabe drankommt -- lokal gewinnt immer, ohne Include-Reihenfolge-Abhaengigkeit.
Bewusst draussen geblieben sind `MODUL_HARDWARE` (die Board-Identitaet; eine
vergessene Zeile waere still ein `EBYTE_E22`) und `BUTTON_PIN` (nur 13 von 31
Boards stimmen ueberein; ein geerbter Falschwert liest einen floatenden Eingang
als Tastendruck).

Beweis nicht durch Text-, sondern durch Praeprozessor-Vergleich:
`test/golden/variant_macros_effective.py` nimmt die Compilation-Database-
Argumentliste jedes Envs, laesst sie mit `-dM -E` durch den Compiler laufen und
haelt fest, was dieser tatsaechlich sieht -- fuer alle 32 Board-Envs
BYTEGLEICH zur Baseline vor dem Umbau, obwohl 31 Dateien geaendert wurden. Der
textuelle `variant_macros_lint.py` haette diese Welle nicht gesehen, weil sie
nur seine Zeilen verschiebt, nicht ihren Inhalt. Nebenbefund mitgefixt: dessen
Selbsttest hatte `LORA_SF` hart als Opfermakro benannt und fiel deshalb an der
eigenen Verlagerung um; er sucht sich sein Opfer jetzt aus der Datei.

**5. Zehn per `#ifdef` getestete Praesenz-Flags ebenfalls in
`src/configuration_default.h` gehoben, mit `_DISABLED`-Sentinel statt
`#undef`** (`6fd68840`, W7-II Teil a). `ENABLE_GPS` (28 von 31 Varianten),
`ENABLE_BMX280` (26), `ENABLE_BMX680` (24), `ENABLE_RTC` (23),
`ENABLE_BMP390`/`ENABLE_AHT20`/`ENABLE_MCP23017` (22),
`ENABLE_SHT21`/`ENABLE_MC811` (21) und `ENABLE_INA226` (18) waren als
identische `#define`-Zeile in einer Mehrheit der Varianten dupliziert -- 227
Zeilen davon sind jetzt weg. Diese Flags werden per `#ifdef`/`#if defined()`
abgefragt, nicht nach Wert; hier ist Abwesenheit selbst das Signal, und ein
schlichter `#ifndef`-Default haette das stillschweigend fuer jedes Board
umgedreht, das die Abwesenheit bewusst gewaehlt hatte. Ein `#undef` als
Opt-out waere legal und stumm gewesen -- ein Tippfehler haette einen Sensor
flottenweit deaktiviert, ohne Warnung. Stattdessen prueft jede gehobene Flag
in `src/configuration_default.h:112-157` `#if !defined(X) &&
!defined(X_DISABLED)`, und eine Variante, die X nicht will, schreibt
`#define X_DISABLED`; ein `#error`-Block am Dateiende
(`src/configuration_default.h:159-197`) macht das gleichzeitige Definieren
beider Makros zum Compile-Fehler statt zur stillen Entscheidung. Ergebnis: 83
Sentinel-Zeilen neu (39 aus vorhandenen `//`-Kommentaren mit Begruendung
uebernommen, 44 neu geschrieben). `vision-master-e290` behaelt seinen
vorbestehenden Tippfehler `ENABLE_MCU811` (nirgends im Quellcode getestet,
also nie wirksam) unveraendert, bekommt aber
`ENABLE_MC811_DISABLED` und einen Kommentar, der den Tippfehler benennt
(`variants/vision-master-e290/configuration.h:13-16`).

Beweis wie bei Schritt 1, mit dem passenden Delta: die Effektiv-Makro-Baseline
zeigt gegen den alten Stand GENAU 92 neue `*_DISABLED`-Zeilen und sonst
nichts -- kein `ENABLE_*` aendert sich auf keinem der 32 Board-Envs. Ein
Advisor-Skript hat zusaetzlich alle 310 Paare (31 Varianten mal 10 Flags)
gegen den fertigen Stand geprueft: jedes Paar traf genau einen der beiden
Faelle, nie beide.

**6. Zwei Upload-Familien `[esp32_s3]` und `[esp32_classic]` tragen jetzt den
`upload_command`, den 23 Varianten zuvor je einzeln wiederholten**
(`6fd68840`, W7-II Teil b). `platformio.ini:1080-1086` definiert die beiden
Basis-Sections -- `[esp32_s3]` schreibt den Bootloader nach `0x0000` und
`safeboot-s3.bin` nach `0x10000`, `[esp32_classic]` nach `0x1000` und
`safeboot.bin` -- und 13 S3- sowie 10 klassische ESP32-Varianten erben das
per `extends`, neun verlieren zusaetzlich ihre eigene `monitor_speed`-Zeile,
eine ihr `upload_protocol`. Eine Stolperfalle stand der Vorlage im Weg:
`${this.__env__}` funktioniert in einer Basis-Section ohne Env nicht, weil
PlatformIO jede Section auch fuer sich auswertet und mit
`ProjectOptionValueError` abbricht (das hat testweise den ersten
compiledb-Lauf blockiert); die Vorlage nutzt daher die SCons-Variable
`$BUILD_DIR`, die der Upload-Schritt genauso ersetzt wie `$UPLOAD_PORT`.
Beweis: das neue `test/golden/variant_ini_effective.py` (11 Selbsttests, in
`test/golden/selftest.sh` verdrahtet) vergleicht `pio project config
--json-output` aller 68 Envs gegen die vor dem Umbau gezogene Baseline,
`$BUILD_DIR` auf den alten Text normalisiert -- PASS, 68 Envs, 692 Schluessel.
Ein echter Upload ueber die neue Vorlage auf DK5EN-93 hat Bootloader,
Safeboot und Firmware geschrieben und den Hash verifiziert.

### `lib/tinyxml2`: die Bibliothek ohne ihr Beispielprogramm vendort

**7. `E22_XML-DevKitC` hatte noch 1160 Bytes freies `dram0_0_seg` -- die
knappste Umgebung im Baum und die DRAM-Haelfte von MEM-04 -- weil das
Beispielprogramm der Bibliothek libstdc++ iostream und locale mit hereinzieht**
(`95d6fbe6`, MEM-04/R3-04). Upstream bezieht `tinyxml2` ueber eine
`lib_deps`-URL; dieses Repository liefert `contrib/html5-printer.cpp` mit
eigenem `main()` aus. Der Arduino-Core definiert kein eigenes `main()`, also
loeste der Linker die `main`-Referenz von crt0 aus dem Beispielprogramm auf --
sichtbar in der Archiv-Kette `crt0.o -> html5-printer.cpp.o (main) ->
globals_io.o / locale_init.o`. Der Fix vendort die Bibliothek unter
`lib/tinyxml2/` mit einem `library.json`, dessen `srcFilter` nur
`tinyxml2.cpp` baut und `contrib/` sowie `test/xmltest.cpp` ausschliesst
(`lib/tinyxml2/library.json`, `lib/tinyxml2/tinyxml2.cpp`,
`lib/tinyxml2/tinyxml2.h`, Quellen v11.0.0 unveraendert, `LICENSE.txt`
unter zlib uebernommen); `variants/E22_XML-DevKitC/platformio.ini` verweist
jetzt auf diese lokale Kopie statt auf die Upstream-URL.

Gemessen, same-base, und vom Orchestrator auf einem unabhaengigen Clean-Rebuild
gegengeprueft (`95d6fbe6`):

```
dram0_0_seg   used 123420 -> 117652   free 1160 -> 6928   (+5768 B)
iram0_0_seg   used 127616 -> 127044   free 3456 -> 4028    (+572 B)
```

Ein Grep nach `html5-printer`/`xmltest`/`locale_init`/`globals_io` in der
Map-Datei danach: 0 Treffer. `src/tinyxml_functions.cpp` blieb unveraendert,
sein Include ist wie zuvor durch `ENABLE_XML` bewacht, und `native_xml` lief
weiterhin 11/11 gruen. `E22-DevKitC` bleibt unberuehrt: die
Library-Dependency-Auswertung listet `tinyxml2` dort weiterhin (der
Include-Text-Scan sieht das `#ifdef` nicht), aber `tinyxml_functions.cpp.o`
kompiliert dort auf null Bytes in jeder Sektion, und aus dem Archiv wird kein
Objekt gezogen. Zusammen mit dem T-Beam-PSRAM-Unflag unten sind damit beide
Haelften von MEM-04 geschlossen -- beide zuvor headroom-losen Umgebungen haben
jetzt wieder Luft.

### `platformio.ini`: PSRAM-Flags von drei klassischen T-Beam-Envs entfernt

**8. Der IRAM-Anteil von MEM-04: `-DBOARD_HAS_PSRAM
-mfix-esp32-psram-cache-issue` kostete `ttgo_tbeam`, `ttgo_tbeam_SX1262` und
`ttgo_tbeam_SX1268` je 20 Bytes freies `iram0_0_seg` -- das war es, was
Upstream-CI auf `9d885b1a` brach** (`a2e9794c`, R3-02). Diese drei Envs erben
die Flags allein aus der Board-Definition `ttgo-t-beam.json`; mit ihnen ruft
`initArduino()` `psramInit()` auf und zieht `esp_spiram_init` sowie
`spiram_psram.c` (4427 B) ins IRAM, weil dieser Code bei deaktiviertem
Cache laufen muss. `build_unflags` in allen drei
`variants/ttgo_tbeam*/platformio.ini` entfernt beide Flags wieder (je +26
Zeilen, ueberwiegend erklaerender Kommentar). Der Tausch war nicht kostenlos
und wird im Commit selbst gegen die eigene Vorhersage vom 2026-09-05
korrigiert: PSRAM ist auf dem Hardware-Node tatsaechlich vorhanden und
funktionsfaehig (4191863 B frei bei Init, gemessen auf T-Beam-92, V1.2,
AXP2101), nicht abgeschaltet, wie ein frueherer Analysebefund vermutet hatte.
Getauscht wurde trotzdem, auf drei gemessenen Gruenden: kein
`ps_malloc`-Aufruf existiert in einer von diesen Envs gebauten Quelldatei (nur
unter `src/t-deck/` und `src/t5-epaper/`, beides S3-Boards); PSRAM kann IRAM
nicht ersetzen, weil IRAM Code haelt, der bei deaktiviertem Cache laeuft, und
PSRAM Datenspeicher hinter demselben Cache ist; und der grosse erhoffte
DRAM-Gewinn (statische Ringe aus `.bss` in PSRAM auslagern, ca. 28 kB) ist
ohnehin unerreichbar, weil `CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY` im
vorgefertigten SDK nicht gesetzt ist.

Gemessen, same-base, alle drei Envs: `iram0_0_seg` frei 20 -> 4972 B
(+4952), `dram0_0_seg` frei 10312 -> 10384 B (+72). Auf Hardware
(T-Beam-92, V1.2, AXP2101) nach dem Flashen verifiziert: PSRAM weiterhin bei
0 nach Init, freier interner Heap 146488 -> 151348 B, SX1276-Init
erfolgreich, WLAN verbunden, `[BOOT];ready`, keine Backtraces. Der Schritt ist
reversibel (den `build_unflags`-Block loeschen holt PSRAM zurueck und zahlt
das IRAM erneut); eine Funktion, die PSRAM will, muss ohnehin mit
`psramFound()` pruefen, da die Flotte gemischt ist.

### `variants/t5_epaper`: die Umgebung kompiliert zum ersten Mal

**9. `env:t5_epaper` galt in jedem Sweep als "known broken", weil als einzige
Variante im Baum ein `configuration.h` fehlte; das war der erste von acht
Blockern, nicht der einzige** (`af179892`). Die neue Datei
`variants/t5_epaper/configuration.h` (71 Zeilen) macht das Board fuer
`variant_macros_lint.py` zur 31. sichtbaren Variante -- genau 11 neue Zeilen
in `variant-macros.txt`, nichts entfernt oder geaendert. In
`variants/t5_epaper/platformio.ini` war `lib_deps` eine 16 Zeilen lange, von
Hand gepflegte Kopie von `[libs]`; `[libs]` hatte inzwischen die
SparkFun-u-blox-Bibliothek dazubekommen, die Kopie nicht, und
`src/esp32/esp32_pmu.cpp:13` inkludiert deren Header unbedingt fuer jedes
ESP32-Board -- die Umgebung konnte also gar nicht uebersetzen. `lib_deps`
erbt jetzt `${libs.lib_deps}` statt es zu kopieren; boardeigene Eintraege wie
`RadioLib@7.1.2` (gegenueber `7.6.0` in `[esp32libs]`) bleiben bewusst
stehen, weil eine Versionsaenderung auf einem Board ohne Bench eine
Verhaltensaenderung waere, keine Bereinigung. Dazu kommt ein `-I
src/t-deck-pro`-Suchpfad fuer den gemeinsam genutzten BQ27220-Treiber und ein
gezielter `build_src_filter`-Eintrag, der nur `bq27220.cpp` und
`bq27220_data_memory.c` aus dem sonst ausgeschlossenen `t-deck-pro/`-Baum
wieder aufnimmt. Die uebrigen fuenf Blocker (ein `#elif`-Zweig in
`loop_functions.cpp`, der `BOARD_T5_EPAPER` nicht kannte; ein veralteter Fork
von `peri_gps.cpp`; ein Windows-Backslash in einem Include; ein
`extern int` gegen ein `volatile int`; ein fehlendes Include-Guard in
`src/configuration_global.h`) liegen ausserhalb dieses Dateisatzes und werden
hier nicht im Detail behauptet. Beansprucht wird nur, dass die Umgebung jetzt
kompiliert (Flash 66.8%/4380717 B, RAM 33.8%/110904 B) -- nichts ueber ihr
Laufzeitverhalten; das Board steht auf keiner Bench.

### Sonstige Variantenkorrektur

**10. Acht Varianten trugen `#define OneWire_GPIO 99` -- auf keinem von ihnen ein
gueltiger GPIO, und die eigenen Kommentare sagten es bereits ("nicht getestet
!!", "getestet ???", "ungenutzt")** (`b07f5be4`, R3-03). Betroffen: die
`configuration.h` von `heltec_wifi_lora_32_V3`, `heltec_wifi_lora_32_V4`,
`heltec_wireless_stick`, `heltec_wireless_tracker`, `ttgo_tbeam_supreme`,
`vision-master-e213`, `vision-master-e290` und `wireless-paper`. Alle acht
tragen jetzt `-1`, die Schreibweise, die `T-ETH-ELITE_1262` bereits benutzte.
Geloescht wurde die Zeile bewusst nicht: die gesamte OneWire-Implementierung
steht in `#ifdef OneWire_GPIO` (`onewire_functions.cpp:13-379`), ohne das
Makro koennte `--owgpio <pin>` den Sensor nie wieder aktivieren -- das waere
Funktionsentfernung, keine Bereinigung. `-1` scheitert stattdessen am
`> 0`-Test, der Treiber bleibt kompiliert, der Default bleibt inaktiv. Die
drei betroffenen T-Beam-Boards behalten ihren echten Pin (GPIO 4). Der
eigentliche Defekt lag tiefer: der Makro-Fallback erreichte den Treiber ohne
die Gueltigkeitspruefung, die der Laufzeitpfad laengst anwendet
(`--owgpio` weist `<= 2` zurueck, `command_functions.cpp:1984`), waehrend
`onewire_functions.cpp:56-58` und `:204-206` das Makro ungeprueft uebernahmen
-- der DS18-Zweig setzte dabei sogar `one_found = true`, meldete also einen
Sensor auf einem nicht existierenden Pin. Beide Fallbacks pruefen jetzt
dieselbe Bedingung, was die Fehlerklasse schliesst statt nur die acht
Einzelfaelle.

### Was in diesem Kapitel bewusst fehlt

Die uebrigen der 28 Commits beruehren `platformio.ini` nur, weil ein neuer
`[env:native_*]`-Testblock dazukommt (etwa fuer den Loop-Scheduler, die
Settings-Umstellung oder die APRS-/mHeard-Charakterisierungstests); das ist
Testinfrastruktur, keine Build-Konfigurationsentscheidung, und wird hier nicht
einzeln aufgefuehrt. Ein Hinweis genuegt: die Host-Testumgebungen leben auf
dem begleitenden Test-Branch. Ebenfalls nicht hier beschrieben: die
`ui_common`-Zusammenlegung der LVGL-Zwillinge und die MC_CAPTURE/MC_DIAG-
Umbenennung der Diagnose-Flags -- beide aendern `platformio.ini` nur an einer
Stelle nebenbei, ihre eigentliche Substanz liegt in Quelldateien ausserhalb
dieses Kapitels.

### Nachgemessen bei der Projektion auf upstream/dev

Beim Aufbau dieses Branches wurde jeder Commit auf acht Leitzielen gebaut und
die Linkerregionen dabei ausgelesen. Der erste Commit traegt nur Upstreams Code
plus die Loeschungen aus K01 -- seine Zahlen sind damit Upstreams Zahlen, und
sie belegen, wie eng zwei Umgebungen dort sitzen:

| Umgebung               | Upstream (Commit 1) | nach dem Kern-Commit | Gewinn |
| ---------------------- | ------------------- | -------------------- | ------ |
| `ttgo_tbeam` IRAM      | **20 B frei**       | 4972 B               | +4952  |
| `E22_XML-DevKitC` DRAM | **1160 B frei**     | 17816 B              | +16656 |
| `E22_XML-DevKitC` IRAM | 3456 B              | 4028 B               | +572   |

Zwanzig Byte im `iram0_0_seg` des T-Beam heisst: die naechste Funktion, die
dorthin gelegt wird, bricht den Link. Genau das ist bei `9d885b1a` schon einmal
passiert und hat die Upstream-CI zerlegt. Die beiden Hebel aus diesem Kapitel --
die PSRAM-Flags und die vendorte tinyxml2 -- holen beide Umgebungen aus dieser
Zone heraus.

Die Zahlen stammen aus `tools/resource_watch.py dram` gegen die `firmware.map`
des jeweiligen Builds, nicht aus der PlatformIO-Zusammenfassung: deren `RAM:`-
Zeile misst gegen das PSRAM-inklusive Gesamtbudget und meldet fuer denselben
T-Beam `7.9 %`.

## K02 Settings-Store: ein Schema fuer beide Plattformen

**11. Der Schritt aendert das Layout nicht: `FLASH_STRUCT_VERSION` bleibt bei
`20260724`, kein Knoten verliert seine Konfiguration beim Update**
(`ff49dfe8`, `624e8f50`, `3fd5dcaa`, `af00c402`, `2b66f703`, `b0adf723`,
`64f3cdd1`, Kampagne W3/W3c). Geprueft gegen `src/configuration_global.h`:
`FLASH_VERSION` und `FLASH_STRUCT_VERSION` sind zwei verschiedene Zahlen mit
zwei verschiedenen Bedeutungen. `FLASH_VERSION` ist der Build-/Release-Stempel
(reine Anzeige in `--info`, aktuell `20260912`); `FLASH_STRUCT_VERSION`
benennt die Generation von `struct s_meshcom_settings` und steht seit dem
2026-07-24-Commit `6e7c012a` unveraendert bei `20260724`
(`src/configuration_global.h:79-89`). Der Header traegt seit diesem Kapitel
einen langen Kommentar, der beide Begriffe auseinanderhaelt und einen
konkreten Vorfall nennt: vor dieser Trennung wurde beim Laden gegen
`FLASH_VERSION` verglichen, sodass JEDES Release mit neuem Datum ueber
`clear_flash()` die Einstellungen jedes aktualisierenden Knotens loeschte --
Rufzeichen, WLAN, Sensoren --, obwohl sich am Struct-Layout nichts geaendert
hatte. Alles, was dieses Kapitel am Struct selbst tut (Felder umsortieren,
zusammenlegen, umbenennen), aendert weder Groesse noch Bedeutung eines
gespeicherten Feldes und loest deshalb keine `FLASH_STRUCT_VERSION`-Anhebung
aus -- das ist keine Behauptung, sondern der Stand im Baum. Ein Bestandsschutz
fuer die Uebergangszeit steht daneben: `FLASH_STRUCT_LEGACY` erkennt den Wert
`20260821`, den Knoten mit der alten Datum-Semantik noch in `node_fversion`
tragen, und akzeptiert ihn als layoutgleich (`configuration_global.h:91-99`).

**12. Fundament, aber an nichts angeschlossen: der Schluessel-Wert-Codec und das
Schema stehen, bevor sie ein Boot-Pfad je sieht** (`ff49dfe8`, `624e8f50`).
`src/settings_store.{h,cpp}` ist der plattformneutrale Codec (kein
Arduino-Include), der ueber ein aufruferseitiges `FieldDescriptor`-Array
arbeitet -- unbekannter Schluessel wird ignoriert, ein fehlender nimmt seinen
Default, eine kaputte Zeile wird uebersprungen ohne das Feld zu beruehren, ein
zu langer Wert wird VERWORFEN statt abgeschnitten (mutationsgeprueft: ein
Abschneiden statt Verwerfen laesst zwei Faelle rotwerden). `src/settings_schema.{h,cpp}`
baut daraus 111-122 Zeilen (je nach Board) aus dem bestehenden `CFG_FIELD_LIST`
von `config_json.h`, damit es keine zweite Abschrift der 109 X()-Zeilen gibt --
deren Reihenfolge die kanonische CRC-Form bildet. Dabei fand die Umstellung
selbst einen Fehler, der die Feldliste betraf: `CFG_POWER_NOT_SET` (-20, "noch
kein TX-Wert gespeichert") lag ausserhalb des gueltigen Bereichs von
`node_power` und waere beim ersten Laden eines Werksknotens auf das
Board-Minimum festgeklemmt worden, bevor die Sanitisierung ihn haette
aufloesen koennen -- ein frischer RAK4631 haette sich dauerhaft auf minimale
Sendeleistung festgelegt. `has_range` schliesst jetzt einen gesetzten
Escape-Wert aus. Nichts davon ist zu diesem Zeitpunkt an einen Boot-Pfad
angeschlossen (Gate: 32/32 Envs, 322 native Faelle plus die neuen
Settings-Suiten).

**13. Der nRF52-Umstieg: das Struct hoert auf, ein Dateiformat zu sein**
(`3fd5dcaa`). `init_flash()` versucht zuerst den neuen Schluessel-Store; nur
wenn der fehlt oder die Sanity-Pruefung durchfaellt (Annahmekriterium:
`fields_set > 0` UND ein nicht-leeres `node_call`, sonst faellt der Pfad auf
den alten Blob zurueck statt auf `flash_reset()`), laeuft der alte Pfad und
migriert einmalig. Der alte Blob wird danach weder aktualisiert noch geloescht
-- ein Downgrade auf eine Firmware ohne Schluessel-Store findet ihn also
unveraendert und kommt auf den Stand VOR der Migration zurueck, veraltet, aber
nicht geloescht. Ein Blob mit dem vor-kompatiblen `0x57`-Marker wird dagegen
auf Default gesetzt, weil kein Code dieses Layout mehr lesen kann und ein
Fehldeuten der Bytes als aktuelles Struct genau der Fall ist, den diese
Aenderung ausschliessen musste. Ein Regressionsfund unterwegs: die erste
Fassung hatte `save_settings()`s Sparen-bei-unveraendert-Wache verloren -- bei
150+ Aufrufstellen quer durch `command_functions.cpp`, `phone_commands.cpp`,
`loop_functions.cpp` und `event_functions.cpp` haette das jeden Aufruf zu
einem Schreiben plus Umbenennen auf internem Flash gemacht, Verschleiss, der
erst Monate spaeter als schreibunfaehiges Dateisystem auffaellt. Wieder
eingebaut, jetzt mit einem 64-Byte-Chunk-Vergleich gegen einen Stapelpuffer,
weil der Vergleich auf dem 4-KB-Loop-Task laeuft. `test/golden/nano_printf_lint.py`
und `native_settings_roundtrip` sind zu diesem Zeitpunkt noch nicht auf
Hardware gelaufen -- keiner der vier Lade-/Speicherpfade hat zu diesem
Zeitpunkt einen echten Boot gesehen.

**14. Fix-Welle nach adversarialer Pruefung: `flash_reset()` reisst nicht mehr das
Dateisystem ab, und der Puffer bekommt seinen echten Fall zurueck**
(`af00c402`). Eine Gegen-Pruefung (`docs/w3-settings-verdict.md`) widerlegte
die bis dahin verfolgte Diagnose eines Einstellungsverlusts auf DK5EN-90 --
es fehlten keine Schema-Zeilen --, fand aber vier eigenstaendige Defekte auf
dem Weg. Der schwerste: `flash_reset()` rief `InternalFS.format()`, was nicht
nur die Einstellungen, sondern auch alle BLE-Bonds unter `/adafruit/bond_prph/`
loeschte -- jeder Reset entkoppelte still jedes je gekoppelte Telefon, ohne
Hinweis in der Einstellungskonsole und ohne Sicherung. `flash_reset()` loescht
jetzt gezielt seine zwei eigenen Dateien, `format()` bleibt nur echter
Rueckfallpfad. Daneben: `kSettingsBufferCap` von 4096 auf 8192 Byte angehoben,
nachdem eine Nachmessung mit beiden Achsen im ungueltigsten Fall (`%.17g` fuer
jede Fliesskommazahl, plus die unten wieder aufgenommenen Zaehler) 4082 Byte
ergab -- 14 Byte Rand am alten Limit; `node_msgid`/`node_ackid` persistieren
wieder, weil ihr Neustart bei 0 bei jedem Boot Nachrichtenkennungen in jedes
Dedup-Fenster der Nachbarn zurueckspielt (`loop_functions.cpp:3331`); und
`max_hop_text` bekommt `CFG_ESC(0)`, weil der Struct-Default 0 ausserhalb
seines gueltigen Bereichs 1..6 lag und beim Laden auf 1 statt auf den
beabsichtigten Default 4 festgeklemmt wurde. Instrumentiert mit 24 rohen
`Serial.printf`-Markern, weil `DO_DEBUG 0` jeden `DEBUG_MSG` und damit jede
Sicht auf diesen Pfad wegkompiliert.

**15. Der eigentliche zweite-Boot-Verlust: newlib-nanos `printf` schrieb `ld`/`lu`
statt Zahlen in die Einstellungsdatei** (`0b2c5b82`). `settings_store.cpp`
kodierte jede Ganzzahl mit `%lld`/`%llu`; die drei nRF52-Umgebungen binden
newlib-NANO ohne `_WANT_IO_LONG_LONG`, das den zweiten `l` nicht als
Laengenmodifikator erkennt und den Rest der Formatanweisung woertlich ausgibt.
Der Migrationsboot schrieb also eine wohlgeformte Datei, in der jedes
Zahlenfeld die zwei Zeichen `ld` oder `lu` trug; der Schreibvorgang und die
Umbenennung meldeten Erfolg, `--info` las unmittelbar danach noch aus dem RAM
richtig, und der Verlust zeigte sich erst einen Neustart spaeter, als
`decode()` diese Werte verwarf und die Felder auf Struct-Default zurueckfielen.
Kein Host-Test konnte das sehen: die Host-libc formatiert `%lld` korrekt, der
identische Code lief unter `pio test` gruen, waehrend er auf Hardware die
gesamte Konfiguration verlor -- eine Eigenschaft der GELINKTEN libc, nicht des
Quelltexts. `encode_u64`/`encode_i64` wandeln jetzt von Hand ohne `printf` im
Pfad; `test/golden/nano_printf_lint.py` verbietet `%ll`/`%j`/`%q` im gesamten
nRF52-Quellsatz. Auf DK5EN-90 ueber drei Neustarts bestaetigt: reale
Dezimalwerte, `unknown_keys=0`, `malformed_lines=0`, `GET /config.json`
identisch zur Baseline vor dem Umbau.

**16. Die Spuren-Suche nach `save;rename_failed`: die Speicherplatz-Hypothese ist
ausgemessen, nicht nur vermutet, und widerlegt** (`fff252d5`). Zwei
`[SETST];save;rename_failed`-Marker auf dem Boot direkt nach einem Reflash
waren die Spur einer moeglichen Ursache -- der Legacy-Blob, der neue Store und
die Temp-Datei gleichzeitig auf einem 28 672-Byte-Dateisystem. `--dumpsettings`
zaehlt jetzt den Dateibestand (`/MeshCom-RAK` 2000 B, `/MeshCom-Settings-Store`
1546 B, kein Bond) und liest auf DK5EN-90 29 von 224 Bloecken -- das
Dateisystem war nicht annaehernd voll, die Hypothese damit gemessen widerlegt.
Zwei weitere DFU-Reflashs produzierten keinen einzigen Fehlschlag mehr, sodass
sich die Ursache an diesem Tag nicht weiter eingrenzen liess. Was bleibt:
`Adafruit_LittleFS` bietet keinen Aufruf fuer freien Speicher, daher laeuft die
Bestandsaufnahme drei Ebenen tief ueber die oeffentliche File-API (zwei Ebenen
haetten jeden BLE-Bond unter `/adafruit/bond_prph/<peer>` uebersehen); ein
einzelner Wiederholungsversuch des `rename()` ist eingebaut und meldet
`save;rename_retry_ok` oder `save;rename_failed_twice`, um bei einem
kuenftigen Auftreten zwischen einem voruebergehenden Flash-Fehler und etwas
Bestaendigem an der Zieldatei zu unterscheiden.

**17. Der scheinbare Fehlschlag war keiner: ein `rename()`, das den Umzug
tatsaechlich ausgefuehrt hatte, wird jetzt als Erfolg gewertet**
(`fa6e4e78`). `legacy_migration_failed` auf DK5EN-90s W3-Migrationsboot war ein
Fehlalarm: `lfs_rename()` meldete Fehlschlag, hatte den Umzug aber tatsaechlich
korrekt ausgefuehrt, und der von `fff252d5` eingebaute Wiederholungsversuch
scheiterte danach aus einem zweiten, unabhaengigen Grund -- seine Quelldatei
war bereits weg. Drei unabhaengige Belege aus einer mitgefuehrten Boot-Mitschrift
(`docs/bench/w3-baseline/rak90-migration-boot-20260916.txt`) beweisen, dass der
Umzug stattfand: der Store existierte beim Boot-Start nicht, das Inventar VOR
jedem Aufraeumen listet exakt drei Dateien mit dem Ziel in seiner neuen Groesse
(1458 B, kein `.tmp`), und ein spaeteres Speichern im selben Boot findet das
Ziel byte-identisch zu dem, was `encode()` erzeugt
(`save;skipped_unchanged`). Fix: `writeFileAtomic()` fragt bei einem als
fehlgeschlagen gemeldeten `rename()` die Zieldatei direkt, ob sie genau die
erwarteten Bytes traegt (`fileHasExactContent()`, jetzt gemeinsam mit dem
Unveraendert-Wache-Vergleich genutzt) -- wenn ja, gilt das Schreiben als
erledigt (`save;rename_false_negative`), die Temp-Datei wird aufgeraeumt und
der Wiederholungsversuch, der die Quelle nach ihrem Verschwinden nur noch
schlimmer machen koennte, entfaellt. Ein echter, nicht rueckgaengig gemachter
Fehlschlag muss weiterhin fehlschlagen (eigener Test dafuer), damit aus dem
Fix nicht "jeder gemeldete rename-Fehlschlag ist in Ordnung" wird. DK5EN-90
exportiert auf dem korrigierten Image sauber: 102 erhalten, 0 geaendert, 0
verloren, 2 wie vorgesehen entfernt.

**18. `node_msgid` erreicht Flash nur noch einmal je 100 Frames, nicht mehr bei
jedem Frame** (`bce95db5`). `src/msgid_counter.h` persistiert nur bei
Vielfachen von `kMsgIdPersistStep` (100), advanciert den gespeicherten Wert
beim Laden um einen vollen Schritt und schreibt genau diesen Wert zurueck --
das Schreiben-beim-Laden ist der tragende Teil: ohne es traegt Flash weiter
den Block des VORHERIGEN Boots, waehrend der Knoten schon aus dem neuen
vergibt, und ein Absturz in den ersten hundert Frames nach einem Boot laesst
den naechsten Boot wieder dort anfangen, wo dieser begann -- die Wiederholung,
die die Persistenz eigentlich verhindern soll, nur subtiler. Ein eigener
Property-Test durchlaeuft jeden Absturzpunkt ueber zwei Bloecke und faellt
genau bei fehlendem Ruecktrag durch. Gemessen auf DK5EN-90: `node_msgid`
24 -> 124 -> 224 ueber zwei Boots, ein `save;ok` je Boot, alles andere
`skipped_unchanged`. Notiert, nicht behoben: `node_ackid` wird geladen,
gespeichert und nirgends im Quelltext inkrementiert -- toter Zustand, siehe
K02 oben (`af00c402` nahm die Schema-Zeile wieder auf, ohne dass etwas den
Zaehler zaehlt).

**19. Der ESP32-Umstieg wird schema-getrieben, und dabei kommt ein
einstellungsfressender Defekt heraus, der auf der anderen Plattform lag**
(`2b66f703`). Die handgeschriebene Liste aus 134 `preferences.get*`- und 132
`put*`-Aufrufen weicht einem einzigen Durchlauf ueber
`settings_schema::fields()`, der ueber `offsetof()`/`sizeof()` dispatcht --
ein neues Feld in der X()-Tabelle ist die einzige Aenderung, die es fortan auf
beiden Plattformen persistiert. Der Umbau legte dabei einen bereits
ausgelieferten Fehler frei, den `bce95db5` (siehe oben) scharf gemacht hatte:
`Preferences preferences` ist EIN geteiltes globales Handle. `init_flash()`
oeffnet es und laedt jedes Feld in diesem Fenster; `sanitize_loaded_settings()`
rief seit `bce95db5` unbedingt `save_settings()` auf, und das sitzt MITTEN im
Ladevorgang -- `save_settings()` endet mit `preferences.end()`. Verifiziert
gegen die installierte arduino-esp32-Quelle: `Preferences::begin()` liefert
sofort `false`, wenn schon gestartet, oeffnet also NICHT neu, sodass
`save_settings()`s Schreiben auf `init_flash()`s eigenem offenen Handle landet;
`end()` setzt `_started = false`, und jedes `getX()` danach oeffnet mit
`if(!_started) return defaultValue;`. Auf jedem ESP32-Boot waere also (a) jedes
noch nicht geladene Feld mit seinem Vor-Lade-Wert in den NVS zurueckgeschrieben
worden, und (b) jedes `getX()` danach haette buchstaeblich seinen Default
zurueckgegeben, ohne den NVS je zu beruehren -- praktisch die zweite Haelfte
aller Einstellungen bei jedem Boot still zurueckgesetzt, ununterscheidbar von
einem Werksreset. Der Fehler erreichte kein Feldgeraet: das einzige fuer
`bce95db5` geflashte Geraet war DK5EN-90, ein nRF52 ohne `Preferences`. Fix:
`sanitize_loaded_settings()` speichert nicht mehr selbst; `init_flash()`
sanitisiert, schliesst sein Handle, speichert dann. Ein neuer Wachposten
(`g_flash_load_in_progress`, `src/esp32/esp32_flash.cpp:42`) laesst
`save_settings()` laut ablehnen statt still zu beschaedigen, falls ein
kuenftiger Aufruf unter den 245 Aufrufstellen doch waehrend eines Ladevorgangs
landet. Regressionssuite `test_esp32_flash_lifecycle` (8 Faelle) laeuft
bewusst OHNE `MC_SAFEBOOT`, weil `sanitize_loaded_settings()` darunter
wegkompiliert ist -- genau deshalb konnte die erste Suite diesen Fehler nicht
fangen. Beide Boot-Images wurden kleiner (686768 -> 686512, 643680 -> 643344).
Ein 15-Zeichen-NVS-Schluessellimit ist jetzt ein `static_assert` in
`settings_schema.cpp` (`Preferences` verwirft einen laengeren Schluessel
still). Auf ESP32-Hardware zu diesem Zeitpunkt noch NICHT geflasht.

**20. Ein Struct, ein Zaehler-Namensraum, eine Gate-Regel je Mitglied**
(`b0adf723`, W3c). `struct s_meshcom_settings` ist ab jetzt einmal definiert,
in `src/meshcom_settings.h`, aus X-Makro-Mitgliederlisten erzeugt, damit ein
nativer Test jedes Mitglied aufzaehlen kann; beide Plattform-Header binden
diesen einen Header ein. Endgueltig entfernt: `send_repeat_time`, `auto_join`
und das tote LoRaWAN-OTAA-Geruest (`api_timer_*`, `periodic_wakeup`,
`g_task_wakeup_timer`) sowie `node_ackid` (geladen, gespeichert, nie
gelesen -- der in `bce95db5` notierte Fund). `node_msgid` bleibt Struct-Mitglied,
wird aber KEINE Schema-Zeile mehr: es ist ein Zaehler hinter
`src/counters_store.h`, ESP32 im eigenen Preferences-Namensraum `"Counters"`
mit einmaligem Fallback-Lesen aus `Credentials/node_msgid`, nRF52 in seiner
eigenen `/counters.txt` -- so trifft weder ein Konfigurationsimport noch ein
BLE-Einstellungsschreiben (siehe K03) den Zaehler mehr rueckwaerts.
Fehlschlagssicheres Gate auf Mitgliederebene: `test/test_settings_members`
(Envs `native_settings_members_esp32`/`_nrf52`) stellt sicher, dass jedes
Struct-Mitglied entweder eine Schema-Zeile hat oder namentlich in
`src/meshcom_settings_runtime.h` als Laufzeitwert deklariert ist --
mutationsgeprueft in beide Richtungen. Das ersetzt den bisherigen
`settings_schema_lint.py`-Textabgleich, der struct-blind war und ein Mitglied
ohne Schema-Zeile stillschweigend durchliess. Der nRF52-Legacy-Blob wird durch
den eingefrorenen `s_ble_settings_v1` gelesen (K03), byte-identisch zu dem, was
offizielles v4.35t schreibt; sein CRC32 wird notiert, eine seither
umgeschriebene Datei erzwingt die Migration erneut. Gate: 868/868 native
Faelle in 27 Envs, 28 von 29 Hardware-Envs gebaut.

**21. `node_gpsbaud`/`node_update` waren zwei Schreibweisen fuer dasselbe Feld,
keine Layoutaenderung** (`64f3cdd1`). `node_gpsbaud` lief als `unsigned long`
(ESP32) und `unsigned int` (nRF52) -- vereinheitlicht auf `uint32_t`, wobei
beide Schreibweisen auf beiden Toolchains bereits 4 Byte mit identischer
Ausrichtung waren (per `static_assert` auf beiden Compilern geprueft): eine
reine Namensfrage, keine Speicherfrage. `node_update` schrumpft von `char[21]`
(ESP32) auf `char[20]` wie nRF52, und der Fehler dahinter war handfester: ESP32
kopierte per `memcpy(node_update, ctemp, 21)`, obwohl `snprintf` nur 20 Byte
gefuellt hatte -- das 21. kopierte Byte war UNINITIALISIERTER STACK, der in
ein Einstellungsfeld gelangte (nie zurueckgelesen, aber dennoch ein Fund).
Ausdruecklich OHNE Struct-Version-Bump, und das ist hier begruendet statt
angenommen: ESP32 persistiert `node_update` gar nicht als Struct-Feld (~266
einzelne NVS-Paare); nRF52, die einzige Plattform mit rohem Struct-Abbild,
hatte `node_update` schon vorher als `[20]`, und `uint32_t` ist Groesse und
Ausrichtung von `unsigned int` identisch. `WisBlock-API.h:567` behaelt
`unsigned int` ABSICHTLICH -- das ist die alte, ungewanderte Struct-Generation,
ein Drahtformat, kein Stilfehler, und ein Aufraeumen dort wuerde die Bytes
jedes noch nicht migrierten Knotens umdeuten.

Zwei Commits im selben Dateisatz betreffen NICHT die Laufzeit-Persistenz,
sondern reine Build-Zeit-Vorgaben in `src/configuration_default.h` (Kampagne
W7, ein anderes Vorhaben als W3): **die neun am haeufigsten wiederholten
Radiowerte (`433.175000` 28x, `LORA_SF 11` 31x) und zehn mehrheitlich gleiche
Praesenz-Flags (`ENABLE_GPS` 28/31 usw.) stehen jetzt einmal statt in 28-31
`variants/<board>/configuration.h`-Kopien** (`233257e3`, `6fd68840`). Beide
Aenderungen sind ueber `test/golden/variant_macros_effective.py` (Compiler-
Praeprozessorlauf je Env) als bytegleich zur alten Baseline verifiziert -- kein
`ENABLE_*` und kein Wert aendert sich auf irgendeinem Board. Diese zwei
Commits teilen mit dem Rest des Kapitels nur die Datei, nicht das Thema: sie
aendern nichts an `struct s_meshcom_settings`, an NVS-Schluesseln oder am
nRF52-Store, und beruehren `FLASH_STRUCT_VERSION` nicht.

Noch offen aus diesem Kapitel: die ESP32-Neuvermessung auf Heltec-93 und die
T-Deck-NVS-only-Schluessel (`b0adf723`), sowie ein zweiter Upgrade-Beweis auf
einem ESP32-Knoten (`2b66f703`) -- beide bislang nur auf DK5EN-90 (nRF52)
erbracht.

## K03 BLE-Settings v1

**22. Das BLE-Drahtformat wird als v1 eingefroren, MIT static_assert-Pins auf
jeden Feld-Offset -- damit sich der interne Struct weiterentwickeln kann, ohne
die Telefon-App zu brechen** (`eb0652a2`, W3). Die nRF52-BLE-Charakteristik
schickt `s_meshcom_settings` bislang als ROHE BYTES an die App und zurueck: es
gibt kein Versionsfeld, nur die Laenge verhandelt (`nrf52_ble.cpp:354,380`
vor diesem Commit). Die App haengt damit nicht nur an `sizeof`, sondern an
jedem Offset, jeder Reihenfolge und jedem Typ eines jeden Mitglieds. Deshalb
kommt dieser Schritt VOR dem Struct-Merge aus K02 (`b0adf723`): ohne Einfrieren
wuerde die App entweder das Schreiben wegen falscher Laenge ablehnen, oder --
schlimmer, falls die Groesse zufaellig noch passt -- es annehmen und die
Konfiguration eines Nutzers still ueber falsch ausgerichtete Felder verstreuen.
`src/nrf52/ble_settings_v1.h` friert den heutigen Stand in einem eigenen Typ
`s_ble_settings_v1` ein, unabhaengig vom lebenden Struct, mit einem
`static_assert` auf die Gesamtgroesse und auf `offsetof()` fuer alle 131
Mitglieder:

```
static_assert(sizeof(s_ble_settings_v1) == 2000, "s_ble_settings_v1 total size drifted from the frozen v1 wire contract");
static_assert(offsetof(s_ble_settings_v1, node_call) == 10, "v1 offset drift: node_call");
```

Ein Umbenennen, Umsortieren oder Vergroessern eines Mitglieds verschiebt die
`offsetof()` aller nachfolgenden Felder und laesst den Build sofort scheitern
statt die Konfiguration eines Nutzers still zu zerstreuen. Die Zahlen sind
DURCH KOMPILIEREN gegen den echten nRF52-ARM-Toolchain gewonnen, nicht von
Hand gerechnet. `src/nrf52/nrf52_ble.cpp` uebersetzt jetzt Mitglied fuer
Mitglied ueber `bleSettingsToV1()`/`bleSettingsFromV1()`
(`src/nrf52/ble_settings_v1.cpp`) statt den Speicher direkt zu kopieren, und
jede Laengenpruefung/jedes `setFixedLen`/`notify` nutzt `sizeof(s_ble_settings_v1)`.
Unabhaengig von der eigenen Testsuite verifiziert: eine temporaer eingebaute
Sonde im ARM-Build bestaetigte `sizeof(s_ble_settings_v1) == sizeof(s_meshcom_settings)`
(2000 Byte) und Offsetgleichheit fuer alle 131 Mitglieder, mit einer bewusst
falschen Gegenprobe zuerst, um zu beweisen, dass die Sonde ueberhaupt
ausgewertet wurde und nicht von einem inkrementellen Build uebersprungen
wurde. Dabei gefunden und noch im selben Commit behoben: CONC-17, ein
Nebenlaeufigkeitsfehler, den keine Host-Suite sehen kann, weil FreeRTOS-
Praeemption sich dort nicht reproduzieren laesst -- die erste Fassung rief
`bleSettingsFromV1()` auf dem GETEILTEN Staging-Puffer AUSSERHALB des
kritischen Abschnitts auf, waehrend der alte Code diesen Puffer nur innerhalb
davon beruehrte; da `settings_rx_callback()` im BLE-Task laeuft und
`applyPendingBleSettings()` unterbrechen kann, haette ein zweites
Einstellungs-Schreiben mitten in der Umwandlung zwei Abbilder
zusammengespleisst und das Ergebnis sowohl angewendet als auch gespeichert.
Das gestagte Abbild wird jetzt unter dem Lock kopiert, die Umwandlung liest
die Kopie. Kostet 2000 B statisch (RAM 35,1% -> 35,9% auf `wiscore_rak4631`).
Der Host-Compiler legt den Struct ANDERS aus als ARM (2008 statt 2000 Byte,
mehrere double-ausgerichtete Offsets verschieben sich um 4) -- der
Offset-Vertrag ist deshalb `#ifndef NATIVE_BUILD` bewacht, die native Suite
beweist Byte-Treue nur gegen ihr eigenes host-spezifisches Golden-Image, nie
gegen das Drahtformat selbst.

**23. Das eingefrorene Layout wird von K02s Struct-Merge weiterverwendet, nicht
neu erfunden** (`b0adf723`). Der nRF52-Legacy-Blob wird durch `s_ble_settings_v1`
gelesen, byte-identisch zu dem, was offizielles v4.35t schreibt; sein CRC32
wird notiert (`/legacy_blob.crc`), eine seither umgeschriebene Datei erzwingt
die Migration erneut und verwirft dabei bewusst einen veralteten
Zaehler-Stand, damit der Blob-Zaehler gewinnt. Und, aus derselben Freeze-Logik
heraus: `bleSettingsFromV1()` kopiert `node_msgid` nicht mehr ins Struct
zurueck, sodass weder ein Konfigurationsimport noch ein BLE-Einstellungsschreiben
den in K02 (`b0adf723`) auf `counters_store.h` verlagerten Zaehler rueckwaerts
drehen kann (Advisor-Fund 1 des W3c-Verdikts).

**24. Die Telefon-Rahmung wird auf eine Funktion vereinheitlicht und bekommt ihren
ersten ausfuehrbaren Test** (`2d348d04`, R1-02 Schritt 1). `sendToPhone()` und
`sendComToPhone()` trugen dieselbe Drei-Wege-Weiche (0x91 mheard, 0x44 JSON,
sonst Text mit 0x40-Tag) fast wortgleich, bis auf einen Arm: der Text-Arm von
`sendComToPhone()` kopierte `blelen-1` statt `blelen` -- ein Byte zu wenig.
Der Com-Text-Arm ist unerreichbar (`BLEComToPhoneBuff` hat genau zwei
Erzeuger, `sendBleJsonRegister()` und den mheard-Versand, beide schreiben
`buffer[0] = 0x44`), sodass im Feld nichts verloren ging -- aber die Weiche war
trotzdem falsch, und die massgebliche Fassung war die von `sendToPhone()`, wo
der Text-Arm wirklich laeuft und wo die 58 Aufrufer von `addBLECommandBack()`
ankommen. Beide Sender rufen jetzt `blePhoneFrame()` (`src/ble_phone_frame.h`),
mit 8 Host-Faellen in der neuen `native_ble_phone_frame`-Umgebung -- der
ERSTE ausfuehrbare Test, den diese Rahmung je hatte, weil sie bislang nur ueber
BLE mit einem echten Telefon beobachtbar war und deshalb ein
Ein-Byte-Unterschied zwischen zwei Kopien unbemerkt ueberleben konnte. Zwei
Fallen dabei entschaerft, bevor sie in den Build kamen: die Summe `blelen + 2`
war unabhaengig vom tatsaechlich kopierten Ausschnitt (`blelen-1`, `blelen`
oder `blelen+1`) -- haette der Helfer eine Laenge zurueckgegeben und der
Aufrufer sie gesendet, waere JEDER BLE-Rahmen um 1 bis 3 Byte geschrumpft,
deshalb liefert die Funktion `bool` und die Sendelaenge bleibt unangetastet;
und die erste Fassung des Ablehnungspfads hatte `ComToPhoneRead` nicht
vorgerueckt, was diesen Ring dauerhaft haette blockieren koennen. Gemessen:
-976 B Flash ueber 32 Envs, RAM unveraendert.

**25. Randfund derselben Woche, ein Aufraeumen des Pruefwerkzeugs statt der
BLE-Settings selbst: der Lint gegen handgeschriebene `extern`-Deklarationen
wird auf `nrf52_ble.cpp` ausgeweitet und findet dort einen ins Leere
zeigenden Verweis** (`2620110c`). `carve_extern_lint.py` deckte bislang nur
vier ausgegliederte Uebersetzungseinheiten ab, obwohl die Gefahr -- ein
`extern` mit gedrifteter Deklaration kompiliert und linkt STILL und liest das
Objekt danach mit falscher Breite, unsichtbar fuer Compiler (eine TU auf
einmal) wie Linker (Namen, nicht Typen) -- keine Eigenschaft des Ausgliederns
ist, sondern des Handschreibens selbst. Die Ausweitung auf zehn Dateien fand
in `nrf52_ble.cpp` ein `extern bool bInitDisplay;` gegen NICHTS -- der Name
existiert sonst nirgends im Baum, weder als Definition noch als zweite
Verwendung. Entfernt. Beruehrt das BLE-Settings-Drahtformat selbst nicht,
teilt mit den drei anderen Commits nur die Datei `nrf52_ble.cpp`.

## K05 UDP-Rahmen und Gateway-Dienst

**26. Gemeinsamer Vertrag fuer den UDP-Rahmen und den Gateway-Dienst, reine Restrukturierung**
(`03c4ba65`, `35b8823f`, `329b1bac`, `85f0c82e`, `8c48243c`). Bisher lag der eingehende
UDP-Frame-Handler, die Socket-Schreib-Primitiven, der Ausgangs-Drain und der Gateway-Service-
Block je einmal in `udp_functions.cpp`/`nrf_eth.cpp`/`nrf52_main.cpp` verankert -- Uebersetzungs-
einheiten, die wegen ArduinoJson, `esp_task_wdt.h`, SPI und der WisBlock-API auf keinem
Host-Build kompilieren, sodass keine Signaturaenderung sie je nativ testbar gemacht haette. Fuenf
Carve-outs loesen das: `handleUdpFrame_esp32()`/`handleUdpFrame_nrf52()` (C1/U1, jetzt in
`src/esp32/udp_frame_esp32.cpp` und `src/nrf52/udp_frame_nrf52.cpp`), die Socket-Primitiven
`udpBeginRaw_`/`udpWriteRaw_`/`udpEndRaw_` (C2), der Ausgangs-Ring-Drain (U2, `udp_drain_esp32.cpp`/
`udp_drain_nrf52.cpp`) und der Gateway-Service-Block (C4, `gateway_service_esp32.cpp`/
`gateway_service_nrf52.cpp`). `src/udp_frame.h`, `src/udp_drain.h` und `src/gateway_service.h`
sind die neuen, plattformneutralen Vertraege: sie deklarieren nichts Drittes, sondern schreiben
fest, was beide Seiten leisten muessen, und mit ihnen die noch offenen Verhaltensunterschiede.
Keine Verhaltensaenderung beabsichtigt und keine gemessen: 32/32 Envs bauen, RAM +0 auf allen
ESP32-Envs, die Regionen auf `ttgo_tbeam`/`E22_XML-DevKitC` sind byte-identisch. Die drei
nRF52-Boards wachsen um 16-24 B RAM -- nachverfolgt statt hingenommen: `nrf_eth.h` (fuer die
`NrfETH`-Klasse) zieht `IPAddress.h` in jede neue Uebersetzungseinheit, deren
`const IPAddress INADDR_NONE(0,0,0,0)` mit interner Bindung dabei je Datei eine eigene Kopie
bekommt (MEM-05, 17 statt 16 Kopien). Das Gegenstueck dieser Carve-outs -- was aus
`nrf_eth.cpp`/`nrf52_main.cpp` verschwindet -- steht in einem anderen Kapitel dieses Changelogs
(nRF52-Netzstack); dort haelt `85f0c82e` auch fest, dass die Zero-Scan-Grenze im nRF52-Pfad zu
diesem Zeitpunkt noch offen bleibt (siehe naechster Punkt).

**27. SEC-05-Paritaet: nRF52 las beim Zero-Scan ein Byte hinter dem Datagramm, und das konnte den
Verdikt kippen** (`3d2ce699`). `cd88ae12` hatte diese Fehlerklasse auf der ESP32-Seite bereits als
SEC-05/SEC-06/BUG-12 behoben, ohne eine nRF52-Datei zu beruehren -- der Carve C1 machte den
Unterschied erst sichtbar, weil beide Schleifen jetzt nebeneinander in eigenen Dateien liegen.
`src/nrf52/udp_frame_nrf52.cpp:64` lief mit `for (int i = 0; i < packetSize; i+=2)`, obwohl der
Schleifenkoerper `inc_udp_buffer[i]` UND `inc_udp_buffer[i + 1]` liest; bei einem ungerade langen
Datagramm liest die letzte Iteration ein Byte hinter dem Empfangenen. Nicht nur ein Out-of-Bounds-
Read: `zerocount` erhoeht sich um 2 je Nullpaar und wird bei jedem Nicht-Null-Paar auf 0
zurueckgesetzt; mehr als `MAX_ZEROS` verwirft das Frame. Der Regressionstest fuettert ein
15-Byte-Allnull-Datagramm mit einem vergifteten Byte an `buf[len]` -- unter der alten Grenze
paart die letzte Iteration das letzte echte Byte mit diesem Byte ausserhalb des Puffers, setzt
`zerocount` auf 0 zurueck, und der Knoten NIMMT ein Frame an, das er haette verwerfen muessen.
Fix: `i + 1 < packetSize`, dieselbe Form wie auf ESP32. Vorher/nachher belegt: 2 von 14 Faellen
schlagen fehl, wenn man die Grenze zurueckdreht, 14/14 bestehen mit dem Fix; der Zero-Scan-Block
in `test_udp_frame_twin` wandert dabei vom DRIFT- ins AGREEMENT-Segment.

**28. Der Aufrufer entscheidet ueber die Verbindungspolitik, und entscheidet klueger** (`3be9a9da`
DR-20, `d7b4b96c` H6-01). Vorher rief `handleUdpFrame_esp32()` bei zu vielen aufeinanderfolgenden
Nullbytes selbst `resetMeshComUDP()` auf -- eine volle WiFi-Teardown/Reconnect-Sequenz aus einem
reinen Frame-Parser heraus. DR-20 dreht die Signatur auf einen Rueckgabewert (0 behandelt, 1 zu
viele Nullen) und verschiebt den Reset-Aufruf zum einzigen Aufrufer, `getMeshComUDP()` in
`src/udp_functions.cpp` -- symmetrisch zu `NrfETH::getUDP()`, das beim gleichen Verdikt DHCP
zuruecksetzt. Der U1-Zwillingsdiff schliesst dabei auf 37/37 Uebereinstimmung, aber weil der
Reset-Aufruf die instrumentierte Uebersetzungseinheit verlaesst, nicht weil sich Socket-Reset und
DHCP-Reset angeglichen haetten -- die Plattformen bleiben bewusst verschieden. H6-01 geht darueber
hinaus: ein einzelnes fehlerhaftes UDP-1990-Datagramm loeste auf dem Heltec V3 die komplette
Teardown-Sequenz aus (`Udp.stop`, `WiFi.disconnect(true,true)`, Neustart-Anfrage) -- ein
"timeout when WiFi un-init", ein 9-Sekunden-Ausfall und mitunter ein Folge-Reboot, obwohl jeder
LAN-Host den Socket erreichen kann. `getMeshComUDP()` (`src/udp_functions.cpp:195-209`) prueft
jetzt zuerst `WiFi.status() == WL_CONNECTED`: solange die Verbindung steht, wird nur der UDP-
Socket neu bewaffnet (`Udp.stop()` + `Udp.begin(LOCAL_PORT)`, TX-Fehlerzaehler auf 0), die volle
Teardown bleibt Fallback fuer einen gescheiterten Bind und den bereits toten Heartbeat-/
MAX_ERR_UDP_TX-Pfad. Vorher/nachher auf dem Produktivabbild reproduziert: vorher Disconnect-Grund
8 plus Un-init-Timeout, nachher "UDP socket re-armed, WiFi kept" bei fortlaufendem Server-Beat.
Im selben Commit, unabhaengig davon: `--onewire gpio <n>` wird jetzt ueber `cmdStoreInt` auf
0..99 begrenzt (vorher nahm die Firmware jeden Wert entgegen, obwohl Werte ausserhalb dieses
Bereichs auf der Hardware ohnehin verworfen werden).

**29. ESP32-Ausgangs-Drain pruefte das falsche Ergebnis, und der Fehlerzaehler zaehlte Lebenszeit
statt Serie (DR-24/DR-25)** (`3be9a9da`). `sendMeshComUDP()` in `src/esp32/udp_drain_esp32.cpp`
wertete bis dahin `udpWriteRaw_esp32()`s Rueckgabewert als Sendefehler -- `WiFiUDP::write()`
puffert nur und kann bei einem nicht-leeren Frame nicht scheitern, waehrend der tatsaechliche
Sendeversuch erst mit `udpEndRaw_esp32()` (`endPacket()`) passiert. Ein echter `endPacket()`-
Fehler blieb dadurch unsichtbar (der Fehlerzaehler `err_cnt_udp_tx` lief nie hoch), waehrend ein
vermeintlicher Schreibfehler die Funktion vor dem Slot-Advance zurueckkehren liess und denselben
Slot beliebig oft wiederholt haette. Fix: beide Aufrufe unbedingt ausfuehren, `endPacket()`s
Ergebnis zaehlen. Dabei wurde sichtbar, dass `err_cnt_udp_tx` nirgends bei Erfolg auf 0 gesetzt
wurde und damit Fehler ueber die gesamte Laufzeit statt in Folge zaehlte -- auf ESP32 unbemerkt,
weil der Zaehler vor DR-24 nie lief; jetzt auf beiden Plattformen konsequent zurueckgesetzt, wie
es `MAX_ERR_UDP_TX` ohnehin unterstellt. Gleichzeitig bekommt das Leck am Ausgang (ein Frame mit
unkonfiguriertem Quell-Rufzeichen, das erst NACH dem Versand erkannt wird, `udp_drain_esp32.cpp`
und `udp_drain_nrf52.cpp`) einen eigenen Zaehler `stat_tx_leak_unconfigured`/
`logTxLeakUnconfigured()` statt den Eingangs-Drop-Zaehler mitzubenutzen -- ein steigender Wert am
Eingang zeigt, dass die Sperre GREIFT, am Ausgang, dass sie es NICHT tut; beide unter einem
Zaehler zu fuehren waere irrefuehrend. Die Definition der neuen Zaehl-/Logstelle selbst liegt in
`lora_functions.cpp` und ist an anderer Stelle dieses Changelogs beschrieben. Beleg: 34/34
Board-Envs, 33/33 native Envs (951 Faelle) -- die sechs U2-Zeilen ruhen auf handgebauten
Agreement-Tests, nicht auf Korpus-Arithmetik, da jedes Korpus-Rufzeichen DK5EN-* ist und die
Sperre so nie ausloest; das ist eine schwaechere Beweisstufe als die naechste Zeile und wird auch
so behandelt.

**30. nRF52-Gateway-Pfad auf neun Punkte ESP32-Paritaet gebracht (D1-01)** (`55a7b4c4`). Beleg ist
der Zwillingsdiff, nicht ein gruener Testlauf: vorher 37 Bloecke, 24 stimmen ueberein/13 weichen
ab, danach 36/1 -- zwoelf Bloecke kippen von ABWEICHUNG zu UEBEREINSTIMMUNG, keiner in die andere
Richtung. Der eine verbliebene Ueberlebende ist DR-20 (siehe oben), bewusst nicht Teil dieser
Welle. Was konkret schliesst: DR-02 ergaenzt in `src/nrf52/udp_frame_nrf52.cpp` dieselbe
Sperre gegen ein unkonfiguriertes Quell-Rufzeichen auf GATE-Frames, die ESP32 schon hatte (zweite
Tuer neben der primaeren LoRa-RX-Sperre); DR-04/DR-05 bringen die Positions-Anzeige
(`sendDisplayPosition()`) und das `bGATEWAY_NOPOS`-Veto auf die Position-Verzweigung; DR-06
korrigiert umgekehrt die ESP32-Seite -- `decodeAPRS()`s Rueckgabewert wird jetzt geprueft, sodass
ein von `decodeAPRS()` verworfenes Frame nicht mehr auf einem genullten `aprsMessage` weiterlaeuft
(vorher nur zufaellig durch `isUnconfiguredCall("")` auf dem leeren Rufzeichenfeld begrenzt);
DR-07/DR-19 vereinheitlichen die EXTUDP-Weiterleitung als eigenen, expliziten Typtest vor
`is_new_packet()` auf beiden Plattformen und entfernen einen sinnlosen `(uint8_t)`-Cast der
Laenge auf nRF52 (latentes Truncation-Risiko, sollte `UDP_TX_BUF_SIZE` je ueber 255 steigen);
DR-08 ergaenzt eine explizite Nulladressen-Pruefung im CONF-Guard auf nRF52
(`udp_frame_nrf52.cpp`), damit ein CONF-Datagramm mit eigener Quelladresse 0.0.0.0 nicht
durchrutscht, waehrend der aufgeloeste Gateway-Server selbst noch unaufgeloest ist; DR-09
ersetzt einen handgestrickten 7-Byte-ACK-Rahmen durch den gemeinsamen `buildAckPhoneFrame()`, der
zusaetzlich das bestaetigende Rufzeichen anhaengt -- vorher sah der ACK eines nRF52-Gateways auf
der Leitung anders aus als der eines ESP32-Gateways. Jeder einzelne Wert wurde gegen den
vorherigen Stand geprueft (Advisor-Pass); als bewusst NICHT umgesetzte Randnotiz haelt die Zeile
fest, dass diese Wachen nur den Relay-Pfad abdecken, nicht `SendAckMessage()`.

**31. C4d/DR-03: nRF52 bekommt die fehlende Warnstufe -- Diagnose, nicht die Sofortaktion**
(`1ba0b064`). ESP32 fuehrt eine zweistufige Heartbeat-Ueberwachung: warnen nach `HB_WARN_TIME`
(35 s) ohne Serverantwort, handeln nach `MAX_HB_RX_TIME` (65 s). nRF52 hatte nur die
handelnde Stufe -- die ersten 65 Sekunden gab es keine Ausgabe, das Log konnte also nie
unterscheiden, ob der Server still ist oder der Link tot ist. `src/nrf52/gateway_service_nrf52.cpp`
bekommt jetzt dieselbe Diagnose (benennt dabei den Ethernet- statt den WiFi-Zustand) plus den
Latch `hb_warn_logged`, geloescht in denselben vier Live-Traffic-Zweigen von
`handleUdpFrame_nrf52()` wie auf ESP32. Ausdruecklich NUR die Diagnose ist portiert, NICHT die
ESP32-Sofortaktion (dort setzt Stufe 1 zusaetzlich die UDP-Sitzung zurueck, wenn der Link auch
unten ist) -- das nRF52-Gegenstueck waere `resetDHCP()`/`initethfixIP()`, und das ist als
eigenes Risiko (N-20: jeder Retry resettet den W5100S hardwareseitig) unbehoben aufgeschrieben,
nicht heimlich mitgezogen. Sechs Testfaelle pruefen den Latch, nicht den Text, darunter vier
vom Advisor nachgefundene Loeschstellen (BEAT/CONF/OTHER auf beiden Plattformen), die vorher
durch keinen Test gedeckt waren.

**32. DR-14 dokumentiert und mit Test gesichert, keine Verhaltensaenderung** (`6b03bd37`,
`src/nrf52/gateway_service_nrf52.cpp`). nRF52 ruft `sendUDP()` nur auf, wenn `getUDP()` gerade
"kein Paket empfangen" meldete, ESP32 leert den TX-Ring dagegen unbedingt in jedem Durchlauf --
eine durch die gemeinsame SPI-Bus-Nutzung von W5100S und SX1262 auf dem RAK4631 begruendete,
bewusste Asymmetrie ("both-valid"), keine Drift, die zu vereinheitlichen waere. Zwei
unabhaengige Leser hatten sie zuvor als Bug gelesen und eine Vereinheitlichung vorgeschlagen (der
urspruengliche DRY-Audit und die M2-Driftmatrix-Zeile); deshalb jetzt an einem Test festgemacht
(`test_gateway_service_twin`, `test_dr14_nrf52_sends_only_when_no_packet_received`) statt nur an
einem Kommentar. `d82aea0c` schliesst dazu passend die Dokumentationsseite ab: `src/gateway_service.h`
und `src/udp_frame.h` hoerten auf, bereits entschiedene Fragen (u. a. DR-18 Teil 1, der EXTUDP-
Typtest) noch als offene Driftmatrix-Punkte zu bewerben -- reiner Kommentar, kein Codeverhalten
aendert sich.

**33. Mechanische Anpassung an die `aprsMessage`-Feldbreiten** (`f50cc954`). Mit der Umstellung von
`aprsMessage` auf feste `char[]`-Felder (an anderer Stelle dieses Changelogs beschrieben) wechseln
`source_call`/`destination_call` in `src/esp32/udp_frame_esp32.cpp:60-61` und
`src/nrf52/udp_frame_nrf52.cpp:55-56` von `char[20]` auf `MC_CALL_LEN_Z` (21) -- vorher war das
Rufzeichenfeld im UDP-Frame-Handler ein Byte zu kurz, sichtbar erst, als GCC die feste
Feldbreite statt eines `String` pruefen konnte. `src/esp32/udp_drain_esp32.cpp` und
`src/nrf52/udp_frame_nrf52.cpp` folgen mit `.c_str()`-Aufrufen, die auf direkte Feldzugriffe
umgestellt werden. Keine der drei Aenderungen ist hier eine neue Entscheidung; sie halten das
UDP-Modul nur konsistent mit der Struktur, deren Substanz in einem anderen Kapitel liegt.

## K08 APRS und Textcodec

Hinweis zur Einordnung: Die APRS-Positionsdecoder-Arbeit dieses Zweigs (`/R=`, `/U=`, `/I=`, `#name`, der `/Y=`-Puffer-Fix) ist bereits Teil dieses Zweigs, weil Upstream unsere frueheren PRs gemerged hat -- sie wird hier nicht erneut als neu ausgewiesen. Was in diesem Kapitel tatsaechlich neu ist, ist reine Restrukturierung: eine Extraktion doppelten Codes (D3-01) und zwei kleine Typ-/Persistenz-Vereinheitlichungen (R2-04, W3), keine der vier Aenderungen behauptet einen Bug-Fix am Wire-Format.

### D3-01: vierzehn gleichfoermige `/X=value`-Bloecke zu einem Helfer zusammengezogen

**34. Reine Extraktion, kein Verhaltensunterschied: vierzehn nahezu identische Tag-Decode-Schleifen in `decodeAPRSPOS()` sind jetzt ein einziger Helfer** (`6b03bd37`, D3-01). Die Bloecke fuer Batt/Alt/Press/Hum/Temp/Temp2/QFE/QNH/GASRES/CO2/Version/Bus-Spannung/Strom/Telemetrie unterschieden sich nur in Tag-Buchstabe, `sscanf`-Format und Zielfeld; sie sind jetzt `aprsExtractTag()` (`src/aprs_functions.cpp:635`, Kommentarblock ab :583), aufgerufen an den bisherigen Stellen (z. B. `:813`-`:828`). Nettosaldo: -290 Zeilen in dieser Datei.

Der Kommentar ueber dem Helfer nennt explizit, was **nicht** hineingezogen wurde und warum: `/N` (NCNT, zweistelliger Tag mit Ziffernbereich, Offset +2, 3-Byte-Kappung statt 7), `/R` (GRC, eigener Puffer plus `;`-getrennte Gruppenliste) und `/D` (Digital, 8-Byte-Bitfeld mit eigener Gueltigkeitspruefung) -- eine Vereinheitlichung haette entweder ungenutzte Parameter erzwungen oder ihr Verhalten still veraendert. Ebenso dokumentiert und bewusst unangetastet gelassen: nur das erste Vorkommen eines Tags zaehlt, ein leerer Wert (`/X=/`) fuehrt zu einem no-op-`sscanf`, und ein zu langer Wert wird auf 7 Zeichen abgeschnitten statt zurueckgewiesen -- alles Bestandsverhalten, jetzt an einer statt vierzehn Stellen.

Ein echter, aber sehr kleiner Fund beim Bau des Helfers: `APRS_TAG_INT_AUTOBASE` ("%i", basisableitend) muss von `APRS_TAG_INT` ("%d") getrennt bleiben, weil `/V=` (Version) und `/Y=` (Telemetrie) als einzige `"%i"` nutzten -- ein Zusammenlegen haette z. B. `"010"` von oktal 8 auf 10 umgedeutet. Da jeder bisherige `/V=`/`/Y=`-Wert einstellig war, war der Unterschied bislang unsichtbar und ist jetzt durch zwei Tests (`test_v_und_y_lesen_oktal_wie_scanf_i`, `test_d_tags_leiten_keine_basis_ab`) in beiden Richtungen festgenagelt, mutationsgeprueft (Umschalten von 'V' auf `APRS_TAG_INT` liefert 8 statt 10 und schlaegt fehl).

### R2-04: `aprsMessage` traegt festen Text statt `String` -- 109 kB Flash zurueckgewonnen

**35. Sieben Arduino-`String`-Felder in `aprsMessage` werden zu festen `char[]`-Feldern; das ist eine Speicher-/Codec-Umstellung, kein Bug-Fix am Protokoll, deckt aber drei latente Fehler auf** (`f50cc954`, R2-04). `decodeAPRS()` baute jedes Feld bereits in einem lokalen `char[UDP_TX_BUF_SIZE]` und kopierte es dann in einen `String`, waehrend rund 110 Aufrufstellen per `.c_str()` wieder einen `char*` herausholten -- ein Serialisierungs-Umweg fuer Daten, die die Struktur nie verlassen, mit sieben malloc/free pro empfangenem Frame. Die neuen Feldbreiten (`MC_PATH_LEN` 121, `MC_CALL_LEN_Z` 21, `MC_PAYLOAD_LEN` 256, `src/aprs_structures.h:54-56`) sind aus den Decoder-Schleifen abgeleitet, nicht geschaetzt, und durch `static_assert` an `MAX_CALL_LEN`/`UDP_TX_BUF_SIZE` gebunden (`src/aprs_functions.cpp:18-20`). `src/mc_text.h` (neu, 159 Zeilen) haelt die acht Textoperationen, die die Umstellung braucht (Arduino-frei, 21 native Testfaelle); `mcAppend()` ist bewusst alles-oder-nichts, weil ein halb angehaengtes Rufzeichen im Pfad ein FALSCHES Rufzeichen auf der Luft waere.

Drei Fehler, die die Umstellung sichtbar machte, mit je eigenem Test:

- `decodeAPRS()`s Payload-Schleife hatte keine eigene Schranke und lief bis `rsize`, obwohl der 255-Byte-Stackpuffer nur bis dahin sicher ist und die Pruefung darueber bereits `MAX_APRS_FRAME_SIZE` (340) zulaesst -- in der Praxis durch Aufrufer-Limits (R1-06) nicht scharf, aber nur zufaellig sicher. Ein 340-Byte-Testframe ohne die neue Schranke ergab SIGABRT (`test_aprs_decode`).
- `initAPRS()` loeschte `msg_source_call` nie; als `String` war das Feld durch Konstruktion leer, als `char[]` blieb das Rufzeichen des vorigen Frames stehen (gefunden durch `test_aprs_corpus`, Fall f008).
- Der PONG-Zweig las `msg_payload+14`, wo `substring(14,17)` frueher auf `""` abgeklemmt haette; die Guard-Pruefung deckt nur ein 6-Zeichen-Praefix ab, ein kuerzeres Payload lief also ueber den Terminator hinaus. Die Klammerung ist jetzt explizit wiederhergestellt.

`src/bp_notice_frame.h` und `src/via_functions.cpp` sind die beiden Aufrufstellen in diesem Kapitel, die auf `mcSet()`/`mcAppend()`/`mcStartsWith()`/`mcIndexOfStr()`/`strcmp()`/`is_equ()` umgestellt wurden, an Stelle von `String`-Zuweisung, `.concat()`, `.startsWith()`, `.indexOf()` und `==`-Stringvergleich. Eine Verhaltensaenderung ist dabei bewusst in Kauf genommen: ein Rufzeichen laenger als 20 Zeichen wird jetzt schon beim Aufteilen verworfen statt erst spaeter durch `checkRegexCall()` -- beide Pfade verwerfen den Frame, das Ergebnis ist dasselbe, nur der Zeitpunkt aendert sich.

Verifikationsstand laut Commit: 34/34 Board-Envs, 33/33 native Envs (948 Faelle), `selftest.sh` exit 0, Flash-Ersparnis von 109 068 B ueber 34 Envs gemessen. Der behauptete Heap-Gewinn selbst ist ausdruecklich unbewiesen -- er braucht laut Commit noch die einstuendige `--heap`-Bankmessung, die hier nicht vorliegt.

Am Rande in `src/aprs_structures.h`: derselbe Umbau wurde spaeter auf `mheardLine` uebertragen (`79242eda`) und fuegte dort `MC_DATE_LEN`/`MC_TIME_LEN` hinzu; die eigentliche Aenderung (mheard-Pfad) liegt aber in `mheard_functions.cpp`, ausserhalb dieses Kapitels, und wird hier nicht als eigener Punkt gefuehrt.

### W3: `node_msgid` erreicht Flash einmal je 100 Frames statt einmal je Frame

**36. Reine Persistenz-Optimierung, kein Wire-Format-Fix: der Message-ID-Zaehler wird jetzt in Bloecken statt bei jedem gesendeten Frame gesichert** (`bce95db5`). Der Zaehler muss einen Neustart ueberleben -- ein bei 0 neu startender Knoten wiederholt IDs, die Nachbarn noch in ihrem Dedup-Ring halten, und deren Frames werden dann verworfen. Das bisherige Verfahren zahlte dafuer mit einem `save_settings()` je gesendetem Frame: auf dem nRF52 ein ~1,5-kB-Dateischreiben plus Umbenennung auf einem 28-kB-Dateisystem, aus acht Aufrufstellen in `loop_functions.cpp`; auf ESP32 ein NVS-Commit. `src/msgid_counter.h` (neu, 70 Zeilen) und `src/msgid_counter.cpp` (neu, 35 Zeilen) kapseln jetzt die Highwater-Mark-Logik, Arduino-frei und nativ getestet: der Zaehler wird nur bei einem Vielfachen von `kMsgIdPersistStep` (100) gesichert, beim Laden um einen ganzen Schritt vorgezogen und dieser vorgezogene Wert sofort zurueckgeschrieben -- jede ID zwischen dem gespeicherten und dem vorgezogenen Wert gilt als verbraucht, genau der Bereich, den ein unsauberes Herunterfahren unbemerkt haette nutzen koennen.

Das Zurueckschreiben beim Laden ist der tragende Teil, nicht nur eine Optimierung: ohne es wuerde Flash weiterhin den VORHERIGEN Block nennen, waehrend der Knoten schon IDs aus dem neuen Block vergibt -- ein Absturz in den ersten hundert Frames nach einem Boot liesse den naechsten Boot wieder dort beginnen, wo dieser begann, also genau die Wiederverwendung, die die Persistenz verhindern soll. Ein Property-Test durchlaeuft deshalb jeden moeglichen Absturzpunkt ueber zwei Bloecke hinweg und schlaegt genau bei dieser Mutation fehl.

Auf DK5EN-90 gemessen: `node_msgid` 24 -> 124 -> 224 ueber zwei Boots, je ein `save;ok` pro Boot. Nicht auf Hardware verifiziert, laut Commit bewusst: dass ein gesendeter Frame keinen Flash-Schreibvorgang mehr ausloest, folgt aus den acht geaenderten Aufrufstellen und ist nativ abgedeckt; ein Beweis auf der Bank haette einen Sendevorgang auf dem echten 433-MHz-Mesh gebraucht, was als Risiko nicht eingegangen wurde.

Nebenbefund, notiert und nicht behoben: `node_ackid` wird geladen, gespeichert und nirgends im Quellbaum inkrementiert -- ein toter Zaehler.

## K06 MHeard

**37. Ein Nebenbefund im Zuge der W2-Speicherpruefung: `mheard_functions.cpp`
gewinnt eine kleine Stack-Absicherung** (`2991034e`, W2). Die grosse Auszahlung
dieser Welle liegt anderswo (`ota_html[]` bekommt `const` und wandert von
`.data` nach `.rodata`, -24 320 B RAM auf beiden Safeboot-Envs); an
`mheard_functions.cpp` selbst aendert dieser Commit nur neun Zeilen und
bereitet damit den Boden fuer die drei folgenden, mheard-eigenen Commits, ohne
selbst eine mheard-Behauptung zu tragen.

**38. mheardLat[]/mheardLon[] werden von `double` auf `float`, mit dem
Genauigkeitsverlust gemessen statt geschaetzt** (`b6416efe`, R3-12). Gemessen
je Env: -648 B auf zwoelf Envs, -640 auf zehn, -400 auf einem, -240 auf neun,
0 auf den beiden Safeboot-Envs ohne mheard; Flash +608 B ueber alle 34 Envs
(~19 B je Env fuer die Konvertierungen an den Zuweisungsstellen). Die
Genauigkeit ist an 600 000 zufaelligen Koordinatenpaaren durch einen
Float-Rundweg GEMESSEN, nicht behauptet: groesster absoluter Fehler
0,00000763 Grad, rund 0,85 m. Bei der Web-GUI-Formatierung `%06.3lf`/`%07.3lf`
rendert das bei 0,15 % der Koordinaten -- ungefaehr eins von 670 -- eine andere
letzte Ziffer, und zwar nur dort, wo der wahre Wert genau auf einer
Rundungsgrenze liegt. Fuer eine Nachbarschaftsanzeige (kein Navigationsziel)
ist 0,85 m kein Problem; die letzte-Ziffer-Abweichung steht hier, damit ein
kuenftiger Mitschnitt-Diff mit genau diesem Muster als das hier erkannt wird
und nicht als Regression. Zwei Dinge, die dieser Umbau NICHT allein sein
durfte: `mheardLat[]`/`mheardLon[]` werden in `src/lora_functions.cpp` und
`src/web_functions/web_functions.cpp` per Hand als `extern` neu deklariert --
eine vergessene Stelle linkt STILL und liest das Array mit falscher Breite,
kein Compiler sieht beide Uebersetzungseinheiten, der Linker prueft nur Namen.
Alle drei Schreibweisen wurden zusammen geaendert, und `carve_extern_lint.py`
deckt diese Dateien seither ab (siehe K03, `2620110c`, unmittelbar davor).
Und: beide Arrays landen roh in `/mheard.dat` auf dem T-Deck
(`file.write((uint8_t*)mheardLat, sizeof(mheardLat))`), also aendert sich mit
dem Typ auch das Dateilayout -- der Ladepfad vergleicht `file.size()` gegen die
Summe der `sizeof()`-Werte und loescht die Datei bei Abweichung, sodass eine
bestehende Datei einmalig verloren geht und neu aufgebaut wird. Selbstheilend,
aber es passiert, und das steht jetzt an der Deklaration statt erst wieder
entdeckt werden zu muessen.

**39. Der Text-Codec fuer mheard-Eintraege ist weg -- 83 kB RAM und 54 kB Flash
zurueck, quer ueber alle 34 Envs** (`09524068`, R2-01). Jeder mheard-Eintrag
lag als pipe-getrennte Zeichenkette in `mheardBuffer[MAX_MHEARD][60]` und
wurde bei jedem Lesen auf DREI verschiedene Arten wieder zerlegt:
`decodeMHeard()` zeichenweise mit einem Arduino-`String`-Anhaengen pro
Zeichen (~55 Iterationen je Lesevorgang), `sendMheard()` ein ZWEITES Mal ueber
elf `getValue()`-Aufrufe, von denen jeder einen `String` allokiert und neu
scannt, plus zwei Kodierstellen mit `snprintf` des gesamten 60-Byte-Datensatzes
-- ein Serialisieren-und-wieder-Deserialisieren fuer Daten, die das Geraet nie
verlassen, auf genau den Pfaden, die `--mheard`, das JSON-Register und die
Web-Oberflaeche bedienen, auf einer Plattform, wo Heap-Verkehr pro
Ausgabezeile der dokumentierte Grund fuer scheiternde BLE-Verbindungsaufbauten
ist. Jetzt ein 20-Byte-Struct (`src/mheard_record.h`, `struct MheardRecord`)
und Feldkopien. Gemessen ueber alle 34 Envs: RAM -3200 B (22 Envs) / -2000 (1)
/ -1200 (9) / 0 (2 Safeboot), zusammen -83 200 B; Flash -876 bis -4928 B je
Env, zusammen -54 676 B; `mheard_functions.cpp` 117 Zeilen kuerzer. Die
Rundung musste exakt sein: `mhdoc["DIST"]` gibt `mh_dist` ROH aus, die
gespeicherte Genauigkeit IST also das Drahtformat, und der alte Text hielt
genau eine Nachkommastelle (`%.1lf`), die der Decoder gerundet zurueckliest.
Zwei naheliegende Kodierungen dafuer waren falsch und wurden vom neuen
`native_mheard_record`-Test gefangen: `(long)(x*10 + 0.5)` rundet Haelften
immer nach oben, waehrend `printf("%.1lf", ...)` auf die gerade Ziffer rundet
(1,25 wird zu "1,2"); und `nearbyint(x*10) / 10` rundet zwar auf die gerade
Ziffer, aber die Multiplikation zerstoert vorher die entscheidende
Information (0,05 als `double` liegt leicht ueber 0,05, `0,05*10` rundet aber
exakt auf 0,5 und geht auf die gerade Null -- `printf` sieht den
Originalwert und sagt "0,1"). Eine binaere Skalierung kann eine dezimale
Rundung nicht nachbilden; `mheardRoundDist()` (`src/mheard_record.h`)
formatiert deshalb einmal je Ablage genau wie frueher, statt bei jedem Lesen.
Bewusst erhalten: die REP-Kodierstelle mischte zwei Structs (Datum, Typ,
Pfadlaenge, Mesh und `ncount` aus `mheardLine`, aber `hw`/`mod`/`rssi`/`snr`/
`dist` aus `mheardLine_save`, den Funkwerten des BESTEHENDEN Eintrags) -- ein
Zusammenlegen haette dieses Verhalten still geaendert, es ist deshalb Feld
fuer Feld nachgebildet, mit einer Notiz warum. Nebenfund, mit behoben: eine
Zeile `json += "\"dist\":" + String(mheardLine.mh_dist + ",", 1);` ist
`double + const char*`, unzulaessiges C++, das nur ueberlebte, weil es unter
`#ifdef HEAP_TEST` liegt und `HEAP_TEST` nirgends im Baum definiert ist.
Persistenz auf `/mheard.dat` (nur T-Deck) aendert das Layout, ueber denselben
selbstheilenden Groessenvergleich abgefangen, der auch bei `b6416efe` greift.

**40. `mheardLine` traegt jetzt Text statt `String`, und der Typwechsel deckt zwei
Vergleichsfehler auf, die sonst still ins Feld gegangen waeren**
(`79242eda`, zweite Haelfte von R2-04). Die sieben `String`-Felder von
`struct mheardLine` werden feste `char[]` (Breiten `MC_DATE_LEN`/`MC_TIME_LEN`
aus den Erzeugern selbst abgeleitet, nicht geschaetzt: `MC_DATE_LEN` 11 ist
exakt `getDateString()`s `currDate[11]`). Zwei Vergleiche waren vorher
`String == String` (Inhaltsvergleich) und waeren als `char[] == char[]` zu
ZEIGERvergleichen geworden -- immer falsch, ohne Warnung, ohne Absturz:
`mheard_functions.cpp:469`, der Ausschluss des EIGENEN Rufzeichens in
`updateHeyPath()` (der Knoten haette sich sonst selbst in seine eigene
Pfadtabelle eingetragen), und `mheard_functions.cpp:638`, die
"HG"-Erkennung, die das Gateway-Bit in `mheardPathLen` setzt (die Markierung
waere aus der GUI verschwunden). Beide sind jetzt `is_equ()`, beide
mutationsgeprueft: dreht man einen Vergleich zurueck auf `==`, faellt genau
sein Test und sonst keiner. Der Advisor-Lauf zu diesem Commit fand zudem, dass
Stack statt Heap auf nRF52 KEIN neutraler Tausch ist: `sizeof(mheardLine)`
steigt von ~112 auf 584 Byte, und drei Funktionen (`sendMheard()`,
`showMHeard()`, `sub_page_mheard()`) halten so eine Struktur lokal auf dem
Loop-Task, der auf nRF52 nur 4 KB hat -- genau dort hatte der N-22-Vorfall
(`9ce62aa0`) schon einmal einen erschoepften Stack gemessen. Mit
`-fstack-usage` auf `wiscore_rak4631` je Funktion gemessen (vorher / nach
Umstellung / nach Fix): `sendMheard()` 728/1184/600, `showMHeard()`
160/656/72, `sub_page_mheard()` 232/696/128 Byte. Fix nach dem N-22-Muster:
die drei lokalen Strukturen wandern `#if NRF52_SERIES` nach BSS (drei
getrennte statische Variablen, kein gemeinsamer -- ein gemeinsamer muesste
uebersetzungseinheitenuebergreifend `extern` sein und koppelte die drei
Funktionen aneinander), kostet 1752 B BSS auf nRF52; ESP32 behaelt den
Stack-Puffer (8 KB Loop-Task). Der tiefste Loop-Pfad geht damit von 2072 Byte
vor diesem Commit ueber 2568 nach der reinen Umstellung auf 1984 Byte nach dem
Fix -- flacher als vorher.

**41. mHeard zeigt zuletzt-gehoert-zuerst, und die nRF52-Hauptschleife bekommt
einen eigenen 8-KB-Task, damit ein einzelnes UDP-Datagramm sie nicht mehr
zuruecksetzen kann** (`173a2970`, ETH-03 + DR-28). Zwei unabhaengige Funde in
einem Commit, disjunkte Dateien innerhalb der Welle: ETH-03 ist ein
Stack-Ueberlauf im Loop-Task (kein Ring-Ueberlauf, wie zunaechst vermutet), auf
der Bank reproduziert -- ein einzelnes Datagramm auf UDP 1799 setzte DK5EN-90
zurueck (`RESETREAS=0x4`, dieselbe Signatur wie N-22). Der Adafruit-Core gibt
dem Loop-Task fest 4 KB (`LOOP_STACK_SZ`, nicht per Flag aenderbar); R2-04
(`79242eda`, siehe oben) hatte `aprsMessage` auf ~590 B Inline-Text
umgestellt, und der Pfad `getExtern()` -> `sendMessage()` traegt zwei davon
plus einen 381-B-Rahmenpuffer. Fix: `nrf52loop()` und `captureDrain()` laufen
jetzt ueber `Scheduler.startLoop()` in einem eigenen 8-KB-Task ("mcloop"), der
Core-Loop wird mit `suspendLoop()` geparkt; schlaegt `xTaskCreate` fehl,
bleibt der 4-KB-Loop und der Bootlog sagt es. Damit ist die Klasse (N-22,
TM-43, ETH-03) geschlossen, nicht nur die Einzelinstanz. Ein zweiter Fund am
selben Fehlerpfad: die `"[RING] overflow"`-Meldung in `sendMessage()` testete
`iWrite == iRead`, was AUCH der leere Ring erfuellt -- sie feuerte bei jeder
Nutzernachricht und lenkte die Untersuchung einen Tag lang auf einen vollen
Ring, den es nicht gab; sie prueft jetzt `txRingDepth() >= MAX_RING-1`.
DR-28 (OPT-D16): `mheardSortedIndex()` (`src/mheard_functions.cpp:727`)
sortiert einen lokalen Indexvektor nach rollover-sicherem Alter (stabil,
Gleichstand behaelt den niedrigeren Slot); `showMHeard()`, `sendMheard()`,
`showMHeardTDECK()` und die Web-Seite iterieren jetzt darueber. Die
Speicherarrays selbst werden nie umsortiert (der LORA-Task schreibt sie
weiterhin direkt), alle Ausgabebytes bleiben gleich, nur die Reihenfolge
aendert sich. Zwei Tests, die zuvor die alte Slot-Reihenfolge festnagelten,
sind auf Zuletzt-gehoert-zuerst umgeschrieben (rot auf dem alten Code, gruen
jetzt), dazu fuenf direkte Tests fuer den neuen Helfer. Regressionsinstrument
fuer ETH-03, `tools/bench/eth03_probe.py`, faellt auf dem Stand vor diesem
Commit durch (Reboot nach einem Datagramm); der Durchlauf mit dem Fix war zum
Zeitpunkt dieses Commits noch offen, weil DK5EN-90 im seriellen
DFU-Bootloader haengengeblieben war und den physischen Doppel-Tap braucht.

## K09 LoRa-Schicht und Funkparameter

**42. Fehlende Einheiten-Konversion auf dem SX126x-Pfad behoben (RF-01, RF-02, RF-03, RF-05, RF-06)**
(`b676c808`). `node_freq`, `node_bw` und `node_cr` halten je nach Board unterschiedliche
Einheiten -- MHz/kHz/4-N-Nenner auf ESP32 (SX127x), Hz/Bandbreiten-Index/Index auf den
nRF52-Boards RAK4630, T114 und T-Echo (OPT-D14) -- und mehrere Schreibstellen haben die
Konversion schlicht vergessen. `--txbw` schrieb 125/250 direkt in `node_bw`, das
`lora_setchip_meshcom()` dem SX126x als rohen Bandbreiten-Enum uebergibt (0/1/2): der Funk
wurde mit dem Zahlenwert 125 oder 250 statt mit dem Enum konfiguriert. `--txcr` und `--txfreq`
konvertierten nur unter `#ifdef BOARD_RAK4630`, waehrend der indexbasierte Pfad tatsaechlich
`BOARD_RAK4630 || USE_HELTEC_T114 || BOARD_T_ECHO` ist -- T114 und T-Echo blieben unkonvertiert.
`lora_setcountry()`s Fall 7 (MAN, manuelle Einstellung) verglich gespeicherte Werte gegen
ESP32-Einheiten-Literale, sodass auf nRF52 kein Vergleich je traf und `--setcountry 7` dort
grundsaetzlich alles auf die Standardwerte zurueckwarf statt die manuellen Werte zu erhalten.
Neu: `src/radio_units.cpp`/`.h`, sechs reine Konversionsfunktionen mit explizitem
`indexed`-Flag statt Board-Makro-Abfrage, dadurch host-testbar; `lora_setchip.cpp:237-280`
normalisiert in Fall 7 jetzt ueber `getBW()`/`getFreq()`/`getCR()`, entscheidet in einer
Einheit und schreibt in der Einheit der Plattform zurueck. Verifiziert per
`test/test_radio_units` (12 Faelle, native) und `test/golden/radio_units_lint.py` (18
Verstoesse vorher, 0 danach); RAM +0 auf allen Envs. Nicht auf dem Bench bestaetigt --
RF-02/RF-05 betreffen T114/T-Echo, die auf keiner Bench dieser Kampagne existieren.

**43. Schutzband-Formel um Faktor 10 falsch: `/100.0` statt `/1000.0` (RF-04)**
(`136dacdd`). Die Umrechnung von kHz auf MHz fuer das Schutzband um die manuell gesetzte
Frequenz stand als `(bw_khz/2.0)/100.0` (`lora_setchip.cpp:259`) -- fuer 250 kHz Bandbreite
1,25 MHz statt der korrekten 0,125 MHz. Im 70-cm-Band machte das den zulaessigen Bereich zu
eng (431,25..437,75 statt 430,125..438,875 MHz); im 868-MHz-Sub-Band SRD860 (869,4..869,65
MHz) war das Schutzband dagegen unerfuellbar -- jede dort manuell gesetzte, legale Frequenz
wurde stillschweigend durch die 70-cm-Standardfrequenz ersetzt, ohne Fehlermeldung. Fix:
`/1000.0`. Wichtig fuer die Betriebssicherheit: nichts wurde je auf der falschen Frequenz
gesendet, da RadioLib bzw. die Semtech-API in der jeweils eigenen Einheit angesteuert wurden --
nur die Validierung war falsch. Verifiziert durch zwei neue Regeln in
`radio_units_lint.py` (3 Verstoesse bei zurueckgedrehtem Fix, 0 danach); `env:native` kann
`lora_setchip.cpp` selbst nicht kompilieren, daher schuetzt hier der Source-Lint, nicht der
Unit-Test.

**44. Laendertabelle aus `lora_setcountry()` herausgezogen (C5-Carve-out, D3-05)**
(`f666e158`). Der 14 Faelle umfassende `switch` fuer die Laender-/Regionsprofile fuellte bisher
direkt `meshcom_settings`; `countryProfile(iCtry, out)` in den neuen Dateien
`src/country_profile.cpp`/`.h` ist jetzt eine reine Funktion ohne Abhaengigkeit auf
`meshcom_settings` oder Radio-Globals, die nur `CountryProfile out` befuellt --
`lora_setcountry()` wendet die sechs Felder danach an. Fall 7 (MAN) bleibt bewusst aussen vor,
da er vorhandene Werte validiert statt Literale zuzuweisen. Beim Herausziehen waere `default:`
sonst auch Fall 7 aufgefangen und haette `--setctry 7` still auf das EU-Profil zurueckgesetzt
-- also genau die manuellen Einstellungen zerstoert, die RF-03 kurz zuvor erst reparierte. Der
Fehler wurde innerhalb desselben Commits gefangen: `country_profile.cpp` hat jetzt ein
explizites `if (iCtry == 7) return false;` (`country_profile.cpp:119`), von
`radio_units_lint.py` durch einen eigenen Test abgesichert. 14 Tabellenfaelle byte-identisch
gegen den Stand vor dem Carve verifiziert.

**45. 14-Fall-`switch` durch statische Tabelle ersetzt (D3-05, zweiter Schritt)**
(`6b03bd37`). `country_profile.cpp` wird von einem 14-Fall-`switch` auf eine
`static const`-Tabelle (13 Zeilen plus separate Default-Zeile) umgestellt, -185 Zeilen, landet
in `__TEXT,__const` statt `.data`. Jeder Landes- und Plattformwert wurde gegen den vorherigen
Stand geprueft. Dabei dokumentiert, nicht behoben: `track_freq` ist eine gemeinsame Spalte fuer
beide Plattformen, obwohl `freq` getrennte MHz/Hz-Spalten hat. Jede Zeile ausser Polen (Code
15, `country_profile.cpp:95`) bezieht den Wert korrekt aus dem plattformspezifischen Makro
`LORA_APRS_FREQUENCY`; Polen traegt das Literal `434.855f`, was auf einem nRF52-Knoten als 434,855
Hz interpretiert wird und wie der 999-Sentinel unterhalb jeder RadioLib-Frequenzgrenze liegt.
Vorbestehender Defekt, originalgetreu uebernommen -- eine Korrektur braucht eine eigene Zeile
und einen Bench-Beleg, kein Nebenprodukt eines DRY-Refactors.

**46. RF-07 (kritischer Abschnitt) als Fehlalarm geschlossen** (`be697fa8`). Der Verdacht: die
`taskENTER_CRITICAL()`-Bloecke um die Display-Warteschlange (`lora_functions.cpp`) sind nur mit
`#if defined(BOARD_RAK4630)` bewacht und wuerden auf `heltec_t114`/`t_echo` wegkompiliert. Durch
`pio run -t compiledb` fuer beide Envs und Grep auf die tatsaechliche Compiler-Kommandozeile
verifiziert: `BOARD_RAK4630` ist auf beiden gesetzt, da `platformio.ini`s gemeinsames
`[nrf52_base]` es an alle drei nRF52-Boards vererbt. Reiner Kommentar-Fix
(`lora_functions.cpp:165-181`), keine Logikaenderung; die Doku warnt jetzt ausdruecklich davor,
die Bedingung auf `|| USE_HELTEC_T114 || BOARD_T_ECHO` zu erweitern, da das ein wirkungsloser
Schein-Fix waere.

**47. RF-08 (Sentinel-Frequenz 999 fuer Land "868") als Fehlalarm geschlossen** (`81d009a6`).
`country_profile.cpp` setzt fuer Laendercode 5 ("868") `track_freq = 999`, da es dort keinen
echten APRS-Track-Subkanal gibt. Ungeklaert war, ob dieser Wert einen Knoten in diesem Land im
Track-Modus tatsaechlich zum Senden auf 999 MHz bringen kann. Ergebnis: `--track` hat keine
Laender-Sperre, der Wert erreicht also `lora_setchip_aprs()`s ESP32-Zweig und
`radio.setFrequency(999)`. Jeder in diesem Baum instanziierte RadioLib-Treiber weist das aber
vor jedem Registerzugriff zurueck (SX1278 137..525 MHz, SX1262 150..960 MHz, SX1268 410..810
MHz) -- 999 liegt oberhalb aller drei Obergrenzen. Als `TRACK_FREQ_NONE_SENTINEL`
(`country_profile.cpp:43`) benannt und die gesamte Absicherungskette dokumentiert, inklusive
Warnung, die Kette vor jeder kuenftigen Aenderung neu herzuleiten. Zahlenwert unveraendert.

**48. `strCountry`-Tabelle von `String[17]` auf `const char* const[17]` umgestellt (R2-07/R3-06)**
(`2991034e`). Die Laendernamen-Tabelle in `src/lora_setchip.cpp:69` (deklariert in
`lora_setchip.h:14`) bestand aus 17 heap-gestuetzten Arduino-`String`-Objekten; jetzt 17
Zeiger ins `.rodata`, keine Allokation mehr. `getCountry()` gibt weiterhin `String` zurueck --
die Konversion passiert nur noch an dieser einen Stelle statt 17-mal beim Start.
`getCountryID()`s Vergleich wurde dabei bewusst auf `strCtry == strCountry[ic]` gedreht (statt
umgekehrt), weil `String::operator==(const char*)` eine garantierte Member-Ueberladung ist, die
umgekehrte Reihenfolge dagegen von einem impliziten temporaeren `String` abhaengt. Gemessen
zusammen mit der begleitenden R2-07-Aenderung (nicht Teil dieser Datei): -848 B RAM auf ESP32,
-648 B auf nRF52.

**49. Toter ESP32-Spinlock `displayMux` und irrefuehrende Kommentare entfernt** (`36c37b5f`). Der
`portMUX_TYPE displayMux` in `lora_functions.cpp` hatte seit Commit `4a250602`, der alle drei
`portENTER_CRITICAL(&displayMux)`-Aufrufe entfernte, keine einzige Nehmen/Geben-Stelle mehr --
nur die Variable und drei Kommentare, die einem Leser weiterhin einen Schutzmechanismus auf
ESP32 vorgaukelten. Tatsaechlich braucht es dort keinen: `OnRxDone()` laeuft auf ESP32
synchron innerhalb von `esp32loop()`, Erzeuger und Verbraucher sind derselbe Task. Auf nRF52,
wo sie es nicht sind, bleibt der bestehende `taskENTER_CRITICAL()`-Schutz unveraendert
bestehen. Variable und `extern`-Deklaration geloescht, die drei Kommentare durch die
zutreffende Beschreibung ersetzt (`lora_functions.cpp:153`ff).

**50. Getrennter Zaehler fuer Leck am Ausgang statt Wiederverwendung des Eingangs-Drop-Zaehlers
(DR-25)** (`3be9a9da`). Der Ausgangs-Drain fuer UDP erkennt ein Frame mit unkonfiguriertem
Quell-Rufzeichen erst NACH dem Versand -- ein Leck, kein Drop. Bislang war keine eigene
Zaehl-/Logstelle dafuer in `lora_functions.cpp` vorgesehen; ein steigender Wert am
Eingangs-Drop-Zaehler (`stat_rx_drop_unconfigured`/`logRxDropUnconfigured()`) bedeutet, dass die
Haupt-Sperre GREIFT, waehrend ein steigender Wert an dieser neuen Stelle bedeutet, dass sie es
NICHT tut -- beide Ereignisse duerfen nicht denselben Zaehler teilen. Neu:
`stat_tx_leak_unconfigured` und `logTxLeakUnconfigured()` (`lora_functions.cpp:270-286`),
gleiches Format (10-s-Markierungsfenster, roher `Serial.printf`, kein `printfdeb()`, das `;`
ausserhalb von `--debug csv` entfernt).

**51. `mheardLine`-Textfelder von `String` auf `char[]` umgestellt, zweite Haelfte von R2-04**
(`79242eda`). Die sieben `String`-Felder von `struct mheardLine` sind jetzt feste `char[]`
(erste Haelfte betraf `aprsMessage`, siehe unten). In `lora_functions.cpp`s `OnRxDone()` werden
die Zuweisungen auf `mcSet()` umgestellt (z. B. `mheardLine.mh_callsign`,
`mh_sourcepath`, `mh_sourcecallsign`, `mh_destinationpath`, `mh_date`, `mh_time`,
`mh_path_payload`); ein Inhaltsvergleich bei `mheardCalls[iset]` wechselt von
`.c_str()`-Vergleich auf `is_equ()`. Auf nRF52 wird die lokale `mheardLine`-Instanz in
`sub_page_mheard()`-Analogfunktionen als `static` statt auf dem Stack gehalten, da
`sizeof(mheardLine)` durch den Typwechsel von ~112 auf 584 Byte gestiegen ist und drei
betroffene Funktionen auf dem 4-KB-Loop-Task laufen, auf dem N-22 bereits einmal
`uxTaskGetStackHighWaterMark(NULL) == 0` gemessen hatte (Fix-Muster identisch zu N-22). Zwei
Vergleiche, die als `String == char*` Inhaltsvergleiche waren und als `char[]` sonst still zu
Zeigervergleichen geworden waeren, liegen in `mheard_functions.cpp`, ausserhalb dieser Datei.

**52. `aprsMessage`-Textfelder von `String` auf `char[]` umgestellt, 109 kB Flash zurueckgewonnen
(R2-04)** (`f50cc954`). Sieben `String`-Felder von `aprsMessage` werden zu festen `char[]`.
In `lora_functions.cpp`s `OnRxDone()` ersetzen `strcmp()`/`is_equ()` sowie die neuen
Hilfsfunktionen aus `src/mc_text.h` (`mcStartsWith()`, `mcIndexOfStr()`, `mcSliceToLong()`,
`mcTruncate()`, `mcSet()`, `mcAppend()`, `mcAppendChar()`) rund 30 vormalige `.c_str()`- bzw.
`String`-Methodenaufrufe (`lora_functions.cpp:1033-1490` u.a.). Eine lokale Callsign-Puffergroesse
wechselt von hartkodiert 20 auf `MC_CALL_LEN_Z` (`lora_functions.cpp:1033`). Gemessen ueber 34
Environments: Flash -109 068 B, RAM +15 024 B (davon +464 B je Env aus einem einzelnen
globalen `pendingDisplayMsg`). Verhaltensaenderung: ein "Rufzeichen" laenger als 20 Zeichen wird
jetzt schon beim Aufteilen verworfen statt erst Momente spaeter durch `checkRegexCall()`; beide
Pfade verwerfen das Frame ohnehin. Der Heap-Vorteil selbst ist unbewiesen und braucht laut
Commit noch die 1-Stunden-`--heap`-Wasserstandsmessung auf der Bench.

**53. `mheardLat`/`mheardLon` von `double` auf `float`, Praezisionskosten gemessen (R3-12)**
(`b6416efe`). Die `extern`-Deklaration in `lora_functions.cpp:116-117` wechselt von
`extern double mheardLat[MAX_MHEARD]`/`mheardLon` auf `extern float`, passend zur eigentlichen
Definition in `mheard_functions.cpp` (nicht Teil dieser Datei). RAM-Ersparnis je nach
Environment zwischen -240 B und -648 B (0 auf den zwei Safeboot-Envs ohne mheard), Flash
+608 B ueber alle 34 Envs (~19 B je Konversionsstelle). Praezision wurde gemessen, nicht
geschaetzt: 600 000 zufaellige Koordinatenpaare durch einen Float-Roundtrip ergaben einen
groessten absoluten Fehler von 0,00000763 Grad (~0,85 m); bei der Web-GUI-Darstellung mit
`%06.3lf`/`%07.3lf` rendert das bei 0,15 % der Koordinaten eine andere letzte Nachkommastelle
(rund 1 von 670, nur an Rundungsgrenzen). Fuer eine Nachbarschaftsanzeige ohne
Navigationsanspruch wird das als unbedenklich bewertet, aber ausdruecklich festgehalten, damit
ein kuenftiger Capture-Diff mit genau diesem Symptom nicht als Regression missverstanden wird.
Beide Arrays sind ausserdem in dieser Datei UND in `web_functions.cpp` von Hand als `extern`
redeklariert; das Fehlen einer der beiden Stellen wuerde still linken und mit falscher Breite
lesen, da der Linker Namen, nicht Typen, abgleicht.

## K04 Kommandotabelle und Setter

Dieses Kapitel deckt den Kommando-Dispatch ab: `command_functions.cpp`
(5773 Zeilen, das Herzstueck), die vier neuen Header
`command_match.h`/`command_setters.h`/`command_toggles.h`/`serial_command.h`,
die beiden plattformspezifischen seriellen Leser sowie die Telefon-Framing-
Funktion in `phone_commands.cpp`. 22 Commits, gruppiert nach Thema statt nach
Reihenfolge.

**54. Der Befehlsabgleich prueft jetzt das ganze Wort, nicht nur ein Praefix.**
`337b98a2` und `184f84ff` (D2-10). `commandCheck()` verglich bisher per
Praefix: die Eingabe wurde auf die Laenge des Kandidaten gekuerzt und dann
verglichen. Jeder Befehlsname, der Praefix eines spaeteren war, wurde daher
vom fruehen Treffer verschluckt -- einzig die Reihenfolge in der Kommandoleiter
hielt beide auseinander. Konkret gefunden an `--softser app0`, das durch
`--softser app` (command_functions.cpp, Zeile vor der App0-Variante) abgefangen
wurde und dadurch `iNextTelemetry` nie zuruecksetzte -- eine reine
Reihenfolge-Korrektur in `337b98a2`.

`184f84ff` ersetzt danach die ganze Vergleichslogik: `commandMatches()` und
`casecmp()` wandern nach `src/command_match.h` (Arduino-frei, kompilierbar
unter `env:native_command_match`, 94 Zeilen) und implementieren die Regel, der
die Leiter informell schon folgte -- endet ein Kommandoname auf ein Leerzeichen,
gilt Praefixvergleich fuer das Argument ("setname " matcht "setname Martin");
sonst muss der Name exakt am Zeilenende oder vor dem Trennzeichen enden ("msg"
matcht "msgid" nicht mehr, "pos" nicht mehr "posshot"). Das entfernt zugleich
einen (gutartigen) Lese-Zugriff auf nicht initialisierten Stack, den die alte
Implementierung brauchte, um "kurze Eingabe matcht nie einen laengeren Befehl"
zu erreichen.

Das ist eine Verhaltensaenderung: `--posx`, `--infox` und `--msgidx` werden ab
jetzt zurueckgewiesen, wo sie vorher `--pos` bzw. `--msg on` ausloesten. Belegt
per Vorher/Nachher-Goldaufnahme auf DK5EN-93 (383 Kommandos je Lauf, von 381
vergleichbaren Bloecken genau zwei Abweichungen, beide durch einen Reboot
waehrend der Aufnahme erklaert) und auf Hardware direkt (`--posx` etc. jetzt
abgelehnt). 11 native Testfaelle pinnen sowohl die neue als auch -- ueber eine
im Testfile mitgefuehrte Kopie -- die alte Regel, damit ein Test wirklich die
AENDERUNG beweist und nicht nur das Endverhalten.

**55. Der serielle Empfangspfad ist jetzt testbar, und ein latenter
Speicherfehler ist damit gefunden und behoben.** `7f5a0469`, `9e2c7147`,
`5c269624`. `checkSerialCommand()` lag in den beiden groessten
Uebersetzungseinheiten des Baums (`esp32_main.cpp`/`nrf52_main.cpp`), die nie
auf einem Host kompilieren. `7f5a0469` verschiebt beide Kopien byte-identisch
in eigene Dateien, `src/esp32/serial_command_esp32.cpp` und
`src/nrf52/serial_command_nrf52.cpp`, deklariert die gemeinsame Signatur in
`src/serial_command.h` und macht damit ausdruecklich **keine** Vereinheitlichung
-- welche der beiden Kopien "richtig" ist, ist eine offene Drift-Matrix-
Entscheidung. Reine Verschiebung, verifiziert byte-identisch in beiden
Funktionskoerpern; einzige Nebenwirkung: vier vormals dateiweite Globale
(`strTextWork`, `strText`, `iTxtPos`, `iTxtLen`) werden file-static.

`9e2c7147` baut daraufhin den ersten Test ueberhaupt fuer diesen Pfad
(`test_serial_command_twin`, zwei Binaries, da beide Kopien dasselbe Symbol
`checkSerialCommand()` definieren und beim Linken kollidieren wuerden). Beim
Nebeneinanderlesen fand der Test-Agent einen echten, plattformuebergreifenden
Fehler und eskalierte ihn, statt ihn im fremden Dateibereich still zu
patchen: beide Leser schreiben vor der Kapazitaetspruefung --

```
strText[iTxtPos] = rd;                      // schreibt auch Index 599
if(iTxtPos < (int)sizeof(strText) - 1)      // erst danach wird entschieden
    iTxtPos++;
```

`strText` ist `static char[600]`. `iTxtPos` saettigt bei 599, aber der
Schreibzugriff auf Index 599 findet bei jedem weiteren Byte trotzdem statt.
Eine 600 Byte lange Zeile ohne NUL/CR/LF fuellt damit alle 600 Slots und
zerstoert den einzigen Terminator des Arrays; das direkt folgende
`strlen(strText)` laeuft dann ueber das Array hinaus in benachbartes BSS, und
diese Laenge geht unter anderem in `printfdeb("...wrong command %s\n",
strText)` ein. `5c269624` behebt das an allen drei Stellen (zwei ESP32, eine
nRF52) durch Kapazitaetspruefung vor dem Schreiben; `src/esp32/serial_command_
esp32.cpp:44-51` und `src/nrf52/serial_command_nrf52.cpp:45-52` zeigen die
korrigierte Reihenfolge. Ein neuer Testfall fuettert 600+ Bytes ohne
Terminator und prueft, dass der Terminator ueberlebt; ein Zuruecksetzen der
Schreibreihenfolge laesst diesen Fall mit einem Signal abstuerzen. Bugfix ohne
beobachtbare Aenderung im Normalbetrieb -- betroffen ist ausschliesslich eine
ueberlange Eingabezeile ohne Zeilenende, die es in keinem Goldkorpus gibt.

**56. 24 numerische Setter teilen sich jetzt eine Parse-Routine, und Muell
erreicht die Einstellungen nicht mehr.** `27be0d12` (Schritt 1), `f8d937fe`
(Schritt 2), D2-07. Jeder der rund 54 Setter-Rungs kopierte
`snprintf(_owner_c, ...); sscanf(_owner_c, "%d", &iVar);` -- ein 300-Byte-
Stackpuffer, dessen einzige Aufgabe es war, den bereits vorhandenen Rest-
String zu halten. `sscanf` laesst sein Ziel bei einem nicht-numerischen
Argument unangetastet, und `int iVar;` war nicht initialisiert: auf der Bank
antwortete `--txpower abc` mit "txpower 16711680 dBm not between -9 and max
22" -- 16711680 ist 0xFF0000, Stack-Muell. Die Bereichspruefung verwarf das
meistens, aber ohne Garantie, und das Lesen selbst ist bereits undefiniertes
Verhalten. Fuenf Setter (Analog-Faktor/-Alpha, Batteriefaktor,
Temperaturoffset ein/aus) hatten ueberhaupt keine Bereichspruefung und
schrieben den Muellwert direkt.

`src/command_setters.h` (262 Zeilen, Arduino-frei, `native_command_setters`)
liefert `cmdArgInt/Float/Dbl` fuer den reinen Parse-Schritt sowie
`cmdStoreInt/Float/Double` (Zeilen 202-262), die parsen, den Bereich pruefen
und nur bei Erfolg schreiben -- `*dest` bleibt bei jedem Fehler unangetastet.
Ein nicht-numerisches Argument wird jetzt zurueckgewiesen statt auf 0
abgebildet: 0 laege bei `--txpower` innerhalb des gueltigen Bereichs -9..22
und wuerde Muell speichern, wie ein frueherer Entwurf tatsaechlich tat und der
eigene Test sofort fing. Die "kein Argument"-Meldung ist bewusst von der
Bereichsmeldung getrennt (`cmdArgNotNumber`), damit "--txpower abc" nicht
"txpower 0 dBm not between -9 and max 22" antwortet -- korrekt ablehnen bei
falscher Begruendung waere weiterhin ein Fehler. Auf DK5EN-93 verifiziert:
`--txpower abc` -> "is not a number" (nichts gespeichert), `--txpower 14` ->
unveraendert "set txpower to 14 dBm". Goldaufnahme mit 383 Kommandos: 332
identisch, 12 unterschiedlich (alle 24 betroffenen Setter antworten jetzt auf
Muell mit der neuen Meldung), 0 unerwartete Abweichungen.

Verhaltensaenderung, gezielt: ein nicht-numerisches Argument bei einem der 24
Setter wird ab jetzt abgelehnt und gemeldet, wo es vorher je nach Stack-Inhalt
zufaellig durchgehen oder verworfen werden konnte. `27be0d12` korrigiert
zusaetzlich OPT-D4: `--specstep` schrieb `node_specsamples` und
`--specsamples` schrieb `node_specstep` (vertauscht) -- bereits gespeicherte
Werte werden dabei nicht angefasst.

**57. 70 der 107 On/Off-Rungs werden zur Tabelle, ohne dass sich am Verhalten
etwas aendert.** `e46326c3` (D2-06). Reine Restrukturierung: `COMMAND_TOGGLES[]`
in `src/command_toggles.h` (171 Zeilen) fasst 70 Rungs zusammen, die nur eine
Teilmenge aus fuenf Dingen taten (Echo, Bool-Flag, Bit in `node_sset*`, Save,
BLE-Notify); `toggleApply()` matcht per `commandMatches()`. Die anderen 37
bleiben handgeschrieben, weil sie echt verzweigen. Das Hochziehen der Tabelle
vor die restliche Leiter ist nur sicher, weil D2-10 (siehe oben) den
Praefixvergleich beseitigt hat -- geprueft in beiden Richtungen, dass keine
Zeile eine andere Zeile abfaengt oder ihr Argument stiehlt.

Drei Feinheiten, alle per Test gepinnt: `node_sset*` sind `int`, kein
`uint16_t`, daher eine Maskenpaar-Spalte statt einer Bitnummer; `mesh on`
setzt sein Flag, waehrend es sein Bit LOESCHT (das Bit heisst "mesh aus"); und
`ina226 on` muss `setupINA226()` NACH `save_settings()` aufrufen, weil es bei
fehlendem Chip vier persistierte Float-Felder auf 0 setzt -- daher die
`TG_POST_FIRST`-Unterscheidung in `toggleApply()`
(`src/command_toggles.h:141-160`). Verifiziert per Reachability-Check
(genau ein aufrufender `toggleApply()`), Artefakt-String-Scan (alle 37
unguarded Zeilennamen im Firmware-Image vorhanden), Hardware-Probe
(`--setinfo on`/`--shortpath off` auf DK5EN-93) und Goldaufnahme (381
Kommandos, null Dispatch-Aenderungen -- die einzigen Diffs sind Zeitstempel
und der Nachrichten-Zaehler).

**58. Drei tote Kommentarbloecke sind weg, und ein Lint braucht sie nicht mehr,
um zu bestehen.** `46eab04d` (D2-01). Reine Subtraktion, 51 Zeilen: der
`/* TEST */ "compress "`-Block sowie zwei `"softser test0"`/`"softser test"`/
`"softser xml"`-Bloecke in `commandAction()`, alle drei bereits vor diesem
Commit auskommentiert -- es handelt sich **nicht** um entfernte Funktionen,
sondern um Kommentare, die der Rung-Parser des Lint-Werkzeugs faelschlich als
lebenden Code las. Genau das war die Falle hinter D2-06: `extract_commands.
py`s Rung-Parser strippt keine Kommentare, hielt den toten `compress`-Block
fuer die erste Zeile der Leiter und liess die neue Toggle-Tabelle deshalb in
einem fruehen Entwurf faktisch wirkungslos einfuegen, bei gruenem Build.
`extract_commands.py --self-test` pruefte bis dahin "Kommentar-Strippen
entfernt etwas" gegen den LEBENDEN Quellcode -- eine Behauptung, die nur galt,
solange der tote Code existierte. Nach der Loeschung lief der Test ins Leere
und schlug fehl, obwohl er bis dahin aus dem falschen Grund gruen war. Er
prueft jetzt gegen eine Fixture statt gegen den echten Baum. Mutationsgeprueft:
macht man das Kommentar-Strippen zum No-Op, schlaegt die Fixture fehl.

**59. Ein Diagnoseschalter statt zwei, und der String-Scan zahlte sich beim
ersten Lauf aus.** `dd399f51` (R3-11/D2-09). `MC_CAPTURE` (TX/RX-Capture-Ring)
und die vier `--spec*`-Kommandos werden zu `MC_DIAG`
zusammengefuehrt (Default 1, `E22_XML-DevKitC` setzt `-D MC_DIAG=0`), definiert
in `src/configuration_global.h`. Der urspruengliche Plan haette
`INSTRUMENT_ENABLED` als Schalter genommen -- das haette `--txcapture`,
`--specstart`, `--specend`, `--specstep` und `--specsamples` aus allen 34
Images entfernt, weil `INSTRUMENT_ENABLED` in keiner `platformio.ini` auf 1
steht. Der obligatorische String-Scan nach dem Umbau fand sofort einen
konkreten Bug: mit `MC_DIAG=0` fuehrte `E22_XML` die vier `--spec*`-Namen in
der `--help`-Ausgabe (`command_functions.cpp:917`, ausserhalb des Guards)
weiterhin -- das Board **bewarb vier Kommandos, die es nicht mehr hatte**.
Bugfix in derselben Aenderung: die Hilfezeile wandert unter den Guard. Nach
der Korrektur: 0 unerklaerte Kommandonamen ueber alle 34 Images.

**60. `--onewire gpio` und `--tempoff` bekommen Grenzen, die vorher fehlten.**
`d7b4b96c` (H6-01, Nebenbefund) und `5f09ad06` (Toggle-Soak, Nebenbefund).
`--onewire gpio <n>` laeuft jetzt ueber `cmdStoreInt` mit der aus
`config_json.h` uebernommenen Grenze 0..99 (vorher wurden auch Werte wie
999999 auf Hardware angenommen). `--tempoff in|out` laeuft ueber
`cmdStoreFloat` mit -50..50 und antwortet bei einer Ablehnung "out of range
(-50..50 degC), ignored"; die Web-Setup-Parameter `tempoffsetindoor`/
`tempoffsetoutdoor` rufen denselben Setter statt eines eigenen ungeklammerten
Schreibpfads. Gefunden ueber ein reales Symptom: ein Golden-Korpus hatte
`--tempoff in 999999` gesetzt und nie zurueckgesetzt, wodurch ein Bankknoten
dauerhaft eine falsche Innentemperatur anzeigte. Verhaltensaenderung, klar
zum Besseren: beide Setter akzeptierten vorher Werte ausserhalb jedes
sinnvollen Bereichs.

**61. `--setowndns` war doppelt vorhanden, die tote Haelfte trug die einzige
Auto-Reboot-Pruefung.** `a50c5614` (W1, Audit-Defekt 2). Zwei
`commandCheck(msg_text+2, "setowndns ")`-Bloecke existierten; der erste,
erreichbare, speicherte den Wert ohne die Reboot-Pruefung, die alle vier
anderen `setown*`-Kommandos (`setownip`, `setowngw`, `setownms`,
`setownntp`) haben. Der zweite, nie erreichte Block trug diese Pruefung --
und zusaetzlich einen eigenen Fehler: `msg_text+11` statt der korrekten `+12`,
was dem gespeicherten Wert ein fuehrendes Leerzeichen vorangestellt haette.
Der tote Block ist geloescht, die Pruefung in den lebenden Block verschoben
(`src/command_functions.cpp:3363` ff.). Verhaltensaenderung, ausdruecklich als
solche markiert: `--setowndns` loest jetzt wie die anderen vier einen
Auto-Reboot aus, wenn sich der IP-Konfigurationsstatus dadurch aendert.
Dieselbe Aenderung korrigiert Defekt 11 -- `--pingcall` begrenzte sein
`snprintf` mit `sizeof(node_call)` statt `sizeof(node_pingcall)`
(`src/command_functions.cpp:3067`); heute harmlos, da beide Felder
`char[10]` sind, aber als Feld-A/sizeof(Feld-B)-Muster im ganzen File
gegengeprueft (kein weiterer Treffer).

**62. `--extudp off` gefolgt von `--extudp on` oeffnete den Socket nicht wieder
neu.** `d05a0dc3` (EXT-02, zweite Haelfte). Die Toggle-Zeile fuer
`--extudp off` trug `nullptr` als Nachaktion, sodass `hasExternIPaddress` und
der `UdpExtern`-Socket beim Ausschalten unangetastet blieben. Fix:
`tg_post_extudp_off()` ruft jetzt `resetExternUDP()`
(`src/command_functions.cpp:160-171`, Tabellenzeile
`src/command_functions.cpp:245`). Die Reihenfolge ist am Code nachgewiesen,
nicht vermutet: `*row.flag` wird in `toggleApply()`
(`src/command_toggles.h:141`) UNBEDINGT geschrieben, bevor der
`TG_POST_FIRST`-Zweig ueberhaupt geprueft wird -- `bEXTUDP` ist beim Lauf der
Nachaktion also bereits `false`, `TG_POST_FIRST` also fuer diese Zeile nicht
noetig. Zwei Regressionstests, gegenseitig mutationsgeprueft (einmal die
Zeile auf `nullptr` zurueckgedreht, einmal der Funktionsrumpf ausgehoehlt bei
intakter Verdrahtung). Bugfix, zum Besseren: der Bediener bekam vorher keine
Fehlermeldung und sah keinen erkennbaren Grund, warum die Gegenstelle nach
einem Off/On-Zyklus stumm blieb.

**63. Radio-Kommandos rufen jetzt eine gemeinsame Einheiten-Konvertierung.**
`b676c808` (RF-01..03, RF-05, RF-06) und `136dacdd` (RF-04). Die eigentliche
Konvertierungstabelle liegt in `src/radio_units.cpp` (ausserhalb dieses
Kapitels); an den Kommando-Aufrufstellen in `command_functions.cpp` aendert
sich der Include (`#include "radio_units.h"`) und die Anrufe fuer `--txbw`,
`--txcr`, `--txfreq`, `--setcountry 7` (MAN) und die `--info`-Frequenzanzeige.
Fuenf Fehler, eine Ursache: `node_freq`/`node_bw`/`node_cr` tragen auf ESP32
und nRF52 unterschiedliche Einheiten (Index vs. physikalischer Wert), und
einzelne Schreibstellen vergassen die Umrechnung -- am schwersten `--txfreq`
auf T114/T-Echo, das MHz statt Hz speicherte. `136dacdd` korrigiert zusaetzlich
die Frequenz-Schutzbandberechnung (`/100.0` statt `/1000.0`), die auf dem
868-MHz-Band vorher UNERFUELLBAR war und dort jede manuell gesetzte Frequenz
kommentarlos durch den 70cm-Standardwert ersetzte. Bugfix: Nichts sendete auf
der falschen Frequenz (jede Plattform konfiguriert ihren eigenen Treiber
korrekt), nur die Validierung war falsch -- betroffen sind vor allem
T114/T-Echo/RAK4630, die auf dieser Bank nicht vorhanden sind. Beide Commits
sagen das ausdruecklich: NICHT auf Hardware bestaetigt fuer RF-02/RF-05,
Bank-Nachweis existiert nur fuer die 868-MHz-Grenzband-Korrektur.

**64. `--keylock on/off` erlaubt die Wiederherstellung eines gesperrten
T-Deck.** `76302aab` (TD-16). Ein als "Absturz" gemeldetes T-Deck (Touch tot,
Tasten werden gelesen aber nicht umgesetzt) war in Wahrheit die persistierte
Tastatursperre `node_keyboardlock`, die `touchpad_read()`/`keypad_read()` und
`tft_on()` blockiert. Der neue Befehl (ausserhalb des Instrument-Guards, mit
Hilfetext) loescht das Flag, weckt das Panel und speichert. Auf DK5EN-14
verifiziert: ueber USB geloescht, uebersteht `--reboot`, Touch danach wieder
funktionsfaehig. Reine Erweiterung -- kein bestehendes Kommando aendert sein
Verhalten, der neue Befehl liefert einen Wiederherstellungsweg fuer einen
zuvor nur per Neuflashen loesbaren Zustand.

**65. `--srvip` bekommt ein nRF52-Gegenstueck, kompiliert aus jedem
Auslieferungs-Image heraus.** `ee545088`. Der ESP32 hatte den Server-Override
seit TM-31; die nRF52-Ethernet-Seite (`NrfETH::startUDP()`) kannte nur
hartkodierte Serveradressen. Der Befehl greift jetzt auch fuer
`NRF52_SERIES` (`src/command_functions.cpp`, Guard erweitert), angewendet
direkt in `startUDP()` statt erst beim naechsten Reboot -- ein Reboot haette
den RAM-only-Override sofort wieder geloescht, was auf RAK-90 vor der
Korrektur auch beobachtet wurde. Verifiziert auf RAK-90 (Server-Adresse
wechselt von der hartkodierten `89.185.97.38` zur gesetzten). Reine
Bank-Erweiterung: kompiliert nur unter `INSTRUMENT_ENABLED`, im Auslieferungs-
Image per String-Scan mit null Treffern bestaetigt -- kein Verhaltens-
unterschied fuer ein reales Netzwerk.

**66. `sendToPhone()`/`sendComToPhone()` teilen sich jetzt eine Framing-Funktion,
und die trug den ersten ausfuehrbaren Test.** `2d348d04` (R1-02, Schritt 1).
Beide Funktionen fuehrten denselben Drei-Wege-Switch (0x91 mheard, 0x44 JSON,
sonst Text mit 0x40-Tag), Zeichen fuer Zeichen gleich bis auf einen Arm: der
Text-Arm von `sendComToPhone()` kopierte `blelen-1` statt `blelen` -- ein
Byte zu wenig. Dieser Arm war jedoch unerreichbar (`BLEComToPhoneBuff` hat
genau zwei Erzeuger, beide setzen `buffer[0] = 0x44`), also nie live falsch.
Beide Drains rufen jetzt `blePhoneFrame()` (`src/ble_phone_frame.h`), Aufrufe
in `src/phone_commands.cpp:112` und `:179`. Zwei latente Fehler wurden dabei
gefunden und beseitigt, beide unbeobachtbar, weil in totem Code: (1) haette
ein aus einem Laengen-Rueckgabewert abgeleiteter Versand jeden BLE-Frame um 1
bis 3 Byte verkuerzt -- deshalb liefert der Helfer `bool`, nicht die Laenge;
(2) der erste Entwurf des Ablehnungspfads haette `ComToPhoneRead` nicht
weitergeschaltet und damit diesen Ring dauerhaft blockiert, waere der Pfad
je erreichbar geworden. Gemessen: -976 B Flash ueber 32 Envs, RAM unveraendert.
Reine Restrukturierung mit Blick auf reales Verhalten: kein aktiver Codepfad
aendert sein Ausgabeverhalten, aber zwei tote Fehler koennen nicht mehr
wiederkehren, sollte der 0x91-Pfad je reaktiviert werden.

**67. Drei neue Nur-Lese-Diagnosebefehle aus der Settings-Persistenz-Kampagne.**
`af00c402` (`--dumpsettings`), `330cb9fc` (`--msgid`), `e8f29bc4`
(`--persiststat` erweitert). Alle drei entstanden, weil `DO_DEBUG 0` jeden
`DEBUG_MSG` auf dem Lade-/Speicherpfad wegkompiliert und ein Boot, der das
Dateisystem formatierte, die Sanity-Pruefung ablehnte oder einen Schreibfehler
hatte, von aussen nicht von einem sauberen Boot zu unterscheiden war.
`--dumpsettings` (nRF52, `src/command_functions.cpp:771` ff.) gibt den
Rohinhalt des Keyed-Settings-Store aus. `--msgid` gibt den Message-ID-Zaehler
als `[SETST];counters;msgid;<n>` aus (beide Plattformen, ausserhalb von
`INSTRUMENT_ENABLED`, weil er den Upgrade-Vorgang auf Auslieferungs-Images
prueft) und musste vor dem `"msg"`-Fall in der Leiter stehen, da
`commandCheck()` sonst auf `--msg on` truncaten wuerde. `--persiststat`
(vormals T-Deck-only, vier Schalter) druckt jetzt alle 17 Zeilen aus
`SETTINGS_PERSIST_ONLY_LIST(_PLATFORM)`, inklusive der 13 T-Deck-Zeilen unter
demselben Guard wie ihre Struct-Member -- der um `BOARD_T_DECK_PRO` erweitert
wird, das die alte Kopie ausliess, obwohl die Felder dort existieren. Alle
drei sind reine Erweiterungen ohne Wirkung auf bestehende Befehle; sie machen
Zustaende sichtbar, die vorher nur ueber Umwege (Frame senden und auf der Luft
mitlesen, oder gar nicht) beobachtbar waren.

## K07 Loop-Scheduler und Plattform-Aktionen

**68. Ein gemeinsamer Scheduler ersetzt sieben handgeschriebene Timer in beiden Hauptschleifen.** (`e64ce346`, D1-10). `src/loop_scheduler.h`/`.cpp` fuehren eine Tabelle aus Timer-Adresse, Intervallfunktion, optionalem `enabled()`, `action()` und Reset-Modus, plus einen Laeufer `loopSchedulerRun()`, der einmal je Schleifendurchlauf `(uint32_t)(now - *timer) >= interval` prueft. Die Timer-Rumpfe selbst wandern unveraendert in `src/esp32/loop_actions_esp32.cpp` und `src/nrf52/loop_actions_nrf52.cpp` (je Plattform, damit der Twin-Test keine Funk-/Sensortreiber linken muss). Migriert sind genau sieben von 26 im Audit als plattformgemeinsam gefuehrten Timern: `retransmit_timer` (2 s), `mcp_refresh_timer` (5 s), `BattTimeWait` (30 s), `heapMonTimer`, `BMP3TimeWait`, `MCU811TimeWait`, `INA226TimeWait` (60 s). Reine Restrukturierung -- die Reihenfolge der sieben zueinander war auf beiden Plattformen bereits identisch, migriert wurde nur ihre Position relativ zum unveraenderten Rest der Schleife.

Die restlichen 19 Kandidaten blieben bewusst unangetastet, mit Begruendung je Fall in `src/loop_scheduler.h`: `posinfo_timer_min` haengt an einem fremden Reset; `softser_refresh_timer` sitzt in einer if/else-if-Kette, die der Scheduler nicht abbilden kann; `onewireTimeWait`/`BMXTimeWait` resetten mit `millis() - lreduction` (Retry-Logik); `ring_status_timer`/`ch_util_timer` vergleichen mit `>` statt `>=` und brauchen die Elapsed-Zeit im Rumpf; `config_to_phone_datetime_timer` ist ein echter neuer Befund -- er wettlaeuft mit `updateTimeClient()` um den letzten Schreibzugriff auf `bNTPDateTimeValid`, und eine Verschiebung haette auf ESP32 die Reihenfolge gekippt. Diese Zurueckhaltung ist fuer einen Reviewer so aussagekraeftig wie die Migration selbst: das Ziel "50 Praedikate" schrumpft im Commit ehrlich auf 7 von 26 belegten.

Ein vom Advisor gefundener Blocker wurde vor dem Merge korrigiert: der Aufruf von `loopSchedulerRun()` lag im ersten Entwurf innerhalb des `if(bRadio)`-Blocks von `esp32loop()` (und im `#else`-Zweig des T5-ePaper-Codes) -- damit waeren Batterie-, Heap- und drei Sensor-Timer auf einem Knoten mit totem Funkchip, auf `esp32-external-radio` und auf `t5_epaper` stillschweigend stehengeblieben. Der Aufruf sitzt jetzt auf Funktionsebene davor; das `bRadio`-Gate traegt seither ausschliesslich `loopEnabled_retransmit()` (`src/esp32/loop_actions_esp32.cpp:51`) selbst, fuer den einzigen der sieben migrierten Timer, der wirklich vom Funkchip abhaengt. Die konkrete Umplatzierung in `esp32_main.cpp` gehoert zu K12 (dort beschrieben). Verifikation laut Commit: neuer Twin-Test `native_loop_scheduler` (16/16 gegen die echte Tabelle, mutationsfest fuer `>=`/`>`, Reset-Modus und Enabled-Gate), 985/985 native Tests, Builds fuer `wiscore_rak4631`, `heltec_wifi_lora_32_V3`, `t_deck_plus`, `ttgo_tbeam`-Varianten, `E22_XML`, `heltec_t114`, `t_echo`, `vision-master-e213`; ~237 Zeilen weniger in beiden Hauptschleifen.

**69. nRF52-Hauptschleife laeuft jetzt in einem eigenen 8-kB-Task -- Fix eines echten Stack-Overflows.** (`173a2970`, ETH-03). Der Adafruit-Core gibt `loop()` fest 4 kB Stack (`LOOP_STACK_SZ`), nicht per Build-Flag aenderbar. Seit `aprsMessage` durch R2-04 Inline-Text traegt, belegt der Pfad `getExtern() -> sendMessage()` zwei solcher Strukturen plus einen 381-Byte-Rahmenpuffer auf diesem Stack; ein einzelnes UDP-1799-Datagramm reproduzierte auf der Bench (DK5EN-90) einen Reset mit `RESETREAS=0x4`, dieselbe Signatur wie ein frueherer Defekt (N-22). Fix in `src/main.cpp`: `nrf52loop()` plus `captureDrain()` laufen ueber `Scheduler.startLoop()` in einem eigenen 8-kB-Task (`"mcloop"`), der Core-Loop-Task wird mit `suspendLoop()` geparkt; schlaegt `xTaskCreate` fehl, bleibt der 4-kB-Loop und der Bootlog meldet es (`[BOOT];loopstack;FAILED`). Echter Bugfix, keine Restrukturierung. Regressionsinstrument `tools/bench/eth03_probe.py` schlaegt auf dem alten Stand fehl; ein PASS auf dem Fix-Build war laut Commit zum Zeitpunkt des Schreibens noch offen (Bootloader-Zugriff auf dem Testgeraet blockiert).

**70. Der RAW-RX-Logpuffer wird erst angelegt, wenn ihn jemand ansieht -- 109 040 B RAM zurueck.** (`d5a071d8`, R1-04). `ringbufferRAWLoraRX` hatte genau einen Leser, die rxlog-Seite der Web-Oberflaeche, kostete aber auf jedem Knoten statisch `MAX_LOG * (UDP_TX_BUF_SIZE+5)` Byte -- auch auf Knoten ohne Webserver. `src/loop_functions.cpp:468` deklariert ihn jetzt als `rawLogLine_t *ringbufferRAWLoraRX = NULL`; `rawLogEnsure()` (`loop_functions.cpp:473`) legt ihn per `calloc()` beim ersten Seitenaufruf an, `charBuffer_aprs()` (`loop_functions.cpp:3384`) liest den Zeiger einmal in eine lokale Variable und ueberspringt bei `NULL`. Kein Lock noetig, aus drei Gruenden, die der Kommentar an Ort und Stelle festhaelt: der Zeiger geht genau einmal von `NULL` auf gueltig und nie zurueck; `calloc()` nullt den Puffer, bevor der Zeiger veroeffentlicht wird, sodass ein Schreiber nie einen halbfertigen Puffer sieht; und ein einzelner ausgerichteter Zeigerschreibzugriff ist auf ARM und Xtensa atomar. Gemessen ueber 34 Environments: -109 040 B RAM gesamt, +7 076 B Flash. Der Commit korrigiert dabei eine falsche Audit-Annahme: der Schreiber laeuft auf dem 16-kB-LORA-Task (`OnRxDone`), nicht auf dem 1-kB-Timer-Task, wie die Auditzeile behauptete. Hardware-Soak fuer das RAK laut Commit noch offen (keine Antenne auf der Bench).

**71. Die OLED-Treiberauswahl war doppelt im Baum und lief gegenlaeufig -- der Audit hatte die falsche Plattform beschuldigt.** (`5b57d555`, DISP-01). Die Auditzeile behauptete, der ESP32-Zweig verdrehe den Display-Typ-Vertrag; das stimmte nicht -- `esp32_isSSD1306()` klassifiziert die Panelgroesse korrekt, gibt den beiden Faellen aber vertauschte Log-Labels aus (`"OLED Display is SSD1306"` auf dem Zweig, der tatsaechlich SH1106 liefert, und umgekehrt); derselbe Fehler stand auch im Kommentar in `esp32_functions.cpp`. Beide Log-Zeilen sind jetzt korrekt beschriftet (`"OLED panel is 0.9 inch -> SSD1306"` / `"...1.3 inch -> SH1106"`). Der eigentliche Defekt war Verdopplung, nicht der Zahlendreher: die Zuordnung Sondenwert -> U8g2-Objekt existierte separat in `esp32_functions.cpp` und `nrf52_functions.cpp`, gespeist von derselben Sonde, und war auseinandergelaufen (ESP32 bildete 1 auf `u8g2_1` ab, nRF52 auf `u8g2_2`). Jetzt genau eine Funktion, `mcSelectU8g2()` (`src/loop_functions.cpp:1041`, deklariert in `loop_functions.h`), guardiert mit `MC_HAS_U8G2` (GRD-01), von beiden Plattformen aufgerufen. Bench-belegt auf Heltec V3 und einem T-Beam (beide bekommen korrekt `u8g2_1`); die nRF52-Seite war laut Commit code-beweisbar verdreht, aber mangels RAK4631-mit-OLED auf der Bench nie hardwareverifiziert. `BOARD_TBEAM_1W`s hartkodierter Wert 1 mit dem alten Zweifelskommentar bleibt bewusst unveraendert -- ein Fehler dort wuerde nur einen ungeprueften Zustand gegen einen anderen tauschen. Verifikation: 34/34 Board-Envs, 33/33 native Envs (951 Faelle), plus ein Reflash von DK5EN-93 mit visueller Pruefung.

**72. Eine einzige Praedikat-Definition entscheidet jetzt, welche Boards ein U8g2-OLED treiben.** (`46ed0f3c`, GRD-01, mit Vorarbeit in `af179892`). Die Aussage "hat dieses Board ein U8g2-Display" stand handgeschrieben an acht Stellen in drei Dateien und drei Schreibweisen -- fuenfmal in `loop_functions.cpp` (u. a. Zeilen 365, 868, 1069, 1441, 1728), einmal in `esp32_functions.cpp`, einmal in `nrf52_functions.cpp`, dazu eine achtarmige `#if/#elif`-Kaskade in `sendPosition()` (`loop_functions.cpp`, um Zeile 4813), die bereits auseinandergelaufen war: `BOARD_T5_EPAPER` fehlte in der Aufzaehlung, sodass der erste Build dieser Umgebung mit `"'u8g2' was not declared in this scope"` starb -- niemand hatte es bemerkt, weil `env:t5_epaper` zuvor nie kompiliert hatte. Der Vorgaenger-Commit `af179892` hatte das minimal mit einem zusaetzlichen `#elif defined(BOARD_T5_EPAPER)`-Zweig geflickt, um den Build ueberhaupt lauffaehig zu machen; `46ed0f3c` loest die eigentliche Ursache: alle acht Stellen werden zu `#if MC_HAS_U8G2`, definiert einmal in `configuration_global.h` neben dem bereits vorhandenen `WP_DISP`-Muster, und die Kaskade in `sendPosition()` schrumpft von neun auf drei Zweige, wobei der `u8g2`-Zweig jetzt strukturell unerreichbar ist, wo es kein u8g2 gibt, statt sich auf eine vollstaendige Aufzaehlung zu verlassen. `BOARD_STICK_V3` behaelt einen eigenen Zweig: es hat ein U8g2-Display, aber keine Track-Seite auf seinem 0,49"-Panel -- eine andere Aussage als "kein OLED". Restrukturierung, keine Verhaltensaenderung: Gate 34/34 Board-Envs; von 68 verglichenen RAM/Flash-Werten waren 62 byteidentisch, die uebrigen sechs unterschieden sich um exakt +/-16 B Flash mit gemischtem Vorzeichen und ohne RAM-Bewegung -- laut Commit Linker-Padding, kein Codeunterschied.

**73. Sieben von acht Sende-Epilogen teilen sich jetzt eine Funktion.** (`6b03bd37`, D3-02). Der Ablauf msgid-Vorschub, bedingtes Persistieren, `checkVia()`, `encodeAPRS()` stand wortgleich an acht Stellen in `loop_functions.cpp`; sieben davon (`sendPing`, `SendPong`, `sendMessage`, `sendPosition`, `sendAPPPosition`, `sendHey`, `sendTelemetry`) rufen jetzt `finalizeAndSendAPRS()` (`loop_functions.cpp:3463`). `SendAckMessage` bleibt bewusst aussen vor: sie verschraenkt `insertOwnTx()`/`addLoraRxBuffer()` zwischen Persistenz und `checkVia()` und passt nicht in die gemeinsame Form. Reine Restrukturierung, laut Commit vom Advisor gegengeprueft (Reihenfolge msgIdAdvance -> needsPersist -> checkVia -> encodeAPRS an allen sieben Stellen bestaetigt). Ohne eigenen Test: `loop_functions.cpp` wird von keinem nativen Environment kompiliert, der Commit vermerkt das ausdruecklich als offene Luecke statt sie zu verschweigen.

**74. Drei RAM-Ringpuffer waren groesser dimensioniert als ihr tatsaechlicher Schreiber je fuellt.** (`2991034e`, R1-01/R1-03). `ringBufferUDPout` schrumpft von `UDP_TX_BUF_SIZE+20` auf `UDP_TX_BUF_SIZE+1` (`loop_functions.cpp:502`), `BLEtoPhoneBuff` von `MAX_MSG_LEN_PHONE+5` auf `UDP_TX_BUF_SIZE+5` (`loop_functions.cpp:513`), `BLEComToPhoneBuff` von `MAX_MSG_LEN_PHONE+5` auf ein festes `246` (`loop_functions.cpp:522`) -- jeweils hergeleitet aus dem tatsaechlichen Schreib-Clamp der zugehoerigen `addXxxBuffer()`-Funktion, nicht aus einer allgemeinen Obergrenze. Die zugehoerigen `extern`-Deklarationen in `loop_functions_extern.h` sind entsprechend nachgezogen. Vor Ort als DRY-Audit-Befund markiert (R1-03/R1-01), tatsaechliche Wirkung ist eine RAM-Einsparung: Commit nennt gemessen -2 928 B auf `ttgo_tbeam`, -2 936 B auf `heltec_V3`, -2 928 B auf `rak4631` fuer die kombinierte R1-01/03/06-Gruppe.

**75. Der Message-ID-Zaehler schreibt nicht mehr bei jedem Frame auf Flash.** (`bce95db5`, W3, verfeinert durch `b0adf723`, W3c). `node_msgid` muss einen Neustart ueberleben -- sonst verwerfen Nachbarknoten wiederholte IDs als Duplikate --, aber jedes originierte Frame loeste zuvor einen vollen `save_settings()` aus: auf nRF52 ein ~1,5-kB-Dateischreibvorgang plus Rename auf einem 28-kB-Dateisystem, auf ESP32 ein NVS-Commit, von acht Aufrufstellen in `loop_functions.cpp`. `bce95db5` fuehrt `msgIdAdvance()`/`msgIdNeedsPersist()` (`src/msgid_counter.h`) ein: der Zaehler wird nur bei Vielfachen von 100 persistiert, beim Laden um einen vollen Schritt vorgezogen und dieser Wert sofort zurueckgeschrieben -- ohne dieses Zurueckschreiben wuerde ein Absturz in den ersten hundert Frames nach einem Boot beim naechsten Boot wieder denselben Bereich vergeben, genau die ID-Wiederverwendung, die die Persistenz verhindern soll; ein Property-Test lauft laut Commit jeden moeglichen Absturzpunkt ueber zwei Bloecke durch. Gemessen auf DK5EN-90: `node_msgid` 24 -> 124 -> 224 ueber zwei Boots, ein `save;ok` je Boot. `b0adf723` (W3c) ersetzt an denselben acht Stellen (u. a. `loop_functions.cpp:3469`, `:5038`) `save_settings()` durch `countersSave()` aus dem neuen `counters_store.h`: damit schreibt der High-Water-Mark-Fall nur noch den Zaehler, nicht mehr den kompletten 4-kB-Settings-Datensatz. Beides echte, messbar begruendete Fixes gegen Flash-Verschleiss, keine reine Umbenennung.

**76. Ein toter Spinlock-Extern verschwindet, mit der Begruendung dafuer.** (`36c37b5f`). Der ESP32-Spinlock `displayMux` hatte laut Commit seit einer frueheren Aenderung (`4a250602`) keine Stelle mehr, die ihn nimmt oder gibt -- nur die `extern`-Deklaration in `loop_functions_extern.h` und Kommentare an drei Stellen ueberlebten, und die Kommentare waren die eigentliche Irrefuehrung: sie behaupteten, `pendingDisplayMsg` sei auf ESP32 gegen `OnRxDone` abgesichert. Das ist unnoetig, weil `OnRxDone` auf ESP32 innerhalb von `esp32loop()` selbst laeuft (Erzeuger und Verbraucher sind derselbe Task); auf nRF52, wo echtes Preemption moeglich ist, bleibt der Schutz per `taskENTER_CRITICAL()` bestehen. Reine Dokumentationskorrektur plus toter-Code-Entfernung, keine Verhaltensaenderung.

Kleinere Randnotizen aus demselben Commit-Satz: `36c37b5f` korrigiert nebenbei die eigene Zaehlung der Timer-Kandidaten fuer D1-10 (78 Praedikatsstellen ueber 43 Timer-Variablen, davon 25 plattformgemeinsam, statt der zuvor kursierenden "50/27") -- die Zahl, auf der `e64ce346` spaeter mit "26 gemeinsam, 7 migriert" aufbaut. `09524068` (R2-01, mheard-Textcodec-Abloesung) zieht in `loop_functions_extern.h` nur die `extern`-Deklaration von `mheardBuffer` auf `MheardRecord mheardRecords[MAX_MHEARD]` nach; die eigentliche Struct-Umstellung liegt in `mheard_functions.cpp`, ausserhalb dieses Kapitels. `3be9a9da` (W6b) nimmt eine einzelne, echte Korrektur in `loop_functions.cpp:5907` vor: der `"udp"`-Ring war laut Commit der einzige, dessen Ringueberlauf-Diagnose (`RING_OVERFLOW`) unterdrueckt wurde; diese Ausnahme entfaellt, weil ein neuer frueher Ruecksprung anderswo genau diesen Ring fuellen kann und die Verdraengung sonst unbeobachtbar bliebe.

Nachtrag aus der neo-Kampagne selbst, ohne Kampagnen-Commit: `heapMonTimer` war der einzige der sieben migrierten Timer, dessen `enabled()` auf den beiden Plattformen etwas anderes bedeutete. Der nRF52 schaltete den Eintrag unter `--setlog on` ganz ab, der ESP32 liess ihn laufen und unterdrueckte nur den Druck -- der Timer las also alle 60 s `ESP.getFreeHeap()` und `ESP.getFreePsram()` und schrieb `lFreeHeap`/`lFreePsram` fort, die ausser diesem Block niemand liest. `loopEnabled_heapMon()` liefert jetzt auf beiden Plattformen `!bDisplayLog` (`src/esp32/loop_actions_esp32.cpp:81`), die `if(!bDisplayLog)`-Huelle im Rumpf entfaellt. Reine Restrukturierung, auf der Konsole nicht unterscheidbar: unter `--setlog off` -- dem Auslieferungszustand -- aendert sich nichts, unter `--setlog on` war die `[HEAP]`-Zeile schon vorher stumm. Der freie Heap geht in dieser Betriebsart ohnehin nicht verloren, er steht einmal je STAT-Fenster im Feld `heap=` (`setlogFormatStat()`, `src/setlog_lines.cpp`) -- worauf sich der Dauerlauf oben stuetzt: zwei der drei Knoten liefern ihren Heap-Verlauf ueber die STAT-Zeile, einer ueber `[HEAP]`.

## K11 nRF52-Netzstack

### EXTUDP-Stoss setzte den Knoten zurueck -- kein haengender Socket, ein zu kleiner Stack

**77. Ein Schub eingehender EXTUDP-Datagramme konnte den RAK4631 per Stack-Overflow neu starten; behoben durch einen eigenen 8-kB-Task fuer die Hauptschleife** (`54b6d162`, `4f4b02f1`, `d05a0dc3`, `173a2970`, `1bea29a1`, ETH-03/ETH-02/ETH-02b). Beobachtet wurde zuerst ein rund 25-sekuendiger Totalausfall des Netzstacks (Webserver, EXTUDP) nach vier kleinen JSON-Datagrammen auf UDP 1799, bei durchgehend antwortendem Seriellport -- das Bild eines haengenden Knotens. Die Untersuchung ist im Baum ehrlich dokumentiert, einschliesslich einer eigenen widerlegten Hypothese:

- `54b6d162` vermutete zunaechst einen SPI-Bus-Stall im Empfangspfad: `getExternUDP()` ruft `UdpExtern.parsePacket()` auf dem W5100S auf, der sich den SPI-Bus mit dem SX1262 teilt, unter der `bSPI_ETH_Active`-Guard, die auch `loopWebserver()` blockiert (`src/nrf52/nrf52_main.cpp`, Guard um Zeile 2384/2439). Naheliegend, aber ungemessen.
- `4f4b02f1` widerlegte diese eigene Hypothese explizit mit Messdaten: vier neue `INSTR_SECTION()`-Sonden (`src/nrf52/nrf52_main.cpp:2384` `extudp_guard`, `:2439` `webserver_loop`, plus zwei in `src/extudp_functions.cpp`) zeigten ueber 397 Schleifendurchlaeufe maximal 977 us -- nichts haengt. Der Verdacht wanderte weiter zu einem faelschlich geloeschten `hasIPaddress`-Flag.
- `d05a0dc3` widerlegte auch diese zweite Theorie: vier Marker an allen vier Stellen, die das Flag loeschen (`ethClearLog()`, `src/nrf52/nrf_eth.cpp:199`, aufgerufen bei `:220`, `:334`, `:348`, `:622`) feuerten null Mal. Erst die Kombination aus Laufzeit-Arithmetik (Traffic 21:11:26, `--info` 83 s spaeter meldet eine Bootzeit von nur 21 s danach) und einer USB-Abmeldung des Geraeteknotens waehrend des Traffics belegte den tatsaechlichen Befund: **der Knoten startet neu.**
- `173a2970` fand die Ursache und behob sie: der nRF52-Loop lief im festen 4-kB-Stack des Adafruit-Cores (`LOOP_STACK_SZ`). R2-04 (siehe K08) hatte `aprsMessage` kurz zuvor auf ~590 Byte Inline-Text umgestellt; der Pfad `getExtern() -> sendMessage()` traegt zwei solche Instanzen plus einen 381-Byte-Rahmenpuffer und ueberschritt damit den Stack -- Reset-Signatur `RESETREAS=0x4 (SREQ)`, dieselbe Klasse wie ein frueherer Defekt (N-22). Fix in `src/main.cpp`: `nrf52loop()` und `captureDrain()` laufen jetzt ueber `Scheduler.startLoop()` in einem eigenen 8-kB-Task (`"mcloop"`), der Core-Loop wird per `suspendLoop()` geparkt; schlaegt `xTaskCreate()` fehl, bleibt der alte 4-kB-Loop und der Bootlog vermerkt es. Nebenbefund im selben Commit: die `[RING] overflow`-Log-Zeile in `sendMessage()` testete `iWrite == iRead`, was auch der leere Ring erfuellt, und hatte die Fehlersuche einen Tag lang auf einen vollen Ring gelenkt, der nie existierte; jetzt `txRingDepth() >= MAX_RING-1`.
- `1bea29a1` bestand den Fix auf echter RAK4631-Hardware (DK5EN-90): `tools/bench/eth03_probe.py` mit vier Datagrammen im 0,5-s-Abstand -- kein `RESETREAS`, `stack_hwm` 590 Woerter (2,3 kB Reserve), HTTP 200 vor und nach dem Stoss, Ergebnis PASS (`test/golden/hw/G2/rak-90/eth03-probe-after.txt`). Auf dem Stand vor dem Fix fiel derselbe Lauf schon nach dem ersten Datagramm.

Als Nebenprodukt dieser Untersuchung wurde `EXT-02` abgeschlossen (`--extudp off` liess `hasExternIPaddress` vorher gesetzt, sodass ein Off/On-Zyklus den Socket nicht neu oeffnete) -- der Fix liegt in `src/extudp_functions.cpp`/`src/command_functions.cpp`, ausserhalb dieses Kapitels, wird hier aber der Vollstaendigkeit halber genannt, weil er Teil derselben Fehlersuche war.

**Auf Hardware bestanden** (`1bea29a1`): PASS-Log wie oben; das Verhalten des Fixes ist damit nicht nur quellbewiesen, sondern auf einem RAK4631 gemessen.

### ETH-02 / ETH-02b: DHCP-Wiederanmeldung und Dienststart nach spaeter Lease

**78. Ein Knoten, der nie eine DHCP-Lease bekam, fragte nie wieder -- und eine spaet erworbene Lease liess Webserver und EXTUDP tot liegen** (`54b6d162`, `1bea29a1`, ETH-02/ETH-02b). `checkDHCP()` ist `Ethernet.maintain()` und erneuert nur eine bestehende Lease; der einzige Pfad, der eine neue erwirbt (`resetDHCP()`), lag ausschliesslich im `if(bGATEWAY)`-Zweig von `gatewayService_nrf52()`. Ein Webserver-only-Knoten oder ein EXTUDP-Peer hatte damit genau einen Versuch, im Setup, und nie wieder einen -- gemessen an den eigenen Zaehlern des Knotens (`got_ip_n;0`, `resets;0` ueber einen physischen Kabelwechsel hinweg). Fix in `src/nrf52/nrf52_main.cpp` ab Zeile 2065: alle 30 s `resetDHCP()`, solange `hasETHlink()` wahr ist und keine feste IP gesetzt ist; `resetDHCP()` statt `initethDHCP()`, weil letzteres den W5100S bei jedem Versuch hardware-resettet.

`1bea29a1` bestaetigte diesen Retry beilaeufig auf Hardware (ein Boot ohne Lease im ersten Anlauf loeste ihn tatsaechlich aus) und deckte dabei ETH-02b auf: eine spaet erworbene Lease liess `web_timer` bei seinem 15-Minuten-Takt stehen, sodass Webserver und Extern-Socket bis zu eine Viertelstunde tot blieben, obwohl der Knoten pingbar war und EXTUDP schon antwortete. Fix direkt im Retry-Zweig (`src/nrf52/nrf52_main.cpp:2106-2120`): sobald `hasIPaddress` nach dem Retry wahr wird, wird `web_timer` auf 0 gesetzt, sodass der naechste Schleifendurchlauf Dienste startet und die `--info`-Kopie des Flags aktualisiert -- Marker `[ETH];event;dhcp_acquired_late`.

Verifikationsstand: ETH-02 ist auf Hardware bestaetigt (siehe oben). ETH-02b ist ebenfalls auf Hardware bestanden, vier Tage spaeter (`11d74abc`, DK5EN-90): Kabel beim Boot gezogen, `--reboot` ueber Seriell, Link faellt ohne Lease (`RESETREAS=0x00000004`), Kabel nach rund 75 s wieder gesteckt. Die Aufzeichnung zeigt `[ETH];event;dhcp_acquire_retry` gefolgt von `got_ip` und `dhcp_acquired_late` bei 77,7 s; `curl` auf den Knoten antwortet sieben Sekunden spaeter mit 200, und das abschliessende `--info` liest `hasIpAddress: yes` -- vor dem Fix aus `1bea29a1` waere das Feld bis zum naechsten 15-Minuten-`web_timer` bei "no" geblieben. Belege in `test/golden/hw/G2/rak-90/eth02b-README.md` und `eth02b-late-lease.txt`.

### DR-16: totes `|| bHeyFirst` liess die erste Telemetrie den vollen Takt abwarten

**79. Kleiner Logikfehler, echte Auswirkung: die erste TM-Meldung nach dem Boot wartete unnoetig den vollen `akt_timer` ab** (`6b03bd37`, DR-16). Das Hey-Gate lief in jedem Schleifendurchlauf zuerst und loeschte `bHeyFirst` bedingungslos, sodass die Telemetrie-Bedingung `... || bHeyFirst` in `src/nrf52/nrf52_main.cpp:2024` nie mehr wahr wurde, wenn sie geprueft wurde. Fix: ein eigenes `bTeleFirst` (`nrf52_main.cpp:284`) trennt das Telemetrie-Gate vom Hey-Gate. Kein Hardware-Test moeglich: `nrf52_main.cpp` wird von keinem nativen Env kompiliert, so im Commit vermerkt.

### DISP-01: zwei widerspruechliche OLED-Zuordnungen, eine vertauschte Log-Zeile

**80. Zwei Kopien derselben Panel-Zuordnung waren auseinandergedriftet, und ihre Log-Ausgabe benannte das falsche Panel** (`5b57d555`). `esp32_functions.cpp` und `src/nrf52/nrf52_functions.cpp` enthielten je eine eigene idtype-zu-u8g2-Zuordnung, gespeist vom selben Sondierungscode, aber mit widersprochenem Ergebnis: dieselbe Kennung 1 stand fuer `u8g2_1` auf ESP32 und fuer `u8g2_2` auf nRF52. Dazu druckte die Panel-Erkennung "SSD1306" fuer den SH1106-Zweig und umgekehrt -- eine vorherige Auswertung hatte diese vertauschten Log-Zeilen gelesen und daraus die falsche Diagnose gezogen, dass der ESP32-Zweig den Vertrag umkehrt. Bench-Nachweis auf DK5EN-93 (Heltec V3) und einem T-Beam zeigt beide Boards sauber mit der ungeaenderten Zuordnung. Fix: eine gemeinsame `mcSelectU8g2()`, aufgerufen von beiden Plattformen (`src/nrf52/nrf52_functions.cpp:40`); die zwei Log-Zeilen benennen jetzt, was sie tatsaechlich messen (Panel-Groesse, nicht Controller-Typ).

### Reine Restrukturierung -- kein Verhaltensunterschied beabsichtigt oder gemessen

Die folgenden Commits sind Teil der DRY-Unifizierungskampagne. Alle beanspruchen und belegen (Ressourcen-Baseline, byte-identische Regionen auf den beiden Envs ohne Spielraum) keine Verhaltensaenderung; sie zaehlen fuer den Bug-Fix-Charakter dieses Kapitels nicht, sind aber der Grund fuer den grossen Loeschsaldo (+291/-1469 ueber die 7 Dateien).

- **GRD-01** (`46ed0f3c`): sieben handgeschriebene Varianten des Praedikats "hat dieses Board ein U8g2-Display" werden zu einem `#if MC_HAS_U8G2` (`configuration_global.h`); in `src/nrf52/nrf52_functions.cpp` betrifft das eine Zeile (:15).
- **D1-10 Loop-Scheduler** (`e64ce346`): sieben gemeinsame Timer-Praedikate (Retransmit, MCP-Refresh, Batterie, Heap-Monitor, drei Sensor-Timer) wandern in eine gemeinsame Tabelle (`src/loop_scheduler.{h,cpp}`); die Rumpfe ziehen unveraendert in `loop_actions_nrf52.cpp`. In `src/nrf52/nrf52_main.cpp` etwa 100 Zeilen weniger.
- **C1-C4-Carve-outs** (`03c4ba65`, `35b8823f`, `7f5a0469`, `8c48243c`, `329b1bac`, `85f0c82e`): der UDP-Frame-Handler, die Socket-Schreib-Primitiven, `checkSerialCommand()` und der Gateway-Service-Block werden aus `nrf52_main.cpp`/`nrf_eth.cpp` in eigene Uebersetzungseinheiten (`udp_frame_nrf52.cpp`, `serial_command_nrf52.cpp` u.a.) verschoben, damit sie kuenftig nativ testbar sind. `nrf_eth.h` gewinnt dabei 13 Zeilen (neue Deklarationen), `nrf_eth.cpp` und `nrf52_main.cpp` verlieren den jeweils verschobenen Koerper. Absichtlich NICHT vereinheitlicht mit der ESP32-Fassung (das ist eine eigene Entscheidung fuer eine spaetere DR-Reihe); `85f0c82e` haelt dabei sechs bestehende Verhaltensunterschiede zwischen den Plattformen fest, ohne sie selbst zu beheben -- darunter eine Zero-Scan-Grenze im nRF52-Empfangspfad, die erst durch das Nebeneinanderliegen der beiden Schleifen als Ein-Zeilen-Diff sichtbar wurde und noch am selben Tag als eigener Fix folgte (`3d2ce699`, siehe K05).
- **W3/W3c Settings-Vereinheitlichung** (`b0adf723`, `3fd5dcaa`, `64f3cdd1`): `struct s_meshcom_settings` wird einmal statt zweimal definiert; in `src/nrf52/WisBlock-API.cpp`/`.h` und `src/nrf52/api_functions.cpp` fallen dabei tote LoRaWAN-OTAA-Ueberreste (Timer-Geruest, `send_repeat_time`, `auto_join`) ersatzlos weg (394 Zeilen in `b0adf723`, weitere 200 in `3fd5dcaa` fuer das alte On-Disk-Kompat-Struct). `64f3cdd1` vereinheitlicht nur die Schreibweise von `node_gpsbaud` (`WisBlock-API.h:328`, `unsigned int` -> `uint32_t`, layoutneutral); der im selben Commit behobene Stack-Byte-Leak bei `node_update` liegt auf der ESP32-Seite und damit ausserhalb dieses Kapitels.
- **`--srvip`-Bench-Override** (`ee545088`): kein Bug-Fix, sondern eine neue Bench-Funktion hinter `INSTRUMENT_ENABLED` -- der nRF52 bekommt (wie der ESP32 seit laengerem) einen Server-Override fuer Testkorpora, in `src/nrf52/nrf_eth.cpp`. Ausgelieferte Images sind unberuehrt.

## K12 ESP32-Kern und Peripherie

**81. `esp32_main.cpp` verliert sieben Timer-Bloecke an den neuen Loop-Scheduler, und ein Advisor-Fund verhindert dabei einen stillen Ausfall auf drei Board-Klassen.** (`e64ce346`, D1-10 -- der Scheduler selbst ist K07s Thema). In `esp32loop()` entfallen die Inline-Bloecke fuer `retransmit_timer`, `mcp_refresh_timer`, `BattTimeWait` (Batterie-/PMU-Teil) und die drei Sensor-Timer; `esp32_main.cpp` schrumpft dadurch um 222 Zeilen. Im ersten Entwurf sass der neue Aufruf `loopSchedulerRun(millis())` innerhalb des `if(bRadio)`-Blocks und im `#else`-Zweig des T5-ePaper-Codes -- damit haetten Batterie-Ablesung, Heap-Monitor und drei Sensor-Timer auf einem Knoten mit totem Funkchip, auf `env:esp32-external-radio` (wo `bRadio` konstruktionsbedingt falsch ist) und auf `t5_epaper` stillschweigend aufgehoert zu laufen. Der Advisor markierte das als Blocker; der Aufruf liegt jetzt auf Funktionsebene vor diesem Block (`esp32_main.cpp:2124`), direkt hinter dem mitverschobenen `BattTimeWait`-Nullinit (`esp32_main.cpp:2121-2122`, zuvor Teil des jetzt entfernten Blocks weiter unten in der Funktion). Nur `retransmit_timer` haengt tatsaechlich vom Funkchip ab; das Gate dafuer sitzt jetzt in `loopEnabled_retransmit()` (K07-Datei). Ein echter, vor dem Merge gefundener und behobener Fehler, keine reine Verschiebung. Der bereits durch `66c0e556` abgespaltene Luefter-/NTC-Block (`FanTimeWait`) bleibt unveraendert direkt in `esp32_main.cpp` stehen (Zeile 3559 ff.) und wandert nicht in den Scheduler.

**82. `adc_chars` war als Array von 36 Kopien deklariert statt als eine Instanz -- echter Bugfix mit gemessener RAM-Zahl.** (`a50c5614`, W1, Defekt 1). In `src/batt_function_old.cpp:198` stand `esp_adc_cal_characteristics_t adc_chars[sizeof(esp_adc_cal_characteristics_t)]` -- ein Array mit so vielen Elementen, wie die Struktur selbst Bytes hat, statt einer einzigen Instanz. Jetzt `esp_adc_cal_characteristics_t adc_chars;`, beide Aufrufstellen (`esp_adc_cal_characterize()`, `esp_adc_cal_raw_to_voltage()`) uebergeben `&adc_chars`. Ein Kommentar an Ort und Stelle warnt ausdruecklich davor, das wieder auf `[sizeof(...)]` "zu korrigieren". Gemessen auf `ttgo_tbeam` per `nm`, nicht geschaetzt: 1296 B -> 36 B, exakt die vom Audit vorhergesagten 1260 B/Board.

**83. ESP32 schrieb ein uninitialisiertes Stack-Byte in ein Settings-Feld.** (`64f3cdd1`, fix(settings)). `node_update` baute in `esp32_main.cpp` den Zeitstempel `"%04i-%02i-%02i %02i:%02i:%02i"` (19 Zeichen + NUL = 20 Byte) in einem `char[80]`-Scratch-Puffer, kopierte davon aber `memcpy(meshcom_settings.node_update, ctemp, 21)` in ein `char[21]`-Feld -- ein Byte mehr, als `snprintf()` gefuellt hatte. Das 21. Byte war damit uninitialisierter Stack-Inhalt, der in einem persistierten Settings-Feld landete; nie zurueckgelesen (der String terminiert bei Index 19), aber ein echter Undefined-Behavior-Fund. Das Feld ist jetzt `char[20]` (wie auf nRF52), und der Kopiervorgang ist auf die Feldgroesse selbst gebunden mit explizit gesetztem Terminator: `memcpy(meshcom_settings.node_update, ctemp, sizeof(meshcom_settings.node_update) - 1); meshcom_settings.node_update[sizeof(meshcom_settings.node_update) - 1] = 0x00;` (`esp32_main.cpp:2916-2917`). Bewusst nicht direkt formatiert wie auf nRF52: die Datumsfelder sind `int`, xtensa-gcc kann `"%02i"` nicht auf zwei Zeichen begrenzen, `-Werror=format-truncation` verwirft ein 20-Byte-Ziel -- laut Commit zwei fehlgeschlagene Anlaeufe, bevor der Weg ueber den breiten Scratch-Puffer blieb. Layoutneutral: `node_gpsbaud` (zweite Korrektur desselben Commits, eine Typspalt-Vereinheitlichung `unsigned long`/`unsigned int` -> `uint32_t`) betrifft laut Commit keine der fuenf K12-Dateien.

**84. Der ungueltige OneWire-Pin 99 verschwindet aus acht Varianten, und der Makro-Fallback lernt die Pruefung, die der Laufzeitpfad schon hatte.** (`b07f5be4`, R3-03). Acht Board-Varianten (u. a. Heltec V3/V4/Stick/Tracker, Vision-Master E213/E290, Wireless Paper, `ttgo_tbeam_supreme`) definierten `OneWire_GPIO 99` -- auf keinem dieser Boards ein gueltiger GPIO, was die alten Kommentare selbst zugaben ("nicht getestet", "ungenutzt"). Alle acht tragen jetzt `-1`, dieselbe Schreibweise, die `T-ETH-ELITE_1262` bereits nutzte. `-1` statt Loeschen der Zeile, weil die gesamte OneWire-Implementierung in `onewire_functions.cpp` hinter `#ifdef OneWire_GPIO` liegt -- Loeschen haette `--owgpio <pin>` als Feature entfernt, nicht nur aufgeraeumt. Der eigentliche Defekt lag tiefer: der Laufzeitpfad weist `--owgpio <= 2` bereits zurueck (`command_functions.cpp`), aber der Makro-Fallback in `init_onewire_dht()` (`onewire_functions.cpp:65`) und `init_onewire_ds18()` (`onewire_functions.cpp:224`) uebernahm den Wert ungeprueft -- der DS18-Zweig setzte dabei zusaetzlich `one_found = true`, meldete also einen Sensor auf einem Pin, den es nicht gibt. Beide Fallbacks pruefen jetzt `if(OneWire_GPIO <= 0) return;`, was die ganze Fehlerklasse schliesst statt nur die acht Einzelfaelle. Echter Bugfix.

**85. Zwei Aenderungen an derselben Zeile, die eine Log-Beschriftung berichtigt und eine Verdopplung entfernt, wirken sich auch auf `esp32_functions.cpp` aus.** (`5b57d555`, DISP-01). Die Kernkorrektur -- vertauschte SSD1306/SH1106-Log-Labels und eine doppelte, gegenlaeufige Sondenwert-Zuordnung -- ist in K07 (`loop_functions.cpp`/`.h`) beschrieben; hier betroffen ist nur der Aufrufer in `esp32_functions.cpp:189`: `initDisplay()` ruft jetzt `u8g2 = mcSelectU8g2(idtype);` statt die Zuordnung `if(idtype == 1) u8g2 = &u8g2_1; else u8g2 = &u8g2_2;` lokal zu wiederholen. Der zugehoerige Kommentarblock ("SSD1306 .... idtype 2") war laut Commit ebenfalls verkehrt beschriftet und ist entfernt. Bench-belegt auf DK5EN-93 (Heltec V3) und einem T-Beam -- beide erhalten weiterhin korrekt `u8g2_1`.

**86. Dieselbe Guard-Vereinheitlichung (`MC_HAS_U8G2`) ersetzt auch die ESP32-Kopie des U8g2-Praedikats.** (`46ed0f3c`, GRD-01). Die vollstaendige Geschichte -- acht Stellen in drei Dateien, eine ausgelaufene Board-Aufzaehlung, die strukturelle statt aufzaehlende Loesung -- steht in K07; in `esp32_functions.cpp:183` aendert der Commit nur den einen betroffenen `#if`-Ausdruck (elf Bedingungen minus der nRF52-only-Boards, zuvor `BOARD_WIRELESS_PAPER` statt der ueberall sonst verwendeten Schreibweise `WP_DISP`) auf `#if MC_HAS_U8G2`. Reine Restrukturierung, keine Verhaltensaenderung; Beleg dafuer siehe K07 (34/34 Envs, RAM/Flash groesstenteils byteidentisch).

**87. Zwei tote Codeuebernahmen aus `esp32_main.cpp` entfernt.** (`573f28fd`, W1/D6-10). Das ungenutzte `#include <SD.h>` in `esp32_main.cpp` ist entfernt -- die Datei beruehrt kein SD-Symbol, der einzige `HAS_SDCARD`-Block darin setzt nur GPIO-Richtungen. `SD.h` selbst bleibt genutzt (in `command_functions.cpp` und `mheard_functions.cpp`); nur `LilyGo_T-Beam-1W` definiert `HAS_SDCARD` ueberhaupt, an dieser Variante ist die Entfernung ueberprueft. Reine Aufraeumung, keine Verhaltensaenderung.

**88. Zwei Umbau-Schnitte loesen den Gateway-Dienst und den Seriell-Befehlsparser aus den beiden 2000-Zeilen-Hauptschleifen, ohne sie zu vereinheitlichen.** (`8c48243c`, C4-Carve; `7f5a0469`, C3-Carve). Beide Commits sind ausdruecklich als unveraendertes Verhalten deklariert und dagegen geprueft: beide Funktionsrumpfe diffen laut Commit byteidentisch gegen den Stand davor, nur eingerueckt bzw. verschoben. `8c48243c` zieht den Gateway-Block aus `esp32loop()` in `gatewayService_esp32()` (eigene Datei `src/esp32/gateway_service_esp32.cpp`, Aufruf jetzt `esp32_main.cpp:3742`) und aus `nrf52loop()` in `gatewayService_nrf52()` -- bewusst zwei getrennte Funktionen, nicht eine gemeinsame, weil die eigentliche Vereinheitlichung (Audit-Zeile D1-09) als Risiko-3-Aenderung explizit hinter einen Soak gestellt und hier nicht versucht wird; der Commit legt dabei offen, wie weit ESP32 und nRF52 in Heartbeat-Watchdog, Sende-vs-Empfangsreihenfolge und Recovery-Pfad bereits auseinanderlaufen. `7f5a0469` zieht `checkSerialCommand()` aus `esp32_main.cpp` in `src/esp32/serial_command_esp32.cpp` (Aufruf jetzt `esp32_main.cpp:3541`) und die nRF52-Fassung in eine eigene Datei -- ebenfalls bewusst getrennt gehalten. Nebeneffekt: vier bisher dateiweit sichtbare Parser-Variablen (`strTextWork`, `strText`, `iTxtPos`, `iTxtLen`) werden dabei `file-static`, da sie ausserhalb nirgends referenziert waren. Beide Commits nennen als Beleg unveraenderte Speicherbelegung auf `ttgo_tbeam`/`E22_XML-DevKitC` und +0 B RAM auf allen 29 ESP32-Envs.

**89. Ein Timer-Konflikt zwischen Batterie-Ablesung und Luefteransteuerung wird durch Aufspalten geloest, statt eine Kadenz der anderen zu opfern.** (`66c0e556`, perf(esp32)). Auf Operator-Entscheidung sollten `BattTimeWait` (bisher 500 ms auf ESP32) und `INA226TimeWait` (bisher 15 s) auf die jeweils langsamere, bereits auf nRF52 gueltige Kadenz vereinheitlicht werden (30 s bzw. 60 s). Die Falle: derselbe `BattTimeWait`-Block treibt auf `LilyGo_T-Beam-1W` zusaetzlich die Luefteransteuerung (`getTempForNTC()` mit Hysterese bei 40/35 Grad, hinter `#if defined(NTC_PIN) && defined(FAN_CTRL)`) -- eine Verlangsamung auf 30 s haette dort das Hochfahren des Luefters an einem 1-W-Leistungsverstaerker um bis zu einer halben Minute verzoegert. Der Luefter-/NTC-Block ist deshalb in einen eigenen 500-ms-Timer (`FanTimeWait`, `esp32_main.cpp:3559`) ausgelagert, mit eigenem `FanWaitCounter`; die Batterie-Ableselogik selbst laeuft jetzt auf 30 s, ohne eigenen Druckzaehler (`BattWaitCounter` ist entfernt, da er nur noch eine bereits durch die neue Kadenz gedrosselte Debug-Ausgabe zusaetzlich gedrosselt haette -- bei alter Logik waere das auf ~10 Minuten gestreckt worden). Echte Verhaltensaenderung mit begruendeter Ausnahme, kein reines Umbenennen. Verifikation laut Commit: 32/32 Envs, Speicherregion auf dem knappen `ttgo_tbeam` unveraendert. Der hier abgespaltene Luefterblock bleibt bis heute unveraendert in `esp32_main.cpp` stehen, auch nachdem `e64ce346` die Batterie-/PMU-Ablesung selbst in den Loop-Scheduler migriert (siehe oben).

**90. R2-04s Umstellung von `String` auf `char[]` in `aprsMessage` zieht in `softser_functions.cpp` eine rein mechanische Anpassung nach sich.** (`f50cc954`, R2-04). Die eigentliche Umstellung (sieben Felder, gemessen -109 068 B Flash / +15 024 B RAM ueber 34 Envs) betrifft keine K12-Datei direkt; in `softser_functions.cpp` sind lediglich die Call-Sites an die neue Zugriffsform angepasst: `aprsmsg.msg_payload.substring(...)` wird zu `String(aprsmsg.msg_payload).substring(...)` (u. a. `softser_functions.cpp:401, 418, 441, 467, 482`), `aprsmsg.msg_payload.charAt(10)` wird zu `aprsmsg.msg_payload[10]` (Zeilen 413, 436, 459, 464, 475). Reine Anpassung an eine geaenderte Typ-Signatur, keine eigenstaendige Logikaenderung; ohne die R1-04-artige Absicherung waere hier nichts zu pruefen, da `msg_payload` weiterhin nullterminiert ist.

Nicht in K12 gelandet, trotz Erwaehnung im Kampagnenkontext: die `--tempoff`-Klemmung auf -50..50 Grad (`5f09ad06`) und die `--onewire gpio`-Klemmung auf 0..99 (`d7b4b96c`) liegen beide vollstaendig in `src/command_functions.cpp` (bzw. `src/udp_functions.cpp`), keiner der fuenf K12-Dateien -- hier daher ausgelassen. `36c37b5f` (Drift-Matrix M1) und `2991034e` (W2) beruehren laut Diff keine K12-Datei.

## K10 LVGL: ui_common und ein lv_conf

**91. Der T-Deck Pro bekommt die LVGL-Heap-Einstellung, die die anderen drei
LVGL-Boards schon laengst hatten** (`50955770`, R4-01). Fuenf `lv_conf.h`
existierten im Baum; vier davon (`variants/t_deck/lv_conf.h`,
`variants/t_deck_plus/lv_conf.h`, `variants/t5_epaper/lv_conf.h`,
`src/t-deck/`) liefen mit `LV_MEM_CUSTOM 1` und `ps_malloc`, die fuenfte,
`config/lv_conf.h`, der einzige Konsument des T-Deck Pro, stand noch auf
`LV_MEM_CUSTOM 0`. Mit `0` legt LVGL seinen Arbeitsspeicher als statisches
Array an (`lib/lvgl/src/misc/lv_mem.c:95`), zwingend im internen DRAM, weil
ein statisches Array nicht im PSRAM liegen kann -- 48 kB, reserviert beim
Boot, unabhaengig davon, was die Oberflaeche je braucht. Gemessen,
`t_deck_pro`, sauberer Build beidseitig: RAM 181 008 -> 131 840 B (-49 168,
55,2 % -> 40,2 %), Flash 1 986 385 -> 1 984 945 B (-1 440, die
TLSF-Implementierung faellt mit weg). Verifiziert im Artefakt: `nm` zeigt
`work_mem_int$3696` (0xc000, `.bss` an einer internen SRAM-Adresse) nicht mehr,
`lv_mem.c.o` traegt nur noch `U ps_malloc` / `U ps_realloc` / `U free` als
Allokator-Referenzen. Die Aenderung geht ins bestehende `config/lv_conf.h`,
nicht in eine sechste Board-Variante -- PlatformIOs Abhaengigkeits-Datenbank
bestaetigt, dass diese Datei genau einen Env erreicht. Kein Feldtest auf
dieser Bank (kein T-Deck Pro vorhanden); der Befund steht auf Build+`nm`.

**92. 20 tote `lv_conf.h`-Kopien geloescht, keine davon vereinheitlicht**
(`6b03bd37`, W7). Zwanzig Board-Variantenverzeichnisse
(`variants/E22*`, `variants/T-ETH-ELITE_1262`, `variants/esp32-loraprs-*`,
`variants/heltec_*`, `variants/t_echo`, `variants/ttgo*`,
`variants/wiscore_rak4631` -- vollstaendig in der Dateiliste oben) trugen je
eine 713-Zeilen-Kopie von `lv_conf.h`, obwohl keiner dieser Boards LVGL
einbindet. Jede Kopie wurde einzeln als unerreichbar nachgewiesen: LVGL steht
in `lib_ignore` des jeweiligen Envs, und keine LVGL-einbindende Quelldatei
liegt in dessen `build_src_filter`. `variants/t_deck/lv_conf.h` und
`variants/t_deck_plus/lv_conf.h` blieben unangetastet -- diese beiden Boards
haben ihr eigenes `lv_conf.h`, wie zuvor. Ausdruecklich NICHT geloescht:
`variants/t5_epaper/lv_conf.h`, obwohl auch t5_epaper das Board-eigene
`config/lv_conf.h` faktisch benutzt -- es bleibt erreichbar ueber LVGLs
eigenen `__has_include`-Fallback ohne explizites
`LV_CONF_INCLUDE_SIMPLE`. Die urspruengliche Audit-Zeile hatte 21 Dateien /
15 010 Zeilen behauptet; korrigiert auf 20 Dateien / 14 240 Zeilen. Verifiziert
per `ls variants/t_deck*/`: beide Verzeichnisse fuehren weiterhin ein eigenes
`lv_conf.h` neben `configuration.h` und `platformio.ini`.

**93. `src/ui_common/` fasst den GPS-Handshake und den LVGL-Screen-Manager der
beiden LVGL-Zwillinge (T-Deck Pro, T5-EPaper) zusammen** (`55a7b4c4`, W6a
D4-01/02). `setupGPS()`, `getAck()` und `GPS_Recovery()` wandern nach
`src/ui_common/gps_protocol.cpp`/`.h`, parametrisiert auf `HardwareSerial&`
statt auf ein Board-Header; `src/t-deck-pro/peri_gps.cpp` schrumpft von 417
auf 271 Zeilen, `src/t5-epaper/peri_gps.cpp` von 387 auf 312. Der
Screen-Manager wird als eine `.c`-Datei fuer beide Boards gefuehrt:
`src/t5-epaper/scr_mrg.cpp` wird zu `src/ui_common/scr_mgr.c` (Rename,
+66/-14), `src/t-deck-pro/ui_scr_mrg.c` (267 Zeilen) entfaellt vollstaendig.
Beide alten Header, `src/t-deck-pro/ui_scr_mrg.h` und
`src/t5-epaper/scr_mrg.h`, bleiben als reine Weiterleitung auf
`../ui_common/scr_mgr.h` stehen, damit `ui_deckpro.cpp` bzw. `t5epaper_main.cpp`
ihre bisherigen Include-Namen unveraendert benutzen koennen. Da beide Envs sich
gegenseitig ausschliessen, spart das keinen Flash-Byte -- der Gewinn ist ein
einziger Codepfad statt zweier auseinanderlaufender. Beim Zusammenfuehren
gefunden, aber bewusst NICHT in den gemeinsamen Code gezogen: T-Deck Pros
PCAS03-Satz trug ein Pruefsummen-`*26` statt des korrekten XOR `*02`
(T5-EPaper hatte es richtig) -- heute folgenlos, da der `setupGPS()`-Aufruf
auf dem T-Deck Pro auskommentiert ist. CAVEAT aus dem Commit selbst: die
`ui_common`-Haelfte ist nur build-verifiziert, kein T-Deck Pro auf dieser
Bank, niemand hat die Oberflaeche danach live gesehen.

**94. `t5_epaper` kompiliert zum ersten Mal, als Voraussetzung fuer die
Zusammenfuehrung** (`af179892`). Der Env galt als dauerhaft defekt (fehlendes
`variants/t5_epaper/configuration.h`); `src/t5-epaper/peri_gps.cpp` war
seither ein veralteter Fork von `src/t-deck-pro/peri_gps.cpp`, 96 Zeilen
auseinandergelaufen, mit vierzehn Referenzen auf `bGPSDEBUG_DETAIL` (ein
Symbol, das nirgendwo im Baum existiert) und einer zweiten, kollidierenden
`TinyGPSPlus gps`-Definition statt des `extern` aus `gps_functions.cpp:51`.
Beides wurde auf den gepflegten Zwilling ausgerichtet -- die eigentliche
Vereinheitlichung (`55a7b4c4`) baute auf diesem lauffaehigen Env auf.

**95. Ein Zeiler in `src/t-deck-pro/ui_deckpro.cpp:1326` folgt dem
Mheard-Struct-Umbau, nicht der LVGL-Vereinheitlichung** (`09524068`,
`79242eda`). Die Anzeigezeile fuer die Mheard-Tabelle wechselt zunaechst von
`decodeMHeard(mheardBuffer[iset], mheardLine)` auf
`mheardLineFromRecord(mheardRecords[iset], mheardLine)` (R2-01, der
Text-Codec faellt weg) und danach von
`mheardLine.mh_time.substring(0, 5).c_str()` auf `%.5s` gegen das jetzt feste
`char[]`-Feld `mh_time` (R2-04, zweite Haelfte). Beide Aenderungen sind
Aufrufstellen-Folgen dieser beiden anderen Kampagnenzeilen; die Datei landet
in dieser Liste nur, weil sie ein LVGL-UI-File des T-Deck Pro ist.

### Kampagnen-Commits

- `79242eda` -- R2-04, zweite Haelfte: mheardLine traegt Text, nicht String
- `55a7b4c4` -- W6a: D1-01 nine decided drift rows, and ui_common for the two LVGL twins
- `50955770` -- R4-01: the fifth lv_conf.h was the one nobody updated -- 49 kB of internal DRAM back
- `09524068` -- R2-01: the mheard text codec is gone -- 83 kB RAM and 54 kB flash back
- `af179892` -- Build envs: t5_epaper compiles for the first time, and a header gets a guard
- `6b03bd37` -- Wave: D3-01, D3-02, D3-05, EXT-01, DR-16/DR-14, 20 dead lv_conf.h

## K13 T-Deck

Drei Commits beruehren die T-Deck-LVGL-Oberflaeche; sie sind Nebenwirkungen
dreier DRY-Unification-Wellen, die primaer andernorts ansetzen (APRS-Struktur,
ESP32-Settings-Schema, RAM-Kampagne), aber jeweils T-Deck-Aufrufstellen
mitziehen mussten, damit das Board weiter baut. `src/t-deck/tdeck_helpers.cpp`
steht in der Dateiliste dieses Kapitels, wird aber von keinem der drei
Commits beruehrt -- siehe Abweichungen am Ende dieses Kapitels.

**96. `aprsMessage`-Felder sind jetzt `char[]`, und zwei T-Deck-Stellen mussten
auf String-Methoden verzichten.** `f50cc954` (R2-04). Die eigentliche
Aenderung -- sieben Arduino-`String`-Felder in `aprsMessage` werden zu festen
`char[]`, weil `decodeAPRS()` jedes Feld ohnehin schon in einem lokalen
`char[UDP_TX_BUF_SIZE]` aufbaut und dann nur zum Herauskopieren in ein
`String` verpackt -- liegt ausserhalb dieses Kapitels
(`src/aprs_structures.h`, `src/aprs_functions.cpp`). In
`src/t-deck/lv_obj_functions.cpp` folgt `tdeck_add_MSG()` dem Typwechsel:
`aprsmsg.msg_source_path.equalsIgnoreCase(...)` wird zu
`strcasecmp(aprsmsg.msg_source_path, local_call.c_str()) == 0` (Zeile 4391),
und `aprsmsg.msg_source_path.length() > 0` wird zu
`strlen(aprsmsg.msg_source_path) > 0` (Zeile 4405). Reine Restrukturierung an
dieser Stelle: gleiche Semantik, anderer Aufruf fuer denselben Feldtyp. Die
einzige Verhaltensaenderung des Gesamtcommits -- eine "Callsign" laenger als
`MAX_CALL_LEN` wird jetzt beim Aufsplitten abgewiesen statt erst spaeter durch
`checkRegexCall()` -- betrifft den Decoder, nicht die hier zitierten
T-Deck-Zeilen; auf der Anzeige wirkt sie sich nicht sichtbar anders aus, da
beide Pfade den Frame ohnehin verwerfen.

**97. `node_audio_start`/`node_audio_msg` sind jetzt `char[128]` statt
`String`, und drei T-Deck-Aufrufstellen sind entsprechend angepasst.**
`2b66f703` (W3, ESP32-Settings-Schema). Der Cutover ersetzt die
handgeschriebene ESP32-`Preferences`-Liste durch einen generischen
Schema-Walk per `offsetof()`/`sizeof()` (`src/settings_schema.cpp`, ausserhalb
dieses Kapitels); ein `FieldDescriptor` persistiert Rohbytes, und ein
`String`-Feld wuerde dabei seinen Heap-Zeiger statt seines Inhalts in den
NVS-Speicher schreiben. Deshalb werden beide Audio-Felder zu `char[128]`
(128 Byte deckt die 100-Zeichen-Grenze der Setup-Textarea mit Marge ab).
Betroffen: `src/t-deck/event_functions.cpp:557-566` (Vergleich per
`strncmp`/Zuweisung per `snprintf` statt `.compareTo()`/direkter Zuweisung),
`src/t-deck/lv_obj_functions.cpp:4064-4065` und `:4208` (`.c_str()` entfaellt,
da `lv_textarea_set_text()`/`audio_play_file_or_cw()` bereits `const char*`
erwarten) sowie `src/t-deck/tdeck_main.cpp:313-314` (`Serial.printf`/
`audio_play_file_or_cw` ohne `.c_str()`). Reine Restrukturierung an allen vier
Stellen -- Vergleichs- und Anzeigesemantik bleiben gleich, nur der
Feldtyp aendert sich. Das Settings-Schema selbst enthaelt in diesem Commit
einen echten, schwerwiegenden Bugfix (ein `sanitize_loaded_settings()`-Aufruf
loeschte auf ESP32 bei jedem Boot etwa die Haelfte aller Einstellungen), der
aber keine der vier zitierten T-Deck-Dateien beruehrt.

**98. `getCountryDropbox()` vergleicht die Laenderliste jetzt per `strcmp`, weil
sie kein `String[]` mehr ist.** `2991034e` (W2, RAM-Kampagne). Die
RAM-Einsparung `R2-07/R3-06` macht `strCountry` zu einem `const char* const[]`
statt `String[]` (Aenderung selbst ausserhalb dieses Kapitels); die einzige
uebersehene Aufrufstelle war `src/t-deck/lv_obj_functions.cpp:1730`
(`strCountry[ic].compareTo("none")` kompilierte auf `const char*` nicht mehr
und wird zu `strcmp(strCountry[ic], "none")`). Kein T-Deck-Build war in der
urspruenglichen Agenten-Stichprobe enthalten; der Fehler wurde erst beim
gezielten Bauen von `t_deck` sichtbar und in derselben Welle behoben, bestaetigt
durch einen erfolgreichen `t_deck`-Build. Reine Restrukturierung: gleiche vier
Vergleichsergebnisse (`< 0`, `== 0`, `> 0` gegen "none"), nur ohne
`String`-Objekt.

**99. Abweichung:** Die Dateiliste dieses Kapitels nennt
`src/t-deck/tdeck_helpers.cpp` (+4/-0), aber keiner der drei genannten
Commits (`f50cc954`, `2b66f703`, `2991034e`) aendert diese Datei --
`git show <hash> -- src/t-deck/tdeck_helpers.cpp` liefert fuer alle drei
einen leeren Diff, und die Commit-Historie der Datei endet bei `5d9877e2`
(einer T-Deck-Keylock-Bank-Vorbereitung ausserhalb dieser drei Commits). Kein
Eintrag dafuer geschrieben, um keine unbelegte Aenderung zu behaupten.

## K14 EXTUDP und Aufzeichnung

**100. EXT-02: ein fehlgeschlagenes Socket-Open meldete Erfolg und versuchte es nie wieder**
(`8f0e9e31`, `d05a0dc3`). `EthernetUDP::begin()`/`WiFiUDP::begin()` liefern 0, wenn kein
Hardware-Socket mehr frei ist -- der W5100S hat vier davon, geteilt mit dem 1990er-Socket, DHCP
und dem Webserver. `startExternUDP()` (`src/extudp_functions.cpp:100`) warf diesen Rueckgabewert
bislang weg und tat danach bedingungslos zwei Dinge: "[EXT]...now listening" drucken, eine nie
gepruefte Behauptung, und `hasExternIPaddress` setzen. Beide Folgen wiegen schwerer als die
fehlende Pruefung selbst -- dieses Flag ist der fruehe Ausstieg der Funktion (`:128`) und genau
das, was `getExternUDP()` (`:418`/`:521`/`:971`) prueft, der Empfangspfad lief also zufrieden in
einen toten Socket, dauerhaft und neustartfest. Fix (`:167-180`): der Rueckgabewert wird geprueft,
hoechstens alle 30 s geloggt, und OHNE Latch zurueckgekehrt -- das Freibleiben des Flags IST der
Retry, denn der Aufrufer prueft `!hasExternIPaddress` in jedem Schleifendurchlauf. Zweite Haelfte
(`d05a0dc3`): `--extudp off` loeschte `hasExternIPaddress` bis dahin nicht, weil die Toggle-Zeile
in `command_functions.cpp` (ausserhalb dieses Kapitels) `nullptr` als Nachaktion trug -- ein
Off/On-Zyklus oeffnete den Socket damit NICHT neu, der Bediener sah keinen Fehler und nichts
passierte. Die Zeile ruft jetzt `tg_post_extudp_off()` -> `resetExternUDP()`. Beleg auf
DK5EN-90 fuer den urspruenglichen Defekt: drei `[EXT]`-Zeilen bei frischem Boot mit korrekten
Adressen, danach 0 von 23 Korpusobjekten beantwortet, ueber zwei Neustarts. Der
Bestaetigungsversuch NACH dem Fix scheiterte allerdings selbst -- der W5100S-Link war zu diesem
Zeitpunkt unten (`hasIpAddress: no`), sodass der Lauf nichts ueber den Fix aussagt. Das ist ehrlich
in `docs/BACKLOG.md` vermerkt: EXT-02 ist ein quellbewiesener Fix mit noch ausstehender
Hardware-Bestaetigung, und der urspruengliche 0-von-23-Befund ist nur eine von zwei moeglichen
Ursachen fuer dieses Symptom, nicht eine bewiesene Diagnose.

**101. EXT-01: eine leere Telemetrie-Antwort loeste einen vollen UDP-Reset aus** (`6b03bd37`).
`sendExtern()`s "no telemetry"-Zweig (Zielpfad `"100001"`) baute keine JSON und liess `c_json`
leer -- die Kontrolle fiel danach in den gemeinsamen Sendeblock, `UdpExtern.write(c_json, 0)`
gab 0 zurueck, und dieses Falsy-Ergebnis wurde als fehlgeschlagener Schreibversuch gelesen: eine
komplette UDP-Socket-Teardown/Neuaufbau bei JEDEM gehoerten Telemetrie-Textframe, dazu ein
halboffenes Paket (`beginPacket()` ohne `endPacket()`). Fix: fruehes Return vor dem Sendeblock.
Bewusste Log-Aenderung dabei, im Quelltext vermerkt statt stillschweigend: die abschliessende
`[EXT];tx`-Zeile fuer einen nie stattgefundenen Versand wird nicht mehr gedruckt. Kein natives
Env kompiliert `sendExtern()`, daher gibt es fuer diese Zeile keinen ausfuehrbaren
Regressionstest -- im Commit ausdruecklich als offene Luecke benannt, nicht verschwiegen.

**102. Wettlauf ueber zwei Tasks im EXTUDP-Ack-Pfad, gefunden vor dem Ausliefern** (`3be9a9da`,
DR-18 Teil 2). `queueExternAck()` schrieb urspruenglich in `externQueue[]`
(`src/extudp_functions.cpp:80` `struct externQueueEntry`), einen Ring mit genau einem Erzeuger
per Konstruktion: `queueExtern()` aus `OnRxDone()`, dem LORA-Task, existiert nur um Arbeit aus
dem Funk-Callback herauszuhalten. Der neue Aufrufer lief dagegen aus `gatewayService_nrf52()`,
dem Hauptloop -- zwei Tasks, die denselben Slot `externQueue[externQueueWrite]` waehlen und einen
ungesperrten `int` inkrementieren, mit `is_json` zwischen Pufferkopie und Freigabe geschrieben.
Der Advisor-Pass hat das vor dem Commit gefunden, nicht danach. Die Loesung ist eine Streichung:
`flushExternQueue()` laeuft im SELBEN Task wie `queueExternAck()`, und der Aufrufer direkt darueber
ruft ohnehin schon `sendExtern()` synchron auf -- der Ring bot hier keinen Vorteil.
`queueExternAck()` (`src/extudp_functions.cpp:849`) sendet jetzt direkt ueber die neue, separate
`sendExternJson()` (`:888`), kein Slot, kein zweiter Erzeuger, kein Wettlauf, und die beiden
Ringplaetze bleiben dem Funkpfad vorbehalten, fuer den sie bemessen sind. Vier weitere Funde aus
demselben Advisor-Pass, im selben Commit behoben: `queueExternAck()` hatte keine `bEXTUDP`-Wache,
wo `queueExtern()` eine hat -- ergaenzt, und der Test pinnt jetzt beide Zustaende statt nur den
Erfolgsfall; das `via`-Feld wurde unescaped in JSON interpoliert (durch Kompilieren belegt: `via`
mit dem Wert `"udp"` erzeugte ungueltiges JSON) und durch eine eigene Transportnamen-Wache
ersetzt, nachdem der erste Fix (`ackAttrCallLen()`, eine reine Grossbuchstaben-Rufzeichen-Wache)
das Feld fuer genau diesen Wert leer geraeumt haette; `buildExternAckJson()` hatte keine
Testabdeckung, obwohl kein natives Env ihren einzigen Aufrufer kompiliert und der Zwilling die
Huelle nur stubt -- jetzt mit direktem Randfalltest. Ausdruecklich als schwaechere Beweisstufe
vermerkt: die sechs U2-Driftzeilen dieser Welle ruhen auf handgebauten Agreement-Tests, nicht auf
Korpus-Arithmetik, weil jedes Korpus-Rufzeichen DK5EN-* ist.

**103. Ring-Puffer fuer die ausgehende Warteschlange von 500 auf 264 Byte verkleinert (R1-06)**
(`2991034e`). `struct externQueueEntry`s `buffer[500]` (`src/extudp_functions.cpp`) war nach dem
tatsaechlichen Groesstfall bemessen zu grosszuegig: der einzige Aufrufer von `queueExtern()`
(`lora_functions.cpp`) reicht `RcvBuffer`/`size` direkt aus `OnRxDone()` durch, und diese Groesse
ist ueber `aprsmsg.msg_len` und `decodeAPRS()`s `rsize`-Argument auf `UDP_TX_BUF_SIZE` (255)
begrenzt -- Hardware-/Doppelpuffer-Grenze. 264 Byte lassen 9 Byte Reserve ueber diesem Maximum,
`queueExtern()`s eigene Klemme sichert es zusaetzlich ab. Die Signatur in
`src/extudp_functions.h:15` behaelt den Parameterschreibweise `buffer[264]` bewusst bei, obwohl
ein Array-Parameter ohnehin zu einem Zeiger zerfaellt -- eine Deklaration mit `500` wuerde
weiterhin eine Kapazitaet bewerben, die es nicht mehr gibt.

**104. MC_DIAG: eine Diagnose-Weiche statt zweier, und der String-Scan, der eine Werbeluege fing**
(`dd399f51`, R3-11/D2-09). Die urspruengliche Entscheidung lautete, `MC_CAPTURE` und die vier
`--spec*`-Kommandos hinter `INSTRUMENT_ENABLED` zu haengen -- eine falsche Praemisse, denn
`INSTRUMENT_ENABLED` ist in JEDEM Environment 0, waehrend `MC_CAPTURE` ueberall ausser E22_XML 1
war; woertlich befolgt haette das `--txcapture`, `--specstart`, `--specend`, `--specstep` und
`--specsamples` aus allen 34 ausgelieferten Abbildern geloescht. Betriebsentscheidung stattdessen:
EIN Schalter, `MC_DIAG`, Vorgabe 1, definiert in `src/configuration_global.h` neben `WP_DISP`
(vorher `MC_CAPTURE` in `src/capture_functions.h`), mit E22_XML-DevKitC als einziger Variante auf 0. Er steuert jetzt beides, was vorher zwei Namen brauchte: den Mitschnittring
(`src/capture_functions.cpp`/`.h`, `#if MC_DIAG` statt `#if MC_CAPTURE`) und die vier
Spektrum-Kommandos (`command_functions.cpp`, ausserhalb dieses Kapitels). WICHTIG FUER DIE
EINORDNUNG: `MC_DIAG` ist eine Feld-Diagnose, die in jedem ausgelieferten Abbild mitgeliefert
wird (Vorgabe 1) -- kein Testinstrument und nichts, was standardmaessig herauskompiliert wird; nur
die eine RAM-knappe Variante schaltet es ab. Der PFLICHTMAESSIGE String-Scan zahlte sich beim
ersten Lauf aus: mit `MC_DIAG=0` trug E22_XML weiterhin genau einmal die Zeichenketten
`specstart`/`specend`/`specstep`/`specsamples` im Abbild -- die Hilfezeile
`command_functions.cpp:917` (Referenz, nicht dieses Kapitels Datei) lag ausserhalb der Guard und
bewarb damit vier Kommandos, die dieses Board gar nicht mehr hatte. Exakt dasselbe Muster wie der
T-Deck-`--mute`-Feldbericht (INS-04). Guard ergaenzt, neu gebaut, neu gescannt: 0 Treffer. Ueber
alle 34 Abbilder: 0 UNERKLAERTE Abwesenheiten. `test/golden/command_name_scan.py` lernt `MC_DIAG`,
liest den Schalterstand dabei aber aus der `.ini`-Datei statt eine Board-Ausnahme hart zu
kodieren -- mutationsgepruefte Absicherung: eine hartkodierte Boardpruefung faellt durch, ein
unbekannter `MC_DIAG`-Wert degradiert zu "braucht einen Menschen", nie zu einem stillen Pass.

**105. Mechanische Anpassung an die `aprsMessage`-Feldbreiten** (`f50cc954`). Mit der Umstellung von
`aprsMessage` auf feste `char[]`-Felder (an anderer Stelle dieses Changelogs beschrieben) werden
in `src/test_inject.cpp` und `src/extudp_functions.cpp` alle direkten `String`-Zuweisungen
(`aprsmsg.msg_source_path = ...` usw.) durch `mcSet(feld, sizeof(feld), wert)` ersetzt und
`.c_str()`-Aufrufe entfernt, wo das Feld jetzt schon ein `char*` ist. Eine begleitende Korrektur
in `src/extudp_functions.cpp` (`getExtern()`): der Antwortpuffer `val[]` fuer den Rahmen
`":{Ziel}Nutzlast"` war mit einer von Hand gerechneten festen Groesse (163 Byte, aus einer
angenommenen Zielpfadlaenge von 9 und 150 Zeichen Nutzlast) dimensioniert; er wird jetzt aus
`MC_PATH_LEN`/`MC_PAYLOAD_LEN` berechnet, denselben Feldbreiten, die `aprsMessage` jetzt selbst
traegt -- damit rechnet der Compiler den tatsaechlichen Groesstfall nach, statt dass ein
Kommentar ihn behauptet. Keine der beiden Aenderungen ist hier eine neue Entscheidung, nur
Konsistenz mit einer Struktur, deren Substanz in einem anderen Kapitel liegt.

**106. ETH-03: EXTUDP-Pfad instrumentiert, keine Ursache in diesem Kapitel** (`4f4b02f1`, `d05a0dc3`).
Ein Stoss eingehender EXTUDP-Datagramme legte einen RAK4631 fuer rund 25 Sekunden komplett lahm
(Webserver und EXTUDP tot, seriell weiter ansprechbar). Vier `INSTR_SECTION()`-Sonden in
`src/extudp_functions.cpp` (`extudp_parse`, `extudp_read`, hinter `#if INSTRUMENT_ENABLED`,
Auslieferungsabbilder unberuehrt) widerlegten die naheliegende Vermutung eines SPI-Bus-Stalls im
Empfangspfad -- 397 Schleifendurchlaeufe, maximal 977 us, nichts haengt. Vier weitere Sonden an
den Stellen, die `hasIPaddress` loeschen koennten, feuerten in der Folge null Mal. Entschieden hat
am Ende keine dieser Sonden, sondern zwei grobe Instrumente: eine Laufzeit-Subtraktion (Traffic um
21:11:26, `--info` 83 s spaeter meldet eine Bootzeit von nur 21 s zuvor) und eine USB-Abmeldung
des Geraeteknotens waehrend des Verkehrs -- der Knoten startet neu. Die eigentliche Ursache (ein
zu kleiner Stack im festen 4-kB-Loop des Adafruit-Cores) und ihr Fix liegen ausserhalb dieses
Kapitels, in einem anderen Abschnitt dieses Changelogs beschrieben; was hier bleibt, ist die
Mess-Infrastruktur, mit der die drei falschen Theorien (SPI-Stall, haengender Pfad, geloeschtes
Flag) der Reihe nach ausgeschlossen wurden, und die vier `[ETH];clear;site`-Sonden, die im Baum
bleiben, weil ihr Nullbefund selbst Teil des Beweises ist.

## K16 Web-GUI

**107. Temperaturoffset auch auf dem Web-Pfad auf -50..50 Grad begrenzt** (`5f09ad06`). Die
Web-Setup-Parameter `tempoffsetindoor`/`tempoffsetoutdoor` (`src/web_functions/web_setup.cpp:609ff`)
schrieben den geparsten Float bislang direkt und ungeprueft in
`meshcom_settings.node_tempi_off`/`node_tempo_off` und speicherten sofort -- unabhaengig von der
Bereichspruefung, die die JSON-Wiederherstellung bereits kannte. Ausloeser: ein
Bench-Korpus-Rest von `--tempoff in 999999` blieb ohne Rueckstellung stehen und zeigte auf dem
Node dauerhaft eine absurde Innentemperatur. Fix: beide Web-Setter rufen jetzt denselben Pfad
wie die Konsole (`commandAction("--tempoff in/out ...")`), der die Grenzen -50..50 Grad
durchsetzt, und melden bei Ablehnung `returnCode 1` mit dem unveraenderten Altwert statt den
abgelehnten Eingabewert zu uebernehmen. Belegt mit `tempoff-web-proof.txt`.

**108. MHeard-Webseite zeigt zuletzt gehoert zuerst statt Rohreihenfolge (DR-28)** (`173a2970`).
`sub_page_mheard()` iterierte bisher linear ueber `mheardCalls[iset]` in physischer
Slot-Reihenfolge. Jetzt liefert `mheardSortedIndex()` einen nach Alter sortierten Indexvektor
(`web_functions.cpp:1476`), ueber den die Seite iteriert; die Speicherarrays selbst bleiben
unveraendert, nur die Anzeigereihenfolge aendert sich. Die Umstellung war Teil derselben
Welle wie ein Stack-Fix im nRF52-Loop-Task (ETH-03), der eine andere Datei betrifft und hier
nicht beansprucht wird.

**109. `sub_page_mheard()`-Stackpuffer auf nRF52 nach BSS verschoben (N-22-Muster)** (`79242eda`).
Seit derselben Aenderung wuchs `struct mheardLine` von ~112 auf 584 Byte (sieben
`String`-Handles wurden feste `char[]`-Felder). Drei Funktionen -- darunter
`sub_page_mheard()` in `web_functions.cpp` -- halten diese Struktur als lokale Variable auf
dem 4-KB-Loop-Task des nRF52, auf dem die N-22-Klasse bereits einmal einen
Stack-High-Water-Mark von 0 gemessen hat. Die lokale Instanz ist auf nRF52 jetzt
`static mheardLine mheardLine` statt eine Stackvariable (`web_functions.cpp:1463`, bedingt auf
`NRF52_SERIES`); ESP32 behaelt den Stackpuffer, da der dortige Loop-Task 8 KB hat und der
Rahmen dort kein Thema ist. Gemessen mit `-fstack-usage` auf `wiscore_rak4631`:
`sub_page_mheard()` 232 -> 696 -> 128 Byte (vorher / nach Umstellung / nach Fix).

**110. RX-Log-Puffer der Webseite wird erst bei Bedarf angelegt, 109 kB RAM zurueckgewonnen (R1-04)**
(`d5a071d8`). `ringbufferRAWLoraRX` hatte genau einen Leser -- die `rxlog`-Seite der Web-GUI --
kostete aber auf jedem Knoten statisch `MAX_LOG * (UDP_TX_BUF_SIZE+5)` Byte, auch auf Knoten
ohne Webserver oder ohne je geoeffnete Seite. `sub_page_rxlog()` (`web_functions.cpp:1306`)
ruft ab `web_functions.cpp:1349` vor der Ausgabe `rawLogEnsure()`; schlaegt die Allokation fehl, zeigt die Seite "RX
log buffer not available (out of memory)" und bleibt bedienbar, statt auf einen Nullzeiger zu
laufen. Gemessen ueber 34 Environments: -109 040 B RAM gesamt, +7 076 B Flash. Der Schreiber
(`OnRxDone`, LORA-Task) und der Alloziierer (Web-Pfad) laufen auf unterschiedlichen Tasks;
laut Commit ohne Lock sicher, weil der Zeiger genau einmal von NULL auf gueltig wechselt und
nie zurueck, `calloc()` vor Veroeffentlichung des Zeigers nullt und der Schreiber den Zeiger
nur einmal in eine lokale Variable liest. Hardware-Verifikation laut Commit noch offen.

**111. Drei weitere Commits erreichen diese Dateien nur als Nebenwirkung von Typumstellungen, ohne
eigenen Web-GUI-Befund:**

- `f50cc954` (R2-04, `aprsMessage` von `String` auf `char[]`) aendert in
  `web_functions.cpp`s `sub_content_messages()` (Nachrichtenliste) lediglich die
  Aufrufstellen von `.c_str()`/`String`-Methoden auf `mcStartsWith()`/`mcIndexOfStr()`/
  `strcmp()`/`strlen()` -- mechanische Anpassung an den neuen Feldtyp, keine
  Verhaltensaenderung an dieser Stelle.
- `b6416efe` (R3-12, `mheardLat`/`mheardLon` von `double` auf `float`) aendert in
  `web_functions.cpp` nur die `extern`-Redeklaration der beiden Arrays
  (`web_functions.cpp:49-50`); die eigentliche Umstellung inklusive der gemessenen
  Praezisionskosten steht in `mheard_functions.cpp` und ist Teil von K09.
- `09524068` (R2-01, mheard-Textcodec entfernt) ersetzt in `sub_page_mheard()` einen
  einzelnen Aufruf `decodeMHeard(mheardBuffer[iset], mheardLine)` durch
  `mheardLineFromRecord(mheardRecords[iset], mheardLine)` -- eine Anpassung an die neue
  Struct-basierte Speicherung, keine neue Web-Funktionalitaet.

## K15 Safeboot und OTA

**112. Ein abgebrochener Upload kann keine Boot-Partition mehr umschalten** (TM-49,
`2cac942e`). `Update.hasError()` wird nur wahr, wenn aktiv etwas fehlschlaegt;
stirbt die Verbindung vor dem `final`-Frame, laeuft `Update.end()` nie und
nichts setzt einen Fehler -- der Completion-Handler hielt ein PARTIELLES
Image damit fuer gut, setzte `_reboot` und schaltete via
`onOTAEnd(success=true)` die Boot-Partition um. Auf dem 16-MB-T-Deck fing die
Slot-Validierung des Bootloaders das ab; auf einem 4-MB-Single-Slot-Board
haette es auf ein halb geschriebenes App-Image umschalten koennen. Die Gegen-
probe ist jetzt ein eigenes Flag statt eines Patches pro Fluchtweg:
`_ota_image_valid` (`src/safeboot/ElegantOTA.h:143`-Nachbarschaft) steht ab
`/ota/start` auf `false` und wird genau an einer Stelle wahr -- nachdem
`Update.end(true)` (Laenge plus die vom Client uebergebene MD5 aus
`Update.setMD5()`) erfolgreich war und `Update.isFinished()` das bestaetigt,
geloggt als `[SAFEBOOT];ota;verify;result;ok`. Beide Completion-Handler,
async und sync, gaten `postUpdateCallback()`, `_reboot` und den HTTP-200 auf
dieses Flag (`src/safeboot/ElegantOTA.cpp:229-252`, gespiegelt fuer den
Sync-Pfad um Zeile 321-332) und brechen sonst mit `incomplete_upload` ab, der
Client bekommt 400, der Knoten faellt auf Safeboots 180-s-Timer zurueck.
`abortActiveUpdate()` loescht das Flag nur, solange `Update.isRunning()`
noch laeuft, damit ein spaeter eintreffender Disconnect nach einem bereits
erfolgreichen `end(true)` kein bereits verdientes Ergebnis zuruecknimmt.
Bankscharfer Test auf einem 4-MB-Board stand laut Commit noch aus.

**113. Ein abgebrochener Upload strandete den Safeboot vollstaendig -- kein
Fallback, keine neue Session moeglich** (TM-46, `59bb405f`). Vor dieser Zeile
blieb `Update.begin()` nach einem abgebrochenen Transfer offen (jede neue
Session bekam 400), `updateInProgress` wurde nie geloescht und der
180-s-Fallback-Timer auf die App-Partition konnte danach nie mehr feuern --
der Knoten blieb im Safeboot haengen, bis er stromlos gemacht wurde. Neu:
`abortActiveUpdate(reason)` (`stale_session` / `write_failed` /
`client_disconnected` / `stalled`) als zentraler Aufraeumpfad, ein
`onDisconnect`-Hook am Upload und ein 30-s-No-Data-Watchdog in `loop()`
(`OTA_STALL_TIMEOUT_MS`, `last_ota_data_millis`); Abort- und End-Callbacks
loeschen das Flag und re-armen den Fallback. Derselbe Commit portiert daneben
den WLAN-Join des Safeboot auf das Produktionsmuster aus
`udp_functions.cpp` (TM-48) -- inhaltlich getrennt von der OTA-Absturzsicherung,
aber dieselbe Datei.

**114. Der neue Stall-Watchdog brach JEDEN Upload ab, nicht nur haengende**
(TM-46-Nachschliff, `5286c294`, `e5882a0f`). `loop()` liest `millis()` gelegentlich einen Moment,
bevor der `async_tcp`-Task ein neueres `last_ota_data_millis` schreibt; die
vorzeichenlose Differenz lief dabei auf ~2^32 auf, und der 30-s-Watchdog
brach einen gesunden Upload sofort ab (Bankbeweis: `abort;reason;stalled` im
selben Millisekunden-Stempel wie der erste Progress-Callback). Fix in
`src/safeboot/main.cpp:349`: vorzeichenbehaftete Delta-Arithmetik
(`(long)(millis() - last_ota_data_millis) > (long)OTA_STALL_TIMEOUT_MS`,
gleiche Behandlung fuer den Fallback-Timer in Zeile 359), dazu `volatile`
fuer `updateInProgress`, `fallback_armed_at` und `last_ota_data_millis`, weil
alle drei von zwei Tasks beruehrt werden. Zusaetzlich schuetzt eine
Session-Generation (`_updateGeneration`, `src/safeboot/ElegantOTA.h:143`,
verwendet in `src/safeboot/ElegantOTA.cpp:78,272-278`) davor, dass der
verspaetete Disconnect einer alten Verbindung eine inzwischen neu gestartete
Session abbricht -- der Disconnect-Hook faengt seine Generation per
`[this, gen]` statt per `[&]` ein und verwirft sich selbst mit
`[SAFEBOOT];ota;disconnect_ignored;gen;<alt>/<neu>`, wenn sie nicht mehr die
aktive ist. Bankbeweis auf DK5EN-14: Kill 5 s in den Upload -> Node in
Sekunden zurueck in der App, sofortiger Retry laeuft ohne `--force` komplett
durch (2x); ein Original-GitHub-Release-Asset per OTA durch den reparierten
Safeboot geflasht (73 s).

**115. `ota_html[]` ist jetzt `const`, 24 kB weniger RAM auf beiden Safeboot-Envs**
(`2991034e`, W2 D5-01). `src/safeboot/ota.h:1` deklarierte das eingebettete
OTA-Formular als `unsigned char ota_html[]` ohne `const` -- das gesamte Array
wurde beim Boot ins RAM kopiert. `const` verschiebt es von `.data` nach
`.rodata`; Flash unveraendert, 24 320 B RAM zurueck auf `esp32-safeboot` wie
auf `esp32-S3-safeboot`. Verifiziert ueber deterministische Rebuilds: die
mitgefuehrten Safeboot-Images wurden neu erzeugt und per SHA-256 als
bitgleich zwischen zwei Builds bestaetigt, sodass der Diff die Aenderung
selbst zeigt und kein Build-Rauschen.

**116. Was ein unterbrochenes OTA fuer den Knoten bedeutet**: vor diesen vier
Commits strandete ein abgebrochener Transfer den Safeboot (offenes
`Update.begin()`, nie geloeschtes
`updateInProgress`, ein Fallback-Timer, der nie mehr feuerte) und konnte, im
Extremfall eines Disconnects exakt nach dem letzten Chunk, unbemerkt auf ein
halbes Image umschalten. Mit beiden Fixes: ein haengender oder abgebrochener
Upload raeumt sich selbst auf (`abortActiveUpdate()`, 30-s-Watchdog,
Session-Generation) und die Boot-Partition schaltet ausschliesslich um, wenn
`Update.isFinished()` das geschriebene Image bestaetigt hat -- alles andere
bleibt auf dem alten App-Image respektive faellt auf Safeboots eigenen
180-s-Timer zurueck. Da jedes Board nur einen App-Slot hat (kein Fallback-
Slot), war und bleibt ein hart abgebrochener Flash-Vorgang selbst der
riskante Fall; die beiden Fixes verhindern die zwei konkret nachgewiesenen
Wege dorthin, sind aber laut den Commits selbst nicht auf echter Hardware im
worst-case (Stromausfall mitten im Schreiben) durchgespielt.

### Kampagnen-Commit (DRY-Kampagne)

- `2991034e` -- W2: ~28 kB of RAM, and two audit rows that cannot be done as written

### Weitere Commits (ausserhalb der DRY-Kampagne, hier mitgeliefert)

- `2cac942e` -- fix(safeboot): TM-49 -- fail-closed OTA completion gate
- `5286c294` -- TM-46 Nachschliff: Cross-Task-Race im Stall-Watchdog, webflash Safeboot-Resume; TM-49 gefiled
- `e5882a0f` -- TM-46: Session-Generation im Safeboot-Upload (Nachzuegler zum Race-Fix-Commit)
- `59bb405f` -- TM-46/47/48: Safeboot-Session-Cleanup, WLAN-Join-Portierung, webflash-Hardware-Check
- `3861f459` -- refactor(config): "Node nicht konfiguriert" einmal definieren (ALT-34)

## K17 Instrumentierung

Vorbemerkung zur Einordnung: diese Mess-Instrumentierung ist keine Fork-Erfindung.
`upstream/dev` traegt sie bereits -- `src/instrument.h` ist dort byte-identisch mit unserer
Fassung, `src/instrument.cpp` unterscheidet sich um achtzehn Zeilen, und sieben Dateien binden
sie dort schon ein. Unser gesamter Anteil sind diese achtzehn Zeilen plus vier weitere
Einbinder (`src/extudp_functions.cpp`, `src/nrf52/nrf_eth.cpp`, `src/esp32/gateway_service_esp32.cpp`,
`src/nrf52/gateway_service_nrf52.cpp`). Das Kapitel gehoert damit in jede Auslieferung dieses
Branches.

Die Mess-Instrumentierung in `src/instrument.cpp`/`.h` steht unter `#if INSTRUMENT_ENABLED`
und ist standardmaessig AUS: `src/instrument.h` definiert `INSTRUMENT_ENABLED` auf `0`, sofern
es nicht per Build-Flag `-D INSTRUMENT_ENABLED=1` explizit gesetzt wird, und selbst dann bleibt
sie auf ESP32 und nRF52 beschraenkt (auf den native/Host-Umgebungen ohne Arduino-Kern wird sie
zwangsweise wieder auf 0 geschaltet). Kein `platformio.ini` in diesem Baum setzt den Schalter
auf 1 -- keine der 34 ausgelieferten Board-Umgebungen baut eine Mess-Firmware. Mit aktivem
Schalter gehen `[INSTR-LOOP]`-Zeilen unconditioniert (nicht ueber `--debug` gegated) auf die
serielle Konsole, weshalb der Code als eigene Opt-in-Bench-Firmware behandelt wird und nicht als
Feldfunktion. Zur Abgrenzung: die Feld-Diagnoseschalter dieses Fork (TX/RX-Mitschnittring,
Spektrum-Parameterkommandos) sind ein getrenntes, standardmaessig aktives Konzept und werden
nicht in diesem Kapitel behandelt.

**117. Stille Kappung der Sektions-Slots konnte neu hinzugefuegte Messpunkte unsichtbar machen**
(`4f4b02f1`). `INSTR_SECTION_SLOTS` (`src/instrument.cpp:61`, vorher auf 16 gesetzt) hat
`instrument_note_section()` bislang bei jedem neuen, noch unbelegten Sektionsnamen einen freien
Slot zuweisen lassen -- ist die Tabelle voll, kehrt die Funktion still zurueck: kein Fehler,
kein Absturz, die Sektion erscheint im Bericht schlicht nicht. Der Baum traegt inzwischen 42
verschiedene `INSTR_SECTION()`-Namen, Slots werden pro Boot nach Erstankunft vergeben und
bleiben fuer die Laufzeit des Boots gebunden. Beim Vermessen von ETH-03 waren auf dem
RAK4631-Gateway-Pfad mindestens neun Sektionen vor dem neu instrumentierten EXTUDP-Block
bereits belegt, so dass die vier neu hinzugefuegten Messpunkte (`extudp_parse`, `extudp_guard`,
`extudp_read`, `webserver_loop`) mit hoher Wahrscheinlichkeit genau die abgeschnittenen gewesen
waeren -- ein Messbericht, der ausgerechnet die Zeilen ausgelassen haette, um die es ging, und
sich wie "die Sektion lief nie" gelesen haette. Behoben durch Anhebung auf 48 Slots. Kostet
ausgelieferte Firmware nichts, da die gesamte Datei innerhalb von `#if INSTRUMENT_ENABLED`
liegt; auf einem Messbuild sind es 32 zusaetzliche `SectionStat`-Eintraege a 20 Byte, ~640 B.
