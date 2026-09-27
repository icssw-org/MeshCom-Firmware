# PN-Wiederholung Variante a) (XOR-Form) — Umsetzung

Branch `dk5en-xor`, Basis `upstream/dev` @ `cf215b5d`. Konzept und Begründung:
`docs/pn-zustellung-dedup.md`. Begleitende Änderungen außerhalb der Firmware:
`docs/pn-retry-server.md`, `docs/pn-retry-app.md`, `docs/pn-retry-mcapp.md`.

## Was die Firmware jetzt tut

**Was als PN gilt**: eine Textmeldung, deren Text auf `{NNN` endet (`{` plus 1–5 Ziffern) und die an
ein persönliches Ziel geht — nicht `*`, keine Gruppennummer (1–6 Ziffern), nicht WLNK-1 oder
APRS2SOTA. Gruppen, `*` und alle anderen Meldungen verhalten sich wie bisher.

**Absender**

- Die Erstsendung ist unverändert.
- Wiederholung k (1–3) einer eigenen PN bekommt in Bit 10–11 der msg_id die Original-Bits (Bit 0–1
  von `_GW_ID`) XOR k — immer vom Original aus gerechnet, nie von der vorigen Kopie. Die restlichen
  30 Bit, Text und `{NNN` bleiben gleich; die FCS wird neu berechnet.
- Die Wiederholungskopie wird unter der Ring-Sperre aus dem Ring gelesen (nRF52) und in einem lokalen
  Puffer umgeschrieben (auf nRF52 `static`, wegen des 4-KB-Loop-Tasks). Ihre msg_id kommt in den eigenen
  Dedup-Ring, damit das eigene Echo nicht als fremde Meldung gilt.
- Ein Echo der eigenen PN bricht die Wiederholung nicht mehr ab, sondern startet die 40-s-Wartezeit neu.
- Hat das Ziel schon geackt (Original-id in `own_msg_id` auf 0x02), gibt das Echo den Slot frei,
  und eine fällige Wiederholung wird verworfen statt gesendet. Das deckt ein `:ackNNN` ab, das
  eintrifft, während eine Kopie noch READY wartet: `findAndStopRingSlot` lässt READY-Slots bewusst
  in Ruhe, weil `doTX()` sie gerade übernehmen kann.
- "Eigene PN" heißt: eigene Knotenkennung in der msg_id **und** eigenes Rufzeichen als Quelle
  (`pnFrameIsOwnPn`). Eine PN, die `sendMessage()` für einen KISS-Client sendet, trägt zwar unsere
  msg_id, aber das Rufzeichen des Clients; ihr `:ackNNN` geht an den Client und stoppt unseren Slot nie.
  Sie bleibt deshalb beim alten Verhalten: byte-gleiche Wiederholung, Abbruch beim ersten Echo. Der
  Client wiederholt selbst.
- Ein `:ackNNN` stoppt die Wiederholung — über LoRa und jetzt auch über den Server-Pfad (ESP32 und
  nRF52). `findAndStopRingSlot` vergleicht dafür den 30-Bit-Kern, ist exportiert und nimmt auf nRF52
  die Ring-Sperre, weil der Server-Pfad dort im Loop-Task läuft.
- "Gehört" (Status 0x00 ans Telefon) wird auch für das Echo einer Wiederholung gemeldet, immer mit der
  Original-msg_id.
- Höchstens 4 Aussendungen (`MAX_RETRANSMIT` 3, per `static_assert` auf ≤ 3 festgehalten, weil k = 4
  wieder das Original ergäbe).

**Empfänger**

- Relais-Entscheidung wie bisher auf der vollen msg_id: alte und neue Firmware leiten Wiederholungen
  weiter.
- Steht eine der drei anderen Bitvarianten schon im Dedup-Ring (stille Abfrage `checkOwnRx`), ist die
  Meldung die Wiederholung einer bekannten PN: sie wird weitergeleitet und quittiert, aber nicht noch
  einmal angezeigt, ans Telefon gegeben, hochgeladen, an Extern-UDP oder KISS gegeben.
- `checkOwnTx` ist unverändert; genau ein `is_new_packet()` je Frame wie bisher.

## Code

| Datei                       | Inhalt                                                                |
| --------------------------- | --------------------------------------------------------------------- |
| `src/pn_retry.h` (neu)      | Reine Hilfsfunktionen: Retry-msg_id, 30-Bit-Kern, PN-Erkennung, FCS   |
| `src/lora_functions.cpp`    | Wiederholung, Echo-Neustart, Empfänger-Erkennung, "gehört", ACK-Stopp |
| `src/lora_functions.h`      | Deklaration `findAndStopRingSlot`                                     |
| `src/udp_functions.cpp`     | Server-`:ackNNN` stoppt die Wiederholung (ESP32)                      |
| `src/nrf52/nrf_eth.cpp`     | Server-`:ackNNN` stoppt die Wiederholung (nRF52/Ethernet)             |
| `test/test_pn_retry/` (neu) | 24 Host-Tests, darunter ein Golden-Frame mit von Hand gerechneter FCS |
| `platformio.ini`            | Optionale Testumgebung `native_pnretry` (nicht in `default_envs`)     |

## Prüfung

- `pio test -e native_pnretry`: 24/24.
- Voll-Compile aller 35 Umgebungen nach sauberem Löschen: 33 grün, identisch zum unveränderten
  Basisstand; `t5_epaper` und `esp32-external-radio` sind schon an `cf215b5d` rot. Keine neuen Warnungen.
  Die neuen Log-Marker (`PNRETRY`, `PNREPEAT`, `PN echo`, `server ACK for retid`) stehen in allen 31
  Firmware-ELFs.
- Code-Review mit gegnerischer Verifikation und unabhängigem Advisor vor dem Commit.
- **Noch nicht auf Hardware getestet.** Offen: Kette A → R1 → R2 → B mit ausgefallenem letztem Sprung,
  gemischt mit 4.35p-Knoten als Relais und Empfänger, ein Gateway mit Server-Anbindung.

## Bekannte Grenzen

- **Server zuerst**: Wiederholungen tragen neue msg_ids. Bis der Server auf den 30-Bit-Kern
  dedupliziert und die Weiterleitung an Gateways regelt, zeigt er Wiederholungen mehrfach und reicht
  sie womöglich an alle Gateways weiter (`docs/pn-retry-server.md`).
- **Server-Pfad im Knoten**: Auf dem UDP/ETH-Pfad laufen Anzeige, BLE und ACK wie bisher vor dem
  Dedup-Tor; die Empfänger-Erkennung greift nur auf dem LoRa-Pfad.
- **Upload**: Ein Gateway, das das Original gehört hat, lädt die Wiederholung nicht erneut hoch. Ein nur
  über den Server erreichbares Ziel bekommt eine Wiederholung nur über ein anderes Gateway.
- **Fehlerkennung**: Eine echte neue PN eines anderen Knotens mit gleichen unteren 20 Knotenbits und
  gleichem Zähler im Dedup-Fenster würde nicht angezeigt (aber weitergeleitet und quittiert). Grob 2e-5
  pro PN; der heutige Dedup hat dieselbe Klasse auf 32 Bit mit härterer Folge.
- **Airtime**: Jedes Echo derselben Kopie startet die Wartezeit neu (begrenzt durch die Zahl der
  Relais); eine unquittierte PN kostet bis zu 4 Flutungen, jede angekommene Kopie eine Quittung.
- **Spätes ACK**: Trifft ein `:ack` ein, während eine Wiederholungskopie noch auf den Versand wartet
  (READY), wird es wie bisher übersprungen; die Kopie geht noch einmal raus.
- **Gruppenerkennung**: `pnDestIsPersonal` behandelt jede 1–6-stellige Ziffernfolge als Gruppe;
  `CheckGroup` kennt 1–99999 und 100001. Abweichung nur bei Zielen wie 000000 oder 999999, die kein
  Rufzeichen sein können.
- **Alte Empfänger** zeigen jede Wiederholung als eigene Nachricht (bis zu 4) und quittieren jede.
