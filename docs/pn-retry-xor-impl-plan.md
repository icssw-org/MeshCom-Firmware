# PN-Wiederholung Variante a) (XOR-Form) — Implementierungsplan

Branch `dk5en-xor`, Basis `upstream/dev` @ `cf215b5d`. Konzept: `docs/pn-zustellung-dedup.md`.

## Wellenstatus

| Welle | Inhalt                                               | Status                                                                             |
| ----- | ---------------------------------------------------- | ---------------------------------------------------------------------------------- |
| W0    | Worktree, Baseline-Sweep (eigener Worktree)          | erledigt: 33/35 grün, `t5_epaper` + `esp32-external-radio` schon am Basisstand rot |
| W1    | Firmware: Helfer + Tests, lora_functions, Server-ACK | erledigt: 14/14 Host-Tests, 4 Referenz-Envs grün, Marker im ELF                    |
| W2    | /fable-review auf W1-Diff, Nacharbeit                | läuft: Review fertig (5 Finder, 1 Verifizierer), Nacharbeit C2/C3/C5/C6/C7/C10     |
| W3    | Docs: Server (C), App, mcapp, Konzeptpapier          | erledigt: 3 Docs + Konzeptpapier in docs/                                          |
| W4    | Voll-Compile aller Envs, Host-Tests, Prettier        | offen                                                                              |
| W5    | Commits, `git push upstream dk5en-xor`               | offen                                                                              |

## Entscheidungen (Operator, 27.09.2026)

- XOR-Form: Wiederholung k (1..3) = Original-Bits 30–31 (aus `_GW_ID` Bit 20–21) XOR k; nie
  schrittweise auf die vorige Kopie. Erstsendung unverändert.
- Nur PN (Text mit `{NNN`-Suffix). Gruppen und `*` unverändert.
- V komplett: Echo stoppt PN-Wiederholung nicht, setzt den 40-s-Takt zurück; `:ackNNN` über LoRa oder
  Server stoppt.
- Empfänger: Wiederholung einer bekannten PN (eine der drei anderen Bitvarianten steht im Dedup-Ring)
  wird weitergeleitet und quittiert, aber nicht erneut angezeigt, ans Telefon gegeben oder hochgeladen.
- `checkOwnTx` bleibt unverändert (Gateways führen dort fremde msg_ids). Status "gehört" für
  Wiederholungen entfällt (Minimalumfang).
- Push nach icssw-org als `dk5en-xor`, kein PR, kein Merge.

## W1 Ownership

| Agent | Exklusive Dateien                                                         |
| ----- | ------------------------------------------------------------------------- |
| A     | `src/pn_retry.h` (neu), `test/test_pn_retry/` (neu), `platformio.ini`     |
| B     | `src/lora_functions.cpp`                                                  |
| C     | `src/udp_functions.cpp`, `src/nrf52/nrf_eth.cpp`                          |
| Orch. | `src/lora_functions.h` (Deklaration `findAndStopRingSlot`, vor der Welle) |

Gemeinsame Ressourcen: kein Writer startet `pio` (Baseline-Sweep läuft; ein pio-Prozess zur Zeit).
Builds und Host-Tests laufen am Gate.

## Nicht im Umfang (dokumentiert)

- Anzeige/ACK vor dem Dedup-Tor beim Gateway als Empfänger über den Server-Pfad (bestehend).
- Airtime-basierte Wartezeit, App-Status "unbestätigt".

## Review W2 (fable-review) — Ergebnis

Behoben: C2 stille Variantenprüfung (`checkOwnRx`), C3 Tests (Golden-Frame, Grenzen) und
`static_assert(MAX_RETRANSMIT <= 3)`, C5 `static`-Puffer auf nRF52, C6 Ring-Snapshot unter Sperre
(nRF52), C7 nur persönliche Ziele gelten als PN, C10 "gehört" auch für Wiederholungs-Echos.

Widerlegt: C1 (30-Bit-Vergleich in `findAndStopRingSlot` trifft nur eigene wartende Slots; zwei
eigene Slots mit gleichem Zähler gibt es nicht).

Bekannte Grenzen (dokumentiert, nicht geändert):

- C4: Wiederholungserkennung beim Empfänger über 30 Bit; eine echte neue PN eines anderen Knotens
  mit gleichen unteren 20 Knotenbits und gleichem Zähler im Dedup-Fenster würde nicht angezeigt
  (aber weitergeleitet und quittiert). Grob 2e-5 pro PN; heute gleiche Klasse mit 2^-22 und
  härterer Folge.
- C8: Auf dem Server-Pfad (UDP/ETH) laufen Anzeige, BLE und ACK wie bisher vor dem Dedup-Tor;
  Wiederholungen mit neuer msg_id erreichen diesen Pfad nur, wenn der Server sie weiterreicht
  (Server-Doku, Voraussetzung).
- C9: Jedes Echo derselben Kopie startet die 40-s-Wartezeit neu; begrenzt durch die Zahl der
  Relais, Abbruch nach `MAX_RETRANSMIT` bleibt. Eine unquittierte PN kostet bis zu 4 Flutungen.
- Server-`:ackNNN` stoppt jetzt die LoRa-Wiederholung (P4) — gehört zu V, im PR ausdrücklich nennen.
