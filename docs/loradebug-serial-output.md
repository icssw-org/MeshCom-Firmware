# LoRa Debug Ausgaben (`--loradebug on`)

Aktivierung via seriellem Kommando oder BLE:
```
--loradebug on
--loradebug off
```

`--loradebug on` setzt intern **drei Flags gleichzeitig**:
- `bLORADEBUG = true` — alle `[MC-DBG]`/`[MC-SM]` Zeilen
- `bDisplayInfo = true` — MH-LoRa / RX-LoRa2 / TX-LoRa Pakete werden ausgegeben
- `bDisplayRetx = true` — Retransmission-Statuszeilen werden ausgegeben

Die Einstellung wird im Flash gespeichert und überlebt einen Reboot.

---

## State Machine `[MC-SM]`

Jeder Zustandswechsel des LoRa-Automaten:

```
[MC-SM] RX_LISTEN -> RX_PROCESS rc=0   Paket wird empfangen
[MC-SM] RX_PROCESS -> RX_LISTEN rc=0   Verarbeitung fertig, wieder horchen
[MC-SM] IDLE -> TX_PREPARE rc=0        TX-Queue hat Daten, Kanal wird geprüft
[MC-SM] TX_ACTIVE -> TX_DONE rc=0      Paket erfolgreich gesendet
[MC-SM] TX_DONE -> RX_LISTEN rc=0      Zurück zum Empfangen
```

`rc≠0` = Fehler beim Übergang (wird vom `serial_monitor.py` als Alert gewertet).

---

## Empfangspfad

### Chip-Werte nach Empfang
```
[LoRa]...Received packet: RSSI:  -110 dBm / SNR:  -5 dB / Frequency error:  234 Hz
```
| Feld | Bedeutung |
|---|---|
| RSSI | Empfangspegel in dBm (typisch -80 bis -130) |
| SNR | Signal-Rausch-Abstand in dB |
| Frequency error | Frequenzabweichung in Hz: <1000 Hz = Kollision, >3000 Hz = falsches Gerät/Frequenz |

### Puffer & Timing
```
[MC-DBG] RX_BUF_SWITCH buf=1          Doppelpuffer-Wechsel (RAK4630)
[MC-DBG] RX_BUF_OVERWRITE buf=1       WARNUNG: Puffer noch in Verwendung!
[MC-DBG] RX_BUF_RELEASE buf=1         Puffer nach Verarbeitung freigegeben
[MC-DBG] RX_RESTART_EARLY src=OnRxDone  Radio sofort wieder auf RX gesetzt
[MC-DBG] CAD_ABORT_BY_RX              CAD-Scan durch eingehendes Paket abgebrochen
[MC-DBG] ONRXDONE_TIME ms=12          Dauer der OnRxDone-Verarbeitung in ms
[MC-WARN] ONRXDONE_SLOW ms=450        Zu langsam! (Schwellwert überschritten)
OnRxDone                               Ende der OnRxDone-Funktion
OnRxTimeout                            Kein Paket, Timeout
OnRxError                              CRC-/Header-Fehler
[MC-DBG] RX_ERROR rssi=-125 snr=-20 ts=12345   CRC-Fehler mit Details (RAK4630)
```

---

## Deduplizierung

```
[MC-DBG] RX_DEDUP_NEW  msg_id=1A2B3C4D           Neues Paket, noch nicht gesehen
[MC-DBG] RX_DEDUP_DUP  msg_id=1A2B3C4D slot=3    Duplikat, wird verworfen
```

---

## TX-Pfad / CSMA

### CSMA-Backoff-Timer
```
[MC-DBG] RX_TIMEOUT_FIRE ts=123456 wait=4675 delta=4680
```
| Feld | Bedeutung |
|---|---|
| `wait` | Berechnete CSMA-Wartezeit in ms (adaptiv nach Kanalauslastung) |
| `delta` | Tatsächlich gewartet in ms |

Typische `wait`-Werte je nach `cad_attempt`:
- Versuch 0: ~4675 ms
- Versuch 1: ~3087 ms
- Versuch 2: ~2087 ms
- Versuch ≥3: ~35 ms (Notfall-TX)

### TX-Gate und CAD
```
[MC-DBG] RX_TIMEOUT_DEFERRED src=receiveFlag    Paket kam kurz vor TX → aufgeschoben
[MC-DBG] TX_GATE_ENTER qlen=2 cad_attempt=0     TX-Gate betreten; 2 Pakete in Queue
[CHECK] radio.scanChannel() / 1                  Erster CAD-Scan startet
[MC-DBG] CAD_SCAN result=0                       0=frei, -702=belegt (LORA_DETECTED)
[MC-DBG] CAD_BUSY_1 attempt=1, double-check..   Kanal belegt → zweiter Scan zur Bestätigung
[MC-DBG] CAD_SCAN result=0                       Zweiter Scan frei
[MC-DBG] CAD_FALSE_POSITIVE                      War ein Fehlalarm
[MC-DBG] CAD_FREE attempt=1                      Kanal frei, TX wird gestartet
[MC-DBG] IRQ_POLL hdr/pre=0x0200 -> TX_ABORT     Header/Präambel im IRQ-Register → TX abgebrochen
[MC-DBG] RADIO_TX len=47                         Paket wird tatsächlich gesendet (47 Byte)
```

---

## Ring-Buffer (TX-Queue)

Alle 30 Sekunden:
```
[MC-DBG] RING_STATUS queued=1 pending=2 retrying=0 done=3 iW=5 iR=4 dedup=7/16
```
| Feld | Bedeutung |
|---|---|
| `queued` | Neu eingereiht, noch nicht gesendet |
| `pending` | Gesendet, ACK ausstehend |
| `retrying` | Wiederholungsversuch läuft |
| `done` | Fertig (ACK erhalten oder kein Retry nötig) |
| `dedup=7/16` | 7 von 16 Dedup-Slots belegt |

### Einzelereignisse
```
[MC-DBG] RING_WRITE slot=2 type=3A status=01 ...    Slot beschrieben
[MC-DBG] RING_PRIO slot=2 prio=1                    Priorität zugewiesen
[MC-DBG] RING_TX_READ slot=2 prio=1 type=3A ...     Slot wird zum Senden ausgewählt
[MC-DBG] RING_DROP_PRIO slot=3 prio=0 ...           Paket mit niedrigerer Prio verdrängt
[MC-DBG] RETRANSMIT retry=1 after_sec=30 msg_id=AABBCCDD   Erneuter Sendeversuch
[MC-DBG] RETRANSMIT_GIVEUP retries=3 msg_id=AABBCCDD       Aufgegeben nach 3 Versuchen
```

---

## ACK-System

```
[MC-DBG] ACK_RECEIVED retid=2 msg_id=AABBCCDD   Eigenes Paket wurde bestätigt
[MC-DBG] ACK_SKIP_READY slot=1 msg_id=...        Slot im READY-Zustand übersprungen
[MC-DBG] ACK_FAST_QUEUED msg_ref=... ack_qlen=1  ACK für fremdes Paket eingereiht
[MC-DBG] ACK_FAST_TX len=12 ack_qlen=0           ACK gesendet
[MC-DBG] ACK_RX_CANCEL msg_id=... cancelled=1    Fremdes ACK gehört → eigenes storniert
[MC-DBG] ACK_FWD_DEDUP msg_id=...                Doppeltes ACK-Forward verhindert
[MC-DBG] GW_ACK_DEDUP msg_id=...                 Doppeltes Gateway-ACK verhindert
```

---

## Relay

```
[MC-DBG] RELAY_LOOP_BLOCKED own_call_in_path   Weiterleitung verhindert (eigenes Call im Pfad)
[MC-DBG] RELAY_QUEUED msg_id=... type=3A len=47  Paket zur Weiterleitung eingereiht
```

---

## Kanal-Statistik (periodisch, immer aktiv)

```
[MC-DBG] CHANNEL_UTIL rx=1200ms tx=300ms util=15%
```
Wird alle 10 Sekunden ausgegeben — unabhängig von `bLORADEBUG`.

```
[MC-HWM] uptime=3600s queue_hwm=3/8 csma_hwm=4 trickle=120000ms
```
Alle 30 Minuten: Maximale Queue-Tiefe, maximale CAD-Versuche, Trickle-Intervall.

---

## APRS-Protokoll-Fehler

Nur sichtbar wenn `bLORADEBUG=true`:
```
APRS decode - Packet discarded, wrong APRS-protocol - size <8> to short!
APRS decode - Source-CallSign Error [DK?ABC]
APRS decode - Destination-Last-CallSign Error [...]
APRS decode - Packet discarded, wrong APRS-protocol - bSourceEndOk (>) missing!
APRS decode - Packet discarded, wrong APRS-protocol - PayloadEnd (0x00) missing!
APRS decode - Packet (47) discarded, wrong FCS <0012>:<0013> wrong! <...>
```
Nach jedem Fehler folgt bei Paketen < 255 Byte ein Hex-Dump (`printAsciiBuffer`).

---

## Pakete auf der seriellen Konsole (bDisplayInfo / bDisplayCont)

Diese Zeilen erscheinen **zusätzlich** wenn `bDisplayInfo` oder `bDisplayCont` gesetzt ist:

```
MH-LoRa: DK7CH-1,DL7OSX-1>*! H01 HW:4 FW:47 ...   Alle gehörten Pakete
RX-LoRa2: DK7CH-1>*! H01 ...                         Akzeptierte Pakete (nach Dedup)
TX-LoRa: OE1ABC-7>*! H01 ...                          Gesendete Pakete
RX-LoRa-All: ...                                       Rohempfang vor Filterung
```

---

## Analyse-Tools

| Tool | Beschreibung |
|---|---|
| `tools/serial_monitor.py --port COMx` | Live-Monitoring mit Alerts |
| `tools/serial_monitor.py --replay <logfile>` | Offline-Analyse gespeicherter Logs |
| `tools/loganalyse.sh <logfile>` | Batch-Auswertung in 15 Abschnitten (Linux/macOS/WSL) |
