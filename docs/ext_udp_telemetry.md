# Erweiterung der extern-UDP-Schnittstelle: externe Sensorwerte im Positions-Beacon

Erweitert die bestehende JSON-UDP-Schnittstelle (Port `1799`, `src/extudp_functions.cpp`)
um einen neuen eingehenden Nachrichtentyp `"tele"`, mit dem ein externer Client
(z.B. MeshComWebDesk) Sensorwerte für einen Node **ohne eigene physische Sensorik**
bereitstellen kann. Die Werte werden direkt in die Sensor-Variablen des Nodes
geschrieben (`meshcom_settings.node_temp`, `node_hum`, `node_press`, ...) und
fließen dadurch automatisch in den nächsten Positions-Beacon ein — genau wie bei
einem Node mit echter Sensorik (z.B. `/T=23.9/H=34.3/Q=1013.6` im APRS-Kommentar).
Für jede empfangende Station (und jeden Monitor/jedes Dashboard) sind sie dadurch
nicht von echter Sensorik unterscheidbar.

Umgesetzt in `src/extudp_functions.cpp`, Funktion `handleExternTelemetry()`
(aufgerufen aus `getExtern()`).

> **Hinweis zur Historie:** Eine frühere Version dieser Erweiterung schickte die
> Werte als separates APRS-Telemetrie-Paket (`T#...`, Zieladresse `100001`). Das
> wurde verworfen, weil gängige Monitore/Dashboards (nachweislich auch die
> MeshCom-Firmware selbst über `sendExtern()`) ausschließlich die im
> Positions-Beacon eingebetteten Sensorfelder auswerten, niemals eigenständige
> `T#`-Pakete. Diese Version schreibt deshalb direkt in die Sensor-Variablen.

---

## 1. Nachrichtenformat (Client → Node)

UDP-Paket an den Node, Port `1799`, JSON-Payload — alle Felder optional, nur
angegebene Felder werden übernommen:

```json
{"type":"tele","temp":23.3,"hum":60,"press":1018.5,"temp2":0,"qnh":1018.5,"gasres":0,"co2":0}
```

| Feld     | Ziel-Variable                    | APRS-Kommentarfeld in `PositionToAPRS()` |
|----------|----------------------------------|-------------------------------------------|
| `temp`   | `meshcom_settings.node_temp`     | `/T=`                                      |
| `hum`    | `meshcom_settings.node_hum`      | `/H=`                                      |
| `press`  | `meshcom_settings.node_press`    | `/P=`                                      |
| `temp2`  | `meshcom_settings.node_temp2`    | `/O=`                                      |
| `qnh`    | `meshcom_settings.node_press_asl`| `/Q=`                                      |
| `gasres` | `meshcom_settings.node_gas_res`  | `/G=`                                      |
| `co2`    | `meshcom_settings.node_co2`      | `/C=`                                      |

Es gibt kein Kanalname/Einheit-Feld mehr — die Bedeutung jedes Werts ist durch
das feste APRS-Kommentarfeld vorgegeben (identisch zu echter Sensorik).

### Beispiel

```json
{"type":"tele","temp":23.3,"hum":60,"press":1018.5}
```

Setzt `node_temp=23.3`, `node_hum=60`, `node_press=1018.5`. `temp2`/`qnh`/`gasres`/`co2`
bleiben unverändert (kein Wert im JSON = keine Änderung an dieser Variable).

---

## 2. Was der Node danach tut

1. **Sofortiger Positions-Beacon:** Der Node löst sofort einen Positions-Beacon aus
   (`sendPosition(0x9999, ...)`, derselbe Mechanismus wie der `--sendpos`-Konsolenbefehl),
   statt auf den nächsten planmäßigen Beacon zu warten (Default alle 30 Minuten,
   `POSINFO_INTERVAL`).
2. **Format am Funk:** Die Werte erscheinen als Teil des normalen `!`-Positionspakets,
   z.B. `DH1FR-2>*!5051.09N/00906.45E#.../B=100/A=000827/P=1018.5/H=60.0/T=23.3/N5/R=...`
   — kein separates Telemetrie-Paket, keine `PARM.`/`UNIT.`-Kopfzeilen nötig.
3. **Dauerhaft, nicht einmalig:** Anders als die frühere `"T:"`-Version werden die
   Werte **nicht** nach dem Versand zurückgesetzt — sie bleiben gesetzt und fließen
   in **jeden** weiteren Positions-Beacon ein (auch die planmäßigen alle 30 Minuten),
   bis sie durch eine neue `"tele"`-Nachricht überschrieben werden oder der Node
   neu startet.

---

## 3. Einschränkungen (bitte in MeshComWebDesk beachten)

- **Schützt echte Sensorhardware.** Erkennt der Node physisch verbaute Sensorik
  (`bmx_found`/`bmp3_found`/`aht20_found`/`sht21_found` — BME280/BMP3xx/AHT20/SHT21),
  wird die externe Nachricht **komplett ignoriert** und im Log vermerkt
  (`"tele ignored: real sensor hardware detected on this node"`). So wird
  verhindert, dass eine Injektion mit den Werten einer echten, am Node verbauten
  Sensorik kollidiert — auf einem Node ganz ohne Sensorik (der Standardfall für
  diese Erweiterung) greift diese Sperre nicht.
- **Keine Quittierung über UDP.** Das Protokoll ist "fire and forget" — es gibt keine
  Antwort/ACK auf Port 1799. Erfolg oder Ablehnung sind nur im seriellen Log des
  Nodes sichtbar (`[EXT] tele accepted: ...` bzw. `[EXT] tele ignored: ...` /
  `[EXT] tele missing recognized fields ...`).
- **Keine Absenderprüfung.** Wie die bestehende `dst`/`msg`-Nachricht ist auch dieser
  Kanal für jedes Gerät im selben WLAN erreichbar (kein IP-Allowlist). Kein neues
  Sicherheitsrisiko gegenüber dem Status quo, aber auch keine zusätzliche Absicherung.
- **Kein freies Kanalnamen-Feld mehr.** Werte, die keinem der 7 festen Felder
  entsprechen (z.B. Regenmenge), können mit dieser Schnittstelle aktuell nicht
  übertragen werden — es gibt in `PositionToAPRS()` kein passendes Kommentarfeld
  dafür.

---

## 4. Fehlerfälle (Log-Meldungen am Node)

| Log-Meldung                                                              | Ursache                                                        |
|---------------------------------------------------------------------------|-----------------------------------------------------------------|
| `[EXT] tele ignored: real sensor hardware detected on this node`          | Node hat echte Sensorik erkannt — externe Werte werden nicht angenommen |
| `[EXT] tele missing recognized fields (temp/hum/press/temp2/qnh/gasres/co2)` | Keines der bekannten Felder im JSON vorhanden                   |

---

## 5. Unverändertes Verhalten

- Die bestehende `{"type":"msg","dst":"...","msg":"..."}`-Nachricht funktioniert
  unverändert weiter (eigener Codepfad, unberührt).
- Die ausgehende `sendExtern()`-Telemetrie (`"type":"tele"` vom Node zum Client,
  bei empfangenen Positionspaketen) ist ein separater, unveränderter Mechanismus.
  Nach einer erfolgreichen Injektion wird sie automatisch die neuen Werte
  widerspiegeln (da sie ja jetzt die echten `node_temp`/`node_hum`/... Variablen
  liest).
- Nodes ohne jemals gesendete `"tele"`-Nachricht verhalten sich exakt wie vorher —
  der neue Code wird nur aktiv, wenn tatsächlich eine passende UDP-Nachricht eintrifft.
- `--gateway`/`--track`-Einstellungen sind für diesen Mechanismus **nicht** relevant
  (im Gegensatz zur früheren `"T:"`-Version) — der Positions-Beacon-Codepfad
  (`sendPosition()`) legt Pakete unabhängig davon immer auch in den lokalen
  LoRa-Sendepuffer.
