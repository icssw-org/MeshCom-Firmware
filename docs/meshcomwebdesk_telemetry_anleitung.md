# Anleitung: Externe Sensorwerte aus MeshComWebDesk senden

Praktische Schritt-für-Schritt-Anleitung, um das `"type":"tele"`-UDP-Telegramm aus
MeshComWebDesk heraus zu nutzen. Die vollständige Protokoll-Referenz (Feldnamen,
Grenzen, Fehlercodes) steht in [`ext_udp_telemetry.md`](ext_udp_telemetry.md) — diese
Anleitung beschreibt nur die praktischen Schritte bis zum ersten funktionierenden Test.

**Wichtig zum Verständnis:** Die Werte werden **nicht** als eigenständiges
Telemetrie-Paket gesendet, sondern direkt in die Sensor-Variablen des Nodes
geschrieben und fließen automatisch ins nächste Positions-Beacon ein — genau wie
bei einem Node mit echter Sensorik. Sie sind dadurch für jeden Monitor/jedes
Dashboard **nicht von echter Sensorik unterscheidbar**.

---

## 1. Voraussetzungen

- Node läuft mit der Firmware, die `handleExternTelemetry()` in `src/extudp_functions.cpp`
  enthält.
- Node und der Rechner mit MeshComWebDesk befinden sich **im selben WLAN/LAN**.
- UDP-Port `1799` ist zwischen beiden nicht durch eine Firewall blockiert.
- Der Node hat **keine echte Sensorik verbaut** (BME280/BMP3xx/AHT20/SHT21) — ist
  welche vorhanden, ignoriert der Node die externen Werte bewusst, um die echte
  Sensorik nicht zu stören.

---

## 2. Einmalige Konfiguration am Node

Die extern-UDP-Schnittstelle ist standardmäßig **aus**. Sie muss einmalig über die
Konsole des Nodes (Serial, BLE, oder die entsprechenden Felder in der offiziellen
MeshCom-App) aktiviert werden:

```
--extudpip <IP-Adresse-von-MeshComWebDesk>
--extudp on
```

Beispiel, wenn der Rechner mit MeshComWebDesk die IP `192.168.1.43` hat:

```
--extudpip 192.168.1.43
--extudp on
```

**Wichtig:** `--extudp on` schlägt fehl ("Please set EXPUDP IP first"), wenn vorher
keine gültige IP mit `--extudpip` gesetzt wurde. Die IP darf außerdem nicht mit der
eigenen Node-IP identisch sein.

> Diese Konfiguration ist **unabhängig** vom eigentlichen `"tele"`-Feature — sie ist
> dieselbe extudp-Einrichtung, die MeshComWebDesk vermutlich schon für den
> Nachrichtenversand (`"type":"msg"`) und den Empfang der Node-eigenen `"type":"pos"`/
> `"type":"tele"`-Ausgaben nutzt. Ist das bei dir schon eingerichtet, kannst du
> Schritt 2 überspringen.

Anders als bei einer früheren Version dieser Erweiterung spielen `--gateway`/
`--track` hier **keine** Rolle — der Positions-Beacon geht davon unabhängig immer
auch über LoRa raus.

---

## 3. Das Telegramm von MeshComWebDesk senden

Ein einfaches UDP-Paket (kein TCP, keine Verbindung nötig) an `<Node-IP>:1799` mit
folgendem JSON-Inhalt — alle Felder optional, nur angegebene werden übernommen:

```json
{"type":"tele","temp":23.3,"hum":60,"press":1018.5}
```

| Feld     | Inhalt                                      |
|----------|----------------------------------------------|
| `type`   | immer `"tele"`                                |
| `temp`   | Temperatur in °C                              |
| `hum`    | Luftfeuchte in %                              |
| `press`  | Luftdruck (QFE) in hPa                        |
| `temp2`  | 2. Temperatursensor (optional)                |
| `qnh`    | Luftdruck auf Meereshöhe (optional)            |
| `gasres` | Gaswiderstand (optional)                       |
| `co2`    | CO2-Wert (optional)                            |

Ein Feld für Regenmenge o.ä. gibt es aktuell nicht — nur diese 7 festen Felder, weil
nur dafür ein passendes APRS-Kommentarfeld existiert (siehe Protokoll-Referenz).

### Schneller Vorab-Test ohne MeshComWebDesk (PowerShell)

```powershell
$client = New-Object System.Net.Sockets.UdpClient
$json = '{"type":"tele","temp":23.3,"hum":60,"press":1018.5}'
$bytes = [System.Text.Encoding]::ASCII.GetBytes($json)
$client.Send($bytes, $bytes.Length, "<Node-IP>", 1799) | Out-Null
$client.Close()
```

`<Node-IP>` durch die tatsächliche IP-Adresse des Nodes ersetzen.

### Beispiel in C# (falls MeshComWebDesk .NET nutzt)

```csharp
using System.Net.Sockets;
using System.Text;

var payload = "{\"type\":\"tele\",\"temp\":23.3,\"hum\":60,\"press\":1018.5}";
var bytes = Encoding.ASCII.GetBytes(payload);

using var client = new UdpClient();
client.Send(bytes, bytes.Length, nodeIpAddress, 1799);
```

---

## 4. Prüfen, ob es angekommen ist

Am Node (serielle Konsole mitlesen):

```
[EXT] tele accepted: temp=23.3 hum=60.0 press=1018.5 temp2=0.0 qnh=0.0 gasres=0.0 co2=0.0
```

Direkt danach löst der Node einen sofortigen Positions-Beacon aus. Im Monitor/Dashboard
sollte die Station kurz danach mit den neuen Werten erscheinen — genau wie eine
Station mit echter Sensorik (z.B. `/T=23.3/H=60.0/P=1018.5` im Positions-Kommentar).

Falls stattdessen diese Meldung im Log erscheint, sieh in
[`ext_udp_telemetry.md`, Abschnitt 4](ext_udp_telemetry.md#4-fehlerf%C3%A4lle-log-meldungen-am-node)
nach der genauen Ursache:

- `[EXT] tele ignored: real sensor hardware detected on this node`
- `[EXT] tele missing recognized fields (temp/hum/press/temp2/qnh/gasres/co2)`

---

## 5. Kurz zusammengefasst

1. Einmalig: `--extudpip <IP>` + `--extudp on` am Node setzen.
2. Aus MeshComWebDesk: UDP-Paket mit dem `"type":"tele"`-JSON an `<Node-IP>:1799` senden
   (nur die 7 bekannten Felder: `temp`, `hum`, `press`, `temp2`, `qnh`, `gasres`, `co2`).
3. Node meldet `[EXT] tele accepted: ...` im Log, setzt die Sensor-Variablen und
   sendet sofort einen Positions-Beacon mit den neuen Werten.
4. Die Werte bleiben dauerhaft gesetzt (nicht nur einmalig) und fließen in jeden
   weiteren Beacon ein, bis sie überschrieben werden oder der Node neu startet.
5. Funktioniert nur, solange der Node **keine** echte Sensorik verbaut hat.
