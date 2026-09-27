# Verlässliche Zustellung persönlicher Nachrichten (PN)

Wiederholungen, Dedup und Rückwärtskompatibilität — eine Auslegeordnung

- Stand: 27.09.2026, DK5EN
- Bezug: `docs/wiederholungen.md` (OE1KBC, Commit `b7cc126f`); dort offen: "Variante b) Martin bitte
  definieren"
- Code-Stand: Verweise ohne Zusatz beziehen sich auf `upstream/dev` @ `cf215b5d`. `fork-main:` meint
  den Firmware-Zweig von DK5EN.
- Schreibweise: die PN-Nummer steht APRS-konform als `Text{NNN` am Ende des Textes (keine
  schließende Klammer), die Quittung lautet `:ackNNN`.
- Zählweise: "Aussendung" meint jede Übertragung einer PN durch den Absender, die erste eingeschlossen.
  Upstream sendet heute bis zu 4-mal (1 + 3 Wiederholungen).
- Geprüft: alle Aussagen mit Code-Verweis wurden in einem unabhängigen Review gegen den Quelltext
  verifiziert. Aussagen über den zentralen Server sind als **Annahme** gekennzeichnet, weil sein
  Quelltext nicht vorliegt.

## 0. Kurzfassung

1. **Heute scheitert die Wiederholung an zwei Stellen.** Erstens bricht der Absender ab, sobald er
   _irgendein_ Echo seiner msg_id hört — auch das des ersten Relais, lange bevor der Empfänger etwas
   bekommen hat. Zweitens ist eine Wiederholung eine byte-gleiche Kopie mit derselben msg_id und
   bleibt im Dedup jedes Relais hängen, das das Original schon kannte.
2. **Der Mesh-Dedup prüft ausschließlich die 32-Bit-msg_id**, in allen Firmware-Versionen. Eine
   Wiederholung mit auch nur einem geänderten msg_id-Bit ist für jedes Relais — alt wie neu — ein neues
   Paket und wird weitergereicht. Kurts Variante a) wirkt im Mesh deshalb sofort, flottenweit.
3. **Genau das ist zugleich das Problem:** fast alle anderen Abnehmer (Empfänger-Display, App, mcapp,
   Web-GUI, meshmap, Gateways, vermutlich der Server) deduplizieren ebenfalls über die msg_id. Was das
   Relais durchlässt, sehen sie mehrfach. **Das Relais braucht eine neue Kennung, alle anderen brauchen
   die alte.**
4. **Der heikelste Abnehmer ist der Weg über den Server:** jedes Gateway speist jede Meldung mit neuer
   msg_id, die ihm der Server schickt, in sein LoRa-Netz ein — auch PN an fremde Rufzeichen. Reicht
   der Server Wiederholungen an alle Gateways weiter (offen, siehe Fragen am Ende), würde ohne
   Server-Anpassung jede Wiederholung in allen Regionen ausgestrahlt. Die Server-Änderung muss
   deshalb **vor** der ersten Absender-Firmware kommen.
5. **a) und b) sind derselbe Mechanismus**: beide geben der Wiederholung für die Relais eine neue
   msg_id und lassen sie woanders als dieselbe PN wiedererkennen — a) an den unteren 30 Bit der msg_id,
   b) an Absender und NNN. Sie lassen sich nahtlos kombinieren.
6. **Empfehlung:** Server zuerst, dann Voraussetzung V (Abbruch nur durch echte Quittung) plus a) in
   einer XOR-Form, nur für PN. Die Erstsendung bleibt dabei byte-gleich zu heute. Für eine längere
   Leiter (9 Aussendungen, Verwahrung) schließt b) an. Variante c) scheidet in gemischten Netzen aus.
   MeshCore löst das Problem auf dieselbe Art (Kapitel 5).

## 1. Ziel

**Jede PN wird entweder quittiert, oder der Absender sieht nach definierter Zeit "unbestätigt". Keine
PN verschwindet still.** "Nicht zugestellt" lässt sich nie sicher feststellen: geht die Quittung auf
die letzte Aussendung verloren, sieht eine zugestellte PN wie eine verlorene aus.

Messbar heißt das:

- Die bestehende Wiederholung erreicht den Empfänger tatsächlich, auch über mehrere Hops.
- Jede PN erscheint bei jedem Abnehmer (Display, App, mcapp, Web-GUI, meshmap, aprs.fi) höchstens
  einmal — bei neuer Firmware sofort, bei alter Firmware mit bekanntem, begrenztem Effekt.
- Der Server kann Mehrfachkopien (viele Gateways × mehrere Aussendungen) mit einer einfachen, schnellen
  Operation deduplizieren.
- Als Ausbau eine längere Leiter zur Maximierung der Zustellwahrscheinlichkeit. Referenz sind zwei
  Leitern aus dem Firmware-Zweig von DK5EN (`--dmretry 3|9`, `fork-main:src/dm_outbox.cpp:75-77`):
  - **3 Aussendungen**: t = 0 / 40 / 80 s
  - **9 Aussendungen**: drei Blöcke zu je 3 Aussendungen im 40-s-Abstand, Blockstart im
    3-Minuten-Raster: t = 0 / 40 / 80 · 180 / 220 / 260 · 360 / 400 / 440 s

## 2. Ausgangslage

### 2.1 Aufbau einer PN

- **msg_id** (32 Bit): `((_GW_ID & 0x3FFFFF) << 10) | (node_msgid & 0x3FF)` — 22 Bit Knotenkennung aus
  der MAC plus 10 Bit laufende Nummer 0..999 (`src/loop_functions.cpp:4071`, gleiches Muster in
  `:3324`, `:4270`, `:4928`, `:4988`). **Die Bits 30–31 sind die Bits 20–21 der Knotenkennung, nicht
  frei.** Der Zähler `node_msgid` wird von allen Meldungsarten geteilt (Text, Position, HEY,
  Telemetrie, Quittung).
- **Frame**: Byte 0 Typ, Byte 1–4 msg_id (little endian), Byte 5 Hop/Flags, dann
  `QUELLE>ZIEL:Text{NNN`, danach Anhang mit HW, MOD, FCS, FW-Version, Last-HW, Unterversion und `0x7e`
  (`src/aprs_functions.cpp:1255-1285`, `:1310-1370`).
- **FCS** ist eine Bytesumme über alle Bytes vor dem FCS-Feld, **einschließlich der msg_id**
  (`src/aprs_functions.cpp:427-453` Prüfung, `:1335-1348` Erzeugung). Wer msg_id-Bits im fertigen
  Frame ändert, muss die FCS neu berechnen, sonst verwirft jeder Empfänger den Frame.
- **Byte 5 ist voll belegt**: Bits 0–3 Hop, 0x10 MESH, 0x20 App-offline, 0x40 Track, 0x80 Server
  (`src/aprs_functions.cpp:1262-1274`).
- **Quittung**: der Empfänger sendet `ZIEL     :ackNNN` als neue Meldung mit **seiner eigenen,
  frischen msg_id** (`src/loop_functions.cpp:4988`, Zähler `:5008-5010`, Format `:5001-5006`). Die
  Original-NNN steht nur im Text. Mehrere Quittungen haben daher schon heute verschiedene msg_ids
  und passieren den Relais-Dedup. Der Satz in `docs/wiederholungen.md`, die ACK-msg_id enthalte die
  laufende Nummer des Absenders und müsse die Wiederholungsbits mitnehmen, trifft auf den Code nicht zu.
- **Zuordnung der Quittung**: der Absender baut die Original-msg_id aus _seiner eigenen_
  Knotenkennung und NNN neu auf (`src/lora_functions.cpp:1040-1041`, `src/udp_functions.cpp:422`,
  `src/nrf52/nrf_eth.cpp:561`) und vergleicht voll 32-bittig (`checkOwnTx`,
  `src/loop_functions.cpp:702-723`).

### 2.2 Dedup heute

- **Schlüssel: nur die msg_id**, 4 Byte, linearer `memcmp` (`src/dedup_functions.cpp:23-52`). Nicht im
  Schlüssel: Quelle, Text, Typ, Hop. Das gilt für alle Versionen seit 4.35a.
- **Ringgröße nach Anzahl, nicht nach Zeit** (`src/configuration_global.h:238-285`): 100 Einträge
  (ESP32-S3, RAK4631), 70 (klassischer ESP32), 60 (XML/SBUFFER-Builds). Gemessen entspricht das rund
  38 min Fenster bei 100 Einträgen (`src/dedup_functions.h:18-30`).
- **Ein Urteil für alles**: ist ein LoRa-Frame ein Duplikat, wird er weder angezeigt, noch ans
  Telefon gegeben, noch hochgeladen, noch weitergeleitet — **und auch nicht erneut quittiert**
  (`src/lora_functions.cpp:897`; `SendAckMessage` für `{NNN` bei `:1081` liegt im Neu-Zweig).
- **Relais senden nie den Rohpuffer weiter**, sondern decodieren und encodieren neu
  (`src/lora_functions.cpp:648`, `:1454`, `:1502-1505`). Das gilt in allen Versionen seit 4.35a.
- **Ausnahme Server-Pfad**: auf dem UDP-Weg vom Server laufen Display, BLE und ACK _vor_ dem
  Dedup-Tor (`src/udp_functions.cpp:455`, `:469`, `:477`, Tor bei `:483`; nRF52
  `src/nrf52/nrf_eth.cpp:621`, `:629`, Tor bei `:642`). Ein Gateway, das selbst Empfänger ist, zeigt
  und quittiert dort jede Kopie, die der Server schickt — auch byte-gleiche.

### 2.3 Wiederholung heute

- **Nur Textmeldungen**, PN _und_ Gruppen/`*` (`src/loop_functions.cpp:4116-4126`; Nicht-Text wird
  auf DONE gezwungen, `src/lora_functions.cpp:2100-2104`).
- **Takt**: bis zu 4 Aussendungen im 40-s-Abstand (`MAX_RETRANSMIT 3`, Schwelle 0x15 bei 2-s-Takt,
  `src/lora_functions.cpp:2085-2133`).
- **Aufbau**: byte-gleiche Kopie des Ringeintrags — **gleiche msg_id**
  (`src/lora_functions.cpp:2170-2171`, `src/txring_functions.cpp:499-504`).
- **Abbruch** durch
  1. **jedes gehörte Echo** mit derselben msg_id, noch vor Decode und Dedup
     (`src/lora_functions.cpp:590-637`),
  2. eine binäre ACK (0x41) mit dieser msg_id (`src/lora_functions.cpp:376-416`). Gateways erzeugen
     diese nur für `*`, WLNK-1, APRS2SOTA und Gruppen, **nie für PN** (`:1254`),
  3. ein per LoRa empfangenes `:ackNNN` (`src/lora_functions.cpp:1036-1063`).
- **Ein über den Server eintreffendes `:ackNNN` stoppt die LoRa-Wiederholung nicht** (es setzt nur
  den Status und benachrichtigt das Telefon, `src/udp_functions.cpp:419-446`,
  `src/nrf52/nrf_eth.cpp:558-596`). `findAndStopRingSlot` ist `static` und wird nur in
  `lora_functions.cpp` aufgerufen (`:263`, `:413`, `:1059`).

### 2.4 Server → LoRa: jedes Gateway speist ein

- Ein Gateway strahlt jede Textmeldung, die ihm der Server schickt, in sein LoRa-Netz aus, wenn deren
  msg_id im Dedup neu ist und nicht eigene ist (`src/udp_functions.cpp:330`, Tor `:483`,
  `:485`, Einspeisung `:497`; nRF52 `src/nrf52/nrf_eth.cpp:642-657`).
- Ausgenommen sind nur PN an das eigene Rufzeichen (`:404`), eigene Quittungen (`:443`) und Positionen
  bei `NOPOS` (`:359`). **Einen Filter nach "Ziel hier bekannt" gibt es nicht.**
- Heute schützt der gemeinsame msg_id-Dedup: dieselbe PN kommt über N Gateways zurück, wird aber
  überall als bekannt verworfen. **Jede Wiederholung mit neuer msg_id wäre dagegen in jeder Region
  neu** — sofern der Server sie weitergibt.
- Die Gateways legen die eingespeisten fremden msg_ids zusätzlich in ihre Tabelle eigener Meldungen
  (`insertOwnTx`, `src/udp_functions.cpp:495`, `src/nrf52/nrf_eth.cpp:655`). Das wird für a) wichtig
  (Kapitel 4).

### 2.5 Die daraus folgenden Probleme

**P1 — Abbruch zu früh.** Beispiel A → R1 → R2 → B, der letzte Sprung R2 → B fällt aus. A hört das
Echo von R1 und bricht ab (2.3, Abbruch 1). Die PN ist verloren, obwohl A noch drei Aussendungen
gehabt hätte. Wiederholt wird heute nur, wenn A _gar kein_ Echo hört.

**P2 — Wiederholung landet im Dedup.** Wenn A wiederholt, trägt die Kopie dieselbe msg_id. Jedes Relais,
das das Original gehört hat, verwirft sie (2.2). Typischer Fall: R1 hat das Original gehört und
weitergegeben, A hat R1s Echo aber nicht gehört (asymmetrische Strecke). A wiederholt, R1 verwirft.

**P3 — Kein erneutes ACK.** Kam das Original beim Empfänger an, ging aber die Quittung verloren, wird
eine Wiederholung mit gleicher msg_id als Duplikat verworfen und _nicht_ erneut quittiert (2.2).

**P4 — Server-ACK wirkungslos.** Eine über das Internet zurückkommende Quittung stoppt die
LoRa-Wiederholung nicht (2.3).

**Befund zu Commit `5efa2171`** (durch `d2979ccc` auskommentiert, also nicht aktiv): Die Maske
`& 0x3FFFFFFF` in `sendMessage` (und ebenso in `SendAckMessage`) löschte die Bits 30–31 der
Original-msg_id. Die Rekonstruktion aus
`:ackNNN` (`src/lora_functions.cpp:1041`) setzt diese Bits aber weiter aus der eigenen Knotenkennung,
ebenso prüft `ackMsgIdFromNode` die oberen 22 Bit (`src/ack_attribution.h:77-80`). Bei allen Knoten,
deren `_GW_ID`-Bits 20–21 nicht beide 0 sind — statistisch drei von vier —, hätte `:ackNNN` die eigene
PN nicht mehr gefunden. Die ringbasierten Vergleiche (Echo, binäre ACK) wären intakt geblieben, weil
sie die bereits maskierte msg_id aus dem Ring lesen.

### 2.6 Flotte: Firmware und Hardware

Quelle: meshmap `/api/all`, abgefragt am 27.09.2026: 1501 Knoten, davon 1476 mit Firmware-Angabe.
Prozente beziehen sich auf die 1476. (Die Firmware-Statistik `/api/v2/fleet/firmware` meldete zur
gleichen Zeit 1475; die Abweichung ist ein Knoten.)

| Firmware          | Knoten | Anteil | Dedup                                     |
| ----------------- | -----: | -----: | ----------------------------------------- |
| 4.35t             |    455 |   31 % | eigener Ring 60/70/100 (`MAX_DEDUP_RING`) |
| 4.35s             |     84 |    6 % | wie 4.35t                                 |
| 4.35p             |    682 |   46 % | wie 4.35t                                 |
| 4.35o             |     27 |    2 % | TX-Ring mitbenutzt, 20/30 Einträge        |
| 4.35k             |     66 |    4 % | TX-Ring mitbenutzt, 20/30 Einträge        |
| 4.35 … 4.35n übr. |    162 |   11 % | TX-Ring mitbenutzt, 20/30 Einträge        |

Dazu 25 Knoten ohne Firmware-Angabe.

- Der eigene Dedup-Ring kam mit dem ersten 4.35p-Commit (`ec961163`, 13.03.2026). Bis 4.35o
  dedupliziert `is_new_packet` über `MAX_RING` = 20 bzw. 30 Einträge
  (`ec961163^:src/lora_functions.cpp:977-979`, `ec961163^:src/configuration_global.h:62-77`) — ein
  Fenster von wenigen Minuten auf belebten Kanälen. Der Schlüssel war immer nur die msg_id.
- **69 % der Knoten laufen auf einer Firmware älter als 4.35t, 17 % älter als 4.35p.** Jede Lösung
  muss mit diesem Bestand über Monate leben.

| Plattform         | Knoten | Typen (Anzahl)                                                                                                                                        |
| ----------------- | -----: | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| ESP32 klassisch   |    855 | TLORA V2.1.6 (490), TBEAM V1.2 (220), TBEAM V1.1 (128), HELTEC V2.1 (17)                                                                              |
| ESP32 kl. oder S3 |     70 | EBYTE E22 (Hardware-String unterscheidet DevKitC und S3-DevKitC nicht)                                                                                |
| ESP32-S3          |    499 | HELTEC V3 (319), T-DECK / T-DECK + (54), T-BEAM 1W (33), HELTEC STICK V3 (27), TBEAM SUPREME (19), HELTEC Tracker (18), HELTEC E290 (15), übrige (14) |
| nRF52840          |     51 | RAK4631 (44), T-ECHO (5), HELTEC T114 (2)                                                                                                             |
| ohne Angabe       |     26 |                                                                                                                                                       |

Mehr als die Hälfte der Flotte ist klassischer ESP32 mit knappem RAM. Jede zusätzliche Tabelle kostet
dort am meisten.

Volumen: meshmap zählte in den letzten 7 Tagen 1470 PN (nur was einen Server erreichte), zuvor 1644.

## 3. Problemfeld: wer dedupliziert worauf

| Abnehmer                        | Dedup-Schlüssel heute                                                                                | Beleg                                                                                                                                                     |
| ------------------------------- | ---------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Relais (alle Versionen)         | msg_id                                                                                               | `src/dedup_functions.cpp:23-52`                                                                                                                           |
| Empfänger-Knoten (Display, ACK) | msg_id (dasselbe Urteil wie Relais); über den Server-Pfad keiner                                     | `src/lora_functions.cpp:897`, `:1081`; `src/udp_functions.cpp:455-483`                                                                                    |
| Gateway (Server → LoRa)         | msg_id                                                                                               | `src/udp_functions.cpp:483-497`                                                                                                                           |
| Web-GUI-Chat des Knotens        | msg_id im Browser (`data-id`)                                                                        | `src/web_functions/web_functions.cpp:949`, `:1940`                                                                                                        |
| MeshCom-App                     | msg_id + Absender + Text; ACK-Zuordnung nur über msg_id                                              | Meshcom-MobileApp `src/DBservices/DataBaseService.ts:283-288`, `:345`, `:372`                                                                             |
| mcapp (MCProxy + Webapp)        | Absender + msg_id (60 min); Push und Webapp nur msg_id; `:ackNNN` über NNN + Adressen                | MCProxy `src/mcapp/storage/ingest.py:1918-1991`, `src/mcapp/push_delivery.py:488-537`, `ingest.py:1768-1870`; webapp `src/stores/messages/dedup.ts:72-87` |
| meshmap                         | API: msgId + Absender; Karte: msgId + Absender innerhalb 300 s; Speicher: msgId + Absender + Sekunde | mcmap `proxy/src/db/apiQueries.ts:196-214`, `proxy/src/db/mergeSnapshots.ts:428-515`, `proxy/src/db/unifiedStore.ts:3063`                                 |
| zentraler Server (C)            | **Annahme**: msg_id (enthält die Knotenkennung bereits in den oberen 22 Bit)                         | Quelltext nicht öffentlich                                                                                                                                |
| APRS-IS                         | Quelle (mit SSID) + Ziel (ohne SSID) + Info-Feld, Pfad ignoriert; bei aprsc 30 s ab Erstempfang      | aprsc `src/dupecheck.c`, `src/config.c` (`dupefilter_storetime = 30`); aprs-is.net/ServerDesign.aspx                                                      |
| aprs.fi                         | keine dokumentierte Nachrichten-Dedup über APRS-IS hinaus                                            | aprs.fi/page/api (`what=msg`)                                                                                                                             |

Daraus folgt:

1. **Relais**: eine Wiederholung braucht eine andere Kennung als das Original, sonst bleibt sie hängen
   (P2). Auch alte Firmware kennt nur die msg_id.
2. **Alle anderen** müssen erkennen, dass Wiederholung und Original dieselbe PN sind, sonst erscheint
   sie mehrfach.
3. **Gateways und Server** bilden eine zweite Verteilebene (2.4). Der Server entscheidet zweimal: was
   er anzeigt, zu APRS-IS und meshmap gibt (einmal pro PN), und was er an Gateways weiterreicht. Wird
   eine Wiederholung dort blockiert, bleibt eine PN, die auf der letzten Funkstrecke einer _anderen_
   Region verloren ging, ohne Wiederholung — P2 auf dem Internetweg. Wird sie durchgereicht, strahlt
   jedes Gateway sie aus. **Welche Gateways der Server heute mit einer PN beliefert (alle oder nur die,
   die das Ziel gehört haben), ist offen.**
4. **App und mcapp** sehen PN nur über ihren angeschlossenen Knoten. Normalisiert der Knoten
   (Wiederholungen unterdrücken, ACK-Status mit der Original-msg_id melden), brauchen sie keine
   Änderung. Die App ordnet ACKs unabhängig von der Firmware nur über die msg_id zu
   (`DataBaseService.ts:345`); an einem Knoten mit alter Firmware sieht sie Kopien.
5. **meshmap** zeigt über die API eine PN pro (msgId, Absender). Auf der Karte gilt ein Fenster von
   300 s, Kopien mit größerem Abstand erscheinen doppelt.
6. **aprs.fi**: aprsc fängt byte-gleiche Kopien nur innerhalb von 30 s ab Erstempfang (die
   Spezifikation spricht von einem "gleitenden" Fenster, der Code von aprsc verlängert es aber nicht;
   javAPRSSrvr wurde nicht geprüft). Beide Referenzleitern haben 40 s Abstand, also käme jede
   Wiederholung durch, sofern der Server sie weitergibt. Die Dedup für aprs.fi muss daher **am
   MeshCom-Server** passieren. Der APRS-Text darf keine Wiederholungsmarke tragen.
   - Laut icssw.org gehen PN nur dann zu APRS-IS, wenn der Text mit `APRS:` beginnt
     (icssw.org/en/unified-messaging). Welche weiteren Pakete der Server gatet, ist nicht öffentlich.
7. **Alte Empfänger-Firmware** zeigt jede Wiederholung mit neuer Kennung als eigene Nachricht, quittiert
   dafür aber auch jede Kopie neu (löst P3).

## 4. Optionen

Die Optionen gliedern sich nach einer einzigen Frage: **wo steckt die Information "das ist eine
Wiederholung"?** Im Relais-Schlüssel selbst, also in der msg_id (a, b), oder außerhalb davon (c).
Davor steht eine Voraussetzung, die für alle gilt (V), danach folgen zwei Ausbauten (d, e).

### 4.1 V — Voraussetzung für alle Optionen: Abbruch nur durch echte Quittung

- **Für PN** stoppt nur ein `:ackNNN` des Empfängers (über LoRa _oder_ Server) die Wiederholung.
  Ein gehörtes Echo markiert "gehört" (Status an die App wie heute) und **verlängert** den Takt bis zur
  nächsten Aussendung, statt sie abzubrechen. Ein Echo heißt nur, dass die PN unterwegs ist; auf
  belebten Kanälen kann der Weg zum Empfänger und zurück länger als 40 s dauern (Feldmessung
  DG0OPK-11/-12/-13, 23.–25.08.2026: Median der Sendeverzögerung für Text 30 s an einem belebten
  Knoten).
- **Für Gruppen** bleibt der heutige Abbruch (Echo oder Gateway-ACK), weil es dort keinen einzelnen
  Empfänger gibt.
- Server-`:ackNNN` stoppt ebenfalls (P4): `findAndStopRingSlot` exportieren und aus
  `udp_functions.cpp` und `nrf_eth.cpp` aufrufen.
- Ohne V bringt keine der folgenden Optionen etwas bei Mehrhop-Verlusten (P1). V ohne eine der
  Optionen bringt ebenfalls wenig, weil die Wiederholung dann in P2 hängen bleibt.
- **Kosten**: jede zusätzliche Aussendung ist eine volle Flutung des Mesh, und jede Kopie, die den
  Empfänger erreicht, löst eine Quittungs-Flutung aus. Bei nicht erreichbarem Empfänger sind das im
  Upstream-Takt bis zu 3 zusätzliche Flutungen pro PN; heute sind es 0, sobald A ein Echo hört.

### 4.2 Der gemeinsame Kern von a) und b)

a) und b) sind **derselbe Mechanismus**. Beide geben jeder Wiederholung eine andere msg_id, damit
Relais — alte wie neue — sie weiterleiten. Beide lassen alle anderen Abnehmer die Wiederholung über
einen zweiten Schlüssel als dieselbe PN wiedererkennen. **Der einzige echte Unterschied ist, woran
wiedererkannt wird:**

- **a)**: an den unteren 30 Bit der msg_id — binär im Kopf des Frames, ohne den Text zu lesen.
- **b)**: an (Absender, NNN) — die `{NNN` steht am Ende des Textes.

Die NNN ist in a) übrigens für alle Aussendungen ebenfalls gleich. Ein Empfänger mit neuer Firmware
könnte in a) also genauso gut auf (Absender, NNN) prüfen. Dann wäre der Empfängerteil von a) und b)
identisch, und der Unterschied bliebe auf den Server und auf die Zahl möglicher Aussendungen beschränkt.

| Baustein                                          | a) XOR-Zähler            | b) frische msg_id                  |
| ------------------------------------------------- | ------------------------ | ---------------------------------- |
| V (Abbruch nur durch `:ackNNN`)                   | gleich                   | gleich                             |
| Relais leiten weiter (alte und neue FW)           | gleich                   | gleich                             |
| `:ackNNN` passt auf jede Aussendung               | gleich (NNN bleibt)      | gleich (NNN bleibt)                |
| Absender: FCS neu, eigene msg_id in eigenen Dedup | gleich                   | gleich                             |
| Empfänger quittiert jede Kopie (P3)               | gleich                   | gleich                             |
| Knoten normalisiert für App, mcapp, Web-GUI       | gleich                   | gleich                             |
| Alte Empfänger zeigen Kopien                      | gleich (je Aussendung 1) | gleich (je Aussendung 1)           |
| Server-Weiterleitung an Gateways klären           | gleich                   | gleich                             |
| **Wiedererkennen im Server**                      | Bit-Maske `& 0x3FFFFFFF` | `{NNN` aus dem Text lesen          |
| **Max. Aussendungen**                             | 4 (3 Codes)              | beliebig                           |
| **Knotenkennung in der msg_id**                   | 20 Bit für Dedup         | volle 22 Bit                       |
| **meshmap**                                       | Maske                    | NNN aus dem Feed oder Inhaltsdedup |

**a) und b) lassen sich deshalb ohne Bruch kombinieren**: Aussendungen 1–4 nach a), ab Aussendung 5
frische msg_id nach b). Ähnlich macht es MeshCore: 2-Bit-Zähler für die ersten vier Aussendungen, danach eine erweiterte Nummer (Kapitel 5.1).

**Verworfen — Wiederholungsmarke im Text**: statt der msg_id den Text zu kennzeichnen hilft keinem
Relais (sie prüfen nur die msg_id), bricht die APRS-IS-Dedup und zeigt die Marke bei alten Empfängern
und auf aprs.fi.

### 4.3 a) Wiederholungszähler in den Bits 30–31 (OE1KBC), in XOR-Form

**Kurts Entwurf**: Erstsendung `00`, Wiederholungen `01`/`10`/`11` in Bit 30–31. Weil diese Bits heute
die obersten zwei Bits der Knotenkennung sind (2.1), würde schon die Erstsendung für drei von vier
Knoten ihre msg_id ändern.

**XOR-Form (Vorschlag)**: Wiederholung k (1..3) bekommt in Bit 30–31 die Original-Bits aus `_GW_ID`
XOR k. Schreibweise im Folgenden: X⊕k.

- Die Erstsendung bleibt byte-gleich zu heute; `msg_id >> 10` kennzeichnet bei ihr weiterhin den Knoten.
- Wiederholungen unterscheiden sich nur in Bit 30–31, die unteren 30 Bit bleiben gleich. Für jeden
  Vergleich genügt `id & 0x3FFFFFFF`.
- **k immer auf die Original-Bits anwenden, nie auf die vorige Kopie.** Die Wiederholung kopiert heute
  den vorigen Ringeintrag (`src/lora_functions.cpp:2170`); schrittweises XOR auf die Kopie ergäbe 01,
  11, 00, und die dritte Wiederholung wäre wieder das Original.
- **Nur für PN.** Gruppen und `*` behalten die heutige Wiederholung. Sonst zeigen alte Knoten im
  ganzen Netz Gruppenmeldungen bis zu 4-mal, und Gateways senden pro Kopie zwei binäre ACKs
  (`src/lora_functions.cpp:1274`, `:1298`).

**Was sich wo ändert:**

- **Relais**: nichts. Alte und neue Firmware leiten jede Wiederholung weiter.
- **Absender** (neue Firmware):
  - Bits in der Ringkopie setzen **und FCS neu berechnen** (2.1).
  - Jede Wiederholungs-msg_id in den eigenen Dedup-Ring eintragen, damit das eigene Echo nicht als
    fremde Meldung gilt.
  - Echo-Abgleich (`src/lora_functions.cpp:590-637`) und `findAndStopRingSlot` auf 30 Bit. Das genügt,
    weil beide nur die eigenen aktiven Ringeinträge durchsuchen (`:267`). Die ACK-Rekonstruktion
    (`:1041`, `src/udp_functions.cpp:422`, `src/nrf52/nrf_eth.cpp:561`) liefert ohnehin die
    Original-msg_id und bleibt unverändert.
  - Die eigenen Wiederholungen **nicht** in `own_msg_id` eintragen und `checkOwnTx` nicht generell
    maskieren. Gateways führen dort auch fremde, vom Server eingespeiste msg_ids (2.4), und
    `own_msg_id[..][4]` ist nur ein Statusbyte, kein Herkunftsmerkmal (`src/loop_functions.cpp:736`).
    Ein maskierter Vergleich würde eine fremde Wiederholung X⊕1 als eigenes Echo behandeln und sie
    weder weiterleiten noch hochladen (`src/lora_functions.cpp:870`, `:897`). Eigene Wiederholungen
    erkennt der Knoten stattdessen am Knotentest in 30-Bit-Form:
    `((id & 0x3FFFFFFF) >> 10) == (_GW_ID & 0xFFFFF)`. Das spart außerdem 3 der 20 Plätze in
    `own_msg_id` pro PN (`MAX_RING` 20).
  - Status "gehört" ans Telefon mit der Original-msg_id melden. Nur `src/lora_functions.cpp:880` kann
    eine Wiederholungs-msg_id tragen; die übrigen Statuspfade betreffen binäre ACKs oder Gruppen.
- **Empfänger** (neue Firmware): das Relais-Urteil bleibt auf der vollen msg_id. Für Anzeige, BLE, Web
  und Upload kommt ein zweites Urteil auf `id & 0x3FFFFFFF` (oder gleichwertig auf (Absender, NNN), 4.2)
  dazu, auch auf dem Server-Pfad (2.2). Jede Kopie wird trotzdem quittiert (P3). Das Zeitfenster dieses
  Urteils muss kurz bleiben (einige Minuten, länger als die Leiter), weil der Zähler `node_msgid` von
  allen Meldungsarten geteilt wird und ein aktiver Knoten die 1000 Werte in weniger als einer Stunde
  durchläuft.
- **Server** (Annahme zu seinem heutigen Verhalten): Dedup-Schlüssel `id & 0x3FFFFFFF` — eine
  UND-Operation, sofern er heute über die msg_id dedupliziert.

**Wo normalisiert wird — Server oder Gateway:**

- **Im Server (empfohlen)**: der Server maskiert im Dedup-Schlüssel. Er sieht alle Kopien, erkennt sie
  als eine PN und gibt sie einmal an Anzeige, APRS-IS und meshmap weiter.
- **Am Gateway (nur Rückfall)**: neue Gateways setzen die Bits vor dem Upload auf das Original zurück
  (plus FCS). Der Server sähe von ihnen immer die Original-msg_id X, auch ohne eigene Änderung. Aber: er
  reicht X an die Gateways zurück, und Gateways, die nur die Wiederholung X⊕1 kannten, halten X für neu
  und speisen es erneut ein (`src/udp_functions.cpp:310`, `:497`) — zusätzliche Airtime und eine
  weitere Kopie für alte Empfänger. Alte Gateways laden ihre Kopien ohnehin unverändert hoch. Deshalb nur,
  falls der Server nicht angepasst werden kann.

**Kosten:**

- Für Dedup-Zwecke zählen nur noch 20 Bit Knotenkennung. Bei 1501 Knoten steigt die erwartete Zahl
  kollidierender Knotenpaare von etwa 0,27 (22 Bit) auf etwa 1,07 (20 Bit).
- Nur 3 Wiederholungscodes. **Die 9er-Leiter passt nicht**: ein wiederverwendeter Code stünde in den
  Relais noch im Dedup-Fenster (~38 min). Dafür ist die Kombination mit b) da (4.2).
- Alte Empfänger zeigen bis zu 4 Kopien.

### 4.4 b) Frische msg_id pro Aussendung, `{NNN` bleibt

- Jede Aussendung bekommt eine neue msg_id aus dem normalen Zähler. `{NNN` bleibt gleich, der
  Empfänger quittiert wie gewohnt `:ackNNN`.
- **Relais**: wie a), alte und neue Firmware leiten weiter.
- **Empfänger** (neue Firmware): zweiter Dedup auf (Absender, NNN) mit Längen- und Prüfsummenvergleich
  gegen NNN-Überlauf und zeitlicher Alterung. Referenz: 16 Einträge, 60 min, geteilt zwischen LoRa- und
  Server-Pfad (`fork-main:src/dm_dedup.h:33-34`).
- **Absender**: der Abbruch hängt an NNN statt an der msg_id.
  - **Minimalform**: in der Ringkopie die msg_id ersetzen (plus FCS), Abbruch über NNN. Aufwand mittel,
    praktisch derselbe wie bei a).
  - **Vollform**: eine Outbox pro PN mit eigenem Zeitplan (`fork-main:src/dm_outbox.cpp`). Nötig für die
    9er-Leiter und für Verwahrung.
- **Server**: Dedup auf (Absender, NNN) — dafür muss der Server `{NNN` aus dem Text lesen (in C ein
  kurzer Parser: letztes `{`, bis zu 5 Zeichen), aber eine Änderung an einer Stelle, die heute
  vermutlich nur die msg_id kennt.
- **Offenlegung zur Referenzimplementierung**: sie ist im Release v4.35t.09.27-neo von DK5EN hinter
  `--dmretry` enthalten (Standard aus) und hat zwei Schwächen, die für upstream zu beheben wären:
  - Frische msg_ids werden aus `millis()` erzeugt (`fork-main:src/dm_outbox_glue.cpp:43-46`). Damit
    kennzeichnet `msg_id >> 10` nicht mehr den Knoten, `ackMsgIdFromNode` schlägt fehl, und zwei Knoten
    mit ähnlicher Laufzeit können dieselbe msg_id erzeugen. Upstream sollte den normalen Zähler
    verwenden.
  - Aussendung 2 verwendet dieselbe msg_id, wenn kein Echo gehört wurde
    (`fork-main:src/dm_outbox.cpp:318-327`). Genau im Fall P2 (R1 hat das Original, A hat R1s Echo
    nicht gehört) verwirft R1 diese Aussendung. Die Regel sollte entfallen.
- **Kosten**: RAM für den zweiten Dedup (Vollform zusätzlich Outbox), spürbar auf klassischem ESP32;
  Server-Parser; meshmap braucht die NNN oder eine Inhaltsdedup; alte Empfänger zeigen so viele Kopien,
  wie es Aussendungen gibt (bis 9).
- **Gewinn**: beliebig lange Leitern, volle 22 Bit Knotenkennung.

### 4.5 c) Gleiche msg_id — die Relais erkennen die Wiederholung

Hier bleibt die msg_id gleich, und die Relais selbst müssen erkennen, dass eine Wiederholung
weitergeleitet werden soll. Drei Orte für diese Information wurden geprüft:

**c1) Zähler in einem Zusatzbyte im Anhang**

- Die Wiederholung trägt einen Zähler nach der FW-Unterversion, vor `0x7e`. Alte Decoder ab 4.35a
  ignorieren dieses Byte; die FCS deckt nur die Bytes davor ab (`src/aprs_functions.cpp:420-492`).
- Neue Relais deduplizieren auf (msg_id, Zähler).
- **Aber: jedes Relais encodiert neu** (2.2). Ein einziges altes Relais auf dem Weg entfernt das
  Zusatzbyte. Danach verwerfen auch neue Relais die Wiederholung, und neue Empfänger quittieren nicht
  neu. c1) wirkt nur auf Wegen, auf denen **jeder** Hop neue Firmware hat — bei 31 % Anteil von 4.35t
  selten.
- Übrige Abnehmer: keine Änderung, die msg_id ist dieselbe. meshmap zeigt auf der Karte Kopien mit mehr
  als 300 s Abstand doppelt (betrifft die 9er-Leiter).
- Kosten: neues Feld in Decode und Encode; alte Empfänger quittieren nicht neu (P3 bleibt).

**c2) Zähler in Byte 5 — verworfen**

- Alle Bits sind belegt (2.1). Bit 3 gehört zum Hop-Feld (`RcvBuffer[5] & 0x0F`), Relais zählen das
  ganze Halbbyte herunter (`src/lora_functions.cpp:1454`). Alte Firmware läse einen gesetzten Zähler als
  Hop 8 + h und trüge das Paket bis zu 15 Hops weit; nur Gateways begrenzen die Pfadlänge.

**c3) Kürzeres Dedup-Fenster nur für PN — verworfen**

- Etwa 30 s statt ~38 min: die Wiederholung käme durch, aber ohne Schutz gegen Rundläufe zwischen
  Relais. Der Dedup ist der einzige Schleifenschutz im Mesh. Außerdem wirkt auch das nur bei neuen
  Relais.

**Urteil zu c)**: in einem gemischten Netz nicht tragfähig. Das ist der Weg, den Meshtastic gewählt
hat, und Meshtastic braucht dafür zusätzlich Wiederholungen durch Relais (Kapitel 5.2).

### 4.6 Ausbau

**d) Store-and-Forward / Verwahrung**

- Ein Speicherknoten übernimmt eine unquittierte PN und stellt sie zu, sobald der Empfänger wieder
  gehört wird. Setzt b) voraus (Zustellung Minuten bis Stunden später mit neuer msg_id). Im Zweig von
  DK5EN vorhanden, hier nur als Perspektive.

**e) Präsenz statt Zeitplan — PN-Zustellung nur bei Lebenszeichen**

- Statt nach festem Takt wiederholt der Absender erst, wenn er ein Lebenszeichen des Empfängers hört
  (Position, HEY, eigene Nachricht). Spart Airtime bei abwesendem Empfänger, braucht aber a) oder b) für
  die eigentliche Wiederholung.

## 5. Was machen Meshtastic und MeshCore

Gelesen wurden Meshtastic-Firmware `master` @ `d6307752` (26.09.2026) mit Abgleich gegen v2.7.26,
Meshtastic-Android `main` @ `a3962733`, MeshCore `main` @ `e9412598` (20.09.2026) und der offene Client
meshcore_py @ `f4427bc3`. Verweise mit `MT:` bzw. `MC:` beziehen sich auf diese Stände.

Beide Systeme nennen die persönliche Nachricht **DM** ("Direct Message"). Eine DM entspricht genau
unserer **PN**: eine Textnachricht an genau einen Empfänger, der sie quittiert.

### 5.1 MeshCore: jede Aussendung ist ein anderes Paket

- **Dedup** über einen 8-Byte-SHA-256 aus Nutzlasttyp und Nutzlast; Pfad und Routing-Bits zählen nicht.
  Ring mit 160 Einträgen, nach Anzahl (MC: `src/Packet.cpp:41-50`, `src/helpers/SimpleMeshTables.h:9`).
- **Wiederholung**: ein 2-Bit-Zähler `attempt` steht _in_ der verschlüsselten Nutzlast neben einem
  unveränderten Zeitstempel. Ab der fünften Aussendung wird die Nummer zusätzlich als Byte ans Ende
  gehängt (MC: `src/helpers/BaseChatMesh.cpp:425-437`). Jede Aussendung hat dadurch einen anderen
  Hash, und Repeater leiten sie weiter. Die Verschlüsselung (AES-ECB ohne Nonce, MC:
  `src/Utils.cpp:85-145`) würde eine byte-gleiche Wiederholung sonst im Dedup verlieren.
- **Quittung**: der Empfänger quittiert **jede** Aussendung; ein Zufallsbyte macht jede Quittung für den
  Repeater-Dedup neu (MC: `src/helpers/BaseChatMesh.cpp:244-255`).
- **Abbruch nur durch die Ende-zu-Ende-Quittung** (MC: `src/helpers/BaseChatMesh.cpp:339`, `:349`). Der
  Absender merkt sich mehrere erwartete Quittungscodes, eine späte Quittung auf eine frühere Aussendung
  zählt also auch (MC: `examples/companion_radio/MyMesh.cpp:1113-1117`).
- **Zeitplan**: 3 Aussendungen, Zeitüberschreitung aus der Airtime berechnet (Flutung
  500 ms + 16 × Airtime; direkt 500 ms + (6 × Airtime + 250 ms) × (Hops + 1), MC:
  `examples/companion_radio/MyMesh.cpp:851-859`), Flutung nach zwei gescheiterten direkten Versuchen
  (meshcore_py `src/meshcore/commands/messaging.py:114-199`).
- **Pfadlernen**: die erste Nachricht wird geflutet, die Quittung bringt den Rückweg mit, danach wird
  quellgeroutet (MC: `src/helpers/BaseChatMesh.cpp:248-252`, `:328-345`).

**Umgelegt auf MeshCom — was wir lernen und übernehmen können:**

- **Der Kern ist übertragbar, und er ist unsere Variante a).** MeshCore ändert mit einem kleinen
  Zähler genau das, worauf seine Repeater deduplizieren — dort den Hash der Nutzlast, bei uns die
  msg_id. Die Repeater müssen davon nichts wissen. Das ist der Grund, warum a) in unserem gemischten
  Netz sofort wirkt. Auch der Ausbau über 4 Aussendungen hinaus ist dort gelöst wie unsere Kombination
  a) + b): klein anfangen, erst danach eine erweiterte Kennung.
- **Übernehmen: Quittung auf jede Aussendung.** Bei uns ist die Quittung schon heute eine eigene
  Meldung mit eigener msg_id (2.1) und passiert jeden Dedup. Es fehlt nur, dass der Empfänger auch eine
  bereits gesehene PN erneut quittiert (P3).
- **Übernehmen: Abbruch nur durch die Ende-zu-Ende-Quittung** — das ist unsere Voraussetzung V. Dass
  eine späte Quittung auf eine frühere Aussendung zählt, bekommen wir geschenkt: `:ackNNN` trägt die NNN,
  und die ist für alle Aussendungen gleich.
- **Übernehmen: Zeitüberschreitung aus der Airtime.** Heute wartet MeshCom fest 40 s. Aus SF, Bandbreite
  und Länge der Meldung ließe sich die Wartezeit berechnen; auf langsamen Einstellungen oder bei vielen
  Hops würde sie länger, bei schnellen kürzer.
- **Nicht übertragbar: Pfadlernen und Quellrouting.** MeshCom flutet jede Meldung; gerichtetes Senden
  wäre ein eigenes, großes Vorhaben.
- **Nicht übertragbar: Wiederholung durch den Client.** Bei MeshCore steuert die App die Versuche. Bei
  MeshCom steuert der Knoten sie, und das soll so bleiben, weil viele Knoten ohne App laufen.

### 5.2 Meshtastic: gleiche Kennung, Hilfe durch die Relais

- **Kennung**: jede Aussendung derselben DM behält die Paket-ID; sie dient auch als Nonce der
  Verschlüsselung (MT: `src/mesh/CryptoEngine.cpp:591-600`). Dedup über (Absender, ID), Ring nach Anzahl
  (MT: `src/mesh/PacketHistory.h:18-26`).
- **Auch Meshtastic bricht bei einem gehörten Echo ab — auch für DMs** (MT:
  `src/mesh/ReliableRouter.cpp:58-90`; Kommentar dort: "For DMs, you also get a real ACK back"). Das
  gleicht Meshtastic auf drei Wegen aus:
  1. **Relais wiederholen selbst**, sobald eine Route gelernt ist: das weiterleitende Relais sendet
     erneut, bis es den nächsten Hop weiterleiten hört (MT: `src/mesh/NextHopRouter.cpp:113-131`). Die
     letzte Aussendung des Absenders flutet wieder (MT: `NextHopRouter.cpp:486-507`).
  2. **Direkte Nachbarn geben eine Wiederholung des Absenders erneut weiter**, erkennbar an
     `hop_start == hop_limit` (MT: `src/mesh/FloodingRouter.cpp:47-56`). Ab dem zweiten Hop greift das
     nicht mehr.
  3. **Die Quittung wird selbst zuverlässig gesendet** (want_ack) und bekommt dabei jeweils eine neue ID
     (MT: `src/mesh/ReliableRouter.cpp:125-137`). Ein direkt gehörtes Duplikat wird mit einer
     0-Hop-Quittung beantwortet (MT: `NextHopRouter.cpp:163-171`).
- **Ehrlicher Status in der App**: eine Quittung vom Ziel zeigt "empfangen", ein bloßes Echo bei einer
  DM zeigt "weitergeleitet, nicht bestätigt" und bietet erneutes Senden an (Meshtastic-Android
  `MeshDataHandlerImpl.kt:433-438`, `Message.kt:257-277`). Nach der letzten Aussendung bekommt das
  Telefon eine Fehlermeldung `MAX_RETRANSMIT` (MT: `NextHopRouter.cpp:473-478`).

**Umgelegt auf MeshCom:**

- Meshtastic entspricht unserer Variante c): gleiche Kennung, die Relais müssen mithelfen. Das
  funktioniert dort, weil die Relais-Firmware es kann. In unserem Netz mit 69 % älterer Firmware
  fehlt diese Hilfe (4.5).
- **Übernehmen: ehrlicher Status.** Die MeshCom-App kennt schon "gehört" und "quittiert" (Status 0x00
  und 0x02). Mit V kommt "unbestätigt" nach der letzten Aussendung dazu. Die App sollte "gehört" nicht
  als Zustellung darstellen.
- **Übernehmen: die Quittung ist eine eigene, neue Meldung.** Das hat MeshCom schon (2.1).
- **Nicht übertragbar: Relais-Wiederholung auf gelernten Routen.** Sie setzt Routenwissen voraus, das
  MeshCom nicht hat.

### 5.3 Warum dort konsistenter zugestellt wird

- **MeshCore** hat alle drei Bausteine gleichzeitig: Abbruch nur durch die Ende-zu-Ende-Quittung, jede
  Aussendung ist für den Dedup neu, und der Empfänger quittiert jede Aussendung. Das sind genau P1, P2
  und P3 aus Kapitel 2.5.
- **Meshtastic** löst P2 und P3 für geflutete DMs jenseits des ersten Hops nicht. Es gewinnt durch
  Wiederholung im Relais auf gelernten Routen, durch zuverlässige Quittungen und durch eine App, die
  "weitergeleitet" und "bestätigt" unterscheidet.
- **Für MeshCom heißt das**: mit V + a) (oder b)) haben wir dieselben drei Bausteine wie MeshCore, ohne
  die Relais anfassen zu müssen. Dazu kommen aus beiden Systemen die airtime-basierte Wartezeit und der
  ehrliche Status in der App.

## 6. Kosten pro Umgebung

Skala: **0** keine Änderung · **S** klein (eine Stelle) · **M** mittel (mehrere Stellen, Tests) · **L**
groß (neue Komponente). "V" ist in jeder Spalte enthalten.

| Umgebung                       | V allein                   | V + a) XOR-Zähler                                           | V + b) frische msg_id                        | V + c) Zähler im Anhang        |
| ------------------------------ | -------------------------- | ----------------------------------------------------------- | -------------------------------------------- | ------------------------------ |
| FW Absender                    | S–M: Abbruch PN vs. Gruppe | M: Bits + FCS, Vergleiche auf 30 Bit, Telefon-Status        | M (Minimalform) bis L (Outbox, 9er-Leiter)   | M: Feld in Encode/Decode       |
| FW Relais (neu)                | 0                          | 0                                                           | 0                                            | M: Dedup auf (msg_id, Zähler)  |
| FW Empfänger (neu)             | 0                          | M: 2. Urteil auf 30 Bit, Re-ACK                             | M: 2. Dedup (Absender, NNN), Re-ACK          | S: Re-ACK bei höherem Zähler   |
| FW Gateway (neu)               | S: Server-ACK stoppt       | S: eigene Wiederholung per Knotentest erkennen              | 0                                            | 0                              |
| Alte FW im Netz (69 % < 4.35t) | 0                          | relayt ✓, zeigt bis 4 Kopien                                | relayt ✓, zeigt bis 4 bzw. 9 Kopien          | relayt ✗, Zähler geht verloren |
| Zentraler Server (Annahme)     | 0                          | S: Maske im Schlüssel; Weiterreichen klären                 | M: `{NNN` parsen; Weiterreichen klären       | 0                              |
| MeshCom-App                    | 0                          | 0, wenn Knoten normalisiert                                 | 0, wenn Knoten normalisiert                  | 0                              |
| mcapp                          | 0                          | 0, wenn Knoten normalisiert                                 | 0, wenn Knoten normalisiert                  | 0                              |
| Web-GUI des Knotens            | 0                          | 0 (liest den normalisierten Telefon-Ring)                   | 0 (dito)                                     | 0                              |
| meshmap                        | 0                          | S: Maske an 4 Schlüsselstellen, falls Server roh weitergibt | M: NNN aus dem Server-Feed oder Inhaltsdedup | 0 (Karte: > 300 s doppelt)     |
| APRS-IS / aprs.fi              | 0                          | 0, wenn Server dedupliziert                                 | 0, wenn Server dedupliziert                  | 0                              |
| RAM klassischer ESP32          | 0                          | klein (2. Urteil)                                           | 2. Dedup, Vollform zusätzlich Outbox         | +1 Byte pro Dedup-Eintrag      |
| Max. Aussendungen              | 4                          | 4                                                           | beliebig (3, 9, Verwahrung)                  | bis 256                        |

Hinweise:

- Der Web-GUI-Chat rendert aus dem Telefon-Ring (`src/web_functions/web_functions.cpp`,
  `sub_content_messages`) und profitiert automatisch von der Normalisierung im Knoten.
- "0, wenn Knoten normalisiert" gilt nur für einen angeschlossenen Knoten mit neuer Firmware.
- meshmap: die vier Schlüsselstellen sind `messageDedupKey` (`apiQueries.ts:211`), `msgFromIdKey`
  (`mergeSnapshots.ts:430`), `messageStatDedupKey` (`messageStats.ts:59`) und `hasMessage`
  (`unifiedStore.ts:3251`).
- mcapp liest in `src/mcapp/linkcheck.py:234-244` die Knotenkennung aus `msg_id >> 10`. Das betrifft
  nur Ping/Pong, die nicht wiederholt werden (`src/loop_functions.cpp:3365`, `:3460`).

## 7. Fazit

### 7.1 Die empfohlene Lösung: V + a) in XOR-Form

In Worten, vom Absender bis zum Server:

1. **Die Erstsendung bleibt, wie sie heute ist** — gleiche msg_id, gleiche Bytes, gleicher Weg.
2. **Der Absender wartet auf `:ackNNN`, nicht auf ein Echo.** Hört er nur ein Echo, weiß er: unterwegs,
   aber noch nicht angekommen. Er markiert "gehört" und wartet etwas länger bis zur nächsten
   Aussendung. Kommt `:ackNNN` — über Funk oder über den Server —, hört er auf.
3. **Jede Wiederholung bekommt eine leicht veränderte msg_id**: die obersten zwei Bits werden mit der
   Wiederholungsnummer (1, 2, 3) XOR-verknüpft, immer ausgehend vom Original. Die unteren 30 Bit —
   20 Bit Knotenkennung plus laufende Nummer — bleiben gleich. Die FCS wird neu berechnet.
4. **Jedes Relais, alt oder neu, sieht eine neue msg_id und leitet weiter.** An den Relais ändert sich
   nichts.
5. **Der Empfänger mit neuer Firmware erkennt die Wiederholung** an den unteren 30 Bit (oder an Absender
   und NNN), zeigt die PN nur einmal an, gibt sie nur einmal an Telefon, Web-GUI und Server weiter —
   und **quittiert trotzdem jede Kopie**, damit eine verlorene Quittung ersetzt wird.
6. **Ein Empfänger mit alter Firmware** zeigt jede Wiederholung als eigene Nachricht (bis zu 4-mal) und
   quittiert jede. Die PN kommt also an; der Schönheitsfehler verschwindet mit dem Update.
7. **Der Server** erkennt alle Kopien einer PN mit einer UND-Maske (`id & 0x3FFFFFFF`) als eine, zeigt
   sie einmal an und gibt sie einmal an APRS-IS und meshmap weiter. Er entscheidet außerdem, ob und an
   welche Gateways er eine Wiederholung weiterreicht.
8. **App, mcapp und Web-GUI** bleiben unverändert, weil ihr Knoten die Kopien schon herausfiltert.
   meshmap bekommt dieselbe Maske, falls der Server die msg_id unverändert weitergibt.
9. **Gruppen und `*`** bleiben, wie sie sind.

**a) und b) liegen dabei eng beieinander** (4.2). Beide geben der Wiederholung für die Relais eine neue
Kennung und lassen sie woanders wiedererkennen. a) erkennt an den unteren 30 Bit der msg_id, b) an
Absender und NNN. Den Ausschlag für a) als ersten Schritt gibt der Server: eine Bit-Maske im Kopf des
Frames ist einfacher als das Lesen der NNN aus dem Text. Wo mehr als 4 Aussendungen gebraucht werden,
schließt b) nahtlos an.

### 7.2 Wirkung im Vergleich

| Aspekt                             | V + a) XOR            | V + b)                 | V + c)                        |
| ---------------------------------- | --------------------- | ---------------------- | ----------------------------- |
| Wirkt ab dem ersten neuen Absender | ja, flottenweit       | ja, flottenweit        | nur auf rein neuen Wegen      |
| Löst P1 (Abbruch zu früh)          | ja (durch V)          | ja (durch V)           | ja (durch V)                  |
| Löst P2 (Dedup)                    | ja                    | ja                     | selten                        |
| Löst P3 (kein Re-ACK)              | ja                    | ja                     | nur bei neuen Empfängern      |
| Wiedererkennen über                | untere 30 Bit msg_id  | Absender + NNN         | gleiche msg_id                |
| Server-Aufwand (Annahme)           | Maske + Weiterreichen | Parser + Weiterreichen | keiner                        |
| Anzeige bei alten Empfängern       | bis 4×                | bis 4× bzw. 9×         | 1×                            |
| Längere Leitern (9er, Verwahrung)  | nein, nur mit b)      | ja                     | ja                            |
| Vorbild                            | MeshCore              | MeshCore (Ausbau)      | Meshtastic (mit Relais-Hilfe) |

### 7.3 Was wir gewinnen

- **PN kommen über mehrere Hops an.** Heute wird bei einem Verlust hinter dem ersten Relais überhaupt
  nicht wiederholt (P1). Mit V wird wiederholt, bis `:ackNNN` kommt, und mit a) kommen die
  Wiederholungen auch durch die Relais (P2).
- **Verlorene Quittungen werden ersetzt**, weil jede Kopie quittiert wird (P3). Der Absender sieht seltener
  "unbestätigt" bei einer PN, die längst angekommen ist.
- **Es wirkt sofort im ganzen Netz.** Kein Relais muss aktualisiert werden; es genügt, dass Absender und
  Server neu sind. Das zählt bei 69 % Knoten auf älterer Firmware.
- **Die Erstsendung bleibt unverändert.** Wer nichts wiederholen muss, merkt nichts, und alle
  Auswertungen, die aus `msg_id >> 10` den Knoten ablesen, funktionieren weiter.
- **Der Server bleibt einfach**: eine Bit-Maske im vorhandenen Schlüssel.
- **Kurts Entwurf bleibt die Grundlage**; die XOR-Form ist eine Verfeinerung, kein Gegenentwurf, und b)
  ist der Ausbau desselben Gedankens.
- **Wir holen zu MeshCore auf**: dieselben drei Bausteine (Abbruch nur durch echte Quittung, jede
  Aussendung für den Dedup neu, Quittung auf jede Aussendung).

### 7.4 Was wir dafür in Kauf nehmen

- **Mehr Sendungen, wenn der Empfänger nicht erreichbar ist.** Jede Wiederholung flutet das Mesh, jede
  angekommene Kopie löst eine Quittung aus. Im Upstream-Takt sind es bis zu 3 zusätzliche Aussendungen
  pro unbestätigter PN.
- **Alte Empfänger zeigen Kopien**, bis zu 4 pro PN, solange sie nicht aktualisiert sind.
- **Die Server-Weiterleitung muss geklärt sein.** Reicht der Server Wiederholungen unverändert an alle
  Gateways weiter (offen), strahlt jedes Gateway jede Wiederholung aus (2.4). Das ist das größte Risiko
  und der Grund, warum der Server zuerst kommt.
- **Zwei Bit weniger Unterscheidung zwischen Knoten** im Dedup-Schlüssel: bei 1501 Knoten etwa ein
  kollidierendes Knotenpaar statt 0,27.
- **Höchstens 4 Aussendungen.** Für die 9er-Leiter braucht es b) zusätzlich.
- **Sorgfalt in der Firmware**: die 30-Bit-Vergleiche müssen an allen Stellen der Prüfliste sitzen,
  `checkOwnTx` darf aber nicht generell maskieren (Gateways führen dort fremde msg_ids). Eine vergessene
  Stelle bricht die Quittung für drei Viertel der Knoten (Befund `5efa2171`, 2.5).
- **c) fällt weg**: in gemischten Netzen wirkungslos, weil alte Relais den Zähler entfernen.

### 7.5 Empfehlung und Reihenfolge

1. **Server zuerst** (vor jeder Absender-Firmware):
   - Dedup für Anzeige, APRS-IS und meshmap auf `id & 0x3FFFFFFF`.
   - Weiterreichen an Gateways festlegen: eine Wiederholung nur an Gateways, die das Ziel gehört haben,
     oder gar nicht.
2. **Firmware Stufe 1: V + a) in XOR-Form, nur für PN.** Prüfliste aus 4.3:
   - Abbruch nur durch `:ackNNN` (auch über den Server), Echo verlängert den Takt.
   - Bits = Original XOR k, FCS neu, Wiederholungs-msg_id in den eigenen Dedup-Ring.
   - 30-Bit-Vergleich im Ring, eigene Wiederholungen per Knotentest statt maskiertem `checkOwnTx`.
   - Status "gehört" ans Telefon mit Original-msg_id.
   - Empfänger: zweites Urteil mit kurzem Fenster, Re-ACK jeder Kopie.
   - App und mcapp bleiben unverändert; meshmap bekommt die Maske, falls der Server roh weitergibt.
3. **Stufe 2, bei Bedarf: b) für die 9er-Leiter und Verwahrung.** Aussendungen 1–4 wie in a), ab
   Aussendung 5 frische msg_id aus dem normalen Zähler. Zusätzlich zu a): Server liest die NNN,
   Empfänger prüft auf (Absender, NNN).
4. **Aus MeshCore und Meshtastic mitnehmen**: Wartezeit aus der Airtime statt fester 40 s; in der App
   "gehört", "quittiert" und "unbestätigt" klar trennen.
5. **Bench-Test vor jeder Stufe**: Kette A → R1 → R2 → B mit erzwungenem Ausfall des letzten Sprungs, ein
   Knoten auf 4.35p als Relais und einer als Empfänger, ein Gateway mit Server-Anbindung.

### 7.6 Offene Fragen an den Server-Betrieb

1. Worauf dedupliziert der Server heute (msg_id allein, msg_id + Rufzeichen, Inhalt)?
2. Welche Gateways bekommt eine PN vom Server (alle, oder nur die, die das Ziel gehört haben)?
3. Welche Pakete gehen zu APRS-IS (nur PN mit `APRS:`, auch Gruppen)?
4. Gibt der Server die msg_id unverändert an meshmap und andere Abnehmer weiter?
