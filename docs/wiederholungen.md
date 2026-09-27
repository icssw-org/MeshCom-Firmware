## Wiederholungen von LoRa Meldungen

## Themen
1. Textmeldungen an DM
2. Gruppenmeldungen
3. Welche Methoden sind möglich
4. Ausnahmen welche nicht wiederholt werden
5. Was ist weiters zu beachten
6. Wie nehmen wir alte FW mit

## 1. Textmeldungen an DM
Die Textmeldungen an DM werden aktuell
#### mit einer MSG-ID belegt welche aus
- 22 Bit 0xFFFFFC0 bestehend aus der Node-MAC-Adresse und
- 10 Bit 0x3FF laufende Nummer welche aber nur von 0 - 999 gebildet wird und nach 999 af 0 springt
- besteht
#### mit laut dem APRS-Format (Kompatibiliät)
- durch anhängen von { und der in der MSG-ID gebildeteten laufenden Nummer gebildet

- [DK5EN] Woher die 22 Bit Knotenkennung kommen (wichtig für die Bitwahl in Kapitel 4a):
    - ESP32: `getMacAddr()` (`src/esp32/esp32_main.cpp`) legt das **letzte** MAC-Byte in die untersten 8 Bit der Knotenkennung. Espressif vergibt pro Chip vier aufeinanderfolgende MACs (WLAN-STA, AP, BT, ETH), die Basis-MAC ist daher durch 4 teilbar. **Die untersten 2 Bit der Knotenkennung sind bei jedem ESP32 00.**
    - Stichprobe gegen meshmap (27.09.2026): bei 97 % der Knoten sind diese 2 Bit 00. Der Rest sind im Wesentlichen nRF52 (RAK), dort stammt die Kennung aus der Geräteadresse des Chips und ist praktisch zufällig.
    - In der MSG-ID liegen diese 2 Bit auf **Bit 10-11** (direkt über der laufenden Nummer).

#### Die Antwort MSG-ID der ACK Message wird aus
- 22 Bit 0xFFFFFC0 bestehend aus der Node-MAC-Adresse des antworteten Node und
- 10 Bit 0x3FF aus der laufende Nummer der Absender-MSG-ID gebildet

- [DK5EN] Das entspricht nicht dem Code: Der Empfänger sendet `:ackNNN` als **neue Meldung mit seiner eigenen, frischen MSG-ID** (eigener Zähler). Die NNN des Absenders steht nur im Text. Folge: Mehrere ACKs auf dieselbe Meldung haben schon heute verschiedene MSG-IDs und passieren den Relais-Dedup. Der Absender ordnet das ACK zu, indem er aus seiner eigenen Knotenkennung und NNN die Original-MSG-ID neu bildet.

## 2. Gruppenmeldungen welche vom nächsten GW mit ACK bestätigt werden

- Die ACK-Meldung wird vom nächsten Gateway wie eine DM ACK gebildet

- Hinweise dazu:
    - @Rainer: **Was wenn kein GW in der Nähe?**
        - @Kurt: **Möglichkeit: eigene Pfade prüfen ob ein GW gehört wird**

- [DK5EN] Einverstanden als zweiter Schritt. Voraussetzungen, bevor Gruppen wiederholt werden:
    - Zuerst muss das GW-ACK für Gruppen zuverlässig kommen. Gruppen- und *-Texte werden schon heute bis zu dreimal wiederholt, aber byte-gleich: Relais verwerfen die Wiederholung als Duplikat, und das erste Echo bricht ab. Bekämen sie eine eigene Wiederholungs-id, liefe ohne ACK jede Wiederholung über jedes Relais – das vervielfacht die Kanalbelegung.
    - Kein GW in Reichweite (Rainers Frage): dann nicht wiederholen, statt blind dreimal zu senden. Kurts Idee "eigene Pfade prüfen" passt; im Fork kennt die Nachbarschaftsmatrix gehörte Gateways bereits.
    - Der Server-Dedup (siehe Kapitel 4a, 30 Bit) muss dann auch Gruppen abdecken.

## 3. Ausnahmen welche nicht wiederholt werden

- Alle POS-Meldungen ohne MSB
- Alle TXT-Meldungen beginnend mit {ping} und {pong} keine MSB
- Alle TXT-Meldungen an ALL
- Alle HEY-Meldungen ohne MSB
- Alle TXT-Meldungen  mit msg_destination_call = 100001 ohne MSB
- Alle TXT-Medlungen mit WLNK-1 oder APRS2SOTA)

- [DK5EN] Die Liste ist im Branch `dk5en-xor` bereits vollständig abgedeckt – nicht beim Erzeugen der MSG-ID, sondern erst beim Bilden einer Wiederholung. Umgeschrieben wird nur eine PN: Textmeldung, persönliches Ziel, Text endet auf `{NNN`.
    - POS, HEY: keine Textmeldung, wird nie umgeschrieben.
    - `{ping}` / `{pong}{NNN}`: Text endet nicht auf `{NNN` (schließende Klammer), wird nicht umgeschrieben.
    - `*` (ALL), `100001` (reine Ziffern = Gruppe), `WLNK-1`, `APRS2SOTA`: Ziel ist nicht persönlich.
    - Host-Tests decken `{ping}`, `{pong}{NNN}` und `100001` ab (`test/test_pn_retry`).
- [DK5EN] **Mit der XOR-Form bekommt keine Meldung "keine MSB" – es wird bei der Erstsendung überhaupt nichts gelöscht.** Im Branch `dk5en-xor` stehen neben den Kommentaren "keine MSB für repeat markieren" in `src/loop_functions.cpp` (7 Stellen) und neben der auskommentierten Zeile `aprsmsg.msg_id = aprsmsg.msg_id & 0x3FFFFFFF;` (in `sendMessage()` und `SendAckMessage()`) jeweils `[DK5EN]`-Hinweise, wie es mit der XOR-Form richtig ist. Die Maske darf nie aktiviert werden: Sie würde die Erstsendung bei rund drei Viertel aller Knoten verändern und `msg_id >> 10` als Knotenkennung zerstören.

## 4. Welche Methoden sind möglich

### Variante a) MSG-ID trägt die Info
- Wir nehmen 2 LSB Bits der MSG-IDfür die Wiederholungskennung
    - '00 ... Erstmeldung
    - '01 ... 1. Wiederholung
    - '10 ... 2. Wiederholung
    - '11 ... 3. Wiederholung
- ACK-Medlungen nehmen diese Bits-ebenfalls mit

- Hinweise dazu:
    - @Martin: XOR Verknüpfung der 2 LSB mit dem Widerholungsstatus
        - So bleibt die Erstsendung byte-gleich zu heute, und msg_id >> 10 zeigt weiterhin den Knoten an. Damit geht der Ursprungs Knoten bei der ersten Aussendung nicht verloren

- [DK5EN] **Neuer Vorschlag: statt der 2 MSB die 2 LSB der Knotenkennung nehmen (MSG-ID Bit 10-11), weiterhin per XOR.**
    - Die 2 MSB der MSG-ID (Bit 30-31) sind heute keine freien Bits, sondern Bit 20-21 der Knotenkennung – bei jedem Knoten anders. Deshalb braucht es dort XOR, und die Tabelle oben (00 = Erstmeldung) stimmt dort für keinen festen Wert.
    - Bit 10-11 sind bei 97 % der Knoten (alle ESP32) schon heute 00. **Damit gilt die Tabelle oben wörtlich: 00 = Erstmeldung, 01/10/11 = 1./2./3. Wiederholung.** XOR mit der Wiederholungsnummer k ist nur für die restlichen ~3 % (nRF52) nötig und ergibt für ESP32 genau diese Tabelle.
    - Keine neuen Verwechslungen zwischen Knoten: Kippt man bei einem ESP32 die unteren 2 Bit, landet man auf den anderen drei MACs desselben Chips (AP, BT, ETH). Die kann kein anderer Knoten als Kennung haben. Bei den MSB gehören die gekippten Bits dagegen zu anderen, real existierenden Chips.
        - Grob geschätzt bei ~1.424 ESP32-Knoten und gleichverteilten MACs: heute ~1 Knotenpaar mit gleicher Kennung, mit MSB-Variante ~4, mit LSB-Variante weiterhin ~1. Das ist eine Schätzung, nicht gemessen – Espressif-MACs kommen in fortlaufenden Blöcken.
    - Der Knoten bleibt erkennbar: `msg_id >> 12` ist über alle vier Aussendungen gleich. Bei einer Wiederholung zeigt `msg_id >> 10` auf eine Nachbar-MAC desselben Chips, die es als Knoten nie gibt.
    - Wiedererkennung einer Wiederholung (Relais, Empfänger, Server): MSG-ID mit Maske `0xFFFFF3FF` vergleichen (statt `0x3FFFFFFF` bei der MSB-Variante).
    - Der Branch `dk5en-xor` ist auf die LSB-Variante umgestellt (`src/pn_retry.h`, Host-Tests, Doku). Die übrigen Stellen im Firmware-Code blieben gleich.
- [DK5EN] Zu "ACK-Meldungen nehmen diese Bits ebenfalls mit": nicht nötig und im Code auch nicht so. Das ACK hat eine eigene, frische MSG-ID (siehe Kapitel 1). Der Absender bildet aus NNN die Original-MSG-ID neu und stoppt die Wiederholung, wenn diese nach der Maske zu einer seiner wartenden Aussendungen passt.

## 5. Was ist weiters zu beachten
- Zeitabläufe bis eine ACK vom Gateway oder Destination-Call zurück kommt
    - Idee @Kurt: stufenweises ACK-System == wurde überhaupt gehört. Dazu müssen die MESH-Knoten selbst nch mthelfen.
- Bevor Nachricht aus dem Buffer fällt kontrollieren ob diese wieder eingereiht wurde?

- [DK5EN] Zeitabläufe: Heute wird fest alle 40 s wiederholt. Ein gehörtes Echo startet die Wartezeit neu. Bei gemessenen ~30 s Median-Wartezeit in der Sendewarteschlange pro Hop (DG0OPK-Mitschnitt) dauert ein Mehrhop-Umlauf leicht länger als 40 s – dann geht eine Wiederholung raus, bevor das ACK überhaupt ankommen kann. Vorschlag: steigende Abstände (z. B. 40 / 80 / 160 s), Werte aus Felddaten festlegen. Offen.
- [DK5EN] Stufenweises ACK: gibt es im Prinzip schon, ohne zusätzliche Aussendung. Das Echo eines Mesh-Knotens setzt den Status "gehört" (0x01), das `:ackNNN` "zugestellt" (0x02). Neu ist nur, dass das Echo die Wiederholung nicht mehr abbricht (Variante V, siehe unten). Ein eigenes ACK pro Mesh-Knoten würde Sendezeit kosten für eine Information, die das Echo gratis liefert.
- [DK5EN] "Bevor die Nachricht aus dem Buffer fällt": berechtigte Frage. Eine wartende PN könnte im Ring von einer Meldung mit höherer Priorität verdrängt werden. Im Branch noch nicht geprüft – offen.

### Variante b) @Martin
- für mich ist im Konzept beantwortet (Kapitel 4.2):
    - Sie ist im Grunde derselbe Mechanismus wie deine Variante a).
    - Der Unterschied liegt nur darin, woran die Wiederholung wiedererkannt wird, und beide lassen sich kombinieren.
    - Die XOR Variante ist die eleganteste Lösung.
- Die Wiederholung bricht nicht mehr beim ersten gehörten Echo ab, sondern erst mit dem :ackNNN, auch wenn es über den Server kommt. Sonst wird bei einem Verlust hinter dem ersten Meshknoten gar nicht wiederholt.
- Gruppen und * bleiben aktuell unverändert.
    - **@Kurt: können wir in einem zweiten Schritt mitnhemen.** Letzlich doch sehr wichtig da die Server in MeshCom ein Teil der stabilen Struktur über große Reichweiten sind.
    - Wenn eine gruppe zumindest eine GW erreicht ist die Verteilung zu den Nachbarnetze und Nachbarländern gegeben.

- [DK5EN] Einverstanden. Im Branch `dk5en-xor` umgesetzt: XOR-Form, Abbruch nur mit `:ackNNN` (LoRa oder Server), Empfänger bestätigt jede Wiederholung erneut, zeigt sie aber nur einmal an und lädt sie nur einmal hoch. Noch nicht auf Hardware getestet (A → R1 → R2 → B mit verlorenem letzten Hop steht aus).
- [DK5EN] Reihenfolge: **Der Server muss seinen Dedup auf die Maske (Kapitel 4a) umstellen, bevor die erste Absender-Firmware draußen ist.** Gateways speisen jede Meldung mit neuer MSG-ID vom Server wieder ins LoRa ein – ohne Server-Änderung kämen Wiederholungen doppelt zurück.

### Variante c) Umstellung auf 5.x
- @Rainer + @Kurt
    - zuerst festlegen wie man alte FW abkoppelt.
        - Änderung von SF/CR und letzlich BW
    - festlegen der neuen Payload-Variante
        - TYPE 1 Byte
        - LNG 1 Byte
    - festegen des neuen Protokolls - wichtig:
        - FCS über alles
        - Anzahl Source/Destinantion Pfad
    - Version mit neuer 2-byte Variante

- [DK5EN] SF/CR/BW ändern heißt harte Netztrennung: alte und neue Knoten hören sich nicht mehr. Das gehört in das MeshCom-5-Konzept (Topologie), nicht in die Wiederholungsfrage – die Wiederholung nach Variante a) funktioniert ohne Trennung.
- [DK5EN] "FCS über alles": Der eigentliche Gewinn wäre ein CRC16 statt der heutigen Bytesumme. Die Bytesumme erkennt z. B. vertauschte Bytes nicht.

## 6. Wie nehmen wir alte FW mit

### Variante a)

- bestehende Nodes mit ältererer FW
    -  bekommen die Meldungen und Wiederholungen wie bis zu drei eingeständige Meldungen mit
        - und bestätigen diese korrekt lesbar für alt/neu
        - Anzeige im Display kommt bis zu drei mal mit selben Text
        - Wenn die APPs bereits aktalisiert sind wird das erkannt.
    -  Senden Meldungen mit 2 MSB Bits welche 00,01,10,11 sein können
        - das bekommen alte/neue Nodes mit und bestätigen korrekt
        - Anzeige bei alt/neu OK
        - APPs können an der FW erkennen wie zu reagieren ist

- [DK5EN] Zählung: Erstsendung plus bis zu 3 Wiederholungen = **bis zu vier** Aussendungen. Ein alter Knoten zeigt denselben Text also bis zu viermal an, nicht dreimal.
- [DK5EN] Alte Relais leiten Wiederholungen weiter: Ihr Dedup vergleicht seit 4.35a nur die volle MSG-ID, jede geänderte Bitvariante gilt dort als neue Meldung. Genau darauf baut Variante a) – keine Änderung an alter FW nötig.
- [DK5EN] "Senden Meldungen mit 2 MSB Bits, welche 00, 01, 10, 11 sein können": Mit der LSB-Variante gilt das für die Bits 10-11. Alte ESP32 senden dort immer 00, alte nRF52 einen beliebigen festen Wert. Beides ist harmlos, weil neue Knoten die Wiederholung über die Maske erkennen und nicht über einen festen Wert.
