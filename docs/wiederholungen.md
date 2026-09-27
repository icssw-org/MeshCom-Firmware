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

#### Die Antwort MSG-ID der ACK Message wird aus
- 22 Bit 0xFFFFFC0 bestehend aus der Node-MAC-Adresse des antworteten Node und
- 10 Bit 0x3FF aus der laufende Nummer der Absender-MSG-ID gebildet

## 2. Gruppenmeldungen welche vom nächsten GW mit ACK bestätigt werden

- Die ACK-Meldung wird vom nächsten Gateway wie eine DM ACK gebildet

- Hinweise dazu:
    - @Rainer: **Was wenn kein GW in der Nähe?**
        - @Kurt: **Möglichkeit: eigene Pfade prüfen ob ein GW gehört wird**

## 3. Ausnahmen welche nicht wiederholt werden

- Alle POS-Meldungen ohne MSB
- Alle TXT-Meldungen beginnend mit {ping} und {pong} keine MSB
- Alle TXT-Meldungen an ALL
- Alle HEY-Meldungen ohne MSB
- Alle TXT-Meldungen  mit msg_destination_call = 100001 ohne MSB
- Alle TXT-Medlungen mit WLNK-1 oder APRS2SOTA)

## 4. Welche Methoden sind möglich

### Variante a) MSG-ID trägt die Info
- Wir nehmen 2 MSB Bits der MSG-IDfür die Wiederholungskennung
    - '00 ... Erstmeldung
    - '01 ... 1. Wiederholung
    - '10 ... 2. Wiederholung
    - '11 ... 3. Wiederholung
- ACK-Medlungen nehmen diese Bits-ebenfalls mit

- Hinweise dazu:
    - @Martin: XOR Verknüpfung der 2 MSB mit dem Widerholungsstatus
        - So bleibt die Erstsendung byte-gleich zu heute, und msg_id >> 10 zeigt weiterhin den Knoten an. Damit geht der Ursprungs Knoten bei der ersten Aussendung nicht verloren

## 5. Was ist weiters zu beachten
- Zeitabläufe bis eine ACK vom Gateway oder Destination-Call zurück kommt
    - Idee @Kurt: stufenweises ACK-System == wurde überhaupt gehört. Dazu müssen die MESH-Knoten selbst nch mthelfen.
- Bevor Nachricht aus dem Buffer fällt kontrollieren ob diese wieder eingereiht wurde?

### Variante b) @Martin
- für mich ist im Konzept beantwortet (Kapitel 4.2):
    - Sie ist im Grunde derselbe Mechanismus wie deine Variante a).
    - Der Unterschied liegt nur darin, woran die Wiederholung wiedererkannt wird, und beide lassen sich kombinieren.
    - Die XOR Variante ist die eleganteste Lösung.
- Die Wiederholung bricht nicht mehr beim ersten gehörten Echo ab, sondern erst mit dem :ackNNN, auch wenn es über den Server kommt. Sonst wird bei einem Verlust hinter dem ersten Meshknoten gar nicht wiederholt.
- Gruppen und * bleiben aktuell unverändert.
    - **@Kurt: können wir in einem zweiten Schritt mitnhemen.** Letzlich doch sehr wichtig da die Server in MeshCom ein Teil der stabilen Struktur über große Reichweiten sind.
    - Wenn eine gruppe zumindest eine GW erreicht ist die Verteilung zu den Nachbarnetze und Nachbarländern gegeben.


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

## 5. Wie nehmen wir alte FW mit

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
