## Wiederholungen von LoRa Meldungen

## Themen
1. Textmeldungen an DM
2. Gruppenmeldungen
3. Welche Methoden sind möglich
4. Wie nehmen wir alte FW mit

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

## 3. Welche Methoden sind möglich

### Variante a) MSG-ID trägt die Info
- Wir nehmen 2 MSB Bits der MSG-IDfür die Wiederholungskennung
    - '00 ... Erstmeldung
    - '01 ... 1. Wiederholung
    - '10 ... 2. Wiederholung
    - '11 ... 3. Wiederholung
- ACK-Medlungen nehmen diese Bits-ebenfalls mit

### Variante b) Martin bitte definieren was Du genau meinst

### Variante c) noch eine Idee?

## 4. Wie nehmen wir alte FW mit

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
