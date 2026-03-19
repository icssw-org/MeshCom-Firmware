# Flood-Networking-Analyse — MeshCom Netzwerk Analyse und Star-Node-Priorität

**Autor:** Martin DK5EN

**Datum:** 2024-06-15

## Funktionsweise des Flood-Netzwerks (Zusammenfassung)

MeshCom ist ein **reines Flood-Netzwerk**. Jeder Knoten, der ein LoRa-Paket
empfängt, sendet es einmal weiter, sofern keine der Abbruchbedingungen greift.
Es gibt keine Routing-Tabelle, keine Next-Hop-Auswahl, kein Link-State-Wissen.
Ein Gateway überbrückt zusätzlich zwischen Internet (UDP) und LoRa.

**TX-Pipeline:**
1. Nachricht wird in den TX-Ringpuffer eingestellt (`ringBuffer[MAX_RING]`)
2. Die Hauptschleife prüft per CAD (Channel Activity Detection) mittels CSMA-Backoff
3. Bei freiem Kanal wird der Slot mit der höchsten Priorität gesendet
4. Weitergeleitete Nachrichten sind Fire-and-Forget (`RING_STATUS_DONE`)
5. Nur der **Ursprungsknoten** sendet erneut (bis zu 3 Versuche, im Abstand von 40 s)

---

## Bedingungen, die ein Paket stoppen

Ein Paket wird von einem Gateway oder Mesh-Knoten **nicht weitergeleitet**,
wenn EINE der folgenden Bedingungen zutrifft:

| # | Bedingung | Code-Stelle |
|---|-----------|-------------|
| 1 | **Hop-Count = 0** — wird *vor* dem Relay dekrementiert, d.h. eine Nachricht mit `max_hop=1` wird einmal weitergeleitet | `lora_functions.cpp:946` |
| 2 | **Schleife erkannt** — eigenes Rufzeichen erscheint bereits im Quellpfad (`,OE1XYZ,` Substring-Match) | `lora_functions.cpp:957-967` |
| 3 | **Doppelte msg_id** — die 4-Byte msg_id befindet sich im Dedup-Ring (`ringBufferLoraRX[MAX_DEDUP_RING]`). Dies ist ein einfacher zirkulärer Puffer ohne Timeout — alte Einträge werden beim Wrap-Around stillschweigend überschrieben | `lora_functions.cpp:1131-1155` |
| 4 | **An sich selbst adressiert** — `destination_call == node_call`, Nachricht wird konsumiert, nicht weitergeleitet | `lora_functions.cpp:943` |
| 5 | **Server-Flag gesetzt** — `msg_server=true` bedeutet, ein Gateway hat dies bereits ins Internet weitergeleitet; andere Gateways speisen es nicht erneut in LoRa ein | `lora_functions.cpp:949-950` |
| 6 | **Pfad zu lang (nur Gateway)** — `msg_last_path_cnt >= max_hop + 1` blockiert Relay selbst wenn Hop-Count noch > 0 ist | `lora_functions.cpp:923-932` |
| 7 | **Nicht-weiterleibarer Typ** — nur TEXT (0x3A), POSITION (0x21), HEY (0x40) werden weitergeleitet; ACKs werden nicht gemasht | `lora_functions.cpp:1005` |
| 8 | **Mesh deaktiviert** — `checkMesh()` gibt false zurück (Knoten als Endpunkt konfiguriert) | `lora_functions.cpp:945` |
| 9 | **TX-Ring voll** — wenn der Ring voll ist und die eingehende Nachricht niedrigere Priorität hat als alle eingehenden Einträge, wird sie verworfen | `lora_functions.cpp:1318-1367` |

**Wichtige Verhaltenshinweise:**
- Weitergeleitete Nachrichten sind **immer** `RING_STATUS_DONE` — sie werden genau einmal gesendet (kein Retransmit-Timer).
- Nur der Ursprungsknoten führt Retransmission durch (nur TEXT, bis zu `MAX_RETRANSMIT=3`, ca. alle 40 s).
- Der Dedup-Ring hat **keinen zeitbasierten Ablauf**. Einträge werden erst durch Wrap-Around verdrängt (nach 60-100 neuen msg_ids je nach Plattform). Das bedeutet, auf einem ruhigen Netzwerk kann eine sehr alte msg_id noch minutenlang im Ring stehen.

---

## Star-Node-Analyse: Hilft es, wenn der zentrale Knoten zuerst sendet?

### Die Kernfrage

Wenn ein "Star-Node" (ein Knoten, der von vielen anderen Knoten gehört wird)
zuerst sendet — reduziert das die gesamte Airtime und Kollisionen im Vergleich
dazu, wenn periphere Knoten zuerst senden?

### Szenario A: Gateway als Star-Node (mit Internetanbindung)

```
      Internet
         |
      [GW-Star] ---- wird von N Mesh-Knoten gehört
       /  |  \
     M1  M2  M3 ... MN
```

**Aktuelles Verhalten (Nachricht von Internet → LoRa):**
1. Gateway empfängt UDP-Paket, stellt es in TX-Ring mit `RING_STATUS_DONE`
2. Gateway führt CSMA/CAD durch, sendet auf LoRa
3. Alle N Mesh-Knoten empfangen gleichzeitig
4. Jeder Mesh-Knoten fügt msg_id zum Dedup-Ring hinzu
5. Jeder Mesh-Knoten dekrementiert Hop-Count, reiht Relay mit `RING_STATUS_DONE` ein
6. Jeder Mesh-Knoten führt unabhängig CSMA/CAD-Backoff mit **Slot-basiertem Timing** durch
7. Knoten senden nacheinander wenn der Kanal frei wird; durch die Slot-Aufteilung und CAD-Erkennung werden Kollisionen effektiv vermieden

**Analyse — Gateway-Star sendet zuerst IST der natürliche Fall:**
- Das Gateway sendet für aus dem Internet stammende Nachrichten *bereits* zuerst
- Das Problem ist, **was danach passiert**: N Knoten wollen alle gleichzeitig weiterleiten
- Das CSMA-Slot-System verteilt die Knoten zuverlässig auf verschiedene Zeitslots. Bei SF11/BW250 reichen 350 ms aus, um 10 Slots darzustellen, die per CAD sicher erkannt werden. In der Praxis wurden bisher maximal 3 gleichzeitig sendende Knoten beobachtet — das System handhabt dies zuverlässig.
- Der Dedup-Ring hilft zusätzlich: Sobald ein Mesh-Knoten das Relay eines Nachbarn hört, prüft er und findet die msg_id bereits in seinem Dedup-Ring → er leitet NICHT erneut weiter... **aber das funktioniert nur, wenn der Knoten sein eigenes Relay noch nicht gesendet hat**

**Zentrale Erkenntnis:** Das CSMA-Prioritätssystem gibt weitergeleiteten Nachrichten `MSG_PRIO_NORMAL` (3) mit einem Basis-Backoff von `CSMA_PRIO_BASE_3 = 4500` ms. Mit `CSMA_PRIO_SLOTS_3 = 10` Slots von je 35 ms ergibt sich ein Slot-Fenster von 350 ms. Bei SF11/BW250 werden diese 10 Slots per CAD zuverlässig erkannt. In der Praxis wurden bisher maximal 3 gleichzeitig sendende Knoten beobachtet — das Slot-System ist somit ausreichend dimensioniert.

**Würde es helfen, wenn der Star-Node zuerst sendet?** Für Internet → LoRa: Das tut er bereits. Für LoRa → LoRa-Relay: Der Star-Node ist nur einer von N Knoten, die das Original gehört haben. Er bekommt das gleiche CSMA-Backoff wie alle anderen. **Dem Star-Node Priorität zu geben würde tatsächlich SCHADEN**, weil:
- Wenn der Star schnell weiterleitet, hören alle N Knoten das Duplikat und unterdrücken per Dedup — gut
- Aber der Star sendet nur das erneut, was alle bereits direkt empfangen haben — **kein Informationsgewinn**
- Die frühe Sendung des Stars verbraucht Airtime, ohne einen neuen Knoten zu erreichen

**Fazit für Gateway-Star:** Das Gateway sendet für seine eigenen Nachrichten bereits zuerst. Für die Weiterleitung anderer Nachrichten gibt es keinen Vorteil, den Star zu priorisieren — es würde nur Airtime verschwenden, um an Knoten zu senden, die das Paket bereits haben.

### Szenario B: Mesh-Star-Node (kein Internet, aber hohe Konnektivität)

```
     M1  M2  M3
       \  |  /
      [M-Star] ---- wird von N Knoten gehört
       /  |  \
     M4  M5  M6 ... MN
```

**Nachricht stammt von M1:**
1. M1 sendet (CSMA-Backoff, Originator-Priorität)
2. M-Star + einige Nachbarn von M1 empfangen
3. Alle empfangenden Knoten reihen Relay ein (RING_STATUS_DONE, MSG_PRIO_NORMAL)
4. CSMA-Wettlauf beginnt unter allen Empfängern

**Wenn M-Star zuerst sendet (Relay):**
- Alle N Knoten hören M-Stars Relay
- Jeder prüft Dedup-Ring → msg_id gefunden → Relay unterdrückt!
- Ergebnis: **nur 2 Sendungen insgesamt** (M1 Original + M-Star Relay)
- Das ist der **Idealfall** — minimale Airtime

**Wenn ein peripherer Knoten Mx zuerst sendet (Relay):**
- Nur M-Star + Mx's nahe Nachbarn hören Mx's Relay
- Andere Knoten (M2, M3, M4...) haben noch kein Relay gehört
- Sie werden ebenfalls Relays senden → **mehrere redundante Sendungen**
- Jedes Relay unterdrückt nur die Relays nahegelegener Knoten
- Ergebnis: möglicherweise 3-5+ Sendungen bevor alle Knoten unterdrückt sind

**Fazit für Mesh-Star:** JA, wenn der Star-Node zuerst weiterleitet, ist das vorteilhaft.
Es würde alle N-1 anderen Relays in einer einzigen Sendung unterdrücken. Die gesamte
Airtime sinkt von potenziell O(N) auf O(1) Relay-Sendungen.

### Wie könnte dies umgesetzt werden?

Aktuell hat die Firmware keinen Mechanismus, um "Star-ness" (Anzahl der
Nachbarn) zu erkennen. Mögliche Ansätze:

1. **MHeard-Count-basierter Prioritätsboost**: Knoten mit mehr Einträgen in ihrer
   MHeard-Tabelle (`MAX_MHEARD`) könnten einen niedrigeren CSMA-Basis-Backoff für
   Relays bekommen. Ein Knoten, der 50 andere Knoten hört, würde schneller
   weiterleiten als einer, der nur 3 hört.

2. **Gateway-Priorität für Relays**: Gateways haben bereits Internetanbindung
   und hören wahrscheinlich viele Knoten. Sie könnten `MSG_PRIO_HIGH` für Relays
   bekommen statt `MSG_PRIO_NORMAL`.

3. **Inverse-Nachbar-Count-Slots**: Statt gleichverteilter zufälliger Slot-Wahl
   `slot_range = MAX_SLOTS / mheard_count` verwenden. Star-Nodes bekommen frühe
   Slots, Leaf-Nodes spätere Slots.

**Zielkonflikt:** Wenn zwei Star-Nodes sich gegenseitig hören, würden beide
versuchen, schnell weiterzuleiten → Kollision zwischen den zwei wertvollsten
Sendern. Ein Tie-Breaking-Mechanismus (z.B. niedrigerer Rufzeichen-Hash gewinnt)
wäre nötig.

---

## Topologie-Szenarien

### Szenario 1: Lineare Kette — 5 Knoten

```
  A --- B --- C --- D --- E
  (jeder hört nur linken/rechten Nachbarn)
```

**Nachricht stammt von A, max_hop = 4:**

| Schritt | Aktion | Airtime-Kosten |
|---------|--------|----------------|
| 1 | A sendet (Originator) | 1 TX |
| 2 | B empfängt, dekrementiert Hop → 3, leitet weiter | 1 TX |
| 3 | A hört B's Relay → Dedup unterdrückt. C empfängt, Hop → 2, leitet weiter | 1 TX |
| 4 | B hört C's Relay → Dedup unterdrückt. D empfängt, Hop → 1, leitet weiter | 1 TX |
| 5 | C hört D's Relay → Dedup unterdrückt. E empfängt, Hop → 0, kein Relay | 0 TX |

**Gesamt: 4 Sendungen, 0 Kollisionen.** Dies ist die optimale Topologie für
Flood-Networking — jedes Relay erreicht genau einen neuen Knoten, und Dedup
unterdrückt sofort die Rückwärtsrichtung.

**Kein Vorteil durch Star-Node-Optimierung** — es gibt keinen Star-Node in einer
linearen Kette. Jeder Knoten hört maximal 2 andere.

**Fehlerfall:** Wenn max_hop < 4, empfängt Knoten E die Nachricht nie. Mit
`MAX_HOP_TEXT_DEFAULT = 4` ist diese Kette an der exakten Grenze.

### Szenario 2: Vollvermascht — 5 Knoten

```
     A ──── B
    /|\    /|
   / | \  / |
  E  |  \/  |
   \ | /\   |
    \|/  \  |
     D ──── C

  (jeder hört jeden → Kollisionen + CAD-Backoff)
```

**Nachricht stammt von A, max_hop = 4:**

| Schritt | Aktion | Airtime-Kosten |
|---------|--------|----------------|
| 1 | A sendet | 1 TX |
| 2 | B, C, D, E empfangen alle gleichzeitig | — |
| 3 | Alle 4 Knoten reihen Relay ein (Hop → 3, RING_STATUS_DONE) | — |
| 4 | CSMA-Wettlauf: Basis-Backoff + Slot-Zuweisung aus Prioritäts-Slots | — |
| 5 | **Bester Fall:** Ein Knoten gewinnt seinen Slot, die anderen 3 hören es → Dedup unterdrückt | 1 TX |
| 6 | **Typischer Fall:** 1-2 Knoten senden bevor die übrigen per Dedup unterdrückt werden | 1-2 TX |
| 7 | **Worst Case:** Bei 4 Knoten belegen diese verschiedene Slots — Kollisionen sind selten | max 4 TX |

**Gesamt: 1 Original + 1-4 Relays = 2-5 Sendungen.**
Optimal wäre 1 Original + 0 Relays = 1 TX (jeder hat A's Sendung bereits gehört).

**Beobachtung:** In einer vollvermaschten Topologie ist **jedes Relay redundant**.
Alle Knoten haben das Original von A bereits empfangen. Die Relay-Versuche fügen
keine neue Information hinzu. Durch das Slot-System und die Dedup-Unterdrückung
bleibt die tatsächliche Anzahl gleichzeitiger Sendungen in der Praxis aber gering
(typischerweise 1-3).

**Auswirkung der Star-Node-Optimierung:** Wenn ein Knoten (z.B. B) als "Star"
bestimmt wird und zuerst weiterleitet (frühester Slot), würden die anderen 3
Knoten B's Relay hören und ihr eigenes per Dedup unterdrücken. Ergebnis:
1 Original + 1 unnötiges Relay = 2 TX. Immer noch verschwenderisch (ideal wäre
1 TX), aber deutlich besser als 5 TX.

**Bessere Optimierung für vollvermascht:** Ein Knoten könnte sein eigenes Relay
unterdrücken, wenn er weiß, dass alle seine MHeard-Nachbarn ebenfalls in
Reichweite des Originators waren. Dies erfordert Topologie-Wissen, das im
Protokoll derzeit nicht existiert.

### Szenario 3: Zwei Cluster mit Brückenknoten

```
  Cluster 1          Brücke          Cluster 2
  A1─A2─A3 ──────── [BR] ──────── B1─B2─B3
  (A1,A2,A3 hören    (hört          (B1,B2,B3 hören
   sich +BR)          beide Seiten)  sich +BR)
```

**Nachricht stammt von A1, max_hop = 4:**

| Schritt | Aktion | Airtime |
|---------|--------|---------|
| 1 | A1 sendet | 1 TX |
| 2 | A2, A3, BR empfangen | — |
| 3 | CSMA-Wettlauf zwischen A2, A3, BR | — |
| 4a | **Wenn BR gewinnt:** BR leitet weiter, A2+A3 unterdrücken per Dedup. B1,B2,B3 empfangen von BR | 1 TX |
| 4b | **Wenn A2 gewinnt:** A2 leitet weiter. A3+BR hören es → A3 unterdrückt. BR hat bereits Dedup-Eintrag aber noch nicht weitergeleitet — **BR unterdrückt sein eigenes Relay!** | 1 TX |
| 5b | B1,B2,B3 **empfangen die Nachricht nie**! | FEHLER |

**Dies ist das kritische Szenario.** Wenn BR's Relay durch Dedup unterdrückt wird
(weil es A2's Relay vor seinem eigenen CSMA-Slot gehört hat), ist Cluster 2
**komplett abgeschnitten**.

**Tatsächliches Code-Verhalten:** Die Dedup-Prüfung erfolgt beim Empfang
(`is_new_packet()` in `OnRxDone`). Aber das Einreihen des Relays erfolgt
ebenfalls in `OnRxDone`. Der Ablauf ist also:

1. BR empfängt von A1 → `is_new_packet()` = true → reiht Relay ein
2. BR hört A2's Relay → `is_new_packet()` = **false** (bereits in Dedup) → Relay wird nicht erneut eingereiht
3. BR's Relay (aus Schritt 1) ist noch im TX-Ring → **es wird trotzdem gesendet!**

**Moment — bei genauerem Lesen des Codes:** Der Dedup-Ring wird beim
**Empfang** geprüft, um zu entscheiden ob verarbeitet/eingereiht wird. Aber BR's
Relay wurde bereits in Schritt 1 eingereiht. Der zweite Empfang derselben msg_id
(von A2) wird einfach ignoriert. BR's eingerichtetes Relay wird NICHT storniert.

**Korrigierte Analyse:** BR wird weiterleiten, weil:
- Schritt 1: BR empfängt von A1, reiht Relay ein (RING_STATUS_DONE)
- Schritt 2: A2 leitet ebenfalls weiter, BR hört es aber Dedup sagt "bereits gesehen" → ignoriert
- Schritt 3: BR's Relay aus Schritt 1 ist noch im TX-Ring, wird gesendet wenn CSMA frei

**Das Brücken-Szenario funktioniert also korrekt.** Der Dedup-Ring verhindert
*erneutes Einreihen*, nicht die Stornierung bereits eingereihter Relays.

**Airtime-Analyse:**
- Bester Fall (BR leitet vor A2/A3 weiter): 1 + 1 = 2 TX, dann leiten B1/B2/B3 untereinander weiter = 1-3 TX. Gesamt: 3-5 TX.
- Typischer Fall: A2 und BR leiten beide weiter (CSMA-Wettlauf). A3 unterdrückt. Dann Relays im B-Cluster. Gesamt: 4-7 TX.

**Star-Node-Optimierung:** Wenn BR als Brücke erkannt wird (hoher MHeard-Count
aus beiden Clustern) und Priorität bekommt:
- A1 sendet → BR leitet zuerst weiter → A2, A3 unterdrücken → B1, B2, B3 empfangen und leiten untereinander weiter (1-2 TX, großteils unterdrückt)
- Gesamt: 1 + 1 + 1 = 3 TX optimal

**Hier liefert die Star-Node-Priorität den größten Mehrwert.** Der Brückenknoten
trägt einzigartige Information (verbindet zwei disjunkte Cluster), daher hat sein
Relay den höchsten Grenznutzen pro TX.

### Szenario 4: Leaf-Knoten an geschäftigem Hub

```
       M1  M2  M3  M4  M5
        \  |   |   |  /
         [HUB] ──────── (Hub hört 10+ Knoten)
        /  |   |   |  \
       M6  M7  M8  M9  M10
                          \
                         [LEAF]  (hört nur M10)
```

**Nachricht stammt von LEAF, max_hop = 4:**

| Schritt | Aktion |
|---------|--------|
| 1 | LEAF sendet |
| 2 | Nur M10 empfängt |
| 3 | M10 leitet weiter (Hop → 3) |
| 4 | HUB + LEAF empfangen M10's Relay. LEAF → Dedup unterdrückt. |
| 5 | HUB leitet weiter (Hop → 2). M1-M9 empfangen. |
| 6 | M1-M9 wollen alle weiterleiten → CSMA-Slot-Verteilung unter 9 Knoten |
| 7 | Durch Slot-Verteilung und Dedup-Unterdrückung senden typischerweise 2-3 Knoten, bevor die übrigen unterdrückt werden |

**Gesamt: ~4-6 TX.** Das Fan-Out vom HUB erzeugt parallele Relay-Anforderungen
unter M1-M9, die durch das Slot-System geordnet abgearbeitet werden.

**Nachricht stammt vom HUB (z.B. über Gateway), max_hop = 4:**

| Schritt | Aktion |
|---------|--------|
| 1 | HUB sendet |
| 2 | M1-M10 empfangen alle gleichzeitig |
| 3 | CSMA-Slot-Verteilung unter 10 Knoten → 1-3 Relays bevor Dedup den Rest unterdrückt |
| 4 | M10's Relay erreicht LEAF (Hop → 2). Relays der anderen sind verschwendet. |

**Gesamt: 1 + 1-3 = 2-4 TX.** LEAF wird durch M10's Relay bedient.

**Problem für die Richtung LEAF → HUB:** LEAF ist komplett von M10 abhängig. Wenn
M10 LEAF's Paket verpasst (Kollision, Interferenz), geht die Nachricht verloren.
LEAF hat keinen alternativen Pfad und keine Retransmission für weitergeleitete
Nachrichten.

**Wenn LEAF der Originator ist:** LEAF *sendet* erneut (nur TEXT, bis zu 3 Mal,
im Abstand von 40 s). Also haben vom LEAF stammende Nachrichten eine angemessene
Zuverlässigkeit. Aber durch LEAF weitergeleitete Nachrichten sind Fire-and-Forget.

**Star-Node-Optimierung für diese Topologie:**
- Wenn HUB zuerst weiterleitet (vor M1-M9), unterdrücken alle peripheren Knoten → ideal
- Aber das eigentliche Problem ist die **LEAF → M10**-Verbindung: ein Single Point of Failure
  ohne Relay-Diversität
- Star-Node-Priorität hilft LEAF nicht — was LEAF hilft ist Retransmission
  auf Originator-Ebene (die für TEXT bereits existiert)

---

## Übersichtstabelle

| Topologie | TX für 1 Msg (aktuell) | TX mit Star-Prio | Nutzen |
|-----------|----------------------|-------------------|--------|
| Linear 5 | 4 (optimal) | 4 | Keiner |
| Vollvermascht 5 | 2-4 | 2 | Moderat (spart 0-2 TX) |
| 2 Cluster + Brücke | 4-7 | 3 | **Erheblich** |
| Leaf an geschäftigem Hub | 4-6 | 3-4 | Moderat |

## Zentrale Erkenntnisse

1. **Star-Node-Erstsendung ist am wertvollsten für Brückenknoten**, die
   ansonsten disjunkte Cluster verbinden. Ihr Relay trägt einzigartige Information.

2. **In vollvermaschten Topologien sind ALLE Relays verschwendet.** Star-Node-Priorität
   reduziert die Verschwendung von N-1 Relays auf 1 Relay, aber das Ideal (0 Relays)
   erfordert Topologie-Bewusstsein, das im Protokoll nicht existiert.

3. **Lineare Ketten sind bereits optimal** für Flood-Networking. Keine
   Optimierung nötig.

4. **Leaf-Knoten sind fragil** aufgrund der Abhängigkeit von einem einzigen Pfad.
   Star-Node-Priorität hilft ihnen nicht; nur Retransmission auf Originator-Ebene
   bietet Zuverlässigkeit.

5. **Der Dedup-Ring storniert KEINE bereits eingereihten Relays.** Ein Knoten, der
   das Original gehört und ein Relay eingereiht hat, wird es trotzdem senden, auch
   wenn er danach das Relay eines anderen hört. Das ist tatsächlich korrekt für das
   Brücken-Szenario, aber verschwenderisch im vollvermaschten Szenario.

6. **Praktische Umsetzung** der Star-Node-Priorität könnte den MHeard-Count
   als Proxy für Konnektivität nutzen. Knoten mit hohem MHeard-Count bekommen
   niedrigeren CSMA-Basis-Backoff für Relay-Nachrichten. Risiko: Zwei Knoten mit
   hohem MHeard-Count in Hörweite würden kollidieren — Tie-Breaking nötig.

7. **Das CSMA-Slot-System funktioniert zuverlässig.** Bei SF11/BW250 reichen
   350 ms aus, um 10 Slots darzustellen, die per CAD sicher erkannt werden.
   In der Praxis wurden bisher maximal 3 gleichzeitig sendende Knoten beobachtet.
   MHeard-bewusstes Backoff könnte die Slot-Zuweisung weiter optimieren, ist aber
   keine dringende Notwendigkeit.

---

## Analyse verpasster Möglichkeiten: Können wir eingerichtete TX stornieren?

### Die relevante Zeitachse

Wenn ein Knoten ein Paket empfängt und ein Relay einreiht, gibt es ein
**2-5 Sekunden CSMA-Backoff-Fenster** bevor das Relay tatsächlich gesendet wird.
Während dieses Fensters ist das Radio im RX-Modus und KANN Relays anderer Knoten
für dieselbe Nachricht empfangen. Die Frage: Sollten wir unser eingereihtes Relay
stornieren, wenn wir hören, dass jemand anderes es zuerst weiterleitet?

```
t=0ms     Knoten empfängt Originalpaket vom Sender
          → is_new_packet() = true
          → fügt msg_id zum Dedup-Ring hinzu
          → reiht Relay im TX-Ring ein (RING_STATUS_DONE)

t=0-5000ms  CSMA-Backoff (Radio im RX-Modus)
            Während dieses Fensters könnte Nachbar-Relay eintreffen:
            → OnRxDone wird ausgelöst
            → is_new_packet() = false (bereits in Dedup)
            → Duplikat wird IGNORIERT — keine weitere Aktion
            → eingereihtes Relay wird NICHT geprüft oder storniert  ← DIES IST DIE LÜCKE

t=5000ms  CAD-Scan → Kanal frei → doTX()
          → getNextTxSlot() wählt das Relay
          → KEINE erneute Dedup-Prüfung gegen den TX-Ring
          → Relay wird gesendet  ← MÖGLICHERWEISE REDUNDANT
```

### Was der Code tatsächlich tut

1. **OnRxDone** (`lora_functions.cpp:547`): ruft `is_new_packet()` auf. Falls false
   (Duplikat), wird das gesamte Paket ignoriert. Kein Scan des TX-Rings erfolgt.

2. **doTX()** (`lora_functions.cpp:1375`): ruft `getNextTxSlot()` auf, wählt den
   Slot mit höchster Priorität, kopiert in `lora_tx_buffer`, sendet. Es gibt
   **keine erneute Dedup-Prüfung** vor dem Senden.

3. **getNextTxSlot()** (`lora_functions.cpp:1241`): sucht nach bester Priorität.
   Kein Bewusstsein dafür, ob die msg_id seit dem Einreihen erneut gehört wurde.

### Die verpasste Optimierung: "Späte Dedup-Stornierung"

Theoretisch könnten wir eine Prüfung an zwei möglichen Stellen einfügen:

**Option A — Stornierung bei doppeltem RX (in OnRxDone):**
Wenn `is_new_packet()` false zurückgibt, den TX-Ring nach einem eingereihten Relay
mit derselben msg_id durchsuchen. Falls gefunden, als konsumiert markieren
(`ringBuffer[slot][0] = 0`).

**Option B — Erneute Prüfung vor TX (in doTX):**
Vor dem Senden eines `RING_STATUS_DONE`-Relays zählen, wie oft die msg_id
empfangen wurde. Falls > 1, stornieren.

### Warum wir dies NICHT sicher tun können

**Das Brücken-Szenario macht beide Optionen zunichte:**

```
  A1─A2─A3 ──── [BR] ──── B1─B2─B3
```

1. A1 sendet. BR und A2 empfangen beide.
2. BR reiht Relay ein (RING_STATUS_DONE).
3. A2 gewinnt den CSMA-Wettlauf, leitet zuerst weiter.
4. BR hört A2's Relay während der CSMA-Wartezeit.

**Mit Spät-Dedup-Stornierung:** BR's Relay würde storniert. Cluster B
**empfängt die Nachricht nie**. Dies ist ein katastrophaler Fehler.

**Ohne Stornierung (aktuelles Verhalten):** BR leitet trotzdem weiter. Cluster B
empfängt. Das System funktioniert korrekt.

Das grundlegende Problem: **Ein Knoten kann nicht wissen, ob sein Relay einzigartige
Information trägt.** BR weiß nicht, dass B1-B3 existieren und nur über ihn
erreichbar sind. Alles was er weiß ist "Ich habe diese msg_id bereits im Dedup."
Allein darauf basierend zu stornieren würde das Bridging brechen.

### Was WÄRE sicher

Eine Stornierung ist nur sicher, wenn der Knoten feststellen kann, dass **alle
Knoten, die er erreichen kann, die Nachricht bereits über einen anderen Pfad
empfangen haben.** Dies erfordert Topologie-Bewusstsein, das das Flood-Protokoll
bewusst vermeidet.

Partielle Heuristiken, die sicher sind aber begrenzten Nutzen haben:

1. **Stornieren wenn Hop-Count = 0 nach Dekrement:** Unser Relay würde sowieso
   nicht weiter propagieren. Aber dieser Fall wird bereits nicht eingereiht
   (geprüft in Zeile 946).

2. **Stornieren wenn MHeard-Count = 1:** Ein Leaf-Knoten, der nur einen Nachbarn
   hört. Wenn dieser Nachbar bereits weitergeleitet hat (wir haben es gehört),
   erreicht unser Relay nur denselben Nachbarn zurück. Sicher zu stornieren —
   **aber Leaf-Knoten leiten ohnehin am wenigsten weiter**, daher sind die
   Einsparungen minimal.

3. **Stornieren wenn ALLE MHeard-Nachbarn im Quellpfad erscheinen:** Bedeutet,
   dass jeder Knoten, den wir erreichen können, dieses Paket bereits bearbeitet hat.
   Sicher, aber aufwändig zu prüfen und in der Praxis selten zutreffend.

### Sind wir am Optimum?

**Ja, für den allgemeinen Fall.** Das aktuelle Design trifft den richtigen Kompromiss:

| Topologie | Aktuelles Verhalten | Mit Stornierung | Bewertung |
|-----------|-------------------|-----------------|-----------|
| Linear | 4 TX (optimal) | 4 TX | Keine Änderung |
| Vollvermascht 5 | 2-5 TX | 1 TX (wenn alle stornieren) | Besser, aber... |
| Brücke | Funktioniert korrekt | **BRICHT** | Inakzeptabel |
| Leaf an Hub | 4-6 TX | 4-6 TX | Keine Änderung |

**Das Brücken-Szenario ist die Randbedingung, die Stornierung unsicher macht.** Ein
Flood-Netzwerk, das ohne Topologie-Bewusstsein funktionieren muss, kann nicht
zwischen "verschwenderischem Relay in dichtem Mesh" und "essentiellem Brücken-Relay"
unterscheiden. Der aktuelle Ansatz — immer einmal weiterleiten, Dedup verhindert
erneutes Einreihen — ist der korrekte Kompromiss.

### Wo die echten Gewinne liegen

Statt eingereihte TX zu stornieren (unsicher), sind die höherwertigen Optimierungen:

1. **CSMA-Slot-Skalierung nach MHeard-Count** — Star-/Brückenknoten leiten
   schneller weiter und unterdrücken mehr Peers per Dedup-bei-RX. Keine
   Stornierung nötig; das schnelle Relay verhindert einfach, dass andere überhaupt
   einreihen (sie hören das Relay bevor ihr eigener CSMA-Timer feuert, und das
   Original ist bereits in ihrem Dedup-Ring, also reihen sie nie ein). **Das ist
   sicher, weil nichts storniert wird — es ändert nur, wer den CSMA-Wettlauf
   gewinnt.**

2. **Unterdrückungszähler** — zählen, wie oft eine msg_id gehört wurde (nicht nur
   gesehen/nicht-gesehen). Wenn N-mal gehört und MHeard-Count klein, ist der Knoten
   tief eingebettet in einem dichten Cluster, wo sein Relay nichts hinzufügt.
   Könnte für **probabilistische Unterdrückung** verwendet werden (Relay mit
   Wahrscheinlichkeit proportional zu heard_count / mheard_count überspringen)
   statt harter Stornierung. Brückenknoten (heard_count=1, mheard_count=hoch)
   würden fast nie unterdrücken.

### Fazit

**Wir sind am korrekten Optimum für ein topologie-unbewusstes Flood-Netzwerk.** Das
bestehende CSMA-Slot-System mit 10 Slots bei 350 ms funktioniert zuverlässig und
deckt die in der Praxis auftretenden Knotenzahlen ab. Eine mögliche Verbesserung,
die kein Szenario bricht, ist **MHeard-bewusstes CSMA-Backoff** — gut vernetzte
Knoten zuerst weiterleiten lassen, sodass ihre einzelne Sendung viele Peers über
den bestehenden Dedup-Mechanismus natürlich unterdrückt. Dies ist eine
Tuning-Änderung an der CSMA-Slot-Berechnung, kein neuer Stornierungsmechanismus.
