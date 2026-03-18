# ADR: Netzwichtigkeits-basierter Relay-Backoff (NC-Importance)

**Status:** Draft

**Datum:** 2026-03-16

**Autor:** Martin DK5EN

---

## Begriffe

In diesem Dokument werden zwei verschiedene NC-Werte verwendet, die klar
unterschieden werden muessen:

| Begriff | Kurzform | Quelle | Bedeutung |
|---------|----------|--------|-----------|
| **Eigener NC** | NC_self | `getMheardCount()` | Wie viele Stationen hoere **ich** direkt? Lokale Berechnung aus der mheard-Tabelle. Immer verfuegbar. |
| **Gemeldeter NC** | NC_reported | `mheardNCount[i]` | Wie viele Stationen hoert **mein Nachbar i**? Wird vom Nachbarn via HEY-Payload (`R<NC>;`) mitgeteilt. Nur verfuegbar wenn der Nachbar ein HEY gesendet hat. |

**Beispiel:** Ich habe NC_self=6 (ich hoere 6 Stationen). Einer meiner Nachbarn,
Station "OE1XYZ", meldet via HEY seinen NC_reported=12 — er hoert 12 Stationen.
Das heisst: OE1XYZ ist ein groeßerer Hub als ich.

Die **Netzwichtigkeit** (Importance) wird aus den NC_reported-Werten **meiner Nachbarn**
berechnet — nicht aus meinem eigenen NC_self.

---

## Kontext

### Problem: Kleine Nodes dominieren den Kanal

Im aktuellen MeshCom-Netz berechnet `csma_compute_timeout_prio()` den Backoff
ausschliesslich anhand der Nachrichten-Prioritaet und der Retry-Stufe — unabhaengig
davon, wie viele Nachbarn ein Node erreicht oder wie wichtig er fuer die Netzstruktur ist.

**Aktuelle CSMA-Parameter** (`configuration_global.h`):

| Prio | Typ | Base (ms) | Slots | Jitter (ms) | Backoff-Fenster |
|------|-----|-----------|-------|-------------|-----------------|
| 1 (Critical) | ACK, DM | 3000 | 10 | 0..350 | 3000..3350 |
| 2 (High) | Broadcast, Gruppe | 3000 | 10 | 0..350 | 3000..3350 |
| 3 (Normal) | **Relay** | 4500 | 10 | 0..350 | 4500..4850 |
| 4 (Low) | Position | 5500 | 10 | 0..350 | 5500..5850 |
| 5 (Background) | HEY | 5500 | 10 | 0..350 | 5500..5850 |

Slot-Groesse: 35ms (28ms CAD + 2ms TX-Switch + 5ms Safety)
Jitter: `random(0, slots + 1) × 35ms`

**Retry-Reduktion** (`attempt` = Anzahl fehlgeschlagener CAD-Versuche):

| Attempt | Reduktion | Relay-Base | Relay-Fenster |
|---------|-----------|-----------|---------------|
| 0 (Erstversuch) | keine | 4500ms | 4500..4850ms |
| 1 (1. Retry) | base × 5/6 (~17%) | 3750ms | 3750..4100ms |
| 2 (2. Retry) | base × 2/3 (~33%) | 3000ms | 3000..3350ms |
| ≥3 (Rapid-fire) | → CSMA_RAPID_RX_MS | 100ms | 100ms (Preamble-Check) |

**Problem:** Innerhalb des Relay-Bandes (Prio 3) gibt es keine Differenzierung.
Ein Blatt-Node (NC=1) hat die gleiche Backoff-Verteilung wie ein zentraler Hub (NC=12).
In der Praxis bedeutet das:

```
Nachricht M wird von 8 Nodes empfangen:

  Node A (NC=12, Hub)     → Backoff: 4637ms
  Node B (NC=3, klein)    → Backoff: 4522ms  ← sendet zuerst
  Node C (NC=1, Blatt)    → Backoff: 4589ms  ← sendet als zweiter

  B sendet → erreicht 3 Nodes → Kanal belegt
  C sendet → erreicht 1 Node  → Kanal belegt
  A sendet → erreicht 12 Nodes, aber Kanal war schon 2x belegt
```

**Resultat:** Der Hub, der mit EINER Transmission 12 Nodes versorgen koennte, kommt
erst dran, nachdem kleine Nodes den Kanal bereits mit Transmissions geringer Reichweite
belegt haben. Die bestehende Duplikat-Suppression kann nicht greifen, weil die kleinen
Nodes VOR dem Hub senden.

### Felddaten: BergLog 2026-03-13/14

Analyse von 5 Knoten (OE1KBC, OE1XIR, OE3MAG, OE3XOC, OE3XWJ) ueber ~16 Stunden,
856.000 Log-Zeilen, 138 eindeutige Rufzeichen im Netz.

#### Netzstruktur

| Knoten | NC (Neighbor Count) | Relay-Rate | Relay-TX Anzahl |
|--------|---------------------|-----------|-----------------|
| OE3MAG | 20* | 83% | 3.885 |
| OE3XOC | 20* | 82% | 2.742 |
| OE3XWJ | 14 | 79% | 2.145 |
| OE1KBC | 9 | 71% | 1.037 |
| OE1XIR | 20* | 74% | 472 |

\* **Achtung: NC=20 ist ein Pufferueberlauf.** In dieser Firmware-Version war
`MAX_MHEARD=20`, daher ist NC=20 der Maximalwert den der Ringpuffer speichern konnte.
Die tatsaechliche Nachbar-Anzahl dieser Knoten liegt deutlich hoeher — aus der
Log-Analyse lassen sich bis zu 150 eindeutige Nodes im Netz identifizieren. Die
NC-Werte in dieser Tabelle sind daher Untergrenzen, nicht exakte Werte.

**Hub-Knoten mit NC≥20 relayen 80% des gesamten Traffics.**

#### Relay-Lawine in Zahlen

| Metrik | Wert |
|--------|------|
| Unique Nachrichten (RX_DEDUP_NEW) | 16.032 |
| Duplikate (RX_DEDUP_DUP) | 25.089 |
| **DUP/NEW-Ratio** | **1,57** (jede Nachricht wird im Schnitt 2,57x pro Node empfangen) |
| CAD-Busy-Rate | **66%** — zwei Drittel aller TX-Versuche finden den Kanal belegt |
| Max. CAD-Retries beobachtet | 27 Versuche bis Kanal frei |
| Raw-RX-Buffer-Overflows | **47.376** — betrifft nur den Webserver-Anzeigepuffer fuer Rohpakete, kein Datenverlust im Mesh-Processing. Logging mittlerweile entfernt, da es ein falsches Bild vermittelte. |
| RELAY_SUPPRESS Events | **0** — Suppression war auf diesen Nodes nicht aktiv |

#### Zombie-Nachrichten durch Dedup-Ueberlauf

Der Dedup-Ring hat 60 Slots. Bei 4,2 neuen Nachrichten/Minute rotiert die Tabelle
alle ~6,5 Minuten. Langsame Multi-Hop-Relays (bis zu 13 Minuten beobachtet) kommen
nach der Dedup-Rotation an und werden als "neu" akzeptiert → erneut relayed.

**Worst Case:** ACK-Nachricht `32312D47` wurde 46x relayed und 52x als "neu" akzeptiert,
weil der Dedup-Ring sie vergessen hatte bevor sie aufhoerte zu bounzen.

#### Beispiel: Ein HEY-Beacon durchs Netz

OE3MAG sendet HEY (msg `9867002B`) um 21:02:13:

```
+4s    zurueck via OE3AOG (1 Hop)
+8s    zurueck via OE3GJC (1 Hop)
+10s   zurueck via OE1UTW (1 Hop)
+17s   zurueck via OE1KBC→OE3MIF (2 Hops)
+6:39  zurueck via OE3CZC (1 Hop, langsamer Pfad)
+13:01 zurueck via OE3CZC→OE3MIF→OE3CZC→OE1MVA (4 Hops!)
```

Ein einziges HEY-Beacon erzeugt mindestens 7 Relay-Kopien die ueber 13 Minuten
zum Originator zurueckkommen. 21% aller Nachrichten erschoepfen alle 4 Hops (H00).

#### Hop-Count-Verteilung

| Hops verbraucht | Anteil |
|-----------------|--------|
| 0 (Origin) | 10,5% |
| 1 Hop | 19,0% |
| 2 Hops | 26,8% |
| 3 Hops | 23,7% |
| 4 Hops (Maximum) | **20,7%** |

#### Fazit der Felddaten

1. **Hub-Knoten (NC=20) sind die groessten Relay-Verstaerker** — 80% Relay-Rate
2. **Ohne Importance-Differenzierung relayen alle Nodes gleich schnell** — Hubs
   gewinnen den Kanal nicht oefter als Blaetter
3. **Relay-Suppression war nicht aktiv** — die Kombination aus Importance-Backoff
   UND Suppression wurde noch nie im Feld getestet
4. **Dedup-Ring zu klein** — 60 Slots bei 4,2 msg/min erzeugt Zombie-Nachrichten
5. **66% CAD-Busy bestaetigt CSMA-Relevanz** — in einem so belasteten Kanal entscheidet
   die Slot-Position, welcher Node den Kanal zuerst belegt

### Warum NC_self allein nicht reicht

Ein Node mit NC_self=10 kann zwei voellig verschiedene Rollen haben:

- **Hub auf dem Berg:** NC_self=10, aber die Nachbarn melden NC_reported=1-2 →
  der Hub ist deren einzige Verbindung zum Netz
- **Node in der Stadt:** NC_self=10, aber die Nachbarn melden NC_reported=8-12 →
  der Node ist voellig redundant, seine Nachbarn erreichen sich problemlos ohne ihn

NC_self sagt nur "ich hoere X Stationen". Erst die **NC_reported-Werte der Nachbarn**
(via HEY-Payload empfangen) zeigen, ob diese Stationen auf uns angewiesen sind oder nicht.

---

## Entscheidung

### 1. Metrik: Netzwichtigkeit (Net Importance)

Die Netzwichtigkeit eines Knotens wird aus den **NC_reported-Werten seiner Nachbarn**
berechnet:

```
Wichtigkeit = Σ (1 / NC_reported[i])   fuer alle aktiven Nachbarn i
```

Nicht der eigene NC_self bestimmt die Wichtigkeit, sondern: **Wie abhaengig sind
meine Nachbarn von mir?** Das erfahren wir aus deren NC_reported — je kleiner deren
NC_reported, desto weniger Alternativen haben sie, desto mehr sind sie auf uns angewiesen.

#### Was die Formel ausdrueckt

`1 / NC_reported` ist der **Verantwortungsanteil** den ich fuer diesen Nachbarn trage:

| NC_reported des Nachbarn | Beitrag `1/NC_reported` | Bedeutung |
|--------------------------|------------------------|-----------|
| 1 | 1.000 | Nachbar hoert NUR mich → 100% Verantwortung |
| 2 | 0.500 | Nachbar hoert 1 weitere Station → 50% Verantwortung |
| 3 | 0.333 | Nachbar hat 2 Alternativen → 33% Verantwortung |
| 5 | 0.200 | Gut angebunden → 20% Verantwortung |
| 10 | 0.100 | Hochredundant → 10% Verantwortung |
| 20 | 0.050 | Sehr redundant → kaum Verantwortung |

Die Summe ergibt: **Wie viele "voll-abhaengige Knoten-Aequivalente" haengen an mir.**

Ein Wert von 5.0 bedeutet: "Es haengen effektiv 5 Knoten vollstaendig an mir."
Ein Wert von 0.5 bedeutet: "Meine Nachbarn haben genug Alternativen."

#### Nicht-Linearitaet ist gewuenscht

Der Sprung von NC=1 auf NC=2 (1.0 → 0.5, -50%) ist riesig — korrekt, denn der
Unterschied zwischen "einziger Weg" und "eine Alternative" ist fundamental.

Der Sprung von NC=10 auf NC=11 (0.100 → 0.091, -9%) ist vernachlaessigbar — korrekt,
denn bei 10 Alternativen macht eine mehr keinen Unterschied.

### 2. Topologie-Validierung

#### 2.1 Stern (Hub mit abhaengigen Blaettern)

```
B   C  D  E   F          (NC_self=1, melden NC_reported=1 via HEY)
 \  |  |  |  /
       A                (NC_self=5)
```

| Node | NC_self | Nachbarn und deren NC_reported | Importance | Rolle |
|------|---------|-------------------------------|------------|-------|
| A | 5 | B→1, C→1, D→1, E→1, F→1 | 1+1+1+1+1 = **5.0** | Hub, sendet zuerst |
| B | 1 | A→5 | 1/5 = **0.2** | Blatt, wartet |

Korrekt: A versorgt 5 abhaengige Knoten (alle melden NC_reported=1 → haengen nur an A).
B braucht nicht zu relayen, A hat das erledigt.

#### 2.2 Lineare Kette

```
          A --- B --- C --- D --- E
NC_self:  1     2     2     2     1
```

| Node | NC_self | Nachbarn → NC_reported | Importance | Reihenfolge |
|------|---------|----------------------|------------|-------------|
| A | 1 | B→2 | 1/2 = **0.50** | 4. |
| B | 2 | A→1, C→2 | 1/1 + 1/2 = **1.50** | 1. (teilt mit D) |
| C | 2 | B→2, D→2 | 1/2 + 1/2 = **1.00** | 3. |
| D | 2 | C→2, E→1 | 1/2 + 1/1 = **1.50** | 1. (teilt mit B) |
| E | 1 | D→2 | 1/2 = **0.50** | 4. |

Korrekt: B und D sind die Knoten direkt neben den isolierten Endpunkten (A meldet
NC_reported=1, E meldet NC_reported=1). B und D tragen die hoechste Verantwortung.

#### 2.3 Dichtes Mesh (alle sehen alle)

```
A--B--C
|\/|\/|        alle NC_self=5, alle melden NC_reported=5
|/\|/\|
D--E--F
```

| Node | NC_self | Nachbarn → NC_reported | Importance |
|------|---------|----------------------|------------|
| jeder | 5 | 5 Nachbarn, alle →5 | 5 × (1/5) = **1.0** |

Korrekt: Alle gleichwertig, maximale Redundanz. Kein Node ist wichtiger als ein anderer.

#### 2.4 Bridge zwischen zwei Clustern

```
Cluster 1:               Cluster 2:
A---B           X           D---E
 \ /           / \           \ /
  C       A,B,C   D,E,F       F

NC_self:  A,B,C=3    X=6    D,E,F=3
NC_reported (via HEY): alle melden ihren NC_self
```

| Node | NC_self | Nachbarn → NC_reported | Importance |
|------|---------|----------------------|------------|
| A (Cluster) | 3 | B→3, C→3, X→6 | 1/3 + 1/3 + 1/6 = **0.83** |
| X (Bridge) | 6 | A→3, B→3, C→3, D→3, E→3, F→3 | 6 × (1/3) = **2.0** |

Korrekt: X ist die Bridge und hat die hoechste Importance. Alle 6 Nachbarn von X
melden NC_reported=3 — sie haben wenig Alternativen und sind auf X angewiesen.

#### 2.5 Zwei Bridges (Redundanz)

Gleiche Topologie, aber mit X und Y als parallele Bridges:

```
Cluster-Nodes: NC_self=4 (2 Cluster-Kollegen + X + Y), melden NC_reported=4
X und Y:       NC_self=7 (3+3+1), melden NC_reported=7
```

| Node | Nachbarn → NC_reported | Importance |
|------|----------------------|------------|
| A (Cluster) | B→4, C→4, X→7, Y→7 | 1/4 + 1/4 + 1/7 + 1/7 = **0.79** |
| X (Bridge) | A→4, B→4, C→4, D→4, E→4, F→4, Y→7 | 6×(1/4) + 1/7 = **1.64** |

Korrekt: X ist immer noch die wichtigste, aber von 2.0 auf 1.64 gesunken.
Die Nachbarn melden jetzt NC_reported=4 statt 3 (sie haben ja durch Y eine
zusaetzliche Verbindung). Die Formel erkennt automatisch die neue Redundanz.

#### 2.6 Entscheidender Vergleich: Berg-Hub vs. Stadt-Node

Beide Nodes haben den gleichen NC_self=10. Der Unterschied liegt in den
**NC_reported-Werten ihrer Nachbarn**:

**Berg-Hub:** NC_self=10, Nachbarn sind Taeler/Almen die wenige Stationen hoeren

```
Nachbarn melden via HEY: NC_reported = 1, 1, 2, 1, 3, 2, 1, 2, 1, 3

Importance = 1/1 + 1/1 + 1/2 + 1/1 + 1/3 + 1/2 + 1/1 + 1/2 + 1/1 + 1/3
           = 1 + 1 + 0.5 + 1 + 0.33 + 0.5 + 1 + 0.5 + 1 + 0.33
           = 7.16
```

**Stadt-Node:** NC_self=10, Nachbarn sind alle gut vernetzt und melden hohe NC_reported

```
Nachbarn melden via HEY: NC_reported = 8, 10, 12, 9, 11, 8, 10, 9, 12, 10

Importance = 1/8 + 1/10 + 1/12 + 1/9 + 1/11 + 1/8 + 1/10 + 1/9 + 1/12 + 1/10
           = 0.125 + 0.1 + 0.083 + 0.111 + 0.091 + 0.125 + 0.1 + 0.111 + 0.083 + 0.1
           = 1.03
```

| | NC_self | NC_reported der Nachbarn | Importance | Backoff |
|---|---------|-------------------------|-----------|---------|
| Berg-Hub | 10 | 1, 1, 2, 1, 3, 2, 1, 2, 1, 3 | **7.16** | kurz → sendet zuerst |
| Stadt-Node | 10 | 8, 10, 12, 9, 11, 8, 10, 9, 12, 10 | **1.03** | lang → wartet |

Gleicher NC_self, aber 7x verschiedene Importance. Der Unterschied kommt ausschliesslich
aus den NC_reported-Werten. Der Berg-Hub hat Nachbarn die auf ihn angewiesen sind
(niedrige NC_reported), der Stadt-Node hat Nachbarn die genug Alternativen haben
(hohe NC_reported).

### 3. Mixed Mode: Nachbarn ohne NC_reported

#### Das Problem

Nicht alle Nachbarn haben bereits ein HEY gesendet. In `mheardNCount[i]` steht
dann `0` — das heisst: wir kennen deren NC_reported nicht. Das betrifft:

- Nodes mit alter Firmware (kein HEY-Support → senden kein NC_reported)
- Neue Nodes die gerade erst aufgetaucht sind (noch kein HEY empfangen)
- Nodes deren letztes HEY laenger als ein Trickle-Zyklus zurueckliegt

Wir koennen nicht unterscheiden zwischen "NC_reported=0 weil unbekannt" und einem
hypothetischen "NC_reported=0 weil keine Nachbarn" (der Fall existiert nicht —
wenn wir ihn hoeren, hat er mindestens NC_self=1).

#### Strategie: Konservativ (unbekannt = abhaengig)

```
Beitrag eines Nachbarn zur Wichtigkeit:
  NC_reported bekannt (> 0):    1 / NC_reported
  NC_reported unbekannt (== 0): 1.0  (Annahme: koennte isoliert sein, NC_self=1)
```

**Begruendung:** Im Zweifel nehmen wir an, dass der Nachbar auf uns angewiesen ist.
Das fuehrt zu einer erhoehten Importance → kuerzerer Backoff → wir relayen lieber
einmal zu viel als einmal zu wenig.

**Selbstkorrektur:** Sobald HEY-Daten eintreffen (spaetestens nach 30s bis 15min
durch Trickle), korrigiert sich der Wert nach unten.

#### Zahlenbeispiel

Node X hat 6 Nachbarn (NC_self=6). Von 3 hat er ein HEY empfangen, von 3 nicht:

```
NC_reported bekannt:   A→5, B→3, C→8
NC_reported unbekannt: D→?, E→?, F→?

Importance = 1/5 + 1/3 + 1/8 + 1.0 + 1.0 + 1.0
           = 0.2 + 0.33 + 0.125 + 1.0 + 1.0 + 1.0
           = 3.66   (konservativ hoch)

Spaeter, nach HEY-Empfang von D, E, F: D→7, E→4, F→10

Importance = 1/5 + 1/3 + 1/8 + 1/7 + 1/4 + 1/10
           = 0.2 + 0.33 + 0.125 + 0.143 + 0.25 + 0.1
           = 1.15   (korrigiert)
```

Der Wert sinkt von 3.66 auf 1.15 sobald die NC_reported-Daten vorliegen.
Im Uebergangszustand sendet der Node "zu frueh" — das ist sicher, nur nicht optimal.

### 4. Slot-basierte Importance-Differenzierung

#### 4.1 Grundprinzip: Base bleibt, Slots differenzieren

Zwei getrennte Achsen bestimmen den Relay-Backoff:

| Achse | Bestimmt durch | Zweck |
|-------|---------------|-------|
| **Base** (ms) | Retry-Attempt (0, 1, 2) | Fairness: wer laenger wartet, kommt frueher dran |
| **Slot-Position** | Importance | Netzwichtigkeit: wer wichtiger ist, bekommt vordere Slots |

**Die Base wird NICHT veraendert.** Sie bleibt bei 4500ms fuer alle Relays.
Die bestehende Retry-Reduktion (×5/6, ×2/3) bleibt der Mechanismus fuer Nodes
die laenger warten mussten.

**Nur die Slot-Position** wird durch die Importance bestimmt:
- Wichtiger Node → vordere Slots (0..2) → kuerzerer Jitter → sendet frueher
- Unwichtiger Node → hintere Slots (7..9) → laengerer Jitter → sendet spaeter

#### 4.2 Slot-Bereich-Berechnung

Innerhalb des Relay-Bandes stehen 10 Slots zur Verfuegung. Jeder Node bekommt
ein Fenster von 3 aufeinanderfolgenden Slots, dessen Position von der Importance
abhaengt:

```
RELAY_TOTAL_SLOTS  = 10   // Gesamtanzahl Slots (0..9)
RELAY_JITTER_WIDTH = 3    // Jeder Node waehlt aus 3 aufeinanderfolgenden Slots
IMP_CAP            = 8.0  // Importance-Obergrenze

imp_ratio  = min(importance, IMP_CAP) / IMP_CAP          // 0.0 .. 1.0
slot_start = (int)((1.0 - imp_ratio) * (RELAY_TOTAL_SLOTS - RELAY_JITTER_WIDTH))
             //    ↑ invertiert: hohe Importance → niedriger slot_start

slot = slot_start + random(0, RELAY_JITTER_WIDTH)         // 3 moegliche Slots
backoff = base + slot × CSMA_SLOT_SIZE                    // base = 4500ms (Attempt 0)
```

#### 4.3 Slot-Zuordnung nach Importance

| Importance | imp_ratio | slot_start | Slot-Bereich | Jitter (ms) | Backoff (Att.0) |
|-----------|-----------|-----------|-------------|-------------|-----------------|
| 8.0 (max/cap) | 1.00 | 0 | 0, 1, 2 | 0..70 | 4500..4570 |
| 7.2 (Berg-Hub) | 0.90 | 0 | 0, 1, 2 | 0..70 | 4500..4570 |
| 5.0 (Stern-Hub) | 0.63 | 2 | 2, 3, 4 | 70..140 | 4570..4640 |
| 2.0 (Bridge) | 0.25 | 5 | 5, 6, 7 | 175..245 | 4675..4745 |
| 1.0 (Cluster) | 0.13 | 6 | 6, 7, 8 | 210..280 | 4710..4780 |
| 0.5 (kleiner Node) | 0.06 | 6 | 6, 7, 8 | 210..280 | 4710..4780 |
| 0.2 (Blatt) | 0.03 | 6 | 6, 7, 8 | 210..280 | 4710..4780 |

**Beobachtungen:**

- **Berg-Hub vs. Bridge:** Minimum 105ms Vorsprung (Slot 2 vs. Slot 5). Der Hub
  startet sein CAD bevor die Bridge ueberhaupt ihren Timer abgelaufen hat.
- **Hubs unter sich:** Berg-Hub und Stern-Hub haben leicht ueberlappende Bereiche
  (Slot 2 ist in beiden). Das ist gewollt — aehnlich wichtige Nodes sollen sich
  per Zufall entzerren, nicht deterministisch blockieren.
- **Kleine Nodes:** Cluster, kleine Nodes und Blaetter landen alle in Slots 6..8.
  Das ist korrekt — sie sind alle "unwichtig" und sollen alle erst senden nachdem
  die Hubs fertig sind. Untereinander entzerren sie sich per Zufall.

#### 4.4 Interaktion mit Retry-Reduktion

Die Retry-Reduktion veraendert **nur die Base**, die Slot-Position bleibt gleich:

```
Attempt 0:  base = 4500ms                (Erstversuch)
Attempt 1:  base = 4500 × 5/6 = 3750ms   (1. Retry, ~17% schneller)
Attempt 2:  base = 4500 × 2/3 = 3000ms   (2. Retry, ~33% schneller)
Attempt ≥3: 100ms                         (Rapid-fire, wie bisher)
```

**Vollstaendige Backoff-Tabelle (Relay, mit Slot-Bereich):**

| Importance | Slots | Attempt 0 | Attempt 1 (base 3750) | Attempt 2 (base 3000) |
|-----------|-------|-----------|----------------------|----------------------|
| 7.2 (Berg-Hub) | 0..2 | 4500..4570ms | 3750..3820ms | 3000..3070ms |
| 5.0 (Stern-Hub) | 2..4 | 4570..4640ms | 3820..3890ms | 3070..3140ms |
| 2.0 (Bridge) | 5..7 | 4675..4745ms | 3925..3995ms | 3175..3245ms |
| 1.0 (Cluster) | 6..8 | 4710..4780ms | 3960..4030ms | 3210..3280ms |
| 0.2 (Blatt) | 6..8 | 4710..4780ms | 3960..4030ms | 3210..3280ms |

**Wichtige Eigenschaft:** Die Slot-Differenzierung bleibt bei Retries **konstant**.
Der Vorsprung des Hubs gegenueber dem Blatt ist immer ~210ms (Slot 0 vs. Slot 6),
unabhaengig vom Attempt. Das ist der entscheidende Unterschied zum alten Ansatz
mit variabler Base — dort schrumpfte die Differenzierung bei Retries.

**Band-Separation:** Bei Attempt 2 liegt der Hub bei 3000..3070ms. Das ist
am unteren Rand des Broadcast-Bandes (3000ms), aber das ist **bestehendes Verhalten** —
schon heute landen Relays bei Attempt 2 in diesem Bereich (3000..3350ms).
Die Importance-Aenderung verschaerft das nicht.

#### 4.5 Zahlenbeispiel: Kaskadeneffekt

```
Nachricht M wird von 5 Nodes empfangen (Attempt 0):

Node A: NC_self=10, NC_reported der Nachbarn=1-2 → Imp=7.2 → Slots 0..2  → 4500..4570ms
Node B: NC_self=6,  NC_reported=3-5              → Imp=1.5 → Slots 6..8  → 4710..4780ms
Node C: NC_self=3,  NC_reported=5-8              → Imp=0.5 → Slots 6..8  → 4710..4780ms
Node D: NC_self=1,  NC_reported=10               → Imp=0.1 → Slots 6..8  → 4710..4780ms
Node E: NC_self=2,  NC_reported=3,8              → Imp=0.5 → Slots 6..8  → 4710..4780ms

t=0ms      Nachricht M empfangen
t=4500ms   Node A (Slot 0) sendet Relay → erreicht 10 Nachbarn
t=4900ms   Nodes B,C,D,E hoeren Duplikat → bestehende Suppression kann greifen
t=4745ms   Node B: wenn nicht supprimiert → sendet Relay (Slot 7)
t=4780ms   Nodes C,E: fast sicher supprimiert (2+ Duplikate gehoert)

Ergebnis: 1-2 Transmissions statt 5
```

**Slot-Backoff und Duplikat-Suppression verstaerken sich gegenseitig:**

1. Slot-Position sorgt dafuer, dass die wichtigen Nodes **zuerst** senden
2. Ihre Transmissions erzeugen Duplikate bei den weniger wichtigen Nodes
3. Duplikat-Suppression storniert die Relays der weniger wichtigen Nodes
4. Ohne Importance-Slots senden die kleinen Nodes zuerst → Suppression greift beim
   Hub → falsche Richtung!

#### Warum 210ms im CSMA-Kontext ausreichen

Der Slot-Vorsprung von 210ms (6 Slots) wirkt nicht wie ein TDMA-Zeitfenster, sondern
als **CSMA/CAD-Prioritaet**. Der entscheidende Mechanismus:

```
t=4500ms   Hub (Slot 0) macht CAD → Kanal FREI → startet Transmission
t=4535ms   Hub sendet (Paket dauert 300-1000ms, Kanal belegt)
t=4710ms   Blatt (Slot 6) macht CAD → Kanal BUSY → wartet (Attempt 1)
t=4800ms   Hub beendet Transmission
t=4810ms   Blatt macht CAD erneut → Kanal FREI
           ABER: Blatt hat inzwischen Hub's Relay als Duplikat empfangen
           → Suppression storniert das eigene Relay → kein TX
```

Der Hub muss seinen TX nicht innerhalb der 210ms **abschliessen**. Er muss nur
innerhalb der 210ms sein CAD durchfuehren und mit der Transmission **beginnen**.
Ab diesem Zeitpunkt blockiert CSMA/CAD automatisch alle spaeter startenden Nodes.

**Felddaten bestaetigen dies:** 66% CAD-Busy-Rate bedeutet, dass Nodes den Grossteil
ihrer Zeit damit verbringen, auf einen freien Kanal zu warten. Ein struktureller
Vorsprung von 6 CAD-Slots (210ms) gibt dem Hub praktisch eine Garantie, den Kanal
vor den Blaettern zu belegen.

Die Slot-Differenzierung ist damit **kein Wettrennen auf Millisekunden**, sondern
ein **Vorfahrts-System**: der Hub geht als erster an die Kreuzung, und CSMA regelt
den Rest.

#### 4.6 Review: Slot-Anzahl der anderen Priority-Baender

Die Einfuehrung von 10 Relay-Slots erfordert eine Ueberpruefung der Slot-Anzahlen
aller Priority-Baender:

| Prio | Typ | Slots aktuell | Begruendung |
|------|-----|---------------|-------------|
| 1 (Critical) | ACK, DM | 10 | Einheitlich 10 Slots = 11 Positionen. Ausreichend fuer ACK-Kollisionsvermeidung. |
| 2 (High) | Broadcast | 10 | Einheitlich 10 Slots. |
| 3 (Normal) | Relay | 10 | Durch Importance-Fenster (3er-Breite) in Subbereiche unterteilt. |
| 4 (Low) | Position | 10 | Einheitlich 10 Slots. |
| 5 (Background) | HEY | 10 | Einheitlich 10 Slots. |

Alle Priority-Baender verwenden einheitlich 10 Slots. Die Differenzierung
erfolgt ausschliesslich ueber die Base-Timeouts (3000/3000/4500/5500/5500ms).

### 5. Betroffener Code

#### 5.1 Neue Konstanten

**Datei:** `src/configuration_global.h`

```cpp
// NC-Importance Relay-Slot-Steuerung
#define RELAY_IMP_CAP            8    // Importance-Obergrenze fuer Slot-Mapping
#define RELAY_TOTAL_SLOTS       10    // Gesamtanzahl Relay-Slots (0..9)
#define RELAY_JITTER_WIDTH       3    // Breite des Slot-Fensters pro Node
```

#### 5.2 Importance-Berechnung

**Datei:** `src/mheard_functions.cpp` (neue Funktion)

```cpp
/**
 * Berechne Netzwichtigkeit als Summe der inversen NC_reported-Werte.
 * Hoher Wert = viele abhaengige Nachbarn = Node ist wichtig fuers Netz.
 *
 * NC_reported = mheardNCount[i], vom Nachbarn via HEY gemeldet.
 * NC_reported == 0 bedeutet: kein HEY empfangen → konservativ als 1 behandeln.
 */
float getNetImportance()
{
    float importance = 0.0f;
    unsigned long now = getUnixClock();

    for(int i = 0; i < MAX_MHEARD; i++)
    {
        if(mheardCalls[i][0] != 0x00)
        {
            if((mheardEpoch[i] + 60*60*12) > now)   // aktiv (letzte 12h)
            {
                int nc_reported = mheardNCount[i];   // NC_reported des Nachbarn
                if(nc_reported > 0)
                    importance += 1.0f / (float)nc_reported;
                else
                    importance += 1.0f;              // unbekannt → konservativ
            }
        }
    }
    return importance;
}
```

#### 5.3 Anpassung CSMA-Backoff

**Datei:** `src/lora_functions.cpp` — Funktion `csma_compute_timeout_prio()`

Nur der `MSG_PRIO_NORMAL`-Case (Relays) wird angepasst.
Die Base bleibt `CSMA_PRIO_BASE_3` (4000ms), nur die Slot-Berechnung aendert sich:

```cpp
case MSG_PRIO_NORMAL:   // Relay
{
    float imp = getNetImportance();
    float imp_capped = (imp > (float)RELAY_IMP_CAP) ? (float)RELAY_IMP_CAP : imp;
    float imp_ratio = imp_capped / (float)RELAY_IMP_CAP;   // 0.0 .. 1.0

    base = CSMA_PRIO_BASE_3;   // 4500ms — unveraendert!

    // Importance bestimmt Slot-Position: hohe Imp → vordere Slots
    int slot_start = (int)((1.0f - imp_ratio)
                     * (float)(RELAY_TOTAL_SLOTS - RELAY_JITTER_WIDTH));
    int slot = slot_start + (int)random(0, RELAY_JITTER_WIDTH);

    if(bDisplayInfo || bLORADEBUG)
        Serial.printf("[MC-IMP] imp=%.2f slot_start=%d slot=%d\n",
            imp, slot_start, slot);

    // Retry-Reduktion auf Base (wie bisher)
    if(attempt >= 2) base = base * 2 / 3;
    else if(attempt >= 1) base = base * 5 / 6;

    return base + (unsigned long)slot * CSMA_SLOT_SIZE;
}
```

**Alle anderen Priority-Cases bleiben unveraendert.** ACKs, eigene Nachrichten,
Position, HEY — alles wie bisher. Die Retry-Reduktion und Rapid-fire-Logik
(Attempt ≥3 → 100ms) bleiben ebenfalls unveraendert.

### 6. Zusammenspiel mit bestehenden Mechanismen

| Mechanismus | Funktion | Aenderung |
|-------------|----------|-----------|
| CSMA/CAD | Channel-Sensing vor TX | Keine |
| Priority-Queue | ACK > Text > Relay > Pos > HEY | Keine |
| Dedup-Ring | Duplikat-Erkennung | Keine |
| Relay-Suppression | Relay stornieren nach N Duplikaten | Keine, aber **zwingende Abhaengigkeit**: Suppression darf erst NACH Importance-Backoff aktiviert werden. Ohne korrekte Sendereihenfolge storniert Suppression die Hubs (hoechste Duplikat-Rate) statt die Blaetter → abhaengige Nachbarn werden ausgehungert. Mit Importance-Backoff senden Hubs zuerst → Suppression storniert korrekt die redundanten Blaetter. |
| Trickle-HEY | Adaptive HEY-Intervalle | Keine |
| **Neu: NC-Importance-Slots** | Relay-Slot-Position proportional zur Netzwichtigkeit | Nur `csma_compute_timeout_prio()`, Base unveraendert |

### 7. Laufzeitkosten

| Operation | Aufwand | Haeufigkeit |
|-----------|---------|------------|
| `getNetImportance()` | O(MAX_MHEARD) = 30-120 Iterationen, Float-Addition | Pro Relay-Entscheidung |
| Float-Berechnung | 3 Multiplikationen, 1 Division | Pro Relay-Entscheidung |
| Gesamt | <1ms auf ESP32 | Nicht zeitkritisch (vor Backoff-Timer) |

RAM: Keine zusaetzlichen Variablen. Nutzt bestehende `mheardNCount[]` und `mheardEpoch[]`.

---

## Risiko-Analyse

### Topologie-Sicherheit

Base = 4500ms fuer alle. Slot-Position bestimmt Jitter (0..315ms bei 10 Slots × 35ms).

| Topologie | NC_self | NC_reported | Imp | Slots | Backoff (Att.0) | Verhalten |
|-----------|---------|-------------|-----|-------|-----------------|-----------|
| Berg-Hub | 10 | 1,1,2,1,3,2,1,2,1,3 | 7.2 | 0..2 | 4500..4570ms | Sendet zuerst — korrekt |
| Stern-Hub | 5 | alle →1 | 5.0 | 2..4 | 4570..4640ms | Sendet frueh — korrekt |
| Bridge | 6 | alle →3 | 2.0 | 5..7 | 4675..4745ms | Nach Hubs, vor Blaettern — korrekt |
| Full-Mesh | 10 | alle →10 | 1.0 | 6..8 | 4710..4780ms | Wartet — korrekt |
| Kette-Mitte | 2 | →2, →2 | 1.0 | 6..8 | 4710..4780ms | Wie Full-Mesh — korrekt |
| Stadt-Node | 10 | alle →8-12 | 1.0 | 6..8 | 4710..4780ms | Wartet — korrekt |
| Stern-Blatt | 1 | →5 | 0.2 | 6..8 | 4710..4780ms | Wartet — korrekt |

### Risiko: Float auf ESP32

ESP32 hat Hardware-FPU. `getNetImportance()` fuehrt maximal 30-120 Float-Additionen
durch. Laufzeit <0.1ms. Kein Risiko.

**Alternative (Integer-only):** Falls Float unerwuenscht, kann die Berechnung
ganzzahlig approximiert werden:

```cpp
// Integer-Variante: importance × 100 als Festkomma
int getNetImportanceFixed()
{
    int importance_x100 = 0;
    for(...) {
        if(nc > 0)
            importance_x100 += 100 / nc;   // ganzzahlige Division
        else
            importance_x100 += 100;
    }
    return importance_x100;
}
```

### Risiko: Importance-Wert veraltet

Nachbar-NC-Werte koennen bis zu 15 Minuten alt sein (Trickle-Imax).
In der Praxis aendert sich die Topologie selten so schnell, dass dies ein
Problem darstellt. Bei Topologie-Aenderungen resettet Trickle auf 30s, sodass
neue NC-Werte schnell verbreitet werden.

### Risiko: Alle Nodes unbekannt (Kaltstart)

Direkt nach dem Einschalten sind alle Nachbar-NCs unbekannt (== 0).
Jeder Nachbar traegt `1.0` bei → Importance = NC_self.

Beispiel: Node mit NC_self=5, alle unbekannt → Importance=5.0 → Slots 2..4.

Das ist akzeptabel: Im Kaltstart lieber zu wichtig einschaetzen als zu unwichtig.
Nach dem ersten Trickle-Zyklus (30s) korrigiert sich der Wert.

---

## Alternativen (verworfen)

### 1. Backoff nur auf NC_self

Jeder Node kennt seinen NC_self immer. Kein Mixed-Mode-Problem.

**Verworfen weil:** NC_self unterscheidet nicht zwischen Hub-auf-Berg (NC_self=10,
Nachbarn melden NC_reported=1) und redundantem Stadt-Node (NC_self=10, Nachbarn melden
NC_reported=10). Genau dafuer haben wir NC_reported via HEY eingefuehrt.

### 2. Inverse Backoff (niedrigster NC sendet zuerst)

Blatt-Knoten als Bridges priorisieren.

**Verworfen weil:** Bridges haben typischerweise hohe NC-Werte, nicht niedrige.
Eine Bridge zwischen 2 Clustern mit je 5 Nodes hat NC=10, nicht NC=2.
Blaetter mit NC=1 sind keine Bridges — sie sind Endpunkte.

### 3. Rollenbasierte Adaption (Hub/Edge auto-detection)

Vergleich eigener NC mit Durchschnitt der Nachbar-NCs.

**Verworfen weil:** Ueberkomplex. Die Importance-Formel liefert die gleiche
Information direkt und ohne kuenstliche Rollenklassifikation. Edge-Detection
ist zudem instabil bei wenig NC-Daten.

### 4. Suppression-Schwellen anpassen statt Backoff

Anstatt die Sendereihenfolge zu aendern, die Suppression aggressiver machen.

**Verworfen weil:** Suppression ist reaktiv (wartet auf Duplikate). Wenn die
falschen Nodes zuerst senden, greifen Duplikate beim Hub statt bei den Blaettern.
Die richtige Reihenfolge muss VOR der Suppression hergestellt werden.

### 5. Probabilistisches Relay (Wuerfeln statt Ordnen)

Jeder Node entscheidet per Zufall ob er relayed: `P(relay) = k / NC_self`.
Bei NC=20 und k=3 relayed ein Node mit 15% Wahrscheinlichkeit. Erwartung: 3 statt 20
Relays pro Nachricht.

**Verworfen weil:** Probabilistisches Relay ist **topologie-blind** und kann
abhaengige Knoten aushungern:

```
Ich (NC=20, P(relay) = 15%)
  ├── 19 Nachbarn mit NC=15-20 — brauchen mein Relay nicht
  └── 1 Blatt-Node (NC=1) — hoert NUR mich

Ich wuerfle: 85% Chance dass ich NICHT relay → Blatt bekommt nichts.
```

Schlimmer noch: Der "Blatt-Node" mit NC=1 koennte eine **Bridge** sein, die auf der
anderen Seite 10 weitere Knoten versorgt. Wenn ich nicht relay, verliert ein ganzer
Cluster die Nachricht.

Das Grundproblem: `P = k/NC` behandelt alle Nachbarn als austauschbar und gleichwertig
redundant. In der Realitaet haengt an einem einzigen Nachbarn mit NC_reported=1 eine
Verantwortung von 100%, waehrend 19 Nachbarn mit NC_reported=20 zusammen nur 5%
Verantwortung tragen. Probabilistisches Relay kann das nicht abbilden — die
Importance-Formel `Σ(1/NC_reported)` dagegen schon.

### 6. Importance veraendert die Base statt die Slots

Wichtige Nodes bekommen eine niedrigere Base (z.B. 3200ms statt 4000ms).

**Verworfen weil:** Die Base-Reduktion bei Retries (×5/6, ×2/3) ist der
Fairness-Mechanismus fuer Nodes die laenger warten mussten. Wenn die Importance
die Base verschiebt, vermischt das zwei verschiedene Konzepte:
- **Base** = "wie dringend" (Wartezeit, Retry-Fairness)
- **Slot** = "wie wichtig" (Netzwichtigkeit)

Die saubere Trennung: Base bleibt fuer alle gleich, nur die Slot-Position
wird durch Importance bestimmt. So bleibt die Retry-Reduktion wirksam und
die Importance-Differenzierung ist bei jedem Attempt gleich stark.

---

## Voraussetzungen und Deployment-Reihenfolge

Die drei Mechanismen (Importance-Backoff, Relay-Suppression, Dedup-Erweiterung)
haben eine **zwingende Abhaengigkeit** in der Aktivierungsreihenfolge.

### Warum Suppression OHNE Importance gefaehrlich ist

Die NC-basierte Relay-Suppression (commit 60ea7d8) storniert ein gequeuetes Relay
wenn genuegend Duplikate von anderen Relayern gehoert werden. Ohne Importance-Backoff
haben alle Nodes die gleiche Backoff-Verteilung — der Bergknoten mit NC=20 hoert
**mehr Duplikate als jeder andere**, weil er die meisten Nachbarn hat:

```
Nachricht M wird von 20 Nodes empfangen (ohne Importance):

  Berg-Hub (NC=20):   Backoff 4137ms (zufaellig)
  Tal-Node A (NC=3):  Backoff 4022ms → sendet zuerst
  Tal-Node B (NC=5):  Backoff 4089ms → sendet als zweiter
  Tal-Node C (NC=2):  Backoff 4105ms → sendet als dritter

  Berg-Hub hoert 3 Duplikate → Suppression storniert sein Relay!

  ABER: Auf der anderen Seite des Berges haengt ein Blatt-Node (NC=1),
  der NUR den Berg-Hub hoert. Dieses Blatt bekommt die Nachricht nie.
  Schlimmer: Das "Blatt" koennte eine Bridge mit 10 Nodes dahinter sein.
```

**Das Problem:** Der Bergknoten ist das wahrscheinlichste Opfer der Suppression,
weil er am schnellsten genuegend Duplikate sammelt. Aber er ist gleichzeitig der
Knoten, dessen Relay am wichtigsten ist — abhaengige Nachbarn haben keine Alternative.

Suppression allein ist **topologie-blind**: sie sieht nur "ich habe genug Duplikate
gehoert", aber nicht "wer von meinen Nachbarn hat die Nachricht noch NICHT".

### Stufe 1: NC-Importance-Backoff (dieses ADR)

Importance-Backoff muss **zuerst** aktiviert werden. Er sorgt dafuer, dass der
Bergknoten (hohe Importance) in den vorderen Slots (0..2) landet und den Kanal
via CSMA/CAD belegt, **bevor** er Duplikate hoert. Sein Relay wird gesendet,
nicht storniert.

**Allein bringt Importance-Backoff** bereits einen Nutzen: die wichtigsten Relays
passieren zuerst, der Hub versorgt seine abhaengigen Nachbarn zuverlaessig. Die
weniger wichtigen Nodes senden etwas spaeter — die Gesamtzahl der Relays sinkt
noch nicht, aber die **Reihenfolge** ist korrekt.

### Stufe 2: NC-basierte Relay-Suppression (commit 60ea7d8)

**Status:** Implementiert und wieder entfernt. Feldtest zeigte: Leaf Nodes mit nur
einer Gegenstelle haben oft keine Nachrichten erhalten. Bestaetigt die Analyse oben —
Suppression ohne Importance-Backoff storniert die falschen Nodes (Hubs statt Blaetter),
wodurch abhaengige Knoten ausgehungert werden.

**Erst mit aktivem Importance-Backoff ist Suppression sicher.** Der Hub sendet zuerst
(Stufe 1). Die Blaetter und redundanten Nodes senden spaeter und hoeren dabei den
Hub als Duplikat → Suppression storniert ihre Relays → korrekte Richtung.

Ohne Stufe 1 wuerde Suppression den Hub stornieren (falsche Richtung).

### Stufe 3: Dedup-Ring vergroessern

**Aktuell:** `MAX_DEDUP_RING = 60` Slots, rotiert bei 4,2 msg/min alle ~6,5 Minuten.

**Problem:** Multi-Hop-Relays koennen bis zu 13 Minuten brauchen (Felddaten). Nach
Dedup-Rotation wird die Nachricht als "neu" akzeptiert → Zombie-Relay.

**Empfehlung:** `MAX_DEDUP_RING` auf 200-256 erhoehen (~50 Minuten Speicher). Kostet
~1 KB RAM, eliminiert Zombie-Nachrichten. Ohne diese Aenderung werden Importance-Backoff
und Suppression durch Zombie-Relays teilweise unterlaufen.

### Zusammenfassung der Abhaengigkeiten

```
Stufe 1: Importance-Backoff     → Hubs senden zuerst (Reihenfolge korrigiert)
         ↓ Voraussetzung fuer
Stufe 2: Relay-Suppression      → Blaetter stornieren (Relay-Anzahl sinkt)
         ↓ wird unterlaufen ohne
Stufe 3: Dedup-Ring erhoehen    → Zombie-Nachrichten eliminiert
```

**Stufe 2 DARF NICHT ohne Stufe 1 aktiviert werden.** Suppression ohne korrekte
Reihenfolge storniert die falschen Nodes und hungert abhaengige Blaetter/Bridges aus.

---

## Offene Punkte

### Deployment-Reihenfolge (blockierend)
- [ ] **Stufe 1 — Importance-Backoff (dieses ADR) zuerst deployen.** Muss VOR Suppression aktiv sein, da Suppression ohne korrekte Sendereihenfolge die falschen Nodes (Hubs) storniert und abhaengige Blaetter/Bridges aushungert.
- [ ] **Stufe 2 — Relay-Suppression (commit 60ea7d8) erst NACH Stufe 1 aktivieren.** War implementiert, wurde wieder entfernt: Leaf Nodes mit nur einer Gegenstelle erhielten keine Nachrichten. Bestaetigt die Abhaengigkeit — Suppression darf erst mit aktivem Importance-Backoff reaktiviert werden.
- [ ] **Stufe 3 — Dedup-Ring vergroessern:** `MAX_DEDUP_RING` von 60 auf 200-256 erhoehen. Aktuell rotiert der Ring alle ~6,5 Minuten — Zombie-Nachrichten (bis 13 Min Multi-Hop) unterlaufen sowohl Suppression als auch Importance-Backoff.

### Parameter-Tuning
- [ ] IMP_CAP-Wert festlegen: 8.0 ist Schaetzung, sollte an realen Topologien validiert werden. BergLog-Daten zeigen NC=20 als typischen Hub-Wert — bei Σ(1/NC_reported) mit gemischten Nachbarn koennte IMP_CAP hoeher angesetzt werden.
- [ ] RELAY_JITTER_WIDTH: 3 Slots Fensterbreite — gut fuer Entzerrung? Oder 2 fuer schaerfere Trennung?
- [ ] Float vs. Integer: ESP32 hat Hardware-FPU, aber Coding-Style im Projekt ist ueberwiegend Integer

### Funktionale Erweiterungen
- [ ] Soll Importance auch fuer Position-Relays gelten, oder nur Text/Broadcast-Relays?
- [ ] Logging: Soll Importance periodisch im Web-UI oder auf der seriellen Konsole angezeigt werden?
- [x] **Critical-Slots erhoehen:** Erledigt — alle Priority-Baender verwenden jetzt einheitlich 10 Slots.
- [x] **Background-Slots reduzieren:** Erledigt — alle Priority-Baender verwenden jetzt einheitlich 10 Slots.

### Feld-Validierung
- [ ] **BergLog-Wiederholung mit aktiver Suppression + Importance:** Die BergLog-Daten (2026-03-13/14) wurden ohne Suppression und ohne Importance aufgezeichnet. Erst ein erneuter Test mit beiden Mechanismen zeigt den kombinierten Effekt.
- [ ] **Metriken fuer Vergleich definieren:** DUP/NEW-Ratio, CAD-Busy-Rate, Relay-TX-Anzahl pro Knoten, Zombie-Nachricht-Zaehler, ACK-Laufzeit. Zielwerte: DUP/NEW < 0.5, CAD-Busy < 30%, keine Zombies.
