# PN-Wiederholung Variante a) (XOR): Änderungen am zentralen MeshCom-Server

- Bezug: `docs/pn-zustellung-dedup.md` (Konzeptpapier, Kap. 2.4, 3, 4.2, 4.3, 7.1, 7.5, 7.6),
  `docs/pn-retry-xor-impl-plan.md` (Umsetzung in der Firmware).
- Firmware-Stand: `dk5en-xor`, Verweise ohne Zusatz beziehen sich auf diesen Zweig.
- **Der Quelltext des zentralen MeshCom-Servers liegt nicht vor.** Jede Aussage über sein heutiges
  Verhalten in diesem Dokument ist deshalb ausdrücklich als **Annahme** gekennzeichnet. Die
  Code-Beispiele sind Vorschläge in C99 ohne externe Abhängigkeiten (Standardbibliothek: `stdint.h`,
  `stdbool.h`, `string.h`, `time.h`) — keine lauffähige Implementierung, sondern eine Vorlage für das
  Team, das den Server betreut.
- Schreibweise: die PN-Nummer steht APRS-konform als `Text{NNN` am Ende des Textes, **ohne**
  schließende Klammer.

## 1. Kurzfassung

**Der Server muss vor der ersten Absender-Firmware angepasst werden** (Konzeptpapier 4.1, 7.5),
sonst strahlt jedes Gateway jede Wiederholung in sein LoRa-Netz aus (Kap. 4). Die Änderung zerfällt
in drei Aufgaben:

1. **Dedup-Schlüssel**: einer PN und all ihren Wiederholungen als eine PN erkennen, über
   `msg_id & 0xFFFFF3FFu` statt über die volle 32-Bit-msg_id. 30 Bit sind stabil: 20 Bit
   Knotenkennung in Bit 12–31 + 10 Bit laufende Nummer in Bit 0–9. Nur die Bits 10–11 ändern sich
   zwischen Erstsendung und Wiederholung (Kap. 3).
2. **Weiterreichen an Gateways entscheiden**: nicht jede Wiederholung ungefiltert an jedes Gateway
   geben, sonst wird jede Region neu geflutet (Kap. 4).
3. **APRS-IS-Gate**: jede PN höchstens einmal ausgeben, Text unverändert, ohne Wiederholungsmarke
   (Kap. 5).

**Was sich dafür am Funk ändert** (Firmware `dk5en-xor`, zur Einordnung des Eingabeformats):

- Die Erstsendung einer PN ist byte-identisch zu heute.
- Wiederholung k (1, 2, 3) trägt in Bit 10–11 der msg_id die Original-Bits XOR k; die restlichen
  30 Bit, `{NNN` im Text und der restliche Text bleiben gleich. Die FCS wird für jede Wiederholung neu
  berechnet.
- Das gilt **nur für PN** (Text an ein einzelnes Rufzeichen, das mit `{` gefolgt von 1–5 Ziffern ohne
  schließende Klammer endet). Gruppen und `*` senden wie heute mit unveränderter msg_id.
- Bis zu 4 Aussendungen im Abstand von rund 40 s (Erstsendung + 3 Wiederholungen, Gesamtspanne
  ~120 s).
- Eine Wiederholung einer bereits zugestellten PN kann über ein Gateway eintreffen, das die
  Erstsendung nie gehört hat (asymmetrische Funkstrecke, anderes Relais). Der Server darf daraus
  nicht schließen, dass es sich um eine neue PN handelt.

## 2. Wie der Server die msg_id aus einem Gateway-Upload liest

Ein Gateway lädt jede empfangene LoRa-Meldung mit einem festen `DATA`-Kopf hoch, gefolgt vom rohen
LoRa-Frame unverändert (`addNodeData()`, `src/udp_functions.cpp:1843`):

```
snprintf(data_buffer, sizeof(data_buffer),
         "DATA%08X%-9.9s%-4.4s%-1.1s%4i%4i03",
         _GW_ID, meshcom_settings.node_call, SOURCE_VERSION,
         SOURCE_VERSION_SUB, rssi, (int)snr);
```

Der Kopf hat feste Länge: `"DATA"`(4) + Gateway-ID hex(8) + Rufzeichen(9) + Version(4) + Unterversion(1)

- RSSI(4) + SNR(4) + `"03"`(2) = **36 Byte**, danach folgt unverändert der rohe Frame. Im Frame ist
  Byte 0 der Meldungstyp, Byte 1–4 sind die msg_id little-endian
  (`encodeStartAPRS()`, `src/aprs_functions.cpp:1257-1260`):

```c
#define UDP_DATA_HEADER_LEN 36  /* "DATA" + 8 hex GW-ID + 9 Rufzeichen + 4 Version
                                  * + 1 Unterversion + 4 RSSI + 4 SNR + "03" */

/* buf/len ist die vollständige UDP-Nutzlast eines Gateway-Uploads.
 * Format: src/udp_functions.cpp:1843 (addNodeData). */
bool udp_data_get_msg_id(const uint8_t *buf, size_t len, uint32_t *out_msg_id)
{
    if (len < UDP_DATA_HEADER_LEN + 5 || memcmp(buf, "DATA", 4) != 0)
        return false;

    const uint8_t *frame = buf + UDP_DATA_HEADER_LEN;  /* roher APRS-Frame */

    /* frame[0]      = Meldungstyp ('!'=Position, ':'=Text, '@'=Hey, ...)
     * frame[1..4]   = msg_id, little-endian (aprs_functions.cpp:1257-1260) */
    *out_msg_id = (uint32_t)frame[1]
                | ((uint32_t)frame[2] << 8)
                | ((uint32_t)frame[3] << 16)
                | ((uint32_t)frame[4] << 24);
    return true;
}
```

Die FCS deckt alle Bytes vor dem FCS-Feld ab, einschließlich der msg_id
(`encodeAPRS()`, `src/aprs_functions.cpp:1335-1348`). Das betrifft den Server nicht direkt — er ändert
die msg_id nicht selbst —, ist aber der Grund, warum jede Wiederholung ihren eigenen, gültigen Frame
mitbringt und der Server die msg_id ungeprüft aus dem Frame übernehmen kann, ohne die FCS erneut zu
prüfen (das hat das sendende Gateway bereits getan, sonst wäre der Frame verworfen worden).

## 3. Dedup: Schlüssel, Fenster, "wer zuerst kam"

**Schlüssel**: `msg_id & 0xFFFFF3FFu`. Das löscht nur die Bits 10–11, in denen sich Erstsendung und
Wiederholungen unterscheiden; die 20 Bit Knotenkennung (Bit 12–31) und die 10 Bit laufende Nummer
(Bit 0–9) bleiben unverändert (Konzeptpapier 2.1, 4.3).

**Fenster**: mindestens 10 Minuten. Die Wiederholungsleiter braucht rund 120 s (4 Aussendungen im
40-s-Abstand), dazu kommt Zustellzeit durchs Mesh — auf einem belebten Knoten wurde für Text ein
Median von 30 s je Sprung gemessen. Ein Fenster von 10 Minuten liegt komfortabel darüber, auch bei
mehreren Hops und Warteschlangen an Relais.

**Warum nicht Stunden**: der Zähler in den unteren 10 Bit (`node_msgid`) wird von **allen**
Meldungsarten eines Knotens geteilt — Text, Position, Hey, Telemetrie, Quittung
(Konzeptpapier 2.1). Ein aktiver Knoten durchläuft die 1000 möglichen Werte in weniger als einer
Stunde. Ein zu langes Fenster lässt eine echte neue PN desselben Knotens mit einem alten,
ausgelaufenen Ringeintrag kollidieren und würde sie fälschlich als Wiederholung verwerfen. Das
Fenster muss kurz genug sein, um diesen Wraparound sicher zu vermeiden, aber länger als die Leiter.

**Wer zuerst kam, gewinnt** — die erste gesehene msg_id (in der Regel die Erstsendung) bleibt die
nach außen sichtbare, stabile msg_id für Anzeige, meshmap und Feeds, auch wenn spätere Kopien mit
anderen Bits 10–11 eintreffen:

```c
#define PN_DEDUP_CAPACITY   512
#define PN_DEDUP_WINDOW_SEC 600    /* >= 10 min, siehe oben */

typedef struct {
    uint32_t core_id;         /* msg_id & 0xFFFFF3FFu -- der stabile Teil */
    uint32_t first_msg_id;    /* ORIGINAL-msg_id der zuerst gesehenen Kopie,
                                * unverändert -- das ist die Kennung, die
                                * downstream (Anzeige, meshmap, Feeds) zu
                                * sehen bekommt */
    time_t   first_seen;      /* 0 = Slot frei */
    bool     gated_to_aprsis; /* siehe Kapitel 5 */
    char     dest_call[10];   /* Zielrufzeichen, siehe Kapitel 4 */
} pn_dedup_entry_t;

static pn_dedup_entry_t pn_ring[PN_DEDUP_CAPACITY];
static size_t pn_ring_next = 0;

/* Sucht einen gültigen Eintrag zu core_id, räumt dabei nebenbei
 * abgelaufene Slots auf. NULL heißt: das ist die erste Kopie. */
static pn_dedup_entry_t *pn_dedup_find(uint32_t core_id, time_t now)
{
    for (size_t i = 0; i < PN_DEDUP_CAPACITY; i++)
    {
        pn_dedup_entry_t *e = &pn_ring[i];
        if (e->first_seen == 0)
            continue;
        if (now - e->first_seen > PN_DEDUP_WINDOW_SEC)
        {
            e->first_seen = 0;   /* abgelaufen, Slot wieder frei */
            continue;
        }
        if (e->core_id == core_id)
            return e;
    }
    return NULL;
}

/* Einmal pro eingehender Kopie einer PN (Meldungstyp ':', Ziel ist ein
 * einzelnes Rufzeichen, kein '*' und keine Gruppe) aufrufen. Liefert den
 * Eintrag für die Entscheidungen in Kapitel 4 und 5 zurück. */
pn_dedup_entry_t *pn_dedup_register(uint32_t msg_id, const char *dest_call,
                                     time_t now, bool *is_first_copy)
{
    uint32_t core_id = msg_id & 0xFFFFF3FFu;

    pn_dedup_entry_t *e = pn_dedup_find(core_id, now);
    if (e != NULL)
    {
        *is_first_copy = false;
        return e;
    }

    e = &pn_ring[pn_ring_next];
    pn_ring_next = (pn_ring_next + 1) % PN_DEDUP_CAPACITY;

    e->core_id = core_id;
    e->first_msg_id = msg_id;
    e->first_seen = now;
    e->gated_to_aprsis = false;
    snprintf(e->dest_call, sizeof(e->dest_call), "%s", dest_call);

    *is_first_copy = true;
    return e;
}
```

Ein Ring mit linearer Suche reicht: PN sind selten (meshmap zählte zuletzt rund 1470 pro Woche über
die gesamte Flotte), das gleichzeitig offene Fenster ist klein. Eine Hashtabelle über `core_id` wäre
eine reine Optimierung, kein funktionaler Unterschied.

Die Maskierung gilt für Textmeldungen an ein einzelnes Rufzeichen (PN). Gruppen und `*` behalten
ihre heutige msg_id über alle Aussendungen hinweg unverändert (Konzeptpapier 4.3), eine Maskierung
schadet dort nicht, bringt aber auch nichts — der Server kann sie auf PN beschränken, um die
zusätzliche Kollisionswahrscheinlichkeit (zwei Bit weniger Knotenunterscheidung, Konzeptpapier 7.4)
nicht unnötig auf den gesamten Verkehr auszuweiten.

## 4. Weiterreichen an Gateways (GATE)

**Das Risiko (Annahme zum heutigen Serververhalten, siehe Kap. 7.6 Frage 2)**: jedes Gateway speist
jede Meldung, die ihm der Server per `GATE`-Frame schickt, in sein LoRa-Netz ein, sobald deren msg_id
in seinem eigenen Dedup neu ist — auch PN an fremde Rufzeichen. Es gibt keinen Filter nach "Ziel hier
bekannt" (`src/udp_functions.cpp:242-330`, Einspeisung `:488-528`; die einzigen Ausnahmen sind PN ans
eigene Rufzeichen, eigene Quittungen und Positionen bei `NOPOS`, `:358-359`, `:403-404`,
`:448-449`). Reicht der Server künftig jede Wiederholung einer PN unverändert an alle Gateways
weiter, strahlt **jede Region** jede Wiederholung aus, obwohl der Empfänger vermutlich nur in einer
einzigen Region sitzt. Das ist der Grund, warum diese Entscheidung vor der Absender-Firmware stehen
muss.

Drei Optionen:

**(i) Nur die erste Kopie weiterreichen** — am einfachsten:

```c
/* (i) nur die Erstsendung geht an die Gateways; jede spätere Kopie wird
 * hier verschluckt. */
void pn_forward_to_gateways_v1(bool is_first_copy,
                                const uint8_t *frame, size_t frame_len)
{
    if (!is_first_copy)
        return;

    gate_broadcast_to_all_gateways(frame, frame_len);
}
```

Verlust: geht die Erstsendung ausgerechnet auf dem letzten Funksprung in der Zielregion verloren,
bekommt diese Region auch über den Server keine zweite Chance — genau der Fall (P2 auf dem
Internetweg), den die Wiederholung eigentlich lösen soll.

**(ii) Wiederholungen nur an Gateways, die das Ziel kürzlich gehört haben** — empfohlen:

```c
/* ASSUMPTION: eine solche Anwesenheitstabelle existiert im Server heute
 * nicht und müsste aus Position/Hey/Quittungen je Gateway aufgebaut
 * werden. */
#define PRESENCE_WINDOW_SEC (2 * 3600)  /* Größenordnung, gegen echten
                                          * Verkehr zu kalibrieren */

/* true, wenn Gateway gateway_id das Rufzeichen call innerhalb von
 * PRESENCE_WINDOW_SEC zuletzt gesehen hat (Position, Hey, Text, ACK). */
bool gateway_recently_heard_call(int gateway_id, const char *call, time_t now);

void pn_forward_to_gateways_v2(const pn_dedup_entry_t *e, bool is_first_copy,
                                const uint8_t *frame, size_t frame_len,
                                time_t now)
{
    if (is_first_copy)
    {
        gate_broadcast_to_all_gateways(frame, frame_len);
        return;
    }

    for (int gw = 0; gw < gate_gateway_count(); gw++)
    {
        if (gateway_recently_heard_call(gw, e->dest_call, now))
            gate_send_to_gateway(gw, frame, frame_len);
    }
}
```

Trifft die Fälle, in denen die Wiederholung tatsächlich nützt (der Empfänger ist in dieser Region
präsent, nur der letzte Sprung ist unsicher), ohne fremde Regionen unnötig zu fluten. Kosten: eine
zusätzliche Tabelle (Gateway × Rufzeichen × Zeitstempel) und ihre Pflege aus dem ohnehin
eintreffenden Verkehr.

**(iii) Alle Kopien an alle Gateways** — nicht empfohlen: entspricht Variante (i) ohne die
`if (!is_first_copy) return;`-Sperre. Flutet bei jeder Wiederholung jede angeschlossene Region neu,
unabhängig davon, ob der Empfänger dort überhaupt existiert.

**Empfehlung**: (ii), mit (i) als Rückfall, falls sich eine Anwesenheitstabelle kurzfristig nicht
umsetzen lässt.

## 5. APRS-IS: höchstens einmal pro PN

aprsc dedupliziert nur byte-identische Zeilen (Quelle mit SSID + Ziel ohne SSID + Info-Feld, Pfad
zählt nicht) und nur innerhalb eines festen Fensters von 30 s ab Erstempfang
(`dupecheck.c`/`config.c`, `dupefilter_storetime = 30`). Die Wiederholungsleiter sendet im
40-s-Abstand — **jenseits** dieses Fensters. aprsc allein fängt also keine Wiederholung ab; jede
käme unverändert bei aprs.fi und anderen APRS-IS-Clients zusätzlich an. Die Dedup für APRS-IS muss
deshalb am MeshCom-Server passieren, nicht bei aprsc.

Der Text darf dabei **keine** Wiederholungsmarke tragen — eine Markierung im Text würde keinem
Relais helfen (die prüfen nur die msg_id) und aprs.fi/APRS-IS zusätzlich mit variierenden Zeilen
für dieselbe PN verwirren (Konzeptpapier 4.2, "Verworfen — Wiederholungsmarke im Text").

```c
/* Direkt vor dem Formatieren der ausgehenden APRS-IS-Zeile aufrufen. */
bool pn_should_gate_to_aprsis(pn_dedup_entry_t *e)
{
    if (e->gated_to_aprsis)
        return false;   /* diese PN wurde schon einmal gegated */

    e->gated_to_aprsis = true;
    return true;
}
```

## 6. PN-Erkennung im Server

Eine PN erkennt sich am Text, nicht an der msg_id: die letzte `{` im Payload, gefolgt von 1 bis 5
Ziffern, **ohne** schließende Klammer. Das unterscheidet sie von APRS-Objekten/Items, die Klammern
anders verwenden, und ist derselbe Parser, den Variante b) für `{NNN` bräuchte (Konzeptpapier 4.4).

```c
/* Sucht das letzte '{' im Payload und liest die anschließende Ziffernfolge
 * (1..5 Stellen) als NNN. Liefert false, wenn keine '{'-Ziffernfolge am
 * Ende steht (z. B. weil eine schließende '}' folgt -- das ist dann kein
 * PN-Anhängsel). */
bool pn_extract_nnn(const char *text, size_t len, unsigned *out_nnn)
{
    if (text == NULL || len == 0)
        return false;

    size_t brace = (size_t)-1;
    for (size_t i = len; i-- > 0; )
    {
        if (text[i] == '{')
        {
            brace = i;
            break;
        }
    }
    if (brace == (size_t)-1)
        return false;

    size_t digits_start = brace + 1;
    size_t ndigits = len - digits_start;
    if (ndigits < 1 || ndigits > 5)
        return false;

    unsigned val = 0;
    for (size_t i = digits_start; i < len; i++)
    {
        if (text[i] < '0' || text[i] > '9')
            return false;   /* z. B. eine schließende '}' -- keine PN */
        val = val * 10 + (unsigned)(text[i] - '0');
    }

    *out_nnn = val;
    return true;
}
```

Für Variante a) reicht dieser Parser, um eine Textmeldung als PN zu klassifizieren (Kap. 3, um die
Maskierung auf PN zu begrenzen); die NNN selbst braucht a) nicht als Dedup-Schlüssel — das ist erst
für Variante b) nötig (Konzeptpapier 4.4, 4.2). Eine Meldung an `*` oder eine Gruppe ist unabhängig
vom Textende keine PN.

## 7. Rückwärtskompatibilität / Einführungsreihenfolge

- **Server zuerst, vor jeder Absender-Firmware** (Konzeptpapier 4.1, 7.5). Ohne die Server-Änderung
  würde jede Wiederholung ungefiltert an alle Gateways gehen (Kap. 4).
- Die Server-Änderung selbst ist **rückwärtskompatibel**: alte Gateways und alte Absender-Firmware
  erzeugen nie Wiederholungen mit veränderten Bits 10–11, ihre Erstsendung hat unverändert Bits
  10–11 = 0 relativ zur Maske — die Maskierung wirkt für sie wie keine Maskierung, weil nie eine
  zweite Kopie mit demselben 30-Bit-Kern eintrifft.
- **Gemischte Flotte über Monate**: 69 % der Knoten liefen zum Erhebungszeitpunkt auf Firmware älter
  als 4.35t (Konzeptpapier 2.6). Alte Gateways laden jede Wiederholung roh hoch, sobald es sie gibt
  (sie haben so oder so kein eigenes Wissen über Bit 10–11); der Server sieht sie wie jede andere
  Kopie und behandelt sie nach den Regeln aus Kap. 3–5, unabhängig von der Gateway-Firmware.
- Erst wenn der Server produktiv auf `msg_id & 0xFFFFF3FFu` dedupliziert und eine Weiterreiche-Regel
  aus Kap. 4 aktiv ist, sollte die erste Absender-Firmware mit der XOR-Wiederholung ausrollen.

## 8. Offene Fragen (an den Server-Betrieb)

Unverändert aus Konzeptpapier 7.6, hier weil sie genau diese Änderung betreffen:

1. Worauf dedupliziert der Server heute (msg_id allein, msg_id + Rufzeichen, Inhalt)?
2. Welche Gateways bekommt eine PN vom Server (alle, oder nur die, die das Ziel gehört haben)?
3. Welche Pakete gehen zu APRS-IS (nur PN mit `APRS:`, auch Gruppen)?
4. Gibt der Server die msg_id unverändert an meshmap und andere Abnehmer weiter?

Ohne Antworten bleiben Kap. 3–5 dieses Dokuments Vorschläge auf Annahmen; sie sind aber so gehalten,
dass sie sich unabhängig vom heutigen Verhalten einbauen lassen (der Dedup-Schlüssel wird nur enger,
nicht ersetzt; das Weiterreichen bekommt eine zusätzliche Filterstufe, keine neue Grundregel).

## 9. Testvorschlag

1. Vier Frames derselben PN erzeugen: gleicher 30-Bit-Kern der msg_id, Bits 10–11 = `X`,
   `X⊕1`, `X⊕2`, `X⊕3` (Original-XOR, nicht schrittweise, Konzeptpapier 4.3), FCS je neu berechnet,
   `{NNN` und Text identisch.
2. Die vier Frames über **zwei unterschiedliche** simulierte Gateways einspielen (z. B. zwei Kopien
   über Gateway A, zwei über Gateway B), um zu prüfen, dass die Dedup nicht am Gateway hängt, sondern
   am Server-weiten Schlüssel.
3. Erwartung:
   - Anzeige/meshmap/APRS-IS erhalten die PN **genau einmal**, mit der msg_id der zuerst
     eingetroffenen Kopie.
   - Weiterreichen an Gateways folgt der gewählten Regel aus Kap. 4: bei (i) nur die erste Kopie geht
     an alle Gateways; bei (ii) geht jede spätere Kopie nur an Gateways, die das Zielrufzeichen
     kürzlich gehört haben.
   - Eine fünfte, künstlich verspätete Kopie (nach Ablauf des 10-Minuten-Fensters) wird als **neue**
     PN behandelt — das Fenster muss endlich sein, nicht als Regressionsfreibrief für eine echte neue
     PN mit zufällig gleichem 30-Bit-Kern (Wraparound, Kap. 3).
