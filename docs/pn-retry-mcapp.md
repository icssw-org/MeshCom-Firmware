# PN-Wiederholung Variante a) (XOR): Änderungen in mcapp (MCProxy + Webapp)

Stand: 27.09.2026, DK5EN. Bezug: Konzeptpapier "Verlässliche Zustellung persönlicher Nachrichten
(PN)" (`docs/pn-zustellung-dedup.md`), Kapitel 3, 4.2, 4.3, 7.1. Quelltext: `MCProxy` und
`webapp` (read-only referenziert, keine Änderung in diesen Repos — dieses Dokument beschreibt nur,
was dort zu ändern wäre).

Firmware-Verhalten auf Branch `dk5en-xor` (Voraussetzung für alles Folgende): die Erstsendung einer
PN ist byte-gleich zu heute. Wiederholung k hat in der msg_id die Bits 10–11 = Original-Bits XOR k;
der restliche 30-Bit-Kern (20 Bit Knotenkennung in Bit 12–31 + 10 Bit laufende Nummer in Bit 0–9)
und das `{NNN`-Suffix im Text bleiben unverändert. Ein Knoten mit der neuen Firmware
filtert Wiederholungen selbst: er gibt nur die erste Kopie einer PN an UDP-JSON/BLE weiter und
meldet jede Quittung mit der **Original**-msg_id. Ein Knoten mit alter Firmware kennt die Bits
10–11 nicht und gibt jede Kopie mit ihrer eigenen, unterschiedlichen msg_id weiter. **mcapp selbst
sendet nie eine PN erneut** — es ist in jedem Fall nur Empfänger dieser Kopien.

## 1. Kurzfassung

**An einem Knoten mit der neuen Firmware muss in mcapp nichts geändert werden.** Der Knoten hat die
Wiederholungen schon herausgefiltert, bevor sie UDP-JSON oder BLE erreichen — genau eine Kopie pro
PN, genau eine Quittung mit der Original-msg_id. Jede der unten beschriebenen Dedup-Stellen in
MCProxy und der Webapp arbeitet exakt wie heute, weil sie nie eine zweite Kopie zu Gesicht bekommt.

Zwei Fälle bleiben, in denen mcapp **defensiv** nachziehen sollte:

- **Der angeschlossene Knoten läuft (noch) mit alter Firmware.** Laut Konzeptpapier §2.6 sind 69 %
  der Flotte älter als 4.35t. Ein solcher Knoten liefert jede Wiederholung als eigene msg_id aus
  (§4.3 "Alte Empfänger zeigen Kopien"), und jede der msg_id-Schlüssel in MCProxy/Webapp behandelt
  sie dann als eigenständige PN.
- **Die Command-Throttle-Falle**: mcapp ist an demselben Knoten auch ein Bot, der auf `!command`
  liest. Der msg_id-Dedup vor dem Throttle (`commands/routing.py:87-92`) erkennt eine Wiederholung
  mit anderer msg_id nicht als Duplikat, und der nachfolgende Content-Throttle
  (`commands/routing.py:131-143`, `commands/constants.py:13` — 5 Minuten Standard) sieht denselben
  Befehlstext ein zweites Mal innerhalb seines Fensters. Das Ergebnis ist eine **zweite,
  unerwünschte Funkantwort** "⏳ Command throttled. Try the same command again shortly."
  (`commands/routing.py:139`) auf eine reine PN-Wiederholung — nicht bloß eine doppelte Anzeige,
  sondern eine zusätzliche Aussendung ins Mesh.

Die Empfehlung in Abschnitt 3 ist ein einziger Normalisierungs-Helfer (`msg_core()`), der an den
Stellen eingesetzt wird, die heute auf der vollen 32-Bit-msg_id schlüsseln. Alle Änderungen sind
additiv und rückwärtskompatibel: an einem Knoten mit neuer Firmware ändert sich nichts sichtbar,
weil `msg_core()` auf der Erstsendung ein No-op ist (Bits 10–11 sind dort bereits 0 relativ zum
Original) und nie eine zweite Kopie ankommt, die genormt werden müsste.

## 2. Ist-Zustand pro Schicht

| Schicht                               | Schlüssel heute                                                  | Beleg                                                                                                                                                                            | Wirkung auf eine XOR-Wiederholung heute                                                                                                              |
| ------------------------------------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| Storage-Ingest-Tor (`store_message`)  | (Absender, msg_id), 60 min                                       | `storage/ingest.py:288-308` (`_claim_recent_ingest`), `:310-326` (`_find_duplicate_row_id`), `:1918-1991` (Aufruf im Ingest-Pfad), `storage/constants.py:29` (`DEDUP_WINDOW_MS`) | volle msg_id ungleich → **eigene Zeile**, keine Deduplizierung                                                                                       |
| Push-Dedup (`PushDedup`)              | msg_id allein, sonst (Absender, Ziel, Text)                      | `push_delivery.py:488-538` (Klasse + `is_duplicate`), Fensterbreite von `storage/constants.py:29` übernommen (`push_delivery.py:57`)                                             | volle msg_id ungleich → **zweite Push-Benachrichtigung**                                                                                             |
| Command-Dedup (`processed_msg_ids`)   | msg_id allein, 5 min                                             | `commands/dedup.py:29` (Init), `:92-96` (`_is_duplicate_msg_id`), `:104-106` (`_mark_msg_id_processed`), Aufruf `commands/routing.py:87-92`                                      | volle msg_id ungleich → **Dedup greift nicht**, Aufruf läuft weiter zum Content-Throttle                                                             |
| Command-Throttle (`command_throttle`) | (Absender, Ziel, Text) bzw. (Absender, Ziel, `!cmd`)             | `commands/dedup.py:66-90` (`_get_content_hash`, `_is_throttled`), Fenster `commands/constants.py:13,58-65`                                                                       | Text ist retry-invariant (nur die msg_id ändert sich) → **Treffer**, wenn Wiederholung im Throttle-Fenster liegt → siehe Kurzfassung                 |
| Webapp-Dedup (`dedupIndex`)           | msg_id allein, sonst (Absender, Ziel, Text)                      | `stores/messages/dedup.ts:72-87` (`getDedupKey`, `isDuplicateByKey`), Fenster `constants/index.ts:7` (`TIME_WINDOW_MS`)                                                          | volle msg_id ungleich → **zweite Zeile im Chat**                                                                                                     |
| Binäre ACK (0x41, `_handle_ack`)      | exaktes `msg_id`, 4 h bzw. 168 h bei `held`                      | `storage/ingest.py:1118-1359` (`_handle_ack`), `:1361-1394` (`_resolve_ack_target`)                                                                                              | siehe unten — abhängig davon, wessen msg_id der ACK trägt                                                                                            |
| Text-ACK (`:ackNNN`)                  | `echo_id` (das `{NNN`-Suffix, retry-invariant) + Adress-Abgleich | `storage/ingest.py:180-217` (`_inline_ack_original`), `:1768-1870` (Aufruf-Stelle im Ingest-Pfad)                                                                                | **bereits retry-sicher** — `echo_id` ändert sich mit den XOR-Bits nicht (Konzeptpapier §4.3: "Die NNN ist … für alle Aussendungen ebenfalls gleich") |
| Linkcheck (Ping/Pong)                 | `msg_id >> 10` (Knotenkennung)                                   | `linkcheck.py:234-244` (`node_prefix_of_msg_id`)                                                                                                                                 | betroffen nicht — Ping/Pong werden nie wiederholt (Konzeptpapier §6, letzter Punkt)                                                                  |

Zur binären ACK im Detail: `_resolve_ack_target` sucht exakt nach `messages.msg_id = ack_for_msg_id`
(`storage/ingest.py:1386-1392`). Das stimmt in zwei Fällen ohne Änderung:

1. Unser eigener Knoten hat neue Firmware und meldet jede Quittung schon mit der Original-msg_id
   (Voraussetzung oben) — der ACK trägt exakt den Wert, den `store_message` beim Absenden unter
   `messages.msg_id` abgelegt hat.
2. Der Absender wiederholt gar nicht (Gruppen, `*`, oder alte Firmware, die byte-identisch mit
   derselben msg_id wiederholt, Konzeptpapier §2.3) — dann gibt es ohnehin nur eine msg_id.

Es bricht in einem dritten Fall: **unser eigener Knoten hat alte Firmware und ist selbst die
Adresse einer PN**, die ein Absender mit `dk5en-xor`-Firmware wiederholt. Die alte Firmware kennt
die XOR-Bits nicht, behandelt jede Kopie als neue, eigenständige Nachricht und generiert für jede
Kopie eine eigene Quittung mit **deren eigener** (unterschiedlicher) msg_id — nicht der
Original-msg_id. Trifft eine solche Quittung für Retry k bei uns ein, sucht
`_resolve_ack_target` nach `msg_id = Original XOR k` und findet die unter `Original` gespeicherte
Zeile nicht.

`ble_protocol.py` liefert dafür nur die Rohwerte, ohne eigene Normalisierung: `hex_msg_id()`
formatiert die 32-Bit-Zahl als 8-stelligen Hex-String (`ble_protocol.py:145-147`), `_decode_ack_frame`
liest `msg_id` unverändert aus Byte 2-5 des ACK-Frames (`ble_protocol.py:211-254`), und
`transform_msg`/`transform_ack` reichen diesen String unverändert weiter
(`ble_protocol.py:806-820`, `:823-832`). `udp_handler.normalize_extudp_ack` tut dasselbe für den
Extern-UDP-Pfad (`udp_handler.py:243-297`) — der 8-stellige Hex-String kommt in beiden Transporten
so an, wie ihn der jeweilige Knoten gebildet hat.

## 3. Empfohlene Änderungen

### 3.1 `msg_core()` — gemeinsamer Normalisierungs-Helfer (S)

Ein einziger Helfer in `src/mcapp/util.py`, neben den bestehenden Kleinhelfern
(`is_placeholder_callsign` bei `util.py:29-37`, `strip_ack_suffix` bei `util.py:70`), damit jede
der folgenden Stellen dieselbe Maske verwendet und nicht fünf Kopien der Bit-Arithmetik entstehen:

```python
# src/mcapp/util.py (neu)
_MSG_ID_CORE_MASK = 0xFFFFF3FF  # löscht Bit 10-11 (dk5en-xor Wiederholungszähler)


def msg_core(msg_id_hex: str | None) -> str | None:
    """Der wiederholungsinvariante Kern einer Firmware-msg_id (dk5en-xor,
    docs/pn-retry-mcapp.md §3.1). Bit 10-11 tragen bei einer Wiederholung
    das XOR des Wiederholungszählers; alle anderen Bit sind für jede Kopie
    derselben PN gleich. Auf einer Erstsendung oder an einem Knoten ohne die
    Wiederholungsfunktion ist das Maskieren ein No-op.

    None/unparsbare Eingabe -> None, wie die vorhandene falsy-msg_id-Regel
    im Dedup-Contract (dedup_contract.json `content_fallback`).
    """
    if not msg_id_hex:
        return None
    try:
        value = int(msg_id_hex, 16)
    except (TypeError, ValueError):
        return msg_id_hex
    return f"{value & _MSG_ID_CORE_MASK:08X}"
```

### 3.2 Storage-Ingest-Tor (M)

Zwei Stellen in `storage/ingest.py` schlüsseln auf der vollen msg_id: der In-Memory-Claim
(`_claim_recent_ingest`, `:288-308`) und die SQL-Rückfalls-Abfrage (`_find_duplicate_row_id`,
`:310-326`). Der In-Memory-Teil ist reines Python:

```python
# storage/ingest.py:299 — vorher: key = (callsign.strip().upper(), msg_id)
key = (callsign.strip().upper(), msg_core(msg_id) or msg_id)
```

Die SQL-Abfrage vergleicht eine TEXT-Spalte gegen den vollen Hex-String
(`storage/ingest.py:320-325`); SQLite kennt keine eingebaute Hex-zu-Int-Funktion, mit der sich
`msg_id & 0xFFFFF3FF` in der WHERE-Klausel ausdrücken ließe. Statt einer Schema-Änderung (neue
Spalte, Migration) genügt eine deterministische Python-Funktion, einmal pro Connection registriert
(`sqlite3.Connection.create_function`), und deren Aufruf in der WHERE-Klausel:

```python
# einmalig bei Connection-Aufbau (SQLiteStorage._get_writer_conn o.ä.)
conn.create_function("msg_core", 1, msg_core, deterministic=True)
```

```python
# storage/ingest.py:320-325 — vorher: "... WHERE msg_id = ? AND timestamp > ? AND ..."
"SELECT id FROM messages WHERE msg_core(msg_id) = ? AND timestamp > ?"
f" AND {sender_base_sql('src')} = ?"
" ORDER BY timestamp ASC, id ASC LIMIT 1",
(msg_core(msg_id), timestamp - DEDUP_WINDOW_MS, callsign.upper()),
```

Ohne Index auf `msg_core(msg_id)` bleibt das ein Funktionsaufruf pro Zeile im bereits durch
`timestamp`/`sender_base_sql` eingeschränkten Fenster — bei den auf mcapp.local gemessenen ~3000
Nachrichten/Woche (`storage/ingest.py:110` Kommentar zu `_RECENT_INGEST_PRUNE_AT`) unkritisch.
`messages.msg_id` bleibt dabei **unverändert der roh empfangene Wert** — nur der Vergleich läuft
über den Kern, die gespeicherte Zeile bleibt forensisch nachvollziehbar.

### 3.3 Push-Dedup (S)

`PushDedup.is_duplicate` bildet den Schlüssel direkt aus `payload.get("msg_id")`
(`push_delivery.py:527-534`):

```python
# push_delivery.py:527-530 — vorher: if msg_id: key = ("id", msg_id)
from .util import msg_core  # noqa: an der bestehenden Import-Stelle ergänzen

msg_id = payload.get("msg_id")
if msg_id:
    key = ("id", msg_core(msg_id))
```

### 3.4 Command-Dedup / Command-Throttle-Falle (S)

Die eigentliche Falle liegt nicht in `commands/dedup.py` — die Klasse ist ein generischer,
schlüsselunabhängiger Cache (`_is_duplicate_msg_id`/`_mark_msg_id_processed`,
`commands/dedup.py:92-106`) — sondern im Aufrufer, der die volle msg_id als Schlüssel übergibt:

```python
# commands/routing.py:87-92 — vorher: msg_id = message_data.get("msg_id")
#                                      if msg_id and self._is_duplicate_msg_id(msg_id): ...
#                                      if msg_id: self._mark_msg_id_processed(msg_id)
msg_id = message_data.get("msg_id")
dedup_key = msg_core(msg_id) or msg_id
if dedup_key and self._is_duplicate_msg_id(dedup_key):
    logger.debug("Duplicate msg_id %s (core=%s), ignoring", msg_id, dedup_key)
    return
if dedup_key:
    self._mark_msg_id_processed(dedup_key)
```

Weil dieser Check vor dem Content-Throttle sitzt (`commands/routing.py:131-143` folgt erst danach)
und jetzt für jede Retry-Kopie desselben Befehls zutrifft, wird der Throttle-Zweig — und damit die
zweite Funkantwort "Command throttled" — für eine reine PN-Wiederholung gar nicht mehr erreicht.
`commands/dedup.py` selbst bleibt unverändert; sie kennt ihren Schlüssel nicht und muss ihn auch
nicht kennen.

### 3.5 Webapp-Dedup-Schlüssel (S)

`getDedupKey` baut den Schlüssel aus `el.msg_id` ohne Normalisierung
(`stores/messages/dedup.ts:78`). Ein TS-Äquivalent von `msg_core()`, neben den anderen reinen
Helfern in derselben Datei:

```ts
// src/stores/messages/dedup.ts (neu, neben getDedupKey)
const MSG_ID_CORE_MASK = 0xfffff3ff; // löscht Bit 10-11 (dk5en-xor Wiederholungszähler)

export function msgCore(msgIdHex: string | undefined): string | null {
  if (!msgIdHex) return null;
  const value = Number.parseInt(msgIdHex, 16);
  if (Number.isNaN(value)) return msgIdHex;
  return (value & MSG_ID_CORE_MASK).toString(16).toUpperCase().padStart(8, "0");
}
```

```ts
// dedup.ts:78 — vorher: if (el.msg_id) return `id:${el.msg_id}`
if (el.msg_id) return `id:${msgCore(el.msg_id) ?? el.msg_id}`;
```

### 3.6 `dedup_contract.json` (S, Abstimmungsbedarf)

Der Vertrag pinnt heute nur "msg_id-primary … unabhängig von src/dst/text"
(`contract/dedup_contract.json:6`, `key_semantics.precedence`) — keine Aussage über eine
Normalisierung der msg_id selbst. Eine Ergänzung um ein optionales
`id_normalization`-Feld (Maske `0xFFFFF3FF`, mit `id_vectors`, die zwei Ids belegen, die sich nur in
Bit 10-11 unterscheiden und denselben Schlüssel ergeben müssen) hält alle drei Implementierungen
synchron. Laut der Datei selbst liegt die kanonische Fassung aber **in `mc-chat`**
(`contract/dedup_contract.json:3`, "SYNC RULE … canonical source lives upstream in mc-chat's
contract/dedup_contract.json"): eine Änderung hier ist erst vollständig, wenn sie dort ebenfalls
landet und die Webapp-Kopie unter `src/stores/messages/__tests__/dedup_contract.json` neu
synchronisiert wird. Das ist außerhalb des Umfangs dieses Dokuments (nur MCProxy/Webapp) und wird
hier nur als Abhängigkeit vermerkt.

### 3.7 Binäre-ACK-Auflösung auf dem Kern (M)

`_resolve_ack_target` vergleicht exakt (`storage/ingest.py:1386-1392`). Mit derselben
SQL-Funktion wie in 3.2:

```python
# storage/ingest.py:1386-1392 — vorher: "... WHERE msg_id = ? AND type = 'msg' AND (...)"
"SELECT id, msg_id, timestamp, delivery_status FROM messages"
" WHERE msg_core(msg_id) = ? AND type = 'msg'"
"   AND (timestamp > ?"
"        OR (delivery_status = 'held' AND timestamp > ?))"
" ORDER BY timestamp DESC LIMIT 1",
(msg_core(msg_id), ack_ts - ACK_MSG_ID_WINDOW_MS, ack_ts - HELD_ACK_WINDOW_MS),
```

Das behebt genau den in Abschnitt 2 beschriebenen dritten Fall (eigener Knoten mit alter Firmware
als PN-Empfänger). Es ändert nichts an den beiden Fällen, die heute schon funktionieren — für sie
ist `msg_core(x) == x`, solange kein Retry-Bit gesetzt ist. Der Text-ACK-Pfad
(`_inline_ack_original`, `storage/ingest.py:180-217`) braucht **keine** entsprechende Änderung: er
schlüsselt über `echo_id`, das laut Konzeptpapier §4.3 für alle Aussendungen gleich bleibt, und ist
damit von den XOR-Bits gar nicht betroffen (siehe Tabelle in Abschnitt 2).

## 4. Nicht ändern: `linkcheck.py`

`node_prefix_of_msg_id` liest die 22-Bit-Knotenkennung aus `msg_id >> 10`
(`linkcheck.py:234-244`), einschließlich der beiden Bits, die eine PN-Wiederholung XOR-verknüpft.
Das bleibt unverändert: Ping/Pong-Frames sind kein Text mit `{NNN`-Suffix und werden von der
XOR-Wiederholung nicht erfasst (Konzeptpapier §4.3 "Nur für PN"; §6 Hinweis "Ping/Pong, die nicht
wiederholt werden"). Eine Maskierung hier hätte keinen Nutzen und würde nur echte
Knotenkennungs-Bits aus einer nie wiederholten Kennung wegschneiden.

## 5. Testfälle

Alle Vektoren unten verwenden ein Paar, das sich nur in Bit 10-11 unterscheidet, z. B.
`E1E05457` (Original, Bit 10-11 = 01) und `E1E05057` (Retry 1, `01 XOR 01 = 00`) — beide mit
identischem 30-Bit-Kern und identischem `{NNN`-Suffix im Text.

1. **`msg_core()` (Python + TS)**: `msg_core("E1E05457") == msg_core("E1E05057") == "E1E05057"`;
   `msg_core(None) is None`; ein unveränderter Erstsende-Wert ist ein No-op
   (`msg_core(x) == x`, wenn Bit 10-11 von `x` bereits 0 sind).
2. **Storage-Ingest-Tor**: `store_message` mit `msg_id="E1E05457"`, danach ein zweiter Aufruf mit
   `msg_id="E1E05057"`, gleicher Absender, innerhalb `DEDUP_WINDOW_MS` — erwartet: genau eine Zeile
   in `messages`, der zweite Aufruf nimmt den Enrichment-Pfad (`_enrich_duplicate_row`), keine neue
   `INSERT`. Regressionstest für den heutigen Zustand: derselbe Vektor ohne `msg_core()` erzeugt
   zwei Zeilen.
3. **Push-Dedup**: zwei `handle_mesh_message`-Aufrufe mit obigem Paar, gleicher Text, innerhalb des
   Coalesce-/Dedup-Fensters — erwartet: genau ein Push (sofort oder als Coalesce-Summary), nicht
   zwei.
4. **Command-Throttle-Falle**: zwei `!time`-Nachrichten desselben Absenders mit dem obigen
   msg_id-Paar, 40 s auseinander (Retry-Takt aus Konzeptpapier §7.1) — erwartet: `send_response`
   wird nur einmal aufgerufen, nie mit dem Text "Command throttled". Regressionstest: derselbe
   Vektor ohne die Änderung aus 3.4 ruft `send_response` zweimal auf, das zweite Mal mit dem
   Throttle-Text.
5. **Webapp-Dedup**: `isDuplicateByKey(getDedupKey({msg_id: "E1E05057", ...}), ts)` liefert `true`,
   nachdem zuvor `recordDedupKey(getDedupKey({msg_id: "E1E05457", ...}), ts0)` mit
   `ts - ts0 <= TIME_WINDOW_MS` aufgerufen wurde.
6. **Binäre-ACK-Auflösung**: eine Zeile mit `msg_id="E1E05457"` liegt in `messages`; ein ACK-Frame
   mit `ack_for_msg_id="E1E05057"` (simuliert einen alten Empfänger-Knoten, der auf Retry 1
   antwortet) trifft ein — erwartet: `_resolve_ack_target` findet dieselbe Zeile und
   `send_success`/`acked` werden auf ihr gesetzt, nicht auf einer neuen. Regressionstest: derselbe
   Vektor ohne die Änderung aus 3.7 liefert `None` (kein Treffer, ACK verpufft ungenutzt,
   `storage/ingest.py:1190-1202` protokolliert nur die Diagnose).
