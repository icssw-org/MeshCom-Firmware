# PN-Wiederholung Variante a) (XOR): Änderungen in der MeshCom-App (iPhone/Android)

Stand: 27.09.2026, DK5EN. Bezug: Konzeptpapier "Verlässliche Zustellung persönlicher Nachrichten
(PN)" (`docs/pn-zustellung-dedup.md`), Kapitel 3, 4.2, 4.3, 5.2, 7.1. App-Quelltext:
`Meshcom-MobileApp` (read-only referenziert, keine Änderung in diesem Repo).

## 1. Kurzfassung

**An einem Knoten mit der neuen Firmware (Variante a, XOR-Zähler in Bit 10–11) muss die App nicht
geändert werden.** Der Knoten filtert die Wiederholungen schon vor der Übergabe ans Telefon: nur
die erste Kopie einer PN geht als Text-Frame über BLE/Seriell hinaus, und die binäre Quittung
(0x41) trägt immer die aus `:ackNNN` rekonstruierte **Original**-msg_id
(Konzeptpapier §4.3 "Status 'gehört' ans Telefon mit der Original-msg_id melden"; §7.1 Punkt 5).
Die App sieht also pro PN weiterhin genau eine Nachricht und genau eine Folge von Statuswechseln —
wie heute.

Zwei Fälle bleiben, in denen eine defensive Änderung sinnvoll ist:

- **Der angeschlossene Knoten läuft (noch) mit alter Firmware.** 69 % der Flotte ist älter als
  4.35t (Konzeptpapier §2.6). Ein solcher Knoten gibt jede Wiederholung mit ihrer eigenen msg_id
  als neue Text-Meldung weiter — die App zeigt die PN dann bis zu 4-mal.
- **Der Status "gehört" wird heute wie eine Zwischenstufe zur Zustellung dargestellt**, ist aber
  nur ein Indiz, dass die PN unterwegs ist (Konzeptpapier §4.1, §7.1 Punkt 2). Mit Variante a) kann
  die App zusätzlich einen ehrlichen vierten Zustand "unbestätigt" nach Ablauf aller Aussendungen
  zeigen, wie es Meshtastic vormacht (Konzeptpapier §5.2).

Alle Änderungen in Abschnitt 3 sind **additiv und rückwärtskompatibel**: sie ändern nichts an dem,
was ein Knoten mit neuer Firmware ohnehin schon liefert, sondern härten die App gegen Knoten ab,
die noch nicht aktualisiert sind, und verbessern die Statusanzeige. Keine ist eine Voraussetzung
für die Firmware-Änderung.

## 2. Ist-Zustand in der App

### 2.1 Dedup beim Schreiben einer Textnachricht

`writeTxtMsg` prüft vor dem Insert, ob dieselbe Nachricht schon existiert:

```ts
// src/DBservices/DataBaseService.ts:283-288
const res = await DatabaseService.db.query(
  `SELECT * FROM TextMessages WHERE msgNr = ${msg.msgNr} AND fromCall = '${msg.fromCall}' AND msgTXT = '${msg.msgTXT}'`,
);
if (res.values && res.values.length > 0) {
  console.log("DB Writing Txt Msg: Message already in database");
  return;
}
```

Der Schlüssel ist **exakt** `msgNr` (die volle 32-Bit msg_id) + `fromCall` + `msgTXT`, ohne
Zeitfenster. Eine Wiederholung mit einer anderen msg_id (Variante a, b oder ein alter Knoten, der
ohnehin jede Wiederholung mit neuer msg_id sendet) besteht diesen Test **nicht als Duplikat** —
sie hat eine andere `msgNr` — und wird als eigene Zeile eingefügt.

### 2.2 ACK-Zuordnung

`ackTxtMsg` sucht ausschließlich über `msgNr`, ohne `fromCall`- oder `isDM`-Einschränkung, und
aktualisiert **alle** Treffer:

```ts
// src/DBservices/DataBaseService.ts:345
const res = await DatabaseService.db.query(
  `SELECT * FROM TextMessages WHERE msgNr = ${msgNr}`,
);
```

```ts
// src/DBservices/DataBaseService.ts:350-372
for (let i = 0; i < res.values.length; i++) {
    const msg: MsgType = res.values[i];
    if (msg.ack !== 2) {
        if (ack_type === 0x01) {
            // msg came from GW
            msg.ack = 2;
        }
        if (ack_type === 0x00) {
            // msg came from another node
            msg.ack = 1;
        }
        if (ack_type === 0x02) {
            // msg came from DM Node. Should 0 and 1 instead of 0 and 2
            msg.ack = 2;
        }
        msg.ackCall = ack_call;
        const query_str = `UPDATE TextMessages SET ack = ${msg.ack}, ackCall = '${DatabaseService.escapeQuotes(ack_call)}' WHERE msgNr = ${msgNr}`;
        const ret = await DatabaseService.db.execute(query_str);
        ...
```

`ack_type` 0x00 ("gehört", Echo von einem anderen Knoten) setzt `ack = 1`; 0x01 (Gateway-ACK) und
0x02 (Quittung vom DM-Empfänger) setzen beide `ack = 2`. Die `UPDATE`-Klausel filtert wieder nur
nach `msgNr`, trifft also **jede** Zeile mit dieser Nummer — unabhängig von Absender oder
Nachrichtentyp. Der Code kommentiert das selbst als unsauber ("More than one message with the
same msgNr!", `DataBaseService.ts:348`), behandelt es aber nicht als Fehler.

Woher `msgNr` und der ACK-Typ kommen:

```ts
// src/utils/AprsParser.ts:247-249
export function readMsgId(dv: DataView, offset: number): number {
  return dv.getUint32(offset, true); // little-endian, siehe Konzeptpapier §2.1
}
```

```ts
// src/hooks/MessageHandler.ts:110
const msgID = readMsgId(msg, 2);
```

```ts
// src/hooks/MessageHandler.ts:693-696, 725
if (msg_type === 0x41){
    console.log("Txt Msg Acknowledge from node, frame len: " + msg_len);
    const ack_state = msg.getUint8(6);
    ...
    DatabaseService.ackTxtMsg(msgID, ack_state, ack_call);
}
```

`msgID` in der 0x41-Ackframe ist dieselbe 32-Bit-Zahl, byte-identisch vom Knoten übernommen. Ein
Knoten mit neuer Firmware liefert hier bereits die Original-msg_id (Konzeptpapier §4.3); die App
muss dafür nichts umrechnen.

### 2.3 `{NNN` im Text — nur bei eigenen Nachrichten entfernt

```ts
// src/hooks/MessageHandler.ts:554-562
if (
  msg_text_.includes("{") &&
  isDM_ === 1 &&
  from_callsign_ === node_call_ref.current
) {
  // if we have more than one sign, we remove the text before the last one
  // get the last sign index
  const last_sign_index = msg_text_.lastIndexOf("{");
  console.log("Last Sign Index: " + last_sign_index);
  console.log("Msg Text Len: " + msg_text_.length);
  const slicedTxt = msg_text_.slice(0, last_sign_index);
  msg_text_ = slicedTxt;
}
```

Die Bedingung greift nur, wenn `from_callsign_` dem **eigenen** Rufzeichen entspricht
(`node_call_ref.current`). Für eine PN, die man selbst **empfängt** — also von einem fremden
Rufzeichen — bleibt `{NNN` (ohne schließende Klammer, konform zur Schreibweise im Konzeptpapier
§0) im angezeigten Text stehen. Das ist der heutige Zustand, unabhängig von jeder
Wiederholungs-Variante, und in Abschnitt 3d als eigener, kosmetischer Punkt behandelt.

### 2.4 Statusanzeige im Chat

```tsx
// src/pages/Chat.tsx:1304-1316
{
  msg.fromCall === config_s.callSign ? (
    <>
      {msg.ackCall && (msg.ack === 1 || msg.ack === 2) ? (
        <>
          <IonText className="msg-ack-call">{msg.ackCall}</IonText>
        </>
      ) : (
        <></>
      )}
      {msg.ack === 0 ? (
        <>
          <IonIcon
            icon={checkmark}
            className="ack-icon"
            size="small"
            slot="end"
            title="sent"
          />
        </>
      ) : (
        <></>
      )}
      {msg.ack === 1 ? (
        <>
          <IonIcon
            icon={cloudOutline}
            className="ack-icon"
            size="small"
            slot="end"
            title={msg.ackCall ? `heard by ${msg.ackCall}` : "heard"}
          />
        </>
      ) : (
        <></>
      )}
      {msg.ack === 2 ? (
        <>
          <IonIcon
            icon={cloudDoneOutline}
            className="ack-icon"
            size="small"
            slot="end"
            title={msg.ackCall ? `acked by ${msg.ackCall}` : "acked"}
          />
        </>
      ) : (
        <></>
      )}
    </>
  ) : (
    <></>
  );
}
```

Drei Zustände, nur für eigene Nachrichten: `checkmark` (gesendet), `cloudOutline` (gehört, `ack=1`),
`cloudDoneOutline` (quittiert, `ack=2`). Es gibt heute keinen vierten Zustand für "nach der letzten
Aussendung immer noch nichts gehört".

### 2.5 Manuelles erneutes Senden

```tsx
// src/pages/Chat.tsx:687-700
if (asActionDetail === "resend") {
  console.log("Resend pressed");
  const resendTxt = selMsg[0].msgTXT;
  if (textAreaInputRef.current) {
    textAreaInputRef.current.value = resendTxt;
  }
  if (activeChatFilter === "DM") {
    const toCallResend = selMsg[0].toCall;
    toCallsign_.current = toCallResend;
    lastDMcallsign.current = toCallResend;
    setShCallsign(true);
    if (callsignInputRef.current) {
      callsignInputRef.current.value = toCallResend;
    }
  }
}
```

Das ist eine **manuelle** Aktion: Text und Zielrufzeichen werden ins Eingabefeld übernommen, der
Nutzer muss erneut auf Senden tippen. Es ist keine automatische Protokoll-Wiederholung und bleibt
von diesem Konzept unberührt (siehe Abschnitt 4).

## 3. Empfohlene Änderungen

### a) Dedup beim Schreiben: auf den 30-Bit-Kern + Zeitfenster prüfen

**Warum ein Fenster nötig ist:** der 30-Bit-Kern der msg_id (Bit 10–11 ausgeblendet) enthält 20 Bit
Knotenkennung (Bit 12–31) plus den 10-Bit-Zähler `node_msgid` (0–999, Bit 0–9, Konzeptpapier §2.1,
§4.3). Dieser Zähler wird von allen Meldungsarten geteilt und läuft laut Konzeptpapier "in weniger
als einer Stunde" durch alle 1000 Werte. Ein reiner Vergleich auf `fromCall` + maskierte `msgNr`
ohne Zeitgrenze würde nach einem Wrap zwei völlig unabhängige, spätere Nachrichten fälschlich als
Duplikat behandeln. Ein Fenster von z. B. 10 Minuten deckt die komplette XOR-Leiter (bis zu 3
Wiederholungen im 40-s-Abstand, siehe Konzeptpapier §2.3/§4.3, also ≤ 120 s plus Funklaufzeit) mit
deutlichem Spielraum ab, bleibt aber weit unter der Zeit, die der Zähler zum Umlauf braucht.

```ts
// Vorschlag für DataBaseService.writeTxtMsg (ersetzt den Block ab :283)
const maskedNr = (msg.msgNr & 0xfffff3ff) >>> 0; // 30-Bit-Kern, Bit 10-11 ausgeblendet, siehe Konzeptpapier §4.3
const DEDUP_WINDOW_MS = 10 * 60 * 1000;
const windowStart = msg.timestamp - DEDUP_WINDOW_MS;

// SQLite beherrscht bitweises UND direkt; 4294964223 = 0xFFFFF3FF
const res = await DatabaseService.db.query(
  `SELECT * FROM TextMessages WHERE (msgNr & 4294964223) = ${maskedNr} AND fromCall = '${msg.fromCall}' AND timestamp > ${windowStart}`,
);
if (res.values && res.values.length > 0) {
  console.log(
    "DB Writing Txt Msg: Message already in database (retry, masked match)",
  );
  return;
}
```

Der bestehende exakte Vergleich (`msgNr` + `fromCall` + `msgTXT`) kann als zusätzlicher, engerer
Test bestehen bleiben oder entfallen — der maskierte Vergleich deckt ihn ab, sobald beide
Nachrichten von demselben Absender im Fenster liegen.

### b) ACK-Zuordnung: maskiert und auf eigene DMs eingeschränkt

Zwei unabhängige Lücken in `ackTxtMsg` (Abschnitt 2.2):

1. Der Vergleich ist heute schon _zu weit_, weil er ausschließlich auf `msgNr` filtert — jede
   eigene Nachricht (Text, Position, HEY, Telemetrie; sie teilen sich denselben Zähler,
   Konzeptpapier §2.1) mit zufällig derselben msg_id würde mitaktualisiert.
2. Mit Variante a) kommt eine binäre ACK mit der Original-msg_id, muss aber auch dann noch die
   _richtige_ Zeile treffen, wenn zwei eigene DMs an unterschiedliche Ziele zufällig denselben
   30-Bit-Kern tragen (nach einem Zähler-Wrap, siehe a).

```ts
// Vorschlag für DataBaseService.ackTxtMsg (ersetzt den Query ab :345)
const maskedNr = (msgNr & 0xfffff3ff) >>> 0;
const currentCallsign = ConfigObject.getConf().CALL;
const res = await DatabaseService.db.query(
  `SELECT * FROM TextMessages WHERE (msgNr & 4294964223) = ${maskedNr} AND fromCall = '${currentCallsign}' AND isDM = 1`,
);
```

Die Einschränkung auf `isDM = 1` ist bewusst: Gruppen-/`*`-Meldungen laufen weiterhin mit gleicher
msg_id über alle Aussendungen (Konzeptpapier §4.3 "Nur für PN"), ihr ACK-Pfad ändert sich nicht und
soll von der maskierten PN-Logik nicht mitgetroffen werden. Bleibt nach dieser Einschränkung mehr
als eine Zeile übrig, ist zusätzlich ein enges Zeitfenster (wie in a) sinnvoll, um zwei eigene DMs
mit kollidierendem 30-Bit-Kern zu trennen.

### c) Statusanzeige: "gesendet" / "gehört" / "quittiert" / neu "unbestätigt"

Heute zeigt `ack === 1` ("gehört", `cloudOutline`, Chat.tsx:1311-1313) ein Echo, das laut
Konzeptpapier §4.1 nur bedeutet "die PN ist unterwegs", nicht dass sie angekommen ist — genau die
Verwechslung, die Meshtastic in seiner App vermeidet ("weitergeleitet, nicht bestätigt" statt
"zugestellt", Konzeptpapier §5.2). Mit Variante a) hört der Absender bis zu 3 Wiederholungen im
40-s-Abstand (Konzeptpapier §2.3, §4.3); die letzte liegt bei t ≈ 80–120 s. Ein Timeout von rund
4 Minuten (4 Aussendungen ~40 s auseinander ≈ 120–160 s, plus Marge für Funklaufzeit und
Serverpfad) ist ein sicherer Punkt, um "unbestätigt" statt "gehört" zu zeigen, wenn bis dahin kein
`ack_type` 0x01/0x02 eingetroffen ist.

```ts
// Vorschlag: reiner Anzeige-Zustand, kein neues DB-Feld nötig
const RETRY_TIMEOUT_MS = 4 * 60 * 1000; // 4 Aussendungen ~40s + Marge, s. Abschnitt 3c

function displayAckState(
  msg: MsgType,
  now: number,
): "sent" | "heard" | "acked" | "unconfirmed" {
  if (msg.ack === 2) return "acked";
  const age = now - msg.timestamp;
  if (age > RETRY_TIMEOUT_MS) return "unconfirmed";
  return msg.ack === 1 ? "heard" : "sent";
}
```

In Chat.tsx würde ein vierter Icon-Zweig (z. B. ein durchgestrichenes Wolken-Symbol) den Fall
`unconfirmed` neben den bestehenden drei Zweigen (1308–1316) ergänzen; der Timer läuft rein
clientseitig über die vorhandene `timestamp`-Spalte, ohne dass die Firmware oder das Frame-Format
etwas Neues liefern muss.

### d) Kosmetisch: `{NNN` auch bei fremden DMs entfernen

Wie in Abschnitt 2.3 gezeigt, gilt die Entfernung von `{NNN` heute nur für eigene Nachrichten
(`from_callsign_ === node_call_ref.current`, MessageHandler.ts:554). Eine empfangene PN von einem
fremden Rufzeichen zeigt die Nummer weiterhin im Chattext. Das ist ein bestehender, von dieser
Wiederholungs-Variante unabhängiger Schönheitsfehler — die PN-Nummer ändert sich durch Variante a)
nicht (Konzeptpapier §4.3: "Die NNN ist in a) übrigens für alle Aussendungen ebenfalls gleich").
Wer ihn beheben will, lockert die Bedingung in Zeile 554 auf `isDM_ === 1` unabhängig vom Absender:

```ts
// src/hooks/MessageHandler.ts:554, gelockerte Bedingung (optional, rein kosmetisch)
if (msg_text_.includes("{") && isDM_ === 1) {
  const last_sign_index = msg_text_.lastIndexOf("{");
  msg_text_ = msg_text_.slice(0, last_sign_index);
}
```

## 4. Was sich nicht ändern darf

- **Die App wiederholt nichts automatisch.** Der Knoten besitzt die Wiederholungslogik
  (Konzeptpapier §5.1 "Nicht übertragbar: Wiederholung durch den Client. […] Bei MeshCom steuert
  der Knoten sie, und das soll so bleiben, weil viele Knoten ohne App laufen."). Der bestehende
  "Resend"-Pfad (Chat.tsx:687–700) bleibt eine **manuelle**, vom Nutzer ausgelöste Aktion, die eine
  komplett neue Nachricht (neue msg_id, neuer Sendezyklus) erzeugt — kein Bestandteil des
  Protokoll-Retries und daher unverändert zu lassen.
- **Die App interpretiert keine msg_id-Bits selbst um.** Maskierung auf 30 Bit ist reine
  Dedup-/Zuordnungslogik (Abschnitt 3a/3b); die App zeigt weiterhin die vom Knoten gelieferten
  Werte (`msgID`, `ack_state`, `ack_call`) unverändert an.
- **Gruppen-/`*`-Nachrichten bleiben vom neuen Dedup- und ACK-Pfad ausgenommen** (Abschnitt 3b),
  weil Variante a) laut Konzeptpapier nur für PN gilt.

## 5. Testfälle

| #   | Szenario                                                                                                            | Erwartung                                                                                                                                                   |
| --- | ------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| T1  | Knoten mit neuer FW, PN kommt ohne Wiederholung an                                                                  | Ein Chateintrag, `ack` durchläuft 0 → 1 → 2 wie heute                                                                                                       |
| T2  | Knoten mit neuer FW, 2. Aussendung nötig (Echo, kein ACK)                                                           | Weiterhin ein Chateintrag; Status zeigt "gehört", nach Timeout ggf. "unbestätigt", falls das ACK ausbleibt                                                  |
| T3  | Knoten mit neuer FW, ACK trifft erst nach der 3. Aussendung ein                                                     | `ackTxtMsg` trifft trotz mehrerer Aussendungen genau die eine Zeile (maskierter Vergleich, 3b); Status springt auf "quittiert"                              |
| T4  | Knoten mit alter FW, 3 Wiederholungen kommen mit je eigener msg_id an                                               | Ohne Änderung 3a: bis zu 3 zusätzliche Chateinträge. Mit 3a: maskierter Dedup erkennt sie als dieselbe PN innerhalb des 10-min-Fensters, ein Eintrag bleibt |
| T5  | Zwei verschiedene PN (fromCall gleich) mit kollidierendem 30-Bit-Kern, aber Zeitabstand > 10 min (nach Zähler-Wrap) | Beide erscheinen als getrennte Chateinträge — das Zeitfenster verhindert Fehl-Dedup                                                                         |
| T6  | Eigene DM an Ziel A und eigene DM an Ziel B kurz hintereinander, zufällig kollidierender 30-Bit-Kern                | ACK für A trifft nicht die Zeile von B (3b, `isDM=1` + Zeitfenster)                                                                                         |
| T7  | Empfangene PN von fremdem Rufzeichen                                                                                | `{NNN` bleibt im Text (heutiges Verhalten, 2.3) — bzw. wird entfernt, falls 3d umgesetzt wird                                                               |
| T8  | Manuelles "Resend" nach ausbleibendem ACK                                                                           | Text und Zielrufzeichen erscheinen im Eingabefeld, kein automatischer Versand (Abschnitt 4)                                                                 |
| T9  | Gruppen-/`*`-Nachricht mit Wiederholung                                                                             | Läuft weiter über den heutigen, unveränderten Pfad (msgNr exakt, kein `isDM`-Filter greift)                                                                 |

## 6. Aufwand

| Punkt                                                 | Aufwand | Begründung                                                                                                                             |
| ----------------------------------------------------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| a) Dedup auf maskierte msgNr + Zeitfenster            | **S**   | Eine SQL-Query in `writeTxtMsg` (DataBaseService.ts:283-288) anpassen                                                                  |
| b) ACK-Zuordnung maskiert + `isDM`/eigenes Rufzeichen | **S**   | Eine SQL-Query in `ackTxtMsg` (DataBaseService.ts:345, 372) anpassen                                                                   |
| c) Vierter Statuszustand "unbestätigt"                | **M**   | Neue Anzeige-Logik plus UI-Zweig in Chat.tsx (1304-1316); kein Schema-/Protokolländerung, aber Timer-Handling pro sichtbarer Nachricht |
| d) `{NNN` auch bei fremden DMs entfernen              | **S**   | Eine Bedingung in MessageHandler.ts:554 lockern; rein kosmetisch, kein Datenmodell betroffen                                           |

Keiner der vier Punkte ist Voraussetzung für die Firmware-Änderung (Abschnitt 1); sie sind
Härtung gegen alte Firmware in der Flotte (a, d) bzw. Verbesserung der Ehrlichkeit der
Statusanzeige (b, c).
