#ifndef _BLE_JSON_FRAME_H_
#define _BLE_JSON_FRAME_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ArduinoJson.h>

// Serialisiert ein JSON-Dokument in einen BLE-Rahmenpuffer: buf[0] ist das
// Typbyte (z.B. 0x44 'D'), das JSON beginnt bei buf+1.
//
// Die Schranke ist die Puffergroesse, nicht die gemessene JSON-Laenge:
// serializeJson() schreibt hoechstens bufsize-1 Bytes und liefert die
// tatsaechlich geschriebene Anzahl. Mit measureJson()+1 als Schranke waere ein
// Dokument, das laenger ist als der Puffer, ein Stack-Ueberlauf (BND-03).
//
// Rueckgabe: Rahmenlaenge (Typbyte + JSON), nie groesser als bufsize. Ein
// abgeschnittenes Dokument ist kein gueltiges JSON mehr - das ist dieselbe
// Eigenschaft, die addBLEComToOutBuffer() beim Klemmen auf 245 Byte ohnehin
// hat, aber ohne Speicherverletzung.
static inline uint16_t bleJsonFrame(const JsonDocument &doc, uint8_t *buf, size_t bufsize)
{
    if (buf == nullptr || bufsize < 2)
        return 0;
    size_t json_len = serializeJson(doc, (char *)buf + 1, bufsize - 1);
    return (uint16_t)(json_len + 1);
}

// JSN-01: fail-soft variant of bleJsonFrame() for a document that may not
// fit payload_max characters of JSON. Buffer-overflow safety alone
// (bleJsonFrame() above) is not enough here: a document that does not fit
// gets serialised up to the buffer limit and the byte stream is cut wherever
// that limit falls -- typically mid string, mid escape sequence, or mid
// object -- producing bytes that are no longer valid JSON at all. This
// truncates the *document* instead: it drops trailing members (everything
// but "TYP", which every consumer switches on to know what the document is)
// and re-measures until the document fits payload_max, then hands the now
// smaller-or-equal document to bleJsonFrame(). What comes out is always a
// complete, parseable JSON object -- possibly missing optional fields, never
// missing a closing quote or brace. Same pattern as updateMHeard()'s
// PP/DIST fallback (mheard_functions.cpp).
static inline uint16_t bleJsonFrameFailSoft(JsonDocument &doc, uint8_t *buf, size_t bufsize, size_t payload_max)
{
    JsonObject obj = doc.as<JsonObject>();

    while (measureJson(doc) > payload_max && obj.size() > 1)
    {
        const char *drop = nullptr;
        for (JsonPair kv : obj)
        {
            if (strcmp(kv.key().c_str(), "TYP") != 0)
                drop = kv.key().c_str();
        }
        if (drop == nullptr)
            break;
        obj.remove(drop);
    }

    return bleJsonFrame(doc, buf, bufsize);
}

#endif
