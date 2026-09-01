#include <Arduino.h>
#include <atomic>
#include <string.h>

// Spitze Klammern (nicht Anfuehrungszeichen) fuer configuration.h und
// printfdeb_functions.h: Anfuehrungszeichen suchen zuerst im Verzeichnis
// DIESER Datei (src/) und wuerden im nativen Testbuild an den Shims in
// test/support/ vorbeigreifen -- siehe die gleiche Begruendung in
// txring_functions.cpp.
#include <configuration.h>
#include <printfdeb_functions.h>

#include "capture_functions.h"

bool bTXCAPTURE = false;

#if MC_CAPTURE   // bTXCAPTURE oben bleibt immer definiert (siehe Header)

// Der RX-Mitschnitt haengt an bLORADEBUG (lora_functions.cpp). Hier nur
// deklariert statt ueber loop_functions_extern.h gezogen: dieses Modul soll
// nativ ohne den ganzen Loop-Header uebersetzbar bleiben. Der Typ muss exakt
// zur Definition in loop_functions.cpp passen (bool) -- eine abweichende
// Deklaration waere ein ODR-Verstoss.
extern bool bLORADEBUG;

// 768 Byte: ~9 typische Positionsbaken (45-130 Byte) oder 2 Maximalframes.
// Kleiner macht den Mitschnitt in Kollisionslagen loechrig, groesser kostet
// RAM, den nRF52 nicht hat.
#define CAPTURE_RING_BYTES  768

// Kopf je Satz: dir(1) len(2) rssi(2) snr(1)
#define CAPTURE_HDR_LEN     6

static uint8_t cap_ring[CAPTURE_RING_BYTES];

// Konsument ist ausschliesslich der Loop (captureDrain()); er fasst nur
// cap_tail schreibend an. Die Sichtbarkeitsordnung (release/acquire) sorgt
// dafuer, dass er die Nutzdaten sieht bevor er den neuen Kopfstand sieht.
static std::atomic<uint16_t> cap_head{0};   // naechste Schreibposition
static std::atomic<uint16_t> cap_tail{0};   // naechste Leseposition
static std::atomic<uint32_t> cap_dropped{0};

// Nur vom Konsumenten (Loop) angefasst, deshalb kein Atomic noetig.
static unsigned long cap_serial_last = 0;

// Produzenten sind ZWEI: OnRxDone() und doTX(). Auf nRF52 laufen die in
// verschiedenen Tasks (Timer-Service bzw. Loop), der Radio-Callback kann
// doTX() also mitten im Schreiben unterbrechen -- beide haetten dann denselben
// Kopfstand gelesen und wuerden einander ueberschreiben. Ein einfaches SPSC-
// Schema traegt hier nicht.
//
// Ein Spinlock waere an dieser Stelle falsch: unterbricht der Timer-Task den
// Loop-Task waehrend dieser das Schloss haelt, dreht der Timer-Task fuer
// immer. Deshalb try-lock -- wer nicht sofort hineinkommt, verwirft seinen
// Frame und zaehlt ihn. Ein gezaehlter Verlust ist harmlos, ein zerrissener
// Ring nicht.
static std::atomic_flag cap_writing = ATOMIC_FLAG_INIT;

static inline uint16_t ring_free(uint16_t head, uint16_t tail)
{
    // Fallunterscheidung statt (head - tail) % CAPTURE_RING_BYTES: uint16_t
    // wird vor der Subtraktion nach int befoerdert, bei head < tail (also nach
    // jedem Umlauf) ist die Differenz negativ, und % liefert in C dann einen
    // negativen Rest. Der Rueckweg nach uint16_t machte daraus einen riesigen
    // "used"-Wert -- und damit reichlich vermeintlich freien Platz, ueber den
    // der naechste Frame ungelesene Saetze ueberschrieb. CAPTURE_RING_BYTES
    // ist keine Zweierpotenz, die Maskierungsabkuerzung gibt es hier nicht.
    // Nachgewiesen von test_capture_ring/test_ueberlauf_wird_gezaehlt_und_gemeldet.
    uint16_t used = (head >= tail)
                  ? (uint16_t)(head - tail)
                  : (uint16_t)(CAPTURE_RING_BYTES - tail + head);

    // Ein Byte bleibt frei, sonst waere voll von leer nicht unterscheidbar.
    return (uint16_t)(CAPTURE_RING_BYTES - used - 1);
}

static inline void ring_put(uint16_t &pos, uint8_t b)
{
    cap_ring[pos] = b;
    pos = (uint16_t)((pos + 1) % CAPTURE_RING_BYTES);
}

static inline uint8_t ring_get(uint16_t &pos)
{
    uint8_t b = cap_ring[pos];
    pos = (uint16_t)((pos + 1) % CAPTURE_RING_BYTES);
    return b;
}

void captureFrame(char dir, const uint8_t *buf, uint16_t len, int16_t rssi, int8_t snr)
{
    if(buf == NULL || len == 0)
        return;

    if(len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE;

    if(cap_writing.test_and_set(std::memory_order_acquire))
    {
        // Der andere Produzent schreibt gerade.
        cap_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    uint16_t head = cap_head.load(std::memory_order_relaxed);
    uint16_t tail = cap_tail.load(std::memory_order_acquire);

    uint16_t need = (uint16_t)(CAPTURE_HDR_LEN + len);
    if(ring_free(head, tail) < need)
    {
        cap_dropped.fetch_add(1, std::memory_order_relaxed);
        cap_writing.clear(std::memory_order_release);
        return;
    }

    uint16_t pos = head;
    ring_put(pos, (uint8_t)dir);
    ring_put(pos, (uint8_t)(len & 0xFF));
    ring_put(pos, (uint8_t)((len >> 8) & 0xFF));
    ring_put(pos, (uint8_t)(rssi & 0xFF));
    ring_put(pos, (uint8_t)((rssi >> 8) & 0xFF));
    ring_put(pos, (uint8_t)snr);
    for(uint16_t i = 0; i < len; i++)
        ring_put(pos, buf[i]);

    // Erst jetzt veroeffentlichen: alles oben Geschriebene ist vor diesem
    // Store sichtbar.
    cap_head.store(pos, std::memory_order_release);

    cap_writing.clear(std::memory_order_release);
}

bool captureFormatNext(char *out, size_t outsz)
{
    if(out == NULL || outsz == 0)
        return false;

    uint16_t tail = cap_tail.load(std::memory_order_relaxed);
    uint16_t head = cap_head.load(std::memory_order_acquire);

    if(tail == head)
    {
        // Verluste auch dann melden, wenn der Ring gerade leergelaufen ist --
        // sonst bliebe die Meldung genau in der Lage aus, die sie erzeugt hat.
        uint32_t lost = cap_dropped.exchange(0, std::memory_order_relaxed);

        // Zweite Verlustquelle, unabhaengig vom Ring: printfdeb() verwirft auf
        // nRF52 Ausgabe, wenn der USB-CDC-Puffer voll bleibt. Ein Frame kann
        // also sauber durch den Ring laufen und trotzdem unvollstaendig oder
        // gar nicht im Log landen. Beide Zahlen gehoeren nebeneinander, sonst
        // erklaert die eine die Luecken, die die andere gerissen hat.
        unsigned long ser_now = printfdebDroppedBytes();
        unsigned long ser_new = ser_now - cap_serial_last;   // Ueberlauf: modulo, korrekt
        cap_serial_last = ser_now;

        // Bei abgeschaltetem Mitschnitt nicht melden -- der Zaehlerstand ist
        // oben trotzdem nachgezogen, damit das Einschalten keinen Rueckstau
        // aus der Zeit davor meldet.
        if(!bLORADEBUG && !bTXCAPTURE)
            return false;

        if(lost == 0 && ser_new == 0)
            return false;

        snprintf(out, outsz, "[MC-TEST] CAPTURE_DROPPED n=%lu serial_bytes=%lu",
                 (unsigned long)lost, ser_new);
        return true;
    }

    uint16_t pos = tail;
    char     dir  = (char)ring_get(pos);
    uint16_t len  = (uint16_t)ring_get(pos);
    len |= (uint16_t)((uint16_t)ring_get(pos) << 8);
    int16_t  rssi = (int16_t)ring_get(pos);
    rssi |= (int16_t)((int16_t)ring_get(pos) << 8);
    int8_t   snr  = (int8_t)ring_get(pos);

    if(len > UDP_TX_BUF_SIZE)
        len = UDP_TX_BUF_SIZE;   // kann nur bei zerstoertem Ring auftreten

    // Hex direkt in den Ausgabepuffer schreiben statt ueber einen zweiten
    // 511-Byte-Zwischenpuffer: der haette denselben Inhalt ein zweites Mal
    // im RAM gehalten, und RAM ist auf nRF52 das knappe Gut.
    int off;
    if(dir == 'T')
        off = snprintf(out, outsz, "[MC-TEST] TX_FRAME len=%u hex=", (unsigned)len);
    else
        off = snprintf(out, outsz, "[MC-TEST] RX_FRAME len=%u rssi=%d snr=%d hex=",
                       (unsigned)len, (int)rssi, (int)snr);

    if(off < 0 || (size_t)off >= outsz)
    {
        // Der Kopf passte schon nicht. Der Ring muss trotzdem weiterruecken,
        // sonst laege derselbe Satz beim naechsten Aufruf wieder an.
        for(uint16_t i = 0; i < len; i++)
            (void)ring_get(pos);
        cap_tail.store(pos, std::memory_order_release);
        if(outsz > 0)
            out[0] = 0x00;
        return false;
    }

    for(uint16_t i = 0; i < len; i++)
    {
        uint8_t b = ring_get(pos);          // immer lesen, auch wenn kein Platz
        if((size_t)off + 3 <= outsz)
        {
            snprintf(out + off, 3, "%02X", b);
            off += 2;
        }
    }
    out[off] = 0x00;

    cap_tail.store(pos, std::memory_order_release);
    return true;
}

void captureDrain(void)
{
    // 96 Byte Vorspann reichen fuer die laengste Kopfzeile
    // ("RX_FRAME len=255 rssi=-128 snr=-128 hex=") mit Reserve.
    static char line[2 * UDP_TX_BUF_SIZE + 96];

    if(captureFormatNext(line, sizeof(line)))
        printfdeb("%s\n", line);
}

#endif // MC_CAPTURE
