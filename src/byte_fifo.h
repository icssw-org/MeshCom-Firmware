// byte_fifo: ein Ring aus Bytes statt aus Schlitzen.
//
// Drei Ausgangsringe (Telefon-Daten, Telefon-Kommandos, UDP-Ausgang) lagen als
// [N][260]-Felder im statischen RAM: 20 Schlitze zu 260 Byte, egal ob ein
// Frame 30 oder 200 Byte hat. Gemessen im Dauerlauf (DK5EN-1, 12 h, 639
// Frames) ist ein Frame im Mittel 77 Byte lang, der Schlitz war also zu 70 %
// Luft. Dieser Ring legt die Frames dicht hintereinander ab:
//
//     [len][payload ...][len][payload ...] ...
//
// Ein Laengenbyte, dann die Nutzlast, dann sofort der naechste Frame. Frames
// duerfen am Ende des Speichers umbrechen. Es gibt keine Kompression und
// keine Kodierung, die Nutzlast liegt byte-genau so drin, wie der Erzeuger
// sie geliefert hat -- die Ersparnis ist allein die weggefallene Reserve.
//
// Zwei Lesestellen, weil die alten Ringe zwei Leser hatten:
//
//   tail    -- der EINE Verbraucher (BLE-Sender, UDP-Drain). Liest den
//              aeltesten ungelesenen Frame und rueckt mit pop() weiter.
//   oldest  -- der Anfang des Verlaufs. Ein Frame bleibt nach dem Lesen
//              liegen, bis der Platz gebraucht wird; die Web-Oberflaeche
//              liest die Nachrichtenseite aus diesem Verlauf (so wie sie
//              vorher den ganzen Schlitzring von toPhoneWrite an durchging).
//
// Verdraengung: Wer schreibt und keinen Platz hat, wirft die aeltesten Frames
// weg, bis es passt. Trifft das einen UNGELESENEN Frame, wandert tail mit --
// dasselbe Verhalten wie addRingPointer(), das den Lesezeiger vor sich her
// schob, nur dass hier gezaehlt wird, wie viele ungelesene Frames es traf.
//
// Nebenlaeufigkeit: auf nRF52 schreibt OnRxDone aus dem Timer-Service-Task,
// gelesen wird aus der Hauptschleife (CONC-15/16). Jede Operation hier
// sperrt sich selbst (BF_LOCK), Aufrufer brauchen keinen eigenen kritischen
// Abschnitt mehr. Auf ESP32 ist BF_LOCK ein Spinlock-Kritischer-Abschnitt (portMUX), im Host-Test leer
// bzw. der Test-Haken.
//
// Kein IRAM: keine Funktion traegt IRAM_ATTR, es wird nichts aus der Liste
// der IRAM-gepinnten libc-Mitglieder gezogen (nur memcpy).

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

struct byte_fifo_t
{
    uint8_t *buf;
    uint16_t cap;
    volatile uint16_t head;      // naechstes freies Byte
    volatile uint16_t tail;      // aeltester ungelesener Frame
    volatile uint16_t oldest;    // aeltester noch vorhandener Frame
    volatile uint16_t used;      // belegte Bytes von oldest bis head
    volatile uint16_t frames;    // Frames von oldest bis head
    volatile uint16_t unread;    // davon ungelesen (von tail bis head)
    volatile uint16_t tail_gen;  // zaehlt jede Bewegung von tail
    volatile uint16_t evict_gen; // zaehlt jede Verdraengung
};

// Statische Initialisierung: byte_fifo_t ring = BYTE_FIFO_INIT(storage);
#define BYTE_FIFO_INIT(storage) { (storage), (uint16_t)sizeof(storage), 0, 0, 0, 0, 0, 0, 0, 0 }

// Alles auf leer, Speicher bleibt.
void bf_reset(byte_fifo_t *f);

// Einen Frame aus zwei Teilen anhaengen (b darf NULL/0 sein). alen+blen muss
// 1..255 sein und in den Ring passen, sonst -1 und nichts passiert.
// Rueckgabe sonst: Anzahl UNGELESENER Frames, die verdraengt wurden (0 im
// Normalfall). Verdraengte, schon gelesene Frames zaehlen nicht -- die sind
// nur Verlauf.
int bf_push2(byte_fifo_t *f, const uint8_t *a, uint8_t alen, const uint8_t *b, uint8_t blen);

static inline int bf_push(byte_fifo_t *f, const uint8_t *a, uint8_t alen)
{
    return bf_push2(f, a, alen, NULL, 0);
}

// Aeltesten ungelesenen Frame kopieren, ohne ihn zu entnehmen. Kopiert
// hoechstens outmax Byte, liefert die VOLLE Frame-Laenge (0 = nichts da).
uint8_t bf_peek(byte_fifo_t *f, uint8_t *out, uint16_t outmax);

// Aeltesten ungelesenen Frame entnehmen (er bleibt als Verlauf liegen).
void bf_pop(byte_fifo_t *f);

// BLE-N1 (Advisor): peek() und tail_gen in EINER Sperre. Getrennt gelesen kann
// ein Schreiber (nRF52: OnRxDone im LORA-Task) zwischen beiden den Frame
// verdraengen -- der Leser haelt dann Frame A mit der Generation von B und
// pop()t spaeter B, der nie gesendet wurde.
uint8_t bf_peek_gen(byte_fifo_t *f, uint8_t *out, uint16_t outmax, uint16_t *gen);

// pop() nur, wenn der aelteste Frame noch der mit Generation gen ist -- Vergleich
// und Entnahme unter einer Sperre. false: er wurde inzwischen verdraengt oder
// entnommen, es wurde nichts entnommen.
bool bf_pop_if(byte_fifo_t *f, uint16_t gen);

static inline bool bf_empty(const byte_fifo_t *f) { return f->unread == 0; }
static inline uint16_t bf_unread(const byte_fifo_t *f) { return f->unread; }
static inline uint16_t bf_frames(const byte_fifo_t *f) { return f->frames; }
static inline uint16_t bf_used(const byte_fifo_t *f) { return f->used; }

// Fuer den Leser, der zwischen peek() und pop() etwas Langsames tut (UDP
// senden): vorher merken, hinterher vergleichen. Ungleich heisst, ein
// Schreiber hat den Frame in der Zwischenzeit verdraengt -- dann nicht
// pop()en, sonst geht ein anderer Frame verloren.
static inline uint16_t bf_tail_gen(const byte_fifo_t *f) { return f->tail_gen; }

// Verlauf lesen, vom aeltesten Frame zum juengsten. Endet mit 0, auch dann,
// wenn waehrend des Lesens verdraengt wurde (der Rest des Verlaufs ist dann
// nicht mehr verlaesslich adressierbar; die Seite zeigt, was sie hat).
struct bf_iter_t
{
    uint16_t pos;
    uint16_t left;
    uint16_t gen;
};

void bf_iter_begin(const byte_fifo_t *f, bf_iter_t *it);
uint8_t bf_iter_next(byte_fifo_t *f, bf_iter_t *it, uint8_t *out, uint16_t outmax);

// Speicher loesen / anbinden (BLE-Pause, AU-17): der Ring gibt seinen Puffer her
// und verhaelt sich danach wie ein leerer Ring ohne Platz -- push liefert -1
// (Frame verworfen), peek 0, pop tut nichts, der Verlauf ist leer. Beide laufen unter
// derselben Sperre (BF_LOCK) wie jede andere Operation: cap und buf werden atomar zu den
// Zugriffen anderer Tasks gesetzt, danach kann KEIN Zugriff mehr auf dem alten Puffer
// sein. Der Aufrufer darf den zurueckgelieferten Puffer sofort freigeben. Frames, die vor
// dem Loesen im Ring lagen, sind nach bf_attach weg. Idempotent: ein zweites bf_detach
// liefert NULL. Nicht auf nRF52 verwenden (dort kein Pause).
uint8_t *bf_detach(byte_fifo_t *f);

// Neuen Speicher anbinden, Ring leer (ein zweites bf_attach ersetzt den Puffer; der
// alte Zeiger geht dabei verloren -- Aufrufer pruefen bf_has_storage()).
void bf_attach(byte_fifo_t *f, uint8_t *storage, uint16_t cap);

// Mehrere Ringe zusammen anbinden, ganz oder gar nicht: alloc(caps[i]) fuer jeden Ring ohne
// Speicher; scheitert eine Zuteilung, wird alles Zugeteilte per release() zurueckgegeben
// und kein Ring angefasst (false). Ringe mit Speicher werden uebersprungen (idempotent).
// n <= BF_ATTACH_MAX.
#define BF_ATTACH_MAX 4
typedef void *(*bf_alloc_fn)(size_t);
typedef void (*bf_free_fn)(void *);
bool bf_attach_all(byte_fifo_t *const *rings, const uint16_t *caps, uint8_t n, bf_alloc_fn alloc,
                   bf_free_fn release);

// Alle Ringe loesen und den Speicher sofort per release() freigeben.
void bf_detach_free(byte_fifo_t *const *rings, uint8_t n, bf_free_fn release);

static inline bool bf_has_storage(const byte_fifo_t *f) { return f->cap != 0; }
