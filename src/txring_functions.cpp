#include "txring_functions.h"
#include "txring_ttl.h"

// Spitze Klammern (nicht Anfuehrungszeichen) fuer printfdeb_functions.h:
// Anfuehrungszeichen-Includes suchen zuerst im Verzeichnis der einschliessenden
// Datei (hier src/) und wuerden IMMER src/printfdeb_functions.h finden --
// unabhaengig von der -I-Reihenfolge in platformio.ini. Nur die
// Spitzklammer-Form respektiert "-I test/support" vor "-I src" und laesst so
// im nativen Testbuild test/support/printfdeb_functions.h (No-Op-Shim)
// greifen, so wie es <configuration.h>/<debugconf.h>/<nrf52/WisBlock-API.h>
// in loop_functions.h bereits vormachen.
#include <printfdeb_functions.h>

#include <loop_functions.h>
#include <loop_functions_extern.h>
#include <aprs_functions.h>

// setlogRingSourceCode() -- Header-Inline, keine Link-Abhaengigkeit auf
// setlog_lines.cpp (env:native_aprs baut diese TU, aber nicht setlog_lines.cpp).
#include <setlog_lines.h>

// M0-1 (0.4): ring enqueues per window and parked-slot overwrites, counted
// in addTxRingEntryCore() below for every caller (addTxRingEntry() and
// addTxRingEntryOnce() alike).
#include <dm_stats.h>

#if defined(NATIVE_BUILD)
// Native Testbuild: loop_functions.cpp (die kanonische Definitionsstelle
// dieser Globals, siehe loop_functions_extern.h) wird hier NICHT mitgebaut
// (build_src_filter in platformio.ini [env:native_aprs] listet nur einzelne
// Dateien explizit auf). Damit txring_functions.o eigenstaendig linkbar ist
// -- und zwar in JEDEM Test-Executable des Envs, nicht nur test_txring,
// da PlatformIO build_src_filter env-weit anwendet -- stellt dieser Block
// die Definitionen nur fuer NATIVE_BUILD bereit. Auf Hardware (ESP32/nRF52)
// ist NATIVE_BUILD nie gesetzt, dort bleibt loop_functions.cpp die einzige
// Definitionsstelle (kein Doppel-Definitions-Konflikt).
unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE + 5];
uint8_t retryCount[MAX_RING];
uint8_t ringPriority[MAX_RING];
uint32_t ringEnqueueTime[MAX_RING];
uint16_t stat_drop_count[6];
uint8_t stat_queue_hwm;
// SL-05: im Hardware-Build steht stat_ring_max in loop_functions.cpp neben
// ch_util_*_accum; nativ gilt dieselbe Begruendung wie fuer die Arrays oben.
std::atomic<uint8_t> stat_ring_max;
// Nachbarschaftsmatrix Stufe 2 (Feldlauf 23.09., siehe txring_functions.h):
// wie die Arrays oben kanonisch in loop_functions.cpp definiert (dort auch
// von den Toggle-Kommandos gesetzt), das native_aprs/native_udp_frame_twin-
// build_src_filter baut diese TU aber nicht mit -- txringInCaseBHold()/
// txringCaseBackoffSlot() unten lesen bNBRCANCEL, darum hier derselbe
// NATIVE_BUILD-Definitionszweig wie fuer ringBuffer usw.
bool bNBRCANCEL;
// GWRX-01: addUdpRxRingEntry() liest bNBRRELAY, gleiche Begruendung.
bool bNBRRELAY;
#endif

// SL-03/SL-06 (siehe txring_functions.h): Herkunft je Ring-Slot. Anders als
// ringPriority[]/ringEnqueueTime[] hier definiert und nicht in
// loop_functions.cpp -- geschrieben wird das Array ausschliesslich in dieser
// Datei, und so braucht der native Testbuild keinen zweiten Definitionszweig.
uint8_t ringSource[MAX_RING] = {0};

// Nachbarschaftsmatrix Stufe 2 (docs/nbr-wichtigkeit-konzept.md 5.1/5.2):
// wie ringSource[] oben unbedingt definiert (kein NATIVE_BUILD-Zweig
// noetig), geschrieben ausschliesslich in addTxRingEntry() unten.
// Welle 2 (edge pool): NbrMask (nbr_mask.h) statt uint32_t; Aggregat-Init
// nullt jedes Element (alle w[] Woerter 0), wie das alte {0} fuer uint32_t.
NbrMask  ringNeed[MAX_RING]  = {};
NbrMask  ringAlone[MAX_RING] = {};
uint8_t  ringKind[MAX_RING]  = {0};

//////////////////////////////////////////////////////////////////////////
// LoRa TX functions

// TX-01 (BACKLOG 3.8k): counts and rate-limits the "this node has no
// callsign yet, transmit refused" marker. Shared by both choke points --
// addTxRingEntry() below (so an unconfigured node's ring never even fills)
// and doTX() in lora_functions.cpp (the hard backstop at the only caller of
// Radio.Send()/startTransmit()). Raw Serial.printf, not printfdeb/DEBUG_MSG
// (stripped/compiled away with --debug off, see the FL-01 marker note) --
// at most one line per 10 s, folding any elided refusals into the
// "refused" count on the next line that does print (TM-21's lesson).
uint32_t stat_tx_refuse_unconfigured = 0;
uint32_t stat_supersede_count = 0;   // TXP W1c (D3)

void logTxRefuseUnconfigured(void)
{
    static bool s_have_marker = false;
    static uint32_t s_last_marker_ms = 0;
    static uint32_t s_refused_since_marker = 0;

    stat_tx_refuse_unconfigured++;
    s_refused_since_marker++;

    uint32_t now = (uint32_t)millis();
    // s_have_marker, not "s_last_marker_ms == 0": millis()==0 is a real,
    // reachable timestamp (boot, and every native-test setUp()), and must
    // not double as an "never printed yet" sentinel -- that would let a
    // second refusal still at ms=0 bypass the floor.
    if(!s_have_marker || (uint32_t)(now - s_last_marker_ms) >= 10000UL)
    {
        Serial.printf("[TX];refuse;unconfigured;ms;%lu;refused;%lu\n",
                      (unsigned long)now, (unsigned long)s_refused_since_marker);
        s_have_marker = true;
        s_last_marker_ms = now;
        s_refused_since_marker = 0;
    }
}

/**
 * Determine message priority from a frame (TXP W1b: no slot needed any more --
 * a pool without a free slot must still classify the NEW entry before it can
 * decide to evict or drop).
 * frame = ringBuffer[slot]+2: [0]=payload_type, [1-4]=msg_id, [5]=flags, [6+]=path
 * Encoded path format: "SOURCE>DEST:" starting at frame[6].
 * @param len    Frame length (= ringBuffer[slot][0])
 * @param status Slot status the entry is classified with (DONE + TEXT = relay)
 */
static uint8_t classifyFrame(const uint8_t *frame, int len, uint8_t status)
{
    uint8_t msg_type = frame[0];

    if(msg_type == MSG_TYPE_ACK)
        return MSG_PRIO_CRITICAL;

    if(msg_type == MSG_TYPE_POSITION)
        return MSG_PRIO_LOW;

    if(msg_type == MSG_TYPE_HEY)
        return MSG_PRIO_BACKGROUND;

    if(msg_type == MSG_TYPE_TEXT)
    {
        // Relay detection: OnRxDone sets RING_STATUS_DONE for relayed packets
        if(status == RING_STATUS_DONE)
            return MSG_PRIO_NORMAL; // Relay

        // User-originated text: parse destination from encoded APRS path
        // Path starts at frame[6] (after type+id+flags)
        // Format: "SOURCE>DEST:" -- find '>' then read until payload_type char
        int path_start = 6;
        int dest_start = -1;
        int dest_end = -1;

        for(int i = path_start; i < len && i < (UDP_TX_BUF_SIZE + 2); i++)
        {
            if(frame[i] == '>' && dest_start < 0)
            {
                dest_start = i + 1;
            }
            else if(dest_start >= 0 && (frame[i] == ':' || frame[i] == MSG_TYPE_TEXT))
            {
                dest_end = i;
                break;
            }
        }

        if(dest_start >= 0 && dest_end > dest_start)
        {
            char dest[12] = {0};
            int dlen = dest_end - dest_start;
            if(dlen > 10) dlen = 10;
            memcpy(dest, &frame[dest_start], dlen);
            dest[dlen] = 0;

            if(strcmp(dest, "*") == 0)
                return MSG_PRIO_HIGH; // Broadcast

            if(CheckGroup(String(dest)) != 0)
                return MSG_PRIO_HIGH; // Group message

            return MSG_PRIO_CRITICAL; // Personal DM
        }

        // Fallback: treat as high priority if parsing fails
        return MSG_PRIO_HIGH;
    }

    // Unknown type: normal priority
    return MSG_PRIO_NORMAL;
}

/**
 * Determine message priority from ring buffer slot content.
 * Ring buffer layout: [0]=len, [1]=status, [2]=payload_type, [3-6]=msg_id, [7]=flags, [8+]=path
 */
uint8_t getMessagePriority(int slot)
{
    return classifyFrame(ringBuffer[slot] + 2, ringBuffer[slot][0], ringBuffer[slot][1]);
}

/**
 * Find the next slot to transmit from the slot pool, based on priority.
 * TXP W1b: scans ALL MAX_RING slots (there is no read window any more).
 * Returns slot index, or -1 if nothing is sendable.
 * Lowest priority value wins; within the same priority the entry with the
 * earliest ringEnqueueTime wins (wrap-safe int32 compare, FIFO by age, not by
 * slot position); equal times fall back to the lower slot index.
 *
 * Nachbarschaftsmatrix Stufe 2, Feldlauf 23.09. (siehe txringInCaseBHold() in
 * txring_functions.h fuer den vollen Befund): ein Fall-B-Relay-Slot, der noch
 * in seiner einmaligen Sperre steckt, darf ab hier keinen anderen Slot mehr
 * blockieren. Der Scan fuehrt daher zwei Kandidaten mit: den besten NICHT
 * gehaltenen Slot (Prio dann FIFO, wie bisher) und den besten GEHALTENEN
 * Slot. Gibt es einen nicht gehaltenen Kandidaten, gewinnt der -- ein Fall-A-
 * Relay, eine eigene Sendung, ein ACK oder eine HN-Meldung (Prio 5) ueberholt
 * so einen gehaltenen Fall-B-Relay. Nur wenn ALLE Kandidaten gehalten sind
 * (Ring voll mit wartenden Fall-B-Relays), faellt die Auswahl auf den besten
 * gehaltenen zurueck -- exakt das heutige Verhalten, und der von
 * csma_compute_timeout_slot() dafuer berechnete Backoff ist dann der
 * Rest-Hold (txringCaseBackoffSlot()).
 */
int getNextTxSlot(void)
{
    uint32_t now_ms = (uint32_t)millis();

    int best_slot = -1;
    uint8_t best_prio = 255;
    int best_held_slot = -1;
    uint8_t best_held_prio = 255;

    for(int pos = 0; pos < MAX_RING; pos++)
    {
        // Only consider slots with data that are ready to send (READY or DONE/fire-and-forget).
        // Skips SENT..threshold (awaiting retransmission timer), EXT_PENDING
        // (a slot owned by an in-flight external TX must never be reselected)
        // and INFLIGHT (doTX() has it) -- none of them is READY or DONE.
        if(ringBuffer[pos][0] > 0 &&
           (ringBuffer[pos][1] == RING_STATUS_READY || ringBuffer[pos][1] == RING_STATUS_DONE))
        {
            uint8_t prio = ringPriority[pos];
            if(txringInCaseBHold(pos, now_ms))
            {
                if(prio < best_held_prio ||
                   (prio == best_held_prio && (int32_t)(ringEnqueueTime[pos] - ringEnqueueTime[best_held_slot]) < 0))
                {
                    best_held_prio = prio;
                    best_held_slot = pos;
                }
            }
            else
            {
                if(prio < best_prio ||
                   (prio == best_prio && (int32_t)(ringEnqueueTime[pos] - ringEnqueueTime[best_slot]) < 0))
                {
                    best_prio = prio;
                    best_slot = pos;
                }
            }
        }
    }

    return (best_slot >= 0) ? best_slot : best_held_slot;
}

/**
 * TXP W1b: the three small state transitions doTX() performs on a slot, kept
 * here (not inline in lora_functions.cpp) so the native tests cover them.
 *
 * txRingBeginTx(): claim a picked slot for the radio. Only a slot that still
 * holds data and is READY or DONE can be claimed (on nRF52 OnRxDone may have
 * released it between getNextTxSlot() and here). *prevStatus receives the
 * status to restore on rollback. The slot keeps its length, so an enqueue
 * cannot take it while the frame is being handed to the radio.
 */
bool txRingBeginTx(int slot, uint8_t *prevStatus, const uint32_t *expectEnqueueTime)
{
    bool ok = false;
#if defined(NRF52_SERIES)
    taskENTER_CRITICAL();
#endif
    if(slot >= 0 && slot < MAX_RING && ringBuffer[slot][0] > 0 &&
       (ringBuffer[slot][1] == RING_STATUS_READY || ringBuffer[slot][1] == RING_STATUS_DONE) &&
       (expectEnqueueTime == nullptr || ringEnqueueTime[slot] == *expectEnqueueTime))
    {
        *prevStatus = ringBuffer[slot][1];
        ringBuffer[slot][1] = RING_STATUS_INFLIGHT;
        ok = true;
    }
#if defined(NRF52_SERIES)
    taskEXIT_CRITICAL();
#endif
    return ok;
}

/**
 * txRingFinishTx(): end of the in-flight window.
 *  TXRING_FIN_SENT     frame went to the radio: an own text that was READY
 *                      (type TEXT) is parked as SENT with its length kept for
 *                      the retry ladder; everything else (one-shot DONE, READY
 *                      non-text) is consumed = slot freed (len 0).
 *  TXRING_FIN_ROLLBACK nothing was sent: previous status restored, len stays,
 *                      the slot is picked again on the next call.
 *  TXRING_FIN_DROP     refused without a send (TX disabled, unconfigured,
 *                      decode failure): consumed, slot freed.
 * Acts only while the slot is still INFLIGHT: if an ACK scan resolved it in the
 * meantime (nRF52, other task) the result stands.
 */
void txRingFinishTx(int slot, uint8_t prevStatus, TxRingFinish outcome)
{
#if defined(NRF52_SERIES)
    taskENTER_CRITICAL();
#endif
    if(slot >= 0 && slot < MAX_RING && ringBuffer[slot][1] == RING_STATUS_INFLIGHT)
    {
        if(outcome == TXRING_FIN_ROLLBACK)
        {
            ringBuffer[slot][1] = prevStatus;
        }
        else if(outcome == TXRING_FIN_SENT &&
                prevStatus != RING_STATUS_DONE && ringBuffer[slot][2] == MSG_TYPE_TEXT)
        {
            ringBuffer[slot][1] = RING_STATUS_SENT; // parked, len kept
        }
        else
        {
            ringBuffer[slot][1] = RING_STATUS_DONE;
            ringBuffer[slot][0] = 0;                // consumed, slot is free
        }
    }
#if defined(NRF52_SERIES)
    taskEXIT_CRITICAL();
#endif
}

/**
 * stat_preempt_count definition (TXP W1b): "doTX picked a slot that was not the
 * oldest occupied one by enqueue time". Occupied = len > 0 in any status (a
 * parked SENT text counts: it is the oldest entry and the pick jumped over it).
 */
bool txRingIsPreempt(int slot)
{
    for(int i = 0; i < MAX_RING; i++)
    {
        if(i == slot || ringBuffer[i][0] == 0)
            continue;
        if((int32_t)(ringEnqueueTime[i] - ringEnqueueTime[slot]) < 0)
            return true;
    }
    return false;
}

/**
 * Nachbarschaftsmatrix Stufe 2, Feldlauf 23.09. -- siehe die Kommentare bei
 * den Deklarationen in txring_functions.h fuer den vollen Befund/die
 * Motivation. Beide Funktionen zusammen ersetzen den frueheren, bei JEDEM
 * CSMA-Re-Arm neu addierten Fall-B-Nachrang durch eine einmalige Deadline ab
 * Einreihen und lassen andere Slots waehrend dieser Sperre nicht mehr
 * mitwarten.
 */

// Gemeinsame Vorbedingung von txringInCaseBHold() und dem Fall-B-Zweig von
// txringCaseBackoffSlot(): bNBRCANCEL, Slot ist ein Relay (RING_KIND_RELAY,
// Zaehl-Kennbit RING_KIND_COUNTED maskiert), Fall B (ringAlone[slot]==0,
// nicht Fall A) und kein Text (Konzept 5.1 -- Text bleibt bei der normalen
// Prio-Basis, unveraendert; der Aufrufer in lora_functions.cpp filtert
// diesen Fall VOR dem Aufruf von txringCaseBackoffSlot() bereits aus,
// txringInCaseBHold() prueft ihn hier trotzdem selbst -- sie wird auch
// unabhaengig davon aus getNextTxSlot() aufgerufen).
static bool txring_is_case_b_relay(int slot)
{
    if(!bNBRCANCEL || slot < 0 || slot >= MAX_RING)
        return false;
    if((ringKind[slot] & 0x7F) != RING_KIND_RELAY)
        return false;
    if(!nbrMaskEmpty(ringAlone[slot]))
        return false;
    if(ringBuffer[slot][2] == MSG_TYPE_TEXT)
        return false;
    return true;
}

bool txringInCaseBHold(int slot, uint32_t now_ms)
{
    if(!txring_is_case_b_relay(slot))
        return false;

    // F8-Stil (siehe txRingAgeBackground()): rollover-sicherer Cast, damit
    // ein Wrap von now_ms/ringEnqueueTime[slot] ueber UINT32_MAX kein
    // negatives/riesiges "waited" liefert.
    uint32_t waited = (uint32_t)(now_ms - ringEnqueueTime[slot]);
    return waited < NBR_RELAY_CASE_B_EXTRA_MS;
}

unsigned long txringCaseBackoffSlot(int slot, int attempt, uint32_t now_ms)
{
    if(slot < 0 || slot >= MAX_RING)
        return 0; // defensiv; der Aufrufer garantiert einen gueltigen Relay-Slot

    if(!nbrMaskEmpty(ringAlone[slot]))
    {
        // Fall A: Vorrang, unveraendert -- nur Fall B litt unter dem
        // re-armten Hold (Feldlauf 23.09.), Fall A war nie betroffen.
        if((uint32_t)(now_ms - ringEnqueueTime[slot]) >= NBR_RELAY_CASE_A_MAX_WAIT_MS)
            return NBR_RELAY_CASE_A_SHORT_MS +
                   (unsigned long)random(0, NBR_RELAY_CASE_A_SLOTS) * CSMA_SLOT_SIZE;

        unsigned long base_a = NBR_RELAY_CASE_A_BASE_MS;
        if(attempt >= 2) base_a = base_a * 2 / 3;
        else if(attempt >= 1) base_a = base_a * 5 / 6;

        return base_a + (unsigned long)random(0, NBR_RELAY_CASE_A_SLOTS) * CSMA_SLOT_SIZE;
    }

    // Fall B (ringAlone[slot]==0). Text ist bereits vom Aufrufer
    // ausgefiltert (siehe Kopfkommentar) -- hier immer die Prio-Basis+Slots.
    uint32_t waited = (uint32_t)(now_ms - ringEnqueueTime[slot]); // F8-Stil, rollover-sicher

    unsigned long base_b;
    switch(ringPriority[slot]) {
        case MSG_PRIO_CRITICAL:   base_b = CSMA_PRIO_BASE_1; break;
        case MSG_PRIO_HIGH:       base_b = CSMA_PRIO_BASE_2; break;
        case MSG_PRIO_NORMAL:     base_b = CSMA_PRIO_BASE_3; break;
        case MSG_PRIO_LOW:        base_b = CSMA_PRIO_BASE_4; break;
        case MSG_PRIO_BACKGROUND: base_b = CSMA_PRIO_BASE_5; break;
        default:                  base_b = CSMA_PRIO_BASE_3; break;
    }
    if(attempt >= 2) base_b = base_b * 2 / 3;
    else if(attempt >= 1) base_b = base_b * 5 / 6;

    // Normale Fall-B-Basis: heutiger Wert OHNE NBR_RELAY_CASE_B_EXTRA_MS --
    // die frueher bei JEDEM Re-Arm neu addierte Sperre entfaellt hier, sie
    // wirkt nur noch unten als einmalige Deadline.
    unsigned long normal_b = base_b +
        (unsigned long)(NBR_RELAY_CASE_B_SLOT_START + random(0, 3)) * CSMA_SLOT_SIZE;

    if(waited < NBR_RELAY_CASE_B_EXTRA_MS)
    {
        // Noch innerhalb der einmaligen Sperre: der laengere der beiden
        // Werte gewinnt -- eine hohe Prio (kleine normale Basis) darf die
        // Sperre nicht per niedrigerer Basis umgehen, eine niedrige Prio
        // (grosse normale Basis) darf laenger warten als die reine Sperre.
        unsigned long remaining = NBR_RELAY_CASE_B_EXTRA_MS - waited;
        return (remaining > normal_b) ? remaining : normal_b;
    }

    if(waited < NBR_RELAY_CASE_B_MAX_WAIT_MS)
        return normal_b; // Sperre abgelaufen, weiterhin Nachrang -- ohne EXTRA

    // 60s-Deckel (NBR_RELAY_CASE_B_MAX_WAIT_MS, Feldlauf 23.09.: 82% der
    // Abbrueche fielen in die ersten 60s): danach wie Fall A nur noch die
    // Kurzsuche -- Schutzabstand nach Empfangsende, dann CAD.
    return NBR_RELAY_CASE_A_SHORT_MS +
           (unsigned long)random(0, NBR_RELAY_CASE_A_SLOTS) * CSMA_SLOT_SIZE;
}

/**
 * BP-01 (BACKLOG) / TM-37 / BP-02 (DJ8MEH-RCA): current TX-pool fill level.
 *
 * TXP W1b (slot pool): counts OCCUPIED slots (ringBuffer[i][0] > 0) over ALL
 * MAX_RING slots. Every slot holding a frame counts: queued (READY/DONE), on
 * air (INFLIGHT), waiting for an ACK (parked SENT) and external-radio pending
 * (EXT_PENDING) -- design decision D5 of the TX pool. "Full" is
 * depth == MAX_RING; the QRS/QRT back-pressure thresholds read this number.
 *
 * (History: BP-02 / DJ8MEH-RCA 2026-08-31 -- an 8-minute QRT episode ran on a
 * phantom depth of 19 while only 3-4 slots were occupied, because the old
 * ring counted the index distance; occupied slots are the honest number.)
 *
 * Deliberately lock-free: a read-only scan (no writes, no printf/malloc) over
 * up to MAX_RING slots. The worst a concurrent OnRxDone enqueue/evict on nRF52
 * can do mid-scan is make the answer stale by roughly the couple of slots that
 * write touches -- irrelevant for a back-pressure notice threshold that only
 * needs "about empty" vs. "still full", not an exact count.
 */
int txRingDepth(void)
{
    int depth = 0;
    for(int i = 0; i < MAX_RING; i++)
        if(ringBuffer[i][0] > 0)
            depth++;
    return depth;
}

/**
 * WQ-01 (2026-09-05): queue panel on the rxlog web page. Per-priority
 * occupancy count for the TX pool, using the exact same lock-free scan as
 * txRingDepth() above (all MAX_RING slots, occupied == ringBuffer[i][0] > 0,
 * INFLIGHT / parked SENT / EXT_PENDING included) -- see that function's doc
 * comment for the counting rationale and the lock-free tradeoff.
 *
 * out[0] receives the total occupied count (identical to txRingDepth());
 * out[1..5] receive the occupied count broken down by ringPriority[i]
 * (MSG_PRIO_CRITICAL..MSG_PRIO_BACKGROUND, i.e. 1..5). An occupied slot
 * whose priority is outside 1..5 only bumps out[0] -- defensive, since
 * getMessagePriority() never returns such a value today.
 *
 * No printf, no lock, no writes to the ring itself -- read-only, same as
 * txRingDepth().
 */
void txRingPrioCounts(uint8_t out[6])
{
    memset(out, 0, 6 * sizeof(out[0]));

    for(int i = 0; i < MAX_RING; i++)
    {
        if(ringBuffer[i][0] > 0)
        {
            out[0]++;
            uint8_t prio = ringPriority[i];
            if(prio >= 1 && prio <= 5)
                out[prio]++;
        }
    }
}

/**
 * TXP W1b (gauge G3, D5): own texts sent and waiting for an ACK/retry, i.e.
 * occupied slots whose status is in the SENT ladder (0x01..0x7F: the aging
 * counter 0x01..0x14 plus the transient threshold value 0x15; EXT_PENDING 0x80,
 * INFLIGHT 0x81, READY 0x00 and DONE 0xFF are outside). out[0] = total, out[1..5]
 * = per ringPriority[] (same bucket rules as txRingPrioCounts()). The sum is a
 * subset of txRingDepth(). Same lock-free read-only scan.
 */
void txRingAckWaitCounts(uint8_t out[6])
{
    memset(out, 0, 6 * sizeof(out[0]));

    for(int i = 0; i < MAX_RING; i++)
    {
        uint8_t st = ringBuffer[i][1];
        if(ringBuffer[i][0] > 0 && st >= RING_STATUS_SENT && st < RING_STATUS_EXT_PENDING)
        {
            out[0]++;
            uint8_t prio = ringPriority[i];
            if(prio >= 1 && prio <= 5)
                out[prio]++;
        }
    }
}

/**
 * BP-03 (DJ8MEH-RCA 2026-08-31, Teil 2; TXP W1a: now every priority via
 * txRingTtlMs(), plus the {CET} age gate): sweep the whole ring and drop every
 * BACKGROUND (HEY, prio 5) entry older than RING_BG_MAX_AGE_MS
 * (configuration_global.h). A priority-starved HEY waiting behind a
 * run of higher-priority traffic can sit for many minutes -- the DJ8MEH
 * blocker sat 10 min -- and a neighbourhood report that stale is worthless
 * on air. Tradeoff (a node's own HEY beacon ages out the same way) is
 * documented at the RING_BG_MAX_AGE_MS definition.
 *
 * Deliberately its OWN function, not folded into getNextTxSlot() (Advisor
 * F1, critical): getNextTxSlot() also runs on the nRF52 FreeRTOS timer task
 * (OnRxDone -> csma_compute_timeout(), lora_functions.cpp) and under the
 * EXTERNAL_RADIO bridge path -- neither may write the ring or call
 * printf/printfdeb. This function is instead called once per 2s tick from
 * the MAIN LOOP on both platforms (esp32_main.cpp/nrf52_main.cpp), right
 * next to updateRetransmissionStatus() -- the same context that already
 * frees slots today.
 *
 * Locking on nRF52: identical taskENTER_CRITICAL()/taskEXIT_CRITICAL()
 * pattern as addTxRingEntry() below, and for the same reason (Advisor F2):
 * the eviction path in addTxRingEntry() can overwrite a slot this sweep has
 * already looked at or is about to look at, from the OTHER task, between an
 * unlocked check and an unlocked drop. Every slot's
 * occupancy/priority/age is therefore read and acted on ONCE, entirely
 * inside a single critical section -- never decided beforehand and acted on
 * after. No printfdeb/malloc/Serial call inside the lock (see
 * printf-malloc-starves-nimble): dropped-slot details are collected into
 * local arrays here and printed once, after taskEXIT_CRITICAL(). Ring size
 * (MAX_RING, at most a few dozen) bounds both the loop and the local arrays.
 *
 * TXP W1b: a freed slot (len 0) is immediately reusable by the next enqueue;
 * there is no read pointer to move. INFLIGHT slots are skipped.
 */
static void txRingSweep(uint32_t now_ms, const CetClock &clk, bool cet_only)
{
    int dropSlot[MAX_RING];
    uint32_t dropAgeS[MAX_RING];
    uint32_t dropMsgId[MAX_RING];
    uint8_t dropPrio[MAX_RING];
    bool dropCet[MAX_RING];
    int dropCount = 0;

#if defined(NRF52_SERIES)
    taskENTER_CRITICAL();
#endif

    for(int i = 0; i < MAX_RING; i++)
    {
        if(ringBuffer[i][0] == 0)
            continue;

        // TXP W1b: doTX() holds an INFLIGHT slot; it frees or parks it itself.
        if(ringBuffer[i][1] == RING_STATUS_INFLIGHT)
            continue;

#if defined(EXTERNAL_RADIO)
        // Owned-slot invariant (F7, same as getNextTxSlot()/the overflow
        // evictor above): a slot awaiting an async external-TX bridge result
        // must never be reclaimed here, or a late TX_RESULT would corrupt
        // whatever gets written into the reused slot next.
        if(ringBuffer[i][1] == RING_STATUS_EXT_PENDING)
            continue;
#endif

        uint32_t age_ms = (uint32_t)(now_ms - ringEnqueueTime[i]); // F8: rollover-safe cast

        // TXP W1a: TTL je Prioritaet (txRingTtlMs(); 0 = keine TTL) ...
        bool ttl_hit = false;
        if(!cet_only)
        {
            uint32_t ttl = txRingTtlMs(ringPriority[i], ringBuffer[i][1],
                                       txRingFrameIsAck(ringBuffer[i] + 2, ringBuffer[i][0]));
            ttl_hit = (ttl != 0 && age_ms > ttl);
        }
        // ... und {CET}-Altersgate (D1/D2): zu alte/ungeprueft datierte Beacon weg.
        bool cet_hit = !ttl_hit && cetFrameStale(ringBuffer[i] + 2, ringBuffer[i][0], clk);
        if(!ttl_hit && !cet_hit)
            continue;

        uint32_t mid = ((uint32_t)ringBuffer[i][6] << 24) | ((uint32_t)ringBuffer[i][5] << 16) |
                       ((uint32_t)ringBuffer[i][4] << 8)  |  (uint32_t)ringBuffer[i][3];

        ringBuffer[i][0] = 0; // free the slot
        retryCount[i] = 0;
        if(ringPriority[i] >= 1 && ringPriority[i] <= 5)
            stat_drop_count[ringPriority[i]]++;

        if(dropCount < MAX_RING)
        {
            dropSlot[dropCount] = i;
            dropAgeS[dropCount] = age_ms / 1000UL;
            dropMsgId[dropCount] = mid;
            dropPrio[dropCount] = ringPriority[i];
            dropCet[dropCount] = cet_hit;
            dropCount++;
        }
    }

#if defined(NRF52_SERIES)
    taskEXIT_CRITICAL();
#endif

    // ---- Ab hier ausserhalb des Locks: nur noch Debug-Ausgabe ----
    if(bLORADEBUG)
    {
        for(int k = 0; k < dropCount; k++)
            printfdeb("[MC-DBG] RING_DROP_STALE slot=%d prio=%d age_s=%lu msg_id=%08X reason=%s\n",
                      dropSlot[k], (int)dropPrio[k], (unsigned long)dropAgeS[k], dropMsgId[k],
                      dropCet[k] ? "cet" : "ttl");
    }
}

// TTL je Prioritaet + {CET}-Gate (Hauptschleife, 2-s-Tick).
void txRingAgeBackground(uint32_t now_ms, const CetClock &clk)
{
    txRingSweep(now_ms, clk, false);
}

// Nur das {CET}-Gate, direkt vor dem Senden (doTX()).
void txRingDropStaleCet(const CetClock &clk)
{
    txRingSweep(0, clk, true);
}

// ---- TXP W1c (D3, Konzept 5.4.1): Supersede in place -----------------------
//
// Ein neuer HEY (Prio 5), eine neue Position (Prio 4) oder ein neues {CET}
// ersetzt einen gleichartigen, noch nicht gesendeten Eintrag im selben Slot,
// statt einen zweiten zu belegen. Reine Frame-Helfer, kein Ring-Zugriff.
// frame = ringBuffer[slot]+2 (Typ bei [0], ab [6] Pfad "ORIG[,via..]>DEST").

// Laenge des Urhebers (erstes Pfad-Token: ab Offset 6 bis zum ersten ',' oder
// '>'). 0 = kein Urheber erkennbar (pfadloser Frame) -> kein Supersede.
static uint8_t supersedeOriginatorLen(const uint8_t *frame, int len)
{
    for(int i = 6; i < len && i < 6 + 12; i++)
    {
        if(frame[i] == ',' || frame[i] == '>')
            return (uint8_t)(i - 6);
    }
    return 0;
}

// Gleiches Ziel (Token nach dem ersten '>' bis zum Typ-Terminator == frame[0],
// hoechstens 12 Byte): HEY "H"/"HG" und NBR-Bericht "HN" sind alle '@' mit
// prio 5 vom selben Urheber und duerfen einander nicht ersetzen (W1c-Advisor).
static bool supersedeSameDest(const uint8_t *a, int alen, const uint8_t *b, int blen)
{
    int ia = 6, ib = 6;
    while(ia < alen && a[ia] != '>') ia++;
    while(ib < blen && b[ib] != '>') ib++;
    if(ia >= alen || ib >= blen)
        return false;
    ia++; ib++;
    for(int k = 0; k < 13; k++)
    {
        bool ea = (ia + k >= alen) || a[ia + k] == a[0];
        bool eb = (ib + k >= blen) || b[ib + k] == b[0];
        if(ea || eb)
            return ea && eb && k > 0;
        if(a[ia + k] != b[ib + k])
            return false;
    }
    return false;   // Ziel laenger als 12 Byte: kein Supersede
}

// {CET}-Text: Praefix "{CET}" direkt nach dem ersten ':' ab Offset 6.
static bool supersedeIsCetText(const uint8_t *frame, int len)
{
    if(len < 8 || frame[0] != MSG_TYPE_TEXT)
        return false;
    for(int i = 6; i < len; i++)
    {
        if(frame[i] == ':')
            return (len - i - 1) >= 5 && memcmp(frame + i + 1, "{CET}", 5) == 0;
    }
    return false;
}

// Nur Slots, die weder in der Luft (INFLIGHT) noch extern gehalten (EXT_PENDING)
// noch auf der SENT-Leiter (0x01..0x14, wartet auf ACK) sind.
static inline bool supersedeStatusOk(uint8_t st)
{
    return st == RING_STATUS_READY || st == RING_STATUS_DONE;
}

/**
 * TX-Ring: kompletter Enqueue-Vorgang (Slot-Wahl, Payload-Kopie, Prio/Overflow,
 * Eviction) in EINER Funktion unter EINEM Lock.
 * N-14: bisher schrieb der Aufrufer selbst in den naechsten freien Slot und
 * rief danach diese Funktion auf, die den Schreibindex erneut gelesen hat — zwei
 * gleichzeitige Schreiber (Main-Loop und, auf nRF52, der FreeRTOS-Timer-
 * Service-Task via OnRxDone) konnten sich denselben Slot teilen und ein
 * zusammengewuerfelter Frame ging auf Sendung. Jetzt liefert der Aufrufer nur
 * noch den fertigen Frame; Slot-Wahl und -Schreiben passieren atomar hier.
 *
 * Lock nur auf nRF52 (taskENTER_CRITICAL/EXIT_CRITICAL, siehe
 * phone_commands.cpp:76-92): auf ESP32 laeuft dieser Pfad ausschliesslich im
 * Main-Loop-Kontext (kein zweiter Schreiber, C-01) und portENTER_CRITICAL
 * braucht ohnehin einen portMUX_TYPE, den es hier nicht gibt — deshalb bleibt
 * ESP32 lock-frei. Innerhalb der kritischen Sektion: keine printfdeb/Serial-
 * Aufrufe, kein Yield (printfdeb mallociert oberhalb 64B, siehe
 * printf-malloc-starves-nimble) — alle Debug-Werte werden hier nur
 * eingesammelt, die Ausgabe erfolgt erst nach taskEXIT_CRITICAL().
 *
 * @param frame          Fertig kodierter Frame (ohne Laenge-/Status-Byte)
 * @param len            Frame-Laenge in Byte
 * @param classify_status Status-Byte, MIT DEM getMessagePriority() den Slot
 *                        einstuft (siehe P15-Hinweis unten) -- fuer
 *                        addTxRingEntry() identisch zu store_status, fuer
 *                        addTxRingEntryOnce() READY (kein DONE, sonst stuft
 *                        eine eigene TEXT-Nachricht als Relay ein).
 * @param store_status   Status-Byte, das tatsaechlich im Slot landet und in
 *                        der RING_WRITE-Zeile erscheint.
 * @param source         Kurzes Label fuer Debug-Ausgabe (z.B. "rx_relay")
 * @param retryCountIn   retryCount[Slot] setzen; -1 (Default) = unangetastet
 *                        lassen (manche Aufrufstellen haben retryCount nie
 *                        zurueckgesetzt — Alt-Verhalten bewusst beibehalten)
 * @param clearSlotFirst true = Slot vor dem Schreiben komplett nullen (nur
 *                        der rx_relay-Aufruf tat das bisher selbst)
 * @param kind           Nachbarschaftsmatrix Stufe 2 (Konzept 5.1): RING_KIND_*
 *                        (txring_functions.h), Default RING_KIND_OTHER. Nur
 *                        rx_relay und der udp_rx-Text (GWRX-01) setzen RING_KIND_RELAY.
 * @param need           Stufe 2 (Welle 2: NbrMask*, nbr_mask.h): Bedarfsmaske
 *                        aus nbrRelayNeed()/nbrServerNeed(), Default nullptr = leere Maske.
 *                        Wird JEDEM Slot zugewiesen, auch wenn leer -- so
 *                        behaelt ein wiederverwendeter Slot nie die Maske
 *                        seines Vorbesitzers.
 * @param alone          Stufe 2 (Welle 2: NbrMask*): Allein-Maske aus
 *                        nbrRelayNeed()/nbrServerNeed(), Default nullptr = leere Maske.
 * @return Slot-Index (>=0) oder -1, wenn die Overflow-Logik den neuen
 *         Eintrag verworfen hat (Ring voll, keine niedrigere Prio zum
 *         Verdraengen vorhanden)
 *
 * P15: gemeinsamer Kern fuer addTxRingEntry() und addTxRingEntryOnce()
 * (beide unten, duenne Wrapper). Vorher schrieb SendAckMessage() mit Status
 * READY (0x00) ein und setzte den Slot NACH addTxRingEntry() per Hand auf
 * DONE (0xFF) -- ausserhalb des Locks. Auf nRF52 laeuft OnRxDone (und mit
 * ihm SendPong()/SendAckMessage()) im LORA-Task, doTX() im Loop-Task; ein
 * Schreiben zwischen den beiden Anweisungen konnte von doTX()s save/restore
 * des gelesenen Status rueckgaengig gemacht werden. classify_status/
 * store_status trennen jetzt "womit klassifiziert" von "was gespeichert
 * wird" -- beides passiert hier, innerhalb DERSELBEN kritischen Sektion,
 * kein Nachtrag mehr noetig.
 */
static int addTxRingEntryCore(const uint8_t* frame, uint16_t len,
                               uint8_t classify_status, uint8_t store_status,
                               const char* source, int retryCountIn, bool clearSlotFirst,
                               uint8_t kind, const NbrMask *need, const NbrMask *alone)
{
    // TX-01 (BACKLOG 3.8k): an unconfigured node (factory callsign) must not
    // transmit at all -- refuse here so its ring never even fills, on top
    // of the hard backstop in doTX() (lora_functions.cpp, the only caller
    // of Radio.Send()/startTransmit()). Checked first, ahead of the length
    // clamp below: this frame is not going anywhere either way.
    if(isUnconfiguredCall(meshcom_settings.node_call))
    {
        logTxRefuseUnconfigured();
        return -1;
    }

    // Verdict Finding 3: defensive Invariante. Alle heutigen Aufrufer sind auf
    // <=255 begrenzt -- encodeAPRS() klemmt intern auf UDP_TX_BUF_SIZE, der
    // Radio-Empfang liefert hoechstens die Modul-Obergrenze von 255 Byte, und
    // der Retransmit-Pfad liest die Laenge aus einem uint8 -- aber diese
    // Grenze lebt bislang ausschliesslich in den Aufrufern und wird am
    // einzigen Engpass (hier) nicht erzwungen. Ein kuenftiger Aufrufer mit
    // len > UDP_TX_BUF_SIZE wuerde per memcpy in den Nachbarslot schreiben,
    // und die Kuerzung auf (uint8_t)len beim Ablegen in ringBuffer[w][0]
    // koennte den Eintrag zusaetzlich als kurz/leer tarnen. len==0 waere ein
    // funktionsloser Ring-Eintrag. Beides wird hier abgewiesen, bevor der Ring
    // ueberhaupt beruehrt wird.
    if(len == 0 || len > UDP_TX_BUF_SIZE)
    {
        // Advisory-Punkt aus dem Testsuite-Audit: nicht stumm verwerfen --
        // wir stehen hier VOR dem Lock, printfdeb ist erlaubt. Faellt diese
        // Zeile je im Feld, hat ein neuer Aufrufer die Invariante verletzt.
        if(bLORADEBUG)
            printfdeb("[MC-DBG] RING_REJECT len=%u src=%s (0 oder > %d)\n",
                      (unsigned)len, source, UDP_TX_BUF_SIZE);
        return -1;
    }

    int w = -1, queued = 0;
    uint8_t msgType, prio;
    uint32_t mid;
    bool droppedNew = false, droppedOld = false;
    bool parkedOverwrite = false;
    int dropSlot = -1;
    uint8_t dropPrio = 0, dropType = 0;
    uint32_t dropId = 0, newLostId = 0;

    // TXP W1b: classify the NEW frame up front, from the frame itself and
    // outside the lock (CheckGroup() builds a String): a pool without a free
    // slot must know the priority before it decides to evict or drop.
    // ringBuffer[slot][2..] = frame, so frame[k] = ringBuffer[slot][k+2].
    prio = classifyFrame(frame, len, classify_status);
    msgType = frame[0];
    mid = ((uint32_t)frame[4] << 24) | ((uint32_t)frame[3] << 16) |
          ((uint32_t)frame[2] << 8)  |  (uint32_t)frame[1];

#if defined(NRF52_SERIES)
    taskENTER_CRITICAL();
#endif

    // Free slot = len 0. Take the first one; count the occupied rest (BP-02:
    // occupied slots, every status -- the depth BEFORE this entry, same
    // counting as txRingDepth()). Read-only scan under the lock, no printf.
    for(int i = 0; i < MAX_RING; i++)
    {
        if(ringBuffer[i][0] == 0)
        {
            if(w < 0)
                w = i;
        }
        else
            queued++;
    }
    int depthAfter = queued + 1;

    // TXP W1c (D3, Konzept 5.4.1): Supersede VOR der Freislot-Wahl. Ein HEY/
    // eine Position desselben Urhebers bzw. ein {CET} ersetzt einen noch nicht
    // gesendeten Eintrag in place (kein Eviction, kein Drop-Zaehler), auch bei
    // vollem Pool. Akzeptierter Kompromiss (Konzept 10.6): ein ersetztes
    // weitergeleitetes HEY verliert seine Signalberichts-Kette.
    int supSlot = -1;
    bool supKeepNew = false;
    uint32_t supOldId = 0;
    if(msgType == MSG_TYPE_HEY || msgType == MSG_TYPE_POSITION)
    {
        uint8_t ol = supersedeOriginatorLen(frame, len);
        if(ol > 0)
        {
            for(int i = 0; i < MAX_RING; i++)
            {
                if(ringBuffer[i][0] == 0 || ringBuffer[i][2] != msgType ||
                   ringPriority[i] != prio || !supersedeStatusOk(ringBuffer[i][1]))
                    continue;
                const uint8_t *qf = ringBuffer[i] + 2;
                if(supersedeOriginatorLen(qf, ringBuffer[i][0]) == ol &&
                   memcmp(qf + 6, frame + 6, ol) == 0 &&
                   supersedeSameDest(qf, ringBuffer[i][0], frame, len))
                {
                    supSlot = i;
                    break;
                }
            }
        }
    }
    // Nur Server-/Relay-Beacons (als Relay DONE eingestuft, prio 3), nie ein
    // eigener Text, der zufaellig mit "{CET}" beginnt (W1c-Advisor).
    else if(classify_status == RING_STATUS_DONE && prio == MSG_PRIO_NORMAL &&
            supersedeIsCetText(frame, len))
    {
        uint32_t newT = 0;
        bool newOk = cetFindInFrame(frame, len, &newT);
        for(int i = 0; i < MAX_RING; i++)
        {
            if(ringBuffer[i][0] == 0 || ringBuffer[i][1] != RING_STATUS_DONE ||
               ringPriority[i] != MSG_PRIO_NORMAL ||
               !supersedeIsCetText(ringBuffer[i] + 2, ringBuffer[i][0]))
                continue;
            uint32_t oldT = 0;
            bool oldOk = cetFindInFrame(ringBuffer[i] + 2, ringBuffer[i][0], &oldT);
            // unparsbar = aelter: ein unparsbarer Neuer gewinnt nur gegen einen
            // ebenfalls unparsbaren; ein parsbarer Neuer gewinnt bei >= (Gleichstand).
            bool newer = newOk ? (!oldOk || (int32_t)(newT - oldT) >= 0) : !oldOk;
            supOldId = ((uint32_t)ringBuffer[i][6] << 24) | ((uint32_t)ringBuffer[i][5] << 16) |
                       ((uint32_t)ringBuffer[i][4] << 8)  |  (uint32_t)ringBuffer[i][3];
            if(newer)
                supSlot = i;
            else
                supKeepNew = true;
            break;
        }
    }
    if(supSlot >= 0 && !supKeepNew)
    {
        supOldId = ((uint32_t)ringBuffer[supSlot][6] << 24) | ((uint32_t)ringBuffer[supSlot][5] << 16) |
                   ((uint32_t)ringBuffer[supSlot][4] << 8)  |  (uint32_t)ringBuffer[supSlot][3];
        w = supSlot;
        depthAfter = queued;   // alter Eintrag raus, neuer rein
        stat_supersede_count++;
    }

    if(supKeepNew)
    {
        // das Queued-{CET} ist neuer: den Neuen verwerfen (kein Drop-Zaehler)
        w = -1;
        depthAfter = queued;
    }
    else if(w < 0)
    {
        // Pool full. Priority-aware eviction: a strictly higher-priority
        // newcomer evicts the OLDEST entry (ringEnqueueTime, wrap-safe) of the
        // WORST priority; otherwise the newcomer is dropped (RING_DROP_NEW).
        // Never evictable: INFLIGHT (doTX() holds the frame, it frees or
        // parks the slot itself) and EXT_PENDING (an external-TX ownership
        // record points at it, its length must stay valid until the bridge
        // result). A parked SENT slot (own text waiting for an ACK) IS
        // evictable, exactly as in the old ring (there only EXT_PENDING
        // was protected, and only in EXTERNAL_RADIO builds): own texts are
        // prio 1/2 and so only fall to a strictly higher priority.
        int worst_slot = -1;
        uint8_t worst_prio = 0;
        for(int i = 0; i < MAX_RING; i++)
        {
            uint8_t st = ringBuffer[i][1];
            if(st == RING_STATUS_EXT_PENDING || st == RING_STATUS_INFLIGHT)
                continue;
            if(ringPriority[i] > worst_prio ||
               (ringPriority[i] == worst_prio && worst_slot >= 0 &&
                (int32_t)(ringEnqueueTime[i] - ringEnqueueTime[worst_slot]) < 0))
            {
                worst_prio = ringPriority[i];
                worst_slot = i;
            }
        }

        if(worst_slot >= 0 && prio < worst_prio)
        {
            // Drop the lowest-priority entry to make room; the newcomer
            // takes its slot (nothing else moves, slot ids stay stable).
            droppedOld = true;
            dropSlot = worst_slot;
            dropPrio = worst_prio;
            dropType = ringBuffer[worst_slot][2];
            dropId = ((uint32_t)ringBuffer[worst_slot][6] << 24) | ((uint32_t)ringBuffer[worst_slot][5] << 16) |
                     ((uint32_t)ringBuffer[worst_slot][4] << 8)  |  (uint32_t)ringBuffer[worst_slot][3];
            if(worst_prio >= 1 && worst_prio <= 5)
                stat_drop_count[worst_prio]++;
            // M0-1 instrumentation (0.4): the evicted entry was a parked SENT
            // (own text waiting for its ACK), not a queued READY/DONE frame.
            parkedOverwrite = ringBuffer[worst_slot][1] != RING_STATUS_READY &&
                              ringBuffer[worst_slot][1] != RING_STATUS_DONE;
            w = worst_slot;
            depthAfter = queued;   // evicted one out, new one in
        }
        else
        {
            // New packet is same or lower priority than everything evictable -- drop it
            droppedNew = true;
            newLostId = mid;
            if(prio >= 1 && prio <= 5)
                stat_drop_count[prio]++;
            depthAfter = queued;
        }
    }

    // Track queue depth for high-water mark (a dropped newcomer means a full pool)
    if(depthAfter > stat_queue_hwm)
        stat_queue_hwm = depthAfter;

    int resultSlot = -1;
    if(supKeepNew)
        resultSlot = -1;
    else if(droppedNew)
        resultSlot = RING_ENQUEUE_FULL;   // QRT Finding 5: no-evict overflow, distinct from -1 (invalid)
    else
    {
        if(clearSlotFirst)
            // sizeof statt UDP_TX_BUF_SIZE+1: der Slot ist UDP_TX_BUF_SIZE+5 Byte
            // gross -- das alte Limit liess die letzten 4 Byte ungeputzt
            // (TXRING-CLEARSLOT-GAP, von test_txring gefunden).
            memset(ringBuffer[w], 0x00, sizeof(ringBuffer[0]));

        memcpy(ringBuffer[w]+2, frame, len);
        ringBuffer[w][1] = store_status;

        if(retryCountIn >= 0)
            retryCount[w] = (uint8_t)retryCountIn;

        // Assign priority and enqueue timestamp
        ringPriority[w] = prio;
        ringEnqueueTime[w] = millis();
        // SL-03/SL-06: Herkunft aus dem `source`-Label festhalten, solange es im
        // Scope ist -- die TX-Zeile in doTX() sieht spaeter nur noch den Slot.
        ringSource[w] = setlogRingSourceCode(source);
        // Nachbarschaftsmatrix Stufe 2 (Konzept 5.1): JEDER Enqueue setzt alle
        // drei Seitenfelder, auch auf ihre Defaults -- sonst wuerde ein Slot, der
        // vorher ein Relay mit alter Bedarfsmaske trug, diese Maske fuer den
        // neuen Eintrag stillschweigend behalten (Regression: der Mithoer-Scan
        // in lora_functions.cpp bricht dann gegen die FALSCHE Nachricht ab).
        ringKind[w]  = kind;
        ringNeed[w]  = need  ? *need  : nbrMaskNone();
        ringAlone[w] = alone ? *alone : nbrMaskNone();
        // Length last: the slot only counts as occupied once it is complete.
        ringBuffer[w][0] = (uint8_t)len;
        resultSlot = w;
    }

#if defined(NRF52_SERIES)
    taskEXIT_CRITICAL();
#endif

    // ---- Ab hier ausserhalb des Locks ----

    if(supKeepNew)
    {
        if(bLORADEBUG)
            printfdeb("[MC-DBG] RING_SUPERSEDE_KEEP prio=%d old_id=%08X new_id=%08X\n",
                      prio, supOldId, mid);
        return -1;
    }

    // M0-1 (0.4): ring enqueues per window and parked-slot overwrites.
    ringstat_enqueue.fetch_add(1);
    if(parkedOverwrite)
        ringstat_parked_overwrite.fetch_add(1);

    // SL-05: Hochwasser des Ringfuellstands im 5-Minuten-Fenster. Bewusst
    // ausserhalb des Locks und ueber txRingDepth() (selbst lock-frei, siehe
    // dessen Doku): der Slot ist hier bereits geschrieben, die Tiefe
    // enthaelt also den neuen Eintrag. Auch bei resultSlot < 0 gemessen -- ein
    // verworfener Eintrag heisst voller Ring, und genau der Wert ist gesucht.
    // Kein Ersatz fuer stat_queue_hwm -- das zaehlt seit dem Boot und wird nie
    // zurueckgesetzt, stat_ring_max wird nach jedem STAT-Druck genullt.
    // CAS-Schleife statt load/store, damit ein gleichzeitiger Schreiber aus
    // dem anderen Task kein Maximum verliert.
    {
        uint8_t depth_now = (uint8_t)txRingDepth();
        uint8_t seen = stat_ring_max.load();
        while(depth_now > seen && !stat_ring_max.compare_exchange_weak(seen, depth_now))
            ; // seen wird von compare_exchange_weak aktualisiert
    }

    // ---- nur noch Debug-Ausgabe ----
    if(bLORADEBUG)
    {
        printfdeb("[MC-DBG] RING_WRITE slot=%d type=%02X status=%02X "
                  "len=%d msg_id=%08X queued=%d/%d src=%s\n",
                  w, msgType, store_status, (int)len, mid, queued, MAX_RING, source);
        printfdeb("[MC-DBG] RING_PRIO slot=%d prio=%d\n", w, prio);

        if(supSlot >= 0)
            printfdeb("[MC-DBG] RING_SUPERSEDE slot=%d prio=%d old_id=%08X new_id=%08X\n",
                      supSlot, prio, supOldId, mid);

        if(droppedOld)
            printfdeb("[MC-DBG] RING_DROP_PRIO slot=%d prio=%d type=%02X "
                      "msg_id=%08X replaced_by_prio=%d src=%s\n",
                      dropSlot, dropPrio, dropType, dropId, prio, source);
        if(droppedNew)
            printfdeb("[MC-DBG] RING_DROP_NEW slot=%d prio=%d type=%02X "
                      "msg_id=%08X (queue full, no lower prio to evict)\n",
                      w, prio, msgType, newLostId);
    }

    return resultSlot;
}

int addTxRingEntry(const uint8_t* frame, uint16_t len, uint8_t ring_status,
                    const char* source, int retryCountIn, bool clearSlotFirst,
                    uint8_t kind, const NbrMask *need, const NbrMask *alone)
{
    return addTxRingEntryCore(frame, len, ring_status, ring_status,
                               source, retryCountIn, clearSlotFirst,
                               kind, need, alone);
}

// GWRX-01: siehe txring_functions.h. Status 0xFF, Label "udp_rx", retryCount 0
// und clearSlotFirst false wie die beiden bisherigen Direktaufrufe.
int addUdpRxRingEntry(const uint8_t* frame, uint16_t len, char payload_type,
                      bool nbrKnown, const NbrMask *need, const NbrMask *alone)
{
    if(bNBRRELAY && nbrKnown && payload_type == ':')
        return addTxRingEntry(frame, len, RING_STATUS_DONE, "udp_rx", 0, false,
                               RING_KIND_RELAY, need, alone);
    return addTxRingEntry(frame, len, RING_STATUS_DONE, "udp_rx", 0);
}

/**
 * P15: eigene Nachricht (DM/Gruppe/Broadcast), die nie wiederholt werden
 * soll -- SendAckMessage()/sendPing()/SendPong() und
 * die Einmal-Texte von sendMessage() ({ping}, {CET}/{SET}/{MCP}, RM1;
 * dmTextSendOnce(), seit der RCA 2026-10-10) wollten bisher alle dasselbe: mit Status
 * DONE (0xFF, "keine Wiederholung") einreihen, aber trotzdem als eigene
 * DM/Gruppen-/Broadcast-Nachricht eingestuft werden, nicht als Relay (siehe
 * getMessagePriority(): eine TEXT-Nachricht mit Status DONE gilt dort als
 * Relay -- RING_STATUS_DONE wird sonst nur von OnRxDone fuer weitergeleitete
 * Pakete gesetzt). addTxRingEntryCore() klassifiziert deshalb mit READY und
 * legt DONE erst danach fest, beides unter demselben Lock (siehe Doku dort).
 *
 * Gleiche Parameter/Rueckgabe/Fehlverhalten wie addTxRingEntry(), nur ohne
 * ring_status -- der ist hier immer implizit DONE. kind/need/alone
 * (Nachbarschaftsmatrix Stufe 2) bleiben auf ihren Defaults: keiner der
 * heutigen Aufrufer ist ein Relay-Slot.
 */
int addTxRingEntryOnce(const uint8_t* frame, uint16_t len, const char* source,
                        int retryCountIn, bool clearSlotFirst,
                        uint8_t kind, const NbrMask *need, const NbrMask *alone)
{
    return addTxRingEntryCore(frame, len, RING_STATUS_READY, RING_STATUS_DONE,
                               source, retryCountIn, clearSlotFirst,
                               kind, need, alone);
}

// TXP W1c: Retry in place (Doku: txring_functions.h). Reihenfolge im Slot:
// erst Nutzlast, Status zuletzt -- der Slot steht bis dahin auf der SENT-Leiter
// und ist fuer getNextTxSlot() unsichtbar.
bool txRingRearmRetry(int slot, const uint8_t *frame, uint16_t len, uint8_t expectStatus)
{
    if(slot < 0 || slot >= MAX_RING || frame == nullptr || len == 0 || len > UDP_TX_BUF_SIZE)
        return false;

    bool ok = false;
    int retry = 0;
    uint32_t mid = 0;

#if defined(NRF52_SERIES)
    taskENTER_CRITICAL();
#endif
    if(ringBuffer[slot][0] == len && ringBuffer[slot][1] == expectStatus)
    {
        if(frame != ringBuffer[slot] + 2)
            memmove(ringBuffer[slot] + 2, frame, len);
        retryCount[slot]++;
        retry = retryCount[slot];
        ringEnqueueTime[slot] = millis();
        ringBuffer[slot][1] = RING_STATUS_READY;
        mid = ((uint32_t)ringBuffer[slot][6] << 24) | ((uint32_t)ringBuffer[slot][5] << 16) |
              ((uint32_t)ringBuffer[slot][4] << 8)  |  (uint32_t)ringBuffer[slot][3];
        ok = true;
    }
#if defined(NRF52_SERIES)
    taskEXIT_CRITICAL();
#endif

    if(ok)
    {
        ringstat_enqueue.fetch_add(1);
        if(bLORADEBUG)
            printfdeb("[MC-DBG] RING_REARM slot=%d msg_id=%08X retry=%d\n", slot, mid, retry);
    }
    return ok;
}
