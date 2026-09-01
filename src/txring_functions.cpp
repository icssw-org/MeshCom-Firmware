#include "txring_functions.h"

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
ring_index_t iWrite;
ring_index_t iRead;
uint8_t retryCount[MAX_RING];
uint8_t ringPriority[MAX_RING];
uint32_t ringEnqueueTime[MAX_RING];
uint16_t stat_drop_count[6];
uint8_t stat_queue_hwm;
#endif

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
 * Determine message priority from ring buffer slot content.
 * Ring buffer layout: [0]=len, [1]=status, [2]=payload_type, [3-6]=msg_id, [7]=flags, [8+]=path
 * Encoded path format: "SOURCE>DEST:" starting at offset 8 (relative to ringBuffer[slot]+2)
 * i.e. ringBuffer[slot][8] is the first char of the source callsign.
 */
uint8_t getMessagePriority(int slot)
{
    uint8_t msg_type = ringBuffer[slot][2];

    if(msg_type == MSG_TYPE_ACK)
        return MSG_PRIO_CRITICAL;

    if(msg_type == MSG_TYPE_POSITION)
        return MSG_PRIO_LOW;

    if(msg_type == MSG_TYPE_HEY)
        return MSG_PRIO_BACKGROUND;

    if(msg_type == MSG_TYPE_TEXT)
    {
        // Relay detection: OnRxDone sets RING_STATUS_DONE for relayed packets
        if(ringBuffer[slot][1] == RING_STATUS_DONE)
            return MSG_PRIO_NORMAL; // Relay

        // User-originated text: parse destination from encoded APRS path
        // Path starts at ringBuffer[slot][8] (offset 6 in encoded data = after type+id+flags)
        // Format: "SOURCE>DEST:" — find '>' then read until payload_type char
        int len = ringBuffer[slot][0];
        int path_start = 8; // slot[2] + 6 bytes header = slot[8]
        int dest_start = -1;
        int dest_end = -1;

        for(int i = path_start; i < len + 2 && i < (UDP_TX_BUF_SIZE + 4); i++)
        {
            if(ringBuffer[slot][i] == '>' && dest_start < 0)
            {
                dest_start = i + 1;
            }
            else if(dest_start >= 0 && (ringBuffer[slot][i] == ':' || ringBuffer[slot][i] == MSG_TYPE_TEXT))
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
            memcpy(dest, &ringBuffer[slot][dest_start], dlen);
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
 * Find the next slot to transmit from the ring buffer, based on priority.
 * Scans all occupied slots between iRead and iWrite.
 * Returns slot index, or -1 if empty.
 * Within same priority, oldest entry (closest to iRead) wins (FIFO).
 */
int getNextTxSlot(void)
{
    if(iWrite == iRead)
        return -1;

    int best_slot = -1;
    uint8_t best_prio = 255;

    int pos = iRead;
    while(pos != iWrite)
    {
#if defined(EXTERNAL_RADIO)
        // owned-slot status invariant: a slot owned by an in-flight external TX must never be
        // reselected. RING_STATUS_EXT_PENDING is neither READY nor DONE, so the
        // test below already skips it; this explicit skip states the intent.
        if(ringBuffer[pos][1] == RING_STATUS_EXT_PENDING)
        {
            pos++;
            if(pos >= MAX_RING)
                pos = 0;
            continue;
        }
#endif
        // Only consider slots with data that are ready to send (READY or DONE/fire-and-forget).
        // Skip slots with status SENT..threshold (awaiting retransmission timer).
        if(ringBuffer[pos][0] > 0 &&
           (ringBuffer[pos][1] == RING_STATUS_READY || ringBuffer[pos][1] == RING_STATUS_DONE))
        {
            uint8_t prio = ringPriority[pos];
            if(prio < best_prio)
            {
                best_prio = prio;
                best_slot = pos;
            }
            // Same prio: keep first found (= oldest = FIFO)
        }

        pos++;
        if(pos >= MAX_RING)
            pos = 0;
    }

    return best_slot;
}

/**
 * Advance iRead past any empty (already-transmitted) slots at the front.
 *
 * Abweichung vom reinen Verschieben: im Original war diese Funktion `static`
 * (TU-lokal). doTX() in lora_functions.cpp ruft sie jedoch ebenfalls auf
 * (dortiger Aufruf blieb beim Extrahieren an Ort und Stelle stehen) -- eine
 * TU-lokale Funktion kann von dort nicht mehr erreicht werden. Daher hier
 * externe Sichtbarkeit (Deklaration in txring_functions.h), Koerper sonst
 * unveraendert.
 */
void advanceIReadPastEmpty(void)
{
    int localRead = iRead;
    int localWrite = iWrite;
    while(localRead != localWrite && ringBuffer[localRead][0] == 0)
    {
        localRead++;
        if(localRead >= MAX_RING)
            localRead = 0;
    }
    iRead = localRead;
}

/**
 * BP-01 (BACKLOG) / TM-37 / BP-02 (DJ8MEH-RCA): current TX-ring fill level.
 *
 * Counts OCCUPIED slots (ringBuffer[i][0] > 0) from iRead up to (but not
 * including) iWrite, wrapping over MAX_RING -- not the index distance
 * iWrite-iRead. A priority-starved entry parked at iRead leaves behind
 * freed holes (retransmitted-and-dropped, or evicted) between iRead and
 * iWrite that the old index-distance arithmetic counted as still queued.
 * DJ8MEH-RCA 2026-08-31: an 8-minute QRT episode ran on a phantom depth of
 * 19 while only 3-4 slots were actually occupied, because QRV only closes
 * at depth 0.
 *
 * Same occupied-slot counting style as the `queued` local inside
 * addTxRingEntry() (that one excludes the slot being written, since it is
 * computed before iWrite advances past it).
 *
 * Deliberately lock-free: both indices are read once each into locals. The
 * occupied-count loop over up to MAX_RING slots is a read-only scan (no
 * writes, no printf/malloc), so it stays cheap enough to keep lock-free.
 * The worst a concurrent OnRxDone enqueue/evict on nRF52 can do mid-scan is
 * make the answer stale by roughly the couple of slots that write touches
 * (occupying or freeing a slot the scan has already passed or not yet
 * reached) -- real, but irrelevant for a back-pressure notice threshold
 * that only needs "about empty" vs. "still full", not an exact count.
 */
int txRingDepth(void)
{
    int w = (int)(uint8_t)iWrite;
    int r = (int)(uint8_t)iRead;
    int depth = 0;
    int i = r;
    while(i != w)
    {
        if(ringBuffer[i][0] > 0)
            depth++;
        i++;
        if(i >= MAX_RING)
            i = 0;
    }
    return depth;
}

/**
 * BP-03 (DJ8MEH-RCA 2026-08-31, Teil 2): sweep the whole ring and drop every
 * BACKGROUND (HEY, prio 5) entry older than RING_BG_MAX_AGE_MS
 * (configuration_global.h). A priority-starved HEY parked at iRead behind a
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
 * pattern as addTxRingEntry() above, and for the same reason (Advisor F2):
 * the overflow-eviction path in addTxRingEntry() (see the N-24 comment
 * further down) can RELOCATE an iRead entry's payload/side-arrays into a
 * slot this sweep has already looked at or is about to look at, from the
 * OTHER task, between an unlocked check and an unlocked drop. Every slot's
 * occupancy/priority/age is therefore read and acted on ONCE, entirely
 * inside a single critical section -- never decided beforehand and acted on
 * after. No printfdeb/malloc/Serial call inside the lock (see
 * printf-malloc-starves-nimble): dropped-slot details are collected into
 * local arrays here and printed once, after taskEXIT_CRITICAL(). Ring size
 * (MAX_RING, at most a few dozen) bounds both the loop and the local arrays.
 *
 * advanceIReadPastEmpty() runs once, inside the same critical section (same
 * placement as its call inside addTxRingEntry() above), so the read pointer
 * clears any run of newly-freed slots at the front in the same atomic step.
 */
void txRingAgeBackground(uint32_t now_ms)
{
    int dropSlot[MAX_RING];
    uint32_t dropAgeS[MAX_RING];
    uint32_t dropMsgId[MAX_RING];
    int dropCount = 0;

#if defined(NRF52_SERIES)
    taskENTER_CRITICAL();
#endif

    for(int i = 0; i < MAX_RING; i++)
    {
        if(ringBuffer[i][0] == 0)
            continue;
        if(ringPriority[i] != MSG_PRIO_BACKGROUND)
            continue;

        uint32_t age_ms = (uint32_t)(now_ms - ringEnqueueTime[i]); // F8: rollover-safe cast
        if(age_ms <= RING_BG_MAX_AGE_MS)
            continue;

#if defined(EXTERNAL_RADIO)
        // Owned-slot invariant (F7, same as getNextTxSlot()/the overflow
        // evictor above): a slot awaiting an async external-TX bridge result
        // must never be reclaimed here, or a late TX_RESULT would corrupt
        // whatever gets written into the reused slot next.
        if(ringBuffer[i][1] == RING_STATUS_EXT_PENDING)
            continue;
#endif

        uint32_t mid = ((uint32_t)ringBuffer[i][6] << 24) | ((uint32_t)ringBuffer[i][5] << 16) |
                       ((uint32_t)ringBuffer[i][4] << 8)  |  (uint32_t)ringBuffer[i][3];

        ringBuffer[i][0] = 0; // free the slot
        retryCount[i] = 0;
        stat_drop_count[MSG_PRIO_BACKGROUND]++;

        if(dropCount < MAX_RING)
        {
            dropSlot[dropCount] = i;
            dropAgeS[dropCount] = age_ms / 1000UL;
            dropMsgId[dropCount] = mid;
            dropCount++;
        }
    }

    advanceIReadPastEmpty();

#if defined(NRF52_SERIES)
    taskEXIT_CRITICAL();
#endif

    // ---- Ab hier ausserhalb des Locks: nur noch Debug-Ausgabe ----
    if(bLORADEBUG)
    {
        for(int k = 0; k < dropCount; k++)
            printfdeb("[MC-DBG] RING_DROP_STALE slot=%d age_s=%lu msg_id=%08X\n",
                      dropSlot[k], (unsigned long)dropAgeS[k], dropMsgId[k]);
    }
}

/**
 * TX-Ring: kompletter Enqueue-Vorgang (Slot-Wahl, Payload-Kopie, Prio/Overflow,
 * iWrite/iRead-Fortschritt) in EINER Funktion unter EINEM Lock.
 * N-14: bisher schrieb der Aufrufer selbst nach ringBuffer[iWrite][...] und
 * rief danach diese Funktion auf, die iWrite erneut gelesen hat — zwei
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
 * @param ring_status    Status-Byte fuer den Slot (RING_STATUS_*)
 * @param source         Kurzes Label fuer Debug-Ausgabe (z.B. "rx_relay")
 * @param retryCountIn   retryCount[Slot] setzen; -1 (Default) = unangetastet
 *                        lassen (manche Aufrufstellen haben retryCount nie
 *                        zurueckgesetzt — Alt-Verhalten bewusst beibehalten)
 * @param clearSlotFirst true = Slot vor dem Schreiben komplett nullen (nur
 *                        der rx_relay-Aufruf tat das bisher selbst)
 * @return Slot-Index (>=0) oder -1, wenn die Overflow-Logik den neuen
 *         Eintrag verworfen hat (Ring voll, keine niedrigere Prio zum
 *         Verdraengen vorhanden)
 */
int addTxRingEntry(const uint8_t* frame, uint16_t len, uint8_t ring_status,
                    const char* source, int retryCountIn, bool clearSlotFirst)
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

    int w, r, queued;
    uint8_t msgType, prio;
    uint32_t mid;
    bool droppedNew = false, droppedOld = false, ringOverflowAdvance = false;
    int dropSlot = -1;
    uint8_t dropPrio = 0, dropType = 0;
    uint32_t dropId = 0, newLostId = 0;

#if defined(NRF52_SERIES)
    taskENTER_CRITICAL();
#endif

    w = iWrite;
    r = iRead;

    if(clearSlotFirst)
        // sizeof statt UDP_TX_BUF_SIZE+1: der Slot ist UDP_TX_BUF_SIZE+5 Byte
        // gross -- das alte Limit liess die letzten 4 Byte ungeputzt
        // (TXRING-CLEARSLOT-GAP, von test_txring gefunden).
        memset(ringBuffer[w], 0x00, sizeof(ringBuffer[0]));

    ringBuffer[w][0] = (uint8_t)len;
    ringBuffer[w][1] = ring_status;
    memcpy(ringBuffer[w]+2, frame, len);

    if(retryCountIn >= 0)
        retryCount[w] = (uint8_t)retryCountIn;

    msgType = ringBuffer[w][2];
    mid = ((uint32_t)ringBuffer[w][6] << 24) | ((uint32_t)ringBuffer[w][5] << 16) |
          ((uint32_t)ringBuffer[w][4] << 8)  |  (uint32_t)ringBuffer[w][3];
    // BP-02: occupied-slot count from iRead up to (excluding) the slot just
    // written at w -- i.e. the depth BEFORE this entry, same counting style
    // as txRingDepth() above. Not index distance: holes freed by retransmit
    // drops/evictions between iRead and w must not inflate this number (see
    // txRingDepth() doc comment, DJ8MEH-RCA). Read-only loop under the lock,
    // no printf/malloc.
    {
        int occ = 0, i = r;
        while(i != w)
        {
            if(ringBuffer[i][0] > 0)
                occ++;
            i++;
            if(i >= MAX_RING)
                i = 0;
        }
        queued = occ;
    }

    // Assign priority and enqueue timestamp
    ringPriority[w] = getMessagePriority(w);
    ringEnqueueTime[w] = millis();
    prio = ringPriority[w];

    // Track queue depth for high-water mark
    if(queued + 1 > stat_queue_hwm)
        stat_queue_hwm = queued + 1;

    // Priority-aware overflow: when queue is full, drop lowest-priority oldest entry
    int nextWrite = w + 1;
    if(nextWrite >= MAX_RING) nextWrite = 0;
    if(nextWrite == r && ringBuffer[r][0] > 0)
    {
        // Find the oldest entry of the lowest priority in the queue
        int worst_slot = -1;
        uint8_t worst_prio = 0;
        int scan = r;
        while(scan != w)
        {
            bool evictable = ringBuffer[scan][0] > 0;
#if defined(EXTERNAL_RADIO)
            // Never evict an in-flight external-TX slot: its length must stay
            // valid until the bridge result resolves it, and the ownership record
            // is not notified by this overflow path. Dropping it here would lose a
            // pending TX or let a late result corrupt the reused slot.
            if(ringBuffer[scan][1] == RING_STATUS_EXT_PENDING)
                evictable = false;
#endif
            if(evictable && ringPriority[scan] > worst_prio)
            {
                worst_prio = ringPriority[scan];
                worst_slot = scan;
            }
            scan++;
            if(scan >= MAX_RING) scan = 0;
        }

        if(worst_slot >= 0 && prio < worst_prio)
        {
            // Drop the lowest-priority entry to make room
            droppedOld = true;
            dropSlot = worst_slot;
            dropPrio = worst_prio;
            dropType = ringBuffer[worst_slot][2];
            dropId = ((uint32_t)ringBuffer[worst_slot][6] << 24) | ((uint32_t)ringBuffer[worst_slot][5] << 16) |
                     ((uint32_t)ringBuffer[worst_slot][4] << 8)  |  (uint32_t)ringBuffer[worst_slot][3];
            stat_drop_count[worst_prio]++;
            ringBuffer[worst_slot][0] = 0; // Mark as empty

            // N-24 (TXRING-ORPHAN-SLOT): liegt der geraeumte Slot NICHT am
            // Lesezeiger, bleibt der Slot an iRead belegt -- der unbedingte
            // Ueberlauf-Fortschalter unten schoebe iRead trotzdem darueber
            // hinweg, und der Eintrag (moeglicherweise CRITICAL, nie
            // gesendet) waere dauerhaft unsichtbar und unbilanziert.
            // Fix: den Eintrag von iRead in den soeben freigewordenen Slot
            // umziehen (Payload + Seitenarrays), iRead-Slot leeren -- danach
            // raeumt advanceIReadPastEmpty() den Lesezeiger regulaer weiter
            // und der Fortschalter unten trifft keinen belegten Slot mehr.
            // Preis: der umgezogene Eintrag verliert seinen FIFO-Rang
            // innerhalb gleicher Prioritaet (getNextTxSlot tie-breakt nach
            // Scan-Reihenfolge ab iRead) -- akzeptiert gegen Totalverlust.
            if (worst_slot != r && ringBuffer[r][0] > 0)
            {
                memcpy(ringBuffer[worst_slot], ringBuffer[r], sizeof(ringBuffer[0]));
                ringPriority[worst_slot]    = ringPriority[r];
                ringEnqueueTime[worst_slot] = ringEnqueueTime[r];
                retryCount[worst_slot]      = retryCount[r];
                ringBuffer[r][0] = 0;
            }
            advanceIReadPastEmpty();
        }
        else
        {
            // New packet is same or lower priority than everything in queue — drop it
            droppedNew = true;
            newLostId = mid;
            stat_drop_count[prio]++;
            ringBuffer[w][0] = 0; // Don't enqueue
        }
    }

    int resultSlot = -1;
    if(!droppedNew)
    {
        // advance tx ring write pointer — inlined addRingPointer (C1): the helper
        // takes volatile int& and cannot bind to the now-atomic iWrite/iRead.
        uint8_t txWnext = (uint8_t)(iWrite + 1);
        if (txWnext >= MAX_RING)
            txWnext = 0;
        iWrite = txWnext;
        if (iRead == txWnext)   // ring full: advance read pointer past the overwritten slot
        {
            uint8_t txRnext = (uint8_t)(txWnext + 1);
            if (txRnext >= MAX_RING)
                txRnext = 0;
            iRead = txRnext;
            ringOverflowAdvance = true;
        }
        resultSlot = w;
    }

#if defined(NRF52_SERIES)
    taskEXIT_CRITICAL();
#endif

    // ---- Ab hier ausserhalb des Locks: nur noch Debug-Ausgabe ----
    if(bLORADEBUG)
    {
        printfdeb("[MC-DBG] RING_WRITE slot=%d type=%02X status=%02X "
                  "len=%d msg_id=%08X queued=%d/%d src=%s\n",
                  w, msgType, ring_status, (int)len, mid, queued, MAX_RING, source);
        printfdeb("[MC-DBG] RING_PRIO slot=%d prio=%d\n", w, prio);

        if(droppedOld)
            printfdeb("[MC-DBG] RING_DROP_PRIO slot=%d prio=%d type=%02X "
                      "msg_id=%08X replaced_by_prio=%d src=%s\n",
                      dropSlot, dropPrio, dropType, dropId, prio, source);
        if(droppedNew)
            printfdeb("[MC-DBG] RING_DROP_NEW slot=%d prio=%d type=%02X "
                      "msg_id=%08X (queue full, no lower prio to evict)\n",
                      w, prio, msgType, newLostId);
        if(ringOverflowAdvance)
            printfdeb("[MC-DBG] RING_OVERFLOW buf=tx\n");
    }

    return resultSlot;
}
