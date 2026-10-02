#include "mc_text.h"
#include "uptime_min.h"   // wrap-safe 16-bit uptime minutes (NBR stamps)
#include "Arduino.h"
#include "configuration.h"

#include "printfdeb_functions.h"
#include "txring_functions.h"
#include "ack_functions.h"
#include "ack_attribution.h"
#include "capture_functions.h"
#include "dedup_functions.h"
#include "setlog_lines.h"
#include "nbr_matrix.h"
#include "nbr_views.h"   // nbrNcntAir(): R<n> der HEY-Gruppe (MeshCom 5 Welle 4)
#include "mh_phone.h"    // mhPhoneLive(): MH-Live-Rahmen an die App
#include "topo_ui.h"     // topoUiChanged(): T-Deck-Anzeigen und /topo.dat
#include "dm_stats.h"       // stage 0/2.1 DM outcome counters (dmstat_*)
#include "reack_limiter.h"  // stage 0.2: rate-limited re-ACK for duplicate DMs
#include "dm_dedup.h"       // stage 2.1: second dedup layer, keyed on (source call, NNN)
#include "instrument.h"     // stage 0.5: --airgap (bAirgap), INSTRUMENT_ENABLED
#include "pn_retry.h"       // PN retry (XOR form): pnRetryCore/pnRetryId/pnFrameIsOwnPn etc.
#include "sto_notice.h"     // stage 4: :sto custody notice, sender side -- every board
#if defined(ENABLE_MSGSTORE)
#include "msgstore_api.h"   // stage 3: store node (last-hop mailbox) receive-path hooks
#endif

#ifdef SX127X
    #include <RadioLib.h>
    extern SX1278 radio;
    extern volatile int transmissionState;
#endif

#ifdef BOARD_E220
    #include <RadioLib.h>
    // RadioModule derived from SX1262 
    extern LLCC68 radio;
    extern volatile int transmissionState;
#endif

#ifdef SX1262X
    #include <RadioLib.h>
    extern SX1262 radio;
    extern volatile int transmissionState;
#endif

#ifdef SX126X
    #include <RadioLib.h>
    extern SX1268 radio;
    extern volatile int transmissionState;
#endif

#if defined(SX1262_E22) || defined(USING_SX1262)
    #include <RadioLib.h>
    extern SX1262 radio;
    extern volatile int transmissionState;
#endif

#ifdef SX1268_E22
    #include <RadioLib.h>
    extern SX1268 radio;
    extern volatile int transmissionState;
#endif

#if defined(SX1262_V3) || defined(SX1262_E290) || defined(SX1262_V4)
    #include <RadioLib.h>
    extern SX1262 radio;
    extern volatile int transmissionState;
#endif

#ifdef BOARD_HELTEC_V4
    #include "esp32/pa_control.h"
#endif

#if defined(BOARD_T5_EPAPER)
#include <t5-epaper/t5epaper_extern.h>
#include <t5-epaper/t5epaper_main.h>
#endif

#if defined(BOARD_RAK4630)
#include <nrf52/nrf52_radio.h>
#endif

#include "lora_functions.h"
#if defined(EXTERNAL_RADIO)
#include "external_radio_txq.h"   // async external-TX ownership record
#endif
#include "loop_functions.h"
#include <loop_functions_extern.h>
#include "aprs_functions.h"
#include <batt_functions.h>
#include <udp_functions.h>
#include <extudp_functions.h>
#include <kiss_functions.h>
#include <lora_setchip.h>

#include "softser_functions.h"

#include "test_inject.h"   // TM-06: raw-inject drain / TX-burst ticker, serviced from OnRxDone()

#include "via_functions.h"

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
#include <t-deck/lv_obj_functions.h>
#endif

// Issue 962 / --deepsleep: RadioLib radio.sleep() for every board where this
// TU has the real `radio` object in scope (see the #ifdef ladder above).
// Mirrors the working precedent at src/t5-epaper/peri_lora.cpp:276.
// WP_DISP boards keep their own Platform::loraToSleep() call instead (see
// lora_functions.h for why). T-Deck Pro is NOT excluded here: its variant
// defines SX1262X, and the `SX1262 radio` that extern resolves to is the
// real global object in esp32_main.cpp (guarded by the same SX1262X), not
// the unrelated `static` (file-local) radio in src/t-deck-pro/peri_lora.cpp,
// which is dead scaffolding -- every function body in that file besides the
// static declarations is commented out, so its local `radio` is never used.
#if (defined(SX127X) || defined(BOARD_E220) || defined(SX1262X) || defined(SX126X) || \
     defined(SX1262_E22) || defined(USING_SX1262) || defined(SX1268_E22) || \
     defined(SX1262_V3) || defined(SX1262_E290) || defined(SX1262_V4) || \
     defined(BOARD_T5_EPAPER)) && !defined(WP_DISP)
void loraDeepSleep()
{
    radio.sleep();
}
#endif

                                        // flag to indicate if we are after receiving
extern unsigned long iReceiveTimeOutTime;

int sendlng = 0;
uint8_t lora_tx_buffer[UDP_TX_BUF_SIZE+10];  // lora tx buffer
uint8_t preamble_cnt = 0;     // stores how often a preamble detect is thrown

unsigned long track_to_meshcom_timer = 0;

bool bNewLine = false;

// ONRXDONE_TIME monitoring (always active, not behind bLORADEBUG)
unsigned long onrxdone_max_ms = 0;
unsigned int  onrxdone_warn_count = 0;

// Deferred display update — avoid I2C transfer inside OnRxDone
volatile bool bPendingDisplayText = false;
volatile bool bPendingDisplayPos = false;

// SPI bus guard — prevent LoRa ISR from accessing SPI while Ethernet (W5100S) is active
// Both chips share the single SPI bus on RAK4631 (pins 3/29/30).
volatile bool bSPI_ETH_Active = false;
volatile bool bPendingRadioRx = false;
struct aprsMessage pendingDisplayMsg;
int16_t pendingDisplayRssi = 0;
int8_t  pendingDisplaySnr = 0;

// RACE-01: the ESP32 spinlock that used to guard pendingDisplayMsg was removed
// in 4a250602 along with every portENTER_CRITICAL(&displayMux) that took it;
// the variable itself outlived them until 2026-09-12. It is gone now, and so is
// the comment that claimed a lock existed here -- which was the actual cost of
// leaving it: three files told a reader this struct was protected on ESP32.
// It does not need to be. On ESP32 OnRxDone runs synchronously inside
// esp32loop() (architecture/09-concurrency-map.md), so the producer
// queueDisplayText()/queueDisplayPosition() and the consumer
// flushDeferredDisplayUpdates() are the same task and cannot preempt each
// other. On nRF52 they genuinely can -- OnRxDone runs in the LoRa task, the
// drain in loop() -- and that side is guarded by taskENTER_CRITICAL() below,
// on all three nRF52 boards (see the RF-07 note).

// RF-07 (BACKLOG): this guard reads as RAK-only but is not. `BOARD_RAK4630`
// is defined for ALL THREE nRF52 boards -- platformio.ini's shared
// [nrf52_base] build_flags carries `-D BOARD_RAK4630="RAK4630"` and both
// heltec_t114 and t_echo build on `${nrf52_base.build_flags}`, deliberately
// (platformio.ini's own comment: "ABSICHTLICH mitgeerbt ... schaltet ueber
// 60 nRF52-Codestellen frei"). Verified by compiling this TU for both envs
// (`pio run -e heltec_t114/t_echo -t compiledb`) and grepping the resulting
// command line: BOARD_RAK4630 is present on both. So the critical section
// below already compiles in on T114/T-Echo, matching the consumer side
// (nrf52_main.cpp's unconditional taskENTER_CRITICAL() around the drain,
// which every nRF52 board builds). Same producer/consumer split as RAK4631
// (OnRxDone runs in the FreeRTOS timer-service task, prio 2, see C-01 /
// architecture/09-concurrency-map.md; the drain runs in loop() at prio 1),
// so the race this guards against is real on T114/T-Echo too -- and already
// covered. Closed as NOT a defect; do not widen this to `|| USE_HELTEC_T114
// || BOARD_T_ECHO`, that would be a no-op at best and misleading at worst.
// Queue display text update for main loop execution
static void queueDisplayText(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
#if defined(BOARD_RAK4630)
    taskENTER_CRITICAL();
#endif
    pendingDisplayMsg = aprsmsg;
    pendingDisplayRssi = rssi;
    pendingDisplaySnr = snr;
    bPendingDisplayText = true;
#if defined(BOARD_RAK4630)
    taskEXIT_CRITICAL();
#endif
}

// Queue display position update for main loop execution
// RF-07: same BOARD_RAK4630 note as queueDisplayText() above -- already
// covers T114/T-Echo, see there.
static void queueDisplayPosition(struct aprsMessage &aprsmsg, int16_t rssi, int8_t snr)
{
#if defined(BOARD_RAK4630)
    taskENTER_CRITICAL();
#endif
    pendingDisplayMsg = aprsmsg;
    pendingDisplayRssi = rssi;
    pendingDisplaySnr = snr;
    bPendingDisplayPos = true;
#if defined(BOARD_RAK4630)
    taskEXIT_CRITICAL();
#endif
}

/**
 * Extract the 4-byte message ID from a ring buffer slot.
 * ringBuffer layout: [0]=len, [1]=status, [2]=msg_type, [3..6]=msg_id (LE)
 */
static uint32_t extractRingMsgId(int slot)
{
    return ((uint32_t)ringBuffer[slot][6] << 24) |
           ((uint32_t)ringBuffer[slot][5] << 16) |
           ((uint32_t)ringBuffer[slot][4] << 8)  |
            (uint32_t)ringBuffer[slot][3];
}

// RX-01 (BACKLOG 3.8k): counts and rate-limits the "frame from an
// unconfigured node, dropped" marker. Not static -- udp_functions.cpp's
// GATE-in path (the "second door", server -> LoRa) shares this counter and
// marker via a local extern declaration there. Raw Serial.printf, not
// printfdeb/DEBUG_MSG (stripped/compiled away with --debug off, see the
// FL-01 marker note) -- at most one line per 10 s, folding any elided drops
// into the "dropped" count on the next line that does print (TM-21's
// lesson).
uint32_t stat_rx_drop_unconfigured = 0;

void logRxDropUnconfigured(const char *call)
{
    static bool s_have_marker = false;
    static uint32_t s_last_marker_ms = 0;
    static uint32_t s_dropped_since_marker = 0;

    stat_rx_drop_unconfigured++;
    s_dropped_since_marker++;

    uint32_t now = (uint32_t)millis();
    // s_have_marker, not "s_last_marker_ms == 0": millis()==0 is a real,
    // reachable timestamp (boot), and must not double as a "never printed
    // yet" sentinel -- that would let a second drop still at ms=0 bypass
    // the floor.
    if(!s_have_marker || (uint32_t)(now - s_last_marker_ms) >= 10000UL)
    {
        Serial.printf("[RX];drop;unconfigured;src;%s;ms;%lu;dropped;%lu\n",
                      call, (unsigned long)now, (unsigned long)s_dropped_since_marker);
        s_have_marker = true;
        s_last_marker_ms = now;
        s_dropped_since_marker = 0;
    }
}

// DR-25 (docs/testplan/drift-matrix.csv, U2): the outbound UDP-out drain's
// own copy of the RX-01 idea (esp32/udp_drain_esp32.cpp, nrf52/udp_drain_
// nrf52.cpp) detects an unconfigured-source frame AFTER the send has already
// happened -- a LEAK, not a drop. It must not share stat_rx_drop_
// unconfigured/logRxDropUnconfigured() above: a rising count at the inbound
// doors (this file's OnRxDone guard, and udp_frame_{esp32,nrf52}.cpp's GATE)
// means the primary guard is WORKING; a rising count here means it is NOT --
// a frame the primary guard should have stopped reached the ring and went
// out. Same shape (10 s marker floor, raw Serial.printf -- not printfdeb,
// which strips ';' outside --debug csv and this line's format depends on
// them), distinct counter and line so the two events are never conflated.
uint32_t stat_tx_leak_unconfigured = 0;

void logTxLeakUnconfigured(const char *call)
{
    static bool s_have_marker = false;
    static uint32_t s_last_marker_ms = 0;
    static uint32_t s_leaked_since_marker = 0;

    stat_tx_leak_unconfigured++;
    s_leaked_since_marker++;

    uint32_t now = (uint32_t)millis();
    if(!s_have_marker || (uint32_t)(now - s_last_marker_ms) >= 10000UL)
    {
        Serial.printf("[TX];leak;unconfigured;src;%s;ms;%lu;leaked;%lu\n",
                      call, (unsigned long)now, (unsigned long)s_leaked_since_marker);
        s_have_marker = true;
        s_last_marker_ms = now;
        s_leaked_since_marker = 0;
    }
}

#if defined(EXTERNAL_RADIO)
// The single in-flight external-radio TX ownership record. At most one ring
// slot may be RING_STATUS_EXT_PENDING at a time (enforced by ExtTxq::begin).
// Defined here (above the ACK-clear paths) so those paths can invalidate it.
static extradio::ExtTxq g_extTxq;

// Bounded external channel-access (CHANNEL_BUSY) attempt budget,
// counted separately from the MeshCom delivery retryCount and tracked PER RING
// SLOT so interleaving queued messages keep independent, bounded episodes.
static extradio::ExtBusyEntry g_extBusy[MAX_RING];

// If `slot` is the externally-pending slot, drop the ownership record (the slot
// is about to be cleared by receive-side ACK handling). Returns true if it was
// owned. A late bridge result for the dropped token is afterwards rejected as
// stale, so it can never resurrect or alter the (possibly reused) slot.
static bool extTxqAckInvalidateIfOwned(int slot)
{
    if(!g_extTxq.owns(slot))
        return false;
    g_extTxq.ackInvalidate(slot);
    extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);  // ACK clears this slot's busy episode
    return true;
}
#endif

/**
 * Find and stop retransmission of a message by uint32_t msg_id.
 * Sets slot status to RING_STATUS_DONE and clears retryCount.
 * Returns slot index, or -1 if not found.
 *
 * PN-Wiederholung (XOR, pn_retry.h): Vergleich ueber pnRetryCore(), damit
 * ein ACK auf die urspruengliche msg-id auch einen auf Retry-Variante
 * umgeschriebenen Slot stoppt (nur eigene pending Slots im Scope).
 *
 * Non-static: auch vom Server-ACK-Pfad gerufen (udp_frame_esp32.cpp,
 * udp_frame_nrf52.cpp), der auf nRF52 im Loop-Task laeuft, waehrend
 * OnRxDone() im LORA-Task laeuft -- Scan und Status-Schreiben deshalb auf
 * RAK4631 unter derselben Ring-Sperre wie anderswo in dieser Datei
 * (queueDisplayText() etc.).
 */
int findAndStopRingSlot(uint32_t msgId)
{
#if defined(BOARD_RAK4630)
    taskENTER_CRITICAL();
#endif
    for(int i = 0; i < MAX_RING; i++)
    {
        if(ringBuffer[i][0] > 0 && ringBuffer[i][1] != RING_STATUS_DONE && ringBuffer[i][1] != RING_STATUS_READY)
        {
            if(pnRetryCore(extractRingMsgId(i)) == pnRetryCore(msgId))
            {
#if defined(EXTERNAL_RADIO)
                // ACK-before-result. If this slot is owned by an in-flight
                // external TX, drop the ownership record now (deliberate release)
                // so a late bridge result for it is later rejected as stale and
                // cannot resurrect or alter the slot once it is reused.
                if(extTxqAckInvalidateIfOwned(i))
                {
                    if(bDisplayRetx)
                        printfdeb("\n[RETX] ext-pending slot retid:%i ACK-resolved before bridge result\n", i);
                }
#endif
                ringBuffer[i][1] = RING_STATUS_DONE;
                ringBuffer[i][0] = 0;  // clear len so getNextTxSlot skips this slot
                retryCount[i] = 0;
#if defined(BOARD_RAK4630)
                taskEXIT_CRITICAL();
#endif
                return i;
            }
        }
    }
#if defined(BOARD_RAK4630)
    taskEXIT_CRITICAL();
#endif
    return -1;
}

// SL-01 -- Dedup-Verdikt zaehlen, genau einmal je Frame und dort, wo die
// Entscheidung faellt. Die Zaehler muessen zu RX_DEDUP_NEW/RX_DEDUP_DUP passen.
static inline bool setlogCountDedup(bool is_new)
{
    if(is_new)
        stat_newid.fetch_add(1);
    else
        stat_dup.fetch_add(1);

    return is_new;
}

// SL-01 -- eigenes Rufzeichen als vollstaendiges Pfadglied im Source-Path?
// Wie die Loop-Erkennung im Relay-Block, aber ohne String-Allokation.
static bool setlogPathHasCall(const char *path, const char *call)
{
    if(path == NULL || call == NULL || call[0] == 0x00)
        return false;

    size_t lc = strlen(call);
    const char *p = path;

    while((p = strstr(p, call)) != NULL)
    {
        bool left_ok  = (p == path) || (p[-1] == ',');
        bool right_ok = (p[lc] == 0x00) || (p[lc] == ',');

        if(left_ok && right_ok)
            return true;

        p += lc;
    }

    return false;
}

/**
 * Handle incoming ACK packet (msg_type 0x41).
 * Returns true if packet was processed as ACK, false otherwise.
 */
static bool handleACK(uint8_t *payload, uint16_t size, int rssi, int snr)
{
    if(payload[0] != MSG_TYPE_ACK)
        return false;

    if(size < 12)
        return false;

    // Plausibilitaetspruefung vor jeder weiteren Verarbeitung: 0x41 ist als
    // ASCII der Buchstabe 'A', ohne diese Pruefung laeuft jedes Bruchstueck,
    // das damit beginnt, durch den ACK-Pfad und wird mit Prio 1 weitergesendet.
    // Begruendung und Feldmessung siehe ack_functions.h.
    if(!isPlausibleAckFrame(payload, size, MAX_HOP_LIMIT))
    {
        if(bLORADEBUG)
            printfdeb("[MC-DBG] ACK_REJECT b5=%02X len=%u\n", payload[5], (unsigned)size);

        return false;
    }

    uint8_t print_buff[30];

    uint8_t n = ackWireAppendixLen(payload, size);
    memcpy(print_buff, payload, 12 + n);

    if(n == ACK_WIRE_APPENDIX_LEN && bLORADEBUG)
        printfdeb("[MC-DBG] ACK_APPENDIX hash=%06X len=%u\n", ackWireHash(payload), (unsigned)size);

    // SL-01: Dedup-Verdikt vor dem Druck, damit die [LOG]-Zeile `DUP:` fuehren
    // kann. is_new_packet() ist eine reine Suche ohne Seiteneffekt.
    bool bIsNew = setlogCountDedup(is_new_packet(print_buff+1));

    if(bDisplayLog)
    {
        // ACK-Frames tragen keinen Pfad, deshalb OWN: immer '-'.
        char tail[56];
        setlogFormatRxTail(tail, sizeof(tail), (int16_t)rssi, (int8_t)snr, !bIsNew, false, (uint32_t)millis());
        printBuffer_ack((char*)"[LOG]", payload, (int16_t)size, tail);
    }

    bool bServerFlag = false;
    if((print_buff[5] & 0x80) == 0x80)
        bServerFlag=true;

    unsigned msg_id = print_buff[6] | (print_buff[7] << 8) | (print_buff[8] << 16) | (print_buff[9] << 24);

    // PN-Wiederholung (XOR, pn_retry.h): quittiert ein Relay oder Gateway eine
    // Retry-Kopie, traegt das ACK deren id (Bits 10-11 gekippt). own_msg_id[]
    // kennt nur die Original-id -- ohne Zurueckfalten fand checkOwnTx() nichts:
    // kein 0x01-Frame ans Telefon, kein Stopp der Wiederholung, und das ACK
    // wurde als fremdes weitergesendet. Ab hier gilt die Original-id.
    msg_id = pnOwnTxLookupId(msg_id, _GW_ID);
    int itxcheck = checkOwnTx(msg_id);

    if(bIsNew || itxcheck >= 0)
    {
        // add rcvMsg to forward to LoRa TX
        if(bIsNew)
        {
            unsigned int mid=(print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
            addLoraRxBuffer(mid, bServerFlag);
        }

        if(itxcheck >= 0)
        {
            // Status frame to the phone is origin-gated: own_msg_id[] also holds
            // foreign msg_ids that a gateway only forwarded from the server to LoRa
            // (see docs/ack-heard-foreign-msgids-fix.md). The state write below stays
            // unconditional so the web rxlog heard/ACK ticks keep working as today.
            if(bAckInfo || own_msg_id[itxcheck][4] < 2)   // 00...not heard, 01...heard, 02...ACK
            {
                if(ackMsgIdFromNode(msg_id, _GW_ID))
                {
                    uint8_t phone_buff[ACK_PHONE_MAX_LEN];
                    uint16_t plen = buildAckPhoneFrame(phone_buff, msg_id, 0x01, "");
                    addBLEOutBuffer(phone_buff, plen);
                    dmstat_gw_ack.fetch_add(1);   // 0.3: itxcheck >= 0 already gates this block

                    if(bDisplayInfo)
                    {
                        printfdeb("\n%s", getTimeString().c_str());
                        printfdeb(" ACK to Phone  %02X %02X%02X%02X%02X %02X %02X", phone_buff[0], phone_buff[4], phone_buff[3], phone_buff[2], phone_buff[1], phone_buff[5], phone_buff[6]);
                    }
                }

                own_msg_id[itxcheck][4] = 0x02;   // 02...ACK
            }

            // stop retransmission in ring buffer for this msg_id
            int ackSlot = findAndStopRingSlot(msg_id);
            if(ackSlot >= 0 && bDisplayRetx)
                printfdeb("\n[RETX] binary ACK for retid:%i stop retransmit msg-id:%08X\n",
                              ackSlot, msg_id);
        }
        else
        {
            // ACK nur weitersenden wenn es eine neue MSG-ID ist && MESH = on && nicht eine MSG-ID ist welche nicht selbst ausgesendet wurde
            if((print_buff[5] & 0x7F) > 0x00 && bMESH && itxcheck < 0 && !checkServerRx(print_buff+6))
            {
                print_buff[5]--;

                addTxRingEntry(print_buff, 12 + n, RING_STATUS_DONE, "rx_ack_fwd", 0);

                if(bDisplayInfo)
                {
                    printfdeb(" This packet to mesh");
                }
            }
        }
    }

    if(bDisplayInfo)
    {
        printfdeb("\n");
    }

    return true;
}

// --nbrdebug (24-h-Dauertest der Nachbarschaftsmatrix, docs/nbr-logformat.md):
// Konsolen-Emitter fuer nbrLog. `line` ist eine fertig formatierte [NBR]-Zeile
// ohne abschliessendes '\n' -- printfdeb() entfernt Semikolons ausserhalb von
// --debug csv, deshalb NIE als Format-String durchreichen, nur als %s-Argument.
static void nbrLogToConsole(const char *line)
{
    printfdeb("%s\n", line);
}

// Gleicht nbrLog an bNBRDEBUG an. Genau zwei Aufrufstellen: einmal beim Boot
// (nach dem Zurueckziehen aus node_sset4) und einmal als post()-Hook der
// --nbrdebug on/off-Toggle-Zeilen -- NICHT in OnRxDone() oder im Loop, damit
// der Zeiger nicht bei jedem Frame neu gesetzt wird.
void nbrDebugApply(void)
{
    nbrLog = bNBRDEBUG ? nbrLogToConsole : NULL;

    // Beim Einschalten soll der erste Schnappschuss beim naechsten faelligen
    // 15-Minuten-Takt kommen, nicht erst 15 Minuten nach dem Boot -- also den
    // Takt hier zuruecksetzen statt nur beim Boot zu initialisieren.
    if(bNBRDEBUG)
        nbrsnap_timer = millis();
}

// W3b (docs/meshcom5-campaign.md Welle 3, docs/meshcom5-topologie/ 4.6):
// gemeinsames NbrDirectInfo-Grundgeruest fuer beide nbrNoteDirect()-Aufrufstellen
// unten (regulaerer ':'/'!'/'@'-Zweig und der HN-Zweig) -- Position/Hoehe
// bleiben hier unbesetzt (NBR_ALT_UNKNOWN, has_pos=false); der '!'-Zweig
// traegt sie danach selbst nach, aus der ohnehin schon fuer nbrNotePos()
// dekodierten Position (nur bei own_frame, Konzept 4.6).
static NbrDirectInfo nbrBuildDirectInfo(const struct aprsMessage &aprsmsg, int16_t rssi_here)
{
    NbrDirectInfo info;
    memset(&info, 0, sizeof(info));

    info.plt = aprsmsg.payload_type;
    info.hw  = aprsmsg.msg_last_hw & 0x7F;

    // Gleiche 0x80-Regel wie frueher mh_mod im MHeard-Block (bis Welle 4):
    // 0x80 gesetzt heisst "letzter Hop ist die
    // sendende Station selbst" (aprs_functions.cpp:129/1122 setzen das Bit
    // beim Senden), sonst kommt der Modulationswert von einem Absender, den
    // dieser Rahmen nicht direkt bestaetigt.
    info.mod = ((aprsmsg.msg_last_hw & 0x80) == 0x80) ? aprsmsg.msg_source_mod
                                                        : (uint8_t)(aprsmsg.msg_source_mod | 0xF0);
    info.rssi = rssi_here;

    // Sekunde aus der Wanduhr, wenn sie steht (gleicher Jahres-Test wie
    // frueher das MHeard (bis Welle 4): "< 2025" heisst
    // "noch kein NTP/GPS/Telefon-Sync seit Boot"), sonst aus millis() (CONTRACT
    // in nbr_matrix.h).
    info.sec = (meshcom_settings.node_date_year >= 2025)
                   ? (uint8_t)meshcom_settings.node_date_second
                   : (uint8_t)((millis() / 1000UL) % 60);

    info.pl   = aprsmsg.msg_last_path_cnt;
    info.mesh = aprsmsg.msg_mesh;
    info.own_frame = is_equ(aprsmsg.msg_source_call, aprsmsg.msg_source_last);
    info.fw   = info.own_frame ? aprsmsg.msg_source_fw_sub_version : 0;

    info.has_pos = false;
    info.lat = NBR_POS_NONE;
    info.lon = NBR_POS_NONE;
    info.alt_m = NBR_ALT_UNKNOWN;

    return info;
}

// W3b: liest das "R<n>"-Feld eines HEY-'@'-Payloads -- dieselbe Grammatik, mit
// der das MHeard bis Welle 4 seinen NCNT las (Referenzkopie heute in
// test/test_topo_shadow/reference/). "R<digits>;" oder
// "R<digits>,<digits>,<digits>;" (0 oder 2 Kommas vor dem ersten ';') ist
// gueltig, alles andere (altes Zwei-Komma-Format, fehlendes 'R', kein Feld)
// liefert false, *out_n bleibt unangetastet. Ein fehlendes abschliessendes
// ';' wird wie dort defensiv angehaengt.
static bool nbrParseHeyReportedCount(const char *payload, long *out_n)
{
    char buf[MC_PAYLOAD_LEN];
    if(!mcSet(buf, sizeof(buf), payload))
        return false;
    mcAppend(buf, sizeof(buf), ";");

    int ipos = mcIndexOf(buf, ';');
    if(ipos <= 0 || !mcStartsWith(buf, "R"))
        return false;

    int icomma = 0;
    for(int i = 1; i < ipos; i++)
    {
        if(buf[i] == ',')
            icomma++;
    }

    if(icomma != 0 && icomma != 2)
        return false;

    *out_n = mcSliceToLong(buf, 1, (size_t)ipos);
    return true;
}

// MeshCom 5 Welle 4 (Konzept 4.9): Live-MH-Rahmen an die App hoechstens einmal
// je Nachbar und Minute, ausgeloest von einer neuen Minute der Kante (x, 0)
// ("ich habe x gehoert"), nicht von jedem Rahmen. nbrMhEdgeBefore() liest die
// Kantenminute des letzten Hops VOR nbrNoteFrame(), nbrMhLiveAfter() NACH
// nbrNoteDirect(); beide suchen per Rufzeichen, damit eine zwischendurch
// verdraengte und neu vergebene Zeile nicht verwechselt wird.
struct NbrMhEdgeMark
{
    bool     known;      // Kante (x, 0) lebte schon vor diesem Rahmen
    uint16_t last_min;
};

static NbrMhEdgeMark nbrMhEdgeBefore(const char *last_hop)
{
    NbrMhEdgeMark mark = {false, 0};
    NbrEdgeView ev;
    int row = nbrFind(nbrMatrix, last_hop);

    if(row > 0 && nbrEdgeGet(nbrMatrix, row, 0, &ev))
    {
        mark.known = true;
        mark.last_min = ev.last_min;
    }

    return mark;
}

static void nbrMhLiveAfter(const char *last_hop, const NbrMhEdgeMark &before, uint16_t now_min)
{
    if(isPhoneReady != 1)
        return;

    NbrEdgeView ev;
    int row = nbrFind(nbrMatrix, last_hop);

    if(row <= 0 || !nbrEdgeGet(nbrMatrix, row, 0, &ev))
        return;

    if(before.known && ev.last_min == before.last_min)
        return;

    mhPhoneLive(row, now_min);
}

// F6 (docs/soak-20260928-verdict.md Finding 6): the HEY destination tells the
// neighbours whether the originator is a gateway. "HG" = yes, "H" = no, any
// other frame (HN report, text, position) says nothing about it.
static NbrGwHint nbrGwHintFromDest(char payload_type, const char *dest)
{
    if(payload_type != '@')
        return NBR_GW_UNKNOWN;
    if(is_equ(dest, "HG"))
        return NBR_GW_YES;
    if(is_equ(dest, "H"))
        return NBR_GW_NO;
    return NBR_GW_UNKNOWN;
}

//////////////////////////////////////////////////////////////////////////
// LoRa RX functions

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    // Testfang-Hook (Katalog doc 08 §4, Mechanismus 2): akzeptierte Frames als
    // Rohbytes — das Gegenstueck zum CRC_PAYLOAD-Dump der VERWORFENEN Frames
    // in checkRX (esp32_main.cpp). OnRxDone ist auf beiden Plattformen der
    // erste Punkt, den nur akzeptierte Frames erreichen.
    //
    // Frueher hing das hinter `-D MC_TEST_HOOKS` und druckte hier direkt, was
    // in keinem Produktionsbuild lief: der Dump saesse im Radio-Callback
    // (LORA-Task) und printfdeb() braucht allein ~900 Byte Stack, dazu
    // ~48 ms Serial-Zeit mitten im RX-Pfad.
    // captureFrame() kopiert nur; ausgegeben wird aus dem Loop
    // (captureDrain() in main.cpp), siehe capture_functions.h.
    if(bLORADEBUG)
        captureFrame('R', payload, size, rssi, snr);

    // Debug I: OnRxDone timing — capture start time
    unsigned long _onrxdone_start = millis();

    {
        unsigned long _rx_s = ch_util_rx_start.exchange(0);
        if(_rx_s > 0)
            ch_util_rx_accum.fetch_add(millis() - _rx_s);
#if defined BOARD_RAK4630
        // Fallback: OnHeaderDetect feuert nicht zuverlaessig auf nRF52,
        // daher RX-Airtime aus Paketlaenge berechnen (wie ESP32 in checkRX).
        else if(size > 0)
            ch_util_rx_accum.fetch_add(Radio.TimeOnAir(MODEM_LORA, size));
#endif
    }

    #if defined BOARD_RAK4630
    // FIX BUG #2 (nRF52): RX sofort neu starten um Blindfenster zu minimieren.
    // Sicherheitskopie: Payload koennte auf internen Radiopuffer zeigen,
    // der durch Radio.Rx() ueberschrieben wird.
    static uint8_t rxPayloadCopy[2][UDP_TX_BUF_SIZE];  // Double-Buffer
    static uint8_t rxBufIndex = 0;                       // Aktueller Buffer-Index
    static bool rxBufInUse[2] = {false, false};          // Buffer-Belegung

    uint16_t rxSize = (size <= UDP_TX_BUF_SIZE) ? size : UDP_TX_BUF_SIZE;

    // RACE-04 fix: critical section around double-buffer swap to prevent
    // re-entrant ISR from corrupting buffer state
    taskENTER_CRITICAL();
    uint8_t nextBuf = (rxBufIndex + 1) % 2;
    bool _overwrite = rxBufInUse[nextBuf];
    rxBufIndex = nextBuf;
    rxBufInUse[rxBufIndex] = true;
    memcpy(rxPayloadCopy[rxBufIndex], payload, rxSize);
    payload = rxPayloadCopy[rxBufIndex];
    size = rxSize;
    taskEXIT_CRITICAL();

    // Debug logging outside critical section
    if(_overwrite)
    {
#ifdef LORA_ISR_DEBUG
        printfdeb("[MC-DBG] RX_BUF_OVERWRITE buf=%d (still in use)\n", rxBufIndex);
#endif
    }
    else if(bLORADEBUG)
    {
#ifdef LORA_ISR_DEBUG
        printfdeb("[MC-DBG] RX_BUF_SWITCH buf=%d\n", rxBufIndex);
#endif
    }
    // SPI guard: defer Radio.Rx() if Ethernet (W5100S) owns the shared SPI bus
    if(bSPI_ETH_Active) {
        bPendingRadioRx = true;
    } else {
        startRadioReceive();
    }
    // RACE-05 fix: CAD abort under critical section
    taskENTER_CRITICAL();
    bool _cad_was_active = cad_in_progress;
    if(cad_in_progress) {
        cad_in_progress = false;
        cad_done_flag = false;
        cad_double_check = false;
    }
    taskEXIT_CRITICAL();
    if(_cad_was_active && bLORADEBUG)
    {
#ifdef LORA_ISR_DEBUG
        printfdeb("[MC-DBG] CAD_ABORT_BY_RX\n");
#endif
    }
    if(bLORADEBUG)
    {
#ifdef LORA_ISR_DEBUG
        printfdeb("[MC-DBG] RX_RESTART_EARLY src=OnRxDone\n");
#endif
    }
    // Log RX_LISTEN -> RX_PROCESS here (not in OnHeaderDetect ISR where
    // Serial.printf is unreliable on nRF52)
    if(bLORADEBUG)
    {
#ifdef LORA_ISR_DEBUG
        printfdeb("[MC-SM] RX_LISTEN -> RX_PROCESS rc=0\n");
#endif
    }
    #endif

    // only for Test T5_EPAPER
    //bDisplayInfo=true;

    uint8_t print_buff[30];

    // SL-02: Puffer fuer RLY/GWU (nie beide im selben Durchlauf). Laengste
    // Zeile: "RLY x12345678 : H03 q=gwfilter prio=5 slot=19".
    char setlog_buf[96];

    //printfdeb("Start OnRxDone:<%c#%-20.20s> %i\n", payload[0], payload+6, size);

    bNewLine=false;

    bLED_GREEN = true;

#if INSTRUMENT_ENABLED
    // 0.5 --airgap: drop the frame here, after the platform RX plumbing has
    // run (buffer swap, LED, timing capture) but before handleACK()/
    // is_new_packet()/mheard -- an airgapped node still occupies its RX
    // slot on air, it just never processes what lands in it.
    if(bAirgap)
    {
#if defined BOARD_RAK4630
        taskENTER_CRITICAL();
        rxBufInUse[rxBufIndex] = false;
        taskEXIT_CRITICAL();
#endif
        is_receiving = false;
        iReceiveTimeOutTime = millis();
        csma_timeout = csma_compute_timeout(cad_attempt);
        return;
    }
#endif

    if(handleACK(payload, size, rssi, snr))
    {
#if defined BOARD_RAK4630
        taskENTER_CRITICAL();
        rxBufInUse[rxBufIndex] = false;
        taskEXIT_CRITICAL();
#endif
        is_receiving = false;

        // Debug I: ONRXDONE_TIME
        if(bLORADEBUG)
            printfdeb("[MC-DBG] ONRXDONE_TIME ms=%lu\n", millis() - _onrxdone_start);

        iReceiveTimeOutTime = millis();
        csma_timeout = csma_compute_timeout(cad_attempt);

        // TM-06: state above is fully settled -- safe point for
        // test_inject_service() to recurse into OnRxDone() (see there).
        test_inject_service();

        return;
    }

    {
        memcpy(RcvBuffer, payload, size);

        // RX-OK do not need retransmission
        // RX-OK: find matching ring buffer slot and release it
        uint32_t dbg_msg_id = 0;
        int rxSlot = -1;
        for(int i = 0; i < MAX_RING; i++)
        {
            if(ringBuffer[i][0] > 0 && ringBuffer[i][1] != RING_STATUS_DONE && ringBuffer[i][1] != RING_STATUS_READY)
            {
                if(memcmp(ringBuffer[i]+3, RcvBuffer+1, 4) == 0)
                {
                    rxSlot = i;
                    dbg_msg_id = extractRingMsgId(i);
                    break;
                }
            }
            else if(ringBuffer[i][0] > 0 && ringBuffer[i][1] == RING_STATUS_READY)
            {
                if(memcmp(ringBuffer[i]+3, RcvBuffer+1, 4) == 0)
                {
                    if(bLORADEBUG)
                        printfdeb("[MC-DBG] ACK_SKIP_READY slot=%d msg_id=%08X\n", i, extractRingMsgId(i));
                }
            }
        }
        if(rxSlot >= 0)
        {
            uint8_t dbg_status = ringBuffer[rxSlot][1];
            uint8_t dbg_lng = ringBuffer[rxSlot][0];
            uint8_t dbg_type = ringBuffer[rxSlot][2];

#if defined(EXTERNAL_RADIO)
            // ACK-before-result — drop external ownership before releasing
            // the slot so a late bridge result is later rejected as stale.
            extTxqAckInvalidateIfOwned(rxSlot);
#endif

            // PN-Wiederholung (XOR, pn_retry.h): das eigene Echo einer PN ist
            // kein Grund, die Wartezeit abzubrechen -- nur ein :ackNNN vom
            // echten Ziel darf das (findAndStopRingSlot()). Wartezeit neu
            // starten (wie doTX() es nach dem Senden tut) statt freizugeben.
            // Nur fuer eine PN, die WIR ausgeloest haben: eine PN, die
            // sendMessage() fuer einen KISS-Client sendet, bleibt beim
            // bisherigen Verhalten (Freigabe beim ersten Echo).
            //
            // M1: Ausnahme -- das Ziel hat diese PN schon geackt (own_msg_id
            // 0x02, ueber die Original-id). Dann nicht neu starten, sondern
            // wie bisher freigeben, sonst bleibt der Slot bis zum naechsten
            // Schwellwert (40 s) belegt.
            bool pnEcho = dbg_type == MSG_TYPE_TEXT &&
               pnFrameIsOwnPn(ringBuffer[rxSlot] + 2, dbg_lng, _GW_ID, meshcom_settings.node_call);
            bool pnEchoAcked = false;
            if(pnEcho)
            {
                uint32_t pnEchoOrigId = pnRetryId(pnFrameMsgId(ringBuffer[rxSlot] + 2), _GW_ID, 0);
                int pnEchoIdx = checkOwnTx(pnEchoOrigId);
                pnEchoAcked = (pnEchoIdx >= 0 && own_msg_id[pnEchoIdx][4] == 0x02);
            }

            if(pnEcho && !pnEchoAcked)
            {
                ringBuffer[rxSlot][1] = RING_STATUS_SENT;

                if(bDisplayRetx)
                    printfdeb("\n[RETX] PN echo, wait restarted retid:%i status:%02X lng;%i msg-id:%c-%08X\n",
                                  rxSlot, dbg_status, dbg_lng, dbg_type, dbg_msg_id);
            }
            else
            {
                // Jetzt erst release
                ringBuffer[rxSlot][1] = RING_STATUS_DONE;
                retryCount[rxSlot] = 0;
                ringBuffer[rxSlot][0] = 0;

                if(bDisplayRetx)
                    printfdeb("\n[RETX] got lora rx for retid:%i no need status:%02X lng;%i msg-id:%c-%08X\n",
                                  rxSlot, dbg_status, dbg_lng, dbg_type, dbg_msg_id);
                if(bLORADEBUG)
                    printfdeb("[MC-DBG] ACK_RECEIVED retid=%d msg_id=%08X\n",
                                  rxSlot, dbg_msg_id);
            }
        }

        struct aprsMessage aprsmsg;
        
        // print which message type we got
        uint16_t msg_type_b_lora = decodeAPRS(RcvBuffer, size, aprsmsg);

        // TM-06(a): no-op unless this call is draining a staged raw-inject
        // frame (test_inject.cpp) -- see test_inject_service() below.
        test_inject_raw_report(size, msg_type_b_lora);

        size = aprsmsg.msg_len;

        int icheck = checkOwnTx(aprsmsg.msg_id);

        // SL-01: genau EIN is_new_packet() je Frame (die Suche druckt unter
        // --loradebug); unten von setlogCountDedup() wiederverwendet.
        bool rx_is_new = is_new_packet(RcvBuffer+1);
        bool rx_dup = !rx_is_new;
        bool rx_own_echo = false;

        // PN-Wiederholung (XOR, pn_retry.h): gleiche PN unter einer anderen
        // Wiederholungsvariante schon gesehen? Reiner msg-id-Dedup (oben)
        // sieht das nicht -- checkOwnRx() ist die stille Ringabfrage (kein
        // zweites is_new_packet()).
        bool rx_pn_shape = msg_type_b_lora == MSG_TYPE_TEXT &&
           pnPayloadIsPn(aprsmsg.msg_payload, strlen(aprsmsg.msg_payload)) &&
           pnDestIsPersonal(aprsmsg.msg_destination_call, strlen(aprsmsg.msg_destination_call));
        bool rx_pn_repeat = false;
        if(rx_is_new && rx_pn_shape)
        {
            uint32_t pn_variants[3];
            pnVariantIds(aprsmsg.msg_id, pn_variants);
            for(int pv = 0; pv < 3 && !rx_pn_repeat; pv++)
            {
                uint8_t pn_variant_buf[4];
                pnIdToLe(pn_variants[pv], pn_variant_buf);
                if(checkOwnRx(pn_variant_buf) >= 0)
                    rx_pn_repeat = true;
            }

            if(rx_pn_repeat && bDisplayInfo)
                printfdeb("[RX] PNREPEAT msg-id:%08X\n", aprsmsg.msg_id);
        }

        if(bDisplayLog)
        {
            rx_own_echo = setlogPathHasCall(aprsmsg.msg_source_path, meshcom_settings.node_call);

            char tail[56];
            setlogFormatRxTail(tail, sizeof(tail), (int16_t)rssi, (int8_t)snr, rx_dup, rx_own_echo, (uint32_t)millis());

            if(LogCallsign[0] != 0x00)
            {
                if(is_equ((char*)LogCallsign, aprsmsg.msg_source_call))
                    printBuffer_aprs((char*)"[LOG]", aprsmsg, tail);
            }
            else
                printBuffer_aprs((char*)"[LOG]", aprsmsg, tail);
        }

        // Nachbarschaftsmatrix Stufe 3 (HN-Bericht): der periodische HEY-artige
        // Nachbarschaftsbericht (payload_type '@', Ziel "HN", max_hop 0, siehe
        // sendNbrReport() in loop_functions.cpp) ist KEIN normaler Frame -- er
        // darf weder in die Stufe-2-Deckungs-/Abbruchpruefung unten laufen noch
        // in den grossen if/else-Block ab msg_type_b_lora==0x00 (Dedup-Zaehlung,
        // trickle_consistent_count, Relay, Gateway-/Server-Upload,
        // EXTUDP, Telefon/BLE, Display). Er fuettert ausschliesslich die eigene
        // Matrix (nbrNoteFrame fuer die Pfadkanten wie jeder andere Frame,
        // danach nbrNoteReport fuer den Berichtsinhalt) und verlaesst OnRxDone
        // ueber denselben Aufraeum-/Timing-Ausstieg wie handleACK() weiter oben
        // (RX-Neustart lief auf nRF52 schon am Funktionsanfang).
        // is_equ() statt strcmp(): gleiche Wahl wie ueberall sonst in diesem
        // Block. Der Selbstschutz gegen das eigene Echo ist bei max_hop 0
        // eigentlich unerreichbar (kein Relay wiederholt so ein Frame), bleibt
        // aber als Guard stehen.
        if(msg_type_b_lora != 0 && aprsmsg.payload_type == '@' &&
           is_equ(aprsmsg.msg_destination_call, "HN") &&
           !is_equ(aprsmsg.msg_source_call, meshcom_settings.node_call))
        {
            uint16_t now_min_hn = uptimeMin16();

            // Gleiches Lazy-Init/Positions-/GW-Flag-Muster wie beim regulaeren
            // '@'/':'/'!'-Zweig weiter unten (Stufe 1) -- ohne Zeile 0 faende
            // nbrFind() das eigene Rufzeichen im Pfad nicht und wuerde dafuer
            // faelschlich eine neue Zeile anlegen, wenn der HN-Bericht der
            // allererste verarbeitete Frame nach dem Boot ist.
            if(!nbrOwnCallIs(nbrMatrix, meshcom_settings.node_call))
                nbrInit(nbrMatrix, meshcom_settings.node_call, now_min_hn);

            if(!nbrRowHasFlag(nbrMatrix, 0, NBR_FLAG_POS) && meshcom_settings.node_lat != 0.0)
                nbrNotePos(nbrMatrix, meshcom_settings.node_call, (float)meshcom_settings.node_lat,
                           (float)meshcom_settings.node_lon, bMESH, 0, now_min_hn);
            if(bGATEWAY)
                nbrRowSetFlag(nbrMatrix, 0, NBR_FLAG_GW);
            else
                nbrRowClearFlag(nbrMatrix, 0, NBR_FLAG_GW);   // F6: --gateway off

            // Welle 4: Kantenminute des letzten Hops vor dem Rahmen (Live-MH).
            NbrMhEdgeMark hn_edge_before = nbrMhEdgeBefore(aprsmsg.msg_source_last);

            nbrNoteFrameGw(nbrMatrix, aprsmsg.msg_source_path, aprsmsg.payload_type,
                           aprsmsg.msg_payload,
                           nbrGwHintFromDest(aprsmsg.payload_type, aprsmsg.msg_destination_path),
                           rssi, snr, now_min_hn);

            // W3b: Direkt-Slot fuer den HN-Bericht selbst (Konzept 4.6) --
            // HN traegt nie eine Position (payload_type ist immer '@'), also
            // ohne has_pos. Wie beim regulaeren Zweig unten: kein eigenes Echo.
            if(!is_equ(aprsmsg.msg_source_last, meshcom_settings.node_call))
            {
                NbrDirectInfo hn_direct_info = nbrBuildDirectInfo(aprsmsg, rssi);
                nbrNoteDirect(nbrMatrix, aprsmsg.msg_source_last, hn_direct_info, now_min_hn);
                nbrMhLiveAfter(aprsmsg.msg_source_last, hn_edge_before, now_min_hn);
            }

            nbrNoteReport(nbrMatrix, aprsmsg.msg_source_call, aprsmsg.msg_payload, now_min_hn);

            topoUiChanged(now_min_hn);

#if defined BOARD_RAK4630
            taskENTER_CRITICAL();
            rxBufInUse[rxBufIndex] = false;
            taskEXIT_CRITICAL();
#endif
            is_receiving = false;

            if(bLORADEBUG)
                printfdeb("[MC-DBG] ONRXDONE_TIME ms=%lu\n", millis() - _onrxdone_start);

            iReceiveTimeOutTime = millis();
            csma_timeout = csma_compute_timeout(cad_attempt);

            test_inject_service();

            return;
        }

        // Nachbarschaftsmatrix Stufe 2 (docs/nbr-wichtigkeit-konzept.md 5.2):
        // fremde Wiederholung erkannt -- ein GEHOERTER Frame mit mindestens
        // zwei Pfad-Token, dessen letzter Hop nicht ich selbst bin, kann den
        // Bedarf eines schon eingereihten eigenen Relays desselben Frames
        // decken. msg_type_b_lora != 0 schliesst einen fehlgeschlagenen
        // decodeAPRS() aus (aprsmsg waere dann nicht verlaesslich befuellt);
        // der Absender-Retry (ein einzelnes Pfad-Token) und das eigene Echo
        // (letzter Hop == ich) duerfen diesen Zweig nie erreichen.
        if(bNBRRELAY && msg_type_b_lora != 0 &&
           (aprsmsg.payload_type == ':' || aprsmsg.payload_type == '!' || aprsmsg.payload_type == '@') &&
           strchr(aprsmsg.msg_source_path, ',') != NULL &&
           !is_equ(aprsmsg.msg_source_last, meshcom_settings.node_call))
        {
            uint16_t now_min_cover = uptimeMin16();
            // Nur ein Vorabtest, ob der Relayer ueberhaupt etwas decken
            // koennte (relevant=0, msg_id=0 -- das unterdrueckt jede
            // SYM-COVER-Zeile hier, die eigentliche, slotgenaue Maske
            // rechnet die Schleife unten je Slot neu).
            // F7: nbrCopyMask() = listeners of the relayer PLUS the relayer
            // itself -- it transmitted this copy, so it has the frame.
            NbrMask cover_gate = nbrCopyMask(nbrMatrix, aprsmsg.msg_source_last, now_min_cover,
                                              bNBRSYM, nbrMaskNone(), 0, NULL);

            // Relayer unbekannt (keine Zeile/keine frischen Hoerer) -> nichts
            // zu entscheiden (Konzept 5.2).
            if(!nbrMaskEmpty(cover_gate))
            {
                char nbr_typ = (aprsmsg.payload_type == ':') ? 'T' :
                               (aprsmsg.payload_type == '!') ? 'P' : 'H';

                for(int nbr_i = 0; nbr_i < MAX_RING; nbr_i++)
                {
                    // ringBuffer[i][1] == RING_STATUS_DONE schliesst
                    // RING_STATUS_EXT_PENDING (0x80) und die SENT-Alterung
                    // (0x01..0x14) bereits per Wertevergleich aus -- ein
                    // Relay-Slot laeuft nie ueber den External-Radio-
                    // Bridge-Pfad (der setzt EXT_PENDING nur fuer eigene
                    // Sends). Gleicher msg_id-Vergleich wie der bestehende
                    // ACK-Scan weiter oben.
                    if(ringBuffer[nbr_i][0] == 0 ||
                       (ringKind[nbr_i] & 0x7F) != RING_KIND_RELAY ||
                       ringBuffer[nbr_i][1] != RING_STATUS_DONE ||
                       memcmp(ringBuffer[nbr_i]+3, RcvBuffer+1, 4) != 0)
                        continue;

                    uint32_t nbr_mid = extractRingMsgId(nbr_i);

                    if(!nbrMaskEmpty(ringAlone[nbr_i]))
                    {
                        // Fall A: Sole-Provider-Veto, nie Abbruch (Konzept 5.2).
                        if(!(ringKind[nbr_i] & RING_KIND_COUNTED))
                        {
                            stat_nbr_refuse_alone++;
                            ringKind[nbr_i] |= RING_KIND_COUNTED;

                            if(nbrLog != NULL)
                            {
                                char alone_hex[NBR_MASK_HEX_LEN + 1];
                                nbrMaskHex(ringAlone[nbr_i], alone_hex, sizeof(alone_hex));
                                char nbr_line[64 + (NBR_MASK_HEX_LEN + 1)];
                                snprintf(nbr_line, sizeof(nbr_line),
                                         "[NBR]|REFUSE|%u|%08X|%c|%s|%s",
                                         (unsigned)now_min_cover, (unsigned)nbr_mid, nbr_typ,
                                         aprsmsg.msg_source_last, alone_hex);
                                nbrLog(nbr_line);
                            }
                        }
                    }
                    else
                    {
                        // Slotgenau neu gerechnet (relevant = der Bedarf
                        // GENAU dieses Slots, VOR dem Abzug): eine per
                        // Symmetrie hinzugefuegte Hoererin loggt hier nur,
                        // wenn ihr Bit auch in diesem Bedarf steht -- sonst
                        // waere die Annahme fuer diesen Slot folgenlos.
                        // F4-1: nbrCopyMask() kann loggen (SYM-COVER) und damit
                        // auf den Loop-Task umschalten; dort kann ein voller
                        // Ring den Slot raeumen und den Eintrag am Lesezeiger
                        // hineinziehen (N-24, txring_functions.cpp). Maske also
                        // zuerst ausserhalb des Locks rechnen, dann unter
                        // demselben Lock wie die Ring-Mutatoren den Slot
                        // erneut pruefen (gleiches Relay, gleicher Bedarf) und
                        // nur dann Bedarf abziehen / freigeben. Stimmt der Slot
                        // nicht mehr, entfaellt dieser Abzug (verpasster Abbruch,
                        // kein Fehlabbruch). Geloggt wird nach dem Lock.
                        NbrMask slot_inferred = nbrMaskNone();
                        NbrMask nbr_before = ringNeed[nbr_i];
                        NbrMask cover = nbrCopyMask(nbrMatrix, aprsmsg.msg_source_last, now_min_cover,
                                                     bNBRSYM, nbr_before, nbr_mid, &slot_inferred);

                        NbrMask nbr_after = nbrMaskAndNot(nbr_before, cover);
                        // Bits, die dieser Abzug tatsaechlich entfernt hat
                        // UND die nur per Symmetrie-Annahme dazukamen --
                        // trailing Feld fuer CANCEL/CANCEL?.
                        NbrMask nbr_removed_inferred = nbrMaskAnd(nbrMaskAndNot(nbr_before, nbr_after), slot_inferred);

                        bool nbr_do_cancel = false;
                        bool nbr_do_possible = false;
#if defined(BOARD_RAK4630)
                        taskENTER_CRITICAL();
#endif
                        if(ringBuffer[nbr_i][0] != 0 &&
                           (ringKind[nbr_i] & 0x7F) == RING_KIND_RELAY &&
                           ringBuffer[nbr_i][1] == RING_STATUS_DONE &&
                           memcmp(ringBuffer[nbr_i]+3, RcvBuffer+1, 4) == 0 &&
                           extractRingMsgId(nbr_i) == nbr_mid &&
                           nbrMaskEmpty(ringAlone[nbr_i]) &&
                           nbrMaskEqual(ringNeed[nbr_i], nbr_before))
                        {
                            ringNeed[nbr_i] = nbr_after;

                            if(nbrMaskEmpty(nbr_after))
                            {
                                if(bNBRCANCEL)
                                {
                                    // Slot freigeben mit denselben drei Schreibzugriffen
                                    // wie die ACK-Freigabe weiter oben -- aber auf einen
                                    // DONE-Slot, den getNextTxSlot() waehlen kann (die
                                    // ACK-Freigabe trifft nur SENT-Slots). Auf nRF52 kann
                                    // doTX() (Loop-Task) den Slot zwischen Auswahl und
                                    // Verbrauch verlieren: ein leerer Sendeversuch, oder
                                    // ein Frame, der trotz CANCEL-Zeile noch rausgeht.
                                    // Dasselbe Fenster hat der N-24-Umzug schon heute
                                    // (Advisor 2026-09-22, Befund 2, akzeptiert).
                                    ringBuffer[nbr_i][1] = RING_STATUS_DONE;
                                    retryCount[nbr_i] = 0;
                                    ringBuffer[nbr_i][0] = 0;
                                    stat_nbr_cancel++;
                                    nbr_do_cancel = true;
                                }
                                else if(!(ringKind[nbr_i] & RING_KIND_COUNTED))
                                {
                                    // Zaehlmodus (--nbrrelay count, Verdict M1):
                                    // dieselbe Rechnung, aber ohne Wirkung.
                                    stat_nbr_cancel_possible++;
                                    ringKind[nbr_i] |= RING_KIND_COUNTED;
                                    nbr_do_possible = true;
                                }
                            }
                        }
#if defined(BOARD_RAK4630)
                        taskEXIT_CRITICAL();
#endif

                        if((nbr_do_cancel || nbr_do_possible) && nbrLog != NULL)
                        {
                            char before_hex[NBR_MASK_HEX_LEN + 1];
                            char after_hex[NBR_MASK_HEX_LEN + 1];
                            char removed_hex[NBR_MASK_HEX_LEN + 1];
                            nbrMaskHex(nbr_before, before_hex, sizeof(before_hex));
                            nbrMaskHex(nbr_after, after_hex, sizeof(after_hex));
                            nbrMaskHex(nbr_removed_inferred, removed_hex, sizeof(removed_hex));
                            char nbr_line[64 + 3 * (NBR_MASK_HEX_LEN + 1)];
                            snprintf(nbr_line, sizeof(nbr_line),
                                     "[NBR]|%s|%u|%08X|%c|%s|%s|%s|%s",
                                     nbr_do_cancel ? "CANCEL" : "CANCEL?",
                                     (unsigned)now_min_cover, (unsigned)nbr_mid, nbr_typ,
                                     aprsmsg.msg_source_last, before_hex, after_hex,
                                     removed_hex);
                            nbrLog(nbr_line);
                        }
                    }
                }
            }
        }

        if(msg_type_b_lora == 0x00)
        {
            if(bDisplayCont)
                printfdeb("[LORA-ERROR]...%03i RCV:%s\n", size, RcvBuffer+6);
        }
        else if(isUnconfiguredCall(aprsmsg.msg_source_call))
        {
            // RX-01 (BACKLOG 3.8k): a node still on the factory callsign is
            // not identifying itself, so nothing it sends is legal to
            // relay -- drop it here, before the topology feed, display, phone/BLE out,
            // the gateway upload and the relay decision below.
            logRxDropUnconfigured(aprsmsg.msg_source_call);

            // SL-02: Ausstieg vor dem Relay-Block, gleiche Aussage "nicht
            // weitergesendet, Grund unconf". Nur fuer neue Frames.
            if(bDisplayLog && !rx_dup)
            {
                setlogFormatRly(setlog_buf, sizeof(setlog_buf), aprsmsg.msg_id,
                                aprsmsg.payload_type, aprsmsg.max_hop & 0x0F, "unconf", 0, -1);
                setlogPrint(setlog_buf);
            }
        }
        else
        {
            // NBR (Konzept ~/Desktop/Nachbarschaftsmatrix.html 4.1/4.4,
            // nbr_matrix.h): Haken ganz vorn in diesem Zweig, VOR Dedup-
            // Pruefung, Raw-Ring-Schreiben und dem Last-Hop-Check unten --
            // Duplikate sind laut 4.1 die Hauptquelle fuer Hoerbeweise (erst
            // die zweite Kopie eines Frames zeigt, dass ein zweiter Nachbar
            // den Absender hoert), und auch das eigene Echo traegt noch den
            // Pfad, der "wer hoert wen" beantwortet.
            if(aprsmsg.payload_type == ':' || aprsmsg.payload_type == '!' || aprsmsg.payload_type == '@')
            {
                // Contract mit der Web-Seite (W3): Minuten seit Boot, nicht
                // die Wanduhr (die steht nach einem Kaltstart auf 1970).
                uint16_t now_min = uptimeMin16();

                // Lazy Init deckt Boot UND ein Laufzeit-"--setcall" gleich mit
                // ab -- kein zusaetzlicher Haken in setup() oder im Settings-
                // Kommando noetig.
                if(!nbrOwnCallIs(nbrMatrix, meshcom_settings.node_call))
                    nbrInit(nbrMatrix, meshcom_settings.node_call, now_min);

                // Zeile 0 bekommt nie einen fremden POS-Frame: eigene Position
                // und eigene Flags (GW, Mesh) kommen aus den Settings, sonst
                // bleibt die Reichweite der eigenen Zeile immer leer (Bench
                // 2026-09-20, erste Seite nach dem Flash).
                if(!nbrRowHasFlag(nbrMatrix, 0, NBR_FLAG_POS) && meshcom_settings.node_lat != 0.0)
                    nbrNotePos(nbrMatrix, meshcom_settings.node_call, (float)meshcom_settings.node_lat,
                               (float)meshcom_settings.node_lon, bMESH, 0, now_min);
                if(bGATEWAY)
                    nbrRowSetFlag(nbrMatrix, 0, NBR_FLAG_GW);
                else
                    nbrRowClearFlag(nbrMatrix, 0, NBR_FLAG_GW);   // F6: --gateway off

                // Trefferzahl (frueher "[NBR] hits=%d path=%s" hinter bLORADEBUG)
                // ist im neuen EDGE/ME/CUT/DROP-Format (docs/nbr-logformat.md,
                // Vertrag der Nachbarschaftsmatrix) nicht mehr vorgesehen -- die
                // Zeilen dort tragen die gleiche Information pro Hoerbeziehung.
                //
                // Welle 4: Kantenminute des letzten Hops vor dem Rahmen (Live-MH).
                NbrMhEdgeMark edge_before = nbrMhEdgeBefore(aprsmsg.msg_source_last);

                nbrNoteFrameGw(nbrMatrix, aprsmsg.msg_source_path, aprsmsg.payload_type,
                               aprsmsg.msg_payload,
                               nbrGwHintFromDest(aprsmsg.payload_type, aprsmsg.msg_destination_path),
                               rssi, snr, now_min);

                // W3b (docs/meshcom5-campaign.md Welle 3): Direkt-Slot-Grundgeruest
                // fuer denselben Rahmen. Position/Hoehe kommen erst unten aus dem
                // '!'-Zweig hinzu (nur own_frame); nbrNoteDirect() selbst steht
                // ganz am Ende dieses Blocks, NACH dem eigenen Echo-Test.
                NbrDirectInfo direct_info = nbrBuildDirectInfo(aprsmsg, rssi);

                // Relayte POS-Frames (Konzept 4.4): der Direkt-Slot traegt die
                // Position nur bei Direktempfang (msg_source_call ==
                // msg_source_last), die Zeile braucht sie aber unabhaengig
                // vom letzten Hop, sonst bleiben die Nachbarn hinter einem
                // Relais positionslos. msg_source_hw ist -- anders als
                // msg_last_hw -- bereits der Absender-Wert ohne Last-Hop-Bit,
                // also ohne Maskierung uebernehmbar.
                if(aprsmsg.payload_type == '!')
                {
                    struct aprsPosition aprspos;

                    if(decodeAPRSPOS(aprsmsg.msg_payload, aprspos) == 0x01)
                    {
                        float nbr_lat = (float)conv_coord_to_dec(aprspos.lat);
                        if(aprspos.lat_c == 'S')
                            nbr_lat = nbr_lat * -1.0f;

                        float nbr_lon = (float)conv_coord_to_dec(aprspos.lon);
                        if(aprspos.lon_c == 'W')
                            nbr_lon = nbr_lon * -1.0f;

                        nbrNotePos(nbrMatrix, aprsmsg.msg_source_call, nbr_lat, nbr_lon, aprsmsg.msg_mesh,
                                   aprsmsg.msg_source_hw, now_min);

                        // W3b: Position/Hoehe im Direkt-Slot NUR aus eigenen
                        // Positionsrahmen (Konzept 4.6), Hoehenumrechnung exakt
                        // wie frueher im MHeard-Block (fw_version > 13 ->
                        // Fuss->Meter).
                        if(direct_info.own_frame)
                        {
                            direct_info.has_pos = true;
                            direct_info.lat = nbr_lat;
                            direct_info.lon = nbr_lon;

                            int nbr_alt_m = aprspos.alt;
                            if(aprsmsg.msg_source_fw_version > 13)
                                nbr_alt_m = (int)((float)nbr_alt_m * 0.3048);
                            direct_info.alt_m = nbr_alt_m;
                        }

                        // W3b: gemeldete Nachbarzahl aus einem '!'-Rahmen (aprspos.ncnt,
                        // /N<k>) -- fuer JEDE Zeile mit einer gueltig dekodierten Position,
                        // nicht nur bei own_frame (wie frueher das MHeard in seinem
                        // own_frame- und seinem relayten Zweig).
                        if(aprspos.ncnt > 0)
                            nbrNoteNcnt(nbrMatrix, aprsmsg.msg_source_call, aprspos.ncnt, now_min);
                    }
                }

                if(!is_equ(aprsmsg.msg_source_last, meshcom_settings.node_call))
                {
                    // W3b: "R<n>" des HEY-Berichts ist die gemeldete Nachbarzahl
                    // des ABSENDERS (bis Welle 4 trug das MHeard sie als
                    // mh_ncount). Vor nbrNoteDirect(), damit ein Live-MH-Rahmen
                    // schon den neuen NCNT traegt.
                    long nbr_hey_ncnt = 0;
                    if(aprsmsg.payload_type == '@' && nbrParseHeyReportedCount(aprsmsg.msg_payload, &nbr_hey_ncnt))
                        nbrNoteNcnt(nbrMatrix, aprsmsg.msg_source_call, (int)nbr_hey_ncnt, now_min);

                    nbrNoteDirect(nbrMatrix, aprsmsg.msg_source_last, direct_info, now_min);
                    nbrMhLiveAfter(aprsmsg.msg_source_last, edge_before, now_min);
                }

                // Welle 4: ersetzt die T-Deck-/T-Deck-Pro-Haken aus
                // updateMheard()/updateHeyPath() (Anzeige, /topo.dat).
                topoUiChanged(now_min);
            }

            // LoRx RX to RAW-Buffer
            //memcpy(ringbufferRAWLoraRX[RAWLoRaWrite], charBuffer_aprs((char*)"", aprsmsg).c_str(), UDP_TX_BUF_SIZE-1);
            charBuffer_aprs(aprsmsg);

            addRingPointer(RAWLoRaWrite, RAWLoRaRead, MAX_LOG, "raw_rx");
            
            //printfdeb("LOG Write next: %i read next:%i\n", RAWLoRaWrite, RAWLoRaRead);

            /*
            RAWLoRaWrite++;
            if(RAWLoRaWrite >= MAX_LOG)
                RAWLoRaWrite=0;
            */

            //printfdeb("1:msg_source_last:%s node_call:%s\n", aprsmsg.msg_source_last.c_str(), meshcom_settings.node_call);

            if(!is_equ(aprsmsg.msg_source_last, meshcom_settings.node_call))
            {
                // print aprs message
                if(bDisplayInfo)
                {
                    printBuffer_aprs((char*)"MH-LoRa", aprsmsg);
                    printfdeb("\n");
                    bNewLine=true;
                }

#if defined(ENABLE_MSGSTORE)
                // S3: presence hook -- a frame heard directly from its
                // originator (no relay hop in the path) arms any HELD
                // mailbox entry addressed to that call. F2
                // (docs/review/fable-dm-stage3-verdict-20260914.md): no
                // server-flag guard -- a gateway that relayed or emitted
                // this frame already appended its own call to the path,
                // which fails the comma test below on its own.
                if(strchr(aprsmsg.msg_source_path, ',') == NULL)
                    msgstorePresence(aprsmsg.msg_source_call);
#endif

                // last heard LoRa MeshCom-Packet
                lastHeardTime = millis();

            }

            //
            ///////////////////////////////////////////////

            // PN-Wiederholung (XOR, pn_retry.h): eine eigene Retry-Kopie
            // traegt eine andere msg-id als checkOwnTx() (icheck) kennt --
            // HEARD trotzdem unter der URSPRUENGLICHEN id buchen, im selben
            // Block (kein zweiter Eintrag). Das Dedup-Verdikt bleibt
            // unveraendert: rx_is_new ist fuer diese Kopie bereits false
            // (eigene addLoraRxBuffer()-Registrierung beim Senden, siehe
            // updateRetransmissionStatus()), der else-Zweig unten bliebe
            // also so oder so zu.
            int heardIcheck = icheck;
            uint32_t heardMsgId = aprsmsg.msg_id;

            if(heardIcheck < 0 && rx_pn_shape && pnIsOwnNodeId(aprsmsg.msg_id, _GW_ID))
            {
                uint32_t pnOrigId = pnRetryId(aprsmsg.msg_id, _GW_ID, 0);
                int pnOrigCheck = checkOwnTx(pnOrigId);
                if(pnOrigCheck >= 0)
                {
                    heardIcheck = pnOrigCheck;
                    heardMsgId = pnOrigId;
                    setlogCountDedup(rx_is_new);    // SL-01: Zaehler wie im else-Zweig
                }
            }

            if(heardIcheck >= 0) // own msg_id (direkt oder PN-Retry-Echo)
            {
                // Status frame to the phone is origin-gated: own_msg_id[] also holds
                // foreign msg_ids that a gateway only forwarded from the server to LoRa
                // (see docs/ack-heard-foreign-msgids-fix.md). The state write below stays
                // unconditional so the web rxlog heard/ACK ticks keep working as today.
                if(msg_type_b_lora == MSG_TYPE_TEXT && (bAckInfo || own_msg_id[heardIcheck][4] == 0x00))   // 00...not heard, 01...heard, 02...ACK, 03...failed, 04...held
                {
                    if(ackMsgIdFromNode(heardMsgId, _GW_ID))
                    {
                        uint16_t plen = buildAckPhoneFrame(print_buff, heardMsgId, 0x00, aprsmsg.msg_source_last);

                        addBLEOutBuffer(print_buff, plen);

                        if(bDisplayInfo)
                        {
                            printfdeb("%s", getTimeString().c_str());
                            printfdeb(" HEARD from <%s> to Phone  %02X %02X%02X%02X%02X %02X %02X\n", aprsmsg.msg_source_path, print_buff[0], print_buff[4], print_buff[3], print_buff[2], print_buff[1], print_buff[5], print_buff[6]);
                            bNewLine=true;
                        }
                    }

                    // dmstat_echo: own_msg_id[] carries no destination, so a
                    // broadcast/group text heard back counts here too -- see
                    // the report for why that split is not cheaply knowable
                    // at this site.
                    if(own_msg_id[heardIcheck][4] == 0x00 &&
                       msg_type_b_lora == MSG_TYPE_TEXT &&
                       mcIndexOfStrFrom(aprsmsg.msg_payload, "{", 1) > 0 &&
                       mcIndexOfStr(aprsmsg.msg_payload, ":ack") <= 0)
                        dmstat_echo.fetch_add(1);

                    // 0x02 (acked), 0x03 (failed, 0.3) and 0x04 (held, S4) are
                    // final/latched: a late relay echo must not turn them
                    // back into "heard" -- a store node holding this DM would
                    // otherwise be downgraded by the very echo that proves
                    // the mesh still relays it.
                    if(own_msg_id[heardIcheck][4] != 0x02 && own_msg_id[heardIcheck][4] != 0x03 && own_msg_id[heardIcheck][4] != 0x04)
                        own_msg_id[heardIcheck][4]=0x01; // 0x01 HEARD
                }
            }
            else
            if(setlogCountDedup(rx_is_new))    // SL-01: Verdikt von oben, Logik unveraendert
            {
                // :|0x11223344|0x05|OE1KBC|>*:Hallo Mike, ich versuche eine APRS Meldung\0x00
                if(bDisplayCont)
                {
                    switch (msg_type_b_lora)
                    {

                        case MSG_TYPE_TEXT: DEBUG_MSG("RADIO", "Received Textmessage"); break;
                        case MSG_TYPE_POSITION: DEBUG_MSG("RADIO", "Received PosInfo"); break;
                        case MSG_TYPE_HEY: DEBUG_MSG("RADIO", "Received Hey"); break;
                        default:
                            DEBUG_MSG("RADIO", "Received unknown");
                            if(bDEBUG)
                                printBuffer(RcvBuffer, size);
                            break;
                    }
                }

                // Trickle-HEY: count received HEY for consistency check
                if(msg_type_b_lora == MSG_TYPE_HEY)
                    trickle_consistent_count++;

                // txtmessage, position, hey
                if(msg_type_b_lora == MSG_TYPE_TEXT || msg_type_b_lora == MSG_TYPE_POSITION || msg_type_b_lora == MSG_TYPE_HEY)
                {
                    // Extern Server (deferred — avoid blocking UDP in radio callback)
                    // PN retry: a repeat copy of a PN we already relayed/saw
                    // must not re-upload to the extern server / KISS -- the
                    // relay decision and addLoraRxBuffer() below still run.
                    if(bEXTUDP && !rx_pn_repeat)
                        queueExtern((char*)"lora", RcvBuffer, size, rssi, snr);

                    // KISS/TCP interface (deferred — same reason). HEY frames are
                    // not representable as AX.25 (buildAx25 discards them) — don't
                    // let them evict text/position from the 2-slot queue.
                    #if defined(ESP32) && !defined(DISABLE_KISS_TCP)
                    if(bKISS && msg_type_b_lora != MSG_TYPE_HEY && !rx_pn_repeat)
                        queueKiss(RcvBuffer, size, rssi, snr);
                    #endif

                    // print aprs message
                    if(bDisplayInfo)
                    {
                        printBuffer_aprs((char*)"RX-LoRa2", aprsmsg);
                        bNewLine=true;
                    }
            
                    // we add now Longname (up to 20), ID - 4, RSSI - 2, SNR - 1 and MODE BYTE - 1
                    // MODE BYTE: LongSlow = 1, MediumSlow = 3
                    // and send the UDP packet (done in the method)

                    // we only send the packet via UDP if we have no collision with UDP rx
                    // und wenn MSG nicht von einem anderen Gateway empfangen wurde welches es bereits vopm Server bekommen hat

                    int lora_msg_len = size; // size ist uint16_t !
                    if (lora_msg_len > UDP_TX_BUF_SIZE)
                    lora_msg_len = UDP_TX_BUF_SIZE; // zur Sicherheit

                    //if(bDEBUG)
                    //    printf("Check-Msg src:%s msg_id: %04X msg_len: %i payload[5]=%i via=%d\n", aprsmsg.msg_source_path.c_str(), aprsmsg.msg_id, lora_msg_len, aprsmsg.max_hop, aprsmsg.msg_server);

                    // Wiederaussendung via LORA
                    // Ringbuffer filling

                    if ((msg_type_b_lora == MSG_TYPE_TEXT || msg_type_b_lora == MSG_TYPE_POSITION || msg_type_b_lora == MSG_TYPE_HEY))
                    {
                        // add RXMsg-ID to ringbuffer
                        addLoraRxBuffer(aprsmsg.msg_id, aprsmsg.msg_server);

                        // add rcvMsg to BLE out Buff
                        // size message is int -> uint16_t buffer size

                        // destination_path without path
                        // *
                        // 99999
                        // XX0XXX-99
                        //
                        // destination_path with path
                        // AA0AAA-99,...,*
                        // AA0AAA-99,...,99999
                        // AA0AAA-99,...,XX0XXX-99

                        char destination_call[MC_CALL_LEN_Z];   // R2-04: war 20, siehe udp_frame_esp32.cpp
                        snprintf(destination_call, sizeof(destination_call), "%s", aprsmsg.msg_destination_call);

                        bool bMeshDestination = true;

                        // SL-02: Grund an den Ausstiegen setzen, eine RLY-Zeile
                        // am Blockende. rly_hop ist der Hop WIE EMPFANGEN
                        // (Relay dekrementiert aprsmsg.max_hop weiter unten).
                        const char *rly_reason = NULL;
                        int rly_prio = 0;
                        int rly_slot = -1;
                        uint8_t rly_hop = aprsmsg.max_hop & 0x0F;

                        // Nachbarschaftsmatrix Stufe 2 (docs/nbr-wichtigkeit-konzept.md
                        // 5.1): Bedarfs-/Allein-Maske des Relays, mit Initialisierer
                        // deklariert VOR jedem goto in diesem Block (skip_relay unten
                        // springt sonst ueber die Initialisierung hinweg -- C++ verbietet
                        // das). Zugewiesen (nicht neu deklariert) kurz vor bSHORTPATH.
                        NbrNeed nn_relay = {nbrMaskNone(), nbrMaskNone(), false, nbrMaskNone()};
                        uint16_t now_min_relay = 0;

                        if(msg_type_b_lora == MSG_TYPE_TEXT)    // text message store&forward
                        {
                            if(strcmp(destination_call, meshcom_settings.node_call) == 0)
                            {
                                ///////////////////////////////////////////////////////////////
                                // check ping
                                if(mcStartsWith(aprsmsg.msg_payload, "{ping}"))
                                {
                                    if(bDisplayInfo)
                                    {
                                        printfdeb("\n");
                                        printfdeb("%s", getTimeString().c_str());
                                        printfdeb("[PING] from:%s to:%s via:%s\n", aprsmsg.msg_source_call, aprsmsg.msg_destination_call, aprsmsg.msg_source_path);
                                        bNewLine=true;
                                    }

                                    SendPong(aprsmsg.msg_source_call, aprsmsg.msg_id);

                                }
                                ///////////////////////////////////////////////////////////////
                                else
                                ///////////////////////////////////////////////////////////////
                                // check pong
                                if(mcStartsWith(aprsmsg.msg_payload, "{pong}"))
                                {
                                    if(bDisplayInfo)
                                    {
                                        printfdeb("\n");
                                        printfdeb("%s", getTimeString().c_str());
                                        printfdeb("[PONG] from:%s to:%s via:%s\n", aprsmsg.msg_source_call, aprsmsg.msg_destination_call, aprsmsg.msg_source_path);
                                        bNewLine=true;
                                    }

                                    queueDisplayText(aprsmsg, rssi, snr);

                                    // P13: das Pong geht auch an den BLE-Client, wie jede andere
                                    // DM an uns (else-Zweig unten). Sonst sieht ein ueber BLE
                                    // gesendetes {ping} (App, McApp) nie eine Antwort. Roh
                                    // weitergereicht: {pong}{<id>} ordnet es dem Ping zu.
                                    addBLEOutBuffer(RcvBuffer, size);

                                    bPingSend = false;

                                }
                                ///////////////////////////////////////////////////////////////
                                else
                                {

                                    int iAckPos=mcIndexOfStr(aprsmsg.msg_payload, ":ack");
                                    int iEnqPos=mcIndexOfStrFrom(aprsmsg.msg_payload, "{", 1);
                                    uint16_t stoNnn=0;   // stage 4: :sto custody notice NNN

                                    if(iAckPos > 0 || mcIndexOfStr(aprsmsg.msg_payload, ":rej") > 0)
                                    {
                                        //
                                        // next sequence only to mark a massage to node_call with ACK
                                        //
                                        unsigned int iAckId = (unsigned int)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iAckPos+4), strlen(aprsmsg.msg_payload));
                                        msg_counter = ((_GW_ID & 0x3FFFFF) << 10) | (iAckId & 0x3FF);

                                        uint16_t plen = buildAckPhoneFrame(print_buff, msg_counter, 0x02, aprsmsg.msg_source_call);

                                        if(bDisplayInfo)
                                        {
                                            printfdeb("\n");
                                            printfdeb("%s", getTimeString().c_str());
                                            printfdeb("[ACK-MSGID] ack_msg_id:%02X%02X%02X%02X\n", print_buff[4], print_buff[3], print_buff[2], print_buff[1]);
                                            bNewLine=true;
                                        }
                                
                                        int iackcheck = checkOwnTx(msg_counter);
                                        if(iackcheck >= 0)
                                        {
                                            own_msg_id[iackcheck][4] = 0x02;   // 02...ACK

                                            // S4: the destination's own ack is the final word --
                                            // forget any store node(s) that were holding this DM.
                                            stoHolderClear(msg_counter);

                                            // 0.3/0.4: peer ACK for an own DM, plus the RTT sample
                                            // for the send-to-ack histogram (M0-1).
                                            // F1: count only the first ACK per own DM
                                            // (the server path may have delivered it already).
                                            // DM-17: a :rej is not an ack -- count only on :ack.
                                            if(iAckPos > 0 && dmStatNoteAck((uint16_t)(iAckId & 0x3FF), millis()))
                                                dmstat_peer_ack.fetch_add(1);

                                            // BUG #8 fix: clear ringBuffer entry to stop retransmission.
                                            // findAndStopRingSlot() compares pnRetryCore(), so this also
                                            // matches a still-queued XOR-retry copy of this msg_id.
                                            int dmSlot = findAndStopRingSlot(msg_counter);
                                            if(dmSlot >= 0 && bDisplayRetx)
                                                printfdeb("\n[RETX] DM-ACK for retid:%i stop retransmit msg-id:%08X\n",
                                                            dmSlot, msg_counter);
                                        }

                                        addBLEOutBuffer(print_buff, plen);
                                    }
                                    else
                                    if(stoNoticeParse(aprsmsg.msg_payload, &stoNnn, NULL))
                                    {
                                        // S4: a store node told us (the original sender) it took
                                        // this DM into custody -- mark it HELD unless it already
                                        // reached a final state (0x02 ack, 0x03 failed); 0x00/0x01/
                                        // 0x04 may still be upgraded/refreshed here.
                                        msg_counter = ((_GW_ID & 0x3FFFFF) << 10) | (stoNnn & 0x3FF);

                                        int iStoCheck = checkOwnTx(msg_counter);

                                        if(iStoCheck >= 0 &&
                                           (own_msg_id[iStoCheck][4] == 0x00 || own_msg_id[iStoCheck][4] == 0x01 || own_msg_id[iStoCheck][4] == 0x04) &&
                                           stoHolderNote(msg_counter, aprsmsg.msg_source_call, stoNnn, millis()))
                                        {
                                            own_msg_id[iStoCheck][4] = 0x04;   // 04...HELD

                                            uint16_t stoPlen = buildAckPhoneFrame(print_buff, msg_counter, ACK_STATUS_HELD, aprsmsg.msg_source_call);
                                            addBLEOutBuffer(print_buff, stoPlen);

                                            if(bDisplayInfo)
                                            {
                                                printfdeb("\n");
                                                printfdeb("%s", getTimeString().c_str());
                                                printfdeb("[HELD] by %s nnn:%03u\n", aprsmsg.msg_source_call, (unsigned)stoNnn);
                                                bNewLine=true;
                                            }
                                        }
                                        // 0x02 (acked) and 0x03 (failed, 0.3) are final states and are
                                        // never downgraded back to held; stoHolderNote()'s per-hour rate
                                        // limit keeps a replayed notice from repeating the phone frame.
                                    }
                                    else
                                    if(iEnqPos > 0)
                                    {
                                        //
                                        // next sequence only reply to a DM-Message
                                        //
                                        unsigned int iAckId = (unsigned int)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iEnqPos+1), strlen(aprsmsg.msg_payload));

                                        if(bDisplayInfo && !bNewLine)
                                        {
                                            printfdeb("\n");
                                            bNewLine=true;
                                        }

                                        // 2.1: second dedup layer, keyed on (source call, NNN).
                                        // Stripped payload computed here, before the mutating
                                        // mcTruncate() below.
                                        char strippedPayload[MC_PAYLOAD_LEN];
                                        mcSet(strippedPayload, sizeof(strippedPayload), aprsmsg.msg_payload);
                                        mcTruncate(strippedPayload, sizeof(strippedPayload), (size_t)iEnqPos);

                                        if(dmDedupCheck(aprsmsg.msg_source_call, (uint16_t)iAckId,
                                                        strippedPayload, strlen(strippedPayload),
                                                        millis()) == DM_DEDUP_DUP)
                                        {
                                            // Duplicate by (call, NNN, payload): re-ack, rate
                                            // limited, but do not display or forward again --
                                            // mheard and the msg_id ring already saw this frame.
                                            if(reackAllowed(aprsmsg.msg_source_call, (uint16_t)iAckId, millis()))
                                            {
                                                SendAckMessage(aprsmsg.msg_source_call, iAckId);
                                                dmstat_reack.fetch_add(1);
                                            }
                                            else
                                                dmstat_reack_limited.fetch_add(1);

                                            if(bDisplayInfo)
                                                printfdeb("[DMDUP] from %s nnn:%03u\n", aprsmsg.msg_source_call, (unsigned)iAckId);
                                        }
                                        else
                                        {
                                            // 0.2 (4a569e6f rework): seed the re-ACK limiter with
                                            // the original ack, so the relayed copy of this DM (a
                                            // duplicate seconds from now) is not acked twice; the
                                            // sender's 40 s retry still is.
                                            reackAllowed(aprsmsg.msg_source_call, (uint16_t)iAckId, millis());
                                            SendAckMessage(aprsmsg.msg_source_call, iAckId);

                                            mcSet(aprsmsg.msg_payload, sizeof(aprsmsg.msg_payload), strippedPayload);

                                            uint8_t tempRcvBuffer[255];

                                            uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                                            queueDisplayText(aprsmsg, rssi, snr);

                                            if(bDisplayVia)
                                                printfdeb("[MESHx]...SRC-PATH:%s ... DST-PATH:%s TEXT:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.msg_payload);


                                            addBLEOutBuffer(tempRcvBuffer, tempsize);
                                        }
                                    }
                                    else
                                    {
                                        //
                                        // next sequence to send incomming DM-Message to Display and/or APP via BLE
                                        //
                                        queueDisplayText(aprsmsg, rssi, snr);

                                        if(bDisplayVia)
                                            printfdeb("[MESHx]...SRC-PATH:%s ... DST-PATH:%s TEXT:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.msg_payload);

                                        addBLEOutBuffer(RcvBuffer, size);
                                    }
                                }
                            }
                            else
                            {
#if defined(ENABLE_MSGSTORE)
                                // Destination is not us: a store node may need to purge an
                                // entry heard acked, hold a fresh DM for its store set, or
                                // cancel a pending delivery it heard a peer store node do
                                // first. All three run before the relay decision below, so
                                // a stored/purged DM is still relayed normally.
                                //
                                // F2 (docs/review/fable-dm-stage3-verdict-20260914.md): no
                                // server-flag guard here -- every frame in OnRxDone arrived
                                // over RF, and the 0x80 bit only records that a
                                // server-connected gateway touched the copy. Excluding those
                                // frames would exclude every gateway-relayed or
                                // app-originated DM, which is exactly the traffic a mailbox
                                // exists for.
                                //
                                // F6: a peer store node's own hop-0 delivery must never be
                                // re-stored here (that is what feeds msgstoreOnPeerDelivery()
                                // below) -- the tell is the path shape a store node's own
                                // delivery actually has: exactly two calls in msg_source_path
                                // (glueDeliver() appends itself to the sender's own single-call
                                // path), and the last one is not the frame's source.
                                int iMboxTagPos = mcIndexOfStrFrom(aprsmsg.msg_payload, "{", 1);
                                uint16_t mboxTagNnn = (iMboxTagPos > 0) ? (uint16_t)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iMboxTagPos + 1), strlen(aprsmsg.msg_payload)) : 0;
                                const char *pMboxComma1 = strchr(aprsmsg.msg_source_path, ',');
                                bool bMboxPathTwoCalls = (pMboxComma1 != NULL &&
                                                          pMboxComma1 > aprsmsg.msg_source_path &&
                                                          strchr(pMboxComma1 + 1, ',') == NULL);
                                bool bMboxPeerDelivery = (rly_hop == 0 &&
                                                          bMboxPathTwoCalls &&
                                                          strcmp(pMboxComma1 + 1, aprsmsg.msg_source_call) != 0 &&
                                                          iMboxTagPos > 0);

                                int iMboxAckPos = mcIndexOfStr(aprsmsg.msg_payload, ":ack");

                                if(iMboxAckPos > 0)
                                {
                                    // S3: purge hook -- :ackNNN heard for someone else's DM.
                                    uint16_t mboxAckNnn = (uint16_t)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iMboxAckPos + 4), strlen(aprsmsg.msg_payload));
                                    msgstoreOnAck(aprsmsg.msg_source_call, destination_call, mboxAckNnn);
                                }
                                else
                                if(strcmp(destination_call, "*") != 0 &&
                                   CheckGroup(destination_call) == 0 &&
                                   mcIndexOfStr(aprsmsg.msg_payload, ":rej") <= 0 &&
                                   !mcStartsWith(aprsmsg.msg_payload, "{") &&   // {ping}/{pong}/{MCP}/{SET}/{CET}: control frames, never a DM
                                   !bMboxPeerDelivery &&                        // F6: don't store a peer's own delivery frame
                                   !rx_pn_repeat &&                             // E2: a repeat XOR copy must not push stored_ms out again
                                   msgstoreEligible(destination_call))
                                {
                                    // S3: store hook -- a DM for our store set.
                                    int iMboxEnqPos = mcIndexOfStrFrom(aprsmsg.msg_payload, "{", 1);
                                    if(iMboxEnqPos > 0)
                                    {
                                        uint16_t mboxNnn = (uint16_t)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iMboxEnqPos + 1), strlen(aprsmsg.msg_payload));
                                        char mboxPayload[MC_PAYLOAD_LEN];
                                        mcSet(mboxPayload, sizeof(mboxPayload), aprsmsg.msg_payload);
                                        mcTruncate(mboxPayload, sizeof(mboxPayload), (size_t)iMboxEnqPos);

                                        msgstoreStore(aprsmsg.msg_source_call, destination_call,
                                                      mboxNnn, mboxPayload, strlen(mboxPayload));
                                    }
                                }

                                // S3: peer-cancel hook -- a hop-0 delivery frame (rly_hop
                                // computed above) whose path already holds one hop is
                                // another store node's mailbox delivery, heard directly.
                                if(bMboxPeerDelivery)
                                    msgstoreOnPeerDelivery(aprsmsg.msg_source_call, mboxTagNnn);
#endif
                                //
                                // next sequence to decode special broadcast messages
                                //
                                bool bSendAckGateway=true;
                                if(mcStartsWith(aprsmsg.msg_payload, "{ping}"))
                                {
                                    bSendAckGateway = false;
                                    bMeshDestination = false;
                                }
                                else
                                if(mcStartsWith(aprsmsg.msg_payload, "{pong}"))
                                {
                                    bSendAckGateway=false;
                                    bMeshDestination = false;
                                }
                                else
                                if(memcmp(aprsmsg.msg_payload, "{MCP}", 5) == 0)
                                {
                                    queueDisplayText(aprsmsg, rssi, snr);

                                    if(bDisplayVia)
                                        printfdeb("[MESHx]...SRC-PATH:%s ... DST-PATH:%s TEXT:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.msg_payload);

                                    bSendAckGateway=false;
                                }
                                else
                                if(memcmp(aprsmsg.msg_payload, "{SET}", 5) == 0)
                                {
                                    queueDisplayText(aprsmsg, rssi, snr);

                                    if(bDisplayVia)
                                        printfdeb("[MESHx]...SRC-PATH:%s ... DST-PATH:%s TEXT:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.msg_payload);

                                    bSendAckGateway=false;
                                }
                                else
                                if(memcmp(aprsmsg.msg_payload, "{CET}", 5) == 0)
                                {
                                    if(memcmp(aprsmsg.msg_payload, "{CET}<", 6) == 0)
                                        bMeshDestination = false;   // falsche Zeit nicht weiter geben
                                    else
                                    {
                                        queueDisplayText(aprsmsg, rssi, snr);

                                        if(bDisplayVia)
                                            printfdeb("[MESHx]...SRC-PATH:%s ... DST-PATH:%s TEXT:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.msg_payload);
                                    }

                                    bSendAckGateway=false;
                                }
                                else
                                {
                                    //
                                    // next sequence to send incomming "Messages to All" to Display and/or APP via BLE
                                    //
                                    if((strcmp(destination_call, "*") == 0 && !bNoMSGtoALL) || CheckOwnGroup(destination_call))
                                    {
                                        queueDisplayText(aprsmsg, rssi, snr);

                                        if(bDisplayVia)
                                            printfdeb("[MESHx]...SRC-PATH:%s ... DST-PATH:%s TEXT:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.msg_payload);

                                        // APP Offline
                                        if(isPhoneReady == 0)
                                        {
                                            // App-Offline-Flag nur fuer die BLE-Kopie setzen: encodeAPRS() serialisiert
                                            // es aus dem Bool (0x20). Nicht in max_hop odern -- das Hop-Nibble muss rein
                                            // bleiben, sonst laeuft der Relay-Guard unten bei Nibble 0 in den Unterlauf
                                            // (0x20 - 1 = 0x1F, auf Luft 15 Hops). Danach den Empfangswert wiederherstellen,
                                            // damit der Relay-Frame (encodeAPRS unten) das Flag nicht neu bekommt.
                                            bool prev_app_offline = aprsmsg.msg_app_offline;
                                            aprsmsg.msg_app_offline = true;

                                            uint8_t tempRcvBuffer[255];

                                            uint16_t tempsize = encodeAPRS(tempRcvBuffer, aprsmsg);

                                            addBLEOutBuffer(tempRcvBuffer, tempsize);

                                            aprsmsg.msg_app_offline = prev_app_offline;
                                        }
                                        else
                                        {
                                            addBLEOutBuffer(RcvBuffer, size);
                                        }
                                    }

                                    // If message already comes from one gateway/server no ACK from another gateway
                                    if(aprsmsg.msg_server)
                                        bSendAckGateway=false;

                                    // Telemetry/Ping no ACK
                                    if(strcmp(destination_call, "100001") == 0)
                                    {
                                        bSendAckGateway=false;

                                        #if defined(ENABLE_SOFTSER)
                                            if(bSOFTSERREAD)
                                            {
                                                queueDisplayText(aprsmsg, rssi, snr);

                                                if(bDisplayVia)
                                                    printfdeb("[MESHx]...SRC-PATH:%s ... DST-PATH:%s TEXT:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path, aprsmsg.msg_payload);

                                                displaySOFTSER(aprsmsg);
                                            }
                                        #endif
                                    }
                                }

                                if(bGATEWAY)
                                {
                                    if(bSendAckGateway)
                                    {
                                        print_buff[6]=aprsmsg.msg_id & 0xFF;
                                        print_buff[7]=(aprsmsg.msg_id >> 8) & 0xFF;
                                        print_buff[8]=(aprsmsg.msg_id >> 16) & 0xFF;
                                        print_buff[9]=(aprsmsg.msg_id >> 24) & 0xFF;

                                        // nur bei Meldungen an fremde mit ACK
                                        if(checkOwnTx(aprsmsg.msg_id) >= 0)
                                        {
                                            // und an alle geht Wolke mit Hackerl an BLE senden
                                            if(strcmp(aprsmsg.msg_destination_call, "*") == 0)
                                            {
                                                // Gateway hoert seine eigene Meldung zurueck und ist selbst der
                                                // Quittierende: Status 0x01 mit eigenem Rufzeichen.
                                                uint8_t phone_buff[ACK_PHONE_MAX_LEN];
                                                uint16_t plen = buildAckPhoneFrame(phone_buff, aprsmsg.msg_id, 0x01, meshcom_settings.node_call);
                                                addBLEOutBuffer(phone_buff, plen);
                                                dmstat_gw_ack.fetch_add(1);   // 0.3: checkOwnTx >= 0 already gates this block
                                            }
                                        }
                                        else
                                        {
                                            if(bDisplayInfo && !bNewLine)
                                            {
                                                printfdeb("\n");
                                                bNewLine=true;
                                            }
                                            
                                            //Check DM Message nicht vom GW ACK nur wenn "*" (an alle), "WLNK-1", "APRS2SOTA" und Group-Message
                                            if(strcmp(destination_call, "*") == 0 || strcmp(destination_call, "WLNK-1") == 0 || strcmp(destination_call, "APRS2SOTA") == 0 || CheckGroup(destination_call) > 0)
                                            {
                                                // ACK MSG 0x41 | 0x01020111 | max_hop | 0x01020304 | 1/0 ack from GW or Node 0x00 = Node, 0x01 = GW
                                                msg_counter=millis();   // ACK mit neuer msg_id versenden

                                                print_buff[0]=MSG_TYPE_ACK;
                                                print_buff[1]=msg_counter & 0xFF;
                                                print_buff[2]=(msg_counter >> 8) & 0xFF;
                                                print_buff[3]=(msg_counter >> 16) & 0xFF;
                                                print_buff[4]=(msg_counter >> 24) & 0xFF;
                                                print_buff[5]=0x80;      // server & max hop
                                                print_buff[5] = print_buff[5] | meshcom_settings.max_hop_text;
                                                // FIX BUG #6: Include original msg_id so sender can match ACK and stop retransmitting
                                                print_buff[6]=aprsmsg.msg_id & 0xFF;
                                                print_buff[7]=(aprsmsg.msg_id >> 8) & 0xFF;
                                                print_buff[8]=(aprsmsg.msg_id >> 16) & 0xFF;
                                                print_buff[9]=(aprsmsg.msg_id >> 24) & 0xFF;
                                                print_buff[10]=0x01;     // switch ack GW / Node currently fixed to 0x00
                                                print_buff[11]=0x00;     // msg always 0x00 at the end
                                                
                                                addTxRingEntry(print_buff, 12, RING_STATUS_DONE, "rx_dm_ack_gw");

                                                if(bDisplayInfo)
                                                {
                                                    printfdeb("%s", getTimeString().c_str());
                                                    printfdeb(" ACK from LoRa GW %02X %02X%02X%02X%02X %02X %02X\n", print_buff[5], print_buff[9], print_buff[8], print_buff[7], print_buff[6], print_buff[10], print_buff[11]);
                                                    bNewLine=true;
                                                }
                                                                                            
                                                unsigned long mid = (print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);

                                                bool bServerFlag = false;
                                                if((print_buff[5] & 0x80) == 0x80)
                                                    bServerFlag=true;

                                                addLoraRxBuffer(mid, bServerFlag);

                                                msg_counter=millis();   // ACK mit neuer msg_id versenden

                                                print_buff[1]=msg_counter & 0xFF;
                                                print_buff[2]=(msg_counter >> 8) & 0xFF;
                                                print_buff[3]=(msg_counter >> 16) & 0xFF;
                                                print_buff[4]=(msg_counter >> 24) & 0xFF;

                                                addTxRingEntry(print_buff, 12, RING_STATUS_DONE, "rx_dm_ack_new");

                                                mid = (print_buff[1]) | (print_buff[2]<<8) | (print_buff[3]<<16) | (print_buff[4]<<24);
                                                
                                                addLoraRxBuffer(mid, bServerFlag);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else
                        if(msg_type_b_lora == MSG_TYPE_POSITION)
                        {
                            if(!bSOFTSERREAD)
                                queueDisplayPosition(aprsmsg, rssi, snr);

                            if(isPhoneReady > 0)
                                addBLEOutBuffer(RcvBuffer, size);

                            #if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)
                            struct aprsPosition aprspos;

                            if(decodeAPRSPOS(aprsmsg.msg_payload, aprspos) == 0x01)
                            {
                                tdeck_add_pos_point(aprsmsg.msg_source_call, 
                                    conv_coord_to_dec(aprspos.lat), 
                                    aprspos.lat_c, 
                                    conv_coord_to_dec(aprspos.lon), 
                                    aprspos.lon_c);
                                tdeck_add_to_pos_view(aprsmsg.msg_source_call, 
                                    conv_coord_to_dec(aprspos.lat), 
                                    aprspos.lat_c, 
                                    conv_coord_to_dec(aprspos.lon), 
                                    aprspos.lon_c,
                                    conv_meter(aprspos.alt));
                            }
                            #endif
                        }

                        // messages to WLNK-1 or APRS2SOTA no need to MESH via a gateWay
                        if(bGATEWAY)
                        {
                            if(strcmp(destination_call, "WLNK-1") == 0)
                                bMeshDestination = false;
                            else
                            if(strcmp(destination_call, "APRS2SOTA") == 0)
                                bMeshDestination = false;
                            else
                            if(strcmp(destination_call, "100001") == 0)
                                bMeshDestination = false;

                            if(!bMeshDestination)
                                rly_reason = "gwfilter";    // SL-02

                            if(aprsmsg.payload_type == ':' && aprsmsg.msg_last_path_cnt >= meshcom_settings.max_hop_text+1)    // TEXT
                            {
                                if(bMeshDestination)
                                    rly_reason = "gwcap";   // SL-02
                                bMeshDestination = false;
                            }
                            if(aprsmsg.payload_type == '!' && aprsmsg.msg_last_path_cnt >= meshcom_settings.max_hop_pos+1)    // POS
                            {
                                if(bMeshDestination)
                                    rly_reason = "gwcap";   // SL-02
                                bMeshDestination = false;
                            }

                            //KBC not usefull if(aprsmsg.payload_type == '@' && meshcom_settings.node_hasIPaddress)    // HEY no Mesh on GATEWAYs with Server-Connected
                            //KBC not usefill bMeshDestination = false;
                        }

                        // ping no mesh
                        if(aprsmsg.payload_type == ':' && strcmp(destination_call, "100001") == 0 && mcStartsWith(aprsmsg.msg_payload, "{ping}"))    // TEXT
                        {
                            if(bMeshDestination)
                                rly_reason = "ping";        // SL-02
                            bMeshDestination = false;
                        }

                        // GATEWAY action before MESH
                        // and not MESHed from another Gateways
                        bool bHeyReportAppended = false;
                        if(bGATEWAY && (!aprsmsg.msg_server || aprsmsg.payload_type == '@'))  // HEY always send to Server
                        {
                            if(aprsmsg.payload_type == '@')
                            {
                                // append own signal report (NCT,RSSI,SNR) before UDP out, so the
                                // server gets the same report the mesh gets — the relay path below
                                // skips its append (RcvBuffer is re-encoded there anyway)
                                appendHeySignalReport(aprsmsg, rssi, snr, nbrNcntAir(nbrMatrix, uptimeMin16()));
                                bHeyReportAppended = true;

                                memset(RcvBuffer, 0x00, UDP_TX_BUF_SIZE);
                                size = encodeAPRS(RcvBuffer, aprsmsg);
                                if(size + 2 > UDP_TX_BUF_SIZE)
                                    size = UDP_TX_BUF_SIZE - 2;
                            }

                            // SL-06: Upload zum Server, unmittelbar vor
                            // addNodeData() und vor dem Hop-Dekrement des Relays.
                            //
                            // PN retry: a repeat copy of a PN we already
                            // uploaded must not upload again -- the relay
                            // decision below (rly_go/rly_reason) still runs
                            // regardless, so the frame keeps propagating.
                            if(!rx_pn_repeat)
                            {
                                if(bDisplayLog)
                                {
                                    setlogFormatGwu(setlog_buf, sizeof(setlog_buf), aprsmsg.msg_id,
                                                    aprsmsg.payload_type, aprsmsg.max_hop & 0x0F, (uint32_t)millis());
                                    setlogPrint(setlog_buf);
                                }

                                addNodeData(RcvBuffer, size, rssi, snr);
                            }
                        }

                        // resend only Packet to all and !owncall
                        // bSetLoRaAPRS = APRS via 433.775 usw.

                        // SL-02: dieselbe Bedingung wie bisher, nur zerlegt, damit
                        // genau ein Grund benannt wird -- Kurzschlussreihenfolge
                        // und damit jeder checkMesh()-Aufruf bleiben erhalten.
                        bool rly_go = false;

                        if(strcmp(destination_call, meshcom_settings.node_call) == 0)
                        {
                            if(rly_reason == NULL)
                                rly_reason = "self";
                        }
                        else
                        if(bSetLoRaAPRS)
                        {
                            if(rly_reason == NULL)
                                rly_reason = "aprs";
                        }
                        else
                        if(!checkMesh(aprsmsg))
                        {
                            if(rly_reason == NULL)
                                rly_reason = "nomesh";
                        }
                        else
                        if(!bMeshDestination)
                        {
                            // Die uebrigen Ruecksetzer des Flags liegen im Zweig
                            // "Ziel ist das eigene Rufzeichen", hier nie erreicht.
                            if(rly_reason == NULL)
                                rly_reason = "gwfilter";
                        }
                        else
                            rly_go = true;

                        if(rly_go)
                        {
                            // MESH only max. hops (default 3...TEXT 1...POS)
                            if((aprsmsg.max_hop & 0x0F) > 0)   // nur das Hop-Nibble zaehlt, Flag-Bits oeffnen den Relay-Guard nicht
                            {
                                // only set Serverflag if connection to MeshCom-Server
                                if(bGATEWAY && meshcom_settings.node_hasIPaddress)
                                    aprsmsg.msg_server = true;  // signal to another gateway not to send to MESHCOM-Server

                                aprsmsg.max_hop--;

                                aprsmsg.msg_last_hw = BOARD_HARDWARE | 0x80; // hardware  last sending node   last sending node (0x80)
                                aprsmsg.msg_source_mod = (getMOD() & 0xF) | (meshcom_settings.node_country << 4); // modulation & country

                                // Loop detection: skip relay if own callsign already in path
                                {
                                    String searchCall = String(",") + meshcom_settings.node_call + ",";
                                    String searchPath = String(",") + aprsmsg.msg_source_path + ",";
                                    if(searchPath.indexOf(searchCall) >= 0)
                                    {
                                        if(bLORADEBUG)
                                            printfdeb("[MC-DBG] RELAY_LOOP_BLOCKED own_call_in_path\n");
                                        rly_reason = "loop";    // SL-02
                                        goto skip_relay;
                                    }
                                }

                                ////////////////////////////////////////////////////////////
                                //Next hop new via
                                if(bDisplayVia)
                                    printfdeb("[MESH<]...DEST-CALL:%s ... DEST-PATH:%s\n", aprsmsg.msg_destination_call, aprsmsg.msg_destination_path);

                                mcSet(aprsmsg.msg_destination_path, sizeof(aprsmsg.msg_destination_path), aprsmsg.msg_destination_call);

                                checkVia(aprsmsg);

                                if(bDisplayVia)
                                    printfdeb("[MESH>]...SRC-PATH:%s ... DEST-PATH:%s\n", aprsmsg.msg_source_path, aprsmsg.msg_destination_path);
                                //
                                ////////////////////////////////////////////////////////////

                                // Nachbarschaftsmatrix Stufe 2 (docs/nbr-wichtigkeit-konzept.md
                                // 5.1): Bedarf VOR dem Umschreiben des Pfads berechnen --
                                // msg_source_path traegt hier noch die Ansicht, wie der Frame
                                // ankam (Absender zuerst, letzter Hop zuletzt), genau die
                                // Ansicht, die nbrRelayNeed() braucht. Nach bSHORTPATH/dem
                                // Anhaengen des eigenen Rufzeichens waere es bereits die
                                // eigene Aussendung.
                                if(bNBRRELAY)
                                {
                                    now_min_relay = uptimeMin16();
                                    nn_relay = nbrRelayNeed(nbrMatrix, aprsmsg.msg_source_path, now_min_relay,
                                                             bNBRSYM, (uint32_t)aprsmsg.msg_id);
                                }

                                if(bSHORTPATH)
                                {
                                    /* short path */
                                    mcSet(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), aprsmsg.msg_source_call);    //call last sending node
                                    mcAppendChar(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), ',');
                                    mcAppend(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), meshcom_settings.node_call);
                                }
                                else
                                {
                                    /*long path*/
                                    mcAppendChar(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), ',');
                                    mcAppend(aprsmsg.msg_source_path, sizeof(aprsmsg.msg_source_path), meshcom_settings.node_call);
                                }

                                if(aprsmsg.payload_type == '@' && !bHeyReportAppended)
                                    appendHeySignalReport(aprsmsg, rssi, snr, nbrNcntAir(nbrMatrix, uptimeMin16()));
                                
                                memset(RcvBuffer, 0x00, UDP_TX_BUF_SIZE);

                                size = encodeAPRS(RcvBuffer, aprsmsg);

                                if(size + 2 > UDP_TX_BUF_SIZE)
                                    size = UDP_TX_BUF_SIZE - 2;

                                // FIX: Relay messages are fire-and-forget.
                                // Only the ORIGINATING node should retransmit.
                                if(bLORADEBUG)
                                {
                                    // Werte aus RcvBuffer statt aus dem Ring lesen: der Slot wird
                                    // erst innerhalb von addTxRingEntry() unter Lock gewaehlt/beschrieben.
                                    unsigned int relay_msg_id = (RcvBuffer[4]<<24) | (RcvBuffer[3]<<16) | (RcvBuffer[2]<<8) | RcvBuffer[1];
                                    printfdeb("[MC-DBG] RELAY_QUEUED msg_id=%08X type=%02X len=%d\n",
                                        relay_msg_id, RcvBuffer[0], size);
                                }

                                // no retransmission for ANY relay message; Slot vorher komplett
                                // nullen (Alt-Verhalten: memset des ganzen Rings vor dem Schreiben)
                                // Nachbarschaftsmatrix Stufe 2 (Konzept 5.1): kind/need/alone
                                // durchreichen, damit der Mithoer-Scan (OnRxDone weiter oben)
                                // und der fallabhaengige CSMA-Backoff (csma_compute_timeout_slot())
                                // diesen Slot wiederfinden. Ohne --nbrrelay bleibt kind
                                // RING_KIND_OTHER wie bisher (nn_relay bleibt leer).
                                // known == false ("kein Wissen": leere Matrix, ungueltiger
                                // Pfad) bleibt RING_KIND_OTHER -- heutiges Fluten, nie Fall B
                                // (Advisor-Fund 2026-09-22, Konzept 1: nichts unterdrueckt auf
                                // Verdacht).
                                rly_slot = addTxRingEntry(RcvBuffer, size, RING_STATUS_DONE, "rx_relay", 0, true,
                                                           (bNBRRELAY && nn_relay.known) ? RING_KIND_RELAY : RING_KIND_OTHER,
                                                           &nn_relay.need, &nn_relay.alone);

                                // SL-02: Rueckgabe ist der belegte Slot bzw. -1,
                                // wenn der Ring den Eintrag verworfen hat.
                                if(rly_slot >= 0)
                                {
                                    rly_reason = "tx";
                                    rly_prio = ringPriority[rly_slot];

                                    // Nachbarschaftsmatrix Stufe 2 (Konzept 5.1): Zaehler je Fall
                                    // und die NEED-Zeile fuers 24-h-Log (docs/nbr-logformat.md-
                                    // Familie), nur wenn die Masken oben tatsaechlich berechnet
                                    // wurden.
                                    if(bNBRRELAY)
                                    {
                                        if(!nn_relay.known)
                                            ; // kein Wissen: kein Fall, kein Zaehler -- nur die NEED-Zeile mit 'U'
                                        else if(!nbrMaskEmpty(nn_relay.alone))
                                            stat_nbr_relay_a++;
                                        else
                                            stat_nbr_relay_b++;

                                        if(nbrLog != NULL)
                                        {
                                            char nbr_typ = (aprsmsg.payload_type == ':') ? 'T' :
                                                           (aprsmsg.payload_type == '!') ? 'P' : 'H';
                                            char nbr_case = !nn_relay.known ? 'U' : (!nbrMaskEmpty(nn_relay.alone)) ? 'A' : 'B';
                                            uint32_t nbr_mid = extractRingMsgId(rly_slot);
                                            char need_hex[NBR_MASK_HEX_LEN + 1];
                                            char alone_hex[NBR_MASK_HEX_LEN + 1];
                                            char inferred_hex[NBR_MASK_HEX_LEN + 1];
                                            nbrMaskHex(nn_relay.need, need_hex, sizeof(need_hex));
                                            nbrMaskHex(nn_relay.alone, alone_hex, sizeof(alone_hex));
                                            nbrMaskHex(nn_relay.inferred, inferred_hex, sizeof(inferred_hex));
                                            char nbr_line[64 + 3 * (NBR_MASK_HEX_LEN + 1)];
                                            snprintf(nbr_line, sizeof(nbr_line),
                                                     "[NBR]|NEED|%u|%08X|%c|%c|%s|%s|%d|%s",
                                                     (unsigned)now_min_relay, (unsigned)nbr_mid,
                                                     nbr_typ, nbr_case,
                                                     need_hex, alone_hex,
                                                     rly_slot, inferred_hex);
                                            nbrLog(nbr_line);
                                        }
                                    }
                                }
                                else
                                    rly_reason = "full";

                                /*
                                if(bDisplayInfo)
                                {
                                    printfdeb(" This packet to mesh\n");
                                    bNewLine=true;
                                }
                                */
                            }
                            else
                                rly_reason = "hop0";    // SL-02

                            skip_relay: ;
                        }
                        else
                        {
                            if(bDisplayInfo && !bNewLine)
                                printfdeb("\n");
                        }

                        // SL-02: genau eine RLY-Zeile je neuem Frame, hinter
                        // allen Ausstiegen des Relay-Blocks (auch skip_relay).
                        if(bDisplayLog && rly_reason != NULL)
                        {
                            setlogFormatRly(setlog_buf, sizeof(setlog_buf), aprsmsg.msg_id,
                                            aprsmsg.payload_type, rly_hop, rly_reason, rly_prio, rly_slot);
                            setlogPrint(setlog_buf);
                        }
                    }
                }
                else
                {
                    // print hex of message
                    if(bDEBUG)
                        printBuffer(RcvBuffer, size);
                }

                // set buffer to 0
                memset(RcvBuffer, 0, UDP_TX_BUF_SIZE);

                //blinkLED();
            }
            else
            {
                // 0.2: split the dedup gate. This is the pure duplicate
                // branch (rx_is_new was false above -- SL-01's dedup verdict
                // above is unchanged, we only read what it already decided).
                // A duplicate DM addressed to us may be a lost :ackNNN's only
                // sign of life; re-ACK it, rate-limited, and stop here: no
                // display, no phone/server forward, no relay. aprsmsg is
                // already fully decoded (decodeAPRS() above, unconditional).
                if(msg_type_b_lora == MSG_TYPE_TEXT &&
                   strcmp(aprsmsg.msg_destination_call, meshcom_settings.node_call) == 0 &&
                   !mcStartsWith(aprsmsg.msg_payload, "{ping}") &&
                   !mcStartsWith(aprsmsg.msg_payload, "{pong}"))
                {
                    int iReackAckPos = mcIndexOfStr(aprsmsg.msg_payload, ":ack");
                    int iReackRejPos = mcIndexOfStr(aprsmsg.msg_payload, ":rej");
                    int iReackEnqPos = mcIndexOfStrFrom(aprsmsg.msg_payload, "{", 1);

                    if(iReackAckPos <= 0 && iReackRejPos <= 0 && iReackEnqPos > 0)
                    {
                        uint16_t reackNnn = (uint16_t)mcSliceToLong(aprsmsg.msg_payload, (size_t)(iReackEnqPos + 1), strlen(aprsmsg.msg_payload));

                        if(reackAllowed(aprsmsg.msg_source_call, reackNnn, millis()))
                        {
                            SendAckMessage(aprsmsg.msg_source_call, reackNnn);
                            dmstat_reack.fetch_add(1);

                            if(bDisplayInfo)
                                printfdeb("\n[REACK] dup from %s nnn:%03u\n", aprsmsg.msg_source_call, reackNnn);
                        }
                        else
                        {
                            dmstat_reack_limited.fetch_add(1);

                            if(bDisplayInfo)
                                printfdeb("\n[REACK-LIMIT] dup from %s nnn:%03u\n", aprsmsg.msg_source_call, reackNnn);
                        }
                    }
                }
            }
        }
    }

    // Note: Radio.Rx() for RAK4630 is now called at the beginning of OnRxDone
    // to minimize the RX blind window (BUG #2 fix).

    // Debug I: ONRXDONE_TIME — measure processing duration
    {
        unsigned long _onrxdone_elapsed = millis() - _onrxdone_start;
        if(bLORADEBUG)
            printfdeb("[MC-DBG] ONRXDONE_TIME ms=%lu\n", _onrxdone_elapsed);
        if(_onrxdone_elapsed > onrxdone_max_ms)
            onrxdone_max_ms = _onrxdone_elapsed;
        if(_onrxdone_elapsed > ONRXDONE_WARN_MS)
        {
            onrxdone_warn_count++;
            if(bLORADEBUG)
                printfdeb("[MC-WARN] ONRXDONE_SLOW ms=%lu threshold=%d\n", _onrxdone_elapsed, ONRXDONE_WARN_MS);
        }
    }

    if(bLORADEBUG)
        printfdeb("OnRxDone\n");

    iReceiveTimeOutTime = millis();
    csma_timeout = csma_compute_timeout(cad_attempt);

#if defined BOARD_RAK4630
    taskENTER_CRITICAL();
    rxBufInUse[rxBufIndex] = false;
    taskEXIT_CRITICAL();
    if(bLORADEBUG)
        printfdeb("[MC-DBG] RX_BUF_RELEASE buf=%d\n", rxBufIndex);
#endif

    if(bLORADEBUG)
        printfdeb("[MC-SM] RX_PROCESS -> RX_LISTEN rc=0\n");
    is_receiving = false;

    // TM-06: state above is fully settled -- safe point for
    // test_inject_service() to recurse into OnRxDone() (see there).
    test_inject_service();
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
    #if defined BOARD_RAK4630
        startRadioReceive();
        // RACE-05 fix: CAD abort under critical section
        taskENTER_CRITICAL();
        if(cad_in_progress) {
            cad_in_progress = false;
            cad_done_flag = false;
            cad_double_check = false;
        }
        taskEXIT_CRITICAL();
    #endif

    if(bLORADEBUG)
        printfdeb("OnRxTimeout\n");

    {
        unsigned long _rx_s = ch_util_rx_start.exchange(0);
        if(_rx_s > 0)
            ch_util_rx_accum.fetch_add(millis() - _rx_s);
    }

    is_receiving = false;
}

/**@brief Function to be executed on Radio Rx Error event
 */

void OnRxError(void)
{
    #if defined BOARD_RAK4630
        startRadioReceive();
        // RACE-05 fix: CAD abort under critical section
        taskENTER_CRITICAL();
        if(cad_in_progress) {
            cad_in_progress = false;
            cad_done_flag = false;
                cad_double_check = false;
        }
        taskEXIT_CRITICAL();
    #endif

    if(bLORADEBUG)
    {
        printfdeb("OnRxError\n");
        #if defined BOARD_RAK4630
        {
            // RadioPktStatus is populated by SX126xGetPacketStatus() in
            // RadioBgIrqProcess before calling this callback (for CRC errors).
            // For header errors the values may be stale — still useful context.
            extern PacketStatus_t RadioPktStatus;
            printfdeb("[MC-DBG] RX_ERROR rssi=%d snr=%d ts=%lu\n",
                RadioPktStatus.Params.LoRa.RssiPkt,
                RadioPktStatus.Params.LoRa.SnrPkt,
                millis());
        }
        #endif
    }

    // SL-04: verlorener Frame. Zaehler unabhaengig von jedem Debug-Flag, die
    // Zeile nur unter --setlog. Die [MC-DBG]-Ausgabe oben bleibt unveraendert.
    stat_rx_err.fetch_add(1);

    if(bDisplayLog)
    {
        int16_t err_rssi = 0;
        int8_t  err_snr  = 0;

        #if defined BOARD_RAK4630
        {
            // Wie oben: von SX126xGetPacketStatus() in RadioBgIrqProcess
            // gefuellt. Bei Header-Fehlern koennen die Werte alt sein.
            extern PacketStatus_t RadioPktStatus;
            err_rssi = (int16_t)RadioPktStatus.Params.LoRa.RssiPkt;
            err_snr  = (int8_t)RadioPktStatus.Params.LoRa.SnrPkt;
        }
        #endif

        // Laenge und Frequenzfehler liefert dieser Pfad nicht (die ESP32-Seite
        // in esp32_main.cpp tut es).
        char err_buf[64];
        setlogFormatErr(err_buf, sizeof(err_buf), err_rssi, err_snr, 0, 0, (uint32_t)millis());
        setlogPrint(err_buf);
    }

    {
        unsigned long _rx_s = ch_util_rx_start.exchange(0);
        if(_rx_s > 0)
            ch_util_rx_accum.fetch_add(millis() - _rx_s);
    }

    is_receiving = false;
}

// is_new_packet() ist nach dedup_functions.cpp gewandert (reine Verschiebung).

//////////////////////////////////////////////////////////////////////////
// LoRa TX functions
//
// getMessagePriority/getNextTxSlot/advanceIReadPastEmpty/addTxRingEntry
// wurden nach txring_functions.cpp verschoben (QA-Welle 2026-08-22, N-14) --
// reine Verschiebung, Logik unveraendert. Siehe txring_functions.h/.cpp und
// test/test_txring/test_txring.cpp.

// SL-03 -- eine Zeile je gestarteter Sendung (Erfolgsausgaenge von doTX()).
// stat_txn zaehlt auch bei --setlog off.
static void setlogPrintTx(const char *line)
{
    stat_txn.fetch_add(1);

    if(bDisplayLog && line[0] != 0x00)
        setlogPrint(line);
}

/**@brief our Lora TX sequence — priority-based slot selection
 */
// Debug K: RADIO_TX -- der Marker "ein Frame geht an den Funkchip".
//
// Stand bis 2026-09-13 an genau einer der drei Sendestellen in doTX(), und dort
// nur im Nicht-RAK-Zweig. Auf dem ESP32 feuerte er damit fuer normale
// MeshCom-Frames, nicht fuer TRACK/LoRa-APRS, auf dem RAK ueberhaupt nie -- als
// Nachweis "gesendet" war er wertlos, und seine Abwesenheit bewies nichts.
// Jetzt an allen sechs Varianten (drei Stellen x RAK/Nicht-RAK).
//
// kind= trennt die Stellen: track = Positions-Frame im Trackbetrieb,
// aprs = LoRa-APRS-Aussendung, msg = normaler MeshCom-Frame (Ping inklusive).
// Das Feld haengt hinten an, der Prefix "[MC-DBG] RADIO_TX len=" bleibt fuer
// bestehende Greps unveraendert.
//
// Auf dem RAK bewusst VOR vTaskSuspendAll() gerufen: printfdeb() allokiert und
// gehoert nicht in einen Abschnitt mit angehaltenem Scheduler.
static inline void logRadioTx(int len, const char *kind)
{
    if(bLORADEBUG)
        printfdeb("[MC-DBG] RADIO_TX len=%d kind=%s\n", len, kind);
}

bool doTX()
{
    //#if not defined(BOARD_T_DECK_PRO)

    // Priority-based slot selection instead of plain FIFO
    int txSlot = getNextTxSlot();
    if (txSlot >= 0)
    {
        // Track latency
        uint8_t prio = ringPriority[txSlot];
        uint32_t latency = millis() - ringEnqueueTime[txSlot];
        if(prio >= 1 && prio <= 5)
        {
            stat_tx_count[prio]++;
            stat_latency_sum[prio] += latency;
            if(latency > stat_latency_max[prio])
                stat_latency_max[prio] = (uint16_t)min(latency, (uint32_t)65535);
        }

        // Track preemption (out-of-order send)
        if(txSlot != iRead)
            stat_preempt_count++;

        sendlng = ringBuffer[txSlot][0];
        if(sendlng >= UDP_TX_BUF_SIZE)
            sendlng = UDP_TX_BUF_SIZE - 1;

        memset(lora_tx_buffer, 0x00, UDP_TX_BUF_SIZE + 1);
        memcpy(lora_tx_buffer, ringBuffer[txSlot] + 2, sendlng);

        lora_tx_buffer[sendlng]=0x00;

        int save_read = txSlot;

        #ifndef BOARD_TLORA_OLV216
        char save_ring_status = ringBuffer[txSlot][1];
        #endif

        if(bLORADEBUG)
        {
            uint32_t tx_mid = ((uint32_t)ringBuffer[txSlot][6] << 24) |
                              ((uint32_t)ringBuffer[txSlot][5] << 16) |
                              ((uint32_t)ringBuffer[txSlot][4] << 8)  |
                               (uint32_t)ringBuffer[txSlot][3];
            // BP-02: qlen/queued in TX markers is txRingDepth() (occupied
            // slots), not the raw index distance -- this marker fires on
            // every TX and would otherwise dominate a log with the old
            // hole-counting number right next to an honest RING_STATUS.
            int queued = txRingDepth();
            if(bLORADEBUG)
                printfdeb("[MC-DBG] RING_TX_READ slot=%d prio=%d type=%02X status=%02X "
                          "len=%d msg_id=%08X retry=%d queued=%d/%d lat=%lums\n",
                          txSlot, prio, ringBuffer[txSlot][2], ringBuffer[txSlot][1],
                          sendlng, tx_mid, retryCount[txSlot], queued, MAX_RING, (unsigned long)latency);
        }

        // SL-03: hier nur formatieren, gedruckt wird erst nach erfolgreichem
        // Sendestart (setlogPrintTx()), damit ein Rollback keine Zeile hinterlaesst.
        char setlog_tx_buf[128];
        setlog_tx_buf[0] = 0x00;

        if(bDisplayLog)
        {
            uint32_t sl_tx_mid = ((uint32_t)ringBuffer[txSlot][6] << 24) |
                                 ((uint32_t)ringBuffer[txSlot][5] << 16) |
                                 ((uint32_t)ringBuffer[txSlot][4] << 8)  |
                                  (uint32_t)ringBuffer[txSlot][3];

            setlogFormatTx(setlog_tx_buf, sizeof(setlog_tx_buf), sl_tx_mid,
                           (char)ringBuffer[txSlot][2],
                           (uint8_t)(ringBuffer[txSlot][7] & 0x0F),
                           prio, (char)ringSource[txSlot], latency, txRingDepth(),
                           cad_attempt, (uint16_t)sendlng, (uint32_t)millis());
        }

        if(ringBuffer[txSlot][1] == RING_STATUS_READY) // mark open to send
            ringBuffer[txSlot][1] = RING_STATUS_SENT; // mark as sent

        // For out-of-order reads: clear slot data length so getNextTxSlot skips it
        // and advance iRead past any empty leading slots
        #if not defined BOARD_RAK4630
        int iReadBeforeAdvance = iRead; // saved for startTransmit failure rollback
        #endif

        ringBuffer[txSlot][0] = 0; // Mark as consumed (data is in lora_tx_buffer)
        advanceIReadPastEmpty();

        // NOTE: Slot clearing (Bug 1 fix) is deferred until after the transmit
        // decision. Two rollback paths (CAD wait, APRS chip-switch failure)
        // restore the slot and retry on the next call.
        // For rollback: we restore ringBuffer[save_read][0] = sendlng.

        // Testfang-Hook TX (siehe capture_functions.h): die Bytes, die dieser
        // Knoten gleich auf den Kanal legt. Ohne diese Seite zeigt der
        // Mitschnitt nur, was empfangen wurde -- ob unser eigenes
        // Re-Enkodieren auf dem Relay-Pfad (Hop-Dekrement, Pfadanhang)
        // byte-treu ist, laesst sich daraus nicht pruefen.
        //
        // Nur puffern, nicht drucken: diese Stelle liegt zwischen der
        // CAD-Entscheidung "Kanal frei" und startTransmit(); eine halbe
        // Sekunde Serial-Ausgabe dazwischen wuerde die Kanalmessung
        // entwerten, auf der der Sendezeitpunkt beruht.
        if(bTXCAPTURE && sendlng > 0)
            captureFrame('T', lora_tx_buffer, (uint16_t)sendlng, 0, 0);

        // we can now tx the message
#if INSTRUMENT_ENABLED
        // 0.5 --airgap: treat like TX disabled, same drop semantics (slot
        // already marked consumed above -- non-rollback).
        if (TX_ENABLE == 1 && !bAirgap)
#else
        if (TX_ENABLE == 1)
#endif
        {
            // TX-01 (BACKLOG 3.8k): hard backstop -- an unconfigured node
            // (factory callsign) must not transmit, no matter what made it
            // into the ring. addTxRingEntry() already refuses to enqueue
            // for such a node; this is the runtime sibling of TX_ENABLE at
            // the only place in the tree that calls
            // Radio.Send()/startTransmit(). Non-rollback: the slot was
            // already marked consumed above, same as the TX-disabled and
            // decode-failure drop paths below.
            if(isUnconfiguredCall(meshcom_settings.node_call))
            {
                logTxRefuseUnconfigured();
                return false;
            }

#ifndef BOARD_TLORA_OLV216
            if(lora_tx_buffer[0] == '<' && bDisplayTrack)
            {
                tx_is_active = true;

                // you can transmit C-string or Arduino string up to
                // 256 characters long
                // Position zumindest alle funf Minuten auch zu MeshCom senden
                if((uint32_t)(millis() - track_to_meshcom_timer) >= 1000 * 60 * 5)
                {
                    #if defined BOARD_RAK4630
                        // N-16: taskENTER_CRITICAL() masks interrupts, including the
                        // tick — SX126xWaitOnBusy() inside Radio.Send() calls delay(1)
                        // in a loop, which needs the tick to ever return, so the old
                        // critical section could hang forever. vTaskSuspendAll() only
                        // blocks task scheduling, which is enough to keep the FreeRTOS
                        // timer-service task (OnRxDone, priority 2, see C-01) from
                        // touching the radio mid-send, without freezing the tick.
                        logRadioTx(sendlng, "track");
                        vTaskSuspendAll();
                        Radio.Send(lora_tx_buffer, sendlng);
                        xTaskResumeAll();
                    #else
                        #ifndef BOARD_T5_EPAPER
                        #ifdef RADIO_CTRL
                            digitalWrite(RADIO_CTRL, LOW);  // TX Mode [OE3WAS]
                            delay(2);
                        #endif
                        #ifdef BOARD_HELTEC_V4
                        enablePATransmit();
                        #endif
                        logRadioTx(sendlng, "track");
                        transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);
                        if(transmissionState != RADIOLIB_ERR_NONE)
                        {
                            printfdeb("[LoRa] startTransmit(track) failed: %d\n", transmissionState);
                            tx_is_active = false;
                            ringBuffer[save_read][0] = sendlng;
                            ringBuffer[save_read][1] = save_ring_status;
                            iRead = iReadBeforeAdvance;
                            return false;
                        }
                        #endif
                        bLED_RED = true;
                    #endif

                    track_to_meshcom_timer = millis();
                }

                if(!lora_setchip_aprs())
                {
                    // Rollback: restore slot so it gets picked up again
                    ringBuffer[save_read][0] = sendlng;
                    ringBuffer[save_read][1] = save_ring_status;

                    return false;
                }
                
                // you can transmit C-string or Arduino string up to
                // 256 characters long
                #if defined BOARD_RAK4630
                    // N-16, see the "track" send above for why vTaskSuspendAll()
                    // replaces taskENTER_CRITICAL() here.
                    logRadioTx(sendlng, "aprs");
                    vTaskSuspendAll();
                    Radio.Send(lora_tx_buffer, sendlng);
                    xTaskResumeAll();
                #else
                    #ifndef BOARD_T5_EPAPER
                    #ifdef RADIO_CTRL
                        digitalWrite(RADIO_CTRL, LOW);  // TX Mode [OE3WAS]
                        delay(2);
                    #endif
                    #ifdef BOARD_HELTEC_V4
                    enablePATransmit();
                    #endif
                    logRadioTx(sendlng, "aprs");
                    transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);
                    if(transmissionState != RADIOLIB_ERR_NONE)
                    {
                        printfdeb("[LoRa] startTransmit(aprs) failed: %d\n", transmissionState);
                        tx_is_active = false;
                        ringBuffer[save_read][0] = sendlng;
                        ringBuffer[save_read][1] = save_ring_status;
                        iRead = iReadBeforeAdvance;
                        return false;
                    }
                    #endif
                    bLED_ORANGE = true;
                #endif

                if(bDisplayInfo)
                {
                    printdeb(getTimeString());
                    printfdeb(" TX-APRS:%s\n", lora_tx_buffer+3);
                }

                bSetLoRaAPRS = true;

                setlogPrintTx(setlog_tx_buf);   // SL-03

                // For text messages needing retransmit: restore length so
                // updateRetransmissionStatus() can find and retransmit them.
                if(ringBuffer[save_read][1] != (char)RING_STATUS_DONE && ringBuffer[save_read][2] == MSG_TYPE_TEXT)
                    ringBuffer[save_read][0] = sendlng;

                return true;
            }
            else
#endif

            {
                struct aprsMessage aprsmsg;

                // print which message type we got
                uint16_t msg_type_b_lora = 0x00;

                msg_type_b_lora = decodeAPRS(lora_tx_buffer, (uint16_t)sendlng, aprsmsg);

                //printfdeb("msg_type_b_lora:%02X tx_waiting:%02X sendlng:%i bDisplayInfo:%i\n", msg_type_b_lora, tx_waiting, sendlng, bDisplayInfo);

                if(msg_type_b_lora != 0x00) // 0x41 ACK
                {
                    tx_is_active = true;

                    // you can transmit C-string or Arduino string up to
                    // 256 characters long
                    #if defined BOARD_RAK4630
                        // N-16, see the "track" send above for why vTaskSuspendAll()
                        // replaces taskENTER_CRITICAL() here.
                        logRadioTx(sendlng, "msg");
                        vTaskSuspendAll();
                        Radio.Send(lora_tx_buffer, sendlng);
                        xTaskResumeAll();
                    #else
                        #ifndef BOARD_T5_EPAPER
                        #ifdef RADIO_CTRL
                            digitalWrite(RADIO_CTRL, LOW);  // TX Mode [OE3WAS]
                            delay(2);
                        #endif
                        #ifdef BOARD_HELTEC_V4
                        enablePATransmit();
                        #endif

                        logRadioTx(sendlng, "msg");

                        transmissionState = radio.startTransmit(lora_tx_buffer, sendlng);
                        if(transmissionState != RADIOLIB_ERR_NONE)
                        {
                            printfdeb("[LoRa] startTransmit failed: %d\n", transmissionState);
                            tx_is_active = false;
                            ringBuffer[save_read][0] = sendlng;
                            #ifndef BOARD_TLORA_OLV216
                            ringBuffer[save_read][1] = save_ring_status;
                            #endif
                            iRead = iReadBeforeAdvance; // re-include slot in scan window
                            return false;
                        }
                        #endif
                        bLED_RED = true;
                    #endif

                    setlogPrintTx(setlog_tx_buf);   // SL-03

                    // 0.4/F4: one transmission of an OWN DM ring slot, first send or
                    // retry alike (dmstat_attempts counts both, per its
                    // dm_stats.h doc comment). Relayed foreign texts, own group
                    // messages/broadcasts and our own :ackNNN sends do not
                    // count (pnFrameIsOwnPn: PN-shaped, our node id, our call).
                    if(ringBuffer[save_read][2] == MSG_TYPE_TEXT &&
                       pnFrameIsOwnPn(lora_tx_buffer, (uint16_t)sendlng, _GW_ID, meshcom_settings.node_call))
                        dmstat_attempts.fetch_add(1);

                    if(bDisplayInfo)
                    {
                        if(lora_tx_buffer[0] == MSG_TYPE_ACK)
                        {
                            printBuffer_ack((char*)"TX-Lora ", lora_tx_buffer, sendlng);
                        }
                        else
                        {
                            printBuffer_aprs((char*)"TX-LoRa ", aprsmsg);
                        }
                    }

                    // For text messages needing retransmit: restore length so
                    // updateRetransmissionStatus() can find and retransmit them.
                    if(ringBuffer[save_read][1] != (char)RING_STATUS_DONE && ringBuffer[save_read][2] == MSG_TYPE_TEXT)
                        ringBuffer[save_read][0] = sendlng;

                    return true;
                }
            }
        }
        else
        {
#if INSTRUMENT_ENABLED
            if(bAirgap)
            {
                if(bLORADEBUG)
                    printfdeb("[AIRGAP];tx-dropped\n");
            }
            else
            {
                DEBUG_MSG("RADIO", "TX DISABLED");
            }
#else
            DEBUG_MSG("RADIO", "TX DISABLED");
#endif
        }

        // Non-rollback drop paths (TX disabled, unconfigured node, or decode failure) — slot stays cleared
    }

    //#endif

    return false;
}

// Messages are to retransmit
// based on:
// unsigned char ringBuffer[MAX_RING][UDP_TX_BUF_SIZE] = {0};

// Maximum retransmit attempts per message
#define MAX_RETRANSMIT 3
// PN-Wiederholung (XOR, pn_retry.h): k = retryCount+1 muss 1..3 bleiben,
// k=4 wuerde die Retry-Variante auf die urspruengliche id zurueckdrehen.
static_assert(MAX_RETRANSMIT <= 3, "MAX_RETRANSMIT must stay <=3: pnRetryId's k=retryCount+1 wraps onto the original id at k=4");

bool updateRetransmissionStatus()
{
    for(int ircheck = 0; ircheck < MAX_RING; ircheck++)
    {

#if defined(EXTERNAL_RADIO)
        // owned-slot reuse invariant: a slot owned by an in-flight external TX must not be
        // aged, coerced to DONE, retransmitted, or dropped here. Its outcome is
        // owned solely by externalTxResolve*() once the bridge TX_RESULT arrives.
        if(ringBuffer[ircheck][1] == RING_STATUS_EXT_PENDING)
            continue;
#endif

        // Non-text messages: force no-retransmit
        if(ringBuffer[ircheck][2] != MSG_TYPE_TEXT)
        {
            ringBuffer[ircheck][1] = RING_STATUS_DONE;
        }

        int size = ringBuffer[ircheck][0];

        if(size > 0 && ringBuffer[ircheck][1] != RING_STATUS_READY && ringBuffer[ircheck][1] != RING_STATUS_DONE)
        {
            ringBuffer[ircheck][1]++;

            // Fixed-interval retransmit: 40s per retry (20 ticks × 2s)
            //   Retry 1-3: each waits 40s → total max 120s (2 min)
            uint8_t threshold = 0x15;

            if(ringBuffer[ircheck][1] == threshold)
            {
                // PN-Wiederholung (XOR, pn_retry.h), M1: das Ziel hat schon
                // geackt (own_msg_id 0x02), dieser Slot ist aber trotzdem
                // faellig -- z.B. weil :ackNNN eintraf, waehrend die letzte
                // Kopie noch READY/CSMA wartete; findAndStopRingSlot()
                // fasst READY-Slots absichtlich nicht an (ein READY-Slot
                // kann gerade von doTX() uebernommen werden). Slot hier
                // freigeben statt erneut
                // zu senden oder aufzugeben (kein zusaetzlicher FAILED).
                if(pnFrameIsOwnPn(&ringBuffer[ircheck][2], (uint16_t)size, _GW_ID, meshcom_settings.node_call))
                {
                    uint32_t pnSlotOrigId = pnRetryId(pnFrameMsgId(&ringBuffer[ircheck][2]), _GW_ID, 0);
                    int pnAckedIdx = checkOwnTx(pnSlotOrigId);
                    if(pnAckedIdx >= 0 && own_msg_id[pnAckedIdx][4] == 0x02)
                    {
                        ringBuffer[ircheck][1] = RING_STATUS_DONE;
                        ringBuffer[ircheck][0] = 0;
                        retryCount[ircheck] = 0;

                        if(bDisplayRetx)
                            printfdeb("\n[RETX] PN already acked, stop retid:%i msg-id:%08X\n",
                                          ircheck, pnSlotOrigId);

                        continue;
                    }
                }

                // Check retry cap
                if(retryCount[ircheck] >= MAX_RETRANSMIT)
                {
                    // Give up — max retries exhausted
                    ringBuffer[ircheck][1] = RING_STATUS_DONE;
                    ringBuffer[ircheck][0] = 0;  // free slot so getNextTxSlot skips it

                    // 0.3 (D8): report failure to app + GUI, scoped to
                    // user-originated DMs only. ACK frames and broadcast
                    // texts are also retransmit-eligible and legitimately
                    // give up (advisor m6) -- reporting those unscoped would
                    // give every node in a sparse net bogus failure notices
                    // about its own ACKs/broadcasts. `size` was captured
                    // above before ringBuffer[ircheck][0] was cleared; the
                    // payload bytes at [2..] are untouched by that clear, so
                    // decoding here is the same decodeAPRS() doTX() already
                    // runs on this slot's content just before transmit.
                    {
                        unsigned int ring_msg_id = (ringBuffer[ircheck][6]<<24) | (ringBuffer[ircheck][5]<<16) | (ringBuffer[ircheck][4]<<8) | ringBuffer[ircheck][3];

                        // PN-Wiederholung (XOR, pn_retry.h), neu fuer den Fork:
                        // nach mind. einem Retry traegt der Slot die XOR-
                        // Variante der id, nicht mehr die Original-id -- vor
                        // checkOwnTx() zurueckfalten, sonst findet weder die
                        // 0x04-Held-Pruefung noch das 0x03-Telefonframe unten
                        // den eigenen own_msg_id-Eintrag.
                        if(pnFrameIsOwnPn(&ringBuffer[ircheck][2], (uint16_t)size, _GW_ID, meshcom_settings.node_call))
                            ring_msg_id = pnRetryId(ring_msg_id, _GW_ID, 0);

                        struct aprsMessage giveupMsg;
                        uint16_t giveupType = decodeAPRS(&ringBuffer[ircheck][2], (uint16_t)size, giveupMsg);

                        if(giveupType == MSG_TYPE_TEXT &&
                           strcmp(giveupMsg.msg_source_call, meshcom_settings.node_call) == 0 &&
                           strcmp(giveupMsg.msg_destination_call, "*") != 0 &&
                           CheckGroup(giveupMsg.msg_destination_call) == 0)
                        {
                            int iGiveupAckPos = mcIndexOfStr(giveupMsg.msg_payload, ":ack");
                            int iGiveupRejPos = mcIndexOfStr(giveupMsg.msg_payload, ":rej");
                            int iGiveupEnqPos = mcIndexOfStrFrom(giveupMsg.msg_payload, "{", 1);

                            if(iGiveupAckPos <= 0 && iGiveupRejPos <= 0 && iGiveupEnqPos > 0)
                            {
                                dmstat_giveup.fetch_add(1);

                                int idx = checkOwnTx(ring_msg_id);

                                // S4: a store node is holding this DM (docs/dm-stage4-plan-
                                // 20260914.md, decision 3) -- the ladder giving up on its own
                                // ring slot is not a failure, the message stays "held" until
                                // the destination's real :ack flips it. Skip the 0x03 frame
                                // and the failed mark. dmstat_giveup already counted this
                                // give-up above (F2, fable-dm-stage4-verdict-20260914.md):
                                // dmstat_giveup_held additionally marks the held subset, it
                                // does not replace the giveup count.
                                if(idx >= 0 && own_msg_id[idx][4] == 0x04)
                                {
                                    dmstat_giveup_held.fetch_add(1);
                                }
                                else
                                {
                                    if(idx >= 0 && own_msg_id[idx][4] != 0x02)
                                        own_msg_id[idx][4] = 0x03;

                                    // M1: das Ziel hat schon geackt (0x02) --
                                    // kein FAILED an die App, obwohl diese
                                    // (spaete XOR-)Kopie gerade aufgibt.
                                    if(idx < 0 || own_msg_id[idx][4] != 0x02)
                                    {
                                        uint8_t giveupPhoneBuff[ACK_PHONE_MAX_LEN];
                                        uint16_t giveupPlen = buildAckPhoneFrame(giveupPhoneBuff, ring_msg_id, ACK_STATUS_FAILED, giveupMsg.msg_destination_call);
                                        addBLEOutBuffer(giveupPhoneBuff, giveupPlen);
                                    }
                                }

                                if(bLORADEBUG)
                                    printfdeb("[MC-DBG] RETRANSMIT_GIVEUP_DM msg_id=%08X dest=%s\n",
                                              ring_msg_id, giveupMsg.msg_destination_call);
                            }
                        }
                    }

                    if(bLORADEBUG)
                    {
                        unsigned int ring_msg_id = (ringBuffer[ircheck][6]<<24) | (ringBuffer[ircheck][5]<<16) | (ringBuffer[ircheck][4]<<8) | ringBuffer[ircheck][3];
                        if(bLORADEBUG)
                            printfdeb("[MC-DBG] RETRANSMIT_GIVEUP retries=%d msg_id=%08X\n",
                            retryCount[ircheck], ring_msg_id);
                    }

                    continue;
                }

                if(bLORADEBUG)
                {
                    unsigned int ring_msg_id = (ringBuffer[ircheck][6]<<24) | (ringBuffer[ircheck][5]<<16) | (ringBuffer[ircheck][4]<<8) | ringBuffer[ircheck][3];
                    if(bLORADEBUG)
                        printfdeb("[MC-DBG] RETRANSMIT retry=%d after_sec=%d msg_id=%08X\n",
                        retryCount[ircheck] + 1, (ringBuffer[ircheck][1] - 1) * 2, ring_msg_id);
                }

                int ring_msg_lng = ringBuffer[ircheck][0];

                if(bDisplayRetx)
                {
                    unsigned int ring_msg_id = (ringBuffer[ircheck][6]<<24) | (ringBuffer[ircheck][5]<<16) | (ringBuffer[ircheck][4]<<8) | ringBuffer[ircheck][3];
                    printfdeb("\n[RETX] Retransmit retid:%i status:%02X lng;%02X msg-id: %c-%08X retry:%d\n",
                        ircheck, ringBuffer[ircheck][1], ringBuffer[ircheck][0], ringBuffer[ircheck][2], ring_msg_id, retryCount[ircheck] + 1);

                    for(int iq=0;iq<ring_msg_lng+2;iq++)
                    {
                        if(ringBuffer[ircheck][iq] >= 0x20 && ringBuffer[ircheck][iq] <= 0x7F)
                            printfdeb("%c", ringBuffer[ircheck][iq]);
                    }
                    printfdeb("\n");
                }

                // PN-Wiederholung (XOR, pn_retry.h): eigene PN-Kopie bekommt
                // eine neue msg-id statt der 1:1-Kopie, damit ein reiner
                // msg-id-Dedup beim Relay sie nicht verwirft. Fremde PNs und
                // Nicht-Text bleiben byte-identisch (LOCAL buffer, der Ring
                // wird nie nachtraeglich gepatcht -- ein anderer Task koennte
                // den Slot gerade senden).
                bool pnRewritten = false;
                uint32_t pnNewId = 0;
                bool pnEligible;
#if defined(NRF52_SERIES)
                static uint8_t pnLocalFrame[UDP_TX_BUF_SIZE];
#else
                uint8_t pnLocalFrame[UDP_TX_BUF_SIZE];
#endif

                // Groesse+Typ unter Lock erneut lesen und dort kopieren -- ein
                // Nebenlaeufer (TX/Retransmit) koennte den Slot zwischen dem
                // `size`-Read oben und hier veraendert haben. Bail-out
                // (pnEligible=false) faellt unten auf den unveraenderten
                // 1:1-Pfad zurueck.
#if defined(BOARD_RAK4630)
                taskENTER_CRITICAL();
#endif
                {
                    int pnLockedSize = ringBuffer[ircheck][0];
                    pnEligible = (pnLockedSize == size) && (pnLockedSize > 0) &&
                                 (size_t)pnLockedSize <= sizeof(pnLocalFrame) &&
                                 ringBuffer[ircheck][2] == MSG_TYPE_TEXT;
                    if(pnEligible)
                        memcpy(pnLocalFrame, &ringBuffer[ircheck][2], (size_t)pnLockedSize);
                }
#if defined(BOARD_RAK4630)
                taskEXIT_CRITICAL();
#endif

                // Nur eigene PN (eigenes Rufzeichen als Quelle): eine PN, die
                // sendMessage() fuer einen KISS-Client sendet, bleibt 1:1.
                if(pnEligible &&
                   pnFrameIsOwnPn(pnLocalFrame, (uint16_t)size, _GW_ID, meshcom_settings.node_call))
                {
                    pnNewId = pnRetryId(pnFrameMsgId(pnLocalFrame), _GW_ID,
                                        (uint8_t)(retryCount[ircheck] + 1));

                    if(pnFrameSetMsgId(pnLocalFrame, (uint16_t)size, pnNewId))
                        pnRewritten = true;
                    // else: kein FCS-Feld gefunden -- 1:1-Pfad unten greift
                }

                // ready for doTX (text messages) or fire-and-forget, wie zuvor
                uint8_t retransmitStatus = (ringBuffer[ircheck][2] == MSG_TYPE_TEXT)
                                            ? RING_STATUS_READY : RING_STATUS_DONE;

                // Quelle ist ircheck, Ziel wird erst innerhalb der Funktion (unter
                // Lock) gewaehlt — ircheck != iWrite im Normalfall (iWrite zeigt auf
                // einen leeren Slot); selbst im theoretischen Gleichstand ist
                // memcpy(dst==src) hier folgenlos (Quelle == Ziel-Byte fuer Byte).
                // Original erst NACH dem Kopieren freigeben, damit die Payload beim
                // Kopiervorgang garantiert noch gueltig ist.
                int retxSlot = addTxRingEntry(pnRewritten ? pnLocalFrame : &ringBuffer[ircheck][2],
                                (uint16_t)size, retransmitStatus,
                                "retransmit", retryCount[ircheck] + 1);

                // SL-03: "retransmit" bildet auf 'o' ab -- die Kennung des
                // Quellslots wird deshalb mitgenommen (wie bei der Prio-Verdraengung).
                if(retxSlot >= 0)
                {
                    ringSource[retxSlot] = ringSource[ircheck];

                    if(pnRewritten)
                    {
                        // Eigene Retry-id registrieren wie sendMessage() (sonst
                        // faelschlich als fremdes eigenes Echo erkannt).
                        if(bGATEWAY && meshcom_settings.node_hasIPaddress)
                            addLoraRxBuffer(pnNewId, true);
                        else
                            addLoraRxBuffer(pnNewId, false);

                        if(bDisplayRetx)
                            printfdeb("\n[RETX] PNRETRY k=%u msg-id:%08X\n",
                                          (unsigned)(retryCount[ircheck] + 1), pnNewId);
                    }
                }

                // Mark original as done and free slot (after copy, so len is correct in new slot)
                ringBuffer[ircheck][1] = RING_STATUS_DONE;
                ringBuffer[ircheck][0] = 0;  // free slot so getNextTxSlot skips it

                return true;
            }
        }
    }

    return false;
}

#if defined(EXTERNAL_RADIO)
// --- asynchronous external-radio TX ownership helpers ----------------------
// These hold a selected ring slot across an asynchronous bridge TX_RESULT. They
// are the seams the ESP32 glue drives (externalTxMarkPendingNext when submitting
// a bridge TX, externalTxResolve* from the TxSink on the terminal TX_RESULT).
// They are not called from doTX() or the local RadioLib path. A socket write is
// NOT success: only a final bridge result completes a slot.

uint32_t externalTxMarkPending(int slot)
{
    if(slot < 0 || slot >= MAX_RING || ringBuffer[slot][0] == 0)
        return 0;   // nothing to own

    // Capture the pre-pending ring status BEFORE overwriting it with EXT_PENDING,
    // so a confirmed RF send can restore the exact native post-send state
    // (READY -> retransmittable; DONE -> one-shot).
    uint8_t pre_status = ringBuffer[slot][1];

    uint32_t token = g_extTxq.begin(slot, extractRingMsgId(slot), pre_status);
    if(token == 0)
        return 0;   // an external TX is already pending (invariant: one at a time)

    // Retain content; do NOT consume the slot at submission (unlike the local
    // radio path). The slot is locked out of selection and retransmission until a
    // bridge result resolves it.
    ringBuffer[slot][1] = RING_STATUS_EXT_PENDING;

    if(bDisplayRetx)
        printfdeb("\n[RETX] ext-TX pending retid:%i msg-id:%08X token:%lu\n",
                  slot, extractRingMsgId(slot), (unsigned long)token);
    return token;
}

// Confirmed bridge RF send (TXO_SUCCESS). RF success is NOT MeshCom delivery: a
// retransmittable entry must re-enter the same native post-send waiting state so
// it can be cleared by a later ACK or retried by retransmission maintenance. A
// genuine one-shot entry completes exactly once (DONE + len=0).
bool externalTxResolveSuccess(uint32_t token)
{
    int slot = g_extTxq.slot();
    uint8_t pre_status = g_extTxq.preStatus();
    uint32_t mid = g_extTxq.msgId();
    if(g_extTxq.resolveSuccess(token) != extradio::ExtTxAction::COMPLETE_SUCCESS)
        return false;   // stale/late result: ring untouched

    if(slot < 0 || slot >= MAX_RING ||
       !extradio::extTxOwnsRingSlot(ringBuffer[slot][1], extractRingMsgId(slot),
                                    RING_STATUS_EXT_PENDING, mid))
    {
        // Token matched but the slot no longer holds the owned message (e.g. an
        // un-notified ring clear): release accounting, never touch the ring.
        extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);
        if(bDisplayRetx)
            printfdeb("\n[RETX] ext-TX success retid:%i token:%lu: slot reused, dropped\n",
                      slot, (unsigned long)token);
        return false;
    }

    if(extradio::extTxRetransmittable(pre_status, ringBuffer[slot][2],
                                      RING_STATUS_READY, MSG_TYPE_TEXT))
    {
        // Retransmittable: emulate doTX()'s post-send state. Enter SENT so
        // updateRetransmissionStatus() ages/retries it and incoming ACK handling
        // clears it. Payload, length, message identity and retryCount are all
        // retained (the EXT_PENDING slot never cleared its length).
        ringBuffer[slot][1] = RING_STATUS_SENT;
        extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);  // RF success: clear this slot's busy episode
        if(bDisplayRetx)
            printfdeb("\n[RETX] ext-TX RF-sent retid:%i token:%lu -> awaiting ACK/retransmit\n",
                      slot, (unsigned long)token);
    }
    else
    {
        // One-shot (non-text, or relay/ACK enqueued DONE): native completion.
        ringBuffer[slot][1] = RING_STATUS_DONE;
        ringBuffer[slot][0] = 0;
        retryCount[slot] = 0;
        extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);  // RF success: clear this slot's busy episode
        if(bDisplayRetx)
            printfdeb("\n[RETX] ext-TX RF-sent retid:%i token:%lu (one-shot, complete)\n",
                      slot, (unsigned long)token);
    }
    return true;
}

// CHANNEL_BUSY: channel access was not granted (NOT an RF send and NOT a consumed
// MeshCom delivery retry). Uses a bounded, separate channel-access budget
// (g_extBusy / EXT_BUSY_MAX_ATTEMPTS) keyed by message identity. Within budget the
// frame returns to READY for a LATER paced reselect (retryCount untouched);
// exhausting the budget is a deliberate non-success terminal. Returns true if the
// owned message was acted on (the glue then arms the real pacing delay), false if
// the result was stale/late (ring untouched).
bool externalTxResolveChannelBusy(uint32_t token)
{
    int      slot = g_extTxq.slot();
    uint32_t mid  = g_extTxq.msgId();

    if(g_extTxq.resolveBusy(token) != extradio::ExtTxAction::REQUEUE_RETRY)
        return false;   // stale/late result: ring untouched, no pacing

    if(slot < 0 || slot >= MAX_RING ||
       !extradio::extTxOwnsRingSlot(ringBuffer[slot][1], extractRingMsgId(slot),
                                    RING_STATUS_EXT_PENDING, mid))
    {
        // Slot no longer holds the owned message (un-notified clear): release the
        // busy episode, touch no ring, and do not arm pacing.
        extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);
        return false;
    }

    if(extradio::extBusyOnBusy(g_extBusy, MAX_RING, slot, mid, EXT_BUSY_MAX_ATTEMPTS)
       == extradio::ExtBusyResult::RETRY)
    {
        // Content intact; re-selectable on a LATER pass once the pacing deadline
        // (armed by the glue) elapses. The MeshCom delivery retryCount is NOT
        // touched — channel access is not message delivery. This slot's episode
        // counter advances independently of any other interleaved message.
        ringBuffer[slot][1] = RING_STATUS_READY;
        if(bDisplayRetx)
            printfdeb("\n[RETX] ext-TX busy retid:%i channel-access attempt:%d/%d (retryCount kept)\n",
                      slot, (slot >= 0 && slot < MAX_RING) ? g_extBusy[slot].attempts : 0,
                      EXT_BUSY_MAX_ATTEMPTS);
        return true;
    }

    // Bounded channel-access budget exhausted: deliberate non-success terminal.
    extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);
    ringBuffer[slot][1] = RING_STATUS_DONE;
    ringBuffer[slot][0] = 0;
    retryCount[slot] = 0;
    if(bDisplayRetx)
        printfdeb("\n[RETX] ext-TX busy retid:%i channel-access budget (%d) exhausted, dropped\n",
                  slot, EXT_BUSY_MAX_ATTEMPTS);
    return true;
}

// UNKNOWN / TIMEOUT / RADIO_ERROR / disconnect / reconfigure: deliberate,
// observable non-success terminal. Never a resend, never confirmed success.
bool externalTxResolveUncertain(uint32_t token)
{
    int slot = g_extTxq.slot();
    uint32_t mid = g_extTxq.msgId();
    if(g_extTxq.resolveUncertain(token) != extradio::ExtTxAction::RELEASE_TERMINAL)
        return false;   // stale/late result: ring untouched

    if(slot < 0 || slot >= MAX_RING ||
       !extradio::extTxOwnsRingSlot(ringBuffer[slot][1], extractRingMsgId(slot),
                                    RING_STATUS_EXT_PENDING, mid))
    {
        // Slot no longer holds the owned message (un-notified clear): release the
        // busy episode, touch no ring.
        extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);
        return false;
    }

    ringBuffer[slot][1] = RING_STATUS_DONE;
    ringBuffer[slot][0] = 0;
    retryCount[slot] = 0;
    extradio::extBusyResetSlot(g_extBusy, MAX_RING, slot);  // terminal: clear this slot's busy episode

    if(bDisplayRetx)
        printfdeb("\n[RETX] ext-TX uncertain retid:%i token:%lu released (no resend)\n",
                  slot, (unsigned long)token);
    return true;
}

bool externalTxPending(void)
{
    return g_extTxq.active();
}

uint32_t externalTxMarkPendingNext(uint8_t *out, uint16_t out_cap, uint16_t *out_len)
{
    if(out == nullptr || out_len == nullptr)
        return 0;

    int slot = getNextTxSlot();
    if(slot < 0)
        return 0;   // nothing eligible

    // Exact same payload selection as the local doTX() path: bytes start at
    // ringBuffer[slot]+2, length is ringBuffer[slot][0] clamped to the buffer.
    int sendlng = ringBuffer[slot][0];
    if(sendlng <= 0)
        return 0;
    if(sendlng >= UDP_TX_BUF_SIZE)
        sendlng = UDP_TX_BUF_SIZE - 1;
    if((uint16_t)sendlng > out_cap)
        return 0;   // caller buffer too small: do not truncate a frame

    uint32_t token = externalTxMarkPending(slot);
    if(token == 0)
        return 0;   // already pending (one external TX at a time)

    // Content is retained by the pending slot; copy it for the transport submit.
    memcpy(out, ringBuffer[slot] + 2, sendlng);
    *out_len = (uint16_t)sendlng;
    return token;
}

unsigned long externalTxBusyBackoffMs(void)
{
    // Reuse the existing CSMA backoff timing (priority-aware, bounded). attempt 0
    // gives a normal base+jitter delay; this is a true delay before the next
    // external submission, not a same-pass reselect.
    return csma_compute_timeout(0);
}
#endif  // EXTERNAL_RADIO

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
    {
        unsigned long _tx_s = ch_util_tx_start.exchange(0);
        if(_tx_s > 0)
            ch_util_tx_accum.fetch_add(millis() - _tx_s);
    }

    if(bLORADEBUG)
        printfdeb("OnTXDone\n");

    #if defined BOARD_RAK4630

        // reset MeshCom
        if(bSetLoRaAPRS)
        {
            // SPI guard: defer if Ethernet owns the shared SPI bus
            if(!bSPI_ETH_Active) {
                lora_setchip_meshcom();
                bSetLoRaAPRS = false;
            }
        }

        // SPI guard: defer Radio.Rx() if Ethernet (W5100S) owns the shared SPI bus
        if(bSPI_ETH_Active) {
            bPendingRadioRx = true;
        } else {
            startRadioReceive();
        }
        iReceiveTimeOutTime = millis();  // force full CSMA timeout before next TX
        csma_reset();

        if(bLORADEBUG)
        {
            printfdeb("[MC-SM] TX_ACTIVE -> TX_DONE rc=0\n");
            printfdeb("[MC-SM] TX_DONE -> RX_LISTEN rc=0\n");
        }

    #endif

    tx_is_active = false;
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
    {
        unsigned long _tx_s = ch_util_tx_start.exchange(0);
        if(_tx_s > 0)
            ch_util_tx_accum.fetch_add(millis() - _tx_s);
    }

    if(bLORADEBUG)
        printfdeb("OnTXTimeout\n");

    #if defined BOARD_RAK4630

        // reset MeshCom
        if(bSetLoRaAPRS)
        {
            lora_setchip_meshcom();
            bSetLoRaAPRS = false;
        }

        startRadioReceive();

        // Der Semtech-Treiber ruft OnTxDone() nur im Erfolgsfall auf, der
        // Fehlerfall landet hier. Ohne diese beiden Zeilen endete die
        // MC-SM-Spur eines fehlgeschlagenen Sendevorgangs bei TX_ACTIVE und
        // wurde nie geschlossen -- eine Auswertung, die Zustandsuebergaenge
        // paart, haengt dann fuer immer im Sendezustand. rc=-1 entspricht dem
        // ESP32, der bei Fehler TX_DONE mit einem transmissionState != 0 meldet.
        if(bLORADEBUG)
        {
            printfdeb("[MC-SM] TX_ACTIVE -> TX_DONE rc=-1\n");
            printfdeb("[MC-SM] TX_DONE -> RX_LISTEN rc=0\n");
        }

    #endif

    tx_is_active = false;
}

/**@brief fires when a preamble is detected 
 * currently not used!
 */
void OnPreambleDetect(void)
{
    printfdeb("OnPreambleDetect\n");
}

/**@brief fires when a header is detected 
 */
void OnHeaderDetect(void)
{
    // Block TX during active reception.
    is_receiving = true;
    ch_util_rx_start = millis();

    // Debug L: HDR_DETECT with state context
    if(bLORADEBUG)
    {
        printfdeb("[MC-SM] RX_LISTEN -> RX_PROCESS rc=0\n");
        printfdeb("[MC-DBG] HDR_DETECT cad_attempt=%d\n", cad_attempt);
    }
}

// Nachbarschaftsmatrix Stufe 2 (docs/nbr-wichtigkeit-konzept.md 5.1): fallab-
// haengiger CSMA-Backoff fuer einen Relay-Slot, wirksam nur unter
// --nbrrelay on (bNBRCANCEL) -- --nbrrelay count rechnet die Masken und
// zaehlt, veraendert aber keine Funkzeitwerte (Verdict M1). Ohne
// bNBRCANCEL oder fuer jeden Nicht-Relay-Slot ist dies byte-identisch zu
// csma_compute_timeout_prio(attempt, ringPriority[slot]), das unveraendert
// bleibt und von test_inject.cpp o.ae. weiter direkt aufrufbar ist.
//
// Fall A/B selbst (Slot-Wahl in getNextTxSlot() + Backoff-Zahlen) sind nach
// txring_functions.cpp ausgelagert (txringInCaseBHold()/txringCaseBackoffSlot(),
// siehe dortige Kommentare fuer den Feldlauf-23.09.-Befund zum re-armten
// Fall-B-Hold: 149 verworfene Relays + 10 eigene HN-Meldungen in 9h), damit
// dieser Feld-bewiesene Code nativ (env:native_aprs, ohne Hardware) testbar
// ist -- txring_functions.cpp bleibt dabei absichtlich frei von
// lora_functions.cpp (dessen csma_compute_timeout_prio() braucht sie
// deshalb hier, nicht dort, siehe die beiden Sonderfaelle unten).
//
// Diese Funktion bleibt nur noch ein duenner Caller: Fruehausstieg (Rapid-
// fire), Vorbedingungspruefung (bNBRCANCEL + RING_KIND_RELAY), der
// Fall-B-Text-Sonderfall (unveraendert bei der normalen Prio-Basis, Konzept
// 5.1 -- Menschen warten darauf) sowie der generische Fallback rufen
// csma_compute_timeout_prio() direkt; alles andere delegiert an
// txringCaseBackoffSlot().
unsigned long csma_compute_timeout_slot(int attempt, int slot) {
    if(attempt >= CSMA_MAX_ATTEMPTS)
        return CSMA_RAPID_RX_MS; // rapid-fire with preamble check, wie csma_compute_timeout_prio()

    uint8_t prio = (slot >= 0) ? ringPriority[slot] : MSG_PRIO_NORMAL;

    if(bNBRCANCEL && slot >= 0 && (ringKind[slot] & 0x7F) == RING_KIND_RELAY)
    {
        // Fall B, Text: komplett bei der heutigen Basis UND heutigen Slots
        // bleiben (Konzept 5.1); Fall A hat keinen Text-Sonderfall.
        if(nbrMaskEmpty(ringAlone[slot]) && ringBuffer[slot][2] == MSG_TYPE_TEXT)
            return csma_compute_timeout_prio(attempt, prio);

        return txringCaseBackoffSlot(slot, attempt, (uint32_t)millis());
    }

    return csma_compute_timeout_prio(attempt, prio);
}

unsigned long csma_compute_timeout(int attempt) {
    // Default (no priority context): use priority of next queued packet
    int txSlot = getNextTxSlot();
    return csma_compute_timeout_slot(attempt, txSlot);
}

unsigned long csma_compute_timeout_prio(int attempt, uint8_t priority) {
    if(attempt >= CSMA_MAX_ATTEMPTS)
        return CSMA_RAPID_RX_MS; // rapid-fire with preamble check

    // Priority-dependent base timeout and slot range
    unsigned long base;
    int slots;
    switch(priority) {
        case MSG_PRIO_CRITICAL:   base = CSMA_PRIO_BASE_1; slots = CSMA_PRIO_SLOTS_1; break;
        case MSG_PRIO_HIGH:       base = CSMA_PRIO_BASE_2; slots = CSMA_PRIO_SLOTS_2; break;
        case MSG_PRIO_NORMAL:     base = CSMA_PRIO_BASE_3; slots = CSMA_PRIO_SLOTS_3; break;
        case MSG_PRIO_LOW:        base = CSMA_PRIO_BASE_4; slots = CSMA_PRIO_SLOTS_4; break;
        case MSG_PRIO_BACKGROUND: base = CSMA_PRIO_BASE_5; slots = CSMA_PRIO_SLOTS_5; break;
        default:                  base = CSMA_PRIO_BASE_3; slots = CSMA_PRIO_SLOTS_3; break;
    }

    // Reduce base on retries (keep priority differentiation)
    if(attempt >= 2) base = base * 2 / 3;      // ~33% reduction on 3rd attempt
    else if(attempt >= 1) base = base * 5 / 6;  // ~17% reduction on 2nd attempt

    return base + (unsigned long)random(0, slots + 1) * CSMA_SLOT_SIZE;
}

void csma_reset(void) {
    // Track max CAD attempts before resetting
    if(cad_attempt > stat_csma_hwm_attempts)
        stat_csma_hwm_attempts = cad_attempt;

    cad_attempt = 0;
    csma_timeout = csma_compute_timeout(0);
}