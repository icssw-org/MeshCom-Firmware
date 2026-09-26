#pragma once

// Carve for the U1 twin (test plan section 4.3, `test_udp_frame_twin`): the
// inbound UDP frame handler, one file per platform.
//
// THIS HEADER DECLARES NOTHING ON PURPOSE. Both handlers are already declared
// where their callers look for them -- handleUdpFrame_esp32() in
// udp_functions.h, handleUdpFrame_nrf52() in nrf52/nrf_eth.h, both established
// by C1 and unchanged by this move. Re-declaring them here would add a third
// spelling of a signature the campaign exists to stop having several of. What
// this file is for is the one thing that had no home: why the move happened
// and what the two copies do differently.
//
// WHY THE MOVE. C1 gave the two handlers the same signature and lifted the
// socket out of them, which is what the plan's C1 row asked for. It was not
// enough to test them -- for the third time in this campaign, and each time
// for the same reason:
//
//   C1/U1  bodies stayed in udp_functions.cpp (web_functions.h, ArduinoJson)
//          and nrf_eth.cpp (SPI, RAK13800_W5100S)
//   C2/U2  drains stayed in udp_functions.cpp and nrf52_main.cpp
//   C3/U3  checkSerialCommand() stayed in the two largest TUs in the tree
//
// None of those TUs compiles on a host, so no amount of signature matching
// made the function linkable into a native binary. All three rows named the
// call as the obstacle when the obstacle was the translation unit. Recorded
// here rather than in a commit message because the same trap is still open
// for every uncarved row of the plan.
//
// WHAT THE TWO COPIES DO DIFFERENTLY. All four were open drift-matrix
// questions as of the M2 review; all four are now DECIDED (2026-09-12,
// docs/testplan/drift-matrix.csv, drift-matrix-review-verdict-20260912.md).
// STAND 2026-09-17 (Welle W6): DR-02, DR-05, DR-18 und DR-20 sind
// IMPLEMENTIERT, zusammen mit DR-04, DR-06, DR-07, DR-08, DR-09 und DR-19 --
// jede Zeile traegt jetzt einen test_agreement_*-Fall in
// test/test_udp_frame_twin/test_udp_frame_twin.cpp statt eines
// test_drift_*-Falls. DR-20 war zuletzt offen, weil ihre Entscheidung
// verlangt, dass der AUFRUFER die Verbindungspolitik entscheidet -- dafuer
// mussten sich udp_functions.h und udp_functions.cpp mitaendern:
// handleUdpFrame_esp32() gibt jetzt int statt void zurueck (0 behandelt, 1
// zu viele Nullen), und getMeshComUDP() (udp_functions.cpp) ruft
// resetMeshComUDP() bei 1 auf -- der Reset selbst ist unveraendert, nur WER
// ihn ausloest zog eine Ebene hoeher, spiegelbildlich zu NrfETH::getUDP(),
// das bei 1 resetDHCP() ruft (nrf52/nrf_eth.cpp:450-453). Damit hat der
// C1-Carve-Kommentarblock (unten) keine offene Zeile mehr.
//
// DR-18 ist jetzt vollstaendig umgesetzt: TEIL (1), die FORMPARITAET (der
// EXTUDP-Abzweig liegt auf beiden Plattformen in einem eigenen Typtest vor
// is_new_packet()), stand schon; TEIL (2), das JSON-Ack-Statusdatagramm
// ({"type":"ack",...} ueber eine eigene Warteschlangen-Funktion, NICHT durch
// sendExtern()'s Typ-Weiche), ist an den DR-09-Stellen auf beiden Plattformen
// ergaenzt (udp_frame_esp32.cpp, udp_frame_nrf52.cpp -- die Stellen, die
// buildAckPhoneFrame()/addBLEOutBuffer() fuer 0x41 rufen). Details und der
// nativ testbare Aufbau in src/extudp_functions.cpp (queueExternAck()) und
// docs/ack-wer-hat-quittiert.md Abschnitt 6.3. Der von der Zeile bestellte
// Test deckt jetzt je einen DEKODIERBAREN Frame pro Typ (0x3A/0x21/0x40) und
// den JSON-Ack-Fall ab (test_agreement_extudp_forwards_one_decodable_frame_
// per_type_plus_ack) -- vorher deckte nur 0x3A positiv ab, 0x21 und 0x40
// waren unbelegt (nachgewiesen: den Typtest auf beiden Seiten auf 0x3A zu
// verengen liess die Suite vorher gruen; jetzt schlaegt dieselbe Mutation
// fehl). Offen bleiben zwei Punkte, die nicht im Dateikreis dieser Welle
// lagen: der G2-EXPECTED-DIFF-Eintrag (test/golden/hw/G2/, Hardware-Lauf) und
// STAND 2026-09-17: EXT-01 ist IM CODE BEHOBEN (extudp_functions.cpp:650-670
// kehrt vor dem Sendeblock zurueck). Der Absatz darunter beschreibt den
// Zustand davor und bleibt als Beleg stehen; bench-bestaetigt ist der Fix
// noch nicht. docs/ack-wer-hat-quittiert.md Abschnitt 6.2 sagte das bereits,
// dieser Header widersprach ihm.
// EXT-01 (BACKLOG, eigener Defekt: ein Telemetrie-Textframe mit Zielpfad
// 100001 reisst den EXTUDP-Socket ab, siehe ack-wer-hat-quittiert.md 6.2).
//
//   - DR-20 (return value / reset call), IMPLEMENTED: ESP32 used to return
//     void and swallow the over-MAX_ZEROS case, calling resetMeshComUDP()
//     itself; nRF52 returns 1 for it and its caller (NrfETH::getUDP())
//     resets DHCP. DECIDED nrf52-correct on WHO decides: both handlers
//     return a status now -- the handler is a parser, not connectivity
//     policy. The reset ITSELF stays platform-specific (ESP32 resets the
//     UDP socket, nRF52 resets DHCP -- there is no shared answer to want);
//     only the caller-decides shape is unified. ESP32's caller is
//     getMeshComUDP() (udp_functions.cpp), mirroring NrfETH::getUDP() -- not
//     gatewayService_esp32().
//   - DR-02 (RX-01 unconfigured-source guard): ESP32 runs the guard on a
//     GATE frame before radiating it onto LoRa; nRF52 has no such guard.
//     DECIDED esp32-correct: port the guard to handleUdpFrame_nrf52() so an
//     unconfigured-source frame from an nRF52 gateway cannot defeat the
//     primary OnRxDone guard's purpose either.
//   - DR-05 (display paths / TM-31 shape): ESP32 reads the dedup gate BEFORE
//     the position branch inserts the msg_id (the documented TM-31 fix) and
//     also calls sendDisplayPosition() there; nRF52 never had the early
//     insert to begin with (the historical TM-31 bug does not reproduce on
//     either side today) and never calls sendDisplayPosition() at all
//     (only sendDisplayText()). DECIDED esp32-correct: port
//     sendDisplayPosition() to nRF52's position branch for display parity;
//     the TM-31 early-dedup difference is now a no-op and can be unified for
//     hygiene without behaviour risk, landing after DR-06's return-value
//     gate so a rejected frame doesn't reach it.
//   - DR-18 (EXTUDP forward), IMPLEMENTED (both parts): ESP32 used to forward
//     to EXTUDP only for recognised msg_type_b (0x3A/0x21/0x40) INSIDE the
//     relay branch; nRF52 forwarded unconditionally, BEFORE that type check.
//     DECIDED: shape parity, not a wider type set -- the EXTUDP forward is
//     now its own explicit type test on BOTH platforms (same 0x3A/0x21/0x40
//     set ESP32 already used), lifted out of the relay branch and kept ahead
//     of is_new_packet() so duplicates still reach EXTUDP as before. Landed
//     together with DR-07 (its outer hasExternIPaddress check is the guard
//     on the lifted call) and DR-19 (drops the (uint8_t) cast on the same
//     line). A widened set including ACK 0x41 in sendExtern()'s own type
//     switch was considered and stays WITHDRAWN -- but part (2), a SEPARATE
//     JSON-ack status datagram that does not touch sendExtern()'s switch, is
//     now implemented at the two buildAckPhoneFrame()/addBLEOutBuffer() call
//     sites (udp_frame_esp32.cpp, udp_frame_nrf52.cpp) via
//     queueExternAck() (extudp_functions.cpp). See
//     docs/ack-wer-hat-quittiert.md §6.3 for the contract this mirrors.
//
// The twin turns each of those into a failing-on-change test rather than a
// comment; see test/test_udp_frame_twin.

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include "ack_attribution.h"   // ackAttrCallLen() -- see buildExternAckJson() below

// DR-18 part 2 (docs/ack-wer-hat-quittiert.md §6.3): the outbound EXTUDP ack
// status datagram, {"type":"ack","msg_id":...,"status":...,"from":...,
// "via":...}. Declared HERE rather than in extudp_functions.h (its natural
// home) because that header is outside this carve's file set; both
// udp_frame_esp32.cpp and udp_frame_nrf52.cpp already include this one.
// Defined in extudp_functions.cpp, queued through the SAME ring buffer
// queueExtern() uses (flushed once per main loop pass, never sent from
// OnRxDone) -- an entry marked JSON is sent verbatim on flush instead of
// going through sendExtern()'s decodeAPRS()-based type switch, which is
// deliberately NOT widened for 0x41 (see the DR-18 bullet above; that
// widening was considered and withdrawn).
void queueExternAck(uint32_t msg_id, uint8_t status, const char *from, const char *via);

// Pure JSON builder queueExternAck() uses -- declared here, not in
// extudp_functions.cpp only, so it is also reachable (and native-pinnable)
// from this header's other two includers without pulling in ArduinoJson:
// native_udp_frame_twin (test/test_udp_frame_twin) has no ArduinoJson
// lib_dep, unlike native_extern, and platformio.ini is outside this carve's
// file set to extend. Hand-rolled rather than ArduinoJson for that reason --
// extern_notice_json.h's externNoticeJson() is the ArduinoJson-based sibling
// for a context (native_extern) that already links ArduinoJson. `from` is
// embedded only when ackAttrCallLen() accepts it (the same [A-Z0-9-] gate
// the wire attachment already uses in ack_attribution.h) -- cheap insurance
// against a JSON special character reaching this string unescaped, and it
// doubles as the doc's "from weggelassen wenn nicht bekannt" rule. Returns
// the number of bytes written (excluding the NUL), 0 if `out` is NULL, too
// small, or the encoding failed.
static inline size_t buildExternAckJson(char *out, size_t out_len, uint32_t msg_id,
                                        uint8_t status, const char *from, const char *via)
{
    if(out == nullptr || out_len == 0)
        return 0;

    char c_msg_id[9];
    snprintf(c_msg_id, sizeof(c_msg_id), "%08X", (unsigned)msg_id);   // uint32_t ist nicht ueberall unsigned int

    int n;
    // ADVISOR-BEFUND W6b (F5): `from` lief durch ackAttrCallLen() und war damit
    // auf [A-Z0-9-] begrenzt, `via` aber wurde ROH interpoliert. Nachgewiesen
    // durch Uebersetzen: via = u"dp ergibt {... "via":"u"dp"} -- kein gueltiges
    // JSON mehr, und der Empfaenger ist MCProxys normalize_extudp_ack().
    // Heute uebergeben beide Aufrufstellen das Literal "udp", der Fehler war
    // also latent; der Kommentar darueber behauptete aber Schutz, den es fuer
    // dieses Feld nicht gab. Jetzt gilt fuer via dieselbe Schranke wie fuer
    // from: was sie nicht passiert, wird weggelassen statt eingebettet.
    // ackAttrCallLen() ist die falsche Schranke fuer via: sie laesst nur
    // GROSSBUCHSTABEN durch, und beide Aufrufstellen uebergeben "udp" --
    // damit waere das Feld immer leer gewesen. via ist kein Rufzeichen,
    // sondern ein Transportname. Eigene Schranke: nur Buchstaben, Ziffern,
    // '-' und '_', hoechstens 12 Zeichen. Alles andere -- und damit jedes
    // JSON-Sonderzeichen -- fuehrt zum leeren Feld statt zu kaputtem JSON.
    char via_safe[13] = {0};
    if(via)
    {
        size_t vn = 0;
        for(; via[vn] && vn < sizeof(via_safe) - 1; vn++)
        {
            char c = via[vn];
            bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
                   || (c >= '0' && c <= '9') || c == '-' || c == '_';
            if(!ok) { vn = 0; break; }
            via_safe[vn] = c;
        }
        if(via[vn] != 0 && vn == sizeof(via_safe) - 1)
            vn = 0;                       // zu lang -> weglassen, nicht kuerzen
        via_safe[vn] = 0;
    }

    if(ackAttrCallLen(from) > 0)
        n = snprintf(out, out_len,
                     "{\"type\":\"ack\",\"msg_id\":\"%s\",\"status\":%u,\"from\":\"%s\",\"via\":\"%s\"}",
                     c_msg_id, (unsigned)status, from, via_safe);
    else
        n = snprintf(out, out_len,
                     "{\"type\":\"ack\",\"msg_id\":\"%s\",\"status\":%u,\"via\":\"%s\"}",
                     c_msg_id, (unsigned)status, via_safe);

    if(n <= 0 || (size_t)n >= out_len)
        return 0;

    return (size_t)n;
}
