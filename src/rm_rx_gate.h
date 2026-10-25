// rm_rx_gate.h -- the one receive hook for remote-management DMs ("RM1 <ctr> ...", RM-04, #1189).
//
// Called for a DM to the exact own call, with the "{NNN" suffix already stripped, from every ingress:
// OnRxDone (LoRa) and the server ingress of a gateway node (udp_frame_esp32.cpp / udp_frame_nrf52.cpp).
//
// Since 2026-10-06 (RM-GWRELAY, operator decision) a COMMAND is accepted from either path, like a reply
// always was: the HMAC tag and the counter authenticate it wherever it travelled. The former "LoRa only"
// rule (RM-D6) keyed on the server flag, which a gateway also sets on every frame it relays over RF, so a
// node behind a gateway could not be managed at all. A frame that arrives on both paths is harmless: the
// second copy has the same counter and tag and gets the cached reply (rmCheck, RM_CACHED), nothing runs twice.
//
// Since 2026-10-07 (RM-DUP) the second copy of the SAME FRAME (same source, same message id, within 60 s) is
// (same text too) dropped here, before the queue: it is consumed and nothing else happens. Otherwise a wrong tag arriving
// on both paths was counted twice and one junk frame cost two strikes of the lockout.
//
// Header-only; needs rmIsReply() (remote_cmd.cpp) and the queues of rm_queue.h.
#ifndef RM_RX_GATE_H
#define RM_RX_GATE_H

#include <string.h>

#include "remote_cmd.h"
#include "rm_client_cmd.h"
#include "rm_queue.h"

// RM switch on and a node password set (an empty or blank password switches RM off, concept 6.2)
inline bool rmRxEnabled(int nodeRm, const char *passwd)
{
    return nodeRm == 1 && passwd != nullptr && passwd[0] != 0x00 && passwd[0] != ' ';
}

// true = consumed: the caller must neither display the DM nor hand it to the phone. A full queue also
// counts as consumed (command dropped, the sender repeats it with a fresh counter); verify, rate limit
// and lockout are rmDrain()'s, in the loop task.
//  - a REPLY ("RM1 <ctr> ok ..." / "RM1 <ctr> err ...") is for the operator to read: never consumed, and
//    queued only while a command still waits for its reply: one this node sent with its own key
//    (rmqReplyWanted()) or one a client handed to sendMessage() (rm_client_cmd.h). The latter has no key:
//    the loop task only uses the reply to stop that command's repeats.
//  - a COMMAND with RM off or no password is ordinary text
//  - msgId/nowMs: message id of the frame (0 = unknown) and millis(), for the duplicate ring (rm_queue.h)
inline bool rmRxTryQueue(const char *src, const char *text, bool enabled, uint32_t msgId, uint32_t nowMs)
{
    if (src == nullptr || text == nullptr || strncmp(text, "RM1 ", 4) != 0)
        return false;

    if (rmIsReply(text))
    {
        if (rmqReplyWanted() || rmClientReplyWanted(nowMs))
            rmReplyPush(src, text);
        return false;
    }

    if (!enabled)
        return false;

    if (rmqSeenBefore(src, msgId, text, nowMs))
        return true; // same frame on the other path: already handled

    rmQueuePush(src, text);
    return true;
}

#endif // RM_RX_GATE_H
