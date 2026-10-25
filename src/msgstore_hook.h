#pragma once

// SNF-GW-01: the store-node
// receive decision, in ONE place. OnRxDone (RF) and both GATE handlers (server
// ingress) ask the same question about a text frame addressed to someone
// else: is it an ack that purges a held DM, a DM that belongs in the mailbox,
// or neither? Keeping the answer here keeps the three call sites from drifting.
//
// Header-only and Arduino-free so it is checkable on the host
//. The caller supplies the
// facts that need firmware state (group lookup, PN-repeat, peer-delivery path
// shape, store-set eligibility); the helper only decides.
//
// The peer-delivery test itself (needs rly_hop) stays RF-only in OnRxDone.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mc_text.h"

enum MboxAction : uint8_t
{
    MBOX_NONE = 0,   // nothing to do
    MBOX_ACK,        // ":ackNNN" heard for someone else's DM -> msgstoreOnAck(src, dst, nnn)
    MBOX_STORE       // DM with "{NNN" for the store set -> msgstoreStore(src, dst, nnn, text[0..textLen))
};

struct MboxDecision
{
    MboxAction action;
    uint16_t   nnn;       // ACK: the acked number; STORE: the DM's own "{NNN" (0 for NONE)
    size_t     textLen;   // STORE: length of the text before "{NNN" (>= 1); 0 otherwise
};

/**
 * @brief True for the Winlink / SOTA APRS gateway calls, exact and case-sensitive
 *        ("WLNK-1", "APRS2SOTA"). NULL or empty: false.
 *
 * Mirrors the special calls in src/regex_functions.cpp (checkRegexCall), which
 * takes an Arduino String; this header must stay String-free.
 */
static inline bool mboxIsServiceCall(const char *call)
{
    if (call == 0 || call[0] == 0)
        return false;
    return strcmp(call, "WLNK-1") == 0 || strcmp(call, "APRS2SOTA") == 0;
}

/**
 * @brief Classifies a text frame for the store node.
 *
 * @param src          source call of the frame (aprsmsg.msg_source_call)
 * @param dst          destination call of the frame
 * @param payload      APRS text payload
 * @param isGroup      CheckGroup(dst) != 0 (caller computes)
 * @param peerDelivery frame is another store node's own hop-0 delivery (RF only)
 * @param pnRepeat     frame is a repeat XOR copy of a PN already seen
 * @param eligible     msgstoreEligible(dst) (caller computes; only consulted for STORE)
 *
 * ACK: ":ack" at index > 0. Deliberately no eligibility / group test -- an ack
 * heard for any DM may purge a matching held entry. Additionally, for dst
 * "WLNK-1" only, the whole payload "ack<digits>" (>= 1 digit, nothing else):
 * SendAckMessage acks the Winlink gateway in that bare form.
 * STORE: never for a service call (mboxIsServiceCall) as src or dst, not "*",
 * not a group, no ":rej" (index > 0), no "{" prefix (control
 * frames {ping}/{pong}/{MCP}/{SET}/{CET}), no "RM1 " prefix (remote-management
 * command DM, concept section 6), not a peer delivery, not a PN repeat,
 * eligible, and a "{NNN" at index >= 1.
 */
static inline MboxDecision mboxClassify(const char *src, const char *dst, const char *payload,
                                         bool isGroup, bool peerDelivery,
                                         bool pnRepeat, bool eligible)
{
    MboxDecision d = { MBOX_NONE, 0, 0 };
    if (dst == 0 || payload == 0)
        return d;

    size_t plen = strlen(payload);

    int ackPos = mcIndexOfStr(payload, ":ack");
    if (ackPos > 0)
    {
        d.action = MBOX_ACK;
        d.nnn = (uint16_t)mcSliceToLong(payload, (size_t)(ackPos + 4), plen);
        return d;
    }

    if (strcmp(dst, "WLNK-1") == 0 && mcStartsWith(payload, "ack") && plen > 3)
    {
        size_t i = 3;
        while (i < plen && payload[i] >= '0' && payload[i] <= '9')
            i++;
        if (i == plen)
        {
            d.action = MBOX_ACK;
            d.nnn = (uint16_t)mcSliceToLong(payload, 3, plen);
            return d;
        }
    }

    if (mboxIsServiceCall(src) ||
        mboxIsServiceCall(dst) ||
        strcmp(dst, "*") == 0 ||
        isGroup ||
        mcIndexOfStr(payload, ":rej") > 0 ||
        mcStartsWith(payload, "{") ||
        mcStartsWith(payload, "RM1 ") ||
        peerDelivery ||
        pnRepeat ||
        !eligible)
        return d;

    int enqPos = mcIndexOfStrFrom(payload, "{", 1);
    if (enqPos > 0)
    {
        d.action = MBOX_STORE;
        d.nnn = (uint16_t)mcSliceToLong(payload, (size_t)(enqPos + 1), plen);
        d.textLen = (size_t)enqPos;
    }
    return d;
}

/**
 * @brief True if the comma-separated source path holds @p call as a whole
 *        element (case-sensitive, SSID included). "DK5EN-1" is not found in
 *        "DK5EN-12". Empty path or empty call: false.
 */
static inline bool mboxPathHasCall(const char *path, const char *call)
{
    if (path == 0 || call == 0 || call[0] == 0)
        return false;

    size_t cl = strlen(call);
    const char *p = path;
    while (*p)
    {
        const char *end = strchr(p, ',');
        size_t el = end ? (size_t)(end - p) : strlen(p);
        if (el == cl && memcmp(p, call, cl) == 0)
            return true;
        if (!end)
            break;
        p = end + 1;
    }
    return false;
}
