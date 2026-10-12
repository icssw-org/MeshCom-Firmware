// hey_echo_upload.h -- decision helper for the gateway upload of own HEY echoes.
//
// A gateway's own HEY ('@' to "HG") comes back relayed by neighbours (with their
// signal-report group appended). checkOwnTx() hits, OnRxDone takes the own-echo
// branch; this predicate decides whether that echo goes to the server. See
// docs/hey-echo-upload-concept.md.
//
// Own-origin rule: heardIcheck >= 0 does NOT mean "this frame is ours".
// own_msg_id also holds the msg_id of FOREIGN HEYs a gateway forwarded from the
// server to LoRa (insertOwnTx() in the udp_frame_* paths). When a neighbour
// relays such a foreign HEY, the gateway lands in the own-echo branch too, and
// must not upload it. So the frame's originator (msg_source_call) has to be
// this node's own call.
//
// Platform-neutral on purpose: compiled into the native suite.
#pragma once

#include <stdint.h>
#include <string.h>

// true = upload this own-HEY echo to the server.
//   msg_type / msg_type_hey   decoded frame type and MSG_TYPE_HEY
//   payload_type              must be '@' (HEY)
//   pn_repeat                 PN repeat copy (text-only; defensive)
//   last_hop                  an echo always has a foreign last hop
//   source_call               originator of the frame; must equal own_call
//                             (exact compare, null/empty -> false)
//   own_call                  this node's call; null -> false (an own-origin
//                             check against no call cannot pass)
inline bool heyEchoShouldUpload(bool is_gateway, uint8_t msg_type, uint8_t msg_type_hey,
                                char payload_type, bool pn_repeat, const char* last_hop,
                                const char* source_call, const char* own_call)
{
    if(!is_gateway || msg_type != msg_type_hey || payload_type != '@' || pn_repeat)
        return false;
    if(own_call == nullptr || own_call[0] == '\0')
        return false;
    if(source_call == nullptr || source_call[0] == '\0' || strcmp(source_call, own_call) != 0)
        return false;
    if(last_hop == nullptr || last_hop[0] == '\0')
        return false;
    if(strcmp(last_hop, own_call) == 0)
        return false;
    return true;
}
