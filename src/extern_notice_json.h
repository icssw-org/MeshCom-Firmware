#ifndef _EXTERN_NOTICE_JSON_H_
#define _EXTERN_NOTICE_JSON_H_

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <ArduinoJson.h>

// BP-01: the EXTUDP notice datagram, built here rather than inline in
// sendExternNotice() (extudp_functions.cpp) so the JSON contract is
// native-testable (test/test_extern_notice_json).
//
// Framed exactly like a text message received over LoRa from the node's own
// callsign, mirroring what sendExtern() emits for a frame off the air —
// including the numeric firmware version (35, not the "4.35" string the
// "node" shape carries). McApp renders only the message shapes it knows; a
// distinct "notice" type (the first cut of this path) was invisible to it
// (operator decision 2026-08-31).
//
// BP-06: "dst" is the target of the message that triggered the notice (a
// group, a DM call, or "*"), not a hardcoded broadcast -- the notice reaches
// the same peer view the triggering message was headed for. This datagram
// never goes on the air regardless of dst, so a DM dst is safe: the DM
// partner never sees it, only the sender's own EXTUDP peer does.
//
// Rueckgabe: Anzahl geschriebener Bytes; die Schranke ist die Puffergroesse,
// nicht measureJson() (JSN-01, siehe ble_json_frame.h).
static inline size_t externNoticeJson(char *out, size_t out_len,
                                      const char *node_call,
                                      uint8_t fw_version,
                                      const char *fw_sub,
                                      unsigned int msg_id,
                                      const char *text,
                                      const char *dst)
{
    if(out == nullptr || out_len == 0)
        return 0;

    char c_msg_id[9];
    snprintf(c_msg_id, sizeof(c_msg_id), "%08X", msg_id);

    JsonDocument cJson;

    cJson["src_type"] = "lora";
    cJson["type"] = "msg";
    cJson["src"] = node_call;
    cJson["dst"] = dst;
    // JSN-01: assign raw -- ArduinoJson escapes JSON strings on
    // serializeJson() already.
    cJson["msg"] = text;
    cJson["msg_id"] = c_msg_id;
    cJson["firmware"] = fw_version;
    cJson["fw_sub"] = fw_sub;
    cJson["rssi"] = 0;
    cJson["snr"] = 0;

    return serializeJson(cJson, out, out_len);
}

#endif // _EXTERN_NOTICE_JSON_H_
