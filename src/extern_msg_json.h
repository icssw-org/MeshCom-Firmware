#ifndef _EXTERN_MSG_JSON_H_
#define _EXTERN_MSG_JSON_H_

#include <stddef.h>
#include <stdint.h>
#include <ArduinoJson.h>

// Size of the c_json buffer in sendExtern() for the "pos"/"msg" datagram.
// The bound that matters is what the AIR can deliver, not what a peer may
// send into the node (the 150-char getExtern() cap is input-side only):
// encodeAPRS() caps "src>dst:payload" at 239 bytes (src/aprs_functions.cpp,
// UDP_TX_BUF_SIZE - 10 - 6-byte header) and decodeAPRS() accepts frames up
// to 255. The JSON is longest for a SHORT src/dst and a payload of nothing
// but '"' (or '\'), the only characters ArduinoJson escapes to two bytes
// after the CHR-01 control-character filter: 5-char src, dst "*", 231
// quotes, every numeric key at its widest -> 636 bytes
// (test_worst_case_air_frame_fits_buffer). The former 500 was already 8
// bytes short of the 54/9/150 case once the three new keys are on; 700
// keeps about 60 bytes of margin above the air bound. In diesem Baum liegen
// c_json und c_tjson auf beiden Plattformen in statischem BSS (siehe die
// Re-Entrance-Begruendung bei sendExtern()), der Puffer kostet also keinen
// Stack.
#define EXTERN_MSG_JSON_BUF 700

// Campaign docs/campaign-extudp-hwid.md: the EXTUDP "msg" (text) datagram,
// built here rather than inline in sendExtern() (extudp_functions.cpp) so the
// key contract -- three new keys on top of the pre-existing wire shape -- is
// native-testable (test/test_extern_msg_json). Same pattern as
// extern_tele_json.h / extern_notice_json.h.
//
// Key order is byte-for-byte the pre-campaign wire (see the golden "before"
// datagram in docs/campaign-extudp-hwid.md):
//   src_type, type("msg"), src, dst, msg, msg_id, firmware, fw_sub, rssi, snr
// with the three new keys APPENDED, never inserted:
//   hw_id, lora_mod, max_hop
//
// Provenance (docs/2026-09-16_firmware-extudp-hw-id-on-text-frames.md,
// sections 3-4): hw_id and lora_mod describe the message's ORIGINATOR, not
// the last relay hop -- they are parsed back out of the frame epilogue on
// every received frame (src/aprs_functions.cpp:421 msg_source_hw, :424
// msg_source_mod), exactly like msg_source_fw_version, which this shape
// already sends as "firmware". max_hop is the remaining hop budget carried
// by THIS copy of the frame (src/aprs_structures.h aprsMessage.max_hop).
//
// firmware asymmetry (preserved exactly, it is what the proxy already
// parses): "node" (own message) sends the version STRING ("4.35",
// SOURCE_VERSION); "lora" (received off the mesh) sends the version as an
// INTEGER (e.g. 35, aprsmsg.msg_source_fw_version). Pass firmware_str
// non-null for the "node" shape and nullptr for "lora" (then firmware_num is
// used). A single function with two firmware parameters was chosen over
// externMsgJsonNode()/externMsgJsonLora() because every OTHER parameter
// (src_type, src, dst, msg, msg_id, fw_sub, rssi, snr, hw_id, mod_byte,
// max_hop) is identical between the two shapes -- unlike the tele datagram,
// where "lora" carries extra fields ("batt", "pressure_alt") the "node"
// shape does not.
//
// lora_mod: mod_byte is the RAW packed byte (aprsMessage.msg_source_mod) --
// low nibble modulation (3..8), high nibble country index
// (src/aprs_functions.cpp, message creation). The helper masks with 0x0F
// itself so the key contract test proves the mask, matching the BLE copy
// MCProxy already stores (docs/2026-09-16_..., section 4 notes).
//
// Rueckgabe: Anzahl geschriebener Bytes; die Schranke ist die Puffergroesse,
// nicht measureJson() (JSN-01, siehe ble_json_frame.h).
static inline size_t externMsgJson(char *out, size_t out_len,
                                   const char *src_type,
                                   const char *src, const char *dst,
                                   const char *msg, const char *msg_id,
                                   const char *firmware_str, uint8_t firmware_num,
                                   const char *fw_sub,
                                   int16_t rssi, int8_t snr,
                                   uint8_t hw_id, uint8_t mod_byte,
                                   uint8_t max_hop)
{
    if(out == nullptr || out_len == 0)
        return 0;

    JsonDocument cJson;

    cJson["src_type"] = src_type;
    cJson["type"] = "msg";
    cJson["src"] = src;
    cJson["dst"] = dst;
    // JSN-01: assign raw -- ArduinoJson escapes JSON strings on
    // serializeJson() already; a separate escaper here would double-escape.
    cJson["msg"] = msg;
    cJson["msg_id"] = msg_id;

    if(firmware_str != nullptr)
        cJson["firmware"] = firmware_str;
    else
        cJson["firmware"] = firmware_num;

    cJson["fw_sub"] = fw_sub;
    cJson["rssi"] = rssi;
    cJson["snr"] = snr;

    cJson["hw_id"] = hw_id;
    cJson["lora_mod"] = mod_byte & 0x0F;
    cJson["max_hop"] = max_hop;

    return serializeJson(cJson, out, out_len);
}

#endif // _EXTERN_MSG_JSON_H_
