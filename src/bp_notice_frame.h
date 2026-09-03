#ifndef _BP_NOTICE_FRAME_H_
#define _BP_NOTICE_FRAME_H_

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <aprs_functions.h>
#include <charset_filter.h>

// BP-01: the notice frame for the phone app / web GUI, filled here rather
// than inline in bpNoticeToPhone() (loop_functions.cpp) so the framing is
// native-testable (test/test_bp_notice_frame).
//
// Sender is the node's own callsign: a pseudo-sender ("response", the
// addBLECommandBack() framing) is not a valid call and lands in McApp's
// spam class (group 9999), where the operator never sees the notice
// (operator decision 2026-08-31).
//
// BP-06: the destination is now the same target the triggering message was
// sent to (a group, a DM call, or "*"), not a hardcoded broadcast -- a
// notice for a message the operator typed into group 20 should show up in
// the 20 chat, not vanish into "*". msg_app_offline still keeps the frame
// local: for a DM dst this makes the notice appear in the sender's own DM
// thread, but the frame is never announced, never retransmitted and never
// goes on the air, so the DM partner never sees it.
static inline void bpNoticeFillFrame(struct aprsMessage &aprsmsg,
                                     const char *node_call,
                                     const char *text,
                                     unsigned int msg_id,
                                     const char *dst)
{
    initAPRS(aprsmsg, ':');

    aprsmsg.msg_len = 0;
    aprsmsg.payload_type = ':';
    aprsmsg.msg_id = msg_id;
    aprsmsg.msg_destination_path = dst;
    aprsmsg.msg_destination_call = dst;
    aprsmsg.msg_source_path = node_call;
    aprsmsg.msg_payload = text;

    aprsmsg.msg_app_offline = true; // Rückmeldungen niemals announcen
}

// BP-07: bytes, not characters -- see the length budget table in
// docs/bp-l1-l4-impl-plan.md (section "src/bp_notice_frame.h"). Chosen so the
// EXTUDP path (the tightest: 400-byte c_json, ~141-byte JSON skeleton with
// the longest possible callsign/dst) still has three-digit headroom, while
// BLE/Web (255-byte UDP_TX_BUF_SIZE) stay comfortably within budget too.
#define BP_NACK_TEXT_MAX 120

// BP-07 (L1/L2): compose the "<prefix><text>" body of a per-message nack --
// "QRT NOT SENT - Hello World 17" -- for the transport that just had a
// message refused or dropped. prefix is always one of the two hardcoded
// literals from bpNackPrefix() (backpressure.h) and is copied verbatim;
// text is the operator's own message and is truncated and sanitized:
//
//  1. Never cuts mid UTF-8 sequence. text arrives already decoded (after the
//     %-escape loop in sendMessage()), so umlauts and emoji are real
//     multi-byte sequences; truncating at a fixed byte count can land inside
//     one. Fix (M2): charset_utf8_safe_truncate() (charset_filter.h) walks
//     FORWARD off whole lead-byte sequences to find the cut point. An
//     earlier hand-rolled version walked BACKWARD from the cut point over
//     continuation bytes (10xxxxxx, 0x80..0xBF) instead -- on a run of stray
//     continuation bytes (never a valid lead, so the loop never found one to
//     stop at) that walked all the way to 0 and dropped the whole text.
//  2. Replaces '"', '\' and every byte < 0x20 with a space. The EXTUDP path
//     embeds this text in JSON; without this rule a text full of quotes
//     could double in length when ArduinoJson escapes it and blow the
//     datagram buffer. This keeps the byte count 1:1 through serialization.
//     Unrelated to rule 1 (JSON escaping, not UTF-8 validity) and applied
//     after the cut, to the bytes that survive it.
//
// out_len-safe: writes at most out_len-1 bytes plus a NUL, even if out_len is
// smaller than the prefix (defensive only -- every real caller sizes its
// buffer for prefix + BP_NACK_TEXT_MAX + "...", see the plan's budget table).
// Returns the number of bytes written, excluding the NUL; 0 if out/out_len
// is invalid.
static inline size_t bpNackCompose(char *out, size_t out_len,
                                   const char *prefix, const char *text)
{
    if(out == nullptr || out_len == 0)
        return 0;

    out[0] = '\0';

    if(prefix == nullptr)
        prefix = "";
    if(text == nullptr)
        text = "";

    size_t room = out_len - 1;   // bytes available before the terminating NUL

    // 1) Prefix, verbatim -- never sanitized, it is not operator text.
    size_t prefix_len = strlen(prefix);
    if(prefix_len > room)
        prefix_len = room;
    memcpy(out, prefix, prefix_len);
    size_t written = prefix_len;
    room -= prefix_len;

    // 2) Text: BP_NACK_TEXT_MAX is the length budget, room (whatever the
    // caller's buffer has left after the prefix -- see out_len-safe above)
    // is the harder limit in a pathologically small out_len. The smaller of
    // the two is the tentative cut point.
    size_t text_len = strlen(text);
    size_t cap = (text_len > BP_NACK_TEXT_MAX) ? BP_NACK_TEXT_MAX : text_len;
    if(cap > room)
        cap = room;

    // M2: forward, UTF-8-safe cut at that tentative point -- see the rule 1
    // comment above for why this replaced the old backward walk.
    size_t take = charset_utf8_safe_truncate(text, text_len, cap);

    // M3: decide whether the text was actually cut -- and therefore needs an
    // ellipsis -- only now, against the FINAL take, after both the
    // BP_NACK_TEXT_MAX budget and the room clamp have had their say. Basing
    // this on text_len > BP_NACK_TEXT_MAX alone (the old bug) missed a cut
    // forced purely by a small out_len: room shortened take, but truncated
    // stayed false, so "..." never got appended even though bytes were lost.
    bool truncated = take < text_len;
    size_t suffix_len = 0;
    if(truncated)
    {
        if(room >= take + 3)
        {
            suffix_len = 3;
        }
        else if(room >= 3)
        {
            // Room for "..." but not for take bytes AND "..." together --
            // shrink take (back onto a UTF-8 boundary again, not a raw byte
            // cut) rather than silently drop the ellipsis.
            take = charset_utf8_safe_truncate(text, text_len, room - 3);
            suffix_len = 3;
        }
        // else: room < 3, no space even for "..." alone -- take already
        // fits room from the cap clamp above, the ellipsis just stays off.
    }

    for(size_t i = 0; i < take; i++)
    {
        char c = text[i];
        if(c == '"' || c == '\\' || (unsigned char)c < 0x20)
            c = ' ';
        out[written++] = c;
    }
    room -= take;

    if(suffix_len == 3 && room >= 3)
    {
        out[written++] = '.';
        out[written++] = '.';
        out[written++] = '.';
    }

    out[written] = '\0';
    return written;
}

#endif // _BP_NOTICE_FRAME_H_
