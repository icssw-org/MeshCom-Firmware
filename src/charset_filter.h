/**
 * CHR-01 / CHR-02 (docs/BACKLOG.md §3.8p) -- UTF-8 allowlist character
 * filter for message text and APRS free text.
 *
 * Pure C++, no Arduino/String dependency -- unit-tested natively
 * (test_charset_filter) and linked straight into aprs_functions.cpp /
 * loop_functions.cpp on every target.
 *
 * Policy (operator-confirmed):
 *   - printable ASCII 0x20-0x7E passes.
 *   - C0 controls (0x00-0x1F) and DEL (0x7F) are stripped.
 *   - C1 controls (U+0080-U+009F) are stripped.
 *   - invalid and overlong UTF-8 byte sequences are dropped. Validation is
 *     strict RFC 3629: rejects overlong forms, encoded surrogates
 *     (U+D800-U+DFFF) and any codepoint above U+10FFFF. A byte that cannot
 *     start or continue a valid sequence is dropped one byte at a time, so
 *     a run of invalid bytes never eats an adjacent valid character.
 *   - a small table of bidi/zero-width format characters is stripped:
 *     U+200B-U+200F, U+202A-U+202E, U+2060-U+2064, U+FEFF.
 *   - everything else -- umlauts, emoji, any other valid printable UTF-8 --
 *     passes unchanged.
 *
 * CHARSET_FILTER_STRIP_SEPARATORS additionally strips a small set of ASCII
 * bytes that this firmware's own APRS parsers treat as field delimiters.
 * This mode is for CHR-02 free-text fields (pos_atxt / node_atxt) at the
 * point they get embedded into a frame -- CHR-01's message-payload
 * chokepoints (decodeAPRS(), encodePayloadAPRS()) use PLAIN, because the
 * message payload legitimately carries several of these bytes as structure
 * (the ACK-id trailer, the HEY per-hop report, position extension fields)
 * and CHR-01 must not touch them.
 *
 * Derived separator set, one byte each, with the parser code that treats it
 * as structure (line numbers as of this filter's introduction):
 *   '{' '}'  -- DM/group-call address prefix "{call}text" and the ACK-id
 *               trailer "text{nnn" (src/loop_functions.cpp:3629-3631,
 *               :3687); consumed by lora_functions.cpp:906 and
 *               udp_functions.cpp:388.
 *   ':'      -- aprsmsg.payload_type terminates the destination path and
 *               can itself be ':' (src/aprs_functions.cpp:294); the
 *               classic APRS message envelope ":ADDRESSEE:text"
 *               (src/aprs_functions.cpp:1339, encodeLoRaAPRSText()).
 *   ','      -- path callsign separator (src/aprs_functions.cpp:211-221,
 *               :309-314); HEY per-hop "count,rssi,snr" report
 *               (src/aprs_functions.cpp:1152-1157, appendHeySignalReport()).
 *   ';'      -- HEY per-hop group terminator (src/aprs_functions.cpp:1157).
 *   '/'      -- position extension field markers /B= /A= /P= /H= /T= /O=
 *               /F= /Q= /G= /N= /C= /V= /Y= (src/aprs_functions.cpp:632,
 *               658-1011, decodeAPRSPOS()).
 * A plain space is deliberately NOT in this set, even though
 * decodeAPRSPOS() also reads it as an atxt terminator
 * (src/aprs_functions.cpp:632) -- that is a pre-existing APRS wire-format
 * constraint on atxt content, not a structure byte this filter enforces.
 */
#ifndef _CHARSET_FILTER_H_
#define _CHARSET_FILTER_H_

#include <stddef.h>

enum charset_filter_mode
{
    CHARSET_FILTER_PLAIN = 0,         // CHR-01: message payload text.
    CHARSET_FILTER_STRIP_SEPARATORS,  // CHR-02: APRS free-text fields.
};

/* Filters buf[0..len) in place. Only ever removes bytes -- surviving bytes
 * are compacted forward, so the result never exceeds len and buf stays
 * safely reusable at its original capacity. buf need not be
 * NUL-terminated: only the first len bytes are read or written, and the
 * caller is responsible for (re-)terminating at the returned length if it
 * needs a C string afterwards. Returns the new length (<=len). A NULL buf
 * or len==0 is a no-op that returns 0. */
size_t charset_filter_apply(char *buf, size_t len, charset_filter_mode mode);

/* UTF-8-safe truncation: returns a length <=max_len (and <=len) such that
 * buf[0..len) is never cut in the middle of a multi-byte sequence -- a
 * sequence that would straddle max_len is dropped whole rather than left
 * broken. Read-only, never modifies buf. Use this at any fixed-size field
 * cap a receiver enforces by counting raw bytes (e.g. the 25-byte atxt
 * limit, src/aprs_functions.cpp:632) so the receiver's byte-counting parser
 * never inherits a split sequence. */
size_t charset_utf8_safe_truncate(const char *buf, size_t len, size_t max_len);

#endif
