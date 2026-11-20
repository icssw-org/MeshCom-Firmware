#pragma once
// APRS-IS line builders. Pure C++, host-testable (no settings, no Arduino).

#include <stddef.h>
#include <stdbool.h>

// Call shape accepted on the APRS-IS header and login line: 1..6 upper-case letters/digits,
// optional "-" plus a 1..2 character alphanumeric SSID, at most 9 characters in total.
// Rejects anything that would break "CALL>APRS,TCPIP*:" ('>', ':', ',', spaces, lower case, ...).
bool aprsIsCallValid(const char *call);

// The WX source call. It must be set (not empty / not blank), valid (aprsIsCallValid) and
// different from the node call: the MeshCom server already forwards this node's position to
// APRS-IS under the node call, a second stream under the same call would flip the symbol on aprs.fi.
// Copies the trimmed call (up to the first space) into out. Returns false otherwise (out emptied).
bool aprsIsWxCall(char *out, size_t out_len, const char *wxcall, const char *nodecall);

// "CALL>APRS,TCPIP*:!<wx_body>" (no CRLF). call is trimmed at the first space and must be valid.
// Returns false if call invalid, body empty or buffer too small.
bool aprsIsFormatWxLine(char *out, size_t out_len, const char *call, const char *wx_body);

// "user CALL pass PASS vers MeshCom <version>\r\n". Empty pass -> "-1" (receive-only).
// Returns false if call invalid or buffer too small.
bool aprsIsFormatLogin(char *out, size_t out_len, const char *call, const char *pass, const char *version);
