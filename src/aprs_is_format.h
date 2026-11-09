#pragma once
// APRS-IS line builders. Pure C++, host-testable (no settings, no Arduino).

#include <stddef.h>

// Effective WX source call: wxcall if set (not empty / not blank), else nodecall.
// Copies the trimmed call (up to the first space) into out. Returns false if both are empty.
bool aprsIsEffectiveCall(char *out, size_t out_len, const char *wxcall, const char *nodecall);

// "CALL>APRS,TCPIP*:!<wx_body>" (no CRLF). call is trimmed at the first space.
// Returns false if call/body empty or buffer too small.
bool aprsIsFormatWxLine(char *out, size_t out_len, const char *call, const char *wx_body);

// "user CALL pass PASS vers MeshCom <version>\r\n". Empty pass -> "-1" (receive-only).
bool aprsIsFormatLogin(char *out, size_t out_len, const char *call, const char *pass, const char *version);
