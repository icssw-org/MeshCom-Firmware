#pragma once

#include <stddef.h>

/* APRSWX egress path (--aprswxpath). Persisted as node_aprswxpath; not an sset2 bit. */
#define APRSWX_PATH_RADIO 0
#define APRSWX_PATH_NET   1
#define APRSWX_PATH_BOTH  2

static inline bool aprsWxPathUsesRadio(int path)
{
    return path == APRSWX_PATH_RADIO || path == APRSWX_PATH_BOTH;
}

static inline bool aprsWxPathUsesNet(int path)
{
    return path == APRSWX_PATH_NET || path == APRSWX_PATH_BOTH;
}

const char *aprsWxPathName(int path);

/* Effective WX source call: node_aprswxcall if set, else node_call.
 * Pointer into meshcom_settings; valid until that field is overwritten. */
const char *aprsWxEffectiveCall(void);

/* Build APRS-IS TX line: CALL>APRS,TCPIP*:!{wx_body}
 * wx_body is PositionToAprsWx / aprsWxFormatPosition output (no leading '!').
 * No MeshCom <FF01> header. Returns false if call/wx empty or buffer too small. */
bool aprsIsFormatWxLine(char *out, size_t out_len, const char *call, const char *wx_body);
