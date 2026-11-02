// msg_body.h -- visible length of a message payload without its ACK marker.
//
// The sender appends "{NNN" (snprintf "%s{%03i", loop_functions.cpp) to DM
// payloads: a '{' plus digits at the very END, no closing brace. 1..5 digits
// are accepted, the same rule as the firmware's ACK parsers (pnPayloadIsPn in
// pn_retry.h, kiss_functions.cpp), so markers from other clients hide too. A '{'
// anywhere else is user text ("Code {x} test") and must stay. Pure, no
// Arduino; works on bytes, '{' and digits are ASCII so UTF-8 is never split.
#pragma once
#include <stddef.h>

#define MSG_BODY_ACK_MAX_DIGITS 5

static inline size_t msg_body_visible_len(const char *s, size_t len)
{
    if (!s)
        return 0;
    size_t d = 0;
    while (d < len && d <= MSG_BODY_ACK_MAX_DIGITS &&
           s[len - 1 - d] >= '0' && s[len - 1 - d] <= '9')
        d++;
    if (d == 0 || d > MSG_BODY_ACK_MAX_DIGITS || len < d + 1)
        return len;
    // marker '{' must be at index > 0 (a leading '{' is a tag, as before)
    if (s[len - 1 - d] == '{' && len - 1 - d > 0)
        return len - 1 - d;
    return len;
}
