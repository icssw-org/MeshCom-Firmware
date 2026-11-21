// pong_format.h -- {pong} payload build and display id, pure (host-testable).
// Wire format: "{pong}{<id>}", id = unsigned 32-bit decimal, at least 3 digits.
// Old nodes show payload chars 14..16, i.e. the last 3 digits of a 10-digit id;
// the display here takes the last 3 digits of whatever length (legacy '-' skipped).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

inline int pongFormatPayload(char *out, size_t n, uint32_t msg_id)
{
    return snprintf(out, n, "{pong}{%03lu}", (unsigned long)msg_id);
}

// out must hold 4 bytes. Never reads past the NUL of payload.
inline void pongDisplayId(const char *payload, char out[4])
{
    out[0] = '\0';
    if (payload == NULL || strncmp(payload, "{pong}{", 7) != 0)
        return;

    char dig[11];
    size_t nd = 0;
    for (const char *p = payload + 7; *p != '\0' && *p != '}' && nd < 10; p++)
    {
        if (*p >= '0' && *p <= '9')
            dig[nd++] = *p;
    }

    if (nd == 0)
        return; // no digits: show nothing rather than a fake "000"

    size_t take = nd < 3 ? nd : 3;
    size_t pad = 3 - take;
    for (size_t i = 0; i < pad; i++)
        out[i] = '0';
    for (size_t i = 0; i < take; i++)
        out[pad + i] = dig[nd - take + i];
    out[3] = '\0';
}
