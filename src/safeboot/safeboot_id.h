// safeboot_id.h -- Safeboot v3: which app image is installed in ota_0, and the serial commands that say so.
// WF-05. The web flasher asks a board that
// sits in Safeboot "--id" / "--info" and reads the answer. Pure functions, no Arduino/IDF types, host-testable.
//
// Marker: the app image carries an mc_board_id_t (src/mc_board_id.h) at image offset 0x120. Safeboot reads
// the first SB_ID_HEAD_LEN bytes of ota_0 and validates them.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../mc_board_id.h"
#include "safeboot_ver.h"

#define SB_ID_MARKER_OFFSET 0x120u
#define SB_ID_APPDESC_OFFSET 0x20u
#define SB_ID_APPDESC_MAGIC 0xABCD5432u
static const size_t SB_ID_HEAD_LEN = SB_ID_MARKER_OFFSET + sizeof(mc_board_id_t);

enum SbIdReason
{
    SB_ID_OK = 0,
    SB_ID_ERASED,       // head is all 0xFF
    SB_ID_NO_IMAGE,     // short read, or not an ESP app image
    SB_ID_NO_MARKER,    // valid image without "MCBOARD1" (built before F4-1a)
    SB_ID_BAD_FORMAT,   // marker present, format != 1
    SB_ID_NO_PARTITION, // set by the caller: ota_0 not found / read failed
};

struct SbIdentity
{
    SbIdReason reason;
    uint16_t hwid;
    char env[sizeof(((mc_board_id_t *)0)->env) + 1];
    char tag[sizeof(((mc_board_id_t *)0)->tag) + 1];
};

inline void sbIdFail(SbIdentity *out, SbIdReason r)
{
    memset(out, 0, sizeof(*out));
    out->reason = r;
}

inline const char *sbIdReasonName(SbIdReason r)
{
    switch (r)
    {
    case SB_ID_OK: return "ok";
    case SB_ID_ERASED: return "erased";
    case SB_ID_NO_IMAGE: return "no-image";
    case SB_ID_NO_MARKER: return "no-marker";
    case SB_ID_BAD_FORMAT: return "bad-format";
    default: return "no-partition";
    }
}

// Validates the first `len` bytes of ota_0. Fills *out (reason always set) and returns the reason.
inline SbIdReason sbIdParse(const uint8_t *b, size_t len, SbIdentity *out)
{
    sbIdFail(out, SB_ID_NO_IMAGE);
    if (!b || len < SB_ID_HEAD_LEN)
        return out->reason;
    bool erased = true;
    for (size_t i = 0; i < SB_ID_HEAD_LEN && erased; i++)
        erased = (b[i] == 0xFF);
    if (erased)
        return out->reason = SB_ID_ERASED;
    uint32_t m = (uint32_t)b[SB_ID_APPDESC_OFFSET] | ((uint32_t)b[SB_ID_APPDESC_OFFSET + 1] << 8) |
                 ((uint32_t)b[SB_ID_APPDESC_OFFSET + 2] << 16) | ((uint32_t)b[SB_ID_APPDESC_OFFSET + 3] << 24);
    if (b[0] != 0xE9 || m != SB_ID_APPDESC_MAGIC)
        return out->reason;
    const uint8_t *p = b + SB_ID_MARKER_OFFSET;
    if (memcmp(p, "MCBOARD1", 8) != 0)
        return out->reason = SB_ID_NO_MARKER;
    if ((p[8] | (p[9] << 8)) != 1)
        return out->reason = SB_ID_BAD_FORMAT;
    out->hwid = (uint16_t)(p[10] | (p[11] << 8));
    memcpy(out->env, p + 12, sizeof(((mc_board_id_t *)0)->env)); // out is zeroed: always terminated
    memcpy(out->tag, p + 44, sizeof(((mc_board_id_t *)0)->tag));
    return out->reason = SB_ID_OK;
}

// "MCBOARD:v1:<hwid>:<hwid>:<env>:<tag|(local)>" (no newline) for a valid identity.
inline int sbIdLine(char *out, size_t n, const SbIdentity &id)
{
    return snprintf(out, n, "MCBOARD:v1:%u:%u:%s:%s", (unsigned)id.hwid, (unsigned)id.hwid, id.env,
                    id.tag[0] ? id.tag : "(local)");
}

enum SbCmd
{
    SB_CMD_NONE = 0,
    SB_CMD_ID,
    SB_CMD_INFO,
};

// Non-blocking line reader: feed one byte at a time; returns the command when a CR/LF ends a line that is
// exactly "--id" or "--info". Lines longer than the buffer are dropped whole (never matched by a prefix).
struct SbLineReader
{
    char buf[32];
    uint8_t len;
    bool overflow;

    SbLineReader() : len(0), overflow(false) {}

    SbCmd feed(char c)
    {
        if (c != '\r' && c != '\n')
        {
            if (len < sizeof(buf) - 1)
                buf[len++] = c;
            else
                overflow = true;
            return SB_CMD_NONE;
        }
        SbCmd r = SB_CMD_NONE;
        if (!overflow)
        {
            buf[len] = 0;
            if (strcmp(buf, "--id") == 0)
                r = SB_CMD_ID;
            else if (strcmp(buf, "--info") == 0)
                r = SB_CMD_INFO;
        }
        len = 0;
        overflow = false;
        return r;
    }
};

// Writes the full answer (newline terminated lines) for a command into out; returns its length, 0 for
// SB_CMD_NONE. Always NUL-terminated when n > 0, never writes past n. `call` is the callsign Safeboot read
// from NVS. Web flasher tools parse these formats -- keep them exact.
inline size_t sbRespond(SbCmd cmd, const SbIdentity &id, const char *call, char *out, size_t n)
{
    if (n == 0)
        return 0;
    out[0] = 0;
    if (cmd == SB_CMD_NONE)
        return 0;
    int w = 0;
    char line[96];
    if (cmd == SB_CMD_ID)
    {
        if (id.reason == SB_ID_OK)
        {
            sbIdLine(line, sizeof(line), id);
            w = snprintf(out, n, "%s\n", line);
        }
        else
            w = snprintf(out, n, "MCBOARD:none:%s\n", sbIdReasonName(id.reason));
    }
    else
    {
        w = snprintf(out, n, "--MeshCom Safeboot %d\n...Call: <%s>", SAFEBOOT_VERSION, call ? call : "");
        if (id.reason == SB_ID_OK && w > 0 && (size_t)w < n)
            w += snprintf(out + w, n - w, " ...NODE %u <SAFEBOOT>", (unsigned)id.hwid);
        if (w > 0 && (size_t)w < n)
            w += snprintf(out + w, n - w, "\n");
        if (id.reason == SB_ID_OK && w > 0 && (size_t)w < n)
            w += snprintf(out + w, n - w, "...AU env=%s tag=%s\n", id.env, id.tag[0] ? id.tag : "(local)");
        if (w > 0 && (size_t)w < n)
            w += snprintf(out + w, n - w, "[SAFEBOOT];ver;%d\n", SAFEBOOT_VERSION);
    }
    if (w < 0)
        return 0;
    return (size_t)w < n ? (size_t)w : n - 1;
}
