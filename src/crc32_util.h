/**
 * CRC-32 (IEEE 802.3 / zlib / PNG), header-only.
 *
 * Parameters -- the ones `crc32` in Python's zlib, `cksum -a crc32b` and
 * `gzip` use, so a value produced here can be reproduced by any standard
 * tool without a table of our own:
 *
 *   width  32
 *   poly   0x04C11DB7, reflected as 0xEDB88320
 *   init   0xFFFFFFFF
 *   refin  true, refout true
 *   xorout 0xFFFFFFFF
 *   check  0xCBF43926 for the ASCII string "123456789"
 *
 * Bitwise, no lookup table: this runs a few times per boot at most (config
 * export/import, CS-03), and a 1 KiB table in flash/RAM is not worth it.
 *
 * Deliberately NOT the CRC in src/t-deck/tdeck_debug.cpp -- that one is a
 * T-Deck screen-content checksum with its own call sites and stays where it
 * is. Pure C++, no Arduino dependency, so the native test build can use it.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

/* Streaming form: feed successive chunks, starting with crc = 0xFFFFFFFF and
 * finishing with ^ 0xFFFFFFFF. Use crc32_buf() unless you really stream. */
inline uint32_t crc32_update(uint32_t crc, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
        {
            /* branchless-ish reflected update; 0xEDB88320 is the reflected poly */
            crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1)));
        }
    }

    return crc;
}

/* One-shot: CRC-32 over len bytes at data. */
inline uint32_t crc32_buf(const void *data, size_t len)
{
    return crc32_update(0xFFFFFFFFUL, data, len) ^ 0xFFFFFFFFUL;
}
