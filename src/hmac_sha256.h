// Portable SHA-256 (RFC 6234 / FIPS 180-4) and HMAC-SHA256 (RFC 2104).
//
// Arduino-free, no mbedtls, no dynamic allocation, C++11-safe. Declarations only;
// the definitions live in hmac_sha256.cpp so the compress function and the K table
// exist once per image instead of once per includer (DRY-05).
// Builds on the host, ESP32 and nRF52 (arm-none-eabi gcc 7, -Ofast): all word
// loads and stores are explicit big-endian byte operations, no type punning.
//
// RM-01.
//
// Streaming use:  sha256Init(c); sha256Update(c, p, n)...; sha256Final(c, out);
// The context is wiped by sha256Final().

#ifndef HMAC_SHA256_H
#define HMAC_SHA256_H

#include <stddef.h>
#include <stdint.h>

struct Sha256Ctx
{
    uint32_t h[8];
    uint64_t len; // message length in bytes so far
    uint8_t buf[64];
    size_t blen; // bytes pending in buf, 0..63
};

namespace hmac_sha256_detail
{
// volatile write loop: best-effort wipe that the optimiser must not drop
void wipe(void *p, size_t n);
} // namespace hmac_sha256_detail

void sha256Init(Sha256Ctx &c);
void sha256Update(Sha256Ctx &c, const uint8_t *data, size_t n);
void sha256Final(Sha256Ctx &c, uint8_t out[32]);
void sha256(const uint8_t *data, size_t n, uint8_t out[32]);

// RFC 2104 with B = 64. A key longer than 64 bytes is hashed first.
void hmacSha256(const uint8_t *key, size_t klen, const uint8_t *msg, size_t mlen, uint8_t out[32]);

// Lower-case hex of the first nbytes of in, NUL-terminated; out holds 2*nbytes+1.
void hexLower(const uint8_t *in, size_t nbytes, char *out);

// Constant-time compare: always walks all n bytes, no early exit.
bool ctEqual(const uint8_t *a, const uint8_t *b, size_t n);

#endif // HMAC_SHA256_H
