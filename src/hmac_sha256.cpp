// Portable SHA-256 (RFC 6234 / FIPS 180-4) and HMAC-SHA256 (RFC 2104).
// Definitions for hmac_sha256.h (RM-01, DRY-05). Arduino-free, no dynamic
// allocation. All word loads and stores are explicit big-endian byte
// operations, no type punning (nRF52 builds with -Ofast).

#include "hmac_sha256.h"

namespace hmac_sha256_detail
{
// volatile write loop: best-effort wipe that the optimiser must not drop
void wipe(void *p, size_t n)
{
    volatile uint8_t *v = static_cast<volatile uint8_t *>(p);
    while (n--)
        *v++ = 0;
}

static uint32_t rotr(uint32_t x, unsigned n)
{
    return (x >> n) | (x << (32u - n));
}

static uint32_t loadBe32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void storeBe32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static const uint32_t *k256()
{
    static const uint32_t K[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    return K;
}

// one 64-byte block; W is a 16-word rolling schedule (64 B of stack)
static void compress(uint32_t h[8], const uint8_t *blk)
{
    const uint32_t *K = k256();
    uint32_t w[16];
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];

    for (unsigned i = 0; i < 64; i++)
    {
        uint32_t wi;
        if (i < 16)
        {
            wi = loadBe32(blk + 4 * i);
        }
        else
        {
            uint32_t w15 = w[(i - 15) & 15], w2 = w[(i - 2) & 15];
            uint32_t s0 = rotr(w15, 7) ^ rotr(w15, 18) ^ (w15 >> 3);
            uint32_t s1 = rotr(w2, 17) ^ rotr(w2, 19) ^ (w2 >> 10);
            wi = w[(i - 16) & 15] + s0 + w[(i - 7) & 15] + s1;
        }
        w[i & 15] = wi;

        uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t t1 = hh + S1 + ch + K[i] + wi;
        uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
        uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t2 = S0 + mj;
        hh = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
    wipe(w, sizeof(w));
}
} // namespace hmac_sha256_detail

void sha256Init(Sha256Ctx &c)
{
    c.h[0] = 0x6a09e667;
    c.h[1] = 0xbb67ae85;
    c.h[2] = 0x3c6ef372;
    c.h[3] = 0xa54ff53a;
    c.h[4] = 0x510e527f;
    c.h[5] = 0x9b05688c;
    c.h[6] = 0x1f83d9ab;
    c.h[7] = 0x5be0cd19;
    c.len = 0;
    c.blen = 0;
    for (size_t i = 0; i < sizeof(c.buf); i++)
        c.buf[i] = 0;
}

void sha256Update(Sha256Ctx &c, const uint8_t *data, size_t n)
{
    if (n == 0 || data == NULL)
        return;
    c.len += (uint64_t)n;

    if (c.blen > 0)
    {
        size_t take = 64 - c.blen;
        if (take > n)
            take = n;
        for (size_t i = 0; i < take; i++)
            c.buf[c.blen + i] = data[i];
        c.blen += take;
        data += take;
        n -= take;
        if (c.blen < 64)
            return;
        hmac_sha256_detail::compress(c.h, c.buf);
        c.blen = 0;
    }
    while (n >= 64)
    {
        hmac_sha256_detail::compress(c.h, data);
        data += 64;
        n -= 64;
    }
    for (size_t i = 0; i < n; i++)
        c.buf[i] = data[i];
    c.blen = n;
}

void sha256Final(Sha256Ctx &c, uint8_t out[32])
{
    const uint64_t bits = c.len << 3;
    c.buf[c.blen++] = 0x80;
    if (c.blen > 56)
    {
        while (c.blen < 64)
            c.buf[c.blen++] = 0;
        hmac_sha256_detail::compress(c.h, c.buf);
        c.blen = 0;
    }
    while (c.blen < 56)
        c.buf[c.blen++] = 0;
    hmac_sha256_detail::storeBe32(c.buf + 56, (uint32_t)(bits >> 32));
    hmac_sha256_detail::storeBe32(c.buf + 60, (uint32_t)bits);
    hmac_sha256_detail::compress(c.h, c.buf);

    for (unsigned i = 0; i < 8; i++)
        hmac_sha256_detail::storeBe32(out + 4 * i, c.h[i]);
    hmac_sha256_detail::wipe(&c, sizeof(c));
}

void sha256(const uint8_t *data, size_t n, uint8_t out[32])
{
    Sha256Ctx c;
    sha256Init(c);
    sha256Update(c, data, n);
    sha256Final(c, out);
}

// RFC 2104 with B = 64. A key longer than 64 bytes is hashed first.
void hmacSha256(const uint8_t *key, size_t klen, const uint8_t *msg, size_t mlen, uint8_t out[32])
{
    uint8_t kpad[64];
    uint8_t khash[32];
    uint8_t inner[32];
    Sha256Ctx c;

    for (size_t i = 0; i < sizeof(kpad); i++)
        kpad[i] = 0;
    if (klen > sizeof(kpad))
    {
        sha256(key, klen, khash);
        for (size_t i = 0; i < sizeof(khash); i++)
            kpad[i] = khash[i];
    }
    else
    {
        for (size_t i = 0; i < klen; i++)
            kpad[i] = key[i];
    }

    for (size_t i = 0; i < sizeof(kpad); i++)
        kpad[i] ^= 0x36;
    sha256Init(c);
    sha256Update(c, kpad, sizeof(kpad));
    sha256Update(c, msg, mlen);
    sha256Final(c, inner);

    for (size_t i = 0; i < sizeof(kpad); i++)
        kpad[i] ^= (0x36 ^ 0x5c);
    sha256Init(c);
    sha256Update(c, kpad, sizeof(kpad));
    sha256Update(c, inner, sizeof(inner));
    sha256Final(c, out);

    hmac_sha256_detail::wipe(kpad, sizeof(kpad));
    hmac_sha256_detail::wipe(khash, sizeof(khash));
    hmac_sha256_detail::wipe(inner, sizeof(inner));
}

// Lower-case hex of the first nbytes of in, NUL-terminated; out holds 2*nbytes+1.
void hexLower(const uint8_t *in, size_t nbytes, char *out)
{
    static const char digits[] = "0123456789abcdef";
    for (size_t i = 0; i < nbytes; i++)
    {
        out[2 * i] = digits[in[i] >> 4];
        out[2 * i + 1] = digits[in[i] & 0x0f];
    }
    out[2 * nbytes] = '\0';
}

// Constant-time compare: always walks all n bytes, no early exit.
bool ctEqual(const uint8_t *a, const uint8_t *b, size_t n)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < n; i++)
        diff |= (uint8_t)(a[i] ^ b[i]);
    return diff == 0;
}
