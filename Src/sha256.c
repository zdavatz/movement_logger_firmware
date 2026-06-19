/**
  ******************************************************************************
  * @file    sha256.c
  * @brief   Tiny streaming SHA-256 (FIPS 180-4). Public-domain reference
  *          implementation (Brad Conte style), trimmed to the 3 entry points
  *          fwupdate.c needs. No allocation, no HAL.
  ******************************************************************************
  */
#include "sha256.h"
#include <string.h>

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x)  (ROTR(x, 2)  ^ ROTR(x, 13) ^ ROTR(x, 22))
#define EP1(x)  (ROTR(x, 6)  ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SIG0(x) (ROTR(x, 7)  ^ ROTR(x, 18) ^ ((x) >> 3))
#define SIG1(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

static const uint32_t K[64] = {
  0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
  0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
  0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
  0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
  0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
  0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
  0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
  0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};

static void sha256_transform(sha256_ctx *ctx, const uint8_t *d)
{
  uint32_t a, b, c, dd, e, f, g, h, t1, t2, m[64];
  int i, j;
  for (i = 0, j = 0; i < 16; ++i, j += 4)
    m[i] = ((uint32_t)d[j] << 24) | ((uint32_t)d[j + 1] << 16) |
           ((uint32_t)d[j + 2] << 8) | ((uint32_t)d[j + 3]);
  for (; i < 64; ++i)
    m[i] = SIG1(m[i - 2]) + m[i - 7] + SIG0(m[i - 15]) + m[i - 16];

  a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; dd = ctx->state[3];
  e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

  for (i = 0; i < 64; ++i) {
    t1 = h + EP1(e) + CH(e, f, g) + K[i] + m[i];
    t2 = EP0(a) + MAJ(a, b, c);
    h = g; g = f; f = e; e = dd + t1;
    dd = c; c = b; b = a; a = t1 + t2;
  }

  ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += dd;
  ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

void Sha256_Init(sha256_ctx *c)
{
  c->bitlen = 0;
  c->buflen = 0;
  c->state[0] = 0x6a09e667; c->state[1] = 0xbb67ae85;
  c->state[2] = 0x3c6ef372; c->state[3] = 0xa54ff53a;
  c->state[4] = 0x510e527f; c->state[5] = 0x9b05688c;
  c->state[6] = 0x1f83d9ab; c->state[7] = 0x5be0cd19;
}

void Sha256_Update(sha256_ctx *c, const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i) {
    c->buf[c->buflen++] = data[i];
    if (c->buflen == 64) {
      sha256_transform(c, c->buf);
      c->bitlen += 512;
      c->buflen = 0;
    }
  }
}

void Sha256_Final(sha256_ctx *c, uint8_t out[SHA256_DIGEST_SIZE])
{
  uint32_t i = c->buflen;

  /* Pad: append 0x80 then zeros, leaving room for the 64-bit length. */
  if (c->buflen < 56) {
    c->buf[i++] = 0x80;
    while (i < 56) c->buf[i++] = 0x00;
  } else {
    c->buf[i++] = 0x80;
    while (i < 64) c->buf[i++] = 0x00;
    sha256_transform(c, c->buf);
    memset(c->buf, 0, 56);
  }

  c->bitlen += (uint64_t)c->buflen * 8;
  c->buf[63] = (uint8_t)(c->bitlen);
  c->buf[62] = (uint8_t)(c->bitlen >> 8);
  c->buf[61] = (uint8_t)(c->bitlen >> 16);
  c->buf[60] = (uint8_t)(c->bitlen >> 24);
  c->buf[59] = (uint8_t)(c->bitlen >> 32);
  c->buf[58] = (uint8_t)(c->bitlen >> 40);
  c->buf[57] = (uint8_t)(c->bitlen >> 48);
  c->buf[56] = (uint8_t)(c->bitlen >> 56);
  sha256_transform(c, c->buf);

  /* Big-endian digest. */
  for (i = 0; i < 4; ++i) {
    out[i]      = (uint8_t)(c->state[0] >> (24 - i * 8));
    out[i + 4]  = (uint8_t)(c->state[1] >> (24 - i * 8));
    out[i + 8]  = (uint8_t)(c->state[2] >> (24 - i * 8));
    out[i + 12] = (uint8_t)(c->state[3] >> (24 - i * 8));
    out[i + 16] = (uint8_t)(c->state[4] >> (24 - i * 8));
    out[i + 20] = (uint8_t)(c->state[5] >> (24 - i * 8));
    out[i + 24] = (uint8_t)(c->state[6] >> (24 - i * 8));
    out[i + 28] = (uint8_t)(c->state[7] >> (24 - i * 8));
  }
}
