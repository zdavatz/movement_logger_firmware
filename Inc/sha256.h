/**
  ******************************************************************************
  * @file    sha256.h
  * @brief   Tiny streaming SHA-256 (FIPS 180-4). Public-domain implementation,
  *          no dynamic allocation, no HAL dependency — used by fwupdate.c to
  *          verify a received firmware image against the host-declared digest
  *          before activating the new bank (F-FWU-3).
  *
  *          STM32U585 has no HASH peripheral and HAL_CRC is not linked, so the
  *          integrity check is done in software. A 73 KB image hashes in a few
  *          ms on the M33 @ 160 MHz — negligible next to the BLE transfer.
  ******************************************************************************
  */
#ifndef PL_SHA256_H
#define PL_SHA256_H

#include <stdint.h>
#include <stddef.h>

#define SHA256_DIGEST_SIZE 32

typedef struct {
  uint32_t state[8];
  uint64_t bitlen;
  uint8_t  buf[64];
  uint32_t buflen;
} sha256_ctx;

void Sha256_Init(sha256_ctx *c);
void Sha256_Update(sha256_ctx *c, const uint8_t *data, uint32_t len);
void Sha256_Final(sha256_ctx *c, uint8_t out[SHA256_DIGEST_SIZE]);

#endif /* PL_SHA256_H */
