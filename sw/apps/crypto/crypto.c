/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)   2025 Niklas Sirch <niklas.sirch1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/


/**
 * @brief Crypto application to test AES implementation
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <rv_crypto.h>

// tiny-aes configuration
#define CBC 0
#define CTR 0
#define ECB 1
#include "tiny_aes.h"

// Peripheral Addresses
#define PN_CFG_CLINT_BASE_ADDRESS 0x2000000
#define CLINT_REG_MTIME_LO (PN_CFG_CLINT_BASE_ADDRESS + 0xBFF8)
#define CLINT_REG_MTIME_HI (PN_CFG_CLINT_BASE_ADDRESS + 0xBFFC)

// Constants
static const char DEFAULT_DATA[AES_BLOCK_BYTES] = "0123456789abcde";
static const uint8_t DEFAULT_KEY_128[AES_128_KEY_BYTES] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
static const uint8_t DEFAULT_KEY_192[AES_192_KEY_BYTES] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};

static const uint8_t DEFAULT_KEY_256[AES_256_KEY_BYTES] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
static const char* DEFAULT_LONG_DATA_UNALIGNED = "RISC-V AES on PicoNut! This is a longer text to test AES encryption and decryption performance and correctness across multiple blocks.";
static const int DEFAULT_LONG_DATA_UNALIGNED_LEN = 135;
static const char DEFAULT_LONG_DATA_ALIGNED[AES_BLOCK_BYTES * 4] = "0123456789abcde"
                                                                   "0123456789abcde"
                                                                   "0123456789abcde"
                                                                   "0123456789abcde";

// Reads the 64-bit RISC-V mtime register
static uint64_t
read_time_ticks(void)
{
    volatile uint32_t* mtime_lo = (volatile uint32_t*)CLINT_REG_MTIME_LO;
    volatile uint32_t* mtime_hi = (volatile uint32_t*)CLINT_REG_MTIME_HI;

    uint32_t hi = *mtime_hi;
    uint32_t lo = *mtime_lo;

    return ((uint64_t)hi << 32) | lo;
}

static void tiny_encrypt_ecb(const uint8_t src[AES_BLOCK_BYTES], const uint8_t key[AES_128_KEY_BYTES], uint8_t dst[AES_BLOCK_BYTES])
{
    struct AES_ctx ctx;
    memcpy(dst, src, AES_BLOCK_BYTES);
    AES_init_ctx(&ctx, key);
    AES_ECB_encrypt(&ctx, dst);
}

static void tiny_decrypt_ecb(
    const uint8_t src[AES_BLOCK_BYTES],
    const uint8_t key[AES_128_KEY_BYTES],
    uint8_t dst[AES_BLOCK_BYTES])
{
    struct AES_ctx ctx;
    memcpy(dst, src, AES_BLOCK_BYTES);
    AES_init_ctx(&ctx, key);
    AES_ECB_decrypt(&ctx, dst);
}

int test_ctr_implementation()
{
    uint8_t key[AES_128_KEY_BYTES];
    uint8_t iv[AES_BLOCK_BYTES];
    uint8_t plain_text[DEFAULT_LONG_DATA_UNALIGNED_LEN];
    uint8_t cipher_text[DEFAULT_LONG_DATA_UNALIGNED_LEN];
    uint8_t decrypted_text[DEFAULT_LONG_DATA_UNALIGNED_LEN];

    memcpy(key, DEFAULT_KEY_128, AES_128_KEY_BYTES);
    memcpy(plain_text, DEFAULT_LONG_DATA_UNALIGNED, DEFAULT_LONG_DATA_UNALIGNED_LEN);

    unsecure_initialization_vector(iv);

    rv_aes_ctr(key, 128, (const uint8_t*)iv, plain_text, cipher_text, DEFAULT_LONG_DATA_UNALIGNED_LEN);
    rv_aes_ctr(key, 128, (const uint8_t*)iv, cipher_text, decrypted_text, DEFAULT_LONG_DATA_UNALIGNED_LEN);

    if(memcmp(plain_text, decrypted_text, DEFAULT_LONG_DATA_UNALIGNED_LEN) != 0)
    {
        return 1; // Error
    }

    return 0; // Success
}

int test_ecb_implementation()
{
    uint8_t key[AES_128_KEY_BYTES];
    uint8_t plain_text[AES_BLOCK_BYTES * 4];
    uint8_t cipher_text[AES_BLOCK_BYTES * 4];
    uint8_t decrypted_text[AES_BLOCK_BYTES * 4];

    memcpy(key, DEFAULT_KEY_128, AES_128_KEY_BYTES);
    memcpy(plain_text, DEFAULT_LONG_DATA_ALIGNED, AES_BLOCK_BYTES * 4);

    rv_aes_ecb(key, 128, plain_text, cipher_text, AES_BLOCK_BYTES * 4, 1);
    rv_aes_ecb(key, 128, cipher_text, decrypted_text, AES_BLOCK_BYTES * 4, 0);

    if(memcmp(plain_text, decrypted_text, AES_BLOCK_BYTES * 4) != 0)
    {
        return 1; // Error
    }

    return 0; // Success
}

int main()
{
    int ret;
    uint8_t key[AES_128_KEY_BYTES];
    uint8_t key192[AES_192_KEY_BYTES];
    uint8_t key256[AES_256_KEY_BYTES];
    uint8_t plain_text[AES_BLOCK_BYTES];
    uint8_t cipher_text[AES_BLOCK_BYTES];
    uint8_t cipher_text_tiny[AES_BLOCK_BYTES]; // New buffer for cross-check
    uint8_t decrypted_text[AES_BLOCK_BYTES];

    memcpy(key, DEFAULT_KEY_128, AES_128_KEY_BYTES);
    memcpy(plain_text, DEFAULT_DATA, AES_BLOCK_BYTES);

    printf("--- AES Implementation Comparison ---\n");

    // 1. Encryption Correctness
    rv_aes_ecb(key, 128, plain_text, cipher_text, AES_BLOCK_BYTES, 1);
    rv_aes_ecb(key, 128, cipher_text, decrypted_text, AES_BLOCK_BYTES, 0);

    if(memcmp(plain_text, decrypted_text, AES_BLOCK_BYTES) != 0)
    {
        printf("Error: RV AES self-check failed!\n");
        return 1;
    }

    // 2. Standard Correctness
    tiny_encrypt_ecb(plain_text, key, cipher_text_tiny);

    if(memcmp(cipher_text, cipher_text_tiny, AES_BLOCK_BYTES) != 0)
    {
        printf("Error: Implementation mismatch! RV AES and Tiny AES produced different ciphertexts.\n");
        return 1;
    }

    ret = test_ecb_implementation();
    if(ret != 0)
    {
        printf("Error: ECB mode implementation failed!\n");
        return 1;
    }

    ret = test_ctr_implementation();
    if(ret != 0)
    {
        printf("Error: CTR mode implementation failed!\n");
        return 1;
    }

    printf("Correctness and Cross-implementation checks passed.\n\n");

    // Benchmarking
    uint64_t t0, t1;

    // Benchmark Tiny AES
    memset(cipher_text, 0, AES_BLOCK_BYTES);

    t0 = read_time_ticks();
    tiny_encrypt_ecb(plain_text, key, cipher_text);
    t1 = read_time_ticks();
    printf("Tiny AES 128 Encryption:\t%llu ticks\n", (t1 - t0));

    t0 = read_time_ticks();
    tiny_decrypt_ecb(cipher_text, key, decrypted_text);
    t1 = read_time_ticks();
    printf("Tiny AES 128 Decryption:\t%llu ticks\n", (t1 - t0));

    // Benchmark RV AES
    memset(cipher_text, 0, AES_BLOCK_BYTES);
    memset(decrypted_text, 0, AES_BLOCK_BYTES);

    t0 = read_time_ticks();
    rv_aes_ecb(key, 128, plain_text, cipher_text, AES_BLOCK_BYTES, 1);
    t1 = read_time_ticks();
    printf("RV AES 128  Encryption:\t%llu ticks\n", (t1 - t0));

    t0 = read_time_ticks();
    rv_aes_ecb(key, 128, cipher_text, decrypted_text, AES_BLOCK_BYTES, 0);
    t1 = read_time_ticks();
    printf("RV AES 128 Decryption:\t%llu ticks\n", (t1 - t0));

    /* RV AES 192-bit benchmark: print tick counts with key length */
    memcpy(key192, DEFAULT_KEY_192, AES_192_KEY_BYTES);
    memset(cipher_text, 0, AES_BLOCK_BYTES);
    memset(decrypted_text, 0, AES_BLOCK_BYTES);

    t0 = read_time_ticks();
    rv_aes_ecb(key192, 192, plain_text, cipher_text, AES_BLOCK_BYTES, 1);
    t1 = read_time_ticks();
    printf("RV AES 192 Encryption:\t%llu ticks\n", (t1 - t0));

    t0 = read_time_ticks();
    rv_aes_ecb(key192, 192, cipher_text, decrypted_text, AES_BLOCK_BYTES, 0);
    t1 = read_time_ticks();
    printf("RV AES 192 Decryption:\t%llu ticks\n", (t1 - t0));

    /* RV AES 256-bit benchmark: print tick counts with key length */
    memcpy(key256, DEFAULT_KEY_256, AES_256_KEY_BYTES);
    memset(cipher_text, 0, AES_BLOCK_BYTES);
    memset(decrypted_text, 0, AES_BLOCK_BYTES);

    t0 = read_time_ticks();
    rv_aes_ecb(key256, 256, plain_text, cipher_text, AES_BLOCK_BYTES, 1);
    t1 = read_time_ticks();
    printf("RV AES 256 Encryption:\t%llu ticks\n", (t1 - t0));

    t0 = read_time_ticks();
    rv_aes_ecb(key256, 256, cipher_text, decrypted_text, AES_BLOCK_BYTES, 0);
    t1 = read_time_ticks();
    printf("RV AES 256 Decryption:\t%llu ticks\n", (t1 - t0));

    printf("            \n\n"); // uart buffer

    return 0;
}