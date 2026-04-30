/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)   2025 Niklas Sirch <niklas.sirch1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This application is a demo. It shows "PicoNut" in colorised ascii art.

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

#include "rv_crypto.h"
#include "rv_aes_asm.h"

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

// Helper to handle key scheduling and block processing based on bit size
static void aes_process_block(const uint8_t* key, int bits, const uint8_t* src, uint8_t* dst, uint8_t encrypt)
{
    uint32_t rk[AES_256_RK_WORDS]; // Max size
    switch(bits)
    {
        case 128:
            if(encrypt)
            {
                aes_128_enc_key_schedule(rk, (uint8_t*)key);
                aes_128_block_encrypt(dst, (uint8_t*)src, rk);
            }
            else
            {
                aes_128_dec_key_schedule(rk, (uint8_t*)key);
                aes_128_block_decrypt(dst, (uint8_t*)src, rk);
            }
            break;

        case 192:
            if(encrypt)
            {
                aes_192_enc_key_schedule(rk, (uint8_t*)key);
                aes_192_block_encrypt(dst, (uint8_t*)src, rk);
            }
            else
            {
                aes_192_dec_key_schedule(rk, (uint8_t*)key);
                aes_192_block_decrypt(dst, (uint8_t*)src, rk);
            }
            break;

        case 256:
        default:
            if(encrypt)
            {
                aes_256_enc_key_schedule(rk, (uint8_t*)key);
                aes_256_block_encrypt(dst, (uint8_t*)src, rk);
            }
            else
            {
                aes_256_dec_key_schedule(rk, (uint8_t*)key);
                aes_256_block_decrypt(dst, (uint8_t*)src, rk);
            }
            break;
    }
}

void rv_aes_ecb(const uint8_t* key, int bits, const uint8_t* src, uint8_t* dst, size_t len, bool doEncrypt)
{
    for(size_t i = 0; i < len; i += AES_BLOCK_BYTES)
    {
        aes_process_block(key, bits, src + i, dst + i, doEncrypt);
    }
}

void rv_aes_ctr(const uint8_t* key, int bits, const uint8_t iv_in[AES_BLOCK_BYTES], const uint8_t* src, uint8_t* dst, size_t len)
{
    uint8_t stream[AES_BLOCK_BYTES];
    uint8_t iv[AES_BLOCK_BYTES];
    memcpy(iv, iv_in, AES_BLOCK_BYTES);

    for(size_t i = 0; i < len; i += AES_BLOCK_BYTES)
    {
        // 1. Encrypt the Counter (IV)
        aes_process_block(key, bits, iv, stream, 1);

        // 2. XOR stream with plaintext/ciphertext
        size_t chunk = (len - i < AES_BLOCK_BYTES) ? (len - i) : AES_BLOCK_BYTES;
        for(size_t j = 0; j < chunk; j++)
        {
            dst[i + j] = src[i + j] ^ stream[j];
        }

        // 3. Increment IV (Big-endian 128-bit increment)
        for(int j = AES_BLOCK_BYTES - 1; j >= 0; j--)
        {
            if(++iv[j] != 0)
                break;
        }
    }
}

void unsecure_initialization_vector(uint8_t iv[AES_BLOCK_BYTES])
{
    for(size_t i = 0; i < AES_BLOCK_BYTES; i++)
    {
        iv[i] = rand() & 0xFF;
    }
}
