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

#ifndef __RV_AES_H__
#define __RV_AES_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Number of bytes in a single AES block
 */
#define AES_BLOCK_BYTES 16

/**
 * @brief Bytes in an AES 128 Cipher key
 */
#define AES_128_KEY_BYTES (4 * 4)
/**
 * @brief Bytes in an AES 192 Cipher key
 */
#define AES_192_KEY_BYTES (4 * 6)
/**
 * @brief Bytes in an AES 256 Cipher key
 */
#define AES_256_KEY_BYTES (4 * 8)

/**
 * @brief AES encryption/decryption in ECB mode
 * @param key Pointer to the AES key
 * @param bits Key size in bits (128, 192, or 256)
 * @param src Pointer to the input ciphertext
 * @param dst Pointer to the output plaintext
 * @param len Length of the input data in bytes (must be multiple of 16 bytes)
 * @param doEncrypt Flag indicating encryption (1) or decryption (0)
 */
void rv_aes_ecb(const uint8_t* key, int bits, const uint8_t* src, uint8_t* dst, size_t len, bool doEncrypt);

/**
 * @brief AES encryption/decryption in CTR mode
 * @param key Pointer to the AES key
 * @param bits Key size in bits (128, 192, or 256)
 * @param iv Pointer to the 16-byte initialization vector (counter)
 * @param src Pointer to the input data
 * @param dst Pointer to the output data
 * @param len Length of the input data in bytes (no multiple of 16 bytes required)
 */
void rv_aes_ctr(const uint8_t* key, int bits, const uint8_t iv[AES_BLOCK_BYTES], const uint8_t* src, uint8_t* dst, size_t len);

/**
 * @brief Insecurely initializes the IV for CTR mode (for testing purposes only, uses stdlib rand)
 * @param iv Pointer to the 16-byte initialization vector to be set
 */
void unsecure_initialization_vector(uint8_t iv[AES_BLOCK_BYTES]);

#endif // __RV_AES_H__