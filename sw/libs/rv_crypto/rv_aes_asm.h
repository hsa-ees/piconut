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

#include <stdint.h>

/*!
@defgroup crypto_block_aes Crypto Block AES
@{

AES  |   Nk  | Nb   | Nr
-----|-------|------|---------------
128  |  4    | 4    | 10
192  |  6    | 4    | 12
256  |  8    | 4    | 14

*/

#ifndef __RV_AES_ASM_H__
#define __RV_AES_ASM_H__

//! Block size in 4-byte words for AES 128
#define AES_128_NB 4
#define AES_192_NB 4
#define AES_256_NB 4

//! Number of rounds for AES 128
#define AES_128_NR 10
#define AES_192_NR 12
#define AES_256_NR 14

#define AES_128_RK_WORDS 44
#define AES_192_RK_WORDS 52
#define AES_256_RK_WORDS 60

//! Number of bytes in the expanded AES 128 key
#define AES_128_RK_BYTES (4 * AES_128_RK_WORDS)
#define AES_192_RK_BYTES (4 * AES_192_RK_WORDS)
#define AES_256_RK_BYTES (4 * AES_256_RK_WORDS)

/*!
@brief Key expansion function for the AES 128 parametrisation - encrypt
@param [out] rk - The expanded key schedule
@param [in]  ck - The cipher key to expand
*/
void aes_128_enc_key_schedule(
    uint32_t* const rk,
    uint8_t* const ck);

/*!
@brief Key expansion function for the AES 192 parametrisation - encrypt
@param [out] rk - The expanded key schedule
@param [in]  ck - The cipher key to expand
*/
void aes_192_enc_key_schedule(
    uint32_t* const rk,
    uint8_t* const ck);

/*!
@brief Key expansion function for the AES 256 parametrisation - encrypt
@param [out] rk - The expanded key schedule
@param [in]  ck - The cipher key to expand
*/
void aes_256_enc_key_schedule(
    uint32_t* const rk,
    uint8_t* const ck);

/*!
@brief Key expansion function for the AES 128 parametrisation - decrypt
@param [out] rk - The expanded key schedule
@param [in]  ck - The cipher key to expand
*/
void aes_128_dec_key_schedule(
    uint32_t* const rk,
    uint8_t* const ck);

/*!
@brief Key expansion function for the AES 192 parametrisation - decrypt
@param [out] rk - The expanded key schedule
@param [in]  ck - The cipher key to expand
*/
void aes_192_dec_key_schedule(
    uint32_t* const rk,
    uint8_t* const ck);

/*!
@brief Key expansion function for the AES 256 parametrisation - decrypt
@param [out] rk - The expanded key schedule
@param [in]  ck - The cipher key to expand
*/
void aes_256_dec_key_schedule(
    uint32_t* const rk,
    uint8_t* const ck);

/*!
@brief single-block AES 128 encrypt function
@param [out] ct - Output cipher text
@param [in]  pt - Input plaintext
@param [in]  rk - The expanded key schedule
@param [in]  nr - Number of encryption rounds to perform.
*/
void aes_128_block_encrypt(
    uint8_t ct[AES_BLOCK_BYTES],
    uint8_t pt[AES_BLOCK_BYTES],
    uint32_t* rk);

/*!
@brief single-block AES 192 encrypt function
@param [out] ct - Output cipher text
@param [in]  pt - Input plaintext
@param [in]  rk - The expanded key schedule
@param [in]  nr - Number of encryption rounds to perform.
*/
void aes_192_block_encrypt(
    uint8_t ct[AES_BLOCK_BYTES],
    uint8_t pt[AES_BLOCK_BYTES],
    uint32_t* rk);

/*!
@brief single-block AES 256 encrypt function
@param [out] ct - Output cipher text
@param [in]  pt - Input plaintext
@param [in]  rk - The expanded key schedule
@param [in]  nr - Number of encryption rounds to perform.
*/
void aes_256_block_encrypt(
    uint8_t ct[AES_BLOCK_BYTES],
    uint8_t pt[AES_BLOCK_BYTES],
    uint32_t* rk);

/*!
@brief single-block AES 128 decrypt function
@param [out] pt - Output plaintext
@param [in]  ct - Input cipher text
@param [in]  rk - The expanded key schedule
@param [in]  nr - Number of decryption rounds to perform.
*/
void aes_128_block_decrypt(
    uint8_t pt[AES_BLOCK_BYTES],
    uint8_t ct[AES_BLOCK_BYTES],
    uint32_t* rk);

/*!
@brief single-block AES 192 decrypt function
@param [out] pt - Output plaintext
@param [in]  ct - Input cipher text
@param [in]  rk - The expanded key schedule
@param [in]  nr - Number of decryption rounds to perform.
*/
void aes_192_block_decrypt(
    uint8_t pt[AES_BLOCK_BYTES],
    uint8_t ct[AES_BLOCK_BYTES],
    uint32_t* rk);

/*!
@brief single-block AES 256 decrypt function
@param [out] pt - Output plaintext
@param [in]  ct - Input cipher text
@param [in]  rk - The expanded key schedule
@param [in]  nr - Number of decryption rounds to perform.
*/
void aes_256_block_decrypt(
    uint8_t pt[AES_BLOCK_BYTES],
    uint8_t ct[AES_BLOCK_BYTES],
    uint32_t* rk);

#endif // __RV_AES_ASM_H__

//! @}