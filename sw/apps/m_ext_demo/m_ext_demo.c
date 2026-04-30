/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)   2026 Tristan Kundrat <tristan.kundrat@tha.de>
                       Niklas Sirch <niklas.sirch1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This application is a demo. It calculates a multiplication of various numbers.

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

#include <stdio.h>
#include <stdint.h>

// Peripheral Addresses
#define PN_CFG_CLINT_BASE_ADDRESS 0x2000000
#define CLINT_REG_MTIME_LOWER_BITS (PN_CFG_CLINT_BASE_ADDRESS + 0xBFF8)
#define CLINT_REG_MTIME_HIGHER_BITS (PN_CFG_CLINT_BASE_ADDRESS + 0xBFFC)


// The following functions use __attribute__((noipa)) to disable all
// optimizations regarding these functions.
// This forces the compiler to compile these into seperate functions, that
// use the mul*/div*/rem* instructions, when compiled with M extension.

// translates to MULHSU and MUL
__attribute__((noipa)) int64_t mul_us_runtime(uint32_t a, int32_t b)
{
    return (int64_t)a * (int64_t)b;
}

// translates to MULHU and MUL
__attribute__((noipa)) uint64_t mul_uu_runtime(uint32_t a, uint32_t b)
{
    return (uint64_t)a * (uint64_t)b;
}

// translates to MULH and MUL
__attribute__((noipa)) int64_t mul_ss_runtime(int32_t a, int32_t b)
{
    return (int64_t)a * (int64_t)b;
}

// translates to DIV
__attribute__((noipa)) int32_t div_runtime(int32_t a, int32_t b)
{
    return a / b;
}

// translates to DIVU
__attribute__((noipa)) uint32_t divu_runtime(uint32_t a, uint32_t b)
{
    return a / b;
}

// translates to REM
__attribute__((noipa)) int32_t rem_runtime(int32_t a, int32_t b)
{
    return a % b;
}

// translates to REMU
__attribute__((noipa)) uint32_t remu_runtime(uint32_t a, uint32_t b)
{
    return a % b;
}

// reads the 64 bit RISC-V mtime register
static uint64_t read_time_ticks()
{
  volatile uint32_t* mtime_lower_bits = (volatile uint32_t*)CLINT_REG_MTIME_LOWER_BITS;
  volatile uint32_t* mtime_higher_bits = (volatile uint32_t*)CLINT_REG_MTIME_HIGHER_BITS;

  uint32_t lower_bits = *mtime_lower_bits;
  uint32_t higher_bits = *mtime_higher_bits;

  return ((uint64_t)higher_bits << 32) | lower_bits;
}

int main()
{
    uint32_t start_b = 40000;
    uint32_t start_a = 500000;

    // Save start ticks for benchmark
    uint64_t t0 = read_time_ticks();

    for(uint32_t a = start_a; a < start_a + 10000; a += 1000)
    {
        for(uint32_t b = start_b; b < start_b + 10000; b += 1000)
        {
            int32_t a_neg = -a;
            int32_t b_neg = -b;

            // Positive numbers
            printf("%u * %u = (uu)%lli = (us)%lli = (ss)%lli\n",
                   a,
                   b,
                   mul_uu_runtime(a, b),
                   mul_us_runtime(a, (int32_t)b),
                   mul_ss_runtime((int32_t)a, (int32_t)b)
               );
            printf("%u / %u = (u)%u = (s)%i\n",
                   a,
                   b,
                   divu_runtime(a, b),
                   div_runtime((int32_t)a, (int32_t)b)
               );
            printf("%u %% %u = (u)%u = (s)%i\n",
                   a,
                   b,
                   remu_runtime(a, b),
                   rem_runtime((int32_t)a, (int32_t)b)
               );

            // Positive and negative
            printf("%u * %d = (us)%lli = (ss)%lli\n",
                   a,
                   b_neg,
                   mul_us_runtime(a, b_neg),
                   mul_ss_runtime((int32_t)a, b_neg)
               );
            printf("%u / %d = (s)%i\n",
                   a,
                   b_neg,
                   div_runtime((int32_t)a, b_neg)
               );
            printf("%d %% %u = (s)%i\n",
                   a_neg,
                   b,
                   rem_runtime(a_neg, (int32_t)b)
               );

            // negative
            printf("%d * %d = (ss)%lli\n",
                   a_neg,
                   b_neg,
                   mul_ss_runtime(a_neg, b_neg)
               );
            printf("%d / %d = (s)%i\n",
                   a_neg,
                   b_neg,
                   div_runtime(a_neg, b_neg)
               );
        }
    }

    // Save end number of ticks for benchmark
    uint64_t t1 = read_time_ticks();

    // Print result
    printf("\nCalculation took %llu ticks.\n", (t1 - t0));

    return 0;
}
