/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)   2025 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include <stdio.h>
#include <stdint.h>

/*
 * SDRAM Memory Test Program
 *  This program performs a basic read/write verification test on three specific 
 *  regions of an external SDRAM:

 *      1. Lower Region:  First 1280 positions (Offsets 0x0 to 0x4FF)
 *      2. Middle Region: Middle 1280 positions (Offsets 0x400000 to 0x4004FF)
 *      3. Upper Region:  Last 1280 positions (Offsets 0x7FFB00 to 0x7FFFFF)

 *  For each region, the program:
 *      - Writes a unique sequential pattern (the loop index `i`) to the memory addresses.
 *      - Reads the values back to verify they match the written pattern.
 *      - Tracks and prints any data mismatches as errors (`err_cont`).
 *      - Displays periodic progress updates to the console via printf.
 */

int main()
{
	// Memory pointer for the current test address
    volatile uint32_t* p_sdram;
             uint32_t  num            = 0; 

    // Base address of the SDRAM region (0x40000000)
             uint32_t* p_sdram_start  = ((uint32_t*)(0x40000000U));

    // Counter to track detected memory errors
             uint32_t  err_cont       = 0;

    printf("start the Test\n");


    // =========================================================================
    // REGION 1: Writing to and reading from the first 1280 addresses (0x0 to 0x500)
    // =========================================================================

    // Write phase
    for(uint32_t i = 0x0; i < 0x501; ++i)
    {
        p_sdram    = p_sdram_start + i;
       *p_sdram    = i; 

        if(i % 0x100 == 0)
        {
            printf("write index to address: 0x%x 0x%x\n", i, p_sdram);
        }
    }
    
    printf("write done\n");
    
    // Read & Verification phase
    for(uint32_t i = 0x0; i < 0x501; ++i)
    {

        p_sdram = p_sdram_start + i;

        if(*p_sdram != i)
        {
            printf("!!!FALSE value at 0x%x: 0x%x\n", p_sdram, *p_sdram);
            err_cont ++;
        }

        if(i % 0x100 == 0)
        {
            printf("read index from address: 0x%x 0x%x\n", i, p_sdram);

        }

    }

    printf("read done\n");
    printf("first address done\n");
    
    // =========================================================================
    // REGION 2: The middle area (Offsets 0x400000 to 0x400500)
    // =========================================================================

    // Write phase
    for(uint32_t i = 0x400000; i < 0x400501; ++i)
    {
        p_sdram = p_sdram_start + i;
        *p_sdram = i; 

        if(i % 0x100 == 0)
        {
            printf("write to address: 0x%x\n", p_sdram);
        }
    }
    
    printf("write done\n");
    
    // Read & Verification phase
    for(uint32_t i = 0x400000; i < 0x400501; ++i)
    {

        p_sdram = p_sdram_start + i;

        if(*p_sdram != i)
        {
            printf("!!!FALSE value at 0x%x: 0x%x\n", p_sdram, *p_sdram);
            err_cont ++;
        }

        if(i % 0x100 == 0)
        {
            printf("read index from address: 0x%x 0x%x\n", i, p_sdram);

        }

    }

    printf("read done\n");

    printf("Middle address done\n");


    // The last 1280 Address are written to and read from

    // Write phase
    for(uint32_t i = 0x7FFB00; i < 0x800000; ++i)
    {
        p_sdram = p_sdram_start + i;
        *p_sdram = i; 

        if(i % 0x100 == 0)
        {
            printf("write to address: 0x%x\n", p_sdram);
        }
    }
    
    printf("write done\n");
    
    // =========================================================================
    // REGION 3: The upper/last area (Offsets 0x7FFB00 to 0x7FFFFF)
    // =========================================================================

    // Read & Verification phase
    for(uint32_t i = 0x7FFB00; i < 0x7FFF0F; ++i)
    {                                 

        p_sdram = p_sdram_start + i;

        if(*p_sdram != i)
        {
            printf("!!!FALSE value at 0x%x: 0x%x\n", p_sdram, *p_sdram);
            err_cont ++;
        }

        if(i % 0x100 == 0)
        {
            printf("read index from address: 0x%x 0x%x\n", i, p_sdram);

        }

    }

    printf("read done\n");

    printf("Last address done\n");


    // =========================================================================
    // FINAL EVALUATION: Display test results
    // =========================================================================
    if(err_cont != 0) printf("you have %d errors\n");

    printf("The test is done\n");

    return 0;
}
