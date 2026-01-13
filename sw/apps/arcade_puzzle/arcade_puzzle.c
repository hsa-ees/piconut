/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
      Technische Hochschule Augsburg, University of Applied Sciences

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
#include <stdlib.h>
#include "puzzle.h"
#include "puzzle_melody_syn.h"

#include <uart_defs.h>
#include <clint_defs.h>

const uint32_t BAUDRATE = 460800;
const uint32_t UART_CLOCK_DIVIDER = 25000000 / BAUDRATE;

#define WIDTH 12
// UART buffer too small for more
#define HEIGHT 12
#define THREE_SECOND_DELAY 1500000

uint32_t get_time_lo()
{
    volatile uint32_t* mtime_lo = (volatile uint32_t*)CLINT_REG_MTIME_LO;
    return *mtime_lo;
}

int main(void)
{
    printf("Set Baudrate to %d            \n\n\n", BAUDRATE); // print more character so they come through before changing baudrate
    volatile uint32_t* uart_clock_divider = (uint32_t*)(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_DIV);
    *(uart_clock_divider) = UART_CLOCK_DIVIDER;

    for(int i = 0; i < THREE_SECOND_DELAY; i++)
    {
        asm volatile("nop");
    }

    int32_t seed = 0;

    puzzle_melody_syn_background();
    while(1)
    {
        printf("\n=====================================\n");
        printf("       Puzzle Game on Piconut!       \n");
        printf("=====================================\n");
        printf("Controls:\n");
        printf(" a - move left\n");
        printf(" d - move right\n");
        printf(" s - move down\n");
        printf("space - rotate block\n");
        printf("=====================================\n");

        printf("Press any key to start...\n");
        volatile uint32_t* uart_rxdata = (uint32_t*)(PN_CFG_UART_BASE_ADDRESS + WB_UART_REG_RXDATA);
        uint32_t last_uart_rxdata;
        while((((last_uart_rxdata = *uart_rxdata) >> 31) & 0x1) == 1)
        {
        }

        seed++;
        seed += get_time_lo();
        puzzle_run(WIDTH, HEIGHT, seed);
    }
    return EXIT_FAILURE;
}