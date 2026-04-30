/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2023    Felix Wagner <felix.wagner@hs-augsburg.de>
                2025    Lukas Bauer  <lukas.bauer1@tha.de>
                2025    Martin Erichsen <martin.erichsen@tha.de>
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

#include "framebuffer_ram.h"

// **************** Dual Port Block Ram ******************
void m_framebuffer_ram::pn_trace(sc_trace_file* tf, int level)
{
    // Port A
    PN_TRACE(tf, clka);
    PN_TRACE(tf, wea);
    PN_TRACE(tf, ena);

    PN_TRACE(tf, addra);
    PN_TRACE(tf, dia);
    PN_TRACE(tf, doa);

    // Port B
    PN_TRACE(tf, clkb);
    PN_TRACE(tf, web);
    PN_TRACE(tf, enb);

    PN_TRACE(tf, addrb);
    PN_TRACE(tf, dib);
    PN_TRACE(tf, dob);
}

/**
 * @brief debugging print function
 */
[[maybe_unused]] static void _print_framebuffer(sc_uint<8>* ram, int width= 40, int height= 30)
{
    printf("\n");
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            int index = (y * width) + x;

            if(ram[index] > 0)
            {
                printf("█");
            }
            else
            {
                printf("░");
            }
        }
        printf("\n");
    }
}

void m_framebuffer_ram::proc_clk_ram_port_a()
{
    while(1)
    {
        wait();

        if(ena == 1)
        {
            PN_ASSERTF(addra.read() < PN_CFG_VIDEO_FB_SIZE,
                ("addra exceeds the available address range: valid: 0..%d, got: %d",
                    PN_CFG_VIDEO_FB_SIZE,
                    (unsigned int)addra.read()));

            // Write the data at dia to the ram at address addra when wea and ena are high
            if(wea == 1)
            {
                ram[addra.read()] = dia;
            }

            // Read the data from the ram at address addra when ena is high and output it to doa
            doa = ram[addra.read()];
        }
    }
}

void m_framebuffer_ram::proc_clk_ram_port_b()
{
    while(1)
    {
        wait();

        if(enb == 1)
        {
            PN_ASSERTF(addrb.read() < PN_CFG_VIDEO_FB_SIZE,
                ("addrb exceeds the available address range: valid: 0..%d, got: %d",
                    PN_CFG_VIDEO_FB_SIZE,
                    (unsigned int)addrb.read()));

            // Write the data at dib to the ram at address addrb when web and enb are high
            if(web == 1)
            {
                ram[addrb.read()] = dib;
            }

            // Read the data from the ram at address addrb when enb is high and output it to dob
            dob = ram[addrb.read()];
        }
    }
}
