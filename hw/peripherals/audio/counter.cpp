/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Tristan Kundrat <tristan.kundrat@tha.de>
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

#include "counter.h"

void m_counter::proc_clk()
{
  count = 0;
  amplitude = 0;
  while (true)
  {
    wait();

    sc_uint<24> new_amplitude = 0;
    sc_uint<32> new_count = 0;

    if (enable.read() && count.read() < frequency_divisor.read())
    {
      new_count = count.read() + 1;

      if (negative_step.read())
      {
        new_amplitude = (sc_uint<24>)((sc_int<25>)amplitude.read() - (sc_int<25>)step_height.read());
        if (new_amplitude > amplitude.read())
          new_amplitude = 0;
      }
      else
      {
        new_amplitude = (sc_uint<24>)((sc_int<25>)amplitude.read() + (sc_int<25>)step_height.read());
        if (new_amplitude < amplitude.read())
          new_amplitude = 0xFFFFFF;
      }

    }
    count = new_count;
    amplitude = new_amplitude;
  }
}
