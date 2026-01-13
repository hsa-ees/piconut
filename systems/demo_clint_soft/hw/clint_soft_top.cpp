/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
                2025 Gundolf Kiefer <gundolf.kiefer@tha.de>

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

#include "clint_soft_top.h"

void m_demo_clint_soft::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {
        cpu->pn_trace(tf, level);
    }
    // Internal traces
}

void m_demo_clint_soft::init_submodules()
{

    // ----------- Create submodules -----------
    // ----------- PicoNut -----------
    cpu = sc_new<m_cpu>("i_cpu");
    cpu->clk(clk);
    cpu->reset(reset);

    cpu->debug_haltrequest_in(dummy_low);
    // Issue: vh_const not working right now. Uncomment the line below
    // after vh_const updated and remove dummy signal.
    // cpu->debug_haltrequest_in(vh_const<bool>(0));
    cpu->debug_haltrequest_ack_out(vh_open);

    // Connect the interrupt signals to the PicoNut processor
    cpu->mtip_in(mtip_signal);
    cpu->msip_in(msip_signal);
    cpu->meip_in(meip_signal);
}

void m_demo_clint_soft::proc_cmb()
{
    dummy_low.write(0);
}