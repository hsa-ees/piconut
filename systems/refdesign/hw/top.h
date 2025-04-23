/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
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


#ifndef __TOP_H__
#define __TOP_H__

#include <systemc.h>
#include <base.h>
#include "elab_alloc.h"

// Include the header files of the submodules
#include "piconut.h"

SC_MODULE(m_top)
{
public:
   // Ports

   sc_in_clk PN_NAME(clk);
   sc_in<bool> PN_NAME(reset);

   // Constructor/Destructors
   SC_CTOR(m_top)
   {
      SC_METHOD(proc_cmb);

      init_submodules();
   }

   // Functions
   void Trace(sc_trace_file * tf, int level = 1);

   // Processes
   void proc_cmb();

   // Submodules
   m_piconut *piconut;

protected:

   sc_signal<bool> PN_NAME(dummy_low);

   // Methods
   void init_submodules();
};

#endif // __TOP_H__