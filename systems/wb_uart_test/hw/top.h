/**
 * @file top.h
 *
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@tha.de>
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
#include "elab_alloc.h"

// Include the header files of the submodules
#include "uart_test.h"



/**
 * @fn SC_MODULE(m_top)
 * @brief wrapper for synthesis
 * @author Lukas Bauer
 *
 * This module is just the top level wrapper for synthesis
 *
 * @par Ports:
 * @param[in] clk_25 the clock signal comming form the board
 * @param[in] reset the reset signal of the module
 * @param[in] rx the receive signal of the uart
 * @param[out] tx the transmit signal of the uart
 *
 */
SC_MODULE(m_top)
{
public:
   // Ports
   sc_in_clk PN_NAME(clk_25);
   sc_in<bool> PN_NAME(reset);
   sc_in<bool> PN_NAME(rx);
   sc_out<bool> PN_NAME(tx);


   // Constructor/Destructors
   SC_CTOR(m_top)
   {

      init_submodules();
   }

   // Functions
   void Trace(sc_trace_file * tf, int level = 1);

   // Processes

   // Submodules
   m_uart_test *uart_test_inst;

protected:

   // MethodsI
   void init_submodules();

};

#endif // __TOP_H__