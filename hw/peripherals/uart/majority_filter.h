/**
 * @file majority_filter.h
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
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
/**
 * @fn SC_MODULE(m_majority_filter)
 * @author Lukas Bauer
 * @brief majority filter to filter a input signal
 *
 * This module allows checking if a signal remains stable for a certain amount of time.
 * If the module is enabled, the input signal is checked every clock cycle.
 * When the signal is low, the counter is incremented. Once the threshold is reached, the output signal is set to low.
 * Otherwise, the output signal remains high.
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] filter_i Signal to be filtered
 * @param[in] capture_i Enable Signal to capture the filter signal
 * @param[in] clear_i Clear the filter signal
 * @param[out] filter_o Filtered Signal
 *
 *
 */

#ifndef __MAJORTIY_FILTER_H__
#define __MAJORTIY_FILTER_H__

#include <systemc.h>
#include <piconut.h>

/// @brief Threshold value used for the majority filter.
#define MAJORITY_FILTER_THRESHOLD 10

SC_MODULE(m_majority_filter)
{
public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk PN_NAME(clk);     // clock of the module
    sc_in<bool> PN_NAME(reset); // reset of the module

    sc_in<bool> PN_NAME(filter_i);  // Signal to be filtered
    sc_in<bool> PN_NAME(capture_i); // Enable Signal to capture the filter signal
    sc_in<bool> PN_NAME(clear_i);   // Clear the filter signal
    sc_out<bool> PN_NAME(filter_o); // Filtered Signal

    /* Constructor... */
    SC_CTOR(m_majority_filter)
    {
        SC_CTHREAD(proc_clk_module, clk.pos());
        reset_signal_is(reset, true);
    }

    /* Functions...*/
    /**
     * @brief this function is used to generate a tracefile
     * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
     * use PN_TRACE(tf, signalname) inside the cpp file to add whatever Signal u need to trace.
     * @param tf this is the tracefile object
     * @param level is used as a selector when to enable a trace*/
    void pn_trace(sc_trace_file * tf, int level = 1);

    /** Processes...
     */

    void proc_clk_module();

protected:
    /** Registers...
     * This are examples of Module Registers
     * you may add your own Registers here*/

    sc_signal<sc_uint<4>> PN_NAME(sample_cnt); // Counter for the samples captured
};
#endif // __MAJORTIY_FILTER_H__