/**
 * @file wishbone_template.h
 * @brief This file contains the definition of the m_wishbone_template module.
 * @author Claus Janicher
 * @author Johannes Hofmann
 * @author Lukas Bauer
 * @author Tristan Kundrat
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Lukas Bauer <lukas.bauer1@tha.de>
                     2025 Tristan Kundrat <tristan.kundrat@tha.de>
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
 * @fn SC_MODULE(m_wishbone_t)
 *
 * This module is the hardware-wishbone SLAVE unit which can be used for the hardware implementation of any systembus peripherals.
 * It is the interface between the wishbone master (currently membrana_wb) and your custom peripheral module.
 *
 * This Implementation shows the wishbone slave usage with three subaddresses. This means the master can directly access your slave subregisters, that are defined in the e_wb_addressrange enumeration type.
 *
 *
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] wb_adr_i Wishbone Adress input
 * @param[out] wb_dat_o Wishbone Data out
 * @param[in] wb_dat_i Wishbone Data in
 * @param[in] wb_we_i Wishbone Write-Enable in
 * @param[in] wb_stb_i Wishbone Strobe in
 * @param[in] wb_cyc_i Wishbone Cycle in
 * @param[out] wb_ack_o Wishbone ACK input
 * @param[in] wb_cti_i // Wishbone cycle type identifier (optional, for registered feedback)
 * @param[in] wb_bte_i // Wishbone burst type extension  (optional, for registered feedback)
 * @param[out] wb_err_o // Wishbone termination w/ error (optional)
 * @param[out] wb_rty_o // Wishbone termination w/ retry (optional)
 *
 *
 */

#ifndef __WISHBONE_T_H__
#define __WISHBONE_T_H__

#include <systemc.h>
#include <piconut.h>

#ifndef WB_DAT_WIDTH
#define WB_DAT_WIDTH 32
#endif

#ifndef WB_ADR_WIDTH
#define WB_ADR_WIDTH 32
#endif

// this should late on be set in the configuration and not in this module, for template and explanation purposes only.
#define CFG_WB_SLAVE_TEMPLATE_ADDRESS 0xF0000000U
#define WB_SLAVE_TEMPLATE_SIZE (3U << 2)

SC_MODULE(m_wishbone_t)
{

public:
    // wishbone states, if you need further ones just add them
    enum e_wb_state
    {
        WB_IDLE = 0,
        WB_READ,
        WB_WRITE1,
        WB_WRITE2

    };

    // for the different wishbone addresses this slave has. You can use one address as well, but with this implementation you have "multiple addresses" for yourself
    enum e_wb_addressrange
    {
        ADR_REG_0 = 0x0,
        ADR_REG_1 = 0x4,
        ADR_REG_2 = 0x8,

    };

    // wishbone slave ports ...
    sc_in_clk PN_NAME(clk);     // clock input
    sc_in<bool> PN_NAME(reset); // reset

    sc_in<bool> PN_NAME(wb_stb_i); // strobe input
    sc_in<bool> PN_NAME(wb_cyc_i); // cycle valid input
    sc_in<bool> PN_NAME(wb_we_i);  // indicates write transfer
    // Optional uncomment if wb slave implement these. See wishbone spec.
    // sc_in<sc_uint<3>> PN_NAME(wb_cti_i);                // cycle type identifier (optional, for registered feedback)
    // sc_in<sc_uint<2>> PN_NAME(wb_bte_i);                // burst type extension (optional, for registered feedback)
    sc_in<sc_uint<WB_DAT_WIDTH / 8>> PN_NAME(wb_sel_i); // byte select inputs
    sc_out<bool> PN_NAME(wb_ack_o);                     // normal termination
    sc_out<bool> PN_NAME(wb_err_o);                     // termination w/ error (optional)
    sc_out<bool> PN_NAME(wb_rty_o);                     // termination w/ retry (optional)

    sc_in<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);  // address bus inputs
    sc_in<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);  // input data bus
    sc_out<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o); // output data bus

    SC_CTOR(m_wishbone_t)
    {
        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_stb_i
                  << wb_cyc_i
                  << wb_we_i
                  << wb_adr_i
                  << wb_dat_i
                  << wb_current_state
                  << wb_sel_i
                  << dummy_register_0
                  << dummy_register_1
                  << dummy_register_2;

        SC_CTHREAD(proc_clk_state, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_clk_wb_slave, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);
    }

    void pn_trace(
        sc_trace_file * tf,
        int level = 1);

    /**
     * @brief Handels the wishbone slave functions */
    void proc_clk_state();

    /**
     * @brief Handels the wishbone slave functions */
    void proc_clk_wb_slave();
    void proc_comb_wb_slave();

    /**
     * @brief This function is for the write byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<32> write_with_byte_select(sc_uint<32> input_word);

    /**
     * @brief This function is for the read byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<32> read_with_byte_select(sc_uint<32> input_word);

protected:
    /** Registers... */
    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);

    sc_signal<sc_uint<WB_DAT_WIDTH>> PN_NAME(dummy_register_0);
    sc_signal<sc_uint<WB_DAT_WIDTH>> PN_NAME(dummy_register_1);
    sc_signal<sc_uint<WB_DAT_WIDTH>> PN_NAME(dummy_register_2);

    sc_signal<bool> PN_NAME(c_wb_write_en);
};

#endif //__WISHBONE_T_H__