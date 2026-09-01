/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) Hermann Zoha <hermann.zoha@tha.de>
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
 * @file sdram_controller.h
 *
 * @defgroup sdram_controller SDRAM-Controller Module
 * @{
 */


#ifndef __SDRAM_CONTROLLER_H__
#define __SDRAM_CONTROLLER_H__


#include "sdram_controller_defs.h"
#include <systemc.h>
#include <piconut.h>

/**
 * @class m_sdram_controller
 * @brief ## SDRAM-Controller peripheral module.
 *
 * @details
 * This module is the hardware-wishbone SLAVE unit which can be used for send Data to an external SDRAM.
 *
 * This implementation illustrates the use of a Wishbone slave that accesses external memory—in this case, SDRAM. 
 *
 *
 *
 @par SDRAM-Controller Ports:
 * @param[in]       clk                 Clock of the module
 * @param[in]       reset               Reset of the module
 * @param[in, out]  wb_slave            Wishbone slave interface
 * @param[out]      sdram_clk           Clock for SDRAM
 * @param[out]      sdram_clk_en        Clock enable for SDRAM
 * @param[out]      sdram_cp_sel_n      Chip select active Low
 * @param[out]      sdram_wr_en_n       Write Enable active low
 * @param[out]      sdram_col_ac_sel_n  Column active select active low
 * @param[out]      sdram_row_ac_sel_n  Row Active Select Sdram active low
 * @param[out]      sdram_addr          Address for SDRAM  (Colomne & Row address)
 * @param[out]      sdram_ba            Sdram Bank address  
 * @param[out]      c_we_o              Write enable signal for sdram_tri_state
 * @param[in]       sdram_dq_i          Data in bus form sdram_tri_state 
 * @param[out]      sdram_dq_o          Data out bus for sdram_tri_state
 * @param[out]      sdram_dqm           Sdram data mask
 *
 *
 */


SC_MODULE(m_sdram_controller), pn_module_if
{

public:
    // wishbone slave ports ...
    sc_in_clk               PN_NAME(clk);                   // clock input
    sc_in<bool>             PN_NAME(reset);                 // reset
    
    pn_wishbone_slave_t wb_slave;

    /** Ports: SDRAM */
    sc_out<bool>            PN_NAME(sdram_clk);             // Clock to Sdram
    sc_out<bool>            PN_NAME(sdram_clk_en);          // Clock enable

    sc_out<bool>            PN_NAME(sdram_cp_sel_n);        // Chip select active low
    sc_out<bool>            PN_NAME(sdram_wr_en_n);         // Write Enable SDRAM active low
    sc_out<bool>            PN_NAME(sdram_col_ac_sel_n);    // Column Active Select Sdram active low
    sc_out<bool>            PN_NAME(sdram_row_ac_sel_n);    // Row Active Select Sdram active low

    sc_out<sc_uint<13>>     PN_NAME(sdram_addr);            // sdram address clom & row
    sc_out<sc_uint<2>>      PN_NAME(sdram_ba);              // sdram bank  
    sc_out<bool>            PN_NAME(c_we_o);                // Write enable signal for sdram_switch
    sc_in<sc_uint<16>>      PN_NAME(sdram_dq_i);            // data bus for sdram_switch
    sc_out<sc_uint<16>>     PN_NAME(sdram_dq_o);            // data bus for sdram_switch
    sc_out<sc_uint<2>>      PN_NAME(sdram_dqm);             // sdram data mask

    /**
     * @brief Constructor.
     *
     * Initializes the SDRAM-Contoller module with the specified base address and pin configuration.
     *
     * @fn m_sdram_controller (sc_module_name, pn_wb_adr_t, const uint8_t, const uint8_t)
     *
     * @param[in] name           Module name for identification in waveform dumps
     * @param[in] base_address   Base address for Wishbone register space
     * @param[in] num_inputs     Number of input pins to enable (default: 0)
     * @param[in] num_outputs    Number of output pins to enable (default: 0)
     *
     */

    SC_HAS_PROCESS(m_sdram_controller);
    m_sdram_controller(
        sc_module_name name,
        pn_wb_adr_t base_address = PN_CFG_SDRAM_CONTROLLER_BASE_ADDRESS)
        : sc_module(name)
        , wb_slave{
              .alen = 32,
              .dlen = 32,
              .base_address = base_address,
              .size = PN_CFG_SDRAM_CONTROLLER_SIZE}
    {
        pn_add_wishbone_slave(&wb_slave);

        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_slave.stb_i
                  << wb_slave.we_i
                  << wb_slave.adr_i
                  << c_current_state
                  << wb_slave.sel_i
                  << c_cnt_react
                  << c_dat_merg
                  << c_rfrsh_strt
                  << c_ba_buf 
                  << c_dq_buf_1
                  << c_col_buf
                  << c_row_buf
                  << wb_slave.cyc_i
                  << c_dq_buf_2;

        SC_METHOD(proc_comb_clk_to_sdram); //Clock from Wishbone to SDRAM-Modul
        sensitive << clk;

        SC_CTHREAD(proc_c_cntr_refrsh, clk.pos()); // Counter for refresh the SDRAM
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_c_cntr_nops, clk.pos()); // proces to count after activate
        reset_signal_is(reset,true);

        SC_CTHREAD(proc_clk_state, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_buffer_write, clk.pos()); // create clock sensitive process
        reset_signal_is(reset, true);

    }

    /**
     * @brief Generate waveform trace.
     *
     * Records all internal signals and ports to a trace file.
     *
     * @param[in,out] tf  Pointer to open sc_trace_file.
     * @param[in] level   Trace level (default: 1).
     */

    void pn_trace(
        sc_trace_file * tf,
        int level = 1);
    /**
     * @brief Wishbone state register */
    void proc_clk_state();

    /**
     * @brief Wishbone slave logic. 
     */
    void proc_comb_wb_slave();

    /**
     * @brief SDRAM-Controller send invertet system clock to SDRAM. 
     */
    void proc_comb_clk_to_sdram();

    /**
     * @brief SDRAM-Controller waitning counter. 
     */
    void proc_c_cntr_nops();

    /**
     * @brief SDRAM-Controller refresh counter. 
     */
    void proc_c_cntr_refrsh();

    /**
     * @brief SDRAM-Controller data and address handler. 
     */
    void proc_buffer_write();

    /** @} */ // end of Processes

protected:

    // SDRAM-Controller states
    enum e_wb_state
    {
        SD_INIT_WAIT = 0,                   //0
        SD_INIT_PRECHARGE,                 //1
        SD_INIT_REFRESH,                    //2
        SD_INIT_REFRESH_2,                  //3
        SD_INIT_NOP_1,                      //4
        SD_INIT_NOP_2,                      //5
        SD_INIT_NOP_3,                      //6
        SD_INIT_NOP_4,                      //7
        SD_INIT_MODE,                       //8
        SD_INIT_IDEL,                       //9
        WB_IDLE,                            //10
        WB_READ,                            //11
        SD_ACTIVE_FOR_READ_0,               //12
        SD_ACTIVE_FOR_WRITE_0,              //13
        SD_ACTIVE_FOR_READ_1,               //14
        SD_ACTIVE_FOR_WRITE_1,              //15
        SD_NOP_AFTER_AKTIVE_READ_1,         //16
        SD_NOP_AFTER_AKTIVE_READ_2,         //17
        SD_NOP_AFTER_AKTIVE_READ_3,         //18
        SD_NOP_AFTER_AKTIVE_WRITE_1,        //19
        SD_NOP_AFTER_AKTIVE_WRITE_2,        //20
        SD_NOP_AFTER_AKTIVE_WRITE_3,        //21
        SD_READ,                            //22
        SD_WRITE,                           //23
        SD_NOP_AFTER_READ_1,                //24
        SD_NOP_AFTER_READ_2,                //25
        SD_NOP_AFTER_READ_3,                //26
        SD_NOP_AFTER_READ_4,                //27
        SD_NOP_AFTER_WRITE_1,               //28
        SD_NOP_AFTER_WRITE_2,               //29
        SD_PRECHARGE,                      //30
        SD_NOP_AFTER_PRECHARGE_1,           //31
        SD_NOP_AFTER_PRECHARGE_2,           //32
        SD_NOP_FOR_REFRESH,                 //33
        SD_REFRESH,                         //34
        SD_NOP_AFTER_REFRESH                //35

    };
    /** Registers... */
    sc_signal<sc_uint<8>>   PN_NAME(c_current_state); 
    sc_signal<sc_uint<8>>   PN_NAME(c_next_state);
    sc_signal<bool>         PN_NAME(c_write_en);
    sc_signal<bool>         PN_NAME(c_read_en);

    sc_signal<sc_uint<13>>  PN_NAME(c_row_buf);      //sdram address row buffer
    sc_signal<sc_uint<9>>   PN_NAME(c_col_buf);      //sdram address clom buffer
    sc_signal<sc_uint<2>>   PN_NAME(c_ba_buf);        //sdram bank buffer
    sc_signal<sc_uint<16>>  PN_NAME(c_dq_buf_1);       //sdram data buffer
    sc_signal<sc_uint<16>>  PN_NAME(c_dq_buf_2);      //sdram data2 buffer
    sc_signal<sc_uint<32>>  PN_NAME(c_dat_merg);       //for assambel the data buffer
    sc_signal<sc_uint<2>>   PN_NAME(c_dqm_buf);      //sdram dqml buffer
    sc_signal<sc_int<22>>   PN_NAME(c_cntr_refrsh);    //count clk for Refresh
    
    sc_signal<bool>         PN_NAME(c_rfrsh_strt);
    sc_signal<bool>         PN_NAME(c_rfrsh_end);
    sc_signal<sc_uint<4>>   PN_NAME(c_cntr_nops);
    sc_signal<bool>         PN_NAME(c_react_cnt_nop);
    sc_signal<bool>         PN_NAME(c_cnt_nop_init);
    sc_signal<bool>         PN_NAME(c_cnt_nop_befor_rfrsh);
    sc_signal<bool>         PN_NAME(c_cnt_nop_after_rfrsh);
    sc_signal<sc_uint<2>>   PN_NAME(c_cnt_react);
    
    sc_signal<bool>         PN_NAME(c_rd_buf_1);
    sc_signal<bool>         PN_NAME(c_rd_buf_2);
    sc_signal<bool>         PN_NAME(c_merg_dat);

};


#endif //__SDRAM_CONTROLLER_H__