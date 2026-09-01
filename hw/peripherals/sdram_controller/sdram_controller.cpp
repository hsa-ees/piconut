/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Hermann Zoha <hermann.zoha@tha.de> 
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


 #include "sdram_controller.h"

 
 /**
  * @brief this function is used to generate a tracefile
  * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
  * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
  * @param tf this is the tracefile object
  * @param level is used as a selector when to enable a trace*/
 void m_sdram_controller::pn_trace(sc_trace_file* tf, int level)
 {
 
     PN_TRACE(tf, clk);
     PN_TRACE(tf, reset);
     PN_TRACE(tf, wb_slave.ack_o);
     PN_TRACE(tf, wb_slave.adr_i);
     PN_TRACE(tf, c_current_state);
     PN_TRACE(tf, wb_slave.cyc_i);
     PN_TRACE(tf, wb_slave.dat_i);
     PN_TRACE(tf, wb_slave.dat_o);
     PN_TRACE(tf, wb_slave.sel_i);
     PN_TRACE(tf, wb_slave.we_i);
     PN_TRACE(tf, wb_slave.err_o);
 
     PN_TRACE(tf, sdram_clk);
     PN_TRACE(tf, sdram_clk_en);
     PN_TRACE(tf, sdram_cp_sel_n);
     PN_TRACE(tf, sdram_wr_en_n);
     PN_TRACE(tf, sdram_col_ac_sel_n);
     PN_TRACE(tf, sdram_row_ac_sel_n);
     PN_TRACE(tf, sdram_addr);
     PN_TRACE(tf, sdram_ba);
     
     PN_TRACE(tf, c_we_o);
     PN_TRACE(tf, sdram_dq_i);
     PN_TRACE(tf, sdram_dq_o);
     PN_TRACE(tf, sdram_dqm);
 
     PN_TRACE(tf, c_write_en);
 
     if(level >= 2)
     {
         PN_TRACE(tf, c_current_state);
         PN_TRACE(tf, c_next_state);
         PN_TRACE(tf, c_row_buf);
         PN_TRACE(tf, c_col_buf);
         PN_TRACE(tf, c_ba_buf);
         PN_TRACE(tf, c_dq_buf_1);
         PN_TRACE(tf, c_dq_buf_2);
         PN_TRACE(tf, c_dat_merg);
         PN_TRACE(tf, c_dqm_buf);
         PN_TRACE(tf, c_cntr_refrsh);
         PN_TRACE(tf, c_cntr_nops);
         
         PN_TRACE(tf, c_rfrsh_strt);
         PN_TRACE(tf, c_rfrsh_end);
         PN_TRACE(tf, c_react_cnt_nop);
         PN_TRACE(tf, c_cnt_nop_befor_rfrsh);
         PN_TRACE(tf, c_cnt_nop_after_rfrsh);
         PN_TRACE(tf, c_cnt_react);
 
         PN_TRACE(tf, c_rd_buf_1);
         PN_TRACE(tf, c_rd_buf_2);
         PN_TRACE(tf, c_merg_dat);
     }
 }
 
 
 void m_sdram_controller::proc_comb_wb_slave() // 
 {
     // set idle Outputs to default
     wb_slave.ack_o = 0;
     wb_slave.dat_o = 0;
     wb_slave.err_o = 0;
     wb_slave.rty_o = 0;
 
     sdram_cp_sel_n     = 1;
     sdram_wr_en_n      = 1;
     sdram_col_ac_sel_n = 1;
     sdram_row_ac_sel_n = 1; 
 
     sdram_addr = 0;
     sdram_ba   = 0;
     sdram_dqm  = 0;
 
     c_write_en     = 0;
     c_read_en   = 0;
     
     c_rd_buf_1 = 0;
     c_rd_buf_2 = 0;
     c_merg_dat = 0;
    
     c_cnt_nop_init         = 0;
     c_react_cnt_nop        = 0;
     c_cnt_nop_befor_rfrsh  = 0;
     c_cnt_nop_after_rfrsh  = 0;

     c_rfrsh_end = 0;

     sdram_dq_o = 0;
     c_we_o     = 0;
    

     c_next_state = c_current_state.read(); // default next state is current state

     switch(c_current_state.read())
    {  
        //wait until the SDRAM has initialized
        case SD_INIT_WAIT:

            sdram_dqm = 11;
            c_react_cnt_nop = 1;
            c_cnt_nop_init = 1;

            if(c_cnt_react.read() == 1)
            {
                c_react_cnt_nop = 0;
                c_next_state =  SD_INIT_NOP_1;
            }

        break;
        
        // All banks must be precharged
        case SD_INIT_PRECHARGE:

            sdram_cp_sel_n      = 0;
            sdram_row_ac_sel_n  = 0;
            sdram_wr_en_n       = 0;
            sdram_addr          = SDRAM_CONTROLLER_PRECHARGE;
            c_next_state        = SD_INIT_NOP_2;  

        break;
        
        // needs  two to refresh cycles
        case SD_INIT_REFRESH:

            sdram_cp_sel_n      = 0;
            sdram_row_ac_sel_n  = 0;
            sdram_col_ac_sel_n  = 0;
            c_next_state        = SD_INIT_NOP_3;

        break;

        case SD_INIT_REFRESH_2:

            sdram_cp_sel_n = 0;
            sdram_row_ac_sel_n = 0;
            sdram_col_ac_sel_n = 0;
            c_next_state = SD_INIT_NOP_4;

        break;

        // Each command must die follwed by a NOP
        case SD_INIT_NOP_1:

            sdram_cp_sel_n  = 0;
            c_react_cnt_nop = 1;
            c_cnt_nop_init  = 1;
            sdram_cp_sel_n  = 0;

            if(c_cnt_react.read() == 1)
            {
                c_react_cnt_nop = 0;
                c_next_state    =  SD_INIT_PRECHARGE;
            }

        break;

        case SD_INIT_NOP_2:

            sdram_cp_sel_n          = 0;
            c_react_cnt_nop         = 1;
            c_cnt_nop_befor_rfrsh   = 1;
            sdram_cp_sel_n          = 0;

            if(c_cnt_react.read() == 1)
            {
                c_react_cnt_nop = 1;
                c_next_state    = SD_INIT_REFRESH;
            }
        break;
        
        case SD_INIT_NOP_3:
            sdram_cp_sel_n          = 0;
            c_react_cnt_nop         = 1;
            c_cnt_nop_befor_rfrsh   = 1;
            sdram_cp_sel_n          = 0;
            if(c_cnt_react.read() == 1)
            {
                c_react_cnt_nop = 1;
                c_next_state    =  SD_INIT_REFRESH_2;
            }
        break;
        case SD_INIT_NOP_4:
            sdram_cp_sel_n          = 0;
            c_react_cnt_nop         = 1;
            c_cnt_nop_befor_rfrsh   = 1;
            sdram_cp_sel_n          = 0;
            if(c_cnt_react.read() == 1)
            {
                c_react_cnt_nop = 1;
                c_next_state    =  SD_INIT_MODE;
            }
        break;
        
            
        break;

        // the mode is set hier 
        case SD_INIT_MODE:

            sdram_cp_sel_n      = 0;
            sdram_row_ac_sel_n  = 0;
            sdram_col_ac_sel_n  = 0;
            sdram_wr_en_n       = 0;
            sdram_addr          = SDRAM_CONTROLLER_OP_CODE_INIT;
            c_next_state        = SD_INIT_IDEL;
        break;

        // in this state, the sdram mode is enabled
        case SD_INIT_IDEL:

            c_next_state = WB_IDLE;

        break;


        case WB_IDLE:                     // Idle State to check if we have a valid transaction

            if (c_rfrsh_strt.read() == 1)       // Check if the SDRAM need refresh
            {
                c_next_state = SD_NOP_FOR_REFRESH;    // Next state: issue NOP to SDRAM to start refresh
            
            }else if(wb_slave.stb_i.read() == 1 && wb_slave.cyc_i.read() == 1) // WB Strobe and cycle valid,
            {
                // is the currend adress rigth
                if((PN_CFG_SDRAM_CONTROLLER_BASE_ADDRESS <= wb_slave.adr_i.read()) &&
                    (wb_slave.adr_i.read() < (PN_CFG_SDRAM_CONTROLLER_BASE_ADDRESS + PN_CFG_SDRAM_CONTROLLER_SIZE ))) 
                {
                    // Check if the Wishbone signal wb_we_i indicates a write operation
                    if(wb_slave.we_i.read() == 1)
                    {
                        c_write_en = 1;                         // Enable write for the controller
                        c_next_state = SD_ACTIVE_FOR_WRITE_0;   // Next state: activate SDRAM for writing
                    }
                    else
                    {
            
                        c_read_en = 1;                          // Enable read for the controller
                        c_next_state = SD_ACTIVE_FOR_READ_0;    // Next state: activate SDRAM for reading
                    }
                }
            }
        break;

        // Activate the SDRAM
        case SD_ACTIVE_FOR_READ_0:

            sdram_cp_sel_n  = 0;
            c_next_state    = SD_ACTIVE_FOR_READ_1;

        break;

        
        case SD_ACTIVE_FOR_WRITE_0:

            sdram_cp_sel_n  = 0;
            c_next_state    = SD_ACTIVE_FOR_WRITE_1;
            
        break;

        // Activate the SDRAM and selcet row and bank 
        case SD_ACTIVE_FOR_READ_1:

            sdram_cp_sel_n      = 0;
            sdram_row_ac_sel_n  = 0;
            sdram_ba            = c_ba_buf.read();
            sdram_addr          = c_row_buf.read(); 
            c_next_state        = SD_NOP_AFTER_AKTIVE_READ_1;

        break;

        // Activate the SDRAM and selcet row and bank
        case SD_ACTIVE_FOR_WRITE_1:

            sdram_cp_sel_n      = 0;
            sdram_row_ac_sel_n  = 0;
            sdram_ba            = c_ba_buf.read();
            sdram_addr          = c_row_buf.read(); 
            c_next_state        = SD_NOP_AFTER_AKTIVE_WRITE_1;

        break;

        //NOP
        case SD_NOP_AFTER_AKTIVE_READ_1:

            sdram_cp_sel_n  = 0;
            c_next_state    = SD_READ;

        break;

        //NOP
        case SD_NOP_AFTER_AKTIVE_WRITE_1:

            sdram_cp_sel_n  = 0;
            c_next_state    = SD_WRITE;

        break;

        // Read 
        case SD_READ:

            sdram_cp_sel_n      = 0;
            sdram_col_ac_sel_n  = 0;
            sdram_ba            = c_ba_buf.read();
            sdram_addr          = c_col_buf.read(); 
            c_next_state        = SD_NOP_AFTER_READ_1;

        break;

        // Write
        case SD_WRITE:

            sdram_cp_sel_n      = 0;
            sdram_col_ac_sel_n  = 0;
            sdram_wr_en_n       = 0;
            sdram_ba            = c_ba_buf.read();
            sdram_addr          = c_col_buf.read();
            c_we_o              = 1;
            sdram_dq_o          = c_dq_buf_1.read();
            c_next_state        = SD_NOP_AFTER_WRITE_1;
            
        break;

        // NOP
        case SD_NOP_AFTER_READ_1:

            sdram_cp_sel_n  = 0;
            c_next_state    = SD_NOP_AFTER_READ_2;
        break;

        // NOP
        case SD_NOP_AFTER_READ_2:

            sdram_cp_sel_n  = 0;
            c_rd_buf_1      = 1;
            c_next_state    = SD_NOP_AFTER_READ_3;

        break;

        // NOP
        case SD_NOP_AFTER_READ_3:

            sdram_cp_sel_n  = 0;
            c_rd_buf_2      = 1;
            c_next_state    = SD_NOP_AFTER_READ_4;

        break;

        // NOP
        case SD_NOP_AFTER_READ_4:

            sdram_cp_sel_n  = 0;
            c_merg_dat      = 1;
            c_next_state    = SD_PRECHARGE;

        break;

        // NOP with the saconed write 
        case SD_NOP_AFTER_WRITE_1:

            sdram_cp_sel_n  = 0;
            sdram_dq_o      = c_dq_buf_2.read();
            c_we_o          = 1;
            sdram_dq_o      = c_dq_buf_2.read();
            c_next_state    = SD_NOP_AFTER_WRITE_2;
                
        break;

        // NOP
        case SD_NOP_AFTER_WRITE_2:

            sdram_cp_sel_n  = 0;
            c_next_state    = SD_PRECHARGE;

        break;

        // Pecharge -> deaktivate sdram and send acknowladge signal and data until the stobe and cycle signal end. 
        case SD_PRECHARGE:
            
            wb_slave.dat_o      = sc_uint<32>((c_dq_buf_2.read() << 16 ) | c_dq_buf_1.read());
            sdram_cp_sel_n      = 0;
            sdram_row_ac_sel_n  = 0;
            sdram_wr_en_n       = 0;
            wb_slave.ack_o      = 1;
            sdram_addr          = SDRAM_CONTROLLER_PRECHARGE;

            if(wb_slave.stb_i.read() == 0 && wb_slave.cyc_i.read() == 0)
            {

            c_next_state = SD_NOP_AFTER_PRECHARGE_1;

            }

        break;

        // NOP Precharge
        case SD_NOP_AFTER_PRECHARGE_1:

            c_next_state = SD_NOP_AFTER_PRECHARGE_2;

        break;

        // NOP Precharge
        case SD_NOP_AFTER_PRECHARGE_2:

            c_next_state = WB_IDLE;

        break;

        // 3 NOPs befor refresh 
        case SD_NOP_FOR_REFRESH:

            c_react_cnt_nop         = 1;
            c_cnt_nop_befor_rfrsh   = 1;
            sdram_cp_sel_n          = 0;

            if(c_cnt_react.read() == 1)
            {
                c_next_state = SD_REFRESH;
            }

        break;

        // Refresh 
        case SD_REFRESH:

            sdram_cp_sel_n      = 0;
            sdram_row_ac_sel_n  = 0;
            sdram_col_ac_sel_n  = 0;
            c_next_state        = SD_NOP_AFTER_REFRESH;

        break;

        // Nops after refresh
        case SD_NOP_AFTER_REFRESH:

            c_react_cnt_nop         = 1;
            c_cnt_nop_after_rfrsh   = 1;
            sdram_cp_sel_n          = 0;

            if(c_cnt_react.read() == 1)
            {
                c_next_state = WB_IDLE;
                c_rfrsh_end = 1;
            }

        break;
    }
}

   


void m_sdram_controller::proc_buffer_write()
    {
        

    
        while(true)
        {

            if(reset.read() == 1)
            {
                c_row_buf   = 0;
                c_col_buf   = 0;
                c_ba_buf    = 0;
                c_dq_buf_1  = 0;
                c_dq_buf_2  = 0;
                c_dat_merg  = 0;
                c_dqm_buf   = 0;
            }else{
                // converted the incoming data into the correct format
                if(c_write_en.read() == 1)
                {
                    c_row_buf   = wb_slave.adr_i.read().range(24,13);
                    c_ba_buf    = wb_slave.adr_i.read().range(12,11);
                    c_col_buf   = wb_slave.adr_i.read().range(10,2);
                    c_dq_buf_1  = wb_slave.dat_i.read().range(15,0);
                    c_dq_buf_2  = wb_slave.dat_i.read().range(31,16);
                
                }else{
                    if(c_rd_buf_1.read() == 1)
                    {
                        c_dq_buf_1 = sdram_dq_i.read();
                    }
                    if(c_rd_buf_2.read() == 1)
                    {
                        c_dq_buf_2 = sdram_dq_i.read();
                    }
                }

                if(c_read_en.read() == 1)
                {
                    c_row_buf   = wb_slave.adr_i.read().range(24,13);
                    c_ba_buf    = wb_slave.adr_i.read().range(12,11);
                    c_col_buf   = wb_slave.adr_i.read().range(10,2);
                }

            }

            wait();
        }
    }


// state transition process
void m_sdram_controller::proc_clk_state()
{
    c_current_state = SD_INIT_WAIT;

    while(true)
    {
        wait();

        c_current_state = c_next_state;
    }
}

//Enables SDRAM and inverts and synchronizes the system clock
void m_sdram_controller::proc_comb_clk_to_sdram()
{
        sdram_clk_en    = 1;
        sdram_clk       = !clk.read();
}

void m_sdram_controller::proc_c_cntr_refrsh()
{

    c_cntr_refrsh   = 170;
    c_rfrsh_strt    = 0;

 while(true)
 {
    wait();
    // IF c_cntr_refrsh is -1, then it refreshes 
    if (c_cntr_refrsh.read()[10] != 1  )
    {
        c_cntr_refrsh = c_cntr_refrsh.read() - 1;
    }else 
    {
        c_rfrsh_strt = 1;
        c_cntr_refrsh = 170;
        

    }
    if (c_rfrsh_end == 1)
    {
        c_cntr_refrsh = 2;
        c_rfrsh_strt = 0;
    }
 }
}

// waiting counter
void m_sdram_controller::proc_c_cntr_nops()
{

    c_cntr_nops = 0;
   
    while(true)
    {
        if (c_react_cnt_nop.read() ==1)
        {
            c_cntr_nops = c_cntr_nops.read() + 1;
        }else
        {
            c_cnt_react = 0;
            c_cntr_nops = 0;
        }
        // waiting for start
        if (c_cnt_nop_init.read() == 1 )
        {
            if (c_cntr_nops.read() == 3)
            { 
                c_cnt_react = 1;
            }
        }  
        // waiting for start refresh
        if(c_cnt_nop_befor_rfrsh.read() == 1)
        {
            if (c_cntr_nops.read() == 3 )
            {
                c_cnt_react = 1;
            }
        }
        // wating for end refresh
        if (c_cnt_nop_after_rfrsh.read() == 1)
        {
            if(c_cntr_nops.read() == 2)
            {
                c_cnt_react = 1;
            }
        }

        wait();

    }

}
