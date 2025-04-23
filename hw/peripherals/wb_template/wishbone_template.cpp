/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
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




#include "wishbone_template.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_wishbone_t::Trace(sc_trace_file *tf, int level)
{

    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, wb_ack_o);
    PN_TRACE(tf, wb_adr_i);
    PN_TRACE(tf, wb_ack_o);
    PN_TRACE(tf, wb_current_state);
    PN_TRACE(tf, wb_cyc_i);
    PN_TRACE(tf, wb_dat_i);
    PN_TRACE(tf, wb_dat_o);
    PN_TRACE(tf, wb_sel_i);
    PN_TRACE(tf, wb_we_i);
    PN_TRACE(tf, wb_err_o);

    PN_TRACE(tf, dummy_register_1);
    PN_TRACE(tf, dummy_register_2);
    PN_TRACE(tf, dummy_register_3);



}


void m_wishbone_t::proc_comb_wb_slave() //Wishbone Statetransition logic (combinatory)
{

    // // set idle Outputs to default
    wb_ack_o = 0;
    wb_dat_o = 0;
    wb_err_o = 0;
    wb_rty_o = 0;

    dummy_register_1 = dummy_register_1.read();
    dummy_register_2 = dummy_register_2.read();
    dummy_register_3 = dummy_register_3.read();




    wb_next_state = wb_current_state.read(); //default next state is current state

    //local function variables
    bool wb_error_occured = false; //Flag for error

    switch(wb_current_state.read())
    {
        case WB_IDLE: //Wishbone Idle State to check if we have a valid transaction

            if(wb_stb_i.read() == 1 && wb_cyc_i.read()==1)      //WB Strobe and cycle valid,

                //check if slave is being addressed in its addressrange, you can also have one address only then you just compare the wb_adr_i to your own CFG_WB_SLAVE_ADDRESS

                std::cout << "wishbone_template.cpp"  << " " << std::hex << wb_adr_i.read() << " "  << std::hex << CFG_WB_SLAVE_ADDRESS << " " << std::hex << CFG_WB_SLAVE_ADDRESS + 0xC  << std::endl;

                if((CFG_WB_SLAVE_ADDRESS<=wb_adr_i.read()) && (wb_adr_i.read() < (CFG_WB_SLAVE_ADDRESS + 0xC)))
                {
                    if(wb_we_i.read() == 1)
                    {
                        wb_next_state=WB_WRITE1; //Master writes to us (Slave)
                    }
                    else
                    {
                        wb_next_state=WB_READ; //Master reads from us (Slave)
                    }
                }

            break;

        case WB_WRITE1: //Master writes to us -> read from input, write to our register

            switch(wb_adr_i.read() - CFG_WB_SLAVE_ADDRESS)  // use Address offset to determine target register
            {
                case ADR_REG_1:
                    // write seomthing in the ADR_REG_1 addressrange
                    // std::cout << "wishbone_template.cpp"  << " " << apply_byte_select(wb_dat_i.read(), (sc_uint<32>) dummy_register_1, wb_sel_i.read()) << std::endl;
                    dummy_register_1 = apply_byte_select((sc_uint<32>) dummy_register_1, wb_dat_i.read(), wb_sel_i.read());
                    break;
                case ADR_REG_2:
                    // write something in the ADR_REG_2 addressrange
                    dummy_register_2 = apply_byte_select((sc_uint<32>) dummy_register_2, wb_dat_i.read(), wb_sel_i.read());
                    break;
                case ADR_REG_3:
                    // do nothing in the ADR_REG_3 addressrange because i this example it's a read only address
                    dummy_register_3 = apply_byte_select((sc_uint<32>) dummy_register_3, wb_dat_i.read(), wb_sel_i.read());
                    break;
                default:
                    // should never reach this, if it does then throw wishbone error
                    wb_err_o = 1;
                    wb_next_state=WB_IDLE;
                    return;
                    break;
            }

            // set the next wishbone state because writing is not finished yet
            wb_next_state=WB_WRITE2;

            break;

        case WB_WRITE2: //Write Acknowledge

            wb_ack_o = 1;

            // after setting the wishbone ack the writing is finished and we can go back to the idle state
            wb_next_state=WB_IDLE;

            break;

        case WB_READ: //Master Reads from us -> Write data to the bus output

            switch(wb_adr_i.read() - CFG_WB_SLAVE_ADDRESS)  //use Address offset to determine target register
            {

                // check which address is being addressed and write the address back as data value
                case e_wb_addressrange::ADR_REG_1:

                    // read the data from the wishbone bus addressed by the offset to the first dummy register with the byte select functionality.

                    wb_dat_o.write(read_with_byte_select((sc_uint<32>) dummy_register_1, wb_sel_i.read()));

                    break;
                case e_wb_addressrange::ADR_REG_2:


                    wb_dat_o.write(read_with_byte_select((sc_uint<32>) dummy_register_2, wb_sel_i.read()));

                    break;
                case e_wb_addressrange::ADR_REG_3:

                    wb_dat_o.write(read_with_byte_select((sc_uint<32>) dummy_register_3, wb_sel_i.read()));

                    break;
                default:
                    wb_error_occured=1;
                    break;
            }

            wb_ack_o = 1;
            wb_next_state=WB_IDLE;

            break;
    }


}


// state transition process
void m_wishbone_t::proc_clk_state() {

    wb_current_state = WB_IDLE;

    while (true)
    {
        wait();

        wb_current_state = wb_next_state;
    }
}


uint32_t m_wishbone_t::apply_byte_select(uint32_t current_data, uint32_t new_data, uint8_t byte_select) {
    uint32_t mask = 0;

    // buffer for returning
    uint32_t ret_buffer, full_buffer;
    bool full_selection = false;

    // Create mask based on byte select signals
    if (byte_select == 0xF) {  // Write full 32-bit word
        full_buffer = new_data;
        full_selection = true;
    }
    else if (byte_select == 0xC) {  // Write only upper 16 bits (bytes 2 and 3)
        mask = 0xFFFF0000;
    }
    else if (byte_select == 0x3) {  // Write only lower 16 bits (bytes 0 and 1)
        mask = 0x0000FFFF;
    }
    else {  // Selective byte masking
        if (byte_select & 0x1) mask |= 0x000000FF;  // Byte 0
        if (byte_select & 0x2) mask |= 0x0000FF00;  // Byte 1
        if (byte_select & 0x4) mask |= 0x00FF0000;  // Byte 2
        if (byte_select & 0x8) mask |= 0xFF000000;  // Byte 3
    }

    // Apply byte select: retain unselected bytes, update selected bytes
    if (full_selection) {
        ret_buffer = full_buffer;
    } else {
        ret_buffer = (current_data & ~mask) | (new_data & mask);
    }

    return ret_buffer;
}

uint32_t m_wishbone_t::read_with_byte_select(uint32_t data, uint8_t byte_select) {
    uint32_t extracted_data = 0;

    // buffer for returning
    uint32_t ret_buffer;
    bool simple_selection = false;

    if (byte_select == 0xF) {  // Full word (all 4 bytes)
        ret_buffer = data;
    }
    else if (byte_select == 0xC) {  // Upper 16 bits (bytes 2 and 3)
        ret_buffer = (data & 0xFFFF0000);
    }
    else if (byte_select == 0x3) {  // Lower 16 bits (bytes 0 and 1)
        ret_buffer = (data & 0x0000FFFF);
    }
    else {  // Selective byte masking (same as before)
        if (byte_select & 0x1) extracted_data |= (data & 0x000000FF);  // Byte 0
        if (byte_select & 0x2) extracted_data |= (data & 0x0000FF00);  // Byte 1
        if (byte_select & 0x4) extracted_data |= (data & 0x00FF0000);  // Byte 2
        if (byte_select & 0x8) extracted_data |= (data & 0xFF000000);  // Byte 3
        simple_selection = true;
    }

    // Apply byte select: retain unselected bytes, update selected bytes
    if (simple_selection) {
        ret_buffer = extracted_data;
    }

    return ret_buffer;
}