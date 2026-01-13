/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
                2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the membrana_soft module.
  For simulation ONLY

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

#include "membrana_soft.h"

#include "elf_reader.h" // contains the elf reader class

void m_membrana_soft::pn_trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 1)
    {
        PN_TRACE(tf, clk);
        PN_TRACE(tf, reset);

        PN_TRACE(tf, stb_iport[0]);
        PN_TRACE(tf, adr_iport[0]);
        PN_TRACE(tf, bsel_iport[0]);
        PN_TRACE(tf, ack_iport[0]);
        PN_TRACE(tf, rdata_iport[0]);

        PN_TRACE(tf, stb_dport[0]);
        PN_TRACE(tf, we_dport[0]);
        PN_TRACE(tf, adr_dport[0]);
        PN_TRACE(tf, bsel_dport[0]);
        PN_TRACE(tf, wdata_dport[0]);
        PN_TRACE(tf, ack_dport[0]);
        PN_TRACE(tf, rdata_dport[0]);

        PN_TRACE(tf, load_reserve[0]);
        PN_TRACE(tf, reserved_addr[0]);
        PN_TRACE(tf, reserved[0]);
        PN_TRACE(tf, amo_dport[0]);
        PN_TRACE(tf, amo_p_addr[0]);
        PN_TRACE(tf, amo_p[0]);

        PN_TRACE(tf, state_iport[0]);
        PN_TRACE(tf, state_dport[0]);
        PN_TRACE(tf, state_iport_next[0]);
        PN_TRACE(tf, state_dport_next[0]);
    }
    // Internal traces
}

void m_membrana_soft::proc_clk()
{
    for(int i = 0; i < PN_CFG_CPU_CORES; i++)
    {
        state_iport[i] = MEMBRANA_IDLE;
        state_dport[i] = MEMBRANA_IDLE;
        ack_i_flag[i] = false;
        ack_d_flag[i] = false;
    }
    while(true)
    {
        wait();
        for(int i = 0; i < PN_CFG_CPU_CORES; i++)
        {
            state_iport[i] = state_iport_next[i];
            state_dport[i] = state_dport_next[i];

            // Writeback
            ack_iport[i].write(ack_i_flag[i]);
            ack_dport[i].write(ack_d_flag[i]);

            rdata_iport[i].write(rdata_i_reg[i].read());
            rdata_dport[i].write(rdata_d_reg[i].read());

            on_rising_edge_clock_all_peripherals();
        }
    }
}

void m_membrana_soft::proc_cmb_iport_0()
{
    proc_cmb_iport_core(0);
}

void m_membrana_soft::proc_cmb_iport_1()
{
    proc_cmb_iport_core(1);
}

void m_membrana_soft::proc_cmb_iport_core(int core)
{

    switch(state_iport[core].read())
    {
        case MEMBRANA_IDLE:
            if(stb_iport[core].read())
            {
                state_iport_next[core] = MEMBRANA_READ;
            }
            ack_i_flag[core] = false;
            break;
        case MEMBRANA_READ:
            // uncomment this if we want to handle 32-bit reads separately
            // if (bsel_iport.read() == 0xF)
            // { // 32-bit read
            //    rdata_i_reg = read32_peripheral(adr_iport.read());
            //    ack_i_flag = true;
            //    state_iport_next = MEMBRANA_IDLE;
            //    break;
            // }
            // else
            // { // automatically handles all other reads
            rdata_i_reg[core] = read_peripheral(adr_iport[core].read(), bsel_iport[core].read());
            ack_i_flag[core] = true;
            state_iport_next[core] = MEMBRANA_IDLE;
            break;
            // }
            // break;
    }
}

void m_membrana_soft::proc_cmb_dport_0()
{
    proc_cmb_dport_core(0);
}

void m_membrana_soft::proc_cmb_dport_1()
{
    proc_cmb_dport_core(1);
}

/**
 * @brief Process for the DPort core, handles AMO and load-reserve operations
 *        This is called by each core's specific dport
 */
void m_membrana_soft::proc_cmb_dport_core(int core)
{
    switch(state_dport[core].read())
    {
        case MEMBRANA_IDLE:
            if(stb_dport[core].read())
            {
                adr_d_reg[core] = adr_dport[core].read();
                if(we_dport[core].read())
                {
                    state_dport_next[core] = MEMBRANA_WRITE;
                }
                else
                {
                    state_dport_next[core] = MEMBRANA_READ;
                }
            }
            ack_d_flag[core] = false;
            break;
        case MEMBRANA_READ: {

#if PN_CFG_CPU_CORES > 1 // Multicore AMO handling
            // used to exit the switch from a loop inside a case
            bool blocked_by_other_amo = 0;
            // only allow reads when AMO is done (atomicy guarantee)
            for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
            {
                if(amo_p[i].read() == 1)
                {
                    if(amo_p_addr[i].read() == adr_d_reg[core].read())
                    {
                        blocked_by_other_amo = 1;
                        break;
                    }
                }
            }

            if(blocked_by_other_amo)
            {
                state_dport_next[core] = MEMBRANA_IDLE;
                ack_d_flag[core] = false;
                break;
            }

            if(amo_dport[core].read())
            {
                amo_p_addr[core] = adr_d_reg.read();
                amo_p[core] = 1;
            }

#endif // multi-amo

            // If reserve is marked, put address in reservation set.
            if(load_reserve[core].read())
            {
                reserved_addr[core] = adr_d_reg[core].read();
                reserved[core] = 1;
            }

            rdata_d_reg[core] = read_peripheral(adr_d_reg[core].read(), bsel_dport[core].read());
            ack_d_flag[core] = true;
            state_dport_next[core] = MEMBRANA_IDLE;

            break;
        }
        case MEMBRANA_WRITE: {

#if PN_CFG_CPU_CORES > 1 // Multicore AMO handling
            // used to exit the switch from a loop inside a case
            bool blocked_by_other_amo = 0;
            // only allow writes when AMO is done (atomicy guarantee)
            for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
            {
                if(amo_p[i].read() == 1)
                {
                    if(amo_p_addr[i].read() == adr_d_reg[core].read())
                    {
                        if(i != core)
                        {
                            blocked_by_other_amo = 1;
                            break;
                        }
                        else
                        {
                            amo_p[i] = 0;
                        }
                    }
                }
            }

            if(blocked_by_other_amo == 1)
            {
                state_dport_next[core] = MEMU_IDLE;
                ack_d_flag[core] = false;
                break;
            }
#endif // multi-amo

            // Invalidate all other reserved addresses on adr_d_reg
            for(size_t i = 0; i < PN_CFG_CPU_CORES; i++)
            {
                // A core can invalidate its own reservation in an interrupt handler
                if((!load_reserve[core] || i != core) && reserved_addr[i].read() == adr_d_reg[core].read())
                {
                    reserved[i] = 0;
                }
            }

            // fail if not reserved
            if(load_reserve[core].read() == 1)
            {
                if(reserved[core].read() == 0x0)
                {
                    // write status 1 through rdata
                    rdata_d_reg[core] = 1;
                    ack_d_flag[core] = true;
                    state_dport_next[core] = MEMBRANA_IDLE;
                    break;
                }
                // write status 0 through rdata (ack is set below)
                rdata_d_reg[core] = 0;
                reserved[core] = 0;
            }

            if(bsel_dport[core].read() == 0xF)
            { // 32-bit write
                write32_peripheral(adr_d_reg[core].read(), wdata_dport[core].read());
                ack_d_flag[core] = true;
                state_dport_next[core] = MEMBRANA_IDLE;
                break;
            }
            // else if (bsel_dport.read() == 0xC || bsel_dport.read() == 0x3) {
            //    write16_peripheral(adr_d_reg.read(), wdata_dport.read(), bsel_dport.read());
            //    ack_d_flag = true;
            //    state_dport_next = MEMBRANA_IDLE;
            //    break;
            // }
            else
            { // handles all other writes
                write_peripheral(adr_d_reg[core].read(), wdata_dport[core].read(), bsel_dport[core].read());
                ack_d_flag[core] = true;
                state_dport_next[core] = MEMBRANA_IDLE;
                break;
            }
            break;
        }
    }
}

void m_membrana_soft::proc_clk_core_trace()
{
    if(pn_cfg_trace_core)
    {
        // Open Output file for core dump
        FILE* coreDumpFile = fopen(NUCLEUS_TRACE_FILE, "w");
        if(coreDumpFile)
        {
            PN_INFO_FILE("This is the core request trace.", coreDumpFile);
        }
        else
        {
            std::cerr << "Failed to open core dump file." << std::endl;
        }

        u_int64_t address;
        u_int64_t data;
        u_int8_t bsel;
        while(true)
        {
            wait();
            switch(state_dport[0].read())
            {
                case MEMBRANA_READ:
                    address = adr_dport[0].read().to_uint();
                    data = rdata_d_reg[0].read().to_uint();
                    bsel = bsel_dport[0].read().to_uint();
                    PN_INFO_FILE("\tDPort read request:\n", coreDumpFile);
                    fprintf(coreDumpFile, "Read request for address 0x%08x with data 0x%08x and bsel 0x%02x\n", address, data, bsel);
                    break;
                case MEMBRANA_WRITE:
                    address = adr_dport[0].read().to_uint();
                    data = wdata_dport[0].read().to_uint();
                    bsel = bsel_dport[0].read().to_uint();
                    PN_INFO_FILE("\tDPort write request:\n", coreDumpFile);
                    fprintf(coreDumpFile, "Write request for address 0x%08x with data 0x%08x and bsel 0x%02x\n", address, data, bsel);
                    break;
            }
            switch(state_iport[0].read())
            {
                case MEMBRANA_READ:
                    address = adr_iport[0].read().to_uint();
                    data = rdata_i_reg[0].read().to_uint();
                    bsel = bsel_iport[0].read().to_uint();
                    PN_INFO_FILE("\tIPort read request:\n", coreDumpFile);
                    fprintf(coreDumpFile, "Read request for address 0x%08x with data 0x%08x and bsel 0x%02x\n", address, data, bsel);
                    break;
            }
        }
        fclose(coreDumpFile);
    }
}

// Helper Methods...

void m_membrana_soft::init_memory(uint64_t size, uint64_t base_adr)
{
#ifdef debug
    cout << "Attempting to initialize memory with size: " << size << " and base address: 0x" << std::hex << base_adr << endl;
#endif
    std::unique_ptr<c_soft_memory> memory = std::make_unique<c_soft_memory>(size, base_adr);
    if(memory)
    {
        add_peripheral(base_adr, std::move(memory));
#ifdef debug
        cout << "Memory successfully initialized and added to peripherals." << endl;
#endif
    }
    else
    {
        std::cerr << "Memory object could not be created." << std::endl;
    }
}

void m_membrana_soft::load_elf(const std::string& elfFileName)
{
#ifdef debug
    std::cout << "Loading ELF file: " << elfFileName << std::endl;
#endif

    if(elfFileName.empty())
    {
        std::cerr << "No ELF file specified" << std::endl;
        return;
    }

    c_elf_reader elfReader(elfFileName);
    if(elfReader.size == 0 || elfReader.start_adr == UINT32_MAX)
    {
        std::cerr << "ELF file size or start address is zero or invalid - check ELF file and reader." << std::endl;
        return;
    }

    // Initialize the memory for the ELF file and some additional space if needed
    init_memory(elfReader.size, elfReader.start_adr);

    // Check if there is a memory peripheral at the start address
    c_soft_peripheral* peripheral = peripheral_interface.find_peripheral(elfReader.start_adr);
    if(peripheral)
    {
        // Ensure the peripheral is a c_soft_memory object
        c_soft_memory* mem = dynamic_cast<c_soft_memory*>(peripheral);
        if(mem)
        {

            // Copy from char* to memory vector in c_soft_memory
            if(!mem->copy_into_memory(elfReader.get_memory(), elfReader.size))
            {
                std::cerr << "Failed to load ELF data into internal memory." << std::endl;
                return;
            }
        }
        else
        {
            std::cerr << "Found peripheral is not a c_soft_memory object." << std::endl;
        }
    }
    else
    {
        std::cerr << "No valid memory peripheral found at address 0x" << std::hex << elfReader.start_adr << std::endl;
    }
}

void m_membrana_soft::add_peripheral(uint64_t base_adr, std::unique_ptr<c_soft_peripheral> peripheral)
{
    peripheral_interface.add_peripheral(base_adr, std::move(peripheral));
}

c_soft_peripheral* m_membrana_soft::find_peripheral(uint64_t address)
{
    return peripheral_interface.find_peripheral(address);
}

void m_membrana_soft::write_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
    peripheral_interface.write_peripheral(address, data, bsel);
}

void m_membrana_soft::write32_peripheral(uint64_t address, uint32_t data)
{
    peripheral_interface.write32_peripheral(address, data);
}

uint32_t m_membrana_soft::read_peripheral(uint64_t address, uint8_t bsel)
{
    return peripheral_interface.read_peripheral(address, bsel);
}

void m_membrana_soft::list_all_peripherals()
{
    peripheral_interface.list_all_peripherals();
}

int m_membrana_soft::get_num_peripherals()
{
    return peripheral_interface.get_num_peripherals();
}

c_soft_peripheral* m_membrana_soft::get_peripheral(int index)
{
    return peripheral_interface.get_peripheral(index);
}

void m_membrana_soft::on_rising_edge_clock_all_peripherals()
{
    peripheral_interface.on_rising_edge_clock_all_peripherals();
}
