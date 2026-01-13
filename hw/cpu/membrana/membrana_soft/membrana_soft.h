/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
                2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains the implementation of the m_membrana_soft module.
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

#ifndef __MEMBRANA_SOFT_H__
#define __MEMBRANA_SOFT_H__

#define NUCLEUS_TRACE_FILE "core_trace.log"

#include <systemc.h>
#include <elf.h>
#include <map>

#include <piconut.h>

#include "c_soft_memory.h"               // contains the memory peripheral
#include "c_soft_peripheral_container.h" // contains the peripheral interface class

SC_MODULE(m_membrana_soft)
{
private:
    // Using a Map with adress as key to store peripherals as unique pointers
    c_soft_peripheral_container peripheral_interface; // instance of the peripheral interface

    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(reserved_addr, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<bool>> PN_NAME_VEC(reserved, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(amo_p_addr, PN_CFG_CPU_CORES); // Atomic-Memory-Operation pending address
    sc_vector<sc_signal<bool>> PN_NAME_VEC(amo_p, PN_CFG_CPU_CORES);             // Atomic-Memory-Operation pending

public:
    // Ports ...
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // Input Ports ...
    // Instruction Ports (IPort) ...
    sc_vector<sc_in<bool>> PN_NAME_VEC(stb_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_in<sc_uint<32>>> PN_NAME_VEC(adr_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_in<sc_uint<4>>> PN_NAME_VEC(bsel_iport, PN_CFG_CPU_CORES);

    sc_vector<sc_out<sc_uint<32>>> PN_NAME_VEC(rdata_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_out<bool>> PN_NAME_VEC(ack_iport, PN_CFG_CPU_CORES);

    // Data Ports (DPort) ...
    sc_vector<sc_in<bool>> PN_NAME_VEC(stb_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>> PN_NAME_VEC(we_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>> PN_NAME_VEC(load_reserve, PN_CFG_CPU_CORES);
    sc_vector<sc_in<bool>> PN_NAME_VEC(amo_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_in<sc_uint<32>>> PN_NAME_VEC(adr_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_in<sc_uint<32>>> PN_NAME_VEC(wdata_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_in<sc_uint<4>>> PN_NAME_VEC(bsel_dport, PN_CFG_CPU_CORES);

    sc_vector<sc_out<sc_uint<32>>> PN_NAME_VEC(rdata_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_out<bool>> PN_NAME_VEC(ack_dport, PN_CFG_CPU_CORES);

    // Constructor
    SC_CTOR(m_membrana_soft)
    {
        SC_CTHREAD(proc_clk, clk.pos());
        reset_signal_is(reset, true);

#if PN_CFG_CPU_CORES >= 1
        SC_METHOD(proc_cmb_iport_0);
        sensitive << reset << stb_iport[0].pos() << state_iport[0] << adr_iport[0] << bsel_iport[0];

        SC_METHOD(proc_cmb_dport_0);
        sensitive << reset << stb_dport[0].pos() << we_dport[0].pos() << adr_dport[0] << wdata_dport[0] << state_dport[0] << bsel_dport[0] << load_reserve[0] << amo_dport[0];
#endif
#if PN_CFG_CPU_CORES >= 2
        SC_METHOD(proc_cmb_iport_1);
        sensitive << reset << stb_iport[1].pos() << state_iport[1] << adr_iport[1] << bsel_iport[1];

        SC_METHOD(proc_cmb_dport_1);
        sensitive << reset << stb_dport[1].pos() << we_dport[1].pos() << adr_dport[1] << wdata_dport[1] << state_dport[1] << bsel_dport[1] << load_reserve[1] << amo_dport[1];
#endif

        // SC_METHOD(proc_cmb_dport);
        // sensitive << reset << stb_dport.pos() << we_dport.pos() << adr_dport << wdata_dport << state_dport << bsel_dport << load_reserve[0];

        SC_CTHREAD(proc_clk_core_trace, clk.pos());
    }

    /**
     * @brief transition process for statemachine and registers
     */
    void proc_clk();

    /**
     * @brief Wrapper for proc_cmb_iport_core
     */
    void proc_cmb_iport_0();

    /**
     * @brief Wrapper for proc_cmb_iport_core
     */
    void proc_cmb_iport_1();

    /**
     * @brief Statemachine for the Iport
     */
    void proc_cmb_iport_core(int core);

    /**
     * @brief Wrapper for proc_cmb_dport_core
     */
    void proc_cmb_dport_0();

    /**
     * @brief Wrapper for proc_cmb_dport_core
     */
    void proc_cmb_dport_1();

    /**
     * @brief Statemachine for the DPort
     *
     * @param core Core which ports will be used
     */
    void proc_cmb_dport_core(int core);

    /**
     * @brief Trace all core requests and print them to a file
     */
    void proc_clk_core_trace();

    // Helper-Functions
    void pn_trace(sc_trace_file * tf, int level = 1);

    /** @brief Initialize memory for peripherals
     *
     * @param size c_soft_memory size
     * @param base_adr Base address of the memory
     */
    void init_memory(uint64_t size, uint64_t base_adr);

    /** @brief Load an ELF file and generates memory
     * initializes memory and adds it to the peripherals map uising init_memory()
     * @param elfFile Path to the ELF file
     */
    void load_elf(const std::string& elfFile);

    /** @brief Add a peripheral to the system
     *
     * @param base_adr Base address of the peripheral
     * @param peripheral Pointer to the peripheral object
     */
    void add_peripheral(uint64_t base_adr, std::unique_ptr<c_soft_peripheral> peripheral);

    /** @brief Find a peripheral by its memory address
     *
     * @param address c_soft_memory address to search for
     * @return Pointer to the found peripheral, or nullptr if not found
     */
    c_soft_peripheral* find_peripheral(uint64_t address);

    /** @brief Write data to a peripheral using byte select
     *
     * @param address Address at which to write
     * @param data Data to write
     * @param bsel Byte select
     */
    void write_peripheral(uint64_t address, uint32_t data, uint8_t bsel);

    /** @brief Write 32-bit data to a peripheral
     *
     * @param address Address at which to write
     * @param data Data to write
     * @param bsel Byte select
     */
    void write32_peripheral(uint64_t address, uint32_t data);

    /** @brief Read data from a peripheral
     *
     * @param address Address from which to read
     * @return Data read from the peripheral
     */
    uint32_t read_peripheral(uint64_t address, uint8_t bsel);

    /** @brief List all peripherals and their information
     */
    void list_all_peripherals();

    /** @brief return the number of peripherals
     */
    int get_num_peripherals();

    /**
     * @brief Get the peripheral object by number
     *
     * @param index
     * @return const c_soft_peripheral*
     */
    c_soft_peripheral* get_peripheral(int index);

    /**
     * @brief Updates all peripheral devices on each system clock cycle
     *
     * This function serves as a central clock distribution mechanism for the system's peripheral
     * devices. It is called during each iteration of the main clock process (proc_clk()) and
     * delegates the call to the peripheral_interface component, which then iterates through
     * all registered peripherals.
     *
     * Each peripheral's on_rising_edge_clock() method is invoked, allowing time-sensitive
     * peripherals like timers, UARTs, and other components to update their internal state
     * in synchronization with the system clock.
     *
     * @see c_soft_peripheral_container::on_rising_edge_clock_all_peripherals()
     * @see c_soft_peripheral::on_rising_edge_clock()
     */
    void on_rising_edge_clock_all_peripherals();

protected:
    // Internal Signals
    enum e_membrana_state
    {
        MEMBRANA_IDLE,
        MEMBRANA_READ,
        MEMBRANA_WRITE
    };
    // state state_iport, state_dport, state_iport_next, state_dport_next;
    sc_vector<sc_signal<sc_uint<2>>> PN_NAME_VEC(state_iport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<2>>> PN_NAME_VEC(state_dport, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<2>>> PN_NAME_VEC(state_iport_next, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<2>>> PN_NAME_VEC(state_dport_next, PN_CFG_CPU_CORES);

    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(rdata_i_reg, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(rdata_d_reg, PN_CFG_CPU_CORES);
    sc_vector<sc_signal<sc_uint<64>>> PN_NAME_VEC(adr_d_reg, PN_CFG_CPU_CORES);

    bool ack_i_flag[PN_CFG_CPU_CORES];
    bool ack_d_flag[PN_CFG_CPU_CORES];

    // Methods

}; // end of class m_membrana_soft

#endif // __MEMBRANA_SOFT_H__
