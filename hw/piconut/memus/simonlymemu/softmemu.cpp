/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_memory Unit (MemU) for simulation ONLY

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

#include "softmemu.h"

void m_soft_memu::Trace(sc_trace_file *tf, int level)
{

   // calling trace of submodules
   if (level >= 1)
   {
      PN_TRACE(tf, clk);
      PN_TRACE(tf, reset);

      PN_TRACE(tf, stb_iport);
      PN_TRACE(tf, adr_iport);
      PN_TRACE(tf, bsel_iport);
      PN_TRACE(tf, ack_iport);
      PN_TRACE(tf, rdata_iport);

      PN_TRACE(tf, stb_dport);
      PN_TRACE(tf, we_dport);
      PN_TRACE(tf, adr_dport);
      PN_TRACE(tf, bsel_dport);
      PN_TRACE(tf, wdata_dport);
      PN_TRACE(tf, ack_dport);
      PN_TRACE(tf, rdata_dport);

      PN_TRACE(tf, state_iport);
      PN_TRACE(tf, state_dport);
      PN_TRACE(tf, state_iport_next);
      PN_TRACE(tf, state_dport_next);
   }
   // Internal traces
}

void m_soft_memu::proc_clk()
{
   state_iport = MEMU_IDLE;
   state_dport = MEMU_IDLE;
   ack_i_flag = false;
   ack_d_flag = false;
   while (true)
   {
      wait();
      state_iport = state_iport_next;
      state_dport = state_dport_next;

      // Writeback
      ack_iport.write(ack_i_flag);
      ack_dport.write(ack_d_flag);

      rdata_iport.write(rdata_i_reg.read());
      rdata_dport.write(rdata_d_reg.read());

      on_rising_edge_clock_all_peripherals();
   }
}

void m_soft_memu::proc_cmb_iport()
{

   switch (state_iport)
   {
   case MEMU_IDLE:
      if (stb_iport.read())
      {
         state_iport_next = MEMU_READ;      }
      ack_i_flag = false;
      break;
   case MEMU_READ:
      // uncomment this if we want to handle 32-bit reads separately
      // if (bsel_iport.read() == 0xF)
      // { // 32-bit read
      //    rdata_i_reg = read32_peripheral(adr_iport.read());
      //    ack_i_flag = true;
      //    state_iport_next = MEMU_IDLE;
      //    break;
      // }
      // else
      // { // automatically handles all other reads
      rdata_i_reg = read_peripheral(adr_iport.read(), bsel_iport.read());
      ack_i_flag = true;
      state_iport_next = MEMU_IDLE;
      break;
      // }
      // break;
   }
}

void m_soft_memu::proc_cmb_dport()
{
   switch (state_dport)
   {
   case MEMU_IDLE:
      if (stb_dport.read())
      {
         adr_d_reg =adr_dport.read();
         if (we_dport.read())
         {
            state_dport_next = MEMU_WRITE;
         }
         else
         {
            state_dport_next = MEMU_READ;
         }
      }
      ack_d_flag = false;
      break;
   case MEMU_READ:
      // uncomment this if we want to handle 32-bit reads separately
      // if (bsel_dport.read() == 0xF)
      // { // 32-bit read
      //    rdata_d_reg = read32_peripheral(adr_dport.read());
      //    ack_d_flag = true;
      //    state_dport_next = MEMU_IDLE;
      // }
      // else
      // { // handle all other reads
      rdata_d_reg = read_peripheral(adr_d_reg.read(), bsel_dport.read());
      ack_d_flag = true;
      state_dport_next = MEMU_IDLE;
      // break;
      // }

      break;
   case MEMU_WRITE:
      if (bsel_dport.read() == 0xF)
      { // 32-bit write
         write32_peripheral(adr_d_reg.read(), wdata_dport.read());
         ack_d_flag = true;
         state_dport_next = MEMU_IDLE;
         break;
      }
      // else if (bsel_dport.read() == 0xC || bsel_dport.read() == 0x3) {
      //    write16_peripheral(adr_d_reg.read(), wdata_dport.read(), bsel_dport.read());
      //    ack_d_flag = true;
      //    state_dport_next = MEMU_IDLE;
      //    break;
      // }
      else
      { // handles all other writes
         write_peripheral(adr_d_reg.read(), wdata_dport.read(), bsel_dport.read());
         ack_d_flag = true;
         state_dport_next = MEMU_IDLE;
         break;
      }
   }
}

void m_soft_memu::proc_clk_core_trace()
{
   if (pn_cfg_trace_core)
   {
      // Open Output file for core dump
      FILE *coreDumpFile = fopen(NUCLEUS_TRACE_FILE, "w");
      if (coreDumpFile)
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
      while (true)
      {
         wait();
         switch (state_dport)
         {
         case MEMU_READ:
            address = adr_dport.read().to_uint();
            data = rdata_d_reg.read().to_uint();
            bsel = bsel_dport.read().to_uint();
            PN_INFO_FILE("\tDPort read request:\n", coreDumpFile);
            fprintf(coreDumpFile, "Read request for address 0x%08x with data 0x%08x and bsel 0x%02x\n", address, data, bsel);
            break;
         case MEMU_WRITE:
            address = adr_dport.read().to_uint();
            data = wdata_dport.read().to_uint();
            bsel = bsel_dport.read().to_uint();
            PN_INFO_FILE("\tDPort write request:\n", coreDumpFile);
            fprintf(coreDumpFile, "Write request for address 0x%08x with data 0x%08x and bsel 0x%02x\n", address, data, bsel);
            break;
         }
         switch (state_iport)
         {
         case MEMU_READ:
            address = adr_iport.read().to_uint();
            data = rdata_i_reg.read().to_uint();
            bsel = bsel_iport.read().to_uint();
            PN_INFO_FILE("\tIPort read request:\n", coreDumpFile);
            fprintf(coreDumpFile, "Read request for address 0x%08x with data 0x%08x and bsel 0x%02x\n", address, data, bsel);
            break;
         }
      }
      fclose(coreDumpFile);
   }
}

// Helper Methods...

void m_soft_memu::init_memory(uint64_t size, uint64_t base_adr)
{
#ifdef debug
   cout << "Attempting to initialize memory with size: " << size << " and base address: 0x" << std::hex << base_adr << endl;
#endif
   std::unique_ptr<c_soft_memory> memory = std::make_unique<c_soft_memory>(size, base_adr);
   if (memory)
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

void m_soft_memu::load_elf(const std::string &elfFileName)
{
#ifdef debug
   std::cout << "Loading ELF file: " << elfFileName << std::endl;
#endif

    if (elfFileName.empty())
    {
        std::cerr << "No ELF file specified" << std::endl;
        return;
    }

   c_elf_reader elfReader(elfFileName);
   if (elfReader.size == 0 || elfReader.start_adr == UINT32_MAX)
   {
      std::cerr << "ELF file size or start address is zero or invalid - check ELF file and reader." << std::endl;
      return;
   }

   // Initialize the memory for the ELF file and some additional space if needed
   init_memory(elfReader.size, elfReader.start_adr);

   // Check if there is a memory peripheral at the start address
   c_soft_peripheral* peripheral = peripheral_interface.find_peripheral(elfReader.start_adr);
   if (peripheral)
   {
      // Ensure the peripheral is a c_soft_memory object
      c_soft_memory *mem = dynamic_cast<c_soft_memory *>(peripheral);
      if (mem)
      {

         elfReader.initialize_from_file();
         // Copy from char* to memory vector in c_soft_memory
         if(mem->copy_into_memory(elfReader.get_memory(), elfReader.size))
         {
            std::cout << "ELF data successfully loaded into memory." << std::endl;
         }
         else
         {
            std::cerr << "Failed to load ELF data into internal memory." << std::endl;
            return;
         }

#ifdef debug
         std::cout << "ELF data successfully loaded into memory." << std::endl;
#endif
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


void m_soft_memu::add_peripheral(uint64_t base_adr, std::unique_ptr<c_soft_peripheral> peripheral)
{
   peripheral_interface.add_peripheral(base_adr, std::move(peripheral));
}

c_soft_peripheral *m_soft_memu::find_peripheral(uint64_t address)
{
   return peripheral_interface.find_peripheral(address);
}

void m_soft_memu::write_peripheral(uint64_t address, uint32_t data, uint8_t bsel)
{
   peripheral_interface.write_peripheral(address, data, bsel);
}

void m_soft_memu::write32_peripheral(uint64_t address, uint32_t data)
{
   peripheral_interface.write32_peripheral(address, data);
}

uint32_t m_soft_memu::read_peripheral(uint64_t address, uint8_t bsel)
{
   return peripheral_interface.read_peripheral(address, bsel);
}

void m_soft_memu::list_all_peripherals()
{
   peripheral_interface.list_all_peripherals();
}

int m_soft_memu::get_num_peripherals (){
   return peripheral_interface.get_num_peripherals();
}

c_soft_peripheral* m_soft_memu::get_peripheral (int index){
   return peripheral_interface.get_peripheral(index);
}

void m_soft_memu::on_rising_edge_clock_all_peripherals()
{
   peripheral_interface.on_rising_edge_clock_all_peripherals();
}
