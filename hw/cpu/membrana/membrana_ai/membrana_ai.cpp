#include "membrana_ai.h"

#include <pn_elf_reader.h>
#include <pn_uart.h>

#include <stdio.h> // For snprintf
#include <fstream>
#include <iomanip>
#include <iostream>

void dump_memory(uint8_t* memory, uint32_t size, uint32_t base_address, const std::string& filename)
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        std::cerr << "Failed to open file for memory dump." << std::endl;
        return;
    }

    file << std::hex << std::setfill('0');
    // Print a header for the columns for better readability
    file << "Address    "
         << "Data (Hexadecimal)   "
         << "ASCII" << std::endl;

    for(size_t i = 0; i < size; i += 16)
    {
        // Initialize a flag to check if the entire line is zero
        bool has_non_zero = false;

        // Buffer for ASCII representation
        char ascii[17];
        ascii[16] = '\0'; // Ensure null-terminated string for ASCII output

        // Check each byte in this line to not print empty lines (because of performance)
        for(int j = 0; j < 16; j++)
        {
            if(i + j < size && memory[i + j] != 0)
            {
                has_non_zero = true; // Set flag if any byte is non-zero
                break;               // No need to continue checking once a non-zero is found
            }
        }

        // Only print this line if a non-zero byte was found
        if(has_non_zero)
        {
            // Print the starting address of the line
            file << "0x" << std::setw(8) << (base_address + i) << ": ";

            // Process each byte in the block for printing
            for(int j = 0; j < 16; j++)
            {
                if(i + j < size)
                {
                    uint8_t byte = memory[i + j];
                    file << std::setw(2) << static_cast<unsigned>(byte) << " ";

                    // Convert byte to ASCII if printable, use '.' otherwise
                    ascii[j] = (byte >= 32 && byte <= 126) ? byte : '.';
                }
                else
                {
                    file << "   ";  // Align non-existent bytes
                    ascii[j] = ' '; // Fill ASCII buffer with spaces for alignment
                }
            }

            // Print ASCII representation at the end of each line
            file << " |" << ascii << "|" << std::endl;
        }
    }

    file.close();
}


// ***************** Init / Done / Tracing *************************************


void m_membrana_ai::init (int _iport_alen, int _iport_dlen, int _dport_alen, int _dport_dlen, int _vport_dlen) {

  // Sanity ...
  PN_ASSERT (_iport_alen == 32 && _iport_dlen == 32 && _dport_alen == 32);

  // Store configuration ...
  iport_alen = _iport_alen;
  iport_dlen = _iport_dlen;
  dport_alen = _dport_alen;
  dport_dlen = _dport_dlen;
  vport_dlen = _vport_dlen;

  // Create memory ...
  emem = PN_MALLOC (uint8_t, PN_CFG_MEMBRANA_EMEM_SIZE);
  emem_base = PN_CFG_MEMBRANA_EMEM_BASE;
  emem_end = PN_CFG_MEMBRANA_EMEM_BASE + PN_CFG_MEMBRANA_EMEM_SIZE;

  // Create (soft) UART ...
  PN_SETO(uart_regs, new pn_uart_regs_t);
  pn_uart_init (uart_regs);
}


m_membrana_ai::~m_membrana_ai () {

  if (pn_cfg_itrace_level >= 1)
    PN_INFOF(("DPORT count: %llu", dport_count));
  //~ dump_memory(emem, PN_CFG_MEMBRANA_EMEM_SIZE, emem_base, "memory.dump");

  PN_FREEP (emem);
  PN_FREEO (uart_regs);
}


void m_membrana_ai::pn_trace (sc_trace_file * tf, int level) {

  // Ports ...
  PN_TRACE(tf, clk);
  PN_TRACE(tf, reset);

  PN_TRACE_BUS(tf, iport_adr, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, iport_bsel, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, iport_rdata, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, iport_stb, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, iport_ack, PN_CFG_CPU_CORES);

  PN_TRACE_BUS(tf, dport_adr, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, dport_bsel, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, dport_rdata, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, dport_wdata, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, dport_stb, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, dport_we, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, dport_ack, PN_CFG_CPU_CORES);
  //~ PN_TRACE_BUS(tf, dport_lrsc, PN_CFG_CPU_CORES);
  //~ PN_TRACE_BUS(tf, dport_amo, PN_CFG_CPU_CORES);

  PN_TRACE_BUS(tf, vport_adr, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, vport_bsel, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, vport_rdata, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, vport_wdata, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, vport_stb, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, vport_we, PN_CFG_CPU_CORES);
  PN_TRACE_BUS(tf, vport_ack, PN_CFG_CPU_CORES);
  //~ PN_TRACE_BUS(tf, vport_lrsc, PN_CFG_CPU_CORES);
  //~ PN_TRACE_BUS(tf, vport_amo, PN_CFG_CPU_CORES);

  // Internal signals ...
  //   (none yet)
}





// ***************** Initialize memory from ELF file ***************************


void m_membrana_ai::load_elf (const char *filename) {
  uint8_t *elf_content;

  pn_c_elf_reader elf_reader;
  if (!elf_reader.read_file (filename))
    PN_ERRORF(("Failed to read ELF file: %s", filename));

  elf_content = elf_reader.get_memory ();
  if (!elf_content)
    PN_WARNINGF(("No defined sections content found in ELF file: %s", filename));
  else {
    if (elf_reader.get_base_addr () < emem_base || elf_reader.get_end_addr () > emem_end)
      PN_ERRORF(("ELF file contents (0x%x-0x%x) do not fit into defined memory (0x%x-0x%x).", elf_reader.get_base_addr (), elf_reader.get_end_addr (), emem_base, emem_end));
    memcpy (emem + elf_reader.get_base_addr () - emem_base, elf_content, elf_reader.get_size ());
  }
}





// ***************** Access helpers ********************************************


uint32_t m_membrana_ai::read32 (uint32_t adr, int bsel) {

  // UART ...
  if (pn_uart_is_addressed ((int) adr - PN_CFG_UART_BASE))
    return pn_uart_read32 (uart_regs, (int) adr - PN_CFG_UART_BASE);

  // Embedded memory ...
  if (adr >= emem_base && adr <= emem_end-4)
    return * (uint32_t *) (emem + adr - emem_base);

  // Default: Return 0xff...ff
  PN_WARNINGF(("Accessing undefined memory: r(0x%08x)", adr));
  return (uint32_t) -1;
}


void m_membrana_ai::write32 (uint32_t adr, int bsel, uint32_t val) {
  int i;

  // UART ...
  if (pn_uart_is_addressed ((int) adr - PN_CFG_UART_BASE))
    pn_uart_write32 (uart_regs, (int) adr - PN_CFG_UART_BASE, val, bsel);

  // Embedded memory ...
  else if (adr >= emem_base && adr <= emem_end - 4) {
    if (bsel == 0x0f) * (uint32_t *) (emem + adr - emem_base) = val;
    else {
      for (i = 0; i < 4; i++) if (bsel & (1 << i))
        emem[adr - emem_base + i] = (val >> (8 * i)) & 0xff;
    }
  }

  // Default: Ignore ...
  else
    //~ PN_WARNINGF(("Accessing undefined memory: w(0x%08x)", adr));
    PN_ERRORF(("Accessing undefined memory: w(0x%08x)", adr));
}





// ***************** Main process **********************************************


void m_membrana_ai::proc_clk_main_soft () {
  uint32_t adr;
  int bsel;
  pn_dport_dat_t data;
  int i;

  while (1) {
    wait ();

    // Set output defaults ...
    iport_ack[0].write (0);
    dport_ack[0].write (0);
    vport_ack[0].write (0);

    // IPort ...
    if (iport_stb[0].read ()) {
      iport_rdata[0].write (read32 (iport_adr[0].read (), iport_bsel[0].read ()));
      iport_ack[0].write (1);
    }

    // DPort ...
    if (dport_stb[0].read ()) {
      bsel = dport_bsel[0].read ().to_uint ();
      adr = dport_adr[0].read ().to_uint ();
      if (dport_we[0].read ()) {

        // Write ...
        data = dport_wdata[0].read ();
        if (pn_cfg_itrace_level >= 1)
          dport_count++;
        if (pn_cfg_itrace_level >= 2)
          PN_INFOF(("w(0x%08x/%x) = 0x%08x", adr, bsel, (uint32_t) data.to_uint ()));
        for (i = 0; i < dport_dlen; i += 32) {
          write32 (adr, bsel & 0xf, data.to_uint ());
          adr += 4;
          bsel >>= 4;
          data >>= 32;
        }
      }
      else {

        // Read ...
        if (pn_cfg_itrace_level >= 1)
          dport_count++;
        if (pn_cfg_itrace_level >= 2)
          PN_INFOF(("r(0x%08x/%x) = 0x%08x", adr, bsel, read32 (adr, 0xf)));
        data = 0;
        for (i = 0; i < dport_dlen; i += 32) {
          data = (data << 32) | read32 (adr, bsel & 0xf);
          adr += 4;
          bsel >>= 4;
        }
        //~ PN_INFOF(("... = %08x", (uint32_t) data));
        dport_rdata[0].write (data);
      }
      dport_ack[0].write (1);
    }

    // VPort ...
    if (vport_stb[0].read ()) {
      bsel = vport_bsel[0].read ().to_uint ();
      adr = vport_adr[0].read ().to_uint ();
      if (vport_we[0].read ()) {

        // Write ...
        data = vport_wdata[0].read ();
        if (pn_cfg_itrace_level >= 1)
          PN_INFOF(("wv(0x%08x/%x) = 0x%08x", adr, bsel, (uint32_t) data.to_uint ()));
        for (i = 0; i < vport_dlen; i += 32) {
          write32 (adr, bsel & 0xf, data.to_uint ());
          adr += 4;
          bsel >>= 4;
          data >>= 32;
        }
      }
      else {

        // Read ...
        if (pn_cfg_itrace_level >= 1)
          PN_INFOF(("rv(0x%08x/%x) = 0x%08x", adr, bsel, read32 (adr, 0xf)));
        data = 0;
        for (i = 0; i < vport_dlen; i += 32) {
          data = (data << 32) | read32 (adr, bsel & 0xf);
          adr += 4;
          bsel >>= 4;
        }
        //~ PN_INFOF(("... = %08x", (uint32_t) data));
        vport_rdata[0].write (data);
      }
      vport_ack[0].write (1);
    }
  }
}

