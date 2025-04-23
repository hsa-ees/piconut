#include <stdint.h>
#include <systemc.h>
#include "emumemu.h"
#include "elf_reader.h"

#define PERIOD_NS 10.0
#define MEM_BASE_ADR 0x10000000

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

// IPort Signals ...
sc_signal<bool> PN_NAME(stb_iport);
sc_signal<sc_uint<32>> PN_NAME(adr_iport);
sc_signal<sc_uint<4>> PN_NAME(bsel_iport);
sc_signal<sc_uint<32>> PN_NAME(rdata_iport);
sc_signal<bool> PN_NAME(ack_iport);
// DPort Signals ...
sc_signal<bool> PN_NAME(stb_dport);
sc_signal<bool> PN_NAME(we_dport);
sc_signal<sc_uint<32>> PN_NAME(adr_dport);
sc_signal<sc_uint<32>> PN_NAME(wdata_dport);
sc_signal<sc_uint<4>> PN_NAME(bsel_dport);
sc_signal<sc_uint<32>> PN_NAME(rdata_dport);
sc_signal<bool> PN_NAME(ack_dport);

/**
 * @brief simulating a read request from core
 * 
 * @param adr address for the read request
 * @param bsel byte select for the read request
 * @param IorD if true, read from IPort, else read from DPort
 * @return uint32_t 
 */
uint32_t readRequest(uint64_t adr, uint8_t bsel);

/**
 * @brief simualting a write request from core
 * 
 * @param data data to write
 * @param adr address to write to 
 * @param bsel byte select for the write request
 */
void writeRequest(uint32_t data, uint64_t adr, uint8_t bsel, bool IorD);

void run_cycle(int cycles = 1)
{
    for (int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char **argv)
{
    pn_parse_enable_trace_core = 1;                   // enable core trace dump
    PN_PARSE_CMD_ARGS(argc, argv);                    // parse command line arguments
    sc_trace_file *tf = PN_BEGIN_TRACE("emumemu_tb"); // create trace file
    FILE *coreDumpFile = fopen(CORE_TRACE_FILE, "a"); // open coreDump file for core requests

    // Initialliaze the Design under Testing (DUT)
    MSimmemu dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    // IPort Signal Map...
    dut_inst.stb_iport(stb_iport);
    dut_inst.adr_iport(adr_iport);
    dut_inst.bsel_iport(bsel_iport);

    dut_inst.rdata_iport(rdata_iport);
    dut_inst.ack_iport(ack_iport);
    // DPort Signal Map...
    dut_inst.stb_dport(stb_dport);
    dut_inst.we_dport(we_dport);
    dut_inst.adr_dport(adr_dport);
    dut_inst.wdata_dport(wdata_dport);
    dut_inst.bsel_dport(bsel_dport);

    dut_inst.rdata_dport(rdata_dport);
    dut_inst.ack_dport(ack_dport);

    // Traces of Signals
    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT
    // traces of local signals here

    sc_start(); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // ************** Add testbench code here ****************

    // Load the ELF file
    dut_inst.loadElf("hello_newlib");

    dut_inst.listAllPeripherals();
    // Attempt to find the Memory peripheral by its address
    uint64_t memoryAddress = MEM_BASE_ADR;                                     // Address where the memory peripheral is expected
    CSoftPeripheral *foundPeripheral = dut_inst.findPeripheral(memoryAddress); // search for peripheral in the list of peripherals
    if (foundPeripheral)
    {
        Memory *memory = dynamic_cast<Memory *>(foundPeripheral); // cast the found peripheral to Memory object
        if (memory)
        {
            // If the peripheral is correctly identified as a Memory object, dump its contents
            memory->dumpMemory("memory_dump.txt");
            std::cout << "Memory dump successful." << std::endl;
        }
        else
        {
            std::cerr << "Found peripheral is not a Memory object." << std::endl;
        }
    }
    else
    {
        std::cerr << "No peripheral found at address 0x" << std::hex << memoryAddress << "." << std::endl;
    }

    // *************** Reset test ***************
    run_cycle(2);
    reset = 1;
    PN_INFO("Set Reset");
    run_cycle(2);
    reset = 0;
    PN_INFO("Reset De-asserted");
    run_cycle(2);

    // *************** Memory Interface test ***************

    // DPort write to memory ...
    stb_dport = 1;            // set the strobe signal
    we_dport = 1;             // set the write enable signal
    adr_dport = 0x10000010;   // set the address
    wdata_dport = 0xDEADBEAF; // set the data to write
    bsel_dport = 0xF;         // set to 32 bit write
    run_cycle(1);
    stb_dport = 0; // reset stb signal
    we_dport = 0;  // reset we signal
    run_cycle(1);

    // Dport read from memory ...

    stb_dport = 1;          // set the strobe signal
    we_dport = 0;           // reset the write enable signal
    adr_dport = 0x10000010; // set the address
    bsel_dport = 0xF;       // set to 32 bit read
    run_cycle(1);
    stb_dport = 0; // reset stb signal
    run_cycle(1);

    // check the read DPort data
    cout << rdata_dport.read() << endl;
    if (rdata_dport.read() == 0xDEADBEAF)
    {
        PN_INFO("Memory DPort Write then Read test passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test failed");
    }

    // IPort read from memory ...
    stb_iport = 1;          // set the strobe signal
    adr_iport = 0x10000010; // set the address
    bsel_iport = 0xF;       // set to 32 bit read
    run_cycle(1);
    stb_iport = 0; // reset stb signal
    run_cycle(1);

    // check the read IPort data
    if (rdata_iport.read() == 0xDEADBEAF)
    {
        PN_INFO("IPort Read test passed");
    }
    else
    {
        PN_INFO("IPort Read test failed");
    }

    // --------------------------------------------------------

    run_cycle(2);

    // Make the Memory Dump again to see the change in a file
    CSoftPeripheral *foundPeripheral2 = dut_inst.findPeripheral(memoryAddress);
    if (foundPeripheral2)
    {
        Memory *memory = dynamic_cast<Memory *>(foundPeripheral2);
        if (memory)
        {
            // If the peripheral is correctly identified as a Memory object, dump its contents
            memory->dumpMemory("memory_dump2.txt");
            std::cout << "Memory dump successful." << std::endl;
        }
        else
        {
            std::cerr << "Found peripheral is not a Memory object." << std::endl;
        }
    }
    else
    {
        std::cerr << "No peripheral found at address 0x" << std::hex << memoryAddress << "." << std::endl;
    }

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}

void readRequest(uint64_t adr, uint8_t bsel, bool IorD)
{
    if (IorD)
    {
        // IPort read from memory ...
        stb_iport = 1;          // set the strobe signal
        adr_iport = adr; // set the address
        bsel_iport = bsel;       // set to 32 bit read
        run_cycle(1);
        stb_iport = 0;          // reset stb signal
        run_cycle(1);
    }
    else
    {
        stb_dport = 1;          // set the strobe signal
        we_dport = 0;           // reset the write enable signal
        adr_dport = adr; // set the address
        bsel_dport = bsel;       // set to 32 bit read
        run_cycle(1);
        stb_dport = 0; // reset stb signal
        run_cycle(1);
    }
}

void writeRequest(uint32_t data, uint64_t adr, uint8_t bsel)
{

    // DPort write to memory ...
    stb_dport = 1;            // set the strobe signal
    we_dport = 1;             // set the write enable signal
    adr_dport = adr;   // set the address
    wdata_dport = data; // set the data to write
    bsel_dport = bsel;         // set to 32 bit write
    run_cycle(1);
    stb_dport = 0; // reset stb signal
    we_dport = 0;  // reset we signal
    run_cycle(1);

}
