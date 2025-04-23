#include <stdint.h>
#include <systemc.h>
#include "softmemu.h"
#include "elf_reader.h"
#include <iomanip>
#include "c_soft_fake_peripheral.h"

#define PERIOD_NS 10.0
#define MEM_BASE_ADR 0x10000000
#define FAKE_PERIPHERAL_BASE_ADR 0x40000000



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
void read_request(uint64_t adr, uint8_t bsel, bool IorD);

/**
 * @brief simualting a write request from core
 *
 * @param data data to write
 * @param adr address to write to
 * @param bsel byte select for the write request
 */
void write_request(uint32_t data, uint64_t adr, uint8_t bsel);

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
    sc_trace_file *tf = PN_BEGIN_TRACE("softmemu_tb"); // create trace file
    FILE *coreDumpFile = fopen(NUCLEUS_TRACE_FILE, "a"); // open coreDump file for core requests

    // Initialliaze the Design under Testing (DUT)
    m_soft_memu dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

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
    dut_inst.load_elf("hello_newlib");
    cout << "ELF file loaded successfully." << endl;

    dut_inst.list_all_peripherals();
    // Attempt to find the c_soft_memory peripheral by its address
    uint64_t memoryAddress = MEM_BASE_ADR;                                     // Address where the memory peripheral is expected
    c_soft_peripheral *foundPeripheral = dut_inst.find_peripheral(memoryAddress); // search for peripheral in the list of peripherals
    if (foundPeripheral)
    {
        c_soft_memory *memory = dynamic_cast<c_soft_memory *>(foundPeripheral); // cast the found peripheral to c_soft_memory object
        if (memory)
        {
            // If the peripheral is correctly identified as a c_soft_memory object, dump its contents
            memory->dump_memory("memory_dump_before_tests.txt");
            std::cout << "Memory dump successful." << std::endl;
        }
        else
        {
            std::cerr << "Found peripheral is not a c_soft_memory object." << std::endl;
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

    // *************** c_soft_memory Interface test ***************

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
        PN_INFO("Memory DPort Write then Read test1:1 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test1:1 failed");
        PN_ERRORF(("Expected: 0xDEADBEAF, Got: ", rdata_dport.read()));
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
        PN_INFO("Memory IPort Read test1:2 passed");
    }
    else
    {
        PN_INFO("Memory IPort Read test1:2 failed");
        PN_ERRORF(("Expected: 0xDEADBEAF, Got: ", rdata_iport.read()));
    }

    // *************** c_soft_memory Interface test2 ***************
    run_cycle(2);
    uint64_t testAdr = 0x10000100;
    uint32_t testData = 0xAAAAAAAA;
    for (int i = 0; i < 16; i++)
    {
        // Filling some memory space with ones to test for valid reads and wirtes
        write_request(0xFFFFFFFF, testAdr, 0xF); // 32bit write request to DPort
        run_cycle();
        testAdr += 4;
    }
    testAdr = 0x10000100;
    write_request(testData, testAdr, 0xF); // 32bit write request to DPort
    run_cycle();
    write_request(testData, testAdr+4, 0xC); // 16bit write request to DPort to upper half of the word
    run_cycle();
    write_request(testData, testAdr+8, 0x3); // 16bit write request to DPort to lower half of the word
    run_cycle();
    write_request(testData, testAdr+12, 0x1); // 8bit write request to DPort
    run_cycle();
    write_request(testData, testAdr+16, 0x2); // 8bit write request to DPort
    run_cycle();
    write_request(testData, testAdr+20, 0x4); // 8bit write request to DPort
    run_cycle();
    write_request(testData, testAdr+24, 0x8); // 8bit write request to DPort
    run_cycle();
    write_request(testData, testAdr+28, 0x0); // 8bit write request to DPort with bsel 0
    run_cycle();
    // c_soft_memory written start reading the values again
    // Read expects only the valid data to be present and rest to be 0 (in simulation)
    read_request(testAdr, 0xF, false);
    if(rdata_dport.read()  == 0xAAAAAAAA)
    {
        PN_INFO("Memory DPort Write then Read test2:1 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:1 failed");
        PN_ERRORF(("Expected: 0xAAAAAAAA, Got: ", rdata_dport.read()));
    }
    run_cycle();
    read_request(testAdr+4, 0xC, false);
    if(rdata_dport.read() == 0xAAAA0000)    // Read from upper half word
    {
        PN_INFO("Memory DPort Write then Read test2:2 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:2 failed");
        PN_ERRORF(("Expected: 0xAAAA0000, Got: ", rdata_dport.read()));
    }
    run_cycle();
    read_request(testAdr+8, 0x3, false);
    if(rdata_dport.read()  == 0x0000AAAA)   // Read from lower half word
    {
        PN_INFO("Memory DPort Write then Read test2:3 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:3 failed");
        PN_ERRORF(("Expected: 0x0000AAAA, Got: ", rdata_dport.read()));
    }
    run_cycle();
    read_request(testAdr+12, 0x1, false);
    if(rdata_dport.read()  == 0x000000AA)   // Read byte fom first byte
    {
        PN_INFO("Memory DPort Write then Read test2:4 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:4 failed");
        PN_ERRORF(("Expected: 0x000000AA, Got: ", rdata_dport.read()));
    }
    run_cycle();
    read_request(testAdr+16, 0x2, false);    // Read byte from second byte
    if(rdata_dport.read() == 0x0000AA00)
    {
        PN_INFO("Memory DPort Write then Read test2:5 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:5 failed");
        PN_ERRORF(("Expected: 0x0000AA00, Got: ", rdata_dport.read()));
    }
    run_cycle();
    read_request(testAdr+20, 0x4, false);    // Read Byte from third byte
    if(rdata_dport.read()  == 0x00AA0000)
    {
        PN_INFO("Memory DPort Write then Read test2:6 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:6 failed");
        PN_ERRORF(("Expected: 0x00AA0000, Got: ", rdata_dport.read()));
    }
    run_cycle();
    read_request(testAdr+24, 0x8, false);    // Read byte from fourth byte
    if(rdata_dport.read()  == 0xAA000000)
    {
        PN_INFO("Memory DPort Write then Read test2:7 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:7 failed");
        PN_ERRORF(("Expected: 0xAA000000, Got: ", rdata_dport.read()));
    }
    run_cycle();
    read_request(testAdr+28, 0x0, false);
    if(rdata_dport.read()  == 0x00000000)
    {
        PN_INFO("Memory DPort Write then Read test2:8 passed");
    }
    else
    {
        PN_INFO("Memory Dport Write then Read test2:8 failed");
        // PN_ERRORF(("Expected: 0x00000000, Got: ", rdata_dport.read()));
        cout << "Expected: 0x00000000, Got: " << hex << rdata_dport.read() << endl;
    }

    /*
    memory_dump should look like this after test2 is run:
    ...
    0x10000100: aa aa aa aa ff ff aa aa aa aa ff ff aa ff ff ff  |................|
    0x10000110: ff aa ff ff ff ff aa ff ff ff ff aa ff ff ff ff  |................|
    0x10000120: ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff  |................|
    0x10000130: ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff ff  |................|
    ...
    */
    // --------------------------------------------------------

    run_cycle(2);


    // *************** c_soft_memory Interface test3 ***************
    // TBD - Add more tests here
    // IPort tests for bsel if needed in future
    // --------------------------------------------------------

    // *************** c_soft_peripheral Interface test ***************
    PN_INFO("Test functionalty of getNumPeripheral and get_peripheral methods for map access");
    std::cout << "Number of peripherals: " << dut_inst.get_num_peripherals() << std::endl;
    c_soft_peripheral *testPeri;
    int testIndex = 0;
    cout << "Testing get_peripheral method with testIndex: " << testIndex << endl;
    testPeri = dut_inst.get_peripheral(testIndex);
    cout << "Information of Test Peripheral is: \n" << testPeri->get_info() << endl;
    testPeri->get_info();


    // Make the c_soft_memory Dump again to see the change in a file
    c_soft_peripheral *foundPeripheral2 = dut_inst.find_peripheral(memoryAddress);
    if (foundPeripheral2)
    {
        c_soft_memory *memory = dynamic_cast<c_soft_memory *>(foundPeripheral2);
        if (memory)
        {
            // If the peripheral is correctly identified as a c_soft_memory object, dump its contents
            memory->dump_memory("memory_dump_after_test.txt");
            std::cout << "Memory dump successful." << std::endl;
        }
        else
        {
            std::cerr << "Found peripheral is not a c_soft_memory object." << std::endl;
        }
    }
    else
    {
        std::cerr << "No peripheral found at address 0x" << std::hex << memoryAddress << "." << std::endl;
    }



    // *************** on rising edge clock test ***************

    std::unique_ptr<c_soft_fake_peripheral> fake_peripheral = 
        std::make_unique<c_soft_fake_peripheral>(c_soft_fake_peripheral::PERIPHERAL_SIZE, FAKE_PERIPHERAL_BASE_ADR);

    dut_inst.add_peripheral(FAKE_PERIPHERAL_BASE_ADR, std::move(fake_peripheral)); // add the peripheral to the DUT

    uint32_t        fake_peripheral_counter_value_1            = 0;
    uint32_t        fake_peripheral_counter_value_2            = 0;

    const uint32_t  cycles_to_run                              = 100;
    const uint32_t  cycles_needed_for_read_or_write_request     = 1;     // every time the register is accessed, the clock cycles and the counter increments 

    uint32_t read_and_write_requests = 0;


    read_request(FAKE_PERIPHERAL_BASE_ADR + c_soft_fake_peripheral::e_soft_regs::COUNTER, 0xF, true);
    fake_peripheral_counter_value_1 = rdata_iport.read();
    read_and_write_requests++;
    
    run_cycle(cycles_to_run);

    read_request(FAKE_PERIPHERAL_BASE_ADR + c_soft_fake_peripheral::e_soft_regs::COUNTER, 0xF, true);
    fake_peripheral_counter_value_2 = rdata_iport.read();
    read_and_write_requests++;

    if (fake_peripheral_counter_value_2 - fake_peripheral_counter_value_1 == cycles_to_run + read_and_write_requests * cycles_needed_for_read_or_write_request)
    {
        PN_INFO("on rising edge clock test passed");
    }
    else
    {
        PN_INFO("on rising edge clock test failed");
        PN_ERRORF(("Expected: 100, Got: %d", 
            fake_peripheral_counter_value_2 - fake_peripheral_counter_value_1 
            - read_and_write_requests * cycles_needed_for_read_or_write_request));
    }



    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}

void read_request(uint64_t adr, uint8_t bsel, bool IorD)
{
    uint32_t temp_data;
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

void write_request(uint32_t data, uint64_t adr, uint8_t bsel)
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
