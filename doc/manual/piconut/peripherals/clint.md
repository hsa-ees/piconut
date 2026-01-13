# CLINT (Core Local Interruptor)
**Author: Christian Zellinger, Alexander Beck 2025**

The CLINT (Core Local Interruptor) module provides timer and software interrupt functionality for the PicoNut processor, following the RISC-V specification. It is responsible for generating machine timer interrupts (MTIP) and machine software interrupts (MSIP) for the processor core.

There are two implementations of the CLINT module in the PicoNut project:
- The **Wishbone CLINT**: A hardware module connected to the Wishbone bus, intended for use on FPGAs.
- The **clint_soft**: A software model for simulation, implementing the same register interface for use in the simulator.

## CLINT Registers

The CLINT module exposes the following memory-mapped registers (all addresses are offsets from the CLINT base address, typically `0x2000000`):

| Register         | Offset   | Width | Description                                 |
|------------------|----------|-------|---------------------------------------------|
| MSIP             | 0x0000   | 32    | Machine Software Interrupt Pending          |
| MTIMECMP         | 0x4000   | 64    | Machine Timer Compare                       |
| MTIME            | 0xBFF8   | 64    | Machine Timer                               |

- **MSIP**: Writing a nonzero value to bit 0 sets the software interrupt pending flag.
- **MTIMECMP**: When `MTIME` is greater than or equal to `MTIMECMP`, the timer interrupt is triggered. **Important**: When writing to the 64-bit MTIMECMP register using 32-bit accesses, always write the high 32 bits first, then the low 32 bits to avoid race conditions per RISC-V specification.
- **MTIME**: Continuously incrementing timer, typically incremented every clock cycle.

## Wishbone CLINT




```{doxygenfunction} SC_MODULE(m_clint)
:project: m_clint
```

The Wishbone CLINT module consists of the following components:

- **Wishbone Slave Interface**: Handles Wishbone protocol transactions for register access.
- **Timer Logic**: Increments the `MTIME` register and compares it to `MTIMECMP` to generate timer interrupts.
- **Interrupt Outputs**: Generates the `msip_o` and `mtip_o` signals for software and timer interrupts.

### Register Access

Register access is handled via the Wishbone bus. The module supports both word and byte accesses, with proper masking and state machine handling for multi-cycle writes.

### Interrupt Generation

- **Software Interrupt (MSIP)**: Set by writing to the MSIP register. The `msip_o` output is asserted when bit 0 of MSIP is set.
- **Timer Interrupt (MTIP)**: Asserted when `MTIME` >= `MTIMECMP`. The `mtip_o` output reflects this condition.

## Clint Soft

The clint_soft module implements the CLINT functionality for simulation, using the `c_soft_peripheral` interface.



```{doxygengroup} clint_soft
:project: clint_soft
```

### CSoftPeripheral Interface

The following methods are implemented from the `c_soft_peripheral` interface to enable the CLINT module to be connected to the simulation environment:

```{doxygenfunction} m_clint_soft
:project: clint_soft
```

```{doxygenfunction} get_info
:project: clint_soft
```

```{doxygenfunction} read8
:project: clint_soft
```

```{doxygenfunction} write8
:project: clint_soft
```

```{doxygenfunction} write32
:project: clint_soft
```

```{doxygenfunction} read32
:project: clint_soft
```

```{doxygenfunction} is_addressed
:project: clint_soft
```

### CLINT Emulation

The following methods are implemented to emulate the CLINT's functionality, including timer increment, interrupt generation, and register access:

```{doxygenfunction} on_rising_edge_clock
:project: clint_soft
```

```{doxygenfunction} update_timer_interrupt
:project: clint_soft
```

```{doxygenfunction} register_msip_callback
:project: clint_soft
```

```{doxygenfunction} register_mtip_callback
:project: clint_soft
```

## Usage in Reference Design (clint_soft)

The following example shows how to integrate the `clint_soft` in an reference design. Also see `refdesign_clint_soft`.

### 1. Instantiation and Configuration

```cpp
#define CLINT_BASE_ADDR 0x2000000UL

// Declare signals for interrupt handling
sc_signal<bool> mtip_signal; // Timer interrupt signal
sc_signal<bool> msip_signal; // Software interrupt signal
sc_signal<bool> meip_signal; // External interrupt signal

// Create CLINT peripheral instance
std::unique_ptr<clint_soft> clint = std::make_unique<clint_soft>();
clint_soft* clint_ptr = clint.get();
```

### 2. Register Interrupt Callbacks

```cpp
// Register software interrupt callback
clint_ptr->register_msip_callback([&msip_signal](bool state) {
    msip_signal.write(state);
    std::cout << "Software interrupt state changed: " << state << std::endl;
});

// Register timer interrupt callback
clint_ptr->register_mtip_callback([&mtip_signal](bool state) {
    mtip_signal.write(state);
});

// Register external interrupt callback 
peripheral_ptr->register_meip_callback([&meip_signal](bool state) {
	meip_signal.write(state);
});
```

### 3. Add Peripheral to System

```cpp
// Add CLINT to peripheral container at standard RISC-V address
dut_inst.piconut->membrana->add_peripheral(PN_CFG_CLINT_BASE_ADDRESS, std::move(clint));

// Connect interrupt signals to PicoNut processor
dut_inst.piconut->mtip_in(mtip_signal);
dut_inst.piconut->msip_in(msip_signal);
dut_inst.piconut->meip_in(meip_signal);
```

### 4. Makefile Configuration

In the `Makefile`, the CLINT module must be added to the peripheral modules:

```makefile
# Include clint_soft as peripheral module
PERIPHERAL_MODULES = c_soft_uart clint_soft
```

### Key Steps for Integration:

1. **Create Instance**: `std::make_unique<clint_soft>()`
2. **Register Callbacks**: Setup interrupt callback functions for timer and software interrupts
3. **Add to Peripheral Container**: `membrana_soft->add_peripheral(PN_CFG_CLINT_BASE_ADDRESS, std::move(clint))`
4. **Connect Interrupt Signals**: Connect MTIP, MSIP, and MEIP signals to processor
5. **Configure Makefile**: Add `clint_soft` to `PERIPHERAL_MODULES`

## Usage in Hardware Reference Design (m_clint)

The following example shows how to integrate the `m_clint` in a hardware reference design. Also see `demo_clint`.

### 1. Module Declaration and Instantiation

```cpp
SC_MODULE(m_top)
{
    // Submodules
    m_piconut *piconut;
    m_clint *clint;
    m_uart *uart;

    // Internal interrupt signals
    sc_signal<bool> PN_NAME(mtip_signal); // Timer interrupt signal
    sc_signal<bool> PN_NAME(msip_signal); // Software interrupt signal
    sc_signal<bool> PN_NAME(meip_signal); // External interrupt signal
};
```

### 2. Clint Instantiation and Wishbone Connections

```cpp
// ----------- WB_CLINT -----------
clint = sc_new<m_clint>("clint");

// Clock and reset connections
clint->reset(reset);
clint->clk(clk);

// Wishbone bus connections
clint->wb_ack_o(wb_ack_clint);
clint->wb_dat_i(wb_dat_o);        // Data from master (PicoNut)
clint->wb_dat_o(wb_dat_i_clint);  // Data to master
clint->wb_we_i(wb_we);            // Write enable from master
clint->wb_stb_i(wb_stb);          // Strobe from master
clint->wb_cyc_i(wb_cyc);          // Cycle from master
clint->wb_sel_i(wb_sel_o);        // Byte select from master
clint->wb_rty_o(wb_rty_clint);    // Retry output
clint->wb_err_o(wb_err_clint);    // Error output
clint->wb_adr_i(wb_adr);          // Address from master

// Interrupt signal connections
clint->msip_o(msip_signal);       // Software interrupt output
clint->mtip_o(mtip_signal);       // Timer interrupt output
wb_peripheral->meip_o(meip_signal);  // External interrupt output
```

### 3. Processor Interrupt Connections

```cpp
// Connect the interrupt signals to the PicoNut processor
piconut->mtip_in(mtip_signal);
piconut->msip_in(msip_signal);
piconut->meip_in(meip_signal);
```

### 4. Address Decoding and Bus Arbitration

```cpp
void m_top::proc_comb_wb()
{
    if(wb_adr.read() >= PN_CFG_CLINT_BASE_ADDRESS && wb_adr.read() < (PN_CFG_CLINT_BASE_ADDRESS + (CLINT_SIZE)))
    {
        // Route to CLINT when address is in CLINT range
        wb_dat_i_pn = wb_dat_i_clint.read();
        wb_ack_pn = wb_ack_clint.read();
    }
    else
    {
        // Route to other peripherals for different addresses
        wb_dat_i_pn = wb_dat_i_uart.read();
        wb_ack_pn = wb_ack_uart.read();
    }
    meip_signal = 0;
}
```

### 5. Makefile Configuration

In the `Makefile`, include the clint module:

```makefile
# PERIPHERAL_MODULES: List of peripheral modules to include into the system
PERIPHERAL_MODULES = clint uart
```


### Key Steps for Hardware Integration:

1. **Instantiate Module**: Create `m_clint` instance in top-level module
2. **Connect Wishbone Bus**: Connect all Wishbone signals (address, data, control, acknowledge)
3. **Connect Interrupts**: Connect `msip_o` and `mtip_o` to processor interrupt inputs
4. **Configure Address Decoding**: Implement bus arbitration logic for CLINT address range
5. **Configure Makefile**: Add `clint` to `PERIPHERAL_MODULES`
6. **Set Base Address**: Define `PN_CFG_CLINT_BASE_ADDRESS` in configuration (if not already done)

## Usage Notes

- The CLINT is essential for timer-based scheduling and inter-processor software interrupts in RISC-V systems.
- The hardware and simulation models are designed to be register-compatible, allowing seamless switching between simulation and FPGA targets.
