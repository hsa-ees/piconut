# Interrupt Implementation

The PicoNut system implements a comprehensive RISC-V machine mode interrupt handling system compliant with the RISC-V Privileged Architecture specification. The implementation includes:

- **Three standard RISC-V interrupt types**: Software Interrupt (MSIP), Timer Interrupt (MTIP), and External Interrupt (MEIP)
- **Complete CSR register set**: MIP, MIE, MSTATUS, MTVEC, MEPC, MCAUSE, MTVAL
- **Hardware trap handling state machine** with proper priority handling
- **CLINT (Core Local Interruptor)** implementation for timer and software interrupts
- **Standard memory-mapped interrupt control** at RISC-V specified addresses

## RISC-V Interrupt Types and Priority

The system implements the three standard machine mode interrupt types with proper priority handling:

1. **Machine External Interrupt (MEIP)** - Highest Priority (Exception Code: 11)
2. **Machine Timer Interrupt (MTIP)** - Medium Priority (Exception Code: 7) 
3. **Machine Software Interrupt (MSIP)** - Lowest Priority (Exception Code: 3)

Interrupts are processed according to RISC-V specification priority rules, with external interrupts taking precedence over timer interrupts, which take precedence over software interrupts.

## CSR Register Implementation

### Machine Interrupt Pending (MIP) Register
- **Address**: 0x344
- **MSIP bit [3]**: Software interrupt pending flag
- **MTIP bit [7]**: Timer interrupt pending flag  
- **MEIP bit [11]**: External interrupt pending flag

### Machine Interrupt Enable (MIE) Register
- **Address**: 0x304
- **MSIE bit [3]**: Software interrupt enable
- **MTIE bit [7]**: Timer interrupt enable
- **MEIE bit [11]**: External interrupt enable

### Machine Status (MSTATUS) Register
- **Address**: 0x300
- **MIE bit [3]**: Global machine interrupt enable flag
- **MPIE bit [7]**: Previous interrupt enable (saved during trap)

### Machine Trap Vector (MTVEC) Register
- **Address**: 0x305
- **BASE field [31:2]**: Trap handler base address
- **MODE field [1:0]**: Vector mode (0=Direct, 1=Vectored)

### Machine Exception Program Counter (MEPC) Register
- **Address**: 0x341
- **Stores**: PC value to return to after trap handling

### Machine Cause (MCAUSE) Register
- **Address**: 0x342
- **Interrupt bit [31]**: Set for interrupts, clear for exceptions
- **Exception Code [30:0]**: Specific interrupt/exception type

### Machine Trap Value (MTVAL) Register
- **Address**: 0x343
- **Implementation**: Basic read-write register (software controlled)
- **Note**: Currently implemented as a stub - no automatic hardware population of trap values

## CLINT (Core Local Interruptor) Implementation

The system provides two CLINT implementations for different use cases:

### CLINT Soft (m_clint_soft)
- **Purpose**: Simulation and software testing
- **Implementation**: C++ based simulation model

### CLINT in Hardware (m_clint)
- **Purpose**: Synthesis and hardware deployment
- **Implementation**: SystemC with Wishbone interface

### CLINT Memory Map
All CLINT implementations follow standard RISC-V memory mapping:

- **Base Address**: 0x2000000
- **MSIP Register**: 0x2000000 (Software interrupt trigger)
- **MTIMECMP Register**: 0x2004000 (Timer compare value)
- **MTIME Register**: 0x200BFF8 (Current timer value)

## Trap Handling State Machine

The controller implements a comprehensive trap handling state machine with the following states:

1. **STATE_TRAP_ENTRY**: Initial trap detection and PC save to MEPC
2. **STATE_TRAP_SAVE_MCAUSE**: Save interrupt cause to MCAUSE register
3. **STATE_TRAP_UPDATE_MSTATUS**: Read current MSTATUS register
4. **STATE_TRAP_UPDATE_MSTATUS2**: Clear MIE bit and update MSTATUS register  
5. **STATE_TRAP_LOAD_HANDLER**: Load trap handler address from MTVEC
6. **STATE_TRAP_SAVE_PC**: Jump to trap handler address
7. **STATE_MRET**: Return from trap (MRET instruction)

### Trap Entry Sequence
The hardware implements a detailed multi-state trap entry sequence:

1. **STATE_TRAP_ENTRY**: 
   - Current PC automatically saved to MEPC register (0x341)
   - Interrupt signal asserted to CSR module
   - PC increment/load disabled during context save

2. **STATE_TRAP_SAVE_MCAUSE**: 
   - Interrupt type determined by priority: External (11) > Timer (7) > Software (3)
   - MCAUSE register (0x342) written with interrupt flag (bit 31=1) + cause code
   - Hardware checks MIP/MIE register combinations for pending interrupts

3. **STATE_TRAP_UPDATE_MSTATUS**: 
   - Current MSTATUS register (0x300) read to preserve state

4. **STATE_TRAP_UPDATE_MSTATUS2**: 
   - Current MIE bit saved to MPIE field in MSTATUS
   - MIE bit cleared to disable global interrupts
   - MSTATUS register updated atomically

5. **STATE_TRAP_LOAD_HANDLER**: 
   - Trap handler base address read from MTVEC register (0x305)
   - Handler address prepared for PC loading

6. **STATE_TRAP_SAVE_PC**: 
   - PC module signaled to load trap handler address
   - Control transfers to interrupt handler
   - Returns to instruction fetch cycle

### Trap Return Sequence
The MRET instruction triggers a streamlined return sequence:

1. **STATE_MRET Execution**: 
   - MRET instruction decoded and recognized
   - CSR module signaled to restore interrupt state
   - Memory access signals explicitly cleared to prevent glitches

2. **MSTATUS Restoration**: 
   - MIE bit restored from MPIE field (re-enable global interrupts)
   - MPIE field set to 1 (per RISC-V specification)
   - Trap state transitions to exit mode

3. **PC Restoration**: 
   - PC module prioritizes MRET signal over all other control inputs
   - Program counter directly loaded from MEPC register value
   - Execution resumes at pre-trap instruction address

4. **Execution Resume**: 
   - System returns to normal instruction fetch cycle
   - Interrupts re-enabled based on restored MIE bit
   - Trap handling context fully restored

## Interrupt Signal Flow Architecture

### External Interrupt Inputs
The system receives interrupt signals from external sources:
- **msip_in**: Software interrupt input from CLINT
- **mtip_in**: Timer interrupt input from CLINT  
- **meip_in**: External interrupt input from peripheral devices

### CSR to Controller Interface
The CSR module provides status information to the controller:
- **mip_msip/mtip/meip**: Current interrupt pending status
- **mie_msip/mtie/meie**: Interrupt enable mask bits
- **mstatus_mie**: Global interrupt enable flag

### Controller to CSR Interface
The controller sends control signals back to CSR:
- **int_ack**: Interrupt acknowledgment signal
- **mret_out**: MRET instruction execution signal
- **Various CSR write enables**: For updating trap-related registers

### Controller to PC Interface
The controller manages program counter during traps:
- **trap_active**: Indicates trap handling in progress
- **trap_addr**: Trap handler address from MTVEC
- **PC control signals**: For saving/restoring execution context

## Software Programming Guide

This programming guide demonstrates how to implement interrupt handling. Also see the full example in the `interrupt_demo.c` application.

### Memory Map and Register Definitions

First, include the Piconut definitions for the CLINT (Core Local Interruptor) memory-mapped registers:

```c
#include <clint_defs.h>
```

**Explanation:**
- `CLINT_BASE_ADDR`: Base address for the Core Local Interruptor, following RISC-V standard
- `CLINT_REG_MSIP_ADDR`: Software interrupt register - writing 1 triggers interrupt, writing 0 clears it
- `CLINT_REG_MTIMECMP_LO/HI`: 64-bit timer compare value split into low and high 32-bit registers
- `CLINT_REG_MTIME_LO/HI`: 64-bit current timer value split into low and high 32-bit registers

### Global Variables for Interrupt Management

```c
// Global variables for interrupt tracking
volatile uint32_t timer_interrupt_count = 0;
volatile uint32_t sw_interrupt_count = 0;
volatile uint32_t timer_running = 0;
volatile uint32_t timer_interval = 0;
volatile uint32_t mtime_hi_value = 0;
volatile uint32_t overflow_count = 0;
volatile uint32_t sw_interrupt_active = 0;
```

**Explanation:**
- All variables are declared `volatile` to prevent compiler optimization that could interfere with interrupt handling
- `timer_interrupt_count`: Counts number of timer interrupts received
- `sw_interrupt_count`: Counts number of software interrupts processed
- `timer_running`: Flag indicating if periodic timer is active
- `timer_interval`: Number of timer ticks between interrupts
- `mtime_hi_value`: Tracks high register value for overflow detection
- `overflow_count`: Counts timer register overflows
- `sw_interrupt_active`: Prevents software interrupt re-triggering

### Main Interrupt Handler

The main interrupt handler is the entry point for all interrupts:

```c
void __attribute__((interrupt)) handle_interrupt() {
    // Get cause of the interrupt
    uint32_t mcause;
    __asm__ volatile("csrr %0, mcause" : "=r"(mcause));
    
    // Check if it's a timer interrupt (bit 31 set, code = 7)
    if ((mcause & 0x80000000) && ((mcause & 0x7FFFFFFF) == 7)) {
        timer_isr();
    }
    // Check if it's a software interrupt (bit 31 set, code = 3)
    else if ((mcause & 0x80000000) && ((mcause & 0x7FFFFFFF) == 3)) {
        sw_isr();
    }
    // Check if it's an external interrupt (bit 31 set, code = 11)
    else if ((mcause & 0x80000000) && ((mcause & 0x7FFFFFFF) == 11)) {
        extern_isr();
    }
}
```

**Explanation:**
- `__attribute__((interrupt))`: GCC attribute that generates proper interrupt prologue/epilogue code
- `csrr %0, mcause`: Inline assembly to read the MCAUSE CSR register
- `mcause & 0x80000000`: Check bit 31 to determine if it's an interrupt (1) or exception (0)
- `mcause & 0x7FFFFFFF`: Extract the exception code (bits 0-30)
- Timer interrupt code = 7, Software interrupt code = 3, External interrupt code = 11
- The handler dispatches to specific ISR functions based on interrupt type

### Software Interrupt Handler

```c
void sw_isr(void) {
    // Clear the software interrupt by writing 0 to MSIP
    volatile uint32_t *msip = (volatile uint32_t *)CLINT_REG_MSIP_ADDR;
    *msip = 0;
    
    // Reset the flag and increment counter
    sw_interrupt_active = 0;
    printf("Software interrupt handled\n");
    printf("sw interrupt active: %d\n", sw_interrupt_active);
    sw_interrupt_count++;
}
```

**Explanation:**
- First action is to clear the interrupt source by writing 0 to MSIP register
- This prevents the interrupt from immediately re-triggering
- Updates global counters and flags for application tracking
- Uses `volatile` pointer to ensure memory access is not optimized away

### Timer Interrupt Handler

```c
void timer_isr(void) {
    timer_interrupt_count++;
    
    // Read current timer values
    volatile uint32_t *mtime_lo = (volatile uint32_t *)CLINT_REG_MTIME_LO;
    volatile uint32_t *mtime_hi = (volatile uint32_t *)CLINT_REG_MTIME_HI;
    uint32_t current_time_lo = *mtime_lo;
    uint32_t current_time_hi = *mtime_hi;
    
    // Track high register changes for information purposes
    if (current_time_hi != mtime_hi_value) {
        overflow_count++;
        mtime_hi_value = current_time_hi;
    }
    
    // Set next interrupt time (full 64-bit comparison)
    volatile uint32_t *mtimecmp_lo = (volatile uint32_t *)CLINT_REG_MTIMECMP_LO;
    volatile uint32_t *mtimecmp_hi = (volatile uint32_t *)CLINT_REG_MTIMECMP_HI;
    
    // Calculate next compare time
    uint64_t current_time = ((uint64_t)current_time_hi << 32) | current_time_lo;
    uint64_t next_time = current_time + timer_interval;
    
    // Set both registers for proper 64-bit comparison (first set LO to maximum value to prevent 
    // spurious timer interrupts during the atomic 64-bit update sequence)
    *mtimecmp_lo = -1; 
    // Write high register to avoid race conditions during the update
    *mtimecmp_hi = (uint32_t)(next_time >> 32);
    *mtimecmp_lo = (uint32_t)(next_time & 0xFFFFFFFF);
}
```

**Explanation:**
- Reads the current 64-bit timer value from MTIME registers
- Tracks timer overflow by monitoring changes in the high register
- Calculates the next interrupt time by adding the timer interval
- Sets MTIMECMP registers in correct order (HI first, then LO) to schedule the next timer interrupt per RISC-V standard
- Uses proper 64-bit arithmetic to handle timer overflow correctly
- Timer interrupt automatically clears when MTIMECMP is updated

### External Interrupt Handler

```c
void extern_isr(void) {
    // This is a placeholder for external interrupts
    printf("External interrupt handled\n");
}
```

**Explanation:**
- Placeholder for handling external interrupts from peripheral devices
- External interrupt clearing depends on the specific interrupt source
- In a real implementation, would identify and clear the specific external interrupt source

### Software Interrupt Triggering

```c
void trigger_sw_interrupt() {
    volatile uint32_t *msip = (volatile uint32_t *)CLINT_REG_MSIP_ADDR;
    uint32_t current_msip = *msip;
    
    printf("Debug: About to trigger interrupt, MSIP=%d\n", current_msip);
    
    // Only set MSIP if it's not already set
    if (current_msip == 0) {
        *msip = 1;  // Set MSIP to trigger software interrupt
        sw_interrupt_active = 1;
        printf("Software interrupt triggered\n");
    } else {
        printf("Debug: Not triggering - MSIP already set\n");
    }
}
```

**Explanation:**
- Checks current MSIP register value to avoid double-triggering
- Sets MSIP to 1 to trigger a software interrupt
- Updates tracking flag to prevent re-triggering
- Software interrupts are edge-triggered and must be manually cleared in the ISR

### Periodic Timer Setup

```c
void start_periodic_timer_interrupts(uint32_t interval_ticks) {
    timer_interval = interval_ticks;
    
    // Read initial mtime value
    volatile uint32_t *mtime_lo = (volatile uint32_t *)CLINT_REG_MTIME_LO;
    volatile uint32_t *mtime_hi = (volatile uint32_t *)CLINT_REG_MTIME_HI;
    uint32_t current_time_lo = *mtime_lo;
    uint32_t current_time_hi = *mtime_hi;
    
    // Store the current high value for overflow detection
    mtime_hi_value = current_time_hi;
    
    // Calculate next compare time as full 64-bit value
    uint64_t current_time = ((uint64_t)current_time_hi << 32) | current_time_lo;
    uint64_t next_time = current_time + interval_ticks;
    
    // Get pointers to MTIMECMP registers
    volatile uint32_t *mtimecmp_lo = (volatile uint32_t *)CLINT_REG_MTIMECMP_LO;
    volatile uint32_t *mtimecmp_hi = (volatile uint32_t *)CLINT_REG_MTIMECMP_HI;
    
    // Set both MTIMECMP registers for proper 64-bit comparison (HI first to avoid race conditions)
    *mtimecmp_hi = (uint32_t)(next_time >> 32);
    *mtimecmp_lo = (uint32_t)(next_time & 0xFFFFFFFF);
    
    printf("mtime: 0x%08x%08x\n", current_time_hi, current_time_lo);
    printf("setting mtimecmp to 0x%08x%08x\n", 
           (uint32_t)(next_time >> 32), (uint32_t)(next_time & 0xFFFFFFFF));
    
    // Mark timer as running
    timer_running = 1;
    
    printf("Started periodic timer interrupts every %u ticks\n", interval_ticks);
}
```

**Explanation:**
- Reads current timer value to set first interrupt time
- Calculates next interrupt time by adding interval to current time
- Sets both MTIMECMP registers in correct order (HI first, then LO) for proper 64-bit comparison per RISC-V standard
- Timer interrupt triggers when MTIME >= MTIMECMP
- Provides debug output showing timer values for verification

### Interrupt System Initialization

```c
void enableInterrupts() {
    void* trap_vector = handle_interrupt;
    printf("Setting up interrupt handler at address: %p\n", trap_vector);
    __asm__ volatile("csrw mtvec, %0" : : "r"(trap_vector));

    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    mstatus |= (1 << 3); // Set MIE bit
    __asm__ volatile("csrw mstatus, %0" : : "r"(mstatus));
    
    uint32_t mie;
    __asm__ volatile("csrr %0, mie" : "=r"(mie));
    mie |= (1 << 7); // Set MTIE bit for timer interrupts
    mie |= (1 << 3); // Set MSIE bit for software interrupts
    mie |= (1 << 11); // Set MEIE bit for external interrupts
    __asm__ volatile("csrw mie, %0" : : "r"(mie));
}
```

**Explanation:**
- `csrw mtvec, %0`: Sets MTVEC register to point to interrupt handler function
- `mstatus |= (1 << 3)`: Sets MIE bit in MSTATUS to enable global interrupts
- `mie |= (1 << 7)`: Sets MTIE bit in MIE to enable timer interrupts
- `mie |= (1 << 3)`: Sets MSIE bit in MIE to enable software interrupts
- `mie |= (1 << 11)`: Sets MEIE bit in MIE to enable external interrupts
- All three interrupt types must be individually enabled in MIE register

### Complete Main Program

```c
int main() {
    // Set up the interrupt handler
    setvbuf(stdout, NULL, _IONBF, 0); // Important for printf to work correctly with interrupts
    
    enableInterrupts();
    
    // Start periodic timer interrupts (every 1,000,000 cycles)
    printf("Starting periodic timer interrupts...\n");
    start_periodic_timer_interrupts(1000000);
    
    // Main program loop - keep running while interrupts occur
    printf("Entering main program loop with both timer and software interrupts...\n");
    
    // Counter for software interrupt triggering
    uint32_t sw_counter = 0;
    uint32_t sw_interval = 10;

    // Simple counter for the main program to show it's still running
    uint32_t main_counter = 0;

    while(sw_interrupt_count < 5) {
        // Increment our software interrupt counter
        sw_counter++;
        
        // Every sw_interval iterations, try to trigger a software interrupt
        if (sw_counter >= sw_interval) {
            // Read MSIP register directly
            volatile uint32_t *msip = (volatile uint32_t *)CLINT_REG_MSIP_ADDR;
            
            if (*msip == 0) {
                printf("Debug: sw_counter(%u) reached threshold, MSIP=%u\n", 
                       sw_counter, *msip);
                trigger_sw_interrupt();
            } else {
                printf("Debug: Not triggering - MSIP already pending\n");
            }
            
            sw_counter = 0; // Reset counter regardless
        }

        // Add a small delay to avoid flooding the console with prints
        for(int i = 0; i < 100; i++) {
            __asm__ volatile("nop");
        }

        main_counter++;
        printf("Status: Timer interrupts: %u, Software interrupts: %u\n", 
               timer_interrupt_count, sw_interrupt_count);
    }
    
    return 0;
}
```

**Explanation:**
- `setvbuf(stdout, NULL, _IONBF, 0)`: Disables output buffering for reliable printf in interrupt context
- Initializes interrupt system and starts periodic timer
- Main program loop demonstrates concurrent interrupt handling
- Software interrupts are triggered periodically from main loop
- Timer interrupts occur automatically based on MTIMECMP settings
- Program terminates after receiving 5 software interrupts
- Status printing shows interrupt counters being updated by ISRs

### Programming Best Practices

1. **Always use volatile for interrupt-shared variables** - Prevents compiler optimization
2. **Clear interrupt sources in ISR** - Prevents immediate re-triggering
3. **Keep ISRs short and simple** - Minimizes interrupt latency
4. **Use proper 64-bit timer arithmetic** - Handles timer overflow correctly
5. **Check interrupt status before triggering** - Prevents race conditions
6. **Disable output buffering** - Ensures printf works reliably in interrupt context
7. **Use appropriate compiler attributes** - `__attribute__((interrupt))` for proper context save/restore

