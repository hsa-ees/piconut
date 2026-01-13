# Timer
**Author: Alexander Beck, Christian Zellinger 2025**

The Timer module provides programmable timer functionality for the PicoNut processor. It is a versatile hardware timer component designed specifically for time-critical applications such as FreeRTOS tick generation and other real-time applications.

The m_timer peripheral is a comprehensive timer implementation that offers advanced configuration options and is fully accessible through the Wishbone interface.

## Timer Registers

The m_timer module implements an STM32 TIM2 compatible register layout with the following memory-mapped registers (all addresses are offsets from the timer base address).

Note: The canonical base address and per-register offsets are defined in hw/peripherals/timer/timer_defs.h as PN_CFG_TIMER_BASE_ADDRESS and the M_TIMER_REG_* enum constants. Use those constants in code to avoid hard-coded addresses.

| Register         | Offset | Width | Access | Description                           |
|------------------|--------|-------|--------|---------------------------------------|
| CR1              | 0x00   | 32    | RW     | Control Register 1                  (M_TIMER_REG_CR1)  |
| CR2              | 0x04   | 32    | RW     | Control Register 2 (dummy)          (M_TIMER_REG_CR2)  |
| SMCR             | 0x08   | 32    | RW     | Slave Mode Control Register (dummy) (M_TIMER_REG_SMCR) |
| DIER             | 0x0C   | 32    | RW     | Interrupt Enable Register           (M_TIMER_REG_DIER) |
| SR               | 0x10   | 32    | RW     | Status Register                     (M_TIMER_REG_SR)   |
| EGR              | 0x14   | 32    | WO     | Event Generation Register (dummy)     |
| CCMR1            | 0x18   | 32    | RW     | Capture/Compare Mode Register 1 (dummy) |
| CCMR2            | 0x1C   | 32    | RW     | Capture/Compare Mode Register 2 (dummy) |
| CCER             | 0x20   | 32    | RW     | Capture/Compare Enable Register (dummy) |
| CNT              | 0x24   | 32    | RW     | Counter Register                 (M_TIMER_REG_CNT)     |
| PSC              | 0x28   | 32    | RW     | Prescaler Register               (M_TIMER_REG_PSC)     |
| ARR              | 0x2C   | 32    | RW     | Auto-Reload Register             (M_TIMER_REG_ARR)     |
| CCR1             | 0x34   | 32    | RW     | Capture/Compare Register 1       (M_TIMER_REG_CCR1)    |
| CCR2             | 0x38   | 32    | RW     | Capture/Compare Register 2            |
| CCR3             | 0x3C   | 32    | RW     | Capture/Compare Register 3            |
| CCR4             | 0x40   | 32    | RW     | Capture/Compare Register 4            |

### Control Register 1 (CR1)

The Control Register 1 configures the basic timer operation:

| Bit | Name | Description |
|-----|------|-------------|
| 0   | CEN  | Counter Enable - Start/stop the timer |
| 1   | UDIS | Update Disable - Disable update events |
| 2   | URS  | Update Request Source - Source of update events |
| 3   | OPM  | One Pulse Mode - Timer stops after one period |
| 4   | DIR  | Direction - 0: Up-counting, 1: Down-counting |
| 6:5 | CMS  | Center-aligned Mode Selection |
| 7   | ARPE | Auto-reload Preload Enable |

### Status Register (SR)

The Status Register contains interrupt flags:

| Bit | Name  | Description |
|-----|-------|-------------|
| 0   | UIF   | Update Interrupt Flag - Set on counter overflow/underflow |
| 1   | CC1IF | Capture/Compare 1 Interrupt Flag |
| 2   | CC2IF | Capture/Compare 2 Interrupt Flag |
| 3   | CC3IF | Capture/Compare 3 Interrupt Flag |
| 4   | CC4IF | Capture/Compare 4 Interrupt Flag |
| 5   | TIF   | Trigger Interrupt Flag |
| 6   | BIF   | Break Interrupt Flag |

### Interrupt Enable Register (DIER)

The Interrupt Enable Register controls which interrupts are enabled:

| Bit | Name  | Description |
|-----|-------|-------------|
| 0   | UIE   | Update Interrupt Enable |
| 1   | CC1IE | Capture/Compare 1 Interrupt Enable |
| 2   | CC2IE | Capture/Compare 2 Interrupt Enable |
| 3   | CC3IE | Capture/Compare 3 Interrupt Enable |
| 4   | CC4IE | Capture/Compare 4 Interrupt Enable |
| 5   | TIE   | Trigger Interrupt Enable |
| 6   | BIE   | Break Interrupt Enable |

## Timer Operation

The m_timer provides several key functionalities:

### Counter Operation

The timer includes a 32-bit counter that can operate in different modes:

- **Up-counting**: Counter increments from 0 to the auto-reload value
- **Down-counting**: Counter decrements from the auto-reload value to 0
- **One-pulse mode**: Timer automatically stops after one complete cycle

### Prescaler

The 32-bit prescaler divides the input clock frequency:

```
f_timer = f_clock / (Prescaler + 1)
```

A prescaler value of 0 runs the timer at full clock frequency, while higher values create proportionally slower timer frequencies.

### Auto-Reload

The auto-reload register defines the timer period:

- **Up-counting**: When counter reaches ARR value, it resets to 0
- **Down-counting**: When counter reaches 0, it reloads with ARR value
- **One-pulse mode**: Timer stops after reload event

### Capture/Compare

The timer provides four capture/compare channels (CCR1-CCR4) that can:

- Generate interrupts when the counter matches the compare value
- Trigger events for external peripherals
- Measure input signal timing (capture mode)

## Timer Module

The Timer module consists of the following components:

- **Wishbone Slave Interface**: Handles Wishbone protocol transactions for register access
- **Timer Logic**: Implements the core timer functionality with configurable counting direction
- **Prescaler Unit**: 32-bit prescaler for clock frequency division
- **Capture/Compare Units**: Four independent capture/compare channels
- **Interrupt Controller**: STM32-compatible interrupt generation logic

### Interrupt Generation

The timer generates interrupts for the following events:

- **Update Interrupt (UIF)**: Generated on counter overflow/underflow
- **Capture/Compare Interrupts (CC1IF-CC4IF)**: Generated when counter matches compare values

The `timer_irq` output is asserted when any enabled interrupt condition occurs.

**Note**: Currently only UIF and CC1IF-CC4IF interrupts are functionally implemented in the hardware. TIF and BIF are defined in the register structure for STM32 compatibility but do not generate interrupts.

## Programming Guide

This section demonstrates how to use the m_timer peripheral with practical examples based on the timer_demo application.

### Basic Timer Setup

To configure the timer from C code prefer using the constants from timer_defs.h:

```c
#include <stdio.h>
#include <stdint.h>
#include <timer_defs.h>

// Use PN_CFG_TIMER_BASE_ADDRESS and M_TIMER_REG_* offsets from timer_defs.h
#define TIMER_CR1  (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_CR1)
#define TIMER_DIER (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_DIER)
#define TIMER_SR   (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_SR)
#define TIMER_CNT  (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_CNT)
#define TIMER_PSC  (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_PSC)
#define TIMER_ARR  (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_ARR)
#define TIMER_CCR1 (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_CCR1)

// Status and interrupt enable bit definitions (unchanged)
#define TIMER_STATUS_UIF   (1 << 0)
#define TIMER_STATUS_CC1IF (1 << 1)
#define TIMER_IE_UIE       (1 << 0)
#define TIMER_IE_CC1IE     (1 << 1)
```

### Timer Configuration

Replace previous hard-coded-offset examples with the constants:

```c
void configure_timer(void) {
    volatile uint32_t *timer_arr = (volatile uint32_t *)TIMER_ARR;
    volatile uint32_t *timer_psc = (volatile uint32_t *)TIMER_PSC;
    volatile uint32_t *timer_ccr1 = (volatile uint32_t *)TIMER_CCR1;
    volatile uint32_t *timer_cr1 = (volatile uint32_t *)TIMER_CR1;
    volatile uint32_t *timer_dier = (volatile uint32_t *)TIMER_DIER;

    // Set auto-reload value (timer period)
    *timer_arr = 1000;     // Count from 0 to 1000

    // Set prescaler (clock division)
    *timer_psc = 1;        // Divide clock by (1 + prescaler) -> here 2

    // Set compare value for capture/compare interrupt
    *timer_ccr1 = 500;     // Interrupt at half period

    // Enable interrupts
    *timer_dier = TIMER_IE_UIE | TIMER_IE_CC1IE; // Update + Compare interrupts

    // Start timer
    *timer_cr1 = 0x01;     // CEN=1 (Counter Enable)
}
```

### Interrupt Handling

Implement interrupt service routines to handle timer events:

```c
// Global interrupt counters
volatile uint32_t update_interrupt_count = 0;
volatile uint32_t compare_interrupt_count = 0;

// Interrupt Service Routine
void timer_isr(void) {
    volatile uint32_t *timer_sr = (volatile uint32_t *)TIMER_SR;
    uint32_t status = *timer_sr;
    
    // Handle update interrupt (overflow)
    if (status & TIMER_STATUS_UIF) {
        update_interrupt_count++;
        printf("Timer overflow: %lu\n", update_interrupt_count);
    }
    
    // Handle capture/compare interrupt
    if (status & TIMER_STATUS_CC1IF) {
        compare_interrupt_count++;
        printf("Compare match: %lu\n", compare_interrupt_count);
    }
    
    // Clear interrupt flags (write-1-to-clear)
    *timer_sr = status & (TIMER_STATUS_UIF | TIMER_STATUS_CC1IF);
}

// Top-level interrupt handler
void __attribute__((interrupt)) handle_interrupt(void) {
    timer_isr();
}
```

### RISC-V Interrupt Setup

Enable global interrupts and configure the interrupt vector:

```c
void enable_interrupts(void) {
    // Set trap vector
    void* trap_vector = handle_interrupt;
    __asm__ volatile("csrw mtvec, %0" : : "r"(trap_vector));

    // Enable global interrupts (MIE in MSTATUS)
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    mstatus |= (1 << 3); // MIE bit
    __asm__ volatile("csrw mstatus, %0" : : "r"(mstatus));

    // Enable external interrupts (MEIE in MIE)
    uint32_t mie;
    __asm__ volatile("csrr %0, mie" : "=r"(mie));
    mie |= (1 << 11); // MEIE bit
    __asm__ volatile("csrw mie, %0" : : "r"(mie));
}
```

### Complete Example

Here's a complete example demonstrating timer usage:

```c
int main() {
    printf("=== m_timer Demo ===\n");
    
    // Configure timer
    configure_timer();
    
    // Enable interrupts
    enable_interrupts();
    
    printf("Timer started. Waiting for interrupts...\n");
    
    // Main loop
    while (1) {
        // Display status
        printf("Status: Update: %lu, Compare: %lu\n", 
               update_interrupt_count, compare_interrupt_count);
        
        // Stop after 10 total interrupts
        if ((update_interrupt_count + compare_interrupt_count) >= 10) {
            volatile uint32_t *timer_cr1 = (volatile uint32_t *)TIMER_CR1;
            *timer_cr1 = 0x00; // Stop timer
            break;
        }
        
        // Delay
        for (volatile int i = 0; i < 10000; i++) {
            __asm__ volatile("nop");
        }
    }
    
    printf("Timer demo completed!\n");
    return 0;
}
```
