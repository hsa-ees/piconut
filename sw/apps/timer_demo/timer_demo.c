/*************************************************************************

  This file is part of the PicoNut project.

 *************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <timer_defs.h>

// Map timer registers using base address and offsets from timer_defs.h
#define TIMER_CONTROL (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_CR1)            // Control Register 1
#define TIMER_INTERRUPT_ENABLE (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_DIER)  // Interrupt Enable Register
#define TIMER_STATUS (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_SR)              // Status Register
#define TIMER_PRESCALER (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_PSC)          // Prescaler Register
#define TIMER_AUTO_RELOAD (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_ARR)        // Auto-Reload Register
#define TIMER_CAPTURE_COMPARE_1 (PN_CFG_TIMER_BASE_ADDRESS + M_TIMER_REG_CCR1) // Capture/Compare Register 1

// Interrupt bits
#define TIMER_STATUS_UIF (1 << 0)
#define TIMER_STATUS_CC1IF (1 << 1)
#define TIMER_IE_UIE (1 << 0)
#define TIMER_IE_CC1IE (1 << 1)

// Global counters
volatile uint32_t extern_interrupt_count = 0;
volatile uint32_t update_interrupt_count = 0;
volatile uint32_t compare_interrupt_count = 0;
volatile uint32_t total_interrupt_count = 0;
volatile uint32_t demo_complete = 0;

// External Interrupt Service Routine (similar to interrupt_demo.c)
void extern_isr(void)
{
    // Read status register
    volatile uint32_t* timer_status = (volatile uint32_t*)TIMER_STATUS;
    uint32_t status = *timer_status;

    // Handle update interrupt (overflow)
    if(status & TIMER_STATUS_UIF)
    {
        update_interrupt_count++;
        printf("Update ISR (Overflow): %lu\n", update_interrupt_count);
    }

    // Handle compare interrupt
    if(status & TIMER_STATUS_CC1IF)
    {
        compare_interrupt_count++;
        printf("Compare ISR (CC1): %lu\n", compare_interrupt_count);
    }

    // Update totals
    total_interrupt_count = update_interrupt_count + compare_interrupt_count;
    extern_interrupt_count = total_interrupt_count;

    // Clear only active flags (hardware uses write-1-to-clear)
    *timer_status = status & (TIMER_STATUS_UIF | TIMER_STATUS_CC1IF);

    // End demo after enough interrupts
    if(total_interrupt_count >= 10)
    {
        demo_complete = 1;
        // Stop timer
        volatile uint32_t* timer_control = (volatile uint32_t*)TIMER_CONTROL;
        *timer_control = 0x00; // CEN = 0
        printf("Timer stopped after %lu interrupts\n", total_interrupt_count);
    }
}

void __attribute__((interrupt)) handle_interrupt()
{
    // Read the Machine Cause (MCAUSE) CSR to determine interrupt type
    // MCAUSE contains both the interrupt flag and the specific cause code
    uint32_t mcause;
    __asm__ volatile("csrr %0, mcause" : "=r"(mcause));

    if((mcause & 0x80000000) && ((mcause & 0x7FFFFFFF) == 11))
    {
        // Dispatch to external interrupt service routine
        extern_isr();
    }
    // Note: Other interrupt types would be handled here in a more complete implementation
}

// Enable global interrupts (similar to interrupt_demo.c)
void enableInterrupts(void)
{
    // Set MTVEC
    void* trap_vector = handle_interrupt;
    __asm__ volatile("csrw mtvec, %0" : : "r"(trap_vector));

    // MSTATUS: Global Interrupt Enable
    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus));
    mstatus |= (1 << 3); // MIE
    __asm__ volatile("csrw mstatus, %0" : : "r"(mstatus));

    // MIE: Enable external interrupts
    uint32_t mie;
    __asm__ volatile("csrr %0, mie" : "=r"(mie));
    mie |= (1 << 11); // MEIE
    __asm__ volatile("csrw mie, %0" : : "r"(mie));
}

int main()
{
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== m_timer External Interrupt Demo ===\n");

    // Configure timer (use timing values from interrupt_demo.c)
    volatile uint32_t* timer_auto_reload = (volatile uint32_t*)TIMER_AUTO_RELOAD;
    volatile uint32_t* timer_prescaler = (volatile uint32_t*)TIMER_PRESCALER;
    volatile uint32_t* timer_cc1 = (volatile uint32_t*)TIMER_CAPTURE_COMPARE_1;
    volatile uint32_t* timer_control = (volatile uint32_t*)TIMER_CONTROL;
    volatile uint32_t* timer_ie = (volatile uint32_t*)TIMER_INTERRUPT_ENABLE;

    // Medium values to test timer logic (should show both interrupt types)
    *timer_auto_reload = 1000; // medium auto-reload value
    *timer_prescaler = 1;      // no prescaling
    *timer_cc1 = 500;          // compare at half the auto-reload value

    // Enable both interrupt types for full timer demo
    *timer_ie = TIMER_IE_UIE | TIMER_IE_CC1IE; // update + compare interrupts

    // Start timer
    *timer_control = 0x01; // CEN = 1

    // Enable global interrupts
    enableInterrupts();

    printf("Timer started. Waiting for external interrupts...\n");

    // Main loop: print status until demo completes
    while(!demo_complete)
    {
        printf("Status: Update: %lu, Compare: %lu, Total: %lu\n",
            update_interrupt_count,
            compare_interrupt_count,
            total_interrupt_count);

        // Longer delay for readability
        for(volatile int i = 0; i < 10000; i++)
        {
            __asm__ volatile("nop");
        }
    }

    printf("\n=== m_timer Demo Results ===\n");
    printf("Update Interrupts (Overflow): %lu\n", update_interrupt_count);
    printf("Compare Interrupts (CC1): %lu\n", compare_interrupt_count);
    printf("Total Interrupts: %lu\n", total_interrupt_count);
    printf("m_timer Demo completed successfully!\n");
    return 0;
}