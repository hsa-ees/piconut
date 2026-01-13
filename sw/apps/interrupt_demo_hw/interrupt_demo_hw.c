#include <stdio.h>
#include <stdint.h>

// Memory-mapped CLINT registers
#define PN_CFG_CLINT_BASE_ADDRESS 0x2000000
#define CLINT_REG_MTIMECMP_LO (PN_CFG_CLINT_BASE_ADDRESS + 0x4000) // Timer Compare Low
#define CLINT_REG_MTIMECMP_HI (PN_CFG_CLINT_BASE_ADDRESS + 0x4004) // Timer Compare High
#define CLINT_REG_MTIME_LO (PN_CFG_CLINT_BASE_ADDRESS + 0xBFF8)    // Timer Low
#define CLINT_REG_MTIME_HI (PN_CFG_CLINT_BASE_ADDRESS + 0xBFFC)    // Timer High

// Global variables for interrupt tracking
volatile uint32_t timer_interrupt_count = 0;
volatile uint32_t timer_interval = 0;

const uint32_t CLK_FRQ = 25000000;       // 25 MHz clock frequency
const uint32_t TIMER_INTERVALL_MS = 500; // 500 ms interval
const uint32_t TIMER_INTERVAL_TICKS = (TIMER_INTERVALL_MS * CLK_FRQ) / 1000;

void timer_isr(void)
{
    timer_interrupt_count++;

    volatile uint32_t* mtime_lo = (volatile uint32_t*)CLINT_REG_MTIME_LO;
    volatile uint32_t* mtime_hi = (volatile uint32_t*)CLINT_REG_MTIME_HI;
    uint32_t current_time_lo = *mtime_lo;
    uint32_t current_time_hi = *mtime_hi;

    volatile uint32_t* mtimecmp_lo = (volatile uint32_t*)CLINT_REG_MTIMECMP_LO;
    volatile uint32_t* mtimecmp_hi = (volatile uint32_t*)CLINT_REG_MTIMECMP_HI;

    uint64_t current_time = ((uint64_t)current_time_hi << 32) | current_time_lo;
    uint64_t next_time = current_time + timer_interval;

    *mtimecmp_lo = -1;
    *mtimecmp_hi = (uint32_t)(next_time >> 32);
    *mtimecmp_lo = (uint32_t)(next_time & 0xFFFFFFFF);

    printf("Timer interrupts: %u\n", timer_interrupt_count);
}

void __attribute__((interrupt)) handle_interrupt()
{
    uint32_t mcause;
    __asm__ volatile("csrr %0, mcause"
                     : "=r"(mcause));

    if((mcause & 0x80000000) && ((mcause & 0x7FFFFFFF) == 7))
    {
        timer_isr();
    }
}

void start_periodic_timer_interrupts(uint32_t interval_ticks)
{
    timer_interval = interval_ticks;

    volatile uint32_t* mtime_lo = (volatile uint32_t*)CLINT_REG_MTIME_LO;
    volatile uint32_t* mtime_hi = (volatile uint32_t*)CLINT_REG_MTIME_HI;
    uint32_t current_time_lo = *mtime_lo;
    uint32_t current_time_hi = *mtime_hi;

    uint64_t current_time = ((uint64_t)current_time_hi << 32) | current_time_lo;
    uint64_t next_time = current_time + interval_ticks;

    volatile uint32_t* mtimecmp_lo = (volatile uint32_t*)CLINT_REG_MTIMECMP_LO;
    volatile uint32_t* mtimecmp_hi = (volatile uint32_t*)CLINT_REG_MTIMECMP_HI;

    *mtimecmp_hi = (uint32_t)(next_time >> 32);
    *mtimecmp_lo = (uint32_t)(next_time & 0xFFFFFFFF);
}

void enableInterrupts()
{
    void* trap_vector = handle_interrupt;
    __asm__ volatile("csrw mtvec, %0"
                     :
                     : "r"(trap_vector));

    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus"
                     : "=r"(mstatus));
    mstatus |= (1 << 3);
    __asm__ volatile("csrw mstatus, %0"
                     :
                     : "r"(mstatus));

    uint32_t mie;
    __asm__ volatile("csrr %0, mie"
                     : "=r"(mie));
    mie |= (1 << 7);
    __asm__ volatile("csrw mie, %0"
                     :
                     : "r"(mie));
}

int main()
{
    setvbuf(stdout, NULL, _IONBF, 0);

    enableInterrupts();

    start_periodic_timer_interrupts(12500000);

    while(1)
    {

        for(int i = 0; i < 100000; i++)
        {
            __asm__ volatile("nop");
        }
    }

    return 0;
}