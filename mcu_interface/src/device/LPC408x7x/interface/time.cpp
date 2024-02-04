#include "time.h"
#include "uart.h"
#include "systick.h"

namespace LPC4078::time {

volatile uint32_t systick_ticks = 0;
volatile uint32_t systick_last_tick = 0;

extern "C" void SysTick_Handler() {
  ++systick_ticks;
  systick_last_tick = MCUCore::data_watchpoint_trace->CYCCNT;
}

void init() {
  MCUCore::core_debug->DEMCR |= (1UL << 24); //CoreDebug_DEMCR_TRCENA_Msk; // Enable Trace
  #if __CORTEX_M == 7
    DWT->LAR = 0xC5ACCE55;                        // Unlock access to DWT registers
  #endif
  MCUCore::data_watchpoint_trace->CYCCNT = 0;
  MCUCore::data_watchpoint_trace->CTRL |= 0x1UL; //DWT_CTRL_CYCCNTENA_Msk;
  MCUCore::sys_tick_configure(SystemCoreClock / 1000);
}

}
