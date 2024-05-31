#pragma once

#include <cstdint>
#include <mcu_core.h>

extern "C" uint32_t SystemCoreClock;

namespace LPC4078 {

[[gnu::always_inline, gnu::optimize("O3")]] static inline void nop() {
  __asm__ __volatile__("mov r0, r0;\n\t":::);
}

[[gnu::always_inline, gnu::optimize("O3")]] static inline void __delay_4cycles(uint32_t cy) { // +1 cycle
  __asm__ __volatile__(
    "  .syntax unified\n\t" // is to prevent CM0,CM1 non-unified syntax
    "1:\n\t"
    "  subs %[cnt],#1\n\t" // 1
    "  mov r0, r0\n\t"            // 1
    "  bne 1b\n\t"         // 1 + (1? reload)
    : [cnt]"+r"(cy)   // output: +r means input+output
    :                 // input:
    : "cc"            // clobbers:
  );
}

// Delay in cycles
[[gnu::always_inline, gnu::optimize("03")]] static inline void delay_cycles(uint32_t x) {
  if (__builtin_constant_p(x)) {
    constexpr uint32_t MAXNOPS = 16;
    if (x <= (MAXNOPS)) {
      switch (x) { case 16: nop(); [[fallthrough]]; case 15: nop(); [[fallthrough]]; case 14: nop(); [[fallthrough]]; case 13: nop(); [[fallthrough]]; case 12: nop(); [[fallthrough]]; case 11: nop(); [[fallthrough]]; case 10: nop(); [[fallthrough]]; case  9: nop(); [[fallthrough]];
                   case  8: nop(); [[fallthrough]]; case  7: nop(); [[fallthrough]]; case  6: nop(); [[fallthrough]]; case  5: nop(); [[fallthrough]]; case  4: nop(); [[fallthrough]]; case  3: nop(); [[fallthrough]]; case  2: nop(); [[fallthrough]]; case  1: nop(); }
    } else { // because of +1 cycle inside delay_4cycles
      const uint32_t rem = (x - 1) % 4;
      switch (rem) { case 3: nop(); [[fallthrough]]; case 2: nop(); [[fallthrough]]; case 1: nop(); }
      if ((x = (x - 1) / 4))
        __delay_4cycles(x);
    }
  } else if ((x >>= 2)) __delay_4cycles(x);
}

[[gnu::always_inline]] static inline void delay_ns(const uint32_t x) {
  delay_cycles( x * (F_CPU / 1000000L) / 1000L );
}

[[gnu::always_inline]] static inline void delay_us(const uint32_t x) {
  delay_cycles( x * (F_CPU / 1000000L));
}

[[gnu::always_inline]] static inline void delay_ms(const uint32_t x) {
  delay_cycles( x * (F_CPU / 1000L));
}

namespace time {

extern volatile uint32_t systick_ticks;
extern volatile uint32_t systick_last_tick;

void init();

[[gnu::always_inline]] static inline uint16_t clockCyclesPerMicrosecond(){
  return ( (SystemCoreClock) / 1000000L );
}

[[gnu::always_inline]] static inline  uint32_t clockCyclesToMicroseconds(uint32_t cycles){
  return ( cycles / clockCyclesPerMicrosecond() );
}

[[gnu::always_inline]] static inline  uint32_t microsecondsToClockCycles(uint32_t microseconds){
  return ( microseconds * clockCyclesPerMicrosecond() );
}

[[gnu::always_inline]] static inline uint32_t millis() {
  return systick_ticks;
}

[[gnu::always_inline]] static inline uint32_t microseconds() {
  return (millis() * 1000) + clockCyclesToMicroseconds(MCUCore::data_watchpoint_trace->CYCCNT - systick_last_tick);
}

[[gnu::always_inline]] static inline void delay(uint32_t ms) {
  uint32_t start = systick_ticks;
  while (systick_ticks - start < ms);
}

[[gnu::always_inline]] static inline void delay_microseconds(uint32_t us) {
  uint32_t start = MCUCore::data_watchpoint_trace->CYCCNT, cycles = microsecondsToClockCycles(us);
  while (MCUCore::data_watchpoint_trace->CYCCNT - start < cycles);
}

} // time

} // LPC4078
