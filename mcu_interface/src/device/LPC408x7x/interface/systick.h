#pragma once

#include <cstdint>
#include <mcu_core.h>
#include <utility/const_functions.h>

#include "system_control.h"

namespace cortex_m4 {

struct SysTick {
  uint32_t CTRL;     // 0x000, RW, SysTick Control and Status Register
  uint32_t LOAD;     // 0x004, RW, SysTick Reload Value Register
  uint32_t VAL;      // 0x008, RW, SysTick Current Value Register
  uint32_t CALIB;    // 0x00C, RO, SysTick Calibration Register
};
static_assert(sizeof(SysTick) == 0x10);
static inline volatile SysTick* const sys_tick = reinterpret_cast<volatile SysTick*>(0xE000E010);

[[gnu::always_inline]] static inline uint32_t sys_tick_configure(const uint32_t ticks) {
  constexpr uint32_t SysTick_LOAD_RELOAD_Msk = 0xFFFFFFUL << 0;
  constexpr uint32_t SysTick_CTRL_CLKSOURCE_Msk = 0x01UL << 2;
  constexpr uint32_t SysTick_CTRL_TICKINT_Msk = 0x01UL << 1;
  constexpr uint32_t SysTick_CTRL_ENABLE_Msk = 0x01UL << 0;

  if (ticks > SysTick_LOAD_RELOAD_Msk) return 1;  // Reload value impossible
  sys_tick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;     // set reload register
  nvic_set_priority(LPC4078::IRQNumber::SysTick, MCUCore::nvic_encode_priority(0, 1, 0));
  sys_tick->VAL   = 0;
  sys_tick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
  nvic_enable_irq(LPC4078::IRQNumber::SysTick);
  return 0;
}

}
