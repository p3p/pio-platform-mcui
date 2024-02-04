#pragma once

#include "system_control.h"

namespace LPC4078 {

struct TimerRegion {
  uint32_t IR;                     // 0x000, RW, Interrupt Register
  uint32_t TCR;                    // 0x004, RW, Timer Control Register
  uint32_t TC;                     // 0x008, RW, Timer Counter Register
  uint32_t PR;                     // 0x00C, RW, Prescale Register
  uint32_t PC;                     // 0x010, RW, Prescale Counter Register
  uint32_t MCR;                    // 0x014, RW, Match Control Register
  uint32_t MR0;                    // 0x018, RW, Match Register 0
  uint32_t MR1;                    // 0x01C, RW, Match Register 1
  uint32_t MR2;                    // 0x020, RW, Match Register 2
  uint32_t MR3;                    // 0x024, RW, Match Register 3
  uint32_t CCR;                    // 0x028, RW, Capture Control Register
  uint32_t CR0;                    // 0x02C, RO, Capture Register 0
  uint32_t CR1;                    // 0x030, RO, Capture Register 1
  uint32_t RESERVED0[2];
  uint32_t EMR;                    // 0x03C, RW, External Match Register
  uint32_t RESERVED1[12];
  uint32_t CTCR;                   // 0x070, RW, Count Control Register
};
static_assert(sizeof(TimerRegion) == 0x074);

static auto& timer0 = *reinterpret_cast<volatile TimerRegion* const>(0x40004000);
static auto& timer1 = *reinterpret_cast<volatile TimerRegion* const>(0x40008000);
static auto& timer2 = *reinterpret_cast<volatile TimerRegion* const>(0x40090000);
static auto& timer3 = *reinterpret_cast<volatile TimerRegion* const>(0x40094000);

}
