#pragma once

#include <cstdint>

#include <utility/const_functions.h>
#include <utility/registers.h>

namespace cortex_m4 {
using namespace MCUI::util::registers;

// Instruction Synchronization Barrier flushes the pipeline in the processor,
// so that all instructions following the ISB are fetched from cache or memory,
// after the instruction has been completed.
[[gnu::always_inline]] static inline void ISB() {
  __asm volatile ("isb 0xF":::"memory");
}

// Data Synchronization Barrier acts as a special kind of Data Memory Barrier.
// It completes when all explicit memory accesses before this instruction complete.
[[gnu::always_inline]] static inline void DSB() {
  __asm volatile ("dsb 0xF":::"memory");
}

// Data Memory Barrier ensures the apparent order of the explicit memory operations before
// and after the instruction, without ensuring their completion.
[[gnu::always_inline]] static inline void DMB() {
  __asm volatile ("dmb 0xF":::"memory");
}

[[gnu::always_inline]] static inline void NOP() {
  __asm__ volatile ("nop");
}

struct SystemControlBlock {
  uint32_t CPUID;         // 0x000, RO, CPUID Base Register
  uint32_t ICSR;          // 0x004, RW, Interrupt Control and State Register
  uint32_t VTOR;          // 0x008, RW, Vector Table Offset Register

  struct AIRCRLayout {
    union {
      reg32_wo_t<bool,  0, 1> VECTRESET;
      reg32_wo_t<bool,  1, 1> VECTCLRACTIVE;
      reg32_wo_t<bool,  2, 1> SYSRESETREQ;
      reg32_t<uint8_t,  8, 3> PRIGROUP;
      reg32_ro_t<bool, 15, 1> ENDIANNESS;
      reg32_wo_t<uint16_t, 16, 16> VECTKEYSTAT;
    };
    static inline constexpr uint16_t VECTKEY = 0x05FA;
  } AIRCR;         // 0x00C, RW, Application Interrupt and Reset Control Register

  uint32_t SCR;           // 0x010, RW, System Control Register
  uint32_t CCR;           // 0x014, RW, Configuration Control Register
  uint8_t  SHP[12];       // 0x018, RW, System Handlers Priority Registers (4-7, 8-11, 12-15)
  uint32_t SHCSR;         // 0x024, RW, System Handler Control and State Register
  uint32_t CFSR;          // 0x028, RW, Configurable Fault Status Register
  uint32_t HFSR;          // 0x02C, RW, HardFault Status Register
  uint32_t DFSR;          // 0x030, RW, Debug Fault Status Register
  uint32_t MMFAR;         // 0x034, RW, MemManage Fault Address Register
  uint32_t BFAR;          // 0x038, RW, BusFault Address Register
  uint32_t AFSR;          // 0x03C, RW, Auxiliary Fault Status Register
  uint32_t PFR[2];        // 0x040, RO, Processor Feature Register
  uint32_t DFR;           // 0x048, RO, Debug Feature Register
  uint32_t ADR;           // 0x04C, RO, Auxiliary Feature Register
  uint32_t MMFR[4];       // 0x050, RO, Memory Model Feature Register
  uint32_t ISAR[5];       // 0x060, RO, Instruction Set Attributes Register
  uint32_t RESERVED0[5];
  uint32_t CPACR;         // 0x088, RW, Coprocessor Access Control Register
};
static_assert(sizeof(SystemControlBlock) == 0x8C);
static inline volatile SystemControlBlock* const system_control_block = reinterpret_cast<SystemControlBlock* const>(0xE000ED00);

struct DataWatchpointTrace {
  uint32_t CTRL;          // 0x000, RW, Control Register
  uint32_t CYCCNT;        // 0x004, RW, Cycle Count Register
  uint32_t CPICNT;        // 0x008, RW, CPI Count Register
  uint32_t EXCCNT;        // 0x00C, RW, Exception Overhead Count Register
  uint32_t SLEEPCNT;      // 0x010, RW, Sleep Count Register
  uint32_t LSUCNT;        // 0x014, RW, LSU Count Register
  uint32_t FOLDCNT;       // 0x018, RW, Folded-instruction Count Register
  uint32_t PCSR;          // 0x01C, RO, Program Counter Sample Register
  uint32_t COMP0;         // 0x020, RW, Comparator Register 0
  uint32_t MASK0;         // 0x024, RW, Mask Register 0
  uint32_t FUNCTION0;     // 0x028, RW, Function Register 0
  uint32_t RESERVED0;
  uint32_t COMP1;         // 0x030, RW, Comparator Register 1
  uint32_t MASK1;         // 0x034, RW, Mask Register 1
  uint32_t FUNCTION1;     // 0x038, RW, Function Register 1
  uint32_t RESERVED1;
  uint32_t COMP2;         // 0x040, RW, Comparator Register 2
  uint32_t MASK2;         // 0x044, RW, Mask Register 2
  uint32_t FUNCTION2;     // 0x048, RW, Function Register 2
  uint32_t RESERVED2;
  uint32_t COMP3;         // 0x050, RW, Comparator Register 3
  uint32_t MASK3;         // 0x054, RW, Mask Register 3
  uint32_t FUNCTION3;     // 0x058, RW, Function Register 3
};
static_assert(sizeof(DataWatchpointTrace) == 0x5C);
static inline volatile DataWatchpointTrace* const data_watchpoint_trace = reinterpret_cast<DataWatchpointTrace* const>(0xE0001000);

struct CoreDebug {
  uint32_t DHCSR;         // 0x000, RW, Debug Halting Control and Status Register
  uint32_t DCRSR;         // 0x004, WO, Debug Core Register Selector Register
  uint32_t DCRDR;         // 0x008, RW, Debug Core Register Data Register
  uint32_t DEMCR;         // 0x00C, RW, Debug Exception and Monitor Control Register
};
static_assert(sizeof(CoreDebug) == 0x10);
static inline volatile CoreDebug* const core_debug = reinterpret_cast<CoreDebug* const>(0xE000EDF0UL);

struct NestedVectoredInterruptController {
  uint32_t ISER[8];        // 0x000, RW, Interrupt Set Enable Register
  uint32_t RESERVED0[24];
  uint32_t ICER[8];        // 0x080, RW, Interrupt Clear Enable Register
  uint32_t RSERVED1[24];
  uint32_t ISPR[8];        // 0x100, RW, Interrupt Set Pending Register
  uint32_t RESERVED2[24];
  uint32_t ICPR[8];        // 0x180, RW, Interrupt Clear Pending Register
  uint32_t RESERVED3[24];
  uint32_t IABR[8];        // 0x200, RW, Interrupt Active bit Register
  uint32_t RESERVED4[56];
  uint8_t  IP[240];        // 0x300, RW, Interrupt Priority Register (8Bit wide)
  uint32_t RESERVED5[644];
  uint32_t STIR;           // 0xE00, WO, Software Trigger Interrupt Register
};
static_assert(sizeof(NestedVectoredInterruptController) == 0xE04);
static inline volatile NestedVectoredInterruptController* const nvic = reinterpret_cast<NestedVectoredInterruptController* const>(0xE000E100);

constexpr uint32_t __NVIC_PRIO_BITS = 5;
[[gnu::always_inline]] static inline void nvic_set_priority(const MCUI::IRQNumber interrupt_number, const uint32_t priority) {
  auto irq = MCUI::util::to_integral(interrupt_number);
  if(irq < 0) { // set Priority for Cortex-M  System Interrupts
    system_control_block->SHP[((uint32_t)(irq) & 0xF)-4] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff);
  } else { // set Priority for device specific Interrupts
    nvic->IP[(uint32_t)(irq)] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff);
  }
}

[[gnu::always_inline]] constexpr uint32_t nvic_encode_priority(const uint32_t priority_group, const uint32_t priority_preempt, const uint32_t priority_sub) {
  const uint32_t priority_group_masked = (priority_group & 0x07UL);
  const uint32_t priority_preempt_bits = ((7UL - priority_group_masked) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - priority_group_masked);
  const uint32_t priority_sub_bits = ((priority_group_masked + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((priority_group_masked - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));
  return ((priority_preempt & (uint32_t)((1UL << (priority_preempt_bits)) - 1UL)) << priority_sub_bits) | ((priority_sub & (uint32_t)((1UL << (priority_sub_bits)) - 1UL)));
}

[[gnu::always_inline]] static inline void nvic_enable_irq(const MCUI::IRQNumber interrupt_number) {
  auto irq = MCUI::util::to_integral(interrupt_number);
  if (irq >= 0) {
    __asm volatile("":::"memory");
    nvic->ISER[(((uint32_t)irq) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)irq) & 0x1FUL));
    __asm volatile("":::"memory");
  }
}

[[gnu::always_inline]] static inline void nvic_disable_irq(const MCUI::IRQNumber interrupt_number) {
  auto irq = MCUI::util::to_integral(interrupt_number);
  if (irq >= 0) {
    nvic->ICER[(((uint32_t)irq) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)irq) & 0x1FUL));
    DSB();
    ISB();
  }
}

[[gnu::always_inline]] static inline uint32_t nvic_irq_is_enabled(const MCUI::IRQNumber interrupt_number) {
  auto irq = MCUI::util::to_integral(interrupt_number);
  if (irq >= 0) {
    return (nvic->ISER[(((uint32_t)irq) >> 5UL)] & (uint32_t)(1UL << (((uint32_t)irq) & 0x1FUL))) ? 1UL : 0UL;
  }
  return 0;
}

[[noreturn]] static inline void nvic_system_reset() {
  DSB(); // Ensure all outstanding memory accesses included buffered write are completed before reset
  // this is convoluted because you need to set correct VECTKEY into VECTKEYSTAT on every write to the AIRCR register
  system_control_block->AIRCR.SYSRESETREQ.underlying_value_ref() = system_control_block->AIRCR.SYSRESETREQ.shift_and_mask(true)
                                                                 | system_control_block->AIRCR.PRIGROUP.masked_underlying_value()
                                                                 | system_control_block->AIRCR.VECTKEYSTAT.shift_and_mask(system_control_block->AIRCR.VECTKEY);
  DSB();
  while(true);
}

[[gnu::always_inline, nodiscard]] static inline uint32_t primask() {
  uint32_t result;
  asm volatile ("MRS %0, primask" : "=r" (result) );
  return result;
}

[[gnu::always_inline]] static inline void primask(uint32_t pm) {
  asm volatile ("MSR primask, %0" : : "r" (pm) : "memory");
}

[[gnu::always_inline]] static inline void nvic_interrupts_enable() {
  asm volatile ("cpsie i" : : : "memory");
}

[[gnu::always_inline]] static inline void nvic_interrupts_disable() {
  asm volatile ("cpsid i" : : : "memory");
}

}
