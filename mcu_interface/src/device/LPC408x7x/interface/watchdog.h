#pragma once

#include <cstdint>
#include <utility/bit_manipulation.h>

namespace LPC4078 {
  struct WWDT_mode {
    WWDT_mode(const WWDT_mode&) = delete;
    union {
      struct {
        bool enable : 1;
        bool causes_reset : 1;
        bool has_triggered : 1;
        bool interrupt : 1;
        bool reload_protected : 1;
      };
      uint32_t raw_value;
    };
  };
  static_assert(sizeof(WWDT_mode) == sizeof(uint32_t));

  struct WWDT_register {
    WWDT_register(const WWDT_register&) = delete;
    WWDT_mode mode;          // RW - Watchdog mode register. This register determines the basic mode and status of the Watchdog Timer
    uint32_t timer_constant; // RW - Watchdog timer constant register. The value in this register determines the time-out value
    uint32_t feed;           // WO - Watchdog feed sequence register. Writing 0xAA followed by 0x55 to this register reloads the Watchdog timer with the value contained in WDTC
    uint32_t timer_value;    // RO - Watchdog timer value register. This register reads out the current value of the Watchdog timer
    uint32_t reserved;
    uint32_t interrupt_warning_timer_compare; // RW - Watchdog Warning Interrupt compare value
    uint32_t window_timer_compare;            // RW - Watchdog Window compare value
  };
  static_assert(sizeof(WWDT_register) == 0x1C);

  constexpr uint32_t watchdog_base_address = 0x4000'0000;
  constexpr uint32_t watchdog_base_frequency = 500'000 / 4; // 500K oscillator with fixed 4x divider

  static inline volatile auto& watchdog = *reinterpret_cast<volatile WWDT_register* const>(watchdog_base_address);

  [[gnu::always_inline]] inline void watchdog_feed() {
    watchdog.feed = 0xAA & MCUI::util::bitset_build_mask(0, 8);
    watchdog.feed = 0x55 & MCUI::util::bitset_build_mask(0, 8);
  }

  [[gnu::always_inline]] inline void watchdog_enable() {
    watchdog.mode.enable = true;
    watchdog_feed();
  }

  [[gnu::always_inline]] inline void watchdog_set_timeout_triggers_reset() {
    watchdog.mode.causes_reset = true;
  }

  [[gnu::always_inline]] inline void watchdog_set_reload_protected() {
    watchdog.mode.reload_protected = true;
  }

  [[gnu::always_inline]] inline void watchdog_set_timeout(const uint32_t value) {
    watchdog.timer_constant = value & MCUI::util::bitset_build_mask(0, 24);
  }

  [[gnu::always_inline, nodiscard]] inline uint32_t watchdog_get_timer_value() {
    return watchdog.timer_value & MCUI::util::bitset_build_mask(0, 24);
  }

  [[gnu::always_inline]] inline void watchdog_set_interrupt_compare(const uint32_t value) {
    watchdog.interrupt_warning_timer_compare = value & MCUI::util::bitset_build_mask(0, 10);
  }

  [[gnu::always_inline]] inline void watchdog_set_window_compare(const uint32_t value) {
    watchdog.interrupt_warning_timer_compare = value & MCUI::util::bitset_build_mask(0, 24);
  }

  [[gnu::always_inline]] inline void watchdog_set_timeout_in_seconds(const float value) {
    const uint32_t wtd_ticks = value * watchdog_base_frequency;
    watchdog_set_timeout(wtd_ticks);
  }

  [[gnu::always_inline]] inline void watchdog_clear_timeout_flag() {
    watchdog.mode.has_triggered = false;
  }

  [[gnu::always_inline, nodiscard]] inline bool watchdog_has_triggered() {
    return watchdog.mode.has_triggered;
  }

}
