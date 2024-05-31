#pragma once

#include <utility/bit_manipulation.h>
#include <utility/registers.h>

#include "system_control.h"
#include "pinmapping.h"
#include "gpio.h"

#include "pwm.h"

namespace LPC4078 {

class HardwarePWM {

  // return a reference to a PWM timer register using a lookup table as they are not contiguous
  [[nodiscard]] static constexpr uint32_t match_register_lookup(const pin_t pin) noexcept {
    constexpr uint32_t MR0_OFFSET = 6, MR4_OFFSET = 16;
    auto base = pin_get_hardware_index(pin) == 0 ? pwm0_hardware_address : pwm1_hardware_address;
    return base + (pin_get_pwm_channel(pin) > 3 ? (MR4_OFFSET + pin_get_pwm_channel(pin) - 4) : (MR0_OFFSET + pin_get_pwm_channel(pin))) * sizeof(uint32_t);
  }

  // return a reference to a PWM timer register using a lookup table as they are not contiguous
  [[nodiscard]] static constexpr auto match_register_ptr(const pin_t pin) noexcept {
    return MCUI::util::memory_ptr<uint32_t>(match_register_lookup(pin));
  }

  // generate a unique bit for each hardware PWM capable pin
  [[nodiscard]] static constexpr uint8_t get_pin_id(const pin_type query_pin) noexcept {
    auto value = ((((query_pin.port() - 1) * 2) + query_pin.get_pwm_hardware_index()) * 6) + query_pin.get_pwm_channel_index();
    return value > 11 ? value - 6 : value;
  }

  // update the bitset an activate hardware pwm channel for output
  static inline void activate_channel(const pin_t pin) {
    volatile PulseWidthModulation *device = pin_get_hardware_index(pin) == 0 ? pwm_device0 : pwm_device1;
    MCUI::util::bit_set(active_pins, get_pin_id(pin));         // mark the pin as active
    MCUI::util::bit_set(active_channels, (2 * pin_get_hardware_index(pin)) + pin_get_pwm_channel_index(pin));         // mark the channel as active
    MCUI::util::bit_set(device->PCR, 8 + pin_get_pwm_channel(pin));  // turn on the pins PWM output (8 offset + PWM channel)
    pin_enable_pwm(pin);
  }

  // update the bitset and deactivate the hardware pwm channel
  static inline void deactivate_channel(const pin_t pin) {
    volatile PulseWidthModulation *device = pin_get_hardware_index(pin) == 0 ? pwm_device0 : pwm_device1;
    MCUI::util::bit_clear(active_pins, get_pin_id(pin));      // mark pin as inactive
    MCUI::util::bit_clear(active_channels, (2 * pin_get_hardware_index(pin)) + pin_get_pwm_channel_index(pin));
    if(!channel_active(pin)) MCUI::util::bit_clear(device->PCR, 8 + pin_get_pwm_channel(pin)); // turn off the PWM output
  }

    // return true if a pwm channel is already attached to a pin
  [[nodiscard]] static constexpr bool channel_active(const pin_t pin) noexcept {
    const uint32_t channel = (2 * pin_get_hardware_index(pin)) + pin_get_pwm_channel_index(pin);
    return pin_has_pwm(pin) && MCUI::util::bit_test(active_channels, channel);
  }

public:
  //static void init(const uint32_t prescale, const uint32_t period) {
  [[gnu::noinline]] static void init(volatile PulseWidthModulation *device, const uint32_t frequency) {
    peripheral_power_on(PeripheralPowerControl::PWM0);
    peripheral_power_on(PeripheralPowerControl::PWM1);

    // Make sure it is in a clean state
    device->IR = 0xFF & 0x0000073FUL;
    // why this mask
    device->TCR = 0;
    device->CTCR = 0;
    device->MCR = 0;
    device->CCR = 0;
    device->PCR &= 0xFF00;
    device->LER = 0;

    // No clock prescaler
    device->PR = 0;

    // Configured to reset TC if it matches MR0, No interrupts
    device->MCR = MCUI::util::bit_value(1);

    // Set the period using channel 0 before enabling peripheral
    device->MR0 = (PeripheralClock / frequency) - 1;

    // Enable Counters(bit0) & Turn on PWM Mode(bit3)
    device->TCR = MCUI::util::bitset_value(0, 3);
  }

  [[nodiscard]] static constexpr bool available(const pin_t pin) noexcept {
    return pin_has_pwm(pin) && !channel_active(pin);
  }

  // return true if a pin is already attached to PWM hardware
  [[nodiscard]] static constexpr bool active(const pin_t pin) noexcept {
    return pin_has_pwm(pin) && MCUI::util::bit_test(active_pins, get_pin_id(pin));
  }

  static inline void set_frequency(const pin_t pin, const uint32_t frequency) {
    set_period(pin, PeripheralClock / frequency);
  }

  static inline void set_period(const pin_t pin, const uint32_t period) {
    volatile PulseWidthModulation *device = pin_get_hardware_index(pin) == 0 ? pwm_device0 : pwm_device1;
    device->TCR = MCUI::util::bit_value(1);
    device->MR0 = period - 1;               // TC resets every period cycles (0 counts so remove 1)
    device->LER = MCUI::util::bitset_value(0);    // Set Latch on MR0
    device->TCR = MCUI::util::bitset_value(0, 3); // Turn on PWM Mode(bit3) and Enable Counters(bit0)
  }

  static inline uint32_t get_period(const pin_t pin) {
    volatile PulseWidthModulation *device = pin_get_hardware_index(pin) == 0 ? pwm_device0 : pwm_device1;
    return device->MR0 + 1; // Add 1 to reflect 0 count.
  }

  static inline void set_us(const pin_t pin, const uint32_t value) {
    set_match(pin, (PeripheralClock / 1000000) * value);
  }

  // update the match register for a channel and set the latch to update on next period
  static inline void set_match(const pin_t pin, const uint32_t value) {
    if (!pin_has_pwm(pin)) return;
    volatile PulseWidthModulation *device = pin_get_hardware_index(pin) == 0 ? pwm_device0 : pwm_device1;
    *match_register_ptr(pin) = value == device->MR0 ? value + 1 : value; // work around for bug if MRn == MR0
    device->LER |= MCUI::util::bit_value(pin_get_pwm_channel(pin));                    // Enable Latch for MRn channel
  }

  static inline bool attach(const pin_t pin, const uint32_t value) {
    if(!available(pin)) return false;
    set_match(pin, value);
    activate_channel(pin);
    return true;
  }

  static inline bool detach(const pin_t pin) {
    if (active(pin)) {
      pin_enable_function(pin, LPC4078::Function::GPIO); // reenable gpio
      gpio_clear(pin);
      deactivate_channel(pin);
      return true;
    }
    return false;
  }

private:
  // 32bit bitset used to track whether a pin is actively using hardware pwm
  static uint32_t active_pins;
  static uint32_t active_channels;
};
}
