#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "pin_definitions.h"

namespace LPC4078 {

static constexpr size_t pin_count = 165;
template <size_t S, typename T, typename R = decltype(std::declval<T>()({}))>
static constexpr auto constexpr_generate_array(T fn) {
  std::array<R, S> a{};
  for (size_t i = 0; i < S; ++i) { a[i] = fn(i); }
  return a;
}

typedef int16_t pin_t;
enum PinMode : uint8_t {
  INACTIVE = 0,
  PULLDOWN,
  PULLUP,
  REPEATER,
  NORMAL = 0,
  OPENDRAIN,
  HYSTERESIS_DISABLED = 0,
  HYSTERESIS_ENABLED,
  INVERTED = 1,
  FAST_SLEW = 1
};

enum Function : uint8_t {
  GPIO,
  FUNC1,
  FUNC2,
  FUNC3,
};


constexpr uint8_t calc_pwm_channel(const size_t index) {
  if ( index == P1_02 || index == P1_18 || index == P2_00 || index == P3_16 || index == P3_24 ) return 1;
  if ( index == P1_03 || index == P1_20 || index == P2_01 || index == P3_17 || index == P3_25 ) return 2;
  if ( index == P1_05 || index == P1_21 || index == P2_02 || index == P3_18 || index == P3_26 ) return 3;
  if ( index == P1_06 || index == P1_23 || index == P2_03 || index == P3_19 || index == P3_27 ) return 4;
  if ( index == P1_07 || index == P1_24 || index == P2_04 || index == P3_20 || index == P3_28 ) return 5;
  if ( index == P1_11 || index == P1_26 || index == P2_05 || index == P3_21 || index == P3_29 ) return 6;
  return 0xFF;
}
constexpr uint8_t calc_pwm_function_index(const size_t index) {
  if ( index == P2_00 || index == P2_01 || index == P2_02 || index == P2_03 || index == P2_04 || index == P2_05 )  return 1;
  if ((index == P1_18 || index == P1_20 || index == P1_21 || index == P1_23 || index == P1_24 || index == P1_26 ) ||
      (index == P3_16 || index == P3_17 || index == P3_18 || index == P3_19 || index == P3_20 || index == P3_21 ) ||
      (index == P3_24 || index == P3_25 || index == P3_26 || index == P3_27 || index == P3_28 || index == P3_29 )) return 2;
  if ( index == P1_02 || index == P1_03 || index == P1_05 || index == P1_06 || index == P1_07 || index == P1_11 )  return 3;
  return 0xFF;
}
constexpr uint8_t calc_pwm_hardware_index(const size_t index) {
  if (index == P1_02 || index == P1_03 || index == P1_05 || index == P1_06 || index == P1_07 || index == P1_11 ||
      index == P3_16 || index == P3_17 || index == P3_18 || index == P3_19 || index == P3_20 || index == P3_21 ) return 0;
  if (index == P1_18 || index == P1_20 || index == P1_21 || index == P1_23 || index == P1_24 || index == P1_26 ||
      index == P2_00 || index == P2_01 || index == P2_02 || index == P2_03 || index == P2_04 || index == P2_05 ||
      index == P3_24 || index == P3_25 || index == P3_26 || index == P3_27 || index == P3_28 || index == P3_29 ) return 1;
  return 0xFF;
}
constexpr uint8_t calc_adc_channel(const size_t index) {
  switch (index) {
    case P0_12: return 6;
    case P0_13: return 7;
    case P0_23: return 0;
    case P0_24: return 1;
    case P0_25: return 2;
    case P0_26: return 3;
    case P1_30: return 4;
    case P1_31: return 5;
  }
  return 0xFF;
}
constexpr uint8_t calc_adc_function_index(const size_t index) {
  switch (index) {
    case P0_12: return 3;
    case P0_13: return 3;
    case P0_23: return 1;
    case P0_24: return 1;
    case P0_25: return 1;
    case P0_26: return 1;
    case P1_30: return 3;
    case P1_31: return 3;
  }
  return 0xFF;
}

struct pin_info_t {
  constexpr pin_info_t() : has_pwm_hardware{false}, pwm_hardware_index{}, pwm_channel{}, pwm_function_index{}, has_adc_hardware{}, adc_channel{}, adc_function_index{} { }
  constexpr pin_info_t(const size_t index) :
    has_pwm_hardware{calc_pwm_channel(index) != 0xFF},
    pwm_hardware_index{calc_pwm_hardware_index(index)},
    pwm_channel{calc_pwm_channel(index)},
    pwm_function_index{calc_pwm_function_index(index)},
    has_adc_hardware{calc_adc_channel(index) != 0xFF},
    adc_channel{calc_adc_channel(index)},
    adc_function_index{calc_adc_function_index(index)} { }
  bool has_pwm_hardware : 1;
  uint8_t pwm_hardware_index : 1;
  uint8_t pwm_channel : 3;
  uint8_t pwm_function_index : 2;
  bool has_adc_hardware : 1;
  uint8_t adc_channel : 3;
  uint8_t adc_function_index : 2;
};
static_assert(sizeof(pin_info_t) == 2);

struct pin_info_lookup {
private:
  static constexpr auto pin_data = constexpr_generate_array<pin_count>([](const size_t index) { return pin_info_t(index); });
  static constexpr std::array<pin_t, 8> adc_channel_to_pin_table { P0_23, P0_24, P0_25, P0_26, P1_30, P1_31, P0_12, P0_13 };
public:
  static constexpr bool is_valid(const pin_t pin) { return (size_t)pin < pin_data.size(); }
  static constexpr uint8_t has_pwm_hardware(const pin_t pin) { return is_valid(pin) ? pin_data[pin].has_pwm_hardware : 0; }
  static constexpr uint8_t pwm_channel(const pin_t pin) { return is_valid(pin) ? pin_data[pin].pwm_channel : 0; }
  static constexpr uint8_t pwm_function_index(const pin_t pin) { return is_valid(pin) ? pin_data[pin].pwm_function_index : 0; }
  static constexpr uint8_t pwm_hardware_index(const pin_t pin) { return is_valid(pin) ? pin_data[pin].pwm_hardware_index : 0; }
  static constexpr uint8_t has_adc_hardware(const pin_t pin) { return is_valid(pin) ? pin_data[pin].has_adc_hardware : 0; }
  static constexpr uint8_t adc_channel(const pin_t pin) { return is_valid(pin) ? pin_data[pin].adc_channel + 1 : 0; }
  static constexpr uint8_t adc_function_index(const pin_t pin) { return is_valid(pin) ? pin_data[pin].adc_function_index : 0; }
  static constexpr bool is_valid_adc_channel(const uint8_t adc_channel) { return adc_channel < adc_channel_to_pin_table.size(); }
  static constexpr pin_t adc_channel_to_pin_index(const uint8_t adc_channel) { return is_valid_adc_channel(adc_channel) ? adc_channel_to_pin_table[adc_channel] : P_NC; }
};

struct pin_type {
  struct gpio_block {
    uint32_t reg_dir;               // 0x00
    uint32_t reg_unused[3];         // 0x04 0x08 0x0C
    uint32_t reg_mask;              // 0x10
    volatile uint32_t reg_control;  // 0x14
    volatile uint32_t reg_set;      // 0x18
    volatile uint32_t reg_clear;    // 0x1C
  };

  struct iocon_type_analog {
    union {
      struct {
        uint8_t function : 3;
        uint8_t mode : 2;
        uint8_t reserved_5 : 1;
        bool input_polarity : 1;
        bool adc_disabled : 1;
        bool filter_mode : 1;
        uint8_t reserved_9;
        bool open_drain : 1;
        uint8_t reserved_11_15 : 5;
        bool dac_enabled : 1;
      };
      uint32_t value;
    };
  };

  struct iocon_type_digital {
    union {
      struct {
        uint8_t function : 3;
        uint8_t mode : 2;
        bool hysteresis : 1;
        bool input_polarity : 1;
        uint8_t reserved_7_8 : 2;
        bool slew_fast_mode : 1;
        bool open_drain : 1;
      };
      uint32_t value;
    };
  };

  struct iocon_type_w {
    union {
      struct {
        uint8_t function : 3;
        uint8_t mode : 2;
        bool hysteresis : 1;
        bool input_polarity : 1;
        bool adc_enabled : 1;
        bool filter_mode : 1;
        bool slew_fast_mode : 1;
        bool open_drain : 1;
      };
      uint32_t value;
    };
  };

  struct iocon_type_usb {
    union {
      struct {
        uint8_t function : 3;
      };
      uint32_t value;
    };
  };

  struct iocon_type_i2c {
    union {
      struct {
        uint8_t function : 3;
        uint8_t reserved_3_5 : 3;
        bool input_polarity : 1;
        bool fast_mode : 1;
        bool output_drive_sink_20ma : 1;
      };
      uint32_t value;
    };
  };

  static constexpr uint32_t reg_gpio_base = 0x20098000;
  static constexpr uint32_t reg_function_base = 0x4002C000;
  static constexpr uint32_t reg_i2c_cfg = 0x4002C07C;

  [[gnu::always_inline]] constexpr pin_type(const uint8_t port, const uint8_t pin) :
        gpio_reg_id(port),
        gpio_reg_bit(pin),
        gpio_pin(index()) {
  }
  [[gnu::always_inline]] constexpr pin_type(const pin_t pin) :
        gpio_reg_id((pin >> 0x05) & 0x07),
        gpio_reg_bit(pin & 0x1F),
        gpio_pin(index()) {
  }

  [[gnu::always_inline, nodiscard]] constexpr uint8_t port() const {
    return gpio_reg_id;
  }

  [[gnu::always_inline, nodiscard]] constexpr uint8_t bit() const {
    return gpio_reg_bit;
  }

  [[gnu::always_inline, nodiscard]] constexpr pin_t index() const {
    const auto maybe_pin = (gpio_reg_id << 5) | gpio_reg_bit;
    if(pin_info_lookup::is_valid(maybe_pin)) return maybe_pin;
    return -1;
  }

  [[gnu::always_inline, nodiscard]] static constexpr pin_t index_from_adc_channel(uint8_t channel) {
    return pin_info_lookup::adc_channel_to_pin_index(channel);
  }

  /**
   * GPIO Pin
   */
  [[gnu::always_inline]] constexpr uint32_t gpio_address() const {
    return reg_gpio_base + sizeof(gpio_block) * gpio_reg_id;
  }
  [[gnu::always_inline]] inline volatile gpio_block& gpio_reg() const {
    return *reinterpret_cast<volatile gpio_block*>(gpio_address());
  }
  [[gnu::always_inline]] inline void toggle() {
    gpio_reg().reg_control ^= gpio_mask();
  }
  [[gnu::always_inline]] inline void set() {
    gpio_reg().reg_set = gpio_mask();
  }
  [[gnu::always_inline]] inline void set(const bool value) {
    value ? set() : clear();
  }
  [[gnu::always_inline, nodiscard]] inline bool get() const {
    return gpio_reg().reg_control & gpio_mask();
  }
  [[gnu::always_inline]] inline void clear() {
    gpio_reg().reg_clear = gpio_mask();
  }
  [[gnu::always_inline]] inline void direction(const bool direction) {
    gpio_reg().reg_dir = direction ? gpio_reg().reg_dir | gpio_mask() : gpio_reg().reg_dir & ~gpio_mask();
  }
  [[gnu::always_inline]] inline bool direction() const {
    return gpio_reg().reg_dir & gpio_mask();
  }
  [[gnu::always_inline]] inline void input() {
    direction(0);
  }
  [[gnu::always_inline]] inline void output() {
    direction(1);
  }

  /**
   *  GPIO Port
   */
  [[gnu::always_inline]] inline void port_mask(const uint32_t mask) {
    gpio_reg().reg_mask = mask;
  }
  [[gnu::always_inline]] inline uint8_t port_mask() const {
    return gpio_reg().reg_mask;
  }
  [[gnu::always_inline]] inline void port_set(const uint32_t bitset) {
    gpio_reg().reg_set = bitset;
  }
  [[gnu::always_inline]] inline void port_clear(const uint32_t bitset) {
    gpio_reg().reg_clear = bitset;
  }
  [[gnu::always_inline]] inline uint32_t port_get() const {
    return gpio_reg().reg_control;
  }

  /**
   * Function
   */
  [[gnu::always_inline]] constexpr uint32_t function_reg_id() const {
    return (gpio_reg_id * 32) + (gpio_reg_bit);
  }
  [[gnu::always_inline]] constexpr uint32_t function_address() const {
    return reg_function_base + (sizeof(uint32_t) * function_reg_id());
  }
  template <typename iocon_t>
  [[gnu::always_inline]] inline volatile iocon_t& function_reg() const {
    return *reinterpret_cast<volatile iocon_t*>(function_address());
  }
  [[gnu::always_inline]] inline void function(const uint8_t func) {
    function_reg<iocon_type_digital>().function = func;
  }

  [[gnu::always_inline]] inline uint8_t function() const {
    return function_reg<iocon_type_digital>().function;
  }
  [[gnu::always_inline]] inline void enable_adc() {
    if (!has_adc()) return;
    mode(PinMode::PULLUP);
    function_reg<iocon_type_analog>().function = pin_info_lookup::adc_function_index(gpio_pin);
    function_reg<iocon_type_analog>().adc_disabled = false;
  }
  [[gnu::always_inline]] inline void disable_adc() {
    if (!has_adc()) return;
    function_reg<iocon_type_analog>().adc_disabled = true;
    mode(PinMode::PULLDOWN);
    function_reg<iocon_type_analog>().function = 0;
  }
  [[gnu::always_inline]] inline bool adc_enabled() const {
    return has_adc() && function() == pin_info_lookup::adc_function_index(gpio_pin);
  }

  [[gnu::always_inline]] inline void enable_pwm() {
    if (!has_pwm()) return;
    function_reg<iocon_type_digital>().function = pin_info_lookup::pwm_function_index(gpio_pin);
  }
  [[gnu::always_inline]] inline bool pwm_enabled() const {
    return has_pwm() && function() == pin_info_lookup::pwm_function_index(gpio_pin);
  }

  /**
   * Mode
   */
  [[gnu::always_inline]] inline void mode(const PinMode pinmode) {
    function_reg<iocon_type_digital>().mode = pinmode;
  }
  [[gnu::always_inline]] inline PinMode mode() const {
    return static_cast<PinMode>(function_reg<iocon_type_digital>().mode);
  }

  /**
   * OpenDrain Mode
   */
  [[gnu::always_inline]] inline void mode_od(const bool enable) {
    function_reg<iocon_type_digital>().open_drain = enable;
  }
  [[gnu::always_inline]] inline bool mode_od() const {
    return static_cast<PinMode>(function_reg<iocon_type_digital>().mode);
  }

  /**
   * Function compatibility flags
   */
  [[gnu::always_inline, nodiscard]] constexpr bool is_interrupt_capable() const {
    return gpio_reg_id == 0 || gpio_reg_id == 2;
  }
  [[gnu::always_inline, nodiscard]] constexpr bool is_valid() const {
    return valid_pin();
  }

  [[gnu::always_inline, nodiscard]] constexpr bool has_adc() const {
    return pin_info_lookup::has_adc_hardware(gpio_pin);
  }
  [[gnu::always_inline, nodiscard]] constexpr uint8_t get_adc_channel() const {
    return pin_info_lookup::adc_channel(gpio_pin) - 1;
  }

  [[gnu::always_inline, nodiscard]] constexpr bool has_pwm() const {
    return pin_info_lookup::has_pwm_hardware(gpio_pin);
  }
  [[gnu::always_inline, nodiscard]] constexpr uint8_t get_pwm_hardware_index() const {
    return pin_info_lookup::pwm_hardware_index(gpio_pin);
  }
  [[gnu::always_inline, nodiscard]] constexpr uint8_t get_pwm_channel() const {
    return pin_info_lookup::pwm_channel(gpio_pin);
  }
  [[gnu::always_inline, nodiscard]] constexpr uint8_t get_pwm_channel_index() const {
    return pin_info_lookup::pwm_channel(gpio_pin) - 1;
  }

private:
  const uint8_t gpio_reg_id;
  const uint8_t gpio_reg_bit;
  const pin_t gpio_pin;

  [[gnu::always_inline]] constexpr uint32_t gpio_mask() const {
    return 0b1 << gpio_reg_bit;
  }
  [[gnu::always_inline]] constexpr uint8_t function_bit() const {
    return 0;
  }
  [[gnu::always_inline]] constexpr uint32_t function_mask() const {
    return 0b111 << function_bit();
  }

  [[gnu::always_inline]] constexpr bool valid_pin() const {
    return pin_info_lookup::is_valid(gpio_pin);
  }
  [[gnu::always_inline]] constexpr uint8_t adc_channel() const {
    return pin_info_lookup::has_adc_hardware(gpio_pin) ? pin_info_lookup::adc_channel(gpio_pin) : 0;
  }
};

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_port(const pin_t pin) {
  return LPC4078::pin_type{pin}.port();
}

[[gnu::always_inline, nodiscard]] constexpr  uint8_t pin_bit(const pin_t pin) {
  return LPC4078::pin_type{pin}.bit();
}

[[gnu::always_inline, nodiscard]] constexpr pin_t pin_index(const pin_t pin) {
  return LPC4078::pin_type{pin}.index();
}

[[gnu::always_inline, nodiscard]] constexpr bool pin_is_valid(const pin_t pin) {
  return LPC4078::pin_type{pin}.is_valid();
}

[[gnu::always_inline, nodiscard]] constexpr bool pin_has_adc(const pin_t pin) {
  return LPC4078::pin_type{pin}.has_adc();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_get_adc_channel(const pin_t pin) {
  return LPC4078::pin_type{pin}.get_adc_channel();
}

[[gnu::always_inline, nodiscard]] inline bool pin_adc_enabled(const pin_t pin) {
  return LPC4078::pin_type{pin}.adc_enabled();
}

[[gnu::always_inline]] inline void pin_set_mode(const pin_t pin, const PinMode mode) {
  LPC4078::pin_type{pin}.mode(mode);
}

[[gnu::always_inline, nodiscard]] inline PinMode pin_get_mode(const pin_t pin) {
  return LPC4078::pin_type{pin}.mode();
}

[[gnu::always_inline]] inline void pin_enable_adc(const pin_t pin) {
  LPC4078::pin_type{pin}.enable_adc();
}

[[gnu::always_inline]] inline void pin_enable_adc_by_channel(const uint8_t channel) {
  LPC4078::pin_type{LPC4078::pin_type::index_from_adc_channel(channel)}.enable_adc();
}

[[gnu::always_inline, nodiscard]] inline bool pin_pwm_enabled(const pin_t pin) {
  return LPC4078::pin_type{pin}.pwm_enabled();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_has_pwm(const pin_t pin) {
  return LPC4078::pin_type{pin}.has_pwm();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_get_hardware_index(const pin_t pin) {
  return LPC4078::pin_type{pin}.get_pwm_hardware_index();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_get_pwm_channel(const pin_t pin) {
  return LPC4078::pin_type{pin}.get_pwm_channel();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_get_pwm_channel_index(const pin_t pin) {
  return LPC4078::pin_type{pin}.get_pwm_channel_index();
}

[[gnu::always_inline]] inline void pin_enable_pwm(const pin_t pin) {
  LPC4078::pin_type{pin}.enable_pwm();
}

[[gnu::always_inline]] inline void pin_enable_function(const pin_t pin, uint8_t function) {
  LPC4078::pin_type{pin}.function(function);
}
} // LPC4078
