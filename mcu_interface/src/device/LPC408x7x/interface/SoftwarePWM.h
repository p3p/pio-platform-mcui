#pragma once

#include <array>

#include <mcu_core.h>
#include <utility/const_functions.h>
#include <utility/bit_manipulation.h>
#include <utility/registers.h>

#include "system_control.h"
#include "pinmapping.h"
#include "timers.h"

namespace LPC4078 {

static inline volatile auto& sw_pwm_timer = timer3;
constexpr IRQNumber sw_pwm_timer_interrupt_number = IRQNumber::TIMER3;

constexpr uint32_t PWM_MAX_SOFTWARE_CHANNELS = 20;
constexpr uint32_t PWM_MATCH_OFFSET = 25; // in timer cycles (25MHz) 1us

struct SwPwmData{ // 14bytes double linked list node, (16 with packing)
  pin_t pin = P_NC;
  uint32_t value = 0;
  SwPwmData* next = nullptr;
  SwPwmData* prev = nullptr;
};

// Memory constrained insert sorted double linked list with unique identifier
template <std::size_t N>
struct SoftwarePwmTable {
  std::array<SwPwmData, N> swpwm_pin_table {}; //16*N bytes (320bytes @ 20 channels)
  SwPwmData* swpwm_table_head = nullptr;
  SwPwmData* swpwm_table_tail = nullptr;
  SwPwmData* swpwm_table_free = &swpwm_pin_table[0];
  std::size_t length = 0;
  static const std::size_t capacity = N;

  SoftwarePwmTable() {
    for(std::size_t i = 0; i < swpwm_pin_table.size() - 1; ++i) {
      swpwm_pin_table[i].next = &swpwm_pin_table[i + 1];
    }
  }

  constexpr bool exists(const pin_t pin) {
    SwPwmData* it = nullptr;
    while((it = (it == nullptr ? swpwm_table_head : it->next))) {
      if(it->pin == pin) return true;
    }
    return false;
  }

  bool remove(const pin_t pin) {
    SwPwmData* it = nullptr;
    while((it = (it == nullptr ? swpwm_table_head : it->next))) {
      if(it->pin == pin) break;
    }

    if(it != nullptr) {
      MCUCore::nvic_disable_irq(sw_pwm_timer_interrupt_number);
      MCUCore::DSB();
      MCUCore::ISB();
      if(it == swpwm_table_head) swpwm_table_head = swpwm_table_head->next;
      if(it == swpwm_table_tail) swpwm_table_tail = swpwm_table_tail->prev;
      if(it->next != nullptr) it->next->prev = it->prev;
      if(it->prev != nullptr) it->prev->next = it->next;
      it->prev = nullptr;
      it->next = swpwm_table_free;
      swpwm_table_free = it;
      length--;
      if(length) MCUCore::nvic_enable_irq(sw_pwm_timer_interrupt_number);
      return true;
    }
    return false;
  }

  bool update(const pin_t pin, const uint32_t value) {
    SwPwmData* pos = nullptr;
    SwPwmData* cur = nullptr;
    SwPwmData* node = nullptr;

    // find the appropriate position and if it exists the currently used node for the pin
    for (SwPwmData* it = swpwm_table_head; it != nullptr; it = it->next) {
      if(it->pin == pin) cur = it;
      if(value < it->value && pos == nullptr) pos = it;
    }

      // Interupts disabled here
    MCUCore::nvic_disable_irq(sw_pwm_timer_interrupt_number);
    MCUCore::DSB();
    MCUCore::ISB();

    // alocate new node
    if(pos != nullptr && pos == cur) {
      // change in place
      cur->value = value;
      MCUCore::nvic_enable_irq(sw_pwm_timer_interrupt_number);
      return true;
    } else if(cur != nullptr) {
      // decouple current node for reinsertion
      if(cur == swpwm_table_head) swpwm_table_head = swpwm_table_head->next;
      if(cur == swpwm_table_tail) swpwm_table_tail = swpwm_table_tail->prev;
      if(cur->next != nullptr) cur->next->prev = cur->prev;
      if(cur->prev != nullptr) cur->prev->next = cur->next;
      node = cur;
    } else {
      // allocate new node from reserve
      node = swpwm_table_free;
      if(node == nullptr) {
        MCUCore::nvic_enable_irq(sw_pwm_timer_interrupt_number);
        return false; // out of reserved memory
      }
      swpwm_table_free = node->next;
      length++;
    }

    // update node data
    node->pin = pin;
    node->value = value;

    if(pos == nullptr) {
      if(swpwm_table_head == nullptr) {
        // initialise head
        swpwm_table_head = node;
        swpwm_table_tail = node;
        node->next = nullptr;
        node->prev = nullptr;
      } else {
        // insert at tail
        swpwm_table_tail->next = node;
        node->prev = swpwm_table_tail;
        node->next = nullptr;
        swpwm_table_tail = node;
      }
    } else if(pos->prev == nullptr) {
      // insert at head
      node->next = swpwm_table_head;
      node->prev = nullptr;
      swpwm_table_head->prev = node;
      swpwm_table_head = node;
    } else {
      // insert in middle
      node->prev = pos->prev;
      node->next = pos;
      node->prev->next = node;
      pos->prev = node;
    }
    // Interupts enabled
    MCUCore::nvic_enable_irq(sw_pwm_timer_interrupt_number);
    return true;
  }
};

class SoftwarePWM {
public:
  static void init(const uint32_t frequency, const uint32_t int_priority = 2) {
    // Setup timer for Timer3 Interrupt controlled PWM
    peripheral_power_on(PeripheralPowerControl::TIM3);

    sw_pwm_timer.PR = 0; // no prescaler
    sw_pwm_timer.MCR = MCUI::util::bitset_value(0, 1); // Interrupt on MR0, reset on MR0
    sw_pwm_timer.MR0 = (PeripheralClock / frequency) - 1; // set frequency
    sw_pwm_timer.TCR = MCUI::util::bit_value(0);       // enable the timer

    MCUCore::nvic_set_priority(sw_pwm_timer_interrupt_number, MCUCore::nvic_encode_priority(0, int_priority, 0));
  }

  static void set_frequency(const uint32_t frequency){
    set_period(PeripheralClock / frequency);
  }

  static void set_period(const uint32_t period) {
    uint32_t old_period = sw_pwm_timer.MR0;
    sw_pwm_timer.MR0 = period - 1;
    sw_pwm_timer.TC = MCUI::util::map(sw_pwm_timer.TC, 0, old_period, 0, sw_pwm_timer.MR0);
    if (sw_pwm_timer.TC > sw_pwm_timer.MR0) {
      sw_pwm_timer.TC = sw_pwm_timer.MR0 - 1;
    }
  }

  static uint32_t get_period() {
    return sw_pwm_timer.MR0 + 1;
  }

  static uint32_t size() {
    return data_table.length;
  }

  static SwPwmData* data() {
    return data_table.swpwm_table_head;
  }

  [[nodiscard]] static bool available(const pin_t pin) {
    return data_table.swpwm_table_free != nullptr;
  }

  [[nodiscard]] static bool active(const pin_t pin) {
    return data_table.exists(pin);
  }

  static void set_us(const pin_t pin, const uint32_t value) {
    set_match(pin, (PeripheralClock / 1000000) * value);
  }

  static void set_match(const pin_t pin, const uint32_t value) {
    data_table.update(pin, value);
  }

  static bool attach(const pin_t pin, const uint32_t value) {
    if (data_table.update(pin, value)) {
      gpio_set_output(pin);
      gpio_clear(pin);
      pin_enable_function(pin, LPC4078::Function::GPIO);            // initialise pin for gpio output
      return true;
    }
    return false;
  }

  static bool detach(const pin_t pin) {
    return data_table.remove(pin);
  }

private:
  static SoftwarePwmTable<PWM_MAX_SOFTWARE_CHANNELS> data_table;
};

} // LPC4078
