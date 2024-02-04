#include <array>

#include "time.h"
#include <utility/const_functions.h>
#include "pinmapping.h"
#include "HardwarePWM.h"
#include "SoftwarePWM.h"
#include "pwm.h"

namespace LPC4078 {

void pwm_init(const uint32_t frequency) {
  // Period defaulted to 20ms (50Hz) for compatibility with servos
  HardwarePWM::init(pwm_device0, frequency);
  HardwarePWM::init(pwm_device1, frequency);
  SoftwarePWM::init(frequency);
}

bool pwm_attach_pin(const pin_t pin, const uint32_t value) {
  // Hardware PWM
  if(HardwarePWM::active(pin)) return true;   // already attached to hardware channel?
  if(HardwarePWM::attach(pin, value)) {       // hardware capable and channel required by pin not in use,
    return true;                              // attach successfully so return
  }

  // Fall back on Timer3 based PWM
  if(SoftwarePWM::active(pin)) return true;   // already attached on software pin
  return SoftwarePWM::attach(pin, value);     // last chance
}

bool pwm_attached(const pin_t pin) {
  return HardwarePWM::active(pin) || SoftwarePWM::active(pin);
}

bool pwm_detach_pin(const pin_t pin) {
  // Hardware PWM capable pin and active
  if (HardwarePWM::detach(pin)) return true;
  // Fall back on Timer3 based PWM
  return SoftwarePWM::detach(pin);
}

bool pwm_write(const pin_t pin, const uint32_t value) {

  if (HardwarePWM::active(pin)) {
    HardwarePWM::set_match(pin, value);
    return true;
  }

  if (SoftwarePWM::active(pin)) {
    SoftwarePWM::set_match(pin, value);
    return true;
  }

  return false;
}

uint32_t pwm_get_period(const pin_t pin) {
  if (HardwarePWM::active(pin)) {
    return HardwarePWM::get_period(pin);
  }

  if (SoftwarePWM::active(pin)) {
    return SoftwarePWM::get_period();
  }

  return 0;
}

bool pwm_write_ratio(const pin_t pin, const uint8_t value) {
  return pwm_write(pin, MCUI::util::map(value, 0, 255, 0, pwm_get_period(pin)));
}

bool pwm_write_ratio(const pin_t pin, const float value) {
  return pwm_write(pin, static_cast<float>(pwm_get_period(pin)) * (value > 1.0f ? 1.0 : (value < 0.0f ? 0.0f : value)));
}

bool pwm_write_us(const pin_t pin, const uint32_t value) {
  if (HardwarePWM::active(pin)) {
    HardwarePWM::set_us(pin, value);
    return true;
  }

  if (SoftwarePWM::active(pin)) {
    SoftwarePWM::set_us(pin, value);
    return true;
  }

  return false;
}

bool pwm_set_frequency(const pin_t pin, const uint32_t frequency) {
  if (!pwm_attached(pin)) {
    pwm_attach_pin(pin);
  }

  if (HardwarePWM::active(pin)) {
    HardwarePWM::set_frequency(pin, frequency);
    return true;
  }

  if (SoftwarePWM::active(pin)) {
    SoftwarePWM::set_frequency(frequency);
    return true;
  }

  return false;
}
} // LPC4078
