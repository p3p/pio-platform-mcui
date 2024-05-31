#pragma once

#include "pinmapping.h"

namespace LPC4078 {

struct PulseWidthModulation {
  uint32_t IR;            // Offset: 0x000 Interrupt Register (R/W)
  uint32_t TCR;           // Offset: 0x004 Timer Control Register (R/W)
  uint32_t TC;            // Offset: 0x008 Timer Counter Register (R/W)
  uint32_t PR;            // Offset: 0x00C Prescale Register (R/W)
  uint32_t PC;            // Offset: 0x010 Prescale Counter Register (R/W)
  uint32_t MCR;           // Offset: 0x014 Match Control Register (R/W)
  uint32_t MR0;           // Offset: 0x018 Match Register 0 (R/W)
  uint32_t MR1;           // Offset: 0x01C Match Register 1 (R/W)
  uint32_t MR2;           // Offset: 0x020 Match Register 2 (R/W)
  uint32_t MR3;           // Offset: 0x024 Match Register 3 (R/W)
  uint32_t CCR;           // Offset: 0x028 Capture Control Register (R/W)
  uint32_t CR0;           // Offset: 0x02C Capture Register 0 (R/ )
  uint32_t CR1;					  // Offset: 0x030 Capture Register 1 (R/ )
  uint32_t CR2;					  // Offset: 0x034 Capture Register 2 (R/ )
  uint32_t CR3;					  // Offset: 0x038 Capture Register 3 (R/ )
	uint32_t RESERVED0;
  uint32_t MR4;					  // Offset: 0x040 Match Register 4 (R/W)
  uint32_t MR5;					  // Offset: 0x044 Match Register 5 (R/W)
  uint32_t MR6;					  // Offset: 0x048 Match Register 6 (R/W)
  uint32_t PCR;					  // Offset: 0x04C PWM Control Register (R/W)
  uint32_t LER;					  // Offset: 0x050 Load Enable Register (R/W)
	uint32_t RESERVED1[7];
  uint32_t CTCR;					// Offset: 0x070 Counter Control Register (R/W)
};
static_assert(sizeof(PulseWidthModulation) == 0x074);

static constexpr uint32_t pwm0_hardware_address = 0x40014000;
static constexpr uint32_t pwm1_hardware_address = 0x40018000;
static inline volatile PulseWidthModulation* const pwm_device0 = reinterpret_cast<volatile PulseWidthModulation*>(pwm0_hardware_address);
static inline volatile PulseWidthModulation* const pwm_device1 = reinterpret_cast<volatile PulseWidthModulation*>(pwm1_hardware_address);

void pwm_init(const uint32_t frequency = 50);
bool pwm_attach_pin(const pin_t pin, const uint32_t value = 0);
bool pwm_write(const pin_t pin, const uint32_t value);
bool pwm_write_ratio(const pin_t pin, const uint8_t value);
bool pwm_write_ratio(const pin_t pin, const float value);
bool pwm_write_us(const pin_t pin, const uint32_t value);
bool pwm_detach_pin(const pin_t pin);

uint32_t pwm_get_period(const pin_t pin);
bool pwm_set_frequency(const pin_t pin, const uint32_t frequency);

} // LPC4078
