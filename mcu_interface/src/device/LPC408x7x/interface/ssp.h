#pragma once

#include <cstdint>
using std::size_t;

#include <limits>
#include <cstdio>

#include <utility/const_functions.h>
#include <utility/registers.h>
#include <mcu_core.h>
#include "system_control.h"

namespace LPC4078 {
using namespace MCUI::util::registers;

// UM10562
// Chapter 21: LPC408x/407x SSP interfaces
// 21.6 Register description
struct SSPRegion {
  struct {
    union {
      reg32_t<uint8_t, 0, 4> DSS;  // Data Size Select
      reg32_t<uint8_t, 4, 2> FRF;  // Frame Format
      reg32_t<uint8_t, 6, 1> CPOL; // Clock Out Polarity
      reg32_t<uint8_t, 7, 1> CPHA; // Clock Out Phase
      reg32_t<uint8_t, 8, 8> SCR;  // Serial Clock Rate
    };
  } CR0; // 0x000, RW, Control Register 0
  struct {
    union {
      reg32_t<uint8_t, 0, 1> LBM; // Loop Back Mode
      reg32_t<uint8_t, 1, 1> SSE; // SSP Enable
      reg32_t<uint8_t, 2, 1> MS;  // Master/Slave Mode
      reg32_t<uint8_t, 3, 1> SOD; // Slave Output Disable
    };
  } CR1; // 0x004, RW, Control Register 1
  struct {
    reg32_fifo_t<uint16_t, 0, 16> DATA;
  } DR; // 0x008, RW, Data Register
  struct {
    union {
      reg32_t<bool, 0, 1> TFE; // Transmit FIFO Empty
      reg32_t<bool, 1, 1> TNF; // Transmit FIFO Not Full
      reg32_t<bool, 2, 1> RNE; // Receive FIFO Not Empty
      reg32_t<bool, 3, 1> RFF; // Receive FIFO Full
      reg32_t<bool, 4, 1> BSY; // Busy
    };
  } SR; // 0x00C, RO, Status Register
  struct {
    reg32_t<uint8_t, 0, 8> CPSDVSR; // This even value between 2 and 254, by which PCLK is divided to yield the prescaler output cloc
  } CPSR;                           // 0x010, RW, Clock Prescale Register
  struct {
    union {
      reg32_t<bool, 0, 1> RORIM; // Receive Overrun Interrupt enable
      reg32_t<bool, 1, 1> RTIM;  // Receive Timeout Interrupt enable
      reg32_t<bool, 2, 1> RXIM;  // Rx FIFO at least half full interrupt enable
      reg32_t<bool, 3, 1> TXIM;  // Tx FIFO at least half empty interrupt
    };
  } IMSC; // 0x014, RW, Interrupt Mask Set and Clear Register
  struct {
    union {
      reg32_t<bool, 0, 1> RORRIS;
      reg32_t<bool, 1, 1> RTRIS;
      reg32_t<bool, 2, 1> RXRIS;
      reg32_t<bool, 3, 1> TXRIS;
    };
  } RIS; // 0x018, RW, Raw Interrupt Status Register
  struct {
    union {
      reg32_t<bool, 0, 1> RORMIS;
      reg32_t<bool, 1, 1> RTMIS;
      reg32_t<bool, 2, 1> RXMIS;
      reg32_t<bool, 3, 1> TXMIS;
    };
  } MIS; // 0x01C, RW, Masked Interrupt Status Register
  struct {
    union {
      reg32_t<bool, 0, 1> RORIC;
      reg32_t<bool, 1, 1> RTIC;
    };
  } ICR; // 0x020, RW, SSPICR Interrupt Clear Register
  struct {
    union {
      reg32_t<bool, 0, 1> RXDMAE; // Receive DMA Enable
      reg32_t<bool, 1, 1> TXDMAE; // Transmit DMA Enable
    };
  } DMACR; // 0x024, RW, DMA Control Register
};
static_assert(sizeof(SSPRegion) == 0x28);

constexpr size_t ssp_device_count = 3;
constexpr uintptr_t ssp_address[ssp_device_count] = { 0x40088000, 0x40030000, 0x400AC000 };
static inline volatile SSPRegion* const ssp_device[ssp_device_count] = {
  reinterpret_cast<volatile SSPRegion*>(ssp_address[0]),
  reinterpret_cast<volatile SSPRegion*>(ssp_address[1]),
  reinterpret_cast<volatile SSPRegion*>(ssp_address[2])
};
constexpr PeripheralPowerControl ssp_power_control_lookup[ssp_device_count] = {
  PeripheralPowerControl::SSP0,
  PeripheralPowerControl::SSP1,
  PeripheralPowerControl::SSP2
};

struct ssp_pin_fn {
  pin_t pin = P_NC;
  uint8_t function = 0;
};

constexpr std::array<std::array<ssp_pin_fn, 4>, 3> ssp_sck_pin_lookup = {
  std::array<ssp_pin_fn, 4> {ssp_pin_fn{P0_15, 2}, ssp_pin_fn{P1_20, 5}, ssp_pin_fn{P2_22, 2}},
                            {ssp_pin_fn{P0_07, 2}, ssp_pin_fn{P1_19, 5}, ssp_pin_fn{P1_31, 2}, ssp_pin_fn{P4_20, 3}},
                            {ssp_pin_fn{P1_00, 4}, ssp_pin_fn{P5_02, 2}}
};
constexpr std::array<std::array<ssp_pin_fn, 4>, 3> ssp_ssel_pin_lookup = {
  std::array<ssp_pin_fn, 4> {ssp_pin_fn{P0_16, 2}, ssp_pin_fn{P1_21, 3}, ssp_pin_fn{P1_28, 5}, ssp_pin_fn{P2_23, 2}},
                            {ssp_pin_fn{P0_06, 2}, ssp_pin_fn{P0_14, 2}, ssp_pin_fn{P1_26, 5}, ssp_pin_fn{P4_21, 3}},
                            {ssp_pin_fn{P1_08, 4}, ssp_pin_fn{P5_03, 2}}
};
constexpr std::array<std::array<ssp_pin_fn, 4>, 3> ssp_miso_pin_lookup = {
  std::array<ssp_pin_fn, 4> {ssp_pin_fn{P0_17, 2}, ssp_pin_fn{P1_23, 5}, ssp_pin_fn{P2_26, 2}},
                            {ssp_pin_fn{P0_08, 2}, ssp_pin_fn{P0_12, 2}, ssp_pin_fn{P1_18, 5}, ssp_pin_fn{P4_22, 3}},
                            {ssp_pin_fn{P1_04, 4}, ssp_pin_fn{P5_01, 2}}
};
constexpr std::array<std::array<ssp_pin_fn, 4>, 3> ssp_mosi_pin_lookup = {
  std::array<ssp_pin_fn, 4> {ssp_pin_fn{P0_18, 2}, ssp_pin_fn{P1_24, 5}, ssp_pin_fn{P2_27, 2}},
                            {ssp_pin_fn{P0_09, 2}, ssp_pin_fn{P0_13, 2}, ssp_pin_fn{P1_22, 5}, ssp_pin_fn{P4_23, 3}},
                            {ssp_pin_fn{P1_01, 4}, ssp_pin_fn{P5_00, 2}}
};


namespace SSP {

struct Config {
  enum class Format : uint8_t {
    SPI, TI, Microwire
  };
  struct Mode {
    constexpr Mode(uint8_t mode) : value{mode} {}
    union {
      struct {
        uint8_t phase : 1;
        uint8_t polarity : 1;
      };
      uint8_t value : 2;
    };
  };
  size_t frequency = 10000000;
  Mode mode = 0;
  Format format = Format::SPI;
  uint8_t data_bits = 8;
  pin_t pin_sck = P_NC;
  pin_t pin_ssel = P_NC;
  pin_t pin_miso = P_NC;
  pin_t pin_mosi = P_NC;
};

static std::array<ssp_pin_fn, 4> pin_function_lookup(const size_t ssp_id, const pin_t pin_sck, const pin_t pin_ssel, const pin_t pin_miso, const pin_t pin_mosi) {
  auto lookup_sck_fn  = [pin_sck](ssp_pin_fn it){ return it.pin == pin_sck; };
  auto lookup_ssel_fn = [pin_ssel](ssp_pin_fn it){ return it.pin == pin_ssel; };
  auto lookup_miso_fn = [pin_miso](ssp_pin_fn it){ return it.pin == pin_miso; };
  auto lookup_mosi_fn = [pin_mosi](ssp_pin_fn it){ return it.pin == pin_mosi; };

  auto& sck_pins  = ssp_sck_pin_lookup[ssp_id];
  auto& ssel_pins = ssp_ssel_pin_lookup[ssp_id];
  auto& miso_pins = ssp_miso_pin_lookup[ssp_id];
  auto& mosi_pins = ssp_mosi_pin_lookup[ssp_id];

  auto sck_fn_it  = std::find_if(sck_pins.begin(),  sck_pins.end(),  lookup_sck_fn);
  auto ssel_fn_it = std::find_if(ssel_pins.begin(), ssel_pins.end(), lookup_ssel_fn);
  auto miso_fn_it = std::find_if(miso_pins.begin(), miso_pins.end(), lookup_miso_fn);
  auto mosi_fn_it = std::find_if(mosi_pins.begin(), mosi_pins.end(), lookup_mosi_fn);

  ssp_pin_fn sck_fn  = sck_fn_it == sck_pins.end() ? ssp_pin_fn{P_NC, 0}: *sck_fn_it;
  ssp_pin_fn ssel_fn = ssel_fn_it == ssel_pins.end() ? ssp_pin_fn{P_NC, 0}: *ssel_fn_it;
  ssp_pin_fn miso_fn = miso_fn_it == miso_pins.end() ? ssp_pin_fn{P_NC, 0}: *miso_fn_it;
  ssp_pin_fn mosi_fn = mosi_fn_it == mosi_pins.end() ? ssp_pin_fn{P_NC, 0}: *mosi_fn_it;

  return {sck_fn, ssel_fn, miso_fn, mosi_fn};
}

[[gnu::always_inline]] static inline void power_on(const size_t ssp_id) {
  peripheral_power_on(ssp_power_control_lookup[ssp_id]);
  MCUCore::ISB();
}

[[gnu::always_inline]] static inline void power_off(const size_t ssp_id) {
  MCUCore::ISB();
  peripheral_power_off(ssp_power_control_lookup[ssp_id]);
}

struct FrequencyDescriptor {
  uint8_t base_divisor;
  uint8_t divisor;
};

static constexpr FrequencyDescriptor find_divisors(const size_t frequency) {
  size_t best_error = std::numeric_limits<size_t>::max();
  uint8_t best_base_divisor = 2;
  uint8_t best_divisor = 0;

  const size_t base_freq = PeripheralClock ;

  for (size_t base_divisor = 2; base_divisor < 0xFF; base_divisor += 2) {
    size_t div_freq = base_divisor * frequency;

    // round to closest
    size_t half_div_freq = div_freq / 2;
    uint8_t divisor = (base_freq / div_freq) + (base_freq % div_freq > half_div_freq);

    size_t error = MCUI::util::abs(static_cast<int32_t>(frequency) - static_cast<int32_t>(base_freq / (base_divisor * divisor)));
    if (error < best_error) {
      best_error = error;
      best_base_divisor = base_divisor;
      best_divisor = divisor;
      if (best_error == 0) break;
    }
  }

  return {best_base_divisor, best_divisor};
}

[[gnu::always_inline]] static inline size_t frequency(const size_t ssp_id) {
  auto& ssp = *ssp_device[ssp_id];
  return PeripheralClock / (ssp.CPSR.CPSDVSR * (ssp.CR0.SCR + 1));
}

[[gnu::always_inline]] static inline void frequency(const size_t ssp_id, const size_t freq) {
  auto& ssp = *ssp_device[ssp_id];
  auto divisors = find_divisors(freq);
  ssp.CPSR.CPSDVSR = divisors.base_divisor;
  ssp.CR0.SCR = (divisors.divisor) - 1;
  // printf("SSP%d: Frequency Requested: %d, Actual: %d\n", ssp_id, freq, frequency(ssp_id));
  return;
}


[[gnu::always_inline]] static inline void init(const size_t ssp_id, const Config config) {
  auto& ssp = *ssp_device[ssp_id];

  power_on(ssp_id);

  ssp.CR0.CPHA = config.mode.phase;
  ssp.CR0.CPOL = config.mode.polarity;
  ssp.CR0.FRF = static_cast<uint8_t>(config.format);
  ssp.CR0.DSS = (config.data_bits - 1) & 0xF;

  frequency(ssp_id, config.frequency);

  ssp.CR1.MS = 0;  // assume master mode atm

  for (auto pin : pin_function_lookup(ssp_id, config.pin_sck, config.pin_ssel, config.pin_miso, config.pin_mosi)) {
    // if (pin.pin == P_NC) printf("SSP%d: Not available on the requested pin\n", ssp_id);
    pin_enable_function(pin.pin, pin.function);
  }

  ssp.CR1.SSE = 1; // enable the device on the bus
}

[[gnu::always_inline]] static inline void configure(const size_t ssp_id, const Config config) {
  auto& ssp = *ssp_device[ssp_id];

  ssp.CR1.SSE = 0; // disable the device on the bus

  ssp.CR0.CPHA = config.mode.phase;
  ssp.CR0.CPOL = config.mode.polarity;
  ssp.CR0.FRF = static_cast<uint8_t>(config.format);
  ssp.CR0.DSS = (config.data_bits - 1) & 0xF;

  frequency(ssp_id, config.frequency);

  ssp.CR1.MS = 0;  // assume master mode atm

  ssp.CR1.SSE = 1; // enable the device on the bus
}

[[gnu::always_inline]] static inline size_t rx_available(const size_t ssp_id) {
  auto& ssp = *ssp_device[ssp_id];
  return ssp.SR.RNE;
}

[[gnu::always_inline]] static inline size_t tx_available(const size_t ssp_id) {
  auto& ssp = *ssp_device[ssp_id];
  return ssp.SR.TNF;
}

[[gnu::always_inline]] static inline size_t busy(const size_t ssp_id) {
  auto& ssp = *ssp_device[ssp_id];
  return ssp.SR.BSY;
}

[[gnu::always_inline]] static inline size_t write(const size_t ssp_id, uint16_t data) {
  auto& ssp = *ssp_device[ssp_id];
  if (ssp.SR.TNF) {
    ssp.DR.DATA = data;
    return 1;
  }
  return 0;
}

[[gnu::always_inline]] static inline size_t read(const size_t ssp_id, uint16_t* data) {
  auto& ssp = *ssp_device[ssp_id];
  if (ssp.SR.RNE) {
    *data = ssp.DR.DATA;
    return 1;
  }
  return 0;
}

} // namespace SSP
} // namespace LPC4078
