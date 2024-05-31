#pragma once

#include <cstdint>
#include <array>
#include <algorithm>
#include <functional>

#include <utility/const_functions.h>
#include <utility/ring_buffer.h>

#include "gpio.h"
#include "clock_control.h"
#include "system_control.h"


namespace LPC4078 {

  namespace UARTRegister {
    struct RBR {
      uint8_t value;
      uint8_t reserved[3];
    };

  }

  struct UARTRegisterBase {
    union { // 0x000
      union {
        uint32_t RBR; // DLAB = 0. Receiver Buffer Register. Contains the next received character to be read.
        uint32_t THR; // DLAB = 0. Transmit Holding Register. The next character to be transmitted is written here.
      };
      uint32_t DLL;   // DLAB = 1. Divisor Latch LSB. Least significant byte of the baud
                      //          rate divisor value. The full divisor is used to generate a baud
                      //          rate from the fractional rate divider.
    };
    union { // 0x004
      uint32_t DLM;   // DLAB = 1. Divisor Latch MSB. Most significant byte of the baud
                      //          rate divisor value. The full divisor is used to generate a baud
                      //          rate from the fractional rate divider.
      uint32_t IER;   // DLAB = 0. Interrupt Enable Register. Contains individual
                      //          interrupt enable bits for the 7 potential UART1 interrupts.
    };
    union { // 0x008
      uint32_t IIR;   // Interrupt ID Register. Identifies which interrupt(s) are pending.
      uint32_t FCR;   // FIFO Control Register. Controls UART1 FIFO usage and modes.
    };
    uint32_t LCR;         // 0x00C Line Control Register. Contains controls for frame formatting and break generation.
  };

  // Shared registers between all UART peripherals
  struct UARTRegisterBaseExt : public UARTRegisterBase {
    uint32_t SPEC0;       // 0x010
    uint32_t LSR;         // 0x014 Line Status Register. Contains flags for transmit and receive status, including line errors.
    uint32_t SPEC1;       // 0x018
    uint32_t SCR;         // 0x01C Scratch Pad Register. 8-bit temporary storage for software.
    uint32_t ACR;         // 0x020 Auto-baud Control Register. Contains controls for the auto-baud feature.
    uint32_t SPEC2;       // 0x024
    uint32_t FDR;         // 0x028 Fractional Divider Register. Generates a clock input for the baud rate divider.
    uint32_t SPEC3;       // 0x02C
    uint32_t TER;         // 0x030
    uint32_t SPEC5[6];    // 0x034
    uint32_t RS485CTRL;   // 0x04C RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes.
    uint32_t RSADRMATCH;  // 0x050 RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode.
    uint32_t RS485DLY;    // 0x054 RS-485/EIA-485 direction control delay.
  };
  static_assert(sizeof(UARTRegisterBaseExt) == 0x58);

  // Table 369: Register overview: UART1 (base address 0x4001 0000)
  struct UART1Register : public UARTRegisterBase {
    uint32_t MCR;         // 0x010 Modem Control Register. Contains controls for flow control handshaking and loopback mode.
    uint32_t LSR;         // 0x014 Line Status Register. Contains flags for transmit and receive status, including line errors.
    uint32_t MSR;         // 0x018 Modem Status Register. Contains handshake signal status flags.
    uint32_t SCR;         // 0x01C Scratch Pad Register. 8-bit temporary storage for software.
    uint32_t ACR;         // 0x020 Auto-baud Control Register. Contains controls for the auto-baud feature.
    uint32_t RESERVED0;   // 0x024
    uint32_t FDR;         // 0x028 Fractional Divider Register. Generates a clock input for the baud rate divider.
    uint32_t RESERVED1;   // 0x02C
    uint32_t TER;         // 0x030 Transmit Enable Register. Turns off UART transmitter for use with software flow control.
    uint32_t RESERVED[6]; // 0x034
    uint32_t RS485CTRL;   // 0x04C RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes.
    uint32_t RSADRMATCH;  // 0x050 RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode.
    uint32_t RS485DLY;    // 0x054 RS-485/EIA-485 direction control delay.
  };
  static_assert(sizeof(UART1Register) == 0x58);

  // Table 392. Register overview: UART0/2/3 (base address: 0x4000 C000, 0x4008 8000, 0x4009 C000)
  struct UART023Register : public UARTRegisterBase {
    uint32_t RESERVED0;   // 0x010
    uint32_t LSR;         // 0x014 Line Status Register. Contains flags for transmit and receive status, including line errors.
    uint32_t RESERVED1;   // 0x018
    uint32_t SCR;         // 0x01C Scratch Pad Register. 8-bit temporary storage for software.
    uint32_t ACR;         // 0x020 Auto-baud Control Register. Contains controls for the auto-baud feature.
    uint32_t RESERVED2;   // 0x024
    uint32_t FDR;         // 0x028 Fractional Divider Register. Generates a clock input for the baud rate divider.
    uint32_t RESERVED3;   // 0x02C
    uint32_t TER;         // 0x030 Transmit Enable Register. Turns off UART transmitter for use with software flow control.
    uint32_t RESERVED[6]; // 0x034
    uint32_t RS485CTRL;   // 0x04C RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes.
    uint32_t RSADRMATCH;  // 0x050 RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode.
    uint32_t RS485DLY;    // 0x054 RS-485/EIA-485 direction control delay.
  };
  static_assert(sizeof(UART023Register) == 0x58);

  // Table 412. Register overview: UART4 (base address: 0x400A 4000)
  struct UART4Register : public UARTRegisterBase {
    uint32_t RESERVED0;   // 0x010
    uint32_t LSR;         // 0x014 Line Status Register. Contains flags for transmit and receive status, including line errors.
    uint32_t RESERVED1;   // 0x018
    uint32_t SCR;         // 0x01C Scratch Pad Register. 8-bit temporary storage for software.
    uint32_t ACR;         // 0x020 Auto-baud Control Register. Contains controls for the auto-baud feature.
    uint32_t ICR;         // 0x024 IrDA Control Register. Enables and configures the IrDA mode.
    uint32_t FDR;         // 0x028 Fractional Divider Register. Generates a clock input for the baud rate divider.
    uint32_t OSR;         // 0x02C Oversampling register. Controls the degree of oversampling during each bit time.
    uint32_t RESERVED[6]; // 0x030
    uint32_t SCICTRL;     // 0x048 Smart card Interface control register. Enables and configures the smart card Interface feature.
    uint32_t RS485CTRL;   // 0x04C RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes.
    uint32_t RSADRMATCH;  // 0x050 RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode.
    uint32_t RS485DLY;    // 0x054 RS-485/EIA-485 direction control delay.
    uint32_t SYNCCTRL;    // 0x058 Synchronous mode control register.
  };
  static_assert(sizeof(UART4Register) == 0x5C);

  static constexpr std::array<uintptr_t, 5> uart_address = { 0x4000C000, 0x40010000, 0x40088000, 0x4009C000, 0x400A4000 };
  static inline const std::array<volatile UARTRegisterBaseExt * const, 5> uart_base = { reinterpret_cast<volatile UARTRegisterBaseExt*>(uart_address[0]),
                                                                                        reinterpret_cast<volatile UARTRegisterBaseExt*>(uart_address[1]),
                                                                                        reinterpret_cast<volatile UARTRegisterBaseExt*>(uart_address[2]),
                                                                                        reinterpret_cast<volatile UARTRegisterBaseExt*>(uart_address[3]),
                                                                                        reinterpret_cast<volatile UARTRegisterBaseExt*>(uart_address[4])
  };

  constexpr auto& uart_lookup_by_id(const size_t uart_id) { return *uart_base[uart_id]; }
  constexpr std::array<PeripheralPowerControl, 5> uart_power_control_bit = { PeripheralPowerControl::UART0, PeripheralPowerControl::UART1, PeripheralPowerControl::UART2, PeripheralPowerControl::UART3, PeripheralPowerControl::UART4 };
  constexpr std::array<IRQNumber, 5> uart_irq_number = { IRQNumber::UART0, IRQNumber::UART1, IRQNumber::UART2, IRQNumber::UART3, IRQNumber::UART4 };

  struct baud_fraction {
    uint16_t fraction;
    uint8_t div_add_val;
    uint8_t mul_val;
  };
  constexpr std::array<const baud_fraction, 73> baud_fraction_lookup = { baud_fraction{0, 0, 15}, {66, 1, 15}, {71, 1, 14}, {76, 1, 13}, {83, 1, 12}, {90, 1, 11}, {100, 1, 10}, {111, 1, 9}, {125, 1, 8}, {133, 2, 15}, {142, 2, 14}, {153, 2, 13}, {166, 2, 12}, {181, 2, 11}, {200, 3, 15}, {214, 3, 14}, {222, 2, 9}, {230, 3, 13}, {250, 3, 12}, {266, 4, 15}, {272, 3, 11}, {285, 4, 14}, {300, 3, 10}, {307, 4, 13}, {333, 5, 15}, {357, 5, 14}, {363, 4, 11}, {375, 3, 8}, {384, 5, 13}, {400, 6, 15}, {416, 5, 12}, {428, 6, 14}, {444, 4, 9}, {454, 5, 11}, {461, 6, 13}, {466, 7, 15}, {500, 7, 14}, {533, 8, 15}, {538, 7, 13}, {545, 6, 11}, {555, 5, 9}, {571, 8, 14}, {583, 7, 12}, {600, 9, 15}, {615, 8, 13}, {625, 5, 8}, {636, 7, 11}, {642, 9, 14}, {666, 10, 15}, {692, 9, 13}, {700, 7, 10}, {714, 10, 14}, {727, 8, 11}, {733, 11, 15}, {750, 9, 12}, {769, 10, 13}, {777, 7, 9}, {785, 11, 14}, {800, 12, 15}, {818, 9, 11}, {833, 10, 12}, {846, 11, 13}, {857, 12, 14}, {866, 13, 15}, {875, 7, 8}, {888, 8, 9}, {900, 9, 10}, {909, 10, 11}, {916, 11, 12}, {923, 12, 13}, {928, 13, 14}, {933, 14, 15}, {1000, 14, 14}};

  struct baud_divisors {
    uint32_t divisor;
    uint8_t div_add_val;
    uint8_t mul_val;
  };

  template <typename IT>
  [[gnu::always_inline]] constexpr auto closest(IT a, IT b, uint16_t target) {
    return target - a->fraction >= b->fraction - target ? b : a;
  }

  template <typename Container>
  constexpr auto binary_search_closest(Container& v, uint16_t target) {
    if (v.empty()) return v.end();
    if (v.begin()->fraction > target) return v.begin();
    else if ((v.end() - 1)->fraction < target) return v.end() - 1;

    size_t offset_left = 0;
    size_t offset_right = v.size();
    size_t offset_mid = 0;

    while (offset_left < offset_right) {
      offset_mid = (offset_left + offset_right) / 2;
      if (target == v[offset_mid].fraction) break;
      if (target < v[offset_mid].fraction) {
        if (target > v[offset_mid - 1].fraction)
          return closest(v.begin() + (offset_mid - 1), v.begin() + offset_mid, target);
        offset_right = offset_mid;
      } else {
        if (target < v[offset_mid + 1].fraction)
          return closest(v.begin() + offset_mid, v.begin() + (offset_mid + 1), target);
        offset_left = offset_mid + 1;
      }

    }

    return v.begin() + offset_mid;
  }

  constexpr baud_divisors find_best_divisors_for_available_fractions(const uint32_t baud, const uint32_t start_divisor, const uint32_t max_divisor) {
    uint32_t baud_error = 60'000'000 / (16 * start_divisor);
    uint32_t error_fract = ((baud_error * 1000) / baud) - 1000;
    auto fract = binary_search_closest(baud_fraction_lookup, error_fract);
    auto best_fract = fract;
    int32_t best_fract_error = error_fract - fract->fraction;
    uint32_t best_divisor = start_divisor;
    for (uint32_t div = start_divisor + 1; div < max_divisor; ++div) {
      baud_error = 60'000'000 / (16 * div);
      error_fract = ((baud_error * 1000) / baud) - 1000;
      fract = binary_search_closest(baud_fraction_lookup, error_fract);
      int32_t fract_error = error_fract - fract->fraction;
      if(MCUI::util::abs(fract_error) < MCUI::util::abs(best_fract_error)) {
        best_fract_error = fract_error;
        best_divisor = div;
        best_fract = fract;
      }
    }
    return {best_divisor, best_fract->div_add_val, best_fract->mul_val};
  }

  constexpr baud_divisors find_divisors(const uint32_t baud) {
    uint32_t base_divisor = 60'000'000 / (16 * baud);
    if (60'000'000 / (base_divisor * 16) == baud) {
      return {base_divisor, 0, 1};
    } else {
      return find_best_divisors_for_available_fractions(baud, base_divisor > 1 ? base_divisor / 2 : 1, base_divisor);
    }
  }

  struct baud_rate_divisors {
    uint32_t baud;
    baud_divisors divisors;
    baud_rate_divisors() : baud{0}, divisors{} { }
    constexpr baud_rate_divisors(const uint32_t baud) : baud{baud}, divisors{find_divisors(baud)} { }
  };
  constexpr std::array<baud_rate_divisors, 18> baud_divisor_lookup = { 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 230400, 250000, 460800, 500000, 576000, 921600, 1000000, 2500000 };

  struct uart_pin_fn {
    pin_t pin;
    uint8_t function;
  };

  constexpr std::array<std::array<uart_pin_fn, 3>, 5> uart_rx_pin_lookup = {
    std::array<uart_pin_fn, 3> {uart_pin_fn{P0_01, 4}, {P0_03, 1}, {P_NC,  0}},
                               {uart_pin_fn{P0_16, 1}, {P2_01, 2}, {P3_17, 3}},
                               {uart_pin_fn{P0_11, 1}, {P2_09, 2}, {P4_23, 2}},
                               {uart_pin_fn{P0_01, 2}, {P0_03, 2}, {P4_29, 2}},
                               {uart_pin_fn{P2_09, 3}, {P5_03, 4}, {P_NC,  0}},
  };
  constexpr std::array<std::array<uart_pin_fn, 3>, 5> uart_tx_pin_lookup = {
    std::array<uart_pin_fn, 3> {uart_pin_fn{P0_00, 4}, {P0_02, 1}, {P_NC,  0}},
                               {uart_pin_fn{P0_15, 1}, {P2_00, 2}, {P3_16, 3}},
                               {uart_pin_fn{P0_10, 1}, {P2_08, 2}, {P4_22, 2}},
                               {uart_pin_fn{P0_00, 2}, {P0_02, 2}, {P4_28, 2}},
                               {uart_pin_fn{P0_22, 3}, {P1_29, 5}, {P5_04, 4}}
  };

  namespace UART {
    struct Config {
      enum class ParityMode : uint8_t { NONE, ODD, EVEN, ONE, ZERO };
      uint32_t baud = 9600;
      uint8_t word_length = 8;
      uint8_t stop_bits = 1;
      ParityMode parity = ParityMode::NONE;
    };

    [[gnu::always_inline]] static constexpr IRQNumber irq_number(const size_t uart_id) { return uart_irq_number[uart_id]; }
    [[gnu::always_inline]] static constexpr PeripheralPowerControl power_control_bit(const size_t uart_id) { return uart_power_control_bit[uart_id]; }

    static std::pair<uart_pin_fn, uart_pin_fn> pin_function_lookup(const size_t uart_id, const pin_t pin_rx, const pin_t pin_tx) {
      auto lookup_rx_fn = [pin_rx](uart_pin_fn it){ return it.pin == pin_rx; };
      auto lookup_tx_fn = [pin_tx](uart_pin_fn it){ return it.pin == pin_tx; };
      auto& rx_pins = uart_rx_pin_lookup[uart_id];
      auto& tx_pins = uart_tx_pin_lookup[uart_id];
      auto rx_fn_it = std::find_if(rx_pins.begin(), rx_pins.end(), lookup_rx_fn);
      auto tx_fn_it = std::find_if(tx_pins.begin(), tx_pins.end(), lookup_tx_fn);
      uart_pin_fn rx_fn = rx_fn_it == rx_pins.end() ? uart_pin_fn{P_NC, 0}: *rx_fn_it;
      uart_pin_fn tx_fn = tx_fn_it == tx_pins.end() ? uart_pin_fn{P_NC, 0}: *tx_fn_it;
      return {rx_fn, tx_fn};
    }

    static void configure_pins(const size_t uart_id, const pin_t pin_rx, const pin_t pin_tx) {
      auto [rx_fn, tx_fn] = pin_function_lookup(uart_id, pin_rx, pin_tx);

      if (rx_fn.pin == pin_rx) {
        pin_set_mode(rx_fn.pin, PinMode::NORMAL);
        pin_enable_function(rx_fn.pin, rx_fn.function);
      }

      if (tx_fn.pin == pin_tx) {
        pin_set_mode(tx_fn.pin, PinMode::NORMAL);
        pin_enable_function(tx_fn.pin, tx_fn.function);
      }
    }

    static void set_baud(volatile UARTRegisterBaseExt& uart, const size_t baud) {
      baud_rate_divisors divisors;

      // try to lookup the correct divisors
      for (auto& baud_desc : baud_divisor_lookup) {
        if (baud_desc.baud == baud) {
          divisors = baud_desc;
          break;
        }
      }

      // lookup failed so calculate divisors
      if (divisors.baud == 0) {
        divisors.divisors = find_divisors(baud);
      }

      //unlock the divisor registers
      uart.LCR |= (1UL << 7);
      uart.DLL = (divisors.divisors.divisor & 0xFF);
      uart.DLM = ((divisors.divisors.divisor >> 8) & 0xFF);
      uart.FDR = (((divisors.divisors.mul_val & 0xF) << 4) | (divisors.divisors.div_add_val & 0xF)) & 0xFF;
      //lock the divisor registers
      uart.LCR &= ~(1UL << 7);
    }
    [[gnu::always_inline]] static inline void set_baud(const size_t uart_id, const size_t baud) {
      auto& uart = uart_lookup_by_id(uart_id);
      return set_baud(uart, baud);
    }

    [[gnu::always_inline]] static inline void enable(const size_t uart_id) {
      peripheral_power_on(power_control_bit(uart_id));

      auto& uart = uart_lookup_by_id(uart_id);
      uart.IER = 0x0;
      uart.FCR = 0x6;

      MCUCore::nvic_set_priority(irq_number(uart_id), MCUCore::nvic_encode_priority(0, 3, 0)); // todo: global compile time config structure
      MCUCore::nvic_enable_irq(irq_number(uart_id));
    }

    [[gnu::always_inline]] static inline void disable(const size_t uart_id) {
      MCUCore::nvic_disable_irq(irq_number(uart_id));
      peripheral_power_off(power_control_bit(uart_id));
    }

    [[gnu::always_inline]] static inline void rx_interrupts_enable(volatile UARTRegisterBaseExt& uart) {
      uart.FCR = 0x87UL; // Enable fifo @ level 2 trigger (8 byte)
      uart.IER = 5UL;
    }
    [[gnu::always_inline]] static inline void rx_interrupts_enable(const size_t uart_id) {
      rx_interrupts_enable(uart_lookup_by_id(uart_id));
    }
    [[gnu::always_inline]] static inline void rx_interrupts_disable(volatile UARTRegisterBaseExt& uart) {
      uart.IER &= ~5UL;
    }
    [[gnu::always_inline]] static inline void rx_interrupts_disable(const size_t uart_id) {
      rx_interrupts_disable(uart_lookup_by_id(uart_id));
    }

    [[gnu::always_inline]] static inline void tx_interrupts_enable(volatile UARTRegisterBaseExt& uart) {
      uart.IER |= 2UL;
    }
    [[gnu::always_inline]] static inline void tx_interrupts_enable(const size_t uart_id) {
      tx_interrupts_enable(uart_lookup_by_id(uart_id));
    }
    [[gnu::always_inline]] static inline void tx_interrupts_disable(volatile UARTRegisterBaseExt& uart) {
      uart.IER &= ~2UL;
    }
    [[gnu::always_inline]] static inline void tx_interrupts_disable(const size_t uart_id) {
      tx_interrupts_disable(uart_lookup_by_id(uart_id));
    }

    [[gnu::always_inline]] static inline void transmit_enable(volatile UARTRegisterBaseExt& uart) {
      uart.TER |= (1UL << 7);
    }
    [[gnu::always_inline]] static inline void transmit_enable(const size_t uart_id) {
      transmit_enable(uart_lookup_by_id(uart_id));
    }

    [[gnu::always_inline]] static inline void transmit_disable(volatile UARTRegisterBaseExt& uart) {
      uart.TER &= ~(1UL << 7);
    }
    [[gnu::always_inline]] static inline void transmit_disable(const size_t uart_id) {
      transmit_disable(uart_lookup_by_id(uart_id));
    }

    static void init(const size_t uart_id, const Config config = {}) {
      auto& uart = uart_lookup_by_id(uart_id);

      set_baud(uart, config.baud);

      uint8_t LCR = (config.word_length - 5);

      if (config.stop_bits == 2) {
        LCR |= (1UL << 2);
      }

      if (config.parity != Config::ParityMode::NONE) {
        LCR |= (1UL << 3);
        LCR |= ((static_cast<uint8_t>(config.parity) - 1) << 4);
      }

      uart.LCR = LCR;
      uart.FCR = 0x6; // reset fifos
      enable(uart_id);
      transmit_enable(uart);
    }

    static inline size_t tx_available(volatile UARTRegisterBaseExt& uart) {
      return uart.LSR & 0x20UL; // THR empty
    }
    static inline size_t tx_available(const size_t uart_id) {
      return tx_available(uart_lookup_by_id(uart_id));
    }

    static inline size_t rx_available(volatile UARTRegisterBaseExt& uart) {
      return uart.LSR & 0x1UL;
    }
    static inline size_t rx_available(const size_t uart_id) {
      return rx_available(uart_lookup_by_id(uart_id));
    }

    static inline size_t read(volatile UARTRegisterBaseExt& uart, char *const value) {
      if (!rx_available(uart)) return 0;
      *value = uart.RBR;
      return 1;
    }
    static inline size_t read(const size_t uart_id, char *const value) {
      return read(uart_lookup_by_id(uart_id), value);
    }

    static inline size_t write(volatile UARTRegisterBaseExt& uart, const char value) {
      if (!tx_available(uart)) return 0;
      uart.THR = value;
      return 1;
    }
    static inline size_t write(const size_t uart_id, const char value) {
      return write(uart_lookup_by_id(uart_id), value);
    }

    // will block until length bytes received
    static inline size_t read(volatile UARTRegisterBaseExt& uart, char* dst, const size_t length) {
      size_t count = 0;
      for (; count < length; ++count) {
        while(read(uart, dst + count) == 0);
      }
      return count;
    }
    static inline size_t read(const size_t uart_id, char* dst, const size_t length) {
      return read(uart_lookup_by_id(uart_id), dst, length);
    }

    // will block until length bytes transmitted
    static inline size_t write(volatile UARTRegisterBaseExt& uart, const char* src, const size_t length) {
      size_t count = 0;
      for (; count < length; ++count) {
        while(write(uart, src[count]) == 0); // block until the byte is in the tx fifo
      }
      return count;
    }
    static inline size_t write(const size_t uart_id, const char* src, const size_t length) {
      return write(uart_lookup_by_id(uart_id), src, length);
    }

    static inline size_t write(volatile UARTRegisterBaseExt& uart, const char *src) {
      if (src == nullptr) return 0;
      return write(uart, src, strlen(src));
    }
    static inline size_t write(const size_t uart_id, const char *src) {
      return write(uart_lookup_by_id(uart_id), src, strlen(src));
    }
  }

  class BufferedUARTC;
  typedef void(BufferedUARTC::*uart_callback_fn)(void);
  struct uart_callback{
    BufferedUARTC * object = nullptr;
    uart_callback_fn method_ptr = nullptr;
    [[gnu::always_inline]] inline void call(){ (object->*method_ptr)(); }
  };

  class RawUARTC {
  public:
    RawUARTC(const uint8_t uart_id) : m_uart{uart_lookup_by_id(uart_id)}, m_uart_id{uart_id} { }
    virtual void configure_pins(const pin_t pin_rx, const pin_t pin_tx) { UART::configure_pins(m_uart_id, pin_rx, pin_tx); }
    virtual void init(UART::Config config) { UART::init(m_uart_id, config); }
    virtual inline size_t read(char* dst, const size_t length) { return UART::read(m_uart, dst, length); }
    [[gnu::always_inline]] virtual inline size_t read(char *const value) { return UART::read(m_uart, value); }
    virtual inline size_t write(const char* src, const size_t length) { return UART::write(m_uart, src, length); }
    [[gnu::always_inline]] virtual inline size_t write(const char *src) { return UART::write(m_uart, src); }
    [[gnu::always_inline]] virtual inline size_t write(const char value) { return UART::write(m_uart, value); }
    [[gnu::always_inline]] virtual inline size_t tx_available() { return UART::tx_available(m_uart); }
    [[gnu::always_inline]] virtual inline size_t rx_available() { return UART::rx_available(m_uart); }
    [[gnu::always_inline]] inline volatile UARTRegisterBaseExt& base_uart() const { return m_uart; }
    [[gnu::always_inline]] inline size_t uart_id() const { return m_uart_id; }

  protected:
    volatile UARTRegisterBaseExt& m_uart;
    const size_t m_uart_id;
  };

  class BufferedUARTC : public RawUARTC {
  public:
    BufferedUARTC(uint8_t uart_id) : RawUARTC(uart_id) { }

    void init(UART::Config config) override {
      redirect_isr(uart_callback{.object = this, .method_ptr = &BufferedUARTC::interrupt});
      RawUARTC::init(config);
      UART::rx_interrupts_enable(m_uart_id);
    }

    [[gnu::always_inline]] inline bool peek(char *const value) {
      return m_buffer.in().peek(value);
    }

    virtual inline size_t read(char* dst, const size_t length) override { return m_buffer.in().read(dst, length); }
    [[gnu::always_inline]] virtual inline size_t read(char *const value) override { return read(value, 1); }

    virtual inline size_t write(const char* src, const size_t length) override {
      if (src == nullptr || length == 0) return 0;


      m_uart.IER &= ~2UL; //disable tx isr
      MCUCore::ISB();

      size_t written = 0;
      if ( m_buffer.out().empty() && (m_uart.LSR & (1UL << 6))) { // buffer and fifo empty
        while (written < length && written < 16) { // the hardware fifo was empty fill it first
          m_uart.THR = src[written];
          ++written;
        }
      }
      do { // TODO: optionally disable blocking on full software buffer? or/and enable blocking until transmit completes?
        written += m_buffer.out().write(src + written, length - written);
        if (m_buffer.out().available()) UART::tx_interrupts_enable(m_uart);
      } while (written < length);
      return written;
    }

    [[gnu::always_inline]] virtual inline size_t write(const char *src) override { return write(src, strlen(src)); }
    [[gnu::always_inline]] virtual inline size_t write(const char value) override { return write(&value, 1); }
    [[gnu::always_inline]] virtual inline size_t tx_available() override { return m_buffer.out().free(); }
    [[gnu::always_inline]] virtual inline size_t rx_available() override { return m_buffer.in().available(); }

    [[gnu::always_inline]] inline void redirect_isr(uart_callback isr) const { s_interrupt_vectors[m_uart_id] = isr; };
    [[gnu::always_inline]] static inline void call_isr(size_t uart_id) { s_interrupt_vectors[uart_id].call(); };

    [[gnu::always_inline]] inline void set_rx_callback(std::function<bool(char)> cb) { m_rx_callback = cb; }

    void interrupt() {
      const uint32_t interrupt_id = (m_uart.IIR & (7UL << 1UL)) >> 1UL;
      if (interrupt_id == 3UL) { // Receive Line Status
        [[maybe_unused]] uint32_t line_status = m_uart.LSR;
        // todo: set a status on any error
        [[maybe_unused]] volatile uint32_t dummy = m_uart.RBR; //clear interrupt state
        return;
      } else if (interrupt_id & (6UL | 2UL)) { // Receive Data Available or Character Time-out indication
        while (m_uart.LSR & 1UL) {
          uint8_t rx_value = m_uart.RBR;
          if (!m_rx_callback || (m_rx_callback && m_rx_callback(rx_value))) {
            m_buffer.in().write(rx_value); // read until empty FIFO, reading RBR clears both interrupt flags
          }
        }
      } else if (interrupt_id == 1UL) { // Transmit Ready
        for(size_t i = 0; i < 16 && m_buffer.out().available(); ++i) { // fifo is empty, so fill upto 16 bytes
          char c = 0;
          m_buffer.out().read(&c);
          m_uart.THR = c;
        }
        if (m_buffer.out().empty()) UART::tx_interrupts_disable(m_uart); // disable THRE interrupt
      }
    }

    static inline uart_callback s_interrupt_vectors[5] = {};
    MCUI::util::InOutRingBuffer<char, 128> m_buffer = {};
    std::function<bool(char)> m_rx_callback = {};
  };

  // template <size_t UART_ID>
  // struct RawUART {

  //   static inline void set_interrupt_vector(void(*isr)(void)) { m_isr = isr; }
  //   static inline volatile UARTRegisterBaseExt& m_uart = uart_lookup_by_id(UART_ID);
  //   static void(*m_isr)(void);
  // };

  // template <size_t UART_ID, size_t BUFFER_SIZE>
  // struct BufferedUART : public RawUART<UART_ID> {
  //   using UART = RawUART<UART_ID>;

  //   static void init(typename UART::Config config = {}) {
  //     UART::set_interrupt_vector(interrupt);
  //     UART::init(config);
  //     UART::rx_interrupts_enable();
  //   }

  //   static bool peek(char *const value) {
  //     return m_buffer.in().peek(value);
  //   }

  //   static inline size_t read(char* dst, size_t length) {
  //     return m_buffer.in().read(dst, length);
  //   }

  //   static inline size_t write(const char* src, size_t length) {
  //     if (src == nullptr || length == 0) return 0;

  //     UART::m_uart.IER &= ~2UL; //disable tx isr
  //     MCUCore::ISB();

  //     size_t written = 0;
  //     if ( m_buffer.out().empty() && (UART::m_uart.LSR & (1UL << 6))) { // buffer and fifo empty
  //       UART::m_uart.THR = src[0]; // transfer 1 byte to trigger tx isr
  //       written = 1 + m_buffer.out().write(src + 1, length - 1);
  //     } else written = m_buffer.out().write(src, length);

  //     UART::tx_interrupts_enable();

  //     return written;
  //   }

  //   static inline size_t write(const char *src) {
  //     if (src == nullptr) return 0;
  //     return write(src, strlen(src));
  //   }

  //   static inline size_t read(char *const value) {
  //     return read(value, 1);
  //   }

  //   static inline size_t write(const char value) {
  //     return write(&value, 1);
  //   }

  //   static size_t available() {
  //     return m_buffer.in().available();
  //   }

  //   [[gnu::always_inline]] static inline void interrupt() {
  //     const uint32_t interrupt_id = (UART::m_uart.IIR & (7UL << 1UL)) >> 1UL;
  //     if (interrupt_id == 3UL) { // Receive Line Status
  //       [[maybe_unused]] uint32_t line_status = UART::m_uart.LSR;
  //       // todo: set a status on any error
  //       [[maybe_unused]] volatile uint32_t dummy = UART::m_uart.RBR;
  //       return;
  //     }

  //     if (interrupt_id & (6UL | 2UL)) { // Receive Data Available or Character Time-out indication
  //       while (UART::m_uart.LSR & 1UL) m_buffer.in().write(UART::m_uart.RBR); // read until empty FIFO, reading RBR clears both interrupt flags
  //     }

  //     // empty the tx queue
  //     if (interrupt_id == 1UL) { // Transmit Ready
  //       // disable THRE interrupt
  //       UART::m_uart.IER &= ~2UL;
  //       MCUCore::ISB();
  //       // wait until not busy
  //       while ((UART::m_uart.LSR & (1UL << 6)) == 0);
  //       // transfer FIFO bytes from queue
  //       for(size_t i = 0; i < 16; ++i) {
  //         if (m_buffer.out().empty()) return; // no data remaining, leave tx interrupt disabled
  //         char c = 0;
  //         m_buffer.out().read(&c);
  //         UART::m_uart.THR = c;
  //       }
  //       // enable the THRE interrupt
  //       UART::m_uart.IER |= 2;
  //     }
  //   }

  // private:
  //   static inline InOutRingBuffer<char, BUFFER_SIZE> m_buffer = {};
  // };


}
