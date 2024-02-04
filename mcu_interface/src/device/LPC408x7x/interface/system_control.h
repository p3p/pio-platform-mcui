#pragma once

#include <cstdint>
#include <functional>
#include <mcu_core.h>
#include <utility/const_functions.h>
#include <utility/bit_manipulation.h>

typedef void(*VECTOR_TABLE_Type)(void);
extern "C" const VECTOR_TABLE_Type __Vectors[240];
extern "C" uint32_t SystemCoreClock;
extern "C" uint32_t PeripheralClock;
extern "C" uint32_t EMCClock;
extern "C" uint32_t USBClock;

namespace LPC4078 {

constexpr inline uint32_t irq_to_vector_table_index(IRQNumber irq) {
  return MCUI::util::to_integral(irq) + 16;
}

enum struct PeripheralPowerControl : uint32_t {
  LCD,
  TIM0,
  TIM1,
  UART0,
  UART1,
  PWM0,
  PWM1,
  I2C0,
  UART4,
  RTC,
  SSP1,
  EMC,
  ADC,
  CAN1,
  CAN2,
  GPIO,
  SPIFI,
  MCPPWM,
  QEI,
  I2C1,
  SSP2,
  SSP0,
  TIM2,
  TIM3,
  UART2,
  UART3,
  I2C2,
  I2S,
  SDC,
  GPDMA,
  ENET,
  USB,
};

// Table 9. Register overview: System control (base address 0x400F C000)
struct SystemControlRegion {
  uint32_t FLASHCFG;    // 0x000, RW, Flash Accelerator Configuration Register. Controls flash access timing
  uint32_t RESERVED00[31];
  uint32_t PLL0CON;     // 0x080, RW, PLL0 Control registers
  uint32_t PLL0CFG;     // 0x084, RW, PLL0 Configuration registers
  uint32_t PLL0STAT;    // 0x088, RO, PLL0 Status registers
  uint32_t PLL0FEED;    // 0x08C, WO, PLL0 Feed registers
  uint32_t RESERVED01[4];
  uint32_t PLL1CON;     // 0x0A0, RW, PLL1 Control registers
  uint32_t PLL1CFG;     // 0x0A4, RW, PLL1 Configuration registers
  uint32_t PLL1STAT;    // 0x0A8, RO, PLL1 Status registers
  uint32_t PLL1FEED;    // 0x0AC, WO, PLL1 Feed registers
  uint32_t RESERVED02[4];
  uint32_t PCON;        // 0x0C0, RW, Power Control register
  uint32_t PCONP;       // 0x0C4, RW, Power Control for Peripherals
  uint32_t PCONP1;      // 0x0C8, RW, Power Control for Peripherals 1
  uint32_t RESERVED03[13];
  uint32_t EMCCLKSEL;   // 0x100, RW, External Memory Controller Clock Selection register
  uint32_t CCLKSEL;     // 0x104, RW, CPU Clock Selection register
  uint32_t USBCLKSEL;   // 0x108, RW, USB Clock Selection register
  uint32_t CLKSRCSEL;   // 0x10C, RW, Clock Source Select Register
  uint32_t CANSLEEPCLR; // 0x110, RW, Allows clearing the current CAN channel sleep state as well as reading back that state
  uint32_t CANWAKEFLAGS;// 0x114, RW, Indicates the wake-up state of the CAN channels
  uint32_t RESERVED04[10];
  uint32_t EXTINT;      // 0x140, RW, External Interrupt Flag Register
  uint32_t RESERVED05;
  uint32_t EXTMODE;     // 0x148, RW, External Interrupt Mode register
  uint32_t EXTPOLAR;    // 0x14C, RW, External Interrupt Polarity Register
  uint32_t RESERVED06[12];
  uint32_t RSID;        // 0x180, RW, Reset Source Identification Register
  uint32_t RESERVED07[7];
  uint32_t SCS;         // 0x1A0, RW, System Control and Status
  uint32_t IRCCTRL;     // 0x1A4, RW, IRC control
  uint32_t PCLKSEL;     // 0x1A8, RW, Peripheral Clock Selection register
  uint32_t RESERVED08;
  uint32_t PBOOST;      // 0x1B0, RW, Power boost register
  uint32_t SPIFICLKSEL; // 0x1B4, RW, SPIFI Clock Selection register
  uint32_t LCD_CFG;     // 0x1B8, RW, LCD Clock configuration register
  uint32_t RESERVED09;
  uint32_t USBINTST;    // 0x1C0, RW, USB Interrupt Status
  uint32_t DMACREQSEL;  // 0x1C4, RW, Selects between alternative requests on DMA channels 0 through 7 and 10 through 15
  uint32_t CLKOUTCFG;   // 0x1C8, RW, Clock Output Configuration register
  uint32_t RSTCON0;     // 0x1CC, RW, Individual peripheral reset control bits
  uint32_t RSTCON1;     // 0x1D0, RW, Individual peripheral reset control bits
  uint32_t RESERVED10[2];
  uint32_t EMCDLYCTL;   // 0x1DC, RW, Values for the 4 programmable delays associated with SDRAM operation
  uint32_t EMCCAL;      // 0x1E0, RW, Controls the calibration counter for programmable delays and returns the result value.
};
static_assert(sizeof(SystemControlRegion) == 0x1E4);
constexpr uint32_t system_control_region_base = 0x400FC000;
static inline volatile auto& system_control = *reinterpret_cast<volatile SystemControlRegion* const>(system_control_region_base);

constexpr uint32_t zero_safe_divide(const uint32_t x, const uint32_t y) { return y == 0 ? 0: x / y; }

constexpr uint32_t osc_frequency_crystal  = 12000000UL;
constexpr uint32_t osc_frequency_rtc      = 32768UL;
constexpr uint32_t osc_frequency_irc      = 12000000UL;
constexpr uint32_t osc_frequency_watchdog = 500000UL;

inline void update_clock_core() {
  const uint32_t active_osc_frequency = ((system_control.CLKSRCSEL & 0x01) == 0) ? osc_frequency_irc : osc_frequency_crystal;

  if ((system_control.CCLKSEL & 0x100) == 0) {
    SystemCoreClock = zero_safe_divide(active_osc_frequency, (system_control.CCLKSEL & 0x1F));
    PeripheralClock = zero_safe_divide(active_osc_frequency, (system_control.PCLKSEL & 0x1F));
    EMCClock        = (SystemCoreClock / ((system_control.EMCCLKSEL & 0x01) + 1));
    return;
  }

  uint32_t multiplied_osc_frequency = active_osc_frequency * ((system_control.PLL0STAT & 0x1F) + 1);
  SystemCoreClock = zero_safe_divide(multiplied_osc_frequency, (system_control.CCLKSEL & 0x1F));
  PeripheralClock = zero_safe_divide(multiplied_osc_frequency, (system_control.PCLKSEL & 0x1F));
  EMCClock        = SystemCoreClock / ((system_control.EMCCLKSEL & 0x01) + 1);
}

inline void update_clock_usb() {
  const uint32_t active_osc_frequency = ((system_control.CLKSRCSEL & 0x01) == 0) ? osc_frequency_irc : osc_frequency_crystal;

  if (system_control.USBCLKSEL & (0x02 << 8)) { // has PLL selected
    uint8_t usb_div = (system_control.USBCLKSEL & 0x1F);
    uint8_t mul = system_control.USBCLKSEL & (0x01 << 8) ? ((system_control.PLL0STAT & 0x1F) + 1) : ((system_control.PLL1STAT & 0x1F) + 1);
    USBClock = active_osc_frequency * mul / usb_div;
  } else USBClock = 0;
}

inline void system_init() {
  system_control.SCS = 0x01UL << 5; // enable main oscillator
  while ((system_control.SCS & (0x01UL << 6)) == 0); // wait for the oscillator to be ready

  system_control.PBOOST |= 0x03; // allow 120Mhz

  system_control.CLKSRCSEL = 0x01; // Selects the main oscillator as the sysclk and PLL0 clock source

  system_control.PLL0CFG  = 0x09; // set multiplier to 10, value is M - 1
  system_control.PLL0CON  = 0x01; // enable PLL0
  system_control.PLL0FEED = 0xAA;
  system_control.PLL0FEED = 0x55; // push the PLL update
  while ((system_control.PLL0STAT & (0x01UL << 10)) == 0); // Wait for PLOCK0

  system_control.PLL1CFG  = 0x23; // Multiplier 4, Divisor 2
  system_control.PLL1CON  = 0x01; // enable PLL1
  system_control.PLL1FEED = 0xAA;
  system_control.PLL1FEED = 0x55; // push the PLL update
  while ((system_control.PLL1STAT & (0x01UL << 10)) == 0); // Wait for PLOCK1

  system_control.CCLKSEL   = 0x101; // use main PLL for input to clock divider, enable the clock divider (divide by 1 just enables it)
  system_control.USBCLKSEL = 0x201; // use alt PLL for the input to the usb clk
  system_control.PCLKSEL   = 0x002; // peripheral clock divider set to 2
  system_control.PCONP     = 0x042887DE; // Enables: PWM1, UART4, SSP1, SSP0

  system_control.FLASHCFG  = 0x00005000 | 0x03A; // set the flash accelerator to 5 (100Mhz+) 5 clock cycles per read. 0x03A MUST be in the lower nibbles

  MCUCore::system_control_block->VTOR = ((int)&__Vectors); // use symbol from linker script to get interrupt vector location, should be RAM origin
  update_clock_core();
  update_clock_usb();
}

inline void peripheral_power_on(PeripheralPowerControl peripheral_bit) {
  MCUI::util::bit_set(system_control.PCONP, MCUI::util::to_integral(peripheral_bit));
}

inline void peripheral_power_off(PeripheralPowerControl peripheral_bit) {
  MCUI::util::bit_clear(system_control.PCONP, MCUI::util::to_integral(peripheral_bit));
}

using ISRFunction  = void(*)(void);
inline void register_isr(IRQNumber irq, ISRFunction fn) {
  // check vectors are in actually in RAM
  if (!MCUI::util::bit_test(MCUCore::system_control_block->VTOR, 29)) {
    ((ISRFunction*)MCUCore::system_control_block->VTOR)[irq_to_vector_table_index(irq)] = fn;
  }
}

} // namespace LPC4078
