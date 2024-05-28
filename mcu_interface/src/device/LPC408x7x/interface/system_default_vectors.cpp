#include <cstdint>

#ifndef LPC_USB_STACK_ENABLED
  #define LPC_USB_STACK_ENABLED Enabled
#endif

namespace MCUI::config {
  enum ConfigValue : uint64_t {
    Disabled,
    Enabled
  };
  inline constexpr ConfigValue systick = Enabled;
  inline constexpr ConfigValue adc = Enabled;
  inline constexpr ConfigValue eeprom = Enabled;
  inline constexpr ConfigValue watchdog = Enabled;
  inline constexpr ConfigValue usb = LPC_USB_STACK_ENABLED;
  inline constexpr ConfigValue can = Enabled;
  inline constexpr ConfigValue rtc = Enabled;
  inline constexpr ConfigValue mcpwm = Enabled;
  inline constexpr ConfigValue ethernet = Enabled;
  inline constexpr ConfigValue usart[] = {
    Enabled,
    Enabled,
    Enabled,
    Enabled,
    Enabled
  };
  inline constexpr ConfigValue timer[] = {
    Enabled,
    Enabled,
    Enabled,
    Enabled
  };
  inline constexpr ConfigValue ssp[] = {
    Enabled,
    Enabled,
    Enabled
  };
  inline constexpr ConfigValue pwm[] = {
    Enabled,
    Enabled
  };
  inline constexpr ConfigValue i2c[] = {
    Enabled,
    Enabled,
    Enabled
  };
  template <ConfigValue V, typename T> inline constexpr T value_if_enabled(T&& value, T&& or_value) {
    if constexpr (V == ConfigValue::Enabled) return value;
    else return or_value;
  }
}

[[gnu::section(".noinit")]] uint32_t SystemCoreClock = 0;
[[gnu::section(".noinit")]] uint32_t PeripheralClock = 0;
[[gnu::section(".noinit")]] uint32_t EMCClock = 0;
[[gnu::section(".noinit")]] uint32_t USBClock = 0;

extern "C" {
  extern uint32_t __StackTop;

  [[noreturn]] void Reset_Handler();
               void Default_Handler();

  [[gnu::weak, gnu::alias("Default_Handler")]] void NMI_Handler();
  [[gnu::weak]] void HardFault_Handler();
  [[gnu::weak, gnu::alias("Exception_Handler")]] void MemManage_Handler();
  [[gnu::weak, gnu::alias("Exception_Handler")]] void BusFault_Handler();
  [[gnu::weak, gnu::alias("Exception_Handler")]] void UsageFault_Handler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void SVC_Handler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void DebugMon_Handler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void PendSV_Handler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void SysTick_Handler();

  [[gnu::weak, gnu::alias("Default_Handler")]] void WDT_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void TIMER0_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void TIMER1_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void TIMER2_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void TIMER3_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void UART0_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void UART1_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void UART2_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void UART3_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void PWM1_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void I2C0_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void I2C1_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void I2C2_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void SPI_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void SSP0_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void SSP1_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void PLL0_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void RTC_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void EINT0_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void EINT1_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void EINT2_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void EINT3_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void ADC_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void BOD_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void USB_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void CAN_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void DMA_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void I2S_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void ENET_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void MCI_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void MCPWM_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void QEI_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void PLL1_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void USBActivity_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void CANActivity_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void UART4_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void SSP2_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void LCD_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void GPIO_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void PWM0_IRQHandler();
  [[gnu::weak, gnu::alias("Default_Handler")]] void EEPROM_IRQHandler();

  [[gnu::weak]] void Reserved_Handler();

  typedef void(*VECTOR_TABLE_Type)(void);
  extern volatile const VECTOR_TABLE_Type __Vectors[64];
  [[gnu::used, gnu::section(".vectors")]] volatile const VECTOR_TABLE_Type __Vectors[64] = {
    (VECTOR_TABLE_Type)(&__StackTop),         /*     Initial Stack Pointer */
    Reset_Handler,                            /*     Reset Handler */
    NMI_Handler,                              /* -14 NMI Handler */
    HardFault_Handler,                        /* -13 Hard Fault Handler */
    MemManage_Handler,                        /* -12 MPU Fault Handler */
    BusFault_Handler,                         /* -11 Bus Fault Handler */
    UsageFault_Handler,                       /* -10 Usage Fault Handler */
    Reserved_Handler,                         /*     Reserved vector sum*/
    Reserved_Handler,                         /*     Reserved vector sum*/
    Reserved_Handler,                         /*     Reserved */
    Reserved_Handler,                         /*     Reserved */
    SVC_Handler,                              /*  -5 SVC Handler */
    DebugMon_Handler,                         /*  -4 Debug Monitor Handler */
    Reserved_Handler,                         /*     Reserved */
    PendSV_Handler,                           /*  -2 PendSV Handler */
    MCUI::config::value_if_enabled<MCUI::config::systick>(SysTick_Handler, Default_Handler),  /*  -1 SysTick Handler */

    /* Interrupts */
    MCUI::config::value_if_enabled<MCUI::config::watchdog>(WDT_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::timer[0]>(TIMER0_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::timer[1]>(TIMER1_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::timer[2]>(TIMER2_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::timer[3]>(TIMER3_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::usart[0]>(UART0_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::usart[1]>(UART1_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::usart[2]>(UART2_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::usart[3]>(UART3_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::pwm[1]>(PWM1_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::i2c[0]>(I2C0_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::i2c[1]>(I2C1_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::i2c[2]>(I2C2_IRQHandler,  Default_Handler),
    0,
    MCUI::config::value_if_enabled<MCUI::config::ssp[0]>(SSP0_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::ssp[1]>(SSP1_IRQHandler, Default_Handler),
    PLL0_IRQHandler,
    MCUI::config::value_if_enabled<MCUI::config::rtc>(RTC_IRQHandler, Default_Handler),
    EINT0_IRQHandler,
    EINT1_IRQHandler,
    EINT2_IRQHandler,
    EINT3_IRQHandler,
    MCUI::config::value_if_enabled<MCUI::config::adc>(ADC_IRQHandler,  Default_Handler),
    BOD_IRQHandler,
    MCUI::config::value_if_enabled<MCUI::config::usb>(USB_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::can>(CAN_IRQHandler,  Default_Handler),
    DMA_IRQHandler,
    I2S_IRQHandler,
    MCUI::config::value_if_enabled<MCUI::config::ethernet>(ENET_IRQHandler, Default_Handler),
    MCI_IRQHandler,
    MCUI::config::value_if_enabled<MCUI::config::mcpwm>(MCPWM_IRQHandler,  Default_Handler),
    QEI_IRQHandler,
    PLL1_IRQHandler,
    MCUI::config::value_if_enabled<MCUI::config::usb>(USBActivity_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::can>(CANActivity_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::usart[3]>(UART4_IRQHandler,  Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::ssp[2]>(SSP2_IRQHandler, Default_Handler),
    LCD_IRQHandler,
    GPIO_IRQHandler,
    MCUI::config::value_if_enabled<MCUI::config::pwm[0]>(PWM0_IRQHandler, Default_Handler),
    MCUI::config::value_if_enabled<MCUI::config::eeprom>(EEPROM_IRQHandler, Default_Handler)
  };

  [[gnu::weak]] void Reserved_Handler() {
    while(1);
  }

  [[gnu::weak]] void HardFault_Handler() {
    while(1);
  }

  [[gnu::weak]] void Exception_Handler() {
    while(1);
  }

  [[gnu::weak]] void Default_Handler() {
    while(1);
  }

}
