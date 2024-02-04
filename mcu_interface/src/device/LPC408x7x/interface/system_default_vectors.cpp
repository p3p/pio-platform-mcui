#include <cstdint>

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
    SysTick_Handler,                          /*  -1 SysTick Handler */

    /* Interrupts */
    WDT_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    TIMER3_IRQHandler,
    UART0_IRQHandler,
    UART1_IRQHandler,
    UART2_IRQHandler,
    UART3_IRQHandler,
    PWM1_IRQHandler,
    I2C0_IRQHandler,
    I2C1_IRQHandler,
    I2C2_IRQHandler,
    0,
    SSP0_IRQHandler,
    SSP1_IRQHandler,
    PLL0_IRQHandler,
    RTC_IRQHandler,
    EINT0_IRQHandler,
    EINT1_IRQHandler,
    EINT2_IRQHandler,
    EINT3_IRQHandler,
    ADC_IRQHandler,
    BOD_IRQHandler,
    USB_IRQHandler,
    CAN_IRQHandler,
    DMA_IRQHandler,
    I2S_IRQHandler,
    ENET_IRQHandler,
    MCI_IRQHandler,
    MCPWM_IRQHandler,
    QEI_IRQHandler,
    PLL1_IRQHandler,
    USBActivity_IRQHandler,
    CANActivity_IRQHandler,
    UART4_IRQHandler,
    SSP2_IRQHandler,
    LCD_IRQHandler,
    GPIO_IRQHandler,
    PWM0_IRQHandler,
    EEPROM_IRQHandler
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
