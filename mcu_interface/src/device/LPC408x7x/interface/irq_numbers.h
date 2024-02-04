#pragma once

#include <cstdint>

namespace LPC4078 {

enum struct IRQNumber : int32_t {
  NonMaskableInt = -14,    // 2 Non Maskable Interrupt
  MemoryManagement = -12,  // 4 Cortex-M3 Memory Management Interrupt
  BusFault = -11,          // 5 Cortex-M3 Bus Fault Interrupt
  UsageFault = -10,        // 6 Cortex-M3 Usage Fault Interrupt
  SVCall = -5,             // 11 Cortex-M3 SV Call Interrupt
  DebugMonitor = -4,       // 12 Cortex-M3 Debug Monitor Interrupt
  PendSV = -2,             // 14 Cortex-M3 Pend SV Interrupt
  SysTick = -1,            // 15 Cortex-M3 System Tick Interrupt
  WDT = 0,                 // Watchdog Timer Interrupt
  TIMER0 = 1,              // Timer0 Interrupt
  TIMER1 = 2,              // Timer1 Interrupt
  TIMER2 = 3,              // Timer2 Interrupt
  TIMER3 = 4,              // Timer3 Interrupt
  UART0 = 5,               // UART0 Interrupt
  UART1 = 6,               // UART1 Interrupt
  UART2 = 7,               // UART2 Interrupt
  UART3 = 8,               // UART3 Interrupt
  PWM1 = 9,                // PWM1 Interrupt
  I2C0 = 10,               // I2C0 Interrupt
  I2C1 = 11,               // I2C1 Interrupt
  I2C2 = 12,               // I2C2 Interrupt
  SSP0 = 14,               // SSP0 Interrupt
  SSP1 = 15,               // SSP1 Interrupt
  PLL0 = 16,               // PLL0 Lock (Main PLL) Interrupt
  RTC = 17,                // Real Time Clock Interrupt
  EINT0 = 18,              // External Interrupt 0 Interrupt
  EINT1 = 19,              // External Interrupt 1 Interrupt
  EINT2 = 20,              // External Interrupt 2 Interrupt
  EINT3 = 21,              // External Interrupt 3 Interrupt
  ADC = 22,                // A/D Converter Interrupt
  BOD = 23,                // Brown-Out Detect Interrupt
  USB = 24,                // USB Interrupt
  CAN = 25,                // CAN Interrupt
  DMA = 26,                // General Purpose DMA Interrupt
  I2S = 27,                // I2S Interrupt
  ENET = 28,               // Ethernet Interrupt
  MCI = 29,                // SD/MMC card I/F Interrupt
  MCPWM = 30,              // Motor Control PWM Interrupt
  QEI = 31,                // Quadrature Encoder Interface Interrupt
  PLL1 = 32,               // PLL1 Lock (USB PLL) Interrupt
  USBActivity = 33,        // USB Activity interrupt
  CANActivity = 34,        // CAN Activity interrupt
  UART4 = 35,              // UART4 Interrupt
  SSP2 = 36,               // SSP2 Interrupt
  LCD = 37,                // LCD Interrupt
  GPIO = 38,               // GPIO Interrupt
  PWM0 = 39,               // PWM0 Interrupt
  EEPROM = 40,             // EEPROM Interrupt
};

} // namespace LPC4078
