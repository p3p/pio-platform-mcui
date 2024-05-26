#include "uart.h"
#include "gpio.h"

namespace LPC4078 {

extern "C" void UART0_IRQHandler() {
  BufferedUARTC::call_isr(0);
}

extern "C" void UART1_IRQHandler() {
  BufferedUARTC::call_isr(1);
}

extern "C" void UART2_IRQHandler() {
  BufferedUARTC::call_isr(2);
}

extern "C" void UART3_IRQHandler() {
  BufferedUARTC::call_isr(3);
}

extern "C" void UART4_IRQHandler() {
  BufferedUARTC::call_isr(4);
}

}
