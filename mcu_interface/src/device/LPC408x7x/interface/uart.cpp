#include "uart.h"
#include "gpio.h"

namespace LPC4078 {

extern "C" void UART0_IRQHandler() {
  BufferedUARTC::call_isr(0);
}

}
