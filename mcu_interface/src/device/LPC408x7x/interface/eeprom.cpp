#include "eeprom.h"

namespace LPC4078 {

extern "C" void EEPROM_IRQHandler() {
  EEPROM::irq_handler();
}

} // namespace LPC4078
