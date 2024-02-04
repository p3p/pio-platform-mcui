#include "usb_cdc.h"

namespace LPC4078 {

[[gnu::section("AHBSRAM0")]] USBCDCBuffer usbcdc_buffer{};

} // namespace LPC4078
