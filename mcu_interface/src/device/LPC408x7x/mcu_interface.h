#pragma once

#include "utility/const_functions.h"
#include <mcu_core.h>

#include "interface/system_control.h"
#include "interface/pinmapping.h"
#include "interface/gpio.h"
#include "interface/adc.h"
#include "interface/pwm.h"
#include "interface/watchdog.h"
#include "interface/time.h"
#include "interface/uart.h"
#include "interface/usb.h"
#include "interface/timers.h"
#include "interface/ssp.h"
#include "interface/iap.h"
#include "interface/eeprom.h"
#include "interface/rtc.h"
#include "interface/i2c.h"
#include "interface/gpdma.h"

#include "driver/usb_device.h"
#include "driver/usb_cdc.h"

namespace MCUI {
  using namespace LPC4078;
}
