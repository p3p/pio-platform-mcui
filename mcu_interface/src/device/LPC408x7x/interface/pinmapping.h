#pragma once

#include "pin_definitions.h"

#ifdef __cplusplus

#include <cstdint>
#include "pin_control.h"
#include "gpio.h"

using LPC4078::pin_t;

#else
#include <stdint.h>
typedef pin_t uint16_t;

#define MCUI_PINID_TO_PORT(P) (P >> 5)
#define MCUI_PINID_TO_PIN() (P & 0x1F)
#define MCUI_PIN_COUNT (PIN_MAX + 1)
#define MCUI_PORT_COUNT (PORT_MAX + 1)

#endif
