#pragma once

#include <cstdint>

namespace LPC4078 {

struct MCPWMRegion {
  uint32_t CON;
  uint32_t CON_SET;
  uint32_t CON_CLR;
  uint32_t CAPCON;
  uint32_t CAPCON_SET;
  uint32_t CAPCON_CLR;
  uint32_t TC0;
  uint32_t TC1;
  uint32_t TC2;
  uint32_t LIM0;
  uint32_t LIM1;
  uint32_t LIM2;
  uint32_t MAT0;
  uint32_t MAT1;
  uint32_t MAT2;
  uint32_t DT;
  uint32_t CP;
  uint32_t CAP0;
  uint32_t CAP1;
  uint32_t CAP2;
  uint32_t INTEN;
  uint32_t INTEN_SET;
  uint32_t INTEN_CLR;
  uint32_t CNTCON;
  uint32_t CNTCON_SET;
  uint32_t CNTCON_CLR;
  uint32_t INTF;
  uint32_t INTF_SET;
  uint32_t INTF_CLR;
  uint32_t CAP_CLR;
};
static_assert(sizeof(MCPWMRegion) == 0x78);

constexpr uintptr_t mcpwm_device_address = 0x400B8000;
static inline volatile auto& mcpwm_device = *reinterpret_cast<volatile MCPWMRegion*>(mcpwm_device_address);

}
