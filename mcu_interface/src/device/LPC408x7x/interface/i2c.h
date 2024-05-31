#pragma once

#include <cstdint>
using std::size_t;

namespace LPC4078 {

struct I2CRegion {
  uint32_t CONSET;      // 0x000, (RW), I2C Control Set Register
  uint32_t STAT;        // 0x004, (RO), I2C Status Register
  uint32_t DAT;         // 0x008, (RW), I2C Data Register
  uint32_t ADR0;        // 0x00C, (RW), I2C Slave Address Register 0
  uint32_t SCLH;        // 0x010, (RW), SCH Duty Cycle Register High Half Word
  uint32_t SCLL;        // 0x014, (RW), SCL Duty Cycle Register Low Half Word
  uint32_t CONCLR;      // 0x018, (WO), I2C Control Clear Register
  uint32_t MMCTRL;      // 0x01C, (RW), Monitor mode control register
  uint32_t ADR1;        // 0x020, (RW), I2C Slave Address Register 1
  uint32_t ADR2;        // 0x024, (RW), I2C Slave Address Register 2
  uint32_t ADR3;        // 0x028, (RW), I2C Slave Address Register 3
  uint32_t DATA_BUFFER; // 0x02C, (WO), Data buffer register
  uint32_t MASK0;       // 0x030, (RW), I2C Slave address mask register 0
  uint32_t MASK1;       // 0x034, (RW), I2C Slave address mask register 1
  uint32_t MASK2;       // 0x038, (RW), I2C Slave address mask register 2
  uint32_t MASK3;       // 0x03C, (RW), I2C Slave address mask register 3
};
static_assert(sizeof(I2CRegion) == 0x040);

constexpr size_t i2c_device_count = 3;
constexpr uintptr_t i2c_device_address[i2c_device_count] = { 0x4001C000, 0x4005C000, 0x400A0000 };
static inline volatile I2CRegion* const i2c_device[i2c_device_count] = {
  reinterpret_cast<volatile I2CRegion*>(i2c_device_address[0]),
  reinterpret_cast<volatile I2CRegion*>(i2c_device_address[1]),
  reinterpret_cast<volatile I2CRegion*>(i2c_device_address[2])
};

} // namespace LPC4078
