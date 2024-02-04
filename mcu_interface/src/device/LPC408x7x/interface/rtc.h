#pragma once

#include <cstdint>

namespace LPC4078 {

struct RTCRegion {
  uint8_t  ILR;
  uint8_t  RESERVED0[7];
  uint8_t  CCR;
  uint8_t  RESERVED1[3];
  uint8_t  CIIR;
  uint8_t  RESERVED2[3];
  uint8_t  AMR;
  uint8_t  RESERVED3[3];
  uint32_t CTIME0;
  uint32_t CTIME1;
  uint32_t CTIME2;
  uint8_t  SEC;
  uint8_t  RESERVED4[3];
  uint8_t  MIN;
  uint8_t  RESERVED5[3];
  uint8_t  HOUR;
  uint8_t  RESERVED6[3];
  uint8_t  DOM;
  uint8_t  RESERVED7[3];
  uint8_t  DOW;
  uint8_t  RESERVED8[3];
  uint16_t DOY;
  uint16_t RESERVED9;
  uint8_t  MONTH;
  uint8_t  RESERVED10[3];
  uint16_t YEAR;
  uint16_t RESERVED11;
  uint32_t CALIBRATION;
  uint32_t GPREG0;
  uint32_t GPREG1;
  uint32_t GPREG2;
  uint32_t GPREG3;
  uint32_t GPREG4;
  uint8_t  RTC_AUXEN;
  uint8_t  RESERVED12[3];
  uint8_t  RTC_AUX;
  uint8_t  RESERVED13[3];
  uint8_t  ALSEC;
  uint8_t  RESERVED14[3];
  uint8_t  ALMIN;
  uint8_t  RESERVED15[3];
  uint8_t  ALHOUR;
  uint8_t  RESERVED16[3];
  uint8_t  ALDOM;
  uint8_t  RESERVED17[3];
  uint8_t  ALDOW;
  uint8_t  RESERVED18[3];
  uint16_t ALDOY;
  uint16_t RESERVED19;
  uint8_t  ALMON;
  uint8_t  RESERVED20[3];
  uint16_t ALYEAR;
  // Boot Event Registers
  uint16_t RESERVED21;
  uint32_t ERSTATUS;
  uint32_t ERCONTROL;
  uint32_t ERCOUNTERS;
  uint32_t RESERVED22;
  uint32_t ERFIRSTSTAMP0;
  uint32_t ERFIRSTSTAMP1;
  uint32_t ERFIRSTSTAMP2;
  uint32_t RESERVED23;
  uint32_t ERLASTSTAMP0;
  uint32_t ERLASTSTAMP1;
  uint32_t ERLASTSTAMP2;
};
static_assert(sizeof(RTCRegion) == 0xAC);

constexpr inline uintptr_t rtc_device_address = 0x40024000;
static inline volatile auto& rtc_device = *reinterpret_cast<volatile RTCRegion*>(rtc_device_address);

} // namespace LPC4078
