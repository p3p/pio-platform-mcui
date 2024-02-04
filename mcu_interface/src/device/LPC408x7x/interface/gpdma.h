#pragma once

#include <cstdint>
using std::size_t;

namespace LPC4078 {

struct GPDMARegion {
  uint32_t IntStat;
  uint32_t IntTCStat;
  uint32_t IntTCClear;
  uint32_t IntErrStat;
  uint32_t IntErrClr;
  uint32_t RawIntTCStat;
  uint32_t RawIntErrStat;
  uint32_t EnbldChns;
  uint32_t SoftBReq;
  uint32_t SoftSReq;
  uint32_t SoftLBReq;
  uint32_t SoftLSReq;
  uint32_t Config;
  uint32_t Sync;
};
static_assert(sizeof(GPDMARegion) == 0x38);

constexpr uintptr_t gpdma_device_address = 0x20080000;
static inline volatile auto& gpdma_device = *reinterpret_cast<volatile GPDMARegion*>(gpdma_device_address);

struct GPDMAChannelRegion {
  uint32_t CSrcAddr;
  uint32_t CDestAddr;
  uint32_t CLLI;
  uint32_t CControl;
  uint32_t CConfig;
};
static_assert(sizeof(GPDMAChannelRegion) == 0x14);

constexpr size_t gpdma_channel_count = 8;
constexpr uintptr_t gpdma_channel_address[gpdma_channel_count] = {
  0x20080100,
  0x20080120,
  0x20080140,
  0x20080160,
  0x20080180,
  0x200801A0,
  0x200801C0,
  0x200801E0
};

static inline volatile GPDMAChannelRegion* const gpdma_channel[gpdma_channel_count] = {
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[0]),
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[1]),
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[2]),
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[3]),
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[4]),
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[5]),
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[6]),
  reinterpret_cast<volatile GPDMAChannelRegion* const>(gpdma_channel_address[7])
};

};
