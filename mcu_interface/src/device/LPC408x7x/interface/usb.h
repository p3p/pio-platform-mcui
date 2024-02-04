#pragma once

#include <cstdint>

namespace LPC4078 {

struct USBRegion {
  uint32_t Revision; // USB Host Registers
  uint32_t Control;
  uint32_t CommandStatus;
  uint32_t InterruptStatus;
  uint32_t InterruptEnable;
  uint32_t InterruptDisable;
  uint32_t HCCA;
  uint32_t PeriodCurrentED;
  uint32_t ControlHeadED;
  uint32_t ControlCurrentED;
  uint32_t BulkHeadED;
  uint32_t BulkCurrentED;
  uint32_t DoneHead;
  uint32_t FmInterval;
  uint32_t FmRemaining;
  uint32_t FmNumber;
  uint32_t PeriodicStart;
  uint32_t LSTreshold;
  uint32_t RhDescriptorA;
  uint32_t RhDescriptorB;
  uint32_t RhStatus;
  uint32_t RhPortStatus1;
  uint32_t RhPortStatus2;
  uint32_t RESERVED0[40];
  uint32_t Module_ID;

  uint32_t IntSt; // USB On-The-Go Registers
  uint32_t IntEn;
  uint32_t IntSet;
  uint32_t IntClr;
  uint32_t StCtrl;
  uint32_t Tmr;
  uint32_t RESERVED1[58];

  uint32_t DevIntSt; // USB Device Interrupt Registers
  uint32_t DevIntEn;
  uint32_t DevIntClr;
  uint32_t DevIntSet;

  uint32_t CmdCode; // USB Device SIE Command Registers
  uint32_t CmdData;

  uint32_t RxData; // USB Device Transfer Registers
  uint32_t TxData;
  uint32_t RxPLen;
  uint32_t TxPLen;
  uint32_t Ctrl;
  uint32_t DevIntPri;

  uint32_t EpIntSt; // USB Device Endpoint Interrupt Regs
  uint32_t EpIntEn;
  uint32_t EpIntClr;
  uint32_t EpIntSet;
  uint32_t EpIntPri;

  uint32_t ReEp; // USB Device Endpoint Realization Reg
  uint32_t EpInd;
  uint32_t MaxPSize;

  uint32_t DMARSt; // USB Device DMA Registers
  uint32_t DMARClr;
  uint32_t DMARSet;
  uint32_t RESERVED2[9];
  uint32_t UDCAH;
  uint32_t EpDMASt;
  uint32_t EpDMAEn;
  uint32_t EpDMADis;
  uint32_t DMAIntSt;
  uint32_t DMAIntEn;
  uint32_t RESERVED3[2];
  uint32_t EoTIntSt;
  uint32_t EoTIntClr;
  uint32_t EoTIntSet;
  uint32_t NDDRIntSt;
  uint32_t NDDRIntClr;
  uint32_t NDDRIntSet;
  uint32_t SysErrIntSt;
  uint32_t SysErrIntClr;
  uint32_t SysErrIntSet;
  uint32_t RESERVED4[15];

  union {
    uint32_t I2C_RX; // USB OTG I2C Registers
    uint32_t I2C_TX;
  };
  uint32_t I2C_STS;
  uint32_t I2C_CTL;
  uint32_t I2C_CLKHI;
  uint32_t I2C_CLKLO;
  uint32_t RESERVED5[824];

  union {
    uint32_t ClkCtrl; // USB Clock Control Registers
    uint32_t OTGClkCtrl;
  };
  union {
    uint32_t ClkSt;
    uint32_t OTGClkSt;
  };
};

constexpr uintptr_t usb_address = 0x2008C000;
static inline volatile USBRegion& usb = *reinterpret_cast<volatile USBRegion* const>(usb_address);

} // namespace LPC4078
