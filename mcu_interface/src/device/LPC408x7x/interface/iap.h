#pragma once

#include <cstdint>
#include <mcu_interface.h>

namespace LPC4078 {

constexpr uintptr_t iap_location = 0x1FFF1FF1;
typedef void (*IAPEntry)(uint32_t*, uint32_t*);
static IAPEntry iap_entry = reinterpret_cast<IAPEntry>(iap_location);

struct IAP {
  enum class CommandID : uint32_t {
    Invalid = 0,
    PrepareSectorForWrite = 50,
    CopyRAMtoFLASH,
    EraseSector,
    BlankCheckSector,
    ReadPartID,
    ReadBootCodeVersion,
    Compare,
    ReinvokeISP,
    ReadSerialNumber,
  };

  enum class StatusCode : uint32_t {
    Success,
    InvalidCommand,
    SrcAddrError,
    DstAddrError,
    SrcAddrNotMapped,
    DstAddrNotMapped,
    CountError,
    InvalidSector,
    SectorNotBlank,
    SectorNotPreparedForWrite,
    CompareError,
    Busy,
    Invalid,
  };

  struct Command {
    CommandID command {CommandID::Invalid};
    uint32_t args[4] {0, 0, 0, 0};
  };

  struct CommandResult {
    StatusCode status {StatusCode::Invalid};
    uint32_t value[4] {0, 0, 0, 0};
  };

  static inline void call(Command&& command, CommandResult& output) {
    iap_entry(reinterpret_cast<uint32_t*>(&command), reinterpret_cast<uint32_t*>(&output));
  }

  template <typename... Args>
  static CommandResult execute(CommandID command, Args... args) {
    CommandResult result;
    MCUCore::nvic_interrupts_disable();
    call({command, {args...}}, result);
    MCUCore::nvic_interrupts_enable();
    return result;
  };

  static inline CommandResult device_serial_number() {
    return execute(CommandID::ReadSerialNumber);
  }

};

} // namespace LPC4078
