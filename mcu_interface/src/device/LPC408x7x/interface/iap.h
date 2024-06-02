#pragma once

#include <cstdint>
#include <mcu_interface.h>

namespace LPC4078 {

constexpr uintptr_t iap_location = 0x1FFF1FF1;
typedef void (*IAPEntry)(uint32_t*, uint32_t*);
static IAPEntry iap_entry = reinterpret_cast<IAPEntry>(iap_location);
constexpr const char* iap_part_id_string[] = {"UNKNOWN", "LPC4088", "LPC4078", "LPC4076", "LPC4074"};
constexpr inline const char* iap_part_id_lookup(uint32_t id) {
  switch(id) {
    case 0x481D3F47: return iap_part_id_string[1];
    case 0x47193F47: return iap_part_id_string[2];
    case 0x47191F43: return iap_part_id_string[3];
    case 0x47011132: return iap_part_id_string[4];
  }
  return iap_part_id_string[0];
}

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
    uint32_t args[4] {};
  };

  struct CommandResult {
    StatusCode status {StatusCode::Invalid};
    uint32_t value[4] {};
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

  struct PartID {
    const char* name;
    uint32_t id;
  };

  static inline PartID device_part_id() {
    auto res = execute(CommandID::ReadPartID);
    return { iap_part_id_lookup(res.value[0]), res.value[0] };
  }

  struct DeviceSerial {
    uint32_t value[4] {};
  };

  static inline DeviceSerial device_serial_number() {
    auto res = execute(CommandID::ReadSerialNumber);
    return {res.value[0], res.value[1], res.value[2], res.value[3]};
  }

};

} // namespace LPC4078
