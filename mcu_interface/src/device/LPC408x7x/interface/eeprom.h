#pragma once

#include <cstdint>
#include <mcu_interface.h>

namespace LPC4078 {

// UM10562
// 37.5 Register description
// Table 724. Register overview: EEPROM controller (base address 0x0020 0000)
struct EEPROMControlRegion {
  uint32_t CMD;        // RW 0x080 EEPROM command
  uint32_t ADDR;       // RW 0x084 EEPROM address
  uint32_t WDATA;      // WO 0x088 EEPROM write data
  uint32_t RDATA;      // RO 0x08C EEPROM read data
  uint32_t WSTATE;     // RW 0x090 EEPROM wait state
  uint32_t CLKDIV;     // RW 0x094 EEPROM clock divider
  uint32_t PWRDWN;     // RW 0x098 EEPROM power-down
  uint32_t RESERVED[975];
  uint32_t INTENCLR;   // WO 0xFD8 EEPROM interrupt enable clear
  uint32_t INTENSET;   // WO 0xFDC EEPROM interrupt enable set
  uint32_t INTSTAT;    // RO 0xFE0 EEPROM interrupt status
  uint32_t INTEN;      // RO 0xFE4 EEPROM interrupt enable
  uint32_t INTSTATCLR; // WO 0xFE8 EEPROM interrupt status clear
  uint32_t INTSTATSET; // WO 0xFEC EEPROM interrupt status set
};
static_assert(sizeof(EEPROMControlRegion) == 0xFF0 - 0x080);

constexpr uintptr_t eeprom_address = 0x00200080UL;
static inline volatile auto& eeprom_device = *reinterpret_cast<volatile EEPROMControlRegion*>(eeprom_address);

struct EEPROM {
  struct Command {
    enum CommandEnum : uint8_t {
      Read8,
      Read16,
      Read32,
      Write8,
      Write16,
      Write32,
      ErasePage,
      ProgramPage = ErasePage,
      Invalid,
      EnablePrefetch,
    };
  };

  static void init() {
    // make sure eeprom is not in power down state
    eeprom_device.PWRDWN = 0UL;

    // initialise clock divisor
    eeprom_device.CLKDIV = (SystemCoreClock / 375000UL) - 1UL;

    // initialise wait states for current base clock

    // calculate clocks per us
    uint32_t phase3 = (15UL * (SystemCoreClock / 1000000UL));
    uint32_t phase2 = (55UL * (SystemCoreClock / 1000000UL));
    uint32_t phase1 = (35UL * (SystemCoreClock / 1000000UL));

    // round up clocks per ns delay requirement and encode with -1 in register
    phase3 = ((phase3 + 999) / 1000) - 1;
    phase2 = ((phase2 + 999) / 1000) - 1;
    phase1 = ((phase1 + 999) / 1000) - 1;

    eeprom_device.WSTATE = phase3 | (phase2 << 8U) | (phase1 << 16U);

    //MCUCore::nvic_enable_irq(IRQNumber::EEPROM);
    //eeprom_device.INTENSET = (1UL << 26U) | (1UL << 28U);
  }

  static inline uint8_t read(uint32_t address) {
    eeprom_device.ADDR = address;
    eeprom_device.CMD = Command::Read8;
    await_read_write();
    return eeprom_device.RDATA;
  }

  static inline void read_page(uint32_t address, uint8_t* buffer, uint32_t buffer_size) {
    address &= 0xFFFFFFC0; // page address
    eeprom_device.ADDR = address;
    eeprom_device.CMD = Command::Read32 | Command::EnablePrefetch;
    for (uint32_t offset = 0; offset < page_size / sizeof(uint32_t) && offset < buffer_size / sizeof(uint32_t); ++offset) {
      reinterpret_cast<uint32_t*>(buffer)[offset] = eeprom_device.RDATA;
    }
    await_read_write();
  }

  static void read(uint32_t address, uint32_t count, uint8_t* buffer, uint32_t buffer_size) {
    if (buffer_size < count || address + count > eeprom_size) return;

    uint32_t command = 0;
    uint32_t stride = 0;
    if ((address & 1) || ((address + count) & 1)) {
      command = Command::Read8;
      stride = sizeof(uint8_t);
    } else if ((address & 2) || ((address + count) & 2)) {
      command = Command::Read16;
      stride = sizeof(uint16_t);
    } else {
      command = Command::Read32;
      stride = sizeof(uint32_t);
    }

    uint32_t last_chunk = UINT32_MAX;
    uint32_t buffer_index = 0;

    // TODO: investigate why the addr counter needs to be volatile because the optimizer hates me
    for (volatile uint32_t addr = address; addr < address + count; addr += stride ) {
      if ((addr & 0xFFFFFFC0) != last_chunk) {
        eeprom_device.ADDR = addr;
        eeprom_device.CMD = command | Command::EnablePrefetch;
        last_chunk = addr & 0xFFFFFFC0;
      }
      await_read_write();
      if (stride == sizeof(uint32_t)) reinterpret_cast<uint32_t*>(buffer)[buffer_index++] = eeprom_device.RDATA;
      else if (stride == sizeof(uint16_t)) reinterpret_cast<uint16_t*>(buffer)[buffer_index++] = eeprom_device.RDATA;
      else reinterpret_cast<uint8_t*>(buffer)[buffer_index++] = eeprom_device.RDATA;
    }
  }

  static inline void write(uint32_t address, uint8_t value) {
    eeprom_device.ADDR = address;
    eeprom_device.CMD = Command::Write8;
    eeprom_device.WDATA = value;
    await_read_write();
  }

  static inline void write_page(uint32_t address, uint8_t* buffer, uint32_t buffer_size) {
    address &= 0xFFFFFFC0; // page address
    eeprom_device.ADDR = address;
    eeprom_device.CMD = Command::Write32;
    for (uint32_t offset = 0, buffer_offset = 0; offset < page_size && offset < buffer_size; offset += sizeof(uint32_t), ++buffer_offset) {
      eeprom_device.WDATA = reinterpret_cast<uint32_t*>(buffer)[buffer_offset];
    }
    await_read_write();
    flush(address);
  }

  static void write(uint32_t address, uint32_t count, const uint8_t* buffer, uint32_t buffer_size) {
    if (buffer_size < count || address + count > eeprom_size) return;

    uint32_t command = 0;
    uint32_t stride = 0;
    if ((address & 1) || ((address + count) & 1)) {
      command = Command::Write8;
      stride = sizeof(uint8_t);
    } else if ((address & 2) || ((address + count) & 2)) {
      command = Command::Write16;
      stride = sizeof(uint16_t);
    } else {
      command = Command::Write32;
      stride = sizeof(uint32_t);
    }

    uint32_t last_chunk = UINT32_MAX;
    uint32_t buffer_index = 0;
    for (volatile uint32_t addr = address; addr < address + count; addr += stride ) {
      if ((addr & 0xFFFFFFC0) != last_chunk) {
        if (last_chunk != UINT32_MAX) flush(last_chunk);
        eeprom_device.ADDR = addr;
        eeprom_device.CMD = command;
        last_chunk = addr & 0xFFFFFFC0;
      }
      if (stride == sizeof(uint32_t)) eeprom_device.WDATA = reinterpret_cast<const uint32_t*>(buffer)[buffer_index++];
      else if (stride == sizeof(uint16_t)) eeprom_device.WDATA = reinterpret_cast<const uint16_t*>(buffer)[buffer_index++];
      else eeprom_device.WDATA = reinterpret_cast<const uint8_t*>(buffer)[buffer_index++];
      await_read_write();
    }
    flush(last_chunk);
  }

  static inline void flush(uint32_t address) {
    eeprom_device.ADDR = address;
    eeprom_device.CMD = Command::ProgramPage;
    await_program_erase();
  }

  static inline void await_read_write() {
    while((eeprom_device.INTSTAT & (1UL << 26U)) == 0);
    eeprom_device.INTSTATCLR = (1UL << 26U);
  }

  static inline void await_program_erase() {
    while((eeprom_device.INTSTAT & (1UL << 28U)) == 0);
    eeprom_device.INTSTATCLR = (1UL << 28U);
  }

  static inline void irq_handler() {}

  static constexpr uint32_t page_size = 64UL;
  static constexpr uint32_t page_count = 63UL;
  static constexpr uint32_t eeprom_size = page_size * page_count;
};

}; // namespace LPC4078
