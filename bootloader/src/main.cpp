#include <mcu_interface.h>

#include <chanfs/ff.h>

#include <charconv>
#include <array>

struct pin_state_t {
  enum State: uint16_t {
    Low,
    High
  };
  pin_t pin;
  State state = Low;
};

constexpr uintptr_t user_flash_start = 0x4000;
constexpr pin_t board_status_led = P3_18;
constexpr pin_t board_uart_tx = P0_02;
constexpr pin_t board_uart_rx = P0_03;
constexpr uint32_t board_uart_baud = 115200;
const char* board_firmware_bin = "firmware.bin";
const char* board_firmware_cur = "firmware.cur";
constexpr const std::array<pin_state_t, 13> board_safe_pin_states { pin_state_t{P1_02}, {P1_10}, {P4_30}, {P1_09}, {P4_23}, {P3_08}, {P3_00}, {P3_27}, {P5_04}, {P1_08}, {P4_31}, {P3_01}, {P3_10} };
constexpr bool board_catch_watchdog_resets = true;

constexpr size_t flash_block_size(size_t block_index) { return block_index < 16 ? 4096 : 32768; }
constexpr size_t flash_addr_to_block(uintptr_t address) {
  if ( address < 0xFFFF ) return address / 4096;
  address -= 0xFFFF;
  return (address / 32768) + 16;
}

MCUI::RawUARTC Serial0(0);

[[noreturn]] void boot() {
  typedef void __attribute__((noreturn))(*user_reset_handler)();
  MCUCore::nvic_interrupts_disable();
  MCUCore::system_control_block->VTOR = (user_flash_start & 0x1FFFFF80);
  MCUCore::nvic_interrupts_enable();

  MCUCore::DMB();
  MCUCore::DSB();
  MCUCore::ISB();
  Serial0.write("boot: jumping to usercode\n");
  MCUCore::MSP(*(uint32_t *)user_flash_start);
  (*(user_reset_handler*)(user_flash_start + 4))();
}

bool flash(FIL& file) {
  char buffer[10];

  auto binary_size = f_size(&file);
  Serial0.write("boot: firmware size: ");
  auto res = std::to_chars(std::begin(buffer), std::end(buffer), binary_size);
  Serial0.write(buffer, res.ptr - buffer);
  Serial0.write("B\n");

  Serial0.write("boot: flashing to 0x");
  res = std::to_chars(std::begin(buffer), std::end(buffer), user_flash_start, 16);
  Serial0.write(buffer, res.ptr - buffer);
  Serial0.write("\n");

  size_t start_sector = flash_addr_to_block(user_flash_start);
  size_t end_sector = flash_addr_to_block(user_flash_start + binary_size);

  auto iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::PrepareSectorForWrite, start_sector, end_sector);
  if (iap_res.status != MCUI::IAP::StatusCode::Success) {
    Serial0.write("boot: IAP::PrepareSectorForWrite failed, error code: ");
    res = std::to_chars(std::begin(buffer), std::end(buffer), MCUI::util::to_integral(iap_res.status));
    Serial0.write(buffer, res.ptr - buffer);
    Serial0.write("\n");
    return false;
  }

  iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::EraseSector, start_sector, end_sector, MCUI::system_clock() / 1000);
  if (iap_res.status != MCUI::IAP::StatusCode::Success) {
    Serial0.write("boot: IAP::EraseSector failed, error code: ");
    res = std::to_chars(std::begin(buffer), std::end(buffer), MCUI::util::to_integral(iap_res.status));
    Serial0.write(buffer, res.ptr - buffer);
    Serial0.write("\n");
    return false;
  }

  Serial0.write("boot: sector range (");
  res = std::to_chars(std::begin(buffer), std::end(buffer), start_sector);
  Serial0.write(buffer, res.ptr - buffer);
  Serial0.write(" - ");
  res = std::to_chars(std::begin(buffer), std::end(buffer), end_sector);
  Serial0.write(buffer, res.ptr - buffer);
  Serial0.write(") erased\n");

  char chunk_buffer[512];
  UINT bytes_read = 0;
  uintptr_t flash_offset = user_flash_start;
  while (!f_eof(&file)) {
    f_read(&file, chunk_buffer, 512, &bytes_read);

    size_t sector = flash_addr_to_block(flash_offset);
    iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::PrepareSectorForWrite, sector, sector);
    if (iap_res.status != MCUI::IAP::StatusCode::Success) {
      Serial0.write("boot: IAP::PrepareSectorForWrite failed, error code: ");
      res = std::to_chars(std::begin(buffer), std::end(buffer), MCUI::util::to_integral(iap_res.status));
      Serial0.write(buffer, res.ptr - buffer);
      Serial0.write("\n");
      return false;
    }

    iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::CopyRAMtoFLASH, flash_offset, (uint32_t)chunk_buffer, 512UL, MCUI::system_clock() / 1000);
    if (iap_res.status != MCUI::IAP::StatusCode::Success) {
      Serial0.write("boot: IAP::CopyRAMtoFLASH failed, error code: ");
      res = std::to_chars(std::begin(buffer), std::end(buffer), MCUI::util::to_integral(iap_res.status));
      Serial0.write(buffer, res.ptr - buffer);
      Serial0.write("\n");
      return false;
    }
    flash_offset += bytes_read;
    MCUI::gpio_set(board_status_led, flash_offset & 0x8000 );
  }
  Serial0.write("boot: flash write sequence complete\n");

  f_rewind(&file);
  flash_offset = user_flash_start;
  while (!f_eof(&file)) {
    f_read(&file, chunk_buffer, 512, &bytes_read);
    iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::Compare, flash_offset, (uint32_t)chunk_buffer, 512UL);
    if (iap_res.status != MCUI::IAP::StatusCode::Success) {
      Serial0.write("boot: IAP::Compare failed, error code: ");
      res = std::to_chars(std::begin(buffer), std::end(buffer), MCUI::util::to_integral(iap_res.status));
      Serial0.write(buffer, res.ptr - buffer);
      Serial0.write("\n");
      Serial0.write("boot: write validation failed @ 0x");
      res = std::to_chars(std::begin(buffer), std::end(buffer), flash_offset + iap_res.value[0], 16);
      Serial0.write(buffer, res.ptr - buffer);
      Serial0.write("\n");
      return false;
    }
    flash_offset += bytes_read;
  }
  Serial0.write("boot: flash validation complete\n");

  return true;
}

int main() {
  for (auto& pin : board_safe_pin_states) {
    MCUI::gpio_set_output(pin.pin);
    MCUI::gpio_set(pin.pin, pin.state);
  }

  MCUI::gpio_set_output(board_status_led);
  MCUI::gpio_clear(board_status_led);

  Serial0.configure_pins(board_uart_rx, board_uart_tx);
  Serial0.init({ .baud = board_uart_baud });
  Serial0.write("boot: bootloader started\n");

  auto part_id = MCUI::IAP::device_part_id();
  Serial0.write("boot: device part id [");
  Serial0.write(part_id.name);
  Serial0.write("]\n");

  char buffer[16];
  auto serial_number = MCUI::IAP::device_serial_number();
  if (serial_number.status == MCUI::IAP::StatusCode::Success) {
    Serial0.write("boot: device serial [");
    char buffer[16];
    std::to_chars(std::begin(buffer), std::end(buffer), serial_number.value[0], 16);
    std::to_chars(std::begin(buffer) + 4, std::end(buffer), serial_number.value[1], 16);
    std::to_chars(std::begin(buffer) + 8, std::end(buffer), serial_number.value[2], 16);
    std::to_chars(std::begin(buffer) + 12, std::end(buffer), serial_number.value[3], 16);
    Serial0.write(buffer, 16);
    Serial0.write("]\n");
  } else {
    Serial0.write("boot: device serial query failed \n");
  }

  if (board_catch_watchdog_resets && MCUI::watchdog_has_triggered()) {
    Serial0.write("boot: user code caused watchdog timeout\n");
    Serial0.write("boot: press space to continue\n");
    MCUI::watchdog_clear_timeout_flag();
    while(true) {
      if (Serial0.rx_available()) {
        Serial0.read(buffer);
        if (buffer[0] == ' ') break;
      }
    }
  }

  FATFS filesystem;
  FIL file;

  f_mount(&filesystem, "", 0);
  FRESULT fr = f_open(&file, board_firmware_bin, FA_READ);
  if (fr == FR_OK) {
    Serial0.write("boot: new firmware binary found\n");
    bool flashed = flash(file);
    fr = f_close(&file);
    if (flashed) {
      f_unlink(board_firmware_cur);
      f_rename(board_firmware_bin, board_firmware_cur);
      Serial0.write("boot: flash complete, firmware binary backed up to ");
      Serial0.write(board_firmware_cur);
      Serial0.write("\n");
    }
  } else if (fr == FR_NOT_READY) {
    Serial0.write("boot: no sd card\n");
  } else if (fr == FR_NO_FILE) {
    Serial0.write("boot: no new firmware\n");
  } else {
    Serial0.write("boot: error checking sdcard (error code: ");
    auto res = std::to_chars(std::begin(buffer), std::end(buffer), fr);
    Serial0.write(buffer, res.ptr - buffer);
    Serial0.write(")\n");
  }
  f_unmount("");

  boot();
}
