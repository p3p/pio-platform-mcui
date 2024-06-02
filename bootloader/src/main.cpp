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

template <typename T> struct as_hex {
  as_hex(T v) : value(v) {};
  T value;
};

char buffer[32];
template <typename V> [[gnu::always_inline]] inline void serial_print(as_hex<V> value) {
  auto res = std::to_chars(std::begin(buffer), std::end(buffer), value.value, 16);
  for (int i = res.ptr - buffer; i < ((int)sizeof(V) * 2); ++i) Serial0.write("0");
  Serial0.write(buffer, res.ptr - buffer);
}

template <typename V> [[gnu::always_inline]] inline void serial_print(V value) {
  auto res = std::to_chars(std::begin(buffer), std::end(buffer), value);
  Serial0.write(buffer, res.ptr - buffer);
}

template <typename T, size_t N> [[gnu::always_inline]] inline void serial_print(T (&str)[N]){
  Serial0.write(str, N - 1);
}

[[gnu::always_inline]] inline void serial_print(const char* c_str){
  Serial0.write(c_str);
}

template<typename... Args> [[gnu::always_inline]] inline void serial_print(Args&&... args) {
  ((serial_print(std::forward<Args>(args))),...);
}

[[noreturn]] void boot() {
  typedef void __attribute__((noreturn))(*user_reset_handler)();
  MCUCore::nvic_interrupts_disable();
  MCUCore::system_control_block->VTOR = (user_flash_start & 0x1FFFFF80);
  MCUCore::nvic_interrupts_enable();

  MCUCore::DMB();
  MCUCore::DSB();
  MCUCore::ISB();
  serial_print("boot: jumping to usercode\n");
  MCUCore::MSP(*(uint32_t *)user_flash_start);
  (*(user_reset_handler*)(user_flash_start + 4))();
}

bool flash(FIL& file) {
  auto binary_size = f_size(&file);
  serial_print("boot: firmware size: ", binary_size, "B\n");
  serial_print("boot: flashing to 0x", as_hex(user_flash_start), "\n");

  size_t start_sector = flash_addr_to_block(user_flash_start);
  size_t end_sector = flash_addr_to_block(user_flash_start + binary_size);

  auto iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::PrepareSectorForWrite, start_sector, end_sector);
  if (iap_res.status != MCUI::IAP::StatusCode::Success) {
    serial_print("boot: IAP::PrepareSectorForWrite failed, error code: ", MCUI::util::to_integral(iap_res.status), "\n");
    return false;
  }

  iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::EraseSector, start_sector, end_sector, MCUI::system_clock() / 1000);
  if (iap_res.status != MCUI::IAP::StatusCode::Success) {
    serial_print("boot: IAP::EraseSector failed, error code: ", MCUI::util::to_integral(iap_res.status), "\n");
    return false;
  }
  serial_print("boot: sector range (", start_sector, " - ", end_sector, ") erased\n");

  char chunk_buffer[512];
  UINT bytes_read = 0;
  uintptr_t flash_offset = user_flash_start;
  while (!f_eof(&file)) {
    f_read(&file, chunk_buffer, 512, &bytes_read);

    size_t sector = flash_addr_to_block(flash_offset);
    iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::PrepareSectorForWrite, sector, sector);
    if (iap_res.status != MCUI::IAP::StatusCode::Success) {
      serial_print("boot: IAP::PrepareSectorForWrite failed, error code: ", MCUI::util::to_integral(iap_res.status), "\n");
      return false;
    }

    iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::CopyRAMtoFLASH, flash_offset, (uint32_t)chunk_buffer, 512UL, MCUI::system_clock() / 1000);
    if (iap_res.status != MCUI::IAP::StatusCode::Success) {
      serial_print("boot: IAP::CopyRAMtoFLASH failed, error code: ", MCUI::util::to_integral(iap_res.status), "\n");
      return false;
    }

    flash_offset += bytes_read;
    MCUI::gpio_set(board_status_led, flash_offset & 0x8000 );
  }
  serial_print("boot: flash write sequence complete\n");

  f_rewind(&file);
  flash_offset = user_flash_start;
  while (!f_eof(&file)) {
    f_read(&file, chunk_buffer, 512, &bytes_read);
    iap_res = MCUI::IAP::execute(MCUI::IAP::CommandID::Compare, flash_offset, (uint32_t)chunk_buffer, 512UL);
    if (iap_res.status != MCUI::IAP::StatusCode::Success) {
      serial_print("boot: IAP::Compare failed, error code: ", MCUI::util::to_integral(iap_res.status), "\n");
      serial_print("boot: write validation failed @ 0x", as_hex(flash_offset + iap_res.value[0]), "\n");
      return false;
    }
    flash_offset += bytes_read;
  }
  serial_print("boot: flash validation complete\n");

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

  serial_print("boot: bootloader started\n");
  serial_print("boot: device part id [", MCUI::IAP::device_part_id().name, "]\n");

  auto serial_number = MCUI::IAP::device_serial_number();
  serial_print("boot: device serial [", as_hex(serial_number.value[0]), as_hex(serial_number.value[1]), as_hex(serial_number.value[2]), as_hex(serial_number.value[3]), "]\n");

  if (board_catch_watchdog_resets && MCUI::watchdog_has_triggered()) {
    serial_print("boot: user code caused watchdog timeout\n");
    serial_print("boot: press space to continue\n");
    MCUI::watchdog_clear_timeout_flag();
    char rx_buffer[16];
    while(true) {
      if (Serial0.rx_available()) {
        Serial0.read(rx_buffer);
        if (rx_buffer[0] == ' ') break;
      }
    }
  }

  FATFS filesystem;
  FIL file;

  f_mount(&filesystem, "", 0);
  FRESULT fr = f_open(&file, board_firmware_bin, FA_READ);
  if (fr == FR_OK) {
    serial_print("boot: new firmware binary found\n");
    bool flashed = flash(file);
    fr = f_close(&file);
    if (flashed) {
      f_unlink(board_firmware_cur);
      f_rename(board_firmware_bin, board_firmware_cur);
      serial_print("boot: flash complete, firmware binary backed up to ", board_firmware_cur, "\n");
    }
  } else if (fr == FR_NOT_READY) {
    serial_print("boot: no sd card\n");
  } else if (fr == FR_NO_FILE) {
    serial_print("boot: no new firmware\n");
  } else {
    serial_print("boot: error checking sdcard (error code: ", fr, ")\n");
  }
  f_unmount("");

  boot();
}
