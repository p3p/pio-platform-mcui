// gcc-arm-none-eabi 11.3 introduced a new warning while linking for the os stubs in libnosys, the stubs, as expected, just return an error but the linker produces warnings anyway

// closer.c:(.text._close_r+0xc): warning: _close is not implemented and will always fail
// fstatr.c:(.text._fstat_r+0xe): warning: _fstat is not implemented and will always fail
// signalr.c:(.text._getpid_r+0x0): warning: _getpid is not implemented and will always fail
// isattyr.c:(.text._isatty_r+0xc): warning: _isatty is not implemented and will always fail
// signalr.c:(.text._kill_r+0xe): warning: _kill is not implemented and will always fail
// lseekr.c:(.text._lseek_r+0x10): warning: _lseek is not implemented and will always fail
// readr.c:(.text._read_r+0x10): warning: _read is not implemented and will always fail
// writer.c:(.text._write_r+0x10): warning: _write is not implemented and will always fail

#include "mcu_interface.h"


extern "C" {

[[gnu::weak]] int _close(int file) {
  return -1;
}

[[gnu::weak, gnu::used]] int _fstat(int file, void *st) {
  return 0;
}

[[gnu::weak]] int _getpid() {
  return 1;
}

[[gnu::weak, gnu::used]] int _isatty(int file) {
  return 1;
}

[[gnu::weak]] int _kill(int pid, int sig) {
  return -1;
}

[[gnu::weak]] int _lseek(int file, int ptr, int dir) {
  return 0;
}

[[gnu::weak]] int _read(int file, char *ptr, int len) {
  return 0;
}

[[gnu::weak]] int _write(int file, char *ptr, int len) {
  if (file == 1 || file == 2) { // stdout or stderr
    return MCUI::UART::write(0, ptr, len);
  }
  return 0;
}

// [[gnu::weak]] int _exit(int code) {
//   return 0;
// }

// [[gnu::weak]] int _sbrk() {
//   return 0;
// }

}
