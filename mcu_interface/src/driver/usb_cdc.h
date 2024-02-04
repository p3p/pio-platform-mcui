#pragma once

#include <utility/ring_buffer.h>

extern void CDC_FlushBuffer();

namespace LPC4078 {

struct USBCDCBuffer {
  MCUI::util::RingBuffer<uint8_t, 512> receive_buffer;
  MCUI::util::RingBuffer<uint8_t, 512> transmit_buffer;
  bool host_connected = false;
  uint32_t state = 0;
};

extern USBCDCBuffer usbcdc_buffer;

struct CDCSerial0 {
  static size_t available() { return  usbcdc_buffer.receive_buffer.available(); }
  static bool peek( uint8_t* value ) { return usbcdc_buffer.receive_buffer.peek(value); }
  static size_t read(uint8_t *const dst, size_t length) { return usbcdc_buffer.receive_buffer.read(dst, length); }
  static size_t write(uint8_t const* src, size_t length) { auto res = usbcdc_buffer.transmit_buffer.write(src, length); CDC_FlushBuffer(); return res;}
  static bool connected() { return usbcdc_buffer.host_connected; }
};

} // namespace LPC4078
