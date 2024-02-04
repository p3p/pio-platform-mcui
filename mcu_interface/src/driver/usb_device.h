#pragma once

#include <interface/gpio.h>
#include <utility/const_functions.h>
#include <interface/time.h>

extern "C" void USB_Init();
extern "C" void USB_Connect(uint32_t);
extern uint32_t MSC_SD_Init(uint8_t);
extern void MSC_RunDeferredCommands();
extern uint32_t MSC_Release_Lock();
extern uint32_t MSC_Acquire_Lock();

extern "C" uint8_t USB_Configuration;

namespace LPC4078 {

struct USBDevice {
  static bool connect(size_t timeout_period, pin_t status_pin = P_NC) {
    USB_Init();
    USB_Connect(0);    // Clear possible connection
    time::delay(1000); // Give OS time to notice
    USB_Connect(1);    // Connect

    const uint32_t usb_timeout = time::millis() + timeout_period;
    volatile uint8_t& usb_ready = USB_Configuration;
    while (!usb_ready && MCUI::util::pending(time::millis(), usb_timeout)) {
      time::delay(50);
      if (status_pin != P_NC) gpio_toggle(status_pin);
    }
    return usb_ready != 0;
  }

  static void disconnect() {
    USB_Connect(0);
  }

  static bool msc_storage_init(uint8_t physical_drive_id = 0) {
    return !MSC_SD_Init(physical_drive_id);
  }

  static void msc_run_deferred_commands() {
    MSC_RunDeferredCommands();
  }

  static uint32_t msc_storage_lock() {
    return MSC_Acquire_Lock();
  }

  static uint32_t msc_storage_release() {
    return MSC_Release_Lock();
  }

};

} // namespace LPC4078
