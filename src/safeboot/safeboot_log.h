// Safeboot console: UART0 on every board, mirrored onto the native USB CDC on
// the ESP32-S3 when a host is attached.
//
// The S3 safeboot image is shared between boards whose USB is a CP2102 on
// UART0 (Heltec V3) and boards with native USB only (T-Deck, T-Deck Plus).
// With ARDUINO_USB_CDC_ON_BOOT=0 the core's Serial is UART0, so on the T-Deck
// nothing the safeboot prints ever reaches the USB port (bench 2026-09-13:
// zero [SAFEBOOT] lines in a full run, every HTTP assertion green). Turning
// CDC_ON_BOOT on would make Serial the CDC and silence the Heltec's UART log
// instead. This tee writes both: UART0 always, HWCDC when connected. An
// unconnected HWCDC only pushes into its ring buffer (no blocking), so the
// Heltec pays nothing.
//
// Include this header LAST in a safeboot translation unit: it renames Serial.
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

#if defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && SOC_USB_SERIAL_JTAG_SUPPORTED && defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE && !ARDUINO_USB_CDC_ON_BOOT
#include <HWCDC.h>
#define SAFEBOOT_LOG_TEE 1

class SafebootTee : public Print {
public:
  void begin(unsigned long baud) {
    Serial0.begin(baud);
    usb_.begin();
  }
  size_t write(uint8_t c) override {
    Serial0.write(c);
    if (usb_.isConnected()) usb_.write(c);
    return 1;
  }
  size_t write(const uint8_t *buf, size_t n) override {
    Serial0.write(buf, n);
    if (usb_.isConnected()) usb_.write(buf, n);
    return n;
  }
  void flush() override { Serial0.flush(); }
  void setDebugOutput(bool en) { Serial0.setDebugOutput(en); }
  int available() { return Serial0.available() + (usb_.isConnected() ? usb_.available() : 0); }

private:
  HWCDC usb_;
};

extern SafebootTee SafebootSerial;
#define Serial SafebootSerial
#else
#define SAFEBOOT_LOG_TEE 0
#endif
