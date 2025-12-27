#pragma once

// Global PSRAM Compatibility Header — prevents build failures
// even if legacy esp_psram.h is referenced anywhere

#ifndef COMPAT_PSRAM_APPLIED
#define COMPAT_PSRAM_APPLIED

// Some firmwares include this header indirectly.
// We shadow it and provide safe replacements.

#if __has_include("esp_psram.h")
  #include "esp_psram.h"
#elif defined(ARDUINO_ARCH_ESP32)
  // The ESP32 Arduino core exposes psramFound() via esp32-hal-psram.h.
  // Avoid providing a conflicting C++ fallback when the core will supply it.
  extern "C" bool psramFound();
#else
  // Replace obsolete APIs safely for non-ESP32 targets.
  static inline bool psramFound() {
    return false;
  }
#endif

#endif
