#pragma once

// Global PSRAM Compatibility Header — prevents build failures
// even if legacy esp_psram.h is referenced anywhere

#ifndef COMPAT_PSRAM_APPLIED
#define COMPAT_PSRAM_APPLIED

// Some firmwares include this header indirectly.
// We shadow it and provide safe replacements.

#if __has_include("esp_psram.h")
  #include "esp_psram.h"
#else
  // Replace obsolete APIs safely
  static bool psramFound() {
    return false;
  }
#endif

#endif
