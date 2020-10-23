#pragma once

#include <string>
#include <fenv.h>
#include <stdio.h>

#if defined(_MSC_VER)
#define TINY_INLINE inline
#else
#define TINY_INLINE __attribute__((always_inline)) inline
#endif

namespace tds {
TINY_INLINE void activate_nan_trap() {
#if !defined(_MSC_VER)
  #if __APPLE__
fprintf(stderr, "Cannot set NaN trap: feenableexcept is not available.");
  #else
  // Set NaN trap
  feenableexcept(FE_INVALID | FE_OVERFLOW);
  #endif
#else
	fprintf(stderr, "Cannot set NaN trap: feenableexcept is not available.");
#endif
}
}  // namespace tds

typedef void (*SubmitProfileTiming)(const std::string& profileName);
