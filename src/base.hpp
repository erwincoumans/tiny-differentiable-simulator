#pragma once

#include <fenv.h>

#include <cstdio>
#include <string>
#include <cassert>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES 1
#endif
#include <cmath>
#undef min
#undef max

#if defined(_MSC_VER)
#define TINY_INLINE __forceinline
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
  fprintf(stderr,
          "Cannot set NaN trap: feenableexcept is not available with Visual "
          "Studio. Instead, compile via the /fp:except flag.");
#endif
}
}  // namespace tds

typedef void (*SubmitProfileTiming)(const char* profileName);
