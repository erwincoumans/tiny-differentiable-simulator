/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TINY_DUAL_DOUBLE_UTILS_H
#define TINY_DUAL_DOUBLE_UTILS_H

#include <math.h>

#include "tiny_dual.h"

typedef ::TinyDual<double> TinyDualDouble;

struct TinyDualDoubleUtils {
  static TinyDualDouble cos1(TinyDualDouble v) {
    return TinyDualDouble(::cos(v.real()));
  }

  template <class T>
  static TinyDualDouble fraction(T, T) = delete;  // C++11

  static TinyDualDouble fraction(int num, int denom) {
    return TinyDualDouble(double(num) / double(denom));
  }

  static TinyDualDouble sin1(TinyDualDouble v) { return sin(v); }

  static TinyDualDouble sqrt1(TinyDualDouble z) {
    double x = ::sqrt(z.real());
    return TinyDualDouble(x, z.dual() / (2. * double(x)));
  }

  static TinyDualDouble zero() { return TinyDualDouble(0.); }
  static TinyDualDouble one() { return TinyDualDouble(1.); }

  static TinyDualDouble two() { return TinyDualDouble(2.); }
  static TinyDualDouble half() { return TinyDualDouble(0.5); }
  static TinyDualDouble pi() { return TinyDualDouble(M_PI); }
  static TinyDualDouble half_pi() { return TinyDualDouble(M_PI / 2.); }

  template <class T>
  static T sqrt1(T v) {
    return sqrt(v);
  }

  static double getDouble(TinyDualDouble v) { return (double)v.real(); }

  template <class T>
  static double getDouble(T v) {
    return (double)v.real();
  }

  template <class T>
  static TinyDualDouble convert(T) = delete;  // C++11

  static TinyDualDouble convert(int value) { return TinyDualDouble(value); }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};

#endif  // TINY_DUAL_DOUBLE_UTILS_H
