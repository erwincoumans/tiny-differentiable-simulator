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

#ifndef TINY_DUAL_UTILS_H
#define TINY_DUAL_UTILS_H

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include "tiny_dual.h"

template <typename Scalar>
struct TinyDualUtils {
  using Dual = TINY::TinyDual<Scalar>;

  static Dual cos1(const Dual& v) { return Dual(::cos(v.real())); }

  template <class T>
  static Dual fraction(T, T) = delete;  // C++11

  static Dual fraction(int num, int denom) {
    return Dual(double(num) / double(denom));
  }

  static Dual sin1(const Dual& v) { return sin(v); }

  static Dual sqrt1(const Dual& z) {
    double x = ::sqrt(z.real());
    return Dual(x, z.dual() / (2. * double(x)));
  }

  static Dual abs(const Dual& v) {
    return Dual(std::abs(v.real()), std::abs(v.dual()));
  }

  static Dual zero() { return Dual(0.); }
  static Dual one() { return Dual(1.); }

  static Dual two() { return Dual(2.); }
  static Dual half() { return Dual(0.5); }
  static Dual pi() { return Dual(M_PI); }
  static Dual half_pi() { return Dual(M_PI / 2.); }

  template <class T>
  static T sqrt1(T v) {
    return sqrt(v);
  }

  static double getDouble(const Dual& v) { return (double)v.real(); }

  template <class T>
  static double getDouble(T v) {
    return (double)v.real();
  }

  static Dual scalar_from_double(double d) { return Dual(d); }
  static Dual scalar_from_string(const std::string& s) { return Dual(std::stod(s)); }

  template <class T>
  static Dual convert(T) = delete;  // C++11

  static Dual convert(int value) { return Dual(value); }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};

typedef TINY::TinyDual<double> TinyDualDouble;
typedef TinyDualUtils<double> TinyDualDoubleUtils;

#endif  // TINY_DUAL_UTILS_H
