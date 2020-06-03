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

#ifndef CERES_UTILS_H
#define CERES_UTILS_H

#include <assert.h>
#include <stdio.h>

#include <ceres/jet.h>
#include "math.h"

/**
 * Supporting functions for automatic differentiation with Ceres dual numbers
 * (i.e. "jets").
 * http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
 *
 * Supports ceres::Jet<double, N> for N partial derivatives.
 */
template <int N>
struct CeresUtils {
  typedef ceres::Jet<double, N> Jet;

  static Jet zero() { return Jet(0.); }
  static Jet one() { return Jet(1.); }

  static Jet two() { return Jet(2.); }
  static Jet half() { return Jet(0.5); }
  static Jet pi() { return Jet(M_PI); }
  static Jet half_pi() { return Jet(M_PI / 2.); }

  template <class T>
  static T cos1(const T& v) {
    using std::cos;
    return cos(v);
  }

  template <class T>
  static T sin1(const T& v) {
    using std::sin;
    return sin(v);
  }

  template <class T>
  static T sqrt1(const T& v) {
    using std::sqrt;
    return sqrt(v);
  }

  static double getDouble(const Jet& v) { return v.a; }
  template <class T>
  static double getDouble(const T& v) {
    return v;
  }

  template <class T>
  static Jet convert(T) = delete;  // C++11

  static Jet convert(int value) { return Jet(double(value)); }

  template <class T>
  static Jet fraction(T, T) = delete;  // C++11

  static Jet fraction(int num, int denom) {
    return Jet(double(num) / double(denom));
  }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};

#endif  // CERES_UTILS_H
