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
#include <ceres/jet.h>
#include <stdio.h>

#include "math.h"

/**
 * Supporting functions for automatic differentiation with Ceres dual numbers
 * (i.e. "jets").
 * http://ceres-solver.org/automatic_derivatives.html#dual-numbers-jets
 *
 * Supports ceres::Jet<InnerScalar, N> for N partial derivatives.
 */
template <int N, typename InnerScalar = double>
struct CeresUtils {
  typedef ceres::Jet<InnerScalar, N> Jet;

  static Jet zero() { return Jet(0.); }
  static Jet one() { return Jet(1.); }

  static Jet two() { return Jet(2.); }
  static Jet half() { return Jet(0.5); }
  static Jet pi() { return Jet(M_PI); }
  static Jet half_pi() { return Jet(M_PI / 2.); }

  template <class T>
  static T cos1(const T &v) {
    using std::cos;
    return cos(v);
  }

  template <class T>
  static T sin1(const T &v) {
    using std::sin;
    return sin(v);
  }

  template <class T>
  static T sqrt1(const T &v) {
    using std::sqrt;
    return sqrt(v);
  }

  template <class T>
  static T atan2(const T &dy, const T &dx) {
    using std::atan2;
    return atan2(dy, dx);
  }

  template <class T>
  static T asin(const T &v) {
    using std::asin;
    return asin(v);
  }

  template <class T>
  static T copysign(const T &x, const T &y) {
    return ::copysign(x, y);
  }

  template <class T>
  static T abs(const T &v) {
    return ceres::abs(v);
  }

  template <class T>
  static T pow(const T &a, const T &b) {
    return ceres::pow(a, b);
  }

  template <class T>
  static T exp(const T &v) {
    return ceres::exp(v);
  }

  template <class T>
  static T log(const T &v) {
    return ceres::log(v);
  }

  template <class T>
  static T tanh(const T &v) {
    return ceres::tanh(v);
  }

  template <class T>
  static T min1(const T &a, const T &b) {
    using std::min;
    return min(a, b);
  }

  template <class T>
  static T max1(const T &a, const T &b) {
    using std::max;
    return max(a, b);
  }

  static double getDouble(const Jet &v) { return v.a; }
  template <class T>
  static double getDouble(const T &v) {
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

  static Jet scalar_from_string(const std::string &txt) {
    double result = atof(txt.c_str());
    return Jet(result);
  }
  static inline Jet scalar_from_double(double value) { return Jet(value); }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};

#endif  // CERES_UTILS_H
