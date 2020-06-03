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

#ifndef CPPAD_UTILS_H
#define CPPAD_UTILS_H

#include <assert.h>
#include <stdio.h>

#include <cppad/cppad.hpp>
#include "math.h"

/**
 * Supporting functions for automatic differentiation with CppAD.
 * https://coin-or.github.io/CppAD/doc/cppad.htm
 */
template <typename InnerScalar = double>
struct CppADUtils {
  typedef typename CppAD::AD<InnerScalar> Scalar;

  static Scalar zero() { return Scalar(0.); }
  static Scalar one() { return Scalar(1.); }

  static Scalar two() { return Scalar(2.); }
  static Scalar half() { return Scalar(0.5); }
  static Scalar pi() { return Scalar(M_PI); }
  static Scalar half_pi() { return Scalar(M_PI / 2.); }

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

  template <class T>
  static T atan2(const T& dy, const T& dx) {
    using std::atan2;
    return atan2(dy, dx);
  }

  static double getDouble(const Scalar& v) { return CppAD::Value(v); }
  template <class T>
  static double getDouble(const T& v) {
    return CppAD::Value(v);
  }

  template <class T>
  static Scalar convert(T) = delete;  // C++11

  static Scalar convert(int value) { return Scalar(double(value)); }

  template <class T>
  static Scalar fraction(T, T) = delete;  // C++11

  static Scalar fraction(int num, int denom) {
    return Scalar(double(num) / double(denom));
  }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};

#endif  // CPPAD_UTILS_H
