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

#ifndef STAN_DOUBLE_UTILS_H
#define STAN_DOUBLE_UTILS_H

#include <assert.h>
#include <stdio.h>

#include <stan/math.hpp>
#include <stan/math/fwd/mat.hpp>
#include <stan/math/fwd/scal.hpp>
#include <stan/math/prim/scal.hpp>

using stan::math::fvar;
typedef fvar<double> standouble;

struct StanDoubleUtils {
  static standouble cos1(standouble v) { return stan::math::cos(v); }

  static standouble sin1(standouble v) { return stan::math::sin(v); }

  static double getDouble(standouble v) { return v.val(); }

  static standouble zero() { return standouble(0.); }
  static standouble one() { return standouble(1.); }

  static standouble two() { return standouble(2.); }
  static standouble half() { return standouble(0.5); }
  static standouble pi() { return standouble(stan::math::pi()); }
  static standouble half_pi() { return standouble(0.5 * stan::math::pi()); }

  template <class T>
  static T sqrt1(T v) {
    return sqrt(v);
  }

  template <class T>
  static double convert(T) = delete;  // C++11

  static standouble convert(int value) { return standouble((double)value); }

  template <class T>
  static double fraction(T, T) = delete;  // C++11

  static standouble fraction(int num, int denom) {
    return double(num) / double(denom);
  }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};

#endif  // STAN_DOUBLE_UTILS_H
