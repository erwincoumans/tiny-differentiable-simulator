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


#ifndef _TINY_DUAL_DOUBLE_UTILS_H
#define _TINY_DUAL_DOUBLE_UTILS_H

#ifdef _WIN32 
//for M_PI
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include "tiny_dual.h"
#include <assert.h>
#include <stdio.h>
#include <string>

namespace TINY
{
  
typedef ::TINY::TinyDual<double> TinyDualDouble;

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

  static TinyDualDouble log(const TinyDualDouble& z)
  {
      return TinyDualDouble(::log(z.real()), z.dual() / z.real());
  }

  static TinyDualDouble pow(const TinyDualDouble& x, const TinyDualDouble& y)
  {
      return exp(log(x) * y);
  }

  static TinyDualDouble copy(TinyDualDouble v) {
      return v;
  }

  static TinyDualDouble exp(TinyDualDouble z)
  {
      double x = ::exp(z.real());
      return TinyDualDouble(x, z.dual() * x);
  }

  static TinyDualDouble tanh(const TinyDualDouble& z)
  {
      double x = ::tanh(z.real());
      return TinyDualDouble(x, z.dual() * (double(1) - x * x));
  }

  template <typename T> int sgn1(T val) {
      return (T(0) < val) - (val < T(0));
  }

  static TinyDualDouble abs(const TinyDualDouble& a)
  {
      double signa = (a.real() < 0) - (0 < a.real());
      return TinyDualDouble(::abs(a.real()), a.dual()  * signa);
  }

  static TinyDualDouble min1(const TinyDualDouble& a, const TinyDualDouble& b)
  {
      if (a.real() <= b.real())
      {
          return a;
      }
      else
      {
          return b;
      }
  }

  static TinyDualDouble max1(const TinyDualDouble& a, const TinyDualDouble& b)
  {
      if (a.real() >= b.real())
      {
          return a;
      }
      else
      {
          return b;
      }
  }

 static TinyDualDouble copysign(const TinyDualDouble& x, const TinyDualDouble& y)
 { 
     return TinyDualDouble(::copysign(x.real(), y.real()));
 }
  
 static TinyDualDouble asin(const TinyDualDouble& z)
 {
     return TinyDualDouble(::asin(z.real()), z.dual() / ::sqrt(double(1) - z.real() * z.real()));
 }

 static TinyDualDouble acos(const TinyDualDouble& z)
 {
     return TinyDualDouble(::acos(z.real()), -z.dual() / ::sqrt(double(1) - z.real() * z.real()));
 }

 static TinyDualDouble atan2(const TinyDualDouble& y, const TinyDualDouble& x)
 {
      TinyDualDouble z = y / x;
      bool neg_x = x.real() < 0;
      bool neg_y = y.real() < 0;

      double quadrant = neg_x ? (neg_y ? -M_PI : M_PI) : 0;
      return TinyDualDouble(atan(z.real()) + quadrant, z.dual() / (double(1) + (z.real()*z.real())));
  }

  static double getDouble(TinyDualDouble v) { return (double)v.real(); }

  static TinyDualDouble scalar_from_string(const std::string& txt) {
      double result = atof(txt.c_str());
      return TinyDualDouble(result);
  }

  template <class T>
  static double getDouble(T v) {
    return (double)v.real();
  }

  
  static TinyDualDouble scalar_from_double(double d) {
      return TinyDualDouble(d);
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
};
#endif  // _TINY_DUAL_DOUBLE_UTILS_H
