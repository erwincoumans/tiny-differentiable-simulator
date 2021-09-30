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

#ifdef USE_CPPAD_CODEGEN
  #include <cppad/cg.hpp>
#else
  #include <cppad/cppad.hpp>
#endif

#include <vector>
#include <string>

#include "math.h"

using std::vector;

/**
 * Supporting functions for automatic differentiation with CppAD.
 * https://coin-or.github.io/CppAD/doc/cppad.htm
 */
template <typename InnerScalar = double>
struct CppADUtils {
  typedef typename CppAD::AD<InnerScalar> Scalar;
#ifdef USE_CPPAD_CODEGEN
  static const bool is_codegen =
     std::is_same_v<InnerScalar, CppAD::cg::CG<double>>;
#else
  static const bool is_codegen = false;
#endif

  static Scalar zero() { return Scalar(0.); }
  static Scalar one() { return Scalar(1.); }

  static Scalar two() { return Scalar(2.); }
  static Scalar half() { return Scalar(0.5); }
  static Scalar pi() { return Scalar(M_PI); }
  static Scalar half_pi() { return Scalar(M_PI / 2.); }

  template <class T>
  static T cos1(const T& v) {
    using CppAD::cos;
    return cos(v);
  }

  template <class T>
  static T sin1(const T& v) {
    using CppAD::sin;
    return sin(v);
  }

  template <class T>
  static T sqrt1(const T& v) {
    using CppAD::sqrt;
    return sqrt(v);
  }

  template <class T>
  static T exp(const T& v) {
    using CppAD::exp;
    return exp(v);
  }

  template <class T>
  static T log(const T& v) {
    using CppAD::log;
    return log(v);
  }

  template <class T>
  static T pow(const T& v, const T& e) {
    using CppAD::pow;
    return pow(v, e);
  }

  template <class T>
  static T tanh(const T& v) {
    using CppAD::tanh;
    return tanh(v);
  }

  template <class T>
  static T atan2(const T& dy, const T& dx) {
    using CppAD::atan2;
    return atan2(dy, dx);
  }
  
  template<class T>
  static T asin(const T& z)
  {
     return CppAD::asin(z);
  }

  template <class T>
  static T min1(const T& a, const T& b) {
#if DEBUG
    printf("Called CppAD CondExpLt\n");
#endif
    return CppAD::CondExpLt(a, b, a, b);
  }

  template <class T>
  static T max1(const T& a, const T
  & b) {
#if DEBUG
    printf("Called CppAD CondExpGt\n");
#endif
    return CppAD::CondExpGt(a, b, a, b);
  }

  template <class T>
  static T abs(const T& a) {
    return CppAD::CondExpGt(a, zero(), a, -a);
  }

  static double getDouble(const Scalar& v) {
    if constexpr (is_codegen) {
      return CppAD::Value(v).getValue();
    } else {
      return CppAD::Value(v);
    }
  }

  template <class T>
  static double getDouble(const T& v) {
    return CppAD::Value(v);
  }

  template <class T>
  static Scalar convert(T) = delete;  // C++11

  static Scalar convert(int value) { return Scalar(double(value)); }

  template <class T>
  static Scalar scalar_from_double(const T& value) {
    return Scalar(double(value));
  }

  template <class T>
  static Scalar scalar_from_string(const T& value) {
    return Scalar(std::stod(value));
  }

  template <class T>
  static Scalar fraction(T, T) = delete;  // C++11

  static Scalar fraction(int num, int denom) {
    return Scalar(double(num) / double(denom));
  }

  static Scalar copy(Scalar v) {
      return Scalar(v);
  }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
  
};

namespace TinyAD 
{

  /* Auto diff functions */ 
  template <typename Scalar = double>
  inline vector<CppAD::AD<Scalar>> independent(vector<CppAD::AD<Scalar>>& v) {
    CppAD::Independent(v);
    return v; // You need to return v for Python
  }

  template <typename Scalar = double>
  inline vector<Scalar> compute_jacobian(const vector<CppAD::AD<Scalar>>& x, const vector<CppAD::AD<Scalar>>& y) {
    size_t nx = x.size();
    size_t ny = y.size();

    vector<Scalar> vx(nx);
    for (int i = 0; i < nx; i++) {
      vx[i] = CppAD::Value(x[i]);
    }

    CppAD::ADFun<Scalar> f(x, y);
    vector<Scalar> jac(nx * ny);
    jac = f.Jacobian(vx);
    return jac;
  }
  
  template <typename Scalar = double>
  inline void print_ad(const std::string& s, const CppAD::AD<Scalar>& v) {
    CppAD::PrintFor(CppAD::AD<Scalar>(-1), s.c_str(), v, "\n");
  }

} 

#endif  // CPPAD_UTILS_H
