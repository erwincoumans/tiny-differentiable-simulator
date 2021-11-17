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

#ifndef _TINY_FLOAT_UTILS_H
#define _TINY_FLOAT_UTILS_H

#define _USE_MATH_DEFINES 1
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "math.h"

#include "tiny_matrix3x3.h"
#include "tiny_pose.h"
#include "tiny_quaternion.h"
#include "tiny_vector3.h"


namespace TINY
{
struct FloatUtils {
  static float zero() { return 0.f; }
  static float one() { return 1.f; }

  static float two() { return 2.f; }
  static float half() { return 0.5f; }
  static float pi() { return float(M_PI); }
  static float half_pi() { return float(M_PI) / 2.f; }

  static float cos1(float v) { return ::cosf(v); }
  static float sin1(float v) { return ::sinf(v); }
  static float atan2(float dy, float dx) { return ::atan2f(dy, dx); }
  static float asin(float v) { return ::asinf(v); }
  static float copysign(float x, float y) { return ::copysignf(x, y); }
  static float abs(float v) { return ::fabsf(v); }
  static double tanh(float v) { return ::tanh(v); }
  static double log(float v) { return ::log(v); }
  static float exp(float v) { return ::exp(v); }
  static float pow(float a, float  b) { return ::pow(a, b); }
  template <class T>
  static float getDouble(T v) {
    return (float)v;
  }

  template <class T>
  static T sqrt1(T v) {
    return sqrt(v);
  }

  template <class T>
  static float getfloat(T v) {
    return (float)v;
  }

  template <class T>
  static float convert(T) = delete;  // C++11

  static float convert(int value) { return float(value); }

  template <class T>
  static float fraction(T, T) = delete;  // C++11

  static float fraction(int num, int denom) {
    return float(num) / float(denom);
  }

  static float scalar_from_string(const std::string& txt) {
    float result = atof(txt.c_str());
    return result;
  }

  static float scalar_from_double(double d) {
    return static_cast<float>(d);
  }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};


typedef ::TINY::TinyVector3<float, FloatUtils> TinyVector3f;
typedef ::TINY::TinyQuaternion<float, FloatUtils> TinyQuaternionf;
typedef ::TINY::TinyMatrix3x3<float, FloatUtils> TinyMatrix3x3f;
typedef ::TINY::TinyPose<float, FloatUtils> TinyPosef;

inline void setFromOpenGLMatrix(TinyPosef& tr, const float* m) {
  TinyMatrix3x3f mat;
  mat.setValue(m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]);
  mat.getRotation(tr.m_orientation);
  tr.m_position.setValue(m[12], m[13], m[14]);
}

inline void getOpenGLMatrix(const TinyPosef& tr, float* m) {
  TinyMatrix3x3f mat(tr.m_orientation);
  m[0] = mat.getRow(0).x();
  m[1] = mat.getRow(1).x();
  m[2] = mat.getRow(2).x();
  m[3] = 0.0f;
  m[4] = mat.getRow(0).y();
  m[5] = mat.getRow(1).y();
  m[6] = mat.getRow(2).y();
  m[7] = 0.0f;
  m[8] = mat.getRow(0).z();
  m[9] = mat.getRow(1).z();
  m[10] = mat.getRow(2).z();
  m[11] = 0.0f;
  m[12] = tr.m_position.x();
  m[13] = tr.m_position.y();
  m[14] = tr.m_position.z();
  m[15] = 1.0f;
}
};
#endif  // _TINY_FLOAT_UTILS_H
