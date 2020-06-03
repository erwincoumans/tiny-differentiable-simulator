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

#ifndef TINY_SPATIAL_MOTION_VECTOR_H
#define TINY_SPATIAL_MOTION_VECTOR_H

#include <stdio.h>
#include "tiny_vector3.h"

template <typename TinyScalar, typename TinyConstants>
class TinySpatialMotionVector {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;

 public:
  int m_size{6};

  TinyVector3 m_topVec, m_bottomVec;
  // TODO(ericheiden) remove unused rows argument (used for MatrixXxX)
  explicit TinySpatialMotionVector(int unused = 0) { set_zero(); }
  TinySpatialMotionVector(const TinyVector3& angular, const TinyVector3& linear)
      : m_topVec(angular), m_bottomVec(linear) {}
  void set_zero() {
    m_topVec.set_zero();
    m_bottomVec.set_zero();
  }

  TinySpatialMotionVector& operator+=(const TinySpatialMotionVector& vec) {
    m_topVec += vec.m_topVec;
    m_bottomVec += vec.m_bottomVec;
    return *this;
  }
  TinySpatialMotionVector& operator-=(const TinySpatialMotionVector& vec) {
    m_topVec -= vec.m_topVec;
    m_bottomVec -= vec.m_bottomVec;
    return *this;
  }

  TinySpatialMotionVector operator-(const TinySpatialMotionVector& vec) const {
    return TinySpatialMotionVector(m_topVec - vec.m_topVec,
                                   m_bottomVec - vec.m_bottomVec);
  }
  TinySpatialMotionVector operator+(const TinySpatialMotionVector& vec) const {
    return TinySpatialMotionVector(m_topVec + vec.m_topVec,
                                   m_bottomVec + vec.m_bottomVec);
  }
  TinySpatialMotionVector operator-() const {
    return TinySpatialMotionVector(-m_topVec, -m_bottomVec);
  }
  TinySpatialMotionVector operator*(const TinyScalar& s) const {
    return TinySpatialMotionVector(s * m_topVec, s * m_bottomVec);
  }

  inline TinyScalar& operator[](int i) {
    if (i < 3)
      return m_topVec[i];
    else
      return m_bottomVec[i - 3];
  }
  const inline TinyScalar& operator[](int i) const {
    if (i < 3)
      return m_topVec[i];
    else
      return m_bottomVec[i - 3];
  }

  /**
   * V1 = mv(w1, v1)
   * V2 = mv(w2, v2)
   * V1 x V2 = mv(w1 x w2, w1 x v2 + v1 x w2)
   */
  template <typename SpatialVectorType>
  SpatialVectorType crossm(const SpatialVectorType& b) const {
    SpatialVectorType out;
    out.m_topVec = m_topVec.cross(b.m_topVec);
    out.m_bottomVec =
        m_topVec.cross(b.m_bottomVec) + m_bottomVec.cross(b.m_topVec);
    return out;
  }

  /**
   * V = mv(w, v)
   * F = fv(n, f)
   * V x* F = fv(w x n + v x f, w x f)
   */
  template <typename SpatialVectorType>
  SpatialVectorType crossf(const SpatialVectorType& b) const {
    SpatialVectorType out;
    out.m_topVec =
        m_topVec.cross(b.m_topVec) + m_bottomVec.cross(b.m_bottomVec);
    out.m_bottomVec = m_topVec.cross(b.m_bottomVec);
    return out;
  }

  template <typename SpatialVectorType>
  TinyScalar dot(const SpatialVectorType& b) const {
    TinyScalar d = m_topVec.dot(b.m_topVec);
    d += m_bottomVec.dot(b.m_bottomVec);
    return d;
  }

  void print(const char* name) const {
    printf("%s\n", name);
    double x = TinyConstants::getDouble(m_topVec.getX());
    double y = TinyConstants::getDouble(m_topVec.getY());
    double z = TinyConstants::getDouble(m_topVec.getZ());
    printf("%.6f,%.6f,%.6f,    ", x, y, z);
    x = TinyConstants::getDouble(m_bottomVec.getX());
    y = TinyConstants::getDouble(m_bottomVec.getY());
    z = TinyConstants::getDouble(m_bottomVec.getZ());
    printf("%.6f,%.6f,%.6f\n", x, y, z);
  }
};

#endif  // TINY_SPATIAL_MOTION_VECTOR_H
