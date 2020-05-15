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

#ifndef TINY_SPATIAL_TRANSFORM_H
#define TINY_SPATIAL_TRANSFORM_H

#include "tiny_matrix3x3.h"
#include "tiny_spatial_motion_vector.h"
#include "tiny_vector3.h"

/**
 * We follow the spatial algebra from Featherstone, but use right-handed
 * transformation matrix applications so that the rotations need to be
 * transposed and the transformation matrices are multiplied from right to left.
 */
template <typename TinyScalar, typename TinyConstants>
class TinySpatialTransform {
 public:
  enum eOutputOperation { None = 0, Add = 1, Subtract = 2 };

  typedef ::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinySpatialMotionVector<TinyScalar, TinyConstants>
      TinySpatialMotionVector;

  TinyVector3 m_translation;
  TinyMatrix3x3 m_rotation;

  TinySpatialTransform() { set_identity(); }

  void set_identity() {
    m_translation.set_zero();
    m_rotation.set_identity();
  }

  /**
   * X1*X2 = plx(E1*E2, r2 + E2T*r1)
   */
  TinySpatialTransform operator*(const TinySpatialTransform& t) const {
    TinySpatialTransform tr = *this;
    tr.m_translation += m_rotation * t.m_translation;
    tr.m_rotation *= t.m_rotation;
    return tr;
  }

  void print(const char* name) const {
    printf("%s\n", name);
    double x = TinyConstants::getDouble(m_translation.getX());
    double y = TinyConstants::getDouble(m_translation.getY());
    double z = TinyConstants::getDouble(m_translation.getZ());
    printf("translation: %f,%f,%f\n", x, y, z);
    printf("rotation:\n");
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        double v = TinyConstants::getDouble(m_rotation[r][c]);
        printf("%f, ", v);
      }
      printf("\n");
    }
  }

  template <typename SpatialVectorType>
  SpatialVectorType transformRotateOnly(const SpatialVectorType& vecIn) const {
    SpatialVectorType outVec;
    outVec.m_topVec = m_rotation * vecIn.m_topVec;
    outVec.m_bottomVec = m_rotation * vecIn.m_bottomVec;
    return outVec;
  }

  TinyVector3 apply(const TinyVector3& point) const {
    return m_rotation * point + m_translation;
  }
  TinyVector3 apply_inverse(const TinyVector3& point) const {
    return m_rotation.transpose() * (point - m_translation);
  }

  TinySpatialTransform get_inverse() const {
    TinySpatialTransform inv;
    inv.m_rotation = m_rotation.transpose();
    inv.m_translation = inv.m_rotation * -m_translation;
    return inv;
  }

  /**
   * V = mv(w, v)
   * X*V = mv(E*w, E*(v - r x w))
   */
  template <typename SpatialVectorType>
  SpatialVectorType apply(const SpatialVectorType& inVec) const {
    SpatialVectorType outVec;

    TinyVector3 rxw = inVec.m_topVec.cross(m_translation);
    TinyVector3 v_rxw = inVec.m_bottomVec + rxw;

    TinyVector3 tmp3 = m_rotation.transpose() * v_rxw;
    TinyVector3 tmp4 = m_rotation.transpose() * inVec.m_topVec;

    outVec.m_topVec = tmp4;
    outVec.m_bottomVec = tmp3;

    return outVec;
  }

  /**
   * V = mv(w, v)
   * inv(X)*V = mv(ET*w, ET*v + r x (ET*w))
   */
  template <typename SpatialVectorType>
  SpatialVectorType apply_inverse(const SpatialVectorType& inVec) const {
    SpatialVectorType outVec;
    outVec.m_topVec = m_rotation * inVec.m_topVec;
    outVec.m_bottomVec =
        m_rotation * inVec.m_bottomVec + m_translation.cross(outVec.m_topVec);
    return outVec;
  }

  /**
   * F = fv(n, f)
   * XT*F = fv(ETn + rxETf, ETf)
   */
  template <typename SpatialVectorType>
  SpatialVectorType apply_transpose(const SpatialVectorType& inVec) const {
    SpatialVectorType outVec;
    outVec.m_bottomVec = m_rotation * inVec.m_bottomVec;
    outVec.m_topVec = m_rotation * inVec.m_topVec;
    outVec.m_topVec +=
        TinyVectorCrossMatrix(m_translation) * outVec.m_bottomVec;

    return outVec;
  }

  /**
   * F = fv(n, f)
   * X^* F = fv(E(n - rxf), Ef)
   */
  template <typename SpatialVectorType>
  SpatialVectorType apply_inverse_transpose(
      const SpatialVectorType& inVec) const {
    const TinyVector3& n = inVec.m_topVec;
    const TinyVector3& f = inVec.m_bottomVec;
    SpatialVectorType outVec;
    outVec.m_topVec = m_rotation.transpose() * (n - m_translation.cross(f));
    outVec.m_bottomVec = m_rotation.transpose() * f;

    return outVec;
  }
};

#endif  // TINY_SPATIAL_TRANSFORM_H
