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

#ifndef TINY_SYMMETRIC_SPATIAL_DYAD_H
#define TINY_SYMMETRIC_SPATIAL_DYAD_H

#include "tiny_spatial_transform.h"

template <typename TinyScalar, typename TinyConstants>
struct TinySymmetricSpatialDyad {
  typedef ::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinySpatialMotionVector<TinyScalar, TinyConstants>
      TinySpatialMotionVector;
  typedef ::TinySpatialTransform<TinyScalar, TinyConstants>
      TinySpatialTransform;

  TinyMatrix3x3 m_topLeftMat, m_topRightMat, m_bottomLeftMat, m_bottomRightMat;

  TinyVector3 m_center_of_mass;
  //
  TinySymmetricSpatialDyad() { setIdentity(); }
  TinySymmetricSpatialDyad(const TinyMatrix3x3& topLeftMat,
                           const TinyMatrix3x3& topRightMat,
                           const TinyMatrix3x3& bottomLeftMat,
                           const TinyMatrix3x3& bottomRightMat) {
    m_topLeftMat = topLeftMat;
    m_topRightMat = topRightMat;
    m_bottomLeftMat = bottomLeftMat;
    m_bottomRightMat = bottomRightMat;
  }
  //
  void setIdentity() {
    m_topLeftMat.set_identity();
    m_topRightMat.set_zero();
    m_bottomLeftMat.set_zero();
    m_bottomRightMat.set_identity();
    m_center_of_mass.set_zero();
  }

  // void setZero() {
  //   m_topLeftMat.set_zero();
  //   m_topRightMat.set_zero();
  //   m_bottomLeftMat.set_zero();
  //   m_bottomRightMat.set_zero();
  //   m_center_of_mass.set_zero();
  // }

  //
  TinySymmetricSpatialDyad& operator-=(const TinySymmetricSpatialDyad& mat) {
    m_topLeftMat -= mat.m_topLeftMat;
    m_topRightMat -= mat.m_topRightMat;
    m_bottomLeftMat -= mat.m_bottomLeftMat;
    m_bottomRightMat -= mat.m_bottomRightMat;
    return *this;
  }

  TinySymmetricSpatialDyad& operator+=(const TinySymmetricSpatialDyad& mat) {
    m_topLeftMat += mat.m_topLeftMat;
    m_topRightMat += mat.m_topRightMat;
    m_bottomLeftMat += mat.m_bottomLeftMat;
    m_bottomRightMat += mat.m_bottomRightMat;
    return *this;
  }

  static TinySymmetricSpatialDyad computeInertiaDyad(
      TinyScalar mass, const TinyVector3& com, const TinyMatrix3x3& inertia_C) {
    TinySymmetricSpatialDyad result;
    TinyVector3 h = com * mass;
    TinyScalar o = TinyConstants::zero();
    TinyMatrix3x3 I = inertia_C + TinyVectorCrossMatrix(com) *
                                      TinyVectorCrossMatrix(com).transpose() *
                                      mass;
    TinySymmetricSpatialDyad& mat = result;
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        mat(r, c) = I(r, c);
      }
    }

    {
      mat(3, 0) = o;
      mat(3, 1) = h[2];
      mat(3, 2) = -h[1];
      mat(4, 0) = -h[2];
      mat(4, 1) = o;
      mat(4, 2) = h[0];
      mat(5, 0) = h[1];
      mat(5, 1) = -h[0];
      mat(5, 2) = o;
    }
    {
      mat(0, 3) = o;
      mat(0, 4) = -h[2];
      mat(0, 5) = h[1];
      mat(1, 3) = h[2];
      mat(1, 4) = o;
      mat(1, 5) = -h[0];
      mat(2, 3) = -h[1];
      mat(2, 4) = h[0];
      mat(2, 5) = o;
    }
    {
      mat(3, 3) = mass;
      mat(3, 4) = o;
      mat(3, 5) = o;
      mat(4, 3) = o;
      mat(4, 4) = mass;
      mat(4, 5) = o;
      mat(5, 3) = o;
      mat(5, 4) = o;
      mat(5, 5) = mass;
    }

    result.m_center_of_mass = com;

    return result;
  }
  TinyScalar operator()(int r, int c) const {
    TinyConstants::FullAssert(r >= 0);
    TinyConstants::FullAssert(c >= 0);
    TinyConstants::FullAssert(r < 6);
    TinyConstants::FullAssert(c < 6);

    if (r < 3) {
      if (c < 3) {
        return m_topLeftMat(r, c);
      } else {
        return m_topRightMat(r, c - 3);
      }
    } else {
      if (c < 3) {
        return m_bottomLeftMat(r - 3, c);
      } else {
        return m_bottomRightMat(r - 3, c - 3);
      }
    }

    return TinyConstants::zero();
  }
  TinyScalar& operator()(int r, int c) {
    TinyConstants::FullAssert(r >= 0);
    TinyConstants::FullAssert(c >= 0);
    TinyConstants::FullAssert(r < 6);
    TinyConstants::FullAssert(c < 6);

    if (r < 3) {
      if (c < 3) {
        return m_topLeftMat(r, c);
      } else {
        return m_topRightMat(r, c - 3);
      }
    } else {
      if (c < 3) {
        return m_bottomLeftMat(r - 3, c);
      } else {
        return m_bottomRightMat(r - 3, c - 3);
      }
    }

    return m_bottomRightMat(0, 0);
  }

  void print(const char* txt) const {
    printf("%s\n", txt);
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        TinyScalar val = (*this)(r, c);
        double v = TinyConstants::getDouble(val);
        printf("%f, ", v);
      }
      printf("\n");
    }
  }

  TinySpatialMotionVector mul_org(const TinySpatialMotionVector& vec) const {
    return TinySpatialMotionVector(
        m_bottomLeftMat * vec.m_topVec +
            m_topLeftMat.transpose() * vec.m_bottomVec,
        m_topLeftMat * vec.m_topVec + m_topRightMat * vec.m_bottomVec);
  }
  TinySpatialMotionVector mul_inv(const TinySpatialMotionVector& vec) const {
    return TinySpatialMotionVector(
        m_topLeftMat * vec.m_topVec + m_topRightMat * vec.m_bottomVec,
        m_bottomLeftMat * vec.m_topVec + m_bottomRightMat * vec.m_bottomVec);
  }

  static TinySymmetricSpatialDyad vTimesvTranspose(
      const TinySpatialMotionVector& vecA,
      const TinySpatialMotionVector& vecB) {
    TinySymmetricSpatialDyad diad;
    for (int i = 0; i < 3; i++) {
      diad.m_topLeftMat[i] = vecA.m_topVec[i] * vecB.m_topVec;
      diad.m_bottomLeftMat[i] = vecA.m_bottomVec[i] * vecB.m_topVec;
      diad.m_topRightMat[i] = vecA.m_topVec[i] * vecB.m_bottomVec;
      diad.m_bottomRightMat[i] = vecA.m_bottomVec[i] * vecB.m_bottomVec;
    }

    return diad;
  }

  TinySymmetricSpatialDyad transposed() const {
    TinySymmetricSpatialDyad mT;
    mT.m_topLeftMat = this->m_topLeftMat.transpose();
    mT.m_bottomRightMat = this->m_bottomRightMat.transpose();
    mT.m_topRightMat = this->m_bottomLeftMat.transpose();
    mT.m_bottomLeftMat = this->m_topRightMat.transpose();
    return mT;
  }

  static TinySymmetricSpatialDyad mul(const TinySymmetricSpatialDyad& a,
                                      const TinySymmetricSpatialDyad& b) {
    TinySymmetricSpatialDyad res;
    res.m_topLeftMat = (a.m_topLeftMat * b.m_topLeftMat) +
                       (a.m_topRightMat * b.m_bottomLeftMat);
    res.m_topRightMat = (a.m_topLeftMat * b.m_topRightMat) +
                        (a.m_topRightMat * b.m_bottomRightMat);
    res.m_bottomLeftMat = (a.m_bottomLeftMat * b.m_topLeftMat) +
                          (a.m_bottomRightMat * b.m_bottomLeftMat);
    res.m_bottomRightMat = (a.m_bottomLeftMat * b.m_topRightMat) +
                           (a.m_bottomRightMat * b.m_bottomRightMat);
    return res;
  }

  static TinySymmetricSpatialDyad shift(const TinySymmetricSpatialDyad& ia,
                                        const TinySpatialTransform& trans) {
    // toMatrix
    TinyMatrix3x3 rx = TinyVectorCrossMatrix(trans.m_translation);
    // rx.print("rx");
    // TinyMatrix3x3 rxT = rx.transpose();
    // trans.m_rotation.print("E:");
    TinyMatrix3x3 Erx = trans.m_rotation.transpose() * rx;
    // Erx.print("Erx");
    TinySymmetricSpatialDyad m;
    m.m_topLeftMat = trans.m_rotation.transpose();
    m.m_topRightMat.set_zero();
    TinyScalar k = (TinyConstants::zero() - TinyConstants::one());
    m.m_bottomLeftMat = Erx * k;
    m.m_bottomRightMat = trans.m_rotation.transpose();
    // m.print("m");
    // trans
    TinySymmetricSpatialDyad mT = m.transposed();
    // mT.print("mT");

    TinySymmetricSpatialDyad mTia = TinySymmetricSpatialDyad::mul(mT, ia);
    // mTia.print("mTia");
    TinySymmetricSpatialDyad mTiam = TinySymmetricSpatialDyad::mul(mTia, m);
    // mTiam.print("mTiam");
    return mTiam;
  }

  TinySymmetricSpatialDyad inverseOld() const {
    TinyMatrix3x3 Binv =
        m_bottomRightMat.inverse() * TinyConstants::fraction(-1, 1);
    TinyMatrix3x3 tmp = m_topRightMat * Binv;
    TinyMatrix3x3 temp = tmp * m_bottomLeftMat + m_topLeftMat;
    TinyMatrix3x3 invI_upper_right = temp.inverse();
    tmp = invI_upper_right * m_topRightMat;
    TinyMatrix3x3 invI_upper_left = (tmp * Binv);
    TinyMatrix3x3 invI_lower_right = (invI_upper_left).transpose();
    tmp = m_bottomLeftMat * invI_upper_left;
    tmp[0][0] -= TinyConstants::one();
    tmp[1][1] -= TinyConstants::one();
    tmp[2][2] -= TinyConstants::one();
    TinyMatrix3x3 invI_lower_left = (Binv * tmp);

    return TinySymmetricSpatialDyad(invI_upper_right, invI_upper_left,
                                    invI_lower_right, invI_lower_left);
  }

  // Inverse of a symmetric block matrix
  // according to (4.1) in
  // http://msvlab.hre.ntou.edu.tw/grades/now/inte/Inverse%20&%20Border/border-LuTT.pdf
  TinySymmetricSpatialDyad inverse() const {
    const TinyMatrix3x3& A = m_topLeftMat;
    const TinyMatrix3x3& B = m_topRightMat;
    const TinyMatrix3x3& C = m_bottomLeftMat;  // denoted as B* in paper
    const TinyMatrix3x3& D = m_bottomRightMat;
    TinyMatrix3x3 Ainv = A.inverse();
    TinyMatrix3x3 Dinv = D.inverse();
    TinyMatrix3x3 DCAB = (D - C * Ainv * B).inverse();
    TinySymmetricSpatialDyad result;
    result.m_topLeftMat = Ainv + Ainv * B * DCAB * C * Ainv;
    result.m_topRightMat = -Ainv * B * DCAB;
    result.m_bottomLeftMat = -DCAB * C * Ainv;
    result.m_bottomRightMat = DCAB;
    return result;
  }
};

#endif  // TINY_SYMMETRIC_SPATIAL_DYAD_H
