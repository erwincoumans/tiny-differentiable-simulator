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

#pragma once

#include <cstdio>

#include "tiny_matrix3x3.h"
#include "tiny_vector3.h"

namespace TINY
{
    /**
     * Block-symmetric 6x6 matrix consisting of four 3x3 matrices.
     */
    template <typename TinyScalar, typename TinyConstants>
    struct TinyMatrix6x6 {
        typedef ::TINY::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
        typedef ::TINY::TinyVector3<TinyScalar, TinyConstants> TinyVector3;

        TinyMatrix3x3 m_topLeftMat, m_topRightMat, m_bottomLeftMat, m_bottomRightMat;

        TinyVector3 m_center_of_mass;
        //
        TinyMatrix6x6() { setIdentity(); }
        TinyMatrix6x6(const TinyMatrix3x3& topLeftMat,
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
        TinyMatrix6x6& operator-=(const TinyMatrix6x6& mat) {
            m_topLeftMat -= mat.m_topLeftMat;
            m_topRightMat -= mat.m_topRightMat;
            m_bottomLeftMat -= mat.m_bottomLeftMat;
            m_bottomRightMat -= mat.m_bottomRightMat;
            return *this;
        }

        TinyMatrix6x6& operator+=(const TinyMatrix6x6& mat) {
            m_topLeftMat += mat.m_topLeftMat;
            m_topRightMat += mat.m_topRightMat;
            m_bottomLeftMat += mat.m_bottomLeftMat;
            m_bottomRightMat += mat.m_bottomRightMat;
            return *this;
        }

        const TinyScalar& operator()(int r, int c) const {
            TinyConstants::FullAssert(r >= 0);
            TinyConstants::FullAssert(c >= 0);
            TinyConstants::FullAssert(r < 6);
            TinyConstants::FullAssert(c < 6);

            if (r < 3) {
                if (c < 3) {
                    return m_topLeftMat(r, c);
                }
                else {
                    return m_topRightMat(r, c - 3);
                }
            }
            else {
                if (c < 3) {
                    return m_bottomLeftMat(r - 3, c);
                }
                else {
                    return m_bottomRightMat(r - 3, c - 3);
                }
            }

            return m_bottomRightMat(0, 0);
        }

        TinyScalar& operator()(int r, int c) {
            TinyConstants::FullAssert(r >= 0);
            TinyConstants::FullAssert(c >= 0);
            TinyConstants::FullAssert(r < 6);
            TinyConstants::FullAssert(c < 6);

            if (r < 3) {
                if (c < 3) {
                    return m_topLeftMat(r, c);
                }
                else {
                    return m_topRightMat(r, c - 3);
                }
            }
            else {
                if (c < 3) {
                    return m_bottomLeftMat(r - 3, c);
                }
                else {
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

        // TinySpatialMotionVector mul_org(const TinySpatialMotionVector& vec) const {
        //   return TinySpatialMotionVector(
        //       m_bottomLeftMat * vec.m_topVec +
        //           m_topLeftMat.transpose() * vec.m_bottomVec,
        //       m_topLeftMat * vec.m_topVec + m_topRightMat * vec.m_bottomVec);
        // }
        // TinySpatialMotionVector mul_inv(const TinySpatialMotionVector& vec) const {
        //   return TinySpatialMotionVector(
        //       m_topLeftMat * vec.m_topVec + m_topRightMat * vec.m_bottomVec,
        //       m_bottomLeftMat * vec.m_topVec + m_bottomRightMat * vec.m_bottomVec);
        // }

        // static TinyMatrix6x6 vTimesvTranspose(
        //     const TinySpatialMotionVector& vecA,
        //     const TinySpatialMotionVector& vecB) {
        //   TinyMatrix6x6 diad;
        //   for (int i = 0; i < 3; i++) {
        //     diad.m_topLeftMat[i] = vecA.m_topVec[i] * vecB.m_topVec;
        //     diad.m_bottomLeftMat[i] = vecA.m_bottomVec[i] * vecB.m_topVec;
        //     diad.m_topRightMat[i] = vecA.m_topVec[i] * vecB.m_bottomVec;
        //     diad.m_bottomRightMat[i] = vecA.m_bottomVec[i] * vecB.m_bottomVec;
        //   }

        //   return diad;
        // }

        TinyMatrix6x6 transposed() const {
            TinyMatrix6x6 mT;
            mT.m_topLeftMat = this->m_topLeftMat.transpose();
            mT.m_bottomRightMat = this->m_bottomRightMat.transpose();
            mT.m_topRightMat = this->m_bottomLeftMat.transpose();
            mT.m_bottomLeftMat = this->m_topRightMat.transpose();
            return mT;
        }

        friend TinyMatrix6x6 operator*(const TinyMatrix6x6& a,
            const TinyMatrix6x6& b) {
            TinyMatrix6x6 res;
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

        TinyMatrix6x6 inverseOld() const {
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

            return TinyMatrix6x6(invI_upper_right, invI_upper_left, invI_lower_right,
                invI_lower_left);
        }

        // Inverse of a symmetric block matrix
        // according to (4.1) in
        // http://msvlab.hre.ntou.edu.tw/grades/now/inte/Inverse%20&%20Border/border-LuTT.pdf
        TinyMatrix6x6 inverse() const {
            const TinyMatrix3x3& A = m_topLeftMat;
            const TinyMatrix3x3& B = m_topRightMat;
            const TinyMatrix3x3& C = m_bottomLeftMat;  // denoted as B* in paper
            const TinyMatrix3x3& D = m_bottomRightMat;
            TinyMatrix3x3 Ainv = A.inverse();
            TinyMatrix3x3 Dinv = D.inverse();
            TinyMatrix3x3 DCAB = (D - C * Ainv * B).inverse();
            TinyMatrix6x6 result;
            result.m_topLeftMat = Ainv + Ainv * B * DCAB * C * Ainv;
            result.m_topRightMat = -Ainv * B * DCAB;
            result.m_bottomLeftMat = -DCAB * C * Ainv;
            result.m_bottomRightMat = DCAB;
            return result;
        }
    };
};
