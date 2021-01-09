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
#include "tiny_vector_x.h"

namespace TINY
{
    /**
     * 6x3 matrix for multi DoF joints consisting of two stacked 3x3 matrices. It can also be used as a 3x6 matrix for
     * multiplication with motion vectors.
     */
    template <typename TinyScalar, typename TinyConstants>
    struct TinyMatrix6x3 {
        typedef ::TINY::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
        typedef ::TINY::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
        typedef ::TINY::TinyVectorX<TinyScalar, TinyConstants> TinyVectorX;

        TinyMatrix3x3 m_top, m_bottom;
        bool m_tranposed = false;

        TinyVector3 m_center_of_mass;
        //
        TinyMatrix6x3() { set_zero(); }
        TinyMatrix6x3(const TinyMatrix3x3& top,
            const TinyMatrix3x3& bottom) {
            m_top = top;
            m_bottom = bottom;
        }

         void set_zero() {
           m_top.set_zero();
           m_bottom.set_zero();
           m_center_of_mass.set_zero();
         }

      TinyMatrix6x3 transpose() const{
        TinyMatrix6x3 out(m_top, m_bottom);
        out.m_tranposed = true;
        return out;
      }

        TinyMatrix6x3& operator-=(const TinyMatrix6x3& mat) {
            m_top -= mat.m_topLeftMat;
            m_bottom -= mat.m_bottomLeftMat;
            return *this;
        }

        TinyMatrix6x3& operator+=(const TinyMatrix6x3& mat) {
            m_top += mat.m_topLeftMat;
            m_bottom += mat.m_bottomLeftMat;
            return *this;
        }

        const TinyScalar& operator()(int r, int c) const {
            TinyConstants::FullAssert(r >= 0);
            TinyConstants::FullAssert(c >= 0);
            TinyConstants::FullAssert(r < 6);
            TinyConstants::FullAssert(c < 3);

            if (r < 3) {
                return m_top(r, c);
            }
            else {
                return m_bottom(r - 3, c);
            }

            return m_bottom(0, 0);
        }

        TinyScalar& operator()(int r, int c) {
            TinyConstants::FullAssert(r >= 0);
            TinyConstants::FullAssert(c >= 0);
            TinyConstants::FullAssert(r < 6);
            TinyConstants::FullAssert(c < 3);

            if (r < 3) {
                return m_top(r, c);
            }
            else {
                return m_bottom(r - 3, c);
            }

            return m_bottom(0, 0);
        }

        void print(const char* txt) const {
            printf("%s\n", txt);
            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < 3; c++) {
                    TinyScalar val = (*this)(r, c);
                    double v = TinyConstants::getDouble(val);
                    printf("%f, ", v);
                }
                printf("\n");
            }
        }

        /**
         * Multiplication with another Matrix6x3 handles the operation as a multiplication with the transpose of this
         * matrix.
         * @param a Matrix6x3
         * @param b Matrix6x3
         * @return TinyMatrix3x3
         */
        friend TinyMatrix3x3 operator*(const TinyMatrix6x3& a,
                                       const TinyMatrix6x3& b) {
          TinyConstants::FullAssert(a.m_tranposed);

          TinyMatrix3x3 res;
            res = a.m_top.transpose() * b.m_top + a.m_bottom.transpose() * b.m_bottom;
            return res;
        }

        /**
         * Multiplication with a Matrix3x3 returns a Matrix6x3
         * @param a Matrix6x3
         * @param b TinyMatrix3x3
         * @return Matrix6x3
         */
        friend TinyMatrix6x3 operator*(const TinyMatrix6x3& a,
                                       const TinyMatrix3x3& b) {
            TinyMatrix6x3 res;
            res.m_top = a.m_top * b;
            res.m_bottom = a.m_bottom * b;
            return res;
        }

        /**
         * Multiplication of a 6x3 matrix with a Vector3 returns a spatial vector
         * @param a
         * @param b
         * @return
         */
//        friend TinyVectorX operator*(const TinyMatrix6x3& m,
//                                       const TinyVector3& v) {
//            TinyVectorX res(6);
//            TinyVector3 top = m.m_top * v;
//            TinyVector3 bottom = m.m_bottom * v;
//            for (int ii = 0; ii < 3; ii++){
//                res[ii] = top[ii];
//                res[ii + 3] = bottom[ii];
//            }
//            return res;
//        }
    };
};
