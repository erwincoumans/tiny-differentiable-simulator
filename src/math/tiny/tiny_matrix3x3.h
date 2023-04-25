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

#ifndef _TINY_MATRIX3x3_H
#define _TINY_MATRIX3x3_H

#include <stdio.h>

#include "tiny_quaternion.h"
#include "tiny_vector3.h"
//#include "math/eigen_algebra.hpp"

/// By default, TinyMatrix3x3 is a row-major unless
/// TDS_USE_COLUMN_MAJOR is defined

namespace TINY
{
template <typename TinyScalar, typename TinyConstants>
class TinyMatrix3x3 {
  typedef ::TINY::TinyQuaternion<TinyScalar, TinyConstants> TinyQuaternion;
  typedef ::TINY::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  /// Data storage for the matrix, each vector is a row or column of the matrix
  TinyVector3 m_el[3];

 public:
  /** @brief No initialization constructor */
  TinyMatrix3x3() = default;

  // necessary for variable-size matrix interoperability
  int m_rows{3};
  int m_cols{3};

  //		explicit TinyMatrix3x3(const TinyScalar *m) {
  // setFromOpenGLSubMatrix(m); }

  /**@brief Constructor from Quaternion */
  explicit TinyMatrix3x3(const TinyQuaternion& q) { setRotation(q); }

  /** @brief Constructor with row or column major formatting */
  TinyMatrix3x3(const TinyScalar& xx, const TinyScalar& xy,
                const TinyScalar& xz, const TinyScalar& yx,
                const TinyScalar& yy, const TinyScalar& yz,
                const TinyScalar& zx, const TinyScalar& zy,
                const TinyScalar& zz) {
    setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
  }

  /** @brief Copy constructor */
  inline TinyMatrix3x3(const TinyMatrix3x3& other) {
    m_el[0] = other.m_el[0];
    m_el[1] = other.m_el[1];
    m_el[2] = other.m_el[2];
  }

  /** @brief Assignment Operator */
  inline TinyMatrix3x3& operator=(const TinyMatrix3x3& other) {
    m_el[0] = other.m_el[0];
    m_el[1] = other.m_el[1];
    m_el[2] = other.m_el[2];
    return *this;
  }

  inline TinyMatrix3x3& operator*=(const TinyMatrix3x3& m) {

#ifdef TDS_USE_COLUMN_MAJOR
  setValue(tdotx(m[0]), tdotx(m[1]), tdotx(m[2]), tdoty(m[0]), tdoty(m[1]),
            tdoty(m[2]), tdotz(m[0]), tdotz(m[1]), tdotz(m[2]));
#else
    setValue(m.tdotx(m_el[0]), m.tdoty(m_el[0]), m.tdotz(m_el[0]),
             m.tdotx(m_el[1]), m.tdoty(m_el[1]), m.tdotz(m_el[1]),
             m.tdotx(m_el[2]), m.tdoty(m_el[2]), m.tdotz(m_el[2]));
#endif //TDS_USE_COLUMN_MAJOR
    return *this;
  }

  inline TinyMatrix3x3(const TinyVector3& v0, const TinyVector3& v1,
                       const TinyVector3& v2) {
    m_el[0] = v0;
    m_el[1] = v1;
    m_el[2] = v2;
  }

  /** @brief Set the values of the matrix explicitly (row or column major)
   *  @param xx Top left
   *  @param xy Top Middle
   *  @param xz Top Right
   *  @param yx Middle Left
   *  @param yy Middle Middle
   *  @param yz Middle Right
   *  @param zx Bottom Left
   *  @param zy Bottom Middle
   *  @param zz Bottom Right*/
  void setValue(const TinyScalar& xx, const TinyScalar& xy,
                const TinyScalar& xz, const TinyScalar& yx,
                const TinyScalar& yy, const TinyScalar& yz,
                const TinyScalar& zx, const TinyScalar& zy,
                const TinyScalar& zz) {
#ifdef TDS_USE_COLUMN_MAJOR
    m_el[0].setValue(xx, yx, zx);
    m_el[1].setValue(xy, yy, zy);
    m_el[2].setValue(xz, yz, zz);
#else
    m_el[0].setValue(xx, xy, xz);
    m_el[1].setValue(yx, yy, yz);
    m_el[2].setValue(zx, zy, zz);
#endif
}
#ifdef TDS_USE_COLUMN_MAJOR
  inline const TinyScalar& get_at(int row, int col) const {
	  TinyConstants::FullAssert(0 <= row && row < 3);
	  TinyConstants::FullAssert(0 <= col && col < 3);
	  return m_el[col][row];
   }

  inline void set_at(int row, int col, const TinyScalar& value) {
	  TinyConstants::FullAssert(0 <= row && row < 3);
	  TinyConstants::FullAssert(0 <= col && col < 3);
	  m_el[col][row] = value;
  }
  inline TinyScalar& operator()(int row, int col) {
    TinyConstants::FullAssert(0 <= row && row < 3);
    TinyConstants::FullAssert(0 <= col && col < 3);
    return m_el[col][row];
  }
#else
  inline const TinyScalar& get_at(int row, int col) const {
	  TinyConstants::FullAssert(0 <= row && row < 3);
	  TinyConstants::FullAssert(0 <= col && col < 3);
	  return m_el[row][col];
  }

  inline void set_at(int row, int col, const TinyScalar& value) {
	  TinyConstants::FullAssert(0 <= row && row < 3);
	  TinyConstants::FullAssert(0 <= col && col < 3);
	  m_el[row][col] = value;
  }

  inline TinyScalar& operator()(int row, int col) {
    TinyConstants::FullAssert(0 <= row && row < 3);
    TinyConstants::FullAssert(0 <= col && col < 3);
    return m_el[row][col];
  }
#endif

  /** @brief Get a mutable reference to a row or column of the matrix as a vector
   *  @param i Row number 0 indexed */
  inline TinyVector3& operator[](int i) {
    TinyConstants::FullAssert(0 <= i && i < 3);
    return m_el[i];
  }

  /** @brief Get a const reference to a row or column of the matrix as a vector
   *  @param i Row number 0 indexed */
  inline const TinyVector3& operator[](int i) const {
    TinyConstants::FullAssert(0 <= i && i < 3);
    return m_el[i];
  }

  inline const TinyScalar& operator()(int row, int col) const {
    TinyConstants::FullAssert(0 <= row && row < 3);
    TinyConstants::FullAssert(0 <= col && col < 3);
#ifdef TDS_USE_COLUMN_MAJOR
    return m_el[col][row];
#else
    return m_el[row][col];
#endif //TDS_USE_COLUMN_MAJOR
  }


  /** @brief Set the matrix from euler angles YPR around ZYX axes
   * @param eulerX Roll about X axis
   * @param eulerY Pitch around Y axis
   * @param eulerZ Yaw about Z axis
   *
   * These angles are used to produce a rotation matrix. The euler
   * angles are applied in ZYX order. I.e a vector is first rotated
   * about X then Y and then Z
   **/
  void setEulerZYX(TinyScalar eulerX, TinyScalar eulerY, TinyScalar eulerZ) {
    ///@todo proposed to reverse this since it's labeled zyx but takes arguments
    /// xyz and it will match all other parts of the code
    TinyScalar ci(TinyConstants::cos1(eulerX));
    TinyScalar cj(TinyConstants::cos1(eulerY));
    TinyScalar ch(TinyConstants::cos1(eulerZ));
    TinyScalar si(TinyConstants::sin1(eulerX));
    TinyScalar sj(TinyConstants::sin1(eulerY));
    TinyScalar sh(TinyConstants::sin1(eulerZ));
    TinyScalar cc = ci * ch;
    TinyScalar cs = ci * sh;
    TinyScalar sc = si * ch;
    TinyScalar ss = si * sh;

#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    // need to transpose
    m_el[0].setValue(cj * ch, cj * sh, -sj);
    m_el[1].setValue(sj * sc - cs, sj * ss + cc, cj * si);
    m_el[2].setValue(sj * cc + ss, sj * cs - sc, cj * ci);
#else
    m_el[0].setValue(cj * ch, sj * sc - cs, sj * cc + ss);
    m_el[1].setValue(cj * sh, sj * ss + cc, sj * cs - sc);
    m_el[2].setValue(-sj, cj * si, cj * ci);
#endif
  }

  void set_rotation_x(TinyScalar angle) {
    TinyScalar c(TinyConstants::cos1(angle));
    TinyScalar s(TinyConstants::sin1(angle));
    TinyScalar o(TinyConstants::zero());
    TinyScalar i(TinyConstants::one());

#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    // need to transpose
    m_el[0].setValue(i, o, o);
    m_el[1].setValue(o, c, s);
    m_el[2].setValue(o, -s, c);
#else
    m_el[0].setValue(i, o, o);
    m_el[1].setValue(o, c, -s);
    m_el[2].setValue(o, s, c);
#endif
  }

  void set_rotation_y(TinyScalar angle) {
    TinyScalar c(TinyConstants::cos1(angle));
    TinyScalar s(TinyConstants::sin1(angle));
    TinyScalar o(TinyConstants::zero());
    TinyScalar i(TinyConstants::one());

#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    // need to transpose
    m_el[0].setValue(c, o, -s);
    m_el[1].setValue(o, i, o);
    m_el[2].setValue(s, o, c);
#else
    m_el[0].setValue(c, o, s);
    m_el[1].setValue(o, i, o);
    m_el[2].setValue(-s, o, c);
#endif
  }

  void set_rotation_z(TinyScalar angle) {
    TinyScalar c(TinyConstants::cos1(angle));
    TinyScalar s(TinyConstants::sin1(angle));
    TinyScalar o(TinyConstants::zero());
    TinyScalar i(TinyConstants::one());

#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    // need to transpose
    m_el[0].setValue(c, s, o);
    m_el[1].setValue(-s, c, o);
    m_el[2].setValue(o, o, i);
#else
    m_el[0].setValue(c, -s, o);
    m_el[1].setValue(s, c, o);
    m_el[2].setValue(o, o, i);
#endif
  }

  /**@brief Set the matrix to the identity */
  void set_identity() {
    setValue(TinyConstants::one(), TinyConstants::zero(),
             TinyConstants::zero(), TinyConstants::zero(),
             TinyConstants::one(), TinyConstants::zero(),
             TinyConstants::zero(), TinyConstants::zero(),
             TinyConstants::one());
  }

  void set_zero() {
    setValue(TinyConstants::zero(), TinyConstants::zero(),
             TinyConstants::zero(), TinyConstants::zero(),
             TinyConstants::zero(), TinyConstants::zero(),
             TinyConstants::zero(), TinyConstants::zero(),
             TinyConstants::zero());
  }

#ifdef TDS_USE_COLUMN_MAJOR
  inline TinyScalar tdotx(const TinyVector3& v) const {
    return getRow(0).dot(v);
  }
  inline TinyScalar tdoty(const TinyVector3& v) const {
    return getRow(1).dot(v);
  }
  inline TinyScalar tdotz(const TinyVector3& v) const {
    return getRow(2).dot(v);
  }
#else

  inline TinyScalar tdotx(const TinyVector3& v) const {
    return m_el[0].x() * v.x() + m_el[1].x() * v.y() + m_el[2].x() * v.z();
  }
  inline TinyScalar tdoty(const TinyVector3& v) const {
    return m_el[0].y() * v.x() + m_el[1].y() * v.y() + m_el[2].y() * v.z();
  }
  inline TinyScalar tdotz(const TinyVector3& v) const {
    return m_el[0].z() * v.x() + m_el[1].z() * v.y() + m_el[2].z() * v.z();
  }

#endif

  /** @brief Set the matrix from a quaternion
   *  @param q The Quaternion to match */
  void setRotation(const TinyQuaternion& q) {
    TinyScalar d = q.length2();
    // TODO reactivate assertion
    if (d == TinyConstants::zero()) {
      return;
    }
    //    TinyConstants::FullAssert(d != TinyConstants::zero());
    TinyScalar s = TinyConstants::two() / d;
    TinyScalar xs = q.x() * s, ys = q.y() * s, zs = q.z() * s;
    TinyScalar wx = q.w() * xs, wy = q.w() * ys, wz = q.w() * zs;
    TinyScalar xx = q.x() * xs, xy = q.x() * ys, xz = q.x() * zs;
    TinyScalar yy = q.y() * ys, yz = q.y() * zs, zz = q.z() * zs;

#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    // need to transpose
    setValue(
      TinyConstants::one() - (yy + zz), xy + wz, xz - wy,
      xy - wz, TinyConstants::one() - (xx + zz), yz + wx,
      xz + wy, yz - wx, TinyConstants::one() - (xx + yy));
#else
    setValue(
      TinyConstants::one() - (yy + zz), xy - wz, xz + wy,
      xy + wz, TinyConstants::one() - (xx + zz), yz - wx,
      xz - wy, yz + wx, TinyConstants::one() - (xx + yy));
#endif
  }

  TinyQuaternion getRotation2() const {
	  TinyQuaternion tmp;
	  getRotation(tmp);
	  return tmp;
  }
  
  /**@brief Get the matrix represented as a quaternion
   * @param q The quaternion which will be set */
  void getRotation(TinyQuaternion& q) const {
// #ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const TinyMatrix3x3& m = *this;
// #else
//     const TinyMatrix3x3 m = transpose();
// #endif
#ifdef USE_CPPAD
    if constexpr (tds::is_cppad_scalar<TinyScalar>::value) {
      // add epsilon to denominator to prevent division by zero
      const TinyScalar eps = TinyConstants::fraction(1, 1000000);
      TinyScalar tr = m(0, 0) + m(1, 1) + m(2, 2);
      TinyScalar q1[4], q2[4], q3[4], q4[4];
      // if (tr > 0)
      {
        TinyScalar S = TinyConstants::sqrt1(
                           TinyConstants::abs(tr + TinyConstants::one())) *
                           TinyConstants::two() +
                       eps;
        q1[0] = TinyConstants::fraction(1, 4) * S;
        q1[1] = (m(2, 1) - m(1, 2)) / S;
        q1[2] = (m(0, 2) - m(2, 0)) / S;
        q1[3] = (m(1, 0) - m(0, 1)) / S;
      }
      // else if ((m(0,0) > m(1,1))&(m(0,0) > m(2,2)))
      {
        TinyScalar S =
            TinyConstants::sqrt1(TinyConstants::abs(
                TinyConstants::one() + m(0, 0) - m(1, 1) - m(2, 2))) *
                TinyConstants::two() +
            eps;
        q2[0] = (m(2, 1) - m(1, 2)) / S;
        q2[1] = TinyConstants::fraction(1, 4) * S;
        q2[2] = (m(0, 1) + m(1, 0)) / S;
        q2[3] = (m(0, 2) + m(2, 0)) / S;
      }
      // else if (m(1,1) > m(2,2))
      {
        TinyScalar S =
            TinyConstants::sqrt1(TinyConstants::abs(
                TinyConstants::one() + m(1, 1) - m(0, 0) - m(2, 2))) *
                TinyConstants::two() +
            eps;
        q3[0] = (m(0, 2) - m(2, 0)) / S;
        q3[1] = (m(0, 1) + m(1, 0)) / S;
        q3[2] = TinyConstants::fraction(1, 4) * S;
        q3[3] = (m(1, 2) + m(2, 1)) / S;
      }
      // else
      {
        TinyScalar S =
            TinyConstants::sqrt1(TinyConstants::abs(
                TinyConstants::one() + m(2, 2) - m(0, 0) - m(1, 1))) *
                TinyConstants::two() +
            eps;
        q4[0] = (m(1, 0) - m(0, 1)) / S;
        q4[1] = (m(0, 2) + m(2, 0)) / S;
        q4[2] = (m(1, 2) + m(2, 1)) / S;
        q4[3] = TinyConstants::fraction(1, 4) * S;
      }
      // (m(0,0) > m(1,1))&(m(0,0) > m(2,2))
      TinyScalar m00_is_max =
          tds::where_gt(m(0, 0), m(1, 1),
                        tds::where_gt(m(0, 0), m(2, 2), TinyConstants::one(),
                                      TinyConstants::zero()),
                        TinyConstants::zero());
      TinyScalar m11_is_max =
          (TinyConstants::one() - m00_is_max) *
          tds::where_gt(m(1, 1), m(2, 2), TinyConstants::one(),
                        TinyConstants::zero());
      TinyScalar m22_is_max = (TinyConstants::one() - m00_is_max) *
                              (TinyConstants::one() - m11_is_max);
      q[0] = tds::where_gt(
          tr, TinyConstants::zero(), q1[0],
          m00_is_max * q2[0] + m11_is_max * q3[0] + m22_is_max * q4[0]);
      q[1] = tds::where_gt(
          tr, TinyConstants::zero(), q1[1],
          m00_is_max * q2[1] + m11_is_max * q3[1] + m22_is_max * q4[1]);
      q[2] = tds::where_gt(
          tr, TinyConstants::zero(), q1[2],
          m00_is_max * q2[2] + m11_is_max * q3[2] + m22_is_max * q4[2]);
      q[3] = tds::where_gt(
          tr, TinyConstants::zero(), q1[3],
          m00_is_max * q2[3] + m11_is_max * q3[3] + m22_is_max * q4[3]);
    } else 
#endif
	{

      TinyScalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();
      TinyScalar temp[4];
      if (trace < TinyConstants::zero()) {
        int i = m_el[0].x() < m_el[1].y() ? (m_el[1].y() < m_el[2].z() ? 2 : 1)
                                          : (m_el[0].x() < m_el[2].z() ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        TinyScalar tmp =
            ((m_el[i][i] - m_el[j][j]) - m_el[k][k]) + TinyConstants::one();
        TinyScalar s = TinyConstants::sqrt1(tmp);
        temp[i] = s * TinyConstants::half();
        s = TinyConstants::half() / s;

        temp[3] = (m_el[j][k] - m_el[k][j]) * s;
        temp[j] = (m_el[i][j] + m_el[j][i]) * s;
        temp[k] = (m_el[i][k] + m_el[k][i]) * s;
      } else {
        TinyScalar s = TinyConstants::sqrt1(trace + TinyConstants::one());
        temp[3] = (s * TinyConstants::half());
        s = TinyConstants::half() / s;

        temp[0] = ((m_el[1][2] - m_el[2][1]) * s);
        temp[1] = ((m_el[2][0] - m_el[0][2]) * s);
        temp[2] = ((m_el[0][1] - m_el[1][0]) * s);
      }
      #ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
      q.setValue(temp[0], temp[1], temp[2], temp[3]);
      #else
      q.setValue(temp[0], temp[1], temp[2], -temp[3]);
      #endif
    }
  }

  /**@brief Return the transpose of the matrix */
  TinyMatrix3x3 transpose() const;

  /** @brief Adds by the target matrix on the right
   *  @param m matrix to be applied
   * Equivilant to this = this + m */
  TinyMatrix3x3& operator+=(const TinyMatrix3x3& m) {
#ifdef TDS_USE_COLUMN_MAJOR
    m_el[0] += m[0];
    m_el[1] += m[1];
    m_el[2] += m[2];
#else

    setValue(m_el[0][0] + m.m_el[0][0], m_el[0][1] + m.m_el[0][1],
             m_el[0][2] + m.m_el[0][2], m_el[1][0] + m.m_el[1][0],
             m_el[1][1] + m.m_el[1][1], m_el[1][2] + m.m_el[1][2],
             m_el[2][0] + m.m_el[2][0], m_el[2][1] + m.m_el[2][1],
             m_el[2][2] + m.m_el[2][2]);
#endif
    return *this;
  }

  /** @brief Substracts by the target matrix on the right
   *  @param m matrix to be applied
   * Equivilant to this = this - m */
  TinyMatrix3x3& operator-=(const TinyMatrix3x3& m) {
#ifdef TDS_USE_COLUMN_MAJOR
    m_el[0] -= m[0];
    m_el[1] -= m[1];
    m_el[2] -= m[2];
#else
    setValue(m_el[0][0] - m.m_el[0][0], m_el[0][1] - m.m_el[0][1],
             m_el[0][2] - m.m_el[0][2], m_el[1][0] - m.m_el[1][0],
             m_el[1][1] - m.m_el[1][1], m_el[1][2] - m.m_el[1][2],
             m_el[2][0] - m.m_el[2][0], m_el[2][1] - m.m_el[2][1],
             m_el[2][2] - m.m_el[2][2]);
#endif
    return *this;
  }

#ifdef TDS_USE_COLUMN_MAJOR
  /** @brief Get a row of the matrix as a vector
   *  @param i Row number 0 indexed */
  inline const TinyVector3 getRow(int i) const {
    TinyConstants::FullAssert(0 <= i && i < 3);
    return TinyVector3(m_el[0][i], m_el[1][i], m_el[2][i]);
  }
#else
  /** @brief Get a row of the matrix as a vector
   *  @param i Row number 0 indexed */
  inline const TinyVector3& getRow(int i) const {
    TinyConstants::FullAssert(0 <= i && i < 3);
    return m_el[i];
  }
#endif// TDS_USE_COLUMN_MAJOR
  void print(const char* txt) const {
    printf("%s\n", txt);
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        TinyScalar val = getRow(r)[c];
        double v = TinyConstants::getDouble(val);
        printf("%f, ", v);
      }
      printf("\n");
    }
  }

  /**@brief Return the inverse of the matrix */
  TinyMatrix3x3 inverse() const {
    TinyVector3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
    TinyScalar det = (*this)[0].dot(co);
    // btFullAssert(det != TinyScalar(0.0));
    TinyConstants::FullAssert(det != TinyConstants::zero());
    TinyScalar s = TinyConstants::one() / det;
#ifdef TDS_USE_COLUMN_MAJOR
   return TinyMatrix3x3(co.x() * s, co.y() * s, co.z() * s,
                         cofac(0, 2, 2, 1) * s, cofac(0, 0, 2, 2) * s,
                         cofac(0, 1, 2, 0) * s, cofac(0, 1, 1, 2) * s,
                         cofac(0, 2, 1, 0) * s, cofac(0, 0, 1, 1) * s);
#else
    return TinyMatrix3x3(
        co.x() * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s, co.y() * s,
        cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s, co.z() * s,
        cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s);
#endif

  }

  inline TinyMatrix3x3 operator-() const {
    return TinyMatrix3x3(-m_el[0], -m_el[1], -m_el[2]);
  }

  /**@brief Calculate the matrix cofactor
   * @param r1 The first row to use for calculating the cofactor
   * @param c1 The first column to use for calculating the cofactor
   * @param r1 The second row to use for calculating the cofactor
   * @param c1 The second column to use for calculating the cofactor
   * See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
   */
  TinyScalar cofac(int r1, int c1, int r2, int c2) const {
    return m_el[r1][c1] * m_el[r2][c2] - m_el[r1][c2] * m_el[r2][c1];
  }

#ifdef TDS_USE_COLUMN_MAJOR
  TinyScalar determinant() const {
    TinyVector3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
    TinyScalar det = (*this)[0].dot(co);
    return det;
  }
#else
  inline TinyScalar
  determinant() const
  {
  	return btTriple((*this)[0], (*this)[1], (*this)[2]);
  }
#endif
  static const TinyMatrix3x3& get_identity() {
    static const TinyMatrix3x3 identityMatrix(
        TinyConstants::one(), TinyConstants::zero(), TinyConstants::zero(),
        TinyConstants::zero(), TinyConstants::one(), TinyConstants::zero(),
        TinyConstants::zero(), TinyConstants::zero(), TinyConstants::one());
    return identityMatrix;
  }

  static const TinyMatrix3x3& get_zero() {
    static const TinyMatrix3x3 identityMatrix(
        TinyConstants::zero(), TinyConstants::zero(), TinyConstants::zero(),
        TinyConstants::zero(), TinyConstants::zero(), TinyConstants::zero(),
        TinyConstants::zero(), TinyConstants::zero(), TinyConstants::zero());
    return identityMatrix;
  }

#ifdef TDS_USE_COLUMN_MAJOR
  inline TinyVector3 dot(const TinyVector3& v) const {
    return TinyVector3(getRow(0).dot(v), getRow(1).dot(v), getRow(2).dot(v));
  }
#else
  inline TinyVector3 dot(const TinyVector3& v) const {
    return TinyVector3((*this)[0].dot(v), (*this)[1].dot(v), (*this)[2].dot(v));
  }
#endif

#ifdef TDS_USE_COLUMN_MAJOR
  /** @brief Get a column of the matrix as a vector 
	*  @param i Column number 0 indexed */
	inline TinyVector3 getColumn(int i) const
	{
		return m_el[i];
	}
#else

  /** @brief Get a column of the matrix as a vector 
	*  @param i Column number 0 indexed */
	inline TinyVector3 get_column(int i) const
	{
		return TinyVector3(m_el[0][i], m_el[1][i], m_el[2][i]);
	}
#endif
#if 0


	/** @brief Set from the rotational part of a 4x4 OpenGL matrix
	*  @param m A pointer to the beginning of the array of scalars*/
	void setFromOpenGLSubMatrix(const TinyScalar* m)
	{
		m_el[0].setValue(m[0], m[4], m[8]);
		m_el[1].setValue(m[1], m[5], m[9]);
		m_el[2].setValue(m[2], m[6], m[10]);
	}
	
	

	/** @brief Set the matrix from euler angles using YPR around YXZ respectively
	*  @param yaw Yaw about Y axis
	*  @param pitch Pitch about X axis
	*  @param roll Roll about Z axis 
	*/
	void setEulerYPR(const TinyScalar& yaw, const TinyScalar& pitch, const TinyScalar& roll)
	{
		setEulerZYX(roll, pitch, yaw);
	}



	static const TinyMatrix3x3& getIdentity()
	{
#if (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)) || defined(BT_USE_NEON)
		static const TinyMatrix3x3
			identityMatrix(v1000, v0100, v0010);
#else
		static const TinyMatrix3x3
			identityMatrix(
				TinyScalar(1.0), TinyScalar(0.0), TinyScalar(0.0),
				TinyScalar(0.0), TinyScalar(1.0), TinyScalar(0.0),
				TinyScalar(0.0), TinyScalar(0.0), TinyScalar(1.0));
#endif
		return identityMatrix;
	}

	/**@brief Fill the rotational part of an OpenGL matrix and clear the shear/perspective
	* @param m The array to be filled */
	void getOpenGLSubMatrix(TinyScalar * m) const
	{
#if defined BT_USE_SIMD_VECTOR3 && defined(BT_USE_SSE_IN_API) && \
    defined(BT_USE_SSE)
		__m128 v0 = m_el[0].mVec128;
		__m128 v1 = m_el[1].mVec128;
		__m128 v2 = m_el[2].mVec128;  //  x2 y2 z2 w2
		__m128* vm = (__m128*)m;
		__m128 vT;

		v2 = _mm_and_ps(v2, btvFFF0fMask);  //  x2 y2 z2 0

		vT = _mm_unpackhi_ps(v0, v1);  //	z0 z1 * *
		v0 = _mm_unpacklo_ps(v0, v1);  //	x0 x1 y0 y1

		v1 = _mm_shuffle_ps(v0, v2, BT_SHUFFLE(2, 3, 1, 3));                    // y0 y1 y2 0
		v0 = _mm_shuffle_ps(v0, v2, BT_SHUFFLE(0, 1, 0, 3));                    // x0 x1 x2 0
		v2 = btCastdTo128f(_mm_move_sd(btCastfTo128d(v2), btCastfTo128d(vT)));  // z0 z1 z2 0

		vm[0] = v0;
		vm[1] = v1;
		vm[2] = v2;
#elif defined(BT_USE_NEON)
		// note: zeros the w channel. We can preserve it at the cost of two more vtrn instructions.
		static const uint32x2_t zMask = (const uint32x2_t){static_cast<uint32_t>(-1), 0};
		float32x4_t* vm = (float32x4_t*)m;
		float32x4x2_t top = vtrnq_f32(m_el[0].mVec128, m_el[1].mVec128);               // {x0 x1 z0 z1}, {y0 y1 w0 w1}
		float32x2x2_t bl = vtrn_f32(vget_low_f32(m_el[2].mVec128), vdup_n_f32(0.0f));  // {x2  0 }, {y2 0}
		float32x4_t v0 = vcombine_f32(vget_low_f32(top.val[0]), bl.val[0]);
		float32x4_t v1 = vcombine_f32(vget_low_f32(top.val[1]), bl.val[1]);
		float32x2_t q = (float32x2_t)vand_u32((uint32x2_t)vget_high_f32(m_el[2].mVec128), zMask);
		float32x4_t v2 = vcombine_f32(vget_high_f32(top.val[0]), q);  // z0 z1 z2  0

		vm[0] = v0;
		vm[1] = v1;
		vm[2] = v2;
#else
		m[0] = TinyScalar(m_el[0].x());
		m[1] = TinyScalar(m_el[1].x());
		m[2] = TinyScalar(m_el[2].x());
		m[3] = TinyScalar(0.0);
		m[4] = TinyScalar(m_el[0].y());
		m[5] = TinyScalar(m_el[1].y());
		m[6] = TinyScalar(m_el[2].y());
		m[7] = TinyScalar(0.0);
		m[8] = TinyScalar(m_el[0].z());
		m[9] = TinyScalar(m_el[1].z());
		m[10] = TinyScalar(m_el[2].z());
		m[11] = TinyScalar(0.0);
#endif
	}

	
	/**@brief Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
	* @param yaw Yaw around Y axis
	* @param pitch Pitch around X axis
	* @param roll around Z axis */
	void getEulerYPR(TinyScalar & yaw, TinyScalar & pitch, TinyScalar & roll) const
	{
		// first use the normal calculus
		yaw = TinyScalar(btAtan2(m_el[1].x(), m_el[0].x()));
		pitch = TinyScalar(btAsin(-m_el[2].x()));
		roll = TinyScalar(btAtan2(m_el[2].y(), m_el[2].z()));

		// on pitch = +/-HalfPI
		if (btFabs(pitch) == SIMD_HALF_PI)
		{
			if (yaw > 0)
				yaw -= SIMD_PI;
			else
				yaw += SIMD_PI;

			if (roll > 0)
				roll -= SIMD_PI;
			else
				roll += SIMD_PI;
		}
	}

	/**@brief Get the matrix represented as euler angles around ZYX
	* @param yaw Yaw around Z axis
	* @param pitch Pitch around Y axis
	* @param roll around X axis 
	* @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/
	void getEulerZYX(TinyScalar & yaw, TinyScalar & pitch, TinyScalar & roll, unsigned int solution_number = 1) const
	{
		struct Euler
		{
			TinyScalar yaw;
			TinyScalar pitch;
			TinyScalar roll;
		};

		Euler euler_out;
		Euler euler_out2;  //second solution
		//get the pointer to the raw data

		// Check that pitch is not at a singularity
		if (btFabs(m_el[2].x()) >= 1)
		{
			euler_out.yaw = 0;
			euler_out2.yaw = 0;

			// From difference of angles formula
			TinyScalar delta = btAtan2(m_el[0].x(), m_el[0].z());
			if (m_el[2].x() > 0)  //gimbal locked up
			{
				euler_out.pitch = SIMD_PI / TinyScalar(2.0);
				euler_out2.pitch = SIMD_PI / TinyScalar(2.0);
				euler_out.roll = euler_out.pitch + delta;
				euler_out2.roll = euler_out.pitch + delta;
			}
			else  // gimbal locked down
			{
				euler_out.pitch = -SIMD_PI / TinyScalar(2.0);
				euler_out2.pitch = -SIMD_PI / TinyScalar(2.0);
				euler_out.roll = -euler_out.pitch + delta;
				euler_out2.roll = -euler_out.pitch + delta;
			}
		}
		else
		{
			euler_out.pitch = -btAsin(m_el[2].x());
			euler_out2.pitch = SIMD_PI - euler_out.pitch;

			euler_out.roll = btAtan2(m_el[2].y() / btCos(euler_out.pitch),
									 m_el[2].z() / btCos(euler_out.pitch));
			euler_out2.roll = btAtan2(m_el[2].y() / btCos(euler_out2.pitch),
									  m_el[2].z() / btCos(euler_out2.pitch));

			euler_out.yaw = btAtan2(m_el[1].x() / btCos(euler_out.pitch),
									m_el[0].x() / btCos(euler_out.pitch));
			euler_out2.yaw = btAtan2(m_el[1].x() / btCos(euler_out2.pitch),
									 m_el[0].x() / btCos(euler_out2.pitch));
		}

		if (solution_number == 1)
		{
			yaw = euler_out.yaw;
			pitch = euler_out.pitch;
			roll = euler_out.roll;
		}
		else
		{
			yaw = euler_out2.yaw;
			pitch = euler_out2.pitch;
			roll = euler_out2.roll;
		}
	}

	/**@brief Create a scaled copy of the matrix 
	* @param s Scaling vector The elements of the vector will scale each column */

	TinyMatrix3x3 scaled(const TinyVector3& s) const
	{
#if (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)) || defined(BT_USE_NEON)
		return TinyMatrix3x3(m_el[0] * s, m_el[1] * s, m_el[2] * s);
#else
		return TinyMatrix3x3(
			m_el[0].x() * s.x(), m_el[0].y() * s.y(), m_el[0].z() * s.z(),
			m_el[1].x() * s.x(), m_el[1].y() * s.y(), m_el[1].z() * s.z(),
			m_el[2].x() * s.x(), m_el[2].y() * s.y(), m_el[2].z() * s.z());
#endif
	}

	/**@brief Return the adjoint of the matrix */
	TinyMatrix3x3 adjoint() const;
	/**@brief Return the matrix with all values non negative */
	TinyMatrix3x3 absolute() const;

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	///Solve33 is from Box2d, thanks to Erin Catto,
	TinyVector3 solve33(const TinyVector3& b) const
	{
		TinyVector3 col1 = getColumn(0);
		TinyVector3 col2 = getColumn(1);
		TinyVector3 col3 = getColumn(2);

		TinyScalar det = btDot(col1, btCross(col2, col3));
		if (btFabs(det) > SIMD_EPSILON)
		{
			det = 1.0f / det;
		}
		TinyVector3 x;
		x[0] = det * btDot(b, btCross(col2, col3));
		x[1] = det * btDot(col1, btCross(b, col3));
		x[2] = det * btDot(col1, btCross(col2, b));
		return x;
	}

	TinyMatrix3x3 transposeTimes(const TinyMatrix3x3& m) const;
	TinyMatrix3x3 timesTranspose(const TinyMatrix3x3& m) const;

	

	///extractRotation is from "A robust method to extract the rotational part of deformations"
	///See http://dl.acm.org/citation.cfm?doid=2994258.2994269
	///decomposes a matrix A in a orthogonal matrix R and a
	///symmetric matrix S:
	///A = R*S.
	///note that R can include both rotation and scaling.
	inline void extractRotation(btQuaternion & q, TinyScalar tolerance = 1.0e-9, int maxIter = 100)
	{
		int iter = 0;
		TinyScalar w;
		const TinyMatrix3x3& A = *this;
		for (iter = 0; iter < maxIter; iter++)
		{
			TinyMatrix3x3 R(q);
			TinyVector3 omega = (R.getColumn(0).cross(A.getColumn(0)) + R.getColumn(1).cross(A.getColumn(1)) + R.getColumn(2).cross(A.getColumn(2))) * (TinyScalar(1.0) / btFabs(R.getColumn(0).dot(A.getColumn(0)) + R.getColumn(1).dot(A.getColumn(1)) + R.getColumn(2).dot(A.getColumn(2))) +
																																					  tolerance);
			w = omega.norm();
			if (w < tolerance)
				break;
			q = btQuaternion(TinyVector3((TinyScalar(1.0) / w) * omega), w) *
				q;
			q.normalize();
		}
	}

	/**@brief diagonalizes this matrix by the Jacobi method.
	* @param rot stores the rotation from the coordinate system in which the matrix is diagonal to the original
	* coordinate system, i.e., old_this = rot * new_this * rot^T.
	* @param threshold See iteration
	* @param iteration The iteration stops when all off-diagonal elements are less than the threshold multiplied
	* by the sum of the absolute values of the diagonal, or when maxSteps have been executed.
	*
	* Note that this matrix is assumed to be symmetric.
	*/
	void diagonalize(TinyMatrix3x3 & rot, TinyScalar threshold, int maxSteps)
	{
		rot.setIdentity();
		for (int step = maxSteps; step > 0; step--)
		{
			// find off-diagonal element [p][q] with largest magnitude
			int p = 0;
			int q = 1;
			int r = 2;
			TinyScalar max = btFabs(m_el[0][1]);
			TinyScalar v = btFabs(m_el[0][2]);
			if (v > max)
			{
				q = 2;
				r = 1;
				max = v;
			}
			v = btFabs(m_el[1][2]);
			if (v > max)
			{
				p = 1;
				q = 2;
				r = 0;
				max = v;
			}

			TinyScalar t = threshold * (btFabs(m_el[0][0]) + btFabs(m_el[1][1]) + btFabs(m_el[2][2]));
			if (max <= t)
			{
				if (max <= SIMD_EPSILON * t)
				{
					return;
				}
				step = 1;
			}

			// compute Jacobi rotation J which leads to a zero for element [p][q]
			TinyScalar mpq = m_el[p][q];
			TinyScalar theta = (m_el[q][q] - m_el[p][p]) / (2 * mpq);
			TinyScalar theta2 = theta * theta;
			TinyScalar cos;
			TinyScalar sin;
			if (theta2 * theta2 < TinyScalar(10 / SIMD_EPSILON))
			{
				t = (theta >= 0) ? 1 / (theta + btSqrt(1 + theta2))
								 : 1 / (theta - btSqrt(1 + theta2));
				cos = 1 / btSqrt(1 + t * t);
				sin = cos * t;
			}
			else
			{
				// approximation for large theta-value, i.e., a nearly diagonal matrix
				t = 1 / (theta * (2 + TinyScalar(0.5) / theta2));
				cos = 1 - TinyScalar(0.5) * t * t;
				sin = cos * t;
			}

			// apply rotation to matrix (this = J^T * this * J)
			m_el[p][q] = m_el[q][p] = 0;
			m_el[p][p] -= t * mpq;
			m_el[q][q] += t * mpq;
			TinyScalar mrp = m_el[r][p];
			TinyScalar mrq = m_el[r][q];
			m_el[r][p] = m_el[p][r] = cos * mrp - sin * mrq;
			m_el[r][q] = m_el[q][r] = cos * mrq + sin * mrp;

			// apply rotation to rot (rot = rot * J)
			for (int i = 0; i < 3; i++)
			{
				TinyVector3& row = rot[i];
				mrp = row[p];
				mrq = row[q];
				row[p] = cos * mrp - sin * mrq;
				row[q] = cos * mrq + sin * mrp;
			}
		}
	}

	void serialize(struct TinyMatrix3x3Data & dataOut) const;

	void serializeFloat(struct TinyMatrix3x3FloatData & dataOut) const;

	void deSerialize(const struct TinyMatrix3x3Data& dataIn);

	void deSerializeFloat(const struct TinyMatrix3x3FloatData& dataIn);

	void deSerializeDouble(const struct TinyMatrix3x3DoubleData& dataIn);
#endif
};

template <typename TinyScalar, typename TinyConstants>
inline TinyVector3<TinyScalar, TinyConstants> operator*(
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m,
    const TinyVector3<TinyScalar, TinyConstants>& v) {
#ifdef TDS_USE_COLUMN_MAJOR
  return TinyVector3<TinyScalar, TinyConstants>(
      m.getRow(0).dot(v), m.getRow(1).dot(v), m.getRow(2).dot(v));
#else
  return TinyVector3<TinyScalar, TinyConstants>(m[0].dot(v), m[1].dot(v),
                                                m[2].dot(v));
#endif
}

template <typename TinyScalar, typename TinyConstants>
inline TinyMatrix3x3<TinyScalar, TinyConstants>
TinyMatrix3x3<TinyScalar, TinyConstants>::transpose() const {
#ifdef TDS_USE_COLUMN_MAJOR
  return TinyMatrix3x3<TinyScalar, TinyConstants>(getRow(0), getRow(1),
                                                  getRow(2));
#else

  return TinyMatrix3x3<TinyScalar, TinyConstants>(
      m_el[0].x(), m_el[1].x(), m_el[2].x(), m_el[0].y(), m_el[1].y(),
      m_el[2].y(), m_el[0].z(), m_el[1].z(), m_el[2].z());
#endif
}

template <typename TinyScalar, typename TinyConstants>
inline TinyMatrix3x3<TinyScalar, TinyConstants> TinyVectorCrossMatrix(
    const TinyVector3<TinyScalar, TinyConstants>& vector) {
  return TinyMatrix3x3<TinyScalar, TinyConstants>(
      TinyConstants::zero(), -vector[2], vector[1], vector[2],
      TinyConstants::zero(), -vector[0], -vector[1], vector[0],
      TinyConstants::zero());
}

template <typename TinyScalar, typename TinyConstants>
inline TinyMatrix3x3<TinyScalar, TinyConstants> operator*(
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m, const TinyScalar& k) {
#ifdef TDS_USE_COLUMN_MAJOR
  return TinyMatrix3x3<TinyScalar, TinyConstants>(m[0] * k, m[1] * k, m[2] * k);
#else
  return TinyMatrix3x3<TinyScalar, TinyConstants>(
      m[0].x() * k, m[0].y() * k, m[0].z() * k, m[1].x() * k, m[1].y() * k,
      m[1].z() * k, m[2].x() * k, m[2].y() * k, m[2].z() * k);
#endif
}

template <typename TinyScalar, typename TinyConstants>
inline TinyMatrix3x3<TinyScalar, TinyConstants> operator+(
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m1,
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m2) {
#ifdef TDS_USE_COLUMN_MAJOR
  return TinyMatrix3x3<TinyScalar, TinyConstants>(m1[0] + m2[0], m1[1] + m2[1],
                                                  m1[2] + m2[2]);
#else
  return TinyMatrix3x3<TinyScalar, TinyConstants>(
      m1[0][0] + m2[0][0], m1[0][1] + m2[0][1], m1[0][2] + m2[0][2],

      m1[1][0] + m2[1][0], m1[1][1] + m2[1][1], m1[1][2] + m2[1][2],

      m1[2][0] + m2[2][0], m1[2][1] + m2[2][1], m1[2][2] + m2[2][2]);
#endif
}
template <typename TinyScalar, typename TinyConstants>
inline TinyMatrix3x3<TinyScalar, TinyConstants> operator-(
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m1,
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m2) {
#ifdef TDS_USE_COLUMN_MAJOR
  return TinyMatrix3x3<TinyScalar, TinyConstants>(m1[0] - m2[0], m1[1] - m2[1],
                                                  m1[2] - m2[2]);
#else
  return TinyMatrix3x3<TinyScalar, TinyConstants>(
      m1[0][0] - m2[0][0], m1[0][1] - m2[0][1], m1[0][2] - m2[0][2],

      m1[1][0] - m2[1][0], m1[1][1] - m2[1][1], m1[1][2] - m2[1][2],

      m1[2][0] - m2[2][0], m1[2][1] - m2[2][1], m1[2][2] - m2[2][2]);
#endif
}
template <typename TinyScalar, typename TinyConstants>
inline TinyMatrix3x3<TinyScalar, TinyConstants> operator*(
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m1,
    const TinyMatrix3x3<TinyScalar, TinyConstants>& m2) {
#ifdef TDS_USE_COLUMN_MAJOR
  return TinyMatrix3x3<TinyScalar, TinyConstants>(
      m1.tdotx(m2[0]), m1.tdotx(m2[1]), m1.tdotx(m2[2]), m1.tdoty(m2[0]),
      m1.tdoty(m2[1]), m1.tdoty(m2[2]), m1.tdotz(m2[0]), m1.tdotz(m2[1]),
      m1.tdotz(m2[2]));
#else
  return TinyMatrix3x3<TinyScalar, TinyConstants>(
      m2.tdotx(m1[0]), m2.tdoty(m1[0]), m2.tdotz(m1[0]), m2.tdotx(m1[1]),
      m2.tdoty(m1[1]), m2.tdotz(m1[1]), m2.tdotx(m1[2]), m2.tdoty(m1[2]),
      m2.tdotz(m1[2]));
#endif
}

#if 0

inline TinyMatrix3x3
TinyMatrix3x3::absolute() const
{
#if defined BT_USE_SIMD_VECTOR3 && \
    (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))
	return TinyMatrix3x3(
		_mm_and_ps(m_el[0].mVec128, btvAbsfMask),
		_mm_and_ps(m_el[1].mVec128, btvAbsfMask),
		_mm_and_ps(m_el[2].mVec128, btvAbsfMask));
#elif defined(BT_USE_NEON)
	return TinyMatrix3x3(
		(float32x4_t)vandq_s32((int32x4_t)m_el[0].mVec128, btv3AbsMask),
		(float32x4_t)vandq_s32((int32x4_t)m_el[1].mVec128, btv3AbsMask),
		(float32x4_t)vandq_s32((int32x4_t)m_el[2].mVec128, btv3AbsMask));
#else
	return TinyMatrix3x3(
		btFabs(m_el[0].x()), btFabs(m_el[0].y()), btFabs(m_el[0].z()),
		btFabs(m_el[1].x()), btFabs(m_el[1].y()), btFabs(m_el[1].z()),
		btFabs(m_el[2].x()), btFabs(m_el[2].y()), btFabs(m_el[2].z()));
#endif
}



inline TinyMatrix3x3
TinyMatrix3x3::adjoint() const
{
	return TinyMatrix3x3(cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0, 1, 1, 2),
					   cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
					   cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1));
}

inline TinyMatrix3x3
TinyMatrix3x3::transposeTimes(const TinyMatrix3x3& m) const
{
#if defined BT_USE_SIMD_VECTOR3 && \
    (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))
	// zeros w
	//    static const __m128i xyzMask = (const __m128i){ -1ULL, 0xffffffffULL };
	__m128 row = m_el[0].mVec128;
	__m128 m0 = _mm_and_ps(m.getRow(0).mVec128, btvFFF0fMask);
	__m128 m1 = _mm_and_ps(m.getRow(1).mVec128, btvFFF0fMask);
	__m128 m2 = _mm_and_ps(m.getRow(2).mVec128, btvFFF0fMask);
	__m128 r0 = _mm_mul_ps(m0, _mm_shuffle_ps(row, row, 0));
	__m128 r1 = _mm_mul_ps(m0, _mm_shuffle_ps(row, row, 0x55));
	__m128 r2 = _mm_mul_ps(m0, _mm_shuffle_ps(row, row, 0xaa));
	row = m_el[1].mVec128;
	r0 = _mm_add_ps(r0, _mm_mul_ps(m1, _mm_shuffle_ps(row, row, 0)));
	r1 = _mm_add_ps(r1, _mm_mul_ps(m1, _mm_shuffle_ps(row, row, 0x55)));
	r2 = _mm_add_ps(r2, _mm_mul_ps(m1, _mm_shuffle_ps(row, row, 0xaa)));
	row = m_el[2].mVec128;
	r0 = _mm_add_ps(r0, _mm_mul_ps(m2, _mm_shuffle_ps(row, row, 0)));
	r1 = _mm_add_ps(r1, _mm_mul_ps(m2, _mm_shuffle_ps(row, row, 0x55)));
	r2 = _mm_add_ps(r2, _mm_mul_ps(m2, _mm_shuffle_ps(row, row, 0xaa)));
	return TinyMatrix3x3(r0, r1, r2);

#elif defined BT_USE_NEON
	// zeros w
	static const uint32x4_t xyzMask = (const uint32x4_t){static_cast<uint32_t>(-1), static_cast<uint32_t>(-1), static_cast<uint32_t>(-1), 0};
	float32x4_t m0 = (float32x4_t)vandq_u32((uint32x4_t)m.getRow(0).mVec128, xyzMask);
	float32x4_t m1 = (float32x4_t)vandq_u32((uint32x4_t)m.getRow(1).mVec128, xyzMask);
	float32x4_t m2 = (float32x4_t)vandq_u32((uint32x4_t)m.getRow(2).mVec128, xyzMask);
	float32x4_t row = m_el[0].mVec128;
	float32x4_t r0 = vmulq_lane_f32(m0, vget_low_f32(row), 0);
	float32x4_t r1 = vmulq_lane_f32(m0, vget_low_f32(row), 1);
	float32x4_t r2 = vmulq_lane_f32(m0, vget_high_f32(row), 0);
	row = m_el[1].mVec128;
	r0 = vmlaq_lane_f32(r0, m1, vget_low_f32(row), 0);
	r1 = vmlaq_lane_f32(r1, m1, vget_low_f32(row), 1);
	r2 = vmlaq_lane_f32(r2, m1, vget_high_f32(row), 0);
	row = m_el[2].mVec128;
	r0 = vmlaq_lane_f32(r0, m2, vget_low_f32(row), 0);
	r1 = vmlaq_lane_f32(r1, m2, vget_low_f32(row), 1);
	r2 = vmlaq_lane_f32(r2, m2, vget_high_f32(row), 0);
	return TinyMatrix3x3(r0, r1, r2);
#else
	return TinyMatrix3x3(
		m_el[0].x() * m[0].x() + m_el[1].x() * m[1].x() + m_el[2].x() * m[2].x(),
		m_el[0].x() * m[0].y() + m_el[1].x() * m[1].y() + m_el[2].x() * m[2].y(),
		m_el[0].x() * m[0].z() + m_el[1].x() * m[1].z() + m_el[2].x() * m[2].z(),
		m_el[0].y() * m[0].x() + m_el[1].y() * m[1].x() + m_el[2].y() * m[2].x(),
		m_el[0].y() * m[0].y() + m_el[1].y() * m[1].y() + m_el[2].y() * m[2].y(),
		m_el[0].y() * m[0].z() + m_el[1].y() * m[1].z() + m_el[2].y() * m[2].z(),
		m_el[0].z() * m[0].x() + m_el[1].z() * m[1].x() + m_el[2].z() * m[2].x(),
		m_el[0].z() * m[0].y() + m_el[1].z() * m[1].y() + m_el[2].z() * m[2].y(),
		m_el[0].z() * m[0].z() + m_el[1].z() * m[1].z() + m_el[2].z() * m[2].z());
#endif
}

inline TinyMatrix3x3
TinyMatrix3x3::timesTranspose(const TinyMatrix3x3& m) const
{
#if (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))
	__m128 a0 = m_el[0].mVec128;
	__m128 a1 = m_el[1].mVec128;
	__m128 a2 = m_el[2].mVec128;

	TinyMatrix3x3 mT = m.transpose();  // we rely on transpose() zeroing w channel so that we don't have to do it here
	__m128 mx = mT[0].mVec128;
	__m128 my = mT[1].mVec128;
	__m128 mz = mT[2].mVec128;

	__m128 r0 = _mm_mul_ps(mx, _mm_shuffle_ps(a0, a0, 0x00));
	__m128 r1 = _mm_mul_ps(mx, _mm_shuffle_ps(a1, a1, 0x00));
	__m128 r2 = _mm_mul_ps(mx, _mm_shuffle_ps(a2, a2, 0x00));
	r0 = _mm_add_ps(r0, _mm_mul_ps(my, _mm_shuffle_ps(a0, a0, 0x55)));
	r1 = _mm_add_ps(r1, _mm_mul_ps(my, _mm_shuffle_ps(a1, a1, 0x55)));
	r2 = _mm_add_ps(r2, _mm_mul_ps(my, _mm_shuffle_ps(a2, a2, 0x55)));
	r0 = _mm_add_ps(r0, _mm_mul_ps(mz, _mm_shuffle_ps(a0, a0, 0xaa)));
	r1 = _mm_add_ps(r1, _mm_mul_ps(mz, _mm_shuffle_ps(a1, a1, 0xaa)));
	r2 = _mm_add_ps(r2, _mm_mul_ps(mz, _mm_shuffle_ps(a2, a2, 0xaa)));
	return TinyMatrix3x3(r0, r1, r2);

#elif defined BT_USE_NEON
	float32x4_t a0 = m_el[0].mVec128;
	float32x4_t a1 = m_el[1].mVec128;
	float32x4_t a2 = m_el[2].mVec128;

	TinyMatrix3x3 mT = m.transpose();  // we rely on transpose() zeroing w channel so that we don't have to do it here
	float32x4_t mx = mT[0].mVec128;
	float32x4_t my = mT[1].mVec128;
	float32x4_t mz = mT[2].mVec128;

	float32x4_t r0 = vmulq_lane_f32(mx, vget_low_f32(a0), 0);
	float32x4_t r1 = vmulq_lane_f32(mx, vget_low_f32(a1), 0);
	float32x4_t r2 = vmulq_lane_f32(mx, vget_low_f32(a2), 0);
	r0 = vmlaq_lane_f32(r0, my, vget_low_f32(a0), 1);
	r1 = vmlaq_lane_f32(r1, my, vget_low_f32(a1), 1);
	r2 = vmlaq_lane_f32(r2, my, vget_low_f32(a2), 1);
	r0 = vmlaq_lane_f32(r0, mz, vget_high_f32(a0), 0);
	r1 = vmlaq_lane_f32(r1, mz, vget_high_f32(a1), 0);
	r2 = vmlaq_lane_f32(r2, mz, vget_high_f32(a2), 0);
	return TinyMatrix3x3(r0, r1, r2);

#else
	return TinyMatrix3x3(
		m_el[0].dot(m[0]), m_el[0].dot(m[1]), m_el[0].dot(m[2]),
		m_el[1].dot(m[0]), m_el[1].dot(m[1]), m_el[1].dot(m[2]),
		m_el[2].dot(m[0]), m_el[2].dot(m[1]), m_el[2].dot(m[2]));
#endif
}


inline TinyVector3
operator*(const TinyVector3& v, const TinyMatrix3x3& m)
{
#if defined BT_USE_SIMD_VECTOR3 && \
    (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))

	const __m128 vv = v.mVec128;

	__m128 c0 = bt_splat_ps(vv, 0);
	__m128 c1 = bt_splat_ps(vv, 1);
	__m128 c2 = bt_splat_ps(vv, 2);

	c0 = _mm_mul_ps(c0, _mm_and_ps(m[0].mVec128, btvFFF0fMask));
	c1 = _mm_mul_ps(c1, _mm_and_ps(m[1].mVec128, btvFFF0fMask));
	c0 = _mm_add_ps(c0, c1);
	c2 = _mm_mul_ps(c2, _mm_and_ps(m[2].mVec128, btvFFF0fMask));

	return TinyVector3(_mm_add_ps(c0, c2));
#elif defined(BT_USE_NEON)
	const float32x4_t vv = v.mVec128;
	const float32x2_t vlo = vget_low_f32(vv);
	const float32x2_t vhi = vget_high_f32(vv);

	float32x4_t c0, c1, c2;

	c0 = (float32x4_t)vandq_s32((int32x4_t)m[0].mVec128, btvFFF0Mask);
	c1 = (float32x4_t)vandq_s32((int32x4_t)m[1].mVec128, btvFFF0Mask);
	c2 = (float32x4_t)vandq_s32((int32x4_t)m[2].mVec128, btvFFF0Mask);

	c0 = vmulq_lane_f32(c0, vlo, 0);
	c1 = vmulq_lane_f32(c1, vlo, 1);
	c2 = vmulq_lane_f32(c2, vhi, 0);
	c0 = vaddq_f32(c0, c1);
	c0 = vaddq_f32(c0, c2);

	return TinyVector3(c0);
#else
	return TinyVector3(m.tdotx(v), m.tdoty(v), m.tdotz(v));
#endif
}



/*
inline TinyMatrix3x3 btMultTransposeLeft(const TinyMatrix3x3& m1, const TinyMatrix3x3& m2) {
return TinyMatrix3x3(
m1[0][0] * m2[0][0] + m1[1][0] * m2[1][0] + m1[2][0] * m2[2][0],
m1[0][0] * m2[0][1] + m1[1][0] * m2[1][1] + m1[2][0] * m2[2][1],
m1[0][0] * m2[0][2] + m1[1][0] * m2[1][2] + m1[2][0] * m2[2][2],
m1[0][1] * m2[0][0] + m1[1][1] * m2[1][0] + m1[2][1] * m2[2][0],
m1[0][1] * m2[0][1] + m1[1][1] * m2[1][1] + m1[2][1] * m2[2][1],
m1[0][1] * m2[0][2] + m1[1][1] * m2[1][2] + m1[2][1] * m2[2][2],
m1[0][2] * m2[0][0] + m1[1][2] * m2[1][0] + m1[2][2] * m2[2][0],
m1[0][2] * m2[0][1] + m1[1][2] * m2[1][1] + m1[2][2] * m2[2][1],
m1[0][2] * m2[0][2] + m1[1][2] * m2[1][2] + m1[2][2] * m2[2][2]);
}
*/

/**@brief Equality operator between two matrices
* It will test all elements are equal.  */
inline bool operator==(const TinyMatrix3x3& m1, const TinyMatrix3x3& m2)
{
#if (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE))

	__m128 c0, c1, c2;

	c0 = _mm_cmpeq_ps(m1[0].mVec128, m2[0].mVec128);
	c1 = _mm_cmpeq_ps(m1[1].mVec128, m2[1].mVec128);
	c2 = _mm_cmpeq_ps(m1[2].mVec128, m2[2].mVec128);

	c0 = _mm_and_ps(c0, c1);
	c0 = _mm_and_ps(c0, c2);

	int m = _mm_movemask_ps((__m128)c0);
	return (0x7 == (m & 0x7));

#else
	return (m1[0][0] == m2[0][0] && m1[1][0] == m2[1][0] && m1[2][0] == m2[2][0] &&
			m1[0][1] == m2[0][1] && m1[1][1] == m2[1][1] && m1[2][1] == m2[2][1] &&
			m1[0][2] == m2[0][2] && m1[1][2] == m2[1][2] && m1[2][2] == m2[2][2]);
#endif
}
#endif
};
#endif  // _TINY_MATRIX3x3_H
