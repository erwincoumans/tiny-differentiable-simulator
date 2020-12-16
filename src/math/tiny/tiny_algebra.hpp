#pragma once

#include <vector>

#include "../spatial_vector.hpp"
#include "tiny_matrix3x3.h"
#include "tiny_matrix6x6.h"
#include "tiny_matrix6x3.h"
#include "tiny_matrix_x.h"
#include "tiny_quaternion.h"
#include "tiny_vector3.h"
#include "tiny_vector_x.h"
#include "../transform.hpp"
#undef min
#undef max

template <typename TinyScalar, typename TinyConstants>
struct TinyAlgebra {
  using Index = int;
  using Scalar = TinyScalar;
  using Vector3 = ::TINY::TinyVector3<TinyScalar, TinyConstants>;
  using VectorX = ::TINY::TinyVectorX<TinyScalar, TinyConstants>;
  using Matrix3 = ::TINY::TinyMatrix3x3<TinyScalar, TinyConstants>;
  using Matrix6 = ::TINY::TinyMatrix6x6<TinyScalar, TinyConstants>;
  using Matrix3X = ::TINY::TinyMatrix3xX<TinyScalar, TinyConstants>;
  using Matrix6x3 = ::TINY::TinyMatrix6x3<TinyScalar, TinyConstants>;
  using MatrixX = ::TINY::TinyMatrixXxX<TinyScalar, TinyConstants>;
  using Quaternion = ::TINY::TinyQuaternion<TinyScalar, TinyConstants>;
  using SpatialVector = tds::SpatialVector<TinyAlgebra>;
  using Transform = tds::Transform<TinyAlgebra>;
  using MotionVector = tds::MotionVector<TinyAlgebra>;
  using ForceVector = tds::ForceVector<TinyAlgebra>;

  template <typename T>
  TINY_INLINE static auto transpose(const T &matrix) {
    return matrix.transpose();
  }

  TINY_INLINE static auto inverse(const Quaternion &quat) {
    return quat.inversed();
  }
  template <typename T>
  TINY_INLINE static auto inverse(const T &matrix) {
    return matrix.inverse();
  }

  template <typename T>
  TINY_INLINE static auto inverse_transpose(const T &matrix) {
    return matrix.inverse().transpose();
  }

  template <typename T1, typename T2>
  TINY_INLINE static auto cross(const T1 &vector_a, const T2 &vector_b) {
    return vector_a.cross(vector_b);
  }

  TINY_INLINE static Index size(const VectorX &v) { return v.m_size; }

  TINY_INLINE static Matrix3X create_matrix_3x(int num_cols) {
    return Matrix3X(num_cols);
  }
  TINY_INLINE static MatrixX create_matrix_x(int num_rows, int num_cols) {
    return MatrixX(num_rows, num_cols);
  }

  TINY_INLINE static int num_rows(const Vector3 &v) {
    return 3;
  }
  TINY_INLINE static int num_rows(const VectorX &v) {
    return v.m_size;
  }
  TINY_INLINE static int num_rows(const SpatialVector &v) {
    return 6;
  }
  template <typename T>
  TINY_INLINE static int num_rows(const T &matrix) {
    return matrix.m_rows;
  }

  template <typename T>
  TINY_INLINE static int num_cols(const T &matrix) {
    return matrix.m_cols;
  }

  /**
   * Returns true if the matrix `mat` is positive-definite, and assigns
   * `mat_inv` to the inverse of mat.
   * `mat` must be a symmetric matrix.
   */
  TINY_INLINE static bool symmetric_inverse(const MatrixX &mat,
                                            MatrixX &mat_inv) {
    return mat.inversed(mat_inv);
  }

  /**
   * V1 = mv(w1, v1)
   * V2 = mv(w2, v2)
   * V1 x V2 = mv(w1 x w2, w1 x v2 + v1 x w2)
   */
  static inline MotionVector cross(const MotionVector &a,
                                   const MotionVector &b) {
    return MotionVector(a.top.cross(b.top),
                        a.top.cross(b.bottom) + a.bottom.cross(b.top));
  }

  /**
   * V = mv(w, v)
   * F = fv(n, f)
   * V x* F = fv(w x n + v x f, w x f)
   */
  static inline ForceVector cross(const MotionVector &a, const ForceVector &b) {
    return ForceVector(a.top.cross(b.top) + a.bottom.cross(b.bottom),
                       a.top.cross(b.bottom));
  }

  /**
   * V = mv(w, v)
   * F = mv(n, f)
   * V.F = w.n + v.f
   */
  TINY_INLINE static Scalar dot(const MotionVector &a, const ForceVector &b) {
    return a.top.dot(b.top) + a.bottom.dot(b.bottom);
  }
  TINY_INLINE static Scalar dot(const ForceVector &a, const MotionVector &b) {
    return dot(b, a);
  }
  /**
   * Multiplication of a matrix6x3 with a force/motion vector handles the operation as a multiplication with the
   * transpose of the matrix.
   * @param a [6x3] matrix
   * @param b MotionVector or ForceVector
   * @return Vector3
   */
  TINY_INLINE static Vector3 dot(const Matrix6x3 &a,
                                 const ForceVector &b) {
    Vector3 res;
    res = a.m_top.transpose() * b.top + a.m_bottom.transpose() * b.bottom;
    return res;
  }
  TINY_INLINE static Vector3 dot(const Matrix6x3 &a,
                                 const MotionVector &b) {
    Vector3 res;
    res = a.m_top.transpose() * b.top + a.m_bottom.transpose() * b.bottom;
    return res;
  }

  template <typename T1, typename T2>
  TINY_INLINE static auto dot(const T1 &vector_a, const T2 &vector_b) {
    return vector_a.dot(vector_b);
  }

    template <template <typename, typename> typename ColumnType>
    TINY_INLINE static VectorX mul_transpose(
            const ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &mat,
            const ColumnType<TinyScalar, TinyConstants> &vec) {
        return mat.mul_transpose(vec);
    }
/**
     * Multiplication of a 6x3 matrix with a Vector3 returns a force vector
     * @param a
     * @param b
     * @return
     */
    TINY_INLINE static ForceVector mul_2_force_vector(
            const Matrix6x3 &mat,
            const Vector3 &vec) {
        ForceVector res;
        res.top = mat.m_top * vec;
        res.bottom = mat.m_bottom * vec;
        return res;
    }
    TINY_INLINE static MotionVector mul_2_motion_vector(
            const Matrix6x3 &mat,
            const Vector3 &vec) {
        MotionVector res;
        res.top = mat.m_top * vec;
        res.bottom = mat.m_bottom * vec;
        return res;
    }

    TINY_INLINE static Scalar norm(const MotionVector &v) {
    return TinyConstants::sqrt1(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] +
                                v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
  }

  TINY_INLINE static Scalar norm(const ForceVector &v) {
    return TinyConstants::sqrt1(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] +
                                v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
  }
  template <typename T>
  TINY_INLINE static Scalar norm(const T &v) {
    return v.length();
  }
  template <typename T>
  TINY_INLINE static Scalar sqnorm(const T &v) {
    return v.sqnorm();
  }

  TINY_INLINE static auto normalize(const Quaternion &q) {
    Scalar ql = q.length();
    return Quaternion(q.x() / ql, q.y() / ql, q.z() / ql, q.w() / ql);
  }
  TINY_INLINE static auto normalize(const Vector3 &v) { return v * (one() / v.length()); }
  TINY_INLINE static auto normalize(const VectorX &v) { return v * (one() / v.length()); }

  /**
   * Cross product in matrix form.
   */
  TINY_INLINE static Matrix3 cross_matrix(const Vector3 &v) {
    return TinyVectorCrossMatrix(v);
  }

  TINY_INLINE static Vector3 zero3() { return Vector3::zero(); }
  TINY_INLINE static Vector3 unit3_x() { return Vector3::makeUnitX(); }
  TINY_INLINE static Vector3 unit3_y() { return Vector3::makeUnitY(); }
  TINY_INLINE static Vector3 unit3_z() { return Vector3::makeUnitZ(); }

  TINY_INLINE static Matrix3 zero33() {
    const Scalar o = TinyConstants::zero();
    return Matrix3(o, o, o, o, o, o, o, o, o);
  }
  TINY_INLINE static Matrix6 zero66() {
    const Scalar o = TinyConstants::zero();
    return Matrix6(o, o, o, o, o, o, o, o, o, o, o, o, o, o, o, o, o, o, o, o,
                   o, o, o, o, o, o, o, o, o, o, o, o, o, o, o, o);
  }
  TINY_INLINE static VectorX zerox(Index size) {
    VectorX v(size);
    v.set_zero();
    return v;
  }
  TINY_INLINE static Matrix3 diagonal3(const Vector3 &v) {
    const Scalar o = TinyConstants::zero();
    return Matrix3(v[0], o, o, o, v[1], o, o, o, v[2]);
  }
  TINY_INLINE static Matrix3 diagonal3(const Scalar &v) {
    const Scalar o = TinyConstants::zero();
    return Matrix3(v, o, o, o, v, o, o, o, v);
  }
  /**
   * Returns a 3x3 identity matrix.
   */
  TINY_INLINE static Matrix3 eye3() { return diagonal3(TinyConstants::one()); }
  TINY_INLINE static void set_identity(Quaternion &quat) {
    quat.set_identity();
  }

  TINY_INLINE static Scalar determinant(const Matrix3 &m) {
    return m.determinant();
  }

  TINY_INLINE static Scalar zero() { return TinyConstants::zero(); }
  TINY_INLINE static Scalar one() { return TinyConstants::one(); }
  TINY_INLINE static Scalar two() { return TinyConstants::two(); }
  TINY_INLINE static Scalar half() { return TinyConstants::half(); }
  TINY_INLINE static Scalar fraction(int a, int b) {
    return TinyConstants::fraction(a, b);
  }
  TINY_INLINE static Scalar pi() { return TinyConstants::pi(); }

  TINY_INLINE static Scalar scalar_from_string(const std::string &s) {
    return TinyConstants::scalar_from_string(s);
  }

  TINY_INLINE static VectorX segment(const VectorX &vec, int start_index,
                                     int length) {
    return vec.segment(start_index, length);
  }

  TINY_INLINE static MatrixX block(const MatrixX &mat, int start_row_index,
                                   int start_col_index, int rows, int cols) {
    return mat.block(start_row_index, start_col_index, rows, cols);
  }

  TINY_INLINE static Matrix3 top(const Matrix6x3 &mat) {
    return mat.m_top;
  }

  TINY_INLINE static Matrix3 bottom(const Matrix6x3 &mat) {
    return mat.m_bottom;
  }

  template <template <typename, typename> typename VectorType>
  TINY_INLINE static void assign_horizontal(
      MatrixX &mat, const VectorType<TinyScalar, TinyConstants> &vec,
      int start_row_index, int start_col_index) {
    mat.assign_vector_horizontal(start_row_index, start_col_index, vec);
  }

  template <template <typename, typename> typename VectorType>
  TINY_INLINE static void assign_vertical(
      MatrixX &mat, const VectorType<TinyScalar, TinyConstants> &vec,
      int start_row_index, int start_col_index) {
    mat.assign_vector_vertical(start_row_index, start_col_index, vec);
  }

  TINY_INLINE static void assign_block(Matrix3 &output, const Matrix6 &input,
                                       Index i, Index j, Index m = 3,
                                       Index n = 3, Index input_i = 0,
                                       Index input_j = 0) {
    if (input_i == 0) {
      if (input_j == 0) {
        output = input.m_topLeftMat;
      } else {
        output = input.m_topRightMat;
      }
    } else {
      if (input_j == 0) {
        output = input.m_bottomLeftMat;
      } else {
        output = input.m_bottomRightMat;
      }
    }
  }
  TINY_INLINE static void assign_block(Matrix6 &output, const Matrix3 &input,
                                       Index i, Index j, Index m = 3,
                                       Index n = 3, Index input_i = 0,
                                       Index input_j = 0) {
    if (i == 0) {
      if (j == 0) {
        output.m_topLeftMat = input;
      } else {
        output.m_topRightMat = input;
      }
    } else {
      if (j == 0) {
        output.m_bottomLeftMat = input;
      } else {
        output.m_bottomRightMat = input;
      }
    }
  }

  TINY_INLINE static void assign_block(Matrix6x3 &output, const Matrix3 &input,
                                       Index i, Index j, Index m = 3,
                                       Index n = 3, Index input_i = 0,
                                       Index input_j = 0) {
      if (i == 0) {
          output.m_top = input;
      } else {
          output.m_bottom = input;
      }
  }

  template <template <typename, typename> typename ColumnType>
  TINY_INLINE static void assign_block(
          ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &output,
          const Matrix3 &input, Index i, Index j, Index m = 3, Index n = 3,
          Index input_i = 0, Index input_j = 0) {
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
      }
    }
  }

  template <template <typename, typename> typename ColumnType>
  TINY_INLINE static void assign_block(
          ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &output,
          const Matrix6x3 &input, Index i, Index j, Index m = 6, Index n = 3,
          Index input_i = 0, Index input_j = 0) {
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        if (input.m_tranposed){
          output(jj + j, ii + i) = input(ii + input_i, jj + input_j);
        }else {
          output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
        }
      }
    }
  }

  template <template <typename, typename> typename ColumnType>
  TINY_INLINE static void assign_block(
      ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &output,
      const ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &input,
      Index i, Index j, Index m, Index n, Index input_i = 0,
      Index input_j = 0) {
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
      }
    }
  }

  template <template <typename, typename> typename ColumnType>
  TINY_INLINE static void assign_block(
      ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &output,
      const ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &input,
      Index i, Index j) {
    for (int ii = 0; ii < input.m_rows; ++ii) {
      for (int jj = 0; jj < input.m_cols; ++jj) {
        output(ii + i, jj + j) = input(ii, jj);
      }
    }
  }

  TINY_INLINE static void assign_column(Matrix3 &m, Index i, const Vector3 &v) {
      for (int j = 0; j < 3; ++j) {
          m(j, i) = v[j];
      }
  }
  TINY_INLINE static void assign_column(Matrix6 &m, Index i,
                                        const SpatialVector &v) {
    for (int j = 0; j < 6; ++j) {
      m(j, i) = v[j];
    }
  }
  template <template <typename, typename> typename ColumnType>
  TINY_INLINE static void assign_column(
      ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &m, Index i,
      const SpatialVector &v) {
    for (int j = 0; j < 6; ++j) {
      m(j, i) = v[j];
    }
  }
  template <template <typename, typename> typename ColumnType>
  TINY_INLINE static void assign_column(
      ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &m, Index i,
      const ColumnType<TinyScalar, TinyConstants> &v) {
    for (int j = 0; j < num_rows(v); ++j) {
      m(j, i) = v[j];
    }
  }

  TINY_INLINE static void assign_row(Matrix3 &m, Index i, const Vector3 &v) {
    for (int j = 0; j < 3; ++j) {
      m(i, j) = v[j];
    }
  }
  TINY_INLINE static void assign_row(Matrix6x3 &m, Index i, const Vector3 &v) {
    for (int j = 0; j < 3; ++j) {
      m(i, j) = v[j];
    }
  }
  TINY_INLINE static void assign_row(Matrix6 &m, Index i,
                                     const SpatialVector &v) {
    for (int j = 0; j < 6; ++j) {
      m(i, j) = v[j];
    }
  }
  template <template <typename, typename> typename ColumnType>
  TINY_INLINE static void assign_row(
      ::TINY::TinyMatrixXxX_<TinyScalar, TinyConstants, ColumnType> &m, Index i,
      const SpatialVector &v) {
    for (int j = 0; j < 6; ++j) {
      m(i, j) = v[j];
    }
  }

  TINY_INLINE static Matrix3 quat_to_matrix(const Quaternion &quat) {
    Matrix3 m;
    m.setRotation(quat);
    return m;
  }
  TINY_INLINE static Matrix3 quat_to_matrix(const Scalar &x, const Scalar &y,
                                            const Scalar &z, const Scalar &w) {
    Matrix3 m;
    m.setRotation(Quaternion(x, y, z, w));
    return m;
  }
  TINY_INLINE static Quaternion matrix_to_quat(const Matrix3 &m) {
    Quaternion quat;
    m.getRotation(quat);
    return quat;
  }
  TINY_INLINE static Quaternion axis_angle_quaternion(const Vector3 &axis,
                                                      const Scalar &angle) {
    Quaternion quat;
    quat.setRotation(axis, angle);
    return quat;
  }
  TINY_INLINE static Vector3 quaternion_axis_angle(const Quaternion quat) {
      Vector3 qv(quat.getX(), quat.getY(), quat.getZ());
      Scalar qv_norm = qv.length();
      Scalar theta = two() * TinyConstants::atan2(qv_norm, quat.getW());

      if (qv_norm < pow(Scalar(std::numeric_limits<double>::epsilon()), fraction(1, 4))){
          return one()/(half() + theta*theta*fraction(1, 48)) * qv;
      }

      return (theta / qv_norm) * qv;
  }
  TINY_INLINE static Scalar quaternion_angle(const Quaternion quat) {
      Vector3 qv(quat.getX(), quat.getY(), quat.getZ());
      Scalar qv_norm = qv.length();
      Scalar theta = two() * TinyConstants::atan2(qv_norm, quat.getW());
      return theta;
  }
  TINY_INLINE static Matrix3 rotation_x_matrix(const Scalar &angle) {
    Matrix3 m;
    m.set_rotation_x(angle);
    return m;
  }

  TINY_INLINE static Matrix3 rotation_y_matrix(const Scalar &angle) {
    Matrix3 m;
    m.set_rotation_y(angle);
    return m;
  }

  TINY_INLINE static Matrix3 rotation_z_matrix(const Scalar &angle) {
    Matrix3 m;
    m.set_rotation_z(angle);
    return m;
  }

  TINY_INLINE static Matrix3 rotation_zyx_matrix(const Scalar &r,
                                                 const Scalar &p,
                                                 const Scalar &y) {
    Matrix3 m;
    m.setEulerZYX(r, p, y);
    return m;
  }

  TINY_INLINE static Vector3 rotate(const Quaternion &q, const Vector3 &v) {
    return q.rotate(v);
  }

  /**
   * Computes the quaternion delta given current rotation q, angular velocity w,
   * time step dt.
   */
  TINY_INLINE static Quaternion quat_velocity(const Quaternion &q,
                                              const Vector3 &w,
                                              const Scalar &dt) {
    return w * q * (dt * half());
    // Quaternion delta(q[3] * w[0] + q[1] * w[2] - q[2] * w[1],
    //                  q[3] * w[1] + q[2] * w[0] - q[0] * w[2],
    //                  q[3] * w[2] + q[0] * w[1] - q[1] * w[0],
    //                  -q[0] * w[0] - q[1] * w[1] - q[2] * w[2]);
    // delta *= half() * dt;
    // print("delta quat", delta);
    // return delta;
  }

  TINY_INLINE static void quat_increment(Quaternion &a, const Quaternion &b) {
    a += b;
  }

  TINY_INLINE static const Scalar &quat_x(const Quaternion &q) { return q.x(); }
  TINY_INLINE static const Scalar &quat_y(const Quaternion &q) { return q.y(); }
  TINY_INLINE static const Scalar &quat_z(const Quaternion &q) { return q.z(); }
  TINY_INLINE static const Scalar &quat_w(const Quaternion &q) { return q.w(); }

  TINY_INLINE static const Quaternion quat_from_xyzw(const Scalar &x,
                                                     const Scalar &y,
                                                     const Scalar &z,
                                                     const Scalar &w) {
    return Quaternion(x, y, z, w);
  }

  TINY_INLINE static void set_zero(Matrix3 &m) { m.set_zero(); }
  TINY_INLINE static void set_zero(Matrix6x3 &m) { m.set_zero(); }
  template <typename S, typename U,
            template <typename, typename> typename ColumnType>
  TINY_INLINE static void set_zero(::TINY::TinyMatrixXxX_<S, U, ColumnType> &m) {
    m.set_zero();
  }

  TINY_INLINE static void set_zero(Vector3 &v) { v.set_zero(); }
  TINY_INLINE static void set_zero(VectorX &v) { v.set_zero(); }
  TINY_INLINE static void set_zero(MotionVector &v) {
    v.top.set_zero();
    v.bottom.set_zero();
  }
  TINY_INLINE static void set_zero(ForceVector &v) {
    v.top.set_zero();
    v.bottom.set_zero();
  }

  /**
   * Non-differentiable comparison operator.
   */
  TINY_INLINE static bool is_zero(const Scalar &a) { return a == zero(); }

  /**
   * Non-differentiable comparison operator.
   */
  TINY_INLINE static bool less_than(const Scalar &a, const Scalar &b) {
    return a < b;
  }

  /**
   * Non-differentiable comparison operator.
   */
  TINY_INLINE static bool less_than_zero(const Scalar &a) { return a < zero(); }

  /**
   * Non-differentiable comparison operator.
   */
  TINY_INLINE static bool greater_than_zero(const Scalar &a) {
    return a > zero();
  }

  /**
   * Non-differentiable comparison operator.
   */
  TINY_INLINE static bool greater_than(const Scalar &a, const Scalar &b) {
    return a > b;
  }

  /**
   * Non-differentiable comparison operator.
   */
  TINY_INLINE static bool equals(const Scalar &a, const Scalar &b) {
    return a == b;
  }

  TINY_INLINE static double to_double(const Scalar &s) {
    return TinyConstants::getDouble(s);
  }
  TINY_INLINE static Scalar from_double(double s) {
    return TinyConstants::scalar_from_double(s);
  }

  static void print(const std::string &title, const Scalar &s) {
    printf("%s %.12f\n", title.c_str(), to_double(s));
  }

  template <typename T>
  static void print(const std::string &title, T object) {
    object.print(title.c_str());
  }

  // /**
  //  * Computes 6x6 matrix by multiplying a and b^T.
  //  */
  // TINY_INLINE static Matrix6 mul_transpose(const SpatialVector &a,
  //                                          const SpatialVector &b) {
  //   return Matrix6::vTimesvTranspose(a, b);
  // }

  TINY_INLINE static Scalar sin(const Scalar &s) {
    return TinyConstants::sin1(s);
  }

  TINY_INLINE static Scalar cos(const Scalar &s) {
    return TinyConstants::cos1(s);
  }

  TINY_INLINE static Scalar tan(const Scalar &s) {
    return TinyConstants::tan1(s);
  }

  TINY_INLINE static Scalar atan2(const Scalar &y, const Scalar &x) {
    return TinyConstants::atan2(y, x);
  }

  TINY_INLINE static Scalar sqrt(const Scalar &s) {
    return TinyConstants::sqrt1(s);
  }

  TINY_INLINE static Scalar abs(const Scalar &s) {
    return TinyConstants::abs(s);
  }

  TINY_INLINE static Scalar pow(const Scalar &s, const Scalar &e) {
    return TinyConstants::pow(s, e);
  }

  TINY_INLINE static Scalar exp(const Scalar &s) {
    return TinyConstants::exp(s);
  }

  TINY_INLINE static Scalar log(const Scalar &s) {
    return TinyConstants::log(s);
  }

  TINY_INLINE static Scalar tanh(const Scalar &s) {
    return TinyConstants::tanh(s);
  }

  TINY_INLINE static Scalar min(const Scalar &a, const Scalar &b) {
    return TinyConstants::min1(a, b);
  }

  TINY_INLINE static Scalar max(const Scalar &a, const Scalar &b) {
    return TinyConstants::max1(a, b);
  }

  TinyAlgebra() = delete;
};
