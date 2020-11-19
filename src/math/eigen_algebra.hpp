#pragma once

#if USE_STAN
#include <stan/math.hpp>
#include <stan/math/fwd.hpp>
#endif

// clang-format off
#include <cppad/cg.hpp>
#include "math/cppad/eigen_mat_inv.hpp"
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "math/conditionals.hpp"
#include "math/tiny/neural_scalar.hpp"

#include "spatial_vector.hpp"

namespace tds {

template <typename ScalarT = double>
struct EigenAlgebraT {
  using Index = Eigen::Index;
  using Scalar = ScalarT;
  using EigenAlgebra = EigenAlgebraT<Scalar>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Matrix6 = Eigen::Matrix<Scalar, 6, 6>;
  using Matrix3X = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
  using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using SpatialVector = tds::SpatialVector<EigenAlgebra>;
  using MotionVector = tds::MotionVector<EigenAlgebra>;
  using ForceVector = tds::ForceVector<EigenAlgebra>;

  template <typename T>
  EIGEN_ALWAYS_INLINE static auto transpose(const T &matrix) {
    return matrix.transpose();
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static auto inverse(const T &matrix) {
    return matrix.inverse();
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static auto inverse_transpose(const T &matrix) {
    return matrix.inverse().transpose();
  }

  template <typename T1, typename T2>
  EIGEN_ALWAYS_INLINE static auto cross(const T1 &vector_a,
                                        const T2 &vector_b) {
    return vector_a.cross(vector_b);
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

  EIGEN_ALWAYS_INLINE static Index size(const VectorX &v) { return v.size(); }

  EIGEN_ALWAYS_INLINE static Matrix3X create_matrix_3x(int num_cols) {
    return Matrix3X(3, num_cols);
  }
  EIGEN_ALWAYS_INLINE static MatrixX create_matrix_x(int num_rows,
                                                     int num_cols) {
    return MatrixX(num_rows, num_cols);
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static int num_rows(const T &matrix) {
    return matrix.rows();
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static int num_cols(const T &matrix) {
    return matrix.cols();
  }

  EIGEN_ALWAYS_INLINE static Scalar determinant(const Matrix3 &m) {
    return m.determinant();
  }

  EIGEN_ALWAYS_INLINE static Scalar determinant(const MatrixX &m) {
    return m.determinant();
  }

  /**
   * CppAD-friendly matrix inverse operation that assumes the input matrix is
   * positive-definite.
   */
  static void plain_symmetric_inverse(const MatrixX &mat, MatrixX &mat_inv) {
    assert(mat.rows() == mat.cols());
    VectorX diagonal = mat.diagonal();
    mat_inv = mat;
    const int n = mat.rows();
    int i, j, k;
    Scalar sum;
    for (i = 0; i < n; i++) {
      mat_inv(i, i) = one() / diagonal[i];
      for (j = i + 1; j < n; j++) {
        sum = zero();
        for (k = i; k < j; k++) {
          sum -= mat_inv(j, k) * mat_inv(k, i);
        }
        mat_inv(j, i) = sum / diagonal[j];
      }
    }
    for (i = 0; i < n; i++) {
      for (j = i + 1; j < n; j++) {
        mat_inv(i, j) = zero();
      }
    }
    for (i = 0; i < n; i++) {
      mat_inv(i, i) = mat_inv(i, i) * mat_inv(i, i);
      for (k = i + 1; k < n; k++) {
        mat_inv(i, i) += mat_inv(k, i) * mat_inv(k, i);
      }
      for (j = i + 1; j < n; j++) {
        for (k = j; k < n; k++) {
          mat_inv(i, j) += mat_inv(k, i) * mat_inv(k, j);
        }
      }
    }
    for (i = 0; i < n; i++) {
      for (j = 0; j < i; j++) {
        mat_inv(i, j) = mat_inv(j, i);
      }
    }
  }

  /**
   * Returns true if the matrix `mat` is positive-definite, and assigns
   * `mat_inv` to the inverse of mat.
   * `mat` must be a symmetric matrix.
   */
  static bool symmetric_inverse(const MatrixX &mat, MatrixX &mat_inv) {
    if constexpr (!is_cppad_scalar<Scalar>::value) {
      Eigen::LLT<MatrixX> llt(mat);
      if (llt.info() == Eigen::NumericalIssue) {
        return false;
      }
      mat_inv = mat.inverse();
    } else {
      plain_symmetric_inverse(mat, mat_inv);
      // // FIXME the atomic op needs to remain in memory but it will fail when
      // the
      // // dimensions of the input matrix are not always the same
      // using InnerScalar = typename Scalar::value_type;
      // static atomic_eigen_mat_inv<InnerScalar> mat_inv_op;
      // mat_inv = mat_inv_op.op(mat);
    }
    return true;
  }

  /**
   * V = mv(w, v)
   * F = mv(n, f)
   * V.F = w.n + v.f
   */
  EIGEN_ALWAYS_INLINE static Scalar dot(const MotionVector &a,
                                        const ForceVector &b) {
    return a.top.dot(b.top) + a.bottom.dot(b.bottom);
  }
  EIGEN_ALWAYS_INLINE static Scalar dot(const ForceVector &a,
                                        const MotionVector &b) {
    return dot(b, a);
  }

  template <typename T1, typename T2>
  EIGEN_ALWAYS_INLINE static auto dot(const T1 &vector_a, const T2 &vector_b) {
    return vector_a.dot(vector_b);
  }

  TINY_INLINE static Scalar norm(const MotionVector &v) {
    using std::sqrt;
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] +
                v[4] * v[4] + v[5] * v[5]);
  }
  TINY_INLINE static Scalar norm(const ForceVector &v) {
    using std::sqrt;
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] +
                v[4] * v[4] + v[5] * v[5]);
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static Scalar norm(const T &v) {
    return v.norm();
  }
  template <typename T>
  EIGEN_ALWAYS_INLINE static Scalar sqnorm(const T &v) {
    return v.squaredNorm();
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static auto normalize(T &v) {
    v.normalize();
    return v;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 cross_matrix(const Vector3 &v) {
    Matrix3 tmp;
    tmp << zero(), -v[2], v[1], v[2], zero(), -v[0], -v[1], v[0], zero();
    return tmp;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 zero33() { return Matrix3::Zero(); }

  EIGEN_ALWAYS_INLINE static VectorX zerox(Index size) {
    return VectorX::Zero(size);
  }

  EIGEN_ALWAYS_INLINE static Matrix3 diagonal3(const Vector3 &v) {
    Matrix3 tmp;
    tmp.setZero();
    tmp(0, 0) = v[0];
    tmp(1, 1) = v[1];
    tmp(2, 2) = v[2];
    return tmp;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 diagonal3(const Scalar &v) {
    Matrix3 tmp;
    tmp.setZero();
    tmp(0, 0) = v;
    tmp(1, 1) = v;
    tmp(2, 2) = v;
    return tmp;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 eye3() { return Matrix3::Identity(); }
  EIGEN_ALWAYS_INLINE static void set_identity(Quaternion &quat) {
    quat = Quaternion(Scalar(1.), Scalar(0.), Scalar(0.), Scalar(0.));
  }

  EIGEN_ALWAYS_INLINE static Scalar zero() { return Scalar(0); }
  EIGEN_ALWAYS_INLINE static Scalar one() { return Scalar(1); }
  EIGEN_ALWAYS_INLINE static Scalar two() { return Scalar(2); }
  EIGEN_ALWAYS_INLINE static Scalar half() { return Scalar(0.5); }
  EIGEN_ALWAYS_INLINE static Scalar pi() { return Scalar(M_PI); }
  EIGEN_ALWAYS_INLINE static Scalar fraction(int a, int b) {
    return (Scalar(a)) / b;
  }

  static Scalar scalar_from_string(const std::string &s) {
    return from_double(std::stod(s));
  }

  EIGEN_ALWAYS_INLINE static Vector3 zero3() { return Vector3::Zero(); }
  EIGEN_ALWAYS_INLINE static Vector3 unit3_x() {
    return Vector3(one(), zero(), zero());
  }
  EIGEN_ALWAYS_INLINE static Vector3 unit3_y() {
    return Vector3(zero(), one(), zero());
  }
  EIGEN_ALWAYS_INLINE static Vector3 unit3_z() {
    return Vector3(zero(), zero(), one());
  }

  EIGEN_ALWAYS_INLINE static VectorX segment(const VectorX &vec,
                                             int start_index, int length) {
    return vec.segment(start_index, length);
  }

  EIGEN_ALWAYS_INLINE static MatrixX block(const MatrixX &mat,
                                           int start_row_index,
                                           int start_col_index, int rows,
                                           int cols) {
    return mat.block(start_row_index, start_col_index, rows, cols);
  }

  EIGEN_ALWAYS_INLINE static void assign_block(Matrix3X &output,
                                               const Matrix3 &input, int i,
                                               int j, int m = -1, int n = -1,
                                               int input_i = 0,
                                               int input_j = 0) {
    if (m < 0) m = input.rows();
    if (n < 0) n = input.cols();
    assert(i + m <= output.rows() && j + n <= output.cols());
    assert(input_i + m <= input.rows() && input_j + n <= input.cols());
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
      }
    }
  }

  EIGEN_ALWAYS_INLINE static void assign_block(Matrix6 &output,
                                               const Matrix3 &input, int i,
                                               int j, int m = -1, int n = -1,
                                               int input_i = 0,
                                               int input_j = 0) {
    if (m < 0) m = input.rows();
    if (n < 0) n = input.cols();
    assert(i + m <= output.rows() && j + n <= output.cols());
    assert(input_i + m <= input.rows() && input_j + n <= input.cols());
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
      }
    }
  }

  EIGEN_ALWAYS_INLINE static void assign_block(Matrix3 &output,
                                               const Matrix6 &input, int i,
                                               int j, int m = -1, int n = -1,
                                               int input_i = 0,
                                               int input_j = 0) {
    if (m < 0) m = input.rows();
    if (n < 0) n = input.cols();
    assert(i + m <= output.rows() && j + n <= output.cols());
    assert(input_i + m <= input.rows() && input_j + n <= input.cols());
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
      }
    }
  }

  EIGEN_ALWAYS_INLINE static void assign_block(MatrixX &output,
                                               const MatrixX &input, int i,
                                               int j, int m = -1, int n = -1,
                                               int input_i = 0,
                                               int input_j = 0) {
    if (m < 0) m = input.rows();
    if (n < 0) n = input.cols();
    assert(i + m <= output.rows() && j + n <= output.cols());
    assert(input_i + m <= input.rows() && input_j + n <= input.cols());
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
      }
    }
  }

  template <int Rows1, int Cols1, int Rows2, int Cols2>
  EIGEN_ALWAYS_INLINE static void assign_block(
      Eigen::Matrix<Scalar, Rows1, Cols1> &output,
      const Eigen::Matrix<Scalar, Rows2, Cols2> &input, int i, int j,
      int m = -1, int n = -1, int input_i = 0, int input_j = 0) {
    if (m < 0) m = input.rows();
    if (n < 0) n = input.cols();
    assert(i + m <= output.rows() && j + n <= output.cols());
    assert(input_i + m <= input.rows() && input_j + n <= input.cols());
    for (int ii = 0; ii < m; ++ii) {
      for (int jj = 0; jj < n; ++jj) {
        output(ii + i, jj + j) = input(ii + input_i, jj + input_j);
      }
    }
  }

  EIGEN_ALWAYS_INLINE static void assign_column(Matrix3 &m, Index i,
                                                const Vector3 &v) {
    m.col(i) = v;
  }

  EIGEN_ALWAYS_INLINE static void assign_column(Matrix3 &m, Index i,
                                                const Matrix6 &v) {
    m.col(i) = v;
  }
  EIGEN_ALWAYS_INLINE static void assign_column(Matrix3X &m, Index i,
                                                const Vector3 &v) {
    m.col(i) = v;
  }

  EIGEN_ALWAYS_INLINE static void assign_column(MatrixX &m, Index i,
                                                const MatrixX &v) {
    m.col(i) = v;
  }

  EIGEN_ALWAYS_INLINE static void assign_column(MatrixX &m, Index i,
                                                const SpatialVector &v) {
    m.block(0, i, 3, 1) = v.top;
    m.block(3, i, 3, 1) = v.bottom;
  }
  template <int Rows, int Cols, typename Derived>
  EIGEN_ALWAYS_INLINE static void assign_column(
      Eigen::Matrix<Scalar, Rows, Cols> &m, Index i,
      const Eigen::DenseBase<Derived> &v) {
    assign_column(m, i, v.eval());
  }

  EIGEN_ALWAYS_INLINE static void assign_row(MatrixX &m, Index i,
                                             const MatrixX &v) {
    m.row(i) = v;
  }

  EIGEN_ALWAYS_INLINE static void assign_row(MatrixX &m, Index i,
                                             const SpatialVector &v) {
    m.block(i, 0, 1, 3) = v.top;
    m.block(i, 3, 1, 3) = v.bottom;
  }

  EIGEN_ALWAYS_INLINE static void assign_horizontal(MatrixX &mat,
                                                    const VectorX &vec,
                                                    int start_row_index,
                                                    int start_col_index) {
    mat.block(start_row_index, start_col_index, 1, vec.rows()) =
        vec.transpose();
  }

  template <int Rows>
  EIGEN_ALWAYS_INLINE static void assign_vertical(
      MatrixX &mat, const Eigen::Matrix<Scalar, Rows, 1> &vec,
      int start_row_index, int start_col_index) {
    mat.block(start_row_index, start_col_index, vec.rows(), 1) = vec;
  }

  template <int Rows, int Cols>
  TINY_INLINE static VectorX mul_transpose(
      const Eigen::Matrix<Scalar, Rows, Cols> &mat,
      const Eigen::Matrix<Scalar, Cols, 1> &vec) {
    return mat.transpose() * vec;
  }
  TINY_INLINE static VectorX mul_transpose(const MatrixX &mat,
                                           const VectorX &vec) {
    return mat.transpose() * vec;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 quat_to_matrix(const Quaternion &quat) {
    // NOTE: Eigen requires quat to be normalized
    return quat.toRotationMatrix();
  }
  EIGEN_ALWAYS_INLINE static Matrix3 quat_to_matrix(const Scalar &x,
                                                    const Scalar &y,
                                                    const Scalar &z,
                                                    const Scalar &w) {
    return Quaternion(w, x, y, z).toRotationMatrix();
  }
  EIGEN_ALWAYS_INLINE static Quaternion matrix_to_quat(const Matrix3 &m) {
    if constexpr (is_cppad_scalar<Scalar>::value) {
      // add epsilon to denominator to prevent division by zero
      const Scalar eps = from_double(1e-6);
      Scalar tr = m(0, 0) + m(1, 1) + m(2, 2);
      Scalar q1[4], q2[4], q3[4], q4[4];
      // if (tr > 0)
      {
        Scalar S = sqrt(abs(tr + 1.0)) * two() + eps;
        q1[0] = fraction(1, 4) * S;
        q1[1] = (m(2, 1) - m(1, 2)) / S;
        q1[2] = (m(0, 2) - m(2, 0)) / S;
        q1[3] = (m(1, 0) - m(0, 1)) / S;
      }
      // else if ((m(0,0) > m(1,1))&(m(0,0) > m(2,2)))
      {
        Scalar S = sqrt(abs(1.0 + m(0, 0) - m(1, 1) - m(2, 2))) * two() + eps;
        q2[0] = (m(2, 1) - m(1, 2)) / S;
        q2[1] = fraction(1, 4) * S;
        q2[2] = (m(0, 1) + m(1, 0)) / S;
        q2[3] = (m(0, 2) + m(2, 0)) / S;
      }
      // else if (m(1,1) > m(2,2))
      {
        Scalar S = sqrt(abs(1.0 + m(1, 1) - m(0, 0) - m(2, 2))) * two() + eps;
        q3[0] = (m(0, 2) - m(2, 0)) / S;
        q3[1] = (m(0, 1) + m(1, 0)) / S;
        q3[2] = fraction(1, 4) * S;
        q3[3] = (m(1, 2) + m(2, 1)) / S;
      }
      // else
      {
        Scalar S = sqrt(abs(1.0 + m(2, 2) - m(0, 0) - m(1, 1))) * two() + eps;
        q4[0] = (m(1, 0) - m(0, 1)) / S;
        q4[1] = (m(0, 2) + m(2, 0)) / S;
        q4[2] = (m(1, 2) + m(2, 1)) / S;
        q4[3] = fraction(1, 4) * S;
      }
      Quaternion q;
      // (m(0,0) > m(1,1))&(m(0,0) > m(2,2))
      Scalar m00_is_max = where_gt(
          m(0, 0), m(1, 1), where_gt(m(0, 0), m(2, 2), one(), zero()), zero());
      Scalar m11_is_max =
          (one() - m00_is_max) * where_gt(m(1, 1), m(2, 2), one(), zero());
      Scalar m22_is_max = (one() - m00_is_max) * (one() - m11_is_max);
      q.w() = where_gt(
          tr, zero(), q1[0],
          m00_is_max * q2[0] + m11_is_max * q3[0] + m22_is_max * q4[0]);
      q.x() = where_gt(
          tr, zero(), q1[1],
          m00_is_max * q2[1] + m11_is_max * q3[1] + m22_is_max * q4[1]);
      q.y() = where_gt(
          tr, zero(), q1[2],
          m00_is_max * q2[2] + m11_is_max * q3[2] + m22_is_max * q4[2]);
      q.z() = where_gt(
          tr, zero(), q1[3],
          m00_is_max * q2[3] + m11_is_max * q3[3] + m22_is_max * q4[3]);
      return q;
    } else {
      return Quaternion(m);
    }
  }
  EIGEN_ALWAYS_INLINE static Quaternion axis_angle_quaternion(
      const Vector3 &axis, const Scalar &angle) {
    return Quaternion(Eigen::AngleAxis(angle, axis));
  }

  EIGEN_ALWAYS_INLINE static Matrix3 rotation_x_matrix(const Scalar &angle) {
    using std::cos, std::sin;
    Scalar c = cos(angle);
    Scalar s = sin(angle);
    Matrix3 temp;
    temp << one(), zero(), zero(), zero(), c, s, zero(), -s, c;
    return temp;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 rotation_y_matrix(const Scalar &angle) {
    using std::cos, std::sin;
    Scalar c = cos(angle);
    Scalar s = sin(angle);
    Matrix3 temp;
    temp << c, zero(), -s, zero(), one(), zero(), s, zero(), c;
    return temp;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 rotation_z_matrix(const Scalar &angle) {
    using std::cos, std::sin;
    Scalar c = cos(angle);
    Scalar s = sin(angle);
    Matrix3 temp;
    temp << c, s, zero(), -s, c, zero(), zero(), zero(), one();
    return temp;
  }

  static Matrix3 rotation_zyx_matrix(const Scalar &r, const Scalar &p,
                                     const Scalar &y) {
    using std::cos, std::sin;
    Scalar ci(cos(r));
    Scalar cj(cos(p));
    Scalar ch(cos(y));
    Scalar si(sin(r));
    Scalar sj(sin(p));
    Scalar sh(sin(y));
    Scalar cc = ci * ch;
    Scalar cs = ci * sh;
    Scalar sc = si * ch;
    Scalar ss = si * sh;
    Matrix3 temp;
    temp << cj * ch, sj * sc - cs, sj * cc + ss, cj * sh, sj * ss + cc,
        sj * cs - sc, -sj, cj * si, cj * ci;
    return temp;
  }

  EIGEN_ALWAYS_INLINE static Vector3 rotate(const Quaternion &q,
                                            const Vector3 &v) {
    return q * v;
  }

  /**
   * Computes the quaternion delta given current rotation q, angular velocity w,
   * time step dt.
   */
  EIGEN_ALWAYS_INLINE static Quaternion quat_velocity(const Quaternion &q,
                                                      const Vector3 &w,
                                                      const Scalar &dt) {
    Quaternion delta((-q.x() * w[0] - q.y() * w[1] - q.z() * w[2]) * (0.5 * dt),
                     (q.w() * w[0] + q.y() * w[2] - q.z() * w[1]) * (0.5 * dt),
                     (q.w() * w[1] + q.z() * w[0] - q.x() * w[2]) * (0.5 * dt),
                     (q.w() * w[2] + q.x() * w[1] - q.y() * w[0]) * (0.5 * dt));
    return delta;
  }

  EIGEN_ALWAYS_INLINE static void quat_increment(Quaternion &a,
                                                 const Quaternion &b) {
    a.x() += b.x();
    a.y() += b.y();
    a.z() += b.z();
    a.w() += b.w();
  }

  EIGEN_ALWAYS_INLINE static const Scalar &quat_x(const Quaternion &q) {
    return q.x();
  }
  EIGEN_ALWAYS_INLINE static const Scalar &quat_y(const Quaternion &q) {
    return q.y();
  }
  EIGEN_ALWAYS_INLINE static const Scalar &quat_z(const Quaternion &q) {
    return q.z();
  }
  EIGEN_ALWAYS_INLINE static const Scalar &quat_w(const Quaternion &q) {
    return q.w();
  }
  EIGEN_ALWAYS_INLINE static const Quaternion quat_from_xyzw(const Scalar &x,
                                                             const Scalar &y,
                                                             const Scalar &z,
                                                             const Scalar &w) {
    // Eigen specific constructor coefficient order
    return Quaternion(w, x, y, z);
  }

  EIGEN_ALWAYS_INLINE static void set_zero(Matrix3X &m) { m.setZero(); }
  EIGEN_ALWAYS_INLINE static void set_zero(Vector3 &m) { m.setZero(); }
  EIGEN_ALWAYS_INLINE static void set_zero(VectorX &m) { m.setZero(); }

  EIGEN_ALWAYS_INLINE static void set_zero(MatrixX &m) { m.setZero(); }
  template <int Size1, int Size2 = 1>
  EIGEN_ALWAYS_INLINE static void set_zero(
      Eigen::Array<Scalar, Size1, Size2> &v) {
    v.setZero();
  }
  EIGEN_ALWAYS_INLINE static void set_zero(MotionVector &v) {
    v.top.setZero();
    v.bottom.setZero();
  }
  EIGEN_ALWAYS_INLINE static void set_zero(ForceVector &v) {
    v.top.setZero();
    v.bottom.setZero();
  }

  /**
   * Non-differentiable comparison operator.
   */
  TINY_INLINE static bool is_zero(const Scalar &a) { return a == zero(); }

  /**
   * Non-differentiable comparison operator.
   */
  EIGEN_ALWAYS_INLINE static bool less_than(const Scalar &a, const Scalar &b) {
    return a < b;
  }

  /**
   * Non-differentiable comparison operator.
   */
  EIGEN_ALWAYS_INLINE static bool less_than_zero(const Scalar &a) {
    return a < 0.;
  }

  /**
   * Non-differentiable comparison operator.
   */
  EIGEN_ALWAYS_INLINE static bool greater_than_zero(const Scalar &a) {
    return a > 0.;
  }

  /**
   * Non-differentiable comparison operator.
   */
  EIGEN_ALWAYS_INLINE static bool greater_than(const Scalar &a,
                                               const Scalar &b) {
    return a > b;
  }

  /**
   * Non-differentiable comparison operator.
   */
  EIGEN_ALWAYS_INLINE static bool equals(const Scalar &a, const Scalar &b) {
    return a == b;
  }

#ifdef USE_STAN
  template <typename InnerScalar>
  TINY_INLINE static std::enable_if_t<
      !std::is_same_v<Scalar, stan::math::fvar<InnerScalar>>, double>
  to_double(const stan::math::fvar<InnerScalar> &s) {
    return stan::math::value_of(s);
  }
#endif

  TINY_INLINE static double to_double(const Scalar &s) {
#ifdef USE_STAN
    if constexpr (std::is_same_v<Scalar, stan::math::var> ||
                  std::is_same_v<Scalar, stan::math::fvar<double>>) {
      return stan::math::value_of(s);
    } else
#endif
        if constexpr (std::is_same_v<std::remove_cv_t<Scalar>,
                                     CppAD::AD<CppAD::cg::CG<double>>>) {
      return CppAD::Value(CppAD::Var2Par(s)).getValue();
    } else if constexpr (std::is_same_v<std::remove_cv_t<Scalar>,
                                        CppAD::AD<double>>) {
      return CppAD::Value(CppAD::Var2Par(s));
    } else {
      return static_cast<double>(s);
    }
  }

  TINY_INLINE static Scalar from_double(double s) {
    return static_cast<Scalar>(s);
  }

  template <int Size1, int Size2>
  static void print(const std::string &title,
                    Eigen::Matrix<Scalar, Size1, Size2> &m) {
    std::cout << title << "\n" << m << std::endl;
  }
  template <int Size1, int Size2 = 1>
  static void print(const std::string &title,
                    Eigen::Array<Scalar, Size1, Size2> &v) {
    std::cout << title << "\n" << v << std::endl;
  }
  static void print(const std::string &title, const Scalar &v) {
    std::cout << title << "\n" << to_double(v) << std::endl;
  }
  template <typename T>
  static void print(const std::string &title, const T &abi) {
    abi.print(title.c_str());
  }

  template <typename T>
  TINY_INLINE static auto sin(const T &s) {
    using std::sin;
    return sin(s);
  }

  template <typename T>
  TINY_INLINE static auto cos(const T &s) {
    using std::cos;
    return cos(s);
  }

  template <typename T>
  TINY_INLINE static auto tan(const T &s) {
    using std::tan;
    return tan(s);
  }

  template <typename T>
  TINY_INLINE static auto atan2(const T &dy, const T &dx) {
    using std::atan2;
    return atan2(dy, dx);
  }

  template <typename T>
  TINY_INLINE static auto abs(const T &s) {
    using std::abs;
    return abs(s);
  }

  template <typename T>
  TINY_INLINE static auto sqrt(const T &s) {
    using std::sqrt;
    return sqrt(s);
  }

  template <typename T>
  TINY_INLINE static auto tanh(const T &s) {
    using std::tanh;
    return tanh(s);
  }

  template <typename T>
  TINY_INLINE static auto pow(const T &s, const T &e) {
    using std::pow;
    return pow(s, e);
  }

  template <typename T>
  TINY_INLINE static auto exp(const T &s) {
    using std::exp;
    return exp(s);
  }

  template <typename T>
  TINY_INLINE static auto log(const T &s) {
    using std::log;
    return log(s);
  }

  template <typename T>
  TINY_INLINE static auto max(const T &x, const T &y) {
    return tds::where_gt(x, y, x, y);
  }

  template <typename T>
  TINY_INLINE static auto min(const T &x, const T &y) {
    return tds::where_lt(x, y, x, y);
  }

  EigenAlgebraT<Scalar>() = delete;
};

typedef EigenAlgebraT<double> EigenAlgebra;

// Helpers for NeuralAlgebra

template <typename Scalar>
struct is_cppad_scalar<NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>>> {
  static constexpr bool value = true;
};

template <typename Algebra>
struct is_eigen_algebra<EigenAlgebraT<Algebra>> {
  static constexpr bool value = true;
};

template <typename Scalar>
static TINY_INLINE NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> where_gt(
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &x,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &y,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_true,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_false) {
  return CppAD::CondExpGt(x.evaluate(), y.evaluate(), if_true.evaluate(),
                          if_false.evaluate());
}

template <typename Scalar>
static TINY_INLINE NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> where_ge(
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &x,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &y,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_true,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_false) {
  return CppAD::CondExpGe(x.evaluate(), y.evaluate(), if_true.evaluate(),
                          if_false.evaluate());
}

template <typename Scalar>
static TINY_INLINE NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> where_lt(
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &x,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &y,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_true,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_false) {
  return CppAD::CondExpLt(x.evaluate(), y.evaluate(), if_true.evaluate(),
                          if_false.evaluate());
}

template <typename Scalar>
static TINY_INLINE NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> where_le(
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &x,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &y,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_true,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_false) {
  return CppAD::CondExpLe(x.evaluate(), y.evaluate(), if_true.evaluate(),
                          if_false.evaluate());
}

template <typename Scalar>
static TINY_INLINE NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> where_eq(
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &x,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &y,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_true,
    const NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>> &if_false) {
  return CppAD::CondExpEq(x.evaluate(), y.evaluate(), if_true.evaluate(),
                          if_false.evaluate());
}

}  // end namespace tds
