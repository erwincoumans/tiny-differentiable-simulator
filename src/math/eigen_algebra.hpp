#pragma once

#include "base.hpp"

#if USE_STAN
#include <stan/math.hpp>
#include <stan/math/fwd.hpp>
#endif

// clang-format off
#ifdef USE_CPPAD
  #ifdef USE_CPPAD_CODEGEN
    #include <cppad/cg.hpp>
  #else
    #include <cppad/cppad.hpp>
  #endif
#include "math/cppad/eigen_mat_inv.hpp"
#endif //USE_CPPAD
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "math/conditionals.hpp"
#include "math/tiny/neural_scalar.hpp"

#include "spatial_vector.hpp"
#undef max
#undef min

#include <cmath>

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
  using Matrix6x3 = Eigen::Matrix<Scalar, 6, 3>;
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

  EIGEN_ALWAYS_INLINE static MatrixX transpose(const MatrixX &matrix) {
    return matrix.transpose();
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static auto inverse(const T &matrix) {
    return matrix.inverse();
  }

#if 0
  template <typename T>
  EIGEN_ALWAYS_INLINE static auto pseudo_inverse(const T &matrix) {

    Eigen::CompleteOrthogonalDecomposition<
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>
    cod(matrix);
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J_pinv =
    cod.pseudoInverse();
    return J_pinv;
  }
#endif

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
   *    main method for Cholesky decomposition.
   *    input/output  a  Symmetric positive def. matrix
   *    output        diagonal  vector of resulting diag of a
   *    inspired by public domain https://math.nist.gov/javanumerics/jama
   */

  static bool cholesky_decomposition(MatrixX &a, VectorX &diagonal) {
    int i, j, k;
    Scalar sum;
    int n = a.cols();
    bool is_positive_definite = true;
    for (i = 0; i < n; i++) {
      for (j = i; j < n; j++) {
        sum = a(i, j);
        for (k = i - 1; k >= 0; k--) {
          sum -= a(i, k) * a(j, k);
        }
        if (i == j) {
          // if (sum <= zero()) {
          //    is_positive_definite = false;
          //    break;
          //}
          diagonal(i) = sqrt(sum);
        } else {
          a(j, i) = sum / diagonal[i];
        }
      }
    }
    return is_positive_definite;
  }

  static bool inverse_cholesky_decomposition(const MatrixX &A, MatrixX &a) {
    int i, j, k;
    int n = A.rows();
    Scalar sum;
    VectorX diagonal(A.rows());
    for (i = 0; i < n; i++)
      for (j = 0; j < n; j++) a(i, j) = A(i, j);
    bool is_positive_definite = cholesky_decomposition(a, diagonal);
    if (is_positive_definite) {
      for (i = 0; i < n; i++) {
        a(i, i) = one() / diagonal[i];
        for (j = i + 1; j < n; j++) {
          sum = zero();
          for (k = i; k < j; k++) {
            sum -= a(j, k) * a(k, i);
          }
          a(j, i) = sum / diagonal[j];
        }
      }
    } else {
      printf("no!\n");
    }
    return is_positive_definite;
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

#if 0
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
#endif

  /**
   *     Inverse of a matrix, using Cholesky decomposition.
   *
   *     input    A  Symmetric positive def. matrix
   *     input    a  storage for the result
   *     output   boolean is_positive_definite if operation succeeded
   */
  static bool symmetric_inverse(const MatrixX &A, MatrixX &a) {
    assert(a.cols() == A.cols());
    assert(a.rows() == A.rows());

    bool is_positive_definite = inverse_cholesky_decomposition(A, a);
    if (is_positive_definite) {
      int n = A.cols();
      int i, j, k;

      for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
          a(i, j) = zero();
        }
      }

      for (i = 0; i < n; i++) {
        a(i, i) = a(i, i) * a(i, i);
        for (k = i + 1; k < n; k++) {
          a(i, i) += a(k, i) * a(k, i);
        }
        for (j = i + 1; j < n; j++) {
          for (k = j; k < n; k++) {
            a(i, j) += a(k, i) * a(k, j);
          }
        }
      }
      for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
          a(i, j) = a(j, i);
        }
      }
    }
    return is_positive_definite;
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
  /**
   * Multiplication of a matrix6x3 with a force/motion vector handles the
   * operation as a multiplication with the transpose of the matrix.
   * @param a [6x3] matrix
   * @param b MotionVector or ForceVector
   * @return Vector3
   */
  EIGEN_ALWAYS_INLINE static Vector3 dot(const Matrix6x3 &a,
                                         const ForceVector &b) {
    Vector3 res;
    for (int i = 0; i < 3; i++) {
      res[i] = a(0, i) * b[0] + a(1, i) * b[1] + a(2, i) * b[2] +
               a(3, i) * b[3] + a(4, i) * b[4] + a(5, i) * b[5];
    }
    return res;
  }
  EIGEN_ALWAYS_INLINE static Vector3 dot(const Matrix6x3 &a,
                                         const MotionVector &b) {
    Vector3 res;
    for (int i = 0; i < 3; i++) {
      res[i] = a(0, i) * b[0] + a(1, i) * b[1] + a(2, i) * b[2] +
               a(3, i) * b[3] + a(4, i) * b[4] + a(5, i) * b[5];
    }
    return res;
  }

  EIGEN_ALWAYS_INLINE static MatrixX mult(const MatrixX &a, const MatrixX &b) {
    return a * b;
  }

  EIGEN_ALWAYS_INLINE static VectorX mult(const MatrixX &a, const VectorX &b) {
    return a * b;
  }
  EIGEN_ALWAYS_INLINE static Vector3 mult(const MatrixX &a, const Vector3 &b) {
    return a * b;
  }

  template <typename T1, typename T2>
  EIGEN_ALWAYS_INLINE static auto dot(const T1 &vector_a, const T2 &vector_b) {
    return vector_a.dot(vector_b);
  }

  TINY_INLINE static Scalar norm(const MotionVector &v) {
    using std::sqrt;
    // prevent sqrt of zero which causes division by zero in gradient pass
    Scalar z = v.top.squaredNorm() + v.bottom.squaredNorm();
    return tds::where_eq(z, zero(), zero(), sqrt(z));
  }
  TINY_INLINE static Scalar norm(const ForceVector &v) {
    using std::sqrt;
    // prevent sqrt of zero which causes division by zero in gradient pass
    Scalar z = v.top.squaredNorm() + v.bottom.squaredNorm();
    return tds::where_eq(z, zero(), zero(), sqrt(z));
  }

  template <typename T>
  EIGEN_ALWAYS_INLINE static Scalar norm(const T &v) {
    using std::sqrt;
    // prevent sqrt of zero which causes division by zero in gradient pass
    Scalar z = v.squaredNorm();
    return tds::where_eq(z, zero(), zero(), sqrt(z));
  }
  template <typename T>
  EIGEN_ALWAYS_INLINE static Scalar sqnorm(const T &v) {
    return v.squaredNorm();
  }

  EIGEN_ALWAYS_INLINE static auto normalize(const Vector3 &v) {
    Scalar z = v.squaredNorm();
    Scalar inv_z = tds::where_eq(z, zero(), one(), one() / sqrt(z));
    return v * inv_z;
  }

  EIGEN_ALWAYS_INLINE static auto normalize(const Quaternion &q) {
    Quaternion v(q);
    Scalar z = q.squaredNorm();
    // don't call Eigen .normalize, since it has a comparison > 0, which fails
    // CppADCodegen assert(z > Scalar(0));
    Scalar inv_z = tds::where_eq(z, zero(), one(), one() / sqrt(z));
    v.x() *= inv_z;
    v.y() *= inv_z;
    v.z() *= inv_z;
    v.w() *= inv_z;
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

  EIGEN_ALWAYS_INLINE static MatrixX eye(int n) {
    MatrixX mat(n, n);
    mat.setIdentity();
    return mat;
  }

  EIGEN_ALWAYS_INLINE static void set_identity(Quaternion &quat) {
    quat.setIdentity();
  }

  EIGEN_ALWAYS_INLINE static Scalar zero() { return Scalar(0); }
  EIGEN_ALWAYS_INLINE static Scalar one() { return Scalar(1); }
  EIGEN_ALWAYS_INLINE static Scalar two() { return Scalar(2); }
  EIGEN_ALWAYS_INLINE static Scalar half() { return Scalar(0.5); }
  EIGEN_ALWAYS_INLINE static Scalar pi() {
    return Scalar(
        3.14159265358979323846264338327950288419716939937510582097494459);
  }
  EIGEN_ALWAYS_INLINE static Scalar half_pi() {
    return Scalar(
        1.57079632679489661923132169163975144209858469968755291048747230);
  }
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

  EIGEN_ALWAYS_INLINE static Matrix3 top(const Matrix6x3 &mat) {
    return mat.block(0, 0, 3, 3);
  }

  EIGEN_ALWAYS_INLINE static Matrix3 bottom(const Matrix6x3 &mat) {
    return mat.block(3, 0, 3, 3);
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

  EIGEN_ALWAYS_INLINE static void assign_block(Matrix6x3 &output,
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
    m.row(i) = v.transpose();
  }

  EIGEN_ALWAYS_INLINE static void assign_row(MatrixX &m, Index i,
                                             const SpatialVector &v) {
    m.block(i, 0, 1, 3) = v.top.transpose();
    m.block(i, 3, 1, 3) = v.bottom.transpose();
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

  TINY_INLINE static Scalar epsilon() {
      return std::numeric_limits<double>::epsilon();
  }

  /**
   * Multiplication of a 6x3 matrix with a Vector3 returns a force vector
   * @param a
   * @param b
   * @return
   */
  TINY_INLINE static ForceVector mul_2_force_vector(const Matrix6x3 &mat,
                                                    const Vector3 &vec) {
    VectorX res = mat * vec;

    return ForceVector(Vector3(res[0], res[1], res[2]),
                       Vector3(res[3], res[4], res[5]));
  }
  TINY_INLINE static MotionVector mul_2_motion_vector(const Matrix6x3 &mat,
                                                      const Vector3 &vec) {
    VectorX res = mat * vec;

    return MotionVector(Vector3(res[0], res[1], res[2]),
                        Vector3(res[3], res[4], res[5]));
  }

  EIGEN_ALWAYS_INLINE static Matrix3 quat_to_matrix(const Quaternion &quat) {
    // NOTE: Eigen requires quat to be normalized
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    return quat.toRotationMatrix().transpose();
#else
    return quat.toRotationMatrix();
#endif
  }
  EIGEN_ALWAYS_INLINE static Matrix3 quat_to_matrix(const Scalar &x,
                                                    const Scalar &y,
                                                    const Scalar &z,
                                                    const Scalar &w) {
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    return Quaternion(w, x, y, z).toRotationMatrix().transpose();
#else
    return Quaternion(w, x, y, z).toRotationMatrix();
#endif
  }
  EIGEN_ALWAYS_INLINE static Quaternion matrix_to_quat(const Matrix3 &matrix) {
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    Matrix3 m = matrix.transpose();
#else
    const Matrix3 &m = matrix;
#endif
    if constexpr (is_cppad_scalar<Scalar>::value) {
      // add epsilon to denominator to prevent division by zero
      const Scalar eps = from_double(1e-6);
      Scalar tr = m(0, 0) + m(1, 1) + m(2, 2);
      Scalar q1[4], q2[4], q3[4], q4[4];
      // if (tr > 0)
      {
        Scalar S = sqrt(abs(tr + 1.0) + eps) * two();
        q1[0] = fraction(1, 4) * S;
        q1[1] = (m(2, 1) - m(1, 2)) / S;
        q1[2] = (m(0, 2) - m(2, 0)) / S;
        q1[3] = (m(1, 0) - m(0, 1)) / S;
      }
      // else if ((m(0,0) > m(1,1))&(m(0,0) > m(2,2)))
      {
        Scalar S = sqrt(abs(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) + eps) * two();
        q2[0] = (m(2, 1) - m(1, 2)) / S;
        q2[1] = fraction(1, 4) * S;
        q2[2] = (m(0, 1) + m(1, 0)) / S;
        q2[3] = (m(0, 2) + m(2, 0)) / S;
      }
      // else if (m(1,1) > m(2,2))
      {
        Scalar S = sqrt(abs(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) + eps) * two();
        q3[0] = (m(0, 2) - m(2, 0)) / S;
        q3[1] = (m(0, 1) + m(1, 0)) / S;
        q3[2] = fraction(1, 4) * S;
        q3[3] = (m(1, 2) + m(2, 1)) / S;
      }
      // else
      {
        Scalar S = sqrt(abs(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) + eps) * two();
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

  EIGEN_ALWAYS_INLINE static Vector3 quaternion_axis_angle(
      const Quaternion quat) {
#if 0
    //eigen angleAxis is non-differentiable, >=0 checks
    auto ang_ax = Eigen::AngleAxis<Scalar>(quat);

    return ang_ax.axis() * ang_ax.angle();


    Vector3 qv(quat.x(), quat.y(), quat.z());
    Scalar qv_norm = norm(qv);//qv.length();
    Scalar theta = two() * atan2(qv_norm, quat.w());
    Scalar scaling = tds::where_lt(qv_norm, 
                                    pow(Scalar(std::numeric_limits<double>::epsilon()), fraction(1, 4)),
                                    one()/(half() + theta*theta*fraction(1, 48)),
                                    (theta / qv_norm));
    return scaling * qv;
#endif
    /* Adapted from Ceres solver library. */
    Scalar eps = Scalar(1e-6);
    Scalar q1 = quat.x();
    Scalar q2 = quat.y();
    Scalar q3 = quat.z();
    Scalar sin_squared_theta = q1 * q1 + q2 * q2 + q3 * q3;

    Scalar sin_theta = sqrt(sin_squared_theta);
    Scalar cos_theta = quat.w();

    Scalar two_theta = two() * where_lt(cos_theta, zero(),
                                        atan2(-sin_theta, -cos_theta),
                                        atan2(sin_theta, cos_theta));
    Scalar k = two_theta / sin_theta;
    k = where_gt(sin_squared_theta, eps, k, two());

    return Vector3(q1 * k, q2 * k, q3 * k);
  }

  EIGEN_ALWAYS_INLINE static const Quaternion quat_difference(
      const Quaternion &start, const Quaternion &end) {
    Quaternion q1 = normalize(start);
    Quaternion q2 = normalize(end);

    /* Ported from PyBullet */
    // The "nearest" operation from PyBullet
    VectorX diff(4);
    diff[0] = q1.x() - q2.x();
    diff[1] = q1.y() - q2.y();
    diff[2] = q1.z() - q2.z();
    diff[3] = q1.w() - q2.w();

    VectorX sum(4);
    sum[0] = q1.x() + q2.x();
    sum[1] = q1.y() + q2.y();
    sum[2] = q1.z() + q2.z();
    sum[3] = q1.w() + q2.w();

    Scalar dd = diff.dot(diff);
    Scalar ss = sum.dot(sum);

    Quaternion closest_end =
        quat_from_xyzw(tds::where_lt(dd, ss, q2.x(), -q2.x()),
                       tds::where_lt(dd, ss, q2.y(), -q2.y()),
                       tds::where_lt(dd, ss, q2.z(), -q2.z()),
                       tds::where_lt(dd, ss, q2.w(), -q2.w()));

    closest_end = normalize(closest_end);
    Quaternion res = closest_end * inverse(q1);
    return normalize(res);
  }

  EIGEN_ALWAYS_INLINE static Matrix3 rotation_x_matrix(const Scalar &angle) {
    using std::cos, std::sin;
    Scalar c = cos(angle);
    Scalar s = sin(angle);
    Matrix3 temp;
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    temp << one(), zero(), zero(), zero(), c, s, zero(), -s, c;
#else
    temp << one(), zero(), zero(), zero(), c, -s, zero(), s, c;
#endif
    // std::cout << "rot_x" << temp << std::endl;
    return temp;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 rotation_y_matrix(const Scalar &angle) {
    using std::cos, std::sin;
    Scalar c = cos(angle);
    Scalar s = sin(angle);
    Matrix3 temp;
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    temp << c, zero(), -s, zero(), one(), zero(), s, zero(), c;
#else
    temp << c, zero(), s, zero(), one(), zero(), -s, zero(), c;
#endif
    return temp;
  }

  EIGEN_ALWAYS_INLINE static Matrix3 rotation_z_matrix(const Scalar &angle) {
    using std::cos, std::sin;
    Scalar c = cos(angle);
    Scalar s = sin(angle);
    Matrix3 temp;
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    temp << c, s, zero(), -s, c, zero(), zero(), zero(), one();
#else
    temp << c, -s, zero(), s, c, zero(), zero(), zero(), one();
#endif
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

#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    temp << cj * ch, cj * sh, -sj, sj * sc - cs, sj * ss + cc, cj * si,
        sj * cc + ss, sj * cs - sc, cj * ci;
#else
    temp << cj * ch, sj * sc - cs, sj * cc + ss, cj * sh, sj * ss + cc,
        sj * cs - sc, -sj, cj * si, cj * ci;
#endif
    return temp;
  }

  EIGEN_ALWAYS_INLINE static Vector3 rotate(const Quaternion &q,
                                            const Vector3 &w) {
    return q * w;

    /* Rotating with an all zero quaternion results in
          a rotation with the identity quaternion in Eigen.
          However in TinyAlgebra this returns a zero vector.
          This function mimics the TinyAlgebra implementation. */
    // Quaternion q2(q.w() * w[0] + q.y() * w[2] - q.z() * w[1],
    //              q.w() * w[1] + q.z() * w[0] - q.x() * w[2],
    //              q.w() * w[2] + q.x() * w[1] - q.y() * w[0],
    //             -q.x() * w[0] - q.y() * w[1] - q.z() * w[2]);
    // q2 *= q.inverse();
    // return Vector3(q2.x(), q2.y(), q2.z());
  }

  /**
   * Computes the quaternion delta given current rotation q, angular velocity w,
   * time step dt.
   */

  EIGEN_ALWAYS_INLINE static Quaternion quat_velocity(const Quaternion &q,
                                                      const Vector3 &w,
                                                      const Scalar &dt) {
    auto ww = (-q.x() * w[0] - q.y() * w[1] - q.z() * w[2]) * (0.5 * dt);
    auto xx = (q.w() * w[0] + q.z() * w[1] - q.y() * w[2]) * (0.5 * dt);
    auto yy = (q.w() * w[1] + q.x() * w[2] - q.z() * w[0]) * (0.5 * dt);
    auto zz = (q.w() * w[2] + q.y() * w[0] - q.x() * w[1]) * (0.5 * dt);

    Quaternion delta = quat_from_xyzw(xx, yy, zz, ww);
    return delta;
  }

  EIGEN_ALWAYS_INLINE static Quaternion quat_velocity_spherical(
      const Quaternion &q, const Vector3 &vel, const Scalar &dt) {
    // return w * q * (dt * half());
    auto w = (-q.x() * vel[0] - q.y() * vel[1] - q.z() * vel[2]) * (0.5 * dt);
    auto x = (q.w() * vel[0] + q.y() * vel[2] - q.z() * vel[1]) * (0.5 * dt);
    auto y = (q.w() * vel[1] + q.z() * vel[0] - q.x() * vel[2]) * (0.5 * dt);
    auto z = (q.w() * vel[2] + q.x() * vel[1] - q.y() * vel[0]) * (0.5 * dt);
    Quaternion delta = quat_from_xyzw(x, y, z, w);
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

  /**@brief Set the quaternion using euler angles, compatible with
   * PyBullet/ROS/Gazebo
   * @param yaw Angle around Z
   * @param pitch Angle around Y
   * @param roll Angle around X */
  EIGEN_ALWAYS_INLINE static const Quaternion quat_from_euler_rpy(
      const Vector3 &rpy) {
    Quaternion q;
    set_euler_rpy(q, rpy);
    return q;
  }

  EIGEN_ALWAYS_INLINE static const Quaternion inverse(const Quaternion &q) {
    Quaternion q2 = normalize(q);
    return q2.inverse();
  }

  EIGEN_ALWAYS_INLINE static const Vector3 get_euler_rpy(const Quaternion &q) {
    Quaternion q2 = normalize(q);

    // From tiny_quaternion.h
    Vector3 rpy;
    Scalar sarg;
    Scalar sqx = q2.x() * q2.x();
    Scalar sqy = q2.y() * q2.y();
    Scalar sqz = q2.z() * q2.z();
    Scalar squ = q2.w() * q2.w();
    sarg = -two() * (q2.x() * q2.z() - q2.w() * q2.y());

    // If the pitch angle is PI/2 or -PI/2, we can only compute
    // the sum roll + yaw.  However, any combination that gives
    // the right sum will produce the correct orientation, so we
    // set rollX = 0 and compute yawZ.

    // Compute results if -0.999 < sarg < 0.999
    rpy[0] = atan2(two() * (q2.y() * q2.z() + q2.w() * q2.x()),
                   squ - sqx - sqy + sqz);
    rpy[1] = asin(sarg);
    rpy[2] = atan2(two() * (q2.x() * q2.y() + q2.w() * q2.z()),
                   squ + sqx - sqy - sqz);

    // Check if sarg <= -0.9999, if so apply fix
    const Scalar thres1 = fraction(-99999, 100000);
    rpy[0] = tds::where_le(sarg, thres1, zero(), rpy[0]);
    rpy[1] = tds::where_le(sarg, thres1, half_pi(), rpy[1]);
    rpy[2] =
        tds::where_le(sarg, thres1, two() * atan2(q2.x(), -q2.y()), rpy[2]);

    // Check if sarg >= 0.9999, if so apply fix
    const Scalar thres2 = fraction(99999, 100000);
    rpy[0] = tds::where_ge(sarg, thres2, zero(), rpy[0]);
    rpy[1] = tds::where_ge(sarg, thres2, half_pi(), rpy[1]);
    rpy[2] =
        tds::where_ge(sarg, thres2, two() * atan2(-q2.x(), q2.y()), rpy[2]);

    return rpy;
  }

  EIGEN_ALWAYS_INLINE static const Vector3 get_euler_rpy2(const Quaternion &q) {
    Quaternion q2 = normalize(q);

    // From tiny_quaternion.h
    Scalar m00, m01, m02;
    Scalar m10, m11, m12;
    Scalar m20, m21, m22;
    const Scalar tx = two() * q2.x();
    const Scalar ty = two() * q2.y();
    const Scalar tz = two() * q2.z();
    const Scalar twx = tx * q2.w();
    const Scalar twy = ty * q2.w();
    const Scalar twz = tz * q2.w();
    const Scalar txx = tx * q2.x();
    const Scalar txy = ty * q2.x();
    const Scalar txz = tz * q2.x();
    const Scalar tyy = ty * q2.y();
    const Scalar tyz = tz * q2.y();
    const Scalar tzz = tz * q2.z();
    m00 = one() - (tyy + tzz);
    m01 = txy - twz;
    m02 = txz + twy;
    m10 = txy + twz;
    m11 = one() - (txx + tzz);
    m12 = tyz - twx;
    m20 = txz - twy;
    m21 = tyz + twx;
    m22 = one() - (txx + tyy);

    // Next extract Euler angles
    Vector3 rpy;

    rpy[0] = atan2(m12, m22);
    Scalar c2 = sqrt(m00 * m00 + m01 * m01);

    rpy[0] = tds::where_gt(rpy[0], zero(), -pi(), rpy[0]);
    rpy[1] = tds::where_gt(rpy[0], zero(), -atan2(-m02, -c2), -atan2(-m02, c2));

    /*
    if (rpy[0] > zero()) {
        rpy[0] -= pi();
        rpy[1] = -atan2(-m02, -c2);
    }
    else {
        rpy[1] = -atan2(-m02, c2);
    }
    */
    Scalar s1 = sin(rpy[0]);
    Scalar c1 = cos(rpy[0]);
    rpy[0] = -rpy[0];
    rpy[2] = -atan2(s1 * m20 - c1 * m10, c1 * m11 - s1 * m21);

    return rpy;
  }

  EIGEN_ALWAYS_INLINE static void set_euler_rpy(Quaternion &q,
                                                const Vector3 &rpy) {
    Scalar phi, the, psi;
    Scalar roll = rpy[0];
    Scalar pitch = rpy[1];
    Scalar yaw = rpy[2];
    phi = roll * half();
    the = pitch * half();
    psi = yaw * half();

    q.x() = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    q.y() = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    q.z() = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
    q.w() = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

    // Normalize quaternion in place
    Quaternion q2 = normalize(q);
    q.x() = q2.x();
    q.y() = q2.y();
    q.z() = q2.z();
    q.w() = q2.w();
  }

  EIGEN_ALWAYS_INLINE static void set_zero(Matrix3 &m) { m.setZero(); }
  EIGEN_ALWAYS_INLINE static void set_zero(Matrix3X &m) { m.setZero(); }
  EIGEN_ALWAYS_INLINE static void set_zero(Matrix6x3 &m) { m.setZero(); }
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

  EIGEN_ALWAYS_INLINE static const Vector3 get_row(const Matrix3 &m,
                                                   const int i) {
    return Vector3(m.row(i));
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
#ifdef USE_CPPAD
#if USE_CPPAD_CODEGEN
        if constexpr (std::is_same_v<std::remove_cv_t<Scalar>,
                                     CppAD::AD<CppAD::cg::CG<double>>>) {
      return CppAD::Value(CppAD::Var2Par(s)).getValue();
    } else if constexpr (std::is_same_v<std::remove_cv_t<Scalar>,
                                        CppAD::AD<double>>) {
      return CppAD::Value(CppAD::Var2Par(s));
    } else
#else
        if constexpr (std::is_same_v<std::remove_cv_t<Scalar>,
                                     CppAD::AD<double>>) {
      return CppAD::Value(CppAD::Var2Par(s));
    } else
#endif
#endif  // USE_CPPAD
    {
      return static_cast<double>(s);
    }
  }

  TINY_INLINE static Scalar from_double(double s) {
    return static_cast<Scalar>(s);
  }

  template <int Size1, int Size2>
  static void print(const std::string &title,
                    const Eigen::Matrix<Scalar, Size1, Size2> &m) {
    printf("%s\n", title.c_str());
    int rows = num_rows(m);
    int cols = num_cols(m);
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        Scalar s = m(r, c);
        double v = to_double(s);
        printf("%2.3f, ", v);
      }
      printf("\n");
    }
  }
  template <int Size1, int Size2 = 1>
  static void print(const std::string &title,
                    const Eigen::Array<Scalar, Size1, Size2> &v) {
    printf("%s\n", title.c_str());
    for (int r = 0; r < v.rows(); r++) {
      for (int c = 0; c < v.cols(); c++) {
        // Scalar s = v[r, c];
        // double d = to_double(s);
        // printf("%2.3f, ", d);
      }
      printf("\n");
    }
  }
  static void print(const std::string &title, const Scalar &v) {
    std::cout << title << "\n" << to_double(v) << std::endl;
  }
  static void print(const std::string &title, const Vector3 &v) {
    printf("%s\n", title.c_str());
    for (int c = 0; c < 3; c++) {
      Scalar val = v[c];
      double v = to_double(val);
      printf("%f, ", v);
    }
    printf("\n");
  }
  static void print(const std::string &title, const VectorX &v) {
    printf("%s\n", title.c_str());
    for (int c = 0; c < size(v); c++) {
      Scalar val = v[c];
      double v = to_double(val);
      printf("%f, ", v);
    }
    printf("\n");
  }
  static void print(const std::string &title, const Quaternion &q) {
    printf("%s (xyzw): \t", title.c_str());
    printf("%.6f  %.6f  %.6f  %.6f\n", to_double(q.x()), to_double(q.y()),
           to_double(q.z()), to_double(q.w()));
  }
  static void print(const std::string &title, const Matrix3 &m) {
    printf("%s\n", title.c_str());
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        Scalar s = m(r, c);
        double v = to_double(s);
        printf("%2.3f, ", v);
      }
      printf("\n");
    }
  }
  static void print(const std::string &title, const Matrix3X &m) {
    printf("%s\n", title.c_str());
    int rows = num_rows(m);
    int cols = num_cols(m);
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        Scalar s = m(r, c);
        double v = to_double(s);
        printf("%2.3f, ", v);
      }
      printf("\n");
    }
  }

  template <typename T>
  static void print(const std::string &title, const T &abi) {
    abi.print(title.c_str());
  }

  template <typename T>
  TINY_INLINE static auto sin(const T &s) {
#ifdef USE_CPPAD
    return CppAD::sin(s);
#else
    using std::sin;
    return sin(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto asin(const T &s) {
#ifdef USE_CPPAD
    return CppAD::asin(s);
#else
    using std::asin;
    return asin(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto cos(const T &s) {
#ifdef USE_CPPAD
    return CppAD::cos(s);
#else
    using std::cos;
    return cos(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto acos(const T &s) {
#ifdef USE_CPPAD
    return CppAD::acos(s);
#else
    using std::acos;
    return acos(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto tan(const T &s) {
#ifdef USE_CPPAD
    return CppAD::tan(s);
#else
    using std::tan;
    return tan(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto atan2(const T &dy, const T &dx) {
#ifdef USE_CPPAD
    return CppAD::atan2(dy, dx);
#else
    using std::atan2;
    return atan2(dy, dx);
#endif
  }

  template <typename T>
  TINY_INLINE static auto abs(const T &s) {
#ifdef USE_CPPAD
    return CppAD::abs(s);
#else
    using std::abs;
    return abs(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto sqrt(const T &s) {
#ifdef USE_CPPAD
    return CppAD::sqrt(s);
#else
    using std::sqrt;
    return sqrt(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto tanh(const T &s) {
#ifdef USE_CPPAD
    return CppAD::tanh(s);
#else
    using std::tanh;
    return tanh(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto pow(const T &s, const T &e) {
#ifdef USE_CPPAD
    return CppAD::pow(s, e);
#else
    using std::pow;
    return pow(s, e);
#endif
  }

  template <typename T>
  TINY_INLINE static auto exp(const T &s) {
#ifdef USE_CPPAD
    return CppAD::exp(s);
#else
    using std::exp;
    return exp(s);
#endif
  }

  template <typename T>
  TINY_INLINE static auto log(const T &s) {
#ifdef USE_CPPAD
    return CppAD::log(s);
#else
    using std::log;
    return log(s);
#endif
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
#ifdef USE_CPPAD
template <typename Scalar>
struct is_cppad_scalar<NeuralScalar<EigenAlgebraT<CppAD::AD<Scalar>>>> {
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
#endif  // USE_CPPAD

}  // end namespace tds
