#pragma once

#include "base.hpp"

namespace tds {
/**
 * Convert between types of `AlgebraFrom` to `AlgebraTo`.
 * Copies the input argument if both algebras are the same.
 */
template <typename AlgebraFrom, typename AlgebraTo>
struct Conversion {
  using ScalarFrom = typename AlgebraFrom::Scalar;
  using QuaternionFrom = typename AlgebraFrom::Quaternion;
  using Vector3From = typename AlgebraFrom::Vector3;
  using VectorXFrom = typename AlgebraFrom::VectorX;
  using Matrix3From = typename AlgebraFrom::Matrix3;
  using Matrix3XFrom = typename AlgebraFrom::Matrix3X;
  using Matrix6From = typename AlgebraFrom::Matrix6;
  using MatrixXFrom = typename AlgebraFrom::MatrixX;

  using ScalarTo = typename AlgebraTo::Scalar;
  using QuaternionTo = typename AlgebraTo::Quaternion;
  using Vector3To = typename AlgebraTo::Vector3;
  using VectorXTo = typename AlgebraTo::VectorX;
  using Matrix3To = typename AlgebraTo::Matrix3;
  using Matrix3XTo = typename AlgebraTo::Matrix3X;
  using Matrix6To = typename AlgebraTo::Matrix6;
  using MatrixXTo = typename AlgebraTo::MatrixX;

  TINY_INLINE static ScalarTo convert(const ScalarFrom& s) {
    // convert to double nonetheless to remove gradient trace
    return AlgebraTo::from_double(AlgebraFrom::to_double(s));
  }

  TINY_INLINE static QuaternionTo convert(const QuaternionFrom& quat) {
    if constexpr (std::is_same_v<AlgebraFrom, AlgebraTo>) {
      return quat;
    } else {
      double x = AlgebraFrom::to_double(AlgebraFrom::quat_x(quat));
      double y = AlgebraFrom::to_double(AlgebraFrom::quat_y(quat));
      double z = AlgebraFrom::to_double(AlgebraFrom::quat_z(quat));
      double w = AlgebraFrom::to_double(AlgebraFrom::quat_w(quat));
      return AlgebraTo::quat_from_xyzw(
          AlgebraTo::from_double(x), AlgebraTo::from_double(y),
          AlgebraTo::from_double(z), AlgebraTo::from_double(w));
    }
  }

  TINY_INLINE static Vector3To convert(const Vector3From& v) {
    if constexpr (std::is_same_v<AlgebraFrom, AlgebraTo>) {
      return v;
    } else {
      double x = AlgebraFrom::to_double(v[0]);
      double y = AlgebraFrom::to_double(v[1]);
      double z = AlgebraFrom::to_double(v[2]);
      return Vector3To(AlgebraTo::from_double(x), AlgebraTo::from_double(y),
                       AlgebraTo::from_double(z));
    }
  }

  TINY_INLINE static VectorXTo convert(const VectorXFrom& v) {
    if constexpr (std::is_same_v<AlgebraFrom, AlgebraTo>) {
      return v;
    } else {
      VectorXTo conv = AlgebraTo::zerox(AlgebraFrom::size(v));
      for (int i = 0; i < static_cast<int>(AlgebraFrom::size(v)); ++i) {
        conv[i] = AlgebraTo::from_double(AlgebraFrom::to_double(v[i]));
      }
      return conv;
    }
  }

  TINY_INLINE static Matrix3To convert(const Matrix3From& m) {
    if constexpr (std::is_same_v<AlgebraFrom, AlgebraTo>) {
      return m;
    } else {
      Matrix3To conv;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          conv(i, j) = AlgebraTo::from_double(AlgebraFrom::to_double(m(i, j)));
        }
      }
      return conv;
    }
  }

  TINY_INLINE static Matrix3XTo convert(const Matrix3XFrom& m) {
    if constexpr (std::is_same_v<AlgebraFrom, AlgebraTo>) {
      return m;
    } else {
      int num_cols = AlgebraFrom::num_cols(m);
      Matrix3XTo conv = AlgebraTo::create_matrix_3x(num_cols);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < num_cols; ++j) {
          conv(i, j) = AlgebraTo::from_double(AlgebraFrom::to_double(m(i, j)));
        }
      }
      return conv;
    }
  }

  TINY_INLINE static Matrix6To convert(const Matrix6From& m) {
    if constexpr (std::is_same_v<AlgebraFrom, AlgebraTo>) {
      return m;
    } else {
      Matrix6To conv;
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          conv(i, j) = AlgebraTo::from_double(AlgebraFrom::to_double(m(i, j)));
        }
      }
      return conv;
    }
  }

  TINY_INLINE static MatrixXTo convert(const MatrixXFrom& m) {
    if constexpr (std::is_same_v<AlgebraFrom, AlgebraTo>) {
      return m;
    } else {
      int num_rows = AlgebraFrom::num_rows(m);
      int num_cols = AlgebraFrom::num_cols(m);
      MatrixXTo conv = AlgebraTo::create_matrix_x(num_rows, num_cols);
      for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j) {
          conv(i, j) = AlgebraTo::from_double(AlgebraFrom::to_double(m(i, j)));
        }
      }
      return conv;
    }
  }
};
}  // namespace tds