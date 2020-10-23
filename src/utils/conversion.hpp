#pragma once

#include "base.hpp"

namespace tds {
/**
 * Convert between types of `AlgebraFrom` to `AlgebraTo`.
 */
template <typename AlgebraFrom, typename AlgebraTo>
struct Conversion {
  using QuaternionFrom = typename AlgebraFrom::Quaternion;
  using Vector3From = typename AlgebraFrom::Vector3;
  using VectorXFrom = typename AlgebraFrom::VectorX;
  using Matrix3From = typename AlgebraFrom::Matrix3;
  using Matrix6From = typename AlgebraFrom::Matrix6;
  using MatrixXFrom = typename AlgebraFrom::MatrixX;

  using QuaternionTo = typename AlgebraTo::Quaternion;
  using Vector3To = typename AlgebraTo::Vector3;
  using VectorXTo = typename AlgebraTo::VectorX;
  using Matrix3To = typename AlgebraTo::Matrix3;
  using Matrix6To = typename AlgebraTo::Matrix6;
  using MatrixXTo = typename AlgebraTo::MatrixX;

  TINY_INLINE static QuaternionTo convert(const QuaternionFrom& quat) {
    double x = AlgebraFrom::to_double(AlgebraFrom::quat_x(quat));
    double y = AlgebraFrom::to_double(AlgebraFrom::quat_y(quat));
    double z = AlgebraFrom::to_double(AlgebraFrom::quat_z(quat));
    double w = AlgebraFrom::to_double(AlgebraFrom::quat_w(quat));
    return AlgebraTo::quat_from_xyzw(
        AlgebraTo::from_double(x), AlgebraTo::from_double(y),
        AlgebraTo::from_double(z), AlgebraTo::from_double(w));
  }

  TINY_INLINE static Vector3To convert(const Vector3From& v) {
    double x = AlgebraFrom::to_double(v[0]);
    double y = AlgebraFrom::to_double(v[1]);
    double z = AlgebraFrom::to_double(v[2]);
    return Vector3To(AlgebraTo::from_double(x), AlgebraTo::from_double(y),
                     AlgebraTo::from_double(z));
  }
};
}  // namespace tds