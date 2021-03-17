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

#include "utils/conversion.hpp"

namespace tds {
/// Pose is a coordinate frame specified as position and orientation
/// quaternion.
template <typename Algebra>
struct Pose {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;

  Vector3 position_;
  Quaternion orientation_;

  Pose() = default;

  Pose(const Vector3& position_, const Quaternion& orientation_)
      : position_(position_), orientation_(orientation_) {}
  Vector3 transform(const Vector3& point) const {
    return Algebra::rotate(orientation_, point) + position_;
  }

  Vector3 inverse_transform(const Vector3& point) const {
    Vector3 point_out;
    point_out = point - position_;
    return Algebra::rotate(Algebra::inverse(orientation_), point_out);
  }

  Pose operator*(const Pose& b) const {
    const Pose& a = *this;
    Pose res = a;
    res.position_ += Algebra::rotate(a.orientation_, b.position_);
    res.orientation_ *= b.orientation_;
    return res;
  }

  void set_identity() {
    position_.set_zero();
    orientation_.set_identity();
  }

  void inverse() {
    orientation_ = Algebra::inverse(orientation_);
    position_ = Algebra::rotate(orientation_, -position_);
  }
};

template <typename AlgebraFrom, typename AlgebraTo = AlgebraFrom>
static inline Pose<AlgebraTo> clone(const Pose<AlgebraFrom>& p) {
  typedef Conversion<AlgebraFrom, AlgebraTo> C;
  return Pose<AlgebraTo>(C::convert(p.position_), C::convert(p.orientation_));
}
}  // namespace tds
