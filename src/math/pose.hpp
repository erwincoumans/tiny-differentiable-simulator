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

namespace tds {
/// Pose is a coordinate frame specified as position and orientation
/// quaternion.
template <typename Algebra>
struct Pose {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;

  Vector3 position;
  Quaternion orientation;

  Pose() = default;

  Pose(const Vector3& position, const Quaternion& orientation)
      : position(position), orientation(orientation) {}
  Vector3 transform(const Vector3& point) const {
    return Algebra::rotate(orientation, point) + position;
  }

  Vector3 inverse_transform(const Vector3& point) const {
    Vector3 point_out;
    point_out = point - position;
    return Algebra::rotate(Algebra::inverse(orientation), point_out);
  }

  Pose operator*(const Pose& b) const {
    const Pose& a = *this;
    Pose res = a;
    res.position += Algebra::rotate(a.orientation, b.position);
    res.orientation *= b.orientation;
    return res;
  }

  void set_identity() {
    position.set_zero();
    orientation.set_identity();
  }

  void inverse() {
    orientation = Algebra::inverse(orientation);
    position = Algebra::rotate(orientation, -position);
  }
};
}  // namespace tds
