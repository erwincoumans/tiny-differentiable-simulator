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

#include "base.hpp"
#include "geometry.hpp"
#include "math/conditionals.hpp"
#include "math/pose.hpp"

namespace tds {
template <typename Algebra>
class RigidBody {
  template <typename OtherAlgebra>
  friend class RigidBody;

  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  typedef tds::Geometry<Algebra> Geometry;
  typedef tds::Pose<Algebra> Pose;

 public:
  Pose world_pose_;
  Vector3 linear_velocity_;
  Vector3 angular_velocity_;
  Vector3 total_force_;
  Vector3 total_torque_;
  Matrix3 inv_inertia_world_;
  Scalar mass_;
  Scalar inv_mass_;

  const Geometry* geometry_;

  RigidBody(const Scalar& mass, const Geometry* geometry)
      : mass_(mass), geometry_(geometry) {
    inv_mass_ = tds::where_eq(mass_, Algebra::zero(), Algebra::zero(),
                              Algebra::one() / mass_);
    inv_inertia_world_ =
        mass_ == Algebra::zero() ? Algebra::zero33() : Algebra::eye3();
    Algebra::set_zero(world_pose_.position_);
    Algebra::set_identity(world_pose_.orientation_);
    Algebra::set_zero(linear_velocity_);
    Algebra::set_zero(angular_velocity_);
    Algebra::set_zero(total_force_);
    Algebra::set_zero(total_torque_);
  }

  template <typename AlgebraTo = Algebra>
  RigidBody<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    RigidBody<AlgebraTo> conv(C::convert(mass_),
                              tds::clone<Algebra, AlgebraTo>(geometry_));
    conv.world_pose_ = tds::clone<Algebra, AlgebraTo>(world_pose_);
    conv.linear_velocity_ = C::convert(linear_velocity_);
    conv.angular_velocity_ = C::convert(angular_velocity_);
    conv.total_force_ = C::convert(total_force_);
    conv.total_torque_ = C::convert(total_torque_);
    return conv;
  }

  TINY_INLINE Pose& world_pose() { return world_pose_; }
  TINY_INLINE const Pose& world_pose() const { return world_pose_; }

  const Geometry* geometry() const { return geometry_; }

  /// Apply gravity force given the acceleration.
  void apply_gravity(const Vector3& gravity_acceleration) {
    Vector3 gravity_force = mass_ * gravity_acceleration;
    apply_central_force(gravity_force);
  }

  const Scalar& mass() const { return mass_; }
  const Scalar& inv_mass() const { return inv_mass_; }
  const Matrix3& inv_inertia_world() const { return inv_inertia_world_; }

  /// Apply a force at the center of mass.
  void apply_central_force(const Vector3& force) { total_force_ += force; }

  void apply_force_impulse(const Scalar& dt) {
    linear_velocity_ += total_force_ * inv_mass_ * dt;
    Vector3 temp = inv_inertia_world_ * total_torque_;
    angular_velocity_ += inv_inertia_world_ * total_torque_ * dt;
  }

  Vector3 get_velocity(const Vector3& rel_pos) {
    return linear_velocity_ + Algebra::cross(angular_velocity_, rel_pos);
  }

  /// Apply an impulse at a position relative to the center of mass.
  void apply_impulse(const Vector3& impulse, const Vector3& rel_pos) {
    linear_velocity_ += inv_mass_ * impulse;
    Vector3 torqueImpulse = Algebra::cross(rel_pos, impulse);
    angular_velocity_ += inv_inertia_world_ * torqueImpulse;
  }

  /// Clear the applied forces and torques to zero.
  void clear_forces() {
    Algebra::set_zero(total_force_);
    Algebra::set_zero(total_torque_);
  }

  void integrate(const Scalar& dt) {
    world_pose_.position_ += linear_velocity_ * dt;
    Algebra::quat_increment(world_pose_.orientation_,
                            Algebra::quat_velocity(world_pose_.orientation_,
                                                   angular_velocity_, dt));
    world_pose_.orientation_ = Algebra::normalize(world_pose_.orientation_);
  }
};

template <typename AlgebraFrom, typename AlgebraTo = AlgebraFrom>
static TINY_INLINE RigidBody<AlgebraTo> clone(
    const RigidBody<AlgebraFrom>& rb) {
  return rb.template clone<AlgebraTo>();
}
}  // namespace tds
