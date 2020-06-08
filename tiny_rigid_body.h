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

#ifndef TINY_RIGIDBODY_H
#define TINY_RIGIDBODY_H

#include "tiny_geometry.h"
#include "tiny_matrix3x3.h"
#include "tiny_pose.h"

template <typename TinyScalar, typename TinyConstants>
struct TinyRigidBody {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyGeometry<TinyScalar, TinyConstants> TinyGeometry;
  typedef ::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;

  TinyPose<TinyScalar, TinyConstants> m_world_pose;
  TinyVector3 m_linear_velocity;
  TinyVector3 m_angular_velocity;
  TinyVector3 m_local_inertia;
  TinyVector3 m_total_force;
  TinyVector3 m_total_torque;
  TinyMatrix3x3 m_inv_inertia_world;
  TinyScalar m_mass;
  TinyScalar m_inv_mass;
  int m_user_index;

  const TinyGeometry* m_geometry;

  TinyRigidBody(TinyScalar mass, const TinyGeometry* geometry)
      : m_mass(mass), m_user_index(-1), m_geometry(geometry) {
    m_inv_mass = m_mass == TinyConstants::zero()
                     ? TinyConstants::zero()
                     : TinyConstants::one() / m_mass;
    m_inv_inertia_world = m_mass == TinyConstants::zero()
                              ? TinyMatrix3x3::get_zero()
                              : TinyMatrix3x3::get_identity();
    m_world_pose.m_position.set_zero();
    m_world_pose.m_orientation.set_identity();
    m_linear_velocity.set_zero();
    m_angular_velocity.set_zero();
    m_total_force.set_zero();
    m_total_torque.set_zero();

    // m_local_inertia(local_inertia)
  }

  /// Apply gravity force given the acceleration.
  void apply_gravity(const TinyVector3& gravity_acceleration) {
    TinyVector3 gravity_force = m_mass * gravity_acceleration;
    apply_central_force(gravity_force);
  }

  /// Apply a force at the center of mass.
  void apply_central_force(const TinyVector3& force) { m_total_force += force; }

  void apply_force_impulse(TinyScalar dt) {
    m_linear_velocity += m_total_force * m_inv_mass * dt;
    m_angular_velocity += m_inv_inertia_world.dot(m_total_torque) * dt;
  }

  TinyVector3 get_velocity(const TinyVector3& rel_pos) {
    return m_linear_velocity + TinyVector3::cross2(m_angular_velocity, rel_pos);
  }

  /// Apply an impulse at a position relative to the center of mass.
  void apply_impulse(const TinyVector3& impulse, const TinyVector3& rel_pos) {
    m_linear_velocity += m_inv_mass * impulse;
    TinyVector3 torqueImpulse = TinyVector3::cross2(rel_pos, impulse);
    m_angular_velocity += m_inv_inertia_world.dot(torqueImpulse);
  }

  /// Clear the applied forces and torques to zero.
  void clear_forces() {
    m_total_force.set_zero();
    m_total_torque.set_zero();
  }

  void integrate(TinyScalar dt) {
    m_world_pose.m_position += m_linear_velocity * dt;
    m_world_pose.m_orientation +=
        (m_angular_velocity * m_world_pose.m_orientation) *
        (dt * TinyConstants::half());
    m_world_pose.m_orientation.normalize();
  }
};

#endif  // TINY_RIGIDBODY_H
