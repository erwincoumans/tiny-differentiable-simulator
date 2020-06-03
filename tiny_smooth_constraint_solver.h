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

#ifndef TINY_SMOOTH_CONSTRAINT_SOLVER_H
#define TINY_SMOOTH_CONSTRAINT_SOLVER_H

#include "tiny_constraint_solver.h"
#include "tiny_rigid_body.h"

template <typename TinyScalar, typename TinyConstants>
struct TinySmoothConstraintSolver
    : public TinyConstraintSolver<TinyScalar, TinyConstants> {
  // Note that the LCP(A, b) is not explicitly constructed.
  // Baumgarte stabilization is used to reduce positional drift.See description
  // in Michael Cline's thesis:
  // https ://www.cs.ubc.ca/grads/resources/thesis/Nov02/Michael_Cline.pdf

  TinyScalar smoothness{};

  // Args:
  // cp: contact point
  // dt : delta time (in seconds)
  void resolveCollision(
      TinyContactPointRigidBody<TinyScalar, TinyConstants>& cp,
      TinyScalar dt) const override {
    typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
    TinyScalar erp =
        TinyConstants::fraction(1, 10);  // BAUMGARTE_ERROR_REDUCTION_PARAMETER
    const TinyVector3& world_point_a = cp.m_world_point_on_a;
    const TinyVector3& world_point_b = cp.m_world_point_on_b;
    TinyVector3 rel_pos_a =
        world_point_a - cp.m_rigid_body_a->m_world_pose.m_position;
    TinyVector3 rel_pos_b =
        world_point_b - cp.m_rigid_body_b->m_world_pose.m_position;
    TinyScalar baumgarte_rel_vel = TinyConstants::zero();

    //    if (cp.m_distance < TinyConstants::zero()) {
    baumgarte_rel_vel = erp * cp.m_distance / dt;
    TinyVector3 vel_a = cp.m_rigid_body_a->get_velocity(rel_pos_a);
    TinyVector3 vel_b = cp.m_rigid_body_b->get_velocity(rel_pos_b);
    TinyVector3 rel_vel = vel_a - vel_b;
    TinyScalar normal_rel_vel = cp.m_world_normal_on_b.dot(rel_vel);
    //          if (normal_rel_vel < TinyConstants::zero()) {
    TinyVector3 temp1 = cp.m_rigid_body_a->m_inv_inertia_world.dot(
        TinyVector3::cross2(rel_pos_a, cp.m_world_normal_on_b));
    TinyVector3 temp2 = cp.m_rigid_body_b->m_inv_inertia_world.dot(
        TinyVector3::cross2(rel_pos_b, cp.m_world_normal_on_b));
    TinyScalar ang =
        cp.m_world_normal_on_b.dot(TinyVector3::cross2(temp1, rel_pos_a) +
                                   TinyVector3::cross2(temp2, rel_pos_b));
    TinyScalar impulse =
        (-(TinyConstants::one() + cp.m_restitution) * normal_rel_vel -
         baumgarte_rel_vel) /
        (cp.m_rigid_body_a->m_inv_mass + cp.m_rigid_body_b->m_inv_mass + ang);
    if (impulse > TinyConstants::zero()) {
      TinyVector3 impulse_vector = impulse * cp.m_world_normal_on_b;
      cp.m_rigid_body_a->apply_impulse(impulse_vector, rel_pos_a);
      cp.m_rigid_body_b->apply_impulse(-impulse_vector, rel_pos_b);

      TinyVector3 lateral_rel_vel =
          rel_vel - normal_rel_vel * cp.m_world_normal_on_b;
      TinyScalar friction_impulse_trial =
          (lateral_rel_vel.length()) /
          (cp.m_rigid_body_a->m_inv_mass + cp.m_rigid_body_b->m_inv_mass + ang);

      TinyScalar friction_coeffcient = cp.m_friction;
      TinyScalar friction_impulse;
      if (friction_impulse_trial < friction_coeffcient * impulse) {
        friction_impulse = friction_impulse_trial;
      } else {
        friction_impulse = friction_coeffcient * impulse;
      }

      if ((lateral_rel_vel.length()) > TinyConstants::fraction(1, 10000)) {
        TinyVector3 friction_dir =
            lateral_rel_vel * (TinyConstants::one() / lateral_rel_vel.length());
        cp.m_rigid_body_a->apply_impulse(-friction_impulse * friction_dir,
                                         rel_pos_a);
        cp.m_rigid_body_b->apply_impulse(friction_impulse * friction_dir,
                                         rel_pos_b);
      }
    }
    //      }
    //    }
  }
};

#endif  // TINY_SMOOTH_CONSTRAINT_SOLVER_H
