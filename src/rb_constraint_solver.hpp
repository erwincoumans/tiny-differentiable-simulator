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

#include "contact_point.hpp"
#include "math/conditionals.hpp"
#include "rigid_body.hpp"


namespace tds {
template <typename Algebra>
struct RigidBodyContactPoint : public ContactPoint<Algebra> {
  typedef tds::RigidBody<Algebra> RigidBody;
  using Scalar = typename Algebra::Scalar;
  RigidBody* rigid_body_a{nullptr};
  RigidBody* rigid_body_b{nullptr};
  Scalar restitution;
  Scalar friction;
};

template <typename Algebra>
class RigidBodyConstraintSolver {
  template <typename OtherAlgebra>
  friend class RigidBodyConstraintSolver;

  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

 public:
  // Error reduction parameter in Baumgarte stabilization
  Scalar erp_{Algebra::fraction(1, 10)};

  virtual ~RigidBodyConstraintSolver() = default;

  template <typename AlgebraTo = Algebra>
  RigidBodyConstraintSolver<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    RigidBodyConstraintSolver<AlgebraTo> conv;
    conv.erp_ = C::convert(erp_);
    return conv;
  }

  // Note that the LCP(A, b) is not explicitly constructed.
  // Baumgarte stabilization is used to reduce positional drift.
  // See description in Michael Cline's thesis:
  // https ://www.cs.ubc.ca/grads/resources/thesis/Nov02/Michael_Cline.pdf

  // Args:
  // cp: contact point
  // dt : delta time (in seconds)
  virtual void resolve_collision(RigidBodyContactPoint<Algebra>& cp,
                                 const Scalar& dt) const {
    const Vector3& world_point_a = cp.world_point_on_a;
    const Vector3& world_point_b = cp.world_point_on_b;
    Vector3 rel_pos_a = world_point_a - cp.rigid_body_a->world_pose().position_;
    Vector3 rel_pos_b = world_point_b - cp.rigid_body_b->world_pose().position_;
    Scalar baumgarte_rel_vel = Algebra::zero();

    if constexpr (is_cppad_scalar<Scalar>::value) {
      Scalar dist_lt_zero = where_lt(cp.distance, Algebra::zero(),
                                     Algebra::one(), Algebra::zero());
      baumgarte_rel_vel = erp_ * cp.distance / dt;
      Vector3 vel_a = cp.rigid_body_a->get_velocity(rel_pos_a);
      Vector3 vel_b = cp.rigid_body_b->get_velocity(rel_pos_b);
      Vector3 rel_vel = vel_a - vel_b;
      Scalar normal_rel_vel = Algebra::dot(cp.world_normal_on_b, rel_vel);
      Scalar normal_rel_vel_lt_zero = where_lt(normal_rel_vel, Algebra::zero(),
                                               Algebra::one(), Algebra::zero());

      Vector3 temp1 = cp.rigid_body_a->inv_inertia_world() *
                      Algebra::cross(rel_pos_a, cp.world_normal_on_b);
      Vector3 temp2 = cp.rigid_body_b->inv_inertia_world() *
                      Algebra::cross(rel_pos_b, cp.world_normal_on_b);
      Scalar ang = Algebra::dot(
          cp.world_normal_on_b,
          Algebra::cross(temp1, rel_pos_a) + Algebra::cross(temp2, rel_pos_b));
      Scalar impulse =
          (-(Algebra::one() + cp.restitution) * normal_rel_vel -
           baumgarte_rel_vel) /
          (cp.rigid_body_a->inv_mass() + cp.rigid_body_b->inv_mass() + ang);
      impulse *= dist_lt_zero * normal_rel_vel_lt_zero;
      Vector3 impulse_vector = impulse * cp.world_normal_on_b;

      Vector3 lateral_rel_vel = rel_vel - normal_rel_vel * cp.world_normal_on_b;
      Scalar lateral_rel_vel_norm = Algebra::norm(lateral_rel_vel);
      Scalar friction_impulse_trial =
          (lateral_rel_vel_norm) /
          (cp.rigid_body_a->inv_mass() + cp.rigid_body_b->inv_mass() + ang);

      Scalar friction_coefficient = cp.friction;
      Scalar friction_impulse =
          where_lt(friction_impulse_trial, friction_coefficient * impulse,
                   friction_impulse_trial, friction_coefficient * impulse);
      Scalar friction_amount =
          where_gt(lateral_rel_vel_norm, Algebra::zero(),
                   Algebra::one() / lateral_rel_vel_norm, Algebra::zero());
      Vector3 friction_dir = lateral_rel_vel * friction_amount;
      impulse_vector -= friction_impulse * friction_dir;
      cp.rigid_body_a->apply_impulse(impulse_vector, rel_pos_a);
      cp.rigid_body_b->apply_impulse(-impulse_vector, rel_pos_b);
    } else {
      if (Algebra::less_than_zero(cp.distance)) {
        baumgarte_rel_vel = erp_ * cp.distance / dt;
        Vector3 vel_a = cp.rigid_body_a->get_velocity(rel_pos_a);
        Vector3 vel_b = cp.rigid_body_b->get_velocity(rel_pos_b);
        Vector3 rel_vel = vel_a - vel_b;
        Scalar normal_rel_vel = Algebra::dot(cp.world_normal_on_b, rel_vel);
        if (Algebra::less_than_zero(normal_rel_vel)) {
          Vector3 temp1 = cp.rigid_body_a->inv_inertia_world() *
                          Algebra::cross(rel_pos_a, cp.world_normal_on_b);
          Vector3 temp2 = cp.rigid_body_b->inv_inertia_world() *
                          Algebra::cross(rel_pos_b, cp.world_normal_on_b);
          Scalar ang = Algebra::dot(cp.world_normal_on_b,
                                    Algebra::cross(temp1, rel_pos_a) +
                                        Algebra::cross(temp2, rel_pos_b));
          Scalar impulse =
              (-(Algebra::one() + cp.restitution) * normal_rel_vel -
               baumgarte_rel_vel) /
              (cp.rigid_body_a->inv_mass() + cp.rigid_body_b->inv_mass() + ang);
          if (Algebra::greater_than_zero(impulse)) {
            Vector3 impulse_vector = impulse * cp.world_normal_on_b;
            cp.rigid_body_a->apply_impulse(impulse_vector, rel_pos_a);
            cp.rigid_body_b->apply_impulse(-impulse_vector, rel_pos_b);

            Vector3 lateral_rel_vel =
                rel_vel - normal_rel_vel * cp.world_normal_on_b;
            Scalar lateral_rel_vel_norm = Algebra::norm(lateral_rel_vel);
            Scalar friction_impulse_trial =
                (lateral_rel_vel_norm) / (cp.rigid_body_a->inv_mass() +
                                          cp.rigid_body_b->inv_mass() + ang);

            Scalar friction_coeffcient = cp.friction;
            Scalar friction_impulse;
            if (Algebra::less_than(friction_impulse_trial,
                                   friction_coeffcient * impulse)) {
              friction_impulse = friction_impulse_trial;
            } else {
              friction_impulse = friction_coeffcient * impulse;
            }

            if (Algebra::greater_than(lateral_rel_vel_norm,
                                      Algebra::fraction(1, 10000))) {
              Vector3 friction_dir =
                  lateral_rel_vel * (Algebra::one() / lateral_rel_vel_norm);
              cp.rigid_body_a->apply_impulse(-friction_impulse * friction_dir,
                                             rel_pos_a);
              cp.rigid_body_b->apply_impulse(friction_impulse * friction_dir,
                                             rel_pos_b);
            }
          }
        }
      }
    }
  }
};
template <typename AlgebraFrom, typename AlgebraTo = AlgebraFrom>
static TINY_INLINE RigidBodyConstraintSolver<AlgebraTo> clone(
    const RigidBodyConstraintSolver<AlgebraFrom>& s) {
  return s.template clone<AlgebraTo>();
}
}  // namespace tds
