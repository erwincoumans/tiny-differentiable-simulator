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

#ifndef TINY_MB_CONSTRAINT_SOLVER_SPRING_H
#define TINY_MB_CONSTRAINT_SOLVER_SPRING_H

#include "tiny_constraint_solver.h"
#include "tiny_mb_constraint_solver.h"
#include "tiny_multi_body.h"

//#define DEBUG

enum TinyVelocitySmoothingMethod {
  SMOOTH_VEL_NONE = 0,
  SMOOTH_VEL_SIGMOID,
  SMOOTH_VEL_TANH,
  SMOOTH_VEL_ABS
};

enum TinyFrictionForceModel {
  FRICTION_NONE = 0,
  FRICTION_COULOMB,
  FRICTION_ANDERSSON,
  FRICTION_HOLLARS,
  FRICTION_BROWN
};

/**
 * Nonlinear spring-damper contact model acting at the acceleration level.
 * Generalizes Hunt-Crossley and Kelvin-Voigt contact models.
 * Contact constraints are enforced smoothly, so that contact forces never
 * vanish completely. This is physically inaccurate, but beneficial for
 * trajectory optimization and potentially system identification.
 */
template <typename TinyScalar, typename TinyConstants>
struct TinyMultiBodyConstraintSolverSpring
    : public TinyMultiBodyConstraintSolver<TinyScalar, TinyConstants> {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyVectorX<TinyScalar, TinyConstants> TinyVectorX;
  typedef ::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
  typedef ::TinyMatrix3xX<TinyScalar, TinyConstants> TinyMatrix3xX;
  typedef ::TinyMultiBody<TinyScalar, TinyConstants> TinyMultiBody;
  typedef ::TinyContactPointMultiBody<TinyScalar, TinyConstants>
      TinyContactPoint;

  /**
   * Spring stiffness k.
   */
  TinyScalar spring_k{TinyConstants::fraction(5000, 1)};
  /**
   * Damper coefficient d.
   */
  TinyScalar damper_d{TinyConstants::fraction(5000, 1)};
  /**
   * Exponent `n` used in the Hunt-Crossley contact model.
   * 3/2 corresponds to contacting spheres under static conditions (Hertz).
   */
  TinyScalar exponent_n{TinyConstants::fraction(3, 2)};
  /**
   * Exponent `n` to be used if penetration is negative, i.e. no actual contact.
   */
  TinyScalar exponent_n_air{TinyConstants::fraction(1, 20)};
  /**
   * Exponent `n` to be used if penetration velocity is negative, i.e. no actual
   * contact.
   */
  TinyScalar exponent_vel_air{TinyConstants::fraction(1, 20)};

  /**
   * Whether a contact is only established when bodies are in collision.
   */
  bool hard_contact_condition{true};

  /**
   * Smoothing method to use for velocity smoothing.
   */
  TinyVelocitySmoothingMethod smoothing_method{SMOOTH_VEL_ABS};
  /**
   * Velocity smoothing coefficient.
   */
  TinyScalar smooth_alpha_vel{TinyConstants::fraction(1, 100)};
  /**
   * Normal force smoothing coefficient.
   */
  TinyScalar smooth_alpha_normal{TinyConstants::fraction(-1, 1)};

  /**
   * Static friction coefficient.
   */
  TinyScalar mu_static{TinyConstants::half()};
  /**
   * Sliding speed coefficient used by Andersson friction model.
   */
  TinyScalar andersson_vs{TinyConstants::one()};
  /**
   * Exponent used by Andersson friction model.
   */
  TinyScalar andersson_p{TinyConstants::one()};
  /**
   * Coefficient for the tanh component used by Andersson friction model.
   */
  TinyScalar andersson_ktanh{TinyConstants::one()};
  /**
   * Transition velocity used by Hollars and Brown friction models that acts as
   * upper limit on the drift velocity. Smaller values reduce drift velocity,
   * but make the friction equation more stiff. See implementation by Peter
   * Eastman et al. in SimTK Simbody
   * https://simtk.org/api_docs/simtkcore/api_docs20/classSimTK_1_1HuntCrossleyForce.html
   */
  TinyScalar v_transition{TinyConstants::fraction(1, 100)};

  TinyFrictionForceModel friction_model{FRICTION_BROWN};

  using TinyMultiBodyConstraintSolver<TinyScalar,
                                      TinyConstants>::needs_outer_iterations;

  TinyMultiBodyConstraintSolverSpring() { needs_outer_iterations = false; }

  /**
   * Compute force magnitude for a compliant contact model based on a nonlinear
   * spring-damper system. It generalizes the Kelvin-Voigt and the Hunt-Crossley
   * contact models, as described in Marhefka & Orin, "A Compliant Contact Model
   * with Nonlinear Damping for Simulation of Robotic Systems".
   * The velocity smoothing component is adapted from Control Toolbox, as
   * described in Neunert, Stäuble, Giftthaler et al. "Whole-Body Nonlinear
   * Model Predictive Control Through Contacts for Quadrupeds".
   * @param x Penetration depth (positive means objects penetrate, if less
   * than zero then objects are not colliding).
   * @param xd Velocity along the contact normal, i.e. the relative velocity
   * between the two bodies projected onto the surface normal.
   * @return Magnitude of the contact normal force.
   */
  virtual TinyScalar compute_contact_force(const TinyScalar& x,
                                           const TinyScalar& xd) const {
    // TODO move these to TinyConstants
    using std::tanh, std::exp, std::fabs, std::pow;

    const TinyScalar one = TinyConstants::one();
    const TinyScalar half = TinyConstants::half();
    const TinyScalar two = TinyConstants::two();
    const TinyScalar zero = TinyConstants::zero();

    // use abs(x) as base since x may be negative and pow() would yield NaN
    TinyScalar xn = pow(fabs(x), x < zero ? exponent_n_air : exponent_n);
    if (x < zero) {
      xn = -xn;
    }
    TinyScalar xdn = pow(fabs(xd), xd < zero ? exponent_vel_air : one);
    if (xd < zero) {
      xdn = -xdn;
    }

    // magnitude of contact normal force
    TinyScalar force;
    // compute damper force
    force = -damper_d * xn * xdn;

    // smooth force
    switch (smoothing_method) {
      case SMOOTH_VEL_SIGMOID:
        force *= one / (one + exp(x * smooth_alpha_vel));
        break;
      case SMOOTH_VEL_TANH:
        force *= half * tanh(-half * x * smooth_alpha_vel) + half;
        break;
      case SMOOTH_VEL_ABS:
        force *=
            half * -x * smooth_alpha_vel / (one + fabs(-x * smooth_alpha_vel)) +
            half;
        break;
      case SMOOTH_VEL_NONE:
      default:
        break;
    }

    // normal spring
    if (smooth_alpha_normal > zero) {
      force -= spring_k * exp(-smooth_alpha_normal * x);
    } else if (x > zero) {
      force -= spring_k * xn;
    }

    return force;
  }

  /**
   * Implements various friction models as described in Brown "Contact Modelling
   * for Forward Dynamics of Human Motion". This implementation only considers a
   * single lateral friction direction opposite to the tangential relative
   * velocity. Viscous friction is not considered.
   * @param fn Contact normal force magnitude.
   * @param v Tangential component of the relative velocity between the two
   * bodies.
   * @param mu Coulomb friction coefficient, or dynamic friction coefficient
   * `mu_d` in the other friction models.
   * @return Magnitude of friction force vector opposite to the tangential
   * relative velocity.
   */
  virtual TinyScalar compute_friction_force(const TinyScalar& fn,
                                            const TinyScalar& v,
                                            const TinyScalar& mu) const {
    // TODO move these to TinyConstants
    using std::tanh, std::exp, std::fabs, std::pow, std::abs, std::min;

    const TinyScalar one = TinyConstants::one();
    const TinyScalar half = TinyConstants::half();
    const TinyScalar fourth = half * half;
    const TinyScalar three_fourth = half + fourth;
    const TinyScalar two = TinyConstants::two();
    const TinyScalar four = two * two;
    const TinyScalar zero = TinyConstants::zero();

    const TinyScalar vvt = v / v_transition;

#ifdef DEBUG
    printf("mu: %.5f   force: %.5f\n", mu, fn);
#endif

    switch (friction_model) {
      default:
      case FRICTION_COULOMB:
        return mu * fn * (v < zero ? -one : one);
      case FRICTION_ANDERSSON:
        return fn *
               (mu + (mu_static - mu) *
                         exp(-pow(abs(v) / andersson_vs, andersson_p))) *
               tanh(andersson_ktanh * v);
      case FRICTION_HOLLARS:
        return fn * min(vvt, one) *
               (mu + (two * (mu_static - mu)) / (one + vvt * vvt));
      case FRICTION_BROWN:
        // Simplified three-parameter model (Eq. (4.5))
        // Brown "Contact Modelling for Forward Dynamics of Human Motion"
        TinyScalar denom = fourth * vvt * vvt + three_fourth;
        return fn * (mu * tanh(four * vvt) +
                     (mu_static - mu) * vvt / (denom * denom));
    }
  }

  // Args:
  // cps: contact points
  // dt : delta time (in seconds)
  virtual void resolveCollision(std::vector<TinyContactPoint>& cps,
                                TinyScalar dt) {
    if (cps.empty()) return;
    const int n_c = static_cast<int>(cps.size());

    const TinyContactPoint& cp0 = cps[0];

    TinyMultiBody* mb_a = cp0.m_multi_body_a;
    TinyMultiBody* mb_b = cp0.m_multi_body_b;

    const int n_a = mb_a->dof_qd();
    const int n_b = mb_b->dof_qd();
    const int n_ab = n_a + n_b;
    if (n_ab == 0) return;

    TinyVector3 vel_a, vel_b;
    TinyVector3 rel_pos_a, rel_pos_b;

    // joint torques to be applied (include DOFs for floating base)
    TinyVectorX tau_a(mb_a->dof_qd()), tau_b(mb_b->dof_qd());
    tau_a.set_zero();
    tau_b.set_zero();

    for (const TinyContactPoint& cp : cps) {
      if (!hard_contact_condition || cp.m_distance < TinyConstants::zero()) {
        const TinyVector3& world_point_a = cp.m_world_point_on_a;
        const TinyVector3& world_point_b = cp.m_world_point_on_b;
        const TinyVector3& world_normal = -cp.m_world_normal_on_b;  // !!!
        TinyMatrix3xX jac_a = mb_a->point_jacobian(cp.m_link_a, world_point_a);
        TinyMatrix3xX jac_b = mb_b->point_jacobian(cp.m_link_b, world_point_b);

        TinyVectorX qd_a(mb_a->m_qd);
        TinyVectorX qd_b(mb_b->m_qd);
        vel_a = jac_a * qd_a;
        vel_b = jac_b * qd_b;
        TinyVector3 rel_vel = vel_a - vel_b;

        // contact normal force
        TinyScalar normal_rel_vel = world_normal.dot(rel_vel);
        TinyScalar force_normal =
            compute_contact_force(-cp.m_distance, normal_rel_vel);
#ifdef DEBUG
        printf("Contact normal force magnitude: %.5f\n", force_normal);
#endif
        TinyVector3 force_vector = world_normal * force_normal;
        tau_a += jac_a.mul_transpose(force_vector);
        tau_b -= jac_b.mul_transpose(force_vector);

        if (friction_model == FRICTION_NONE) {
          continue;
        }
        // unilateral friction force
        TinyVector3 lateral_rel_vel =
            rel_vel - normal_rel_vel * cp.m_world_normal_on_b;
        //      lateral_rel_vel.print("lateral_rel_vel");
        const TinyScalar lateral = lateral_rel_vel.length();
        //      printf("lateral_rel_vel.length(): %.6f\n", lateral);

        TinyVector3 fr_direction1, fr_direction2;
        if (lateral < TinyConstants::fraction(1, 10000)) {
          // use the plane space of the contact normal as friction directions
          cp.m_world_normal_on_b.plane_space(fr_direction1, fr_direction2);
        } else {
          // use the negative lateral velocity and its orthogonal as friction
          // directions
          fr_direction1 = lateral_rel_vel * (TinyConstants::one() / lateral);
          //        fr_direction2 = fr_direction1.cross(cp.m_world_normal_on_b);
        }
        TinyScalar friction =
            compute_friction_force(force_normal, lateral, cp.m_friction);
        TinyVector3 friction_vector = fr_direction1 * friction;
        tau_a += jac_a.mul_transpose(friction_vector);
        tau_b -= jac_b.mul_transpose(friction_vector);
      }
    }
    // apply forces
    if (n_a > 0) {
      int tau_offset_a = 0;
      if (mb_a->m_isFloating) {
        for (int i = 0; i < 6; ++i) {
          mb_a->m_baseAppliedForce[i] += tau_a[i];
        }
        tau_offset_a = 6;
      }
      for (int i = 0; i < mb_a->dof_actuated(); ++i) {
        mb_a->m_tau[i] += tau_a[i + tau_offset_a];
      }
    }
    if (n_b > 0) {
      int tau_offset_b = 0;
      if (mb_b->m_isFloating) {
        for (int i = 0; i < 6; ++i) {
          mb_b->m_baseAppliedForce[i] += tau_b[i];
        }
        tau_offset_b = 6;
      }
      for (int i = 0; i < mb_b->dof_actuated(); ++i) {
        mb_b->m_tau[i] += tau_b[i + tau_offset_b];
      }
    }
  }
};

#endif  // TINY_MB_CONSTRAINT_SOLVER_SPRING_H
