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
#include "mb_constraint_solver.hpp"
#include "multi_body.hpp"
#include "math/tiny/neural_scalar.hpp"

//#define DEBUG


namespace tds {

enum VelocitySmoothingMethod {
  SMOOTH_VEL_NONE = 0,
  SMOOTH_VEL_SIGMOID,
  SMOOTH_VEL_TANH,
  SMOOTH_VEL_ABS
};

enum FrictionForceModel {
  FRICTION_NONE = 0,
  FRICTION_COULOMB,
  FRICTION_ANDERSSON,
  FRICTION_HOLLARS,
  FRICTION_BROWN,
  FRICTION_NEURAL,
};

/**
 * Nonlinear spring-damper contact model acting at the acceleration level.
 * Generalizes Hunt-Crossley and Kelvin-Voigt contact models.
 * Contact constraints are enforced smoothly, so that contact forces never
 * vanish completely. This is physically inaccurate, but beneficial for
 * trajectory optimization and potentially system identification.
 */
template <typename Algebra>
class MultiBodyConstraintSolverSpring
    : public MultiBodyConstraintSolver<Algebra> {
  using Scalar = typename Algebra::Scalar;
  typedef typename Algebra::Vector3 Vector3;
  typedef typename Algebra::VectorX VectorX;
  typedef typename Algebra::Matrix3 Matrix3;
  typedef typename Algebra::Matrix3X Matrix3X;
  typedef MultiBodyContactPoint<Algebra> ContactPoint;

 public:
  /**
   * Spring stiffness k.
   */
  Scalar spring_k_{Algebra::fraction(50000, 1)};
  /**
   * Damper coefficient d.
   */
  Scalar damper_d_{Algebra::fraction(50000, 1)};
  /**
   * Exponent `n` used in the Hunt-Crossley contact model.
   * 3/2 corresponds to contacting spheres under static conditions (Hertz).
   */
  Scalar exponent_n_{Algebra::fraction(3, 2)};
  /**
   * Exponent `n` to be used if penetration is negative, i.e. no actual contact.
   */
  Scalar exponent_n_air_{Algebra::fraction(1, 20)};
  /**
   * Exponent `n` to be used if penetration velocity is negative, i.e. no actual
   * contact.
   */
  Scalar exponent_vel_air_{Algebra::fraction(1, 20)};

  /**
   * Whether a contact is only established when bodies are in collision.
   */
  bool hard_contact_condition_{true};

  /**
   * Smoothing method to use for velocity smoothing.
   */
  VelocitySmoothingMethod smoothing_method_{SMOOTH_VEL_NONE};
  /**
   * Velocity smoothing coefficient.
   */
  Scalar smooth_alpha_vel_{Algebra::fraction(1, 100)};
  /**
   * Normal force smoothing coefficient.
   */
  Scalar smooth_alpha_normal_{Algebra::fraction(-1, 1)};

  /**
   * Static friction coefficient.
   */
  Scalar mu_static_{Algebra::half()};
  /**
   * Sliding speed coefficient used by Andersson friction model.
   */
  Scalar andersson_vs_{Algebra::one()};
  /**
   * Exponent used by Andersson friction model.
   */
  Scalar andersson_p_{Algebra::one()};
  /**
   * Coefficient for the tanh component used by Andersson friction model.
   */
  Scalar andersson_ktanh_{Algebra::one()};
  /**
   * Transition velocity used by Hollars and Brown friction models that acts as
   * upper limit on the drift velocity. Smaller values reduce drift velocity,
   * but make the friction equation more stiff. See implementation by Peter
   * Eastman et al. in SimTK Simbody
   * https://simtk.org/api_docs/simtkcore/api_docs20/classSimTK_1_1HuntCrossleyForce.html
   */
  Scalar v_transition_{Algebra::fraction(1, 100)};

  FrictionForceModel friction_model_{FRICTION_BROWN};

 protected:
  using MultiBodyConstraintSolver<Algebra>::needs_outer_iterations_;

 public:
  MultiBodyConstraintSolverSpring() { needs_outer_iterations_ = false; }

  template <typename AlgebraTo = Algebra>
  MultiBodyConstraintSolverSpring<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    MultiBodyConstraintSolverSpring<AlgebraTo> conv;
    conv.needs_outer_iterations_ = needs_outer_iterations_;
    conv.spring_k_ = C::convert(spring_k_);
    conv.damper_d_ = C::convert(damper_d_);
    conv.exponent_n_ = C::convert(exponent_n_);
    conv.exponent_n_air_ = C::convert(exponent_n_air_);
    conv.exponent_vel_air_ = C::convert(exponent_vel_air_);
    conv.hard_contact_condition_ = hard_contact_condition_;
    conv.smoothing_method_ = C::convert(smoothing_method_);
    conv.smooth_alpha_vel_ = C::convert(smooth_alpha_vel_);
    conv.smooth_alpha_normal_ = C::convert(smooth_alpha_normal_);
    conv.mu_static_ = C::convert(mu_static_);
    conv.andersson_vs_ = C::convert(andersson_vs_);
    conv.andersson_p_ = C::convert(andersson_p_);
    conv.andersson_ktanh_ = C::convert(andersson_ktanh_);
    conv.v_transition_ = C::convert(v_transition_);
    conv.friction_model_ = friction_model_;
    return conv;
  }

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
  virtual Scalar compute_contact_force(const Scalar& x,
                                       const Scalar& xd) const {
    const Scalar one = Algebra::one();
    const Scalar half = Algebra::half();
    const Scalar two = Algebra::two();
    const Scalar zero = Algebra::zero();

    // use abs(x) as base since x may be negative and pow() would yield NaN
    Scalar x_exp = tds::where_lt(x, zero, exponent_n_air_, exponent_n_);
    Scalar xn_pow = Algebra::pow(Algebra::abs(x), x_exp);
    Scalar xn = tds::where_lt(x, zero, -xn_pow, xn_pow);

    Scalar xd_exp = tds::where_lt(xd, zero, exponent_vel_air_, one);
    Scalar xdn_pow = Algebra::pow(Algebra::abs(xd), xd_exp);
    Scalar xdn = tds::where_lt(xd, zero, -xdn_pow, xdn_pow);

    // magnitude of contact normal force
    Scalar force;
    // compute damper force
    force = -damper_d_ * xn * xdn;

    // smooth force
    switch (smoothing_method_) {
      case SMOOTH_VEL_SIGMOID:
        force *= one / (one + Algebra::exp(x * smooth_alpha_vel_));
        break;
      case SMOOTH_VEL_TANH:
        force *= half * Algebra::tanh(-half * x * smooth_alpha_vel_) + half;
        break;
      case SMOOTH_VEL_ABS:
        force *= half * -x * smooth_alpha_vel_ /
                     (one + Algebra::abs(-x * smooth_alpha_vel_)) +
                 half;
        break;
      case SMOOTH_VEL_NONE:
      default:
        break;
    }

    // normal spring
    if constexpr (is_cppad_scalar<Scalar>::value) {
    // if constexpr (true) {
      Scalar pos_n = tds::where_gt(smooth_alpha_normal_, zero, one, zero);
      Scalar if_f = force - spring_k_ * Algebra::exp(-smooth_alpha_normal_ * x);
      Scalar else_f = tds::where_gt(x, zero, force - spring_k_ * xn, force);
      force = pos_n * if_f + (one - pos_n) * else_f;
    } else {
      if (smooth_alpha_normal_ > zero) {
        force -= spring_k_ * Algebra::exp(-smooth_alpha_normal_ * x);
      } else if (x > zero) {
        force -= spring_k_ * xn;
      }
    }

    // if constexpr (is_neural_scalar<Algebra>::value) {
    //   // evaluate neural network blueprint (if available)
    //   x.assign("contact_normal_force/x");
    //   xd.assign("contact_normal_force/xd");
    //   force.assign("contact_normal_force/force");
    // }

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
  virtual Scalar compute_friction_force(const Scalar& fn, const Scalar& v,
                                        const Scalar& mu) const {
    const Scalar one = Algebra::one();
    const Scalar half = Algebra::half();
    const Scalar fourth = half * half;
    const Scalar three_fourth = half + fourth;
    const Scalar two = Algebra::two();
    const Scalar four = two * two;
    const Scalar zero = Algebra::zero();
    const Scalar epsilon = Algebra::fraction(1, 1000);

    Scalar vvt = v / v_transition_;
    // if (vvt > Algebra::fraction(10000, 1)) {
    //   printf("vvt:          %.8f\n", Algebra::getDouble(vvt));
    //   printf("v_transition_: %.8f\n", Algebra::getDouble(v_transition_));
    //   vvt = Algebra::fraction(10000, 1);
    // }

#ifdef DEBUG
    printf("mu: %.5f   force: %.5f\n", mu, fn);
#endif

    switch (friction_model_) {
      default:
      case FRICTION_COULOMB:
        return mu * fn * tds::where_lt(v, zero, -one, one);
      case FRICTION_ANDERSSON:
        return fn *
               (mu + (mu_static_ - mu) *
                         Algebra::exp(-Algebra::pow(
                             Algebra::abs(v) / andersson_vs_, andersson_p_))) *
               Algebra::tanh(andersson_ktanh_ * v);
      case FRICTION_HOLLARS:
        return fn * Algebra::min(vvt, one) *
               (mu + (two * (mu_static_ - mu)) / (one + vvt * vvt));
      case FRICTION_BROWN: {
        // Simplified three-parameter model (Eq. (4.5))
        // Brown "Contact Modelling for Forward Dynamics of Human Motion"
        Scalar denom = fourth * vvt * vvt + three_fourth;
        // XXX add epsilon in tanh() to prevent taking gradients close to zero
        return fn * (mu * Algebra::tanh(four * vvt + epsilon) +
                     (mu_static_ - mu) * vvt / (denom * denom));
      }
      case FRICTION_NEURAL:
        // if constexpr (is_neural_scalar<Algebra>::value) {
        //   // evaluate neural network blueprint (if available)
        //   fn.assign("contact_friction_force/fn");
        //   v.assign("contact_friction_force/v");
        //   Scalar force = zero;
        //   force.assign("contact_friction_force/force");
        //   return force.evaluate();
        // }
        return zero;
    }
  }

 public:
  // Args:
  // cps: contact points
  // dt : delta time (in seconds)
  virtual void resolve_collision(const std::vector<ContactPoint>& cps,
                                 const Scalar& /*dt*/) {
    if (cps.empty()) return;

    const ContactPoint& cp0 = cps[0];
    const Scalar kEpsilon = Algebra::from_double(1e-5);

    tds::MultiBody<Algebra>* mb_a = cp0.multi_body_a;
    tds::MultiBody<Algebra>* mb_b = cp0.multi_body_b;

    const int n_a = mb_a->dof_qd();
    const int n_b = mb_b->dof_qd();
    const int n_ab = n_a + n_b;
    if (n_ab == 0) return;

    Vector3 vel_a, vel_b;
    Vector3 rel_pos_a, rel_pos_b;

    // joint torques to be applied (include DOFs for floating base)
    VectorX tau_a(mb_a->dof_qd()), tau_b(mb_b->dof_qd());
    Algebra::set_zero(tau_a);
    Algebra::set_zero(tau_b);

    for (const ContactPoint& cp : cps) {
      const Vector3& world_point_a = cp.world_point_on_a;
      const Vector3& world_point_b = cp.world_point_on_b;
      const Vector3& world_normal = -cp.world_normal_on_b;  // !!!
      Matrix3X jac_a =
          point_jacobian(*mb_a, mb_a->q(), cp.link_a, world_point_a, false);
      Matrix3X jac_b =
          point_jacobian(*mb_b, mb_b->q(), cp.link_b, world_point_b, false);

      // Algebra::print("jac_b", jac_b);
      // mb_b->print_state();
      // Matrix3X jac_a =
      //     point_jacobian_fd(*mb_a, mb_a->q(), cp.link_a, world_point_a);
      // Matrix3X jac_b_fd =
      //     point_jacobian_fd(*mb_b, mb_b->q(), cp.link_b, world_point_b);
      // Algebra::print("jac_b_fd", jac_b_fd);

      // jac_b = jac_b_fd;

      VectorX qd_a(mb_a->qd());
      VectorX qd_b(mb_b->qd());
      vel_a = jac_a * qd_a;
      vel_b = jac_b * qd_b;
      Vector3 rel_vel = vel_a - vel_b;
      // Algebra::print("rel_vel", rel_vel);

      // contact normal force
      Scalar normal_rel_vel = world_normal.dot(rel_vel);
      Scalar force_normal = compute_contact_force(-cp.distance, normal_rel_vel);
#ifdef DEBUG
      printf("Contact normal force magnitude: %.5f\n", force_normal);
#endif
      Vector3 force_vector = world_normal * force_normal;

      // only apply force if distance < 0
      Scalar collision = where_lt(cp.distance, Algebra::zero(), Algebra::one(),
                                  Algebra::zero());
      force_vector *= collision;

      tau_a += Algebra::mul_transpose(jac_a, force_vector);
      tau_b -= Algebra::mul_transpose(jac_b, force_vector);

      if (friction_model_ == FRICTION_NONE) {
        continue;
      }
      
      // unilateral friction force
      Vector3 lateral_rel_vel = rel_vel - normal_rel_vel * cp.world_normal_on_b;
      // Algebra::print("lateral_rel_vel", lateral_rel_vel);

      // to prevent division by zero in norm function
      lateral_rel_vel[2] += kEpsilon;
      Scalar lateral = Algebra::norm(lateral_rel_vel);
      // + Algebra::scalar_from_double(0.001);
      // printf("lateral_rel_vel.length(): %.6f\n",
      //        Algebra::getDouble(lateral));

      // Vector3 coulomb_fr_direction2 =
      // coulomb_fr_direction1.cross(cp.world_normal_on_b);

      // if lateral < Algebra::fraction(1, 1000) ...
      // Scalar fr_case = where_lt(lateral, Algebra::fraction(1, 1000),
      //                           Algebra::one(), Algebra::zero());
      Vector3 fr_direction1;
      // if constexpr (is_cppad_scalar<Scalar>::value) {
      if constexpr (true) {
        // use the negative lateral velocity and its orthogonal as friction
        // directions
        fr_direction1 = lateral_rel_vel * (Algebra::one() / lateral);
      } else {
        if (lateral < Algebra::fraction(1, 1000)) {
          Vector3 plane_fr_direction1, plane_fr_direction2;
          // use the plane space of the contact normal as friction directions
          MultiBodyConstraintSolver<Algebra>::plane_space(
              cp.world_normal_on_b, plane_fr_direction1, plane_fr_direction2);
          fr_direction1 = plane_fr_direction1;
        } else {
          // use the negative lateral velocity and its orthogonal as friction
          // directions
          fr_direction1 = lateral_rel_vel * (Algebra::one() / lateral);
        }
      }

      // if (lateral > Algebra::fraction(10000, 1)) {
      //   lateral_rel_vel.print("lateral_rel_vel");
      //   printf("lateral_rel_vel.length(): %.6f\n",
      //          Algebra::getDouble(lateral));
      //   rel_vel.print("rel_vel");
      //   cp.world_normal_on_b.print("cp.world_normal_on_b");
      //   // lateral = Algebra::fraction(10000, 1);
      // }

      Scalar friction = collision * compute_friction_force(
                                        force_normal, lateral, cp.friction);
      // if (friction > Algebra::fraction(10000, 1)) {
      // printf("friction: %.6f\n", Algebra::getDouble(friction));

      // printf("force_normal: %.6f\n",
      // Algebra::getDouble(force_normal)); printf("lateral: %.6f\n",
      // Algebra::getDouble(lateral)); friction =
      // Algebra::fraction(10000, 1);
      // }
      Vector3 friction_vector = fr_direction1 * friction;

      if constexpr (is_neural_algebra<Algebra>::value) {
        force_normal.assign("friction/fn");
        world_point_a[0].assign("friction/point.x");
        world_point_a[1].assign("friction/point.y");
        world_point_a[2].assign("friction/point.z");
        rel_vel[0].assign("friction/rel_vel.x");
        rel_vel[1].assign("friction/rel_vel.y");
        rel_vel[2].assign("friction/rel_vel.z");
        friction_vector[0].assign("friction/fr_vec.x");
        friction_vector[1].assign("friction/fr_vec.y");
        // friction_vector[2].assign("friction/fr_vec.z");
        friction_vector[0].evaluate();
        friction_vector[1].evaluate();
        // friction_vector[2].evaluate();
        // Algebra::print("friction_vector", friction_vector);
      }

      tau_a += Algebra::mul_transpose(jac_a, friction_vector);
      tau_b -= Algebra::mul_transpose(jac_b, friction_vector);

      // friction_vector = fr_direction2 * friction;
      // tau_a += Algebra::mul_transpose(jac_a, friction_vector);
      // tau_b -= Algebra::mul_transpose(jac_b, friction_vector);
    }
    // apply forces
    // Algebra::print("tau_b", tau_b);
    if (n_a > 0) {
      int tau_offset_a = 0;
      if (mb_a->is_floating()) {
        for (int i = 0; i < 6; ++i) {
          mb_a->base_applied_force()[i] += tau_a[i];
        }
        tau_offset_a = 6;
      }
      for (int i = 0; i < mb_a->dof_actuated(); ++i) {
        mb_a->tau(i) += tau_a[i + tau_offset_a];
      }
    }
    if (n_b > 0) {
      int tau_offset_b = 0;
      if (mb_b->is_floating()) {
        for (int i = 0; i < 6; ++i) {
          mb_b->base_applied_force()[i] += tau_b[i];
        }
        tau_offset_b = 6;
      }
      for (int i = 0; i < mb_b->dof_actuated(); ++i) {
        mb_b->tau(i) += tau_b[i + tau_offset_b];
      }
    }
  }
};

template <typename AlgebraFrom, typename AlgebraTo = AlgebraFrom>
static TINY_INLINE MultiBodyConstraintSolverSpring<AlgebraTo> clone(
    const MultiBodyConstraintSolverSpring<AlgebraFrom>& s) {
  return s.template clone<AlgebraTo>();
}
}  // namespace tds
