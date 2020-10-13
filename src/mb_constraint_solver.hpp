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
#include "dynamics/jacobian.hpp"
#include "dynamics/mass_matrix.hpp"
#include "multi_body.hpp"

namespace tds {
template <typename Algebra>
struct MultiBodyContactPoint : public ContactPoint<Algebra> {
  typedef tds::MultiBody<Algebra> MultiBody;
  using Scalar = typename Algebra::Scalar;

  MultiBody* multi_body_a;
  MultiBody* multi_body_b;
  Scalar restitution;
  Scalar friction;
  int link_a;
  int link_b;
};

template <typename Algebra>
class MultiBodyConstraintSolver {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using VectorX = typename Algebra::VectorX;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix3X = typename Algebra::Matrix3X;
  using MatrixX = typename Algebra::MatrixX;

 public:
  typedef tds::MultiBody<Algebra> MultiBody;
  typedef tds::Geometry<Algebra> Geometry;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MultiBodyContactPoint<Algebra> ContactPoint;

 private:
  SubmitProfileTiming profile_timing_func_{nullptr};

 public:
  int pgs_iterations_{50};
  double least_squares_residual_threshold_{0};
  std::vector<int> limit_dependency_;

  // Error reduction parameter in Baumgarte stabilization
  Scalar erp_{Algebra::fraction(1, 100)};
  // Constraint Force Mixing
  Scalar cfm_{Algebra::fraction(1, 100000)};

  // Number of friction force directions
  int num_friction_dir_{1};

  virtual ~MultiBodyConstraintSolver() = default;

 private:
  /**
   * Projected Gauss-Seidel solver for a MLCP defined by coefficient matrix A
   * and vector b.
   *
   *   Ax + b >= 0
   *   s.t. x >= lo and x <= hi and xT(Ax + b) = 0
   *
   * where lo and hi are the respective lower and upper bounds on x.
   *
   * Reference: Jakub Stępień, PhD Thesis, 2013, p. 91.
   * Code reference: Bullet physics engine, Erwin Coumans.
   * https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h#L35
   */
  void solve_pgs(const MatrixX& A, const VectorX& b, VectorX& x,
                 int num_iterations, double least_squares_residual_threshold,
                 const VectorX* lo = nullptr,
                 const VectorX* hi = nullptr) const {
    assert(Algebra::num_rows(A) == Algebra::size(b));
    assert(Algebra::num_cols(A) == Algebra::size(x));

    Scalar delta;
    double least_squares_residual;
    for (int k = 0; k < num_iterations; ++k) {
      least_squares_residual = 0;
      for (int i = 0; i < Algebra::num_rows(A); ++i) {
        delta = Algebra::zero();
        for (int j = 0; j < i; j++) delta += A(i, j) * x[j];
        for (int j = i + 1; j < Algebra::num_rows(A); j++)
          delta += A(i, j) * x[j];

        Scalar a_diag = A(i, i);
        Scalar x_old = x[i];
        x[i] = (b[i] - delta) / a_diag;
        Scalar s = Algebra::one();
        if (!limit_dependency_.empty() && limit_dependency_[i] >= 0) {
          s = x[limit_dependency_[i]];
          if (s < Algebra::zero()) {
            s = Algebra::one();
          }
        }

        if (lo && x[i] < (*lo)[i] * s) {
          x[i] = (*lo)[i] * s;
        }
        if (hi && x[i] > (*hi)[i] * s) {
          x[i] = (*hi)[i] * s;
        }
        // Scalar diff = x[i] - x_old;
        // least_squares_residual += Algebra::getDouble(diff * diff);
      }
      // if (least_squares_residual < least_squares_residual_threshold) {
      //   printf("PGS converged: residual %f < %f at iteration %i / % i.\n ",
      //          least_squares_residual, least_squares_residual_threshold, k +
      //          1, num_iterations);
      // }
    }
  }

 protected:
  /**
   * Whether this method requires an outer iteration loop (such as the
   * sequential impulse method).
   */
  bool needs_outer_iterations_{false};

 public:
  TINY_INLINE bool needs_outer_iterations() const {
    return needs_outer_iterations_;
  }

  // Solve impulse-based collision response using MLCP formulation from
  // Jakub Stępień, PhD Thesis, 2013.
  //
  // Args:
  // cps: contact points with distances < 0
  // dt : delta time (in seconds)
  virtual void resolve_collision(const std::vector<ContactPoint>& cps,
                                 const Scalar& dt) {
    if (cps.empty()) return;
    const int n_c = static_cast<int>(cps.size());

    const ContactPoint& cp0 = cps[0];

    MultiBody* mb_a = cp0.multi_body_a;
    MultiBody* mb_b = cp0.multi_body_b;

    const int n_a = mb_a->dof_qd();
    const int n_b = mb_b->dof_qd();
    const int n_ab = n_a + n_b;
    if (n_ab == 0) return;

    MatrixX mass_matrix_a(n_a, n_a);
    mass_matrix(*mb_a, &mass_matrix_a);
    bool is_positive_definite_a = true;
    bool is_positive_definite_b = true;

    MatrixX mass_matrix_a_inv(Algebra::num_rows(mass_matrix_a),
                              Algebra::num_cols(mass_matrix_a));
    if (Algebra::num_cols(mass_matrix_a) * Algebra::num_rows(mass_matrix_a) >
        0) {
      submit_profile_timing("inverse_mass_matrix_a");
      is_positive_definite_a =
          Algebra::symmetric_inverse(mass_matrix_a, mass_matrix_a_inv);
      submit_profile_timing("");
    }

    MatrixX mass_matrix_b(n_b, n_b);
    mass_matrix(*mb_b, &mass_matrix_b);
    MatrixX mass_matrix_b_inv(Algebra::num_rows(mass_matrix_b),
                              Algebra::num_cols(mass_matrix_b));
    if (Algebra::num_cols(mass_matrix_b) * Algebra::num_rows(mass_matrix_b) >
        0) {
      submit_profile_timing("inverse_mass_matrix_b");
      is_positive_definite_b =
          Algebra::symmetric_inverse(mass_matrix_b, mass_matrix_b_inv);
      submit_profile_timing("");
    }
    if (!is_positive_definite_a) {
      printf("LCP: mass matrix a is not positive definite\n");
    }
    if (!is_positive_definite_b) {
      printf("LCP: mass matrix b is not positive definite\n");
    }
    assert(is_positive_definite_a);
    assert(is_positive_definite_b);

    MatrixX mass_matrix_inv(n_ab, n_ab);
    Algebra::set_zero(mass_matrix_inv);
    Algebra::assign_block(mass_matrix_inv, mass_matrix_a_inv, 0, 0);
    Algebra::assign_block(mass_matrix_inv, mass_matrix_b_inv, n_a, n_a);
    // mass_matrix_inv.print("Mass matrix (a and b combined)");

    // Assemble constraint Jacobian J_C for a and b
    // The convention for constructing the constraint Jacobian is as follows:
    // For each contact point i the rows are as follows:
    //  i    is for the contact normal
    //  c+i  is for the friction direction towards the lateral velocity

    int dof_per_contact = 1 + num_friction_dir_;

    MatrixX jac_con(dof_per_contact * n_c, n_ab);
    Algebra::set_zero(jac_con);
    VectorX lcp_b(dof_per_contact * n_c);
    Algebra::set_zero(lcp_b);

    for (int i = 0; i < n_c; ++i) {
      const ContactPoint& cp = cps[i];
      // all contact points are already assumed to have distance < 0
      if (cp.distance > Algebra::zero()) continue;

      const Vector3& world_point_a = cp.world_point_on_a;
      Matrix3X jac_a = point_jacobian(*mb_a, cp.link_a, world_point_a);
      VectorX jac_a_i = Algebra::mul_transpose(jac_a, cp.world_normal_on_b);
      Algebra::assign_horizontal(jac_con, jac_a_i, i, 0);

      const Vector3& world_point_b = cp.world_point_on_b;

      Matrix3X jac_b = point_jacobian(*mb_b, cp.link_b, world_point_b);
      // Matrix3X jac_b =
      //     point_jacobian_fd(*mb_b, mb_b->m_q, cp.link_b,
      //     world_point_b);
      // jac_b.print("jac_b");
      VectorX jac_b_i = Algebra::mul_transpose(jac_b, cp.world_normal_on_b);
      // jac_b_i.print("jac_b_i");

      std::vector<Scalar> qd_empty;
      int szb = cp.multi_body_b->dof_qd();
      qd_empty.resize(szb, Algebra::zero());
      std::vector<Scalar> tau_jac;
      tau_jac.resize(szb);
      for (int i = 0; i < szb; i++) {
        tau_jac[i] = -jac_b_i[i];
      }

      // compare with unit impulse method
      // std::vector<Scalar> qdd_delta_unit_impulse;
      // qdd_delta_unit_impulse.resize(szb);
      // cp.multi_body_b->forward_dynamics(
      //    cp.multi_body_b->m_q, qd_empty, tau_jac, qdd_delta_unit_impulse,
      //    Algebra::fraction(100, 10));  // Algebra::zero());

      Algebra::assign_horizontal(jac_con, jac_b_i, i, n_a);

      VectorX qd_a(cp.multi_body_a->qd());
      VectorX qd_b(cp.multi_body_b->qd());
      // qd_a.print("qd_a");
      // qd_b.print("qd_b");
      Vector3 vel_a = jac_a * qd_a;
      Vector3 vel_b = jac_b * qd_b;
      Vector3 rel_vel = vel_a - vel_b;
      //      rel_vel.print("rel_vel");
      Scalar normal_rel_vel = Algebra::dot(cp.world_normal_on_b, rel_vel);
      // printf("normal_rel_vel: %.4f\n", normal_rel_vel);

      // Baumgarte stabilization
      Scalar baumgarte_rel_vel = erp_ * cp.distance / dt;

      lcp_b[i] = -(Algebra::one() + cp.restitution) * normal_rel_vel -
                 baumgarte_rel_vel;

      // friction direction
      Vector3 lateral_rel_vel = rel_vel - normal_rel_vel * cp.world_normal_on_b;
      // lateral_rel_vel.print("lateral_rel_vel");
      const Scalar lateral = Algebra::norm(lateral_rel_vel);
      // printf("Algebra::norm(lateral_rel_vel): %.6f\n",
      //        Algebra::getDouble(lateral));

      Vector3 fr_direction1, fr_direction2;
      //      cp.world_normal_on_b.print("contact normal");
      //      fflush(stdout);
      if (lateral < Algebra::fraction(1, 10000)) {
        // use the plane space of the contact normal as friction directions
        plane_space(cp.world_normal_on_b, fr_direction1, fr_direction2);
      } else {
        // use the negative lateral velocity and its orthogonal as friction
        // directions
        fr_direction1 = lateral_rel_vel * (Algebra::one() / lateral);
        fr_direction2 = Algebra::cross(fr_direction1, cp.world_normal_on_b);
      }

      Scalar l1 = Algebra::dot(fr_direction1, rel_vel);
      lcp_b[n_c + i] = -l1;
      // printf("l1=%f\n", l1);
      if (num_friction_dir_ > 1) {
        Scalar l2 = Algebra::dot(fr_direction2, rel_vel);
        lcp_b[2 * n_c + i] = -l2;
        // printf("l2=%f\n", l2);
      }

      //      fr_direction1.print("friction direction 1");
      //      fr_direction2.print("friction direction 2");
      VectorX jac_a_i_fr1 = Algebra::mul_transpose(jac_a, fr_direction1);
      Algebra::assign_horizontal(jac_con, jac_a_i_fr1, n_c + i, 0);
      VectorX jac_b_i_fr1 = Algebra::mul_transpose(jac_b, fr_direction1);
      Algebra::assign_horizontal(jac_con, jac_b_i_fr1, n_c + i, n_a);
      if (num_friction_dir_ > 1) {
        VectorX jac_a_i_fr2 = Algebra::mul_transpose(jac_a, fr_direction2);
        Algebra::assign_horizontal(jac_con, jac_a_i_fr2, 2 * n_c + i, 0);
        VectorX jac_b_i_fr2 = Algebra::mul_transpose(jac_b, fr_direction2);
        Algebra::assign_horizontal(jac_con, jac_b_i_fr2, 2 * n_c + i, n_a);
      }
    }

    // jac_con.print("Constraint Jacobian");

    MatrixX jac_con_t = Algebra::transpose(jac_con);
    MatrixX lcp_A;

    {
      submit_profile_timing("lcpA");
      lcp_A = jac_con * mass_matrix_inv * jac_con_t;
      submit_profile_timing("");
    }

    // apply CFM
    // This is _necessary_ for fixed base systems where the degrees of freedom
    // would otherwise not allow for the lateral friction directions, leading
    // to zero constraint Jacobian rows and eventually zeros on the diagonal
    // of the A matrix.
    {
      submit_profile_timing("cfm_");
      for (int i = 0; i < dof_per_contact * n_c; ++i) {
        lcp_A(i, i) += cfm_;
      }
      submit_profile_timing("");
    }

    //    lcp_A.print("MLCP A");
    //    lcp_b.print("MLCP b");

    VectorX lcp_p(dof_per_contact * n_c);
    Algebra::set_zero(lcp_p);
    VectorX con_lo(dof_per_contact * n_c);
    VectorX con_hi(dof_per_contact * n_c);
    Algebra::set_zero(con_lo);
    limit_dependency_.reserve(dof_per_contact * n_c);
    limit_dependency_.resize(dof_per_contact * n_c);
    for (int i = 0; i < n_c; ++i) {
      limit_dependency_[i] = -1;
      con_hi[i] = Algebra::fraction(100000, 1);
      // ||friction impulse|| <= mu * ||normal impulse||
      con_hi[n_c + i] = cps[i].friction;
      con_lo[n_c + i] = -cps[i].friction;
      limit_dependency_[n_c + i] = i;
      if (num_friction_dir_ > 1) {
        con_hi[2 * n_c + i] = cps[i].friction;
        con_lo[2 * n_c + i] = -cps[i].friction;
        limit_dependency_[2 * n_c + i] = i;
      }
    }
    {
      //      fflush(stdout);
      submit_profile_timing("solve_pgs");
      solve_pgs(lcp_A, lcp_b, lcp_p, pgs_iterations_,
                least_squares_residual_threshold_, &con_lo, &con_hi);
      submit_profile_timing("");
    }
    //    lcp_p.print("MLCP impulse solution");

    if (n_a > 0) {
      VectorX p_a = Algebra::segment(lcp_p, 0, n_c);
      MatrixX jac_con_a = Algebra::block(jac_con, 0, 0, n_c, n_a);
      VectorX delta_qd_a =
          mass_matrix_a_inv * Algebra::mul_transpose(jac_con_a, p_a);
      // add friction impulse
      VectorX p_a_fr = Algebra::segment(lcp_p, 0, n_c);
      MatrixX jac_con_a_fr = Algebra::block(jac_con, n_c, 0, n_c, n_a);
      //      p_a_fr.print("Friction impulse a");
      delta_qd_a +=
          mass_matrix_a_inv * Algebra::mul_transpose(jac_con_a_fr, p_a_fr);
      // delta_qd_a.print("delta qd for multi body a:");
      for (int i = 0; i < n_a; ++i) {
        mb_a->qd(i) += delta_qd_a[i];
      }
    }
    if (n_b > 0) {
      VectorX p_b = Algebra::segment(lcp_p, 0, n_c);
      MatrixX jac_con_b = Algebra::block(jac_con, 0, n_a, n_c, n_b);

      // p_b[0] = 1;
      VectorX delta_qd_b =
          mass_matrix_b_inv * Algebra::mul_transpose(jac_con_b, p_b);

      // add friction impulse
      if (1) {
        // friction direction 1
        VectorX p_b_fr = Algebra::segment(lcp_p, n_c, n_c);
        MatrixX jac_con_b_fr = Algebra::block(jac_con, n_c, n_a, n_c, n_b);
        //        p_b_fr.print("Friction 1 impulse b");
        // MatrixX imp = jac_con_b_fr * mass_matrix_b_inv;
        // VectorX fr_qd =
        //    Algebra::mul_transpose(imp, p_b_fr);
        VectorX fr_qd =
            mass_matrix_b_inv * Algebra::mul_transpose(jac_con_b_fr, p_b_fr);
        // fr_qd.print("Friction 1 contribution on q delta for b");
        delta_qd_b += fr_qd;
      }
      if (num_friction_dir_ > 1) {
        // friction direction 2
        VectorX p_b_fr = Algebra::segment(lcp_p, 2 * n_c, n_c);
        MatrixX jac_con_b_fr = Algebra::block(jac_con, 2 * n_c, n_a, n_c, n_b);
        //        p_b_fr.print("Friction 2 impulse b");
        // MatrixX imp = mass_matrix_b_inv* jac_con_b_fr;
        // VectorX fr_qd =
        //    Algebra::mul_transpose(imp, p_b_fr);
        VectorX fr_qd =
            mass_matrix_b_inv * Algebra::mul_transpose(jac_con_b_fr, p_b_fr);
        //        fr_qd.print("Friction 2 contribution on q delta for b");
        delta_qd_b -= fr_qd;
      }

      for (int i = 0; i < n_b; ++i) {
        mb_b->qd(i) -= delta_qd_b[i];
      }
    }
  }

 private:
  /**
   * Treat this vector as normal vector of a plane and compute two
   * orthogonal direction vectors of that plane.
   * p and q will be unit vectors, the normal vector does not need to be unit
   * length.
   */
  inline void plane_space(const Vector3& n, Vector3& p, Vector3& q) const {
    if (n[2] * n[2] > Algebra::half()) {
      // choose p in y-z plane
      Scalar a = n[1] * n[1] + n[2] * n[2];
      Scalar k = Algebra::sqrt(a);
      p[0] = Algebra::zero();
      p[1] = -n[2] * k;
      p[2] = n[1] * k;
      // set q = n x p
      q[0] = a * k;
      q[1] = -n[0] * p[2];
      q[2] = n[0] * p[1];
    } else {
      // choose p in x-y plane
      Scalar a = n[0] * n[0] + n[1] * n[1];
      Scalar k = Algebra::sqrt(a);
      p[0] = -n[1] * k;
      p[1] = n[0] * k;
      p[2] = Algebra::zero();
      // set q = n x p
      q[0] = -n[2] * p[1];
      q[1] = n[2] * p[0];
      q[2] = a * k;
    }
  }

  TINY_INLINE void submit_profile_timing(const std::string& name) const {
    if (profile_timing_func_) {
      profile_timing_func_(name);
    }
  }
};
}  // namespace tds
