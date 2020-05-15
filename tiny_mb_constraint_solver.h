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

#ifndef TINY_MB_CONSTRAINT_SOLVER_H
#define TINY_MB_CONSTRAINT_SOLVER_H

#include "tiny_constraint_solver.h"
#include "tiny_multi_body.h"

template <typename TinyScalar, typename TinyConstants>
struct TinyContactPointMultiBody
    : public TinyContactPoint<TinyScalar, TinyConstants> {
  typedef ::TinyMultiBody<TinyScalar, TinyConstants> TinyMultiBody;
  TinyMultiBody* m_multi_body_a;
  TinyMultiBody* m_multi_body_b;
  TinyScalar m_restitution;
  TinyScalar m_friction;
  int m_link_a;
  int m_link_b;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyMultiBodyConstraintSolver {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyVectorX<TinyScalar, TinyConstants> TinyVectorX;
  typedef ::TinyMatrix3xX<TinyScalar, TinyConstants> TinyMatrix3xX;
  typedef ::TinyMatrixXxX<TinyScalar, TinyConstants> TinyMatrixXxX;
  typedef ::TinyMultiBody<TinyScalar, TinyConstants> TinyMultiBody;
  typedef ::TinyContactPointMultiBody<TinyScalar, TinyConstants>
      TinyContactPoint;

  /**
   * Whether this method requires an outer iteration loop (such as the
   * sequential impulse method).
   */
  bool needs_outer_iterations{false};

  int m_pgs_iterations{50};
  double m_least_squares_residual_threshold{0};
  std::vector<int> m_limit_dependency;

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
  void solve_pgs(const TinyMatrixXxX& A, const TinyVectorX& b, TinyVectorX& x,
                 int num_iterations, double least_squares_residual_threshold,
                 const TinyVectorX* lo = nullptr,
                 const TinyVectorX* hi = nullptr) const {
    assert(A.m_rows == b.m_size);
    assert(A.m_cols == x.m_size);

    TinyScalar delta;
    double least_squares_residual;
    for (int k = 0; k < num_iterations; ++k) {
      least_squares_residual = 0;
      for (int i = 0; i < A.m_rows; ++i) {
        delta = TinyConstants::zero();
        for (int j = 0; j < i; j++) delta += A(i, j) * x[j];
        for (int j = i + 1; j < A.m_rows; j++) delta += A(i, j) * x[j];

        TinyScalar a_diag = A(i, i);
        TinyScalar x_old = x[i];
        x[i] = (b[i] - delta) / a_diag;
        TinyScalar s = TinyConstants::one();
        if (!m_limit_dependency.empty() && m_limit_dependency[i] >= 0) {
          s = x[m_limit_dependency[i]];
          if (s < TinyConstants::zero()) {
            s = TinyConstants::one();
          }
        }

        if (lo && x[i] < (*lo)[i] * s) {
          x[i] = (*lo)[i] * s;
        }
        if (hi && x[i] > (*hi)[i] * s) {
          x[i] = (*hi)[i] * s;
        }
        TinyScalar diff = x[i] - x_old;
        //        least_squares_residual += TinyConstants::getDouble(diff *
        //        diff);
      }
      //      if (least_squares_residual < least_squares_residual_threshold) {
      //        printf("PGS converged: residual %f < %f at iteration %i /
      //        %i.\n",
      //               least_squares_residual, least_squares_residual_threshold,
      //               k + 1, num_iterations);
      //      }
    }
  }

  // Solve impulse-based collision response using MLCP formulation from
  // Jakub Stępień, PhD Thesis, 2013.
  //
  // Args:
  // cps: contact points with distances < 0
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

    TinyMatrixXxX mass_matrix_a(n_a, n_a);
    mb_a->mass_matrix(&mass_matrix_a);
    bool is_positive_definite_a = true;
    bool is_positive_definite_b = true;

    TinyMatrixXxX mass_matrix_a_inv(mass_matrix_a.m_rows, mass_matrix_a.m_cols);
    if (mass_matrix_a.m_cols * mass_matrix_a.m_rows > 0) {
      mb_a->submitProfileTiming("inverse_mass_matrix_a");
      is_positive_definite_a = mass_matrix_a.inversed(mass_matrix_a_inv);
      mb_a->submitProfileTiming("");
    }

    TinyMatrixXxX mass_matrix_b(n_b, n_b);
    mb_b->mass_matrix(&mass_matrix_b);
    TinyMatrixXxX mass_matrix_b_inv(mass_matrix_b.m_rows, mass_matrix_b.m_cols);
    if (mass_matrix_b.m_cols * mass_matrix_b.m_rows > 0) {
      mb_b->submitProfileTiming("inverse_mass_matrix_b");
      is_positive_definite_b = mass_matrix_b.inversed(mass_matrix_b_inv);
      mb_b->submitProfileTiming("");
    }
    if (!is_positive_definite_a) {
      printf("LCP: mass matrix a is not positive definite");
    }
    if (!is_positive_definite_b) {
      printf("LCP: mass matrix b is not positive definite");
    }
    assert(is_positive_definite_a);
    assert(is_positive_definite_b);

    TinyMatrixXxX mass_matrix_inv(n_ab, n_ab);
    mass_matrix_inv.set_zero();
    mass_matrix_inv.assign_matrix(0, 0, mass_matrix_a_inv);
    mass_matrix_inv.assign_matrix(n_a, n_a, mass_matrix_b_inv);
    // mass_matrix_inv.print("Mass matrix (a and b combined)");

    // Assemble constraint Jacobian J_C for a and b
    // The convention for constructing the constraint Jacobian is as follows:
    // For each contact point i the rows are as follows:
    //  i    is for the contact normal
    //  c+i  is for the friction direction towards the lateral velocity

    int num_friction_dir = 1;
    int dof_per_contact = 1 + num_friction_dir;

    TinyMatrixXxX jac_con(dof_per_contact * n_c, n_ab);
    jac_con.set_zero();
    TinyVectorX lcp_b(dof_per_contact * n_c);
    lcp_b.set_zero();

    TinyScalar erp =
        TinyConstants::fraction(1, 100);  // BAUMGARTE_ERROR_REDUCTION_PARAMETER
    TinyScalar cfm =
        TinyConstants::fraction(1, 100000);  // Constraint Force Mixing

    for (int i = 0; i < n_c; ++i) {
      const TinyContactPoint& cp = cps[i];
      // all contact points are already assumed to have distance < 0
      if (cp.m_distance > TinyConstants::zero()) continue;

      const TinyVector3& world_point_a = cp.m_world_point_on_a;
      TinyMatrix3xX jac_a =
          cp.m_multi_body_a->point_jacobian(cp.m_link_a, world_point_a);
      TinyVectorX jac_a_i = jac_a.mul_transpose(cp.m_world_normal_on_b);
      jac_con.assign_vector_horizontal(i, 0, jac_a_i);

      const TinyVector3& world_point_b = cp.m_world_point_on_b;

      TinyMatrix3xX jac_b =
          cp.m_multi_body_b->point_jacobian(cp.m_link_b, world_point_b);
      // jac_b.print("jac_b");
      TinyVectorX jac_b_i = jac_b.mul_transpose(cp.m_world_normal_on_b);
      // jac_b_i.print("jac_b_i");

      std::vector<TinyScalar> qd_empty;
      int szb = cp.m_multi_body_b->m_qd.size();
      qd_empty.resize(szb, TinyConstants::zero());
      std::vector<TinyScalar> tau_jac;
      tau_jac.resize(szb);
      for (int i = 0; i < szb; i++) {
        tau_jac[i] = -jac_b_i[i];
      }

      // compare with unit impulse method
      // std::vector<TinyScalar> qdd_delta_unit_impulse;
      // qdd_delta_unit_impulse.resize(szb);
      // cp.m_multi_body_b->forward_dynamics(
      //    cp.m_multi_body_b->m_q, qd_empty, tau_jac, qdd_delta_unit_impulse,
      //    TinyConstants::fraction(100, 10));  // TinyConstants::zero());

      jac_con.assign_vector_horizontal(i, n_a, jac_b_i);

      TinyVectorX qd_a(cp.m_multi_body_a->m_qd);
      TinyVectorX qd_b(cp.m_multi_body_b->m_qd);
      // qd_a.print("qd_a");
      // qd_b.print("qd_b");
      TinyVector3 vel_a = jac_a * qd_a;
      TinyVector3 vel_b = jac_b * qd_b;
      TinyVector3 rel_vel = vel_a - vel_b;
      //      rel_vel.print("rel_vel");
      TinyScalar normal_rel_vel = cp.m_world_normal_on_b.dot(rel_vel);
      // printf("normal_rel_vel: %.4f\n", normal_rel_vel);

      // Baumgarte stabilization
      TinyScalar baumgarte_rel_vel = erp * cp.m_distance / dt;

      lcp_b[i] = -(TinyConstants::one() + cp.m_restitution) * normal_rel_vel -
                 baumgarte_rel_vel;

      // friction direction
      TinyVector3 lateral_rel_vel =
          rel_vel - normal_rel_vel * cp.m_world_normal_on_b;
      //      lateral_rel_vel.print("lateral_rel_vel");
      const TinyScalar lateral = lateral_rel_vel.length();
      //      printf("lateral_rel_vel.length(): %.6f\n", lateral);

      TinyVector3 fr_direction1, fr_direction2;
      //      cp.m_world_normal_on_b.print("contact normal");
      //      fflush(stdout);
      if (lateral < TinyConstants::fraction(1, 10000)) {
        // use the plane space of the contact normal as friction directions
        cp.m_world_normal_on_b.plane_space(fr_direction1, fr_direction2);
      } else {
        // use the negative lateral velocity and its orthogonal as friction
        // directions
        fr_direction1 = lateral_rel_vel * (TinyConstants::one() / lateral);
        fr_direction2 = fr_direction1.cross(cp.m_world_normal_on_b);
      }

      TinyScalar l1 = fr_direction1.dot(rel_vel);
      lcp_b[n_c + i] = -l1;
      // printf("l1=%f\n", l1);
      if (num_friction_dir > 1) {
        TinyScalar l2 = fr_direction2.dot(rel_vel);
        lcp_b[2 * n_c + i] = -l2;
        // printf("l2=%f\n", l2);
      }

      //      fr_direction1.print("friction direction 1");
      //      fr_direction2.print("friction direction 2");
      TinyVectorX jac_a_i_fr1 = jac_a.mul_transpose(fr_direction1);
      jac_con.assign_vector_horizontal(n_c + i, 0, jac_a_i_fr1);
      TinyVectorX jac_b_i_fr1 = jac_b.mul_transpose(fr_direction1);
      jac_con.assign_vector_horizontal(n_c + i, n_a, jac_b_i_fr1);
      if (num_friction_dir > 1) {
        TinyVectorX jac_a_i_fr2 = jac_a.mul_transpose(fr_direction2);
        jac_con.assign_vector_horizontal(2 * n_c + i, 0, jac_a_i_fr2);
        TinyVectorX jac_b_i_fr2 = jac_b.mul_transpose(fr_direction2);
        jac_con.assign_vector_horizontal(2 * n_c + i, n_a, jac_b_i_fr2);
      }
    }

    //    jac_con.print("Constraint Jacobian");

    TinyMatrixXxX jac_con_t = jac_con.transpose();
    TinyMatrixXxX lcp_A;

    {
      mb_b->submitProfileTiming("lcpA");
      lcp_A = jac_con * mass_matrix_inv * jac_con_t;
      mb_b->submitProfileTiming("");
    }

    // apply CFM
    // This is _necessary_ for fixed base systems where the degrees of freedom
    // would otherwise not allow for the lateral friction directions, leading
    // to zero constraint Jacobian rows and eventually zeros on the diagonal
    // of the A matrix.
    {
      mb_b->submitProfileTiming("cfm");
      for (int i = 0; i < dof_per_contact * n_c; ++i) {
        lcp_A(i, i) += cfm;
      }
      mb_b->submitProfileTiming("");
    }

    //    lcp_A.print("MLCP A");
    //    lcp_b.print("MLCP b");

    TinyVectorX lcp_p(dof_per_contact * n_c);
    lcp_p.set_zero();
    TinyVectorX con_lo(dof_per_contact * n_c);
    TinyVectorX con_hi(dof_per_contact * n_c);
    con_lo.set_zero();
    m_limit_dependency.reserve(dof_per_contact * n_c);
    m_limit_dependency.resize(dof_per_contact * n_c);
    for (int i = 0; i < n_c; ++i) {
      m_limit_dependency[i] = -1;
      con_hi[i] = TinyConstants::fraction(1000, 1);
      // ||friction impulse|| <= mu * ||normal impulse||
      con_hi[n_c + i] = cps[i].m_friction;
      con_lo[n_c + i] = -cps[i].m_friction;
      m_limit_dependency[n_c + i] = i;
      if (num_friction_dir > 1) {
        con_hi[2 * n_c + i] = cps[i].m_friction;
        con_lo[2 * n_c + i] = -cps[i].m_friction;
        m_limit_dependency[2 * n_c + i] = i;
      }
    }
    {
      //      fflush(stdout);
      mb_b->submitProfileTiming("solve_pgs");
      solve_pgs(lcp_A, lcp_b, lcp_p, m_pgs_iterations,
                m_least_squares_residual_threshold, &con_lo, &con_hi);
      mb_b->submitProfileTiming("");
    }
    //    lcp_p.print("MLCP impulse solution");

    if (n_a > 0) {
      TinyVectorX p_a = lcp_p.segment(0, n_c);
      TinyMatrixXxX jac_con_a = jac_con.block(0, 0, n_c, n_a);
      TinyVectorX delta_qd_a = mass_matrix_a_inv * jac_con_a.mul_transpose(p_a);
      // add friction impulse
      TinyVectorX p_a_fr = lcp_p.segment(0, n_c);
      TinyMatrixXxX jac_con_a_fr = jac_con.block(n_c, 0, n_c, n_a);
      //      p_a_fr.print("Friction impulse a");
      delta_qd_a += mass_matrix_a_inv * jac_con_a_fr.mul_transpose(p_a_fr);
      // delta_qd_a.print("delta qd for multi body a:");
      for (int i = 0; i < n_a; ++i) {
        mb_a->m_qd[i] += delta_qd_a[i];
      }
    }
    if (n_b > 0) {
      TinyVectorX p_b = lcp_p.segment(0, n_c);
      TinyMatrixXxX jac_con_b = jac_con.block(0, n_a, n_c, n_b);

      // p_b[0] = 1;
      TinyVectorX delta_qd_b = mass_matrix_b_inv * jac_con_b.mul_transpose(p_b);

      // add friction impulse
      if (1) {
        // friction direction 1
        TinyVectorX p_b_fr = lcp_p.segment(n_c, n_c);
        TinyMatrixXxX jac_con_b_fr = jac_con.block(n_c, n_a, n_c, n_b);
        //        p_b_fr.print("Friction 1 impulse b");
        // TinyMatrixXxX imp = jac_con_b_fr * mass_matrix_b_inv;
        // TinyVectorX fr_qd =
        //    imp.mul_transpose(p_b_fr);
        TinyVectorX fr_qd =
            mass_matrix_b_inv * jac_con_b_fr.mul_transpose(p_b_fr);
        //        fr_qd.print("Friction 1 contribution on q delta for b");
        delta_qd_b += fr_qd;
      }
      if (num_friction_dir > 1) {
        // friction direction 2
        TinyVectorX p_b_fr = lcp_p.segment(2 * n_c, n_c);
        TinyMatrixXxX jac_con_b_fr = jac_con.block(2 * n_c, n_a, n_c, n_b);
        //        p_b_fr.print("Friction 2 impulse b");
        // TinyMatrixXxX imp = mass_matrix_b_inv* jac_con_b_fr;
        // TinyVectorX fr_qd =
        //    imp.mul_transpose(p_b_fr);
        TinyVectorX fr_qd =
            mass_matrix_b_inv * jac_con_b_fr.mul_transpose(p_b_fr);
        //        fr_qd.print("Friction 2 contribution on q delta for b");
        delta_qd_b += fr_qd;
      }

      for (int i = 0; i < n_b; ++i) {
        mb_b->m_qd[i] -= delta_qd_b[i];
      }
    }
  }
};

#endif  // TINY_MB_CONSTRAINT_SOLVER_H
