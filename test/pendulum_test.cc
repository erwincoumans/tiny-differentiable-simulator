// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "examples/pendulum.h"
#include "fix64_scalar.h"
#include "testing/base/public/gmock.h"
#include "testing/base/public/gunit.h"
#include "tiny_double_utils.h"
#include "tiny_dual_double_utils.h"
#include "tiny_multi_body.h"
#include "tiny_spatial_transform.h"
#include "tiny_world.h"

namespace {

class TinyRigidBodyTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

template <typename TinyScalar, typename TinyConstants>
void check_forward_kinematics(double tolerance = 1e-3) {
  std::vector<TinySpatialTransform<TinyScalar, TinyConstants>> links_X_world;
  TinyWorld<TinyScalar, TinyConstants> world;
  TinyMultiBody<TinyScalar, TinyConstants>* mb = world.create_multi_body();
  init_compound_pendulum<TinyScalar, TinyConstants>(*mb, world, 5);
  std::vector<TinyScalar> q(mb->dof(), TinyConstants::zero());
  std::vector<TinyScalar> qd(mb->dof_qd(), TinyConstants::zero());

  // add some "randomness" to the joint angles
  q[1] = TinyConstants::fraction(4, 10);
  q[3] = -TinyConstants::fraction(8, 10);

  TinySpatialTransform<TinyScalar, TinyConstants> base_X_world;
  mb->forward_kinematics_q(q, &base_X_world, &links_X_world);
  mb->forward_kinematics(q, qd);
  for (int i = 0; i < mb->m_dof; ++i) {
    const auto& x1 = mb->m_links[i].m_X_world;
    const auto& x2 = links_X_world[i];
    auto diff_t = (x1.m_translation - x2.m_translation).length();
    EXPECT_NEAR(abs(TinyConstants::getDouble(diff_t)), 0., tolerance);
    auto diff_r0 = (x1.m_rotation[0] - x2.m_rotation[0]).length();
    EXPECT_NEAR(abs(TinyConstants::getDouble(diff_r0)), 0., tolerance);
    auto diff_r1 = (x1.m_rotation[1] - x2.m_rotation[1]).length();
    EXPECT_NEAR(abs(TinyConstants::getDouble(diff_r1)), 0., tolerance);
    auto diff_r2 = (x1.m_rotation[2] - x2.m_rotation[2]).length();
    EXPECT_NEAR(abs(TinyConstants::getDouble(diff_r2)), 0., tolerance);
  }
}

template <typename TinyScalar, typename TinyConstants>
void compare_analytic_double_pendulum(double tolerance = 1e-3) {
  TinyScalar dt =
      TinyConstants::fraction(1, 2400);  // XXX this time step is VERY small
  int steps = 15;
  std::vector<TinyScalar> analytic_q1s, analytic_q2s, analytic_qd1s,
      analytic_qd2s;
  std::vector<TinyScalar> our_q1s, our_q2s, our_qd1s, our_qd2s;

  {
    // Compute trajectory from analytic model
    // initial conditions
    TinyScalar q1 = TinyConstants::zero(), q2 = TinyConstants::zero();
    TinyScalar qd1 = TinyConstants::zero(), qd2 = TinyConstants::zero();
    // TODO(ericheiden): why is gravity not negative?
    TinyScalar g = TinyConstants::fraction(981, 100);
    TinyScalar m1 = TinyConstants::one(), m2 = TinyConstants::one();
    TinyScalar l1 = TinyConstants::half(), l2 = TinyConstants::half();

    // compute trajectory
    double_pendulum_trajectory<TinyScalar, TinyConstants>(
        l1, l2, m1, m2, q1, q2, qd1, qd2, g, dt, steps, &analytic_q1s,
        &analytic_q2s, &analytic_qd1s, &analytic_qd2s);
  }
  {
    // Compute trajectory from our multi-body simulation.
    TinyWorld<TinyScalar, TinyConstants> world;
    TinyMultiBody<TinyScalar, TinyConstants>* mb = world.create_multi_body();
    init_compound_pendulum<TinyScalar, TinyConstants>(*mb, world);

    // sanity check
    EXPECT_EQ(mb->m_links.size(), 2);

    std::vector<TinyScalar> q(mb->dof(), TinyConstants::zero());
    std::vector<TinyScalar> qd(mb->dof_qd(), TinyConstants::zero());
    std::vector<TinyScalar> tau(mb->dof_qd(), TinyConstants::zero());
    std::vector<TinyScalar> qdd(mb->dof_qd(), TinyConstants::zero());

    our_q1s.reserve(steps);
    our_q2s.reserve(steps);
    our_qd1s.reserve(steps);
    our_qd2s.reserve(steps);
    TinyVector3<double, DoubleUtils> gravity(0., 0., -9.81);

    for (int t = 0; t < steps; ++t) {
      our_q1s[t] = q[0];
      our_q2s[t] = q[1];
      our_qd1s[t] = qd[0];
      our_qd2s[t] = qd[1];
      mb->forward_kinematics(q, qd);
      world.step(dt);
      mb->forward_dynamics(q, qd, tau, gravity, qdd);
      mb->integrate(q, qd, qdd, dt);
    }
  }

  // Compare trajectories.
  for (int t = 0; t < steps; ++t) {
    printf("### Testing at time step %i / %i ###\n", t + 1, steps);
    EXPECT_NEAR(TinyConstants::getDouble(analytic_q1s[t]),
                TinyConstants::getDouble(our_q1s[t]), tolerance);
    EXPECT_NEAR(TinyConstants::getDouble(analytic_q2s[t]),
                TinyConstants::getDouble(our_q2s[t]), tolerance);
    EXPECT_NEAR(TinyConstants::getDouble(analytic_qd1s[t]),
                TinyConstants::getDouble(our_qd1s[t]), tolerance);
    EXPECT_NEAR(TinyConstants::getDouble(analytic_qd2s[t]),
                TinyConstants::getDouble(our_qd2s[t]), tolerance);
  }
}

template <typename TinyScalar, typename TinyConstants>
void check_jacobian(bool floating_base = false, double tolerance = 1e-3,
                    int num_links = 5, int link_index = 4) {
  TinyWorld<TinyScalar, TinyConstants> world;
  TinyMultiBody<TinyScalar, TinyConstants>* mb = world.create_multi_body();

  init_compound_pendulum<TinyScalar, TinyConstants>(*mb, world, num_links);
  mb->m_isFloating = floating_base;
  mb->initialize();

  std::vector<TinyScalar> q(mb->dof(), TinyConstants::zero());
  std::vector<TinyScalar> qd(mb->dof_qd(), TinyConstants::zero());

  // add some "randomness" to the joint angles
  q[mb->dof() - 1] = TinyConstants::fraction(4, 10);
  q[mb->dof() - 3] = -TinyConstants::fraction(8, 10);
  if (floating_base) {
    // set "random" base transform
    q[0] = -TinyConstants::fraction(13, 29);
    q[1] = TinyConstants::fraction(8, 3);
    q[2] = TinyConstants::fraction(19, 5);
    q[4] = -TinyConstants::fraction(7, 3);
    q[5] = -TinyConstants::fraction(19, 5);
    q[6] = TinyConstants::fraction(42, 11);
  }

  const TinyVector3<TinyScalar, TinyConstants> point(
      TinyConstants::fraction(1, 3), TinyConstants::fraction(21, 5),
      -TinyConstants::fraction(13, 3));
  point.print("point");

  auto jac_const = mb->point_jacobian(q, link_index, point);
  jac_const.print("jac_const");
  auto jac_fd = mb->point_jacobian_fd(q, link_index, point,
                                      TinyConstants::fraction(1, 10000));
  jac_fd.print("jac_fd");

  mb->m_q = q;
  auto jac_reuse_fk = mb->point_jacobian(link_index, point);
  jac_reuse_fk.print("jac_reuse_fk");

  point.print("point");

  auto point_world = mb->body_to_world(link_index, point);
  point_world.print("point_world");

  for (int i = 0; i < mb->m_dof; ++i) {
    auto diff_const_fd = (jac_const[i] - jac_fd[i]).length();
    EXPECT_NEAR(abs(TinyConstants::getDouble(diff_const_fd)), 0., tolerance);
    auto diff_fd_reuse = (jac_fd[i] - jac_reuse_fk[i]).length();
    EXPECT_NEAR(abs(TinyConstants::getDouble(diff_fd_reuse)), 0., tolerance);
  }
}

TEST_F(TinyRigidBodyTest, TestFiveNdulumForwardKinematicsDouble) {
  check_forward_kinematics<double, DoubleUtils>();
}

// TEST_F(TinyRigidBodyTest, TestFiveNdulumForwardKinematicsFix64Scalar) {
//   check_forward_kinematics<Fix64Scalar, Fix64Scalar>();
// }

TEST_F(TinyRigidBodyTest, TestDoublePendulumDouble) {
  compare_analytic_double_pendulum<double, DoubleUtils>();
}

// TEST_F(TinyRigidBodyTest, TestDoublePendulumFix64Scalar) {
//   compare_analytic_double_pendulum<Fix64Scalar, Fix64Scalar>();
// }

TEST_F(TinyRigidBodyTest, TestFiveNdulumJacobianDouble) {
  check_jacobian<double, DoubleUtils>();
}

// TEST_F(TinyRigidBodyTest, TestFiveNdulumJacobianFix64Scalar) {
//   check_jacobian<Fix64Scalar, Fix64Scalar>();
// }

TEST_F(TinyRigidBodyTest, TestFiveNdulumJacobianFloatingBase) {
  check_jacobian<double, DoubleUtils>(true);
}

}  // namespace
