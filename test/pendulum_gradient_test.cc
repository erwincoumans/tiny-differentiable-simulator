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

#include "pendulum_gradient.h"

#include "base/logging.h"
#include "examples/stan_double_utils.h"
#include "testing/base/public/gmock.h"
#include "testing/base/public/gunit.h"

namespace {

class PendulumGradientTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

double kTestTolerance = 1e-4;
// Basic bullet physics test. A fallings spherical object. Follows the example
TEST_F(PendulumGradientTest, GradientPositionTest) {
  standouble initial_q(-3. / 2., 1);
  TinyVector3<standouble, StanDoubleUtils> gravity(0, 0, -9.81);
  standouble end_q =
      pendulum_velocity_gradient_wrt_position<standouble, StanDoubleUtils>(
          initial_q, gravity);
  LOG(INFO) << "end_q.tangent()=" << end_q.tangent();
  standouble delta = StanDoubleUtils::fraction(1, 10000);
  initial_q += delta;

  standouble end_q_delta =
      pendulum_velocity_gradient_wrt_position<standouble, StanDoubleUtils>(
          initial_q, gravity);

  standouble grad_finite_diff = (end_q_delta - end_q) / delta;
  LOG(INFO) << "grad_finite_diff=" << grad_finite_diff.val();

  EXPECT_NEAR(end_q.tangent(), grad_finite_diff.val(), kTestTolerance);
}

}  // namespace
