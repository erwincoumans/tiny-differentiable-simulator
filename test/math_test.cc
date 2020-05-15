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

#include "fix64_scalar.h"
#include "testing/base/public/gmock.h"
#include "testing/base/public/gunit.h"
#include "tiny_double_utils.h"
#include "tiny_dual_double_utils.h"
#include "tiny_quaternion.h"
#include "tiny_spatial_transform.h"

namespace {

class TinyRigidBodyTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

template <typename TinyScalar, typename TinyConstants>
void check_quaternion_euler_conversion(double tolerance = 1e-10) {
  TinyQuaternion<TinyScalar, TinyConstants> q(
      TinyConstants::zero(), TinyConstants::zero(), TinyConstants::zero(),
      TinyConstants::one());
  TinyVector3<TinyScalar, TinyConstants> rpy = q.get_euler_rpy();
  EXPECT_NEAR(TinyConstants::getDouble(rpy[0]), 0., tolerance);
  EXPECT_NEAR(TinyConstants::getDouble(rpy[1]), 0., tolerance);
  EXPECT_NEAR(TinyConstants::getDouble(rpy[2]), 0., tolerance);
  q.set_euler_rpy(rpy);
  EXPECT_NEAR(TinyConstants::getDouble(q.getX()), 0., tolerance);
  EXPECT_NEAR(TinyConstants::getDouble(q.getY()), 0., tolerance);
  EXPECT_NEAR(TinyConstants::getDouble(q.getZ()), 0., tolerance);
  EXPECT_NEAR(TinyConstants::getDouble(q.getW()), 1., tolerance);
}

TEST_F(TinyRigidBodyTest, TestQuaternionEulerConversion) {
  check_quaternion_euler_conversion<double, DoubleUtils>();
}

}  // namespace
