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

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stan/math/fwd/core.hpp>
#include <stan/math/fwd/scal/fun/cos.hpp>
#include <stan/math/fwd/scal/fun/sin.hpp>
#include <string>

#include "base/commandlineflags.h"
#include "base/logging.h"
#include "file/base/path.h"
#include "testing/base/public/gunit.h"
#include "third_party/absl/strings/str_format.h"

namespace tinyrigidbody {
namespace apps {
namespace {

TEST(mathFwdCoreFvar, copyCtor) {
  using stan::math::fvar;

  // get the gradient with respect to a (hence the 1) in forward mode diff
  fvar<double> a(2, 1);
  fvar<double> b(a);
  fvar<double> c = stan::math::sin(a);
  EXPECT_FLOAT_EQ(c.tangent(), stan::math::cos(a).val());
  EXPECT_FLOAT_EQ(a.val_, b.val_);
  EXPECT_FLOAT_EQ(a.d_, b.d_);
}

}  // namespace
}  // namespace apps
}  // namespace tinyrigidbody

extern "C" {

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
}
