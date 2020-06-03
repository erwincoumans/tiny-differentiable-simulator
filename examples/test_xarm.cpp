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

#include <fenv.h>
#include "tiny_spatial_motion_vector.h"
#include "tiny_symmetric_spatial_dyad.h"
#include "xarm.h"

#include "fix64_scalar.h"
#include "tiny_double_utils.h"
#include "tiny_matrix3x3.h"
#include "tiny_multi_body.h"
#include "tiny_pose.h"
#include "tiny_quaternion.h"
#include "tiny_vector3.h"

template <typename TinyScalar, typename TinyConstants>
void xarm6_fk_fd(TinyMultiBody<TinyScalar, TinyConstants>& mb,
                 std::vector<TinyScalar>& qdd) {
  std::vector<TinyScalar> q;
  std::vector<TinyScalar> qd;
  std::vector<TinyScalar> tau;

  qdd.clear();
  for (int i = 0; i < mb.m_links.size(); i++) {
    q.push_back(TinyConstants::fraction(1, 10) +
                TinyConstants::fraction(1, 10) * TinyConstants::fraction(i, 1));
    qd.push_back(TinyConstants::fraction(3, 10) +
                 TinyConstants::fraction(1, 10) *
                     TinyConstants::fraction(i, 1));
    tau.push_back(TinyConstants::zero());
    qdd.push_back(TinyConstants::zero());
  }
  // mb.forwardKinematics(q, qd);
  // mb.forwardDynamics(q, qd, tau, qdd, TinyConstants::zero());

  TinyVector3<TinyScalar, TinyConstants> gravity(
      TinyConstants::zero(), TinyConstants::zero(),
      TinyConstants::fraction(-981, 100));
  mb.forward_dynamics(q, qd, tau, gravity, qdd);
}
int main(int argc, char* argv[]) {
  // Set NaN trap
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

  std::vector<double> new_qdd;

  bool isFloating = false;

  {
    TinyMultiBody<double, DoubleUtils> mb(isFloating);
    init_xarm6<double, DoubleUtils>(mb);
    xarm6_fk_fd<double, DoubleUtils>(mb, new_qdd);
  }

  {
    std::vector<Fix64Scalar> new_qdd_fp;
    TinyMultiBody<Fix64Scalar, Fix64Scalar> mb;
    mb.m_isFloating = false;
    init_xarm6<Fix64Scalar, Fix64Scalar>(mb);
    xarm6_fk_fd<Fix64Scalar, Fix64Scalar>(mb, new_qdd_fp);

    for (std::size_t i = 0; i < new_qdd.size(); ++i) {
      double new_d = Fix64Scalar::getDouble(new_qdd_fp[i]);
      double old_d = new_qdd[i];
      if (abs(old_d - new_d) > 1e-4) {
        printf(
            "ERROR: FP Discrepancy between double (%.4f) and fix64 (%.4f) "
            "qdd.\n",
            old_d, new_d);
      } else {
        printf("OK: double matching fix64 (%.4f)\n", old_d);
      }
    }
  }

  return 0;
}
