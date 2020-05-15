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

#include "stan_double_utils.h"
#include "tiny_double_utils.h"
#include "tiny_matrix3x3.h"
#include "tiny_quaternion.h"
#include "tiny_vector3.h"
#include "xarm.h"

template <typename TinyScalar, typename TinyConstants>
void xarm6_fk_fd(TinyMultiBody<TinyScalar, TinyConstants>& mb) {
  std::vector<TinyScalar> q;
  std::vector<TinyScalar> qd;
  std::vector<TinyScalar> tau;
  std::vector<TinyScalar> qdd;

  for (int i = 0; i < mb.m_links.size(); i++) {
    q.push_back(TinyConstants::fraction(1, 10) +
                TinyConstants::fraction(1, 10) * TinyConstants::fraction(i, 1));
    qd.push_back(TinyConstants::fraction(3, 10) +
                 TinyConstants::fraction(1, 10) *
                     TinyConstants::fraction(i, 1));
    tau.push_back(TinyConstants::zero());
    qdd.push_back(TinyConstants::zero());
  }
  mb.forward_kinematics(q, qd);
  TinyVector3<TinyScalar, TinyConstants> gravity(
      TinyConstants::zero(), TinyConstants::zero(),
      TinyConstants::fraction(981, 100));
  mb.forward_dynamics(q, qd, tau, gravity, qdd);
}

int main() {
  TinyMultiBody<standouble, StanDoubleUtils> mb;
  init_xarm6<standouble, StanDoubleUtils>(mb);
  xarm6_fk_fd<standouble, StanDoubleUtils>(mb);
}
