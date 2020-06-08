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

#ifndef EXPERIMENTAL_USERS_ERWINCOUMANS_DIFFERENTIABLE_TINYRIGIDBODY_PENDULUM_GRADIENT_H_
#define EXPERIMENTAL_USERS_ERWINCOUMANS_DIFFERENTIABLE_TINYRIGIDBODY_PENDULUM_GRADIENT_H_

#include "pendulum.h"

// simulate a half-cycle and report the q at that time
template <typename TinyScalar, typename TinyConstants>
TinyScalar pendulum_velocity_gradient_wrt_position(
    TinyScalar q_initial,
    const TinyVector3<TinyScalar, TinyConstants>& gravity) {
  TinyWorld<TinyScalar, TinyConstants> world;
  TinyMultiBody<TinyScalar, TinyConstants> mb;
  init_compound_pendulum<TinyScalar, TinyConstants>(mb, world);

  TinyScalar prev_qd = TinyConstants::zero();
  TinyScalar cur_qd = TinyConstants::zero();

  std::vector<TinyScalar> q;
  q.push_back(q_initial);
  q.push_back(TinyConstants::zero());

  std::vector<TinyScalar> qd;
  qd.push_back(cur_qd);
  qd.push_back(TinyConstants::zero());

  std::vector<TinyScalar> tau;
  std::vector<TinyScalar> qdd;
  qdd.push_back(TinyConstants::zero());
  qdd.push_back(TinyConstants::zero());
  tau.push_back(TinyConstants::zero());
  tau.push_back(TinyConstants::zero());
  int step = 0;
  TinyScalar dt = TinyConstants::fraction(1, 1000);  // 0.001 sec time step
  while (prev_qd * qd[0] >= TinyConstants::zero()) {
    prev_qd = qd[0];
    mb.forward_dynamics(q, qd, tau, gravity, qdd);
    mb.integrate(q, qd, qdd, dt);
  }

  return q[0];
}

#endif  // EXPERIMENTAL_USERS_ERWINCOUMANS_DIFFERENTIABLE_TINYRIGIDBODY_PENDULUM_GRADIENT_H_
