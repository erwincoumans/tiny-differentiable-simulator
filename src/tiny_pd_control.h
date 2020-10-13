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

#ifndef TINY_PD_CONTROL
#define TINY_PD_CONTROL

#include <vector>
#include <iostream>

struct TinyPDController {
  /**
   * Compute joint torques tau given desired joint positions and velocities
   * using PD control. If `qd_desired` is empty, the desired velocity is
   * assumed to be zero.
   * @param dof Controllable degrees of freedom.
   * @param q Current joint positions. May exclude the 7 dimensions for
   * the floating base coordinates, in case of a floating-base system.
   * @param qd Current joint velocities. May exclude the 6 dimensions for
   * the floating base velocity, in case of a floating-base system.
   * @param tau Output joint torques.
   * @param kp Position gain.
   * @param kd Velocity gain.
   * @param min_force Lower bound on calculated torque.
   * @param max_force Upper bound on calculated torque.
   * @param q_desired Desired joint positions. May exclude the 7 dimensions for
   * the floating base coordinates, in case of a floating-base system.
   * @param qd_desired Optional desired joint velocities. May be empty or
   * exclude the 6 dimensions for the floating base velocity. If empty, the
   * desired velocity is assumed to be zero.
   */
  template <typename TinyScalar, typename TinyConstants>
  static bool compute(
      int dof, const std::vector<TinyScalar>& q,
      const std::vector<TinyScalar>& qd, std::vector<TinyScalar>& tau,
      const TinyScalar& kp, const TinyScalar& kd, const TinyScalar& min_force,
      const TinyScalar& max_force, const std::vector<TinyScalar>& q_desired,
      const std::vector<TinyScalar>& qd_desired = std::vector<TinyScalar>()) {
    int dof_q_floating = dof + 7;
    int dof_qd_floating = dof + 6;
    if (!(q.size() == dof_q_floating || q.size() == dof)) {
      fprintf(stderr,
              "PD controller has incorrect input dimensions: q (%i) must match "
              "system DOF (%i) or floating-base DOF (%i).\n",
              static_cast<int>(q.size()), dof, dof_q_floating);
      return false;
    }
    if (!(qd.size() == dof_qd_floating || qd.size() == dof)) {
      fprintf(stderr,
              "PD controller has incorrect input dimensions: qd (%i) must "
              "match system DOF (%i) or floating-base DOF (%i).\n",
              static_cast<int>(qd.size()), dof, dof_qd_floating);
      return false;
    }
    if (!(q_desired.size() == dof_q_floating || q_desired.size() == dof)) {
      fprintf(stderr,
              "PD controller has incorrect input dimensions: q_desired (%i) "
              "must match system DOF (%i) or floating-base DOF (%i).\n",
              static_cast<int>(q_desired.size()), dof, dof_q_floating);
      return false;
    }
    if (!(max_force > min_force)) {
      fprintf(stderr,
              "PD controller has incorrect input: max_force (%.3f) must be "
              "greater than min_force (%.3f).\n",
              TinyConstants::getDouble(max_force),
              TinyConstants::getDouble(min_force));
      return false;
    }
    if (!(qd_desired.empty() || qd_desired.size() == dof_qd_floating ||
          qd_desired.size() == dof)) {
      fprintf(stderr,
              "PD controller has incorrect input dimensions: qd_desired (%i) "
              "must be either empty or match system DOF (%i) or floating-base "
              "DOF (%i).\n",
              static_cast<int>(qd_desired.size()), dof, dof_qd_floating);
      return false;
    }
    assert(q.size() == dof_q_floating || q.size() == dof);
    assert(qd.size() == dof_qd_floating || qd.size() == dof);
    assert(q_desired.size() == dof_q_floating || q_desired.size() == dof);
    assert(max_force > min_force);
    assert(qd_desired.empty() || qd_desired.size() == dof_qd_floating ||
           qd_desired.size() == dof);
    int q_offset = q.size() == dof ? 0 : 7;
    int qd_offset = qd.size() == dof ? 0 : 6;
    int q_des_offset = q_desired.size() == dof ? 0 : 7;
    int qd_des_offset = qd_desired.size() == dof ? 0 : 6;
    tau.resize(dof);
    for (int i = 0; i < dof; i++) {
      TinyScalar position_error = q_desired[i + q_des_offset] - q[i + q_offset];
      TinyScalar desired_velocity = qd_desired.empty()
                                        ? TinyConstants::zero()
                                        : qd_desired[i + qd_des_offset];
      TinyScalar velocity_error = (desired_velocity - qd[i + qd_offset]);
      TinyScalar force = kp * position_error + kd * velocity_error;
      tau[i] = std::max(min_force, std::min(max_force, force));
    }
    return true;
  }
};

#endif  // TINY_PD_CONTROL
