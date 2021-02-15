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
#pragma once

#include <nlohmann/json.hpp>

#include "neural_network_from_json.hpp"
#include "osqp_mpc_controller.hpp"

typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

using std::vector;

namespace tds {

namespace {

void ExtendVector(vector<MyScalar>& v1, const vector<MyScalar> v2) {
  v1.insert(v1.end(), v2.begin(), v2.end());
}

}

class TorqueStanceLegController {
 public:
  TorqueStanceLegController(SimpleRobot* robot, bool use_cpp_mpc) :
      robot_(robot), use_cpp_mpc_(use_cpp_mpc) {
    if (!use_cpp_mpc_) {
      std::string nn_json_filename;
      tds::FileUtils::find_file("mpc_ffn_model.json",
                                nn_json_filename);
      std::ifstream weights_json_file(nn_json_filename);
      nlohmann::json data;
      weights_json_file >> data;

      net_ = NeuralNetworkFromJson<
          TinyAlgebra < double, TINY::DoubleUtils>>(data);
    }
  }

  vector<MyScalar> GetAction(const vector<MyScalar>& desired_speed,
                             double desired_twisting_speed,
                             tds::SimpleRobot& robot,
                             const tds::COMVelocityEstimator&
                             com_velocity_estimator,
                             tds::OpenloopGaitGenerator&
                             openloop_gait_generator) {
    vector<MyScalar> friction_coeffs = {0.45, 0.45, 0.45, 0.45};
    vector<MyScalar> desired_com_position = {0.0, 0.0, 0.42};
    vector<MyScalar> desired_com_velocity = {desired_speed[0],
                                             desired_speed[1], 0.0};
    vector<MyScalar> desired_com_roll_pitch_yaw = {0.0, 0.0, 0.0};
    vector<MyScalar>
        desired_com_angular_velocity = {0.0, 0.0, desired_twisting_speed};
    vector<int> foot_contact_state;
    for (tds::LegState state: openloop_gait_generator.GetDesiredLegState()) {
      foot_contact_state.push_back(
          state == tds::STANCE || state == tds::EARLY_CONTACT);
    }

    vector<MyScalar> mpc_input;
    vector<MyScalar> com_vel = com_velocity_estimator.com_velocity_body_frame;
    vector<MyScalar> com_roll_pitch_yaw = robot.GetBaseRollPitchYaw();
    com_roll_pitch_yaw[2] = 0.0;
    vector<MyScalar> com_angular_velocity = robot.GetBaseRollPitchYawRate();
    vector<MyScalar> foot_positions_base_frame = robot
        .GetFootPositionsInBaseFrame();

    if (use_cpp_mpc_) {
      auto predicted_contact_forces = convex_mpc_.ComputeContactForces(
          {0.0}, com_vel, com_roll_pitch_yaw, com_angular_velocity,
          foot_contact_state, foot_positions_base_frame, friction_coeffs,
          desired_com_position, desired_com_velocity,
          desired_com_roll_pitch_yaw, desired_com_angular_velocity
      );
      vector<MyScalar> torque_output;
      for (int leg_id = 0; leg_id < num_legs_; leg_id++) {
        vector<MyScalar> leg_force = {
            predicted_contact_forces[leg_id * 3],
            predicted_contact_forces[leg_id * 3 + 1],
            predicted_contact_forces[leg_id * 3 + 2]};
        vector<MyScalar> motor_torques = robot_->MapContactForceToJointTorques(
            leg_id, leg_force);
        ExtendVector(torque_output, motor_torques);
      }
      return torque_output;
    } else {
      vector<MyScalar> mpc_input;
      ExtendVector(mpc_input, com_vel);
      ExtendVector(mpc_input, com_roll_pitch_yaw);
      ExtendVector(mpc_input, com_angular_velocity);
      vector<MyScalar> foot_contact_state_double;
      for (int state: foot_contact_state) {
        foot_contact_state_double.push_back(double(state));
      }
      ExtendVector(mpc_input, foot_contact_state_double);
      ExtendVector(mpc_input, foot_positions_base_frame);
      ExtendVector(mpc_input, friction_coeffs);
      ExtendVector(mpc_input, desired_com_position);
      ExtendVector(mpc_input, desired_com_velocity);
      ExtendVector(mpc_input, desired_com_roll_pitch_yaw);
      ExtendVector(mpc_input, desired_com_angular_velocity);
      vector<MyScalar> mpc_stance_torque_output;
      net_.compute(mpc_input, mpc_stance_torque_output);
      return mpc_stance_torque_output;
    }

  }
 private:
  bool use_cpp_mpc_;
  SimpleRobot* robot_;
  NeuralNetworkFromJson<MyAlgebra> net_;
  int num_legs_ = 4;
  ConvexMpc convex_mpc_ = ConvexMpc(
      /*mass=*/220.0 / 9.8,
      /*inertia=*/{0.07335, 0, 0, 0, 0.25068, 0, 0, 0, 0.25447},
      /*num_legs=*/num_legs_,
      /*planning_horizon=*/10,
      /*timestep=*/0.025,
      /*qp_weights=*/{5, 5, 0.2, 0, 0, 10, 0., 0., 1., 1., 1., 0., 0}
  );
};

} // namespace tds


