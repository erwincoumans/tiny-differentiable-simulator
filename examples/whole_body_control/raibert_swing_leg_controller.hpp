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

#include <math.h>

#include "com_velocity_estimator.hpp"
#include "openloop_gait_generator.hpp"
#include "simple_robot.hpp"
namespace tds {
class RaibertSwingLegController {
  // The position correction coefficients in Raibert's formula.
  double kp = 0.1;
 public:
  RaibertSwingLegController(
      SimpleRobot* robot,
      OpenloopGaitGenerator* gait_generator,
      COMVelocityEstimator* state_estimator,
      double desired_height = 0.42,
      double foot_clearance = 0.01
  ) : robot_(robot), gait_generator_(gait_generator), state_estimator_
      (state_estimator) {
    desired_speed_ = {0.0, 0.0, 0.0};
    desired_height_ = desired_height - foot_clearance;
    desired_twisting_speed_ = 0.0;

    Reset();
  }

  void Reset() {
    last_leg_state_ = gait_generator_->GetDesiredLegState();
    phase_switch_foot_local_position_ =
        GetUnflattenedFootPositionsInBaseFrame();
    joint_angles_.clear();
  }

  void Update() {
    std::vector<LegState> new_leg_state = gait_generator_->GetDesiredLegState();
    // Detects phase switch foreach leg so we can remember the feet position
    // at the beginning of the swing phase.
    for (size_t leg_id = 0; leg_id < new_leg_state.size(); leg_id++) {
      if ((new_leg_state[leg_id] == SWING) &&
          (new_leg_state[leg_id] != last_leg_state_[leg_id])) {
        phase_switch_foot_local_position_[leg_id] =
            GetUnflattenedFootPositionsInBaseFrame()[leg_id];
      }
    }

    last_leg_state_ = new_leg_state;
  }

  std::unordered_map<int, std::vector<double>> GetAction() {
    std::vector<double> com_velocity =
        state_estimator_->com_velocity_body_frame;
    com_velocity[2] = 0.0;

    double yaw_dot = robot_->GetBaseRollPitchYawRate()[2];
    std::vector<std::vector<double>> hip_positions =
        robot_->GetHipPositionsinBaseFrame();

    std::vector<LegState> leg_states = gait_generator_->GetLegState();
    for (size_t leg_id = 0; leg_id < leg_states.size(); leg_id++) {
      LegState leg_state = leg_states[leg_id];
      if ((leg_state == STANCE) || (leg_state == EARLY_CONTACT)) {
        continue;
      }

      std::vector<double> hip_offset = hip_positions[leg_id];
      std::vector<double> twisting_vector = {
          -hip_offset[1], hip_offset[0], 0.0
      };

      std::vector<double> hip_horizontal_velocity(3);
      for (size_t i = 0; i < 3; i++) {
        hip_horizontal_velocity[i] = com_velocity[i] + yaw_dot *
            twisting_vector[i];
      }

      std::vector<double> target_hip_horizontal_velocity(3);
      for (size_t i = 0; i < 3; i++) {
        target_hip_horizontal_velocity[i] =
            desired_speed_[i] + desired_twisting_speed_ * twisting_vector[i];
      }

      std::vector<double> foot_target_position(3);
      for (size_t i = 0; i < 3; i++) {
        foot_target_position[i] =
            hip_horizontal_velocity[i] * gait_generator_->GetStanceDuration()
            [leg_id] / 2.0 - kp * (target_hip_horizontal_velocity[i] -
                hip_horizontal_velocity[i]);
      }

      foot_target_position[0] += hip_offset[0];
      foot_target_position[1] += hip_offset[1];
      foot_target_position[2] -= desired_height_;

      std::vector<double> foot_position = GetSwingFootTrajectory
          (gait_generator_->GetNormalizedPhase()[leg_id],
           phase_switch_foot_local_position_[leg_id], foot_target_position);

      std::vector<int> joint_ids;
      std::vector<double> joint_angles;
      robot_->ComputeMotorAnglesFromFootLocalPosition(leg_id, foot_position,
                                                      joint_ids, joint_angles);

      assert(joint_ids.size() == joint_angles.size());
      for (size_t i = 0; i < joint_ids.size(); i++) {
        int joint_id = joint_ids[i];
        double joint_angle = joint_angles[i];
        joint_angles_[joint_id] = std::make_pair(joint_angle, leg_id);
      }
    }
    std::unordered_map<int, std::vector<double>> action;
    for (const auto& pair: joint_angles_) {
      int joint_id = pair.first;
      double joint_angle = pair.second.first;
      int leg_id = pair.second.second;
      if (gait_generator_->GetDesiredLegState()[leg_id] == SWING) {
        action[joint_id] = {joint_angle, 220., 0.0, 2.0, 0.0};
      }
    }
    return action;
  }

  void SetDesiredSpeed(const std::vector<double>& speed) {
    desired_speed_ = speed;
  }

  void SetDesiredTwistingSpeed(double speed) {
    desired_twisting_speed_ = speed;
  }

 private:
  std::vector<double> desired_speed_;
  double desired_height_;
  double desired_twisting_speed_;
  std::vector<LegState> last_leg_state_;
  std::vector<std::vector<double>> phase_switch_foot_local_position_;
  std::unordered_map<int, std::pair<double, int>> joint_angles_;
  OpenloopGaitGenerator* gait_generator_;
  SimpleRobot* robot_;
  COMVelocityEstimator* state_estimator_;

  // Generates the swing trajectory using a parabola.
  std::vector<double> GetSwingFootTrajectory(
      double input_phase, const std::vector<double>& start_pos,
      const std::vector<double>& end_pos) {
    double phase;
    if (input_phase <= 0.5) {
      phase = 0.8 * sin(input_phase * M_PI);
    } else {
      phase = 0.8 + (input_phase - 0.5) * 0.4;
    }

    double x = (1.0 - phase) * start_pos[0] + phase * end_pos[0];
    double y = (1.0 - phase) * start_pos[1] + phase * end_pos[1];
    double max_clearance = 0.1;
    double mid = std::max(end_pos[2], start_pos[2]) + max_clearance;
    double z = GenParabola(phase, start_pos[2], mid, end_pos[2]);

    return {x, y, z};
  }

  // Gets a point on a parabola y = a x^2 + b x + c.
  // The Parabola is determined by three points (0, start), (0.5, mid), (1, end) in
  // the plane.
  double GenParabola(double phase, double start, double mid, double end) {
    double mid_phase = 0.5;
    double delta_1 = mid - start;
    double delta_2 = end - start;
    double delta_3 = std::pow(mid_phase, 2) - mid_phase;
    double coef_a = (delta_1 - delta_2 * mid_phase) / delta_3;
    double coef_b = (delta_2 * std::pow(mid_phase, 2) - delta_1) / delta_3;
    double coef_c = start;

    return coef_a * std::pow(phase, 2) + coef_b * phase + coef_c;
  }

  std::vector<std::vector<double>> GetUnflattenedFootPositionsInBaseFrame() {
    std::vector<std::vector<double>> ret;
    ret.resize(robot_->GetNumLegs());
    std::vector<double> positions = robot_->GetFootPositionsInBaseFrame();

    for (size_t i = 0; i < robot_->GetNumLegs(); i++) {
      ret[i] = {
          positions[i * 3], positions[i * 3 + 1],
          positions[i * 3 + 2]
      };
    }
    return ret;
  }

};
} // namespace tds