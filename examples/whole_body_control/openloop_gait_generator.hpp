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

#include <utility>
#include <vector>
#include <cassert>
#include <math.h>

#include "simple_robot.hpp"

namespace tds {

enum LegState {
  SWING = 0,
  STANCE,
  EARLY_CONTACT,
  LOSE_CONTACT
};

std::vector<LegState> LAIKAGO_TROTTING = {
    SWING, STANCE, STANCE, SWING
};
double _NOMINAL_CONTACT_DETECTION_PHASE = 0.1;
std::vector<double> _NOMINAL_STANCE_DURATION = {0.3, 0.3, 0.3, 0.3};
std::vector<double> _NOMINAL_DUTY_FACTOR = {0.6, 0.6, 0.6, 0.6};
std::vector<double> _INITIAL_LEG_PHASE = {0.9, 0.0, 0.0, 0.9};

// A flexible open-loop gait generator. Each leg has its own cycle and duty
// factor. And the state of each leg alternates between stance and swing. One
// can easily formulate a set of common quadruped gaits like trotting, pacing,
// bounding, etc by tweaking the input parameters.
class OpenloopGaitGenerator {

 public:
  OpenloopGaitGenerator(SimpleRobot* robot,
                        std::vector<double> stance_duration =
                        _NOMINAL_STANCE_DURATION,
                        std::vector<double> duty_factor = _NOMINAL_DUTY_FACTOR,
                        std::vector<LegState> initial_leg_state = LAIKAGO_TROTTING,
                        std::vector<double> initial_leg_phase = _INITIAL_LEG_PHASE,
                        double contact_detection_phase_threshold =
                        _NOMINAL_CONTACT_DETECTION_PHASE
  ) :
      robot_(robot),
      stance_duration_(std::move(stance_duration)),
      duty_factor_(std::move(duty_factor)),
      initial_leg_phase_(std::move(initial_leg_phase)),
      initial_leg_state_(std::move(initial_leg_state)),
      contact_detection_phase_threshold_(contact_detection_phase_threshold) {
    CheckInputs();

    swing_duration_.resize(robot_->GetNumLegs());
    next_leg_state_.resize(robot_->GetNumLegs());
    initial_state_ratio_in_cycle_.resize(robot_->GetNumLegs());
    for (size_t leg_id = 0; leg_id < robot_->GetNumLegs(); leg_id++) {
      swing_duration_[leg_id] =
          stance_duration_[leg_id] / duty_factor_[leg_id] -
              stance_duration_[leg_id];
      // The ratio in cycle is duty factor if initial state of the leg is
      // STANCE, and 1 - duty_factory if the initial state of the leg is SWING.
      if (initial_leg_state_[leg_id] == SWING) {
        initial_state_ratio_in_cycle_[leg_id] = 1 - duty_factor_[leg_id];
        next_leg_state_[leg_id] = STANCE;
      } else {
        initial_state_ratio_in_cycle_[leg_id] = duty_factor_[leg_id];
        next_leg_state_[leg_id] = SWING;
      }
    }
    Reset();
  }

  void Reset() {
    normalized_phase_ = std::vector<double>(robot_->GetNumLegs(), 0.0);
    leg_state_ = initial_leg_state_;
    desired_leg_state_ = initial_leg_state_;
  }

  void Update(double current_time) {
    std::vector<bool> contact_state = robot_->GetFootContacts();
    for (size_t leg_id = 0; leg_id < robot_->GetNumLegs(); leg_id++) {
      double full_cycle_period = stance_duration_[leg_id] /
          duty_factor_[leg_id];
      double augmented_time = current_time + initial_leg_phase_[leg_id] *
          full_cycle_period;
      double phase_in_full_cycle = std::fmod(augmented_time,
                                             full_cycle_period)
          / full_cycle_period;
      double ratio = initial_state_ratio_in_cycle_[leg_id];
      if (phase_in_full_cycle < ratio) {
        desired_leg_state_[leg_id] = initial_leg_state_[leg_id];
        normalized_phase_[leg_id] = phase_in_full_cycle / ratio;
      } else {
        // A phase switch happens for this leg.
        desired_leg_state_[leg_id] = next_leg_state_[leg_id];
        normalized_phase_[leg_id] = (phase_in_full_cycle - ratio) / (1.0 -
            ratio);
      }
      leg_state_[leg_id] = desired_leg_state_[leg_id];

      // No contact detection at the beginning of each SWING/STANCE phase.
      if (normalized_phase_[leg_id] < contact_detection_phase_threshold_) {
        continue;
      }

      if ((leg_state_[leg_id] == SWING) && contact_state[leg_id]) {
        leg_state_[leg_id] = EARLY_CONTACT;
      }
      if ((leg_state_[leg_id] == STANCE) && !contact_state[leg_id]) {
        leg_state_[leg_id] = LOSE_CONTACT;
      }
    }
  }

  std::vector<LegState> GetDesiredLegState() {
    return desired_leg_state_;
  }

  std::vector<LegState> GetLegState() {
    return leg_state_;
  }

  std::vector<double> GetStanceDuration() {
    return stance_duration_;
  }

  std::vector<double> GetNormalizedPhase() {
    return normalized_phase_;
  }

 private:
  SimpleRobot* robot_;
  std::vector<double> stance_duration_, duty_factor_, initial_leg_phase_,
      swing_duration_, initial_state_ratio_in_cycle_, normalized_phase_;
  std::vector<LegState> desired_leg_state_, leg_state_, initial_leg_state_,
      next_leg_state_;
  double contact_detection_phase_threshold_;

  void CheckInputs() {
    assert(stance_duration_.size() == robot_->GetNumLegs());
    assert(duty_factor_.size() == robot_->GetNumLegs());
    assert(initial_leg_phase_.size() == robot_->GetNumLegs());
    assert(initial_leg_state_.size() == robot_->GetNumLegs());
  }
};

} // namespace tds