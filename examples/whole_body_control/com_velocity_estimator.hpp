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

#include <cassert>
#include <iostream>
#include <utility>
#include <deque>

#include "simple_robot.hpp"

namespace tds {

// A stable O(1) moving filter for incoming data streams. Implements the
// Neumaier's algorithm to calculate the moving window average,
//  which is numerically stable.
class MovingWindowFilter {
 public:

  MovingWindowFilter() {}

  MovingWindowFilter(int window_size) : window_size_(window_size) {
    assert(window_size_ > 0);
    sum_ = 0.0;
    correction_ = 0.0;
  }

  // Computes the moving window average.
  double CalculateAverage(double new_value) {
    if (value_deque_.size() < window_size_) {
      // pass
    } else {
      // The left most value needs to be subtracted from the moving sum first.
      UpdateNeumaierSum(-value_deque_.front());
      value_deque_.pop_front();
    }
    // Add the new value.
    UpdateNeumaierSum(new_value);
    value_deque_.push_back(new_value);

    return (sum_ + correction_) / double(window_size_);
  }

  std::deque<double> GetValueQueue() {
    return value_deque_;
  }
 private:
  int window_size_;
  double sum_, correction_;
  std::deque<double> value_deque_;

  // Updates the moving window sum using Neumaier's algorithm.
  //
  // For more details please refer to:
  // https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
  void UpdateNeumaierSum(double value) {
    double new_sum = sum_ + value;
    if (std::abs(sum_) >= std::abs(value)) {
      // If previous sum is bigger, low-order digits of value are lost.
      correction_ += (sum_ - new_sum) + value;
    } else {
      correction_ += (value - new_sum) + sum_;
    }
    sum_ = new_sum;
  }
};

// Estimator of the CoM velocity.
class COMVelocityEstimator {
  typedef double MyScalar;
  typedef ::TINY::DoubleUtils MyTinyConstants;
  typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

 public:
  std::vector<MyScalar> com_velocity_world_frame, com_velocity_body_frame;
  explicit COMVelocityEstimator(SimpleRobot* robot, int window_size = 20)
      : robot_(robot), window_size_(window_size) {
    Reset();
  }

  void Reset() {
    velocity_filter_x_ = MovingWindowFilter(window_size_);
    velocity_filter_y_ = MovingWindowFilter(window_size_);
    velocity_filter_z_ = MovingWindowFilter(window_size_);
    com_velocity_world_frame =
        {MyTinyConstants::zero(), MyTinyConstants::zero(),
         MyTinyConstants::zero()};
    com_velocity_body_frame =
        {MyTinyConstants::zero(), MyTinyConstants::zero(),
         MyTinyConstants::zero()};
  }

  void Update() {
    std::vector<MyScalar> velocity = robot_->GetBaseVelocity();
    double vx = velocity_filter_x_.CalculateAverage(velocity[0]);
    double vy = velocity_filter_y_.CalculateAverage(velocity[1]);
    double vz = velocity_filter_z_.CalculateAverage(velocity[2]);
    std::vector<MyScalar> base_orientation = robot_->GetBaseOrientation();
    com_velocity_body_frame = robot_->TransfromAngularVelocityToLocalFrame
        ({vx, vy, vz}, base_orientation);
  }

 private:
  SimpleRobot* robot_;
  int window_size_;
  MovingWindowFilter velocity_filter_x_, velocity_filter_y_, velocity_filter_z_;
};

} // namespace tds