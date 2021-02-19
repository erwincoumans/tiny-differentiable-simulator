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

#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include <cassert>

#include "world.hpp"
#include "mb_constraint_solver.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "visualizer/meshcat/meshcat_urdf_visualizer.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "tiny_inverse_kinematics.h"

namespace tds {

enum MotorControlMode {
  MOTOR_CONTROL_POSITION = 0,
  MOTOR_CONTROL_TORQUE,
  MOTOR_CONTROL_HYBRID
};

class SimpleRobot {
  typedef double MyScalar;
  typedef ::TINY::DoubleUtils MyTinyConstants;
  typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;
  typedef Transform<MyAlgebra> TinySpatialTransform;

  double knee_angle = -0.5;
  double abduction_angle = 0.2;
  std::vector<MyScalar> initial_poses = {
      abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
      abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
  };
  std::regex toe_name_pattern = std::regex("jtoe.*");
  double laikago_default_abduction_angle = 0.0;
  double laikago_default_hip_angle = 0.07;
  double laikago_default_knee_angle = -1.25 + 0.66;
  std::vector<MyScalar> init_motor_angles = {
      laikago_default_abduction_angle, laikago_default_hip_angle,
      laikago_default_knee_angle, laikago_default_abduction_angle,
      laikago_default_hip_angle,
      laikago_default_knee_angle, laikago_default_abduction_angle,
      laikago_default_hip_angle,
      laikago_default_knee_angle, laikago_default_abduction_angle,
      laikago_default_hip_angle,
      laikago_default_knee_angle,
  };
  std::vector<std::vector<double>> default_hip_positions = {
      {0.21, -0.1157, 0}, {0.21, 0.1157, 0},
      {-0.21, -0.1157, 0}, {-0.21, 0.1157, 0},
  };

 public:
  SimpleRobot() {}
  explicit SimpleRobot(double time_step,
                       MeshcatUrdfVisualizer<MyAlgebra>& meshcat_viz,
                       int num_legs = 4)
      : time_step_(time_step),
        num_legs_(num_legs) {
    world_.default_friction = 10.0;

    SetupPlane(meshcat_viz);
    SetupLaikago(meshcat_viz);

    forward_kinematics(*robot_mb_, robot_mb_->q(), robot_mb_->qd());
    meshcat_viz.sync_visual_transforms(robot_mb_);

    std::cout << "start settle down\n";
    SettleDownForReset(meshcat_viz);
    std::cout << "done settle down\n";
  }

  void Reset() {
    step_counter_ = 0;
  }

  void Step(const std::vector<MyScalar>& action,
            const MotorControlMode& mode,
            MeshcatUrdfVisualizer<MyAlgebra>& meshcat_viz) {
    forward_kinematics(*robot_mb_, robot_mb_->q(), robot_mb_->qd());
    forward_dynamics(*robot_mb_,
                     TINY::TinyVector3<
                         MyScalar, MyTinyConstants>(TINY::DoubleUtils::zero(),
                                                    TINY::DoubleUtils::zero(),
                                                    TINY::DoubleUtils::fraction(
                                                        -980, 100)));
    ApplyAction(action, mode);

    integrate_euler_qdd(*robot_mb_, time_step_);
    world_.step(time_step_);
    integrate_euler(*robot_mb_, time_step_);

    skip_simulation_sync_cnt_--;
    if (skip_simulation_sync_cnt_ <= 0) {
      skip_simulation_sync_cnt_ = 16;
      meshcat_viz.sync_visual_transforms(robot_mb_);
    }

    step_counter_++;
  }

  // Returns the linear velocity of the robot's base.
  std::vector<MyScalar> GetBaseVelocity() {
    return {robot_mb_->qd(3), robot_mb_->qd(4), robot_mb_->qd(5)};
  }

  // Returns the angular velocity of the robot's base.
  std::vector<MyScalar> GetBaseAngularVelocity() {
    return {robot_mb_->qd(0), robot_mb_->qd(1), robot_mb_->qd(2)};
  }

  // Returns the orientation of the robot's base in quaternion.
  std::vector<MyScalar> GetBaseOrientation() {
    MyAlgebra::Quaternion rn = robot_mb_->get_orientation();
    return {rn[0], rn[1], rn[2], rn[3]};
  }

  // Returns the orientation of the robot's base in euler angle.
  std::vector<MyScalar> GetBaseRollPitchYaw() {
    return TinyVector3ToVector(robot_mb_->get_orientation().get_euler_rpy());
  }

  // Returns the orientation of the robot's base in euler angle.
  std::vector<MyScalar> GetBaseRollPitchYawRate() {
    return TransfromAngularVelocityToLocalFrame(GetBaseAngularVelocity(),
                                                GetBaseOrientation());
  }

  // Given the angular velocity of the robot in world frame, returns
  // the angular velocity based on the given orientation in local (robot's)
  // frame.
  std::vector<MyScalar> TransfromAngularVelocityToLocalFrame
      (const std::vector<MyScalar>& angular_velocity,
       const std::vector<MyScalar>& orientation) {
    TinySpatialTransform tr;
    tr.translation = TINY::TinyVector3<MyScalar, MyTinyConstants>
        (MyTinyConstants::zero(),
         MyTinyConstants::zero(),
         MyTinyConstants::zero());
    auto orn = TINY::TinyQuaternion<MyScalar, MyTinyConstants>(
        orientation[0], orientation[1], orientation[2], orientation[3]
    );
    tr.rotation.setRotation(orn);
    auto rel_vel = tr.apply_inverse(TINY::TinyVector3<MyScalar, MyTinyConstants>
                                        (angular_velocity[0],
                                         angular_velocity[1],
                                         angular_velocity[2]));
    return TinyVector3ToVector(rel_vel);
  }

  // Returns the robot's foot positions in the base frame in a flattened vector.
  std::vector<MyScalar> GetFootPositionsInBaseFrame() {
    assert(foot_link_ids_.size() == num_legs_);
    std::vector<MyScalar> foot_positions;
    for (int foot_id: foot_link_ids_) {
      for (auto item: GetLinkPosInBaseFrame(foot_id)) {
        foot_positions.push_back(item);
      }
    }
    return foot_positions;
  }

  void ComputeMotorAnglesFromFootLocalPosition(
      int leg_id, const std::vector<double>& foot_position,
      std::vector<int>& joint_position_idxs, std::vector<double>&
  joint_angles) {
    assert(foot_link_ids_.size() == num_legs_);
    int toe_id = foot_link_ids_[leg_id];
    int motors_per_leg = num_motors_ / num_legs_;
    joint_position_idxs.clear();
    joint_position_idxs.resize(motors_per_leg);
    for (size_t i = 0; i < motors_per_leg; i++) {
      joint_position_idxs[i] = leg_id * motors_per_leg + i;
    }

    joint_angles = GetJointAnglesFromLinkPosition
        (foot_position, toe_id, joint_position_idxs);
  }

  std::vector<std::vector<MyScalar>> GetHipPositionsinBaseFrame() {
    return default_hip_positions;
  }

  double GetTimeSinceReset() {
    return step_counter_ * time_step_;
  }

  int GetNumLegs() {
    return num_legs_;
  }

  int GetNumMotors() {
    return num_motors_;
  }

  std::vector<bool> GetFootContacts() {
    return {false, false, false, false};
  }

  void PrintRobotState() {
    robot_mb_->print_state();
  }

  std::vector<double> MapContactForceToJointTorques(int leg_id,
                                                    std::vector<double> contact_force) {
    auto jacobian_vector = ComputeJacobian(leg_id);
    TINY::TinyVector3<MyScalar, MyTinyConstants> contact_force_tiny_vector(
        contact_force[0], contact_force[1], contact_force[2]
    );
    MyAlgebra::Matrix3X mat(1, 3);
    mat.assign_vector_horizontal(0, 0, contact_force_tiny_vector);
    auto all_motor_torques = mat * jacobian_vector;
    int motors_per_leg = num_motors_ / num_legs_;
    int com_dof = 6;
    std::vector<double> motor_torques;

    for (int joint_id = leg_id * motors_per_leg;
         joint_id < (leg_id + 1) * motors_per_leg; joint_id++) {
      motor_torques.push_back(all_motor_torques(0, com_dof + joint_id));
    }
    return motor_torques;
  }

 private:
  double time_step_;
  int step_counter_;
  int num_legs_ = 4;
  int num_motors_ = 12;
  int skip_simulation_sync_cnt_ = 0;
  double kp_ = 220.0;
  double kd_ = 2.0;
  World<MyAlgebra> world_;
  MultiBodyConstraintSolver<MyAlgebra> mb_solver_;
  UrdfParser<MyAlgebra> parser_;
  MultiBody<MyAlgebra>* plane_mb_, * robot_mb_;
  std::vector<int> foot_link_ids_;

  // Loads the plane urdf, adds the multi_body to world_, and sets up
  // visualization.
  void SetupPlane(MeshcatUrdfVisualizer<MyAlgebra>& meshcat_viz) {
    std::string plane_file_name;
    FileUtils::find_file("plane_implicit.urdf", plane_file_name);
    char plane_search_path[TINY_MAX_EXE_PATH_LEN];
    FileUtils::extract_path(plane_file_name.c_str(), plane_search_path,
                            TINY_MAX_EXE_PATH_LEN);
    plane_mb_ = world_.create_multi_body();
    plane_mb_->set_floating_base(false);
    {
      UrdfStructures<MyAlgebra> plane_urdf_structures =
          parser_.load_urdf(plane_file_name);
      UrdfToMultiBody<MyAlgebra>::convert_to_multi_body(
          plane_urdf_structures, world_, *plane_mb_, 0);
      std::string texture_path = "checker_purple.png";
      meshcat_viz.m_path_prefix = plane_search_path;
      meshcat_viz.convert_visuals(plane_urdf_structures, texture_path, 0);
    }
  }

  // Loads the laikago urdf, adds the multi_body to world_, and sets up
  // visualization.
  void SetupLaikago(MeshcatUrdfVisualizer<MyAlgebra>& meshcat_viz) {
    char robot_search_path[TINY_MAX_EXE_PATH_LEN];
    std::string robot_file_name;
    FileUtils::find_file("laikago/laikago_toes_zup_joint_order.urdf",
                         robot_file_name);
    FileUtils::extract_path(robot_file_name.c_str(), robot_search_path,
                            TINY_MAX_EXE_PATH_LEN);

    std::ifstream ifs(robot_file_name);
    std::string urdf_string;
    if (!ifs.is_open()) {
      std::cout << "Error, cannot open file_name: " << robot_file_name
                << std::endl;
      exit(-1);
    }

    urdf_string = std::string((std::istreambuf_iterator<char>(ifs)),
                              std::istreambuf_iterator<char>());
    StdLogger logger;
    UrdfStructures<MyAlgebra> urdf_structures;
    int flags = 0;
    parser_.load_urdf_from_string(urdf_string, flags, logger,
                                  urdf_structures);
    // create graphics structures
    std::string texture_path = "laikago_tex.jpg";
    meshcat_viz.m_path_prefix = robot_search_path;

    bool floating_base = true;
    robot_mb_ = world_.create_multi_body();
    robot_mb_->set_floating_base(true);
    UrdfToMultiBody<MyAlgebra>::convert_to_multi_body(
        urdf_structures, world_, *robot_mb_, 0);
    robot_mb_->initialize();
    meshcat_viz.convert_visuals(urdf_structures, texture_path, robot_mb_);
    int start_index = 0;
    if (floating_base) {
      start_index = 7;
      robot_mb_->q_[0] = 0;
      robot_mb_->q_[1] = 0;
      robot_mb_->q_[2] = 0;
      robot_mb_->q_[3] = 1;

      robot_mb_->q_[4] = 0;
      robot_mb_->q_[5] = 0;
      robot_mb_->q_[6] = 1.5;

      robot_mb_->qd_[0] = 0;
      robot_mb_->qd_[1] = 0;
      robot_mb_->qd_[2] = 0;
      robot_mb_->qd_[3] = 0;
    }
    if (robot_mb_->q_.size() >= 12) {
      for (int cc = 0; cc < 12; cc++) {
        robot_mb_->q_[start_index + cc] = initial_poses[cc];
      }
    }
    robot_mb_->set_position(TINY::TinyVector3<double, TINY::DoubleUtils>(
        1.5, -2.0, 0.5));
    robot_mb_->set_orientation(TINY::TinyQuaternion<MyScalar, MyTinyConstants>(
        0.0, 0.0, 0.0, 1.0));

    BuildUrdfIds(urdf_structures);
  }

  void BuildUrdfIds(const UrdfStructures<MyAlgebra>& urdf_structures) {
    int num_joints = urdf_structures.joints.size();
    for (int joint_id = 0; joint_id < num_joints; joint_id++) {
      auto joint = urdf_structures.joints[joint_id];
      if (std::regex_match(joint.joint_name, toe_name_pattern)) {
        foot_link_ids_.push_back(joint_id);
      }
    }
  }

  void SettleDownForReset(MeshcatUrdfVisualizer<MyAlgebra>& meshcat_viz) {
    for (size_t i = 0; i < 1500; i++) {
      Step(init_motor_angles, MOTOR_CONTROL_POSITION, meshcat_viz);
    }
    printf("after settle down\n");
    robot_mb_->print_state();
  }

  void ApplyAction(const std::vector<MyScalar>& action,
                   const MotorControlMode& mode) {
    if (mode == MOTOR_CONTROL_TORQUE) {
      assert(action.size() == num_motors_);
      for (size_t i = 0; i < action.size(); i++) {
        robot_mb_->tau_[i] = action[i];
      }
    } else if (mode == MOTOR_CONTROL_POSITION) {
      assert(action.size() == num_motors_);
      for (size_t i = 0; i < action.size(); i++) {
        robot_mb_->tau_[i] =
            -1 * (kp_ * (robot_mb_->q(i + 7) - action[i])) - kd_ *
                robot_mb_->qd(i + 6);
      }
    } else if (mode == MOTOR_CONTROL_HYBRID) {
      assert(action.size() == 5 * num_motors_);
      for (size_t motor_id = 0; motor_id < num_motors_; motor_id++) {
        double desired_motor_angle = action[motor_id * 5];
        double kp = action[motor_id * 5 + 1];
        double desried_motor_velocity = action[motor_id * 5 + 2];
        double kd = action[motor_id * 5 + 3];
        double additional_torque = action[motor_id * 5 + 4];
        double torque =
            -1 * (kp * (robot_mb_->q(motor_id + 7) - desired_motor_angle))
                - kd *
                    (robot_mb_->qd(motor_id + 6) - desried_motor_velocity)
                + additional_torque;
        robot_mb_->tau_[motor_id] = torque;
      }
    }
  }

  // Computes the link's local position in the robot frame.
  std::vector<MyScalar> GetLinkPosInBaseFrame(int link_id) {
    Transform<MyAlgebra> base_X_world;
    std::vector<Transform<MyAlgebra> > links_X_world;
    std::vector<Transform<MyAlgebra> > links_X_base;

    forward_kinematics_q(*robot_mb_, robot_mb_->q(), &base_X_world,
                         &links_X_world,
                         &links_X_base);
    return TinyVector3ToVector(links_X_base[link_id].translation);
  }

  static std::vector<MyScalar> TinyVector3ToVector(const TINY::TinyVector3<
      MyScalar, MyTinyConstants>& tiny_vector3) {
    return {tiny_vector3[0], tiny_vector3[1], tiny_vector3[2]};
  }

  // Uses Inverse Kinematics to calculate joint angles.
  std::vector<double> GetJointAnglesFromLinkPosition(
      const std::vector<double>& link_position,
      int link_id,
      const std::vector<int>& joint_ids) {
    Transform base_world_tr = robot_mb_->get_world_transform(-1);
    TinySpatialTransform local_base_tr;
    local_base_tr.translation =
        TINY::TinyVector3<MyScalar, MyTinyConstants>(TINY::DoubleUtils::zero(),
                                                     TINY::DoubleUtils::zero(),
                                                     TINY::DoubleUtils::zero());
    local_base_tr.rotation.setRotation(
        TINY::TinyQuaternion<MyScalar, MyTinyConstants>(
            0.0, 0.0, 0.0, 1.0));
    Transform world_tr = base_world_tr * local_base_tr;
    TinySpatialTransform local_link_tr;
    local_link_tr.rotation.set_identity();
    local_link_tr.translation =
        TINY::TinyVector3<MyScalar, MyTinyConstants>(link_position[0],
                                                     link_position[1],
                                                     link_position[2]);
    Transform link_tr = world_tr * local_link_tr;
    std::vector<double> world_link_pos = {link_tr.translation[0], link_tr
        .translation[1], link_tr.translation[2]};
    TINY::TinyVector3<MyScalar, MyTinyConstants> target_world_pos(
        world_link_pos[0], world_link_pos[1], world_link_pos[2]
    );

    TINY::TinyInverseKinematics<MyScalar, MyTinyConstants, TINY::IK_JAC_PINV>
        inverse_kinematics;
    inverse_kinematics.weight_reference = MyTinyConstants::fraction(0, 10);
    // step size
    inverse_kinematics.alpha = MyTinyConstants::fraction(3, 10);
    inverse_kinematics.targets.emplace_back(link_id, target_world_pos);
    inverse_kinematics.q_reference = robot_mb_->q();
    MyAlgebra::VectorX all_joint_angles2;
    inverse_kinematics.compute(*robot_mb_, robot_mb_->q(), all_joint_angles2);

    std::vector<MyScalar> vec;
    for (size_t i = 0; i < all_joint_angles2.size() - 7; i++) {
      vec.push_back(all_joint_angles2[i + 7]);
    }
    // Extract the relevant joint angles.
    std::vector<double> joint_angles;
    for (int joint_id : joint_ids) {
      joint_angles.push_back(vec[joint_id]);
    }
    return joint_angles;
  }

  MyAlgebra::Matrix3X ComputeJacobian(int leg_id) {
    int link_id = foot_link_ids_[leg_id];
    auto world_point = robot_mb_->links()[link_id].X_world.translation;
    auto local_point =
        robot_mb_->get_world_transform(-1).apply_inverse(world_point);
    return point_jacobian2(*robot_mb_, link_id, local_point, true);
  }

};

} // namespace tds