#include <iostream>
#include <thread>

#include "simple_robot.hpp"
#include "com_velocity_estimator.hpp"
#include "openloop_gait_generator.hpp"
#include "raibert_swing_leg_controller.hpp"
#include <torch/script.h>

#include "math/neural_network.hpp"

typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

using tds::NeuralNetwork;
using std::vector;

namespace {

void PrintVector(const vector<MyScalar>& v) {
  std::cerr << std::endl << "vector: ";
  for (MyScalar item: v) {
    std::cerr << item << ", ";
  }
}

void ExtendVector(vector<MyScalar>& v1, const vector<MyScalar> v2) {
  v1.insert(v1.end(), v2.begin(), v2.end());
}

vector<MyScalar> GetMpcInput(const vector<MyScalar>& desired_speed,
                             double desired_twisting_speed,
                             tds::SimpleRobot& robot,
                             const tds::COMVelocityEstimator&
                             com_velocity_estimator,
                             tds::OpenloopGaitGenerator&
                             openloop_gait_generator
) {
  vector<MyScalar> friction_coeffs = {0.45, 0.45, 0.45, 0.45};
  vector<MyScalar> desired_com_position = {0.0, 0.0, 0.42};
  vector<MyScalar> desired_com_velocity = {desired_speed[0],
                                           desired_speed[1], 0.0};
  vector<MyScalar> desired_com_roll_pitch_yaw = {0.0, 0.0, 0.0};
  vector<MyScalar>
      desired_com_angular_velocity = {0.0, 0.0, desired_twisting_speed};
  vector<MyScalar> foot_contact_state;
  for (tds::LegState state: openloop_gait_generator.GetDesiredLegState()) {
    foot_contact_state.push_back(
        double(state == tds::STANCE || state == tds::EARLY_CONTACT));
  }

  vector<MyScalar> mpc_input;
  vector<MyScalar> com_vel = com_velocity_estimator.com_velocity_body_frame;
  vector<MyScalar> com_roll_pitch_yaw = robot.GetBaseRollPitchYaw();
  com_roll_pitch_yaw[2] = 0.0;
  vector<MyScalar> com_angular_velocity = robot.GetBaseRollPitchYawRate();
  vector<MyScalar> foot_positions_base_frame = robot
      .GetFootPositionsInBaseFrame();

  ExtendVector(mpc_input, com_vel);
  ExtendVector(mpc_input, com_roll_pitch_yaw);
  ExtendVector(mpc_input, com_angular_velocity);
  ExtendVector(mpc_input, foot_contact_state);
  ExtendVector(mpc_input, foot_positions_base_frame);
  ExtendVector(mpc_input, friction_coeffs);
  ExtendVector(mpc_input, desired_com_position);
  ExtendVector(mpc_input, desired_com_velocity);
  ExtendVector(mpc_input, desired_com_roll_pitch_yaw);
  ExtendVector(mpc_input, desired_com_angular_velocity);

  std::cout << "\ncom_vel= ";
  PrintVector(com_vel);
  std::cout << "\ncom_roll_pitch_yaw= ";
  PrintVector(com_roll_pitch_yaw);
  std::cout << "\ncom_angular_velocity= ";
  PrintVector(com_angular_velocity);
//  PrintVector(foot_contact_state);
//  PrintVector(foot_positions_base_frame);
//  PrintVector(friction_coeffs);
//  PrintVector(desired_com_position);
//  PrintVector(desired_com_velocity);
//  PrintVector(desired_com_roll_pitch_yaw);
//  PrintVector(desired_com_angular_velocity);

  return mpc_input;
}

}

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr
        << "usage: laikago_tds_sim <path-to-exported-script-module>\n";
    return -1;
  }
  torch::jit::script::Module module;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n" << e.msg() << std::endl;
    return -1;
  }
  std::cout << "Torch model loaded\n";
  tds::NeuralNetwork<MyAlgebra> net(module);

  MeshcatUrdfVisualizer<MyAlgebra> meshcat_viz;
  std::cout << "Waiting for meshcat server" << std::endl;
  meshcat_viz.delete_all();

  double time_step = 0.001;
  tds::SimpleRobot robot(time_step, meshcat_viz);
  robot.PrintRobotState();
//  return 1;
  tds::COMVelocityEstimator com_velocity_estimator(&robot);
  tds::OpenloopGaitGenerator openloop_gait_generator(&robot);
  tds::RaibertSwingLegController
      raibert_swing_leg_controller(&robot, &openloop_gait_generator,
                                   &com_velocity_estimator);

  robot.Reset();
  com_velocity_estimator.Reset();
  openloop_gait_generator.Reset();
  raibert_swing_leg_controller.Reset();

  int cnt = 0;

  while (1) {
    std::this_thread::sleep_for(std::chrono::duration<double>(0.003));

    vector<MyScalar> desired_speed = {0.0, 0.0, 0.0};
    double desired_twisting_speed = 0.0;

    raibert_swing_leg_controller.SetDesiredSpeed(desired_speed);
    raibert_swing_leg_controller.SetDesiredTwistingSpeed
        (desired_twisting_speed);

    openloop_gait_generator.Update(robot.GetTimeSinceReset());
    com_velocity_estimator.Update();
    raibert_swing_leg_controller.Update();

    vector<MyScalar> mpc_input = GetMpcInput(desired_speed,
                                             desired_twisting_speed,
                                             robot,
                                             com_velocity_estimator,
                                             openloop_gait_generator);

    std::cout << "\n\niteration: " << cnt << std::endl;
    std::cout << "mpc iutput=\n";
    PrintVector(mpc_input);

    vector<MyScalar> mpc_stance_torque_output;
    net.compute(mpc_input, mpc_stance_torque_output);
    std::cout << "\n\nstance action=";
    PrintVector(mpc_stance_torque_output);

    auto swing_action = raibert_swing_leg_controller.GetAction();

    std::vector<double> hybrid_action;
    for (size_t joint_id = 0; joint_id < robot.GetNumMotors(); joint_id++) {
//      if (swing_action.find(joint_id) != swing_action.end()) {
//        std::vector<double> command = swing_action[joint_id];
//        for (double item: command) {
//          hybrid_action.push_back(item);
//        }
//      } else {
        for (size_t i = 0; i < 4; i++) {
          hybrid_action.push_back(0.0);
        }
        hybrid_action.push_back(mpc_stance_torque_output[joint_id]);
//      }
    }

    // TODO: check stance input, what's building up this error?

//    if (cnt >= 0) {
//      std::cout << "\n\nstance action=";
//      PrintVector(mpc_stance_torque_output);
//      std::cout << "\n\nswing action=";
//      for (auto item: swing_action) {
//        std::cout << "key= " << item.first;
//        PrintVector(item.second);
//      }
//      std::cout << "\n\nfinal action=";
//      PrintVector(hybrid_action);
//      printf("\nbefore step\n");
//      robot.PrintRobotState();
//    }
//    printf("\nbefore step\n");
//    robot.PrintRobotState();

    robot.Step(hybrid_action, tds::MOTOR_CONTROL_HYBRID, meshcat_viz);
//    printf("\nafter step\n");
//    robot.PrintRobotState();

//    if (!swing_action.empty()) {
//      std::cout << "found non-empty swing action!!!" << std::endl;
//      exit(1);
//    }

    if (cnt == 305) {
      break;
    }

    cnt++;
  }
}