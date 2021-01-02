#include <iostream>
#include <thread>

#include "simple_robot.hpp"
#include "com_velocity_estimator.hpp"
#include "openloop_gait_generator.hpp"
//#include <torch/script.h>

#include "math/neural_network.hpp"

typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

//using tds::NeuralNetwork;
using std::vector;

namespace {

void ExtendVector(vector<MyScalar>& v1, const vector<MyScalar> v2) {
  v1.insert(v1.end(), v2.begin(), v2.end());
}

vector<MyScalar> GetMpcInput(const vector<MyScalar>& desired_speed,
                             double desired_twisting_speed,
                             tds::SimpleRobot& robot,
                             const tds::COMVelocityEstimator&
                             com_velocity_estimator,
                             const tds::OpenloopGaitGenerator&
                             openloop_gait_generator
) {
  vector<MyScalar> friction_coeffs = {0.45, 0.45, 0.45, 0.45};
  vector<MyScalar> desired_com_position = {0.0, 0.0, 0.45};
  vector<MyScalar> desired_com_velocity = {desired_speed[0],
                                           desired_speed[1], 0.0};
  vector<MyScalar> desired_com_roll_pitch_yaw = {0.0, 0.0, 0.0};
  vector<MyScalar>
      desired_com_angular_velocity = {0.0, 0.0, desired_twisting_speed};
  vector<MyScalar> foot_contact_state;
  for (const tds::LegState
        & state: openloop_gait_generator.desired_leg_state_) {
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
  return mpc_input;
}

}

int main(int argc, char* argv[]) {
  //if (argc != 2) {
  //  std::cerr
  //      << "usage: laikago_tds_sim <path-to-exported-script-module>\n";
  //  return -1;
  //}
  //torch::jit::script::Module module;
  //try {
  //  // Deserialize the ScriptModule from a file using torch::jit::load().
  //  module = torch::jit::load(argv[1]);
  //}
  //catch (const c10::Error& e) {
  //  std::cerr << "error loading the model\n" << e.msg() << std::endl;
  //  return -1;
  //}
  std::cout << "Torch model loaded\n";
  tds::NeuralNetwork<MyAlgebra> net;
  //(module);

  MeshcatUrdfVisualizer<MyAlgebra> meshcat_viz;
  std::cout << "Waiting for meshcat server" << std::endl;
  meshcat_viz.delete_all();

  double time_step = 1. / 1000.;
  tds::SimpleRobot robot(time_step, meshcat_viz);
  
  tds::COMVelocityEstimator com_velocity_estimator(robot);
  tds::OpenloopGaitGenerator openloop_gait_generator(robot);
  while (1) {
    std::this_thread::sleep_for(std::chrono::duration<double>(time_step));

    vector<MyScalar> desired_speed = {0.1, 0.0, 0.0};
    double desired_twisting_speed = 0.1;

    vector<MyScalar> mpc_input = GetMpcInput(desired_speed,
                                             desired_twisting_speed,
                                             robot,
                                             com_velocity_estimator,
                                             openloop_gait_generator);

    // model stuff
//    vector<MyScalar> mpc_output;
//    net.compute(mpc_input, mpc_output);
//    robot.Step(mpc_output, meshcat_viz);

    robot.Step(vector<MyScalar>(12, 0.5), meshcat_viz);
  }
}