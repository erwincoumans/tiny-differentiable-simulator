#include <iostream>

#include "simple_robot.hpp"
#include "com_velocity_estimator.hpp"
#include "openloop_gait_generator.hpp"
#include <torch/script.h>

#include "math/neural_network.hpp"

typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

//using tds::NeuralNetwork;

int main(int argc, char* argv[]) {
//  if (argc != 2) {
//    std::cerr
//        << "usage: laikago_tds_sim <path-to-exported-script-module>\n";
//    return -1;
//  }
//
//  torch::jit::script::Module module;
//  try {
//    // Deserialize the ScriptModule from a file using torch::jit::load().
//    module = torch::jit::load(argv[1]);
//  }
//  catch (const c10::Error& e) {
//    std::cerr << "error loading the model\n" << e.msg() << std::endl;
//    return -1;
//  }
//
//  std::cout << "Torch model loaded\n";
//
//  NeuralNetwork<TinyAlgebra<double, TINY::DoubleUtils>> net(module);

  MeshcatUrdfVisualizer<MyAlgebra> meshcat_viz;
  std::cout << "Waiting for meshcat server" << std::endl;
  meshcat_viz.delete_all();

  tds::SimpleRobot robot(10, meshcat_viz);
  tds::COMVelocityEstimator com_velocity_estimator(robot);
  tds::OpenloopGaitGenerator openloop_gait_generator(robot);
}