#include <iostream>
#include "simple_robot.hpp"
#include "com_velocity_estimator.hpp"
#include "openloop_gait_generator.hpp"

typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

int main(int argc, char* argv[]) {
  MeshcatUrdfVisualizer<MyAlgebra> meshcat_viz;
  std::cout << "Waiting for meshcat server" << std::endl;
  meshcat_viz.delete_all();

  tds::SimpleRobot robot(10, meshcat_viz);
  tds::COMVelocityEstimator com_velocity_estimator(robot);
  tds::OpenloopGaitGenerator openloop_gait_generator(robot);
}