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

#include <chrono>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <thread>

#include "visualizer/meshcat/meshcat_urdf_visualizer.h"
#include "visualizer/tinyrenderer/tinyrenderer_urdf_visualizer.h"
#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
#include "math/tiny/tiny_algebra.hpp"
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

using namespace tds;

using namespace TINY;
double knee_angle = -0.5;
double abduction_angle = 0.2;
int frameskip_gfx_sync =
    16;  // only sync every 10 frames (sim at 1000 Hz, gfx at ~60hz)

double initial_poses[] = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};

int main(int argc, char* argv[]) {
  World<MyAlgebra> world;
  UrdfParser<MyAlgebra> parser;

  // create graphics
  //MeshcatUrdfVisualizer<MyAlgebra> meshcat_viz;
  TinyRendererUrdfVisualizer<MyAlgebra> meshcat_viz;

  meshcat_viz.delete_all();

  //double pos[3]={0.,0.,1.};
  //meshcat_viz.create_sphere_instance(0.01,pos ,"test", 0xff0000);


  std::string plane_file_name;
  FileUtils::find_file("plane_implicit.urdf", plane_file_name);
  char plane_search_path[TINY_MAX_EXE_PATH_LEN];
  FileUtils::extract_path(plane_file_name.c_str(), plane_search_path,
      TINY_MAX_EXE_PATH_LEN);
  MultiBody<MyAlgebra>& plane_mb = *world.create_multi_body();
  plane_mb.set_floating_base(false);
  {
    UrdfStructures<MyAlgebra> plane_urdf_structures =
        parser.load_urdf(plane_file_name);
    UrdfToMultiBody<MyAlgebra>::convert_to_multi_body(
        plane_urdf_structures, world, plane_mb,0);
    std::string texture_path = "checker_purple.png";
    meshcat_viz.m_path_prefix = plane_search_path;
    meshcat_viz.convert_visuals(plane_urdf_structures, texture_path,&plane_mb);
  }

  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string file_name;
  FileUtils::find_file("laikago/laikago_toes_zup.urdf", file_name);
  FileUtils::extract_path(file_name.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);

  std::ifstream ifs(file_name);
  std::string urdf_string;
  if (!ifs.is_open()) {
    std::cout << "Error, cannot open file_name: " << file_name << std::endl;
    exit(-1);
  }

  urdf_string = std::string((std::istreambuf_iterator<char>(ifs)),
                            std::istreambuf_iterator<char>());
  StdLogger logger;
  UrdfStructures<MyAlgebra> urdf_structures;
  int flags = 0;
  parser.load_urdf_from_string(urdf_string, flags, logger, urdf_structures);
  // create graphics structures
  std::string texture_path = "laikago_tex.jpg";
  meshcat_viz.m_path_prefix = search_path;
  
  bool floating_base = true;
  MultiBody<MyAlgebra>& mb = *world.create_multi_body();
  mb.set_floating_base(true);
  UrdfToMultiBody<MyAlgebra>::convert_to_multi_body(
      urdf_structures, world, mb,0);
  mb.initialize();
  meshcat_viz.convert_visuals(urdf_structures, texture_path,&mb);
  int start_index = 0;
  if (floating_base) {
    start_index = 7;
    mb.q_[0] = 0;
    mb.q_[1] = 0;
    mb.q_[2] = 0;
    mb.q_[3] = 1;

    mb.q_[4] = 0;
    mb.q_[5] = 0;
    mb.q_[6] = 1.5;

    mb.qd_[0] = 0;
    mb.qd_[1] = 0;
    mb.qd_[2] = 0;
    mb.qd_[3] = 0;
  }
  if (mb.q_.size() >= 12) {
    for (int cc = 0; cc < 12; cc++) {
      mb.q_[start_index + cc] = initial_poses[cc];
    }
  }
  mb.set_position(TinyVector3<double, DoubleUtils>(0., 0., 0.7));

  TinyVector3<double, DoubleUtils> grav(DoubleUtils::zero(),
                                        DoubleUtils::zero(),
                                        DoubleUtils::fraction(-981, 100));
  double dt = 1. / 1000.;
  int sync_counter = 0;

  while (1) {
    //std::this_thread::sleep_for(std::chrono::duration<double>(dt));

    forward_kinematics(mb);

    // pd control
    if (1) {
      // use PD controller to compute tau
      int qd_offset = mb.is_floating() ? 6 : 0;
      int q_offset = mb.is_floating() ? 7 : 0;
      int num_targets = mb.tau_.size() - qd_offset;
      std::vector<double> q_targets;
      q_targets.resize(mb.tau_.size());

      double kp = 150;
      double kd = 3;
      double max_force = 550;
      int param_index = 0;

      for (int i = 0; i < mb.tau_.size(); i++) {
        mb.tau_[i] = 0;
      }
      int tau_index = 0;
      int pose_index = 0;
      for (int i = 0; i < mb.links_.size(); i++) {
        if (mb.links_[i].joint_type != JOINT_FIXED) {
          double q_desired = initial_poses[pose_index++];
          double q_actual = mb.q_[q_offset];
          double qd_actual = mb.qd_[qd_offset];
          double position_error = (q_desired - q_actual);
          double desired_velocity = 0;
          double velocity_error = (desired_velocity - qd_actual);
          double force = kp * position_error + kd * velocity_error;

          if (force < -max_force) force = -max_force;
          if (force > max_force) force = max_force;
          mb.tau_[tau_index] = force;
          q_offset++;
          qd_offset++;
          param_index++;
          tau_index++;
        }
      }
    }

    forward_dynamics(mb, grav);
    
    mb.clear_forces();

    integrate_euler_qdd(mb, dt);


    world.step(dt);

    integrate_euler(mb, dt);
    sync_counter++;
    if (sync_counter > frameskip_gfx_sync) {
      sync_counter = 0;
      meshcat_viz.sync_visual_transforms(&mb);
    }
  }

  printf("finished\n");
  return EXIT_SUCCESS;
}
