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

#include "meshcat_urdf_visualizer.h"
#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_urdf_parser.h"
#include "tiny_urdf_to_multi_body.h"

double knee_angle = -0.5;
double abduction_angle = 0.2;
int frameskip_gfx_sync =
    16;  // only sync every 10 frames (sim at 1000 Hz, gfx at ~60hz)

double initial_poses[] = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};

int main(int argc, char* argv[]) {
  TinyWorld<double, DoubleUtils> world;
  TinyUrdfParser<double, DoubleUtils> parser;

  // create graphics
  MeshcatUrdfVisualizer<double, DoubleUtils> meshcat_viz;
  meshcat_viz.delete_all();

  std::string plane_file_name;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_file_name);
  char plane_search_path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_file_name.c_str(), plane_search_path,
                              TINY_MAX_EXE_PATH_LEN);
  TinyMultiBody<double, DoubleUtils>& plane_mb = *world.create_multi_body();
  plane_mb.m_isFloating = false;
  {
    TinyUrdfStructures<double, DoubleUtils> plane_urdf_structures =
        parser.load_urdf(plane_file_name);
    TinyUrdfToMultiBody<double, DoubleUtils>::convert_to_multi_body(
        plane_urdf_structures, world, plane_mb);
    std::string texture_path = "checker_purple.png";
    meshcat_viz.m_path_prefix = plane_search_path;
    meshcat_viz.convert_visuals(plane_urdf_structures, texture_path);
  }

  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string file_name;
  TinyFileUtils::find_file("laikago/laikago_toes_zup.urdf", file_name);
  TinyFileUtils::extract_path(file_name.c_str(), search_path,
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
  TinyUrdfStructures<double, DoubleUtils> urdf_structures;
  int flags = 0;
  parser.load_urdf_from_string(urdf_string, flags, logger, urdf_structures);
  // create graphics structures
  std::string texture_path = "laikago_tex.jpg";
  meshcat_viz.m_path_prefix = search_path;
  meshcat_viz.convert_visuals(urdf_structures, texture_path);
  bool floating_base = true;
  TinyMultiBody<double, DoubleUtils>& mb = *world.create_multi_body();
  mb.m_isFloating = true;
  TinyUrdfToMultiBody<double, DoubleUtils>::convert_to_multi_body(
      urdf_structures, world, mb);
  mb.initialize();

  int start_index = 0;
  if (floating_base) {
    start_index = 7;
    mb.m_q[0] = 0;
    mb.m_q[1] = 0;
    mb.m_q[2] = 0;
    mb.m_q[3] = 1;

    mb.m_q[4] = 0;
    mb.m_q[5] = 0;
    mb.m_q[6] = 1.5;

    mb.m_qd[0] = 0;
    mb.m_qd[1] = 0;
    mb.m_qd[2] = 0;
    mb.m_qd[3] = 0;
  }
  if (mb.m_q.size() >= 12) {
    for (int cc = 0; cc < 12; cc++) {
      mb.m_q[start_index + cc] = initial_poses[cc];
    }
  }
  mb.set_position(TinyVector3<double, DoubleUtils>(0., 0., 3.));

  TinyVector3<double, DoubleUtils> grav(DoubleUtils::zero(),
                                        DoubleUtils::zero(),
                                        DoubleUtils::fraction(-981, 100));
  double dt = 1. / 1000.;
  int sync_counter = 0;

  while (1) {
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));

    mb.forward_kinematics();

    // pd control
    if (1) {
      // use PD controller to compute tau
      int qd_offset = mb.m_isFloating ? 6 : 0;
      int q_offset = mb.m_isFloating ? 7 : 0;
      int num_targets = mb.m_tau.size() - qd_offset;
      std::vector<double> q_targets;
      q_targets.resize(mb.m_tau.size());

      double kp = 150;
      double kd = 3;
      double max_force = 550;
      int param_index = 0;

      for (int i = 0; i < mb.m_tau.size(); i++) {
        mb.m_tau[i] = 0;
      }
      int tau_index = 0;
      int pose_index = 0;
      for (int i = 0; i < mb.m_links.size(); i++) {
        if (mb.m_links[i].m_joint_type != JOINT_FIXED) {
          double q_desired = initial_poses[pose_index++];
          double q_actual = mb.m_q[q_offset];
          double qd_actual = mb.m_qd[qd_offset];
          double position_error = (q_desired - q_actual);
          double desired_velocity = 0;
          double velocity_error = (desired_velocity - qd_actual);
          double force = kp * position_error + kd * velocity_error;

          if (force < -max_force) force = -max_force;
          if (force > max_force) force = max_force;
          mb.m_tau[tau_index] = force;
          q_offset++;
          qd_offset++;
          param_index++;
          tau_index++;
        }
      }
    }

    mb.forward_dynamics(grav);

    mb.integrate_q(dt);

    world.step(dt);

    mb.integrate(dt);
    sync_counter++;
    if (sync_counter > frameskip_gfx_sync) {
      sync_counter = 0;
      meshcat_viz.sync_visual_transforms(&mb);
    }
  }

  printf("finished\n");
  return EXIT_SUCCESS;
}
