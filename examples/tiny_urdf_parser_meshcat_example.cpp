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
#include "tiny_urdf_parser.h"
#include "tiny_urdf_to_multi_body.h"
#include "tiny_file_utils.h"

double knee_angle = -0.5;
double abduction_angle = 0.2;

double initial_poses[] = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};

int main(int argc, char* argv[]) {
  
  std::string file_name;
  TinyFileUtils::find_file("laikago/laikago_toes_zup.urdf", file_name);
  char search_path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(file_name.c_str(), search_path, TINY_MAX_EXE_PATH_LEN);
  
  std::ifstream ifs(file_name);
  std::string urdf_string;

  if (!ifs.is_open()) {
    std::cout << "Error, cannot open file_name: " << file_name << std::endl;
    exit(-1);
  }

  urdf_string = std::string((std::istreambuf_iterator<char>(ifs)),
                            std::istreambuf_iterator<char>());

  StdLogger logger;

  TinyUrdfParser<double, DoubleUtils> parser;
  TinyUrdfStructures<double, DoubleUtils> urdf_structures;
  int flags = 0;
  parser.load_urdf_from_string(urdf_string, flags, logger, urdf_structures);

  // create graphics
  MeshcatUrdfVisualizer<double, DoubleUtils> meshcat_viz;
  meshcat_viz.delete_all();
  std::string texture_path = "laikago_tex.jpg";
  meshcat_viz.m_path_prefix = search_path;
  meshcat_viz.convert_visuals(urdf_structures, texture_path);

  // create multibody
  TinyWorld<double, DoubleUtils> world;
  bool floating_base = false;
  TinyMultiBody<double, DoubleUtils> mb(floating_base);

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
    mb.m_q[6] = 0;

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
  TinyVector3<double, DoubleUtils> grav(DoubleUtils::zero(),
                                        DoubleUtils::zero(),
                                        DoubleUtils::fraction(-981, 100));
  double dt = 1. / 240.;
  for (int i = 0; i < 10000; i++) {
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    mb.forward_dynamics(grav);
    mb.integrate(dt);
    meshcat_viz.sync_visual_transforms(&mb);
  }

  printf("finished\n");
  return EXIT_SUCCESS;
}
