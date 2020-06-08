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

#include <fenv.h>
#include <stdio.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "pybullet_visualizer_api.h"
#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_mb_constraint_solver_spring.h"
#include "tiny_multi_body.h"
#include "tiny_system_constructor.h"

typedef PyBulletVisualizerAPI VisualizerAPI;

int main(int argc, char *argv[]) {
  std::string connection_mode = "gui";
  //"pendulum5.urdf";
  //"sphere8cube.urdf";
  //"cheetah_link0_1.urdf";
  std::string urdf_filename;
  TinyFileUtils::find_file("laikago/laikago_toes_zup.urdf", urdf_filename);

  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);

  if (argc > 1) urdf_filename = std::string(argv[1]);
  bool floating_base = true;

  // Set NaN trap
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

  printf("floating_base=%d\n", floating_base);
  printf("urdf_filename=%s\n", urdf_filename.c_str());
  auto *sim2 = new VisualizerAPI();
  bool isConnected2 = sim2->connect(eCONNECT_DIRECT);

  auto *sim = new VisualizerAPI();
  printf("connection_mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_SHARED_MEMORY;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "gui") mode = eCONNECT_GUI;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  bool isConnected = sim->connect(mode);
  if (!isConnected) {
    printf("Cannot connect\n");
    return -1;
  }

  sim->setTimeOut(10);
  sim->resetSimulation();
  int grav_id = sim->addUserDebugParameter("gravity", -10, 10, -9.81);

  TinyWorld<double, DoubleUtils> world;
  TinyMultiBody<double, DoubleUtils> *system = world.create_multi_body();
  TinySystemConstructor<> constructor(urdf_filename, plane_filename);
  constructor.m_is_floating = floating_base;
  constructor(sim2, sim, world, &system);
  //  delete world.m_mb_constraint_solver;
  //  world.m_mb_constraint_solver =
  //      new TinyMultiBodyConstraintSolverSpring<double, DoubleUtils>;

  fflush(stdout);

  if (floating_base) {
    double knee_angle = -0.5;
    double abduction_angle = 0.2;
    double initial_poses[] = {
        abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
        abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    };

    if (system->m_q.size() >= 12) {
      for (int cc = 0; cc < 12; cc++) {
        system->m_q[7 + cc] = initial_poses[cc];
      }
    }
    system->m_q[6] = 0.55;
  }

  std::vector<double> qd_des(system->m_qd);
  std::vector<double> qdd_des(system->m_qdd);
  std::vector<double> tau_res(system->m_tau);

  double dt = 1. / 1000.;
  double yaw = 0;
  double distance = 1;
  while (sim->canSubmitCommand()) {
    double gravZ = sim->readUserDebugParameter(grav_id);
    world.set_gravity(TinyVector3<double, DoubleUtils>(0, 0, gravZ));

    system->forward_kinematics();
    system->clear_forces();
    system->forward_dynamics(world.get_gravity());
    system->inverse_dynamics(system->m_q, qd_des, qdd_des, -world.get_gravity(),
                             system->m_tau);
    system->m_baseAppliedForce = system->m_baseBiasForce;
    std::cout << "Inverse dynamics tau:  ";
    for (auto &t : system->m_tau) {
      std::cout << t << "\t";
    }
    std::cout << "\n";
    world.step(dt);
    system->forward_dynamics(world.get_gravity());

    system->integrate(dt);

    PyBulletUrdfImport<double, DoubleUtils>::sync_graphics_transforms(system,
                                                                      *sim);
    yaw += 0.05;
    btVector3 basePos(0, 0, 0.2);
    btQuaternion baseOrn(0, 0, 0, 1);
    sim->resetDebugVisualizerCamera(distance, -20, yaw, basePos);
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
  }

  sim->disconnect();
  sim2->disconnect();

  delete sim;
  delete sim2;

  return EXIT_SUCCESS;
}
