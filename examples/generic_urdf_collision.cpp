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

#include "Utils/b3Clock.h"
#include "tiny_double_utils.h"
#include "tiny_mb_constraint_solver_spring.h"
#include "tiny_multi_body.h"
#include "tiny_system_constructor.h"

#include "pybullet_visualizer_api.h"
#include "tiny_file_utils.h"

typedef PyBulletVisualizerAPI VisualizerAPI;

static VisualizerAPI *gSim = nullptr;
void MyTinySubmitProfileTiming3(const std::string &profile_name) {
  if (gSim) {
    gSim->submitProfileTiming(profile_name);
  }
}

int main(int argc, char *argv[]) {
  std::string connection_mode = "gui";

  std::string urdf_filename;
  //"cheetah_link0_1.urdf"
  //"pendulum5.urdf"
  //"sphere2.urdf"
  TinyFileUtils::find_file("sphere8cube.urdf", urdf_filename);
  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);

  if (argc > 1) urdf_filename = std::string(argv[1]);
  bool floating_base = true;

  // Set NaN trap
  feenableexcept(FE_INVALID | FE_OVERFLOW);

  printf("floating_base=%d\n", floating_base);
  printf("urdf_filename=%s\n", urdf_filename.c_str());
  VisualizerAPI *sim2 = new VisualizerAPI();
  bool isConnected2 = sim2->connect(eCONNECT_DIRECT);

  VisualizerAPI *sim = new VisualizerAPI();

  printf("connection_mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "gui") mode = eCONNECT_GUI;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  bool isConnected = sim->connect(mode);
  if (!isConnected) {
    printf("Cannot connect\n");
    return -1;
  }

  sim->resetSimulation();
  sim->setTimeOut(10);
  int grav_id = sim->addUserDebugParameter("gravity", -10, 10, -9.81);

  int rotateCamera = 0;

  TinyWorld<double, DoubleUtils> world;
  TinyMultiBody<double, DoubleUtils> *system = world.create_multi_body();
  TinySystemConstructor<> constructor(urdf_filename, plane_filename);
  constructor.m_is_floating = floating_base;
  constructor(sim2, sim, world, &system);
  delete world.m_mb_constraint_solver;
  world.m_mb_constraint_solver =
      new TinyMultiBodyConstraintSolverSpring<double, DoubleUtils>;

  //  system->m_q[0] = 2.;
  //  system->m_q[1] = 1.2;
  //  system->m_q[2] = 0.1;
  //  system->m_qd[3] = 5;
  system->m_q[1] = .2;
  fflush(stdout);

  if (floating_base) {
    // apply some "random" rotation
    system->m_q[0] = 0.06603363263475902;
    system->m_q[1] = 0.2764891273883223;
    system->m_q[2] = 0.2477976811032405;
    system->m_q[3] = 0.9261693317298725;
    system->m_q[6] = 2;
  }

  double dt = 1. / 1000.;
  double time = 0;
  while (sim->canSubmitCommand()) {
    double gravZ = sim->readUserDebugParameter(grav_id);
    world.set_gravity(TinyVector3<double, DoubleUtils>(0, 0, gravZ));

    {
      // system->control(dt, control);
      sim->submitProfileTiming("forwardDynamics");
      system->forward_dynamics(world.get_gravity());
      sim->submitProfileTiming("");
      PyBulletUrdfImport<double, DoubleUtils>::sync_graphics_transforms(system,
                                                                        *sim);
      system->clear_forces();
    }

    {
      sim->submitProfileTiming("integrate_q");
      system->integrate_q(dt);  //??
      sim->submitProfileTiming("");
    }

    {
      sim->submitProfileTiming("world_step");
      world.step(dt);
      fflush(stdout);
      sim->submitProfileTiming("");
      time += dt;
    }

    {
      sim->submitProfileTiming("integrate");
      system->integrate(dt);
      // system->print_state();
      sim->submitProfileTiming("");
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    sim->setGravity(btVector3(0, 0, gravZ));
  }

  sim->disconnect();
  sim2->disconnect();

  delete sim;
  delete sim2;

  return EXIT_SUCCESS;
}
