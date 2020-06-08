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

#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for

#include "LinearMath/btQuaternion.h"
#include "fix64_scalar.h"
#include "pendulum.h"
#include "pybullet_visualizer_api.h"
#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_multi_body.h"
#include "tiny_rigid_body.h"
#include "tiny_world.h"

int main(int argc, char* argv[]) {
  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);

  char path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;

  std::string connection_mode = "gui";

  // Set NaN trap
  feenableexcept(FE_INVALID | FE_OVERFLOW);

  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI* visualizer = new VisualizerAPI();
  printf("mode=%s\n", (char*)connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  visualizer->connect(mode);
  visualizer->setAdditionalSearchPath(search_path);

  if (visualizer->canSubmitCommand()) {
    visualizer->resetSimulation();
  }
  TinyWorld<double, DoubleUtils> world;

  typedef TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

  std::vector<TinyRigidBody<double, DoubleUtils>*> bodies;
  std::vector<int> visuals;

  std::vector<TinyMultiBody<double, DoubleUtils>*> mbbodies;
  std::vector<int> mbvisuals;

  TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();
  init_compound_pendulum<double, DoubleUtils>(*mb, world, 5);
  mbbodies.push_back(mb);
  if (visualizer->canSubmitCommand()) {
    for (int i = 0; i < mb->m_links.size(); i++) {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
      // apply some linear joint damping
      mb->m_links[i].m_damping = 5.;
    }
  }

  std::vector<double> q(mb->dof(), DoubleUtils::zero());
  std::vector<double> qd(mb->dof_qd(), DoubleUtils::zero());
  std::vector<double> tau(mb->dof_qd(), DoubleUtils::zero());
  std::vector<double> qdd(mb->dof_qd(), DoubleUtils::zero());

  TinyVector3<double, DoubleUtils> gravity(0., 0., -9.81);

  TinyMatrixXxX<double, DoubleUtils> M(mb->m_links.size(), mb->m_links.size());

  double dt = 1. / 240.;
  while (visualizer->isConnected()) {
    // mb->clear_forces();
    mb->forward_kinematics(q, qd);

    { world.step(dt); }

    { mb->forward_dynamics(q, qd, tau, gravity, qdd); }

    {
      mb->integrate(q, qd, qdd, dt);
      // printf("q: [%.3f %.3f] \tqd: [%.3f %.3f]\n", q[0], q[1], qd[0], qd[1]);
      mb->mass_matrix(q, &M);
      M.print("M");
      if (qd[0] < -1e4) {
        assert(0);
      }
    }
    if (visualizer->canSubmitCommand()) {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      // sync transforms
      int visual_index = 0;
      if (!mbvisuals.empty()) {
        for (int b = 0; b < mbbodies.size(); b++) {
          for (int l = 0; l < mbbodies[b]->m_links.size(); l++) {
            const TinyMultiBody<double, DoubleUtils>* body = mbbodies[b];
            if (body->m_links[l].m_X_visuals.empty()) continue;

            int sphereId = mbvisuals[visual_index++];

            TinyQuaternion<double, DoubleUtils> rot;
            const TinySpatialTransform<double, DoubleUtils>& geom_X_world =
                body->m_links[l].m_X_world * body->m_links[l].m_X_visuals[0];
            btVector3 base_pos(geom_X_world.m_translation.getX(),
                               geom_X_world.m_translation.getY(),
                               geom_X_world.m_translation.getZ());
            geom_X_world.m_rotation.getRotation(rot);
            btQuaternion base_orn(rot.getX(), rot.getY(), rot.getZ(),
                                  rot.getW());
            visualizer->resetBasePositionAndOrientation(sphereId, base_pos,
                                                        base_orn);
          }
        }
      }
    }
  }

  visualizer->disconnect();
  delete visualizer;

  return 0;
}
