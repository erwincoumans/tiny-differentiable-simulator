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

#include "pendulum.h"
int logId = -1;
#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for
#include "assert.h"
#include "fix64_scalar.h"
#include "tiny_dual.h"
#include "tiny_matrix3x3.h"
#include "tiny_quaternion.h"
#include "tiny_vector3.h"
#include "tiny_world.h"

#include "pybullet_visualizer_api.h"
#include "tiny_double_utils.h"
#include "tiny_dual_double_utils.h"
#include "tiny_multi_body.h"
#include "tiny_pose.h"
#include "tiny_rigid_body.h"

int main(int argc, char *argv[]) {
  const std::string search_path =
      "/home/eric/bullet3/examples/pybullet/gym/pybullet_data";
  std::string connection_mode = "gui";

  // Set NaN trap
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI *visualizer = new VisualizerAPI();
  printf("mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  visualizer->connect(mode);
  visualizer->setAdditionalSearchPath(search_path);

  if (visualizer->canSubmitCommand()) {
    visualizer->resetSimulation();
  }

  TinyWorld<double, DoubleUtils> world;
  // world.m_profileTimingFunc = MyTinySubmitProfileTiming;
  typedef TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

  std::vector<TinyRigidBody<double, DoubleUtils> *> bodies;
  std::vector<int> visuals;

  std::vector<TinyMultiBody<double, DoubleUtils> *> mbbodies;
  std::vector<int> mbvisuals;

  TinyMultiBody<double, DoubleUtils> *mb = world.create_multi_body();
  init_compound_pendulum<double, DoubleUtils>(*mb, world);
  mbbodies.push_back(mb);
  if (visualizer->canSubmitCommand()) {
    int sphereId = visualizer->loadURDF("sphere_small.urdf");
    mbvisuals.push_back(sphereId);
    sphereId = visualizer->loadURDF("sphere_small.urdf");
    mbvisuals.push_back(sphereId);
  }

  // initial conditions
  double q1 = 0., q2 = 0.;
  double qd1 = 0, qd2 = 0;
  double g = 9.81;
  double m1 = 1, m2 = 1;
  double l1 = 0.5, l2 = 0.5;

  // compute trajectory
  double dt = 1. / 240.;
  int steps = 5000;
  std::vector<double> q1s, q2s, qd1s, qd2s;
  double_pendulum_trajectory<double, DoubleUtils>(
      l1, l2, m1, m2, q1, q2, qd1, qd2, g, dt, steps, &q1s, &q2s, &qd1s, &qd2s);

  std::vector<double> q(2), qd(2);

  for (int t = 0; t < steps; ++t) {
    {
      world.step(dt);
    }

    {
      q[0] = q1s[t];
      q[1] = q2s[t];
      qd[0] = qd1s[t];
      qd[1] = qd2s[t];
      printf("q: [%.3f %.3f] \tqd: [%.3f %.3f]\n", q[0], q[1], qd[0], qd[1]);
      mb->forward_kinematics(q, qd);
    }

    if (1) {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      // sync transforms
      int visual_index = 0;
      if (!mbvisuals.empty()) {
        for (int b = 0; b < mbbodies.size(); b++) {
          for (int l = 0; l < mbbodies[b]->m_links.size(); l++) {
            const TinyMultiBody<double, DoubleUtils> *body = mbbodies[b];
            if (body->m_links[l].m_X_visuals.empty()) continue;

            int sphereId = mbvisuals[visual_index++];

            TinyQuaternion<double, DoubleUtils> rot;
            const TinySpatialTransform<double, DoubleUtils> geom_X_world =
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

  if (logId >= 0) {
    visualizer->stopStateLogging(logId);
  }

  return 0;
}
