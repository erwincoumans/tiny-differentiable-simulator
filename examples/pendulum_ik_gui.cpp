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

#include <stdio.h>

#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for

#include "LinearMath/btQuaternion.h"
#include "fix64_scalar.h"
#include "pendulum.h"
#include "pybullet_visualizer_api.h"
#include "tiny_double_utils.h"
#include "tiny_dual.h"
#include "tiny_dual_double_utils.h"
#include "tiny_eigen_helper.h"
#include "tiny_file_utils.h"
#include "tiny_multi_body.h"
#include "tiny_rigid_body.h"
#include "tiny_vector3.h"
#include "tiny_world.h"

int main(int argc, char* argv[]) {
  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);

  char path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;

  std::string connection_mode = "gui";

  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI* visualizer = new VisualizerAPI;
  printf("mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  visualizer->connect(mode);
  visualizer->setAdditionalSearchPath(search_path);

  typedef TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

  visualizer->resetSimulation();
  TinyWorld<double, DoubleUtils> world;

  std::vector<TinyRigidBody<double, DoubleUtils>*> bodies;
  std::vector<int> visuals;

  std::vector<TinyMultiBody<double, DoubleUtils>*> mbbodies;
  std::vector<int> mbvisuals;

  TinyVector3<double, DoubleUtils> target(0., 0.5, 1.5);
  int target_id = -1;  // ID of URDF sphere for the target

  TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();
  init_compound_pendulum<double, DoubleUtils>(*mb, world, 5);
  mb->m_isFloating = false;
  mb->initialize();
  mbbodies.push_back(mb);

  if (visualizer->canSubmitCommand()) {
    for (int i = 0; i < mb->m_links.size(); i++) {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
    }

    b3RobotSimulatorLoadUrdfFileArgs args;
    args.m_startPosition.setX(target.getX());
    args.m_startPosition.setY(target.getY());
    args.m_startPosition.setZ(target.getZ());
    target_id = visualizer->loadURDF("sphere_small.urdf", args);
    b3RobotSimulatorChangeVisualShapeArgs vargs;
    vargs.m_objectUniqueId = target_id;
    vargs.m_hasRgbaColor = true;
    vargs.m_rgbaColor = btVector4(1, 0, 0, 1);
    visualizer->changeVisualShape(vargs);
  }

  std::vector<double> q;
  std::vector<double> qd;
  std::vector<double> tau;
  std::vector<double> qdd;

  double update_rate = 5e-2;

  for (int i = 0; i < mb->m_links.size(); i++) {
    q.push_back(DoubleUtils::zero());
    qd.push_back(DoubleUtils::zero());
    qdd.push_back(DoubleUtils::zero());
    tau.push_back(DoubleUtils::zero());
  }

  double dt = 1. / 240.;
  double time = 0;
  while (visualizer->isConnected()) {
    mb->forward_kinematics(q, qd);

    const int ee = mb->m_dof - 1;  // end-effector link index

    TinyVector3<double, DoubleUtils> geom_offset;
    geom_offset.set_zero();
    TinySpatialTransform<double, DoubleUtils> ee_body_tf;
    ee_body_tf.set_identity();
    if (!mb->m_links[ee].m_X_visuals.empty()) {
      // consider the offset of the bob attached to the end-effector
      ee_body_tf = mb->m_links[ee].m_X_visuals[0];
      geom_offset = ee_body_tf.m_translation;
    }
    auto jac = mb->point_jacobian(q, ee, geom_offset);
    jac.print("End-effector Jacobian");
    auto ee_tf = mb->m_links[ee].m_X_world * ee_body_tf;

    TinyVector3<double, DoubleUtils> error = target - ee_tf.m_translation;
    error.print("End-effector error");

    auto pi = helper::pseudo_inverse(jac);
    pi.print("pseudo inverse:");

    // Jacobian transpose IK
    //           auto delta_q = jac.mul_transpose(error);

    // Jacobian pseudo-inverse IK
    auto delta_q = pi * error;
    printf("delta q:");
    for (int i = 0; i < mb->m_links.size(); i++) {
      q[i] += update_rate * delta_q[i];
      printf(" %.3f", delta_q[i]);
    }
    double error_norm = error.length();
    printf("\t\tError: %.3f\n", error_norm);

    if (error_norm < 5e-2) {
      printf("+++ Choosing new target +++\n");
      // pick a new random target location
      double angle = -sin(time) * 0.8;
      target.setY(1.5 * sin(angle));
      target.setZ(1.5 * cos(angle));
      btVector3 target_pos(target.getX(), target.getY(), target.getZ());
      btQuaternion quat;
      visualizer->resetBasePositionAndOrientation(target_id, target_pos, quat);
      time += dt;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    // sync transforms
    int visual_index = 0;
    if (!mbvisuals.empty()) {
      for (int b = 0; b < mbbodies.size(); b++) {
        for (int l = 0; l < mbbodies[b]->m_links.size(); l++) {
          const TinyMultiBody<double, DoubleUtils>* body = mbbodies[b];
          int sphereId = mbvisuals[visual_index++];

          TinyQuaternion<double, DoubleUtils> rot;
          const TinySpatialTransform<double, DoubleUtils>& geom_X_world =
              body->m_links[l].m_X_world;
          btVector3 base_pos(geom_X_world.m_translation.getX(),
                             geom_X_world.m_translation.getY(),
                             geom_X_world.m_translation.getZ());
          geom_X_world.m_rotation.getRotation(rot);
          btQuaternion base_orn(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
          visualizer->resetBasePositionAndOrientation(sphereId, base_pos,
                                                      base_orn);
        }
      }
    }
  }

  visualizer->disconnect();
  delete visualizer;

  return 0;
}
