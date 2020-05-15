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

#include "tiny_double_utils.h"
#include "tiny_world.h"

int main(int argc, char* argv[]) {
  typedef ::TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

  double mass = 1.0;

  TinyWorld<double, DoubleUtils> world;
  double radius = 0.5;
  const TinyGeometry<double, DoubleUtils>* geom = world.create_sphere(radius);
  const TinyRigidBody<double, DoubleUtils>* body =
      world.create_rigid_body(mass, geom);
  double dt = 1. / 60.;
  for (int i = 0; i < 100; i++) {
    world.step(dt);
    printf("pos=%f,%f,%f\n", body->m_world_pose.m_position.getX(),
           body->m_world_pose.m_position.getY(),
           body->m_world_pose.m_position.getZ());
    // btQuaternion base_orn(body->m_world_pose.m_orientation.getX(),
    //                      body->m_world_pose.m_orientation.getY(),
    //                      body->m_world_pose.m_orientation.getZ(),
    //                      body->m_world_pose.m_orientation.getW());
  }
  return 0;
}
