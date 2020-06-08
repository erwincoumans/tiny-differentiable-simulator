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

#include "LinearMath/btQuaternion.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/PhysicsDirectC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "assert.h"
#include "tiny_world.h"
#include "xarm.h"
// DoubleScalar
// FixedPointScalar
#include "fix64_scalar.h"
#include "tiny_dual.h"
#include "tiny_matrix3x3.h"
#include "tiny_quaternion.h"
#include "tiny_vector3.h"
typedef TinyDual<double> TinyDualDouble;
#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for

#include "tiny_double_utils.h"
#include "tiny_dual_double_utils.h"
#include "tiny_multi_body.h"
#include "tiny_pose.h"
#include "tiny_rigid_body.h"

typedef TinyVector3<TinyDualDouble, TinyDualDoubleUtils> tinydualvec3;
typedef TinyVector3<double, DoubleUtils> dvec3;
typedef TinyVector3<Fix64Scalar, Fix64Scalar> fvec;
typedef TinyMatrix3x3<double, DoubleUtils> dmat3;

b3RobotSimulatorClientAPI_NoDirect visualizer;

void MyTinySubmitProfileTiming(const std::string& profileName) {
  visualizer.submitProfileTiming(profileName);
}

int main(int argc, char* argv[]) {
  Fix64Scalar x = -Fix64Scalar::one() / Fix64Scalar::ten();
  Fix64Scalar res = Fix64Scalar::sin1(x);

  // b3PhysicsClientHandle sm = b3ConnectPhysicsDirect();
  b3PhysicsClientHandle sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
  b3RobotSimulatorClientAPI_InternalData data;
  data.m_physicsClientHandle = sm;
  data.m_guiHelper = 0;
  visualizer.setInternalData(&data);
  if (visualizer.canSubmitCommand()) {
    int logId = visualizer.startStateLogging(STATE_LOGGING_PROFILE_TIMINGS,
                                             "/tmp/fileName.json");
    visualizer.resetSimulation();

    {
      TinyWorld<double, DoubleUtils> world;
      // world.m_profileTimingFunc = MyTinySubmitProfileTiming;
      typedef TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

      {
        double mass = 0.0;
        visualizer.loadURDF("plane.urdf");
        const TinyGeometry<double, DoubleUtils>* geom = world.create_plane();
        TinyRigidBody<double, DoubleUtils>* body =
            world.create_rigid_body(mass, geom);
      }
      std::vector<TinyRigidBody<double, DoubleUtils>*> bodies;
      std::vector<int> visuals;

      TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();
      init_xarm6<double, DoubleUtils>(*mb);

      {
        int sphereId = visualizer.loadURDF("sphere2.urdf");
        double mass = 0.0;
        double radius = 0.5;
        const TinyGeometry<double, DoubleUtils>* geom =
            world.create_sphere(radius);
        TinyRigidBody<double, DoubleUtils>* body =
            world.create_rigid_body(mass, geom);
        body->m_world_pose.m_position =
            TinyVector3<double, DoubleUtils>::create(0, 0, 0.5);
        bodies.push_back(body);
        visuals.push_back(sphereId);
      }
      {
        int sphereId = visualizer.loadURDF("sphere2.urdf");
        double mass = 1.0;

        double radius = 0.5;
        const TinyGeometry<double, DoubleUtils>* geom =
            world.create_sphere(radius);
        TinyRigidBody<double, DoubleUtils>* body =
            world.create_rigid_body(mass, geom);
        body->m_world_pose.m_position =
            TinyVector3<double, DoubleUtils>::create(0, 0.22, 1.5);
        bodies.push_back(body);
        visuals.push_back(sphereId);
      }

      std::vector<double> q;
      std::vector<double> qd;
      std::vector<double> tau;
      std::vector<double> qdd;
      TinyVector3<double, DoubleUtils> gravity(0., 0., -9.81);

      for (int i = 0; i < mb->m_links.size(); i++) {
        q.push_back(DoubleUtils::fraction(1, 10) +
                    DoubleUtils::fraction(1, 10) * DoubleUtils::fraction(i, 1));
        qd.push_back(DoubleUtils::fraction(3, 10) +
                     DoubleUtils::fraction(1, 10) *
                         DoubleUtils::fraction(i, 1));
        tau.push_back(DoubleUtils::zero());
        qdd.push_back(DoubleUtils::zero());
      }

      double dt = 1. / 60.;
      for (int i = 0; i < 300; i++) {
        {
          visualizer.submitProfileTiming("world.step");
          world.step(dt);
          visualizer.submitProfileTiming("");
        }
        {
            // visualizer.submitProfileTiming("xarm_fk");
            // mb->forwardKinematics(q, qd);
            // visualizer.submitProfileTiming("");
        }

        {
          visualizer.submitProfileTiming("xarm_aba");
          mb->forward_dynamics(q, qd, tau, gravity, qdd);
          visualizer.submitProfileTiming("");
        }
        {
          visualizer.submitProfileTiming("xarm_aba");
          mb->integrate(q, qd, qdd, dt);
          visualizer.submitProfileTiming("");
        }
        if (1) {
          std::this_thread::sleep_for(std::chrono::duration<double>(dt));
          // sync transforms
          for (int b = 0; b < bodies.size(); b++) {
            const TinyRigidBody<double, DoubleUtils>* body = bodies[b];
            int sphereId = visuals[b];
            btVector3 base_pos(body->m_world_pose.m_position.getX(),
                               body->m_world_pose.m_position.getY(),
                               body->m_world_pose.m_position.getZ());
            btQuaternion base_orn(body->m_world_pose.m_orientation.getX(),
                                  body->m_world_pose.m_orientation.getY(),
                                  body->m_world_pose.m_orientation.getZ(),
                                  body->m_world_pose.m_orientation.getW());
            visualizer.resetBasePositionAndOrientation(sphereId, base_pos,
                                                       base_orn);
          }
        }
      }
    }
    visualizer.stopStateLogging(logId);
  }

  return 0;
}
