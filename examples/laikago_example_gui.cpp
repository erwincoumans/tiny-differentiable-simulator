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

#include "pybullet_urdf_import.h"
#include "tiny_urdf_to_multi_body.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <thread>
bool useLaikago = true;

#define USE_TRB
#include "tiny_double_utils.h"

bool floating_base = true;

#include "pybullet_visualizer_api.h"
#include "tiny_file_utils.h"

typedef PyBulletVisualizerAPI VisualizerAPI;

double knee_angle = -0.5;
double abduction_angle = 0.2;

double initial_poses[] = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};

typedef ::TinyVector3<double, DoubleUtils> dvec3;
typedef ::TinyMatrix3x3<double, DoubleUtils> dmat3;

static VisualizerAPI* gSim = 0;
void MyTinySubmitProfileTiming3(const std::string& profile_name) {
  if (gSim) {
    gSim->submitProfileTiming(profile_name);
  }
}

int main(int argc, char* argv[]) {
  double dt = useLaikago ? 1. / 1000 : 1. / 240.;

  // btVector3 laikago_initial_pos(0, 0.2, 1.65);
  btVector3 laikago_initial_pos =
      useLaikago ? btVector3(0, 0, .55) : btVector3(0, .2, 1.65);
  // btVector3 laikago_initial_pos(-3, 2, .65);
  // btVector3 laikago_initial_pos(2, 0, .65);
  btQuaternion laikago_initial_orn(0, 0, 0, 1);
  // laikago_initial_orn.setEulerZYX(-0.1, 0.1, 0);
  // laikago_initial_orn.setEulerZYX(0.7, 0, 0);
  double initialXvel = 0;
  btVector3 initialAngVel(0, 0, 0);

  std::string connection_mode = "gui";

  std::string plane_urdf_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_urdf_filename);

  char path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_urdf_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;

  printf("search_path=%s\n", search_path.c_str());
  VisualizerAPI* sim2 = new VisualizerAPI();
  bool isConnected2 = sim2->connect(eCONNECT_DIRECT);
  sim2->setAdditionalSearchPath(search_path.c_str());

  VisualizerAPI* sim = new VisualizerAPI();

  printf("mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  bool isConnected = sim->connect(mode);
  gSim = sim;
  sim->setTimeOut(1e30);
  sim2->setTimeOut(1e30);
  sim->setAdditionalSearchPath(search_path.c_str());
  int logId = sim->startStateLogging(STATE_LOGGING_PROFILE_TIMINGS,
                                     "/tmp/laikago_timing.json");

  if (!isConnected) {
    printf("Cannot connect\n");
    return -1;
  }

  sim->setTimeOut(1e30);
  sim->resetSimulation();

  int vidLogId = -1;

  int rotateCamera = 0;

  TinyWorld<double, DoubleUtils> world;
  world.m_profileTimingFunc = MyTinySubmitProfileTiming3;
  typedef ::TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

  std::vector<TinyRigidBody<double, DoubleUtils>*> bodies;
  std::vector<int> visuals;

  std::vector<TinyMultiBody<double, DoubleUtils>*> mbbodies;
  std::vector<int> paramUids;

  int grav_id = sim->addUserDebugParameter("gravity", -10, 0, 0);
  int kp_id = sim->addUserDebugParameter("kp", 0, 200, 150);
  int kd_id = sim->addUserDebugParameter("kd", 0, 13, 3.);
  int force_id = sim->addUserDebugParameter("max force", 0, 1500, 550);

#ifdef USE_TRB
  if (useLaikago) {
    TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();
    int robotId = sim2->loadURDF("plane_implicit.urdf");
    TinyUrdfStructures<double, DoubleUtils> urdf_data;
    PyBulletUrdfImport<double, DoubleUtils>::extract_urdf_structs(
        urdf_data, robotId, *sim2, *sim);
    TinyUrdfToMultiBody<double, DoubleUtils>::convert_to_multi_body(urdf_data,
                                                                    world, *mb);
    mb->initialize();
  }
  if (!useLaikago) {
    TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();
    int robotId = sim2->loadURDF("sphere2.urdf");
    TinyUrdfStructures<double, DoubleUtils> urdf_data;
    PyBulletUrdfImport<double, DoubleUtils>::extract_urdf_structs(urdf_data, 0,
                                                                  *sim2, *sim);
    TinyUrdfToMultiBody<double, DoubleUtils>::convert_to_multi_body(urdf_data,
                                                                    world, *mb);
    mb->initialize();
  }

  TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();
  {
    // int robotId = sim2->loadURDF("sphere8cube.urdf");// laikago /
    // laikago_toes_zup.urdf");// sphere8cube.urdf");//pendulum2.urdf");//
    // laikago / laikago_toes_zup.urdf"); int robotId =
    // sim2->loadURDF("pendulum2.urdf");// "laikago/laikago_toes_zup.urdf");
    int robotId = -1;
    if (useLaikago) {
      robotId = sim2->loadURDF("laikago/laikago_toes_zup.urdf");
    } else {
      robotId = sim2->loadURDF("sphere2.urdf");
    }

    TinyUrdfStructures<double, DoubleUtils> urdf_data;
    PyBulletUrdfImport<double, DoubleUtils>::extract_urdf_structs(
        urdf_data, robotId, *sim2, *sim);
    TinyUrdfToMultiBody<double, DoubleUtils>::convert_to_multi_body(urdf_data,
                                                                    world, *mb);

    mb->m_base_X_world.m_translation.setValue(
        laikago_initial_pos[0], laikago_initial_pos[1], laikago_initial_pos[2]);
    mb->m_base_X_world.m_rotation.setRotation(
        TinyQuaternion<double, DoubleUtils>(
            laikago_initial_orn[0], laikago_initial_orn[1],
            laikago_initial_orn[2], laikago_initial_orn[3]));
    mb->m_isFloating = floating_base;
    mb->m_profileTimingFunc = MyTinySubmitProfileTiming3;

    int pose_index = 0;
    for (int i = 0; i < sim2->getNumJoints(robotId); i++) {
      b3JointInfo jointInfo;
      sim2->getJointInfo(robotId, i, &jointInfo);
      if (jointInfo.m_jointType != eFixedType) {
        paramUids.push_back(sim->addUserDebugParameter(
            jointInfo.m_jointName, -4, 4, initial_poses[pose_index++]));
      }
    }
    mbbodies.push_back(mb);
    mb->initialize();
    // mb->m_q[5] = 3;
    int start_index = 0;
    if (floating_base) {
      start_index = 7;
      mb->m_q[0] = laikago_initial_orn[0];
      mb->m_q[1] = laikago_initial_orn[1];
      mb->m_q[2] = laikago_initial_orn[2];
      mb->m_q[3] = laikago_initial_orn[3];

      mb->m_q[4] = laikago_initial_pos[0];
      mb->m_q[5] = laikago_initial_pos[1];
      mb->m_q[6] = laikago_initial_pos[2];

      mb->m_qd[0] = initialAngVel[0];
      mb->m_qd[1] = initialAngVel[1];
      mb->m_qd[2] = initialAngVel[2];
      if (useLaikago) {
        mb->m_qd[3] = initialXvel;
      }
    }
    if (mb->m_q.size() >= 12) {
      for (int cc = 0; cc < 12; cc++) {
        mb->m_q[start_index + cc] = initial_poses[cc];
      }
    }
    // mb->m_q[7] = -1.57/2.;
  }
  // mb->m_profileTimingFunc = MyTinySubmitProfileTiming3;
  // sim->removeBody(robotId);
#endif  // USE_TRB

  sim->setTimeStep(dt);
  while (sim->isConnected()) {
    sim->submitProfileTiming("loop");
    {
      sim->submitProfileTiming("sleep_for");
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      sim->submitProfileTiming("");
    }
    double gravZ = sim->readUserDebugParameter(grav_id);
    sim->setGravity(btVector3(0, 0, gravZ));
    double kp = sim->readUserDebugParameter(kp_id);
    double kd = sim->readUserDebugParameter(kd_id);
    double max_force = sim->readUserDebugParameter(force_id);
    int param_index = 0;

#ifdef USE_TRB

    // while (sim->canSubmitCommand())
    int num_qd_dofs = mb->dof_qd();

    // while (sim->canSubmitCommand())  // for (int i = 0; i < 3000; i++)
    {
      double gravZ = sim->readUserDebugParameter(grav_id);
      world.set_gravity(TinyVector3<double, DoubleUtils>(0, 0, gravZ));
      {
        sim->submitProfileTiming("forward_kinematics");
        mb->forward_kinematics();
        sim->submitProfileTiming("");
      }

      if (1) {
        sim->submitProfileTiming("read _params");
        // use PD controller to compute tau
        int qd_offset = mb->m_isFloating ? 6 : 0;
        int q_offset = mb->m_isFloating ? 7 : 0;
        int num_targets = mb->m_tau.size() - qd_offset;
        std::vector<double> q_targets;
        q_targets.resize(mb->m_tau.size());

        double kp = sim->readUserDebugParameter(kp_id);
        double kd = sim->readUserDebugParameter(kd_id);
        double max_force = sim->readUserDebugParameter(force_id);
        int param_index = 0;

        for (int i = 0; i < mb->m_tau.size(); i++) {
          mb->m_tau[i] = 0;
        }

        int tau_index = 0;
        for (int i = 0; i < mb->m_links.size(); i++) {
          if (mb->m_links[i].m_joint_type != JOINT_FIXED) {
            double q_desired =
                sim->readUserDebugParameter(paramUids[param_index]);
            double q_actual = mb->m_q[q_offset];
            double qd_actual = mb->m_qd[qd_offset];
            double position_error = (q_desired - q_actual);
            double desired_velocity = 0;
            btScalar velocity_error = (desired_velocity - qd_actual);
            btScalar force = kp * position_error + kd * velocity_error;
            btClamp(force, -max_force, max_force);
            mb->m_tau[tau_index] = force;
            q_offset++;
            qd_offset++;
            param_index++;
            tau_index++;
          }
        }
        sim->submitProfileTiming("");
        printf("-------------------\n");
        for (int i = 0; i < mb->m_tau.size(); i++) {
          printf("mb->m_tau[%d]=%f\n", i, mb->m_tau[i]);
        }
      }

      {
        sim->submitProfileTiming("forwardDynamics");
        mb->forward_dynamics(world.get_gravity());
        sim->submitProfileTiming("");
      }

      {
        sim->submitProfileTiming("integrate_q");
        mb->integrate_q(dt);  //??
        sim->submitProfileTiming("");
      }

      {
        sim->submitProfileTiming("world_step");
        world.step(dt);
        sim->submitProfileTiming("");
      }

      {
        sim->submitProfileTiming("integrate");
        mb->integrate(dt);
        // printf("mb qdd:%f,%f,%f\n", mb->m_qdd[0], mb->m_qdd[1],
        // mb->m_qdd[2]); printf("q: [%.3f %.3f] \tqd: [%.3f %.3f]\n", q[0],
        // q[1], qd[0],
        for (int i = 0; i < mb->m_q.size(); i++) {
          printf("mb->m_q[%d] = %f\n", i, mb->m_q[i]);
        }
        sim->submitProfileTiming("");
      }
      if (1) {
        sim->submitProfileTiming("sync graphics");

        // sync physics to visual transforms
        {
          for (int b = 0; b < mbbodies.size(); b++) {
            const TinyMultiBody<double, DoubleUtils>* body = mbbodies[b];
            PyBulletUrdfImport<double, DoubleUtils>::sync_graphics_transforms(
                body, *sim);
          }
        }
        sim->submitProfileTiming("");
      }
    }
#endif  // USE_TRB
    {
      sim->submitProfileTiming("get_keyboard_events");
      b3KeyboardEventsData keyEvents;
      sim->getKeyboardEvents(&keyEvents);
      if (keyEvents.m_numKeyboardEvents) {
        for (int i = 0; i < keyEvents.m_numKeyboardEvents; i++) {
          b3KeyboardEvent& e = keyEvents.m_keyboardEvents[i];

          if (e.m_keyCode == 'r' && e.m_keyState & eButtonTriggered) {
            rotateCamera = 1 - rotateCamera;
          }
        }
      }
      sim->submitProfileTiming("");
    }

    if (rotateCamera) {
      sim->submitProfileTiming("rotateCamera");
      static double yaw = 0;
      double distance = 1;
      yaw += 0.1;
      btVector3 basePos(0, 0, 0);
      btQuaternion baseOrn(0, 0, 0, 1);
      sim->resetDebugVisualizerCamera(distance, -20, yaw, basePos);
      sim->submitProfileTiming("");
    }
    sim->submitProfileTiming("");
  }

  sim->stopStateLogging(logId);
  printf("sim->disconnect\n");

  sim->disconnect();

  printf("delete sim\n");
  delete sim;

  printf("exit\n");
}
