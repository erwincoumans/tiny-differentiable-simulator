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

#include "urdf/pybullet_urdf_import.hpp"
#include "urdf/urdf_to_multi_body.hpp"

#include <assert.h>
#include <stdio.h>

#include <chrono>
#include <thread>

#include "Utils/b3Clock.h"
#include "math/tiny/tiny_double_utils.h"
#include "tiny_inverse_kinematics.h"
#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#include "mb_constraint_solver_spring.hpp"
#include "mb_constraint_solver.hpp"

#include "tiny_pd_control.h"

#include "visualizer/pybullet/pybullet_visualizer_api.h"
#include "utils/file_utils.hpp"
#include "pybullet_visual_instance_generator.h"
#include "math/tiny/tiny_algebra.hpp"

using namespace TINY;
using namespace tds;
#undef max
#undef min

typedef PyBulletVisualizerAPI VisualizerAPI;

int make_sphere(VisualizerAPI* sim, float r = 1, float g = 0.6f, float b = 0,
                float a = 0.7f) {
  int sphere_id = sim->loadURDF("sphere_small.urdf");
  b3RobotSimulatorChangeVisualShapeArgs vargs;
  vargs.m_objectUniqueId = sphere_id;
  vargs.m_hasRgbaColor = true;
  vargs.m_rgbaColor = btVector4(r, g, b, a);
  sim->changeVisualShape(vargs);
  return sphere_id;
}

void print(const std::vector<double>& v) {
  for (std::size_t i = 0; i < v.size(); ++i) {
    printf("%.3f", v[i]);
    if (i < v.size() - 1) {
      printf(", ");
    }
  }
  printf("\n");
}

void update_position(VisualizerAPI* sim, int object_id, double x, double y,
                     double z) {
  btVector3 pos(x, y, z);
  btQuaternion orn(0, 0, 0, 1);
  sim->resetBasePositionAndOrientation(object_id, pos, orn);
}

template <typename Scalar, typename Util>
void update_position(VisualizerAPI* sim, int object_id,
                     const TinyVector3<Scalar, Util>& pos) {
  update_position(sim, object_id, Util::getDouble(pos.m_x),
                  Util::getDouble(pos.m_y), Util::getDouble(pos.m_z));
}

/**
 * Pattern generator for a symmetric walking gait (trotting).
 * The diagonal feet move in the same way (symmetrically).
 * Each step is a positive half-period of a sine curve.
 */
struct GaitGenerator {
  /**
   * Period length in seconds.
   */
  double period_length{1.};

  /**
   * Extent of a single step along x.
   */
  double step_length{0.2};
  /**
   * Extent of a single step along z.
   */
  double step_height{0.1};

  /**
   * Time step in seconds.
   */
  double dt;

  /**
   * Ratio of period length (in [0,1]) for the lifting phase of each foot.
   * Must be less than 0.5.
   */
  double lift_ratio{0.3};

  typedef TinyVector3<double, DoubleUtils> Vector3;

  explicit GaitGenerator(double dt) : dt(dt) {}

  /**
   * Computes targets (in world coordinates) for each foot at time t.
   * @param time Time in seconds.
   * @param fr 3D position of front-right foot.
   * @param fl 3D position of front-left foot.
   * @param br 3D position of back-right foot.
   * @param bl 3D position of back-left foot.
   */
  virtual void compute(double time, Vector3& fr, Vector3& fl, Vector3& br,
                       Vector3& bl) const {
    assert(lift_ratio < 0.5);

    double period = std::fmod(time, period_length);
    double rel_period = period / period_length;
    double rel_step = rel_period > 0.5 ? rel_period - 0.5 : rel_period;

    double z, dx;
    if (rel_step <= lift_ratio) {
        dx = 0;// step_length / (lift_ratio * period_length) * dt;
      z = step_height * std::max(0., std::sin(rel_step * M_PI / lift_ratio));
    } else {
      dx = 0;
      z = 0;
    }
    if (rel_period <= 0.5) {
      // lift FR, BL
      fr.m_z = z;
      bl.m_z = z;
      fr.m_x += dx;
      bl.m_x += dx;
    } else {
      // lift FL, BR
      fl.m_z = z;
      br.m_z = z;
      fl.m_x += dx;
      br.m_x += dx;
    }
  }
};



int main(int argc, char* argv[]) {
  double dt = 1. / 1000;

  // btVector3 laikago_initial_pos(0, 0.2, 1.65);
  btVector3 laikago_initial_pos = btVector3(2, -1, 0.45);// 1, 2, .39);
  // btVector3 laikago_initial_pos(-3, 2, .65);
  // btVector3 laikago_initial_pos(2, 0, .65);
  btQuaternion laikago_initial_orn(0, 0, 0, 1);
   //laikago_initial_orn.setEulerZYX(-0.1, 0.1, 0);
   laikago_initial_orn.setEulerZYX(1.4, 0, 0);
  double initialXvel = 0;
  btVector3 initialAngVel(0, 0, 0);
  double knee_angle = -0.5;
  double abduction_angle = 0.2;
  double initial_poses[] = {
      abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
      abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
  };

  std::string connection_mode = "gui";

  std::string laikago_filename;
  FileUtils::find_file("laikago/laikago_toes_zup.urdf", laikago_filename);

  std::string plane_filename;
  FileUtils::find_file("plane_implicit.urdf", plane_filename);

  char path[TINY_MAX_EXE_PATH_LEN];
  FileUtils::extract_path(plane_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;

  printf("search_path=%s\n", search_path.c_str());
  VisualizerAPI* sim2 = new VisualizerAPI();
  bool isConnected2 =
      sim2->connect(eCONNECT_DIRECT);  // eCONNECT_SHARED_MEMORY);
  sim2->setAdditionalSearchPath(search_path.c_str());

  VisualizerAPI* sim = new VisualizerAPI();
  printf("mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  bool isConnected = sim->connect(mode);
  sim->setTimeOut(1e30);
  sim2->setTimeOut(1e30);
  sim->setAdditionalSearchPath(search_path.c_str());
  int logId = sim->startStateLogging(STATE_LOGGING_PROFILE_TIMINGS,
                                     "/tmp/laikago_timing.json");

  if (!isConnected || !isConnected2) {
    printf("Cannot connect\n");
    return -1;
  }

  sim->setTimeOut(1e30);
  sim->resetSimulation();

  b3Clock clock;

  int rotateCamera = 0;

  World<TinyAlgebra<double, DoubleUtils> > world;
  typedef ::tds::RigidBody<TinyAlgebra<double, DoubleUtils> > TinyRigidBodyDouble;
  typedef ::TinyVector3<double, DoubleUtils> TinyVector3;
  typedef ::tds::Transform<TinyAlgebra< double,DoubleUtils> > TinySpatialTransform;

  std::vector<RigidBody<TinyAlgebra<double, DoubleUtils> >*> bodies;
  std::vector<int> visuals;

  std::vector<MultiBody<TinyAlgebra<double, DoubleUtils> >*> mbbodies;
  std::vector<int> paramUids;

  int gravY_id = sim->addUserDebugParameter("gravityY", -10, 10, 0);// -9.8);
  int gravZ_id = sim->addUserDebugParameter("gravityZ", -100, 0, -10);// -9.8);
  int kp_id = sim->addUserDebugParameter("kp", 0, 200, 150);
  int kd_id = sim->addUserDebugParameter("kd", 0, 13, 3.);
  int force_id = sim->addUserDebugParameter("max force", 0, 1500, 550);

  PyBulletInstanceGenerator gen(sim);
  {
    MultiBody<TinyAlgebra<double, DoubleUtils> >* mb = world.create_multi_body();
    int robotId = sim->loadURDF(plane_filename);
    UrdfStructures< TinyAlgebra<double, DoubleUtils> > urdf_data;
    PyBulletUrdfImport< TinyAlgebra<double, DoubleUtils> >::extract_urdf_structs(
        urdf_data, robotId, sim, sim);
    UrdfToMultiBody< TinyAlgebra<double, DoubleUtils> >::convert_to_multi_body(urdf_data,
                                                                    world, *mb, &gen);
    mb->initialize();
    sim->removeBody(robotId);
  }
  int start_index = 0;
  MultiBody< TinyAlgebra<double, DoubleUtils> >* mb = world.create_multi_body();
  {
	//don't merge fixed links, we like to have the feet indices
    //b3RobotSimulatorLoadUrdfFileArgs args;
    //args.m_flags |= URDF_MERGE_FIXED_LINKS;
    int robotId = sim->loadURDF(laikago_filename);

    UrdfStructures< TinyAlgebra<double, DoubleUtils> > urdf_data;
    PyBulletUrdfImport< TinyAlgebra<double, DoubleUtils> >::extract_urdf_structs(
        urdf_data, robotId, sim, sim);
    UrdfToMultiBody< TinyAlgebra<double, DoubleUtils> >::convert_to_multi_body(urdf_data,
                                                                    world, *mb, &gen);

    mbbodies.push_back(mb);
    mb->set_floating_base(false);
    mb->initialize();
    sim->removeBody(robotId);
    
    
    start_index = mb->is_floating() ? 7 : 0;
    
    mb->qd_[0] = initialAngVel[0];
    mb->qd_[1] = initialAngVel[1];
    mb->qd_[2] = initialAngVel[2];
    mb->qd_[3] = initialXvel;
    mb->set_position(TinyVector3(laikago_initial_pos[0], laikago_initial_pos[1], laikago_initial_pos[2]));
    mb->set_orientation(TinyQuaternion<double, DoubleUtils>(laikago_initial_orn[0], laikago_initial_orn[1], laikago_initial_orn[2], laikago_initial_orn[3]));


    if (mb->q_.size() >= 12) {
      for (int cc = 0; cc < 12; cc++) {
        mb->q_[start_index + cc] = initial_poses[cc];
      }
    }
  }
  world.default_friction = 0.1;
  world.set_mb_constraint_solver(
      //new MultiBodyConstraintSolverSpring<TinyAlgebra<double, DoubleUtils> >);
        new MultiBodyConstraintSolver<TinyAlgebra<double, DoubleUtils> >);
      
  printf("Initial state:\n");
  mb->print_state();
  auto q_ref = mb->q_;

  //std::vector<double> q_target;
  TinyVectorX q_target = mb->q_;
  // body indices of feet
  const int foot_fr = 3;// 2;
  const int foot_fl = 7;// 5;
  const int foot_br = 11;// 8;
  const int foot_bl = 15;// 11;
  const TinyVector3 foot_offset(0, 0, 0);// -0.24, -0.02);

  TinyInverseKinematics<TinyAlgebra<double, DoubleUtils>, IK_JAC_PINV> inverse_kinematics;
  // controls by how much the joint angles should be close to the initial q
  inverse_kinematics.weight_reference = 0;
  // step size
  inverse_kinematics.alpha = 0.3;
  inverse_kinematics.targets.emplace_back(foot_fr, TinyVector3::zero());
  inverse_kinematics.targets.emplace_back(foot_fl, TinyVector3::zero());
  inverse_kinematics.targets.emplace_back(foot_br, TinyVector3::zero());
  inverse_kinematics.targets.emplace_back(foot_bl, TinyVector3::zero());
  inverse_kinematics.q_reference = q_ref;// mb->q_;
  for (auto& target : inverse_kinematics.targets) {
    target.body_point = foot_offset;
  }


  int sphere_fr = make_sphere(sim, 1, 0, 0);
  int sphere_fl = make_sphere(sim, 1, 1, 0);
  int sphere_br = make_sphere(sim, 0.3f, .7f, 0);
  int sphere_bl = make_sphere(sim, 0.2f, 0.5f, 1);

  int sphere_target_fr = make_sphere(sim, 1, 0, 0);
  int sphere_target_fl = make_sphere(sim, 1, 1, 0);
  int sphere_target_br = make_sphere(sim, 0.3f, .7f, 0);
  int sphere_target_bl = make_sphere(sim, 0.2f, 0.5f, 1);


  GaitGenerator gait(dt);
  gait.step_length = 0.2;
  gait.step_height = 0.25;
  gait.lift_ratio = 0.3;
  gait.period_length = 0.5;
  int walking_start = 500;  // step number when to start walking (first settle)

  //auto* servo = new TinyServoActuator<double, DoubleUtils>(
  //    mb->dof_actuated(), 150., 3., -500., 500.);
  //mb->set>m_actuator = servo;

  sim->setTimeStep(dt);
  double time = 0;
  double zrot = 0;
  ::tds::forward_kinematics<TinyAlgebra<double, DoubleUtils> >(*mb);
  
  sim->resetDebugVisualizerCamera(1, -10, 54, laikago_initial_pos);

  for (int step = 0; sim->isConnected(); ++step) {

      if (0)
      {
          btQuaternion laikago_initial_orn(0, 0, 0, 1);
          laikago_initial_orn.setEulerZYX(zrot, 0, 0);
          //zrot += 0.001;
          mb->set_orientation(TinyQuaternion<double, DoubleUtils>(laikago_initial_orn[0], laikago_initial_orn[1], laikago_initial_orn[2], laikago_initial_orn[3]));
      }
    sim->submitProfileTiming("loop");
    {
      sim->submitProfileTiming("sleep_for");
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      sim->submitProfileTiming("");
    }
    
    {
      double gravY = sim->readUserDebugParameter(gravY_id);
      double gravZ = sim->readUserDebugParameter(gravZ_id);
      world.set_gravity(TinyVector3(0, gravY, gravZ));
      {
        sim->submitProfileTiming("forward_kinematics");
        ::tds::forward_kinematics<TinyAlgebra<double, DoubleUtils> >(*mb);
        sim->submitProfileTiming("");
      }

      {
        TinySpatialTransform base_X_world;
        std::vector<TinySpatialTransform> links_X_world;
        ::tds::forward_kinematics_q<TinyAlgebra<double, DoubleUtils >>(*mb, mb->q_, &base_X_world, &links_X_world);

        TinyVector3 foot_pos = links_X_world[foot_fr].apply(foot_offset);
        update_position(sim, sphere_fr,
                        foot_pos);
        update_position(sim, sphere_fl,
                        links_X_world[foot_fl].apply(foot_offset));
        update_position(sim, sphere_br,
                        links_X_world[foot_br].apply(foot_offset));
        update_position(sim, sphere_bl,
                        links_X_world[foot_bl].apply(foot_offset));

        update_position(sim, sphere_target_fr,
                        inverse_kinematics.targets[0].position);
        update_position(sim, sphere_target_fl,
                        inverse_kinematics.targets[1].position);
        update_position(sim, sphere_target_br,
                        inverse_kinematics.targets[2].position);
        update_position(sim, sphere_target_bl,
                        inverse_kinematics.targets[3].position);

        if (step == walking_start) 
        {
          for (int i = 0; i < inverse_kinematics.targets.size(); ++i) {
            auto& target = inverse_kinematics.targets[i];
            auto pos = links_X_world[target.link_index].apply(foot_offset);
            target.position = pos;
            // printf("Initial position of foot %i:\t %.3f\t %.3f\t %.3f\n", i,pos.m_x, pos.m_y, pos.m_z);
            target.position.m_z = 0;
          }
        } 
        else if (step > walking_start) 
        {
          gait.compute(time - walking_start * dt,
                       inverse_kinematics.targets[0].position,
                       inverse_kinematics.targets[1].position,
                       inverse_kinematics.targets[2].position,
                       inverse_kinematics.targets[3].position);

          TinyVectorX start_q = mb->q_;
          inverse_kinematics.compute(*mb, start_q, q_target);
        }
      }
      {
        double kp = sim->readUserDebugParameter(kp_id);
        double kd = sim->readUserDebugParameter(kd_id);
        double max_force = sim->readUserDebugParameter(force_id);
        double min_force = -max_force;
        std::vector<double> control(mb->dof_actuated());
        for (int i = 0; i < mb->dof_actuated(); ++i) {
          control[i] = q_target[i + start_index];
        }
         // pd control
        if (0) {
            // use PD controller to compute tau
            int qd_offset = mb->is_floating() ? 6 : 0;
            int q_offset = mb->is_floating() ? 7 : 0;
            int num_targets = mb->tau_.size() - qd_offset;
            std::vector<double> q_targets;
            q_targets.resize(mb->tau_.size());

            int param_index = 0;

            for (int i = 0; i < mb->tau_.size(); i++) {
                mb->tau_[i] = 0;
            }
            int tau_index = 0;
            int pose_index = 0;
            for (int i = 0; i < mb->links_.size(); i++) {
                if (mb->links_[i].joint_type != JOINT_FIXED) {
                    double q_desired = control[pose_index++];//initial_poses[pose_index++];// 
                    double q_actual = mb->q_[q_offset];
                    double qd_actual = mb->qd_[qd_offset];
                    double position_error = (q_desired - q_actual);
                    double desired_velocity = 0;
                    double velocity_error = (desired_velocity - qd_actual);
                    double force = kp * position_error + kd * velocity_error;

                    if (force < -max_force) force = -max_force;
                    if (force > max_force) force = max_force;
                    mb->tau_[tau_index] = force;
                    q_offset++;
                    qd_offset++;
                    param_index++;
                    tau_index++;
                }
            }
        }
        else
        {
            mb->q_ = q_target;
            mb->qd_.set_zero();
        }






      }

      if (1)
      {
          {
              sim->submitProfileTiming("forwardDynamics");
              ::tds::forward_dynamics<TinyAlgebra<double, DoubleUtils> >(*mb, world.get_gravity());
              sim->submitProfileTiming("");
              mb->clear_forces();
          }

          {
              sim->submitProfileTiming("integrate_q");
              ::tds::integrate_euler_qdd<TinyAlgebra<double, DoubleUtils> >(*mb, dt);
              sim->submitProfileTiming("");
          }

          {
              if (step % 1000 == 0) {
                  printf("Step %06d \t Time: %.3f\n", step, time);
              }
              sim->submitProfileTiming("world_step");
              world.step(dt);
              sim->submitProfileTiming("");
              
          }
          {
              sim->submitProfileTiming("integrate");
              ::tds::integrate_euler(*mb, dt);
              sim->submitProfileTiming("");
          }
      }

      time += dt;

      if (1) {
        sim->submitProfileTiming("sync graphics");

        // sync physics to visual transforms
        {
          for (int b = 0; b < mbbodies.size(); b++) {
            const MultiBody<TinyAlgebra<double, DoubleUtils>>* body = mbbodies[b];
            PyBulletUrdfImport< TinyAlgebra<double, DoubleUtils>>::sync_graphics_transforms(
                body, sim);
          }
        }
        sim->submitProfileTiming("");
      }
    }
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

#pragma clang diagnostic pop
