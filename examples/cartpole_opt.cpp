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

#include <assert.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <fenv.h>
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <thread>

#include "Utils/b3Clock.h"

#include "pybullet_visualizer_api.h"
typedef PyBulletVisualizerAPI VisualizerAPI;

#include "ceres_utils.h"
#include "pybullet_urdf_import.h"
#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_multi_body.h"
#include "tiny_urdf_to_multi_body.h"
#include "tiny_world.h"

/**
 * Direct shooting trajectory optimization problem formulated as Nonlinear
 * Least Squares problem for Ceres optimizers.
 * @tparam Steps Number of time steps, i.e. length of the trajectory.
 * @tparam ControlDim Dimensionality of the control input vector.
 * @tparam CostFunction Functional that is evaluated for a state-action pair
 * and returns the residuals.
 */
template <int Steps, int ControlDim, typename CostFunction>
struct TrajectoryOptimizationFunctional {
  static const int TrajDim = Steps * ControlDim;

  typedef ceres::Jet<double, TrajDim> Scalar;
  typedef CeresUtils<TrajDim> Utils;

  /**
   * Integration time step in seconds.
   */
  static inline Scalar m_dt{Scalar(1. / 60.)};

  static inline TinyVector3<Scalar, Utils> m_gravity{
      Utils::zero(), Utils::zero(), Scalar(-9.81)};

  static void setup(VisualizerAPI *sim, const std::string &urdf_filename,
                    bool is_floating = false) {
    // by default, apply control signals to the first N degrees of freedom
    for (int i = 0; i < ControlDim; ++i) {
      m_control_indices[i] = i;
    }
    TinyMultiBody<Scalar, Utils> *mb = m_world.create_multi_body();
    int robotId = sim->loadURDF(urdf_filename);
    TinyUrdfStructures<Scalar, Utils> urdf_data;
    PyBulletUrdfImport<Scalar, Utils>::extract_urdf_structs(urdf_data, robotId,
                                                            *sim, *sim);
    TinyUrdfToMultiBody<Scalar, Utils>::convert_to_multi_body(urdf_data,
                                                              m_world, *mb);
    mb->m_isFloating = is_floating;
    mb->initialize();
    m_system = *mb;
  }

  static ceres::Problem *problem(double *initial_trajectory = nullptr) {
    ceres::Problem *problem = new ceres::Problem;
    if (!initial_trajectory) {
      initial_trajectory = new double[TrajDim];
      for (int i = 0; i < TrajDim; ++i) {
        initial_trajectory[i] = 0.;
      }
    }
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, CostFunction::ResidualDim,
                                        TrajDim>(new CostFunctor);
    problem->AddResidualBlock(cost_function, nullptr, initial_trajectory);
    return problem;
  }

 private:
  static inline TinyWorld<Scalar, Utils> m_world;
  static inline TinyMultiBody<Scalar, Utils> m_system;
  static inline CostFunction m_cost;

  /**
   * Indices in MultiBody.m_tau where the control signal is applied.
   */
  static inline std::array<int, ControlDim> m_control_indices;

 public:
  struct CostFunctor {
    template <typename T>
    bool operator()(const T *const x, T *residual) const {
      Scalar time = Utils::zero();
      // reset world and system
      for (int i = 0; i < m_system.dof(); ++i) {
        m_system.m_q[i] = Utils::zero();
      }
      m_system.m_q[1] = Utils::pi();
      for (int i = 0; i < m_system.dof_qd(); ++i) {
        m_system.m_qd[i] = Utils::zero();
        m_system.m_qdd[i] = Utils::zero();
        m_system.m_tau[i] = Utils::zero();
      }
      for (int i = 0; i < CostFunction::ResidualDim; ++i) {
        if constexpr (std::is_same_v<T, double>) {
          residual[i] = 0.;
        } else {
          residual[i] = Utils::zero();
        }
      }
      int traj_idx = 0;
      for (int step = 0; step < Steps; ++step) {
        m_system.forward_kinematics();
        m_world.step(m_dt);
        for (int tau_idx : m_control_indices) {
          m_system.m_tau[tau_idx] = Scalar(x[traj_idx]);
          ++traj_idx;
        }
        m_system.forward_dynamics(m_gravity);
        m_system.integrate(m_dt);

        if (!m_cost(m_system, residual, step == Steps - 1)) {
          return false;
        }

        //        printf("%.3f ", Utils::getDouble(m_system.m_tau[0]));
        time += m_dt;
      }
      //      printf("\n"); fflush(stdout);
      return true;
    }
  };
};

struct CartpoleCost {
  static const int ResidualDim = 4;

  template <template <typename, typename> class System, typename T,
            typename Scalar, typename Utils>
  bool operator()(const System<Scalar, Utils> &system, T *residual,
                  bool last_step) const {
    if (!last_step) {
      // we only apply the cost on the last state of the trajectory
      residual[0] = T(0.);
      residual[1] = T(0.);
      residual[2] = T(0.);
      residual[3] = T(0.);
      return true;
    }
    if constexpr (std::is_same_v<T, double>) {
      // cart should be close to the origin
      residual[0] = Utils::getDouble(system.m_q[0] * system.m_q[0]);
      // pole should be upright (zero angle)
      residual[1] = Utils::getDouble(system.m_q[1] * system.m_q[1]);
      // cart velocity should be zero
      residual[2] = Utils::getDouble(system.m_qd[0] * system.m_qd[0]);
      // pole angular velocity should be zero
      residual[3] = Utils::getDouble(system.m_qd[1] * system.m_qd[1]);
    } else {
      // cart should be close to the origin
      residual[0] = system.m_q[0] * system.m_q[0];
      // pole should be upright (zero angle)
      residual[1] = system.m_q[1] * system.m_q[1];
      // cart velocity should be zero
      residual[2] = system.m_qd[0] * system.m_qd[0];
      // pole angular velocity should be zero
      residual[3] = system.m_qd[1] * system.m_qd[1];
    }

    return true;
  }
};

static VisualizerAPI *gSim = nullptr;
void MyTinySubmitProfileTiming3(const std::string &profile_name) {
  if (gSim) {
    gSim->submitProfileTiming(profile_name);
  }
}

int main(int argc, char *argv[]) {
  std::string connection_mode = "gui";
  std::string urdf_filename;
  TinyFileUtils::find_file("cartpole.urdf", urdf_filename);
  bool floating_base = false;
  // Set NaN trap
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

  printf("floating_base=%d\n", floating_base);
  printf("urdf_filename=%s\n", urdf_filename.c_str());
  VisualizerAPI *sim2 = new VisualizerAPI();
  bool isConnected2 = sim2->connect(eCONNECT_DIRECT);

  const int Steps = 100;

  TrajectoryOptimizationFunctional<Steps, 1, CartpoleCost> traj_opt;
  traj_opt.setup(sim2, urdf_filename);
  double controls[Steps];
  for (int i = 0; i < Steps; ++i) {
    controls[i] = 0.;
  }
  ceres::Problem *problem = traj_opt.problem(controls);
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
  printf("ceres::Solve, wait\n");
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << "\n";
  fflush(stdout);

  VisualizerAPI *sim = new VisualizerAPI();
  printf("connection_mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_SHARED_MEMORY;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "gui") mode = eCONNECT_GUI;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;
  bool isConnected = sim->connect(mode);
  gSim = sim;

  int logId = sim->startStateLogging(STATE_LOGGING_PROFILE_TIMINGS,
                                     "/tmp/laikago_timing.json");

  if (!isConnected) {
    printf("Cannot connect\n");
    return -1;
  }

  sim->setTimeOut(10);
  sim->resetSimulation();
  sim2->resetSimulation();

  b3Clock clock;
  double fixedTimeStep = 1. / 60.;

  int rotateCamera = 0;

  TinyWorld<double, DoubleUtils> world;
  world.m_profileTimingFunc = MyTinySubmitProfileTiming3;
  typedef TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

  std::vector<TinyRigidBody<double, DoubleUtils> *> bodies;
  std::vector<int> visuals;

  std::vector<TinyMultiBody<double, DoubleUtils> *> mbbodies;
  std::vector<int> paramUids;

  TinyMultiBody<double, DoubleUtils> *mb = world.create_multi_body();
  {
    int robotId = sim2->loadURDF(urdf_filename);
    TinyUrdfStructures<double, DoubleUtils> urdf_data;
    PyBulletUrdfImport<double, DoubleUtils>::extract_urdf_structs(
        urdf_data, robotId, *sim2, *sim);
    TinyUrdfToMultiBody<double, DoubleUtils>::convert_to_multi_body(urdf_data,
                                                                    world, *mb);
    if (!floating_base) mb->m_base_X_world.m_translation.setValue(0, 0, 1);
    mb->m_isFloating = floating_base;
    mbbodies.push_back(mb);
  }
  mb->initialize();

  if (floating_base) {
    // apply some "random" rotation
    mb->m_q[0] = 0.06603363263475902;
    mb->m_q[1] = 0.2764891273883223;
    mb->m_q[2] = 0.2477976811032405;
    mb->m_q[3] = 0.9261693317298725;
    mb->m_q[6] = 1;
  }
  // initial cartpole angle
  mb->m_q[1] = M_PI;

  TinyVector3<double, DoubleUtils> gravity(0., 0., -9.81);

  double dt = 1. / 60.;
  for (int step = 0; step < Steps; ++step) {
    {
      sim->submitProfileTiming("forward_kinematics");
      mb->forward_kinematics();
      sim->submitProfileTiming("");
    }

    {
      sim->submitProfileTiming("world_step");
      world.step(dt);
      sim->submitProfileTiming("");
    }

    {
      sim->submitProfileTiming("forwardDynamics");
      mb->m_tau[0] = controls[step];
      mb->forward_dynamics(gravity);
      sim->submitProfileTiming("");
    }
    {
      sim->submitProfileTiming("integrate");
      mb->integrate(dt);
      // printf("q: [%.3f %.3f] \tqd: [%.3f %.3f]\n", mb->m_q[0],
      // mb->m_q[1],
      //       mb->m_qd[0], mb->m_qd[1]);
      sim->submitProfileTiming("");
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    // sync physics to visual transforms
    {
      for (int b = 0; b < mbbodies.size(); b++) {
        const TinyMultiBody<double, DoubleUtils> *body = mbbodies[b];
        PyBulletUrdfImport<double, DoubleUtils>::sync_graphics_transforms(body,
                                                                          *sim);
      }
    }
    sim2->configureDebugVisualizer(COV_ENABLE_SINGLE_STEP_RENDERING, 1);
    b3KeyboardEventsData keyEvents;
    sim->getKeyboardEvents(&keyEvents);
    if (keyEvents.m_numKeyboardEvents) {
      for (int i = 0; i < keyEvents.m_numKeyboardEvents; i++) {
        b3KeyboardEvent &e = keyEvents.m_keyboardEvents[i];

        if (e.m_keyCode == 'r' && e.m_keyState & eButtonTriggered) {
          rotateCamera = 1 - rotateCamera;
        }
      }
    }

    if (rotateCamera) {
      static double yaw = 0;
      double distance = 1;
      yaw += 0.1;
      btVector3 basePos(0, 0, 0);
      btQuaternion baseOrn(0, 0, 0, 1);
      sim->resetDebugVisualizerCamera(distance, -20, yaw, basePos);
    }
    b3Clock::usleep(1000. * 1000. * fixedTimeStep);
  }
  sim->stopStateLogging(logId);

  sim->disconnect();
  sim2->disconnect();

  delete sim;
  delete sim2;

  return EXIT_SUCCESS;
}
