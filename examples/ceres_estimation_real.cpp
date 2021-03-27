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

#define _USE_MATH_DEFINES
#include <cmath>

#include <ceres/iteration_callback.h>
#include <ceres/types.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#include "load_ibm_data.h"
#include "utils/pendulum.hpp"
#include "visualizer/pybullet/pybullet_visualizer_api.h"
#include "utils/ceres_estimator_old.hpp"
#include "utils/file_utils.hpp"
#include "multi_body.hpp"
#include "world.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math/tiny/tiny_algebra.hpp"

#define JUST_VISUALIZE false
#define USE_PBH true
// whether the state consists of [q qd] or just q
#define STATE_INCLUDES_QD false
// whether to estimate the diagonal elements of the inertia 3x3 matrix
#define ESTIMATE_LENGTH false
#define ESTIMATE_MASS true
#define ESTIMATE_INERTIA true
#define ESTIMATE_INITIAL_VELOCITY true
#define ESTIMATE_JOINT_DAMPING false
#define ESTIMATE_TIME_STEP true
std::vector<double> start_state;
const int param_dim_length = ESTIMATE_LENGTH ? 2 : 0;
const int param_dim_mass = ESTIMATE_MASS ? 2 : 0;
const int param_dim_inertia = ESTIMATE_INERTIA ? 6 : 0;
const int param_dim_initial_vel = ESTIMATE_INITIAL_VELOCITY ? 2 : 0;
const int param_dim_joint_damping = ESTIMATE_JOINT_DAMPING ? 2 : 0;
const int param_dim_time_step = ESTIMATE_TIME_STEP ? 1 : 0;
const int param_dim = param_dim_inertia + param_dim_length + param_dim_mass +
                      param_dim_initial_vel + param_dim_joint_damping + param_dim_time_step;
#define TRUE_LENGTH_LINK1 0.091
#define TRUE_LENGTH_LINK2 0.070
#define TRUE_MASS_LINK1 0.1 // Not actually in the paper.
#define TRUE_MASS_LINK2 0.1 // Not actually in the paper.

using namespace tds;
using namespace TINY;

#ifdef USE_MATPLOTLIB
template <typename T>
void plot_trajectory(const std::vector<std::vector<T>> &states)
{
  typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                             CeresUtils<4>>
      Utils;
  for (int i = 0; i < static_cast<int>(states[0].size()); ++i)
  {
    std::vector<double> traj(states.size());
    for (int t = 0; t < static_cast<int>(states.size()); ++t)
    {
      traj[t] = Utils::getDouble(states[t][i]);
    }
    plt::named_plot("state[" + std::to_string(i) + "]", traj);
  }
  plt::legend();
  plt::show();
}
#endif

template <typename T>
void visualize_trajectory(const std::vector<std::vector<T>> &states,
                          const std::vector<T> &params, const T &dt)
{
  typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                             CeresUtils<param_dim>>
      Utils;
  using Algebra = TinyAlgebra<T, Utils>;
  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI *visualizer = new VisualizerAPI();
  std::string plane_filename;
  FileUtils::find_file("plane_implicit.urdf", plane_filename);
  char path[TINY_MAX_EXE_PATH_LEN];
  FileUtils::extract_path(plane_filename.c_str(), path,
                          TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;
  visualizer->connect(eCONNECT_GUI);
  visualizer->setAdditionalSearchPath(search_path);
  if (visualizer->canSubmitCommand())
  {
    visualizer->resetSimulation();
  }
  World<Algebra> world;
  MultiBody<Algebra> *mb = world.create_multi_body();
  std::vector<T> link_lengths{TRUE_LENGTH_LINK1, TRUE_LENGTH_LINK2};
  init_compound_pendulum<Algebra>(*mb, world, 2, link_lengths);
  std::vector<int> mbvisuals;
  if (visualizer->canSubmitCommand())
  {
    for (int i = 0; i < mb->size(); i++)
    {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
    }
  }

  btVector3 basePos(0, 0, -0.2);
  double distance = 1.8;
  visualizer->resetDebugVisualizerCamera(distance, 0, 90, basePos);

  std::vector<T> q(2);
  for (const std::vector<T> &state : states)
  {
    q[0] = state[0];
    q[1] = state[1];
    forward_kinematics(*mb, q);
    // printf("  q: [%.6f  %.6f]\n", Utils::getDouble(q[0]),
    //        Utils::getDouble(q[1]));

    std::this_thread::sleep_for(
        std::chrono::duration<double>(Utils::getDouble(dt * 20.)));
    // sync transforms
    int visual_index = 0;
    for (std::size_t l = 0; l < mb->size(); l++)
    {
      int sphereId = mbvisuals[visual_index++];
      typename Algebra::Quaternion rot;
      const Transform<Algebra> &geom_X_world =
          (*mb)[l].X_world * (*mb)[l].X_visuals[0];
      btVector3 base_pos(Utils::getDouble(geom_X_world.translation.getX()),
                         Utils::getDouble(geom_X_world.translation.getY()),
                         Utils::getDouble(geom_X_world.translation.getZ()));
      geom_X_world.rotation.getRotation(rot);
      btQuaternion base_orn(
          Utils::getDouble(rot.getX()), Utils::getDouble(rot.getY()),
          Utils::getDouble(rot.getZ()), Utils::getDouble(rot.getW()));
      visualizer->resetBasePositionAndOrientation(sphereId, base_pos, base_orn);
    }
  }

  visualizer->disconnect();
  delete visualizer;
}

/**
 * Roll-out pendulum dynamics, and compute states [q, qd].
 */
template <typename Scalar = double, typename Utils = DoubleUtils>
void rollout_pendulum(const std::vector<Scalar> &params,
                      std::vector<std::vector<Scalar>> &output_states,
                      int time_steps, double dt)
{
  using Algebra = TinyAlgebra<Scalar, Utils>;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  Vector3 gravity(Utils::zero(), Utils::zero(),
                  Utils::fraction(-981, 100));
  output_states.resize(time_steps);
  World<Algebra> world;
  MultiBody<Algebra> *mb = world.create_multi_body();
  int param_count = 0;
#if ESTIMATE_LENGTH
  std::vector<Scalar> link_lengths(params.begin() + param_count,
                                   params.begin() + param_count + 2);
  param_count += 2;
#else
  std::vector<Scalar> link_lengths = {Scalar(TRUE_LENGTH_LINK1),
                                      Scalar(TRUE_LENGTH_LINK2)};
#endif
#if ESTIMATE_MASS
  std::vector<Scalar> masses(params.begin() + param_count,
                             params.begin() + param_count + 2);
  param_count += 2;
#else
  std::vector<Scalar> masses = {Scalar(TRUE_MASS_LINK1),
                                Scalar(TRUE_MASS_LINK2)};
#endif
  init_compound_pendulum<Algebra>(*mb, world, 2, link_lengths, masses);
#if ESTIMATE_INERTIA
  Matrix3 inertia_0;
  inertia_0.set_zero();
  inertia_0(0, 0) = params[param_count + 0];
  inertia_0(1, 1) = params[param_count + 1];
  inertia_0(2, 2) = params[param_count + 2];
  Vector3 com_0(Utils::zero(), link_lengths[0],
                Utils::zero());
  (*mb)[0].rbi =
      RigidBodyInertia<Algebra>(
          masses[0], com_0, inertia_0);
  Matrix3 inertia_1;
  inertia_1.set_zero();
  inertia_1(0, 0) = params[param_count + 3];
  inertia_1(1, 1) = params[param_count + 4];
  inertia_1(2, 2) = params[param_count + 5];
  Vector3 com_1(Utils::zero(), link_lengths[1],
                Utils::zero());
  (*mb)[1].rbi =
      RigidBodyInertia<Algebra>(
          masses[1], com_1, inertia_1);
  param_count += 6;
#endif
  if (static_cast<int>(start_state.size()) >= mb->dof())
  {
    for (int i = 0; i < mb->dof(); ++i)
    {
      mb->q(i) = Scalar(start_state[i]);
    }
    if (static_cast<int>(start_state.size()) >= 2 * mb->dof())
    {
      for (int i = 0; i < mb->dof_qd(); ++i)
      {
        mb->qd(i) = Scalar(start_state[i + mb->dof()]); // / 5.;
      }
    }
  }

#if ESTIMATE_INITIAL_VELOCITY
  mb->qd(0) = params[param_count];
  mb->qd(1) = params[param_count + 1];
  param_count += 2;
#endif

#if ESTIMATE_JOINT_DAMPING
  (*mb)[0].damping = params[param_count];
  (*mb)[1].damping = params[param_count + 1];
  param_count += 2;
#endif

#if ESTIMATE_TIME_STEP
  Scalar actual_dt = params[param_count++];
#else
  Scalar actual_dt(dt);
#endif

  for (int t = 0; t < time_steps; ++t)
  {
#if STATE_INCLUDES_QD
    output_states[t].resize(2 * mb->dof());
#else
    output_states[t].resize(mb->dof());
#endif
    for (int i = 0; i < mb->dof(); ++i)
    {
      output_states[t][i] = mb->q(i);
#if STATE_INCLUDES_QD
      output_states[t][i + mb->dof()] = mb->qd(i);
#endif
    }
    forward_dynamics(*mb, gravity);

    // if (t > 150) {
    //   mb->print_state();
    // }
    integrate_euler(*mb, actual_dt);
  }

#if !USE_PBH
  // visualize_trajectory(output_states, params, Scalar(dt));
#endif
}

template <ResidualMode ResMode>
class PendulumEstimator
    : public CeresEstimatorOld<param_dim, (1 + STATE_INCLUDES_QD) * 2,
                               ResMode>
{
public:
  typedef CeresEstimatorOld<param_dim, (1 + STATE_INCLUDES_QD) * 2, ResMode>
      CE;
  using CE::kStateDim, CE::kParameterDim;
  using CE::parameters;
  using typename CE::ADScalar;

  int time_steps;

  // sane parameter initialization (link lengths)
  PendulumEstimator(int time_steps, double dt, double initial_link_length = 0.5,
                    double initial_mass = 0.1)
      : CE(dt), time_steps(time_steps)
  {
    int param_count = 0;
#if ESTIMATE_LENGTH
    for (int i = 0; i < 2; ++param_count, ++i)
    {
      parameters[param_count] = {"link_length_" + std::to_string(i + 1),
                                 initial_link_length, 0.05, 0.15};
    }
#endif
#if ESTIMATE_MASS
    for (int i = 0; i < 2; ++param_count, ++i)
    {
      parameters[param_count] = {"mass_" + std::to_string(i + 1), initial_mass,
                                 0.00001, 1.5};
    }
#endif
#if ESTIMATE_INERTIA
    parameters[param_count + 0] = {"I0_xx", 0.005, 0.002, 1.0};
    parameters[param_count + 1] = {"I0_yy", 0.005, 0.002, 1.0};
    parameters[param_count + 2] = {"I0_zz", 0.005, 0.002, 1.0};
    parameters[param_count + 3] = {"I1_xx", 0.005, 0.002, 1.0};
    parameters[param_count + 4] = {"I1_yy", 0.005, 0.002, 1.0};
    parameters[param_count + 5] = {"I1_zz", 0.005, 0.002, 1.0};
    param_count += 6;
#endif
#if ESTIMATE_INITIAL_VELOCITY
    parameters[param_count + 0] = {"qd[0]", 0.0, -50., 50.0};
    parameters[param_count + 1] = {"qd[1]", 0.0, -50., 50.0};
    param_count += 2;
#endif
#if ESTIMATE_JOINT_DAMPING
    parameters[param_count + 0] = {"damping[0]", 0.0, 0., 5.0};
    parameters[param_count + 1] = {"damping[1]", 0.0, 0., 5.0};
    param_count += 2;
#endif
#if ESTIMATE_TIME_STEP
    parameters[param_count++] = {"dt", 1. / 400, 1. / 1000., 1. / 50};
#endif
  }

  void rollout(const std::vector<ADScalar> &params,
               std::vector<std::vector<ADScalar>> &output_states,
               double &dt,
               std::size_t ref_id) const override
  {
    rollout_pendulum<ADScalar, CeresUtils<kParameterDim>>(params, output_states,
                                                          time_steps, dt);
  }
  void rollout(const std::vector<double> &params,
               std::vector<std::vector<double>> &output_states,
               double &dt,
               std::size_t ref_id) const override
  {
    rollout_pendulum(params, output_states, time_steps, dt);
  }
};

void print_states(const std::vector<std::vector<double>> &states)
{
  for (const auto &s : states)
  {
    for (double d : s)
    {
      printf("%.2f ", d);
    }
    printf("\n");
  }
}

void write_trajectory_file(const std::string &filename,
                           std::vector<double> params, int time_steps,
                           double dt)
{
  std::vector<std::vector<double>> states;
  rollout_pendulum<double, DoubleUtils>(params, states, time_steps, dt);
  std::ofstream traj_file(filename);
  for (int t = 0; t < time_steps; ++t)
  {
    traj_file << (t * dt);
    for (double v : states[t])
    {
      traj_file << "\t" << v;
    }
    traj_file << "\n";
  }
  traj_file.close();
}

int main(int argc, char *argv[])
{
  const double dt = 1. / 400;
  const double time_limit = 1.0;
  const int time_steps = time_limit / dt;
  const double init_params = 0.2;

  // if (argc != 2) {
  //   std::cout << "Usage: " << argv[0] << " <dataset.csv>\n";
  //   return 1;
  // }

  google::InitGoogleLogging(argv[0]);

  FileUtils::prefixes.push_back("tds/data/");

  std::string filename =
      "ibm-double-pendulum/original/dpc_dataset_csv/0.csv";
  auto dataset = LoadIbmPendulumFile<double>(filename);
  dataset.resize(time_steps); // Discard after the clip time.
  std::cout << "Using " << dataset.size() << " data steps.\n";
  auto target_states = PendulumIk(dataset);

  if (dataset.empty())
  {
    exit(1);
  }

  std::ofstream true_traj_file("true_trajectory.csv");
  for (int t = 0; t < time_steps; ++t)
  {
    true_traj_file << (t * dt);
    for (double v : target_states[t])
    {
      true_traj_file << "\t" << v;
    }
    true_traj_file << "\n";
  }
  true_traj_file.close();

  std::vector<double> target_times;
  target_times.reserve(dataset.size());
  for (const auto &row : dataset)
  {
    target_times.push_back(row[0]);
  }
  start_state = target_states[0];

#if JUST_VISUALIZE
  std::string connection_mode = "gui";
  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI *visualizer = new VisualizerAPI();
  printf("mode=%s\n", (char *)connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct")
    mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory")
    mode = eCONNECT_SHARED_MEMORY;

  visualizer->connect(mode);
  std::string plane_filename;
  FileUtils::find_file("plane_implicit.urdf", plane_filename);

  char path[TINY_MAX_EXE_PATH_LEN];
  FileUtils::extract_path(plane_filename.c_str(), path,
                          TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;
  visualizer->setAdditionalSearchPath(search_path);
  std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  if (visualizer->canSubmitCommand())
  {
    visualizer->resetSimulation();
  }
  std::vector<TinyRigidBody<double, DoubleUtils> *> bodies;
  std::vector<int> visuals;

  std::vector<MultiBody<double, DoubleUtils> *> mbbodies;
  std::vector<int> mbvisuals;

  World<double, DoubleUtils> world;
  MultiBody<double, DoubleUtils> *mb = world.create_multi_body();
  init_compound_pendulum<double, DoubleUtils>(*mb, world, 2);
  mbbodies.push_back(mb);

  if (visualizer->canSubmitCommand())
  {
    for (int i = 0; i < mb->size(); i++)
    {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
      // apply some linear joint damping
      (*mb)[i].damping = 5.;
    }
  }
  while (true)
  {
    const auto state = target_states[0];
    // for (const auto &state : target_states) {
    mb->q(0) = state[0];
    mb->q(1) = state[1];
    forward_kinematics(*mb, );
    if (visualizer->canSubmitCommand())
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      // sync transforms
      int visual_index = 0;
      if (!mbvisuals.empty())
      {
        for (int b = 0; b < mbbodies.size(); b++)
        {
          for (int l = 0; l < mbbodies[b]->size(); l++)
          {
            const MultiBody<double, DoubleUtils> *body = mbbodies[b];
            if ((*bo)[dyl].X_visuals.empty())
              continue;

            int sphereId = mbvisuals[visual_index++];

            Quaternion<double, DoubleUtils> rot;
            const Transform<double, DoubleUtils> &geom_X_world =
                (*bo)[dyl].X_world * (*bo)[dyl].X_visuals[0];
            btVector3 base_pos(geom_X_world.translation.getX(),
                               geom_X_world.translation.getY(),
                               geom_X_world.translation.getZ());
            geom_X_world.rotation.getRotation(rot);
            btQuaternion base_orn(rot.getX(), rot.getY(), rot.getZ(),
                                  rot.getW());
            visualizer->resetBasePositionAndOrientation(sphereId, base_pos,
                                                        base_orn);
          }
        }
      }
    }
    // }
  }
#endif

  typedef PendulumEstimator<RES_MODE_1D> Estimator;

  std::function<std::unique_ptr<Estimator>()> construct_estimator =
      [&target_times, &target_states, &time_steps, &dt, &init_params]() {
        auto estimator =
            std::make_unique<Estimator>(time_steps, dt, init_params);
        estimator->target_times = {target_times};
        estimator->target_trajectories = {target_states};
        estimator->options.minimizer_progress_to_stdout = !USE_PBH;
        estimator->options.max_num_consecutive_invalid_steps = 100;
        // divide each cost term by integer time step ^ 2 to reduce gradient
        // explosion
        estimator->divide_cost_by_time_factor = 0.; // 10.;
        estimator->divide_cost_by_time_exponent = 1.2;
        return estimator;
      };

#if USE_PBH
  std::array<double, param_dim> initial_guess;
  for (int i = 0; i < param_dim; ++i)
  {
    initial_guess[i] = init_params;
  }
  BasinHoppingEstimator<param_dim, Estimator> bhe(construct_estimator,
                                                  initial_guess);
  bhe.time_limit = 300;
  bhe.run();

  printf("Optimized parameters:");
  for (int i = 0; i < param_dim; ++i)
  {
    printf(" %.8f", bhe.params[i]);
  }
  printf("\n");

  printf("Best cost: %f\n", bhe.best_cost());

  printf("Best parameters:\n");
  // just create an estimator to get the parameter names
  auto estimator = construct_estimator();
  std::vector<double> best_params;
  int pi = 0;
  for (const auto &p : bhe.params)
  {
    best_params.push_back(p);
    const auto &named_param = estimator->parameters[pi];
    printf("\t%s: %.6f\n", named_param.name.c_str(), p);
    ++pi;
  }
  target_states.clear();
#else
  std::unique_ptr<Estimator> estimator = construct_estimator();
  estimator->setup(new ceres::HuberLoss(1.));

#ifdef JUST_SAVE_GRADIENT
  double cost;
  int gradient_dim = estimator->kResidualDim * param_dim;
  double gradient[gradient_dim];
  estimator->compute_gradient(estimator->vars(), &cost, gradient);
  std::ofstream gradient_file("estimation_gradient.csv");
  for (int i = 0; i < gradient_dim; ++i)
  {
    gradient_file << gradient[i] << '\n';
  }
  gradient_file.close();
  return 0;
#endif

  auto summary = estimator->solve();
  std::cout << summary.FullReport() << std::endl;
  std::cout << "Final cost: " << summary.final_cost << "\n";

  std::vector<double> best_params;
  for (const auto &p : estimator->parameters)
  {
    printf("%s: %.3f\n", p.name.c_str(), p.value);
    best_params.push_back(p.value);
  }

  std::ofstream file("param_evolution.txt");
  for (const auto &params : estimator->parameter_evolution())
  {
    for (int i = 0; i < static_cast<int>(params.size()); ++i)
    {
      file << params[i];
      if (i < static_cast<int>(params.size()) - 1)
        file << "\t";
    }
    file << "\n";
  }
  file.close();
#endif

#if !USE_PBH
  for (int solver_iteration = 0;
       solver_iteration < estimator->parameter_evolution().size();
       ++solver_iteration)
  {
    const auto &params = estimator->parameter_evolution()[solver_iteration];
    const std::vector<double> step_params(params.begin(), params.end());
    rollout_pendulum<double, DoubleUtils>(step_params, target_states,
                                          time_steps, dt);
    std::ostringstream fn;
    fn << "step_" << std::setw(3) << std::setfill('0') << solver_iteration
       << "_estimated_trajectory.csv";
    write_trajectory_file(fn.str(), step_params, time_steps, dt);
  }
#endif

  write_trajectory_file("best_estimated_trajectory.csv", best_params,
                        time_steps, dt);

  std::vector<std::vector<double>> best_states;
  rollout_pendulum(best_params, best_states, time_steps, dt);
  while (true)
  {
    visualize_trajectory(best_states, best_params, dt);
    printf("Playing back in 5s again...");
    std::this_thread::sleep_for(std::chrono::duration<double>(5.));
  }

  return EXIT_SUCCESS;
}
