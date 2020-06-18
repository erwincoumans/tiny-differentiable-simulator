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

#include <fstream>

#include "data/ibm-double-pendulum/load_data.h"
#include "pendulum.h"
#include "pybullet_visualizer_api.h"
#include "tiny_ceres_estimator.h"
#include "tiny_file_utils.h"
#include "tiny_multi_body.h"
#include "tiny_world.h"

#define JUST_VISUALIZE false
#define USE_PBH true
// whether the state consists of [q qd] or just q
#define STATE_INCLUDES_QD false
// whether to estimate the diagonal elements of the inertia 3x3 matrix
#define ESTIMATE_INERTIA false
std::vector<double> start_state;
const int param_dim = ESTIMATE_INERTIA ? 10 : 4;

#ifdef USE_MATPLOTLIB
template <typename T>
void plot_trajectory(const std::vector<std::vector<T>> &states) {
  typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                             CeresUtils<4>>
      Utils;
  for (int i = 0; i < static_cast<int>(states[0].size()); ++i) {
    std::vector<double> traj(states.size());
    for (int t = 0; t < static_cast<int>(states.size()); ++t) {
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
                          const std::vector<T> &params, const T &dt) {
  typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                             CeresUtils<param_dim>>
      Utils;
  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI *visualizer = new VisualizerAPI();
  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);
  char path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;
  visualizer->connect(eCONNECT_GUI);
  visualizer->setAdditionalSearchPath(search_path);
  if (visualizer->canSubmitCommand()) {
    visualizer->resetSimulation();
  }
  TinyWorld<T, Utils> world;
  TinyMultiBody<T, Utils> *mb = world.create_multi_body();
  std::vector<T> link_lengths(params.begin(), params.begin() + 2);
  std::vector<T> masses(2);
  for (int i = 0; i < 2; ++i) {
    masses[i] = params[2 + i];
  }
  init_compound_pendulum<T, Utils>(*mb, world, 2, link_lengths, masses);
  std::vector<int> mbvisuals;
  if (visualizer->canSubmitCommand()) {
    for (int i = 0; i < mb->m_links.size(); i++) {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
    }
  }

  btVector3 basePos(0, 0, -0.2);
  double distance = 1.8;
  visualizer->resetDebugVisualizerCamera(distance, 0, 90, basePos);

  std::vector<T> q(2);
  for (const std::vector<T> &state : states) {
    q[0] = state[0];
    q[1] = state[1];
    mb->forward_kinematics(q);
    printf("  q: [%.6f  %.6f]\n", Utils::getDouble(q[0]),
           Utils::getDouble(q[1]));

    std::this_thread::sleep_for(
        std::chrono::duration<double>(Utils::getDouble(dt)));
    // sync transforms
    int visual_index = 0;
    for (int l = 0; l < mb->m_links.size(); l++) {
      // if (mb->m_X_visuals.empty()) continue;

      int sphereId = mbvisuals[visual_index++];
      TinyQuaternion<T, Utils> rot;
      const TinySpatialTransform<T, Utils> &geom_X_world =
          mb->m_links[l].m_X_world * mb->m_links[l].m_X_visuals[0];
      btVector3 base_pos(Utils::getDouble(geom_X_world.m_translation.getX()),
                         Utils::getDouble(geom_X_world.m_translation.getY()),
                         Utils::getDouble(geom_X_world.m_translation.getZ()));
      geom_X_world.m_rotation.getRotation(rot);
      // printf("Sphere %i position: %.6f %.6f %.6f\n", sphereId, base_pos[0],
      // base_pos[1], base_pos[2]);
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
                      int time_steps, double dt) {
  TinyVector3<Scalar, Utils> gravity(Utils::zero(), Utils::zero(),
                                     Utils::fraction(-981, 100));
  output_states.resize(time_steps);
  TinyWorld<Scalar, Utils> world;
  TinyMultiBody<Scalar, Utils> *mb = world.create_multi_body();
  std::vector<Scalar> link_lengths(params.begin(), params.begin() + 2);
  std::vector<Scalar> masses(params.begin() + 2, params.begin() + 4);
  init_compound_pendulum<Scalar, Utils>(*mb, world, 2, link_lengths, masses);
#if ESTIMATE_INERTIA
  TinyMatrix3x3<Scalar, Utils> inertia_0;
  inertia_0.set_zero();
  inertia_0(0, 0) = params[4];
  inertia_0(1, 1) = params[5];
  inertia_0(2, 2) = params[6];
  TinyVector3<Scalar, Utils> com_0(Utils::zero(), link_lengths[0],
                                   Utils::zero());
  mb->m_links[0].m_I =
      TinySymmetricSpatialDyad<Scalar, Utils>::computeInertiaDyad(
          masses[0], com_0, inertia_0);
  TinyMatrix3x3<Scalar, Utils> inertia_1;
  inertia_1.set_zero();
  inertia_1(0, 0) = params[7];
  inertia_1(1, 1) = params[8];
  inertia_1(2, 2) = params[9];
  TinyVector3<Scalar, Utils> com_1(Utils::zero(), link_lengths[1],
                                   Utils::zero());
  mb->m_links[1].m_I =
      TinySymmetricSpatialDyad<Scalar, Utils>::computeInertiaDyad(
          masses[1], com_1, inertia_1);
#endif
  if (static_cast<int>(start_state.size()) >= mb->dof()) {
    for (int i = 0; i < mb->dof(); ++i) {
      mb->m_q[i] = Scalar(start_state[i]);
    }
    if (static_cast<int>(start_state.size()) >= 2 * mb->dof()) {
      for (int i = 0; i < mb->dof_qd(); ++i) {
        mb->m_qd[i] = Scalar(start_state[i + mb->dof()]); // / 5.;
      }
    }
  }
  for (int t = 0; t < time_steps; ++t) {
#if STATE_INCLUDES_QD
    output_states[t].resize(2 * mb->dof());
#else
    output_states[t].resize(mb->dof());
#endif
    for (int i = 0; i < mb->dof(); ++i) {
      output_states[t][i] = mb->m_q[i];
#if STATE_INCLUDES_QD
      output_states[t][i + mb->dof()] = mb->m_qd[i];
#endif
    }
    mb->forward_dynamics(gravity);

    // if (t > 150) {
    //   mb->print_state();
    // }
    // mb->integrate_q(Scalar(dt));
    mb->integrate(Scalar(dt));
  }

#if !USE_PBH
  // visualize_trajectory(output_states, params, Scalar(dt));
#endif
}

template <ResidualMode ResMode>
class PendulumEstimator
    : public TinyCeresEstimator<param_dim, (1 + STATE_INCLUDES_QD) * 2,
                                ResMode> {
public:
  typedef TinyCeresEstimator<param_dim, (1 + STATE_INCLUDES_QD) * 2, ResMode>
      CeresEstimator;
  using CeresEstimator::kStateDim, CeresEstimator::kParameterDim;
  using CeresEstimator::parameters;
  using typename CeresEstimator::ADScalar;

  int time_steps;

  // sane parameter initialization (link lengths)
  PendulumEstimator(int time_steps, double dt, double initial_link_length = 0.5,
                    double initial_mass = 0.5)
      : CeresEstimator(dt), time_steps(time_steps) {
    for (int i = 0; i < 2; ++i) {
      parameters[i] = {"link_length_" + std::to_string(i + 1),
                       initial_link_length, 0.1, 0.4};
    }
    for (int i = 0; i < 2; ++i) {
      parameters[2 + i] = {"mass_" + std::to_string(i + 1), initial_mass, 0.05,
                           0.4};
    }
#if ESTIMATE_INERTIA
    parameters[4] = {"I0_xx", 0.005, 0.02, 0.3};
    parameters[5] = {"I0_yy", 0.005, 0.02, 0.3};
    parameters[6] = {"I0_zz", 0.005, 0.02, 0.3};
    parameters[7] = {"I1_xx", 0.005, 0.02, 0.3};
    parameters[8] = {"I1_yy", 0.005, 0.02, 0.3};
    parameters[9] = {"I1_zz", 0.005, 0.02, 0.3};
#endif

    /// XXX just for testing
    // std::vector<double> params = {
    //     0.20814544087513831006, 2.00000000000000000000,
    //     0.14999999999999999445, 1.85036736762471165640};

    // for (int i = 0; i < 2; ++i) {
    //   parameters[i] = {"link_length_" + std::to_string(i + 1), params[i],
    //   0.15,
    //                    2.};
    // }
    // for (int i = 0; i < 2; ++i) {
    //   parameters[2 + i] = {"mass_" + std::to_string(i + 1), params[i + 2],
    //   0.15,
    //                        2.};
    // }

    // std::vector<std::vector<double>> states;
    // rollout_pendulum<double, DoubleUtils>(params, states, time_steps, dt);
    // plot_trajectory(states);
  }

  void rollout(const std::vector<ADScalar> &params,
               std::vector<std::vector<ADScalar>> &output_states,
               double dt) const override {
    rollout_pendulum<ADScalar, CeresUtils<kParameterDim>>(params, output_states,
                                                          time_steps, dt);
  }
  void rollout(const std::vector<double> &params,
               std::vector<std::vector<double>> &output_states,
               double dt) const override {
    rollout_pendulum(params, output_states, time_steps, dt);
  }
};

void print_states(const std::vector<std::vector<double>> &states) {
  for (const auto &s : states) {
    for (double d : s) {
      printf("%.2f ", d);
    }
    printf("\n");
  }
}

int main(int argc, char *argv[]) {
  const double dt = 1. / 400;
  const double time_limit = 5;
  const int time_steps = time_limit / dt;
  const double init_params = 0.2;

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <dataset.csv>\n";
    return 1;
  }

  google::InitGoogleLogging(argv[0]);

  auto dataset = LoadIbmPendulumFile<double>(argv[1]);
  dataset.resize(time_steps); // Discard after the clip time.
  auto target_states = PendulumIk(dataset);
  assert(dataset.size() > 0);

  std::vector<double> target_times;
  target_times.reserve(dataset.size());
  for (const auto &row : dataset) {
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
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);

  char path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;
  visualizer->setAdditionalSearchPath(search_path);
  std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  if (visualizer->canSubmitCommand()) {
    visualizer->resetSimulation();
  }
  std::vector<TinyRigidBody<double, DoubleUtils> *> bodies;
  std::vector<int> visuals;

  std::vector<TinyMultiBody<double, DoubleUtils> *> mbbodies;
  std::vector<int> mbvisuals;

  TinyWorld<double, DoubleUtils> world;
  TinyMultiBody<double, DoubleUtils> *mb = world.create_multi_body();
  init_compound_pendulum<double, DoubleUtils>(*mb, world, 2);
  mbbodies.push_back(mb);

  if (visualizer->canSubmitCommand()) {
    for (int i = 0; i < mb->m_links.size(); i++) {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
      // apply some linear joint damping
      mb->m_links[i].m_damping = 5.;
    }
  }
  while (true) {
    for (const auto &state : target_states) {
      mb->m_q[0] = state[0];
      mb->m_q[1] = state[1];
      mb->forward_kinematics();
      if (visualizer->canSubmitCommand()) {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        // sync transforms
        int visual_index = 0;
        if (!mbvisuals.empty()) {
          for (int b = 0; b < mbbodies.size(); b++) {
            for (int l = 0; l < mbbodies[b]->m_links.size(); l++) {
              const TinyMultiBody<double, DoubleUtils> *body = mbbodies[b];
              if (body->m_links[l].m_X_visuals.empty())
                continue;

              int sphereId = mbvisuals[visual_index++];

              TinyQuaternion<double, DoubleUtils> rot;
              const TinySpatialTransform<double, DoubleUtils> &geom_X_world =
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
  }
#endif

  typedef PendulumEstimator<RES_MODE_1D> Estimator;

  std::function<std::unique_ptr<Estimator>()> construct_estimator =
      [&target_times, &target_states, &time_steps, &dt, &init_params]() {
        auto estimator =
            std::make_unique<Estimator>(time_steps, dt, init_params);
        estimator->target_times = target_times;
        estimator->target_states = target_states;
        estimator->options.minimizer_progress_to_stdout = !USE_PBH;
        estimator->options.max_num_consecutive_invalid_steps = 100;
        // divide each cost term by integer time step ^ 2 to reduce gradient
        // explosion
        estimator->divide_cost_by_time_factor = 10.;
        estimator->divide_cost_by_time_exponent = 1.2;
        return estimator;
      };

#if USE_PBH
  std::array<double, param_dim> initial_guess;
  for (int i = 0; i < param_dim; ++i) {
    initial_guess[i] = init_params;
  }
  BasinHoppingEstimator<param_dim, Estimator> bhe(construct_estimator,
                                                  initial_guess);
  bhe.time_limit = 20;
  bhe.run();

  printf("Optimized parameters:");
  for (int i = 0; i < param_dim; ++i) {
    printf(" %.8f", bhe.params[i]);
  }
  printf("\n");

  printf("Best cost: %f\n", bhe.best_cost());

  std::vector<double> best_params;
  for (const auto &p : bhe.params) {
    best_params.push_back(p);
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
  for (int i = 0; i < gradient_dim; ++i) {
    gradient_file << gradient[i] << '\n';
  }
  gradient_file.close();
  return 0;
#endif

  double cost;
  double gradient[4];
  estimator->compute_gradient(estimator->vars(), &cost, gradient);
  std::cout << "Gradient: " << gradient[0] << "  " << gradient[1] << "  "
            << gradient[2] << "  " << gradient[3] << "  \n";
  std::cout << "Cost: " << cost << "\n";

  auto summary = estimator->solve();
  std::cout << summary.FullReport() << std::endl;
  std::cout << "Final cost: " << summary.final_cost << "\n";

  std::vector<double> best_params;
  for (const auto &p : estimator->parameters) {
    printf("%s: %.3f\n", p.name.c_str(), p.value);
    best_params.push_back(p.value);
  }

  std::ofstream file("param_evolution.txt");
  for (const auto &params : estimator->parameter_evolution()) {
    for (int i = 0; i < static_cast<int>(params.size()); ++i) {
      file << params[i];
      if (i < static_cast<int>(params.size()) - 1)
        file << "\t";
    }
    file << "\n";
  }
  file.close();
#endif

  rollout_pendulum<double, DoubleUtils>(best_params, target_states, time_steps,
                                        dt);
  std::ofstream traj_file("estimated_trajectory.csv");
  for (int t = 0; t < time_steps; ++t) {
    traj_file << (t * dt);
    for (double v : target_states[t]) {
      traj_file << "\t" << v;
    }
    traj_file << "\n";
  }
  traj_file.close();

  return EXIT_SUCCESS;
}
