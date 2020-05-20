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
std::vector<double> start_state;

/**
 * Load data from txt files provided by Schmidt & Lipson.
 */
bool load_schmidt_lipson(const std::string &filename,
                         std::vector<double> &times,
                         std::vector<std::vector<double>> &states,
                         double clip_after_t = 0.) {
  std::ifstream input(filename);
  if (!input) {
    std::cerr << "Could not open file \"" << filename << "\".\n";
    return false;
  }
  std::string header;
  getline(input, header);
  if (header.size() < 3 || header[0] != '%') {
    std::cerr << "Invalid first line in file \"" << filename << "\".\n";
    return false;
  }
  int num_columns = 0;
  std::size_t pos = 0;
  while ((pos = header.find(" ", pos + 1)) != std::string::npos) {
    ++num_columns;
  }
  printf("Loading %i columns from %s: %s\n", num_columns, filename.c_str(),
         header.c_str() + 2);
  std::stringstream ss;
  double val;
  double last_time, time_delta_avg = 0;
  int num_lines = 0;
  for (std::string line; getline(input, line); ++num_lines) {
    ss = std::stringstream(line);
    ss >> val;  // ignore first zero
    ss >> val;
    times.push_back(val);
    if (num_lines > 0) {
      time_delta_avg += val - last_time;
    }
    last_time = val;
    if (clip_after_t > 0. && last_time > clip_after_t) {
      break;
    }
    std::vector<double> state(num_columns);
    for (int i = 0; i < num_columns; ++i) {
      ss >> val;
      state[i] = val;
    }
    state[0] -= M_PI_2;
    states.push_back(state);
  }
  time_delta_avg /= num_lines;
  printf("Loaded %i samples with a mean time delta of %.6f s.\n", num_lines,
         time_delta_avg);
  return true;
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
  init_compound_pendulum<Scalar, Utils>(
      *mb, world, static_cast<int>(params.size()), params);
  if (static_cast<int>(start_state.size()) >= mb->dof()) {
    for (int i = 0; i < mb->dof(); ++i) {
      mb->m_q[i] = Scalar(start_state[i]);
    }
    if (static_cast<int>(start_state.size()) >= 2 * mb->dof()) {
      for (int i = 0; i < mb->dof(); ++i) {
        mb->m_qd[i] = Scalar(start_state[i + mb->dof()]);
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
    mb->integrate(Scalar(dt));
  }
}

template <int NumLinks, ResidualMode ResMode>
class PendulumEstimator
    : public TinyCeresEstimator<NumLinks, (1 + STATE_INCLUDES_QD) * NumLinks,
                                ResMode> {
 public:
  typedef TinyCeresEstimator<NumLinks, (1 + STATE_INCLUDES_QD) * NumLinks,
                             ResMode>
      CeresEstimator;
  using CeresEstimator::kStateDim, CeresEstimator::kParameterDim;
  using CeresEstimator::parameters;
  using typename CeresEstimator::ADScalar;

  int time_steps;

  // sane parameter initialization (link lengths)
  PendulumEstimator(int time_steps, double dt, double initial_link_length = 0.5)
      : CeresEstimator(dt), time_steps(time_steps) {
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i] = {"link_length_" + std::to_string(i + 1),
                       initial_link_length, 0.15, 2.};
    }
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
  const double dt = 1. / 100;
  const int time_steps = 500;
  const int param_dim = 2;
  const double init_params = 0.2;
  const double time_limit = 5;

  std::string exp_filename;
  TinyFileUtils::find_file("schmidt-lipson-exp-data/real_double_pend_h_1.txt",
                           exp_filename);
  std::vector<double> target_times;
  std::vector<std::vector<double>> target_states;
  // clip earlier
  bool success = load_schmidt_lipson(exp_filename, target_times, target_states,
                                     time_limit);
  assert(success);
  printf("load_schmidt_lipson - success? %i\n", success);
  start_state = target_states[0];

#if JUST_VISUALIZE
  std::string connection_mode = "gui";
  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI *visualizer = new VisualizerAPI();
  printf("mode=%s\n", (char *)connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

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
              if (body->m_links[l].m_X_visuals.empty()) continue;

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

  typedef PendulumEstimator<param_dim, RES_MODE_1D> Estimator;

  std::function<std::unique_ptr<Estimator>()> construct_estimator =
      [&target_times, &target_states, &time_steps, &dt, &init_params]() {
        auto estimator =
            std::make_unique<Estimator>(time_steps, dt, init_params);
        estimator->target_times = target_times;
        estimator->target_states = target_states;
        estimator->options.minimizer_progress_to_stdout = false;
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
  rollout_pendulum<double, DoubleUtils>(best_params, target_states, time_steps,
                                        dt);
  std::ofstream file("estimated_trajectory.csv");
  for (int t = 0; t < time_steps; ++t) {
    file << (t * dt);
    for (double v : target_states[t]) {
      file << "\t" << v;
    }
    file << "\n";
  }
  file.close();
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

  auto summary = estimator->solve();
  std::cout << summary.FullReport() << std::endl;

  for (const auto &p : estimator->parameters) {
    printf("%s: %.3f\n", p.name.c_str(), p.value);
  }

  std::ofstream file("param_evolution.txt");
  for (const auto &params : estimator->parameter_evolution()) {
    for (int i = 0; i < static_cast<int>(params.size()); ++i) {
      file << params[i];
      if (i < static_cast<int>(params.size()) - 1) file << "\t";
    }
    file << "\n";
  }
  file.close();
#endif

  return EXIT_SUCCESS;
}
