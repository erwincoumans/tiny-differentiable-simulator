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
#include "tiny_ceres_estimator.h"
#include "tiny_file_utils.h"
#include "tiny_multi_body.h"
#include "tiny_world.h"

double dt;

/**
 * Load data from txt files provided by Schmidt & Lipson.
 */
bool load_schmidt_lipson(const std::string &filename,
                         std::vector<double> &times,
                         std::vector<std::vector<double>> &states) {
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
    std::vector<double> state(num_columns);
    for (int i = 0; i < num_columns; ++i) {
      ss >> val;
      state[i] = val * M_PI / 180.;
    }
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
                      int time_steps) {
  TinyVector3<Scalar, Utils> gravity(Utils::zero(), Utils::zero(),
                                     Utils::fraction(-981, 100));
  output_states.resize(time_steps);
  TinyWorld<Scalar, Utils> world;
  TinyMultiBody<Scalar, Utils> *mb = world.create_multi_body();
  init_compound_pendulum<Scalar, Utils>(
      *mb, world, static_cast<int>(params.size()), params);
  for (int t = 0; t < time_steps; ++t) {
    output_states[t].resize(2 * mb->dof());
    for (int i = 0; i < mb->dof(); ++i) {
      output_states[t][i] = mb->m_q[i];
      output_states[t][i + mb->dof()] = mb->m_qd[i];
    }
    mb->forward_dynamics(gravity);
    mb->integrate(Scalar(dt));
  }
}

template <int TimeSteps, int NumLinks, ResidualMode ResMode>
class PendulumEstimator
    : public TinyCeresEstimator<NumLinks, 2 * NumLinks, TimeSteps, ResMode> {
 public:
  typedef TinyCeresEstimator<NumLinks, 2 * NumLinks, TimeSteps, ResMode>
      CeresEstimator;
  using CeresEstimator::kStateDim, CeresEstimator::kParameterDim;
  using CeresEstimator::parameters, CeresEstimator::samples_every_n_frames;
  using typename CeresEstimator::ADScalar;

  // sane parameter initialization (link lengths)
  PendulumEstimator(double initial_link_length = 0.5) {
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i] = {"link_length_" + std::to_string(i + 1),
                       initial_link_length, 0.05, 1.};
    }
  }

  void rollout(
      const std::vector<ADScalar> &params,
      std::vector<std::vector<ADScalar>> &output_states) const override {
    rollout_pendulum<ADScalar, CeresUtils<kParameterDim>>(params, output_states,
                                                          TimeSteps * samples_every_n_frames);
  }
  void rollout(const std::vector<double> &params,
               std::vector<std::vector<double>> &output_states) const override {
    rollout_pendulum(params, output_states, TimeSteps * samples_every_n_frames);
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
  dt = 1. / 60;
  const int samples_every_n_frames = 2;
  const int time_steps = 495;
  const int param_dim = 2;
  double init_params = 0.2;
  PendulumEstimator<time_steps, param_dim, RES_MODE_TIME> estimator(
      init_params);
  // collect training data from the "real" system
  // rollout_pendulum(std::vector<double>{3., 4.}, estimator.target_states,
  //                  time_steps);

  std::string exp_filename;
  TinyFileUtils::find_file("schmidt-lipson-exp-data/real_double_linear_h_1.txt",
                           exp_filename);
  std::vector<double> times;
  bool success = load_schmidt_lipson(exp_filename, times, estimator.target_states);
  assert(success);
  printf("Success? %i\n", success);
  // recording rate is 30 Hz, simulation rate is 60 Hz
  estimator.samples_every_n_frames = samples_every_n_frames;
  // divide each cost term by integer time step ^ 2 to reduce gradient explosion
  estimator.divide_cost_by_time_step_factor = 60.;
  estimator.divide_cost_by_time_step_exponent = 1.;
  //  printf("Target states: ");
  //  print_states(estimator.target_states);
  estimator.setup(new ceres::HuberLoss(1.));
  // double cost;
  // double gradient[param_dim * time_steps];
  // estimator.compute_gradient(estimator.vars(), &cost, gradient);
  // std::ofstream gradient_file("estimation_gradient.csv");
  // for (int t = 0; t < time_steps; ++t) {
  //   for (int i = 0; i < param_dim; ++i) {
  //     gradient_file << gradient[t * param_dim + i];
  //     if (i < param_dim - 1)
  //       gradient_file << "\t";
  //   }
  //   gradient_file << "\n";
  // }
  // gradient_file.close();
  // return 0;

  auto summary = estimator.solve();
  std::cout << summary.FullReport() << std::endl;

  for (const auto &p : estimator.parameters) {
    printf("%s: %.3f\n", p.name.c_str(), p.value);
  }

  std::ofstream file("param_evolution.txt");
  for (const auto &params : estimator.parameter_evolution()) {
    for (int i = 0; i < static_cast<int>(params.size()); ++i) {
      file << params[i];
      if (i < static_cast<int>(params.size()) - 1) file << "\t";
    }
    file << "\n";
  }
  file.close();

  fflush(stdout);

  return EXIT_SUCCESS;
}

// int main(int argc, char* argv[]) {
//  const int time_steps = 50;
//  PendulumEstimator<time_steps, 2, RES_MODE_TIME> estimator;
//  // collect training data from the "real" system
//  rollout_pendulum(std::vector<double>{3., 4.}, estimator.target_states,
//                   time_steps);
//  //  printf("Target states: ");
//  //  print_states(estimator.target_states);
//
//  estimator.setup(new ceres::HuberLoss(1.));
//  estimator.options.minimizer_progress_to_stdout = false;
//
//  std::ofstream file("/home/eric/tinyrigidbody/grid_search.txt");
//
//  double delta = 0.1;
//  for (double link1 = 0.1; link1 <= 10.; link1 += delta) {
//    for (double link2 = 0.1; link2 <= 10.; link2 += delta) {
//      printf("Optimizing with theta = [%.3f, %.3f]\n", link1, link2);
//      estimator.parameters[0] = link1;
//      estimator.parameters[1] = link2;
//      auto summary = estimator.solve();
//      file << link1 << '\t' << link2 << '\t' << summary.final_cost << '\t'
//           << static_cast<double>(estimator.parameters[0]) << '\t'
//           << static_cast<double>(estimator.parameters[1]) << '\n';
//    }
//  }
//  file.close();
//
//  fflush(stdout);
//
//  return EXIT_SUCCESS;
//}

// int main(int argc, char *argv[])
// {

// // return 0;

//   const int time_steps = 500;
//   const int param_dim = 2;
//   typedef PendulumEstimator<time_steps, param_dim, RES_MODE_TIME> Estimator;
//   // collect training data from the "real" system
//   std::vector<std::vector<double>> target_states;
//   rollout_pendulum(std::vector<double>{3., 4.}, target_states, time_steps);

//   std::function<std::unique_ptr<Estimator>()> construct_estimator =
//       [&target_states]() {
//         auto estimator = std::make_unique<Estimator>();
//         estimator->target_states = target_states;
//         estimator->options.minimizer_progress_to_stdout = false;
//         return estimator;
//       };

//   std::array<double, param_dim> initial_guess{8, 4};
//   BasinHoppingEstimator<param_dim, Estimator> bhe(construct_estimator,
//                                                   initial_guess);
//   bhe.time_limit = 0.2;
//   bhe.run();

//   printf("Optimized parameters:");
//   for (int i = 0; i < param_dim; ++i)
//   {
//     printf(" %.8f", bhe.params[i]);
//   }
//   printf("\n");

//   fflush(stdout);

//   return EXIT_SUCCESS;
// }

// int main(int argc, char* argv[]) {
//  const int time_steps = 50;
//  const int param_dim = 2;
//  typedef PendulumEstimator<time_steps, param_dim, RES_MODE_TIME> Estimator;
//  // collect training data from the "real" system
//  std::vector<std::vector<double>> target_states;
//  rollout_pendulum(std::vector<double>{3., 4.}, target_states, time_steps);
//
//  std::function<std::unique_ptr<Estimator>()> construct_estimator =
//      [&target_states]() {
//        auto estimator = std::make_unique<Estimator>();
//        estimator->target_states = target_states;
//        estimator->options.minimizer_progress_to_stdout = false;
//        return estimator;
//      };
//
//  std::ofstream file("/home/eric/tinyrigidbody/grid_search_bhe.txt");
//
//  double delta = 0.1;
//  for (double link1 = 0.1; link1 <= 10.; link1 += delta) {
//    for (double link2 = 0.1; link2 <= 10.; link2 += delta) {
//      printf("Optimizing with theta = [%.3f, %.3f]\n", link1, link2);
//
//      std::array<double, param_dim> initial_guess{link1, link2};
//      BasinHoppingEstimator<param_dim, Estimator> bhe(construct_estimator,
//                                                      initial_guess);
//      bhe.time_limit = 0.2;
//      bhe.run();
//      file << link1 << '\t' << link2 << '\t' << bhe.best_cost() << '\t'
//           << static_cast<double>(bhe.params[0]) << '\t'
//           << static_cast<double>(bhe.params[1]) << '\n';
//    }
//  }
//  file.close();
//
//  fflush(stdout);
//
//  return EXIT_SUCCESS;
//}
