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
#include "tiny_multi_body.h"
#include "tiny_world.h"

template <typename Scalar = double, typename Utils = DoubleUtils>
void rollout_pendulum(const std::vector<Scalar>& params,
                      std::vector<std::vector<Scalar>>& output_states,
                      int time_steps) {
  static const Scalar dt = Utils::fraction(1, 60);
  TinyVector3<Scalar, Utils> gravity(Utils::zero(), Utils::zero(),
                                     Utils::fraction(-981, 100));
  output_states.resize(time_steps);
  TinyWorld<Scalar, Utils> world;
  TinyMultiBody<Scalar, Utils>* mb = world.create_multi_body();
  init_compound_pendulum<Scalar, Utils>(
      *mb, world, static_cast<int>(params.size()), params);
  for (int t = 0; t < time_steps; ++t) {
    output_states[t] = mb->m_q;
    mb->forward_dynamics(gravity);
    mb->integrate(dt);
  }
}

template <int TimeSteps, int NumLinks, ResidualMode ResMode>
class PendulumEstimator
    : public TinyCeresEstimator<NumLinks, NumLinks, TimeSteps, ResMode> {
 public:
  typedef TinyCeresEstimator<NumLinks, NumLinks, TimeSteps, ResMode>
      CeresEstimator;
  using CeresEstimator::kStateDim, CeresEstimator::kParameterDim;
  using CeresEstimator::parameters;
  using typename CeresEstimator::ADScalar;

  // sane parameter initialization (link lengths)
  PendulumEstimator(double initial_link_length = 0.5) {
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i] = {"link_length_" + std::to_string(i + 1),
                       initial_link_length, 0.1, 10.};
    }
  }

  void rollout(
      const std::vector<ADScalar>& params,
      std::vector<std::vector<ADScalar>>& output_states) const override {
    rollout_pendulum<ADScalar, CeresUtils<kParameterDim>>(params, output_states,
                                                          TimeSteps);
  }
  void rollout(const std::vector<double>& params,
               std::vector<std::vector<double>>& output_states) const override {
    rollout_pendulum(params, output_states, TimeSteps);
  }
};

void print_states(const std::vector<std::vector<double>>& states) {
  for (const auto& s : states) {
    for (double d : s) {
      printf("%.2f ", d);
    }
    printf("\n");
  }
}

// int main(int argc, char* argv[]) {
//  const int time_steps = 50;
//  double init_params = 0.5;
//  PendulumEstimator<time_steps, 2, RES_MODE_TIME> estimator(init_params);
//  // collect training data from the "real" system
//  rollout_pendulum(std::vector<double>{3., 4.}, estimator.target_states,
//                   time_steps);
//  //  printf("Target states: ");
//  //  print_states(estimator.target_states);
//  estimator.setup(new ceres::HuberLoss(1.));
//  auto summary = estimator.solve();
//  std::cout << summary.FullReport() << std::endl;
//
//  for (const auto& p : estimator.parameters) {
//    printf("%s: %.3f\n", p.name.c_str(), p.value);
//  }
//
//  std::ofstream file("param_evolution.txt");
//  for (const auto& params : estimator.parameter_evolution()) {
//    for (int i = 0; i < static_cast<int>(params.size()); ++i) {
//      file << params[i];
//      if (i < static_cast<int>(params.size()) - 1) file << "\t";
//    }
//    file << "\n";
//  }
//  file.close();
//
//  fflush(stdout);
//
//  return EXIT_SUCCESS;
//}

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
//  std::ofstream file("grid_search.txt");
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

int main(int argc, char* argv[]) {
  const int time_steps = 50;
  const int param_dim = 2;
  typedef PendulumEstimator<time_steps, param_dim, RES_MODE_TIME> Estimator;
  // collect training data from the "real" system
  std::vector<std::vector<double>> target_states;
  rollout_pendulum(std::vector<double>{3., 4.}, target_states, time_steps);

  std::function<std::unique_ptr<Estimator>()> construct_estimator =
      [&target_states]() {
        auto estimator = std::make_unique<Estimator>();
        estimator->target_states = target_states;
        estimator->options.minimizer_progress_to_stdout = false;
        return estimator;
      };

  std::array<double, param_dim> initial_guess{8, 4};
  BasinHoppingEstimator<param_dim, Estimator> bhe(construct_estimator,
                                                  initial_guess);
  bhe.time_limit = 0.2;
  bhe.run();

  printf("Optimized parameters:");
  for (int i = 0; i < param_dim; ++i) {
    printf(" %.8f", bhe.params[i]);
  }
  printf("\n");

  fflush(stdout);

  return EXIT_SUCCESS;
}

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
//  std::ofstream file("grid_search_bhe.txt");
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
