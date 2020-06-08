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

#include <fenv.h>

#include <deque>
#include <fstream>

#include "pybullet_visualizer_api.h"
typedef PyBulletVisualizerAPI VisualizerAPI;

#include "tiny_file_utils.h"
#include "tiny_gym_env.h"
#include "tiny_neural_network.h"
#include "tiny_policy_opt.h"
#include "tiny_smooth_constraint_solver.h"

std::string search_path;

const int time_steps = 200;
const bool stochastic_policy = false;

template <typename Scalar, typename Utils>
struct CheetahEnvironment
    : public TinyUrdfGymEnvironment<Scalar, Utils, 17, 6, TinyActuator> {
  typedef TinyUrdfGymEnvironment<Scalar, Utils, 17, 6, TinyActuator>
      GymEnvironment;
  using GymEnvironment::dt, GymEnvironment::set_action;
  using GymEnvironment::kStateDim, GymEnvironment::kActionDim;
  using GymEnvironment::render;
  using GymEnvironment::system, GymEnvironment::do_simulation;
  using GymEnvironment::world, GymEnvironment::frame_skip,
      GymEnvironment::gravity;

  explicit CheetahEnvironment(int frame_skip = 4)
      : GymEnvironment(
            TinySystemConstructor<>(search_path + "cheetah_link0_1.urdf",
                                    search_path + "plane_implicit.urdf", false,
                                    0.1, 0.1),
            Scalar(1e-3), frame_skip) {
    world.m_constraint_solver = new TinySmoothConstraintSolver<Scalar, Utils>;
  }

  void reset(std::vector<Scalar> &state) const {
    system->initialize();
    // TODO add randomness to state initialization?
    get_state(state);
  }

  void get_state(std::vector<Scalar> &state) const override {
    state.resize(kStateDim);
    for (int i = 1; i < system->dof(); ++i) {
      state[i - 1] = system->m_q[i];
    }
    for (int i = 0; i < system->dof_qd(); ++i) {
      state[i + system->dof() - 1] = system->m_qd[i];
    }
  }

  void set_action(const std::vector<Scalar> &action) override {
    for (int i = 0; i < kActionDim; ++i) {
      system->m_tau[i + 3] = 30. * action[i];
    }
  }

  static void clamp(Scalar &v, double low, double high) {
    using std::min, std::max;
    v = max(Scalar(low), min(Scalar(high), v));
  }

  void enforce_joint_limits(int i, double low, double high) {
    Scalar l(low), h(high);
    if (system->m_q[i] < l) {
      system->m_q[i] = l;
      system->m_tau[i] = Utils::zero();
      system->m_qd[i] = Utils::zero();
      system->m_qdd[i] = Utils::zero();
    } else if (system->m_q[i] > h) {
      system->m_q[i] = h;
      system->m_tau[i] = Utils::zero();
      system->m_qd[i] = Utils::zero();
      system->m_qdd[i] = Utils::zero();
    }
  }

  bool reached_joint_limits(int i, double low, double high) {
    Scalar l(low), h(high);
    return (system->m_q[i] < l || system->m_q[i] > h);
  }

  bool do_simulation2(VisualizerAPI *vis_api = nullptr) {
    for (int i = 0; i <= frame_skip; ++i) {
      system->forward_dynamics(gravity);
      system->integrate_q(dt);

      //      if (reached_joint_limits(3, -0.52, 1.05))
      //        return true;
      //      if (reached_joint_limits(4, -0.785, .78))
      //        return true;
      //      if (reached_joint_limits(5, -.4, .785))
      //        return true;
      //
      //      if (reached_joint_limits(6, -1, .7))
      //        return true;
      //      if (reached_joint_limits(7, -1.2, .87))
      //        return true;
      //      if (reached_joint_limits(8, -.5, .5))
      //        return true;

      world.step(dt);
      system->integrate(dt);
      if (vis_api) {
        render(vis_api);
      }
    }
    return false;
  }

  virtual void step(const std::vector<Scalar> &action, int step,
                    std::vector<Scalar> &next_state, Scalar &cost, bool &done,
                    VisualizerAPI *vis_api = nullptr) {
    // equivalent to step function from original gym environment
    // https://github.com/openai/gym/blob/master/gym/envs/mujoco/half_cheetah.py
    cost = Utils::zero();
    for (int i = 0; i < kActionDim; ++i) {
      cost += action[i] * action[i];
    }
    set_action(action);
    Scalar x_before = system->m_q[0];
    for (int i = 0; i <= frame_skip; ++i) {
      system->forward_dynamics(gravity);
      system->integrate_q(dt);

      //      if (reached_joint_limits(3, -0.52, 1.05))
      //        return true;
      //      if (reached_joint_limits(4, -0.785, .78))
      //        return true;
      //      if (reached_joint_limits(5, -.4, .785))
      //        return true;
      //
      //      if (reached_joint_limits(6, -1, .7))
      //        return true;
      //      if (reached_joint_limits(7, -1.2, .87))
      //        return true;
      //      if (reached_joint_limits(8, -.5, .5))
      //        return true;

      world.step(dt);
      system->integrate(dt);
      if (vis_api) {
        render(vis_api);
      }

      //      cost += 100. - system->m_qd[0];

      // additional cost terms
      //      cost += system->m_q[1] * system->m_q[1];
      //      cost += 2. * (0.1 - system->m_q[2] * system->m_q[2]);
      for (int i = 3; i < system->dof(); ++i) {
        cost += system->m_q[i] * system->m_q[i];
        cost += 10. * system->m_qd[i] * system->m_qd[i];
      }
      cost += 100. * system->m_qd[5] * system->m_qd[5];
      cost += 100. * system->m_qd[8] * system->m_qd[8];
      cost += 100. * system->m_q[5] * system->m_q[5];
      cost += 100. * system->m_q[8] * system->m_q[8];
    }
    //    bool done2 = do_simulation2(vis_api);
    Scalar x_after = system->m_q[0];
    Scalar vel_dist = 1. - (x_after - x_before) / dt;
    //    cost += 1. - (x_after - x_before) / dt;
    get_state(next_state);

    done = false;
    if (done) {
      //      cost += Scalar(1000.);
      done = false;
      //      return;
    }
  }
};

template <typename Scalar, typename Utils>
struct CartpoleEnvironment
    : public TinyUrdfGymEnvironment<Scalar, Utils, 6, 1, TinyActuator> {
  typedef TinyUrdfGymEnvironment<Scalar, Utils, 6, 1, TinyActuator>
      GymEnvironment;
  using GymEnvironment::dt, GymEnvironment::set_action;
  using GymEnvironment::kStateDim, GymEnvironment::kActionDim;
  using GymEnvironment::render;
  using GymEnvironment::system, GymEnvironment::do_simulation;
  using GymEnvironment::world, GymEnvironment::frame_skip,
      GymEnvironment::gravity;

 private:
  std::random_device rd_;
  std::mt19937 gen_{rd_()};

 public:
  explicit CartpoleEnvironment(int frame_skip = 0)
      : GymEnvironment(TinySystemConstructor<>(search_path + "cartpole.urdf",
                                               "", false, 0., 0.),
                       Scalar(1. / 60.), frame_skip) {}

  void reset(std::vector<Scalar> &state) {
    system->initialize();
    system->m_q[1] = Scalar(M_PI_2);
    // add randomness to state initialization
    if (stochastic_policy) {
      std::normal_distribution<double> d{0., 0.01};
      for (int i = 0; i < system->dof(); ++i) {
        system->m_q[i] += Scalar(d(this->gen_));
        system->m_qd[i] += Scalar(d(this->gen_));
      }
    }
    get_state(state);
  }

  void get_state(std::vector<Scalar> &state) const override {
    state.resize(kStateDim);
    state[0] = system->m_q[0];
    state[1] = Utils::cos1(system->m_q[1]);
    state[2] = Utils::sin1(system->m_q[1]);
    state[3] = system->m_qd[0];
    state[4] = Utils::cos1(system->m_qd[1]);
    state[5] = Utils::sin1(system->m_qd[1]);
  }

  void set_action(const std::vector<Scalar> &action) override {
    system->m_tau[0] = 15. * action[0];
  }

 private:
  std::normal_distribution<double> dist_{
      std::normal_distribution<double>(0., 1.)};

 public:
  virtual void step(const std::vector<Scalar> &action_src, int step,
                    std::vector<Scalar> &next_state, Scalar &cost, bool &done,
                    VisualizerAPI *vis_api = nullptr) {
    using std::exp;
    cost = Utils::zero();
    std::vector<Scalar> action(kActionDim);
    if (stochastic_policy) {
      Scalar sigmoid_std = exp(action_src[1]);
      sigmoid_std /= sigmoid_std + Utils::one();
      sigmoid_std *= Utils::one() * 0.01;
      action[0] = dist_(gen_) * sigmoid_std + action_src[0];
    } else {
      action = action_src;
    }
    for (int i = 0; i < kActionDim; ++i) {
      cost += 0.01 * action[i] * action[i];
    }
    set_action(action);
    Scalar x_before = system->m_q[0];
    for (int frame = 0; frame <= frame_skip; ++frame) {
      system->forward_dynamics(gravity);
      system->integrate_q(dt);

      world.step(dt);
      system->integrate(dt);
      //      system->print_state();
      if (vis_api) {
        render(vis_api);
      }
      //      for (int i = 0; i < system->dof(); ++i) {
      cost += 10. * system->m_q[0] * system->m_q[0];
      cost += 1. * system->m_qd[0] * system->m_qd[0];
      //      }
      //      cost += 0.1 * system->m_q[1] * system->m_q[1];
      Scalar height = (0.5 - system->get_world_com(1).z()) * 2.;
      // Scalar height = Utils::sin1(system->m_q[1]);
      //      if (step == 0) {
      //        printf("Height:  %.3f\n", Utils::getDouble(height));
      //      }
      cost += 10. * height * height;
      cost += 1. * system->m_qd[1] * system->m_qd[1];
    }
    // if (step >= time_steps - 2)
    //   cost /= std::sqrt(step);
    // if (step < time_steps - 50) cost = Utils::zero();
    // cost += 10. * system->m_q[0] * system->m_q[0];
    //    bool done2 = do_simulation2(vis_api);
    //    Scalar x_after = system->m_q[0];
    //    Scalar vel_dist = 1. - (x_after - x_before) / dt;
    //    cost += 1. - (x_after - x_before) / dt;
    get_state(next_state);

    cost /= time_steps;

    done = false;
    //    if (done) {
    //      //      cost += Scalar(1000.);
    //      done = false;
    //      //      return;
    //    }
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

VisualizerAPI *vis_api = nullptr, *sim_api = nullptr;

const int param_dim = 12;  // 30; //12;
typedef TinyPolicyOptimizer<param_dim, time_steps, CartpoleEnvironment>
    PolicyOptimizer;

std::unique_ptr<PolicyOptimizer> construct_policy_opt() {
  typedef CartpoleEnvironment<double, DoubleUtils> EnvironmentD;
  TinyNeuralNetworkSpecification policy_spec(EnvironmentD::kStateDim, true);
  //  policy_spec.add_linear_layer(NN_ACT_RELU, 3, true);
  //  policy_spec.add_linear_layer(NN_ACT_RELU, 16);
  //  policy_spec.add_linear_layer(NN_ACT_RELU, 16);
  int output_dim = stochastic_policy ? EnvironmentD::kActionDim * 2
                                     : EnvironmentD::kActionDim;
  policy_spec.add_linear_layer(NN_ACT_TANH, output_dim, false);

  auto optimizer = std::make_unique<PolicyOptimizer>(policy_spec);
  optimizer->environment_d.setup(sim_api, vis_api);
  optimizer->environment_ad.setup(sim_api, vis_api);

  printf("ParamDim: %i,  actual NN dim: %i,  biases: %i,  weights: %i\n\n",
         param_dim, policy_spec.num_parameters(), policy_spec.num_biases(),
         policy_spec.num_weights());
  return optimizer;
}

int main(int argc, char *argv[]) {
  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);

  char path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  search_path = path;

  std::string connection_mode = "gui";

  // Set NaN trap
  feenableexcept(FE_INVALID | FE_OVERFLOW);

  vis_api = new VisualizerAPI();
  vis_api->setTimeOut(1e30);
  printf("mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;
  vis_api->connect(mode);
  vis_api->resetSimulation();

  sim_api = new VisualizerAPI();
  sim_api->setTimeOut(1e30);
  sim_api->connect(eCONNECT_DIRECT);

  //  typedef CheetahEnvironment<double, DoubleUtils> EnvironmentD;
  typedef CartpoleEnvironment<double, DoubleUtils> EnvironmentD;
  auto optimizer = construct_policy_opt();
  optimizer->environment_d.setup(sim_api, vis_api);
  optimizer->environment_ad.setup(sim_api, sim_api);
  optimizer->options.line_search_direction_type =
      ceres::LineSearchDirectionType::LBFGS;
  // // return 0;

  optimizer->options.line_search_direction_type =
      ceres::LineSearchDirectionType::STEEPEST_DESCENT;
  optimizer->options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
  //  optimizer->setup(new ceres::HuberLoss(1.));
  optimizer->setup(NN_INIT_HE);

  // //  auto summary = optimizer->solve();
  // //  std::cout << summary.FullReport() << std::endl;
  // //  for (const auto& p : optimizer->parameters) {
  // //    printf("%s: %.3f\n", p.name.c_str(), p.value);
  // //  }

  std::array<double, param_dim> initial_guess;
  for (int i = 0; i < param_dim; ++i) {
    initial_guess[i] = optimizer->parameters[i].value;
  }
  // for (int i = 0; i < 6; ++i) {
  //   initial_guess[i + 6] += 0.01;
  // }
  BasinHoppingOptimizer<param_dim, PolicyOptimizer> bh_opt(
      &construct_policy_opt, initial_guess);
  bh_opt.time_limit = 60;

  bh_opt.run();

  //   int num_iterations = 50;
  //   int batch_size = 1;
  //   double gradient_limit = 30;
  //   int history_size = 15;
  //   TinyVectorX<double, DoubleUtils> params(param_dim, optimizer->vars());
  //   TinyVectorX<double, DoubleUtils> gradient(param_dim),
  //       gradient_accum(param_dim);
  //   std::deque<TinyVectorX<double, DoubleUtils>> gradient_history;
  //   double cost;
  //   for (int i = 0; i < num_iterations; ++i) {
  //     params.print("params");
  //     gradient_accum.set_zero();
  //     for (int j = 0; j < batch_size; ++j) {
  //       optimizer->compute_gradient(params.data(), &cost, gradient.data());
  //       if (j == 0) {
  //         printf("## Iteration %02d ## \tCost: %.3f\n", i, cost);
  //         fflush(stdout);
  //       }
  //       for (int j = 0; j < param_dim; ++j) {
  // //        gradient_accum += gradient * 1. / batch_size;
  //         gradient_accum[j] += std::max(-gradient_limit,
  //         std::min(gradient_limit, gradient[j])) * 1. / batch_size;
  //       }
  //     }
  //     gradient_accum.print("gradient_accum");
  //     gradient_history.push_back(gradient_accum);
  //     if (gradient_history.size() > history_size) {
  //       gradient_history.pop_front();
  //     }
  //     printf("   |Gradient|: %.6f\n", gradient_accum.length());
  //     for (int j = 0; j < param_dim; ++j) {
  //       double grad_j_avg = 0;
  //       for (const auto& grad : gradient_history) {
  //         grad_j_avg += grad[j] / gradient_history.size();
  //       }
  //       //      params[j] -= std::max(-gradient_limit,
  //       std::min(gradient_limit,
  //       //      -0.01 * gradient_accum[j]));
  //       if (std::abs(grad_j_avg) > 1e-5) {
  //         // simple RMS Prop
  //         params[j] -= 0.05 * std::max(-gradient_limit,
  //         std::min(gradient_limit, gradient_accum[j] / grad_j_avg));
  //       }
  //     }
  //     //      if (gradient.length() < 1e-4) {
  //     //        printf("Converged.\n");
  //     //        break;
  //     //      }
  //   }

  {
    // play back policy
    // sim_api->resetSimulation();
    // vis_api->resetSimulation();
    // optimizer->environment_d.setup(sim_api, vis_api, true);
    // std::vector<double> params(optimizer->vars(),
    //                            optimizer->vars() + param_dim);
    std::vector<double> params(bh_opt.params.begin(), bh_opt.params.end());

    //   printf("Test:\n");
    // for (int i = 0; i < 6; ++i) {
    //   params[i] = 1.0;
    //   params[6 + i] = 1.;
    // }
    //       TinyNeuralNetwork<double, DoubleUtils>
    //       policy(optimizer->policy_spec);
    // policy.set_parameters(params);
    // std::vector<double> output(1, 0.0);
    // policy.compute({0., 0., 0., 0., 0., 0.}, output);
    // printf("output: %f\n", output[0]);

    printf("params:");
    for (int i = 0; i < param_dim; ++i) {
      if (i % 6 == 0) printf("\n\t");
      printf("%.3f  ", params[i]);
    }
    printf("\n");

    std::vector<double> state, action;
    double cost;
    bool done;
    while (vis_api->isConnected()) {
      optimizer->environment_d.reset(state);
      optimizer->environment_d.system->print_state();
      TinyNeuralNetwork<double, DoubleUtils> policy(optimizer->policy_spec);
      policy.set_parameters(params);
      // policy.initialize();
      for (int step = 0; step < time_steps; ++step) {
        policy.compute(state, action);
        optimizer->environment_d.step(action, step, state, cost, done, vis_api);
        // printf("step %i  --  cost: %.5f\n", step, cost);
        // optimizer->environment_d.system->print_state();
        if (done) {
          break;
        }
      }
    }
  }

  //  std::ofstream file("/home/eric/tinyrigidbody/param_evolution.txt");
  //  for (const auto& params : optimizer->parameter_evolution()) {
  //    for (int i = 0; i < static_cast<int>(params.size()); ++i) {
  //      file << params[i];
  //      if (i < static_cast<int>(params.size()) - 1) file << "\t";
  //    }
  //    file << "\n";
  //  }
  //  file.close();

  fflush(stdout);

  return EXIT_SUCCESS;
}

// int main(int argc, char* argv[]) {
//  const int time_steps = 50;
//  PendulumEstimator<time_steps, 2, RES_MODE_TIME> optimizer;
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
//  std::array<double, param_dim> initial_guess{8, 4};
//  BasinHoppingEstimator<param_dim, Estimator> bhe(construct_estimator,
//                                                  initial_guess);
//  bhe.time_limit = 0.2;
//  bhe.run();
//
//  printf("Optimized parameters:");
//  for (int i = 0; i < param_dim; ++i) {
//    printf(" %.8f", bhe.params[i]);
//  }
//  printf("\n");
//
//  fflush(stdout);
//
//  return EXIT_SUCCESS;
//}

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
