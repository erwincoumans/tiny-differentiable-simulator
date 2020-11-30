/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <ceres/ceres.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "math/tiny/ceres_utils.h"
#include "math/tiny/tiny_double_utils.h"
#include "parameter.hpp"

// #define USE_MATPLOTLIB 1

#ifdef USE_MATPLOTLIB
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

namespace tds {
enum ResidualMode { RES_MODE_1D, RES_MODE_STATE };

template <int ParameterDim, int StateDim, ResidualMode ResMode = RES_MODE_1D>
class CeresEstimatorOld : ceres::IterationCallback {
 public:
  static const int kParameterDim = ParameterDim;
  static const int kStateDim = StateDim;
  static const ResidualMode kResidualMode = ResMode;
  static const int kResidualDim = kResidualMode == RES_MODE_1D ? 1 : kStateDim;

  static_assert(kParameterDim >= 1);
  static_assert(kStateDim >= 1);

  typedef ceres::Jet<double, kParameterDim> ADScalar;

  std::array<EstimationParameter, kParameterDim> parameters;

  void set_params(const std::array<double, kParameterDim> &params) {
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i].value = params[i];
    }
  }

  // Reference trajectories from ground-truth system dynamics, i.e. a list of
  // states.
  std::vector<std::vector<std::vector<double>>> target_trajectories;

  // Times from ground-truth trajectory, i.e. when the target_trajectories
  // happend, may be left empty (then target_trajectories are assumed to overlap
  // with simulation states).
  std::vector<std::vector<double>> target_times;

  std::size_t minibatch_size{1};

  mutable double dt{1e-2};

  bool use_finite_diff{false};

  /**
   * The following 2 terms are for optional time normalization to mitigate
   * gradient explosion over long roll-outs: c_t' = c_t / (a * t)^b For cost
   * term `c_t` at time step t. If a is zero, no normalization is performed.
   */

  /**
   * Factor `a` to use to divide each cost term by the particular time step t to
   * mitigate gradient explosion over long roll-outs. If it is zero, no time
   * normalization is performed.
   */
  double divide_cost_by_time_factor{0};
  /**
   * Exponent `b` to use to divide each cost term by the particular time step t
   * to mitigate gradient explosion over long roll-outs.
   */
  double divide_cost_by_time_exponent{1};

  /**
   * Whether to set parameter bounds (line search optimizers do not support
   * this).
   */
  bool set_bounds{true};

  /**
   * Cost penalty for every roll-out state that is outside sensible limits.
   */
  static inline double nonfinite_penalty{1e3};

  ceres::Solver::Options options;

  CeresEstimatorOld(double dt) : dt(dt) {
    options.minimizer_progress_to_stdout = true;
    options.callbacks.push_back(this);
  }

 private:
  ceres::CostFunction *cost_function_{nullptr};

 public:
  virtual void rollout(const std::vector<ADScalar> &params,
                       std::vector<std::vector<ADScalar>> &output_states,
                       double &dt, std::size_t ref_id) const = 0;
  virtual void rollout(const std::vector<double> &params,
                       std::vector<std::vector<double>> &output_states,
                       double &dt, std::size_t ref_id) const = 0;

  virtual ceres::Problem &setup(ceres::LossFunction *loss_function = nullptr) {
    assert(!target_trajectories.empty() &&
           static_cast<int>(target_trajectories[0].size()) >= kStateDim);

    if (cost_function_) {
      delete cost_function_;
    }

    if (target_times.size() != target_trajectories.size()) {
      target_times.resize(target_trajectories.size(), {});
    }

    if (use_finite_diff) {
      cost_function_ =
          new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL,
                                             kResidualDim, kParameterDim>(
              new CostFunctor(this));
    } else {
      cost_function_ =
          new ceres::AutoDiffCostFunction<CostFunctor, kResidualDim,
                                          kParameterDim>(new CostFunctor(this));
    }

    if (vars_) {
      delete[] vars_;
    }
    vars_ = new double[kParameterDim];
    problem_.AddResidualBlock(cost_function_, loss_function, vars_);

    for (int i = 0; i < kParameterDim; ++i) {
      vars_[i] = parameters[i].value;
    }
    if (set_bounds) {
      for (int i = 0; i < kParameterDim; ++i) {
        problem_.SetParameterLowerBound(vars_, i, parameters[i].minimum);
        problem_.SetParameterUpperBound(vars_, i, parameters[i].maximum);
      }
    }

    return problem_;
  }

  void compute_loss(const double *parameters, double *cost,
                    double *gradient) const {
    double const *const *params = &parameters;
    cost_function_->Evaluate(params, cost, &gradient);
  }

  void gradient_descent(double learning_rate, int iterations) {
    double gradient[kParameterDim];
    double cost;
    param_evolution_.clear();
    for (int i = 0; i < iterations; ++i) {
      compute_loss(vars_, &cost, gradient);
      printf("Gradient descent step %i - cost: %.6f\n", i, cost);
      for (int j = 0; j < kParameterDim; ++j) {
        current_param_[j] = vars_[j];
        vars_[j] -= learning_rate * gradient[j];
      }
      param_evolution_.push_back(current_param_);
    }
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i].value = vars_[i];
    }
  }

  ceres::Solver::Summary solve() {
    ceres::Solver::Summary summary;
    param_evolution_.clear();
    best_cost_ = std::numeric_limits<double>::max();
    for (int i = 0; i < kParameterDim; ++i) {
      vars_[i] = parameters[i].value;
      best_params_[i] = parameters[i].value;
    }
    ceres::Solve(options, &problem_, &summary);
    if (summary.final_cost > best_cost_) {
      printf(
          "Ceres returned a parameter vector with a final cost of %.8f whereas "
          "during the optimization a parameter vector with a lower cost of "
          "%.8f was found. Returning the best parameter vector.\n",
          summary.final_cost, best_cost_);
      for (int i = 0; i < kParameterDim; ++i) {
        parameters[i].value = best_params_[i];
      }
    } else {
      for (int i = 0; i < kParameterDim; ++i) {
        parameters[i].value = vars_[i];
      }
    }
    return summary;
  }

  const double *vars() const { return vars_; }

  virtual ~CeresEstimatorOld() {
    if (vars_) {
      delete[] vars_;
      vars_ = nullptr;
    }
  }

  const std::vector<std::array<double, kParameterDim>> &parameter_evolution()
      const {
    return param_evolution_;
  }

  double best_cost() const { return best_cost_; }

  const std::array<double, kParameterDim> &best_parameters() const {
    return best_params_;
  }

 private:
  ceres::Problem problem_;
  double *vars_{nullptr};
  std::vector<std::array<double, kParameterDim>> param_evolution_;
  mutable std::array<double, kParameterDim> current_param_;

  mutable double best_cost_{std::numeric_limits<double>::max()};
  mutable std::array<double, kParameterDim> best_params_;

  struct CostFunctor {
    CeresEstimatorOld *parent{nullptr};

    mutable std::vector<std::size_t> ref_indices;

    CostFunctor(CeresEstimatorOld *parent) : parent(parent) {
      ref_indices.resize(parent->target_trajectories.size());
      for (std::size_t i = 0; i < parent->target_trajectories.size(); ++i) {
        ref_indices[i] = i;
      }
    }

#ifdef USE_MATPLOTLIB
    template <typename T>
    static void plot_trajectory(const std::vector<std::vector<T>> &states,
                                const std::string &title = "Figure") {
      typedef std::conditional_t<std::is_same_v<T, double>, TINY::DoubleUtils,
                                 CeresUtils<kParameterDim>>
          Utils;
      for (int i = 0; i < static_cast<int>(states[0].size()); ++i) {
        std::vector<double> traj(states.size());
        for (int t = 0; t < static_cast<int>(states.size()); ++t) {
          traj[t] = Utils::getDouble(states[t][i]);
        }
        plt::named_plot("state[" + std::to_string(i) + "]", traj);
      }
      plt::legend();
      plt::title(title);
      plt::show();
    }
#endif

    template <typename T>
    static void print_states(const std::vector<std::vector<T>> &states) {
      typedef std::conditional_t<std::is_same_v<T, double>, TINY::DoubleUtils,
                                 CeresUtils<kParameterDim>>
          Utils;
      for (const auto &s : states) {
        for (const T &d : s) {
          printf("%.2f ", Utils::getDouble(d));
        }
        printf("\n");
      }
    }

    // Computes the cost (residual) for input parameters x.
    template <typename T>
    bool operator()(const T *const x, T *residual) const {
      static thread_local int num_evaluations = 0;
      ++num_evaluations;
      // ref_indices[0] = 26; //92;
      // if (ref_indices.size() > 1) {
      //   // shuffle indices before minibatching
      //   std::random_shuffle(ref_indices.begin(), ref_indices.end());
      // }

      // select the right scalar traits based on the type of the input
      typedef std::conditional_t<std::is_same_v<T, double>, TINY::DoubleUtils,
                                 CeresUtils<kParameterDim>>
          Utils;
      T regularization = Utils::zero();
      for (int i = 0; i < kParameterDim; ++i) {
        // store current parameters as double for logging purposes
        parent->current_param_[i] = Utils::getDouble(x[i]);
        // apply regularization
        regularization += parent->parameters[i].l2_regularization * x[i] * x[i];
        regularization +=
            parent->parameters[i].l1_regularization * Utils::abs(x[i]);
      }
      for (int i = 0; i < kResidualDim; ++i) {
        // TODO consider adding separate residual dimension for parameter
        // regularization
        residual[i] = regularization;
      }
      int nonfinite = 0;
      const std::vector<T> params(x, x + kParameterDim);

      std::string ref_id_str = "ref_id:";

      for (std::size_t traj_id = 0; traj_id < parent->minibatch_size;
           ++traj_id) {
        const std::size_t ref_id = ref_indices[traj_id];

        // first roll-out simulation given the current parameters
        std::vector<std::vector<T>> rollout_states;
        double &dt = parent->dt;
        parent->rollout(params, rollout_states, dt, ref_id);
        // printf("dt: %.6f\n", dt);
        // plot_trajectory(rollout_states);
        // printf("Rollout states:\n");
        // print_states(rollout_states);

        ref_id_str += " " + std::to_string(ref_id);

        const auto &target_states = parent->target_trajectories[ref_id];
        // plot_trajectory(target_states);
        const auto &target_times = parent->target_times[ref_id];
        int n_rollout = static_cast<int>(rollout_states.size());
        int n_target = static_cast<int>(target_states.size());

        if (target_times.empty() && n_rollout != n_target) {
          fprintf(stderr,
                  "If no target_times are provided to CeresEstimator, the "
                  "number of target_trajectories (%i) must match the number of "
                  "roll-out states (%i).\n",
                  n_target, n_rollout);
          return false;
        }

        T difference;

        std::vector<T> rollout_state(kStateDim, Utils::zero());
        double time;
        std::vector<std::vector<double>> error_evolution;
        // plot_trajectory(target_states, "target_states");
        // plot_trajectory(rollout_states, "rollout_states");
        for (int t = 0; t < n_target; ++t) {
          if (target_times.empty()) {
            // assume target states line up with rollout states
            rollout_state = rollout_states[t];
            time = dt * t;
          } else {
            // linear interpolation of rollout states at the target times
            double target_time = target_times[t];
            // numerically stable way to get index of rollout state
            int rollout_i =
                static_cast<int>(std::floor(target_time / dt + 0.5));
            if (rollout_i >= n_rollout - 1) {
              // fprintf(stderr,
              //         "Target time %.4f (step %i) corresponds to a state
              //         (%i) " "that has not been rolled out.\n",
              //         target_time, t, rollout_i);
              break;
            }
            double alpha = (target_time - rollout_i * dt) / dt;
            const std::vector<T> &left = rollout_states[rollout_i];
            const std::vector<T> &right = rollout_states[rollout_i + 1];

            // printf("left=%f\t", Utils::getDouble(left[2]));
            // printf("right=%f\t", Utils::getDouble(right[2]));

            for (int i = 0; i < kStateDim; ++i) {
              rollout_state[i] = (1. - alpha) * left[i] + alpha * right[i];
            }
            time = target_time;
          }

          // skip time steps for which no target state samples exist
          std::vector<double> error_state(kStateDim);
          for (int i = 0; i < kStateDim; ++i) {
            difference = target_states[t][i] - rollout_state[i];
            difference *= difference;
            double dd = Utils::getDouble(difference);
            if (std::isinf(dd) || std::isnan(dd)) {
              //               printf("!!d!! %f\t", dd);
              //               printf("target_states[t][i]=%f\t",
              //               target_states[t][i]);
              //               printf("rollout_state[i]=%f\t",
              //                      Utils::getDouble(rollout_state[i]));
              // #ifdef USE_MATPLOTLIB
              //               plot_trajectory(rollout_states);
              // #endif
              ++nonfinite;
              continue;
            } else if (std::abs(dd) > 1e10) {
              ++nonfinite;
              printf(" NONFINITE!!! ");
              // #ifdef USE_MATPLOTLIB
              //               plot_trajectory(rollout_states);
              // #endif
              continue;
            }
            // printf("%.3f  ", Utils::getDouble(difference));

            // discount contribution of errors at later time steps to mitigate
            // gradient explosion on long roll-outs
            // if (parent->divide_cost_by_time_factor != 0. && t > 0) {
            //   difference /= std::pow(parent->divide_cost_by_time_factor *
            //   time,
            //                          parent->divide_cost_by_time_exponent);
            // }

            if constexpr (kResidualMode == RES_MODE_1D) {
              *residual += difference / T(double(parent->minibatch_size));
            } else if constexpr (kResidualMode == RES_MODE_STATE) {
              residual[i] += difference / T(double(parent->minibatch_size));
            }
            error_state[i] = dd;
          }
          error_evolution.push_back(error_state);
          // printf("[[res: %.3f]]  ", Utils::getDouble(*residual));
        }

        printf("params: ");
        for (int ri = 0; ri < kParameterDim; ++ri) {
          printf("%.4f  ", Utils::getDouble(params[ri]));
        }
        printf("\tresidual: ");
        for (int ri = 0; ri < kResidualDim; ++ri) {
          printf("%.6f  ", Utils::getDouble(residual[ri]));
        }
        printf("\t%s", ref_id_str.c_str());
        if (nonfinite > 0) {
          std::cerr << "\tnonfinite: " << nonfinite;
          for (int ri = 0; ri < kResidualDim; ++ri) {
            residual[ri] += nonfinite * nonfinite_penalty;
          }
        }
        // std::cout << "  thread ID: " << std::this_thread::get_id();
        printf("\n");
      }

      if constexpr (kResidualMode == RES_MODE_1D) {
        double res = Utils::getDouble(*residual);
        if (res < parent->best_cost_) {
          if (num_evaluations > 1) {
            printf("Found new best cost %.6f < %.6f\n", res,
                   parent->best_cost_);
          }
          for (int ri = 0; ri < kParameterDim; ++ri) {
            parent->best_params_[ri] = Utils::getDouble(params[ri]);
          }
          parent->best_cost_ = res;
        }
      }
      // plot_trajectory(error_evolution);
      // plt::named_plot("difference", error_evolution);
      // plt::show();

      // if (parent->options.minimizer_progress_to_stdout &&
      // Utils::getDouble(residual[0]) < 0.) {
      // } else {
      //   printf("\tcost: %.6f  nonfinite: %d\n",
      //   Utils::getDouble(residual[0]), nonfinite);
      // }
      return true;
    }
  };

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary &summary) override {
    param_evolution_.push_back(current_param_);
    return ceres::SOLVER_CONTINUE;
  }
};

/**
 * Implements Parallel Basin Hopping that combines local gradient-based
 * optimization using Ceres with random guessing to overcome poor local
 * optima.
 *
 * McCarty & McGuire "Parallel Monotonic Basin Hopping for Low Thrust
 * Trajectory Optimization"
 */
template <int ParameterDim, typename Estimator>
class BasinHoppingEstimator {
  static const int kParameterDim = ParameterDim;
  typedef std::function<std::unique_ptr<Estimator>()> EstimatorConstructor;

 public:
  EstimatorConstructor estimator_constructor;
  std::array<double, kParameterDim> params;
  std::size_t num_workers;

  /**
   * Time limit in seconds.
   */
  double time_limit{1.0};

  /**
   * Terminate if estimation cost drops below this value.
   */
  double cost_limit{1e-3};

  /**
   * Initial standard deviation used for Gaussian noise applied to the
   * parameters, normalized by the bounds of the parameter.
   */
  double initial_std{1.};

  /**
   * Whether to reduce the standard deviation of the random guess in the
   * parameter as the iteration count increases.
   */
  bool fade_std{false};

  BasinHoppingEstimator(
      const EstimatorConstructor &estimator_constructor,
      const std::array<double, kParameterDim> &initial_guess,
      std::size_t num_workers = std::thread::hardware_concurrency())
      : estimator_constructor(estimator_constructor),
        params(initial_guess),
        num_workers(num_workers) {
    workers_.reserve(num_workers);
  }

  // Run global optimizer.
  void run() {
    using namespace std::chrono;
    best_cost_ = std::numeric_limits<double>::max();
    std::cout << "Starting " << num_workers << " worker(s).\n";
    auto start_time = high_resolution_clock::now();
    for (std::size_t k = 0; k < num_workers; ++k) {
      workers_.emplace_back([this, k, &start_time]() {
        std::seed_seq seed{
            // Time
            static_cast<std::size_t>(std::chrono::high_resolution_clock::now()
                                         .time_since_epoch()
                                         .count()),
            // counter
            k};

        std::mt19937 eng(seed);

        auto estimator = this->estimator_constructor();
        estimator->setup(new ceres::TrivialLoss);  // new ceres::HuberLoss(1.));
                                                   // // TODO expose this
        if (k == 0) {
          // set initial guess
          estimator->set_params(this->params);
        } else {
          for (auto &p : estimator->parameters) {
            p = p.random_value();
          }
        }
        for (int iter = 0;; ++iter) {
          {
            std::unique_lock<std::mutex> lock(this->mutex_);
            // check stopping criteria
            auto stop_time = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(stop_time - start_time);
            bool time_up = static_cast<long>(duration.count()) >=
                           static_cast<long>(time_limit * 1e3);
#ifdef DEBUG
            if (time_up) {
              std::cout << "time up\n";
            }
            if (this->best_cost_ < this->cost_limit) {
              std::cout << "this->best_cost_ < this->cost_limit\n";
            }
            if (this->stop_) {
              std::cout << "this->stop_\n";
            }
#endif
            if (time_up || this->stop_ || this->best_cost_ < this->cost_limit) {
              std::cout << "Thread " << k << " has terminated after " << iter
                        << " iterations. ";
              printf("time up? %d  stop? %d  best cost? %d\n", time_up,
                     this->stop_, this->best_cost_ < this->cost_limit);
              return;
            }
          }
          double &solver_time_limit =
              estimator->options.max_solver_time_in_seconds;
          if (solver_time_limit > time_limit) {
            solver_time_limit = time_limit;
          }
          auto summary = estimator->solve();
          std::cout << summary.FullReport() << std::endl;
          {
            std::unique_lock<std::mutex> lock(this->mutex_);
            if (estimator->best_cost() < this->best_cost_) {
              this->best_cost_ = estimator->best_cost();
              printf("FOUND NEW BEST COST: %.6f\n", estimator->best_cost());
              for (int i = 0; i < kParameterDim; ++i) {
                this->params[i] = estimator->parameters[i].value;
              }
            }
          }
          // apply random change to the parameters
          for (int i = 0; i < kParameterDim; ++i) {
            auto &param = estimator->parameters[i];
            if (fade_std) {
              std::normal_distribution<double> d{
                  this->params[i],
                  initial_std / (iter + 1.) * (param.maximum - param.minimum)};
              param.value = d(eng);
            } else {
              std::normal_distribution<double> d{
                  this->params[i],
                  initial_std * (param.maximum - param.minimum)};
              param.value = d(eng);
            }
            param.value = std::max(param.minimum, param.value);
            param.value = std::min(param.maximum, param.value);
          }
        }
      });
    }
    for (auto &worker : workers_) {
      worker.join();
    }
    workers_.clear();
  }

  void stop() {
    std::lock_guard<std::mutex> lock{mutex_};
    stop_ = true;
  }

  virtual ~BasinHoppingEstimator() {
    stop();
    for (auto &worker : workers_) {
      worker.join();
    }
  }

  double best_cost() const { return best_cost_; }

 private:
  std::vector<std::thread> workers_;
  std::mutex mutex_;
  bool stop_{false};

  double best_cost_;
};
}  // namespace tds
