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

#ifndef TINY_CERES_ESTIMATOR_H
#define TINY_CERES_ESTIMATOR_H

#include <ceres/ceres.h>

#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "ceres_utils.h"
#include "tiny_double_utils.h"

#ifdef USE_MATPLOTLIB
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

struct EstimationParameter {
  std::string name{"unnamed_param"};
  double value{1.0};
  double minimum{-std::numeric_limits<double>::infinity()};
  double maximum{std::numeric_limits<double>::infinity()};

  // coefficient of L2 regularization for this parameter
  double regularization{0.};

  EstimationParameter &operator=(double rhs) {
    value = rhs;
    return *this;
  }
  explicit operator double() const { return value; }

  double random_value() const {
    return minimum + (double(rand()) / RAND_MAX * (maximum - minimum));
  };
};

enum ResidualMode { RES_MODE_1D, RES_MODE_STATE };

template <int ParameterDim, int StateDim, ResidualMode ResMode = RES_MODE_1D>
class TinyCeresEstimator : ceres::IterationCallback {
 public:
  static const int kParameterDim = ParameterDim;
  static const int kStateDim = StateDim;
  static const ResidualMode kResidualMode = ResMode;
  static const int kResidualDim = kResidualMode == RES_MODE_1D ? 1 : kStateDim;

  static_assert(kParameterDim >= 1);
  static_assert(kStateDim >= 1);

  typedef ceres::Jet<double, kParameterDim> ADScalar;

  typedef TinyCeresEstimator<kParameterDim, kStateDim, kResidualMode>
      CeresEstimator;

  std::array<EstimationParameter, kParameterDim> parameters;

  void set_params(const std::array<double, kParameterDim> &params) {
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i].value = params[i];
    }
  }

  // Sample states from ground-truth system dynamics, i.e. a list of states.
  std::vector<std::vector<double>> target_states;

  // Times from ground-truth trajectory, i.e. when the target_states happend,
  // may be left empty (then target_states are assumed to overlap with
  // simulation states).
  std::vector<double> target_times;

  double dt{1e-2};

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

  ceres::Solver::Options options;

  TinyCeresEstimator(double dt) : dt(dt) {
    options.minimizer_progress_to_stdout = true;
    options.callbacks.push_back(this);
  }

 private:
  ceres::CostFunction *cost_function_{nullptr};

 public:
  virtual void rollout(const std::vector<ADScalar> &params,
                       std::vector<std::vector<ADScalar>> &output_states,
                       double dt) const = 0;
  virtual void rollout(const std::vector<double> &params,
                       std::vector<std::vector<double>> &output_states,
                       double dt) const = 0;

  ceres::Problem &setup(ceres::LossFunction *loss_function = nullptr) {
    assert(!target_states.empty() &&
           static_cast<int>(target_states[0].size()) >= kStateDim);

    if (cost_function_) {
      delete cost_function_;
    }
    cost_function_ =
        new ceres::AutoDiffCostFunction<CostFunctor, kResidualDim,
                                        kParameterDim>(new CostFunctor(this));
    if (vars_) {
      delete[] vars_;
    }
    vars_ = new double[kParameterDim];
    problem_.AddResidualBlock(cost_function_, loss_function, vars_);

    for (int i = 0; i < kParameterDim; ++i) {
      problem_.SetParameterLowerBound(vars_, i, parameters[i].minimum);
      problem_.SetParameterUpperBound(vars_, i, parameters[i].maximum);
      vars_[i] = parameters[i].value;
    }

    return problem_;
  }

  void compute_loss(const double *parameters, double *cost,
                    double *gradient) const {
    double const *const *params = &parameters;
    cost_function_->Evaluate(params, cost, &gradient);
  }

  ceres::Solver::Summary solve() {
    ceres::Solver::Summary summary;
    param_evolution_.clear();
    for (int i = 0; i < kParameterDim; ++i) {
      vars_[i] = parameters[i].value;
    }
    ceres::Solve(options, &problem_, &summary);
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i].value = vars_[i];
    }
    return summary;
  }

  const double *vars() const { return vars_; }

  virtual ~TinyCeresEstimator() {
    if (vars_) {
      delete[] vars_;
      vars_ = nullptr;
    }
  }

  const std::vector<std::array<double, kParameterDim>> &parameter_evolution()
      const {
    return param_evolution_;
  }

 private:
  ceres::Problem problem_;
  double *vars_{nullptr};
  std::vector<std::array<double, kParameterDim>> param_evolution_;
  mutable std::array<double, kParameterDim> current_param_;

  struct CostFunctor {
    CeresEstimator *parent{nullptr};

    CostFunctor(CeresEstimator *parent) : parent(parent) {}

#ifdef USE_MATPLOTLIB
    template <typename T>
    void plot_trajectory(const std::vector<std::vector<T>> &states) const {
      typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
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
      plt::show();
    }
#endif

    // Computes the cost (residual) for input parameters x.
    template <typename T>
    bool operator()(const T *const x, T *residual) const {
      // first roll-out simulation given the current parameters
      const std::vector<T> params(x, x + kParameterDim);
      std::vector<std::vector<T>> rollout_states;
      const double dt = parent->dt;
      parent->rollout(params, rollout_states, dt);
      // plot_trajectory(rollout_states);

      const auto &target_states = parent->target_states;
      // plot_trajectory(target_states);
      const auto &target_times = parent->target_times;
      int n_rollout = static_cast<int>(rollout_states.size());
      int n_target = static_cast<int>(target_states.size());
      if (target_times.empty() &&
          rollout_states.size() != target_states.size()) {
        fprintf(
            stderr,
            "If no target_times are provided to TinyCeresEstimator, the "
            "number of target_states (%i) must match the number of roll-out "
            "states (%i).\n",
            n_target, n_rollout);
        return false;
      }

      // select the right scalar traits based on the type of the input
      typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                                 CeresUtils<kParameterDim>>
          Utils;
      T regularization = Utils::zero();
      for (int i = 0; i < kParameterDim; ++i) {
        // store current parameters as double for logging purposes
        parent->current_param_[i] = Utils::getDouble(x[i]);
        // weighted sum of parameter L2 norms
        regularization += parent->parameters[i].regularization * x[i] * x[i];
      }
      for (int i = 0; i < kResidualDim; ++i) {
        // TODO consider adding separate residual dimension for parameter
        // regularization
        residual[i] = regularization;
      }

      T difference;

      int nonfinite = 0;
      std::vector<T> rollout_state(kStateDim, Utils::zero());
      double time;
      std::vector<std::vector<double>> error_evolution;
      // plot_trajectory(target_states);
      // plot_trajectory(rollout_states);
      for (int t = 0; t < n_target; ++t) {
        if (target_times.empty()) {
          // assume target states line up with rollout states
          rollout_state = rollout_states[t];
          time = dt * t;
        } else {
          // linear interpolation of rollout states at the target times
          double target_time = target_times[t];
          // numerically stable way to get index of rollout state
          int rollout_i = static_cast<int>(std::floor(target_time / dt + 0.5));
          if (rollout_i >= n_rollout - 1) {
            // fprintf(stderr,
            //         "Target time %.4f (step %i) corresponds to a state (%i) "
            //         "that has not been rolled out.\n",
            //         target_time, t, rollout_i);
            break;
          }
          double alpha = (target_time - rollout_i * dt) / dt;
          const std::vector<T> &left = rollout_states[rollout_i];
          const std::vector<T> &right = rollout_states[rollout_i + 1];

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
            ++nonfinite;
            continue;
          } else if (std::abs(dd) > 1e10) {
            ++nonfinite;
            printf(" NONFINITE!!! ");
#ifdef USE_MATPLOTLIB
            plot_trajectory(rollout_states);
#endif
            continue;
          }
          // printf("%.3f  ", Utils::getDouble(difference));

          // discount contribution of errors at later time steps to mitigate
          // gradient explosion on long roll-outs
          if (parent->divide_cost_by_time_factor != 0. && t > 0) {
            difference /= std::pow(parent->divide_cost_by_time_factor * time,
                                   parent->divide_cost_by_time_exponent);
          }

          if constexpr (kResidualMode == RES_MODE_1D) {
            *residual += difference;
          } else if constexpr (kResidualMode == RES_MODE_STATE) {
            residual[i] += difference;
          }
          error_state[i] = dd;
        }
        error_evolution.push_back(error_state);
        // printf("[[res: %.3f]]  ", Utils::getDouble(*residual));
      }
      // plot_trajectory(error_evolution);
      // plt::named_plot("difference", error_evolution);
      // plt::show();

      // if (parent->options.minimizer_progress_to_stdout &&
      // Utils::getDouble(residual[0]) < 0.) {
      printf("params: ");
      for (int ri = 0; ri < kParameterDim; ++ri) {
        printf("%.4f  ", Utils::getDouble(params[ri]));
      }
      printf("\tresidual: ");
      for (int ri = 0; ri < kResidualDim; ++ri) {
        printf("%.6f  ", Utils::getDouble(residual[ri]));
      }
      if (nonfinite > 0) {
        std::cerr << "nonfinite: " << nonfinite;
      }
      // std::cout << "  thread ID: " << std::this_thread::get_id();
      printf("\n");
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
  bool fade_std{true};

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
        auto estimator = this->estimator_constructor();
        estimator->setup(new ceres::HuberLoss(1.));  // TODO expose this
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
            if (summary.final_cost < this->best_cost_) {
              this->best_cost_ = summary.final_cost;
              printf("FOUND NEW BEST COST: %.6f\n", summary.final_cost);
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
              param.value = d(this->gen_);
            } else {
              std::normal_distribution<double> d{
                  this->params[i],
                  initial_std * (param.maximum - param.minimum)};
              param.value = d(this->gen_);
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

  std::random_device rd_;
  std::mt19937 gen_{rd_()};
};

#endif  // TINY_CERES_ESTIMATOR_H
