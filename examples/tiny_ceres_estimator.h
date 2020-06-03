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

#ifndef TINY_ESTIMATOR_H
#define TINY_ESTIMATOR_H

#include <ceres/ceres.h>

#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "ceres_utils.h"
#include "tiny_double_utils.h"

struct EstimationParameter {
  std::string name{"unnamed_param"};
  double value{1.0};
  double minimum{-std::numeric_limits<double>::infinity()};
  double maximum{std::numeric_limits<double>::infinity()};

  EstimationParameter& operator=(double rhs) {
    value = rhs;
    return *this;
  }
  explicit operator double() const { return value; }

  double random_value() const {
    return minimum + (double(rand()) / RAND_MAX * (maximum - minimum));
  };
};

enum ResidualMode { RES_MODE_1D, RES_MODE_STATE, RES_MODE_TIME };

template <int ParameterDim, int StateDim, int TimeDim,
          ResidualMode ResMode = RES_MODE_1D>
class TinyCeresEstimator : ceres::IterationCallback {
 public:
  static const int kParameterDim = ParameterDim;
  static const int kStateDim = StateDim;
  static const int kTimeDim = TimeDim;
  static const ResidualMode kResidualMode = ResMode;
  static const int kResidualDim =
      kResidualMode == RES_MODE_1D
          ? 1
          : (kResidualMode == RES_MODE_STATE ? kStateDim : kTimeDim);

  static_assert(kParameterDim >= 1);
  static_assert(kStateDim >= 1);
  static_assert(kTimeDim >= 1);

  typedef ceres::Jet<double, ParameterDim> ADScalar;

  typedef TinyCeresEstimator<kParameterDim, kStateDim, kTimeDim, kResidualMode>
      CeresEstimator;

  std::array<EstimationParameter, kParameterDim> parameters;

  void set_params(const std::array<double, kParameterDim>& params) {
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i].value = params[i];
    }
  }

  // Samples from ground-truth system dynamics, i.e. a list of states.
  std::vector<std::vector<double>> target_states;

  ceres::Solver::Options options;

  TinyCeresEstimator() {
    options.minimizer_progress_to_stdout = true;
    options.callbacks.push_back(this);
  }

  virtual void rollout(
      const std::vector<ADScalar>& params,
      std::vector<std::vector<ADScalar>>& output_states) const = 0;
  virtual void rollout(
      const std::vector<double>& params,
      std::vector<std::vector<double>>& output_states) const = 0;

  ceres::Problem& setup(ceres::LossFunction* loss_function = nullptr) {
    assert(static_cast<int>(target_states.size()) == kTimeDim);
    assert(static_cast<int>(target_states[0].size()) == kStateDim);

    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, kResidualDim,
                                        kParameterDim>(new CostFunctor(this));
    vars_ = new double[kParameterDim];
    problem_.AddResidualBlock(cost_function, loss_function, vars_);

    for (int i = 0; i < kParameterDim; ++i) {
      problem_.SetParameterLowerBound(vars_, i, parameters[i].minimum);
      problem_.SetParameterUpperBound(vars_, i, parameters[i].maximum);
    }

    return problem_;
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

  virtual ~TinyCeresEstimator() {
    if (vars_) {
      delete[] vars_;
      vars_ = nullptr;
    }
  }

  const std::vector<std::array<double, kParameterDim>>& parameter_evolution()
      const {
    return param_evolution_;
  }

 private:
  ceres::Problem problem_;
  double* vars_{nullptr};

  mutable std::vector<std::array<double, kParameterDim>> param_evolution_;
  mutable std::array<double, kParameterDim> current_param_;

  struct CostFunctor {
    CeresEstimator* parent{nullptr};

    CostFunctor(CeresEstimator* parent) : parent(parent) {}

    // Computes the cost (residual) for input parameters x.
    // TODO use stan::math reverse-mode AD
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
      const std::vector<T> params(x, x + kParameterDim);
      std::vector<std::vector<T>> output_states;
      parent->rollout(params, output_states);
      typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                                 CeresUtils<kParameterDim>>
          Utils;
      for (int i = 0; i < kParameterDim; ++i) {
        parent->current_param_[i] = Utils::getDouble(x[i]);
      }
      for (int i = 0; i < kResidualDim; ++i) {
        residual[i] = Utils::zero();
      }

      T difference;
      for (int t = 0; t < kTimeDim; ++t) {
        for (int i = 0; i < kStateDim; ++i) {
          difference = parent->target_states[t][i] - output_states[t][i];
          if constexpr (kResidualMode == RES_MODE_1D) {
            *residual += difference * difference;
          } else if constexpr (kResidualMode == RES_MODE_TIME) {
            residual[t] += difference * difference;
          } else if constexpr (kResidualMode == RES_MODE_STATE) {
            residual[i] += difference * difference;
          }
        }
      }
      return true;
    }
  };

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) override {
    param_evolution_.push_back(current_param_);
    return ceres::SOLVER_CONTINUE;
  }
};

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

  double cost_limit{1e-20};

  /**
   * Initial standard deviation used for Gaussian noise applied to the
   * parameters, normalized by the bounds of the parameter.
   */
  double initial_std{1.};

  BasinHoppingEstimator(
      const EstimatorConstructor& estimator_constructor,
      const std::array<double, kParameterDim>& initial_guess,
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
    for (std::size_t k = 0; k < num_workers; ++k) {
      workers_.emplace_back([this, k]() {
        auto start_time = high_resolution_clock::now();
        auto estimator = this->estimator_constructor();
        estimator->setup(new ceres::HuberLoss(1.));  // TODO expose this
        if (k == 0) {
          // set initial guess
          estimator->set_params(this->params);
        } else {
          for (auto& p : estimator->parameters) {
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
            //            if (time_up) {
            //              std::cout << "time up\n";
            //            }
            //            if (this->best_cost_ < this->cost_limit) {
            //              std::cout << "this->best_cost_ <
            //              this->cost_limit\n";
            //            }
            //            if (this->stop_) {
            //              std::cout << "this->stop_\n";
            //            }
            if (time_up || this->stop_ || this->best_cost_ < this->cost_limit) {
              std::cout << "Thread " << k << " has terminated after " << iter
                        << " iterations.\n";
              return;
            }
          }
          auto summary = estimator->solve();
          {
            std::unique_lock<std::mutex> lock(this->mutex_);
            if (summary.final_cost < this->best_cost_) {
              this->best_cost_ = summary.final_cost;
              for (int i = 0; i < kParameterDim; ++i) {
                this->params[i] = estimator->parameters[i].value;
              }
            }
            // apply random step to the parameters
            //            std::cout << "Thread " << k << " uses parameters [  ";
            for (int i = 0; i < kParameterDim; ++i) {
              auto& param = estimator->parameters[i];
              std::normal_distribution<double> d{
                  this->params[i],
                  initial_std / (iter + 1.) * (param.maximum - param.minimum)};
              //              printf("{%.3f}  ", param.value);
              param.value = d(this->gen_);
              //              printf("(%.3f)  ", param.value);
              param.value = std::max(param.minimum, param.value);
              param.value = std::min(param.maximum, param.value);
              //              printf("%.3f  ;;", param.value);
            }
            //            printf("]\n");
          }
        }
      });
    }
    for (auto& worker : workers_) {
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
    for (auto& worker : workers_) {
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

#endif  // TINY_ESTIMATOR_H
