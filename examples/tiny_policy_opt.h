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

#ifndef TINY_POLICY_OPT_H
#define TINY_POLICY_OPT_H

#include <ceres/ceres.h>

#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "ceres_utils.h"
#include "tiny_double_utils.h"
#include "tiny_neural_network.h"

struct EstimationParameter {
  std::string name{"unnamed_param"};
  double value{1.0};
  double minimum{-2};
  double maximum{2};

  EstimationParameter& operator=(double rhs) {
    value = rhs;
    return *this;
  }
  explicit operator double() const { return value; }

  double random_value() const {
    return minimum + (double(rand()) / RAND_MAX * (maximum - minimum));
  };
};

enum ResidualMode { RES_MODE_1D, RES_MODE_TIME };

template <int ParameterDim, int TimeDim,
          template <typename, typename> class Environment,
          ResidualMode ResMode = RES_MODE_1D>
class TinyPolicyOptimizer : ceres::IterationCallback {
 public:
  static const int kParameterDim = ParameterDim;
  static const int kTimeDim = TimeDim;
  static const ResidualMode kResidualMode = ResMode;

  typedef ceres::Jet<double, kParameterDim> ADScalar;
  typedef CeresUtils<kParameterDim> ADUtils;
  typedef Environment<double, DoubleUtils> EnvironmentD;
  typedef Environment<ADScalar, ADUtils> EnvironmentAD;

  static const int kStateDim = EnvironmentD::kStateDim;
  static const int kActionDim = EnvironmentD::kActionDim;
  static const int kResidualDim = kResidualMode == RES_MODE_1D ? 1 : kTimeDim;

  static_assert(kParameterDim >= 1);
  static_assert(kStateDim >= 1);
  static_assert(kTimeDim >= 1);

  typedef TinyPolicyOptimizer<kParameterDim, kTimeDim, Environment,
                              kResidualMode>
      PolicyOptimizer;

  ceres::Solver::Options options;

  TinyNeuralNetworkSpecification policy_spec;

  EnvironmentD environment_d;
  EnvironmentAD environment_ad;

  std::array<EstimationParameter, kParameterDim> parameters;

  explicit TinyPolicyOptimizer(
      const TinyNeuralNetworkSpecification& policy_spec)
      : policy_spec(policy_spec) {
    options.minimizer_progress_to_stdout = true;
    options.callbacks.push_back(this);
    //    options.line_search_direction_type =
    //    ceres::LineSearchDirectionType::LBFGS;

    if (policy_spec.num_parameters() != kParameterDim) {
      fprintf(stderr,
              "The number of parameters (%i) does not match the parameters "
              "from the network specification (%i).\n",
              kParameterDim, policy_spec.num_parameters());
      assert(0);
    }
  }

  template <typename Scalar, typename Utils>
  void rollout(const std::vector<Scalar>& params, std::vector<Scalar>& costs) {
    std::vector<Scalar> state;
    if constexpr (std::is_same_v<Scalar, double>) {
      environment_d.reset(state);
    } else {
      environment_ad.reset(state);
    }
    TinyNeuralNetwork<Scalar, Utils> policy(policy_spec);
    policy.set_parameters(params);
    Scalar c;
    bool done = false;
    std::vector<Scalar> action;
    for (int step = 0; step < kTimeDim; ++step) {
      policy.compute(state, action);
      if constexpr (std::is_same_v<Scalar, double>) {
        environment_d.step(action, step, state, c, done);
      } else {
        environment_ad.step(action, step, state, c, done);
      }
      costs[step] += c;
      //      if (done) {
      //        return;
      //      }
    }
  }

  void set_params(const std::array<double, kParameterDim>& params) {
    for (int i = 0; i < kParameterDim; ++i) {
      parameters[i].value = params[i];
    }
  }

  double* vars() { return vars_; }
  const double* vars() const { return vars_; }

  ceres::CostFunction* cost_function_{nullptr};

  void compute_gradient(const double* parameters, double* cost,
                        double* gradient) const {
    double const* const* params = &parameters;
    cost_function_->Evaluate(params, cost, &gradient);
  }

  ceres::Problem& setup(
      TinyNeuralNetworkInitialization nn_init = NN_INIT_XAVIER,
      ceres::LossFunction* loss_function = nullptr) {
    cost_function_ =
        new ceres::AutoDiffCostFunction<CostFunctor, kResidualDim,
                                        kParameterDim>(new CostFunctor(this));
    //    cost_function_ =
    //        new ceres::NumericDiffCostFunction<CostFunctor, ceres::FORWARD,
    //                                           kResidualDim, kParameterDim>(
    //            new CostFunctor(this));
    vars_ = new double[kParameterDim];
    problem_.AddResidualBlock(cost_function_, loss_function, vars_);

    std::vector<double> weights, biases;
    policy_spec.template initialize<double, DoubleUtils>(weights, biases,
                                                         nn_init);
    for (int i = 0; i < policy_spec.num_weights(); ++i) {
      parameters[i] = weights[i];
      parameters[i].name = "weight_" + std::to_string(i);
    }
    for (int i = 0; i < policy_spec.num_biases(); ++i) {
      parameters[i + policy_spec.num_weights()] = biases[i];
      parameters[i].name = "bias_" + std::to_string(i);
    }

    param_evolution_.clear();
    for (int i = 0; i < kParameterDim; ++i) {
      vars_[i] = parameters[i].value;
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

  virtual ~TinyPolicyOptimizer() {
    if (vars_) {
      delete[] vars_;
      vars_ = nullptr;
    }
  }

  //  const std::vector<std::array<double, kParameterDim>>&
  //  parameter_evolution()
  //      const {
  //    return param_evolution_;
  //  }

  double* vars_{nullptr};

 private:
  ceres::Problem problem_;

  mutable std::vector<std::array<double, kParameterDim>> param_evolution_;
  mutable std::array<double, kParameterDim> current_param_;

  struct CostFunctor {
    PolicyOptimizer* parent{nullptr};

    /**
     * Number of rollouts to be averaged.
     */
    int batch_size{10};

    CostFunctor(PolicyOptimizer* parent) : parent(parent) {}

    // Computes the cost (residual) for input parameters x.
    // TODO use stan::math reverse-mode AD
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
      const std::vector<T> params(x, x + kParameterDim);
      typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                                 ADUtils>
          Utils;
      std::vector<T> costs(kTimeDim, T(0.));
      for (int i = 0; i < batch_size; ++i) {
        parent->template rollout<T, Utils>(params, costs);
      }
      for (int k = 0; k < kTimeDim; ++k) {
        costs[k] /= T((double)batch_size);
        //        printf("%.3f ", Utils::getDouble(costs[k]));
      }
      //      printf("\n");
      //      for (int i = 0; i < kParameterDim; ++i) {
      //        parent->current_param_[i] = Utils::getDouble(x[i]);
      //      }

      int t = 0;
      if constexpr (kResidualMode == RES_MODE_1D) {
        *residual = Utils::zero();
      }
      for (const auto& cost : costs) {
        if constexpr (kResidualMode == RES_MODE_1D) {
          *residual += cost;
        } else if constexpr (kResidualMode == RES_MODE_TIME) {
          residual[t] = cost;
          ++t;
        }
      }
      return true;
    }
  };

  ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) override {
    //    param_evolution_.push_back(current_param_);
    return ceres::SOLVER_CONTINUE;
  }
};

template <int ParameterDim, typename Optimizer>
class BasinHoppingOptimizer {
  static const int kParameterDim = ParameterDim;
  typedef std::function<std::unique_ptr<Optimizer>()> OptimizerConstructor;

 public:
  OptimizerConstructor optimizer_constructor;
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
  double initial_std{2.};

  BasinHoppingOptimizer(
      const OptimizerConstructor& optimizer_constructor,
      const std::array<double, kParameterDim>& initial_guess,
      std::size_t num_workers = std::thread::hardware_concurrency())
      : optimizer_constructor(optimizer_constructor),
        params(initial_guess),
        num_workers(num_workers) {
    workers_.reserve(num_workers);
  }

  // Run global optimizer.
  void run() {
    using namespace std::chrono;
    best_cost_ = std::numeric_limits<double>::max();
    std::cout << "Starting " << num_workers << " worker(s).\n";
    int improvement_iter = 0;
    for (std::size_t k = 0; k < num_workers; ++k) {
      workers_.emplace_back([this, k, &improvement_iter]() {
        auto start_time = high_resolution_clock::now();
        auto optimizer = this->optimizer_constructor();
        optimizer->setup();
        optimizer->options.minimizer_progress_to_stdout = false;
        if (k == 0) {
          // set initial guess
          optimizer->set_params(this->params);
        } else {
          for (auto& p : optimizer->parameters) {
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
          auto summary = optimizer->solve();
          {
            std::unique_lock<std::mutex> lock(this->mutex_);
            if (summary.final_cost < this->best_cost_) {
              this->best_cost_ = summary.final_cost;
              for (int i = 0; i < kParameterDim; ++i) {
                this->params[i] = optimizer->parameters[i].value;
              }
              printf("Thread %i found new best cost %.5f.\n",
                     static_cast<int>(k), this->best_cost_);
              ++improvement_iter;
            }
            // apply random step to the parameters
            //            std::cout << "Thread " << k << " uses parameters [  ";
            for (int i = 0; i < kParameterDim; ++i) {
              auto& param = optimizer->parameters[i];
              double iter_div = (improvement_iter + 1.) * 0.8;
              //              double iter_div = (iter + 1.);
              //              iter_div *= iter_div;
              std::normal_distribution<double> d{
                  this->params[i],
                  initial_std / iter_div * (param.maximum - param.minimum)};
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

  virtual ~BasinHoppingOptimizer() {
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

#endif  // TINY_POLICY_OPT_H
