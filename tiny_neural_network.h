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

#ifndef TINY_NEURAL_NETWORK_H
#define TINY_NEURAL_NETWORK_H

#include <cassert>
#include <cmath>
#include <random>
#include <vector>

enum TinyNeuralNetworkActivation {
  NN_ACT_IDENTITY = -1,
  NN_ACT_TANH,
  NN_ACT_SIN,
  NN_ACT_RELU,
  NN_ACT_SOFT_RELU,
  NN_ACT_SIGMOID,
  NN_ACT_SOFTSIGN
};

enum TinyNeuralNetworkInitialization {
  NN_INIT_ZERO = -1,
  NN_INIT_XAVIER = 0,  // good for tanh activations
  NN_INIT_HE,          // good for sigmoid activations
};

/**
 * Implements a fully-connected neural network consisting of linear layers with
 * weights and optional biases to be stored externally.
 */
class TinyNeuralNetworkSpecification {
 protected:
  std::vector<TinyNeuralNetworkActivation> activations_;
  std::vector<int> layers_{0};
  std::vector<bool> use_bias_{true};

 public:
  explicit TinyNeuralNetworkSpecification(int input_dim = 0,
                                          bool use_input_bias = true) {
    layers_[0] = input_dim;
    use_bias_[0] = use_input_bias;
  }
  TinyNeuralNetworkSpecification(int input_dim,
                                 const std::vector<int>& layer_sizes,
                                 TinyNeuralNetworkActivation activation,
                                 bool learn_bias = true) {
    layers_[0] = input_dim;
    for (int size : layer_sizes) {
      add_linear_layer(activation, size, learn_bias);
    }
  }

  void set_input_dim(int dim) { layers_[0] = dim; }

  void add_linear_layer(TinyNeuralNetworkActivation activation, int units,
                        bool learn_bias = true) {
    activations_.push_back(activation);
    layers_.push_back(units);
    use_bias_.push_back(learn_bias);
  }

  int input_dim() const { return layers_[0]; }
  int output_dim() const { return layers_.back(); }
  int num_weights() const {
    int num = 0;
    for (std::size_t i = 1; i < layers_.size(); ++i) {
      num += layers_[i - 1] * layers_[i];
    }
    return num;
  }
  int num_biases() const {
    int num = 0;
    for (std::size_t i = 0; i < layers_.size(); ++i) {
      num += use_bias_[i] ? layers_[i] : 0;
    }
    return num;
  }
  int num_parameters() const { return num_weights() + num_biases(); }

  template <typename TinyScalar, typename TinyConstants>
  static void print_states(const std::vector<TinyScalar>& numbers) {
    for (const auto& n : numbers) {
      printf("%.2f ", TinyConstants::getDouble(n));
    }
    printf("\n");
  }

  /**
   * Initializes the weights and biases using the given method.
   */
  template <typename TinyScalar, typename TinyConstants>
  void initialize(
      std::vector<TinyScalar>& weights, std::vector<TinyScalar>& biases,
      TinyNeuralNetworkInitialization init_method = NN_INIT_XAVIER) const {
    weights.resize(num_weights());
    biases.resize(num_biases(), TinyConstants::zero());

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    for (std::size_t i = 1; i < layers_.size(); ++i) {
      double std;
      switch (init_method) {
        case NN_INIT_ZERO:
          break;
        case NN_INIT_HE:
          std = std::sqrt(2. / layers_[i - 1]);
          break;
        default:
        case NN_INIT_XAVIER:
          std = std::sqrt(2. / (layers_[i - 1] + layers_[i]));
      }
      std::normal_distribution<double> d{0., std * std};
      for (int ci = 0; ci < layers_[i]; ++ci) {
        for (int pi = 0; pi < layers_[i - 1]; ++pi) {
          if (init_method == NN_INIT_ZERO) {
            weights[ci * layers_[i - 1] + pi] = TinyConstants::zero();
          } else {
            weights[ci * layers_[i - 1] + pi] = d(gen_);
          }
        }
      }
    }
    printf("NN weights:  ");
    this->template print_states<TinyScalar, TinyConstants>(weights);
    printf("NN biases:  ");
    this->template print_states<TinyScalar, TinyConstants>(biases);
  }

  /**
   * Infers the output of the neural network for the given input, and the
   * provided weights and biases.
   */
  template <typename TinyScalar, typename TinyConstants>
  void compute(const std::vector<TinyScalar>& weights,
               const std::vector<TinyScalar>& biases,
               const std::vector<TinyScalar>& input,
               std::vector<TinyScalar>& output) const {
    assert(static_cast<int>(weights.size() == num_weights()));
    assert(static_cast<int>(biases.size() == num_biases()));
    assert(static_cast<int>(input.size()) == input_dim());

    using std::tanh, std::exp, std::sin, std::max, std::max, std::log;
    const TinyScalar zero = TinyConstants::zero();
    const TinyScalar one = TinyConstants::one();

    output.resize(layers_.back());
    std::vector<TinyScalar> previous = input;
    std::vector<TinyScalar> current;
    std::size_t weight_i = 0;
    std::size_t bias_i = 0;

    if (use_bias_[0]) {
      for (int ci = 0; ci < layers_[0]; ++ci) {
        previous[ci] += biases[bias_i++];
      }
    }
    for (std::size_t i = 1; i < layers_.size(); ++i) {
      current.resize(layers_[i]);
      for (int ci = 0; ci < layers_[i]; ++ci) {
        current[ci] = zero;
        if (use_bias_[i]) {
          current[ci] += biases[bias_i++];
        }
        for (int pi = 0; pi < layers_[i - 1]; ++pi) {
          current[ci] += previous[pi] * weights[ci * layers_[i - 1] + pi];
        }
        switch (activations_[i - 1]) {
          case NN_ACT_TANH:
            current[ci] = tanh(current[ci]);
            break;
          case NN_ACT_SIN:
            current[ci] = sin(current[ci]);
            break;
          case NN_ACT_RELU:
            current[ci] = max(zero, current[ci]);
            break;
          case NN_ACT_SOFT_RELU:
            current[ci] = log(one + exp(current[ci]));
            break;
          case NN_ACT_SIGMOID: {
            TinyScalar exp_x = exp(current[ci]);
            current[ci] = exp_x / (exp_x + one);
            break;
          }
          case NN_ACT_SOFTSIGN:
            current[ci] = current[ci] / (one + abs(current[ci]));
            break;
          case NN_ACT_IDENTITY:
          default:
            break;
        }
      }
      previous = current;
    }
    output = current;
  }
};

/**
 * Implements a fully-connected neural network consisting of linear layers with
 * weights and optional biases to be stored internally.
 */
template <typename TinyScalar, typename TinyConstants>
class TinyNeuralNetwork : public TinyNeuralNetworkSpecification {
 public:
  std::vector<TinyScalar> weights;
  std::vector<TinyScalar> biases;

  using TinyNeuralNetworkSpecification::compute;
  using TinyNeuralNetworkSpecification::initialize;

  explicit TinyNeuralNetwork(int input_dim = 0)
      : TinyNeuralNetworkSpecification(input_dim) {}
  TinyNeuralNetwork(int input_dim, const std::vector<int>& layer_sizes,
                    TinyNeuralNetworkActivation activation,
                    bool learn_bias = true)
      : TinyNeuralNetworkSpecification(input_dim, layer_sizes, activation,
                                       learn_bias) {}
  explicit TinyNeuralNetwork(const TinyNeuralNetworkSpecification& spec)
      : TinyNeuralNetworkSpecification(spec) {}

  void initialize(
      TinyNeuralNetworkInitialization init_method = NN_INIT_XAVIER) {
    this->template initialize<TinyScalar, TinyConstants>(weights, biases,
                                                         init_method);
  }

  void compute(const std::vector<TinyScalar>& input,
               std::vector<TinyScalar>& output) {
    this->template compute<TinyScalar, TinyConstants>(weights, biases, input,
                                                      output);
  }

  void set_parameters(const std::vector<TinyScalar>& params) {
    assert(static_cast<int>(params.size()) == num_parameters());
    weights.resize(num_weights());
    biases.resize(num_biases());
    std::copy(params.begin(), params.begin() + num_weights(), weights.begin());
    std::copy(params.begin() + num_weights(), params.end(), biases.begin());
  }
};

#endif  // TINY_NEURAL_NETWORK_H
