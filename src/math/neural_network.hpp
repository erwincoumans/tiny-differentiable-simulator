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
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <array>
#include "math/conditionals.hpp"
#undef min
#undef max

namespace tds {

enum NeuralNetworkActivation {
  NN_ACT_IDENTITY = -1,
  NN_ACT_TANH,
  NN_ACT_SIN,
  NN_ACT_RELU,
  NN_ACT_SOFT_RELU,
  NN_ACT_ELU,
  NN_ACT_SIGMOID,
  NN_ACT_SOFTSIGN
};

enum NeuralNetworkInitialization {
  NN_INIT_ZERO = -1,
  NN_INIT_XAVIER = 0,  // good for tanh activations
  NN_INIT_HE,          // good for sigmoid activations
};

/**
 * Statically sized NN specification with a predefined number of layers so that
 * the number of parameters can be statically inferred at compile time.
 */
template <std::size_t NumLayers>
struct StaticNeuralNetworkSpecification {
  static const inline std::size_t kNumLayers = NumLayers;

  std::array<NeuralNetworkActivation, kNumLayers> activations;
  std::array<int, kNumLayers> layers;
  std::array<bool, kNumLayers> use_bias;

  constexpr StaticNeuralNetworkSpecification(
      std::size_t default_num_hidden_units = 5, bool default_use_bias = true)
      : activations(), layers(), use_bias() {
    for (std::size_t i = 0; i < kNumLayers; ++i) {
      layers[i] = default_num_hidden_units;
      use_bias[i] = default_use_bias;
    }
  }

  constexpr int input_dim() const { return layers[0]; }
  constexpr int output_dim() const { return layers.back(); }
  constexpr int num_weights() const {
    int num = 0;
    for (std::size_t i = 1; i < kNumLayers; ++i) {
      num += layers[i - 1] * layers[i];
    }
    return num;
  }
  constexpr int num_units(int layer_id) const { return layers[layer_id]; }
  constexpr int num_biases() const {
    int num = 0;
    for (std::size_t i = 0; i < kNumLayers; ++i) {
      num += use_bias[i] ? layers[i] : 0;
    }
    return num;
  }
  constexpr int num_parameters() const { return num_weights() + num_biases(); }
  constexpr int num_layers() const { return static_cast<int>(kNumLayers); }
};

/**
 * Implements a fully-connected neural network consisting of linear layers with
 * weights and optional biases to be stored externally.
 */
class NeuralNetworkSpecification {
 protected:
  std::vector<NeuralNetworkActivation> activations_;
  std::vector<int> layers_{std::vector<int>{0}};
  std::vector<bool> use_bias_{true};

 public:
  explicit NeuralNetworkSpecification(int input_dim = 0,
                                      bool use_input_bias = true) {
    layers_[0] = input_dim;
    use_bias_[0] = use_input_bias;
  }
  NeuralNetworkSpecification(int input_dim, const std::vector<int>& layer_sizes,
                             NeuralNetworkActivation activation,
                             bool learn_bias = true) {
    layers_[0] = input_dim;
    for (int size : layer_sizes) {
      add_linear_layer(activation, size, learn_bias);
    }
  }

  template <std::size_t NumLayers>
  NeuralNetworkSpecification(
      const StaticNeuralNetworkSpecification<NumLayers>& spec) {
    activations_.resize(NumLayers);
    layers_.resize(NumLayers);
    use_bias_.resize(NumLayers);
    for (std::size_t i = 0; i < NumLayers; ++i) {
      activations_[i] = spec.activations[i];
      layers_[i] = spec.layers[i];
      use_bias_[i] = spec.use_bias[i];
    }
  }

  void set_input_dim(int dim, bool use_input_bias = true) { 
      layers_[0] = dim;
      use_bias_[0] = use_input_bias;
  }

  void add_linear_layer(NeuralNetworkActivation activation, int units,
                        bool learn_bias = true) {
    activations_.push_back(activation);
    layers_.push_back(units);
    use_bias_.push_back(learn_bias);
  }

  bool empty() const { return layers_.empty(); }
  int input_dim() const { return layers_[0]; }
  int output_dim() const { return layers_.back(); }
  int num_weights() const {
    int num = 0;
    for (std::size_t i = 1; i < layers_.size(); ++i) {
      num += layers_[i - 1] * layers_[i];
    }
    return num;
  }
  int num_units(int layer_id) const { return layers_[layer_id]; }
  int num_biases() const {
    int num = 0;
    for (std::size_t i = 0; i < layers_.size(); ++i) {
      num += use_bias_[i] ? layers_[i] : 0;
    }
    return num;
  }
  int num_parameters() const { return num_weights() + num_biases(); }
  int num_layers() const { return static_cast<int>(layers_.size()); }

  template <typename Algebra>
  static void print_states(
      const std::vector<typename Algebra::Scalar>& numbers) {
    for (const auto& n : numbers) {
      printf("%.2f ", Algebra::to_double(n));
    }
    printf("\n");
  }

  /**
   * Initializes the weights and biases using the given method.
   */
  template <typename Algebra>
  void initialize(
      std::vector<typename Algebra::Scalar>& weights,
      std::vector<typename Algebra::Scalar>& biases,
      NeuralNetworkInitialization init_method = NN_INIT_XAVIER) const {
    weights.resize(num_weights());
    biases.resize(num_biases(), Algebra::from_double(0.5));

    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    std::size_t weight_i = 0;
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
        for (int pi = 0; pi < layers_[i - 1]; ++pi, ++weight_i) {
          if (init_method == NN_INIT_ZERO) {
            weights[weight_i] = Algebra::zero();
          } else {
            weights[weight_i] = Algebra::from_double(d(gen_));
          }
        }
      }
    }
#ifndef NDEBUG
    printf("NN weights:  ");
    this->template print_states<Algebra>(weights);
    printf("NN biases:  ");
    this->template print_states<Algebra>(biases);
#endif
  }

  /**
   * Infers the output of the neural network for the given input, and the
   * provided weights and biases.
   */
  template <typename Algebra>
  void compute(const std::vector<typename Algebra::Scalar>& weights,
               const std::vector<typename Algebra::Scalar>& biases,
               const std::vector<typename Algebra::Scalar>& input,
               std::vector<typename Algebra::Scalar>& output) const {
    int nw = num_weights();
    int w = weights.size();

    assert(static_cast<int>(weights.size() == num_weights()));
    
    int nb = num_biases();
    int b = biases.size();

    assert(static_cast<int>(biases.size() == num_biases()));
    
    int id = input_dim();
    int isz = input.size();
    assert(static_cast<int>(input.size()) == input_dim());

    const typename Algebra::Scalar zero = Algebra::zero();
    const typename Algebra::Scalar one = Algebra::one();

    output.resize(layers_.back());
    std::vector<typename Algebra::Scalar> previous = input;
    std::vector<typename Algebra::Scalar> current;
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
        for (int pi = 0; pi < layers_[i - 1]; ++pi, ++weight_i) {
          current[ci] += previous[pi] * weights[weight_i];
        }
        switch (activations_[i - 1]) {
          case NN_ACT_TANH:
            current[ci] = Algebra::tanh(current[ci]);
            break;
          case NN_ACT_SIN:
            current[ci] = Algebra::sin(current[ci]);
            break;
          case NN_ACT_RELU:
            current[ci] = Algebra::max(zero, current[ci]);
            break;
          case NN_ACT_SOFT_RELU:
            current[ci] = Algebra::log(one + Algebra::exp(current[ci]));
            break;
          case NN_ACT_ELU:
            current[ci] = tds::where_ge(current[ci], zero, current[ci],
                                        Algebra::exp(current[ci]) - one);
            break;
          case NN_ACT_SIGMOID: {
            typename Algebra::Scalar exp_x = Algebra::exp(current[ci]);
            current[ci] = exp_x / (exp_x + one);
            break;
          }
          case NN_ACT_SOFTSIGN:
            current[ci] = current[ci] / (one + Algebra::abs(current[ci]));
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

  template <typename Algebra>
  void save_graphviz(const std::string& filename,
                     const std::vector<std::string>& input_names = {},
                     const std::vector<std::string>& output_names = {},
                     const std::vector<typename Algebra::Scalar>& weights = {},
                     const std::vector<typename Algebra::Scalar>& biases = {},
                     bool show_arrows = false) const {
    std::ofstream file(filename);
    if (show_arrows) {
      file << "digraph G {\n\t";
    } else {
      file << "graph G {\n\t";
    }
    file << "rankdir=LR;\n\tsplines=false;\n\tedge[style=invis];\n\tranksep="
            "1.4;\n\t";
    file << "node[shape=circle,color=black,style=filled,fillcolor=lightgray];"
            "\n\t";
    for (std::size_t i = 0; i < layers_.size(); ++i) {
      file << "{\n\t";
      for (int j = 0; j < layers_[i]; ++j) {
        file << "\tn_" << i << "_" << j << " [label=";
        if (i == 0 && j < input_names.size()) {
          file << "\"" << input_names[j] << "\"";
        } else if (i == static_cast<int>(layers_.size()) - 1 &&
                   j < output_names.size()) {
          file << "\"" << output_names[j] << "\"";
        } else {
          file << "\"\"";
        }
        file << "];\n\t";
      }
      file << "}\n\t";
    }
    file << "// prevent tilting\n\t";
    for (std::size_t i = 2; i < layers_.size(); ++i) {
      file << "n_" << i - 1 << "_0-";
      file << (show_arrows ? '>' : '-');
      file << "n_" << i << "_0;\n\t";
    }
    file << "edge[style=solid";
    if (!show_arrows) {
      file << ", tailport=e, headport=w";
    }
    file << "];\n\t";
    double max_weight = 2;
    if (!weights.empty()) {
      max_weight = std::abs(Algebra::from_double(weights[0]));
      for (const auto& w : weights) {
        max_weight = std::max(max_weight, std::abs(Algebra::from_double(w)));
      }
    }
    std::size_t weight_i = 0;
    for (std::size_t i = 1; i < layers_.size(); ++i) {
      for (int j = 0; j < layers_[i]; ++j) {
        for (int k = 0; k < layers_[i - 1]; ++k, ++weight_i) {
          file << "\tn_" << i - 1 << "_" << k << "-";
          file << (show_arrows ? '>' : '-');
          file << "n_" << i << "_" << j;
          if (!weights.empty()) {
            file << "[penwidth="
                 << std::to_string(
                        std::abs(Algebra::from_double(weights[weight_i])) /
                        max_weight * 3.0)
                 << "]";
          }
          file << ";\n";
        }
      }
    }
    file << "}\n";
    std::cout << "Saved neural network graphviz file at " << filename << ".\n";
  }
};

/**
 * Implements a fully-connected neural network consisting of linear layers with
 * weights and optional biases to be stored internally.
 */
template <typename Algebra>
class NeuralNetwork : public NeuralNetworkSpecification {
 public:
  std::vector<typename Algebra::Scalar> weights;
  std::vector<typename Algebra::Scalar> biases;

  using NeuralNetworkSpecification::compute;
  using NeuralNetworkSpecification::initialize;

  explicit NeuralNetwork(int input_dim = 0, bool use_input_bias = true)
      : NeuralNetworkSpecification(input_dim, use_input_bias) {}
  NeuralNetwork(int input_dim, const std::vector<int>& layer_sizes,
                NeuralNetworkActivation activation, bool learn_bias = true)
      : NeuralNetworkSpecification(input_dim, layer_sizes, activation,
                                   learn_bias) {}
  explicit NeuralNetwork(const NeuralNetworkSpecification& spec)
      : NeuralNetworkSpecification(spec) {}

  void initialize(NeuralNetworkInitialization init_method = NN_INIT_XAVIER) {
    this->template initialize<Algebra>(weights, biases, init_method);
  }

  void compute(const std::vector<typename Algebra::Scalar>& input,
               std::vector<typename Algebra::Scalar>& output) const {
    this->template compute<Algebra>(weights, biases, input, output);
  }


  void set_parameters(const std::vector<typename Algebra::Scalar>& params) {
    int params_size = params.size();
    int num_actual_params = num_parameters();
    assert(static_cast<int>(params_size) >= num_actual_params);
    weights.resize(num_weights());
    biases.resize(num_biases());
    std::copy(params.begin(), params.begin() + num_weights(), weights.begin());
    std::copy(params.begin() + num_weights(),
              params.begin() + num_weights() + num_biases(), biases.begin());
  }

  void get_parameters(std::vector<typename Algebra::Scalar>& params) {
      int num_actual_params = num_parameters();
      params.resize(num_actual_params);
      std::copy(weights.begin(), weights.begin() + num_weights(), params.begin());
      std::copy(biases.begin(),biases.begin()+num_biases(), params.begin() + num_weights());
  }
  void print_params() const {
    printf("NN weights:  ");
    this->template print_states<Algebra>(weights);
    printf("NN biases:  ");
    this->template print_states<Algebra>(biases);
  }

  void save_graphviz(const std::string& filename,
                     const std::vector<std::string>& input_names = {},
                     const std::vector<std::string>& output_names = {}) const {
    dynamic_cast<NeuralNetworkSpecification*>(this)
        ->template save_graphviz<Algebra>(filename, input_names, output_names,
                                          weights, biases);
  }
};

}  // namespace tds

#endif  // TINY_NEURAL_NETWORK_H
