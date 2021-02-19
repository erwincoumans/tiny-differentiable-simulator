#ifndef TINY_NEURAL_NETWORK_FROM_JSON_H
#define TINY_NEURAL_NETWORK_FROM_JSON_H

#include <nlohmann/json.hpp>

#include "math/neural_network.hpp"

namespace tds {

template<typename Algebra>
class NeuralNetworkFromJson : public NeuralNetwork<Algebra> {
 public:
  // Default constructor.
  NeuralNetworkFromJson(){}

  // Constructs a NeuralNetwork with the given json model state file's weights,
  // biases, layer sizes and activation functions.
  NeuralNetworkFromJson(const nlohmann::json& json) {
    std::vector<tds::NeuralNetworkActivation> activations;
    std::vector<int> layer_sizes;
    bool input_dim_set = false;

    for (auto&[key, value]: json.items()) {
      if (key.find(".activation") != std::string::npos) {
        NeuralNetworkActivation
            activation = MapTorchActivationTypeToTdsType(value);
        if (activation != tds::NN_ACT_IDENTITY) {
          activations.push_back(activation);
          std::cout
              << "Activation function type loaded: " << activation
              << std::endl;
        }
      } else if (key.find(".weight") != std::string::npos) {
        if (!input_dim_set) {
          // Input layer
          this->layers_[0] = value[0].size();
          input_dim_set = true;
        }
        int layer_size = value.size();
        layer_sizes.push_back(layer_size);
        this->weights.reserve(
            this->weights.size() + layer_size * value[0].size());
        for (const auto& vec: value) {
          for (double value: vec) {
            this->weights.push_back(value);
          }
        }
      } else if (key.find(".bias") != std::string::npos) {
        this->biases.reserve(this->biases.size() + value.size());
        for (double i: value) {
          this->biases.push_back(i);
        }
      }
    }

    this->use_bias_[0] = false;
    assert(activations.size() <= layer_sizes.size());
    for (size_t i = 0; i < layer_sizes.size(); i++) {
      if (i >= activations.size()) {
        this->add_linear_layer(NN_ACT_IDENTITY, layer_sizes[i], true);
      } else {
        this->add_linear_layer(activations[i], layer_sizes[i], true);
      }
    }
    std::cout << "Model loaded from torch script: " << std::endl;
    std::cout << "Layer sizes: ";
    for (auto i: this->layers_) {
      std::cout << i << ", ";
    }
    std::cout << std::endl;

    std::cout << "Activations: ";
    for (auto i: this->activations_) {
      std::cout << i << ", ";
    }
    std::cout << std::endl;
  }

 private:
  // Returns a NeuralNetworkActivation given a torch activation function
  // type.
  NeuralNetworkActivation MapTorchActivationTypeToTdsType(const std::string& type) {
    if (type.find("ReLU") != std::string::npos) {
      return tds::NN_ACT_RELU;
    } else if (type.find("Tanh") != std::string::npos) {
      return tds::NN_ACT_TANH;
    } else if (type.find("ELU") != std::string::npos) {
      return tds::NN_ACT_ELU;
    } else if (type.find("Sigmoid") != std::string::npos) {
      return tds::NN_ACT_SIGMOID;
    } else if (type.find("Softsign") != std::string::npos) {
      return tds::NN_ACT_SOFTSIGN;
    } else {
      return tds::NN_ACT_IDENTITY;
    }
  }
};

} // namespace tds

#endif  // TINY_NEURAL_NETWORK_FROM_JSON_H