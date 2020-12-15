// Example to load a PyTorch trained model .pt file into a TDS NeuralNetwork.

#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include <torch/script.h>

#include "math/neural_network.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_algebra.hpp"

using tds::NeuralNetwork;
using tds::NeuralNetworkActivation;

int main(int argc, const char* argv[]) {
  if (argc != 2) {
    std::cerr
        << "usage: load_pytorch_network <path-to-exported-script-module>\n";
    return -1;
  }

  torch::jit::script::Module module;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load(argv[1]);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n" << e.msg() << std::endl;
    return -1;
  }

  std::cout << "Torch model loaded\n";

  NeuralNetwork<TinyAlgebra<double, TINY::DoubleUtils>> net(module);

  // Test to verify that the LibTorch model and TDS model would return similar
  // results given the same input.
  int input_dim = net.input_dim();

  std::vector<double> input_vec = {
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.47403184e-04,
   -2.94704954e-02,  0.00000000e+00,  3.05853542e-02,  9.51030292e-03,
    5.17256707e-02,  1.00000000e+00,  1.00000000e+00,  1.00000000e+00,
    1.00000000e+00,  2.10564360e-01, -7.85591453e-03, -2.40782738e-01,
    6.08715266e-02,  2.03281105e-01, -2.38092959e-01, -8.93045291e-02,
   -2.17512503e-01, -2.31911838e-01, -2.38997698e-01, -6.37538265e-03,
   -2.29222238e-01,  4.50000000e-01,  4.50000000e-01,  4.50000000e-01,
    4.50000000e-01,  0.00000000e+00,  0.00000000e+00,  2.40000000e-01,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00
  };

  // torch
  std::vector<torch::jit::IValue> torch_input;
  torch_input.push_back(torch::tensor(input_vec, torch::dtype(torch::kDouble)));
  std::cout << "torch output: " << module.forward(torch_input) << std::endl;
  // tds
  std::vector<double> output;
  net.compute(input_vec, output);
  std::cout << "tds output: ";
  for (auto item: output) {
    std::cout << item << ", ";
  }
  std::cout << std::endl;

}
