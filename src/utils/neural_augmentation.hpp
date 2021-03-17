#ifndef NEURAL_AUGMENTATION_H
#define NEURAL_AUGMENTATION_H

#include "math/tiny/neural_algebra.hpp"
#include "parameter.hpp"


namespace tds {

struct NeuralAugmentation {
  std::vector<NeuralNetworkSpecification> specs;

  std::vector<std::pair<std::vector<std::string>, std::vector<std::string>>>
      output_inputs;

  static inline int default_hidden_layers{2};
  static inline int default_hidden_units{5};
  static inline NeuralNetworkActivation default_activation_fn = NN_ACT_ELU;

  // L1 regularization for input weights (lasso) to encourage sparse inputs
  double input_lasso_regularization{0};
  // L2 regularization term for upper layers
  double upper_l2_regularization{0};

  double weight_limit = 0.1;
  double bias_limit = 0.2;

  typedef tds::EigenAlgebra DoubleAlgebra;

  NeuralNetworkSpecification &
  add_wiring(const std::string &output, const std::vector<std::string> &inputs,
             int hidden_layers = default_hidden_layers,
             int hidden_units = default_hidden_units,
             NeuralNetworkActivation output_fn = NN_ACT_IDENTITY) {
    return add_wiring(std::vector<std::string>{output}, inputs, hidden_layers,
                      hidden_units, output_fn);
  }

  NeuralNetworkSpecification &
  add_wiring(const std::vector<std::string> &outputs,
             const std::vector<std::string> &inputs,
             int hidden_layers = default_hidden_layers,
             int hidden_units = default_hidden_units, bool input_bias = false,
             bool output_bias = false,
             NeuralNetworkActivation output_fn = NN_ACT_IDENTITY) {
    NeuralNetworkSpecification spec(static_cast<int>(inputs.size()),
                                    input_bias);
    // define overparameterized NN
    for (int li = 0; li < hidden_layers; ++li) {
      spec.add_linear_layer(default_activation_fn, hidden_units);
    }
    // output layer
    spec.add_linear_layer(NN_ACT_IDENTITY, static_cast<int>(outputs.size()),
                          output_bias);
    output_inputs.push_back(std::make_pair(outputs, inputs));
    specs.push_back(spec);
    return specs.back();
  }

  NeuralNetworkSpecification &
  add_wiring(const std::vector<std::string> &outputs,
             const std::vector<std::string> &inputs,
             const NeuralNetworkSpecification &spec) {
    assert(spec.input_dim() == static_cast<int>(inputs.size()));
    assert(spec.output_dim() == static_cast<int>(outputs.size()));
    output_inputs.push_back(std::make_pair(outputs, inputs));
    specs.push_back(spec);
    return specs.back();
  }

  /**
   * Registers the previously defined blueprints and their corresponding network
   * parameters from the provided `params`.
   */
  template <typename Algebra>
  void instantiate(const std::vector<typename Algebra::Scalar> &params,
                   std::size_t param_index_offset = 0) const {
    static_assert(
        !is_neural_algebra<Algebra>::value,
        "Parameter vector provided to NeuralAugmentation::instantiate() must "
        "not be of NeuralScalar type.\n");

    using NAlgebra = NeuralAlgebra<Algebra>;
    if (param_index_offset >= params.size()) {
      return;
    }

    NAlgebra::Scalar::clear_all_blueprints();
    typedef typename NAlgebra::Scalar::NeuralNetworkType NeuralNetwork;
    std::size_t pi = param_index_offset;
    for (std::size_t i = 0; i < specs.size(); ++i) {
      NeuralNetwork net(specs[i]);
      int net_size = net.num_parameters();
      std::vector<typename Algebra::Scalar> net_params(
          params.begin() + pi, params.begin() + pi + net_size);
      net.set_parameters(net_params);
      NAlgebra::Scalar::add_blueprint(output_inputs[i].first,
                                      output_inputs[i].second, net);
      pi += net_size;
    }
  }

  /**
   * Assigns the names and l1, l2 loss coefficients on the parameters to match
   * the architectures of the defined networks.
   */
  void assign_estimation_parameters(
      std::vector<EstimationParameter> &params,
      std::size_t param_index_offset = 0,
      NeuralNetworkInitialization init_method = NN_INIT_XAVIER) const {
    std::size_t pi = param_index_offset;
    for (std::size_t i = 0; i < specs.size(); ++i) {
      std::string output_name = "net";
      for (const auto &output : output_inputs[i].first) {
        output_name += "_" + output;
      }
      std::string net_prefix = output_name + "_";
      // create sensible initialization for network weights
      std::vector<double> init_weights, init_biases;
      specs[i].template initialize<DoubleAlgebra>(init_weights, init_biases,
                                                  init_method);

      specs[i].template save_graphviz<DoubleAlgebra>(
          "init_net_" + std::to_string(i) + ".dot", output_inputs[i].second,
          output_inputs[i].first, init_weights, init_biases);

      int num_first_layer_weights =
          specs[i].num_units(1) * specs[i].input_dim();
      for (int wi = 0; wi < specs[i].num_weights(); ++wi, ++pi) {
        params[pi].name = net_prefix + "w_" + std::to_string(wi);
        params[pi].minimum = -weight_limit;
        params[pi].maximum = weight_limit;
        params[pi].value = init_weights[wi];
        if (wi < num_first_layer_weights) {
          // L1 lasso on input weights encourages input sparsity
          params[pi].l1_regularization = input_lasso_regularization;
        }
        // L2 regularization of weights in upper layers
        params[pi].l2_regularization = upper_l2_regularization;
      }
      for (int bi = 0; bi < specs[i].num_biases(); ++bi, ++pi) {
        params[pi].name = net_prefix + "b_" + std::to_string(bi);
        params[pi].minimum = -bias_limit;
        params[pi].maximum = bias_limit;
        params[pi].value = init_biases[bi];
      }
    }
  }

  /**
   * Assigns the names and l1, l2 loss coefficients on the parameters to match
   * the architectures of the defined networks.
   */
  template <std::size_t ParameterDim>
  void assign_estimation_parameters(
      std::array<EstimationParameter, ParameterDim> &params,
      std::size_t param_index_offset = 0,
      NeuralNetworkInitialization init_method = NN_INIT_XAVIER) const {
    if (num_total_parameters() + param_index_offset > ParameterDim) {
      std::cerr << "Error: at least "
                << num_total_parameters() + param_index_offset
                << " parameters are necessary for the neural augmentation.\n";
      assert(0);
      exit(1);
      return;
    }
    std::size_t pi = param_index_offset;
    for (std::size_t i = 0; i < specs.size(); ++i) {
      std::string output_name = "net";
      for (const auto &output : output_inputs[i].first) {
        output_name += "_" + output;
      }
      std::string net_prefix = output_name + "_";
      // create sensible initialization for network weights
      std::vector<double> init_weights, init_biases;
      specs[i].template initialize<DoubleAlgebra>(init_weights, init_biases,
                                                  init_method);

      specs[i].template save_graphviz<DoubleAlgebra>(
          "init_net_" + std::to_string(i) + ".dot", output_inputs[i].second,
          output_inputs[i].first, init_weights, init_biases);

      int num_first_layer_weights =
          specs[i].num_units(1) * specs[i].input_dim();
      for (int wi = 0; wi < specs[i].num_weights(); ++wi, ++pi) {
        params[pi].name = net_prefix + "w_" + std::to_string(wi);
        params[pi].minimum = -weight_limit;
        params[pi].maximum = weight_limit;
        params[pi].value = init_weights[wi];
        if (wi < num_first_layer_weights) {
          // L1 lasso on input weights encourages input sparsity
          params[pi].l1_regularization = input_lasso_regularization;
        }
        // L2 regularization of weights in upper layers
        params[pi].l2_regularization = upper_l2_regularization;
      }
      for (int bi = 0; bi < specs[i].num_biases(); ++bi, ++pi) {
        params[pi].name = net_prefix + "b_" + std::to_string(bi);
        params[pi].minimum = -bias_limit;
        params[pi].maximum = bias_limit;
        params[pi].value = init_biases[bi];
      }
    }
  }

  template <typename Algebra>
  std::vector<NeuralNetwork<Algebra>> extract_neural_networks(
      const std::map<std::string, double> &named_params) const {
    std::vector<NeuralNetwork<Algebra>> result;
    for (std::size_t i = 0; i < specs.size(); ++i) {
      NeuralNetwork<Algebra> net(specs[i]);
      net.weights.resize(net.num_weights());
      net.biases.resize(net.num_biases());
      std::string output_name = "net";
      for (const auto &output : output_inputs[i].first) {
        output_name += "_" + output;
      }
      std::string net_prefix = output_name + "_";
      for (int wi = 0; wi < net.num_weights(); ++wi) {
        std::string pname = net_prefix + "w_" + std::to_string(wi);
        if (named_params.find(pname) == named_params.end()) {
          std::cerr << "Error: Could not find value for NN weight \"" << pname
                    << "\".\n";
          net.weights[wi] = Algebra::zero();
        } else {
          net.weights[wi] = Algebra::from_double(named_params.at(pname));
        }
      }
      for (int bi = 0; bi < net.num_biases(); ++bi) {
        std::string pname = net_prefix + "b_" + std::to_string(bi);
        if (named_params.find(pname) == named_params.end()) {
          std::cerr << "Error: Could not find value for NN bias \"" << pname
                    << "\".\n";
          net.biases[bi] = Algebra::zero();
        } else {
          net.biases[bi] = Algebra::from_double(named_params.at(pname));
        }
      }
      result.push_back(net);
    }
    return result;
  }

  std::size_t num_total_parameters() const {
    std::size_t num = 0;
    for (const auto &spec : specs) {
      num += spec.num_parameters();
    }
    return num;
  }

  void save_graphviz(const std::vector<double> &params = {},
                     const std::string &prefix = "", bool show_arrows = true,
                     std::size_t param_index_offset = 0) const {
    std::size_t pi = param_index_offset;
    for (std::size_t i = 0; i < specs.size(); ++i) {
      std::vector<double> weights(specs[i].num_weights(), 1.),
          biases(specs[i].num_biases(), 1.);
      for (int j = 0; j < specs[i].num_weights() && pi < params.size();
           ++j, ++pi) {
        weights[j] = params[pi];
      }
      for (int j = 0; j < specs[i].num_biases() && pi < params.size();
           ++j, ++pi) {
        biases[j] = params[pi];
      }
      specs[i].template save_graphviz<DoubleAlgebra>(
          prefix + "net_" + std::to_string(i) + ".dot", output_inputs[i].second,
          {output_inputs[i].first}, weights, biases, show_arrows);
    }
  }
};

} // namespace tds

#endif // NEURAL_AUGMENTATION_H
