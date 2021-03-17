#pragma once

#include <ctime>
#include <cxxopts.hpp>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "nlohmann/json.hpp"

#ifdef USE_PAGMO
#include <pagmo/algorithm.hpp>
// #include <pagmo/algorithms/ipopt.hpp>
#include <pagmo/algorithms/nlopt.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#endif

#ifdef USE_OPTIM
#include "utils/optim_gd.hpp"
#endif

#ifdef USE_CERES
#include "ceres_estimator.hpp"
#endif

#include "neural_augmentation.hpp"
#include "optimization_problem.hpp"
#include "utils/stopwatch.hpp"

namespace tds {
class Experiment {
 protected:
  const std::string name;
  const std::filesystem::path logdir;
  nlohmann::json log;

  NeuralAugmentation augmentation_;

  Stopwatch watch_evolution;
  Stopwatch watch_total;

  std::size_t evolution{0};

 public:
  virtual ~Experiment() {}

  Experiment(const std::string& name) : name(name), logdir("./logs/" + name) {
    std::filesystem::create_directories(logdir);
    log["name"] = name;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    log["created"] = oss.str();
    log["settings"]["optimizer"] = "pagmo";
    log["settings"]["pagmo"]["solver"] = "nlopt";
    log["settings"]["pagmo"]["nlopt"]["solver"] = "lbfgs";
    log["settings"]["pagmo"]["nlopt"]["xtol_rel"] = 1e-2;
    log["settings"]["pagmo"]["nlopt"]["xtol_abs"] = 1e-10;
    log["settings"]["pagmo"]["nlopt"]["verbosity"] = 1;
    log["settings"]["pagmo"]["nlopt"]["max_time"] = 10.;
    log["settings"]["pagmo"]["optimlib"]["method"] = 6;
    log["settings"]["pagmo"]["optimlib"]["iter_max"] = 20;
    log["settings"]["pagmo"]["optimlib"]["print_level"] = 0;
    log["settings"]["pagmo"]["optimlib"]["par_step_size"] = 0.1;
    log["settings"]["pagmo"]["optimlib"]["step_decay"] = true;
    log["settings"]["pagmo"]["optimlib"]["step_decay_periods"] = 10;
    log["settings"]["pagmo"]["optimlib"]["step_decay_val"] = 0.5;
    log["settings"]["pagmo"]["optimlib"]["par_momentum"] = 0.9;
    log["settings"]["pagmo"]["optimlib"]["par_ada_norm_term"] = 1e-8;
    log["settings"]["pagmo"]["optimlib"]["par_ada_rho"] = 0.9;
    log["settings"]["pagmo"]["optimlib"]["ada_max"] = false;
    log["settings"]["pagmo"]["optimlib"]["par_adam_beta_1"] = 0.9;
    log["settings"]["pagmo"]["optimlib"]["par_adam_beta_2"] = 0.999;
    log["settings"]["pagmo"]["optimlib"]["clip_grad"] = false;
    log["settings"]["pagmo"]["optimlib"]["clip_max_norm"] = false;
    log["settings"]["pagmo"]["optimlib"]["clip_min_norm"] = false;
    log["settings"]["pagmo"]["optimlib"]["clip_norm_type"] = 2;
    log["settings"]["pagmo"]["optimlib"]["clip_norm_bound"] = 5.0;
    log["settings"]["pagmo"]["num_islands"] = 5;
    log["settings"]["pagmo"]["num_individuals"] = 7;
    log["settings"]["pagmo"]["num_evolutions"] = 20;
    log["parameter_evolution"] = {};
    log["settings"]["augmentation"]["input_lasso_regularization"] = 0.;
    log["settings"]["augmentation"]["upper_l2_regularization"] = 0.;
    log["settings"]["augmentation"]["weight_limit"] = 0.5;
    log["settings"]["augmentation"]["bias_limit"] = 0.5;
    log["settings"]["augmentation"]["augmentation_is_residual"] = true;
    log["settings"]["separate_log_per_episode"] = false;
    log["settings"]["separate_graphviz_per_episode"] = true;
    log["settings"]["log_prefix"] = "";
  }

  NeuralAugmentation& augmentation() { return augmentation_; }
  const NeuralAugmentation& augmentation() const { return augmentation_; }

  bool save_log(const std::string& filename) const {
    namespace fs = std::filesystem;
    std::ofstream file(logdir / filename);
    if (!file.good()) {
      std::cerr << "Failed to save experiment log at " << filename << std::endl;
      return false;
    }
    file << log.dump(2);
    file.close();
    std::string abs_filename = fs::canonical(logdir / filename).u8string();
    std::cout << "Saved experiment log at " << abs_filename << std::endl;
    return true;
  }

  void parse_settings(int argc, char* argv[]) {
    cxxopts::Options options(name);
    std::map<std::string, nlohmann::json*> index;
    get_options(log["settings"], &options, &index);
    try {
      auto result = options.parse(argc, argv);
      if (result.count("help")) {
        std::cout << options.help() << std::endl;
        std::exit(0);
      }
      for (auto& [key, value] : index) {
        // clang-format off
        switch (value->type()) {
          case nlohmann::detail::value_t::boolean:
            *value = result[key].as<bool>();
            break;
          case nlohmann::detail::value_t::number_float:
            *value = result[key].as<double>();
            break;
          case nlohmann::detail::value_t::number_integer:
            *value = result[key].as<int>();
            break;
          case nlohmann::detail::value_t::number_unsigned:
            *value = result[key].as<unsigned int>();
            break;
          case nlohmann::detail::value_t::string: default: 
            *value = result[key].as<std::string>();
            break;
        }
        // clang-format on
      }
    } catch (const cxxopts::OptionException& e) {
      std::cout << "error parsing options: " << e.what() << std::endl;
      std::exit(1);
    }
  }

  nlohmann::json& operator[](const std::string& key) { return log[key]; }
  const nlohmann::json& operator[](const std::string& key) const {
    return log[key];
  }

  virtual void after_iteration(const std::vector<double>&) {}

  template <typename OptimizationProblem>
  void run(OptimizationProblem& problem) {
    if constexpr (OptimizationProblem::kDiffMethod ==
                  tds::DIFF_CPPAD_CODEGEN_AUTO) {
      // CodeGenSettings settings;
      // // use current parameter values as function arguments for generating
      // the
      // // gradient code
      // for (const auto& param : problem.parameters()) {
      //   settings.default_x.push_back(param.value);
      // }
      // OptimizationProblem::CostFunctor::Compile(settings);
    }

    std::cout << std::endl;
    std::cout << "Running experiment \"" << name << "\" with "
              << OptimizationProblem::kParameterDim
              << " parameters, consisting of "
              << augmentation_.num_total_parameters()
              << " neural network parameters." << std::endl;

    if (!problem.parameters().empty()) {
      std::cout << "Parameters:\n";
      for (const auto& p : problem.parameters()) {
        std::cout << "\t" << p << "\t[" << p.minimum << ", " << p.maximum << "]"
                  << std::endl;
      }
    }
    std::cout << std::endl;

    watch_total.start();
    if (log["settings"]["optimizer"] == "pagmo") {
      run_pagmo(problem);
      return;
    }
    if (log["settings"]["optimizer"] == "ceres") {
      run_ceres(problem);
      return;
    }
    std::cerr << "Invalid optimizer setting \"" << log["settings"]["optimizer"]
              << "\".\n";
    std::exit(1);
  }

 protected:
  template <typename OptimizationProblemT>
  void update_log(OptimizationProblemT& problem) {
    log["param_dim"] = problem.kParameterDim;
    log["diff_method"] = tds::diff_method_name(problem.kDiffMethod);
    for (const auto& param : problem.parameters()) {
      log["params"][param.name] = param.value;
    }
  }

#ifdef USE_PAGMO
  pagmo::algorithm get_pagmo_algorithm() const {
    if (log["settings"]["pagmo"]["solver"] == "nlopt") {
      pagmo::nlopt solver(log["settings"]["pagmo"]["nlopt"]["solver"]);
      solver.set_maxtime(
          log["settings"]["pagmo"]["nlopt"]["max_time"]);  // in seconds
      solver.set_verbosity(log["settings"]["pagmo"]["nlopt"]
                              ["verbosity"]);  // print every n function evals
      solver.set_xtol_abs(log["settings"]["pagmo"]["nlopt"]["xtol_abs"]);
      solver.set_xtol_rel(log["settings"]["pagmo"]["nlopt"]["xtol_rel"]);

      pagmo::algorithm algo{solver};
      return algo;
    }
    if (log["settings"]["pagmo"]["solver"] == "optimlib") {
#ifdef USE_OPTIM
      auto gd = tds::optim_gd();
      gd.settings.gd_settings.method =
          log["settings"]["pagmo"]["optimlib"]["method"];
      gd.settings.iter_max = log["settings"]["pagmo"]["optimlib"]["iter_max"];
      gd.settings.print_level =
          log["settings"]["pagmo"]["optimlib"]["print_level"];
      gd.settings.gd_settings.par_step_size =
          log["settings"]["pagmo"]["optimlib"]["par_step_size"];
      gd.settings.gd_settings.step_decay =
          log["settings"]["pagmo"]["optimlib"]["step_decay"];
      gd.settings.gd_settings.step_decay_periods =
          log["settings"]["pagmo"]["optimlib"]["step_decay_periods"];
      gd.settings.gd_settings.step_decay_val =
          log["settings"]["pagmo"]["optimlib"]["step_decay_val"];
      gd.settings.gd_settings.par_momentum =
          log["settings"]["pagmo"]["optimlib"]["par_momentum"];
      gd.settings.gd_settings.par_ada_norm_term =
          log["settings"]["pagmo"]["optimlib"]["par_ada_norm_term"];
      gd.settings.gd_settings.par_ada_rho =
          log["settings"]["pagmo"]["optimlib"]["par_ada_rho"];
      gd.settings.gd_settings.ada_max =
          log["settings"]["pagmo"]["optimlib"]["ada_max"];
      gd.settings.gd_settings.par_adam_beta_1 =
          log["settings"]["pagmo"]["optimlib"]["par_adam_beta_1"];
      gd.settings.gd_settings.par_adam_beta_2 =
          log["settings"]["pagmo"]["optimlib"]["par_adam_beta_2"];
      gd.settings.gd_settings.clip_grad =
          log["settings"]["pagmo"]["optimlib"]["clip_grad"];
      gd.settings.gd_settings.clip_max_norm =
          log["settings"]["pagmo"]["optimlib"]["clip_max_norm"];
      gd.settings.gd_settings.clip_min_norm =
          log["settings"]["pagmo"]["optimlib"]["clip_min_norm"];
      gd.settings.gd_settings.clip_norm_type =
          log["settings"]["pagmo"]["optimlib"]["clip_norm_type"];
      gd.settings.gd_settings.clip_norm_bound =
          log["settings"]["pagmo"]["optimlib"]["clip_norm_bound"];
      pagmo::algorithm algo{gd};
      return algo;
#else
      throw std::runtime_error(
          "'USE_OPTIM' option must be set to use the OptimLib optimizers in "
          "Pagmo.");
#endif
    }
    if (log["settings"]["pagmo"]["solver"] == "sade") {
      pagmo::algorithm algo{pagmo::sade()};
      return algo;
    }
    throw std::runtime_error("Unknown Pagmo solver: " +
                             (std::string)log["settings"]["pagmo"]["solver"]);
  }
#endif

  template <typename OptimizationProblemT>
  void run_pagmo(OptimizationProblemT& problem) {
#ifdef USE_PAGMO
    update_log(problem);
    pagmo::problem prob(problem);
    pagmo::algorithm algo = get_pagmo_algorithm();
    // pagmo::algorithm algo{pagmo::sade()};
    // pagmo::algorithm algo{pagmo::ipopt()};
    std::size_t num_islands = log["settings"]["pagmo"]["num_islands"];
    std::size_t num_individuals = log["settings"]["pagmo"]["num_individuals"];
    std::size_t num_evolutions = log["settings"]["pagmo"]["num_evolutions"];
    pagmo::archipelago archi{num_islands, algo, prob, num_individuals};
    for (evolution = 0; evolution < num_evolutions; ++evolution) {
      std::cout << "###### EVOLUTION " << evolution << " ######\n";
      watch_evolution.start();
      archi.evolve();
      archi.wait_check();
      log["episodes"][evolution]["runtime"] = watch_evolution.stop();
      log["total_duration"] = watch_total.elapsed();
      int i = 0, best_island = 0;
      for (const auto& island : archi) {
        auto params = island.get_population().champion_x();
        double f = island.get_population().champion_f()[0];
        if (f < archi[best_island].get_population().champion_f()[0]) {
          best_island = i;
        }
        std::cout << "Best x from island " << (i++) << ": ";
        for (std::size_t j = 0; j < problem.kParameterDim; ++j) {
          std::cout << params[j] << " ";
        }
        std::cout << "\tf(x): " << f << std::endl;
      }
      auto best_params = archi[best_island].get_population().champion_x();
      for (std::size_t j = 0; j < problem.kParameterDim; ++j) {
        problem.parameters()[j].value = best_params[j];
        std::cout << problem.parameters()[j] << std::endl;
      }

      double best_cost = problem.fitness(best_params)[0];
      std::cout << "Best cost from evolution " << evolution << ": " << best_cost
                << std::endl;
      log["episodes"][evolution]["best_cost"] = best_cost;
      std::cout << "Training cost: " << best_cost << std::endl;
      // std::cout << "Gradient: ";
      // const auto& gradient = problem.gradient(best_params);
      // for (std::size_t i = 0; i < problem.kParameterDim; ++i) {
      //   std::cout << gradient[i] << "  ";
      // }
      // std::cout << std::endl;

      for (const auto& p : problem.parameters()) {
        log["episodes"][evolution]["parameters"][p.name] = p.value;
      }

      std::string log_prefix = log["settings"]["log_prefix"];
      std::string dotfilename = log_prefix + name;
      if (log["settings"]["separate_graphviz_per_episode"]) {
        dotfilename += "_" + std::to_string(evolution);
      }
      augmentation_.save_graphviz(best_params, logdir / dotfilename);

      after_iteration(best_params);

      std::string logfilename = log_prefix + name;
      if (log["settings"]["separate_log_per_episode"]) {
        logfilename += "_" + std::to_string(evolution);
      }
      logfilename += ".json";
      save_log(logfilename);
    }
#else
    throw std::runtime_error(
        "CMake option 'USE_PAGMO' needs to be active to use Pagmo.");
#endif
  }

  template <typename OptimizationProblemT>
  void run_ceres(OptimizationProblemT& problem) {
#ifdef USE_CERES
    update_log(problem);
    tds::CeresEstimator estimator(&problem);
    estimator.setup();
    auto summary = estimator.solve();
    std::cout << summary.FullReport() << std::endl;
    // parameters = problem.parameters();
    log["settings"]["solver"]["name"] = "ceres-LM";
    log["total_duration"] = watch_total.elapsed();
#else
    throw std::runtime_error(
        "CMake option 'USE_CERES' needs to be active to use Ceres.");
#endif
  }

  virtual void save_settings() {}

  // get JSON leaves and populate options with settings
  static void get_options(nlohmann::json& data, cxxopts::Options* options,
                          std::map<std::string, nlohmann::json*>* index,
                          const std::string& prefix = "") {
    if (!data.is_object()) return;

    std::string pref = prefix.empty() ? "" : prefix + "_";
    for (auto& [key, value] : data.items()) {
      if (value.is_primitive()) {
        switch (value.type()) {
          case nlohmann::detail::value_t::boolean:
            options->add_options()(prefix + key, "",
                                   cxxopts::value<bool>()->default_value(
                                       std::to_string((bool)value)));
            break;
          case nlohmann::detail::value_t::number_float:
            options->add_options()(prefix + key, "",
                                   cxxopts::value<double>()->default_value(
                                       std::to_string((double)value)));
            break;
          case nlohmann::detail::value_t::number_integer:
            options->add_options()(prefix + key, "",
                                   cxxopts::value<int>()->default_value(
                                       std::to_string((int)value)));
            break;
          case nlohmann::detail::value_t::number_unsigned:
            options->add_options()(
                prefix + key, "",
                cxxopts::value<unsigned int>()->default_value(
                    std::to_string((unsigned int)value)));
            break;
          case nlohmann::detail::value_t::string:
          default:
            options->add_options()(
                prefix + key, "",
                cxxopts::value<std::string>()->default_value(value));
            break;
        }
        (*index)[prefix + key] = &value;
      } else if (value.is_object()) {
        get_options(value, options, index, prefix + key);
      }
    }
  }
};
}  // namespace tds
