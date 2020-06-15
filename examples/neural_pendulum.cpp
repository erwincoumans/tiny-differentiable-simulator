#include <fstream>

#include "neural_scalar.h"
#include "pendulum.h"
#include "pybullet_visualizer_api.h"
#include "tiny_ceres_estimator.h"
#include "tiny_file_utils.h"
#include "tiny_multi_body.h"
#include "tiny_world.h"

// whether to use Parallel Basin Hopping
#define USE_PBH false
// whether the state consists of [q qd] or just q
#define STATE_INCLUDES_QD false
std::vector<double> start_state;
const int param_dim = 2;

#ifdef USE_MATPLOTLIB
template <typename T>
void plot_trajectory(const std::vector<std::vector<T>> &states) {
  typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                             CeresUtils<param_dim>>
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

template <typename T>
void visualize_trajectory(const std::vector<std::vector<T>> &states,
                          const std::vector<T> &params, const T &dt) {
  typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                             CeresUtils<param_dim>>
      Utils;
  typedef PyBulletVisualizerAPI VisualizerAPI;
  VisualizerAPI *visualizer = new VisualizerAPI();
  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);
  char path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_filename.c_str(), path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string search_path = path;
  visualizer->connect(eCONNECT_GUI);
  visualizer->setAdditionalSearchPath(search_path);
  if (visualizer->canSubmitCommand()) {
    visualizer->resetSimulation();
  }
  TinyWorld<T, Utils> world;
  TinyMultiBody<T, Utils> *mb = world.create_multi_body();
  std::vector<T> link_lengths(params.begin(), params.begin() + 2);
  std::vector<T> masses(2);
  for (int i = 0; i < 2; ++i) {
    masses[i] = params[2 + i];
  }
  init_compound_pendulum<T, Utils>(*mb, world, 2, link_lengths, masses);
  std::vector<int> mbvisuals;
  if (visualizer->canSubmitCommand()) {
    for (int i = 0; i < mb->m_links.size(); i++) {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
    }
  }

  btVector3 basePos(0, 0, -0.2);
  double distance = 1.8;
  visualizer->resetDebugVisualizerCamera(distance, 0, 90, basePos);

  std::vector<T> q(2);
  for (const std::vector<T> &state : states) {
    q[0] = state[0];
    q[1] = state[1];
    mb->forward_kinematics(q);
    printf("  q: [%.6f  %.6f]\n", Utils::getDouble(q[0]),
           Utils::getDouble(q[1]));

    std::this_thread::sleep_for(
        std::chrono::duration<double>(Utils::getDouble(dt)));
    // sync transforms
    int visual_index = 0;
    for (int l = 0; l < mb->m_links.size(); l++) {
      // if (mb->m_X_visuals.empty()) continue;

      int sphereId = mbvisuals[visual_index++];
      TinyQuaternion<T, Utils> rot;
      const TinySpatialTransform<T, Utils> &geom_X_world =
          mb->m_links[l].m_X_world * mb->m_links[l].m_X_visuals[0];
      btVector3 base_pos(Utils::getDouble(geom_X_world.m_translation.getX()),
                         Utils::getDouble(geom_X_world.m_translation.getY()),
                         Utils::getDouble(geom_X_world.m_translation.getZ()));
      geom_X_world.m_rotation.getRotation(rot);
      // printf("Sphere %i position: %.6f %.6f %.6f\n", sphereId, base_pos[0],
      // base_pos[1], base_pos[2]);
      btQuaternion base_orn(
          Utils::getDouble(rot.getX()), Utils::getDouble(rot.getY()),
          Utils::getDouble(rot.getZ()), Utils::getDouble(rot.getW()));
      visualizer->resetBasePositionAndOrientation(sphereId, base_pos, base_orn);
    }
  }

  visualizer->disconnect();
  delete visualizer;
}

/**
 * Roll-out pendulum dynamics, and compute states [q, qd].
 */
template <typename Scalar = double, typename Utils = DoubleUtils>
void rollout_pendulum(const std::vector<Scalar> &params,
                      std::vector<std::vector<Scalar>> &output_states,
                      int time_steps, double dt,
                      const std::array<double, 2> &damping = {0., 0.}) {
  TinyVector3<Scalar, Utils> gravity(Utils::zero(), Utils::zero(),
                                     Utils::fraction(-981, 100));
  output_states.resize(time_steps);
  TinyWorld<Scalar, Utils> world;
  TinyMultiBody<Scalar, Utils> *mb = world.create_multi_body();
  init_compound_pendulum<Scalar, Utils>(*mb, world, 2);

  //   std::vector<Scalar> link_lengths(params.begin(), params.begin() + 2);
  //   std::vector<Scalar> masses(params.begin() + 2, params.begin() + 4);
  //   init_compound_pendulum<Scalar, Utils>(*mb, world, 2, link_lengths,
  //   masses);
  // #if ESTIMATE_INERTIA
  //   TinyMatrix3x3<Scalar, Utils> inertia_0;
  //   inertia_0.set_zero();
  //   inertia_0(0, 0) = params[4];
  //   inertia_0(1, 1) = params[5];
  //   inertia_0(2, 2) = params[6];
  //   TinyVector3<Scalar, Utils> com_0(Utils::zero(), link_lengths[0],
  //                                    Utils::zero());
  //   mb->m_links[0].m_I =
  //       TinySymmetricSpatialDyad<Scalar, Utils>::computeInertiaDyad(
  //           masses[0], com_0, inertia_0);
  //   TinyMatrix3x3<Scalar, Utils> inertia_1;
  //   inertia_1.set_zero();
  //   inertia_1(0, 0) = params[7];
  //   inertia_1(1, 1) = params[8];
  //   inertia_1(2, 2) = params[9];
  //   TinyVector3<Scalar, Utils> com_1(Utils::zero(), link_lengths[1],
  //                                    Utils::zero());
  //   mb->m_links[1].m_I =
  //       TinySymmetricSpatialDyad<Scalar, Utils>::computeInertiaDyad(
  //           masses[1], com_1, inertia_1);
  // #endif

  if constexpr (std::is_same_v<Scalar, double>) {
    mb->m_links[0].m_damping = damping[0];
    mb->m_links[1].m_damping = damping[1];
  }
  // if constexpr (is_neural_scalar<Scalar, Utils>::value) {
  //   if (!params.empty()) {
  //     mb->m_tau[0].connect(&(mb->m_qd[0]));
  //     mb->m_tau[1].connect(&(mb->m_qd[1]));

  //     mb->m_tau[0].net().weights[0] = params[0].evaluate();
  //     mb->m_tau[1].net().weights[0] = params[1].evaluate();

  //     // printf("tau[0]'s net:\n");
  //     // mb->m_tau[0].net().print_params();
  //     // printf("tau[1]'s net:\n");
  //     // mb->m_tau[1].net().print_params();
  //   }
  // }
  if constexpr (is_neural_scalar<Scalar, Utils>::value) {
    if (!params.empty()) {
      typedef typename Scalar::NeuralNetworkType NeuralNetwork;
      NeuralNetwork net_tau_0(1);
      net_tau_0.add_linear_layer(NN_ACT_IDENTITY, 1, false);
      net_tau_0.initialize();
      net_tau_0.weights[0] = params[0].evaluate();
      Scalar::add_blueprint("tau_0", {"qd_0"}, net_tau_0);

      NeuralNetwork net_tau_1(1);
      net_tau_1.add_linear_layer(NN_ACT_IDENTITY, 1, false);
      net_tau_1.initialize();
      net_tau_1.weights[0] = params[1].evaluate();
      Scalar::add_blueprint("tau_1", {"qd_1"}, net_tau_1);

      // assign scalar names so that the defined blueprints can be used
      mb->m_qd[0].assign("qd_0");
      mb->m_qd[1].assign("qd_1");
      mb->m_tau[0].assign("tau_0");
      mb->m_tau[1].assign("tau_1");
    }
  }

  if (static_cast<int>(start_state.size()) >= mb->dof()) {
    for (int i = 0; i < mb->dof(); ++i) {
      mb->m_q[i] = Utils::scalar_from_double(start_state[i]);
    }
    if (static_cast<int>(start_state.size()) >= 2 * mb->dof()) {
      for (int i = 0; i < mb->dof_qd(); ++i) {
        mb->m_qd[i] = Utils::scalar_from_double(start_state[i + mb->dof()]);
      }
    }
  }
  for (int t = 0; t < time_steps; ++t) {
#if STATE_INCLUDES_QD
    output_states[t].resize(2 * mb->dof());
#else
    output_states[t].resize(mb->dof());
#endif
    for (int i = 0; i < mb->dof(); ++i) {
      output_states[t][i] = mb->m_q[i];
#if STATE_INCLUDES_QD
      output_states[t][i + mb->dof()] = mb->m_qd[i];
#endif
    }
    mb->forward_dynamics(gravity);

    // if (t > 150) {
    //   mb->print_state();
    // }
    // mb->integrate_q(Scalar(dt));
    mb->integrate(Utils::scalar_from_double(dt));
  }

#if !USE_PBH
  // visualize_trajectory(output_states, params, Scalar(dt));
#endif
}

template <ResidualMode ResMode>
class PendulumEstimator
    : public TinyCeresEstimator<param_dim, (1 + STATE_INCLUDES_QD) * 2,
                                ResMode> {
 public:
  typedef TinyCeresEstimator<param_dim, (1 + STATE_INCLUDES_QD) * 2, ResMode>
      CeresEstimator;
  using CeresEstimator::kStateDim, CeresEstimator::kParameterDim;
  using CeresEstimator::parameters;
  using typename CeresEstimator::ADScalar;

  int time_steps;

  PendulumEstimator(int time_steps, double dt)
      : CeresEstimator(dt), time_steps(time_steps) {
    for (int i = 0; i < param_dim; ++i) {
      parameters[i] = {"weight_" + std::to_string(i + 1), 0., -1., 1.};
    }
  }

  void rollout(const std::vector<ADScalar> &params,
               std::vector<std::vector<ADScalar>> &output_states,
               double dt) const override {
    typedef CeresUtils<kParameterDim> ADUtils;
    typedef NeuralScalar<ADScalar, ADUtils> NScalar;
    typedef NeuralScalarUtils<ADScalar, ADUtils> NUtils;
    auto n_params = NUtils::to_neural(params);
    std::vector<std::vector<NScalar>> n_output_states;
    rollout_pendulum<NScalar, NUtils>(n_params, n_output_states, time_steps,
                                      dt);
    for (const auto &state : n_output_states) {
      output_states.push_back(NUtils::from_neural(state));
    }
  }
  void rollout(const std::vector<double> &params,
               std::vector<std::vector<double>> &output_states,
               double dt) const override {
    typedef NeuralScalar<double, DoubleUtils> NScalar;
    typedef NeuralScalarUtils<double, DoubleUtils> NUtils;
    auto n_params = NUtils::to_neural(params);
    std::vector<std::vector<NScalar>> n_output_states;
    rollout_pendulum<NScalar, NUtils>(n_params, n_output_states, time_steps,
                                      dt);
    for (const auto &state : n_output_states) {
      output_states.push_back(NUtils::from_neural(state));
    }
  }
};

void print_states(const std::vector<std::vector<double>> &states) {
  for (const auto &s : states) {
    for (double d : s) {
      printf("%.2f ", d);
    }
    printf("\n");
  }
}

int main(int argc, char *argv[]) {
  typedef PendulumEstimator<RES_MODE_1D> Estimator;

  // typedef CeresUtils<param_dim> ADUtils;
  // typedef NeuralScalar<Estimator::ADScalar, ADUtils> NScalar;
  // typedef NeuralScalarUtils<Estimator::ADScalar, ADUtils> NUtils;
  // typedef NeuralScalar<double, DoubleUtils> NScalar;
  // typedef NeuralScalarUtils<double, DoubleUtils> NUtils;
  // NScalar a(NUtils::scalar_from_double(3.));
  // NScalar b(NUtils::scalar_from_double(5.));
  // NScalar c(NUtils::scalar_from_double(9.));
  // b = c;
  // TinyVector3<NScalar, NUtils> com(a, b, c);
  // printf("%f\n", NUtils::getDouble(com.length()));

  // NScalar mass(NUtils::scalar_from_double(.5));
  // std::vector<NScalar> vs(3);
  // vs.push_back(a);
  // vs[0] += b;
  // vs[1] = NUtils::convert(0);
  // printf("%f\n", NUtils::getDouble(vs[0]));
  // com.setValue(NUtils::convert(0), NUtils::convert(1), NUtils::convert(0));
  // com.print("com");
  // TinyMatrix3x3<NScalar, NUtils> inertia_C;
  // inertia_C.set_identity();
  // TinyVector3<NScalar, NUtils> h = com * mass;
  // NScalar norm_cm = h.length();
  // // NScalar norm_cm = (com * mass).length();
  // printf("inplace norm: %f\n", NUtils::getDouble(norm_cm));
  // NScalar o = NUtils::zero();
  // TinyMatrix3x3<NScalar, NUtils> hx = TinyVectorCrossMatrix(com);
  // TinyMatrix3x3<NScalar, NUtils> hxt = hx.transpose();
  // TinyMatrix3x3<NScalar, NUtils> hxxt = hx * hxt;
  // TinyMatrix3x3<NScalar, NUtils> I = inertia_C + hx * hx.transpose(); // *
  // mass;

  // return 0;

  const double dt = 1. / 500;
  const double time_limit = 5;
  const int time_steps = time_limit / dt;

  google::InitGoogleLogging(argv[0]);

  // keep the target_times empty, since target time steps match the model
  std::vector<double> target_times;
  std::vector<std::vector<double>> target_states;
  // rollout pendulum with damping
  std::vector<double> empty_params;
  std::array<double, 2> true_damping{0.2, 0.1};
  rollout_pendulum(empty_params, target_states, time_steps, dt, true_damping);
  start_state = target_states[0];

  std::function<std::unique_ptr<Estimator>()> construct_estimator =
      [&target_times, &target_states, &time_steps, &dt]() {
        auto estimator = std::make_unique<Estimator>(time_steps, dt);
        estimator->target_times = target_times;
        estimator->target_states = target_states;
        estimator->options.minimizer_progress_to_stdout = !USE_PBH;
        estimator->options.max_num_consecutive_invalid_steps = 100;
        // divide each cost term by integer time step ^ 2 to reduce gradient
        // explosion
        estimator->divide_cost_by_time_factor = 10.;
        estimator->divide_cost_by_time_exponent = 1.2;
        return estimator;
      };

#if USE_PBH
  std::array<double, param_dim> initial_guess;
  for (int i = 0; i < param_dim; ++i) {
    initial_guess[i] = 0.0;
  }
  BasinHoppingEstimator<param_dim, Estimator> bhe(construct_estimator,
                                                  initial_guess);
  bhe.time_limit = 20;
  bhe.run();

  printf("Optimized parameters:");
  for (int i = 0; i < param_dim; ++i) {
    printf(" %.8f", bhe.params[i]);
  }
  printf("\n");

  printf("Best cost: %f\n", bhe.best_cost());

  std::vector<double> best_params;
  for (const auto &p : bhe.params) {
    best_params.push_back(p);
  }
  target_states.clear();
#else
  std::unique_ptr<Estimator> estimator = construct_estimator();
  estimator->setup(new ceres::HuberLoss(1.));

  // XXX verify cost is zero for the true network weights
  double cost;
  double gradient[4];
  double my_params[] = {-true_damping[0], -true_damping[1]};
  estimator->compute_loss(my_params, &cost, gradient);
  std::cout << "Gradient: " << gradient[0] << "  " << gradient[1] << "  "
            << gradient[2] << "  " << gradient[3] << "  \n";
  std::cout << "Cost: " << cost << "\n";
  assert(cost < 1e-4);

  // return 0;

  auto summary = estimator->solve();
  std::cout << summary.FullReport() << std::endl;
  std::cout << "Final cost: " << summary.final_cost << "\n";

  std::vector<double> best_params;
  for (const auto &p : estimator->parameters) {
    printf("%s: %.3f\n", p.name.c_str(), p.value);
    best_params.push_back(p.value);
  }

  std::ofstream file("param_evolution.txt");
  for (const auto &params : estimator->parameter_evolution()) {
    for (int i = 0; i < static_cast<int>(params.size()); ++i) {
      file << params[i];
      if (i < static_cast<int>(params.size()) - 1) file << "\t";
    }
    file << "\n";
  }
  file.close();
#endif

  rollout_pendulum<double, DoubleUtils>(best_params, target_states, time_steps,
                                        dt);
  std::ofstream traj_file("estimated_neural_trajectory.csv");
  for (int t = 0; t < time_steps; ++t) {
    traj_file << (t * dt);
    for (double v : target_states[t]) {
      traj_file << "\t" << v;
    }
    traj_file << "\n";
  }
  traj_file.close();

  return EXIT_SUCCESS;
}
