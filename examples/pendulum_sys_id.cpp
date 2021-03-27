#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "multi_body.hpp"
#include "utils/differentiation.hpp"
#include "utils/pendulum.hpp"
#include "world.hpp"

const int kNumLinks = 1;
const int kNumTimesteps = 100;

const bool kLossOnQ = true;
const bool kLossOnQd = true;
const bool kLossOnQdd = false;

std::vector<std::vector<double>> groundtruth_states;

/**
 * Basic system identification experiment where the lengths of the links of a
 * compound pendulum need to be estimated, given the trajectories of joint
 * positions, (and optionally) velocities and accelerations.
 */

template <typename Algebra>
struct Simulation {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar dt{Algebra::from_double(0.01)};

  // dimensionality of the parameter vector (important for some AD libraries)
  static const int kDim = kNumLinks;

  void rollout(const std::vector<Scalar>& params,
               std::vector<std::vector<Scalar>>* output_states) const {
    tds::World<Algebra> world;
    tds::MultiBody<Algebra> &pendulum = *world.create_multi_body();
    init_compound_pendulum(pendulum, world, kNumLinks, params);
    std::vector<Scalar> state(3 * kNumLinks);
    for (int t = 0; t < kNumTimesteps; ++t) {
      // populate state with q, qd, qdd
      int i = 0;
      for (int j = 0; j < kNumLinks; ++j, ++i) {
        state[i] = pendulum.q(j);
      }
      for (int j = 0; j < kNumLinks; ++j, ++i) {
        state[i] = pendulum.qd(j);
      }
      for (int j = 0; j < kNumLinks; ++j, ++i) {
        state[i] = pendulum.qdd(j);
      }
      output_states->push_back(state);

      // simulate and integrate
      tds::forward_dynamics(pendulum, world.get_gravity());
      tds::integrate_euler(pendulum, dt);
    }
  }

  // call operator that computes the cost; to be executed by the optimizer
  Scalar operator()(const std::vector<Scalar>& params) const {
    assert(!groundtruth_states.empty());

    Scalar cost = Algebra::zero();
    std::vector<std::vector<Scalar>> our_states;
    rollout(params, &our_states);
    for (int t = 0; t < kNumTimesteps; ++t) {
      const auto& true_state = groundtruth_states[t];
      const auto& our_state = our_states[t];
      // compute squared L2 norm over state difference
      if constexpr (kLossOnQ) {
        for (int j = 0; j < kNumLinks; ++j) {
          Scalar diff = Algebra::from_double(true_state[j]) - our_state[j];
          cost += diff * diff;
        }
      }
      if constexpr (kLossOnQd) {
        for (int j = 0; j < kNumLinks; ++j) {
          Scalar diff =
              Algebra::from_double(true_state[j + 3]) - our_state[j + 3];
          cost += diff * diff;
        }
      }
      if constexpr (kLossOnQdd) {
        for (int j = 0; j < kNumLinks; ++j) {
          Scalar diff =
              Algebra::from_double(true_state[j + 6]) - our_state[j + 6];
          cost += diff * diff;
        }
      }
    }
    cost /= kNumTimesteps;
    return cost;
  }
};

int main(int argc, char* argv[]) {
  // first generate a groundtruth trajectory from a pendulum with all link
  // lengths = 1.5
  Simulation<tds::EigenAlgebra> sim;
  std::vector<double> true_params(kNumLinks, 1.5);
  sim.rollout(true_params, &groundtruth_states);

  // run gradient descent
  double learning_rate = 0.3;
  typedef tds::GradientFunctional<tds::DIFF_CPPAD_AUTO, Simulation> GradFun;
  std::vector<double> our_params(kNumLinks, 1.0);  // initial guess
  GradFun diffsim;
  for (int i = 0; i < 100; ++i) {
    printf("Iteration %i - cost: %.5f\n", i, diffsim.value(our_params));
    const auto& grad = diffsim.gradient(our_params);
    for (int j = 0; j < kNumLinks; ++j) {
      our_params[j] -= learning_rate * grad[j];
    }
  }
  printf("Estimated parameters (should be [");
  for (int j = 0; j < kNumLinks; ++j) {
    printf(" %.2f", true_params[j]);
  }
  printf(" ]):\n");
  for (int j = 0; j < kNumLinks; ++j) {
    printf("\t%.5f", our_params[j]);
  }
  printf("\n");

  return EXIT_SUCCESS;
}