#include <ostream>

#include "environments/cartpole_environment.h"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"

typedef TinyAlgebra<double, ::TINY::DoubleUtils> TAlgebra;

typedef CartpoleEnv<TAlgebra> Environment;

std::vector<double> trained_weights = {0.181676, 66.597227, 72.595358,
                                       108.902911, -0.708356};

int main(int argc, char* argv[]) {
  ContactSimulation<TAlgebra> sim;
  Environment env(sim);
  env.init_neural_network(trained_weights);
  auto obs = env.reset();
  bool done = false;
  double total_reward = 0;
  int steps = 0;
  int max_steps = 1000;

  while (!done && steps < max_steps) {
    auto action = env.policy(obs);
    double reward;
    env.step(action, obs, reward, done);
    total_reward += reward;
    steps++;
  }

  std::cout << "reward:" << total_reward << " steps:" << steps << std::endl;
}
