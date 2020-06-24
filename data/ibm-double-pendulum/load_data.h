#include <array>
#include <cmath>
#include <fstream>
#include <vector>

// Load data from IBM CSV file.
template <typename T> auto LoadIbmPendulumFile(const std::string &filename) {
  std::vector<std::array<T, 7>> output;
  std::ifstream f(filename);

  T time = 0.0;
  while (f.good()) {
    output.emplace_back();
    output.back()[0] = time;
    for (int i = 0; i < 6; ++i) {
      f >> output.back()[i + 1];
      f.get();
    }
    time += 1.0 / 400.0;
  }

  return output;
}

// Compute joint angles from a list of bob positions.
template <typename T>
auto PendulumIk(const std::vector<std::array<T, 7>> &input) {
  T mean_l1 = 0.0, mean_l2 = 0.0;
  for (int i = 0; i < input.size(); ++i) {
    // Incremental average calculation.
    const auto &[time, x0, y0, x1, y1, x2, y2] = input[i];
    const auto l1 = std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
    const auto l2 = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    mean_l1 = mean_l1 + (l1 - mean_l1) / (i + 1);
    mean_l2 = mean_l2 + (l2 - mean_l2) / (i + 1);
  }

  std::vector<std::vector<T>> output;
  output.reserve(input.size());
  for (int i = 0; i < input.size(); ++i) {
    // Closed-form IK.
    const auto &[time, x0, y0, x1, y1, x2, y2] = input[i];
    const auto q0 = std::atan2((y1 - y0) / mean_l1, (x1 - x0) / mean_l1);
    const auto q1 = std::atan2((y2 - y1) / mean_l2, (x2 - x1) / mean_l2) - q0;
    output.push_back({q0, q1});
  }

  return output;
}
