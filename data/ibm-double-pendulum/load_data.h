#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
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

  std::cout << "Loaded " << output.size() << " rows from " << filename;
  if (output.size() > 0) {
    std::cout << " (dt = " << output[1][0] - output[0][0] << ")";
  }
  std::cout << ".\n";

  return output;
}

// Compute joint angles from a list of bob positions.
template <typename T>
auto PendulumIk(const std::vector<std::array<T, 7>> &input) {
  std::vector<std::vector<T>> output;
  output.reserve(input.size());
  for (int i = 0; i < input.size(); ++i) {
    // Closed-form IK.
    const auto &[time, x0, y0, x1, y1, x2, y2] = input[i];
    const auto q0 = std::atan2(y1 - y0, x1 - x0);
    const auto q1 = std::atan2(y2 - y1, x2 - x1) - q0;
    output.push_back({q0 - M_PI_2, q1});
  }

  return output;
}
