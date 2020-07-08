#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

// Load data from IBM CSV file.
template <typename T>
auto LoadIbmPendulumFile(const std::string &filename) {
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

void prevent_wraparound(double &q, double last_q, double &q_offset) {
  const double wraparound_thresh = 0.9 * M_PI;
  q += q_offset;
  double diff = q - last_q;
  if (std::fabs(diff) < wraparound_thresh) return;
  double delta = diff > 0 ? -2 * M_PI : 2 * M_PI;
  q += delta;
  q_offset += delta;
}

// Compute joint angles from a list of bob positions.
template <typename T>
auto PendulumIk(const std::vector<std::array<T, 7>> &input) {
  std::vector<std::vector<T>> output;
  output.reserve(input.size());
  double q0_offset = 0;
  double q1_offset = 0;
  double last_q0, last_q1;
  for (int i = 0; i < input.size(); ++i) {
    // Closed-form IK.
    const auto &[time, x0, y0, x1, y1, x2, y2] = input[i];

    auto q0 = std::atan2(y1 - y0, x1 - x0);
    if (i > 0) {
      prevent_wraparound(q0, last_q0, q0_offset);
    }

    auto q1 = std::atan2(y2 - y1, x2 - x1) - q0;
    if (i > 0) {
      prevent_wraparound(q1, last_q1, q1_offset);
    }

    last_q0 = q0;
    last_q1 = q1;
    if (q1 > M_PI) q1 -= 2 * M_PI;
    output.push_back({q0 - M_PI_2, q1});
  }

  return output;
}
