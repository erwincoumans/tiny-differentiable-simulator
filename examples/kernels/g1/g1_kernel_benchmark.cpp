// Checks the generated G1 kernels against the generic TDS model
// (g1_model.hpp), and measures the simulation speed of each backend.
//
// Usage: g1_kernel_benchmark [--worlds N] [--frames F] [--backend B]
//                            [--block-size S] [--no-verify] [--no-benchmark]
//
// Verification runs one step from random states near the standing pose, and
// simulates the drop and stand of the G1 example for 2 seconds, with the
// generic model in double precision and with each kernel backend and
// precision. It fails (exit code 1) if a kernel differs too much from the
// generic model. The benchmark simulates N worlds for F frames of 6 substeps,
// with all backends or only backend B (serial, openmp or cuda), with S
// threads per CUDA block.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "g1_kernels.h"
#include "g1_model.hpp"
#include "math/eigen_algebra.hpp"
#include "utils/kernel_batch.hpp"
#include "utils/stopwatch.hpp"

using Model = G1Model<tds::EigenAlgebra>;

struct Backend {
  tds::KernelBackend backend;
  bool use_float;
  std::string name() const {
    return tds::to_string(backend) + (use_float ? " float" : " double");
  }
};

// random states near the standing pose, with some feet in contact
std::vector<std::vector<double>> random_states(const Model& model, int n) {
  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> uniform(-1., 1.);
  std::vector<std::vector<double>> states;
  const int dof = model.system->dof();
  for (int w = 0; w < n; ++w) {
    std::vector<double> state(model.state_dim(), 0.);
    // small base rotation about a random axis
    double axis[3] = {uniform(rng), uniform(rng), uniform(rng)};
    double norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] +
                            axis[2] * axis[2]);
    double half_angle = 0.05 * uniform(rng);
    for (int i = 0; i < 3; ++i) {
      state[i] = std::sin(half_angle) * axis[i] / norm;
    }
    state[3] = std::cos(half_angle);
    state[4] = 0.1 * uniform(rng);
    state[5] = 0.1 * uniform(rng);
    state[6] = Model::standing_height + 0.01 * uniform(rng);
    for (int i = 7; i < dof; ++i) {
      state[i] = 0.1 * uniform(rng);
    }
    for (int i = dof; i < model.state_dim(); ++i) {
      state[i] = 0.5 * uniform(rng);
    }
    states.push_back(state);
  }
  return states;
}

std::vector<std::vector<double>> random_controls(const Model& model, int n) {
  std::mt19937 rng(5678);
  std::uniform_real_distribution<double> uniform(0., 1.);
  const int a = model.num_actuators();
  std::vector<std::vector<double>> controls;
  for (int w = 0; w < n; ++w) {
    std::vector<double> control(model.control_dim());
    for (int i = 0; i < a; ++i) {
      control[i] = 0.2 * (2. * uniform(rng) - 1.);
      control[a + i] = 500. + 1000. * uniform(rng);
      control[2 * a + i] = 5. + 10. * uniform(rng);
    }
    controls.push_back(control);
  }
  return controls;
}

template <typename Float>
void run_kernel_step(const Backend& backend, int num_worlds,
                     const std::vector<std::vector<double>>& states,
                     const std::vector<std::vector<double>>& controls,
                     std::vector<std::vector<double>>& next_states) {
  tds::KernelBatch<Float> batch(g1_kernel_model(), num_worlds,
                                backend.backend);
  for (int w = 0; w < num_worlds; ++w) {
    batch.set_world("state", w, states[w].data());
    batch.set_world("control", w, controls[w].data());
  }
  batch.upload("state");
  batch.upload("control");
  batch.launch("step");
  batch.download("next_state");
  next_states.assign(num_worlds,
                     std::vector<double>(g1_kernels::next_state_dim));
  for (int w = 0; w < num_worlds; ++w) {
    batch.get_world("next_state", w, next_states[w].data());
  }
}

// simulates the G1 example for the given number of frames, returns the
// final state
template <typename Float>
std::vector<double> run_kernel_frames(const Backend& backend,
                                      const Model& model, int frames) {
  tds::KernelBatch<Float> batch(g1_kernel_model(), 1, backend.backend);
  batch.fill("state", model.default_state());
  batch.fill("control", model.default_control());
  batch.upload("state");
  batch.upload("control");
  for (int f = 0; f < frames; ++f) {
    for (int s = 0; s < g1_kernels::substeps; ++s) {
      batch.launch("step");
      batch.swap("state", "next_state");
    }
  }
  batch.download("state");
  std::vector<double> state(g1_kernels::state_dim);
  batch.get_world("state", 0, state.data());
  return state;
}

double pitch(const std::vector<double>& state) {
  // rotation about y of the base quaternion (x, y, z, w)
  double x = state[0], y = state[1], z = state[2], w = state[3];
  return std::asin(std::clamp(2. * (w * y - z * x), -1., 1.));
}

template <typename Float>
double benchmark(const Backend& backend, const Model& model, int num_worlds,
                 int frames, int block_size) {
  tds::KernelBatch<Float> batch(g1_kernel_model(), num_worlds,
                                backend.backend);
  batch.cuda_block_size() = block_size;
  batch.fill("state", model.default_state());
  batch.fill("control", model.default_control());
  batch.upload("state");
  batch.upload("control");
  auto frame = [&]() {
    for (int s = 0; s < g1_kernels::substeps; ++s) {
      batch.launch("step");
      batch.swap("state", "next_state");
    }
    batch.launch("poses");
    batch.download("poses");
  };
  // warm up
  frame();
  tds::Stopwatch timer;
  timer.start();
  for (int f = 0; f < frames; ++f) {
    frame();
  }
  return timer.stop() / frames;
}

int main(int argc, char* argv[]) {
  int num_worlds = 4096;
  int frames = 20;
  int block_size = 64;
  bool verify = true;
  bool run_benchmark = true;
  std::string only_backend;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--worlds" && i + 1 < argc) {
      num_worlds = std::max(1, atoi(argv[++i]));
    } else if (arg == "--frames" && i + 1 < argc) {
      frames = std::max(1, atoi(argv[++i]));
    } else if (arg == "--block-size" && i + 1 < argc) {
      block_size = std::max(1, atoi(argv[++i]));
    } else if (arg == "--backend" && i + 1 < argc) {
      only_backend = argv[++i];
    } else if (arg == "--no-verify") {
      verify = false;
    } else if (arg == "--no-benchmark") {
      run_benchmark = false;
    }
  }

  Model model;
  const tds::KernelModel& kernels = g1_kernel_model();
  std::vector<Backend> backends;
  for (auto b : {tds::KernelBackend::SERIAL, tds::KernelBackend::OPENMP,
                 tds::KernelBackend::CUDA}) {
    if (b == tds::KernelBackend::CUDA && !kernels.has_cuda()) continue;
    if (!only_backend.empty() &&
        b != tds::kernel_backend_from_string(only_backend)) {
      continue;
    }
    backends.push_back({b, false});
    backends.push_back({b, true});
  }
  for (const auto& f : kernels.functions) {
    printf("kernel %s: %d operations, %d temporaries\n", f.name.c_str(),
           f.num_operations, f.num_temporaries);
  }

  bool passed = true;
  if (verify) {
    // one step from random states
    const int n = 256;
    auto states = random_states(model, n);
    auto controls = random_controls(model, n);
    std::vector<std::vector<double>> expected(
        n, std::vector<double>(model.state_dim()));
    for (int w = 0; w < n; ++w) {
      model.step(states[w].data(), controls[w].data(), expected[w].data());
    }
    printf("\none step from %d random states, max |error| of q and qd:\n", n);
    for (const auto& backend : backends) {
      std::vector<std::vector<double>> next_states;
      if (backend.use_float) {
        run_kernel_step<float>(backend, n, states, controls, next_states);
      } else {
        run_kernel_step<double>(backend, n, states, controls, next_states);
      }
      double q_error = 0, qd_error = 0;
      const int dof = model.system->dof();
      for (int w = 0; w < n; ++w) {
        for (int i = 0; i < model.state_dim(); ++i) {
          double& error = i < dof ? q_error : qd_error;
          error = std::max(error, std::abs(next_states[w][i] - expected[w][i]));
        }
      }
      // the float kernels differ by rounding, mostly in the contact solver
      const double tolerance = backend.use_float ? 1e-2 : 1e-9;
      const bool ok = q_error < tolerance && qd_error < tolerance;
      printf("  %-16s q %.3g  qd %.3g  %s\n", backend.name().c_str(),
             q_error, qd_error, ok ? "ok" : "FAILED");
      passed = passed && ok;
    }

    // the drop and stand of the G1 example
    const int verify_frames = 120;
    std::vector<double> state = model.default_state();
    std::vector<double> control = model.default_control();
    std::vector<double> next_state(model.state_dim());
    for (int f = 0; f < verify_frames; ++f) {
      for (int s = 0; s < Model::substeps; ++s) {
        model.step(state.data(), control.data(), next_state.data());
        std::swap(state, next_state);
      }
    }
    printf("\nG1 example after %d frames: pelvis x, z, pitch\n",
           verify_frames);
    printf("  %-16s %.5f %.5f %.5f\n", "generic double", state[4], state[6],
           pitch(state));
    for (const auto& backend : backends) {
      auto s = backend.use_float
                   ? run_kernel_frames<float>(backend, model, verify_frames)
                   : run_kernel_frames<double>(backend, model, verify_frames);
      const bool ok = std::abs(s[4] - state[4]) < 1e-3 &&
                      std::abs(s[6] - state[6]) < 1e-3 &&
                      std::abs(pitch(s) - pitch(state)) < 1e-3;
      printf("  %-16s %.5f %.5f %.5f  %s\n", backend.name().c_str(), s[4],
             s[6], pitch(s), ok ? "ok" : "FAILED");
      passed = passed && ok;
    }
  }
  if (!run_benchmark) {
    return passed ? 0 : 1;
  }

  // generic model: time a few worlds serially
  {
    const int n = 4;
    std::vector<double> state = model.default_state();
    std::vector<double> control = model.default_control();
    std::vector<double> next_state(model.state_dim());
    tds::Stopwatch timer;
    timer.start();
    for (int w = 0; w < n; ++w) {
      for (int s = 0; s < Model::substeps; ++s) {
        model.step(state.data(), control.data(), next_state.data());
        std::swap(state, next_state);
      }
    }
    double per_world = timer.stop() / n;
    printf("\n%d worlds, %d frames of %d substeps\n", num_worlds, frames,
           Model::substeps);
    printf("  %-16s %10.3f ms/frame  %10.0f world-frames/s  (extrapolated "
           "from %d worlds)\n",
           "generic double", per_world * num_worlds * 1e3, 1. / per_world, n);
  }
  for (const auto& backend : backends) {
    double t = backend.use_float
                   ? benchmark<float>(backend, model, num_worlds, frames,
                                      block_size)
                   : benchmark<double>(backend, model, num_worlds, frames,
                                       block_size);
    printf("  %-16s %10.3f ms/frame  %10.0f world-frames/s\n",
           backend.name().c_str(), t * 1e3, num_worlds / t);
  }
  return passed ? 0 : 1;
}
