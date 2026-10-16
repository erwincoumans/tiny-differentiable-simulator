// Generates the batched CPU and CUDA kernels of the laikago environment
// (examples/environments/laikago_environment2.h) with CppADCodeGen, see
// src/utils/kernel_codegen.hpp. They replace the hand-converted
// omp_model_laikago_forward_zero.h of the earlier CUDA codegen.
//
// Usage: laikago_codegen [output_directory]
// The kernel function "step" maps the input of
// LaikagoContactSimulation::step_forward_original (q, qd, action, kp, kd,
// max_force) to its output (q, qd, visual world transforms, up_dot_world_z).
// The generated files are checked in to examples/kernels/laikago/generated.

// clang-format off
#include "utils/differentiation.hpp"
#include "utils/kernel_codegen.hpp"
// clang-format on

#include <string>
#include <vector>

// trace the environment without its generated kernel
#define LAIKAGO_KERNEL_CODEGEN
#include "environments/laikago_environment2.h"

int main(int argc, char* argv[]) {
  const std::string directory = argc > 1 ? argv[1] : "generated";

  using Algebra = tds::EigenAlgebraT<tds::KernelCodegen::ADScalar>;
  using ADScalar = tds::KernelCodegen::ADScalar;
  LaikagoEnv<Algebra> env(false);
  auto& sim = env.contact_sim;

  const int input_dim = sim.input_dim_with_action_and_variables();
  const int output_dim = sim.output_dim();
  std::vector<double> input(input_dim, 0.);
  // base height
  input[2] = 0.48;

  tds::KernelCodegen codegen("laikago");
  codegen.add_string("urdf", "laikago/laikago_toes_zup_xyz_xyzrot.urdf");
  // time step of LaikagoContactSimulation
  codegen.add_constant("dt", 1e-3);
  // input = state (q, qd), action, kp, kd, max_force
  // output = state, world transform of each visual, up_dot_world_z
  codegen.add_constant("state_dim", sim.input_dim());
  codegen.add_constant("action_dim", sim.action_dim());
  codegen.add_constant("num_visuals", sim.num_visual_links_);
  // the defaults of LaikagoEnv::reset
  codegen.add_constant("default_kp", 100.);
  codegen.add_constant("default_kd", 2.);
  codegen.add_constant("default_max_force", 50.);
  codegen.add_function(
      "step", {{"input", input_dim}}, {{"output", output_dim}}, input,
      [&](const std::vector<ADScalar>& x, std::vector<ADScalar>& y) {
        sim.step_forward_original(x, y);
      });
  codegen.write(directory);
  return 0;
}
