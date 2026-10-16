// Generates the batched CPU and CUDA kernels of the G1 model (g1_model.hpp)
// with CppADCodeGen, see src/utils/kernel_codegen.hpp.
//
// Usage: g1_codegen [output_directory]
// writes g1_kernels.h, g1_kernels_impl.h, g1_kernels_cpu.cpp and
// g1_kernels_cuda.cu. The generated files are checked in to
// examples/kernels/g1/generated, the CMake option TDS_GENERATE_KERNELS
// regenerates them at build time instead.

// clang-format off
#include "utils/differentiation.hpp"
#include "utils/kernel_codegen.hpp"
// clang-format on

#include <iostream>
#include <string>
#include <vector>

#include "g1_model.hpp"

int main(int argc, char* argv[]) {
  const std::string directory = argc > 1 ? argv[1] : "generated";

  using Algebra = tds::EigenAlgebraT<tds::KernelCodegen::ADScalar>;
  using ADScalar = tds::KernelCodegen::ADScalar;
  G1Model<Algebra> model;
  // the generic model, for the default state and control
  G1Model<tds::EigenAlgebra> double_model;

  const int state_dim = model.state_dim();
  const int control_dim = model.control_dim();
  const int poses_dim = model.poses_dim();

  tds::KernelCodegen codegen("g1");
  codegen.add_string("urdf", G1Model<Algebra>::urdf_name);
  codegen.add_constant("dt", double_model.dt);
  codegen.add_constant("frame_dt", G1Model<Algebra>::frame_dt);
  codegen.add_constant("substeps", G1Model<Algebra>::substeps);
  codegen.add_constant("num_actuators", model.num_actuators());
  codegen.add_constant("num_visuals", model.num_visuals());
  codegen.add_constant("standing_height", G1Model<Algebra>::standing_height);
  codegen.add_constant("drop_height", G1Model<Algebra>::drop_height);
  codegen.add_constant("default_kp", G1Model<Algebra>::default_kp);
  codegen.add_constant("default_kd", G1Model<Algebra>::default_kd);

  std::vector<double> state = double_model.default_state();
  std::vector<double> control = double_model.default_control();
  std::vector<double> step_inputs = state;
  step_inputs.insert(step_inputs.end(), control.begin(), control.end());

  codegen.add_function(
      "step", {{"state", state_dim}, {"control", control_dim}},
      {{"next_state", state_dim}}, step_inputs,
      [&](const std::vector<ADScalar>& x, std::vector<ADScalar>& y) {
        model.step(&x[0], &x[state_dim], &y[0]);
      });
  codegen.add_function(
      "poses", {{"state", state_dim}}, {{"poses", poses_dim}}, state,
      [&](const std::vector<ADScalar>& x, std::vector<ADScalar>& y) {
        model.poses(&x[0], &y[0]);
      });
  codegen.write(directory);
  return 0;
}
