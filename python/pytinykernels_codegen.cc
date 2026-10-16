// The world kernel generator of pytinykernels (pytinykernels.cc): traces a
// world description (src/bake/world_spec.hpp) with CppADCodeGen into the
// sources of batched kernels, see src/bake/world_codegen.hpp. In a separate
// file, because it takes long to compile.

#include "bake/world_codegen.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

namespace py = pybind11;

void bind_kernel_codegen(py::module& m) {
  m.def(
      "generate_world",
      [](const std::string& spec_json, const std::string& directory,
         int max_segment_kb) {
        py::gil_scoped_release release;
        const tds::WorldSpec spec = tds::WorldSpec::from_json(spec_json);
        tds::KernelCodegen::Options options;
        options.max_segment_kb = max_segment_kb;
        return tds::generate_world_kernels(spec, directory, options).dump();
      },
      py::arg("spec"), py::arg("directory"), py::arg("max_segment_kb") = 256,
      "Traces the world described by the JSON spec and writes the sources of "
      "its kernels (kernels_cpu.cpp, kernels_cuda.cu, kernels.json) to the "
      "directory. Returns the layout of the world as JSON.");
  m.def(
      "world_layout",
      [](const std::string& spec_json) {
        py::gil_scoped_release release;
        const tds::WorldSpec spec = tds::WorldSpec::from_json(spec_json);
        tds::WorldModel<tds::EigenAlgebra> model(spec);
        return tds::world_layout(model).dump();
      },
      py::arg("spec"),
      "The layout of the world described by the JSON spec (bodies, joints, "
      "links, contacts and array sizes), without generating kernels.");
}
