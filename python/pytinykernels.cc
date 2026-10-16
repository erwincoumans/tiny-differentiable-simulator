// Python bindings of the batched kernels generated with CppADCodeGen
// (src/utils/kernel_codegen.hpp, examples/kernels), and of tds::KernelBatch
// that runs them on the CPU or with CUDA.
//
//   import pytinykernels as tk
//   batch = tk.KernelBatch("g1", num_worlds=4096, backend="cuda",
//                          precision="float")
//   batch.fill("state", default_state)
//   batch.upload("state")
//   batch.launch("step")              # state, control -> next_state
//   batch.swap("state", "next_state")
//   batch.download("state")
//   state = batch.array("state").T    # numpy view (num_worlds, state_dim)

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

#include "g1_kernels.h"
#include "laikago_kernels.h"
#include "utils/kernel_batch.hpp"

namespace py = pybind11;

namespace {

const std::vector<std::string>& model_names() {
  static const std::vector<std::string> names = {"g1", "laikago"};
  return names;
}

const tds::KernelModel& get_model(const std::string& name) {
  if (name == "g1") return g1_kernel_model();
  if (name == "laikago") return laikago_kernel_model();
  throw std::runtime_error("Unknown kernel model \"" + name + "\".");
}

template <typename Float>
void bind_batch(py::module& m, const char* class_name) {
  using Batch = tds::KernelBatch<Float>;
  py::class_<Batch>(m, class_name)
      .def(py::init([](const std::string& model, int num_worlds,
                       const std::string& backend, int num_threads) {
             return new Batch(get_model(model), num_worlds,
                              tds::kernel_backend_from_string(backend),
                              num_threads);
           }),
           py::arg("model"), py::arg("num_worlds"),
           py::arg("backend") = "openmp", py::arg("num_threads") = 0)
      .def_property_readonly(
          "model", [](const Batch& b) { return &b.model(); },
          py::return_value_policy::reference)
      .def_property_readonly("num_worlds", &Batch::num_worlds)
      .def_property_readonly(
          "backend", [](const Batch& b) { return tds::to_string(b.backend()); })
      .def_property_readonly("device_name", &Batch::device_name)
      .def_property(
          "cuda_block_size", [](Batch& b) { return b.cuda_block_size(); },
          [](Batch& b, int size) { b.cuda_block_size() = size; })
      .def("array_names", &Batch::array_names)
      .def("dim", &Batch::dim, py::arg("name"))
      .def("add_array", &Batch::add_array, py::arg("name"), py::arg("dim"))
      // numpy view of the host copy of the array, of shape (dim, num_worlds).
      // swap() exchanges the memory of two arrays, so get the views again
      // after a swap.
      .def(
          "array",
          [](py::object self, const std::string& name) {
            Batch& b = self.cast<Batch&>();
            const py::ssize_t dim = b.dim(name);
            const py::ssize_t n = b.num_worlds();
            return py::array_t<Float>(
                {dim, n},
                {static_cast<py::ssize_t>(n * sizeof(Float)),
                 static_cast<py::ssize_t>(sizeof(Float))},
                b.host(name), self);
          },
          py::arg("name"))
      .def(
          "fill",
          [](Batch& b, const std::string& name,
             const std::vector<double>& values) { b.fill(name, values); },
          py::arg("name"), py::arg("values"))
      .def("upload", &Batch::upload, py::arg("name"))
      .def("download", &Batch::download, py::arg("name"))
      .def("launch", &Batch::launch, py::arg("function"),
           py::arg("inputs") = std::vector<std::string>(),
           py::arg("outputs") = std::vector<std::string>())
      .def("swap", &Batch::swap, py::arg("a"), py::arg("b"))
      .def("copy", &Batch::copy, py::arg("dst"), py::arg("src"))
      .def("copy_elements", &Batch::copy_elements, py::arg("dst"),
           py::arg("dst_begin"), py::arg("src"), py::arg("src_begin"),
           py::arg("count"))
      .def("synchronize", &Batch::synchronize);
}

}  // namespace

PYBIND11_MODULE(pytinykernels, m) {
  m.doc() =
      "Batched CPU and CUDA kernels generated from Tiny Differentiable "
      "Simulator models";

  py::class_<tds::KernelArray>(m, "KernelArray")
      .def_readonly("name", &tds::KernelArray::name)
      .def_readonly("dim", &tds::KernelArray::dim)
      .def("__repr__", [](const tds::KernelArray& a) {
        return a.name + "[" + std::to_string(a.dim) + "]";
      });

  py::class_<tds::KernelFunction>(m, "KernelFunction")
      .def_readonly("name", &tds::KernelFunction::name)
      .def_readonly("inputs", &tds::KernelFunction::inputs)
      .def_readonly("outputs", &tds::KernelFunction::outputs)
      .def_readonly("num_operations", &tds::KernelFunction::num_operations)
      .def_readonly("num_temporaries", &tds::KernelFunction::num_temporaries);

  py::class_<tds::KernelModel>(m, "KernelModel")
      .def_readonly("name", &tds::KernelModel::name)
      .def_readonly("functions", &tds::KernelModel::functions)
      .def_readonly("constants", &tds::KernelModel::constants)
      .def_readonly("strings", &tds::KernelModel::strings)
      .def_property_readonly("has_cuda", &tds::KernelModel::has_cuda)
      .def("function", &tds::KernelModel::function,
           py::return_value_policy::reference_internal);

  m.def("models", &model_names, "names of the compiled kernel models");
  m.def("model", &get_model, py::arg("name"),
        py::return_value_policy::reference);

  bind_batch<float>(m, "KernelBatchFloat");
  bind_batch<double>(m, "KernelBatchDouble");

  m.def(
      "KernelBatch",
      [](const std::string& model, int num_worlds, const std::string& backend,
         const std::string& precision, int num_threads) -> py::object {
        py::module self = py::module::import("pytinykernels");
        if (precision == "float") {
          return self.attr("KernelBatchFloat")(model, num_worlds, backend,
                                               num_threads);
        }
        if (precision == "double") {
          return self.attr("KernelBatchDouble")(model, num_worlds, backend,
                                                num_threads);
        }
        throw std::runtime_error("precision must be \"float\" or \"double\"");
      },
      py::arg("model"), py::arg("num_worlds"), py::arg("backend") = "openmp",
      py::arg("precision") = "float", py::arg("num_threads") = 0,
      "creates a KernelBatchFloat or KernelBatchDouble");
}
