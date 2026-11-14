// Python bindings of the batched kernels generated with CppADCodeGen
// (src/utils/kernel_codegen.hpp), and of tds::KernelBatch that runs them on
// the CPU or with CUDA:
//  - kernels that are compiled at run time and loaded with load_model
//    (src/utils/kernel_loader.hpp), the CUDA kernels compiled with NVRTC
//    (nvrtc_compile), as the tds_bake package (python/tds_bake) does
//  - with TDS_KERNELS_CODEGEN, generate_world traces a world description
//    into such kernels (pytinykernels_codegen.cc)
//  - with TDS_KERNELS_BUILTIN_MODELS, the kernels compiled by CMake
//    (examples/kernels: "g1" and "laikago")
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

#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "utils/kernel_batch.hpp"
#include "utils/kernel_cuda_driver.hpp"
#include "utils/kernel_gl_interop.hpp"
#include "utils/kernel_loader.hpp"
#include "utils/kernel_nvrtc.hpp"

#ifdef TDS_KERNELS_BUILTIN_MODELS
#include "g1_kernels.h"
#include "laikago_kernels.h"
#endif

namespace py = pybind11;

#ifdef TDS_KERNELS_CODEGEN
// pytinykernels_codegen.cc
void bind_kernel_codegen(py::module& m);
#endif

namespace {

using ModelPtr = std::shared_ptr<const tds::KernelModel>;

std::vector<std::string> builtin_model_names() {
#ifdef TDS_KERNELS_BUILTIN_MODELS
  return {"g1", "laikago"};
#else
  return {};
#endif
}

ModelPtr builtin_model(const std::string& name) {
  // the builtin models are static, the pointers do not own them
  auto no_delete = [](const tds::KernelModel*) {};
#ifdef TDS_KERNELS_BUILTIN_MODELS
  if (name == "g1") return ModelPtr(&g1_kernel_model(), no_delete);
  if (name == "laikago") return ModelPtr(&laikago_kernel_model(), no_delete);
#endif
  (void)no_delete;
  throw std::runtime_error("Unknown kernel model \"" + name + "\".");
}

ModelPtr to_model(py::object model) {
  if (py::isinstance<py::str>(model)) {
    return builtin_model(model.cast<std::string>());
  }
  return model.cast<ModelPtr>();
}

// a tds::KernelBatch that keeps its model alive
template <typename Float>
struct Batch {
  ModelPtr model;
  std::unique_ptr<tds::KernelBatch<Float>> batch;

  Batch(ModelPtr m, int num_worlds, const std::string& backend,
        int num_threads)
      : model(std::move(m)),
        batch(new tds::KernelBatch<Float>(
            *model, num_worlds, tds::kernel_backend_from_string(backend),
            num_threads)) {}
};

// The launches and copies of a batch at the events of another engine, run
// from native code without Python: the engine calls run(context, event,
// substep) - a plain C function - at each of its events, which enqueues the
// event's launches and copies on the batch's stream (Batch.program). An
// engine that records its stream into a CUDA graph records them with it.
template <typename Float>
struct Program {
  enum Kind { LAUNCH, COPY, COPY_ELEMENTS };
  struct Operation {
    Kind kind;
    std::string a;  // the function, or the destination array
    std::string b;  // the source array
    int dst_begin{0}, src_begin{0}, count{0};
  };
  Batch<Float>* batch{nullptr};
  py::object owner;  // keeps the batch alive
  std::map<int, std::vector<Operation>> events;
  std::string error;  // the first error, if any

  static void run(void* context, int event, int /*substep*/) {
    auto* program = static_cast<Program*>(context);
    auto it = program->events.find(event);
    if (it == program->events.end()) return;
    // (no exceptions into the caller's C code)
    try {
      tds::KernelBatch<Float>& b = *program->batch->batch;
      for (const Operation& op : it->second) {
        if (op.kind == LAUNCH) {
          b.launch(op.a);
        } else if (op.kind == COPY) {
          b.copy(op.a, op.b);
        } else {
          b.copy_elements(op.a, op.dst_begin, op.b, op.src_begin, op.count);
        }
      }
    } catch (const std::exception& e) {
      if (program->error.empty()) program->error = e.what();
      fprintf(stderr, "pytinykernels: program event %d: %s\n", event,
              e.what());
    }
  }
};

template <typename Float>
void bind_program(py::module& m, const std::string& class_name) {
  using P = Program<Float>;
  py::class_<P>(m, class_name.c_str())
      // the address of the C function
      //   void run(void* context, int event, int substep)
      // and of the context to call it with
      .def_property_readonly(
          "function",
          [](const P&) { return reinterpret_cast<uintptr_t>(&P::run); })
      .def_property_readonly(
          "context", [](P& p) { return reinterpret_cast<uintptr_t>(&p); })
      .def_property_readonly("error", [](const P& p) { return p.error; })
      .def_property_readonly("events",
                             [](const P& p) {
                               std::vector<int> events;
                               for (const auto& e : p.events) {
                                 events.push_back(e.first);
                               }
                               return events;
                             })
      .def("run", [](P& p, int event, int substep) {
             py::gil_scoped_release release;
             P::run(&p, event, substep);
           },
           py::arg("event"), py::arg("substep") = 0);
}

template <typename Float>
void bind_batch(py::module& m, const char* class_name) {
  using B = Batch<Float>;
  bind_program<Float>(m, std::string(class_name) + "Program");
  py::class_<B>(m, class_name)
      .def(py::init([](py::object model, int num_worlds,
                       const std::string& backend, int num_threads) {
             return new B(to_model(model), num_worlds, backend, num_threads);
           }),
           py::arg("model"), py::arg("num_worlds"),
           py::arg("backend") = "openmp", py::arg("num_threads") = 0)
      .def_property_readonly("model", [](const B& b) { return b.model; })
      .def_property_readonly("num_worlds",
                             [](const B& b) { return b.batch->num_worlds(); })
      .def_property_readonly(
          "backend",
          [](const B& b) { return tds::to_string(b.batch->backend()); })
      .def_property_readonly(
          "precision",
          [](const B&) {
            return std::is_same_v<Float, float> ? "float" : "double";
          })
      .def_property_readonly("device_name",
                             [](const B& b) { return b.batch->device_name(); })
      .def_property(
          "cuda_block_size", [](B& b) { return b.batch->cuda_block_size(); },
          [](B& b, int size) { b.batch->cuda_block_size() = size; })
      // the CUDA stream of the launches and copies, as an integer (a
      // CUstream or cudaStream_t), 0 for the legacy default stream
      .def_property(
          "stream",
          [](B& b) { return reinterpret_cast<uintptr_t>(b.batch->stream()); },
          [](B& b, uintptr_t stream) {
            b.batch->set_stream(reinterpret_cast<void*>(stream));
          })
      .def("array_names", [](const B& b) { return b.batch->array_names(); })
      .def("has_array",
           [](const B& b, const std::string& name) {
             return b.batch->has_array(name);
           },
           py::arg("name"))
      .def("dim", [](const B& b, const std::string& name) {
             return b.batch->dim(name);
           },
           py::arg("name"))
      .def("add_array",
           [](B& b, const std::string& name, int dim) {
             b.batch->add_array(name, dim);
           },
           py::arg("name"), py::arg("dim"))
      // numpy view of the host copy of the array, of shape (dim, num_worlds).
      // swap() exchanges the memory of two arrays, so get the views again
      // after a swap.
      .def(
          "array",
          [](py::object self, const std::string& name) {
            B& b = self.cast<B&>();
            const py::ssize_t dim = b.batch->dim(name);
            const py::ssize_t n = b.batch->num_worlds();
            return py::array_t<Float>(
                {dim, n},
                {static_cast<py::ssize_t>(n * sizeof(Float)),
                 static_cast<py::ssize_t>(sizeof(Float))},
                b.batch->host(name), self);
          },
          py::arg("name"))
      // address of the array that the kernels use, device memory for the
      // CUDA backend, with the layout of array()
      .def(
          "device_pointer",
          [](B& b, const std::string& name) {
            return reinterpret_cast<uintptr_t>(b.batch->device(name));
          },
          py::arg("name"))
      .def(
          "fill",
          [](B& b, const std::string& name,
             const std::vector<double>& values) { b.batch->fill(name, values); },
          py::arg("name"), py::arg("values"))
      .def("upload",
           [](B& b, const std::string& name) { b.batch->upload(name); },
           py::arg("name"))
      .def("download",
           [](B& b, const std::string& name) {
             py::gil_scoped_release release;
             b.batch->download(name);
           },
           py::arg("name"))
      .def("launch",
           [](B& b, const std::string& function,
              const std::vector<std::string>& inputs,
              const std::vector<std::string>& outputs, int iterations) {
             py::gil_scoped_release release;
             b.batch->launch(function, inputs, outputs, iterations);
           },
           py::arg("function"), py::arg("inputs") = std::vector<std::string>(),
           py::arg("outputs") = std::vector<std::string>(),
           py::arg("iterations") = -1)
      .def("swap",
           [](B& b, const std::string& x, const std::string& y) {
             b.batch->swap(x, y);
           },
           py::arg("a"), py::arg("b"))
      .def("copy",
           [](B& b, const std::string& dst, const std::string& src) {
             b.batch->copy(dst, src);
           },
           py::arg("dst"), py::arg("src"))
      .def("copy_elements",
           [](B& b, const std::string& dst, int dst_begin,
              const std::string& src, int src_begin, int count) {
             b.batch->copy_elements(dst, dst_begin, src, src_begin, count);
           },
           py::arg("dst"), py::arg("dst_begin"), py::arg("src"),
           py::arg("src_begin"), py::arg("count"))
      .def("synchronize", [](B& b) {
        py::gil_scoped_release release;
        b.batch->synchronize();
      })
      // A Program (see above) of the operations of each event:
      // {event: [("launch", function), ("copy", dst, src),
      //          ("copy_elements", dst, dst_begin, src, src_begin, count)]}
      .def(
          "program",
          [](py::object self, const std::map<int, py::list>& events) {
            B& b = self.cast<B&>();
            auto program = std::make_unique<Program<Float>>();
            program->batch = &b;
            program->owner = self;
            using Op = typename Program<Float>::Operation;
            for (const auto& [event, operations] : events) {
              auto& ops = program->events[event];
              for (const auto& item : operations) {
                const py::tuple t = py::cast<py::tuple>(item);
                const std::string kind = t[0].cast<std::string>();
                Op op;
                if (kind == "launch" && t.size() == 2) {
                  op.kind = Program<Float>::LAUNCH;
                  op.a = t[1].cast<std::string>();
                  b.model->function(op.a);  // (throws for unknown ones)
                } else if (kind == "copy" && t.size() == 3) {
                  op.kind = Program<Float>::COPY;
                  op.a = t[1].cast<std::string>();
                  op.b = t[2].cast<std::string>();
                  if (b.batch->dim(op.a) != b.batch->dim(op.b)) {
                    throw std::runtime_error("program: \"" + op.a +
                                             "\" and \"" + op.b +
                                             "\" have different sizes");
                  }
                } else if (kind == "copy_elements" && t.size() == 6) {
                  op.kind = Program<Float>::COPY_ELEMENTS;
                  op.a = t[1].cast<std::string>();
                  op.dst_begin = t[2].cast<int>();
                  op.b = t[3].cast<std::string>();
                  op.src_begin = t[4].cast<int>();
                  op.count = t[5].cast<int>();
                  if (op.count < 0 || op.dst_begin < 0 || op.src_begin < 0 ||
                      op.dst_begin + op.count > b.batch->dim(op.a) ||
                      op.src_begin + op.count > b.batch->dim(op.b)) {
                    throw std::runtime_error("program: copy_elements out of "
                                             "range");
                  }
                } else {
                  throw std::runtime_error(
                      "program: an operation is (\"launch\", function), "
                      "(\"copy\", dst, src) or (\"copy_elements\", dst, "
                      "dst_begin, src, src_begin, count)");
                }
                ops.push_back(op);
              }
            }
            return program.release();
          },
          py::arg("events"), py::return_value_policy::take_ownership);
}

py::dict cuda_device_info(int device) {
  const auto& driver = tds::CudaDriver::get(device);
  py::dict info;
  info["name"] = driver.device_name();
  info["compute_capability"] = driver.compute_capability();
  info["driver_version"] = driver.driver_version();
  info["ordinal"] = driver.device_ordinal();
  return info;
}

py::dict nvrtc_info(const std::string& library_path) {
  const auto& nvrtc = tds::Nvrtc::get(library_path);
  py::dict info;
  info["library"] = nvrtc.library_path();
  info["version"] = nvrtc.version();
  info["archs"] = nvrtc.supported_archs();
  return info;
}

}  // namespace

PYBIND11_MODULE(pytinykernels, m) {
  m.doc() =
      "Batched CPU and CUDA kernels generated from Tiny Differentiable "
      "Simulator models";
  // changes when the generated kernels or their interface change, part of
  // the cache key of the tds_bake package
  m.attr("kernel_format_version") = 2;

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
      .def_readonly("num_temporaries", &tds::KernelFunction::num_temporaries)
      .def_readonly("segments", &tds::KernelFunction::segments)
      .def_readonly("scratch_dim", &tds::KernelFunction::scratch_dim)
      .def_readonly("loop_input", &tds::KernelFunction::loop_input)
      .def_readonly("default_iterations",
                    &tds::KernelFunction::default_iterations);

  py::class_<tds::KernelModel, std::shared_ptr<tds::KernelModel>>(
      m, "KernelModel")
      .def_readonly("name", &tds::KernelModel::name)
      .def_readonly("functions", &tds::KernelModel::functions)
      .def_readonly("constants", &tds::KernelModel::constants)
      .def_readonly("strings", &tds::KernelModel::strings)
      .def_property_readonly("has_cuda", &tds::KernelModel::has_cuda)
      .def("has_kernels", &tds::KernelModel::has_kernels, py::arg("backend"),
           py::arg("precision"))
      .def("function", &tds::KernelModel::function,
           py::return_value_policy::reference_internal);

  m.def("models", &builtin_model_names,
        "names of the kernel models compiled into the module");
  m.def(
      "model",
      [](const std::string& name) {
        return std::const_pointer_cast<tds::KernelModel>(builtin_model(name));
      },
      py::arg("name"), "a kernel model compiled into the module");

  m.def(
      "load_model",
      [](const std::string& manifest,
         const std::map<std::string, std::string>& cpu_libraries,
         const std::map<std::string, std::vector<std::string>>& cuda_images,
         int device) {
        py::gil_scoped_release release;
        return tds::load_kernel_model(manifest, cpu_libraries, cuda_images,
                                      device);
      },
      py::arg("manifest"),
      py::arg("cpu_libraries") = std::map<std::string, std::string>(),
      py::arg("cuda_images") =
          std::map<std::string, std::vector<std::string>>(),
      py::arg("device") = 0,
      "Loads kernels compiled at run time: manifest is the kernels.json of "
      "the generated sources, cpu_libraries maps a precision to the shared "
      "library of the CPU kernels, cuda_images maps a precision to the "
      "CUBIN or PTX files of the CUDA kernels.");

  m.def("cuda_available",
        [](int device) {
          try {
            tds::CudaDriver::get(device);
            return true;
          } catch (const std::exception&) {
            return false;
          }
        },
        py::arg("device") = 0);
  m.def("cuda_device", &cuda_device_info, py::arg("device") = 0,
        "name, compute capability and driver version of a CUDA device");
  m.def("nvrtc", &nvrtc_info, py::arg("library") = "",
        "Loads NVRTC (from the library path if given, on first use), returns "
        "its path, version and supported architectures.");
  m.def(
      "nvrtc_compile",
      [](const std::string& source, const std::string& name,
         int compute_capability, const std::vector<std::string>& options,
         const std::string& library) {
        tds::Nvrtc::Result result;
        {
          py::gil_scoped_release release;
          result = tds::Nvrtc::get(library).compile(source, name,
                                                    compute_capability,
                                                    options);
        }
        py::dict d;
        d["image"] = py::bytes(result.image);
        d["is_ptx"] = result.is_ptx;
        d["arch"] = result.arch;
        d["log"] = result.log;
        return d;
      },
      py::arg("source"), py::arg("name"), py::arg("compute_capability"),
      py::arg("options") = std::vector<std::string>(),
      py::arg("library") = "",
      "Compiles CUDA source with NVRTC into a CUBIN (or PTX for an "
      "architecture that NVRTC does not support). Releases the GIL, so that "
      "parts can be compiled in parallel threads.");

  bind_batch<float>(m, "KernelBatchFloat");
  bind_batch<double>(m, "KernelBatchDouble");

  py::class_<tds::CudaGlInstanceWriter>(m, "CudaGlInstanceWriter",
                                        R"(Writes the poses of the visual
shapes of a CUDA batch into the instance arrays of an OpenGL vertex buffer
(CUDA-OpenGL interop), see src/utils/kernel_gl_interop.hpp. Create it with
the OpenGL context current.)")
      .def(py::init<unsigned, size_t, size_t, const std::vector<int>&,
                    const std::vector<int>&, const std::vector<int>&,
                    const std::vector<float>&, int>(),
           py::arg("vbo"), py::arg("positions_offset"),
           py::arg("orientations_offset"), py::arg("worlds"),
           py::arg("visuals"), py::arg("instances"), py::arg("offsets"),
           py::arg("device") = 0)
      .def_property_readonly("size", &tds::CudaGlInstanceWriter::size)
      .def(
          "read",
          [](tds::CudaGlInstanceWriter& w, int num_instances) {
            std::vector<float> positions, orientations;
            w.read(num_instances, positions, orientations);
            py::array_t<float> p({num_instances, 4}), o({num_instances, 4});
            std::copy(positions.begin(), positions.end(), p.mutable_data());
            std::copy(orientations.begin(), orientations.end(),
                      o.mutable_data());
            return py::make_tuple(p, o);
          },
          py::arg("num_instances"),
          "positions and orientations (num_instances, 4) of the first "
          "instances in the vertex buffer, for tests")
      .def(
          "write",
          [](tds::CudaGlInstanceWriter& w, Batch<float>& b,
             const std::string& name) {
            w.write(b.batch->device(name), b.batch->num_worlds(), false);
          },
          py::arg("batch"), py::arg("name") = "poses")
      .def(
          "write",
          [](tds::CudaGlInstanceWriter& w, Batch<double>& b,
             const std::string& name) {
            w.write(b.batch->device(name), b.batch->num_worlds(), true);
          },
          py::arg("batch"), py::arg("name") = "poses");

  m.def(
      "KernelBatch",
      [](py::object model, int num_worlds, const std::string& backend,
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
      "creates a KernelBatchFloat or KernelBatchDouble, model is a "
      "KernelModel or the name of a model compiled into the module");

#ifdef TDS_KERNELS_CODEGEN
  m.attr("has_codegen") = true;
  bind_kernel_codegen(m);
#else
  m.attr("has_codegen") = false;
#endif
}
