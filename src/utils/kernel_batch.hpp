#pragma once

// Runs the generated kernels of a tds::KernelModel (kernel_model.hpp) for a
// batch of worlds, on the CPU (serially or with OpenMP) or with CUDA.
//
// The batch owns named arrays, one for each input and output array of the
// model functions, in structure-of-arrays layout: element k of world i is at
// host(name)[k * num_worlds() + i]. With the CUDA backend, the arrays are kept
// in device memory, and only copied with upload() and download(), so that a
// simulation can step many times without transfers. Example, a frame of the
// G1 (examples/kernels/g1) with 6 substeps:
//
//   tds::KernelBatch<float> batch(g1_kernel_model(), 4096,
//                                 tds::KernelBackend::CUDA);
//   ... fill batch.host("state") and batch.host("control")
//   batch.upload("state");
//   batch.upload("control");
//   for (int s = 0; s < 6; ++s) {
//     batch.launch("step");  // state, control -> next_state
//     batch.swap("state", "next_state");
//   }
//   batch.launch("poses");  // state -> poses
//   batch.download("poses");

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "kernel_model.hpp"

namespace tds {

enum class KernelBackend { SERIAL, OPENMP, CUDA };

inline std::string to_string(KernelBackend backend) {
  switch (backend) {
    case KernelBackend::SERIAL:
      return "serial";
    case KernelBackend::OPENMP:
      return "openmp";
    case KernelBackend::CUDA:
      return "cuda";
  }
  return "unknown";
}

inline KernelBackend kernel_backend_from_string(const std::string& name) {
  if (name == "serial") return KernelBackend::SERIAL;
  if (name == "openmp" || name == "omp") return KernelBackend::OPENMP;
  if (name == "cuda") return KernelBackend::CUDA;
  throw std::runtime_error("Unknown kernel backend \"" + name +
                           "\", use serial, openmp or cuda.");
}

template <typename Float>
class KernelBatch {
 public:
  // num_threads is the number of OpenMP threads, 0 uses all
  KernelBatch(const KernelModel& model, int num_worlds, KernelBackend backend,
              int num_threads = 0)
      : model_(model),
        num_worlds_(num_worlds),
        backend_(backend),
        num_threads_(backend == KernelBackend::SERIAL ? 1 : num_threads) {
    if (num_worlds <= 0) {
      throw std::runtime_error("KernelBatch: num_worlds must be positive.");
    }
    const std::string backend_name =
        backend == KernelBackend::CUDA ? "cuda" : "cpu";
    const std::string precision =
        std::is_same_v<Float, float> ? "float" : "double";
    if (!model.has_kernels(backend_name, precision)) {
      throw std::runtime_error("The " + backend_name + " " + precision +
                               " kernels of model \"" + model.name +
                               "\" are not compiled.");
    }
    for (const auto& f : model.functions) {
      for (const auto& array : f.inputs) add_array(array.name, array.dim);
      for (const auto& array : f.outputs) add_array(array.name, array.dim);
      if (f.scratch_dim > 0) add_array(scratch_name(f), f.scratch_dim);
    }
  }

  ~KernelBatch() {
    if (backend_ == KernelBackend::CUDA) {
      for (auto& [name, array] : arrays_) {
        model_.cuda->deallocate(array.device);
      }
    }
  }

  KernelBatch(const KernelBatch&) = delete;
  KernelBatch& operator=(const KernelBatch&) = delete;

  const KernelModel& model() const { return model_; }
  int num_worlds() const { return num_worlds_; }
  KernelBackend backend() const { return backend_; }
  // number of threads per block of the CUDA kernels
  int& cuda_block_size() { return cuda_block_size_; }

  // Creates a zero-initialized array, for example to keep another state.
  // Does nothing if the array exists with the same size.
  void add_array(const std::string& name, int dim) {
    auto it = arrays_.find(name);
    if (it != arrays_.end()) {
      if (it->second.dim != dim) {
        throw std::runtime_error("KernelBatch: array \"" + name +
                                 "\" exists with another size.");
      }
      return;
    }
    Array& array = arrays_[name];
    array.dim = dim;
    array.host.assign(static_cast<size_t>(dim) * num_worlds_, Float(0));
    if (backend_ == KernelBackend::CUDA) {
      check(model_.cuda->allocate(reinterpret_cast<void**>(&array.device),
                                  bytes(array)));
      upload(name);
    }
  }

  bool has_array(const std::string& name) const {
    return arrays_.find(name) != arrays_.end();
  }
  std::vector<std::string> array_names() const {
    std::vector<std::string> names;
    for (const auto& [name, array] : arrays_) names.push_back(name);
    return names;
  }
  int dim(const std::string& name) const { return array(name).dim; }

  // the host copy of the array
  Float* host(const std::string& name) { return array(name).host.data(); }
  const Float* host(const std::string& name) const {
    return array(name).host.data();
  }
  // the array that the kernels use, device memory for the CUDA backend
  Float* device(const std::string& name) {
    Array& a = array(name);
    return backend_ == KernelBackend::CUDA ? a.device : a.host.data();
  }

  // copies the host array to the device, does nothing on the CPU
  void upload(const std::string& name) {
    if (backend_ != KernelBackend::CUDA) return;
    Array& a = array(name);
    check(model_.cuda->copy_to_device(a.device, a.host.data(), bytes(a),
                                      nullptr));
  }
  // copies the device array to the host and waits until it is copied, does
  // nothing on the CPU
  void download(const std::string& name) {
    if (backend_ != KernelBackend::CUDA) return;
    Array& a = array(name);
    check(model_.cuda->copy_to_host(a.host.data(), a.device, bytes(a),
                                    nullptr));
    synchronize();
  }

  // sets the host array of all worlds to the same values (of size dim)
  template <typename T>
  void fill(const std::string& name, const std::vector<T>& values) {
    Array& a = array(name);
    if (static_cast<int>(values.size()) != a.dim) {
      throw std::runtime_error("KernelBatch::fill: wrong size for array \"" +
                               name + "\".");
    }
    for (int k = 0; k < a.dim; ++k) {
      std::fill_n(a.host.begin() + static_cast<size_t>(k) * num_worlds_,
                  num_worlds_, Float(values[k]));
    }
  }
  // sets / gets the host array of one world
  template <typename T>
  void set_world(const std::string& name, int world, const T* values) {
    Array& a = array(name);
    for (int k = 0; k < a.dim; ++k) {
      a.host[static_cast<size_t>(k) * num_worlds_ + world] = Float(values[k]);
    }
  }
  template <typename T>
  void get_world(const std::string& name, int world, T* values) const {
    const Array& a = array(name);
    for (int k = 0; k < a.dim; ++k) {
      values[k] = T(a.host[static_cast<size_t>(k) * num_worlds_ + world]);
    }
  }

  // Runs the function for all worlds. The arrays default to the ones named
  // like the function's inputs and outputs. iterations is the number of
  // iterations of a loop function, -1 uses its default. With the CUDA backend
  // the launch is asynchronous, download() and synchronize() wait for it.
  void launch(const std::string& function_name,
              const std::vector<std::string>& input_names = {},
              const std::vector<std::string>& output_names = {},
              int iterations = -1) {
    const KernelFunction& f = model_.function(function_name);
    std::vector<const Float*> inputs;
    std::vector<Float*> outputs;
    bind(f, f.inputs, input_names, inputs);
    bind(f, f.outputs, output_names, outputs);
    if (iterations < 0) iterations = f.default_iterations;
    if (f.scratch_dim > 0) outputs.push_back(device(scratch_name(f)));
    if (backend_ == KernelBackend::CUDA) {
      const auto& launcher = f.template cuda<Float>();
      launcher(num_worlds_, num_worlds_, inputs.data(), outputs.data(),
               cuda_block_size_, nullptr, iterations);
      check(model_.cuda->last_error());
    } else {
      const auto& launcher = f.template cpu<Float>();
      launcher(num_worlds_, num_worlds_, inputs.data(), outputs.data(),
               num_threads_, iterations);
    }
  }

  // swaps the contents of two arrays of the same size, without copying
  void swap(const std::string& a, const std::string& b) {
    Array& array_a = array(a);
    Array& array_b = array(b);
    if (array_a.dim != array_b.dim) {
      throw std::runtime_error("KernelBatch::swap: arrays \"" + a + "\" and \"" +
                               b + "\" have different sizes.");
    }
    std::swap(array_a.host, array_b.host);
    std::swap(array_a.device, array_b.device);
  }

  // Copies elements [src_begin, src_begin + count) of array src to elements
  // [dst_begin, dst_begin + count) of array dst, for all worlds, in the arrays
  // that the kernels use (device memory for the CUDA backend). For example
  // to copy the state part of an output to an input.
  void copy_elements(const std::string& dst, int dst_begin,
                     const std::string& src, int src_begin, int count) {
    Array& d = array(dst);
    Array& s = array(src);
    if (count < 0 || dst_begin < 0 || src_begin < 0 ||
        dst_begin + count > d.dim || src_begin + count > s.dim) {
      throw std::runtime_error("KernelBatch::copy_elements: out of range.");
    }
    // the elements of all worlds are contiguous in structure-of-arrays layout
    const size_t offset_d = static_cast<size_t>(dst_begin) * num_worlds_;
    const size_t offset_s = static_cast<size_t>(src_begin) * num_worlds_;
    const size_t n = static_cast<size_t>(count) * num_worlds_;
    if (backend_ == KernelBackend::CUDA) {
      check(model_.cuda->copy_on_device(d.device + offset_d,
                                        s.device + offset_s,
                                        n * sizeof(Float), nullptr));
    } else {
      std::copy_n(s.host.begin() + offset_s, n, d.host.begin() + offset_d);
    }
  }

  // copies array src to dst of the same size, see copy_elements
  void copy(const std::string& dst, const std::string& src) {
    if (dim(dst) != dim(src)) {
      throw std::runtime_error("KernelBatch::copy: arrays \"" + dst +
                               "\" and \"" + src + "\" have different sizes.");
    }
    copy_elements(dst, 0, src, 0, dim(src));
  }

  // waits until the CUDA kernels and copies are done
  void synchronize() {
    if (backend_ == KernelBackend::CUDA) {
      check(model_.cuda->synchronize(nullptr));
    }
  }

  std::string device_name() const {
    if (backend_ == KernelBackend::CUDA) return model_.cuda->device_name();
    return "CPU";
  }

 private:
  struct Array {
    int dim{0};
    std::vector<Float> host;
    Float* device{nullptr};
  };

  const KernelModel& model_;
  int num_worlds_;
  KernelBackend backend_;
  int num_threads_;
  int cuda_block_size_{32};
  std::map<std::string, Array> arrays_;

  Array& array(const std::string& name) {
    auto it = arrays_.find(name);
    if (it == arrays_.end()) {
      throw std::runtime_error("KernelBatch: no array \"" + name + "\".");
    }
    return it->second;
  }
  const Array& array(const std::string& name) const {
    auto it = arrays_.find(name);
    if (it == arrays_.end()) {
      throw std::runtime_error("KernelBatch: no array \"" + name + "\".");
    }
    return it->second;
  }

  size_t bytes(const Array& a) const { return a.host.size() * sizeof(Float); }

  // the array of the temporaries of a function between the segments of its
  // kernels, and of the iterations of a loop function
  static std::string scratch_name(const KernelFunction& f) {
    return f.name + ".scratch";
  }

  template <typename Pointer>
  void bind(const KernelFunction& f, const std::vector<KernelArray>& params,
            const std::vector<std::string>& names,
            std::vector<Pointer>& pointers) {
    if (!names.empty() && names.size() != params.size()) {
      throw std::runtime_error("KernelBatch::launch: function \"" + f.name +
                               "\" takes " + std::to_string(params.size()) +
                               " input and " +
                               std::to_string(f.outputs.size()) +
                               " output arrays.");
    }
    for (size_t p = 0; p < params.size(); ++p) {
      const std::string& name = names.empty() ? params[p].name : names[p];
      if (array(name).dim != params[p].dim) {
        throw std::runtime_error("KernelBatch::launch: array \"" + name +
                                 "\" has the wrong size for function \"" +
                                 f.name + "\".");
      }
      pointers.push_back(device(name));
    }
  }

  static void check(const std::string& error) {
    if (!error.empty()) {
      throw std::runtime_error("CUDA error: " + error);
    }
  }
};

}  // namespace tds
