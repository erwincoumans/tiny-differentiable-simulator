#pragma once

// Interface of the batched kernels that tds::KernelCodegen (kernel_codegen.hpp)
// generates. This header does not depend on CppAD or CUDA, it is included by
// the generated sources and by the code that runs the kernels, see
// kernel_batch.hpp.
//
// A kernel model is a set of functions (such as step and poses), each mapping
// named input arrays to named output arrays. The kernels evaluate a function
// for many independent worlds at once. All arrays use a structure-of-arrays
// layout: element k of world i is at array[k * stride + i].

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#if defined(__CUDACC__)
#define TDS_KERNEL_FUNCTION __host__ __device__ inline
#else
#define TDS_KERNEL_FUNCTION inline
#endif

#if defined(_MSC_VER)
#define TDS_RESTRICT __restrict
#else
#define TDS_RESTRICT __restrict__
#endif

namespace tds {

struct KernelArray {
  std::string name;
  int dim;
};

// Evaluates the function for num_worlds worlds on the CPU. inputs and outputs
// point to the arrays of the function, in the order of KernelFunction::inputs
// and KernelFunction::outputs. num_threads = 1 runs serially, num_threads = 0
// uses all OpenMP threads. If the function has a scratch array
// (KernelFunction::scratch_dim > 0), it follows the outputs. iterations is
// the number of iterations of a loop function (KernelFunction::loop_input).
template <typename Float>
using KernelCpuLauncher = std::function<void(
    int num_worlds, int stride, const Float* const* inputs,
    Float* const* outputs, int num_threads, int iterations)>;

// Launches the CUDA kernel of the function for num_worlds worlds, the inputs
// and outputs are device pointers. If the function has a scratch array
// (KernelFunction::scratch_dim > 0), it follows the outputs. The launch is
// asynchronous on the given stream (a cudaStream_t or CUstream, nullptr is
// the default stream).
template <typename Float>
using KernelCudaLauncher = std::function<void(
    int num_worlds, int stride, const Float* const* inputs,
    Float* const* outputs, int block_size, void* stream, int iterations)>;

struct KernelFunction {
  std::string name;
  std::vector<KernelArray> inputs;
  std::vector<KernelArray> outputs;
  // size of the generated code
  int num_operations{0};
  int num_temporaries{0};
  // number of separately compiled segments, and the size of the scratch
  // array per world that the CUDA kernels need between the segments and for
  // the iterations of a loop function
  int segments{1};
  int scratch_dim{0};
  // A loop function evaluates its single output from its inputs iterations
  // times, where the input array loop_input (of the size of the output) is
  // the previous output in all but the first iteration. Such as the
  // iterations of a projected Gauss-Seidel solver.
  std::string loop_input;
  int default_iterations{1};

  // only set for the compiled backends and precisions
  KernelCpuLauncher<float> cpu_float;
  KernelCpuLauncher<double> cpu_double;
  KernelCudaLauncher<float> cuda_float;
  KernelCudaLauncher<double> cuda_double;

  template <typename Float>
  const KernelCpuLauncher<Float>& cpu() const {
    if constexpr (std::is_same_v<Float, float>) {
      return cpu_float;
    } else {
      return cpu_double;
    }
  }
  template <typename Float>
  const KernelCudaLauncher<Float>& cuda() const {
    if constexpr (std::is_same_v<Float, float>) {
      return cuda_float;
    } else {
      return cuda_double;
    }
  }
};

// CUDA functions to manage the device memory of the kernels: the CUDA runtime
// functions compiled with the CUDA kernels of a model (kernel_cuda_api.cuh),
// so that code that runs the kernels does not need the CUDA headers, or the
// CUDA driver functions for the kernels that are compiled at run time
// (kernel_cuda_driver.hpp).
struct KernelCudaApi {
  // all functions return an empty string on success, or the CUDA error
  std::string (*allocate)(void** ptr, size_t size);
  std::string (*deallocate)(void* ptr);
  std::string (*copy_to_device)(void* dst, const void* src, size_t size,
                                void* stream);
  std::string (*copy_to_host)(void* dst, const void* src, size_t size,
                              void* stream);
  std::string (*copy_on_device)(void* dst, const void* src, size_t size,
                                void* stream);
  std::string (*synchronize)(void* stream);
  // returns the error of the last kernel launch
  std::string (*last_error)();
  std::string (*device_name)();
};

struct KernelModel {
  std::string name;
  std::vector<KernelFunction> functions;
  // constants of the model, such as the time step
  std::map<std::string, double> constants;
  std::map<std::string, std::string> strings;
  // only set when the CUDA kernels are compiled
  const KernelCudaApi* cuda{nullptr};
  // keeps the libraries and CUDA modules of kernels that are loaded at run
  // time alive, see kernel_loader.hpp
  std::shared_ptr<void> resources;

  bool has_cuda() const { return cuda != nullptr; }

  // whether the kernels of all functions are available for the backend
  // ("cpu" or "cuda") and precision ("float" or "double")
  bool has_kernels(const std::string& backend,
                   const std::string& precision) const {
    for (const auto& f : functions) {
      const bool has =
          backend == "cuda"
              ? (precision == "float" ? bool(f.cuda_float)
                                      : bool(f.cuda_double)) && has_cuda()
              : (precision == "float" ? bool(f.cpu_float)
                                      : bool(f.cpu_double));
      if (!has) return false;
    }
    return !functions.empty();
  }

  const KernelFunction& function(const std::string& function_name) const {
    for (const auto& f : functions) {
      if (f.name == function_name) return f;
    }
    throw std::runtime_error("Kernel model \"" + name +
                             "\" has no function \"" + function_name + "\".");
  }
  double constant(const std::string& constant_name) const {
    auto it = constants.find(constant_name);
    if (it == constants.end()) {
      throw std::runtime_error("Kernel model \"" + name +
                               "\" has no constant \"" + constant_name + "\".");
    }
    return it->second;
  }
  const std::string& string(const std::string& string_name) const {
    auto it = strings.find(string_name);
    if (it == strings.end()) {
      throw std::runtime_error("Kernel model \"" + name +
                               "\" has no string \"" + string_name + "\".");
    }
    return it->second;
  }
};

// Calls f(begin, end) for consecutive ranges that cover [0, n), one range per
// OpenMP thread when OpenMP is enabled and num_threads != 1.
template <typename F>
inline void kernel_parallel_ranges(int n, int num_threads, F&& f) {
#ifdef _OPENMP
  if (num_threads != 1 && n > 1) {
    const int threads =
        std::min(n, num_threads > 0 ? num_threads : omp_get_max_threads());
#pragma omp parallel num_threads(threads)
    {
      const int t = omp_get_thread_num();
      const int count = omp_get_num_threads();
      const int begin = static_cast<int>(static_cast<long long>(n) * t / count);
      const int end =
          static_cast<int>(static_cast<long long>(n) * (t + 1) / count);
      if (begin < end) f(begin, end);
    }
    return;
  }
#endif
  if (n > 0) f(0, n);
}

// Calls f(i) for i in [0, n), in parallel with OpenMP when it is enabled and
// num_threads != 1.
template <typename F>
inline void kernel_parallel_for(int n, int num_threads, F&& f) {
#ifdef _OPENMP
  if (num_threads != 1) {
    const int threads = num_threads > 0 ? num_threads : omp_get_max_threads();
#pragma omp parallel for schedule(static) num_threads(threads)
    for (int i = 0; i < n; ++i) {
      f(i);
    }
    return;
  }
#endif
  for (int i = 0; i < n; ++i) {
    f(i);
  }
}

}  // namespace tds
