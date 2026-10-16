#pragma once

// Loads kernels that are compiled at run time into a tds::KernelModel
// (kernel_model.hpp), to run them with tds::KernelBatch (kernel_batch.hpp).
//
// tds::KernelCodegen::write_runtime_sources writes the sources and the
// description kernels.json of a model. The CPU kernels (kernels_cpu.cpp) are
// compiled into a shared library per precision, for example
//   c++ -O2 -shared -fPIC -DTDS_KERNELS_FLOAT kernels_cpu.cpp
//       -o kernels_cpu_float.so
// and the CUDA kernels (kernels_cuda.cu) are compiled with NVRTC
// (kernel_nvrtc.hpp) into CUBIN or PTX images per precision. Large functions
// are split into separately compiled parts (-DTDS_KERNEL_PART=p), that can
// be compiled in parallel. Then
//
//   auto model = tds::load_kernel_model("kernels.json",
//       {{"float", "kernels_cpu_float.so"}},
//       {{"float", {"kernels_cuda_float.cubin"}}});
//   tds::KernelBatch<float> batch(*model, 4096, tds::KernelBackend::CUDA);
//
// The tds_bake Python package (python/tds_bake) does all of this.

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamic_library.hpp"
#include "kernel_cuda_driver.hpp"
#include "kernel_model.hpp"
#include "nlohmann/json.hpp"

namespace tds {

namespace kernel_loader {

// the libraries and CUDA modules of a loaded model
struct Resources {
  std::vector<std::unique_ptr<DynamicLibrary>> libraries;
  std::vector<CudaDriver::CUmodule> modules;
  const CudaDriver* driver{nullptr};

  ~Resources() {
    if (driver) {
      for (auto module : modules) driver->unload_module(module);
    }
  }
};

inline std::string read_file(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Cannot read " + path);
  }
  std::ostringstream content;
  content << file.rdbuf();
  return content.str();
}

// the C function of a function, see KernelCodegen::runtime_cpu_source
template <typename Float>
using CpuKernel = void (*)(int begin, int end, int stride,
                           const Float* const* inputs, Float* const* outputs,
                           Float* scratch, int iterations);

template <typename Float>
KernelCpuLauncher<Float> cpu_launcher(std::vector<CpuKernel<Float>> kernels,
                                      int num_outputs, bool has_scratch) {
  return [kernels, num_outputs, has_scratch](
             int num_worlds, int stride, const Float* const* inputs,
             Float* const* outputs, int num_threads, int iterations) {
    Float* scratch = has_scratch ? outputs[num_outputs] : nullptr;
    kernel_parallel_ranges(num_worlds, num_threads, [&](int begin, int end) {
      // all segments for a few worlds at a time, which keeps their scratch
      // arrays in the cache
      const int chunk = 16;
      for (int b = begin; b < end; b += chunk) {
        const int e = std::min(end, b + chunk);
        for (const auto& kernel : kernels) {
          kernel(b, e, stride, inputs, outputs, scratch, iterations);
        }
      }
    });
  };
}

template <typename Float>
KernelCudaLauncher<Float> cuda_launcher(
    const CudaDriver* driver, std::vector<CudaDriver::CUfunction> kernels,
    int num_inputs, int num_outputs, bool has_scratch) {
  return [driver, kernels, num_inputs, num_outputs, has_scratch](
             int num_worlds, int stride, const Float* const* inputs,
             Float* const* outputs, int block_size, void* stream,
             int iterations) {
    if (num_worlds <= 0) return;
    // the kernel parameters: num_worlds, stride, the input and output arrays
    // in the order of the function, scratch and iterations
    std::vector<const void*> arrays;
    for (int a = 0; a < num_inputs; ++a) arrays.push_back(inputs[a]);
    for (int a = 0; a < num_outputs; ++a) arrays.push_back(outputs[a]);
    arrays.push_back(has_scratch ? outputs[num_outputs] : nullptr);
    std::vector<void*> params = {&num_worlds, &stride};
    for (auto& array : arrays) params.push_back(&array);
    params.push_back(&iterations);
    const unsigned num_blocks = (num_worlds + block_size - 1) / block_size;
    for (auto kernel : kernels) {
      const auto result = driver->launch(kernel, num_blocks, block_size,
                                         params.data(), stream);
      if (result != 0) {
        CudaDriver::last_launch_error() = driver->error_string(result);
        return;
      }
    }
  };
}

}  // namespace kernel_loader

// Loads the kernels described by the manifest kernels.json. cpu_libraries
// maps a precision ("float" or "double") to the shared library of the CPU
// kernels (compiled from all "cpu_parts" of kernels.json), cuda_images maps a
// precision to the CUBIN or PTX files of the CUDA kernels (such as one per
// part, see "parts" in kernels.json). Precisions
// and backends without files are not available in the model (see
// KernelModel::has_kernels).
inline std::shared_ptr<KernelModel> load_kernel_model(
    const std::string& manifest_path,
    const std::map<std::string, std::string>& cpu_libraries,
    const std::map<std::string, std::vector<std::string>>& cuda_images,
    int cuda_device = 0) {
  using nlohmann::json;
  const json manifest = json::parse(kernel_loader::read_file(manifest_path));
  if (manifest.value("format", "") != "tds_kernels") {
    throw std::runtime_error(manifest_path +
                             " is not a description of kernels.");
  }
  auto model = std::make_shared<KernelModel>();
  auto resources = std::make_shared<kernel_loader::Resources>();
  model->resources = resources;
  model->name = manifest.at("name").get<std::string>();
  for (const auto& [name, value] : manifest.at("constants").items()) {
    if (value.is_number()) model->constants[name] = value.get<double>();
  }
  for (const auto& [name, value] : manifest.at("strings").items()) {
    model->strings[name] = value.get<std::string>();
  }
  for (const auto& f : manifest.at("functions")) {
    KernelFunction function;
    function.name = f.at("name").get<std::string>();
    for (const auto& a : f.at("inputs")) {
      function.inputs.push_back(
          {a.at("name").get<std::string>(), a.at("dim").get<int>()});
    }
    for (const auto& a : f.at("outputs")) {
      function.outputs.push_back(
          {a.at("name").get<std::string>(), a.at("dim").get<int>()});
    }
    function.num_operations = f.value("num_operations", 0);
    function.num_temporaries = f.value("num_temporaries", 0);
    function.segments = f.value("segments", 1);
    function.scratch_dim = f.value("scratch_dim", 0);
    function.loop_input = f.value("loop_input", "");
    function.default_iterations = f.value("default_iterations", 1);
    model->functions.push_back(function);
  }

  for (const auto& [precision, path] : cpu_libraries) {
    if (precision != "float" && precision != "double") {
      throw std::runtime_error("Unknown precision \"" + precision + "\".");
    }
    auto library = std::make_unique<DynamicLibrary>();
    if (!library->open({path})) {
      throw std::runtime_error("Cannot load the CPU kernels " + path + ": " +
                               library->error());
    }
    for (auto& f : model->functions) {
      auto symbol = [&](int k) {
        return "tds_kernel_" + f.name + "_" + std::to_string(k) + "_" +
               precision;
      };
      // the CPU kernels are not split into segments
      std::vector<void*> kernels = {library->symbol(symbol(0))};
      if (!kernels.back()) {
        throw std::runtime_error(path + " has no function " + symbol(0));
      }
      const int num_outputs = static_cast<int>(f.outputs.size());
      const bool has_scratch = f.scratch_dim > 0;
      if (precision == "float") {
        std::vector<kernel_loader::CpuKernel<float>> typed;
        for (void* k : kernels) {
          typed.push_back(reinterpret_cast<kernel_loader::CpuKernel<float>>(k));
        }
        f.cpu_float = kernel_loader::cpu_launcher<float>(typed, num_outputs,
                                                         has_scratch);
      } else {
        std::vector<kernel_loader::CpuKernel<double>> typed;
        for (void* k : kernels) {
          typed.push_back(
              reinterpret_cast<kernel_loader::CpuKernel<double>>(k));
        }
        f.cpu_double = kernel_loader::cpu_launcher<double>(typed, num_outputs,
                                                           has_scratch);
      }
    }
    resources->libraries.push_back(std::move(library));
  }

  if (!cuda_images.empty()) {
    const CudaDriver& driver = CudaDriver::get(cuda_device);
    resources->driver = &driver;
    model->cuda = driver.kernel_cuda_api();
    for (const auto& [precision, paths] : cuda_images) {
      if (precision != "float" && precision != "double") {
        throw std::runtime_error("Unknown precision \"" + precision + "\".");
      }
      // the kernels of the parts, each part can be in its own image
      std::map<std::string, CudaDriver::CUfunction> kernels;
      for (const auto& path : paths) {
        std::string image = kernel_loader::read_file(path);
        // PTX must be null-terminated
        image.push_back('\0');
        auto module = driver.load_module(image.data());
        resources->modules.push_back(module);
        for (const auto& f : model->functions) {
          for (int k = 0; k < f.segments; ++k) {
            const std::string symbol = "tds_kernel_" + f.name + "_" +
                                       std::to_string(k) + "_" + precision;
            if (kernels.count(symbol)) continue;
            auto kernel = driver.find_function(module, symbol);
            if (kernel) kernels[symbol] = kernel;
          }
        }
      }
      for (auto& f : model->functions) {
        std::vector<CudaDriver::CUfunction> segments;
        for (int k = 0; k < f.segments; ++k) {
          const std::string symbol = "tds_kernel_" + f.name + "_" +
                                     std::to_string(k) + "_" + precision;
          auto it = kernels.find(symbol);
          if (it == kernels.end()) {
            throw std::runtime_error("The CUDA kernels have no function " +
                                     symbol);
          }
          segments.push_back(it->second);
        }
        const int num_inputs = static_cast<int>(f.inputs.size());
        const int num_outputs = static_cast<int>(f.outputs.size());
        const bool has_scratch = f.scratch_dim > 0;
        if (precision == "float") {
          f.cuda_float = kernel_loader::cuda_launcher<float>(
              &driver, segments, num_inputs, num_outputs, has_scratch);
        } else {
          f.cuda_double = kernel_loader::cuda_launcher<double>(
              &driver, segments, num_inputs, num_outputs, has_scratch);
        }
      }
    }
  }
  return model;
}

}  // namespace tds
