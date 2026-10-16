#pragma once

// Compiles CUDA sources at run time with NVRTC, loaded at run time from the
// CUDA toolkit or from the nvidia-cuda-nvrtc Python package, so that neither
// nvcc nor the CUDA headers are needed. See kernel_loader.hpp.

#include <algorithm>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamic_library.hpp"

namespace tds {

class Nvrtc {
 public:
  using nvrtcResult = int;
  using nvrtcProgram = struct _nvrtcProgram*;

  struct Result {
    // CUBIN for the requested architecture, or null-terminated PTX when NVRTC
    // does not support the architecture (the driver compiles the PTX)
    std::string image;
    bool is_ptx{false};
    // the architecture the image is compiled for, such as "sm_120"
    std::string arch;
    std::string log;
  };

  // NVRTC, loaded on first use from library_path, the TDS_NVRTC_LIBRARY
  // environment variable, the CUDA toolkit or the library search path.
  // Throws if NVRTC cannot be loaded.
  static Nvrtc& get(const std::string& library_path = "") {
    static std::mutex mutex;
    static std::unique_ptr<Nvrtc> nvrtc;
    std::lock_guard<std::mutex> lock(mutex);
    if (!nvrtc) nvrtc.reset(new Nvrtc(library_path));
    return *nvrtc;
  }

  const std::string& library_path() const { return library_.path(); }
  // such as 12080 for 12.8
  int version() const { return 1000 * major_ + 10 * minor_; }
  // the supported architectures, such as 120 for sm_120
  const std::vector<int>& supported_archs() const { return archs_; }

  // Compiles the source for the compute capability (such as 120). The
  // options are passed to NVRTC, such as "-DTDS_KERNELS_FLOAT". Throws with
  // the compiler log if the compilation fails.
  Result compile(const std::string& source, const std::string& name,
                 int compute_capability,
                 const std::vector<std::string>& options) const {
    Result result;
    // CUBIN for a supported architecture, otherwise PTX for the newest
    // supported one, which the driver compiles for the device
    int arch = compute_capability;
    if (!archs_.empty() &&
        std::find(archs_.begin(), archs_.end(), arch) == archs_.end()) {
      int best = 0;
      for (int a : archs_) {
        if (a <= compute_capability && a > best) best = a;
      }
      if (best == 0) {
        throw std::runtime_error(
            "NVRTC " + std::to_string(major_) + "." + std::to_string(minor_) +
            " does not support compute capability " +
            std::to_string(compute_capability));
      }
      arch = best;
      result.is_ptx = true;
    }
    const bool can_cubin = nvrtcGetCUBIN && nvrtcGetCUBINSize;
    if (!can_cubin) result.is_ptx = true;
    result.arch = (result.is_ptx ? "compute_" : "sm_") + std::to_string(arch);

    std::vector<std::string> all_options = {"-arch=" + result.arch,
                                            "-std=c++17"};
    all_options.insert(all_options.end(), options.begin(), options.end());
    std::vector<const char*> option_ptrs;
    for (const auto& o : all_options) option_ptrs.push_back(o.c_str());

    nvrtcProgram program = nullptr;
    check(nvrtcCreateProgram(&program, source.c_str(), name.c_str(), 0,
                             nullptr, nullptr),
          "nvrtcCreateProgram");
    const nvrtcResult compiled = nvrtcCompileProgram(
        program, static_cast<int>(option_ptrs.size()), option_ptrs.data());
    size_t log_size = 0;
    nvrtcGetProgramLogSize(program, &log_size);
    if (log_size > 1) {
      result.log.resize(log_size);
      nvrtcGetProgramLog(program, &result.log[0]);
      result.log.resize(log_size - 1);
    }
    if (compiled != 0) {
      nvrtcDestroyProgram(&program);
      throw std::runtime_error("NVRTC failed to compile " + name + ": " +
                               error_string(compiled) + "\n" + result.log);
    }
    size_t size = 0;
    if (result.is_ptx) {
      check(nvrtcGetPTXSize(program, &size), "nvrtcGetPTXSize");
      result.image.resize(size);
      check(nvrtcGetPTX(program, &result.image[0]), "nvrtcGetPTX");
    } else {
      check(nvrtcGetCUBINSize(program, &size), "nvrtcGetCUBINSize");
      result.image.resize(size);
      check(nvrtcGetCUBIN(program, &result.image[0]), "nvrtcGetCUBIN");
    }
    nvrtcDestroyProgram(&program);
    return result;
  }

 private:
  DynamicLibrary library_;
  int major_{0};
  int minor_{0};
  std::vector<int> archs_;

  nvrtcResult (*nvrtcVersion)(int*, int*) = nullptr;
  const char* (*nvrtcGetErrorString)(nvrtcResult) = nullptr;
  nvrtcResult (*nvrtcCreateProgram)(nvrtcProgram*, const char*, const char*,
                                    int, const char* const*,
                                    const char* const*) = nullptr;
  nvrtcResult (*nvrtcCompileProgram)(nvrtcProgram, int,
                                     const char* const*) = nullptr;
  nvrtcResult (*nvrtcDestroyProgram)(nvrtcProgram*) = nullptr;
  nvrtcResult (*nvrtcGetProgramLogSize)(nvrtcProgram, size_t*) = nullptr;
  nvrtcResult (*nvrtcGetProgramLog)(nvrtcProgram, char*) = nullptr;
  nvrtcResult (*nvrtcGetPTXSize)(nvrtcProgram, size_t*) = nullptr;
  nvrtcResult (*nvrtcGetPTX)(nvrtcProgram, char*) = nullptr;
  nvrtcResult (*nvrtcGetCUBINSize)(nvrtcProgram, size_t*) = nullptr;
  nvrtcResult (*nvrtcGetCUBIN)(nvrtcProgram, char*) = nullptr;
  nvrtcResult (*nvrtcGetNumSupportedArchs)(int*) = nullptr;
  nvrtcResult (*nvrtcGetSupportedArchs)(int*) = nullptr;

  static std::vector<std::string> library_names(const std::string& path) {
    std::vector<std::string> names;
    if (!path.empty()) names.push_back(path);
    if (const char* env = std::getenv("TDS_NVRTC_LIBRARY")) {
      names.push_back(env);
    }
    std::vector<std::string> dirs;
    for (const char* var : {"CUDA_HOME", "CUDA_PATH"}) {
      if (const char* env = std::getenv(var)) dirs.push_back(env);
    }
#ifdef _WIN32
    const std::vector<std::string> files = {
        "nvrtc64_130_0.dll", "nvrtc64_120_0.dll", "nvrtc64_112_0.dll"};
    for (const auto& dir : dirs) {
      for (const auto& file : files) names.push_back(dir + "\\bin\\" + file);
    }
#else
    const std::vector<std::string> files = {"libnvrtc.so.13", "libnvrtc.so.12",
                                            "libnvrtc.so"};
    dirs.push_back("/usr/local/cuda");
    for (const auto& dir : dirs) {
      for (const auto& file : files) names.push_back(dir + "/lib64/" + file);
    }
#endif
    names.insert(names.end(), files.begin(), files.end());
    return names;
  }

  explicit Nvrtc(const std::string& library_path) {
    if (!library_.open(library_names(library_path))) {
      throw std::runtime_error(
          "Cannot load NVRTC, install the CUDA toolkit or "
          "\"pip install nvidia-cuda-nvrtc-cu12\":\n" +
          library_.error());
    }
    bool ok = true;
    ok &= library_.load(nvrtcVersion, "nvrtcVersion");
    ok &= library_.load(nvrtcGetErrorString, "nvrtcGetErrorString");
    ok &= library_.load(nvrtcCreateProgram, "nvrtcCreateProgram");
    ok &= library_.load(nvrtcCompileProgram, "nvrtcCompileProgram");
    ok &= library_.load(nvrtcDestroyProgram, "nvrtcDestroyProgram");
    ok &= library_.load(nvrtcGetProgramLogSize, "nvrtcGetProgramLogSize");
    ok &= library_.load(nvrtcGetProgramLog, "nvrtcGetProgramLog");
    ok &= library_.load(nvrtcGetPTXSize, "nvrtcGetPTXSize");
    ok &= library_.load(nvrtcGetPTX, "nvrtcGetPTX");
    if (!ok) {
      throw std::runtime_error("NVRTC " + library_.path() +
                               " misses functions.");
    }
    // optional, CUBIN output needs NVRTC 11.1
    library_.load(nvrtcGetCUBINSize, "nvrtcGetCUBINSize");
    library_.load(nvrtcGetCUBIN, "nvrtcGetCUBIN");
    library_.load(nvrtcGetNumSupportedArchs, "nvrtcGetNumSupportedArchs");
    library_.load(nvrtcGetSupportedArchs, "nvrtcGetSupportedArchs");
    nvrtcVersion(&major_, &minor_);
    int num_archs = 0;
    if (nvrtcGetNumSupportedArchs && nvrtcGetSupportedArchs &&
        nvrtcGetNumSupportedArchs(&num_archs) == 0 && num_archs > 0) {
      archs_.resize(num_archs);
      nvrtcGetSupportedArchs(archs_.data());
    }
  }

  std::string error_string(nvrtcResult result) const {
    const char* s = nvrtcGetErrorString ? nvrtcGetErrorString(result) : nullptr;
    return s ? s : "NVRTC error " + std::to_string(result);
  }

  void check(nvrtcResult result, const char* what) const {
    if (result != 0) {
      throw std::runtime_error(std::string(what) + " failed: " +
                               error_string(result));
    }
  }
};

}  // namespace tds
