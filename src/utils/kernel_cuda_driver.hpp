#pragma once

// The CUDA driver API, loaded at run time from the NVIDIA driver (libcuda),
// for CUDA kernels that are compiled at run time with NVRTC
// (kernel_nvrtc.hpp, kernel_loader.hpp). Neither the CUDA toolkit nor its
// headers are needed to compile this.
//
// All functions use the primary context of the device, which the CUDA runtime
// (and libraries such as PyTorch) use as well, and the default stream, so
// that the kernels and copies are ordered with their work.

#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamic_library.hpp"
#include "kernel_model.hpp"

namespace tds {

class CudaDriver {
 public:
  using CUresult = int;
  using CUdevice = int;
  using CUcontext = struct CUctx_st*;
  using CUmodule = struct CUmod_st*;
  using CUfunction = struct CUfunc_st*;
  using CUstream = struct CUstream_st*;
  using CUdeviceptr = unsigned long long;
  using CUgraphicsResource = struct CUgraphicsResource_st*;

  // The driver of the device, loaded on first use. Throws if there is no
  // CUDA driver or device.
  static CudaDriver& get(int device = 0) {
    static std::mutex mutex;
    static std::vector<std::unique_ptr<CudaDriver>> drivers;
    std::lock_guard<std::mutex> lock(mutex);
    for (auto& d : drivers) {
      if (d->device_ordinal_ == device) return *d;
    }
    drivers.emplace_back(new CudaDriver(device));
    return *drivers.back();
  }

  // whether a CUDA driver and device are available, without throwing
  static bool available(std::string* error = nullptr) {
    try {
      get();
      return true;
    } catch (const std::exception& e) {
      if (error) *error = e.what();
      return false;
    }
  }

  int device_ordinal() const { return device_ordinal_; }
  const std::string& device_name() const { return device_name_; }
  // compute capability, such as 120 for sm_120
  int compute_capability() const { return compute_capability_; }
  int driver_version() const { return driver_version_; }

  // makes the primary context current on the calling thread
  void make_current() const {
    CUcontext current = nullptr;
    if (cuCtxGetCurrent(&current) != 0 || current != context_) {
      check(cuCtxSetCurrent(context_), "cuCtxSetCurrent");
    }
  }

  // empty string on success, or the error of the result
  std::string error_string(CUresult result) const {
    if (result == 0) return std::string();
    const char* name = nullptr;
    const char* description = nullptr;
    if (cuGetErrorName) cuGetErrorName(result, &name);
    if (cuGetErrorString) cuGetErrorString(result, &description);
    return std::string(name ? name : "CUDA error " + std::to_string(result)) +
           (description ? std::string(": ") + description : std::string());
  }

  void check(CUresult result, const char* what) const {
    if (result != 0) {
      throw std::runtime_error(std::string(what) + " failed: " +
                               error_string(result));
    }
  }

  // Loads a module from a CUBIN or PTX image (PTX must be null-terminated).
  CUmodule load_module(const void* image) const {
    make_current();
    CUmodule module = nullptr;
    check(cuModuleLoadData(&module, image), "cuModuleLoadData");
    return module;
  }
  void unload_module(CUmodule module) const {
    if (!module) return;
    make_current();
    cuModuleUnload(module);
  }
  // the kernel, or nullptr if the module does not have it
  CUfunction find_function(CUmodule module, const std::string& name) const {
    CUfunction f = nullptr;
    if (cuModuleGetFunction(&f, module, name.c_str()) != 0) return nullptr;
    return f;
  }
  CUfunction function(CUmodule module, const std::string& name) const {
    CUfunction f = nullptr;
    check(cuModuleGetFunction(&f, module, name.c_str()),
          ("cuModuleGetFunction(" + name + ")").c_str());
    return f;
  }

  // launches a kernel with a one-dimensional grid
  CUresult launch(CUfunction f, unsigned grid_size, unsigned block_size,
                  void** params, void* stream) const {
    make_current();
    return cuLaunchKernel(f, grid_size, 1, 1, block_size, 1, 1, 0,
                          static_cast<CUstream>(stream), params, nullptr);
  }

  // tds::KernelCudaApi of the driver, for tds::KernelBatch
  const KernelCudaApi* kernel_cuda_api() const { return &api_; }

  // the error of the last kernel launch on the calling thread, see
  // KernelCudaApi::last_error
  static std::string& last_launch_error() {
    static thread_local std::string error;
    return error;
  }

  // driver functions
  CUresult (*cuInit)(unsigned) = nullptr;
  CUresult (*cuDriverGetVersion)(int*) = nullptr;
  CUresult (*cuDeviceGet)(CUdevice*, int) = nullptr;
  CUresult (*cuDeviceGetCount)(int*) = nullptr;
  CUresult (*cuDeviceGetName)(char*, int, CUdevice) = nullptr;
  CUresult (*cuDeviceGetAttribute)(int*, int, CUdevice) = nullptr;
  CUresult (*cuDevicePrimaryCtxRetain)(CUcontext*, CUdevice) = nullptr;
  CUresult (*cuCtxGetCurrent)(CUcontext*) = nullptr;
  CUresult (*cuCtxSetCurrent)(CUcontext) = nullptr;
  CUresult (*cuMemAlloc)(CUdeviceptr*, size_t) = nullptr;
  CUresult (*cuMemFree)(CUdeviceptr) = nullptr;
  CUresult (*cuMemcpyHtoDAsync)(CUdeviceptr, const void*, size_t,
                                CUstream) = nullptr;
  CUresult (*cuMemcpyDtoHAsync)(void*, CUdeviceptr, size_t,
                                CUstream) = nullptr;
  CUresult (*cuMemcpyDtoDAsync)(CUdeviceptr, CUdeviceptr, size_t,
                                CUstream) = nullptr;
  CUresult (*cuStreamSynchronize)(CUstream) = nullptr;
  CUresult (*cuModuleLoadData)(CUmodule*, const void*) = nullptr;
  CUresult (*cuModuleUnload)(CUmodule) = nullptr;
  CUresult (*cuModuleGetFunction)(CUfunction*, CUmodule,
                                  const char*) = nullptr;
  CUresult (*cuLaunchKernel)(CUfunction, unsigned, unsigned, unsigned,
                             unsigned, unsigned, unsigned, unsigned, CUstream,
                             void**, void**) = nullptr;
  CUresult (*cuGetErrorName)(CUresult, const char**) = nullptr;
  CUresult (*cuGetErrorString)(CUresult, const char**) = nullptr;
  // OpenGL interop, nullptr if the driver does not have it
  CUresult (*cuGraphicsGLRegisterBuffer)(CUgraphicsResource*, unsigned,
                                         unsigned) = nullptr;
  CUresult (*cuGraphicsUnregisterResource)(CUgraphicsResource) = nullptr;
  CUresult (*cuGraphicsMapResources)(unsigned, CUgraphicsResource*,
                                     CUstream) = nullptr;
  CUresult (*cuGraphicsUnmapResources)(unsigned, CUgraphicsResource*,
                                       CUstream) = nullptr;
  CUresult (*cuGraphicsResourceGetMappedPointer)(CUdeviceptr*, size_t*,
                                                 CUgraphicsResource) = nullptr;

 private:
  DynamicLibrary library_;
  int device_ordinal_{0};
  CUdevice device_{0};
  CUcontext context_{nullptr};
  std::string device_name_;
  int compute_capability_{0};
  int driver_version_{0};
  KernelCudaApi api_{};

  explicit CudaDriver(int device) : device_ordinal_(device) {
#ifdef _WIN32
    const std::vector<std::string> names = {"nvcuda.dll"};
#else
    const std::vector<std::string> names = {"libcuda.so.1", "libcuda.so"};
#endif
    if (!library_.open(names)) {
      throw std::runtime_error("Cannot load the CUDA driver: " +
                               library_.error());
    }
    bool ok = true;
    ok &= library_.load(cuInit, "cuInit");
    ok &= library_.load(cuDriverGetVersion, "cuDriverGetVersion");
    ok &= library_.load(cuDeviceGet, "cuDeviceGet");
    ok &= library_.load(cuDeviceGetCount, "cuDeviceGetCount");
    ok &= library_.load(cuDeviceGetName, "cuDeviceGetName");
    ok &= library_.load(cuDeviceGetAttribute, "cuDeviceGetAttribute");
    ok &= library_.load(cuDevicePrimaryCtxRetain, "cuDevicePrimaryCtxRetain");
    ok &= library_.load(cuCtxGetCurrent, "cuCtxGetCurrent");
    ok &= library_.load(cuCtxSetCurrent, "cuCtxSetCurrent");
    ok &= library_.load(cuMemAlloc, "cuMemAlloc_v2");
    ok &= library_.load(cuMemFree, "cuMemFree_v2");
    ok &= library_.load(cuMemcpyHtoDAsync, "cuMemcpyHtoDAsync_v2");
    ok &= library_.load(cuMemcpyDtoHAsync, "cuMemcpyDtoHAsync_v2");
    ok &= library_.load(cuMemcpyDtoDAsync, "cuMemcpyDtoDAsync_v2");
    ok &= library_.load(cuStreamSynchronize, "cuStreamSynchronize");
    ok &= library_.load(cuModuleLoadData, "cuModuleLoadData");
    ok &= library_.load(cuModuleUnload, "cuModuleUnload");
    ok &= library_.load(cuModuleGetFunction, "cuModuleGetFunction");
    ok &= library_.load(cuLaunchKernel, "cuLaunchKernel");
    ok &= library_.load(cuGetErrorName, "cuGetErrorName");
    ok &= library_.load(cuGetErrorString, "cuGetErrorString");
    if (!ok) {
      throw std::runtime_error("The CUDA driver " + library_.path() +
                               " misses functions, it is too old.");
    }
    library_.load(cuGraphicsGLRegisterBuffer, "cuGraphicsGLRegisterBuffer");
    library_.load(cuGraphicsUnregisterResource,
                  "cuGraphicsUnregisterResource");
    library_.load(cuGraphicsMapResources, "cuGraphicsMapResources");
    library_.load(cuGraphicsUnmapResources, "cuGraphicsUnmapResources");
    library_.load(cuGraphicsResourceGetMappedPointer,
                  "cuGraphicsResourceGetMappedPointer_v2");
    check(cuInit(0), "cuInit");
    cuDriverGetVersion(&driver_version_);
    int count = 0;
    check(cuDeviceGetCount(&count), "cuDeviceGetCount");
    if (device < 0 || device >= count) {
      throw std::runtime_error("There is no CUDA device " +
                               std::to_string(device) + " (" +
                               std::to_string(count) + " devices).");
    }
    check(cuDeviceGet(&device_, device), "cuDeviceGet");
    char name[256] = {0};
    cuDeviceGetName(name, sizeof(name) - 1, device_);
    device_name_ = name;
    // CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR / MINOR
    int major = 0, minor = 0;
    cuDeviceGetAttribute(&major, 75, device_);
    cuDeviceGetAttribute(&minor, 76, device_);
    compute_capability_ = 10 * major + minor;
    check(cuDevicePrimaryCtxRetain(&context_, device_),
          "cuDevicePrimaryCtxRetain");
    make_current();
    init_api();
  }

  // KernelCudaApi uses plain function pointers, they use the driver of
  // device 0. TODO: other devices.
  void init_api() {
    api_.allocate = [](void** ptr, size_t size) {
      const CudaDriver& d = get();
      d.make_current();
      CUdeviceptr p = 0;
      CUresult r = d.cuMemAlloc(&p, size > 0 ? size : 1);
      *ptr = reinterpret_cast<void*>(static_cast<uintptr_t>(p));
      return d.error_string(r);
    };
    api_.deallocate = [](void* ptr) {
      const CudaDriver& d = get();
      if (!ptr) return std::string();
      d.make_current();
      return d.error_string(d.cuMemFree(device_ptr(ptr)));
    };
    api_.copy_to_device = [](void* dst, const void* src, size_t size,
                             void* stream) {
      const CudaDriver& d = get();
      d.make_current();
      return d.error_string(d.cuMemcpyHtoDAsync(
          device_ptr(dst), src, size, static_cast<CUstream>(stream)));
    };
    api_.copy_to_host = [](void* dst, const void* src, size_t size,
                           void* stream) {
      const CudaDriver& d = get();
      d.make_current();
      return d.error_string(d.cuMemcpyDtoHAsync(
          dst, device_ptr(src), size, static_cast<CUstream>(stream)));
    };
    api_.copy_on_device = [](void* dst, const void* src, size_t size,
                             void* stream) {
      const CudaDriver& d = get();
      d.make_current();
      return d.error_string(d.cuMemcpyDtoDAsync(
          device_ptr(dst), device_ptr(src), size,
          static_cast<CUstream>(stream)));
    };
    api_.synchronize = [](void* stream) {
      const CudaDriver& d = get();
      d.make_current();
      return d.error_string(
          d.cuStreamSynchronize(static_cast<CUstream>(stream)));
    };
    api_.last_error = []() {
      std::string error;
      std::swap(error, last_launch_error());
      return error;
    };
    api_.device_name = []() { return get().device_name(); };
  }

  static CUdeviceptr device_ptr(const void* ptr) {
    return static_cast<CUdeviceptr>(reinterpret_cast<uintptr_t>(ptr));
  }
};

}  // namespace tds
