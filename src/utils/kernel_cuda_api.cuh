#pragma once

// CUDA runtime functions for tds::KernelCudaApi (kernel_model.hpp), included
// by the CUDA kernels that tds::KernelCodegen generates.

#include <cuda_runtime.h>

#include <string>

#include "kernel_model.hpp"

namespace tds {

namespace kernel_cuda {

inline std::string error_string(cudaError_t status) {
  if (status == cudaSuccess) return std::string();
  return std::string(cudaGetErrorName(status)) + ": " +
         cudaGetErrorString(status);
}

inline std::string allocate(void** ptr, size_t size) {
  return error_string(cudaMalloc(ptr, size));
}

inline std::string deallocate(void* ptr) {
  return error_string(cudaFree(ptr));
}

inline std::string copy(void* dst, const void* src, size_t size,
                        cudaMemcpyKind kind, void* stream) {
  return error_string(cudaMemcpyAsync(dst, src, size, kind,
                                      static_cast<cudaStream_t>(stream)));
}

inline std::string copy_to_device(void* dst, const void* src, size_t size,
                                  void* stream) {
  return copy(dst, src, size, cudaMemcpyHostToDevice, stream);
}

inline std::string copy_to_host(void* dst, const void* src, size_t size,
                                void* stream) {
  return copy(dst, src, size, cudaMemcpyDeviceToHost, stream);
}

inline std::string copy_on_device(void* dst, const void* src, size_t size,
                                  void* stream) {
  return copy(dst, src, size, cudaMemcpyDeviceToDevice, stream);
}

inline std::string synchronize(void* stream) {
  return error_string(
      cudaStreamSynchronize(static_cast<cudaStream_t>(stream)));
}

inline std::string last_error() { return error_string(cudaGetLastError()); }

inline std::string device_name() {
  int device = 0;
  cudaDeviceProp properties;
  if (cudaGetDevice(&device) != cudaSuccess ||
      cudaGetDeviceProperties(&properties, device) != cudaSuccess) {
    return std::string();
  }
  return properties.name;
}

}  // namespace kernel_cuda

inline const KernelCudaApi* kernel_cuda_api() {
  static const KernelCudaApi api = {
      &kernel_cuda::allocate,       &kernel_cuda::deallocate,
      &kernel_cuda::copy_to_device, &kernel_cuda::copy_to_host,
      &kernel_cuda::copy_on_device, &kernel_cuda::synchronize,
      &kernel_cuda::last_error,     &kernel_cuda::device_name};
  return &api;
}

}  // namespace tds
