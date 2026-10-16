#pragma once

// Writes the world poses of the visual shapes of a batch of worlds directly
// into the instance arrays of an OpenGL vertex buffer with CUDA-OpenGL
// interop, so that rendering a batch that runs with CUDA needs no copies
// through the CPU. For the vertex buffer of TinyGLInstancingRenderer
// (src/visualizer/opengl), see python/tds_bake/render.py.
//
// The poses array has the layout of tds::KernelBatch arrays: element k of
// world i at poses[k * num_worlds + i], 7 elements per visual shape
// (position, quaternion x, y, z, w). Entry j of the writer copies the pose of
// visual visuals[j] of world worlds[j], moved by offsets[3 j .. 3 j + 2], to
// the instance instances[j] of the vertex buffer.

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "kernel_cuda_driver.hpp"
#include "kernel_nvrtc.hpp"

namespace tds {

class CudaGlInstanceWriter {
 public:
  using CUdeviceptr = CudaDriver::CUdeviceptr;

  // Registers the OpenGL vertex buffer with CUDA, the OpenGL context must be
  // current. positions_offset and orientations_offset are the byte offsets
  // of the instance arrays (4 floats per instance) in the vertex buffer.
  CudaGlInstanceWriter(unsigned vbo, size_t positions_offset,
                       size_t orientations_offset,
                       const std::vector<int>& worlds,
                       const std::vector<int>& visuals,
                       const std::vector<int>& instances,
                       const std::vector<float>& offsets, int device = 0)
      : driver_(CudaDriver::get(device)),
        positions_offset_(positions_offset),
        orientations_offset_(orientations_offset),
        count_(static_cast<int>(worlds.size())) {
    if (visuals.size() != worlds.size() || instances.size() != worlds.size() ||
        offsets.size() != 3 * worlds.size()) {
      throw std::runtime_error(
          "CudaGlInstanceWriter: worlds, visuals, instances and offsets "
          "(3 per entry) must have the same number of entries.");
    }
    if (!driver_.cuGraphicsGLRegisterBuffer) {
      throw std::runtime_error("The CUDA driver has no OpenGL interop.");
    }
    driver_.make_current();
    kernels();  // compiles the kernels on first use
    driver_.check(driver_.cuGraphicsGLRegisterBuffer(&resource_, vbo, 0),
                  "cuGraphicsGLRegisterBuffer");
    for (int k : instances) max_instance_ = std::max(max_instance_, k);
    worlds_ = upload(worlds.data(), worlds.size() * sizeof(int));
    visuals_ = upload(visuals.data(), visuals.size() * sizeof(int));
    instances_ = upload(instances.data(), instances.size() * sizeof(int));
    offsets_ = upload(offsets.data(), offsets.size() * sizeof(float));
  }

  ~CudaGlInstanceWriter() {
    driver_.make_current();
    if (resource_) driver_.cuGraphicsUnregisterResource(resource_);
    for (CUdeviceptr p : {worlds_, visuals_, instances_, offsets_}) {
      if (p) driver_.cuMemFree(p);
    }
  }

  CudaGlInstanceWriter(const CudaGlInstanceWriter&) = delete;
  CudaGlInstanceWriter& operator=(const CudaGlInstanceWriter&) = delete;

  // Writes the poses (a device pointer to float or double) of a batch of
  // num_worlds worlds to the vertex buffer, on the stream (nullptr is the
  // default stream). The vertex buffer is mapped during the copy.
  void write(const void* poses, int num_worlds, bool is_double,
             void* stream = nullptr) {
    if (count_ == 0) return;
    auto cu_stream = static_cast<CudaDriver::CUstream>(stream);
    driver_.make_current();
    driver_.check(driver_.cuGraphicsMapResources(1, &resource_, cu_stream),
                  "cuGraphicsMapResources");
    CUdeviceptr base = 0;
    size_t size = 0;
    CudaDriver::CUresult result =
        driver_.cuGraphicsResourceGetMappedPointer(&base, &size, resource_);
    const size_t end = 16 * size_t(max_instance_ + 1);
    if (result == 0 && (positions_offset_ + end > size ||
                        orientations_offset_ + end > size)) {
      result = -1;
    }
    if (result == 0) {
      CUdeviceptr positions = base + positions_offset_;
      CUdeviceptr orientations = base + orientations_offset_;
      CUdeviceptr poses_ptr = reinterpret_cast<CUdeviceptr>(poses);
      int count = count_;
      void* params[] = {&count,      &num_worlds, &poses_ptr,
                        &worlds_,    &visuals_,   &instances_,
                        &offsets_,   &positions,  &orientations};
      const unsigned block = 128;
      result = driver_.launch(is_double ? kernels().write_double
                                        : kernels().write_float,
                              (count_ + block - 1) / block, block, params,
                              stream);
    }
    driver_.cuGraphicsUnmapResources(1, &resource_, cu_stream);
    if (result != 0) {
      throw std::runtime_error(
          "CudaGlInstanceWriter: " +
          (result == -1 ? std::string("the vertex buffer is too small")
                        : driver_.error_string(result)));
    }
  }

  int size() const { return count_; }

  // Copies the positions, orientations and the 8 floats after them (colors,
  // scales) of the first num_instances instances from the vertex buffer,
  // for tests. The arrays have 4 floats per instance.
  void read(int num_instances, std::vector<float>& positions,
            std::vector<float>& orientations) {
    driver_.make_current();
    driver_.check(driver_.cuGraphicsMapResources(1, &resource_, nullptr),
                  "cuGraphicsMapResources");
    CUdeviceptr base = 0;
    size_t size = 0;
    auto result =
        driver_.cuGraphicsResourceGetMappedPointer(&base, &size, resource_);
    positions.assign(4 * size_t(num_instances), 0.f);
    orientations.assign(4 * size_t(num_instances), 0.f);
    if (result == 0) {
      result = driver_.cuMemcpyDtoHAsync(positions.data(),
                                         base + positions_offset_,
                                         positions.size() * sizeof(float),
                                         nullptr);
    }
    if (result == 0) {
      result = driver_.cuMemcpyDtoHAsync(orientations.data(),
                                         base + orientations_offset_,
                                         orientations.size() * sizeof(float),
                                         nullptr);
    }
    if (result == 0) result = driver_.cuStreamSynchronize(nullptr);
    driver_.cuGraphicsUnmapResources(1, &resource_, nullptr);
    driver_.check(result, "CudaGlInstanceWriter::read");
  }

 private:
  struct Kernels {
    CudaDriver::CUmodule module{nullptr};
    CudaDriver::CUfunction write_float{nullptr};
    CudaDriver::CUfunction write_double{nullptr};
  };

  const CudaDriver& driver_;
  CudaDriver::CUgraphicsResource resource_{nullptr};
  size_t positions_offset_;
  size_t orientations_offset_;
  int count_;
  int max_instance_{-1};
  CUdeviceptr worlds_{0}, visuals_{0}, instances_{0}, offsets_{0};

  CUdeviceptr upload(const void* data, size_t bytes) {
    CUdeviceptr p = 0;
    if (bytes == 0) return p;
    driver_.check(driver_.cuMemAlloc(&p, bytes), "cuMemAlloc");
    driver_.check(driver_.cuMemcpyHtoDAsync(p, data, bytes, nullptr),
                  "cuMemcpyHtoD");
    return p;
  }

  // the kernels, compiled with NVRTC once per process
  const Kernels& kernels() {
    static std::mutex mutex;
    static Kernels kernels;
    std::lock_guard<std::mutex> lock(mutex);
    if (!kernels.module) {
      static const char* source = R"(
template <typename T>
__device__ void write_poses(int count, int num_worlds, const T* poses,
                            const int* worlds, const int* visuals,
                            const int* instances, const float* offsets,
                            float* positions, float* orientations) {
  const int j = blockIdx.x * blockDim.x + threadIdx.x;
  if (j >= count) return;
  const int k = instances[j];
  if (k < 0) return;
  const T* p = poses + (long long)(7 * visuals[j]) * num_worlds + worlds[j];
  positions[4 * k + 0] = float(p[0]) + offsets[3 * j + 0];
  positions[4 * k + 1] = float(p[num_worlds]) + offsets[3 * j + 1];
  positions[4 * k + 2] = float(p[2 * num_worlds]) + offsets[3 * j + 2];
  positions[4 * k + 3] = 1.f;
  orientations[4 * k + 0] = float(p[3 * num_worlds]);
  orientations[4 * k + 1] = float(p[4 * num_worlds]);
  orientations[4 * k + 2] = float(p[5 * num_worlds]);
  orientations[4 * k + 3] = float(p[6 * num_worlds]);
}
extern "C" __global__ void tds_write_poses_float(
    int count, int num_worlds, const float* poses, const int* worlds,
    const int* visuals, const int* instances, const float* offsets,
    float* positions, float* orientations) {
  write_poses<float>(count, num_worlds, poses, worlds, visuals, instances,
                     offsets, positions, orientations);
}
extern "C" __global__ void tds_write_poses_double(
    int count, int num_worlds, const double* poses, const int* worlds,
    const int* visuals, const int* instances, const float* offsets,
    float* positions, float* orientations) {
  write_poses<double>(count, num_worlds, poses, worlds, visuals, instances,
                      offsets, positions, orientations);
}
)";
      const auto result = Nvrtc::get().compile(
          source, "gl_interop.cu", driver_.compute_capability(), {});
      std::string image = result.image;
      image.push_back('\0');
      kernels.module = driver_.load_module(image.data());
      kernels.write_float =
          driver_.function(kernels.module, "tds_write_poses_float");
      kernels.write_double =
          driver_.function(kernels.module, "tds_write_poses_double");
    }
    return kernels;
  }
};

}  // namespace tds
