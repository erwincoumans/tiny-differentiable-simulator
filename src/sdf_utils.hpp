#pragma once

#include <functional>

#include "visualizer/opengl/tiny_gl_instancing_renderer.h"

namespace tds {
struct RenderShape {
  std::vector<GfxVertexFormat1> vertices;
  std::vector<int> indices;

  size_t num_triangles;
  size_t num_vertices;
};

struct MarchingCubesConfig {
  float minX;
  float maxX;
  float minY;
  float maxY;
  float minZ;
  float maxZ;

  int ncellsX;
  int ncellsY;
  int ncellsZ;

  float gradX;
  float gradY;
  float gradZ;

  float minValue = 0;

  bool compute_normals = true;
};

template <typename Algebra>
class SDF {
  using Vector3 = typename Algebra::Vector3;

 protected:
  Vector3 max_boundaries;
  Vector3 min_boundaries;

 public:
  // getters for the boundaries of the geometry
  const Vector3& get_max_boundaries() const { return max_boundaries; }

  const Vector3& get_min_boundaries() const { return min_boundaries; }

  virtual typename Algebra::Scalar distance(
      const typename Algebra::Vector3& point) const = 0;
};
}  // namespace tds