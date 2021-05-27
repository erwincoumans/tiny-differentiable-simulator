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

}  // namespace tds