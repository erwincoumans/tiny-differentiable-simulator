#pragma once

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "geometry.hpp"
#include "sdf_utils.hpp"
#include "utils/sdf_to_mesh/marching_cubes.hpp"
#include "visualizer/opengl/tiny_gl_instancing_renderer.h"
#include "visualizer/opengl/tiny_opengl3_app.h"


namespace tds {

struct RenderShape {
  std::vector<GfxVertexFormat1> vertices;
  std::vector<int> indices;

  size_t num_triangles;
  size_t num_vertices;
};

template <typename Algebra>
RenderShape convert_sdf_to_mesh(
    const Geometry<Algebra> &shape, int num_cells,
    bool compute_normals = true)  // const references
{
  using Vector3 = typename Algebra::Vector3;
  using Vector4 = MC::Vector4<Algebra>;
  using Scalar = typename Algebra::Scalar;

  int ncellsX, ncellsY, ncellsZ;
  ncellsX = ncellsY = ncellsZ = num_cells;

  double mcMaxX = Algebra::to_double(shape.get_max_boundaries().x());
  double mcMinX = Algebra::to_double(shape.get_min_boundaries().x());
  double mcMaxY = Algebra::to_double(shape.get_max_boundaries().y());
  double mcMinY = Algebra::to_double(shape.get_min_boundaries().y());
  double mcMaxZ = Algebra::to_double(shape.get_max_boundaries().z());
  double mcMinZ = Algebra::to_double(shape.get_min_boundaries().z());

  double gradX = (mcMaxX - mcMinX) / ncellsX / 2;
  double gradY = (mcMaxY - mcMinY) / ncellsY / 2;
  double gradZ = (mcMaxZ - mcMinZ) / ncellsZ / 2;

  double minValue = 0.;

  std::vector<Vector4> mcDataPoints((ncellsX + 1) * (ncellsY + 1) *
                                    (ncellsZ + 1));
  Vector3 stepSize((mcMaxX - mcMinX) / ncellsX, (mcMaxY - mcMinY) / ncellsY,
                   (mcMaxZ - mcMinZ) / ncellsZ);

  int YtimesZ = (ncellsY + 1) * (ncellsZ + 1);  // for extra speed
  for (int i = 0; i < ncellsX + 1; i++) {
    int ni = i * YtimesZ;  // for speed
    double vertX = mcMinX + i * stepSize.x();
    for (int j = 0; j < ncellsY + 1; j++) {
      int nj = j * (ncellsZ + 1);  // for speed
      double vertY = mcMinY + j * stepSize.y();
      for (int k = 0; k < ncellsZ + 1; k++) {
        Vector4 vert(vertX, vertY, mcMinZ + k * stepSize.z(), 0);
        vert.val = shape.distance(vert.vec);
        mcDataPoints[ni + nj + k] = vert;
      }
    }
  }

  // Run the optimized Marching Cubes on the data
  MC::MCShape<Algebra> mesh =
      MarchingCubes(ncellsX, ncellsY, ncellsZ, gradX, gradY, gradZ, minValue,
                    mcDataPoints.data());

  RenderShape render_mesh;
  const double DIFF = 1e-4;

  // Compute the mean of all vertices
  Scalar mean_x = Algebra::zero();
  Scalar mean_y = Algebra::zero();
  Scalar mean_z = Algebra::zero();

  for (const auto &v : mesh.vertices) {
    mean_x = mean_x + v.p.x();
    mean_y = mean_y + v.p.y();
    mean_z = mean_z + v.p.z();
  }

  mean_x = mean_x / mesh.vertices.size();
  mean_y = mean_y / mesh.vertices.size();
  mean_z = mean_z / mesh.vertices.size();

  Vector3 mean_origin(mean_x, mean_y, mean_z);

  for (const auto &v : mesh.vertices) {
    GfxVertexFormat1 vgl;
    vgl.x = Algebra::to_double(v.p.x());
    vgl.y = Algebra::to_double(v.p.y());
    vgl.z = Algebra::to_double(v.p.z());
    vgl.nx = Algebra::to_double(v.norm.x());
    vgl.ny = Algebra::to_double(v.norm.y());
    vgl.nz = Algebra::to_double(v.norm.z());
    vgl.w = 1.;

    // Algebra::normalize is not supported by EigenAlgebra::Vector3
    // Vector3 d_normal = Algebra::normalize(v.p - mean_origin);
    Vector3 distance = v.p - mean_origin;
    Vector3 d_normal = distance * (Algebra::one() / Algebra::norm(distance));
    vgl.u =
        0.5 + Algebra::to_double(Algebra::atan2(d_normal.x(), d_normal.y()) /
                                 (2 * Algebra::pi()));
    vgl.v =
        0.5 - Algebra::to_double(Algebra::asin(d_normal.z()) / Algebra::pi());

    // Debug: Check if u or v is out of bounds
    // if (vgl.u > 1 || vgl.u < 0 || vgl.v > 1 || vgl.v < 0) {
    //   printf("u: %f, v: %f\n", vgl.u, vgl.v);
    // }

    if (compute_normals) {
      auto x = v.p.x();
      auto y = v.p.y();
      auto z = v.p.z();

      Vector3 x_upper(x + DIFF, y, z);
      Vector3 x_lower(x - DIFF, y, z);
      auto x_diff = shape.distance(x_upper) - shape.distance(x_lower);

      Vector3 y_upper(x, y + DIFF, z);
      Vector3 y_lower(x, y - DIFF, z);
      auto y_diff = shape.distance(y_upper) - shape.distance(y_lower);

      Vector3 z_upper(x, y, z + DIFF);
      Vector3 z_lower(x, y, z - DIFF);
      auto z_diff = shape.distance(z_upper) - shape.distance(z_lower);

      auto norm =
          std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);

      Vector3 normals(x_diff / norm, y_diff / norm, z_diff / norm);
      vgl.nx = normals.x();
      vgl.ny = normals.y();
      vgl.nz = normals.z();
    }

    render_mesh.vertices.push_back(vgl);
  }

  for (const auto &t : mesh.index_triangles) {
    render_mesh.indices.push_back(t[0]);
    render_mesh.indices.push_back(t[1]);
    render_mesh.indices.push_back(t[2]);
  }

  render_mesh.num_vertices = mesh.vertices.size();
  render_mesh.num_triangles = mesh.index_triangles.size();

  return render_mesh;
}
}  // namespace tds