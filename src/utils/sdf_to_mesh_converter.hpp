#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/make_mesh_3.h>

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "stb_image/stb_image.h"
#include "utils/file_utils.hpp"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include "visualizer/opengl/utils/tiny_logging.h"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"

struct CGALShape {
  std::vector<GfxVertexFormat1> vertices;
  std::vector<int> indices;

  size_t num_triangles;
  size_t num_vertices;
};

template <typename K, typename Tr>
struct SdfToMeshConverter {
  using FT = typename K::FT;
  using Point = typename K::Point_3;
  using Mesh_domain = CGAL::Labeled_mesh_domain_3<K>;
  typedef FT(Function)(const Point &);

#ifdef CGAL_CONCURRENT_MESH_3
  using Concurrency_tag = CGAL::Parallel_tag;
#else
  using Concurrency_tag = CGAL::Sequential_tag;
#endif

  using C3T3 = CGAL::Mesh_complex_3_in_triangulation_3<Tr>;
  using Mesh_criteria = CGAL::Mesh_criteria_3<Tr>;
  using Vertex_handle = typename C3T3::Vertex_handle;
  using Facet = typename C3T3::Facet;

  using Surface_mesh = CGAL::Surface_mesh<Point>;
  using vertex_descriptor =
      typename boost::graph_traits<Surface_mesh>::vertex_descriptor;

  Mesh_domain domain;
  Mesh_criteria criteria;
  C3T3 c3t3;
  Function *mesh_function;
  const double DIFF = 1e-4;

  SdfToMeshConverter(Function *mesh_func, const Mesh_domain &dom,
                     const Mesh_criteria &crit)
      : mesh_function(mesh_func), domain(dom), criteria(crit) {}

  const C3T3 &generate_mesh() {
    c3t3 = CGAL::make_mesh_3<C3T3>(domain, criteria);
    return c3t3;
  }

  const Tr &get_triangulation() const { return c3t3.triangulation(); }

  CGAL::Vector_3<K> compute_normal(const Point &p) {
    auto x = p.x();
    auto y = p.y();
    auto z = p.z();

    Point x_upper(x + DIFF, y, z);
    Point x_lower(x - DIFF, y, z);
    auto x_diff = mesh_function(x_upper) - mesh_function(x_lower);

    Point y_upper(x, y + DIFF, z);
    Point y_lower(x, y - DIFF, z);
    auto y_diff = mesh_function(y_upper) - mesh_function(y_lower);

    Point z_upper(x, y, z + DIFF);
    Point z_lower(x, y, z - DIFF);
    auto z_diff = mesh_function(z_upper) - mesh_function(z_lower);

    auto norm = std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);

    CGAL::Vector_3<K> result(x_diff / norm, y_diff / norm, z_diff / norm);
    return result;
  }

  CGALShape convert_to_shape() {
    this->generate_mesh();
    const Tr &tr = this->get_triangulation();

    CGALShape shape;
    std::unordered_map<Vertex_handle, int> V;
    int inum = 1;

    // Load Vertices
    auto vertices_num = tr.number_of_vertices();
    shape.num_vertices = vertices_num;

    for (auto vit = tr.finite_vertices_begin(); vit != tr.finite_vertices_end();
         ++vit) {
      V[vit] = inum++;
      auto p = tr.point(vit);

      GfxVertexFormat1 vertex_curr;
      vertex_curr.x = CGAL::to_double(p.x());
      vertex_curr.y = CGAL::to_double(p.y());
      vertex_curr.z = CGAL::to_double(p.z());
      vertex_curr.w = 1.;

      auto v_normal = compute_normal(p.point());
      vertex_curr.nx = CGAL::to_double(v_normal.x());
      vertex_curr.ny = CGAL::to_double(v_normal.y());
      vertex_curr.nz = CGAL::to_double(v_normal.z());
      vertex_curr.u = vertex_curr.v =
          0; //(double) inum / (double) vertices_num;

      shape.vertices.push_back(vertex_curr);
    }

    // Load triangles
    auto triangles_num = c3t3.number_of_facets_in_complex();

    shape.num_triangles = triangles_num;

    for (auto fit = c3t3.facets_in_complex_begin();
         fit != c3t3.facets_in_complex_end(); ++fit) {
      Facet f = (*fit);

      if (f.first->subdomain_index() >
          f.first->neighbor(f.second)->subdomain_index()) {
        f = tr.mirror_facet(f);
      }

      Vertex_handle vh1 = f.first->vertex((f.second + 1) % 4);
      Vertex_handle vh2 = f.first->vertex((f.second + 2) % 4);
      Vertex_handle vh3 = f.first->vertex((f.second + 3) % 4);

      auto v1 = shape.vertices[V[vh1] - 1];
      auto v2 = shape.vertices[V[vh2] - 1];
      auto v3 = shape.vertices[V[vh3] - 1];

      CGAL::Vector_3<K> v1_vec(v1.x, v1.y, v1.z);
      CGAL::Vector_3<K> v2_vec(v2.x, v2.y, v2.z);
      CGAL::Vector_3<K> v3_vec(v3.x, v3.y, v3.z);

      CGAL::Vector_3<K> v1_norm(v1.nx, v1.ny, v1.nz);
      CGAL::Vector_3<K> v2_norm(v2.nx, v2.ny, v2.nz);
      CGAL::Vector_3<K> v3_norm(v3.nx, v3.ny, v3.nz);

      auto side_normal = CGAL::cross_product(v2_vec - v1_vec, v3_vec - v2_vec);
      auto face_normal = (v1_norm + v2_norm + v3_norm) / 3;

      if (face_normal * side_normal < 0) {
        std::swap(vh1, vh3);
      }

      shape.indices.push_back(V[vh1] - 1);
      shape.indices.push_back(V[vh2] - 1);
      shape.indices.push_back(V[vh3] - 1);
    }

    return shape;
  }

  void output_to_medit(std::ostream &output) { c3t3.output_to_medit(output); }
};