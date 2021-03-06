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
#include "tiny_obj_loader.h"
#include "utils/file_utils.hpp"
#include "utils/sdf_to_mesh_converter.hpp"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include "visualizer/opengl/utils/tiny_logging.h"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point;
typedef FT(Function)(const Point &);
typedef CGAL::Labeled_mesh_domain_3<K> Mesh_domain;

#ifdef CGAL_CONCURRENT_MESH_3
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

typedef CGAL::Mesh_triangulation_3<Mesh_domain, CGAL::Default,
                                   Concurrency_tag>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr> C3t3;
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;
using namespace CGAL::parameters;
using namespace TINY;

FT sphere_function(const Point &p) {
  return CGAL::squared_distance(p, Point(CGAL::ORIGIN)) - 1.0;
}

int main(int argc, char **argv) {
  TinyOpenGL3App app("sdf_to_mesh_test", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);

  Mesh_domain domain = Mesh_domain::create_implicit_mesh_domain(
      sphere_function, K::Sphere_3(CGAL::ORIGIN, 3.0));
  Mesh_criteria criteria(facet_angle = 30.0, facet_size = 0.3,
                         facet_distance = 0.025, cell_radius_edge = 2.0,
                         cell_size = 0.1);

  // Outputing mesh in medit format to a file
  // std::ofstream medit_file("out.mesh");
  // c3t3.output_to_medit(medit_file);

  // Directly reading into local data structures
  SdfToMeshConverter<K, Tr> mesh(sphere_function, domain, criteria);
  auto shape = mesh.convert_to_shape();

  TinyVector3f pos(0, 0, -0.02);
  TinyQuaternionf orn(0, 0, 0, 1);
  TinyVector3f color(1, 1, 1);
  TinyVector3f scaling(1, 1, 1);
  int textureIndex = -1;
  int red = 0;
  int green = 128;
  int blue = 255;

  int texWidth = 1024;
  int texHeight = 1024;

  std::vector<unsigned char> texels;
  texels.resize(texWidth * texHeight * 3);
  for (int i = 0; i < texWidth * texHeight * 3; i++)
    texels[i] = 255;

  for (int i = 0; i < texWidth; i++) {
    for (int j = 0; j < texHeight; j++) {
      int a = i < texWidth / 2 ? 1 : 0;
      int b = j < texWidth / 2 ? 1 : 0;

      if (a == b) {
        texels[(i + j * texWidth) * 3 + 0] = red;
        texels[(i + j * texWidth) * 3 + 1] = green;
        texels[(i + j * texWidth) * 3 + 2] = blue;
      }
    }
  }

  {
    // app.m_renderer->write_single_instance_color_to_cpu(color, inst);
    pos.setValue(1, 0, 0);
    int shape = app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
    float opacity = 0.7;
    app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling,
                                               opacity);
  }

  pos.setValue(0, 0, -0.02);

  textureIndex = app.m_instancingRenderer->register_texture(
      &texels[0], texWidth, texHeight);

  int shape_id = app.m_renderer->register_shape(
      &shape.vertices[0].x, shape.vertices.size(), &shape.indices[0],
      shape.num_triangles * 3);
  int instance_id = app.m_renderer->register_graphics_instance(
      shape_id, pos, orn, color, scaling, 1.f);

  int upAxis = 2;
  app.m_renderer->write_transforms();
  while (!app.m_window->requested_exit()) {
    app.m_renderer->update_camera(upAxis);
    {
      DrawGridData data;
      data.upAxis = upAxis;
      app.draw_grid(data);
    }
    { app.m_renderer->render_scene(); }

    { app.swap_buffer(); }
  }

  return 0;
}
