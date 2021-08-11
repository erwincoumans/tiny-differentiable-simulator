#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "geometry.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "stb_image/stb_image.h"
#include "tiny_obj_loader.h"
#include "utils/file_utils.hpp"
#include "utils/sdf_to_mesh/marching_cubes.hpp"
#include "utils/sdf_to_mesh_converter.hpp"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include "visualizer/opengl/utils/tiny_logging.h"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"

typedef TinyAlgebra<double, TINY::DoubleUtils> Algebra;
using Vector3 = typename Algebra::Vector3;
using namespace TINY;

float simple_squared_distance(Vector3 p1, Vector3 p2) {
  return sqrtf(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) +
               pow(p1.z() - p2.z(), 2));
}

double simple_sphere_function(Vector3 p) {
  return simple_squared_distance(p, Vector3::zero()) - 1.0;
}

MC::FORMULA<Algebra> sphere_function(simple_sphere_function);

int main(int argc, char **argv) {
  TinyOpenGL3App app("sdf_to_mesh_test", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);

  // Shapes that can be rendered
  tds::Sphere<Algebra> sphere(1.0);
  tds::Capsule<Algebra> capsule(0.1, 1.0);
  tds::Cylinder<Algebra> cylinder(.1, 2.0);
  tds::Plane<Algebra> plane;
  
  tds::RenderShape shape = tds::convert_sdf_to_mesh<Algebra>(cylinder, 100);

  TinyVector3f pos(0, 0, 0);
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

  pos.setValue(0, 0, 0);

  textureIndex = app.m_instancingRenderer->register_texture(
      &texels[0], texWidth, texHeight);

  int shape_id = app.m_renderer->register_shape(
      &shape.vertices[0].x, shape.vertices.size(), &shape.indices[0],
      shape.num_triangles * 3);
  int instance_id = app.m_renderer->register_graphics_instance(
      shape_id, pos, orn, color, scaling, 1.f);

  int upAxis = 1;
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
