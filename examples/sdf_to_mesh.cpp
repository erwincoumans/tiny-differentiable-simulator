#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "stb_image/stb_image.h"
#include "tiny_obj_loader.h"
#include "utils/file_utils.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include "visualizer/opengl/utils/tiny_logging.h"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"
#include "geometry.hpp"

#include "utils/sdf_to_mesh/marching_cubes.hpp"

typedef TinyAlgebra<double, TINY::DoubleUtils> Algebra;
using Vector3 = typename Algebra::Vector3;
using namespace TINY;

#define BOUND 3.0
#define NCELLS 50
#define GRAD 0.06
#define MINVAL 0.0

struct CGALShape
{
  std::vector<GfxVertexFormat1> vertices;
  std::vector<int> indices;

  size_t num_triangles;
  size_t num_vertices;
};

float simple_squared_distance(Vector3 p1, Vector3 p2)
{
  return sqrtf(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
}

double simple_sphere_function(Vector3 p)
{
  return simple_squared_distance(p, Vector3::zero()) - 1.0;
}

FORMULA<Algebra> sphere_function(simple_sphere_function);

int main(int argc, char **argv)
{
  TinyOpenGL3App app("sdf_to_mesh_test", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);

  // Create a shape from the simple Marching Cubes algorithm
  float minX = -BOUND;
  float maxX = BOUND;
  float minY = -BOUND;
  float maxY = BOUND;
  float minZ = -BOUND;
  float maxZ = BOUND;

  int ncellsX = NCELLS;
  int ncellsY = NCELLS;
  int ncellsZ = NCELLS;

  float gradX = GRAD;
  float gradY = GRAD;
  float gradZ = GRAD;

  float minValue = MINVAL;

  MCShape<Algebra> result = MarchingCubes<Algebra>(minX, maxX, minY, maxY, minZ, maxZ,
                                                   ncellsX, ncellsY, ncellsZ,
                                                   gradX, gradY, gradZ, minValue,
                                                   sphere_function);

  CGALShape shape;
  for (const auto &v : result.vertices)
  {
    GfxVertexFormat1 vgl;
    vgl.x = v.p.x();
    vgl.y = v.p.y();
    vgl.z = v.p.z();
    vgl.nx = v.norm.x();
    vgl.ny = v.norm.y();
    vgl.nz = v.norm.z();
    vgl.w = 1.;
    vgl.u = vgl.v = 0.;

    shape.vertices.push_back(vgl);
  }

  for (const auto &t : result.index_triangles)
  {
    shape.indices.push_back(t[0]);
    shape.indices.push_back(t[1]);
    shape.indices.push_back(t[2]);
  }

  shape.num_vertices = result.vertices.size();
  shape.num_triangles = result.index_triangles.size();

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

  for (int i = 0; i < texWidth; i++)
  {
    for (int j = 0; j < texHeight; j++)
    {
      int a = i < texWidth / 2 ? 1 : 0;
      int b = j < texWidth / 2 ? 1 : 0;

      if (a == b)
      {
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
  while (!app.m_window->requested_exit())
  {
    app.m_renderer->update_camera(upAxis);
    {
      DrawGridData data;
      data.upAxis = upAxis;
      app.draw_grid(data);
    }
    {
      app.m_renderer->render_scene();
    }

    {
      app.swap_buffer();
    }
  }

  return 0;
}
