
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include "visualizer/opengl/utils/tiny_logging.h"
#include "tiny_obj_loader.h"
#include "utils/file_utils.hpp"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"
#include "stb_image/stb_image.h"

using namespace TINY;

int main(int argc, char* argv[]) {
  TinyChromeUtilsStartTimings();

  TinyOpenGL3App app("test", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);

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
  for (int i = 0; i < texWidth * texHeight * 3; i++) texels[i] = 255;

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

  textureIndex = app.m_instancingRenderer->register_texture(
      &texels[0], texWidth, texHeight);

  int shape = app.register_cube_shape(10, 10, 0.01, textureIndex, 40);  //, 10);
  
  int inst = app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);
  //app.m_renderer->write_single_instance_color_to_cpu(color, inst);
  pos.setValue(0, 0, 1);
  shape = app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
  float opacity = 0.7;
  app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling, opacity);


  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  pos.setValue(0, -2, 1);
  //test Wavefront obj loading
  std::string warn;
  std::string err;
  char basepath[1024];
  bool triangulate = true;
  std::string laikago_urdf_filename;
  ::tds::FileUtils::find_file("laikago/chassis_zup.obj", laikago_urdf_filename);
  ::tds::FileUtils::extract_path(laikago_urdf_filename.c_str(), basepath, 1024);
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, laikago_urdf_filename.c_str(),
      basepath, triangulate);

  std::vector<int> instance_ids;
  for (int i = 0; i < shapes.size(); i++)
  {
      std::vector<int> indices;
      std::vector<GfxVertexFormat1> vertices;
      TinyMeshUtils::extract_shape(attrib, shapes[i], materials, indices, vertices, textureIndex);
      textureIndex = -1;
      color.setValue(1, 1, 1);
      if (shapes[i].mesh.material_ids.size())
      {
          const tinyobj::material_t& mat = materials[shapes[i].mesh.material_ids[0]];
          color.setValue(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
          if (mat.diffuse_texname.length())
          {
              std::string texture_file_name = std::string(basepath) + mat.diffuse_texname;
              std::vector< unsigned char> buffer;
              int width, height, n;
              unsigned char* image = stbi_load(texture_file_name.c_str(), &width, &height, &n, 3);
              
              textureIndex = app.m_renderer->register_texture(image, width, height);
              free(image);
          }
      }
      shape = app.m_renderer->register_shape(&vertices[0].x, vertices.size(), &indices[0], indices.size(), B3_GL_TRIANGLES, textureIndex);
      int instance_id = app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);
      instance_ids.push_back(instance_id);
  }

  app.m_renderer->write_transforms();
  while (!app.m_window->requested_exit()) {
    B3_PROFILE("mainloop");
    int upAxis = 2;

    app.m_renderer->update_camera(upAxis);
    DrawGridData data;
    data.upAxis = 2;

    {
      // B3_PROFILE("draw_grid");
      app.draw_grid(data);
    }
    const char* bla = "3d label";
    app.draw_text_3d(bla, 0, 0, 1, 1);
    {
      B3_PROFILE("render_scene");
      app.m_renderer->render_scene();
    }

    {
      // B3_PROFILE("swap_buffer");
      app.swap_buffer();
    }
  }
  TinyChromeUtilsStopTimingsAndWriteJsonFile("diffsim.json");
}