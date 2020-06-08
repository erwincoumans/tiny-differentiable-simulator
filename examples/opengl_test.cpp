
#include "opengl_window/tiny_opengl3_app.h"
#include "utils/tiny_chrome_trace_util.h"
#include "utils/tiny_logging.h"

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
  app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);
  pos.setValue(0, 0, 2);
  shape = app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
  app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);

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