#include "opengl_window/tiny_opengl3_app.h"
#include "utils/tiny_chrome_trace_util.h"
#include "utils/tiny_logging.h"
#include "utils/tiny_clock.h"

#include "imgui.h"
//#include "imgui_impl_glfw.h"
#include "opengl3/imgui_impl_opengl3.h"

float g_MouseWheel = 0;
float gMouseX = 0;
float gMouseY = 0;
int g_MousePressed[3] = { 0,0,0 };


void MyMouseMoveCallback(float x, float y)
{
    gMouseX = x;
    gMouseY = y;
}
void MyMouseButtonCallback(int button, int state, float x, float y)
{
    gMouseX = x;
    gMouseY = y;
    g_MousePressed[button] = state;
}

int main(int argc, char* argv[]) {

  TinyOpenGL3App app("test", 1024, 768);
  app.m_renderer->init();
  
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

  textureIndex = -1;// app.m_instancingRenderer->register_texture(      &texels[0], texWidth, texHeight);
  int shape = app.register_cube_shape(1, 1, 1, textureIndex, 40);  //, 10);
  app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);
  app.m_renderer->write_transforms();
  while (!app.m_window->requested_exit()) {
      B3_PROFILE("mainloop");
      int upAxis = 2;
      app.m_renderer->update_camera(upAxis);
      app.draw_grid();
      app.m_renderer->render_scene();
      
      app.swap_buffer();
  }

#if 0

  TinyChromeUtilsStartTimings();
  TinyClock clock;
  clock.reset();
  double prev_time = clock.get_time_seconds();
  TinyOpenGL3App app("test", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);
  app.m_window->set_mouse_move_callback(MyMouseMoveCallback);
  app.m_window->set_mouse_button_callback(MyMouseButtonCallback);
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

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
  //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

  // Setup Dear ImGui style
  //ImGui::StyleColorsDark();
  ImGui::StyleColorsClassic();
  const char* glsl_version = "#version 330";
  
  // Setup Platform/Renderer bindings
  ImGui_ImplOpenGL3_Init(glsl_version);
  bool res = ImGui_ImplOpenGL3_CreateDeviceObjects();
  
  while (!app.m_window->requested_exit()) {
    B3_PROFILE("mainloop");
    int upAxis = 2;

    app.m_renderer->update_camera(upAxis);
    DrawGridData data;
    data.upAxis = 2;

    {
      // B3_PROFILE("draw_grid");
      //app.draw_grid(data);
    }
    const char* bla = "3d label";
    app.draw_text_3d(bla, 0, 0, 1, 1);
    {
      B3_PROFILE("render_scene");
      app.m_renderer->render_scene();
    }

    
    float width = (float) app.m_window->get_width();
    float height = (float) app.m_window->get_height();

    io.DisplaySize = ImVec2((float)width, (float)height);
    io.DisplayFramebufferScale = ImVec2(app.m_window->get_retina_scale(), app.m_window->get_retina_scale());
    double t = clock.get_time_seconds();
    float dt = t - prev_time;
    prev_time = t;
    io.DeltaTime = (float)dt;
    io.MousePos = ImVec2((float)gMouseX, (float)gMouseY);
    io.RenderDrawListsFn = ImGui_ImplOpenGL3_RenderDrawData;// ImGui_ImplBullet_RenderDrawLists;

    for (int i = 0; i < 3; i++)
    {
        io.MouseDown[i] = g_MousePressed[i];
    }
    io.MouseWheel = g_MouseWheel;
    
    ImGui::NewFrame();
    ImGui::Text("Hello, world!");
    static int counter = 0;
    if (ImGui::Button("Button"))  // Buttons return true when clicked (NB: most widgets return true when edited/activated)
        counter++;
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::ShowDemoWindow();
    ImGui::Render();
    ImGui::EndFrame();
    {
      // B3_PROFILE("swap_buffer");
      app.swap_buffer();
    }
  }
  TinyChromeUtilsStopTimingsAndWriteJsonFile("diffsim.json");
#endif
}
