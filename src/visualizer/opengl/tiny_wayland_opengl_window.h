// Native Wayland window using an optional, dynamically loaded GLFW 3.4+.
#ifndef TINY_WAYLAND_OPENGL_WINDOW_H
#define TINY_WAYLAND_OPENGL_WINDOW_H
#include "tiny_window_interface.h"
#include <memory>
#include <string>

class TinyWaylandOpenGLWindow : public TinyWindowInterface {
  struct Data;
  std::unique_ptr<Data> m_data;
 public:
  explicit TinyWaylandOpenGLWindow(const char* glfwLibrary = nullptr);
  ~TinyWaylandOpenGLWindow() override;
  void create_window(const TinyWindowConstructionInfo& ci) override;
  void close_window() override;
  void start_rendering() override;
  void end_rendering() override;
  void pump_messages() override;
  void run_main_loop() override;
  float get_time_in_seconds() override;
  bool requested_exit() const override;
  void set_request_exit() override;
  bool set_vsync(bool enabled) override;
  bool is_modifier_key_pressed(int key) override;
  void set_window_title(const char* title) override;
  float get_retina_scale() const override;
  void set_allow_retina(bool allow) override;
  int get_width() const override;
  int get_height() const override;
  int file_open_dialog(char*, int) override { return 0; }
  void set_mouse_move_callback(TinyMouseMoveCallback cb) override;
  TinyMouseMoveCallback get_mouse_move_callback() override;
  void set_mouse_button_callback(TinyMouseButtonCallback cb) override;
  TinyMouseButtonCallback get_mouse_button_callback() override;
  void set_resize_callback(TinyResizeCallback cb) override;
  TinyResizeCallback get_resize_callback() override;
  void set_wheel_callback(TinyWheelCallback cb) override;
  TinyWheelCallback get_wheel_callback() override;
  void set_keyboard_callback(TinyKeyboardCallback cb) override;
  TinyKeyboardCallback get_keyboard_callback() override;
  void set_render_callback(TinyRenderCallback cb) override;
};
#endif
