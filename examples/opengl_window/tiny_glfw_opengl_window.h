// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TINY_GLFW_OPENGL_WINDOW_H
#define TINY_GLFW_OPENGL_WINDOW_H

#ifdef B3_USE_GLFW

#include "../CommonInterfaces/CommonWindowInterface.h"

#define b3gDefaultOpenGLWindow GLFWOpenGLWindow

class GLFWOpenGLWindow : public CommonWindowInterface {
  struct GLFWOpenGLWindowInternalData* m_data;

 protected:
 public:
  GLFWOpenGLWindow();

  virtual ~GLFWOpenGLWindow();

  virtual void create_default_window(int width, int height, const char* title);

  virtual void create_window(const b3gWindowConstructionInfo& ci);

  virtual void close_window();

  virtual void run_main_loop();
  virtual float get_time_in_seconds();

  virtual bool requested_exit() const;
  virtual void set_request_exit();

  virtual void start_rendering();

  virtual void end_rendering();

  virtual bool is_modifier_key_pressed(int key);

  virtual void set_mouse_move_callback(b3MouseMoveCallback mouseCallback);
  virtual b3MouseMoveCallback get_mouse_move_callback();

  virtual void set_mouse_button_callback(b3MouseButtonCallback mouseCallback);
  virtual b3MouseButtonCallback get_mouse_button_callback();

  virtual void set_resize_callback(b3ResizeCallback resizeCallback);
  virtual b3ResizeCallback getResizeCallback();

  virtual void set_wheel_callback(TinyWheelCallback wheelCallback);
  virtual TinyWheelCallback get_wheel_callback();

  virtual void set_keyboard_callback(b3KeyboardCallback keyboardCallback);
  virtual b3KeyboardCallback get_keyboard_callback();

  virtual void set_render_callback(b3RenderCallback renderCallback);

  virtual void set_window_title(const char* title);

  virtual float get_retina_scale() const;
  virtual void set_allow_retina(bool allow);

  virtual int get_width() const;
  virtual int get_height() const;

  virtual int file_open_dialog(char* fileName, int maxFileNameLength);

  void keyboardCallbackInternal(int key, int state);
  void mouseButtonCallbackInternal(int button, int state);
  void mouseCursorCallbackInternal(double xPos, double yPos);
  void resizeInternal(int width, int height);
};
#endif  // B3_USE_GLFW
#endif  // TINY_GLFW_OPENGL_WINDOW_H
