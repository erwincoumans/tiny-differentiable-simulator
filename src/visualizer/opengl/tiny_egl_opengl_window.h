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

#ifndef TINY_EGL_OPENGL_WINDOW_H
#define TINY_EGL_OPENGL_WINDOW_H

#include "tiny_window_interface.h"

class EGLOpenGLWindow : public TinyWindowInterface {
  struct EGLInternalData2* m_data;
  bool m_OpenGLInitialized;
  bool m_requestedExit;

 public:
  EGLOpenGLWindow();
  virtual ~EGLOpenGLWindow();

  virtual void create_default_window(int width, int height, const char* title) {
    TinyWindowConstructionInfo ci(width, height);
    ci.m_title = title;
    create_window(ci);
  }

  virtual void create_window(const TinyWindowConstructionInfo& ci);

  virtual void close_window();

  virtual void run_main_loop();
  virtual float get_time_in_seconds();

  virtual bool requested_exit() const;
  virtual void set_request_exit();

  virtual void pump_messages() {}
  
  virtual void start_rendering();

  virtual void end_rendering();

  virtual bool is_modifier_key_pressed(int key);

  virtual void set_mouse_move_callback(TinyMouseMoveCallback mouseCallback);
  virtual TinyMouseMoveCallback get_mouse_move_callback();

  virtual void set_mouse_button_callback(TinyMouseButtonCallback mouseCallback);
  virtual TinyMouseButtonCallback get_mouse_button_callback();

  virtual void set_resize_callback(TinyResizeCallback resizeCallback);
  virtual TinyResizeCallback get_resize_callback();

  virtual void set_wheel_callback(TinyWheelCallback wheelCallback);
  virtual TinyWheelCallback get_wheel_callback();

  virtual void set_keyboard_callback(TinyKeyboardCallback keyboardCallback);
  virtual TinyKeyboardCallback get_keyboard_callback();

  virtual void set_render_callback(TinyRenderCallback renderCallback);

  virtual void set_window_title(const char* title);

  virtual float get_retina_scale() const;
  virtual void set_allow_retina(bool allow);

  virtual int get_width() const;
  virtual int get_height() const;

  virtual int file_open_dialog(char* fileName, int maxFileNameLength);
};

#endif  // TINY_EGL_OPENGL_WINDOW_H
