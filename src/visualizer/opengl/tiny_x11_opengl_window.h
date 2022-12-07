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

#ifndef TINY_X11_OPENGL_WINDOW_H
#define TINY_X11_OPENGL_WINDOW_H

#define TinyDefaultOpenGLWindow TinyX11OpenGLWindow

#include "tiny_window_interface.h"

class TinyX11OpenGLWindow : public TinyWindowInterface {
  struct InternalData2* m_data;
  bool m_OpenGLInitialized;
  bool m_requestedExit;

 protected:
  void enable_opengl();

  void disable_opengl();

  void pump_message();

  int get_ascii_code_from_virtual_keycode(int orgCode);

 public:
  TinyX11OpenGLWindow();

  virtual ~TinyX11OpenGLWindow();

  virtual void create_window(const TinyWindowConstructionInfo& ci);

  virtual void close_window();

  virtual void start_rendering();
  
  virtual void pump_messages();

  virtual void render_all_objects();

  virtual void end_rendering();

  virtual float get_retina_scale() const { return 1.f; }
  virtual void set_allow_retina(bool /*allowRetina*/){};

  virtual void run_main_loop();
  virtual float get_time_in_seconds();

  virtual bool requested_exit() const;
  virtual void set_request_exit();

  virtual bool is_modifier_key_pressed(int key);

  virtual void set_mouse_move_callback(TinyMouseMoveCallback mouseCallback);
  virtual void set_mouse_button_callback(TinyMouseButtonCallback mouseCallback);
  virtual void set_resize_callback(TinyResizeCallback resizeCallback);
  virtual void set_wheel_callback(TinyWheelCallback wheelCallback);
  virtual void set_keyboard_callback(TinyKeyboardCallback keyboardCallback);

  virtual TinyMouseMoveCallback get_mouse_move_callback();
  virtual TinyMouseButtonCallback get_mouse_button_callback();
  virtual TinyResizeCallback get_resize_callback();
  virtual TinyWheelCallback get_wheel_callback();
  virtual TinyKeyboardCallback get_keyboard_callback();

  virtual void set_render_callback(TinyRenderCallback renderCallback);

  virtual void set_window_title(const char* title);

  virtual int get_width() const;

  virtual int get_height() const;

  int file_open_dialog(char* filename, int maxNameLength);
};

#endif  // TINY_X11_OPENGL_WINDOW_H
