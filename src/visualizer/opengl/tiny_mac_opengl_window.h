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

#ifndef TINY_MAC_OPENGL_WINDOW_H
#define TINY_MAC_OPENGL_WINDOW_H

#include "tiny_window_interface.h"

#define TinyDefaultOpenGLWindow MacOpenGLWindow

class MacOpenGLWindow : public TinyWindowInterface {
  struct MacOpenGLWindowInternalData* m_internalData;

 public:
  MacOpenGLWindow();
  virtual ~MacOpenGLWindow();

  void init(int width, int height, const char* windowTitle);

  void close_window();

  void start_rendering();
  
  virtual void pump_messages();

  void end_rendering();  // swap buffers

  virtual bool requested_exit() const;

  virtual void set_request_exit();

  void get_mouse_coordinates(int& x, int& y);

  void run_main_loop();

  virtual bool is_modifier_key_pressed(int key);

  void set_mouse_button_callback(TinyMouseButtonCallback mouseCallback);

  void set_mouse_move_callback(TinyMouseMoveCallback mouseCallback);

  void set_resize_callback(TinyResizeCallback resizeCallback);

  void set_keyboard_callback(TinyKeyboardCallback keyboardCallback);

  virtual TinyMouseMoveCallback get_mouse_move_callback();

  virtual TinyMouseButtonCallback get_mouse_button_callback();

  virtual TinyResizeCallback get_resize_callback();

  virtual TinyWheelCallback get_wheel_callback();

  TinyKeyboardCallback get_keyboard_callback();

  void set_wheel_callback(TinyWheelCallback wheelCallback);

  float get_retina_scale() const;

  virtual void set_allow_retina(bool allow);

  virtual void create_window(const TinyWindowConstructionInfo& ci);

  virtual float get_time_in_seconds();

  virtual int get_width() const;
  virtual int get_height() const;

  virtual void set_render_callback(TinyRenderCallback renderCallback);

  virtual void set_window_title(const char* title);

  int file_open_dialog(char* filename, int maxNameLength);
};

#endif  // TINY_MAC_OPENGL_WINDOW_H
