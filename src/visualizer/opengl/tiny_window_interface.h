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

#ifndef TINY_WINDOW_INTERFACE_H
#define TINY_WINDOW_INTERFACE_H

#include "tiny_common_callbacks.h"
#include <functional>

struct TinyWindowConstructionInfo {
  int m_width;
  int m_height;
  bool m_fullscreen;
  int m_colorBitsPerPixel;
  void* m_windowHandle;
  const char* m_title;
  int m_openglVersion;
  int m_renderDevice;

  TinyWindowConstructionInfo(int width = 1024, int height = 768)
      : m_width(width),
        m_height(height),
        m_fullscreen(false),
        m_colorBitsPerPixel(32),
        m_windowHandle(0),
        m_title("title"),
        m_openglVersion(3),
        m_renderDevice(-1) {}
};

class TinyWindowInterface {
 public:
 

  virtual void create_default_window(int width, int height, const char* title) {
    TinyWindowConstructionInfo ci(width, height);
    ci.m_title = title;
    create_window(ci);
  }

  virtual void create_window(const TinyWindowConstructionInfo& ci) = 0;

  virtual void close_window() = 0;

  virtual void run_main_loop() = 0;
  virtual float get_time_in_seconds() = 0;

  virtual bool requested_exit() const = 0;
  virtual void set_request_exit() = 0;
  virtual void set_request_exit2()
  {
      set_request_exit();

      set_keyboard_callback2(0);
      set_wheel_callback2(0);
      set_resize_callback2(0);
      set_mouse_button_callback2(0);
      set_mouse_move_callback2(0);
      
  }

  virtual void start_rendering() = 0;

  virtual void pump_messages() = 0;

  virtual void end_rendering() = 0;

  virtual bool is_modifier_key_pressed(int key) = 0;

  virtual void set_mouse_move_callback(TinyMouseMoveCallback mouseCallback) = 0;
  virtual TinyMouseMoveCallback get_mouse_move_callback() = 0;

  virtual void set_mouse_button_callback(
      TinyMouseButtonCallback mouseCallback) = 0;
  virtual TinyMouseButtonCallback get_mouse_button_callback() = 0;

  virtual void set_resize_callback(TinyResizeCallback resizeCallback) = 0;
  virtual TinyResizeCallback get_resize_callback() = 0;

  virtual void set_wheel_callback(TinyWheelCallback wheelCallback) = 0;
  virtual TinyWheelCallback get_wheel_callback() = 0;

  

    static inline std::function<void(int, int)> s_keyboard_callback;

    static void bridge_keyboard_callback(int a, int b)
    {
        if (s_keyboard_callback)
            s_keyboard_callback(a,b);
    }

    void set_keyboard_callback2(std::function<void(int, int)> callback)
    {
        s_keyboard_callback = callback;
        set_keyboard_callback(bridge_keyboard_callback);
    }

    static inline std::function<void(float, float)> s_mouse_move_callback;

    static void bridge_mouse_move_callback(float a, float b)
    {
        if (s_mouse_move_callback)
            s_mouse_move_callback(a,b);
    }


    void set_mouse_move_callback2(std::function<void(float, float)> callback)
    {
        s_mouse_move_callback = callback;
        set_mouse_move_callback( bridge_mouse_move_callback);
    }

    static inline std::function<void(int,int,float, float)> s_mouse_button_callback;

    static void bridge_mouse_button_callback(int a,int b,float c, float d)
    {
        if (s_mouse_button_callback)
            s_mouse_button_callback(a,b,c,d);
    }

    void set_mouse_button_callback2(std::function<void(int,int,float, float)> callback)
    {
        s_mouse_button_callback = callback;
        set_mouse_button_callback(bridge_mouse_button_callback);
    }


    static inline std::function<void(float, float)> s_resize_callback;

    static void bridge_resize_callback(float a, float b)
    {
        if (s_resize_callback)
        {
            s_resize_callback(a,b);
        }
    }

    void set_resize_callback2(std::function<void(float, float)> callback)
    {
        s_resize_callback = callback;
        set_resize_callback( bridge_resize_callback);
    }


    static inline std::function<void(float, float)> s_wheel_callback;

    static void bridge_wheel_callback(float a, float b)
    {
        if (s_wheel_callback)
            s_wheel_callback(a,b);
    }

    void set_wheel_callback2(std::function<void(float, float)> callback)
    {
        s_wheel_callback = callback;
        set_wheel_callback(bridge_wheel_callback);
    }
  
  virtual ~TinyWindowInterface() {
  }

  virtual void set_keyboard_callback(TinyKeyboardCallback keyboardCallback) = 0;
  virtual TinyKeyboardCallback get_keyboard_callback() = 0;

  virtual void set_render_callback(TinyRenderCallback renderCallback) = 0;

  virtual void set_window_title(const char* title) = 0;

  virtual float get_retina_scale() const = 0;
  virtual void set_allow_retina(bool allow) = 0;

  virtual int get_width() const = 0;
  virtual int get_height() const = 0;

  virtual int file_open_dialog(char* fileName, int maxFileNameLength) = 0;
};

#endif  // TINY_WINDOW_INTERFACE_H
