/*
Copyright (c) 2012 Advanced Micro Devices, Inc.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use
of this software. Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
// Originally written by Erwin Coumans

#ifndef TINY_WIN32_WINDOW_H
#define TINY_WIN32_WINDOW_H

struct TinyWin32InternalData2;

#include "tiny_window_interface.h"

class TinyWin32Window : public TinyWindowInterface {
 protected:
  struct TinyWin32InternalData2* m_data;

  void pumpMessage();

 public:
  TinyWin32Window();

  virtual ~TinyWin32Window();

  virtual void create_window(const TinyWindowConstructionInfo& ci);

  virtual void switchFullScreen(bool fullscreen, int width = 0, int height = 0,
                                int colorBitsPerPixel = 0);

  virtual void close_window();

  virtual void run_main_loop();

  virtual void start_rendering();

  virtual void pump_messages();

  virtual void renderAllObjects();

  virtual void end_rendering();

  virtual float get_time_in_seconds();

  virtual void setDebugMessage(int x, int y, const char* message);

  virtual bool requested_exit() const;

  virtual void set_request_exit();

  virtual void getMouseCoordinates(int& x, int& y);

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

  virtual bool is_modifier_key_pressed(int key);
};

#endif  // TINY_WIN32_WINDOW_H
