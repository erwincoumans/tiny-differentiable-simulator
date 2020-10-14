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

#ifndef TINY_WIN32_OPENGL_WINDOW_H
#define TINY_WIN32_OPENGL_WINDOW_H

#include "tiny_win32_window.h"

#define TinyDefaultOpenGLWindow TinyWin32OpenGLWindow

class TinyWin32OpenGLWindow : public TinyWin32Window {
  bool m_OpenGLInitialized;

 protected:
  void enableOpenGL();

  void disableOpenGL();

 public:
  TinyWin32OpenGLWindow();

  virtual ~TinyWin32OpenGLWindow();

  virtual void create_window(const TinyWindowConstructionInfo& ci);

  virtual void close_window();

  virtual void start_rendering();

  virtual void renderAllObjects();

  virtual void end_rendering();

  virtual float get_retina_scale() const { return 1.f; }
  virtual void set_allow_retina(bool /*allowRetina*/){};

  virtual int get_width() const;
  virtual int get_height() const;

  virtual int file_open_dialog(char* fileName, int maxFileNameLength);
};

#endif  // TINY_WIN32_OPENGL_WINDOW_H
