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

#ifndef TINY_OPENGL_FONTSTASH_CALLBACKS_H
#define TINY_OPENGL_FONTSTASH_CALLBACKS_H

#include "tiny_font_stash.h"
struct PrimInternalData;
class TinyGLPrimitiveRenderer;

struct InternalOpenGL2RenderCallbacks : public RenderCallbacks {
  virtual PrimInternalData* getData() = 0;

  virtual ~InternalOpenGL2RenderCallbacks();

  virtual void update_texture(sth_texture* texture, sth_glyph* glyph,
                              int textureWidth, int textureHeight);
  virtual void render(sth_texture* texture);

  void display2();
};

void dumpTextureToPng(int screenWidth, int screenHeight, const char* fileName);

struct SimpleOpenGL2RenderCallbacks : public InternalOpenGL2RenderCallbacks {
  PrimInternalData* m_data;
  virtual PrimInternalData* getData() { return m_data; }
  SimpleOpenGL2RenderCallbacks(PrimInternalData* data) : m_data(data) {}
  virtual ~SimpleOpenGL2RenderCallbacks() {}
};

struct OpenGL2RenderCallbacks : public InternalOpenGL2RenderCallbacks {
  TinyGLPrimitiveRenderer* m_primRender2;
  virtual PrimInternalData* getData();

  virtual void set_world_position(float pos[3]) {}
  virtual void set_world_orientation(float orn[4]) {}
  virtual void set_color_rgba(float color[4]) {}

  OpenGL2RenderCallbacks(TinyGLPrimitiveRenderer* primRender);
  virtual ~OpenGL2RenderCallbacks();
};

#endif  // TINY_OPENGL_FONTSTASH_CALLBACKS_H
