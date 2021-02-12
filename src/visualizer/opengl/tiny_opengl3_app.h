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

#ifndef TINY_OPENGL3_APP_H
#define TINY_OPENGL3_APP_H

#include "tiny_common_graphics_app_interface.h"
#include "tiny_gl_instancing_renderer.h"
#include "tiny_gl_primitive_renderer.h"
#include "tiny_window_interface.h"

struct TinyOpenGL3App : public TinyCommonGraphicsApp {
  struct TinyOpenGL3AppInternalData* m_data;

  class TinyGLPrimitiveRenderer* m_primRenderer;
  class TinyGLInstancingRenderer* m_instancingRenderer;
  virtual void set_background_color(float red, float green, float blue);
  virtual void set_mp4_fps(int fps);

  TinyOpenGL3App(const char* title, int width, int height,
                 bool allowRetina = true, int windowType = 0,
                 int renderDevice = -1, int maxNumObjectCapacity = 128 * 1024,
                 int maxShapeCapacityInBytes = 128 * 1024 * 1024);

  virtual ~TinyOpenGL3App();

  virtual int register_cube_shape(float halfExtentsX = 1.f,
                                  float halfExtentsY = 1.f,
                                  float halfExtentsZ = 1.f,
                                  int textureIndex = -1,
                                  float textureScaling = 1);
  virtual int register_graphics_unit_sphere_shape(EnumSphereLevelOfDetail lod,
                                                  int textureId = -1);

  virtual int register_graphics_capsule_shape(float radius,float half_height,int up_axis,int textureId);

  virtual void register_grid(int xres, int yres, const ::TINY::TinyVector3f& color0,
                             const ::TINY::TinyVector3f& color1);
  void dump_next_frame_to_png(const char* pngFilename);
  void dump_frames_to_video(const char* mp4Filename);
  virtual void get_screen_pixels(unsigned char* rgbaBuffer,
                                 int bufferSizeInBytes, float* depthBuffer,
                                 int depthBufferSizeInBytes);
  virtual void set_viewport(int width, int height);

  void draw_grid(DrawGridData data);
  void draw_grid()
  {
      DrawGridData data = DrawGridData();
      draw_grid(data);
  }
  virtual void set_up_axis(int axis);
  virtual int get_up_axis() const;

  virtual void swap_buffer();
  virtual void draw_text(const char* txt, int posX, int posY, float size,
                         float colorRGBA[4]);
  virtual void draw_text_3d(const char* txt, float posX, float posZY,
                            float posZ, float size);
  virtual void draw_text_3d(const char* txt, float position[3],
                            float orientation[4], float color[4], float size,
                            int optionFlag);

  virtual void draw_textured_tect(float x0, float y0, float x1, float y1,
                                  float color[4], float u0, float v0, float u1,
                                  float v1, int useRGBA);
  struct sth_stash* get_font_stash();
};

#endif  // TINY_OPENGL3_APP_H
