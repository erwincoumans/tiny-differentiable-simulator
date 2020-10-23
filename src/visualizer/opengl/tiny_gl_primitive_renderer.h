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

#ifndef _GL_PRIMITIVE_RENDERER_H
#define _GL_PRIMITIVE_RENDERER_H

struct PrimVec2 {
  PrimVec2() {}
  PrimVec2(float x, float y) {
    p[0] = x;
    p[1] = y;
  }
  float p[2];
};

struct PrimVec4 {
  PrimVec4() {}
  PrimVec4(float x, float y, float z, float w) {
    p[0] = x;
    p[1] = y;
    p[2] = z;
    p[3] = w;
  }

  float p[4];
};

struct PrimVertex {
  PrimVertex(const PrimVec4& p, const PrimVec4& c, const PrimVec2& u)
      : position(p), colour(c), uv(u) {}

  PrimVertex() {}
  PrimVec4 position;
  PrimVec4 colour;
  PrimVec2 uv;
};

class TinyGLPrimitiveRenderer {
  int m_screenWidth;
  int m_screenHeight;

  struct PrimInternalData* m_data;
  struct PrimInternalData2* m_data2;
  void load_buffer_data();

 public:
  TinyGLPrimitiveRenderer(int screenWidth, int screenHeight);
  virtual ~TinyGLPrimitiveRenderer();

  void draw_rect(float x0, float y0, float x1, float y1, float color[4]);
  void draw_textured_tect(float x0, float y0, float x1, float y1,
                          float color[4], float u0, float v0, float u1,
                          float v1, int useRGBA = 0);
  void draw_textured_rect_3d(const PrimVertex& v0, const PrimVertex& v1,
                             const PrimVertex& v2, const PrimVertex& v3,
                             float viewMat[16], float projMat[16],
                             bool useRGBA = true);
  void draw_line();  // float from[4], float to[4], float color[4]);
  void set_screen_size(int width, int height);

  void draw_textured_rect2(float x0, float y0, float x1, float y1,
                           float color[4], float u0, float v0, float u1,
                           float v1, int useRGBA = 0);
  void draw_textured_rect2a(float x0, float y0, float x1, float y1,
                            float color[4], float u0, float v0, float u1,
                            float v1, int useRGBA = 0);
  void flushBatchedRects();

  void draw_textured_rect_3d2_text(bool useRGBA = true);
  void draw_textured_rect_3d2(PrimVertex* vertices, int numVertices,
                              bool useRGBA = true);

  PrimInternalData* get_data() { return m_data; }
};

#endif  //_GL_PRIMITIVE_RENDERER_H
