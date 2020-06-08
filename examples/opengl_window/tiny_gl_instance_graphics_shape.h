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

#ifndef TINY_GL_INSTANCE_GRAPHICS_SHAPE_H
#define TINY_GL_INSTANCE_GRAPHICS_SHAPE_H
#include <vector>

struct GLInstanceVertex {
  float xyzw[4];
  float normal[3];
  float uv[2];
};
struct TinyGLInstanceGraphicsShape {
  std::vector<GLInstanceVertex>* m_vertices;
  int m_numvertices;
  std::vector<int>* m_indices;
  int m_numIndices;
  float m_scaling[4];

  TinyGLInstanceGraphicsShape() : m_vertices(0), m_indices(0) {}

  virtual ~TinyGLInstanceGraphicsShape() {
    delete m_vertices;
    delete m_indices;
  }
};

#endif  // TINY_GL_INSTANCE_GRAPHICS_SHAPE_H
