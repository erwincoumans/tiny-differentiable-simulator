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

#ifndef PRIM_INTERNAL_DATA
#define PRIM_INTERNAL_DATA

#include "tiny_opengl_include.h"

struct PrimInternalData {
  GLuint m_shaderProg;
  GLint m_viewmatUniform;
  GLint m_projMatUniform;
  GLint m_positionUniform;
  GLint m_colourAttribute;
  GLint m_positionAttribute;
  GLint m_textureAttribute;
  GLuint m_vertexBuffer;
  GLuint m_vertexBuffer2;

  GLuint m_vertexArrayObject;
  GLuint m_vertexArrayObject2;

  GLuint m_indexBuffer;
  GLuint m_indexBuffer2;
  GLuint m_texturehandle;
};

#endif  // PRIM_INTERNAL_DATA
