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

#ifndef GL_RENDER_TO_TEXTURE_H
#define GL_RENDER_TO_TEXTURE_H

/// See
/// http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/
#include "tiny_opengl_include.h"

enum {
  RENDERTEXTURE_COLOR = 1,
  RENDERTEXTURE_DEPTH,
};
struct GLRenderToTexture {
  GLuint m_framebufferName;
  GLuint m_depthrenderbuffer;
  GLuint m_renderTextureId;
  bool m_initialized;
  int m_renderTextureType;
  int m_width;
  int m_height;

 public:
  GLRenderToTexture();

  virtual ~GLRenderToTexture();

  void init(int width, int height, GLuint textureId,
            int renderTextureType = RENDERTEXTURE_COLOR);
  bool enable();
  void disable();
};

#endif  // GL_RENDER_TO_TEXTURE_H
