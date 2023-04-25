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

#ifndef NO_OPENGL3

/// See
/// http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-14-render-to-texture/

#include "tiny_gl_render_to_texture.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

bool gIntelLinuxglDrawBufferWorkaround = false;

GLRenderToTexture::GLRenderToTexture() : m_framebufferName(0) {
#if !defined(_WIN32) && !defined(__APPLE__)
  const GLubyte* ven = glGetString(GL_VENDOR);
  printf("ven = %s\n", ven);

  if (strncmp((const char*)ven, "Intel", 5) == 0) {
    printf(
        "Workaround for some crash in the Intel OpenGL driver on "
        "Linux/Ubuntu\n");
    gIntelLinuxglDrawBufferWorkaround = true;
  }
#endif  //! defined(_WIN32) && !defined(__APPLE__)

  m_width = -1;
  m_height = -1;
}

void GLRenderToTexture::init(int width, int height, GLuint textureId,
                             int renderTextureType) {
  m_renderTextureType = renderTextureType;
  m_width = width;
  m_height = height;

  glGenFramebuffers(1, &m_framebufferName);
  glBindFramebuffer(GL_FRAMEBUFFER, m_framebufferName);


  switch (m_renderTextureType) {
    case RENDERTEXTURE_COLOR: {
      glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textureId, 0);

        // The depth buffer
      glGenRenderbuffers(1, &m_depthrenderbuffer);

      glBindRenderbuffer(GL_RENDERBUFFER, m_depthrenderbuffer);
      glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,width,height); 	
      glFramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthrenderbuffer);

      break;
    }
    case RENDERTEXTURE_DEPTH: {
      glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, textureId, 0);
      break;
    }
    default: { assert(0); }
  };

  glBindFramebuffer(GL_FRAMEBUFFER, 0);


}

bool GLRenderToTexture::enable() {
  bool status = false;

  glBindFramebuffer(GL_FRAMEBUFFER, m_framebufferName);

  switch (m_renderTextureType) {
    case RENDERTEXTURE_COLOR: {
      // Set the list of draw buffers.
      GLenum drawBuffers[2] = {GL_COLOR_ATTACHMENT0, 0};
      glDrawBuffers(1, drawBuffers);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |
              GL_STENCIL_BUFFER_BIT);

      // glCullFace(GL_BACK);
      // glFrontFace(GL_CCW);
      glEnable(GL_DEPTH_TEST);

      break;
    }
    case RENDERTEXTURE_DEPTH: {
      // Intel OpenGL driver crashes when using GL_NONE for glDrawBuffer on
      // Linux, so use a workaround
      if (gIntelLinuxglDrawBufferWorkaround) {
        GLenum drawBuffers[2] = {GL_COLOR_ATTACHMENT0, 0};
        glDrawBuffers(1, drawBuffers);
      } else {
        glDrawBuffer(GL_NONE);
      }
      break;
    }
    default: { assert(0); }
  };

  // Always check that our framebuffer is ok
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE) {
    status = true;
  }

  return status;
}

void GLRenderToTexture::disable() { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

GLRenderToTexture::~GLRenderToTexture() {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  if (m_depthrenderbuffer) {
    glDeleteRenderbuffers(1, &m_depthrenderbuffer);
  }

  if (m_framebufferName) {
    glDeleteFramebuffers(1, &m_framebufferName);
  }
}
#endif
