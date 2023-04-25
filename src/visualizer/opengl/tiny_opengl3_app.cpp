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

#include "tiny_opengl3_app.h"
#include "tiny_shape_data.h"

#ifdef TINY_USE_EGL
#include "tiny_egl_opengl_window.h"
#else
#endif  // TINY_USE_EGL

#ifdef B3_USE_GLFW
#include "GLFWOpenGLWindow.h"
#else
#ifdef __APPLE__
#include "tiny_mac_opengl_window.h"
#else

#ifdef _WIN32
#include "tiny_win32_opengl_window.h"
#else
// let's cross the fingers it is Linux/X11
#include "tiny_x11_opengl_window.h"
#define BT_USE_X11  // for runtime backend selection, move to build?
#endif              //_WIN32
#endif              //__APPLE__
#endif              // B3_USE_GLFW

#include <stdio.h>

#include <assert.h>
#include <string.h>  //memset
#include <vector>
#include "math/tiny/tiny_float_utils.h"
#include "tiny_font_stash.h"
#include "tiny_fonts.h"
#include "tiny_gl_instancing_renderer.h"
#include "tiny_gl_primitive_renderer.h"
#include "tiny_gl_render_to_texture.h"
#include "tiny_opengl_fontstashcallbacks.h"
#include "tiny_gl_instance_renderer_internal_data.h"

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#endif  // _WIN32

using namespace TINY;

struct TinyOpenGL3AppInternalData {
  GLuint m_fontTextureId;
  GLuint m_largeFontTextureId;
  struct sth_stash* m_fontStash;
  struct sth_stash* m_fontStash2;

  RenderCallbacks* m_renderCallbacks;
  RenderCallbacks* m_renderCallbacks2;

  int m_droidRegular;
  int m_droidRegular2;
  int m_textureId;

  std::string m_frameDumpPngFileName;
  FILE* m_ffmpegFile;
  GLRenderToTexture* m_renderTexture;
  void* m_userPointer;
  int m_upAxis;  // y=1 or z=2 is supported
  int m_customViewPortWidth;
  int m_customViewPortHeight;
  int m_mp4Fps;

  void* m_cudaVboPointer;
  bool m_cudaVboRegistered;


  TinyOpenGL3AppInternalData()
      : m_fontTextureId(0),
        m_largeFontTextureId(0),
        m_fontStash(0),
        m_fontStash2(0),
        m_renderCallbacks(0),
        m_renderCallbacks2(0),
        m_droidRegular(0),
        m_droidRegular2(0),
        m_textureId(-1),
        m_ffmpegFile(0),
        m_renderTexture(0),
        m_userPointer(0),
        m_upAxis(1),
        m_customViewPortWidth(-1),
        m_customViewPortHeight(-1),
        m_mp4Fps(60),
        m_cudaVboPointer(0),
        m_cudaVboRegistered(false){}
};

static TinyOpenGL3App* gApp = 0;

static void SimpleResizeCallback(float widthf, float heightf) {
  int width = (int)widthf;
  int height = (int)heightf;
  if (gApp && gApp->m_instancingRenderer)
    gApp->m_instancingRenderer->resize(width, height);

  if (gApp && gApp->m_primRenderer)
    gApp->m_primRenderer->set_screen_size(width, height);
}

static void SimpleKeyboardCallback(int key, int state) {
  if (key == TINY_KEY_ESCAPE && gApp && gApp->m_window) {
    gApp->m_window->set_request_exit();
  } else {
    // gApp->defaultKeyboardCallback(key,state);
  }
}

void SimpleMouseButtonCallback(int button, int state, float x, float y) {
  gApp->defaultMouseButtonCallback(button, state, x, y);
}
void SimpleMouseMoveCallback(float x, float y) {
  gApp->defaultMouseMoveCallback(x, y);
}

void SimpleWheelCallback(float deltax, float deltay) {
  gApp->defaultWheelCallback(deltax, deltay);
}

static GLuint BindFont(const CTexFont* _Font) {
  GLuint TexID = 0;
  glGenTextures(1, &TexID);
  glBindTexture(GL_TEXTURE_2D, TexID);
  glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
  glPixelStorei(GL_UNPACK_LSB_FIRST, GL_FALSE);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
  glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, _Font->m_TexWidth, _Font->m_TexHeight,
               0, GL_RED, GL_UNSIGNED_BYTE, _Font->m_TexBytes);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glBindTexture(GL_TEXTURE_2D, 0);

  return TexID;
}

// static unsigned int s_indexData[INDEX_COUNT];
// static GLuint s_indexArrayObject, s_indexBuffer;
// static GLuint s_vertexArrayObject,s_vertexBuffer;

extern unsigned char OpenSansData[];

struct MyRenderCallbacks : public RenderCallbacks {
  TinyGLInstancingRenderer* m_instancingRenderer;

  std::vector<unsigned char> m_rgbaTexture;
  float m_color[4];
  float m_worldPosition[3];
  float m_worldOrientation[4];

  int m_textureIndex;

  MyRenderCallbacks(TinyGLInstancingRenderer* instancingRenderer)
      : m_instancingRenderer(instancingRenderer), m_textureIndex(-1) {
    for (int i = 0; i < 4; i++) {
      m_color[i] = 1;
      m_worldOrientation[i] = 0;
    }
    m_worldPosition[0] = 0;
    m_worldPosition[1] = 0;
    m_worldPosition[2] = 0;

    m_worldOrientation[0] = 0;
    m_worldOrientation[1] = 0;
    m_worldOrientation[2] = 0;
    m_worldOrientation[3] = 1;
  }
  virtual ~MyRenderCallbacks() { m_rgbaTexture.clear(); }

  virtual void set_world_position(float pos[3]) {
    for (int i = 0; i < 3; i++) {
      m_worldPosition[i] = pos[i];
    }
  }

  virtual void set_world_orientation(float orn[4]) {
    for (int i = 0; i < 4; i++) {
      m_worldOrientation[i] = orn[i];
    }
  }

  virtual void set_color_rgba(float color[4]) {
    for (int i = 0; i < 4; i++) {
      m_color[i] = color[i];
    }
  }
  virtual void update_texture(sth_texture* texture, sth_glyph* glyph,
                              int textureWidth, int textureHeight) {
    if (glyph) {
      m_rgbaTexture.resize(textureWidth * textureHeight * 3);
      for (int i = 0; i < textureWidth * textureHeight; i++) {
        m_rgbaTexture[i * 3 + 0] = texture->m_texels[i];
        m_rgbaTexture[i * 3 + 1] = texture->m_texels[i];
        m_rgbaTexture[i * 3 + 2] = texture->m_texels[i];
      }
      bool flipPixelsY = false;
      m_instancingRenderer->update_texture(m_textureIndex, &m_rgbaTexture[0],
                                           flipPixelsY);
    } else {
      if (textureWidth && textureHeight) {
        texture->m_texels =
            (unsigned char*)malloc(textureWidth * textureHeight);
        memset(texture->m_texels, 0, textureWidth * textureHeight);
        if (m_textureIndex < 0) {
          m_rgbaTexture.resize(textureWidth * textureHeight * 3);
          bool flipPixelsY = false;
          m_textureIndex = m_instancingRenderer->register_texture(
              &m_rgbaTexture[0], textureWidth, textureHeight, flipPixelsY);

          int strideInBytes = 9 * sizeof(float);
          int numVertices = sizeof(cube_vertices_textured) / strideInBytes;
          int numIndices = sizeof(cube_indices) / sizeof(int);

          float halfExtentsX = 1;
          float halfExtentsY = 1;
          float halfExtentsZ = 1;
          float textureScaling = 4;

          std::vector<GfxVertexFormat1> verts;
          verts.resize(numVertices);
          for (int i = 0; i < numVertices; i++) {
            verts[i].x = halfExtentsX * cube_vertices_textured[i * 9];
            verts[i].y = halfExtentsY * cube_vertices_textured[i * 9 + 1];
            verts[i].z = halfExtentsZ * cube_vertices_textured[i * 9 + 2];
            verts[i].w = cube_vertices_textured[i * 9 + 3];
            verts[i].nx = cube_vertices_textured[i * 9 + 4];
            verts[i].ny = cube_vertices_textured[i * 9 + 5];
            verts[i].nz = cube_vertices_textured[i * 9 + 6];
            verts[i].u = cube_vertices_textured[i * 9 + 7] * textureScaling;
            verts[i].v = cube_vertices_textured[i * 9 + 8] * textureScaling;
          }

          int shapeId = m_instancingRenderer->register_shape(
              &verts[0].x, numVertices, cube_indices, numIndices,
              B3_GL_TRIANGLES, m_textureIndex);
          TinyVector3f pos = TinyVector3f(0, 0, 0);
          TinyQuaternionf orn(0, 0, 0, 1);
          // b3Vector4 color = b3MakeVector4(1, 1, 1, 1);
          TinyVector3f scaling = TinyVector3f(.1, .1, .1);
          // m_instancingRenderer->register_graphics_instance(shapeId, pos, orn,
          // color, scaling);
          m_instancingRenderer->write_transforms();
        } else {
          assert(0);
        }
      } else {
        delete texture->m_texels;
        texture->m_texels = 0;
        // there is no m_instancingRenderer->freeTexture (yet), all textures are
        // released at reset/deletion of the renderer
      }
    }
  }
  virtual void render(sth_texture* texture) {
    int index = 0;

    float width = 1;
    std::vector<unsigned int> indices;
    indices.resize(texture->nverts);
    for (int i = 0; i < indices.size(); i++) {
      indices[i] = i;
    }

    m_instancingRenderer->draw_textured_triangle_mesh(
        m_worldPosition, m_worldOrientation,
        &texture->newverts[0].position.p[0], texture->nverts, &indices[0],
        indices.size(), m_color, m_textureIndex);
  }
};

static void printGLString(const char* name, GLenum s) {
  const char* v = (const char*)glGetString(s);
  printf("%s = %s\n", name, v);
}

bool sOpenGLVerbose = true;


TinyOpenGL3App::TinyOpenGL3App(const char* title, int width, int height,
                               bool allowRetina, int windowType,
                               int renderDevice, int maxNumObjectCapacity,
                               int maxShapeCapacityInBytes) {
  gApp = this;

  m_data = new TinyOpenGL3AppInternalData;

  if (windowType == 0) {
    m_window = new TinyDefaultOpenGLWindow();
  } else if (windowType == 1) {
#ifdef BT_USE_X11
    m_window = new TinyX11OpenGLWindow();
#else
    printf("X11 requires Linux. Loading default window instead. \n");
    m_window = new TinyDefaultOpenGLWindow();
#endif
  } else if (windowType == 2) {
#ifdef TINY_USE_EGL
    m_window = new EGLOpenGLWindow();
#else
    printf("EGL window requires compilation with BT_USE_EGL.\n");
    printf("Loading default window instead. \n");
    m_window = new TinyDefaultOpenGLWindow();
#endif
  } else {
    printf("Unknown window type %d must be (0=default, 1=X11, 2=EGL).\n",
           windowType);
    printf("Loading default window instead. \n");
    m_window = new TinyDefaultOpenGLWindow();
  }

  m_window->set_allow_retina(allowRetina);

  TinyWindowConstructionInfo ci;
  ci.m_title = title;
  ci.m_width = width;
  ci.m_height = height;
  ci.m_renderDevice = renderDevice;
  m_window->create_window(ci);

  m_window->set_window_title(title);

  assert(glGetError() == GL_NO_ERROR);

  {
    printGLString("Version", GL_VERSION);
    printGLString("Vendor", GL_VENDOR);
    printGLString("Renderer", GL_RENDERER);
  }

  glClearColor(m_backgroundColorRGB[0], m_backgroundColorRGB[1],
               m_backgroundColorRGB[2], 1.f);

  m_window->start_rendering();
  width = m_window->get_width();
  height = m_window->get_height();

  assert(glGetError() == GL_NO_ERROR);

  // gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

#ifdef USE_GLEW
#ifndef __APPLE__
#ifndef _WIN32
#ifndef B3_USE_GLFW
  // some Linux implementations need the 'glewExperimental' to be true
  glewExperimental = GL_TRUE;
#endif  // B3_USE_GLFW
#endif  //_WIN32

#ifndef B3_USE_GLFW
  if (glewInit() != GLEW_OK) exit(1);  // or handle the error in a nicer way
  if (!GLEW_VERSION_2_1)  // check that the machine supports the 2.1 API.
    exit(1);              // or handle the error in a nicer way
#endif                    // B3_USE_GLFW
#endif                    //__APPLE__
#endif                    // USE_GLEW

  glGetError();  // don't remove this call, it is needed for Ubuntu

  assert(glGetError() == GL_NO_ERROR);

  m_parameterInterface = 0;

  assert(glGetError() == GL_NO_ERROR);

  m_instancingRenderer = new TinyGLInstancingRenderer(maxNumObjectCapacity,
                                                      maxShapeCapacityInBytes);

  m_primRenderer = new TinyGLPrimitiveRenderer(width, height);

  m_renderer = m_instancingRenderer;
  m_window->set_resize_callback(SimpleResizeCallback);

  m_instancingRenderer->init();
  m_instancingRenderer->resize(width, height);
  m_primRenderer->set_screen_size(width, height);
  assert(glGetError() == GL_NO_ERROR);

  m_instancingRenderer->init_shaders();

  m_window->set_mouse_move_callback(SimpleMouseMoveCallback);
  m_window->set_mouse_button_callback(SimpleMouseButtonCallback);
  m_window->set_keyboard_callback(SimpleKeyboardCallback);
  m_window->set_wheel_callback(SimpleWheelCallback);

  TwGenerateDefaultFonts();
  m_data->m_fontTextureId = BindFont(g_DefaultNormalFont);
  m_data->m_largeFontTextureId = BindFont(g_DefaultLargeFont);

  {
    m_data->m_renderCallbacks = new OpenGL2RenderCallbacks(m_primRenderer);
    m_data->m_renderCallbacks2 = new MyRenderCallbacks(m_instancingRenderer);
    m_data->m_fontStash2 = sth_create(512, 512, m_data->m_renderCallbacks2);
    m_data->m_fontStash = sth_create(
        512, 512, m_data->m_renderCallbacks);  // 256,256);//,1024);//512,512);

    assert(glGetError() == GL_NO_ERROR);

    if (!m_data->m_fontStash) {
      printf("Could not create stash");
      // fprintf(stderr, "Could not create stash.\n");
    }

    if (!m_data->m_fontStash2) {
      printf("Could not create fontStash2");
    }

    unsigned char* data2 = OpenSansData;
    unsigned char* data = (unsigned char*)data2;
    if (!(m_data->m_droidRegular =
              sth_add_font_from_memory(m_data->m_fontStash, data))) {
      printf("error!\n");
    }
    if (!(m_data->m_droidRegular2 =
              sth_add_font_from_memory(m_data->m_fontStash2, data))) {
      printf("error!\n");
    }

    assert(glGetError() == GL_NO_ERROR);
  }
}

struct sth_stash* TinyOpenGL3App::get_font_stash() {
  return m_data->m_fontStash;
}

template <typename T>
inline int projectWorldCoordToScreen(T objx, T objy, T objz,
                                     const T modelMatrix[16],
                                     const T projMatrix[16],
                                     const int viewport[4], T* winx, T* winy,
                                     T* winz) {
  int i;
  T in2[4];
  T tmp[4];

  in2[0] = objx;
  in2[1] = objy;
  in2[2] = objz;
  in2[3] = T(1.0);

  for (i = 0; i < 4; i++) {
    tmp[i] = in2[0] * modelMatrix[0 * 4 + i] + in2[1] * modelMatrix[1 * 4 + i] +
             in2[2] * modelMatrix[2 * 4 + i] + in2[3] * modelMatrix[3 * 4 + i];
  }

  T out[4];
  for (i = 0; i < 4; i++) {
    out[i] = tmp[0] * projMatrix[0 * 4 + i] + tmp[1] * projMatrix[1 * 4 + i] +
             tmp[2] * projMatrix[2 * 4 + i] + tmp[3] * projMatrix[3 * 4 + i];
  }

  if (out[3] == T(0.0)) return 0;
  out[0] /= out[3];
  out[1] /= out[3];
  out[2] /= out[3];
  /* Map x, y and z to range 0-1 */
  out[0] = out[0] * T(0.5) + T(0.5);
  out[1] = out[1] * T(0.5) + T(0.5);
  out[2] = out[2] * T(0.5) + T(0.5);

  /* Map x,y to viewport */
  out[0] = out[0] * viewport[2] + viewport[0];
  out[1] = out[1] * viewport[3] + viewport[1];

  *winx = out[0];
  *winy = out[1];
  *winz = out[2];
  return 1;
}

void TinyOpenGL3App::draw_text_3d(const char* txt, float position[3],
                                  float orientation[4], float color[4],
                                  float size, int optionFlag) {
  float viewMat[16];
  float projMat[16];
  TinyCamera* cam = m_instancingRenderer->get_active_camera();

  cam->get_camera_view_matrix(viewMat);
  cam->get_camera_projection_matrix(projMat);

  // TinyVector3f camPos;
  // cam->get_camera_position(camPos);
  // TinyVector3f cp= TinyVector3f(camPos[0],camPos[2],camPos[1]);
  // TinyVector3f p = TinyVector3f(worldPosX,worldPosY,worldPosZ);
  // float dist = (cp-p).length();
  // float dv = 0;//dist/1000.f;
  //
  // printf("str = %s\n",unicodeText);

  float dx = 0;

  // int measureOnly=0;

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  int viewport[4] = {0, 0, m_instancingRenderer->get_screen_width(),
                     m_instancingRenderer->get_screen_height()};

  float posX = position[0];
  float posY = position[1];
  float posZ = position[2];
  float winx, winy, winz;

  if (optionFlag & TinyCommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera) {
    if (!projectWorldCoordToScreen(position[0], position[1], position[2],
                                   viewMat, projMat, viewport, &winx, &winy,
                                   &winz)) {
      return;
    }
    posX = winx;
    posY = m_instancingRenderer->get_screen_height() / 2 +
           (m_instancingRenderer->get_screen_height() / 2) - winy;
    posZ = 0.f;
  }

  if (optionFlag & TinyCommonGraphicsApp::eDrawText3D_TrueType) {
    bool measureOnly = false;

    float fontSize = 64;  // 512;//128;

    if (optionFlag & TinyCommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera) {
      sth_draw_text(m_data->m_fontStash, m_data->m_droidRegular, fontSize, posX,
                    posY, txt, &dx,
                    this->m_instancingRenderer->get_screen_width(),
                    this->m_instancingRenderer->get_screen_height(),
                    measureOnly, m_window->get_retina_scale(), color);
      sth_end_draw(m_data->m_fontStash);
      sth_flush_draw(m_data->m_fontStash);
    } else {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      m_data->m_renderCallbacks2->set_color_rgba(color);

      m_data->m_renderCallbacks2->set_world_position(position);
      m_data->m_renderCallbacks2->set_world_orientation(orientation);

      sth_draw_text3D(m_data->m_fontStash2, m_data->m_droidRegular2, fontSize,
                      0, 0, 0, txt, &dx, size, color, 0);
      sth_end_draw(m_data->m_fontStash2);
      sth_flush_draw(m_data->m_fontStash2);
      glDisable(GL_BLEND);
    }
  } else {
    // float width = 0.f;
    int pos = 0;
    // float color[]={0.2f,0.2,0.2f,1.f};
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_data->m_largeFontTextureId);

    // float width = r.x;
    // float extraSpacing = 0.;

    float startX = posX;
    float startY = posY + g_DefaultLargeFont->m_CharHeight * size;
    float z = position[2];  // 2.f*winz-1.f;//*(far

    if (optionFlag & TinyCommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera) {
      posX = winx;
      posY = m_instancingRenderer->get_screen_height() / 2 +
             (m_instancingRenderer->get_screen_height() / 2) - winy;
      z = 2.f * winz - 1.f;
      startY = posY - g_DefaultLargeFont->m_CharHeight * size;
    }

    while (txt[pos]) {
      int c = txt[pos];
      // r.h = g_DefaultNormalFont->m_CharHeight;
      // r.w = g_DefaultNormalFont->m_CharWidth[c]+extraSpacing;
      float endX = startX + g_DefaultLargeFont->m_CharWidth[c] * size;
      if (optionFlag & TinyCommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera) {
        endX = startX + g_DefaultLargeFont->m_CharWidth[c] * size;
      }
      float endY = posY;

      // float currentColor[]={1.f,1.,0.2f,1.f};

      //	m_primRenderer->draw_textured_tect(startX, startY, endX, endY,
      // currentColor,g_DefaultLargeFont->m_CharU0[c],g_DefaultLargeFont->m_CharV0[c],g_DefaultLargeFont->m_CharU1[c],g_DefaultLargeFont->m_CharV1[c]);
      float u0 = g_DefaultLargeFont->m_CharU0[c];
      float u1 = g_DefaultLargeFont->m_CharU1[c];
      float v0 = g_DefaultLargeFont->m_CharV0[c];
      float v1 = g_DefaultLargeFont->m_CharV1[c];
      // float color[4] =
      // {currentColor[0],currentColor[1],currentColor[2],currentColor[3]};
      float x0 = startX;
      float x1 = endX;
      float y0 = startY;
      float y1 = endY;
      int screenWidth = m_instancingRenderer->get_screen_width();
      int screenHeight = m_instancingRenderer->get_screen_height();

      float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
      if (optionFlag & TinyCommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera) {
        PrimVertex vertexData[4] = {
            PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(screenWidth),
                                1.f - 2.f * y0 / float(screenHeight), z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u0, v0)),
            PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(screenWidth),
                                1.f - 2.f * y1 / float(screenHeight), z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u0, v1)),
            PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(screenWidth),
                                1.f - 2.f * y1 / float(screenHeight), z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u1, v1)),
            PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(screenWidth),
                                1.f - 2.f * y0 / float(screenHeight), z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u1, v0))};
        m_primRenderer->draw_textured_rect_3d(vertexData[0], vertexData[1],
                                              vertexData[2], vertexData[3],
                                              identity, identity, false);
      } else {
        PrimVertex vertexData[4] = {
            PrimVertex(PrimVec4(x0, y0, z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u0, v0)),
            PrimVertex(PrimVec4(x0, y1, z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u0, v1)),
            PrimVertex(PrimVec4(x1, y1, z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u1, v1)),
            PrimVertex(PrimVec4(x1, y0, z, 1.f),
                       PrimVec4(color[0], color[1], color[2], color[3]),
                       PrimVec2(u1, v0))};

        m_primRenderer->draw_textured_rect_3d(vertexData[0], vertexData[1],
                                              vertexData[2], vertexData[3],
                                              viewMat, projMat, false);
      }
      // DrawTexturedRect(0,r,g_DefaultNormalFont->m_CharU0[c],g_DefaultNormalFont->m_CharV0[c],g_DefaultNormalFont->m_CharU1[c],g_DefaultNormalFont->m_CharV1[c]);
      //	DrawFilledRect(r);

      startX = endX;
      // startY = endY;

      pos++;
    }
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  glDisable(GL_BLEND);
}

void TinyOpenGL3App::draw_text_3d(const char* txt, float worldPosX,
                                  float worldPosY, float worldPosZ,
                                  float size1) {
  float position[3] = {worldPosX, worldPosY, worldPosZ};
  float orientation[4] = {0, 0, 0, 1};
  float color[4] = {0, 0, 0, 1};
  int optionFlags = TinyCommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera;
  draw_text_3d(txt, position, orientation, color, size1, optionFlags);
}

void TinyOpenGL3App::draw_text(const char* txt, int posXi, int posYi,
                               float size, float colorRGBA[4]) {
  float posX = (float)posXi;
  float posY = (float)posYi;

  //
  // printf("str = %s\n",unicodeText);

  float dx = 0;

  // int measureOnly=0;

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if (1)  // m_useTrueTypeFont)
  {
    bool measureOnly = false;

    float fontSize = 64 * size;  // 512;//128;
    sth_draw_text(m_data->m_fontStash, m_data->m_droidRegular, fontSize, posX,
                  posY, txt, &dx,
                  this->m_instancingRenderer->get_screen_width(),
                  this->m_instancingRenderer->get_screen_height(), measureOnly,
                  m_window->get_retina_scale(), colorRGBA);

    sth_end_draw(m_data->m_fontStash);
    sth_flush_draw(m_data->m_fontStash);
  } else {
    // float width = 0.f;
    int pos = 0;
    // float color[]={0.2f,0.2,0.2f,1.f};
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_data->m_largeFontTextureId);

    // float width = r.x;
    // float extraSpacing = 0.;

    float startX = posX;
    float startY = posY;

    while (txt[pos]) {
      int c = txt[pos];
      // r.h = g_DefaultNormalFont->m_CharHeight;
      // r.w = g_DefaultNormalFont->m_CharWidth[c]+extraSpacing;
      float endX = startX + g_DefaultLargeFont->m_CharWidth[c];
      float endY = startY + g_DefaultLargeFont->m_CharHeight;

      float currentColor[] = {0.2f, 0.2, 0.2f, 1.f};

      m_primRenderer->draw_textured_tect(
          startX, startY, endX, endY, currentColor,
          g_DefaultLargeFont->m_CharU0[c], g_DefaultLargeFont->m_CharV0[c],
          g_DefaultLargeFont->m_CharU1[c], g_DefaultLargeFont->m_CharV1[c]);

      // DrawTexturedRect(0,r,g_DefaultNormalFont->m_CharU0[c],g_DefaultNormalFont->m_CharV0[c],g_DefaultNormalFont->m_CharU1[c],g_DefaultNormalFont->m_CharV1[c]);
      //	DrawFilledRect(r);

      startX = endX;
      // startY = endY;

      pos++;
    }
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  glDisable(GL_BLEND);
}

void TinyOpenGL3App::draw_textured_tect(float x0, float y0, float x1, float y1,
                                        float color[4], float u0, float v0,
                                        float u1, float v1, int useRGBA) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  m_primRenderer->draw_textured_tect(x0, y0, x1, y1, color, u0, v0, u1, v1,
                                     useRGBA);
  glDisable(GL_BLEND);
}



int TinyOpenGL3App::register_cube_shape(float halfExtentsX, float halfExtentsY,
                                        float halfExtentsZ, int textureIndex,
                                        float textureScaling) {
  int strideInBytes = 9 * sizeof(float);
  int numVertices = sizeof(cube_vertices_textured) / strideInBytes;
  int numIndices = sizeof(cube_indices) / sizeof(int);

  std::vector<GfxVertexFormat1> verts;
  verts.resize(numVertices);
  for (int i = 0; i < numVertices; i++) {
    verts[i].x = halfExtentsX * cube_vertices_textured[i * 9];
    verts[i].y = halfExtentsY * cube_vertices_textured[i * 9 + 1];
    verts[i].z = halfExtentsZ * cube_vertices_textured[i * 9 + 2];
    verts[i].w = cube_vertices_textured[i * 9 + 3];
    verts[i].nx = cube_vertices_textured[i * 9 + 4];
    verts[i].ny = cube_vertices_textured[i * 9 + 5];
    verts[i].nz = cube_vertices_textured[i * 9 + 6];
    verts[i].u = cube_vertices_textured[i * 9 + 7] * textureScaling;
    verts[i].v = cube_vertices_textured[i * 9 + 8] * textureScaling;
  }

  int shapeId = m_instancingRenderer->register_shape(
      &verts[0].x, numVertices, cube_indices, numIndices, B3_GL_TRIANGLES,
      textureIndex);
  return shapeId;
}

void TinyOpenGL3App::register_grid(int cells_x, int cells_z,
                                   const TinyVector3f& color0,
                                   const TinyVector3f& color1) {
  TinyVector3f cubeExtents = TinyVector3f(0.5, 0.5, 0.5);
  double halfHeight = 0.1;
  cubeExtents[m_data->m_upAxis] = halfHeight;
  int cubeId =
      register_cube_shape(cubeExtents[0], cubeExtents[1], cubeExtents[2]);
  TinyQuaternionf orn(0, 0, 0, 1);
  TinyVector3f center = TinyVector3f(0, 0, 0);
  TinyVector3f scaling = TinyVector3f(1, 1, 1);

  for (int i = 0; i < cells_x; i++) {
    for (int j = 0; j < cells_z; j++) {
      TinyVector3f color;
      if ((i + j) % 2 == 0) {
        color = color0;
      } else {
        color = color1;
      }
      if (this->m_data->m_upAxis == 1) {
        center = TinyVector3f((i + 0.5f) - cells_x * 0.5f, -halfHeight,
                              (j + 0.5f) - cells_z * 0.5f);
      } else {
        center = TinyVector3f((i + 0.5f) - cells_x * 0.5f,
                              (j + 0.5f) - cells_z * 0.5f, -halfHeight);
      }
      m_instancingRenderer->register_graphics_instance(cubeId, center, orn,
                                                       color, scaling);
    }
  }
}

int TinyOpenGL3App::register_graphics_unit_sphere_shape(
    EnumSphereLevelOfDetail lod, int textureId) {
  int red = 0;
  int green = 255;
  int blue = 0;  // 0;// 128;
  if (textureId < 0) {
    if (m_data->m_textureId < 0) {
      int texWidth = 1024;
      int texHeight = 1024;
      std::vector<unsigned char> texels;
      texels.resize(texWidth * texHeight * 3);
      for (int i = 0; i < texWidth * texHeight * 3; i++) texels[i] = 255;

      for (int i = 0; i < texWidth; i++) {
        for (int j = 0; j < texHeight; j++) {
          int a = i < texWidth / 2 ? 1 : 0;
          int b = j < texWidth / 2 ? 1 : 0;

          if (a == b) {
            texels[(i + j * texWidth) * 3 + 0] = red;
            texels[(i + j * texWidth) * 3 + 1] = green;
            texels[(i + j * texWidth) * 3 + 2] = blue;
            //					texels[(i+j*texWidth)*4+3] =
            // 255;
          }
          /*else
          {
          texels[i*3+0+j*texWidth] = 255;
          texels[i*3+1+j*texWidth] = 255;
          texels[i*3+2+j*texWidth] = 255;
          }
          */
        }
      }

      m_data->m_textureId = m_instancingRenderer->register_texture(
          &texels[0], texWidth, texHeight);
    }
    textureId = m_data->m_textureId;
  }

  int strideInBytes = 9 * sizeof(float);

  int graphicsShapeIndex = -1;

  switch (lod) {
    case SPHERE_LOD_POINT_SPRITE: {
      int numVertices = sizeof(point_sphere_vertices) / strideInBytes;
      int numIndices = sizeof(point_sphere_indices) / sizeof(int);
      graphicsShapeIndex = m_instancingRenderer->register_shape(
          &point_sphere_vertices[0], numVertices, point_sphere_indices,
          numIndices, B3_GL_POINTS, textureId);
      break;
    }

    case SPHERE_LOD_LOW: {
      int numVertices = sizeof(low_sphere_vertices) / strideInBytes;
      int numIndices = sizeof(low_sphere_indices) / sizeof(int);
      graphicsShapeIndex = m_instancingRenderer->register_shape(
          &low_sphere_vertices[0], numVertices, low_sphere_indices, numIndices,
          B3_GL_TRIANGLES, textureId);
      break;
    }
    case SPHERE_LOD_MEDIUM: {
      int numVertices =
          sizeof(textured_detailed_sphere_vertices) / strideInBytes;
      int numIndices = sizeof(textured_detailed_sphere_indices) / sizeof(int);
      graphicsShapeIndex = m_instancingRenderer->register_shape(
          &textured_detailed_sphere_vertices[0], numVertices,
          textured_detailed_sphere_indices, numIndices, B3_GL_TRIANGLES,
          textureId);
      break;
    }
    case SPHERE_LOD_HIGH:
    default: {
      int numVertices =
          sizeof(textured_detailed_sphere_vertices) / strideInBytes;
      int numIndices = sizeof(textured_detailed_sphere_indices) / sizeof(int);
      graphicsShapeIndex = m_instancingRenderer->register_shape(
          &textured_detailed_sphere_vertices[0], numVertices,
          textured_detailed_sphere_indices, numIndices, B3_GL_TRIANGLES,
          textureId);
      break;
    }
  };
  return graphicsShapeIndex;
}


int TinyOpenGL3App::register_graphics_cylinder_shape(float radius, float half_height, int up_axis, int textureId, bool flat_caps) {
        int red = 0;
    int green = 255;
    int blue = 0;  // 0;// 128;
    if(textureId < 0) {
        if(m_data->m_textureId < 0) {
            int texWidth = 1024;
            int texHeight = 1024;
            std::vector<unsigned char> texels;
            texels.resize(texWidth * texHeight * 3);
            for(int i = 0; i < texWidth * texHeight * 3; i++) texels[i] = 255;

            for(int i = 0; i < texWidth; i++) {
                for(int j = 0; j < texHeight; j++) {
                    int a = i < texWidth / 2 ? 1 : 0;
                    int b = j < texWidth / 2 ? 1 : 0;

                    if(a == b) {
                        texels[(i + j * texWidth) * 3 + 0] = red;
                        texels[(i + j * texWidth) * 3 + 1] = green;
                        texels[(i + j * texWidth) * 3 + 2] = blue;
                    }
                    /*else
                    {
                    texels[i*3+0+j*texWidth] = 255;
                    texels[i*3+1+j*texWidth] = 255;
                    texels[i*3+2+j*texWidth] = 255;
                    }
                    */
                }
            }

            m_data->m_textureId = m_instancingRenderer->register_texture(
                &texels[0],texWidth,texHeight);
        }
        textureId = m_data->m_textureId;
    }

    int strideInBytes = 9 * sizeof(float);

    int graphicsShapeIndex = -1;

    
    
    int numVertices =
        sizeof(textured_detailed_sphere_vertices) / strideInBytes;
    int numIndices = sizeof(textured_detailed_sphere_indices) / sizeof(int);
    //scale and transform

    std::vector<float> transformedVertices;

    {
        
        int numVertices = sizeof(textured_detailed_sphere_vertices) / strideInBytes;
        transformedVertices.resize(numVertices * 9);
        for(int i = 0; i < numVertices; i++)
        {
            TinyVector3f vert;
            vert.setValue(textured_detailed_sphere_vertices[i * 9 + 0],
                textured_detailed_sphere_vertices[i * 9 + 1],
                textured_detailed_sphere_vertices[i * 9 + 2]);

            TinyVector3f trVer = (2 *radius * vert);
            if (flat_caps)
            {
                if(trVer[up_axis] > 0)
                    trVer[up_axis] = half_height;
                else
                    trVer[up_axis] = -half_height;
            } else
            {
                if(trVer[up_axis] > 0)
                    trVer[up_axis] += half_height;
                else
                    trVer[up_axis] -= half_height;
            
            }
            
            transformedVertices[i * 9 + 0] = trVer[0];
            transformedVertices[i * 9 + 1] = trVer[1];
            transformedVertices[i * 9 + 2] = trVer[2];
            transformedVertices[i * 9 + 3] = textured_detailed_sphere_vertices[i * 9 + 3];
            transformedVertices[i * 9 + 4] = textured_detailed_sphere_vertices[i * 9 + 4];
            transformedVertices[i * 9 + 5] = textured_detailed_sphere_vertices[i * 9 + 5];
            transformedVertices[i * 9 + 6] = textured_detailed_sphere_vertices[i * 9 + 6];
            transformedVertices[i * 9 + 7] = textured_detailed_sphere_vertices[i * 9 + 7];
            transformedVertices[i * 9 + 8] = textured_detailed_sphere_vertices[i * 9 + 8];
        }
    }


    graphicsShapeIndex = m_instancingRenderer->register_shape(
        &transformedVertices[0],numVertices,
        textured_detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,
        textureId);
    
    return graphicsShapeIndex;
}

int TinyOpenGL3App::register_graphics_capsule_shape(float radius, float half_height, int up_axis, int textureId) {

    bool flat_caps = false;
    return register_graphics_cylinder_shape(radius, half_height, up_axis, textureId, flat_caps);

}


void TinyOpenGL3App::draw_grid(DrawGridData data) {
  int gridSize = data.gridSize;
  float upOffset = data.upOffset;
  int upAxis = data.upAxis;
  TinyVector3f gridColor;
  gridColor[0] = data.gridColor[0];
  gridColor[1] = data.gridColor[1];
  gridColor[2] = data.gridColor[2];

  int sideAxis = -1;
  int forwardAxis = -1;

  switch (upAxis) {
    case 1:
      forwardAxis = 2;
      sideAxis = 0;
      break;
    case 2:
      forwardAxis = 1;
      sideAxis = 0;
      break;
    default:
      assert(0);
  };
  // TinyVector3f gridColor = TinyVector3f(0.5,0.5,0.5);

  std::vector<unsigned int> indices;
  std::vector<TinyVector3f> vertices;
  int lineIndex = 0;
  for (int i = -gridSize; i <= gridSize; i++) {
    {
      assert(glGetError() == GL_NO_ERROR);
      TinyVector3f from = TinyVector3f(0, 0, 0);
      from[sideAxis] = float(i);
      from[upAxis] = upOffset;
      from[forwardAxis] = float(-gridSize);
      TinyVector3f to = TinyVector3f(0, 0, 0);
      to[sideAxis] = float(i);
      to[upAxis] = upOffset;
      to[forwardAxis] = float(gridSize);
      vertices.push_back(from);
      indices.push_back(lineIndex++);
      vertices.push_back(to);
      indices.push_back(lineIndex++);
      // m_instancingRenderer->draw_line(from,to,gridColor);
    }

    assert(glGetError() == GL_NO_ERROR);
    {
      assert(glGetError() == GL_NO_ERROR);
      TinyVector3f from = TinyVector3f(0, 0, 0);
      from[sideAxis] = float(-gridSize);
      from[upAxis] = upOffset;
      from[forwardAxis] = float(i);
      TinyVector3f to = TinyVector3f(0, 0, 0);
      to[sideAxis] = float(gridSize);
      to[upAxis] = upOffset;
      to[forwardAxis] = float(i);
      vertices.push_back(from);
      indices.push_back(lineIndex++);
      vertices.push_back(to);
      indices.push_back(lineIndex++);
      // m_instancingRenderer->draw_line(from,to,gridColor);
    }
  }

  m_instancingRenderer->draw_lines(&vertices[0], gridColor, vertices.size(),
                                   sizeof(TinyVector3f), &indices[0],
                                   indices.size(), 1);

  if (data.drawAxis)
  {
      m_instancingRenderer->draw_line(TinyVector3f(0, 0, 0), TinyVector3f(1, 0, 0),
          TinyVector3f(1, 0, 0), 3);
      m_instancingRenderer->draw_line(TinyVector3f(0, 0, 0), TinyVector3f(0, 1, 0),
          TinyVector3f(0, 1, 0), 3);
      m_instancingRenderer->draw_line(TinyVector3f(0, 0, 0), TinyVector3f(0, 0, 1),
          TinyVector3f(0, 0, 1), 3);

      m_instancingRenderer->draw_point(TinyVector3f(1, 0, 0), TinyVector3f(1, 0, 0),
                                       6);
      m_instancingRenderer->draw_point(TinyVector3f(0, 1, 0), TinyVector3f(0, 1, 0),
                                       6);
      m_instancingRenderer->draw_point(TinyVector3f(0, 0, 1), TinyVector3f(0, 0, 1),
                                       6);
  }

}

void TinyOpenGL3App::set_background_color(float red, float green, float blue) {
  TinyCommonGraphicsApp::set_background_color(red, green, blue);
  glClearColor(m_backgroundColorRGB[0], m_backgroundColorRGB[1],
               m_backgroundColorRGB[2], 1.f);
}

TinyOpenGL3App::~TinyOpenGL3App() {
  delete m_instancingRenderer;
  delete m_primRenderer;
  sth_delete(m_data->m_fontStash);
  delete m_data->m_renderCallbacks;

  sth_delete(m_data->m_fontStash2);
  delete m_data->m_renderCallbacks2;

  TwDeleteDefaultFonts();
  m_window->close_window();

  delete m_window;
  delete m_data;
}

void TinyOpenGL3App::set_viewport(int width, int height) {
  m_data->m_customViewPortWidth = width;
  m_data->m_customViewPortHeight = height;
  if (width >= 0) {
    glViewport(0, 0, width, height);
  } else {
    glViewport(
        0, 0,
        m_window->get_retina_scale() * m_instancingRenderer->get_screen_width(),
        m_window->get_retina_scale() *
            m_instancingRenderer->get_screen_height());
  }
}


void TinyOpenGL3App::get_screen_pixels(std::vector<unsigned char>& rgbaBuffer,
                                  std::vector<float>& depthBuffer) {
  int width = m_data->m_customViewPortWidth >= 0
                  ? m_data->m_customViewPortWidth
                  : (int)m_window->get_retina_scale() *
                        m_instancingRenderer->get_screen_width();
  int height = m_data->m_customViewPortHeight >= 0
                   ? m_data->m_customViewPortHeight
                   : (int)m_window->get_retina_scale() *
                         m_instancingRenderer->get_screen_height();

  if (m_data->m_renderTexture) {
      width = m_data->m_renderTexture->m_width;
      height = m_data->m_renderTexture->m_height;
  }
  rgbaBuffer.resize(width*height*4);
  {
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, &rgbaBuffer[0]);
    int glstat;
    glstat = glGetError();
    assert(glstat == GL_NO_ERROR);
  }
  depthBuffer.resize(width * height);
  {
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT,
                 &depthBuffer[0]);
    int glstat;
    glstat = glGetError();
    assert(glstat == GL_NO_ERROR);
  }
}

//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image/stb_image_write.h"
static void writeTextureToFile(int textureWidth, int textureHeight,
                               const char* fileName, FILE* ffmpegVideo) {
  int numComponents = 4;
  // glPixelStorei(GL_PACK_ALIGNMENT,1);

  assert(glGetError() == GL_NO_ERROR);
  // glReadBuffer(GL_BACK);//COLOR_ATTACHMENT0);
  
  //int mem_bytes = textureWidth * textureHeight * numComponents * 4;
  //printf("mem_bytes=%d\n", mem_bytes);
  std::vector<float> orgPixelsArray;
  orgPixelsArray.resize(textureWidth * textureHeight * numComponents);
  //float* orgPixels =
  //    (float*)malloc(mem_bytes);
  // printf("orgPixels=%x\n", orgPixels);
  float* orgPixels = &orgPixelsArray[0];
  glReadPixels(0, 0, textureWidth, textureHeight, GL_RGBA, GL_FLOAT, &orgPixelsArray[0]);
  // it is useful to have the actual float values for debugging purposes

  // convert float->char
  std::vector<char> pixelArray;
  pixelArray.resize(textureWidth * textureHeight * numComponents);
  char* pixels = &pixelArray[0];
  
  assert(glGetError() == GL_NO_ERROR);

  for (int j = 0; j < textureHeight; j++) {
    for (int i = 0; i < textureWidth; i++) {
      pixels[(j * textureWidth + i) * numComponents] =
          char(orgPixels[(j * textureWidth + i) * numComponents] * 255.f);
      pixels[(j * textureWidth + i) * numComponents + 1] =
          char(orgPixels[(j * textureWidth + i) * numComponents + 1] * 255.f);
      pixels[(j * textureWidth + i) * numComponents + 2] =
          char(orgPixels[(j * textureWidth + i) * numComponents + 2] * 255.f);
      pixels[(j * textureWidth + i) * numComponents + 3] =
          char(orgPixels[(j * textureWidth + i) * numComponents + 3] * 255.f);
    }
  }

  if (ffmpegVideo) {
    fwrite(pixels, textureWidth * textureHeight * numComponents, 1,
           ffmpegVideo);
    // fwrite(pixels,
    // 100,1,ffmpegVideo);//textureWidth*textureHeight*numComponents, 1,
    // ffmpegVideo);
  } else {
    if (1) {
      // swap the pixels
      unsigned char tmp;

      for (int j = 0; j < textureHeight / 2; j++) {
        for (int i = 0; i < textureWidth; i++) {
          for (int c = 0; c < numComponents; c++) {
            tmp = pixels[(j * textureWidth + i) * numComponents + c];
            pixels[(j * textureWidth + i) * numComponents + c] =
                pixels[((textureHeight - j - 1) * textureWidth + i) *
                           numComponents +
                       c];
            pixels[((textureHeight - j - 1) * textureWidth + i) *
                       numComponents +
                   c] = tmp;
          }
        }
      }
    }
    stbi_write_png_compression_level = 0;
    stbi_write_png(fileName, textureWidth, textureHeight, numComponents, pixels,
                   textureWidth * numComponents);

  }

}

void TinyOpenGL3App::swap_buffer() {
  if (m_data->m_frameDumpPngFileName!="") {

    int width = (int)m_window->get_retina_scale() *
                m_instancingRenderer->get_screen_width();
    int height = (int)m_window->get_retina_scale() *
                 this->m_instancingRenderer->get_screen_height();

    if (m_data->m_renderTexture) {
      width = m_data->m_renderTexture->m_width;
      height = m_data->m_renderTexture->m_height;
    }
    writeTextureToFile(width, height, m_data->m_frameDumpPngFileName.c_str(),
                       m_data->m_ffmpegFile);
    m_data->m_renderTexture->disable();
    if (m_data->m_ffmpegFile == 0) {
      m_data->m_frameDumpPngFileName = "";
    }
  }
  m_window->end_rendering();
  m_window->start_rendering();
}

void TinyOpenGL3App::set_mp4_fps(int fps) { m_data->m_mp4Fps = fps; }

// see also
// http://blog.mmacklin.com/2013/06/11/real-time-video-capture-with-ffmpeg/
void TinyOpenGL3App::dump_frames_to_video(const char* mp4FileName) {
  if (mp4FileName) {
    int width = (int)m_window->get_retina_scale() *
                m_instancingRenderer->get_screen_width();
    int height = (int)m_window->get_retina_scale() *
                 m_instancingRenderer->get_screen_height();
    char cmd[8192];

    sprintf(cmd,
            "ffmpeg -r %d -f rawvideo -pix_fmt rgba -s %dx%d -i - "
            "-threads 0 -y -b:v 50000k   -c:v libx264 -preset slow -crf 22 -an "
            "  -pix_fmt yuv420p -vf vflip %s",
            m_data->m_mp4Fps, width, height, mp4FileName);

    if (m_data->m_ffmpegFile) {
      pclose(m_data->m_ffmpegFile);
    }
    if (mp4FileName) {
      m_data->m_ffmpegFile = popen(cmd, "w");

      m_data->m_frameDumpPngFileName = mp4FileName;
    }
  } else {
    if (m_data->m_ffmpegFile) {
      fflush(m_data->m_ffmpegFile);
      pclose(m_data->m_ffmpegFile);
      m_data->m_frameDumpPngFileName = "";
    }
    m_data->m_ffmpegFile = 0;
  }
}


void TinyOpenGL3App::dump_next_frame_to_png(const char* filename,
                                            bool render_to_texture,
                                            int render_width,
                                            int render_height) {
  // open pipe to ffmpeg's stdin in binary write mode

  m_data->m_frameDumpPngFileName = filename;

  // you could use m_renderTexture to allow to render at higher resolutions,
  // such as 4k or so
  if (render_to_texture) {
    enable_render_to_texture(render_width, render_height);
  }
}



// CUDA includes
//#define USE_SYSTEM_CUDA
#ifdef USE_SYSTEM_CUDA
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#else
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#define dlsym GetProcAddress
std::string DYNAMIC_CUDA_PATH = "nvcuda.dll";
std::string DYNAMIC_CUDART_PATH = "cudart64_110.dll";
#else
#include <dlfcn.h>
std::string DYNAMIC_CUDA_PATH = "/usr/lib/x86_64-linux-gnu/libcuda.so";
std::string DYNAMIC_CUDART_PATH = "/usr/local/cuda/lib64/libcudart.so";
#endif

using namespace std;

enum TINY_CUDA_CODES {
  cudaSuccess = 0,
  CU_GET_PROC_ADDRESS_DEFAULT = 0,
  cudaEnableDefault = 0,

  cudaGraphicsMapFlagsNone = 0,
  cudaGraphicsMapFlagsReadOnly = 1,
  cudaGraphicsMapFlagsWriteDiscard = 2,

  cudaMemcpyHostToHost = 0,     /**< Host   -> Host */
  cudaMemcpyHostToDevice = 1,   /**< Host   -> Device */
  cudaMemcpyDeviceToHost = 2,   /**< Device -> Host */
  cudaMemcpyDeviceToDevice = 3, /**< Device -> Device */
  cudaMemcpyDefault = 4 /**< Direction of the transfer is inferred from the pointer values.
           Requires unified virtual addressing */
};

// CUDA driver API functions.
typedef struct cudaGraphicsResource* cudaGraphicsResource_t;
typedef struct CUstream_st* cudaStream_t;
typedef struct cudaArray* cudaArray_t;

// see
// https://docs.nvidia.com/cuda/cuda-runtime-api/driver-vs-runtime-api.html#driver-vs-runtime-api
// cuda driver (cuda.so)
TINY_CUDA_CODES (*cuDriverGetVersion)(int* version);
TINY_CUDA_CODES (*cuInit)(unsigned int flags);
TINY_CUDA_CODES (*cuDeviceGetCount)(int* count);
TINY_CUDA_CODES (*cuDeviceGetCount2)(int* count);
TINY_CUDA_CODES (*cuGetProcAddress) (const char* symbol, void** pfn, int cudaVersion, uint64_t flags);
TINY_CUDA_CODES (*cudaMalloc)(void** devPtr, size_t size);
TINY_CUDA_CODES (*cudaFree)(void* devPtr);

TINY_CUDA_CODES (*cudaGLRegisterBufferObject) ( GLuint bufObj );
TINY_CUDA_CODES (*cudaGLMapBufferObject) ( void** devPtr, GLuint bufObj );
TINY_CUDA_CODES (*cudaGLUnmapBufferObject) ( GLuint bufObj );



TINY_CUDA_CODES (*cudaMemcpyFromArray)
(void* dst, const cudaArray_t src, size_t wOffset, size_t hOffset, size_t count,
 unsigned int kind);

#define LOAD_CUDA_FUNCTION(name, version)                                  \
  name = reinterpret_cast<decltype(name)>(dlsym(cuda_lib, #name version)); \
  if (!name) cout << "Error:" << #name << " not found in CUDA library" << endl

// cuda runtime (cudart.so)
TINY_CUDA_CODES (*cudaGraphicsGLRegisterImage) (cudaGraphicsResource** resource, uint64_t image, int target,
 unsigned int flags);
// Returns the requested driver API function pointer.
// see also
// https://forums.developer.nvidia.com/t/some-questions-about-cugetprocaddress/191749
TINY_CUDA_CODES (*cudaGetDriverEntryPoint)
(const char* symbol, void** funcPtr, unsigned long long flags);
TINY_CUDA_CODES (*cudaGraphicsMapResources)
(int count, cudaGraphicsResource_t* resources, cudaStream_t stream);
TINY_CUDA_CODES (*cudaGraphicsSubResourceGetMappedArray)
(cudaArray_t* array, cudaGraphicsResource_t resource, unsigned int arrayIndex,
 unsigned int mipLevel);
TINY_CUDA_CODES (*cudaGraphicsUnmapResources)
(int count, cudaGraphicsResource_t* resources, cudaStream_t stream);

#define LOAD_CUDART_FUNCTION(name, version)                                  \
  name = reinterpret_cast<decltype(name)>(dlsym(cudart_lib, #name version)); \
  if (!name) cout << "Error:" << #name << " not found in CUDA library" << endl



static bool s_cuda_initialized = false;


int init_cuda(bool verbose) {

    if (s_cuda_initialized) 
            return s_cuda_initialized;

#ifdef _WIN32
  HMODULE cuda_lib = (HMODULE)LoadLibraryA(DYNAMIC_CUDA_PATH.c_str());
  HMODULE cudart_lib = (HMODULE)LoadLibraryA(DYNAMIC_CUDART_PATH.c_str());
#else
  void* cuda_lib = dlopen(DYNAMIC_CUDA_PATH.c_str(), RTLD_NOW);
  void* cudart_lib = dlopen(DYNAMIC_CUDART_PATH.c_str(), RTLD_NOW);
  // could use dlerror() on error
#endif
  if (!cuda_lib) {
    cout << "Unable to load library " << DYNAMIC_CUDA_PATH << endl << endl;
    return false;
  }
  if (!cudart_lib) {
    cout << "Unable to load library " << DYNAMIC_CUDART_PATH << endl << endl;
    return false;
  }

  LOAD_CUDA_FUNCTION(cuDriverGetVersion, "");
  LOAD_CUDA_FUNCTION(cuInit, "");
  LOAD_CUDA_FUNCTION(cuDeviceGetCount, "");
  LOAD_CUDA_FUNCTION(cuGetProcAddress, "");
 
  LOAD_CUDART_FUNCTION(cudaGetDriverEntryPoint, "");
  LOAD_CUDART_FUNCTION(cudaGraphicsGLRegisterImage, "");
  LOAD_CUDART_FUNCTION(cudaGraphicsMapResources, "");
  LOAD_CUDART_FUNCTION(cudaGraphicsSubResourceGetMappedArray, "");
  LOAD_CUDART_FUNCTION(cudaMemcpyFromArray, "");
  LOAD_CUDART_FUNCTION(cudaGraphicsUnmapResources, "");
  LOAD_CUDART_FUNCTION(cudaMalloc, "");
  LOAD_CUDART_FUNCTION(cudaFree, "");
  LOAD_CUDART_FUNCTION(cudaGLRegisterBufferObject,"");
  LOAD_CUDART_FUNCTION(cudaGLMapBufferObject, "");
  LOAD_CUDART_FUNCTION(cudaGLUnmapBufferObject, "");
  

  auto result = cuInit(0);
  int cuda_driver_version;
  result = cuDriverGetVersion(&cuda_driver_version);
  if (verbose) {
    cout << "CUDA driver version:" << cuda_driver_version << endl;
  }
  int device_count = 0;
  result = cuDeviceGetCount(&device_count);
  if (verbose) {
    cout << "CUDA device count:" << device_count << endl;
  }
  result = cuGetProcAddress("cuDeviceGetCount", (void**)&cuDeviceGetCount2,
                            cuda_driver_version, CU_GET_PROC_ADDRESS_DEFAULT);
  if (cudaSuccess != result) {
    cout << "cuDeviceGetCount not found" << endl;
    return false;
  }

  s_cuda_initialized = true;
  return s_cuda_initialized;
}

#endif


struct ptr2int {
  union {
    void* pointer;
    uint64_t intval;
  };
};


TinyCudaVbo TinyOpenGL3App::cuda_map_vbo(bool verbose)
{
    

    init_cuda(verbose);
    auto internal_renderer_data = m_instancingRenderer->get_internal_data();

    if (!m_data->m_cudaVboRegistered)
    {
        cudaGLRegisterBufferObject(internal_renderer_data->m_vbo);
        m_data->m_cudaVboRegistered = true;
    }
    cudaGLMapBufferObject(&m_data->m_cudaVboPointer, internal_renderer_data->m_vbo);

    int position_buffer_size_in_bytes = 4*sizeof(float)*internal_renderer_data->m_totalNumInstances;
    int orientation_buffer_size_in_bytes = 4*sizeof(float)*internal_renderer_data->m_totalNumInstances;
    int color_buffer_size_in_bytes = 4*sizeof(float)*internal_renderer_data->m_totalNumInstances;

    TinyCudaVbo vbo;
    char* pp = (char*)m_data->m_cudaVboPointer;
    vbo.vertices_ptr = pp;
    vbo.num_instances = internal_renderer_data->m_totalNumInstances;
    vbo.positions_ptr = pp+internal_renderer_data->m_maxShapeCapacityInBytes;
    vbo.orientations_ptr = pp+internal_renderer_data->m_maxShapeCapacityInBytes+position_buffer_size_in_bytes;
    vbo.colors_ptr = pp+internal_renderer_data->m_maxShapeCapacityInBytes+position_buffer_size_in_bytes+orientation_buffer_size_in_bytes;
    vbo.scalings_ptr = pp+internal_renderer_data->m_maxShapeCapacityInBytes+position_buffer_size_in_bytes+orientation_buffer_size_in_bytes+color_buffer_size_in_bytes;
    return vbo;
}

void TinyOpenGL3App::cuda_unmap_vbo()
{
    if (m_data->m_cudaVboPointer!=0)
    {
        auto internal_renderer_data = m_instancingRenderer->get_internal_data();
        cudaGLUnmapBufferObject(internal_renderer_data->m_vbo);
        m_data->m_cudaVboPointer = 0;
    }
}




uint64_t TinyOpenGL3App::cuda_register_texture_image(uint64_t renderTextureId, bool verbose) {
  init_cuda(verbose);

  GLuint shDrawTex;  // draws a texture
  cudaGraphicsResource* cuda_tex_result_resource;
  // get read-write access to OpenGL texture buffer
  GLuint tex_cudaResult;
  tex_cudaResult = cudaGraphicsGLRegisterImage(
      &cuda_tex_result_resource, renderTextureId, GL_TEXTURE_2D,
      cudaGraphicsMapFlagsReadOnly);  // cudaGraphicsMapFlagsNone);

  assert(tex_cudaResult == cudaSuccess);

  ptr2int caster;
  caster.pointer = cuda_tex_result_resource;
  return caster.intval;
}

uint64_t TinyOpenGL3App::cuda_malloc(int num_bytes) { 
  ptr2int caster;
  GLuint tex_cudaResult;
   tex_cudaResult = cudaMalloc(&caster.pointer, num_bytes); 
  assert(tex_cudaResult == cudaSuccess);
  return caster.intval;
}

void TinyOpenGL3App::cuda_free(uint64_t cuda_ptr) {
  ptr2int caster;
  caster.intval = cuda_ptr;
  cudaFree(caster.pointer);
}
uint64_t TinyOpenGL3App::cuda_copy_texture_image( uint64_t cuda_resource_int, uint64_t dest_memory_int, int num_bytes, bool gpu_device_destination, int w_offset, int h_offset) { 

  ptr2int caster;
  caster.intval = cuda_resource_int;
  cudaGraphicsResource* cuda_tex_result_resource = (cudaGraphicsResource*)caster.pointer;
  caster.intval = dest_memory_int;
  void* dest_memory = caster.pointer;

  GLuint tex_cudaResult = cudaGraphicsMapResources(1, &cuda_tex_result_resource, NULL);
  assert(tex_cudaResult == cudaSuccess);

  // https://stackoverflow.com/questions/9406844/cudagraphicsresourcegetmappedpointer-returns-unknown-error
  // In other words, use GetMappedPointer for mapped buffer objects. Use
  // GetMappedArray for mapped texture objects.
  cudaArray* texture_ptr;
  tex_cudaResult = cudaGraphicsSubResourceGetMappedArray(
      &texture_ptr, cuda_tex_result_resource, 0, 0);
  assert(tex_cudaResult == cudaSuccess);

  tex_cudaResult = cudaMemcpyFromArray(
      dest_memory, texture_ptr, w_offset, h_offset, num_bytes,
      gpu_device_destination ? cudaMemcpyDeviceToDevice
                             : cudaMemcpyDeviceToHost);
  
  assert(tex_cudaResult == cudaSuccess);

  tex_cudaResult = cudaGraphicsUnmapResources(1, &cuda_tex_result_resource, 0);
  assert(tex_cudaResult == cudaSuccess);
  return (uint64_t)cuda_tex_result_resource;
}



uint64_t TinyOpenGL3App::enable_render_to_texture(int render_width,
                                              int render_height) {
  if (!m_data->m_renderTexture) {
    m_data->m_renderTexture = new GLRenderToTexture();
    glGenTextures(1, &m_data->m_renderTexture->m_renderTextureId);

    // "Bind" the newly created texture : all future texture functions will
    // modify this texture
    glBindTexture(GL_TEXTURE_2D, m_data->m_renderTexture->m_renderTextureId);

    // Give an empty image to OpenGL ( the last "0" )
    // glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, g_OpenGLWidth,g_OpenGLHeight,
    // 0,GL_RGBA, GL_UNSIGNED_BYTE, 0); glTexImage2D(GL_TEXTURE_2D,
    // 0,GL_RGBA32F, g_OpenGLWidth,g_OpenGLHeight, 0,GL_RGBA, GL_FLOAT, 0);

    if (render_width < 0) {
      render_width = m_instancingRenderer->get_screen_width() *
                     m_window->get_retina_scale();
    }
    if (render_height < 0) {
      render_height = this->m_instancingRenderer->get_screen_height() *
                      m_window->get_retina_scale();
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, render_width, render_height,
                 0,GL_RGBA, GL_UNSIGNED_BYTE, 0);
                 
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, render_width, render_height, 0,
    //             GL_RGBA, GL_FLOAT, 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR_MIPMAP_LINEAR);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    m_data->m_renderTexture->init(render_width, render_height,
                                  m_data->m_renderTexture->m_renderTextureId,
                                  RENDERTEXTURE_COLOR);

  }

  m_data->m_renderTexture->enable();

  return m_data->m_renderTexture->m_renderTextureId;
}

void TinyOpenGL3App::set_up_axis(int axis) {
  assert((axis == 1) || (axis == 2));  // only Y or Z is supported at the moment
  m_data->m_upAxis = axis;
}
int TinyOpenGL3App::get_up_axis() const { return m_data->m_upAxis; }
#endif  //#ifndef NO_OPENGL3
