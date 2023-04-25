#ifndef NO_OPENGL3
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

/// todo: make this configurable in the gui
bool useShadowMap = true;

#include <assert.h>
#include <stdio.h>
#include <algorithm>  // std::sort
#include <vector>
#include "utils/tiny_min_max.h"
#include "tiny_opengl_include.h"
#include "tiny_window_interface.h"
#include "utils/tiny_logging.h"


struct caster2 {
  void setInt(int v) { i = v; }
  float getFloat() {
    float v = ((float)i) + .25f;
    return v;
  }

  union {
    int i;
    float f;
  };
};

#define MAX_POINTS_IN_BATCH 1024
#define MAX_LINES_IN_BATCH 1024
#define MAX_TRIANGLES_IN_BATCH 8192

#ifdef B3_USE_GLFW

#else
#ifndef __APPLE__
#ifndef glVertexAttribDivisor

#ifndef NO_GLEW

#define glVertexAttribDivisor glVertexAttribDivisorARB
#endif  // NO_GLEW
#endif  // glVertexAttribDivisor
#ifndef GL_COMPARE_REF_TO_TEXTURE
#define GL_COMPARE_REF_TO_TEXTURE GL_COMPARE_R_TO_TEXTURE
#endif  // GL_COMPARE_REF_TO_TEXTURE
#ifndef glDrawElementsInstanced
#ifndef NO_GLEW
#define glDrawElementsInstanced glDrawElementsInstancedARB
#endif  // NO_GLEW
#endif
#endif  //__APPLE__
#endif  // B3_USE_GLFW
#include "tiny_gl_instancing_renderer.h"

#include <stdio.h>
#include <string.h>

#include "tiny_resizable_pool.h"

#include "tiny_load_shader.h"

#include "tiny_gl_instance_renderer_internal_data.h"

// GLSL shader strings, embedded using build3/stringify
#include "Shaders/createShadowMapInstancingPS.h"
#include "Shaders/createShadowMapInstancingVS.h"
#include "Shaders/instancingPS.h"
#include "Shaders/instancingVS.h"
#include "Shaders/pointSpritePS.h"
#include "Shaders/pointSpriteVS.h"
#include "Shaders/projectiveTextureInstancingPS.h"
#include "Shaders/projectiveTextureInstancingVS.h"
#include "Shaders/useShadowMapInstancingPS.h"
#include "Shaders/useShadowMapInstancingVS.h"

#include "Shaders/segmentationMaskInstancingPS.h"
#include "Shaders/segmentationMaskInstancingVS.h"

#include "Shaders/linesPS.h"
#include "Shaders/linesVS.h"

#include "stb_image/stb_image_write.h"
#include "tiny_gl_render_to_texture.h"

using namespace TINY;

static const char* triangleVertexShaderText =
    "#version 330\n"
    "precision highp float;"
    "uniform mat4 MVP;\n"
    "uniform vec3 vCol;\n"
    "layout (location = 0) in vec3 vPos;\n"
    "layout (location = 1) in vec2 vUV;\n"

    "out vec3 clr;\n"
    "out vec2 uv0;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = MVP * vec4(vPos,1);\n"
    "    clr = vCol;\n"
    "    uv0 = vUV;\n"
    "}\n";


static const char* triangleFragmentShader =
    "#version 330\n"
    "precision highp float;"
    "in vec3 clr;\n"
    "in vec2 uv0;"
    "out vec4 color;"
    "uniform sampler2D Diffuse;"
    "void main()\n"
    "{\n"
    "    vec4 texel = texture(Diffuse,uv0);\n"
    "    color = vec4(clr,texel.r)*texel;\n"
    "}\n";

std::string triangleVertexShaderTextInit = triangleVertexShaderText;
std::string triangleFragmentShaderInit = triangleFragmentShader;

std::string useShadowMapInstancingVertexShaderInit = useShadowMapInstancingVertexShader;
std::string useShadowMapInstancingFragmentShaderInit = useShadowMapInstancingFragmentShader;

std::string createShadowMapInstancingVertexShaderInit = createShadowMapInstancingVertexShader;
std::string createShadowMapInstancingFragmentShaderInit = createShadowMapInstancingFragmentShader;

std::string segmentationMaskInstancingVertexShaderInit = segmentationMaskInstancingVertexShader;
std::string segmentationMaskInstancingFragmentShaderInit = segmentationMaskInstancingFragmentShader;

std::string instancingVertexShaderInit = instancingVertexShader;
std::string instancingFragmentShaderInit = instancingFragmentShader;

//#include
//"../../opencl/gpu_rigidbody_pipeline/b3GpuNarrowphaseAndSolver.h"//for
// m_maxNumObjectCapacity

static InternalDataRenderer* sData2;

GLint lineWidthRange[2] = {1, 1};

struct b3GraphicsInstance {
  GLuint m_cube_vao;
  GLuint m_index_vbo;
  GLuint m_textureIndex;
  int m_numIndices;
  int m_numVertices;

  int m_numGraphicsInstances;
  std::vector<int> m_tempObjectUids;
  int m_instanceOffset;
  int m_vertexArrayOffset;
  int m_primitiveType;
  float m_materialShinyNess;
  TinyVector3f m_materialSpecularColor;
  int m_flags;  // transparency etc

  b3GraphicsInstance()
      : m_cube_vao(-1),
        m_index_vbo(-1),
        m_textureIndex(-1),
        m_numIndices(-1),
        m_numVertices(-1),
        m_numGraphicsInstances(0),
        m_instanceOffset(0),
        m_vertexArrayOffset(0),
        m_primitiveType(B3_GL_TRIANGLES),
        m_materialShinyNess(41),
        m_materialSpecularColor(TinyVector3f(.5, .5, .5)),
        m_flags(0) {}
};

bool m_ortho = false;

// static GLfloat depthLightModelviewMatrix[16];

static void checkError(const char* functionName) {
  GLenum error;
  while ((error = glGetError()) != GL_NO_ERROR) {
    fprintf(stderr, "GL error 0x%X detected in %s\n", error, functionName);
  }
}

extern int gShapeIndex;

struct InternalTextureHandle {
  GLuint m_glTexture;
  int m_width;
  int m_height;
  int m_enableFiltering;
};

struct TinyPublicGraphicsInstanceData {
  int m_shapeIndex;
  int m_internalInstanceIndex;
  GLfloat m_position[4];
  GLfloat m_orientation[4];
  GLfloat m_color[4];
  GLfloat m_scale[4];

  void clear() {}
};

typedef TinyPoolBodyHandle<TinyPublicGraphicsInstanceData>
    TinyPublicGraphicsInstance;

struct InternalDataRenderer : public GLInstanceRendererInternalData {
  TinyCamera m_defaultCamera1;
  TinyCamera* m_activeCamera;

  GLfloat m_projectionMatrix[16];
  GLfloat m_viewMatrix[16];
  GLfloat m_projectiveTextureProjectionMatrix[16];
  GLfloat m_projectiveTextureViewMatrix[16];
  GLfloat m_viewMatrixInverse[16];
  bool m_useProjectiveTexture;

  TinyVector3f m_lightPos;
  TinyVector3f m_lightSpecularIntensity;

  GLuint m_defaultTexturehandle;
  std::vector<InternalTextureHandle> m_textureHandles;

  GLRenderToTexture* m_shadowMap;
  GLuint m_shadowTexture;

  GLuint m_renderFrameBuffer;

  TinyResizablePool<TinyPublicGraphicsInstance> m_publicGraphicsInstances;

  int m_shadowMapWidth;
  int m_shadowMapHeight;
  float m_shadowMapWorldSize;
  bool m_updateShadowMap;

  InternalDataRenderer()
      : m_activeCamera(&m_defaultCamera1),
        m_shadowMap(0),
        m_shadowTexture(0),
        m_renderFrameBuffer(0),
        m_shadowMapWidth(8192),
        m_shadowMapHeight(8192),
        m_shadowMapWorldSize(25),
        m_updateShadowMap(true)

  {
    m_lightPos = TinyVector3f(-50, 30, 40);
    m_lightSpecularIntensity.setValue(1, 1, 1);

    // clear to zero to make it obvious if the matrix is used uninitialized
    for (int i = 0; i < 16; i++) {
      m_projectionMatrix[i] = 0;
      m_viewMatrix[i] = 0;
      m_viewMatrixInverse[i] = 0;
      m_projectiveTextureProjectionMatrix[i] = 0;
      m_projectiveTextureViewMatrix[i] = 0;
    }

    m_useProjectiveTexture = false;
  }
};

struct GLInstanceRendererInternalData*
TinyGLInstancingRenderer::get_internal_data() {
  return m_data;
}

static GLuint triangleShaderProgram;
static GLint triangle_mvp_location = -1;
static GLint triangle_vpos_location = -1;
static GLint triangle_vUV_location = -1;
static GLint triangle_vcol_location = -1;
static GLuint triangleVertexBufferObject = 0;
static GLuint triangleVertexArrayObject = 0;
static GLuint triangleIndexVbo = 0;

static GLuint linesShader;                   // The line renderer
static GLuint useShadowMapInstancingShader;  // The shadow instancing renderer
static GLuint
    createShadowMapInstancingShader;  // The shadow instancing renderer
static GLuint projectiveTextureInstancingShader;  // The projective texture
                                                  // instancing renderer
static GLuint segmentationMaskInstancingShader;   // The segmentation mask
                                                  // instancing renderer

static GLuint instancingShader;  // The instancing renderer
static GLuint
    instancingShaderPointSprite;  // The point sprite instancing renderer

// static bool                 done = false;

static GLint lines_ModelViewMatrix = 0;
static GLint lines_ProjectionMatrix = 0;
static GLint lines_position = 0;
static GLint lines_colour = 0;
GLuint lineVertexBufferObject = 0;
GLuint lineVertexArrayObject = 0;
GLuint lineIndexVbo = 0;

GLuint linesVertexBufferObject = 0;
GLuint linesVertexArrayObject = 0;
GLuint linesIndexVbo = 0;

static GLint useShadow_ViewMatrixInverse = 0;
static GLint useShadow_ModelViewMatrix = 0;
static GLint useShadow_lightSpecularIntensity = 0;
static GLint useShadow_materialSpecularColor = 0;
static GLint useShadow_MVP = 0;
static GLint useShadow_lightPosIn = 0;
static GLint useShadow_cameraPositionIn = 0;
static GLint useShadow_materialShininessIn = 0;

static GLint useShadow_ProjectionMatrix = 0;
static GLint useShadow_DepthBiasModelViewMatrix = 0;
static GLint useShadow_uniform_texture_diffuse = 0;
static GLint useShadow_shadowMap = 0;

static GLint createShadow_depthMVP = 0;

static GLint projectiveTexture_ViewMatrixInverse = 0;
static GLint projectiveTexture_ModelViewMatrix = 0;
static GLint projectiveTexture_lightSpecularIntensity = 0;
static GLint projectiveTexture_materialSpecularColor = 0;
static GLint projectiveTexture_MVP = 0;
static GLint projectiveTexture_lightPosIn = 0;
static GLint projectiveTexture_cameraPositionIn = 0;
static GLint projectiveTexture_materialShininessIn = 0;

static GLint projectiveTexture_ProjectionMatrix = 0;
static GLint projectiveTexture_TextureMVP = 0;
static GLint projectiveTexture_uniform_texture_diffuse = 0;
static GLint projectiveTexture_shadowMap = 0;

static GLint ModelViewMatrix = 0;
static GLint ProjectionMatrix = 0;
static GLint regularLightDirIn = 0;

static GLint segmentationMaskModelViewMatrix = 0;
static GLint segmentationMaskProjectionMatrix = 0;

static GLint uniform_texture_diffuse = 0;

static GLint screenWidthPointSprite = 0;
static GLint ModelViewMatrixPointSprite = 0;
static GLint ProjectionMatrixPointSprite = 0;
// static GLint	uniform_texture_diffusePointSprite= 0;

TinyGLInstancingRenderer::TinyGLInstancingRenderer(int maxNumObjectCapacity,
                                                   int maxShapeCapacityInBytes)
    : m_textureenabled(true),
      m_textureinitialized(false),
      m_screenWidth(0),
      m_screenHeight(0),
      m_upAxis(1),
      m_planeReflectionShapeIndex(-1) {
  m_data = new InternalDataRenderer;
  m_data->m_maxNumObjectCapacity = maxNumObjectCapacity;
  m_data->m_maxShapeCapacityInBytes = maxShapeCapacityInBytes;

  m_data->m_totalNumInstances = 0;

  sData2 = m_data;

  m_data->m_instance_positions_ptr.resize(m_data->m_maxNumObjectCapacity * 4);
  m_data->m_instance_quaternion_ptr.resize(m_data->m_maxNumObjectCapacity * 4);
  m_data->m_instance_colors_ptr.resize(m_data->m_maxNumObjectCapacity * 4);
  m_data->m_instance_scale_ptr.resize(m_data->m_maxNumObjectCapacity * 4);
}

void TinyGLInstancingRenderer::remove_all_instances() {
  m_data->m_totalNumInstances = 0;

  for (int i = 0; i < m_graphicsInstances.size(); i++) {
    if (m_graphicsInstances[i]->m_index_vbo) {
      glDeleteBuffers(1, &m_graphicsInstances[i]->m_index_vbo);
    }
    if (m_graphicsInstances[i]->m_cube_vao) {
      glDeleteVertexArrays(1, &m_graphicsInstances[i]->m_cube_vao);
    }
    delete m_graphicsInstances[i];
  }
  m_graphicsInstances.clear();
  m_data->m_publicGraphicsInstances.exit_handles();
  m_data->m_publicGraphicsInstances.init_handles();
}

TinyGLInstancingRenderer::~TinyGLInstancingRenderer() {
  delete m_data->m_shadowMap;
  glDeleteTextures(1, &m_data->m_shadowTexture);
  glDeleteTextures(1, &m_data->m_defaultTexturehandle);

  remove_all_instances();

  sData2 = 0;

  if (m_data) {
    if (m_data->m_vbo) glDeleteBuffers(1, &m_data->m_vbo);
  }
  delete m_data;
}

int TinyGLInstancingRenderer::get_shape_index_from_instance(int srcIndex) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex);
  if (pg) {
    return pg->m_shapeIndex;
  }
  return -1;
}

bool TinyGLInstancingRenderer::read_single_instance_transform_to_cpu(
    float* position, float* orientation, int shapeIndex) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(shapeIndex);
  if (pg) {
    int srcIndex = pg->m_internalInstanceIndex;

    if ((srcIndex < m_data->m_totalNumInstances) && (srcIndex >= 0)) {
      position[0] = m_data->m_instance_positions_ptr[srcIndex * 4 + 0];
      position[1] = m_data->m_instance_positions_ptr[srcIndex * 4 + 1];
      position[2] = m_data->m_instance_positions_ptr[srcIndex * 4 + 2];

      orientation[0] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 0];
      orientation[1] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 1];
      orientation[2] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 2];
      orientation[3] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 3];
      return true;
    }
  }

  return false;
}

void TinyGLInstancingRenderer::write_single_instance_transform_to_cpu(
    const TinyVector3f& position, const TinyQuaternionf& orientation, int srcIndex2) {
  if (srcIndex2 < 0) 
      return;

  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);

  if (pg == 0) return;

  int srcIndex = pg->m_internalInstanceIndex;

  assert(srcIndex < m_data->m_totalNumInstances);
  assert(srcIndex >= 0);
  m_data->m_instance_positions_ptr[srcIndex * 4 + 0] = position[0];
  m_data->m_instance_positions_ptr[srcIndex * 4 + 1] = position[1];
  m_data->m_instance_positions_ptr[srcIndex * 4 + 2] = position[2];
  m_data->m_instance_positions_ptr[srcIndex * 4 + 3] = 1;

  m_data->m_instance_quaternion_ptr[srcIndex * 4 + 0] = orientation[0];
  m_data->m_instance_quaternion_ptr[srcIndex * 4 + 1] = orientation[1];
  m_data->m_instance_quaternion_ptr[srcIndex * 4 + 2] = orientation[2];
  m_data->m_instance_quaternion_ptr[srcIndex * 4 + 3] = orientation[3];
}

void TinyGLInstancingRenderer::read_single_instance_transform_from_cpu(
    int srcIndex2, float* position, float* orientation) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int srcIndex = pg->m_internalInstanceIndex;

  assert(srcIndex < m_data->m_totalNumInstances);
  assert(srcIndex >= 0);
  position[0] = m_data->m_instance_positions_ptr[srcIndex * 4 + 0];
  position[1] = m_data->m_instance_positions_ptr[srcIndex * 4 + 1];
  position[2] = m_data->m_instance_positions_ptr[srcIndex * 4 + 2];

  orientation[0] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 0];
  orientation[1] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 1];
  orientation[2] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 2];
  orientation[3] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 3];
}

void TinyGLInstancingRenderer::write_single_instance_flags_to_cpu(
    int flags, int srcIndex2) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int srcIndex = pg->m_internalInstanceIndex;

  int shapeIndex = pg->m_shapeIndex;
  b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
  if (flags & B3_INSTANCE_DOUBLE_SIDED) {
    gfxObj->m_flags |= B3_INSTANCE_DOUBLE_SIDED;
  } else {
    gfxObj->m_flags &= ~B3_INSTANCE_DOUBLE_SIDED;
  }
}

void TinyGLInstancingRenderer::write_single_instance_color_to_cpu(
    const double* color, int srcIndex2) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int srcIndex = pg->m_internalInstanceIndex;

  int shapeIndex = pg->m_shapeIndex;
  b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
  if (color[3] < 1) {
    gfxObj->m_flags |= B3_INSTANCE_TRANSPARANCY;
  } else {
    gfxObj->m_flags &= ~B3_INSTANCE_TRANSPARANCY;
  }

  m_data->m_instance_colors_ptr[srcIndex * 4 + 0] = float(color[0]);
  m_data->m_instance_colors_ptr[srcIndex * 4 + 1] = float(color[1]);
  m_data->m_instance_colors_ptr[srcIndex * 4 + 2] = float(color[2]);
  m_data->m_instance_colors_ptr[srcIndex * 4 + 3] = float(color[3]);
}

void TinyGLInstancingRenderer::write_single_instance_color_to_cpu(
    const float* color, int srcIndex2) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int srcIndex = pg->m_internalInstanceIndex;
  int shapeIndex = pg->m_shapeIndex;
  b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];

  if (color[3] < 1) {
    gfxObj->m_flags |= B3_INSTANCE_TRANSPARANCY;
  } else {
    gfxObj->m_flags &= ~B3_INSTANCE_TRANSPARANCY;
  }

  m_data->m_instance_colors_ptr[srcIndex * 4 + 0] = color[0];
  m_data->m_instance_colors_ptr[srcIndex * 4 + 1] = color[1];
  m_data->m_instance_colors_ptr[srcIndex * 4 + 2] = color[2];
  m_data->m_instance_colors_ptr[srcIndex * 4 + 3] = color[3];
}

void TinyGLInstancingRenderer::write_single_instance_scale_to_cpu(
    const float* scale, int srcIndex2) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int srcIndex = pg->m_internalInstanceIndex;

  m_data->m_instance_scale_ptr[srcIndex * 4 + 0] = scale[0];
  m_data->m_instance_scale_ptr[srcIndex * 4 + 1] = scale[1];
  m_data->m_instance_scale_ptr[srcIndex * 4 + 2] = scale[2];
  caster2 c;
  c.setInt(srcIndex2);
  m_data->m_instance_scale_ptr[srcIndex * 4 + 3] = c.getFloat();
}

void TinyGLInstancingRenderer::write_single_instance_specular_color_to_cpu(
    const double* specular, int srcIndex2) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int graphicsIndex = pg->m_internalInstanceIndex;

  int totalNumInstances = 0;

  int gfxObjIndex = -1;

  for (int i = 0; i < m_graphicsInstances.size(); i++) {
    totalNumInstances += m_graphicsInstances[i]->m_numGraphicsInstances;
    if (srcIndex2 < totalNumInstances) {
      gfxObjIndex = i;
      break;
    }
  }
  if (gfxObjIndex > 0) {
    m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[0] = specular[0];
    m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[1] = specular[1];
    m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[2] = specular[2];
  }
}
void TinyGLInstancingRenderer::write_single_instance_specular_color_to_cpu(
    const float* specular, int srcIndex2) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int srcIndex = pg->m_internalInstanceIndex;

  int totalNumInstances = 0;

  int gfxObjIndex = -1;

  for (int i = 0; i < m_graphicsInstances.size(); i++) {
    totalNumInstances += m_graphicsInstances[i]->m_numGraphicsInstances;
    if (srcIndex2 < totalNumInstances) {
      gfxObjIndex = i;
      break;
    }
  }
  if (gfxObjIndex > 0) {
    m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[0] = specular[0];
    m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[1] = specular[1];
    m_graphicsInstances[gfxObjIndex]->m_materialSpecularColor[2] = specular[2];
  }
}

void TinyGLInstancingRenderer::write_single_instance_scale_to_cpu(
    const double* scale, int srcIndex2) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
  assert(pg);
  int srcIndex = pg->m_internalInstanceIndex;

  m_data->m_instance_scale_ptr[srcIndex * 4 + 0] = scale[0];
  m_data->m_instance_scale_ptr[srcIndex * 4 + 1] = scale[1];
  m_data->m_instance_scale_ptr[srcIndex * 4 + 2] = scale[2];
  caster2 c;
  c.setInt(srcIndex2);
  m_data->m_instance_scale_ptr[srcIndex * 4 + 3] = c.getFloat();
}

void TinyGLInstancingRenderer::write_single_instance_transform_to_gpu(
    float* position, float* orientation, int objectUniqueId) {
  glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
  // glFlush();

  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(objectUniqueId);
  assert(pg);
  int objectIndex = pg->m_internalInstanceIndex;

  char* orgBase = (char*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
  // b3GraphicsInstance* gfxObj = m_graphicsInstances[k];
  int totalNumInstances = 0;
  for (int k = 0; k < m_graphicsInstances.size(); k++) {
    b3GraphicsInstance* gfxObj = m_graphicsInstances[k];
    totalNumInstances += gfxObj->m_numGraphicsInstances;
  }

  int POSITION_BUFFER_SIZE = (totalNumInstances * sizeof(float) * 4);

  char* base = orgBase;

  float* positions = (float*)(base + m_data->m_maxShapeCapacityInBytes);
  float* orientations =
      (float*)(base + m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE);

  positions[objectIndex * 4] = position[0];
  positions[objectIndex * 4 + 1] = position[1];
  positions[objectIndex * 4 + 2] = position[2];
  positions[objectIndex * 4 + 3] = position[3];

  orientations[objectIndex * 4] = orientation[0];
  orientations[objectIndex * 4 + 1] = orientation[1];
  orientations[objectIndex * 4 + 2] = orientation[2];
  orientations[objectIndex * 4 + 3] = orientation[3];

  glUnmapBuffer(GL_ARRAY_BUFFER);
  // glFlush();
}

void TinyGLInstancingRenderer::write_transforms() {
  { assert(glGetError() == GL_NO_ERROR); }
  { glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo); }

  {
    // without the flush, the glBufferSubData can spike to really slow (seconds
    // slow)
    glFlush();
  }

  { assert(glGetError() == GL_NO_ERROR); }

#ifdef B3_DEBUG
  {
    int totalNumInstances = 0;
    for (int k = 0; k < m_graphicsInstances.size(); k++) {
      b3GraphicsInstance* gfxObj = m_graphicsInstances[k];
      totalNumInstances += gfxObj->m_numGraphicsInstances;
    }
    assert(m_data->m_totalNumInstances == totalNumInstances);
  }
#endif  // B3_DEBUG

  int POSITION_BUFFER_SIZE = (m_data->m_totalNumInstances * sizeof(float) * 4);
  int ORIENTATION_BUFFER_SIZE =
      (m_data->m_totalNumInstances * sizeof(float) * 4);
  int COLOR_BUFFER_SIZE = (m_data->m_totalNumInstances * sizeof(float) * 4);
  //	int SCALE_BUFFER_SIZE = (totalNumInstances*sizeof(float)*4);

#if 1
  {
    //	printf("m_data->m_totalNumInstances = %d\n",
    // m_data->m_totalNumInstances);
    {
      glBufferSubData(GL_ARRAY_BUFFER, m_data->m_maxShapeCapacityInBytes,
                      m_data->m_totalNumInstances * sizeof(float) * 4,
                      &m_data->m_instance_positions_ptr[0]);
    }
    {
      glBufferSubData(GL_ARRAY_BUFFER,
                      m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE,
                      m_data->m_totalNumInstances * sizeof(float) * 4,
                      &m_data->m_instance_quaternion_ptr[0]);
    }
    {
      glBufferSubData(GL_ARRAY_BUFFER,
                      m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE +
                          ORIENTATION_BUFFER_SIZE,
                      m_data->m_totalNumInstances * sizeof(float) * 4,
                      &m_data->m_instance_colors_ptr[0]);
    }
    {
      glBufferSubData(GL_ARRAY_BUFFER,
                      m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE +
                          ORIENTATION_BUFFER_SIZE + COLOR_BUFFER_SIZE,
                      m_data->m_totalNumInstances * sizeof(float) * 4,
                      &m_data->m_instance_scale_ptr[0]);
    }
  }
#else

  char* orgBase = (char*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
  if (orgBase) {
    for (int k = 0; k < m_graphicsInstances.size(); k++) {
      // int k=0;
      b3GraphicsInstance* gfxObj = m_graphicsInstances[k];

      char* base = orgBase;

      float* positions = (float*)(base + m_data->m_maxShapeCapacityInBytes);
      float* orientations = (float*)(base + m_data->m_maxShapeCapacityInBytes +
                                     POSITION_BUFFER_SIZE);
      float* colors = (float*)(base + m_data->m_maxShapeCapacityInBytes +
                               POSITION_BUFFER_SIZE + ORIENTATION_BUFFER_SIZE);
      float* scaling = (float*)(base + m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE + ORIENTATION_BUFFER_SIZE +
                                COLOR_BUFFER_SIZE);

      // static int offset=0;
      // offset++;

      for (int i = 0; i < gfxObj->m_numGraphicsInstances; i++) {
        int srcIndex = i + gfxObj->m_instanceOffset;

        positions[srcIndex * 4] =
            m_data->m_instance_positions_ptr[srcIndex * 4];
        positions[srcIndex * 4 + 1] =
            m_data->m_instance_positions_ptr[srcIndex * 4 + 1];
        positions[srcIndex * 4 + 2] =
            m_data->m_instance_positions_ptr[srcIndex * 4 + 2];
        positions[srcIndex * 4 + 3] =
            m_data->m_instance_positions_ptr[srcIndex * 4 + 3];

        orientations[srcIndex * 4] =
            m_data->m_instance_quaternion_ptr[srcIndex * 4];
        orientations[srcIndex * 4 + 1] =
            m_data->m_instance_quaternion_ptr[srcIndex * 4 + 1];
        orientations[srcIndex * 4 + 2] =
            m_data->m_instance_quaternion_ptr[srcIndex * 4 + 2];
        orientations[srcIndex * 4 + 3] =
            m_data->m_instance_quaternion_ptr[srcIndex * 4 + 3];

        colors[srcIndex * 4] = m_data->m_instance_colors_ptr[srcIndex * 4];
        colors[srcIndex * 4 + 1] =
            m_data->m_instance_colors_ptr[srcIndex * 4 + 1];
        colors[srcIndex * 4 + 2] =
            m_data->m_instance_colors_ptr[srcIndex * 4 + 2];
        colors[srcIndex * 4 + 3] =
            m_data->m_instance_colors_ptr[srcIndex * 4 + 3];

        scaling[srcIndex * 4] = m_data->m_instance_scale_ptr[srcIndex * 4];
        scaling[srcIndex * 4 + 1] =
            m_data->m_instance_scale_ptr[srcIndex * 4 + 1];
        scaling[srcIndex * 4 + 2] =
            m_data->m_instance_scale_ptr[srcIndex * 4 + 2];
        scaling[srcIndex * 4 + 3] =
            m_data->m_instance_scale_ptr[srcIndex * 4 + 3];
      }
    }
  } else {
    b3Error("ERROR glMapBuffer failed\n");
  }
  assert(glGetError() == GL_NO_ERROR);

  glUnmapBuffer(GL_ARRAY_BUFFER);
  // if this glFinish is removed, the animation is not always working/blocks
  //@todo: figure out why
  // glFlush();

#endif

  {
    glBindBuffer(GL_ARRAY_BUFFER, 0);  // m_data->m_vbo);
  }

  { assert(glGetError() == GL_NO_ERROR); }
}

void TinyGLInstancingRenderer::rebuild_graphics_instances() {
  m_data->m_totalNumInstances = 0;

  std::vector<int> usedObjects;
  m_data->m_publicGraphicsInstances.get_used_handles(usedObjects);

  for (int i = 0; i < usedObjects.size(); i++) {
    int srcIndex2 = usedObjects[i];
    TinyPublicGraphicsInstance* pg =
        m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
    assert(pg);
    int srcIndex = pg->m_internalInstanceIndex;

    pg->m_position[0] = m_data->m_instance_positions_ptr[srcIndex * 4 + 0];
    pg->m_position[1] = m_data->m_instance_positions_ptr[srcIndex * 4 + 1];
    pg->m_position[2] = m_data->m_instance_positions_ptr[srcIndex * 4 + 2];
    pg->m_orientation[0] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 0];
    pg->m_orientation[1] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 1];
    pg->m_orientation[2] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 2];
    pg->m_orientation[3] = m_data->m_instance_quaternion_ptr[srcIndex * 4 + 3];
    pg->m_color[0] = m_data->m_instance_colors_ptr[srcIndex * 4 + 0];
    pg->m_color[1] = m_data->m_instance_colors_ptr[srcIndex * 4 + 1];
    pg->m_color[2] = m_data->m_instance_colors_ptr[srcIndex * 4 + 2];
    pg->m_color[3] = m_data->m_instance_colors_ptr[srcIndex * 4 + 3];
    pg->m_scale[0] = m_data->m_instance_scale_ptr[srcIndex * 4 + 0];
    pg->m_scale[1] = m_data->m_instance_scale_ptr[srcIndex * 4 + 1];
    pg->m_scale[2] = m_data->m_instance_scale_ptr[srcIndex * 4 + 2];
    pg->m_scale[3] = m_data->m_instance_scale_ptr[srcIndex * 4 + 3];
  }
  for (int i = 0; i < m_graphicsInstances.size(); i++) {
    m_graphicsInstances[i]->m_numGraphicsInstances = 0;
    m_graphicsInstances[i]->m_instanceOffset = 0;
    m_graphicsInstances[i]->m_tempObjectUids.clear();
  }
  for (int i = 0; i < usedObjects.size(); i++) {
    int srcIndex2 = usedObjects[i];
    TinyPublicGraphicsInstance* pg =
        m_data->m_publicGraphicsInstances.get_handle(srcIndex2);
    if (pg && pg->m_shapeIndex < m_graphicsInstances.size() &&
        pg->m_shapeIndex >= 0) {
      m_graphicsInstances[pg->m_shapeIndex]->m_tempObjectUids.push_back(
          srcIndex2);
    }
  }

  int curOffset = 0;
  m_data->m_totalNumInstances = 0;

  for (int i = 0; i < m_graphicsInstances.size(); i++) {
    m_graphicsInstances[i]->m_instanceOffset = curOffset;
    m_graphicsInstances[i]->m_numGraphicsInstances = 0;

    for (int g = 0; g < m_graphicsInstances[i]->m_tempObjectUids.size(); g++) {
      curOffset++;
      int objectUniqueId = m_graphicsInstances[i]->m_tempObjectUids[g];
      TinyPublicGraphicsInstance* pg =
          m_data->m_publicGraphicsInstances.get_handle(objectUniqueId);
      TinyVector3f pos(pg->m_position[0], pg->m_position[1], pg->m_position[2]);
      TinyQuaternionf orn(pg->m_orientation[0], pg->m_orientation[1],
                          pg->m_orientation[2], pg->m_orientation[3]);
      TinyVector3f color(pg->m_color[0], pg->m_color[1], pg->m_color[2]);
      TinyVector3f scaling(pg->m_scale[0], pg->m_scale[1], pg->m_scale[2]);
      register_graphics_instance_internal(objectUniqueId, pos, orn, color,
                                          scaling);
    }
  }
}

void TinyGLInstancingRenderer::remove_graphics_instance(int instanceUid) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(instanceUid);
  assert(pg);
  if (pg) {
    m_data->m_publicGraphicsInstances.free_handle(instanceUid);
    rebuild_graphics_instances();
  }
}

int TinyGLInstancingRenderer::register_graphics_instance_internal(
    int newUid, const TinyVector3f& position, const TinyQuaternionf& quaternion,
    const TinyVector3f& color, const TinyVector3f& scaling,
    float opacity) {
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(newUid);
  int shapeIndex = pg->m_shapeIndex;
  //	assert(pg);
  //	int objectIndex = pg->m_internalInstanceIndex;

  b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
  int index = gfxObj->m_numGraphicsInstances + gfxObj->m_instanceOffset;
  pg->m_internalInstanceIndex = index;

  int maxElements = m_data->m_instance_positions_ptr.size();
  if (index * 4 < maxElements) {
    m_data->m_instance_positions_ptr[index * 4] = position[0];
    m_data->m_instance_positions_ptr[index * 4 + 1] = position[1];
    m_data->m_instance_positions_ptr[index * 4 + 2] = position[2];
    m_data->m_instance_positions_ptr[index * 4 + 3] = 1;

    m_data->m_instance_quaternion_ptr[index * 4] = quaternion[0];
    m_data->m_instance_quaternion_ptr[index * 4 + 1] = quaternion[1];
    m_data->m_instance_quaternion_ptr[index * 4 + 2] = quaternion[2];
    m_data->m_instance_quaternion_ptr[index * 4 + 3] = quaternion[3];

    m_data->m_instance_colors_ptr[index * 4] = color[0];
    m_data->m_instance_colors_ptr[index * 4 + 1] = color[1];
    m_data->m_instance_colors_ptr[index * 4 + 2] = color[2];
    m_data->m_instance_colors_ptr[index * 4 + 3] = opacity;

    m_data->m_instance_scale_ptr[index * 4] = scaling[0];
    m_data->m_instance_scale_ptr[index * 4 + 1] = scaling[1];
    m_data->m_instance_scale_ptr[index * 4 + 2] = scaling[2];
    caster2 c;
    c.setInt(newUid);
    m_data->m_instance_scale_ptr[index * 4 + 3] = c.getFloat();

    if (opacity < 1 && opacity > 0)
    {
      gfxObj->m_flags |= B3_INSTANCE_TRANSPARANCY;
    }
    gfxObj->m_numGraphicsInstances++;
    m_data->m_totalNumInstances++;
  } else {
    printf("register_graphics_instance out of range, %d\n", maxElements);
    return -1;
  }
   return newUid;  // gfxObj->m_numGraphicsInstances;
}


std::vector<int> TinyGLInstancingRenderer::register_graphics_instances(int shapeIndex,
                                         const std::vector<::TINY::TinyVector3f>& positions,
                                         const std::vector<::TINY::TinyQuaternionf>& quaternions,
                                         const std::vector<::TINY::TinyVector3f>& colors,
                                         const std::vector<::TINY::TinyVector3f>& scalings,
                                         float opacity,
                                         bool rebuild)
{
    std::vector<int> uids;
    
    if ((positions.size() == quaternions.size()) && 
        (positions.size() == colors.size()) && 
        (positions.size() == scalings.size() ))
    {
        uids.reserve(positions.size());

        for (int i=0;i<positions.size();i++)
        {
            int uid = register_graphics_instance(shapeIndex, positions[i], quaternions[i], colors[i], scalings[i], opacity, false);
            uids.push_back(uid);
        }
    }
    if (rebuild)
    {
        rebuild_graphics_instances();
    }
    return uids;
}

int TinyGLInstancingRenderer::register_graphics_instance(
    int shapeIndex, const TinyVector3f& position,
    const TinyQuaternionf& quaternion, const TinyVector3f& color,
    const TinyVector3f& scaling, float opacity, bool rebuild) {
  int newUid = m_data->m_publicGraphicsInstances.alloc_handle();
  TinyPublicGraphicsInstance* pg =
      m_data->m_publicGraphicsInstances.get_handle(newUid);
  pg->m_shapeIndex = shapeIndex;

  // assert(shapeIndex == (m_graphicsInstances.size()-1));
  assert(m_graphicsInstances.size() < m_data->m_maxNumObjectCapacity - 1);
  //if (shapeIndex == (m_graphicsInstances.size() - 1)) 
  //{
  //  register_graphics_instance_internal(newUid, position, quaternion, color,
  //                                      scaling, opacity);
  //} else 
  {
    int srcIndex = m_data->m_totalNumInstances++;
    pg->m_internalInstanceIndex = srcIndex;
    
    m_data->m_instance_positions_ptr[srcIndex * 4 + 0] = position[0];
    m_data->m_instance_positions_ptr[srcIndex * 4 + 1] = position[1];
    m_data->m_instance_positions_ptr[srcIndex * 4 + 2] = position[2];
    m_data->m_instance_positions_ptr[srcIndex * 4 + 3] = 1.;

    m_data->m_instance_quaternion_ptr[srcIndex * 4 + 0] = quaternion[0];
    m_data->m_instance_quaternion_ptr[srcIndex * 4 + 1] = quaternion[1];
    m_data->m_instance_quaternion_ptr[srcIndex * 4 + 2] = quaternion[2];
    m_data->m_instance_quaternion_ptr[srcIndex * 4 + 3] = quaternion[3];

    m_data->m_instance_colors_ptr[srcIndex * 4 + 0] = color[0];
    m_data->m_instance_colors_ptr[srcIndex * 4 + 1] = color[1];
    m_data->m_instance_colors_ptr[srcIndex * 4 + 2] = color[2];
    m_data->m_instance_colors_ptr[srcIndex * 4 + 3] = opacity;

    m_data->m_instance_scale_ptr[srcIndex * 4 + 0] = scaling[0];
    m_data->m_instance_scale_ptr[srcIndex * 4 + 1] = scaling[1];
    m_data->m_instance_scale_ptr[srcIndex * 4 + 2] = scaling[2];
    caster2 c;
    c.setInt(newUid);
    m_data->m_instance_scale_ptr[srcIndex * 4 + 3] = c.getFloat();

    if (rebuild)
    {
        rebuild_graphics_instances();
    }
  }

  return newUid;
}

void TinyGLInstancingRenderer::remove_texture(int textureIndex) {
  if ((textureIndex >= 0) && (textureIndex < m_data->m_textureHandles.size())) {
    InternalTextureHandle& h = m_data->m_textureHandles[textureIndex];
    glDeleteTextures(1, &h.m_glTexture);
  }
}

int TinyGLInstancingRenderer::register_texture(const unsigned char* texels,
                                               int width, int height,
                                               bool flipPixelsY) {
  assert(glGetError() == GL_NO_ERROR);
  glActiveTexture(GL_TEXTURE0);
  int textureIndex = m_data->m_textureHandles.size();
  //  const GLubyte*	image= (const GLubyte*)texels;
  GLuint textureHandle;
  glGenTextures(1, (GLuint*)&textureHandle);
  glBindTexture(GL_TEXTURE_2D, textureHandle);

  assert(glGetError() == GL_NO_ERROR);

  InternalTextureHandle h;
  h.m_glTexture = textureHandle;
  h.m_width = width;
  h.m_height = height;
  h.m_enableFiltering = true;
  m_data->m_textureHandles.push_back(h);
  if (texels) {
    update_texture(textureIndex, texels, flipPixelsY);
  }
  return textureIndex;
}

void TinyGLInstancingRenderer::replace_texture(int shapeIndex, int textureId) {
  if ((shapeIndex >= 0) && (shapeIndex < m_graphicsInstances.size())) {
    b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
    if (textureId >= 0 && textureId < m_data->m_textureHandles.size()) {
      gfxObj->m_textureIndex = textureId;
      gfxObj->m_flags |= B3_INSTANCE_TEXTURE;
    } else {
      gfxObj->m_textureIndex = -1;
      gfxObj->m_flags &= ~B3_INSTANCE_TEXTURE;
    }
  }
}

void TinyGLInstancingRenderer::update_texture(int textureIndex,
                                              const unsigned char* texels,
                                              bool flipPixelsY) {
  if ((textureIndex >= 0) && (textureIndex < m_data->m_textureHandles.size())) {
    glActiveTexture(GL_TEXTURE0);
    assert(glGetError() == GL_NO_ERROR);
    InternalTextureHandle& h = m_data->m_textureHandles[textureIndex];
    glBindTexture(GL_TEXTURE_2D, h.m_glTexture);
    assert(glGetError() == GL_NO_ERROR);

    if (flipPixelsY) {
      // textures need to be flipped for OpenGL...
      std::vector<unsigned char> flippedTexels;
      flippedTexels.resize(h.m_width * h.m_height * 3);

      for (int j = 0; j < h.m_height; j++) {
        for (int i = 0; i < h.m_width; i++) {
          flippedTexels[(i + j * h.m_width) * 3] =
              texels[(i + (h.m_height - 1 - j) * h.m_width) * 3];
          flippedTexels[(i + j * h.m_width) * 3 + 1] =
              texels[(i + (h.m_height - 1 - j) * h.m_width) * 3 + 1];
          flippedTexels[(i + j * h.m_width) * 3 + 2] =
              texels[(i + (h.m_height - 1 - j) * h.m_width) * 3 + 2];
        }
      }

      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, h.m_width, h.m_height, 0, GL_RGB,
                   GL_UNSIGNED_BYTE, &flippedTexels[0]);
    } else {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, h.m_width, h.m_height, 0, GL_RGB,
                   GL_UNSIGNED_BYTE, &texels[0]);
    }
    assert(glGetError() == GL_NO_ERROR);
    if (h.m_enableFiltering) {
      glGenerateMipmap(GL_TEXTURE_2D);
    }
    assert(glGetError() == GL_NO_ERROR);
  }
}

void TinyGLInstancingRenderer::activate_texture(int textureIndex) {
  glActiveTexture(GL_TEXTURE0);

  if (textureIndex >= 0 && textureIndex < m_data->m_textureHandles.size()) {
    glBindTexture(GL_TEXTURE_2D,
                  m_data->m_textureHandles[textureIndex].m_glTexture);
  } else {
    glBindTexture(GL_TEXTURE_2D, 0);
  }
}

void TinyGLInstancingRenderer::update_shape(int shapeIndex,
                                            const float* vertices) {
  b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
  int numvertices = gfxObj->m_numVertices;

  glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
  int vertexStrideInBytes = 9 * sizeof(float);
  int sz = numvertices * vertexStrideInBytes;
#if 0
	char* dest=  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_WRITE_ONLY);//GL_WRITE_ONLY
	memcpy(dest+vertexStrideInBytes*gfxObj->m_vertexArrayOffset,vertices,sz);
	glUnmapBuffer( GL_ARRAY_BUFFER);
#else
  glBufferSubData(GL_ARRAY_BUFFER,
                  vertexStrideInBytes * gfxObj->m_vertexArrayOffset, sz,
                  vertices);
#endif
}


std::vector<int> TinyGLInstancingRenderer::get_shape_vertex_count() const
{
    std::vector<int> vertex_counts;
    vertex_counts.resize(m_graphicsInstances.size());
    for (int i=0;i<m_graphicsInstances.size();i++)
    {
        vertex_counts[i] = m_graphicsInstances[i]->m_numVertices;
    }
    return vertex_counts;
}

std::vector<int> TinyGLInstancingRenderer::get_shape_vertex_offsets() const
{
    std::vector<int> vertex_offsets;
    vertex_offsets.resize(m_graphicsInstances.size());
    for (int i=0;i<m_graphicsInstances.size();i++)
    {
        vertex_offsets[i] = m_graphicsInstances[i]->m_vertexArrayOffset;
    }
    return vertex_offsets;
}



int TinyGLInstancingRenderer::register_shape(const float* vertices,
                                             int numvertices,
                                             const int* indices, int numIndices,
                                             int primitiveType, int textureId, bool double_sided) {
  b3GraphicsInstance* gfxObj = new b3GraphicsInstance;

  if (textureId >= 0) {
    gfxObj->m_textureIndex = textureId;
    gfxObj->m_flags |= B3_INSTANCE_TEXTURE;
  }
  if (double_sided) {
    gfxObj->m_flags |= B3_INSTANCE_DOUBLE_SIDED;
  }

  gfxObj->m_primitiveType = primitiveType;

  if (m_graphicsInstances.size()) {
    b3GraphicsInstance* prevObj =
        m_graphicsInstances[m_graphicsInstances.size() - 1];
    gfxObj->m_instanceOffset =
        prevObj->m_instanceOffset + prevObj->m_numGraphicsInstances;
    gfxObj->m_vertexArrayOffset =
        prevObj->m_vertexArrayOffset + prevObj->m_numVertices;
  } else {
    gfxObj->m_instanceOffset = 0;
  }

  m_graphicsInstances.push_back(gfxObj);
  gfxObj->m_numIndices = numIndices;
  gfxObj->m_numVertices = numvertices;

  int vertexStrideInBytes = 9 * sizeof(float);
  int sz = numvertices * vertexStrideInBytes;
  int totalUsed = vertexStrideInBytes * gfxObj->m_vertexArrayOffset + sz;
  assert(totalUsed < m_data->m_maxShapeCapacityInBytes);
  if (totalUsed >= m_data->m_maxShapeCapacityInBytes) {
    return -1;
  }

  glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);

#if 0

	char* dest=  (char*)glMapBuffer( GL_ARRAY_BUFFER,GL_WRITE_ONLY);//GL_WRITE_ONLY

#ifdef B3_DEBUG

#endif  // B3_DEBUG

	memcpy(dest+vertexStrideInBytes*gfxObj->m_vertexArrayOffset,vertices,sz);
	glUnmapBuffer( GL_ARRAY_BUFFER);
#else
  glBufferSubData(GL_ARRAY_BUFFER,
                  vertexStrideInBytes * gfxObj->m_vertexArrayOffset, sz,
                  vertices);
#endif

  glGenBuffers(1, &gfxObj->m_index_vbo);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gfxObj->m_index_vbo);
  int indexBufferSizeInBytes = gfxObj->m_numIndices * sizeof(int);

  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBufferSizeInBytes, NULL,
               GL_STATIC_DRAW);
  glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, indexBufferSizeInBytes, indices);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glGenVertexArrays(1, &gfxObj->m_cube_vao);
  glBindVertexArray(gfxObj->m_cube_vao);
  glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  return m_graphicsInstances.size() - 1;
}

void TinyGLInstancingRenderer::init_shaders() {
  int POSITION_BUFFER_SIZE =
      (m_data->m_maxNumObjectCapacity * sizeof(float) * 4);
  int ORIENTATION_BUFFER_SIZE =
      (m_data->m_maxNumObjectCapacity * sizeof(float) * 4);
  int COLOR_BUFFER_SIZE = (m_data->m_maxNumObjectCapacity * sizeof(float) * 4);
  int SCALE_BUFFER_SIZE = (m_data->m_maxNumObjectCapacity * sizeof(float) * 4);

  {
    triangleShaderProgram =
        gltLoadShaderPair(triangleVertexShaderTextInit.c_str(), triangleFragmentShaderInit.c_str());

    // triangle_vpos_location = glGetAttribLocation(triangleShaderProgram,
    // "vPos"); triangle_vUV_location =
    // glGetAttribLocation(triangleShaderProgram, "vUV");

    triangle_mvp_location = glGetUniformLocation(triangleShaderProgram, "MVP");
    triangle_vcol_location =
        glGetUniformLocation(triangleShaderProgram, "vCol");

    glLinkProgram(triangleShaderProgram);
    glUseProgram(triangleShaderProgram);

    glGenVertexArrays(1, &triangleVertexArrayObject);
    glBindVertexArray(triangleVertexArrayObject);

    glGenBuffers(1, &triangleVertexBufferObject);
    glGenBuffers(1, &triangleIndexVbo);

    int sz = MAX_TRIANGLES_IN_BATCH * sizeof(GfxVertexFormat0);
    glBindVertexArray(triangleVertexArrayObject);
    glBindBuffer(GL_ARRAY_BUFFER, triangleVertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER, sz, 0, GL_DYNAMIC_DRAW);

    glBindVertexArray(0);
  }

  linesShader = gltLoadShaderPair(linesVertexShader, linesFragmentShader);
  lines_ModelViewMatrix = glGetUniformLocation(linesShader, "ModelViewMatrix");
  lines_ProjectionMatrix =
      glGetUniformLocation(linesShader, "ProjectionMatrix");
  lines_colour = glGetUniformLocation(linesShader, "colour");
  lines_position = glGetAttribLocation(linesShader, "position");
  glLinkProgram(linesShader);
  glUseProgram(linesShader);

  {
    glGenVertexArrays(1, &linesVertexArrayObject);
    glBindVertexArray(linesVertexArrayObject);

    glGenBuffers(1, &linesVertexBufferObject);
    glGenBuffers(1, &linesIndexVbo);

    int sz = MAX_LINES_IN_BATCH * sizeof(TinyVector3f);
    glBindVertexArray(linesVertexArrayObject);
    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER, sz, 0, GL_DYNAMIC_DRAW);

    glBindVertexArray(0);
  }
  {
    glGenVertexArrays(1, &lineVertexArrayObject);
    glBindVertexArray(lineVertexArrayObject);

    glGenBuffers(1, &lineVertexBufferObject);
    glGenBuffers(1, &lineIndexVbo);

    int sz = MAX_POINTS_IN_BATCH * sizeof(TinyVector3f);
    glBindVertexArray(lineVertexArrayObject);
    glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER, sz, 0, GL_DYNAMIC_DRAW);

    glBindVertexArray(0);
  }

  // glGetIntegerv(GL_ALIASED_LINE_WIDTH_RANGE, range);
  glGetIntegerv(GL_SMOOTH_LINE_WIDTH_RANGE, lineWidthRange);

  projectiveTextureInstancingShader =
      gltLoadShaderPair(projectiveTextureInstancingVertexShader,
                        projectiveTextureInstancingFragmentShader);

  glLinkProgram(projectiveTextureInstancingShader);
  glUseProgram(projectiveTextureInstancingShader);
  projectiveTexture_ViewMatrixInverse = glGetUniformLocation(
      projectiveTextureInstancingShader, "ViewMatrixInverse");
  projectiveTexture_ModelViewMatrix = glGetUniformLocation(
      projectiveTextureInstancingShader, "ModelViewMatrix");
  projectiveTexture_lightSpecularIntensity = glGetUniformLocation(
      projectiveTextureInstancingShader, "lightSpecularIntensityIn");
  projectiveTexture_materialSpecularColor = glGetUniformLocation(
      projectiveTextureInstancingShader, "materialSpecularColorIn");
  projectiveTexture_MVP =
      glGetUniformLocation(projectiveTextureInstancingShader, "MVP");
  projectiveTexture_ProjectionMatrix = glGetUniformLocation(
      projectiveTextureInstancingShader, "ProjectionMatrix");
  projectiveTexture_TextureMVP =
      glGetUniformLocation(projectiveTextureInstancingShader, "TextureMVP");
  projectiveTexture_uniform_texture_diffuse =
      glGetUniformLocation(projectiveTextureInstancingShader, "Diffuse");
  projectiveTexture_shadowMap =
      glGetUniformLocation(projectiveTextureInstancingShader, "shadowMap");
  projectiveTexture_lightPosIn =
      glGetUniformLocation(projectiveTextureInstancingShader, "lightPosIn");
  projectiveTexture_cameraPositionIn = glGetUniformLocation(
      projectiveTextureInstancingShader, "cameraPositionIn");
  projectiveTexture_materialShininessIn = glGetUniformLocation(
      projectiveTextureInstancingShader, "materialShininessIn");

  glUseProgram(0);

  useShadowMapInstancingShader = gltLoadShaderPair(
      useShadowMapInstancingVertexShaderInit.c_str(), useShadowMapInstancingFragmentShaderInit.c_str());

  glLinkProgram(useShadowMapInstancingShader);
  glUseProgram(useShadowMapInstancingShader);
  useShadow_ViewMatrixInverse =
      glGetUniformLocation(useShadowMapInstancingShader, "ViewMatrixInverse");
  useShadow_ModelViewMatrix =
      glGetUniformLocation(useShadowMapInstancingShader, "ModelViewMatrix");
  useShadow_lightSpecularIntensity = glGetUniformLocation(
      useShadowMapInstancingShader, "lightSpecularIntensityIn");
  useShadow_materialSpecularColor = glGetUniformLocation(
      useShadowMapInstancingShader, "materialSpecularColorIn");
  useShadow_MVP = glGetUniformLocation(useShadowMapInstancingShader, "MVP");
  useShadow_ProjectionMatrix =
      glGetUniformLocation(useShadowMapInstancingShader, "ProjectionMatrix");
  useShadow_DepthBiasModelViewMatrix = glGetUniformLocation(
      useShadowMapInstancingShader, "DepthBiasModelViewProjectionMatrix");
  useShadow_uniform_texture_diffuse =
      glGetUniformLocation(useShadowMapInstancingShader, "Diffuse");
  useShadow_shadowMap =
      glGetUniformLocation(useShadowMapInstancingShader, "shadowMap");
  useShadow_lightPosIn =
      glGetUniformLocation(useShadowMapInstancingShader, "lightPosIn");
  useShadow_cameraPositionIn =
      glGetUniformLocation(useShadowMapInstancingShader, "cameraPositionIn");
  useShadow_materialShininessIn =
      glGetUniformLocation(useShadowMapInstancingShader, "materialShininessIn");

  createShadowMapInstancingShader =
      gltLoadShaderPair(createShadowMapInstancingVertexShaderInit.c_str(),
                        createShadowMapInstancingFragmentShaderInit.c_str());
  glLinkProgram(createShadowMapInstancingShader);
  glUseProgram(createShadowMapInstancingShader);
  createShadow_depthMVP =
      glGetUniformLocation(createShadowMapInstancingShader, "depthMVP");

  glUseProgram(0);

  segmentationMaskInstancingShader =
      gltLoadShaderPair(segmentationMaskInstancingVertexShaderInit.c_str(),
                        segmentationMaskInstancingFragmentShaderInit.c_str());
  glLinkProgram(segmentationMaskInstancingShader);
  glUseProgram(segmentationMaskInstancingShader);

  segmentationMaskModelViewMatrix =
      glGetUniformLocation(segmentationMaskInstancingShader, "ModelViewMatrix");
  segmentationMaskProjectionMatrix = glGetUniformLocation(
      segmentationMaskInstancingShader, "ProjectionMatrix");

  glUseProgram(0);

  instancingShader =
      gltLoadShaderPair(instancingVertexShaderInit.c_str(), instancingFragmentShaderInit.c_str());
  glLinkProgram(instancingShader);
  glUseProgram(instancingShader);
  ModelViewMatrix = glGetUniformLocation(instancingShader, "ModelViewMatrix");
  ProjectionMatrix = glGetUniformLocation(instancingShader, "ProjectionMatrix");
  uniform_texture_diffuse = glGetUniformLocation(instancingShader, "Diffuse");
  regularLightDirIn = glGetUniformLocation(instancingShader, "lightDirIn");

  glUseProgram(0);

  instancingShaderPointSprite =
      gltLoadShaderPair(pointSpriteVertexShader, pointSpriteFragmentShader);
  glUseProgram(instancingShaderPointSprite);
  ModelViewMatrixPointSprite =
      glGetUniformLocation(instancingShaderPointSprite, "ModelViewMatrix");
  ProjectionMatrixPointSprite =
      glGetUniformLocation(instancingShaderPointSprite, "ProjectionMatrix");
  screenWidthPointSprite =
      glGetUniformLocation(instancingShaderPointSprite, "screenWidth");

  glUseProgram(0);

  // GLuint offset = 0;

  glGenBuffers(1, &m_data->m_vbo);
  checkError("glGenBuffers");

  glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);

  int size = m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE +
             ORIENTATION_BUFFER_SIZE + COLOR_BUFFER_SIZE + SCALE_BUFFER_SIZE;
  m_data->m_vboSize = size;

  glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);  // GL_STATIC_DRAW);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void TinyGLInstancingRenderer::init() {
  assert(glGetError() == GL_NO_ERROR);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  assert(glGetError() == GL_NO_ERROR);

  //	glClearColor(float(0.),float(0.),float(0.4),float(0));

  assert(glGetError() == GL_NO_ERROR);

  assert(glGetError() == GL_NO_ERROR);

  {
    if (m_textureenabled) {
      if (!m_textureinitialized) {
        glActiveTexture(GL_TEXTURE0);

        GLubyte* image = new GLubyte[256 * 256 * 3];
        for (int y = 0; y < 256; ++y) {
          //					const int	t=y>>5;
          GLubyte* pi = image + y * 256 * 3;
          for (int x = 0; x < 256; ++x) {
            if (x < 2 || y < 2 || x > 253 || y > 253) {
              pi[0] = 255;  // 0;
              pi[1] = 255;  // 0;
              pi[2] = 255;  // 0;
            } else {
              pi[0] = 255;
              pi[1] = 255;
              pi[2] = 255;
            }

            /*
            const int		s=x>>5;
            const GLubyte	b=180;
            GLubyte			c=b+((s+t&1)&1)*(255-b);
            pi[0]=c;
            pi[1]=c;
            pi[2]=c;
            */

            pi += 3;
          }
        }

        glGenTextures(1, (GLuint*)&m_data->m_defaultTexturehandle);
        glBindTexture(GL_TEXTURE_2D, m_data->m_defaultTexturehandle);
        assert(glGetError() == GL_NO_ERROR);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, image);
        glGenerateMipmap(GL_TEXTURE_2D);

        assert(glGetError() == GL_NO_ERROR);

        delete[] image;
        m_textureinitialized = true;
      }

      assert(glGetError() == GL_NO_ERROR);

      glBindTexture(GL_TEXTURE_2D, m_data->m_defaultTexturehandle);
      assert(glGetError() == GL_NO_ERROR);
    } else {
      glDisable(GL_TEXTURE_2D);
      assert(glGetError() == GL_NO_ERROR);
    }
  }
  // glEnable(GL_COLOR_MATERIAL);

  assert(glGetError() == GL_NO_ERROR);

  //	  glEnable(GL_CULL_FACE);
  //	  glCullFace(GL_BACK);
}

void TinyGLInstancingRenderer::resize(int width, int height) {
  m_screenWidth = width;
  m_screenHeight = height;
}

const TinyCamera* TinyGLInstancingRenderer::get_active_camera() const {
  return m_data->m_activeCamera;
}

TinyCamera* TinyGLInstancingRenderer::get_active_camera() {
  return m_data->m_activeCamera;
}

void TinyGLInstancingRenderer::set_active_camera(TinyCamera* cam) {
  m_data->m_activeCamera = cam;
}

void TinyGLInstancingRenderer::set_camera(const TinyCamera& cam) {
  m_data->m_activeCamera->copy_data(cam);
}

void TinyGLInstancingRenderer::set_light_specular_intensity(
    const float lightSpecularIntensity[3]) {
  m_data->m_lightSpecularIntensity[0] = lightSpecularIntensity[0];
  m_data->m_lightSpecularIntensity[1] = lightSpecularIntensity[1];
  m_data->m_lightSpecularIntensity[2] = lightSpecularIntensity[2];
}

void TinyGLInstancingRenderer::set_light_position(const float lightPos[3]) {
  m_data->m_lightPos[0] = lightPos[0];
  m_data->m_lightPos[1] = lightPos[1];
  m_data->m_lightPos[2] = lightPos[2];
}

void TinyGLInstancingRenderer::set_shadow_map_resolution(
    int shadowMapResolution) {
  m_data->m_shadowMapWidth = shadowMapResolution;
  m_data->m_shadowMapHeight = shadowMapResolution;
  m_data->m_updateShadowMap = true;
}

void TinyGLInstancingRenderer::set_shadow_map_world_size(float worldSize) {
  m_data->m_shadowMapWorldSize = worldSize;
  m_data->m_updateShadowMap = true;
}

void TinyGLInstancingRenderer::set_light_position(const double lightPos[3]) {
  m_data->m_lightPos[0] = lightPos[0];
  m_data->m_lightPos[1] = lightPos[1];
  m_data->m_lightPos[2] = lightPos[2];
}

void TinyGLInstancingRenderer::set_projective_texture_matrices(
    const float viewMatrix[16], const float projectionMatrix[16]) {
  for (int i = 0; i < 16; i++) {
    m_data->m_projectiveTextureViewMatrix[i] = viewMatrix[i];
    m_data->m_projectiveTextureProjectionMatrix[i] = projectionMatrix[i];
  }
}

void TinyGLInstancingRenderer::setProjectiveTexture(bool useProjectiveTexture) {
  m_data->m_useProjectiveTexture = useProjectiveTexture;
}

void TinyGLInstancingRenderer::update_camera(int upAxis) {
  assert(glGetError() == GL_NO_ERROR);
  m_upAxis = upAxis;

  m_data->m_activeCamera->set_camera_up_axis(upAxis);
  m_data->m_activeCamera->set_aspect_ratio((float)m_screenWidth /
                                           (float)m_screenHeight);
  m_data->m_defaultCamera1.update();
  m_data->m_activeCamera->get_camera_projection_matrix(
      m_data->m_projectionMatrix);
  m_data->m_activeCamera->get_camera_view_matrix(m_data->m_viewMatrix);
  float viewMat[16];
  float viewMatInverse[16];

  for (int i = 0; i < 16; i++) {
    viewMat[i] = m_data->m_viewMatrix[i];
  }
  TinyPosef tr;
  setFromOpenGLMatrix(tr, viewMat);
  tr.inverse();
  getOpenGLMatrix(tr, viewMatInverse);
  for (int i = 0; i < 16; i++) {
    m_data->m_viewMatrixInverse[i] = viewMatInverse[i];
  }
}

void TinyGLInstancingRenderer::get_projection_matrix(float projMatrix[16]) const {
  for (int i = 0; i < 16; i++) {
    projMatrix[i] = m_data->m_projectionMatrix[i];
  }
}

void TinyGLInstancingRenderer::set_projection_matrix(const float projMatrix[16]) {
  for (int i = 0; i < 16; i++) {
    m_data->m_projectionMatrix[i] = projMatrix[i];
  }
}

void TinyGLInstancingRenderer::get_view_matrix(float viewMatrix[16]) const {
  for (int i = 0; i < 16; i++) {
    viewMatrix[i] = m_data->m_viewMatrix[i];
  }
}

void TinyGLInstancingRenderer::set_view_matrix(const float viewMatrix[16]) {
  for (int i = 0; i < 16; i++) {
    m_data->m_viewMatrix[i] = viewMatrix[i];
  }
  TinyPosef tr;
  setFromOpenGLMatrix(tr, viewMatrix);
  tr.inverse();
  float viewMatInverse[16];
  getOpenGLMatrix(tr, viewMatInverse);
  for (int i = 0; i < 16; i++) {
    m_data->m_viewMatrixInverse[i] = viewMatInverse[i];
  }
}

void writeTextureToPng(int textureWidth, int textureHeight,
                       const char* fileName, int numComponents) {
  assert(glGetError() == GL_NO_ERROR);
  glPixelStorei(GL_PACK_ALIGNMENT, 4);

  glReadBuffer(GL_NONE);
  float* orgPixels =
      (float*)malloc(textureWidth * textureHeight * numComponents * 4);
  char* pixels =
      (char*)malloc(textureWidth * textureHeight * numComponents * 4);
  glReadPixels(0, 0, textureWidth, textureHeight, GL_DEPTH_COMPONENT, GL_FLOAT,
               orgPixels);
  assert(glGetError() == GL_NO_ERROR);
  for (int j = 0; j < textureHeight; j++) {
    for (int i = 0; i < textureWidth; i++) {
      float val = orgPixels[(j * textureWidth + i)];
      if (val != 1.f) {
        // printf("val[%d,%d]=%f\n", i,j,val);
      }
      pixels[(j * textureWidth + i) * numComponents] =
          char(orgPixels[(j * textureWidth + i)] * 255.f);
      pixels[(j * textureWidth + i) * numComponents + 1] = 0;  // 255.f;
      pixels[(j * textureWidth + i) * numComponents + 2] = 0;  // 255.f;
      pixels[(j * textureWidth + i) * numComponents + 3] = 127;

      // pixels[(j*textureWidth+i)*+1]=val;
      // pixels[(j*textureWidth+i)*numComponents+2]=val;
      // pixels[(j*textureWidth+i)*numComponents+3]=255;
    }

    /*	pixels[(j*textureWidth+j)*numComponents]=255;
            pixels[(j*textureWidth+j)*numComponents+1]=0;
            pixels[(j*textureWidth+j)*numComponents+2]=0;
            pixels[(j*textureWidth+j)*numComponents+3]=255;
            */
  }
  if (0) {
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
          pixels[((textureHeight - j - 1) * textureWidth + i) * numComponents +
                 c] = tmp;
        }
      }
    }
  }

  stbi_write_png(fileName, textureWidth, textureHeight, numComponents, pixels,
                 textureWidth * numComponents);

  free(pixels);
}

void TinyGLInstancingRenderer::render_scene() {
  // avoid some Intel driver on a Macbook Pro to lock-up
  // todo: figure out what is going on on that machine

  // glFlush();

  std::vector<TinyViewportTile> tiles;
  
  render_scene2(tiles);
}

void TinyGLInstancingRenderer::render_scene2(std::vector<TinyViewportTile>& tiles) {
  // avoid some Intel driver on a Macbook Pro to lock-up
  // todo: figure out what is going on on that machine

  // glFlush();

  if (m_data->m_useProjectiveTexture) {
    render_scene_internal(tiles, B3_USE_PROJECTIVE_TEXTURE_RENDERMODE);
  } else {
    if (useShadowMap) {
      render_scene_internal(tiles, B3_CREATE_SHADOWMAP_RENDERMODE);

      if (m_planeReflectionShapeIndex >= 0) {
        /* Don't update color or depth. */
        glDisable(GL_DEPTH_TEST);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

        /* Draw 1 into the stencil buffer. */
        glEnable(GL_STENCIL_TEST);
        glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
        glStencilFunc(GL_ALWAYS, 1, 0xffffffff);

        /* Now render floor; floor pixels just get their stencil set to 1. */
        render_scene_internal(tiles, B3_USE_SHADOWMAP_RENDERMODE_REFLECTION_PLANE);

        /* Re-enable update of color and depth. */
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glEnable(GL_DEPTH_TEST);

        /* Now, only render where stencil is set to 1. */
        glStencilFunc(GL_EQUAL, 1, 0xffffffff); /* draw if ==1 */
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

        // draw the reflection objects
        render_scene_internal(tiles, B3_USE_SHADOWMAP_RENDERMODE_REFLECTION);

        glDisable(GL_STENCIL_TEST);
      }

      render_scene_internal(tiles, B3_USE_SHADOWMAP_RENDERMODE);
    } else {
      render_scene_internal(tiles);
    }
  }
}
struct PointerCaster {
  union {
    int m_baseIndex;
    GLvoid* m_pointer;
  };

  PointerCaster() : m_pointer(0) {}
};

#if 0
static void    b3CreateFrustum(
                        float left,
                        float right,
                        float bottom,
                        float top,
                        float nearVal,
                        float farVal,
                        float frustum[16])
{

    frustum[0*4+0] = (float(2) * nearVal) / (right - left);
    frustum[0*4+1] = float(0);
    frustum[0*4+2] = float(0);
    frustum[0*4+3] = float(0);

    frustum[1*4+0] = float(0);
    frustum[1*4+1] = (float(2) * nearVal) / (top - bottom);
    frustum[1*4+2] = float(0);
    frustum[1*4+3] = float(0);

    frustum[2*4+0] = (right + left) / (right - left);
    frustum[2*4+1] = (top + bottom) / (top - bottom);
    frustum[2*4+2] = -(farVal + nearVal) / (farVal - nearVal);
    frustum[2*4+3] = float(-1);

    frustum[3*4+0] = float(0);
    frustum[3*4+1] = float(0);
    frustum[3*4+2] = -(float(2) * farVal * nearVal) / (farVal - nearVal);
    frustum[3*4+3] = float(0);

}
#endif

static void b3Matrix4x4Mul(GLfloat aIn[4][4], GLfloat bIn[4][4],
                           GLfloat result[4][4]) {
  for (int j = 0; j < 4; j++)
    for (int i = 0; i < 4; i++)
      result[j][i] = aIn[0][i] * bIn[j][0] + aIn[1][i] * bIn[j][1] +
                     aIn[2][i] * bIn[j][2] + aIn[3][i] * bIn[j][3];
}

static void b3Matrix4x4Mul16(GLfloat aIn[16], GLfloat bIn[16],
                             GLfloat result[16]) {
  for (int j = 0; j < 4; j++)
    for (int i = 0; i < 4; i++)
      result[j * 4 + i] =
          aIn[0 * 4 + i] * bIn[j * 4 + 0] + aIn[1 * 4 + i] * bIn[j * 4 + 1] +
          aIn[2 * 4 + i] * bIn[j * 4 + 2] + aIn[3 * 4 + i] * bIn[j * 4 + 3];
}

static void b3CreateDiagonalMatrix(GLfloat value, GLfloat result[4][4]) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == j) {
        result[i][j] = value;
      } else {
        result[i][j] = 0.f;
      }
    }
  }
}

static void b3CreateOrtho(GLfloat left, GLfloat right, GLfloat bottom,
                          GLfloat top, GLfloat zNear, GLfloat zFar,
                          GLfloat result[4][4]) {
  b3CreateDiagonalMatrix(1.f, result);

  result[0][0] = 2.f / (right - left);
  result[1][1] = 2.f / (top - bottom);
  result[2][2] = -2.f / (zFar - zNear);
  result[3][0] = -(right + left) / (right - left);
  result[3][1] = -(top + bottom) / (top - bottom);
  result[3][2] = -(zFar + zNear) / (zFar - zNear);
}

static void b3CreateLookAt(const TinyVector3f& eye, const TinyVector3f& center,
                           const TinyVector3f& up, GLfloat result[16]) {
  TinyVector3f f = (center - eye).normalized();
  TinyVector3f u = up.normalized();
  TinyVector3f s = (f.cross(u)).normalized();
  u = s.cross(f);

  result[0 * 4 + 0] = s.x();
  result[1 * 4 + 0] = s.y();
  result[2 * 4 + 0] = s.z();

  result[0 * 4 + 1] = u.x();
  result[1 * 4 + 1] = u.y();
  result[2 * 4 + 1] = u.z();

  result[0 * 4 + 2] = -f.x();
  result[1 * 4 + 2] = -f.y();
  result[2 * 4 + 2] = -f.z();

  result[0 * 4 + 3] = 0.f;
  result[1 * 4 + 3] = 0.f;
  result[2 * 4 + 3] = 0.f;

  result[3 * 4 + 0] = -s.dot(eye);
  result[3 * 4 + 1] = -u.dot(eye);
  result[3 * 4 + 2] = f.dot(eye);
  result[3 * 4 + 3] = 1.f;
}

void TinyGLInstancingRenderer::draw_textured_triangle_mesh(
    float worldPosition[3], float worldOrientation[4], const float* vertices,
    int numvertices, const unsigned int* indices, int numIndices,
    float colorRGBA[4], int textureIndex, int vertexLayout) {
  int sz = sizeof(GfxVertexFormat0);

  glActiveTexture(GL_TEXTURE0);
  activate_texture(textureIndex);
  checkError("activate_texture");

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glUseProgram(triangleShaderProgram);

  TinyQuaternionf orn(worldOrientation[0], worldOrientation[1],
                      worldOrientation[2], worldOrientation[3]);
  TinyVector3f pos =
      TinyVector3f(worldPosition[0], worldPosition[1], worldPosition[2]);

  TinyPosef worldTrans(pos, orn);
  float worldMatUnk[16];
  getOpenGLMatrix(worldTrans, worldMatUnk);
  float modelMat[16];
  for (int i = 0; i < 16; i++) {
    modelMat[i] = worldMatUnk[i];
  }
  float viewProjection[16];
  b3Matrix4x4Mul16(m_data->m_projectionMatrix, m_data->m_viewMatrix,
                   viewProjection);
  float MVP[16];
  b3Matrix4x4Mul16(viewProjection, modelMat, MVP);
  glUniformMatrix4fv(triangle_mvp_location, 1, GL_FALSE, (const GLfloat*)MVP);
  checkError("glUniformMatrix4fv");

  glUniform3f(triangle_vcol_location, colorRGBA[0], colorRGBA[1], colorRGBA[2]);
  checkError("glUniform3f");

  glBindVertexArray(triangleVertexArrayObject);
  checkError("glBindVertexArray");

  glBindBuffer(GL_ARRAY_BUFFER, triangleVertexBufferObject);
  checkError("glBindBuffer");

  glBufferData(GL_ARRAY_BUFFER, sizeof(GfxVertexFormat0) * numvertices, 0,
               GL_DYNAMIC_DRAW);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GfxVertexFormat0) * numvertices,
                  vertices);

  PointerCaster posCast;
  posCast.m_baseIndex = 0;
  PointerCaster uvCast;
  uvCast.m_baseIndex = 8 * sizeof(float);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GfxVertexFormat0),
                        posCast.m_pointer);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(GfxVertexFormat0),
                        uvCast.m_pointer);
  checkError("glVertexAttribPointer");
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  glVertexAttribDivisor(0, 0);
  glVertexAttribDivisor(1, 0);
  checkError("glVertexAttribDivisor");

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triangleIndexVbo);
  int indexBufferSizeInBytes = numIndices * sizeof(int);

  glBufferData(GL_ELEMENT_ARRAY_BUFFER, numIndices * sizeof(int), NULL,
               GL_DYNAMIC_DRAW);
  glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, numIndices * sizeof(int),
                  indices);

  glDrawElements(GL_TRIANGLES, numIndices, GL_UNSIGNED_INT, 0);

  // glDrawElements(GL_TRIANGLES, numIndices, GL_UNSIGNED_INT,indices);
  checkError("glDrawElements");

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);
  glUseProgram(0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
  checkError("glBindVertexArray");
}

void TinyGLInstancingRenderer::draw_point(const TinyVector3f& position,
                                          const TinyVector3f& color,
                                          float pointDrawSize) {
  draw_points(&position, color, 1, 3 * sizeof(float), pointDrawSize);
}
void TinyGLInstancingRenderer::draw_points(const TinyVector3f* positions,
                                           const TinyVector3f& color,
                                           int numPoints,
                                           int pointStrideInBytes,
                                           float pointDrawSize) {
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);

  assert(glGetError() == GL_NO_ERROR);
  glUseProgram(linesShader);
  glUniformMatrix4fv(lines_ProjectionMatrix, 1, false,
                     &m_data->m_projectionMatrix[0]);
  glUniformMatrix4fv(lines_ModelViewMatrix, 1, false, &m_data->m_viewMatrix[0]);
  glUniform4f(lines_colour, color[0], color[1], color[2], 1);

  glPointSize(pointDrawSize);
  glBindVertexArray(lineVertexArrayObject);

  glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);

  int maxPointsInBatch = MAX_POINTS_IN_BATCH;
  int remainingPoints = numPoints;
  int offsetNumPoints = 0;
  while (1) {
    int curPointsInBatch = TinyMin(maxPointsInBatch, remainingPoints);
    if (curPointsInBatch) {
      glBufferSubData(
          GL_ARRAY_BUFFER, 0, curPointsInBatch * pointStrideInBytes,
          positions + offsetNumPoints * (pointStrideInBytes / sizeof(float)));
      glEnableVertexAttribArray(0);
      int numFloats = 3;  // pointStrideInBytes / sizeof(float);
      glVertexAttribPointer(0, numFloats, GL_FLOAT, GL_FALSE,
                            pointStrideInBytes, 0);
      glDrawArrays(GL_POINTS, 0, curPointsInBatch);
      remainingPoints -= curPointsInBatch;
      offsetNumPoints += curPointsInBatch;
    } else {
      break;
    }
  }

  glBindVertexArray(0);
  glPointSize(1);
  glUseProgram(0);
}

void TinyGLInstancingRenderer::draw_lines(const TinyVector3f* positions,
                                          const TinyVector3f& color,
                                          int numPoints, int pointStrideInBytes,
                                          const unsigned int* indices,
                                          int numIndices, float lineWidthIn) {
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);

  float lineWidth = lineWidthIn;
  TinyClamp(lineWidth, (float)lineWidthRange[0], (float)lineWidthRange[1]);
  glLineWidth(lineWidth);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);

  assert(glGetError() == GL_NO_ERROR);
  glUseProgram(linesShader);
  glUniformMatrix4fv(lines_ProjectionMatrix, 1, false,
                     &m_data->m_projectionMatrix[0]);
  glUniformMatrix4fv(lines_ModelViewMatrix, 1, false, &m_data->m_viewMatrix[0]);
  glUniform4f(lines_colour, color.x(), color.y(), color.z(), 1);

  //	glPointSize(pointDrawSize);
  glBindVertexArray(linesVertexArrayObject);

  assert(glGetError() == GL_NO_ERROR);
  glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferObject);

  {
    glBufferData(GL_ARRAY_BUFFER, numPoints * pointStrideInBytes, 0,
                 GL_DYNAMIC_DRAW);

    glBufferSubData(GL_ARRAY_BUFFER, 0, numPoints * pointStrideInBytes,
                    positions);
    assert(glGetError() == GL_NO_ERROR);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, linesVertexBufferObject);
    glEnableVertexAttribArray(0);

    assert(glGetError() == GL_NO_ERROR);
    int numFloats = 3;
    glVertexAttribPointer(0, numFloats, GL_FLOAT, GL_FALSE, pointStrideInBytes,
                          0);
    assert(glGetError() == GL_NO_ERROR);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, linesIndexVbo);
    int indexBufferSizeInBytes = numIndices * sizeof(int);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBufferSizeInBytes, NULL,
                 GL_DYNAMIC_DRAW);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, indexBufferSizeInBytes,
                    indices);

    glDrawElements(GL_LINES, numIndices, GL_UNSIGNED_INT, 0);
  }

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  //	for (int i=0;i<numIndices;i++)
  //		printf("indicec[i]=%d]\n",indices[i]);
  assert(glGetError() == GL_NO_ERROR);
  glBindVertexArray(0);
  assert(glGetError() == GL_NO_ERROR);
  glPointSize(1);
  assert(glGetError() == GL_NO_ERROR);
  glUseProgram(0);
}

void TinyGLInstancingRenderer::draw_line(const TinyVector3f& from,
                                         const TinyVector3f& to,
                                         const TinyVector3f& color,
                                         float lineWidth) {
  assert(glGetError() == GL_NO_ERROR);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, 0);
  assert(glGetError() == GL_NO_ERROR);

  glUseProgram(linesShader);

  assert(glGetError() == GL_NO_ERROR);

  glUniformMatrix4fv(lines_ProjectionMatrix, 1, false,
                     &m_data->m_projectionMatrix[0]);
  glUniformMatrix4fv(lines_ModelViewMatrix, 1, false, &m_data->m_viewMatrix[0]);
  glUniform4f(lines_colour, color[0], color[1], color[2], 1);

  assert(glGetError() == GL_NO_ERROR);

  const float vertexPositions[] = {from[0], from[1], from[2], 1,
                                   to[0],   to[1],   to[2],   1};
  int sz = sizeof(vertexPositions);
  assert(glGetError() == GL_NO_ERROR);

  TinyClamp(lineWidth, (float)lineWidthRange[0], (float)lineWidthRange[1]);
  glLineWidth(lineWidth);

  assert(glGetError() == GL_NO_ERROR);

  glBindVertexArray(lineVertexArrayObject);
  assert(glGetError() == GL_NO_ERROR);

  glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);

  assert(glGetError() == GL_NO_ERROR);

  { glBufferSubData(GL_ARRAY_BUFFER, 0, sz, vertexPositions); }

  assert(glGetError() == GL_NO_ERROR);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, lineVertexBufferObject);
  assert(glGetError() == GL_NO_ERROR);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
  assert(glGetError() == GL_NO_ERROR);

  glDrawArrays(GL_LINES, 0, 2);
  assert(glGetError() == GL_NO_ERROR);

  glBindVertexArray(0);
  glLineWidth(1);

  assert(glGetError() == GL_NO_ERROR);
  glUseProgram(0);
}

struct SortableTransparentInstance {
  float m_projection;

  int m_shapeIndex;
  int m_instanceId;
  std::vector<int> tiles;
  std::vector<int> tile_instance;
    
};

struct TransparentDistanceSortPredicate {
  inline bool operator()(const SortableTransparentInstance& a,
                         const SortableTransparentInstance& b) const {
    return (a.m_projection > b.m_projection);
  }
};

TinyVector3f TinyGLInstancingRenderer::get_camera_position() const {
  GLfloat* viewMatrixInverse = (GLfloat*)&m_data->m_viewMatrixInverse[0];
  TinyVector3f cameraPos(viewMatrixInverse[12], viewMatrixInverse[13],
                         viewMatrixInverse[14]);
  return cameraPos;
}

TinyVector3f TinyGLInstancingRenderer::get_camera_target() const {
  // use 2 units forward, better would be to use the average of near/far plane?
  TinyVector3f cameraTarget = get_camera_position() + 2.f*get_camera_forward_vector();
  return cameraTarget;
}

TinyVector3f TinyGLInstancingRenderer::get_camera_forward_vector() const {
  GLfloat* viewMatrix = (GLfloat*)&m_data->m_viewMatrix[0];
  TinyVector3f forward(-viewMatrix[2], -viewMatrix[6], -viewMatrix[10]);
  return forward;
}

void TinyGLInstancingRenderer::render_scene_internal(std::vector<TinyViewportTile>& tiles, int orgRenderMode) {
  B3_PROFILE("render_scene_internal");
  int renderMode = orgRenderMode;
  bool reflectionPass = false;
  bool reflectionPlanePass = false;

  if (orgRenderMode == B3_USE_SHADOWMAP_RENDERMODE_REFLECTION_PLANE) {
    reflectionPlanePass = true;
    renderMode = B3_USE_SHADOWMAP_RENDERMODE;
  }
  if (orgRenderMode == B3_USE_SHADOWMAP_RENDERMODE_REFLECTION) {
    reflectionPass = true;
    renderMode = B3_USE_SHADOWMAP_RENDERMODE;
  }

  if (!useShadowMap) {
    renderMode = orgRenderMode;
  }

  if (orgRenderMode == B3_USE_PROJECTIVE_TEXTURE_RENDERMODE) {
    renderMode = B3_USE_PROJECTIVE_TEXTURE_RENDERMODE;
  }


{
  B3_PROFILE("rebuild tiles");
for (int tile = 0; tile< tiles.size();tile++)
{
  tiles[tile].internal_visual_instances.resize(tiles[tile].visual_instances.size());
    for (int vi = 0; vi< tiles[tile].visual_instances.size();vi++)
    {
      int viz_instance = tiles[tile].visual_instances[vi];

      TinyPublicGraphicsInstance* pg =
          viz_instance >= 0?
        m_data->m_publicGraphicsInstances.get_handle(viz_instance) : 0;

      if (pg) 
      {
        tiles[tile].internal_visual_instances[vi] = pg->m_internalInstanceIndex;
      } else
      {
        tiles[tile].internal_visual_instances[vi] = -1;
      }
    }
}                        
}
  //	glEnable(GL_DEPTH_TEST);

  GLint dims[4];
  glGetIntegerv(GL_VIEWPORT, dims);
  // we need to get the viewport dims, because on Apple Retina the viewport
  // dimension is different from screenWidth
  // printf("dims=%d,%d,%d,%d\n",dims[0],dims[1],dims[2],dims[3]);
  // Accept fragment if it closer to the camera than the former one
  // glDepthFunc(GL_LESS);

  // Cull triangles which normal is not towards the camera
  glEnable(GL_CULL_FACE);

  { init(); }

  assert(glGetError() == GL_NO_ERROR);

  float depthProjectionMatrix[4][4];
  GLfloat depthModelViewMatrix[4][4];
  // GLfloat depthModelViewMatrix2[4][4];

  // For projective texture mapping
  // float textureProjectionMatrix[4][4];
  // GLfloat textureModelViewMatrix[4][4];

  // Compute the MVP matrix from the light's point of view
  if (renderMode == B3_CREATE_SHADOWMAP_RENDERMODE) {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    if (m_data->m_shadowMap && m_data->m_updateShadowMap) {
      m_data->m_updateShadowMap = false;
      glDeleteTextures(1, &m_data->m_shadowTexture);
      delete m_data->m_shadowMap;
      m_data->m_shadowMap = 0;
    }
    if (!m_data->m_shadowMap) {
      glActiveTexture(GL_TEXTURE0);

      glGenTextures(1, &m_data->m_shadowTexture);
      glBindTexture(GL_TEXTURE_2D, m_data->m_shadowTexture);
      // glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT16,m_screenWidth,m_screenHeight,0,GL_DEPTH_COMPONENT,GL_FLOAT,0);
      // glTexImage2D(GL_TEXTURE_2D,0,GL_DEPTH_COMPONENT32,m_screenWidth,m_screenHeight,0,GL_DEPTH_COMPONENT,GL_FLOAT,0);

#ifdef OLD_SHADOWMAP_INIT
      glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, shadowMapWidth,
                   shadowMapHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
#else  //OLD_SHADOWMAP_INIT                                                              \
		//Reduce size of shadowMap if glTexImage2D call fails as may happen in some cases \
		//https://github.com/bulletphysics/bullet3/issues/40

      int size;
      glGetIntegerv(GL_MAX_TEXTURE_SIZE, &size);
      if (size < m_data->m_shadowMapWidth) {
        m_data->m_shadowMapWidth = size;
      }
      if (size < m_data->m_shadowMapHeight) {
        m_data->m_shadowMapHeight = size;
      }
      GLuint err;
      do {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16,
                     m_data->m_shadowMapWidth, m_data->m_shadowMapHeight, 0,
                     GL_DEPTH_COMPONENT, GL_FLOAT, 0);
        err = glGetError();
        if (err != GL_NO_ERROR) {
          m_data->m_shadowMapHeight >>= 1;
          m_data->m_shadowMapWidth >>= 1;
        }
      } while (err != GL_NO_ERROR && m_data->m_shadowMapWidth > 0);
#endif  // OLD_SHADOWMAP_INIT

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

      float l_ClampColor[] = {1.0, 1.0, 1.0, 1.0};
      glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, l_ClampColor);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
      //			glTexParameteri(GL_TEXTURE_2D,
      // GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                      GL_COMPARE_REF_TO_TEXTURE);

      m_data->m_shadowMap = new GLRenderToTexture();
      m_data->m_shadowMap->init(m_data->m_shadowMapWidth,
                                m_data->m_shadowMapHeight,
                                m_data->m_shadowTexture, RENDERTEXTURE_DEPTH);
    }
    m_data->m_shadowMap->enable();
    glViewport(0, 0, m_data->m_shadowMapWidth, m_data->m_shadowMapHeight);
    // glClearColor(1,1,1,1);
    glClear(GL_DEPTH_BUFFER_BIT);
    // glClearColor(0.3,0.3,0.3,1);

    //		m_data->m_shadowMap->disable();
    //	return;
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);  // Cull back-facing triangles -> draw only
                           // front-facing triangles

    assert(glGetError() == GL_NO_ERROR);
  } else {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
  }

  b3CreateOrtho(
      -m_data->m_shadowMapWorldSize, m_data->m_shadowMapWorldSize,
      -m_data->m_shadowMapWorldSize, m_data->m_shadowMapWorldSize, 1, 300,
      depthProjectionMatrix);  //-14,14,-14,14,1,200, depthProjectionMatrix);
  float depthViewMatrix[4][4];
  // TinyVector3f center = TinyVector3f(0, 0, 0);
  // m_data->m_activeCamera->get_camera_target_position(center);
  TinyVector3f center = get_camera_target();
  TinyVector3f camPos = get_camera_position();
  // float upf[3];
  // m_data->m_activeCamera->get_camera_up_vector(upf);
  TinyVector3f up, lightFwd;
  TinyVector3f lightDir = m_data->m_lightPos.normalized();
  lightDir.plane_space(up, lightFwd);
  //	TinyVector3f up = TinyVector3f(upf[0],upf[1],upf[2]);
  b3CreateLookAt(m_data->m_lightPos + center, center, up,
                 &depthViewMatrix[0][0]);
  // b3CreateLookAt(lightPos,m_data->m_cameraTargetPosition,TinyVector3f(0,1,0),(float*)depthModelViewMatrix2);

  GLfloat depthModelMatrix[4][4];
  b3CreateDiagonalMatrix(1.f, depthModelMatrix);

  b3Matrix4x4Mul(depthViewMatrix, depthModelMatrix, depthModelViewMatrix);

  GLfloat depthMVP[4][4];
  b3Matrix4x4Mul(depthProjectionMatrix, depthModelViewMatrix, depthMVP);

  GLfloat biasMatrix[4][4] = {{0.5, 0.0, 0.0, 0.0},
                              {0.0, 0.5, 0.0, 0.0},
                              {0.0, 0.0, 0.5, 0.0},
                              {0.5, 0.5, 0.5, 1.0}};

  GLfloat depthBiasMVP[4][4];
  b3Matrix4x4Mul(biasMatrix, depthMVP, depthBiasMVP);

  // TODO: Expose the projective texture matrix setup. Temporarily set it to be
  // the same as camera view projection matrix.
  GLfloat textureMVP[16];
  b3Matrix4x4Mul16(m_data->m_projectiveTextureProjectionMatrix,
                   m_data->m_projectiveTextureViewMatrix, textureMVP);

  // float m_frustumZNear=0.1;
  // float m_frustumZFar=100.f;

  // b3CreateFrustum(-m_frustumZNear, m_frustumZNear, -m_frustumZNear,
  // m_frustumZNear, m_frustumZNear,
  // m_frustumZFar,(float*)depthProjectionMatrix);

  // b3CreateLookAt(lightPos,m_data->m_cameraTargetPosition,TinyVector3f(0,0,1),(float*)depthModelViewMatrix);

  {
    //	update_camera();
    // m_data->m_activeCamera->get_camera_projection_matrix(
    //     m_data->m_projectionMatrix);
    // m_data->m_activeCamera->get_camera_view_matrix(m_data->m_viewMatrix);
  }

  assert(glGetError() == GL_NO_ERROR);

  //	glBindBuffer(GL_ARRAY_BUFFER, 0);
  {
    glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
    // glFlush();
  }
  assert(glGetError() == GL_NO_ERROR);

  int totalNumInstances = 0;

  for (int i = 0; i < m_graphicsInstances.size(); i++) {
    totalNumInstances += m_graphicsInstances[i]->m_numGraphicsInstances;
  }

  std::vector<SortableTransparentInstance> transparentInstances;
  {
    int curOffset = 0;
    // GLuint lastBindTexture = 0;

    transparentInstances.reserve(totalNumInstances);

    // float fwd[3];
    // m_data->m_activeCamera->get_camera_forward_vector(fwd);
    // TinyVector3f camForwardVec;
    // camForwardVec.setValue(fwd[0], fwd[1], fwd[2]);
    TinyVector3f camForwardVec = get_camera_forward_vector();

    for (int obj = 0; obj < m_graphicsInstances.size(); obj++) {
      b3GraphicsInstance* gfxObj = m_graphicsInstances[obj];

      if (gfxObj->m_numGraphicsInstances) {
        SortableTransparentInstance inst;

        inst.m_shapeIndex = obj;

        if ((gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) == 0) {
          inst.m_instanceId = curOffset;
          TinyVector3f centerPosition;
          centerPosition.setValue(
              m_data->m_instance_positions_ptr[inst.m_instanceId * 4 + 0],
              m_data->m_instance_positions_ptr[inst.m_instanceId * 4 + 1],
              m_data->m_instance_positions_ptr[inst.m_instanceId * 4 + 2]);
          centerPosition *= -1;  // reverse sort opaque instances
          inst.m_projection = centerPosition.dot(camForwardVec);
          transparentInstances.push_back(inst);
        } else {
          for (int i = 0; i < gfxObj->m_numGraphicsInstances; i++) {
            inst.m_instanceId = curOffset + i;
            TinyVector3f centerPosition;

            centerPosition.setValue(
                m_data->m_instance_positions_ptr[inst.m_instanceId * 4 + 0],
                m_data->m_instance_positions_ptr[inst.m_instanceId * 4 + 1],
                m_data->m_instance_positions_ptr[inst.m_instanceId * 4 + 2]);
            inst.m_projection = centerPosition.dot(camForwardVec);
            transparentInstances.push_back(inst);
          }
        }
        curOffset += gfxObj->m_numGraphicsInstances;
      }
    }
    TransparentDistanceSortPredicate sorter;

    // transparentInstances.quickSort(sorter);
    {
     B3_PROFILE("sort transparentInstances");
    std::sort(transparentInstances.begin(), transparentInstances.end(), sorter);
    }
  }


bool precompute_tiles = true;
if (precompute_tiles)
{
  B3_PROFILE("build tile_instances");
  for (int a=0;a<transparentInstances.size();a++)
  {
    int shapeIndex = transparentInstances[a].m_shapeIndex;
    transparentInstances[a].tiles.reserve(tiles.size());
    transparentInstances[a].tile_instance.reserve(tiles.size());
    
    b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];
    {
      for (int tile = 0; tile< tiles.size();tile++)
      {
          int instanceIdStart = transparentInstances[a].m_instanceId;
          int instanceIdEnd = instanceIdStart+gfxObj->m_numGraphicsInstances;
          
    
          for (int vi = 0; vi< tiles[tile].internal_visual_instances.size();vi++)
          {
          
          if ((tiles[tile].internal_visual_instances[vi] >= instanceIdStart)&& 
            (tiles[tile].internal_visual_instances[vi] < instanceIdEnd)
            )
          {
            transparentInstances[a].tiles.push_back(tile);
            transparentInstances[a].tile_instance.push_back(tiles[tile].internal_visual_instances[vi]);
          }
        }
      }
    }
    //printf("transparentInstances[%d].tiles.size()=%d\n", a, (int)transparentInstances[a].tiles.size());
  }
}

  // two passes: first for opaque instances, second for transparent ones.
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i < transparentInstances.size(); i++) {
      int shapeIndex = transparentInstances[i].m_shapeIndex;

      // during a reflectionPlanePass, only draw the plane, nothing else
      if (reflectionPlanePass) {
        if (shapeIndex != m_planeReflectionShapeIndex) continue;
      }

      b3GraphicsInstance* gfxObj = m_graphicsInstances[shapeIndex];

      // only draw stuff (opaque/transparent) if it is the right pass
      int drawThisPass =
          (pass == 0) == ((gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) == 0);

      // transparent objects don't cast shadows (to simplify things)
      if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
        if (renderMode == B3_CREATE_SHADOWMAP_RENDERMODE) drawThisPass = 0;
      }

      if (drawThisPass && gfxObj->m_numGraphicsInstances) {
        glActiveTexture(GL_TEXTURE0);
        GLuint curBindTexture = 0;
        if (gfxObj->m_flags & B3_INSTANCE_TEXTURE) {
          curBindTexture =
              m_data->m_textureHandles[gfxObj->m_textureIndex].m_glTexture;

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);

          if (m_data->m_textureHandles[gfxObj->m_textureIndex]
                  .m_enableFiltering) {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          } else {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
          }
        } else {
          curBindTexture = m_data->m_defaultTexturehandle;
        }

        // disable lazy evaluation, it just leads to bugs
        // if (lastBindTexture != curBindTexture)
        { glBindTexture(GL_TEXTURE_2D, curBindTexture); }
        // lastBindTexture = curBindTexture;

        assert(glGetError() == GL_NO_ERROR);
        //	int myOffset = gfxObj->m_instanceOffset*4*sizeof(float);

        int POSITION_BUFFER_SIZE = (totalNumInstances * sizeof(float) * 4);
        int ORIENTATION_BUFFER_SIZE = (totalNumInstances * sizeof(float) * 4);
        int COLOR_BUFFER_SIZE = (totalNumInstances * sizeof(float) * 4);
        //		int SCALE_BUFFER_SIZE =
        //(totalNumInstances*sizeof(float)*3);

        glBindVertexArray(gfxObj->m_cube_vao);

        int vertexStride = 9 * sizeof(float);
        PointerCaster vertex;
        vertex.m_baseIndex = gfxObj->m_vertexArrayOffset * vertexStride;

        // vertex position
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 9 * sizeof(float),
                              vertex.m_pointer);
        // instance_position
        glVertexAttribPointer(
            1, 4, GL_FLOAT, GL_FALSE, 0,
            (GLvoid*)(transparentInstances[i].m_instanceId * 4 * sizeof(float) +
                      m_data->m_maxShapeCapacityInBytes));
        // instance_quaternion
        glVertexAttribPointer(
            2, 4, GL_FLOAT, GL_FALSE, 0,
            (GLvoid*)(transparentInstances[i].m_instanceId * 4 * sizeof(float) +
                      m_data->m_maxShapeCapacityInBytes +
                      POSITION_BUFFER_SIZE));

        PointerCaster uv;
        uv.m_baseIndex = 7 * sizeof(float) + vertex.m_baseIndex;

        PointerCaster normal;
        normal.m_baseIndex = 4 * sizeof(float) + vertex.m_baseIndex;

        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 9 * sizeof(float),
                              uv.m_pointer);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float),
                              normal.m_pointer);
        // instance_color
        glVertexAttribPointer(
            5, 4, GL_FLOAT, GL_FALSE, 0,
            (GLvoid*)(transparentInstances[i].m_instanceId * 4 * sizeof(float) +
                      m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE +
                      ORIENTATION_BUFFER_SIZE));
        // instance_scale
        glVertexAttribPointer(
            6, 4, GL_FLOAT, GL_FALSE, 0,
            (GLvoid*)(transparentInstances[i].m_instanceId * 4 * sizeof(float) +
                      m_data->m_maxShapeCapacityInBytes + POSITION_BUFFER_SIZE +
                      ORIENTATION_BUFFER_SIZE + COLOR_BUFFER_SIZE));

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glEnableVertexAttribArray(3);
        glEnableVertexAttribArray(4);
        glEnableVertexAttribArray(5);
        glEnableVertexAttribArray(6);
        glVertexAttribDivisor(0, 0);
        glVertexAttribDivisor(1, 1);
        glVertexAttribDivisor(2, 1);
        glVertexAttribDivisor(3, 0);
        glVertexAttribDivisor(4, 0);
        glVertexAttribDivisor(5, 1);
        glVertexAttribDivisor(6, 1);

        int indexCount = gfxObj->m_numIndices;
        GLvoid* indexOffset = 0;

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gfxObj->m_index_vbo);
        {
          if (gfxObj->m_primitiveType == B3_GL_POINTS) {
            glUseProgram(instancingShaderPointSprite);
            glUniformMatrix4fv(ProjectionMatrixPointSprite, 1, false,
                               &m_data->m_projectionMatrix[0]);
            glUniformMatrix4fv(ModelViewMatrixPointSprite, 1, false,
                               &m_data->m_viewMatrix[0]);
            glUniform1f(screenWidthPointSprite, float(m_screenWidth));

            // glUniform1i(uniform_texture_diffusePointSprite, 0);
            assert(glGetError() == GL_NO_ERROR);
            glPointSize(20);

#ifndef __APPLE__
            glEnable(GL_POINT_SPRITE_ARB);
//					glTexEnvi(GL_POINT_SPRITE_ARB,
// GL_COORD_REPLACE_ARB, GL_TRUE);
#endif

            glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
            glDrawElementsInstanced(GL_POINTS, indexCount, GL_UNSIGNED_INT,
                                    indexOffset,
                                    gfxObj->m_numGraphicsInstances);
          } else {
            if (gfxObj->m_flags & B3_INSTANCE_DOUBLE_SIDED) {
              glDisable(GL_CULL_FACE);
            }

            switch (renderMode) {
              case B3_SEGMENTATION_MASK_RENDERMODE: {

                glUseProgram(segmentationMaskInstancingShader);
                if (tiles.size())
                {
                  if (precompute_tiles)
                    {
                       for (int nt = 0; nt < transparentInstances[i].tiles.size();nt++)
                            {
                              
                              int tile = transparentInstances[i].tiles[nt];
                              int instanceId = transparentInstances[i].tile_instance[nt];
                                 B3_PROFILE("tile_render_viewport_segmask");

                                glViewport(
                                  tiles[tile].viewport_dims[0],
                                  tiles[tile].viewport_dims[1],
                                  tiles[tile].viewport_dims[2],
                                  tiles[tile].viewport_dims[3]);

								glUniformMatrix4fv(segmentationMaskProjectionMatrix, 1, false,
									                &tiles[tile].projection_matrix[0]);
								glUniformMatrix4fv(segmentationMaskModelViewMatrix, 1, false,
									                &tiles[tile].view_matrix[0]);


                                //if (0) {
                                //  glDrawElementsInstancedBaseInstance(
                                //      GL_TRIANGLES, indexCount, GL_UNSIGNED_INT,
                                //      indexOffset, 1, qq);
                                //} else 
                                {
                                      glVertexAttribPointer(
                                          1, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes));
                                      glVertexAttribPointer(
                                          2, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          5, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          6, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE +
                                                    COLOR_BUFFER_SIZE));

                                      glDrawElements(GL_TRIANGLES, indexCount,
                                                    GL_UNSIGNED_INT, 0);
                                }
                              
                            }
                    } else
                      {
                        for (unsigned int qq = 0; qq < gfxObj->m_numGraphicsInstances;
                             qq++) {
                              
                              B3_PROFILE("for each qq");

                            for (int tile = 0; tile< tiles.size();tile++)
                            {
                              
                              B3_PROFILE("for each tile");

                                int instanceId = transparentInstances[i].m_instanceId+qq;

                                for (int vi = 0; vi< tiles[tile].internal_visual_instances.size();vi++)
                                {
                                  B3_PROFILE("for each tile visual instance");
                                
                                if (tiles[tile].internal_visual_instances[vi] == instanceId)
                                {
                                  B3_PROFILE("tile_render_viewport_segmask");

                                glViewport(
                                  tiles[tile].viewport_dims[0],
                                  tiles[tile].viewport_dims[1],
                                  tiles[tile].viewport_dims[2],
                                  tiles[tile].viewport_dims[3]);

								glUniformMatrix4fv(segmentationMaskProjectionMatrix, 1, false,
									                &tiles[tile].projection_matrix[0]);
								glUniformMatrix4fv(segmentationMaskModelViewMatrix, 1, false,
									                &tiles[tile].view_matrix[0]);


                                //if (0) {
                                //  glDrawElementsInstancedBaseInstance(
                                //      GL_TRIANGLES, indexCount, GL_UNSIGNED_INT,
                                //      indexOffset, 1, qq);
                                //} else 
                                {
                                      glVertexAttribPointer(
                                          1, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes));
                                      glVertexAttribPointer(
                                          2, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          5, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          6, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE +
                                                    COLOR_BUFFER_SIZE));

                                      glDrawElements(GL_TRIANGLES, indexCount,
                                                    GL_UNSIGNED_INT, 0);
                                }
                                }

                                }
                              }
                             }
                            }
                        glViewport(dims[0], dims[1], dims[2], dims[3]);
                } else
                {
                    glUniformMatrix4fv(segmentationMaskProjectionMatrix, 1, false,
                                       &m_data->m_projectionMatrix[0]);
                    glUniformMatrix4fv(segmentationMaskModelViewMatrix, 1, false,
                                       &m_data->m_viewMatrix[0]);
                    glDrawElementsInstanced(GL_TRIANGLES, indexCount,
                                            GL_UNSIGNED_INT, indexOffset,
                                            gfxObj->m_numGraphicsInstances);
                }
                break;
              }
              case B3_DEFAULT_RENDERMODE: {
                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  glDepthMask(false);
                  glEnable(GL_BLEND);
                  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                }

                glUseProgram(instancingShader);
                glUniformMatrix4fv(ProjectionMatrix, 1, false,
                                   &m_data->m_projectionMatrix[0]);
                glUniformMatrix4fv(ModelViewMatrix, 1, false,
                                   &m_data->m_viewMatrix[0]);

                TinyVector3f gLightDir = m_data->m_lightPos;
                gLightDir.normalize();
                glUniform3f(regularLightDirIn, gLightDir[0], gLightDir[1],
                            gLightDir[2]);

                glUniform1i(uniform_texture_diffuse, 0);

                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  int instanceId = transparentInstances[i].m_instanceId;
                  glVertexAttribPointer(
                      1, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes));
                  glVertexAttribPointer(
                      2, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE));
                  glVertexAttribPointer(
                      5, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE +
                                ORIENTATION_BUFFER_SIZE));
                  glVertexAttribPointer(
                      6, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE + ORIENTATION_BUFFER_SIZE +
                                COLOR_BUFFER_SIZE));

                  glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
                } else {

                    if (tiles.size()) {
                        
                        if (precompute_tiles)
                          {
                            for (int nt = 0; nt < transparentInstances[i].tiles.size();nt++)
                            {
                              
                              int tile = transparentInstances[i].tiles[nt];
                              int instanceId = transparentInstances[i].tile_instance[nt];
                              
                                B3_PROFILE("tile_render_viewpor");

                                glViewport(
                                  tiles[tile].viewport_dims[0],
                                  tiles[tile].viewport_dims[1],
                                  tiles[tile].viewport_dims[2],
                                  tiles[tile].viewport_dims[3]);

								glUniformMatrix4fv(ProjectionMatrix, 1, false,
									                &tiles[tile].projection_matrix[0]);
								glUniformMatrix4fv(ModelViewMatrix, 1, false,
									                &tiles[tile].view_matrix[0]);


                                //if (0) {
                                //  glDrawElementsInstancedBaseInstance(
                                //      GL_TRIANGLES, indexCount, GL_UNSIGNED_INT,
                                //      indexOffset, 1, qq);
                                //} else 
                                {
                                      glVertexAttribPointer(
                                          1, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes));
                                      glVertexAttribPointer(
                                          2, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          5, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          6, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE +
                                                    COLOR_BUFFER_SIZE));

                                      glDrawElements(GL_TRIANGLES, indexCount,
                                                    GL_UNSIGNED_INT, 0);
                                }
                            }
                          } else
                            {
                        B3_PROFILE("tile_render_search");
                        
                        //for (unsigned int qq = 0; qq < gfxObj->m_numGraphicsInstances;qq++) 
                        {

                            B3_PROFILE("for each qq");

                            for (int tile = 0; tile< tiles.size();tile++)
                            {
                              
                              B3_PROFILE("for each tile");

                                
                                  int instanceIdStart = transparentInstances[i].m_instanceId;
                                  int instanceIdEnd = instanceIdStart+gfxObj->m_numGraphicsInstances;
                                

                                for (int vi = 0; vi< tiles[tile].internal_visual_instances.size();vi++)
                                {
                                  
                                  B3_PROFILE("for each tile visual ");
                                
                                  if ((tiles[tile].internal_visual_instances[vi] >= instanceIdStart)&& 
                                    (tiles[tile].internal_visual_instances[vi] < instanceIdEnd)
                                  )
                                {
                                  
                                  int instanceId = tiles[tile].internal_visual_instances[vi];
                                  
                                  B3_PROFILE("tile_render_viewpor");

                                glViewport(
                                  tiles[tile].viewport_dims[0],
                                  tiles[tile].viewport_dims[1],
                                  tiles[tile].viewport_dims[2],
                                  tiles[tile].viewport_dims[3]);

								glUniformMatrix4fv(ProjectionMatrix, 1, false,
									                &tiles[tile].projection_matrix[0]);
								glUniformMatrix4fv(ModelViewMatrix, 1, false,
									                &tiles[tile].view_matrix[0]);


                                //if (0) {
                                //  glDrawElementsInstancedBaseInstance(
                                //      GL_TRIANGLES, indexCount, GL_UNSIGNED_INT,
                                //      indexOffset, 1, qq);
                                //} else 
                                {
                                      glVertexAttribPointer(
                                          1, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes));
                                      glVertexAttribPointer(
                                          2, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          5, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE));
                                      glVertexAttribPointer(
                                          6, 4, GL_FLOAT, GL_FALSE, 0,
                                          (GLvoid*)((instanceId)*4 * sizeof(float) +
                                                    m_data->m_maxShapeCapacityInBytes +
                                                    POSITION_BUFFER_SIZE +
                                                    ORIENTATION_BUFFER_SIZE +
                                                    COLOR_BUFFER_SIZE));

                                      glDrawElements(GL_TRIANGLES, indexCount,
                                                    GL_UNSIGNED_INT, 0);
                                }
                                }

                                }
                              }
                             }
                        }
                        glViewport(dims[0], dims[1], dims[2], dims[3]);
                      
                  } else {
                    
                    B3_PROFILE("glDrawElementsInstanced");
                    glDrawElementsInstanced(GL_TRIANGLES, indexCount,
                                            GL_UNSIGNED_INT, indexOffset,
                                            gfxObj->m_numGraphicsInstances);
                  }
                }

                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  glDisable(GL_BLEND);
                  glDepthMask(true);
                }

                break;
              }
              case B3_CREATE_SHADOWMAP_RENDERMODE: {
                glUseProgram(createShadowMapInstancingShader);
                glUniformMatrix4fv(createShadow_depthMVP, 1, false,
                                   &depthMVP[0][0]);
                glDrawElementsInstanced(GL_TRIANGLES, indexCount,
                                        GL_UNSIGNED_INT, indexOffset,
                                        gfxObj->m_numGraphicsInstances);
                break;
              }

              case B3_USE_SHADOWMAP_RENDERMODE: {
                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  glDepthMask(false);
                  glEnable(GL_BLEND);
                  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                }

                glUseProgram(useShadowMapInstancingShader);
                glUniformMatrix4fv(useShadow_ProjectionMatrix, 1, false,
                                   &m_data->m_projectionMatrix[0]);
                // glUniformMatrix4fv(useShadow_ModelViewMatrix, 1, false,
                // &m_data->m_viewMatrix[0]);
                // glUniformMatrix4fv(useShadow_ViewMatrixInverse, 1, false,
                // &m_data->m_viewMatrix[0]);
                // glUniformMatrix4fv(useShadow_ViewMatrixInverse, 1, false,
                // &m_data->m_viewMatrixInverse[0]);
                // glUniformMatrix4fv(useShadow_ModelViewMatrix, 1, false,
                // &m_data->m_viewMatrix[0]);

                glUniform3f(useShadow_lightSpecularIntensity,
                            m_data->m_lightSpecularIntensity[0],
                            m_data->m_lightSpecularIntensity[1],
                            m_data->m_lightSpecularIntensity[2]);
                glUniform3f(useShadow_materialSpecularColor,
                            gfxObj->m_materialSpecularColor[0],
                            gfxObj->m_materialSpecularColor[1],
                            gfxObj->m_materialSpecularColor[2]);

                float MVP[16];
                if (reflectionPass) {
                  // todo: create an API to select this reflection matrix, to
                  // allow reflection planes different from Z-axis up through
                  // (0,0,0)
                  float tmp[16];
                  float reflectionMatrix[16] = {1, 0, 0,  0, 0, 1, 0, 0,
                                                0, 0, -1, 0, 0, 0, 0, 1};
                  glCullFace(GL_FRONT);
                  b3Matrix4x4Mul16(m_data->m_viewMatrix, reflectionMatrix, tmp);
                  b3Matrix4x4Mul16(m_data->m_projectionMatrix, tmp, MVP);
                } else {
                  b3Matrix4x4Mul16(m_data->m_projectionMatrix,
                                   m_data->m_viewMatrix, MVP);
                  glCullFace(GL_BACK);
                }

                glUniformMatrix4fv(useShadow_MVP, 1, false, &MVP[0]);
                // gLightDir.normalize();
                glUniform3f(useShadow_lightPosIn, m_data->m_lightPos[0],
                            m_data->m_lightPos[1], m_data->m_lightPos[2]);
                // TinyVector3f camPos;
                // m_data->m_activeCamera->get_camera_position(camPos);
                glUniform3f(useShadow_cameraPositionIn, camPos[0], camPos[1],
                            camPos[2]);
                glUniform1f(useShadow_materialShininessIn,
                            gfxObj->m_materialShinyNess);

                glUniformMatrix4fv(useShadow_DepthBiasModelViewMatrix, 1, false,
                                   &depthBiasMVP[0][0]);
                glActiveTexture(GL_TEXTURE1);
                glBindTexture(GL_TEXTURE_2D, m_data->m_shadowTexture);
                glUniform1i(useShadow_shadowMap, 1);

                // sort transparent objects

                // gfxObj->m_instanceOffset

                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  int instanceId = transparentInstances[i].m_instanceId;
                  glVertexAttribPointer(
                      1, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes));
                  glVertexAttribPointer(
                      2, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE));
                  glVertexAttribPointer(
                      5, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE +
                                ORIENTATION_BUFFER_SIZE));
                  glVertexAttribPointer(
                      6, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE + ORIENTATION_BUFFER_SIZE +
                                COLOR_BUFFER_SIZE));
                  glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
                } else {
                  glDrawElementsInstanced(GL_TRIANGLES, indexCount,
                                          GL_UNSIGNED_INT, indexOffset,
                                          gfxObj->m_numGraphicsInstances);
                }

                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  glDisable(GL_BLEND);
                  glDepthMask(true);
                }
                glActiveTexture(GL_TEXTURE1);
                glBindTexture(GL_TEXTURE_2D, 0);

                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, 0);
                break;
              }
              case B3_USE_PROJECTIVE_TEXTURE_RENDERMODE: {
                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  glDepthMask(false);
                  glEnable(GL_BLEND);
                  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                }

                glUseProgram(projectiveTextureInstancingShader);
                glUniformMatrix4fv(projectiveTexture_ProjectionMatrix, 1, false,
                                   &m_data->m_projectionMatrix[0]);
                glUniform3f(projectiveTexture_lightSpecularIntensity,
                            m_data->m_lightSpecularIntensity[0],
                            m_data->m_lightSpecularIntensity[1],
                            m_data->m_lightSpecularIntensity[2]);
                glUniform3f(projectiveTexture_materialSpecularColor,
                            gfxObj->m_materialSpecularColor[0],
                            gfxObj->m_materialSpecularColor[1],
                            gfxObj->m_materialSpecularColor[2]);

                float MVP[16];
                if (reflectionPass) {
                  float tmp[16];
                  float reflectionMatrix[16] = {1, 0, 0,  0, 0, 1, 0, 0,
                                                0, 0, -1, 0, 0, 0, 0, 1};
                  glCullFace(GL_FRONT);
                  b3Matrix4x4Mul16(m_data->m_viewMatrix, reflectionMatrix, tmp);
                  b3Matrix4x4Mul16(m_data->m_projectionMatrix, tmp, MVP);
                } else {
                  b3Matrix4x4Mul16(m_data->m_projectionMatrix,
                                   m_data->m_viewMatrix, MVP);
                  glCullFace(GL_BACK);
                }

                glUniformMatrix4fv(projectiveTexture_MVP, 1, false, &MVP[0]);
                glUniform3f(projectiveTexture_lightPosIn, m_data->m_lightPos[0],
                            m_data->m_lightPos[1], m_data->m_lightPos[2]);
                // TinyVector3f camPos;
                // m_data->m_activeCamera->get_camera_position(camPos);
                glUniform3f(projectiveTexture_cameraPositionIn, camPos[0],
                            camPos[1], camPos[2]);
                glUniform1f(projectiveTexture_materialShininessIn,
                            gfxObj->m_materialShinyNess);

                glUniformMatrix4fv(projectiveTexture_TextureMVP, 1, false,
                                   &textureMVP[0]);

                // sort transparent objects
                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  int instanceId = transparentInstances[i].m_instanceId;
                  glVertexAttribPointer(
                      1, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes));
                  glVertexAttribPointer(
                      2, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE));
                  glVertexAttribPointer(
                      5, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE +
                                ORIENTATION_BUFFER_SIZE));
                  glVertexAttribPointer(
                      6, 4, GL_FLOAT, GL_FALSE, 0,
                      (GLvoid*)((instanceId)*4 * sizeof(float) +
                                m_data->m_maxShapeCapacityInBytes +
                                POSITION_BUFFER_SIZE + ORIENTATION_BUFFER_SIZE +
                                COLOR_BUFFER_SIZE));
                  glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
                } else {
                  glDrawElementsInstanced(GL_TRIANGLES, indexCount,
                                          GL_UNSIGNED_INT, indexOffset,
                                          gfxObj->m_numGraphicsInstances);
                }

                if (gfxObj->m_flags & B3_INSTANCE_TRANSPARANCY) {
                  glDisable(GL_BLEND);
                  glDepthMask(true);
                }

                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, 0);
                break;
              }
              default: {
                //	assert(0);
              }
            };
            if (gfxObj->m_flags & B3_INSTANCE_DOUBLE_SIDED) {
              glEnable(GL_CULL_FACE);
            }
          }

          // glDrawElementsInstanced(GL_LINE_LOOP, indexCount, GL_UNSIGNED_INT,
          // (void*)indexOffset, gfxObj->m_numGraphicsInstances);
        }
      }
    }
  }

  {
    // glFlush();
  }
  if (renderMode == B3_CREATE_SHADOWMAP_RENDERMODE) {
    //	writeTextureToPng(shadowMapWidth,shadowMapHeight,"shadowmap.png",4);
    m_data->m_shadowMap->disable();
    glBindFramebuffer(GL_FRAMEBUFFER, m_data->m_renderFrameBuffer);
    glViewport(dims[0], dims[1], dims[2], dims[3]);
  }

  assert(glGetError() == GL_NO_ERROR);
  {
    glUseProgram(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  glDisable(GL_CULL_FACE);
  assert(glGetError() == GL_NO_ERROR);
}

void TinyGLInstancingRenderer::cleanup_shaders() {}

void TinyGLInstancingRenderer::set_plane_reflection_shape_index(int index) {
  m_planeReflectionShapeIndex = index;
}

void TinyGLInstancingRenderer::enable_shadow_map() {
  glActiveTexture(GL_TEXTURE0);
  // glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, m_data->m_shadowTexture);
  // glBindTexture(GL_TEXTURE_2D, m_data->m_defaultTexturehandle);
}

void TinyGLInstancingRenderer::clear_z_buffer() {
  glClear(GL_DEPTH_BUFFER_BIT);
}

int TinyGLInstancingRenderer::get_max_shape_capacity() const {
  return m_data->m_maxShapeCapacityInBytes;
}
int TinyGLInstancingRenderer::get_instance_capacity() const {
  return m_data->m_maxNumObjectCapacity;
}

void TinyGLInstancingRenderer::set_render_frame_buffer(
    unsigned int renderFrameBuffer) {
  m_data->m_renderFrameBuffer = (GLuint)renderFrameBuffer;
}

int TinyGLInstancingRenderer::get_total_num_instances() const {
  return m_data->m_totalNumInstances;
}

#endif  // NO_OPENGL3
