#ifndef GL_INSTANCE_RENDERER_INTERNAL_DATA_H
#define GL_INSTANCE_RENDERER_INTERNAL_DATA_H

#include <vector>
#include "tiny_opengl_include.h"

struct GLInstanceRendererInternalData {
  std::vector<GLfloat> m_instance_positions_ptr;
  std::vector<GLfloat> m_instance_quaternion_ptr;
  std::vector<GLfloat> m_instance_colors_ptr;
  std::vector<GLfloat> m_instance_scale_ptr;

  int m_vboSize;
  GLuint m_vbo;
  int m_totalNumInstances;
  int m_maxNumObjectCapacity;
  int m_maxShapeCapacityInBytes;
};

#endif  // GL_INSTANCE_RENDERER_INTERNAL_DATA_H
