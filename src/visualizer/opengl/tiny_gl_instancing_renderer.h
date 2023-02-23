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

#ifndef GL_INSTANCING_RENDERER_H
#define GL_INSTANCING_RENDERER_H

#include <vector>

#include "tiny_camera.h"
#include "tiny_vertex_format.h"

enum { B3_GL_TRIANGLES = 1, B3_GL_POINTS };

enum {
  B3_INSTANCE_TRANSPARANCY = 1,
  B3_INSTANCE_TEXTURE = 2,
  B3_INSTANCE_DOUBLE_SIDED = 4,
};

enum {
  B3_DEFAULT_RENDERMODE = 1,
  // B3_WIREFRAME_RENDERMODE,
  B3_CREATE_SHADOWMAP_RENDERMODE,
  B3_USE_SHADOWMAP_RENDERMODE,
  B3_USE_SHADOWMAP_RENDERMODE_REFLECTION,
  B3_USE_SHADOWMAP_RENDERMODE_REFLECTION_PLANE,
  B3_USE_PROJECTIVE_TEXTURE_RENDERMODE,
  B3_SEGMENTATION_MASK_RENDERMODE,
};

struct TinyViewportTile
{
  std::vector<int> visual_instances;
  std::vector<int> internal_visual_instances;
  std::array<int, 4> viewport_dims;
  std::array<float, 16> view_matrix;
  std::array<float, 16> projection_matrix;
};

class TinyGLInstancingRenderer {
  std::vector<struct b3GraphicsInstance*> m_graphicsInstances;

  struct InternalDataRenderer* m_data;

  bool m_textureenabled;
  bool m_textureinitialized;

  int m_screenWidth;
  int m_screenHeight;

  int m_upAxis;

  int m_planeReflectionShapeIndex;

  int register_graphics_instance_internal(int shapeIndex,
                                          const ::TINY::TinyVector3f& position,
                                          const ::TINY::TinyQuaternionf& quaternion,
                                          const ::TINY::TinyVector3f& color,
                                          const ::TINY::TinyVector3f& scaling,
                                          float opacity = 1.f);
  

 public:

  void rebuild_graphics_instances();

  TinyGLInstancingRenderer(int m_maxObjectCapacity,
                           int maxShapeCapacityInBytes = 56 * 1024 * 1024);
  virtual ~TinyGLInstancingRenderer();

  virtual void init();

  virtual void render_scene();
  virtual void render_scene2(std::vector<TinyViewportTile>& tiles);
  virtual void render_scene_internal(std::vector<TinyViewportTile>& tiles, int orgRenderMode = B3_DEFAULT_RENDERMODE);

  void init_shaders();
  void cleanup_shaders();
  virtual void remove_all_instances();
  virtual void remove_graphics_instance(int instanceUid);

  virtual void update_shape(int shapeIndex, const float* vertices);

  /// vertices must be in the format x,y,z,w, nx,ny,nz, u,v
  
  virtual int register_shape1(std::vector<float>& vertices, std::vector<int>& indices, int textureIndex = -1, bool double_sided = false)
  {
      return register_shape(&vertices[0], vertices.size()/9, &indices[0], indices.size(), B3_GL_TRIANGLES, textureIndex, double_sided);
  }


  virtual int register_shape(const float* vertices, int numvertices,
                             const int* indices, int numIndices,
                             int primitiveType = B3_GL_TRIANGLES,
                             int textureIndex = -1,
                             bool double_sided = false);

  std::vector<int> get_shape_vertex_count() const;
  std::vector<int> get_shape_vertex_offsets() const;

  virtual int register_texture1(const std::vector<unsigned char>& texels, int width,
      int height, bool flipPixelsY = true)
  {
      return register_texture(&texels[0], width,height, flipPixelsY);
  }
  virtual int register_texture(const unsigned char* texels, int width,
                               int height, bool flipPixelsY = true);
  virtual void update_texture(int textureIndex, const unsigned char* texels,
                              bool flipPixelsY = true);
  virtual void activate_texture(int textureIndex);
  virtual void replace_texture(int shapeIndex, int textureId);
  virtual int get_shape_index_from_instance(int srcIndex);
  virtual void remove_texture(int textureIndex);

  /// position x,y,z, quaternion x,y,z,w, color r,g,b,a, scaling x,y,z
  virtual int register_graphics_instance(int shapeIndex,
                                         const ::TINY::TinyVector3f& position,
                                         const ::TINY::TinyQuaternionf& quaternion,
                                         const ::TINY::TinyVector3f& color,
                                         const ::TINY::TinyVector3f& scaling,
                                         float opacity = 1.f,
                                         bool rebuild = true);


   std::vector<int> register_graphics_instances(int shapeIndex,
                                         const std::vector<::TINY::TinyVector3f>& positions,
                                         const std::vector<::TINY::TinyQuaternionf>& quaternions,
                                         const std::vector<::TINY::TinyVector3f>& colors,
                                         const std::vector<::TINY::TinyVector3f>& scalings,
                                         float opacity = 1.f,
                                         bool rebuild=true);


  void write_transforms();

  virtual bool read_single_instance_transform_to_cpu(float* position,
                                                     float* orientation,
                                                     int srcIndex);

  virtual void write_single_instance_transform_to_cpu(const ::TINY::TinyVector3f& position,
                                                      const ::TINY::TinyQuaternionf& orientation,
                                                      int srcIndex);

  virtual void read_single_instance_transform_from_cpu(int srcIndex,
                                                       float* position,
                                                       float* orientation);

  virtual void write_single_instance_transform_to_gpu(float* position,
                                                      float* orientation,
                                                      int srcIndex);

  virtual void write_single_instance_color_to_cpu2(const ::TINY::TinyVector3f& rgb_color, float alpha, int srcIndex)
  {
      float rgba_color[4] = {rgb_color[0],rgb_color[1], rgb_color[2], alpha};
      write_single_instance_color_to_cpu(rgba_color, srcIndex);
  }

  virtual void write_single_instance_color_to_cpu(const float* color,
                                                  int srcIndex);
  virtual void write_single_instance_color_to_cpu(const double* color,
                                                  int srcIndex);
  virtual void write_single_instance_flags_to_cpu(int flags, int srcIndex2);

  virtual void write_single_instance_specular_color_to_cpu(
      const double* specular, int srcIndex2);
  virtual void write_single_instance_specular_color_to_cpu(
      const float* specular, int srcIndex2);

  virtual void write_single_instance_scale_to_cpu(const float* scale,
                                                  int srcIndex);
  virtual void write_single_instance_scale_to_cpu(const double* scale,
                                                  int srcIndex);

  virtual struct GLInstanceRendererInternalData* get_internal_data();

  virtual void draw_line(const ::TINY::TinyVector3f& from, const ::TINY::TinyVector3f& to,
                         const ::TINY::TinyVector3f& color, float lineWidth = 1);

  virtual void draw_lines(const ::TINY::TinyVector3f* positions,
                          const ::TINY::TinyVector3f& color, int numPoints,
                          int pointStrideInBytes, const unsigned int* indices,
                          int numIndices, float pointDrawSize);
  virtual void draw_points(const ::TINY::TinyVector3f* positions,
                           const ::TINY::TinyVector3f& color, int numPoints,
                           int pointStrideInBytes, float pointDrawSize);
  virtual void draw_point(const ::TINY::TinyVector3f& position,
                          const ::TINY::TinyVector3f& color, float pointSize = 1);
  virtual void draw_textured_triangle_mesh(
      float worldPosition[3], float worldOrientation[4], const float* vertices,
      int numvertices, const unsigned int* indices, int numIndices,
      float color[4], int textureIndex = -1, int vertexLayout = 0);

  virtual void update_camera(int upAxis = 1);

  virtual const TinyCamera* get_active_camera() const;
  virtual TinyCamera* get_active_camera();
  TinyCamera get_active_camera2() const
  {
    const TinyCamera* cam = get_active_camera();
    return *cam;
  }

  void set_camera(const TinyCamera& cam);

  virtual void set_active_camera(TinyCamera* cam);

  void get_projection_matrix(float projMatrix[16]) const;
  void set_projection_matrix(const float projMatrix[16]);
  void get_view_matrix(float viewMatrix[16]) const;
  void set_view_matrix(const float viewMatrix[16]);

  virtual void set_light_position(const float lightPos[3]);
  virtual void set_light_position(const double lightPos[3]);
  virtual void set_shadow_map_resolution(int shadowMapResolution);
  virtual void set_shadow_map_world_size(float worldSize);
  void set_light_specular_intensity(const float lightSpecularIntensity[3]);
  virtual void set_projective_texture_matrices(
      const float viewMatrix[16], const float projectionMatrix[16]);
  virtual void setProjectiveTexture(bool useProjectiveTexture);

  virtual void resize(int width, int height);
  virtual int get_screen_width() { return m_screenWidth; }
  virtual int get_screen_height() { return m_screenHeight; }

  virtual int get_max_shape_capacity() const;

  virtual int get_instance_capacity() const;

  virtual int get_total_num_instances() const;

  virtual void enable_shadow_map();

  virtual void set_plane_reflection_shape_index(int index);

  virtual void clear_z_buffer();

  virtual void set_render_frame_buffer(unsigned int renderFrameBuffer);

private:
  ::TINY::TinyVector3f get_camera_position() const;
  ::TINY::TinyVector3f get_camera_target() const;
  ::TINY::TinyVector3f get_camera_forward_vector() const;
};

#endif  // GL_INSTANCING_RENDERER_H
