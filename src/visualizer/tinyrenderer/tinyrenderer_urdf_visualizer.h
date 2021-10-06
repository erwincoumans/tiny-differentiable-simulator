/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TINYRENDERER_URDF_VISUALIZER_H
#define TINYRENDERER_URDF_VISUALIZER_H

#include <iostream>
#include <map>
#include <string>

#include "../meshcat/meshcat_cube_data.h"
#include "multi_body.hpp"
#include "urdf_structures.hpp"
#include "tiny_obj_loader.h"
#include "tinyrenderer.h"
#include "visualizer/opengl/tiny_vertex_format.h"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"
#include "stb_image/stb_image.h"
#define _USE_MATH_DEFINES
#include <math.h>

template <typename Algebra>
struct TinyRendererUrdfVisualizer {
    
  typedef ::tds::UrdfStructures<Algebra> TinyUrdfStructures;
  typedef ::tds::UrdfLink<Algebra> TinyUrdfLink;
  typedef ::tds::UrdfVisual<Algebra> UrdfVisual;
  
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;
  using Matrix3x3 = typename Algebra::Matrix3;
  typedef tds::Transform<Algebra> Transform;
  typedef ::tds::MultiBody<Algebra> TinyMultiBody;

  struct TinyVisualLinkInfo {
    std::string vis_name;
    int link_index;
    Vector3 origin_rpy;
    Vector3 origin_xyz;
    Vector3 inertia_xyz;
    Vector3 inertia_rpy;
  };

  //std::map<std::string, int> m_link_name_to_index;
  std::map<int, TinyVisualLinkInfo> m_b2vis;
  
  int m_uid;
  std::string m_texture_data;
  std::string m_texture_uuid;
  std::string m_path_prefix;

  TinySceneRenderer m_scene;
  std::vector<int> m_instances;
  std::map<std::string, int> m_name_to_instance;

  TinyRendererUrdfVisualizer()
      : m_uid(1234){
  }

  void delete_all() {
    
  }

  void load_texture(const std::string &texture_path) {
    m_texture_data = texture_data_broken_robot;

    FILE *fp = fopen((m_path_prefix + texture_path).c_str(), "rb");
    if (fp) {
      fseek(fp, 0, SEEK_END);
      unsigned int datasize = static_cast<unsigned int>(ftell(fp));
      fseek(fp, 0, SEEK_SET);
      unsigned char *data = static_cast<unsigned char *>(malloc(datasize));
      if (data) {
        int bytesRead;
        bytesRead = fread(data, 1, datasize, fp);

#if 0
        m_texture_data = std::string("data:image/png;base64,") +
                         base64_encode(data, datasize);
#endif
      }
      free(data);
      fclose(fp);
    }
  }

  void convert_link_visuals(TinyUrdfLink &link, int link_index,
                            bool useTextureUuid, std::vector<int>& visual_instance_uids) {
    for (int vis_index = 0; vis_index < (int)link.urdf_visual_shapes.size();
         vis_index++) {
      UrdfVisual &v =
          link.urdf_visual_shapes[vis_index];

      printf("v.geom_type=%d", v.geometry.geom_type);
      std::string vis_name =
          std::string("/meshcat/") + link.link_name + std::to_string(m_uid);
      TinyVisualLinkInfo b2v;
      b2v.vis_name = vis_name;
      b2v.link_index = link_index;
      b2v.origin_rpy = v.origin_rpy;
      b2v.origin_xyz = v.origin_xyz;
      b2v.inertia_xyz = link.urdf_inertial.origin_xyz;
      b2v.inertia_rpy = link.urdf_inertial.origin_rpy;
      int color_rgb = 0xffffff;
      double world_pos[3] = {0, 0, 0};
      const char* meshcat_path = vis_name.c_str();
      switch (v.geometry.geom_type)
      {
          case tds::TINY_SPHERE_TYPE:
          {
            create_sphere_instance(v.geometry.sphere.radius, world_pos, meshcat_path, color_rgb);
            break;
         }
         case tds::TINY_CAPSULE_TYPE: 
         {
            int up_axis = 2;//??
            create_capsule_instance(v.geometry.capsule.radius, v.geometry.capsule.length, up_axis, world_pos, meshcat_path, color_rgb);
            break;
         }
         case tds::TINY_BOX_TYPE: 
         {
            create_box_instance(v.geometry.box.extents[0],v.geometry.box.extents[1],v.geometry.box.extents[2],
                world_pos, meshcat_path, color_rgb);
            break;
          }

        case tds::TINY_MESH_TYPE: 
        {
            // printf("mesh filename=%s\n", v.geom_meshfilename.c_str());
    

            if (v.geometry.mesh.file_name.length()) {

                Vector3 scaling(1,1,1);
                double texture_scaling = 1.;

                std::vector<int> shape_ids;
                std::vector<int> colors;
                std::string mtl_string = "";
                //load_obj_shapes(obj_data, mtl_string, shape_ids, colors, scaling, useTextureUuid);

                tinyobj::attrib_t attrib;
                std::vector<tinyobj::shape_t> shapes;
                std::vector<tinyobj::material_t> materials;

                // test Wavefront obj loading
                std::string warn;
                std::string err;
                char basepath[1024];
                bool triangulate = true;

                
                //reader.ParseFromString(obj_string, mtl_string);
                std::string obj_filename = m_path_prefix+v.geometry.mesh.file_name;
                ::tds::FileUtils::extract_path(obj_filename.c_str(), basepath, 1024);
                bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                            obj_filename.c_str(), basepath, triangulate);
                for (int i = 0; i < shapes.size(); i++) {
                  std::vector<int> indices;
                  std::vector<GfxVertexFormat1> gfx_vertices;
                  int textureIndex = -1;
                  TinyMeshUtils::extract_shape(attrib, shapes[i], materials, indices,
                                               gfx_vertices, textureIndex);
                  
                  std::vector<double> vertices;
                  std::vector<double> normals;
                  std::vector<double> uvs;

                  //// apply scaling
                  for (int vv = 0; vv < gfx_vertices.size(); vv++) {
                    gfx_vertices[vv].x *= scaling[0];
                    gfx_vertices[vv].y *= scaling[1];
                    gfx_vertices[vv].z *= scaling[2];
                    vertices.push_back(gfx_vertices[vv].x * scaling[0]);
                    vertices.push_back(gfx_vertices[vv].y * scaling[1]);
                    vertices.push_back(gfx_vertices[vv].z * scaling[2]);

                    normals.push_back(gfx_vertices[vv].nx);
                    normals.push_back(gfx_vertices[vv].ny);
                    normals.push_back(gfx_vertices[vv].nz);
                    uvs.push_back(gfx_vertices[vv].u);
                    uvs.push_back(gfx_vertices[vv].v);
                  }

                  std::vector<unsigned char> texture = {  255,0,0,//red, 
                    0,255,0,//green
                    0,0,255,//, blue
                    255,255,255 //white
                    };
                  int texture_width = 2;
                  int texture_height = 2;
                  double texscaling= 1.;

                  
                  ::TINY::TinyVector3f color(1, 1, 1);
                  if (shapes[i].mesh.material_ids.size()) {
                    int mat_index = shapes[i].mesh.material_ids[0];
                    if (mat_index >= 0 && mat_index < materials.size()) {
                      const tinyobj::material_t &mat = materials[mat_index];
                      color.setValue(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
                      if (mat.diffuse_texname.length()) {
                        std::string texture_file_name =
                            std::string(basepath) + mat.diffuse_texname;
                        std::vector<unsigned char> buffer;
                        int width, height, n;
                        unsigned char *image =
                            stbi_load(texture_file_name.c_str(), &width, &height, &n, 3);
                        if (image)
                        {
                            texture.resize(width*height*3);
                            texture_width = width;
                            texture_height = height;
                            for (int w=0;w<texture_width;w++)
                            {
                                for (int h=0;h<texture_height;h++)
                                {
                                    int index = 3*(w+h*texture_width)+0;
                                    texture[index+0] = image[index+0];
                                    texture[index+1] = image[index+1];
                                    texture[index+2] = image[index+2];
                                }
                            }
                        }
                        free(image);
                      }
                    }
                  }
               
                  create_mesh_instance(vertices, normals, uvs, indices, texture, texture_width, texture_height, texture_scaling,
                      world_pos, meshcat_path, color_rgb);
                }
            }
            break;
          }
          default:
          {
              printf("Unknown geometry type: %d\n", v.geometry.geom_type);
          }
      };

      visual_instance_uids.push_back(m_uid);
      m_b2vis[m_uid++] = b2v;
    }
  }

  void convert_visuals(TinyUrdfStructures &urdf,
                       const std::string &texture_path, TinyMultiBody* body) {
    load_texture(texture_path);

    //m_link_name_to_index.clear();
    {
      int link_index = -1;
      std::string link_name = urdf.base_links[0].link_name;
      //m_link_name_to_index[link_name] = link_index;
      std::vector<int> indices;
      convert_link_visuals(urdf.base_links[0], link_index, false, body? body->visual_instance_uids() : indices);
      
    }

    for (int link_index = 0; link_index < (int)urdf.links.size();
         link_index++) {
      std::string link_name = urdf.links[link_index].link_name;
      //m_link_name_to_index[link_name] = link_index;
      std::vector<int> indices;
      convert_link_visuals(urdf.links[link_index], link_index, false, body? body->links_[link_index].visual_instance_uids : indices);
    }
  }

  void sync_visual_transforms(const TinyMultiBody *body) {
    // sync base transform
    for (int v = 0; v < body->visual_instance_uids().size(); v++) {
      int visual_id = body->visual_instance_uids()[v];
      if (m_b2vis.find(visual_id) != m_b2vis.end()) {
        Quaternion rot;
        Transform geom_X_world =
            body->base_X_world() * body->X_visuals()[v];

        const Matrix3x3 &m =
            geom_X_world.rotation;

        const TinyVisualLinkInfo &viz = m_b2vis.at(visual_id);
        // printf("vis_name=%s\n", viz.vis_name.c_str());
        double world_pos[3] = {geom_X_world.translation.getX(),
                               geom_X_world.translation.getY(),
                               geom_X_world.translation.getZ()};
        double world_mat[9] = {m.getRow(0)[0], m.getRow(1)[0], m.getRow(2)[0],
                               m.getRow(0)[1], m.getRow(1)[1], m.getRow(2)[1],
                               m.getRow(0)[2], m.getRow(1)[2], m.getRow(2)[2]};

          std::vector<float> pos = {float(geom_X_world.translation.getX()),
                                 float(geom_X_world.translation.getY()),
                                 float(geom_X_world.translation.getZ())};

          auto orn_d = Algebra::matrix_to_quat(m);
          std::vector<float> orn = {float(orn_d.x()),
                                 float(orn_d.y()),
                                 float(orn_d.z()),
                                 float(orn_d.w())};
          int instance_uid = m_name_to_instance[viz.vis_name];
          m_scene.set_object_position(instance_uid, pos);
          m_scene.set_object_orientation(instance_uid, orn);
      }
    }

    for (int l = 0; l < body->links().size(); l++) {
      for (int v = 0; v < body->links()[l].visual_instance_uids.size(); v++) {
        int visual_id = body->links()[l].visual_instance_uids[v];
        if (m_b2vis.find(visual_id) != m_b2vis.end()) {
          Quaternion rot;
          Transform geom_X_world =
              body->links()[l].X_world * body->links()[l].X_visuals[v];
          
          const Matrix3x3 &m =
              geom_X_world.rotation;
          const TinyVisualLinkInfo &viz = m_b2vis.at(visual_id);
          // printf("vis_name=%s\n", viz.vis_name.c_str());
          double world_mat[9] = {
              m.getRow(0)[0], m.getRow(1)[0], m.getRow(2)[0],
              m.getRow(0)[1], m.getRow(1)[1], m.getRow(2)[1],
              m.getRow(0)[2], m.getRow(1)[2], m.getRow(2)[2]};
          std::vector<float> pos = {float(geom_X_world.translation.getX()),
                                 float(geom_X_world.translation.getY()),
                                 float(geom_X_world.translation.getZ())};

          auto orn_d = Algebra::matrix_to_quat(m);
          std::vector<float> orn = {float(orn_d.x()),
                                 float(orn_d.y()),
                                 float(orn_d.z()),
                                 float(orn_d.w())};

          
          int instance_uid = m_name_to_instance[viz.vis_name];
          m_scene.set_object_position(instance_uid, pos);
          m_scene.set_object_orientation(instance_uid, orn);
        }
      }
    }

    get_camera_image();
  }

  void get_camera_image()
  {
    double NEAR_PLANE = 0.01;
    double FAR_PLANE = 1000;
    double HFOV = 58.0;
    double VFOV = 45.0;
    double left=-tan(M_PI*HFOV/360.0)*NEAR_PLANE;
    double right=tan(M_PI*HFOV/360.0)*NEAR_PLANE;
    double bottom=-tan(M_PI*VFOV/360.0)*NEAR_PLANE;
    double top=tan(M_PI*VFOV/360.0)*NEAR_PLANE;
    auto projection_matrix = TinySceneRenderer::compute_projection_matrix( left, right, bottom, top, NEAR_PLANE, FAR_PLANE);


    std::vector<float> up = {0.,0.,1.};
    std::vector<float> eye = {2.,4.,1.};
    std::vector<float> target = {0.,0.,0.};
    auto view_matrix = TinySceneRenderer::compute_view_matrix(eye, target, up);

    RenderBuffers buffers(640,480);//256,256);
    m_scene.get_camera_image(m_instances, view_matrix, projection_matrix, buffers);
    TGAImage img(buffers.m_width,buffers.m_height,3);
    for (int i=0;i<buffers.m_width;i++)
    {
        for (int j=0;j<buffers.m_height;j++)
        {
            unsigned char red = buffers.rgb[3*(i+buffers.m_width*j)+0];
            unsigned char green = buffers.rgb[3*(i+buffers.m_width*j)+1];
            unsigned char blue = buffers.rgb[3*(i+buffers.m_width*j)+2];
            TGAColor rgb(red,green,blue);
            img.set(i,j,rgb);
        }
    }
    char fileName[1024];
    static int frame=0;
    sprintf(fileName, "img%d.tga", frame++);
    printf("writing frame %s\n", fileName);
    img.write_tga_file(fileName);
    
  }

  void create_box_instance(double half_extents_x, double half_extents_y, double half_extents_z, 
      double world_pos[3], const std::string& meshcat_path, int color_rgb = 0xffffff )
  {
      if (meshcat_path.length())
      {
          std::vector<unsigned char> texture = {  255,0,0,//red, 
            0,255,0,//green
            0,0,255,//, blue
            255,255,255 //white
          };
          int texwidth = 2;
          int texheight = 2;
          std::vector<double> half_extents = {half_extents_x, half_extents_y, half_extents_z};
          double texscaling= 1.;
          int capsulex_model = m_scene.create_cube(half_extents, texture, texwidth, texheight, texscaling);
          int sphere_instance = m_scene.create_object_instance(capsulex_model);
          float red = ((color_rgb>>16)&0xff)/255.;
          float green = ((color_rgb>>8)&0xff)/255.;
          float blue = (color_rgb&0xff)/255.;
          std::vector<float> color = {red, green, blue};
          m_scene.set_object_color(sphere_instance,color);
          m_name_to_instance[meshcat_path]=sphere_instance;
          m_instances.push_back(sphere_instance);
      }
  }

  void create_capsule_instance(double radius, double half_height, int up_axis, double world_pos[3], const std::string& meshcat_path, int color_rgb = 0xffffff )
  {
      if (meshcat_path.length())
      {
          std::vector<unsigned char> texture = {  255,0,0,//red, 
            0,255,0,//green
            0,0,255,//, blue
            255,255,255 //white
          };
          int texwidth = 2;
          int texheight = 2;
          int capsule_model = m_scene.create_capsule(radius,half_height,up_axis,texture, texwidth, texheight);
          int capsule_instance = m_scene.create_object_instance(capsule_model);
          float red = ((color_rgb>>16)&0xff)/255.;
          float green = ((color_rgb>>8)&0xff)/255.;
          float blue = (color_rgb&0xff)/255.;
          std::vector<float> color = {red, green, blue};
          m_scene.set_object_color(capsule_instance,color);
          m_name_to_instance[meshcat_path]=capsule_instance;
          m_instances.push_back(capsule_instance);
      }
  }
  

  void create_sphere_instance(double radius, double world_pos[3], const std::string& meshcat_path, int color_rgb = 0xffffff )
  {
      if (meshcat_path.length())
      {
          std::vector<unsigned char> texture = {  255,0,0,//red, 
            0,255,0,//green
            0,0,255,//, blue
            255,255,255 //white
          };
          int texwidth = 2;
          int texheight = 2;
          int sphere_model = m_scene.create_capsule(radius,radius,0,texture, texwidth, texheight);
          int sphere_instance = m_scene.create_object_instance(sphere_model);
          float red = ((color_rgb>>16)&0xff)/255.;
          float green = ((color_rgb>>8)&0xff)/255.;
          float blue = (color_rgb&0xff)/255.;
          std::vector<float> color = {red, green, blue};
          m_scene.set_object_color(sphere_instance,color);
          m_name_to_instance[meshcat_path]=sphere_instance;
          m_instances.push_back(sphere_instance);
      }
  }

  void create_mesh_instance(const std::vector<double>& vertices,
                                   const std::vector<double>& normals,
                                   const std::vector<double>& uvs,
                                   const std::vector<int>& indices,
                                   const std::vector<unsigned char>& texture,
                                   int texture_width, int texture_height,
                                   float texture_scaling,
                                   double world_pos[3], const std::string& meshcat_path, int color_rgb = 0xffffff )
  {
      if (meshcat_path.length())
      {
          int mesh_model = m_scene.create_mesh(vertices, normals, uvs, indices, texture, texture_width, texture_height, texture_scaling);
          int mesh_instance = m_scene.create_object_instance(mesh_model);
          float red = ((color_rgb>>16)&0xff)/255.;
          float green = ((color_rgb>>8)&0xff)/255.;
          float blue = (color_rgb&0xff)/255.;
          std::vector<float> color = {red, green, blue, 1.};
          m_scene.set_object_color(mesh_instance,color);
          std::vector<float> position={float(world_pos[0]),float(world_pos[1]),float(world_pos[2])};
          m_scene.set_object_position(mesh_instance, position);
          m_name_to_instance[meshcat_path]=mesh_instance;
          m_instances.push_back(mesh_instance);
      }
  }

};

#endif  // TINYRENDERER_URDF_VISUALIZER_H
