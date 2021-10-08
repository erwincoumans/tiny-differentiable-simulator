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

#ifndef OPENGL_URDF_VISUALIZER_H
#define OPENGL_URDF_VISUALIZER_H

#include <iostream>
#include <map>
#include <string>

#include "geometry.hpp"
#include "multi_body.hpp"
#include "stb_image/stb_image.h"
#include "tiny_obj_loader.h"
#include "urdf_structures.hpp"
#include "utils/file_utils.hpp"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"

// disabled #define USE_SDF_TO_MESH, since code crashes when radius == 0
// #define USE_SDF_TO_MESH
#ifdef USE_SDF_TO_MESH
#include "utils/sdf_to_mesh_converter.hpp"
#endif

template <typename Algebra>
struct OpenGLUrdfVisualizer {
  typedef ::tds::UrdfStructures<Algebra> TinyUrdfStructures;
  typedef ::tds::UrdfLink<Algebra> TinyUrdfLink;
  typedef ::tds::UrdfVisual<Algebra> UrdfVisual;

  typedef ::tds::MultiBody<Algebra> TinyMultiBody;
  typedef tds::Transform<Algebra> Transform;
  using TinyVector3 = typename Algebra::Vector3;
  using TinyMatrix3 = typename Algebra::Matrix3;
  using Quaternion = typename Algebra::Quaternion;

  struct TinyVisualLinkInfo {
    std::string vis_name;
    int link_index;
    TinyVector3 origin_rpy;
    TinyVector3 origin_xyz;
    TinyVector3 inertia_xyz;
    TinyVector3 inertia_rpy;
    std::vector<int> visual_shape_uids;
    std::vector<::TINY::TinyVector3f> shape_colors;
  };

  std::map<std::string, int> m_link_name_to_index;
  std::map<int, TinyVisualLinkInfo> m_b2vis;

  int m_uid;
  std::string m_texture_data;
  std::string m_texture_uuid;
  std::string m_path_prefix;

  TinyOpenGL3App m_opengl_app;

  OpenGLUrdfVisualizer(int width = 1024, int height = 768,
                       const char *title = "Tiny Differentiable Simulator")
      : m_uid(1234), m_opengl_app(title, width, height) {
    m_opengl_app.m_renderer->init();
    m_opengl_app.set_up_axis(2);
    m_opengl_app.m_renderer->get_active_camera()->set_camera_distance(4);
    m_opengl_app.m_renderer->get_active_camera()->set_camera_pitch(-30);
    m_opengl_app.m_renderer->get_active_camera()->set_camera_target_position(
        0, 0, 0);
  }

  void delete_all() {
    // todo
  }

  void load_obj_shapes(const std::string &obj_filename,
                       std::vector<int> &shape_ids,
                       std::vector<::TINY::TinyVector3f> &colors,
                       const ::TINY::TinyVector3f &scaling) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    // test Wavefront obj loading
    std::string warn;
    std::string err;
    char basepath[1024];
    bool triangulate = true;

    ::tds::FileUtils::extract_path(obj_filename.c_str(), basepath, 1024);
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                obj_filename.c_str(), basepath, triangulate);

    for (int i = 0; i < shapes.size(); i++) {
      std::vector<int> indices;
      std::vector<GfxVertexFormat1> vertices;
      int textureIndex = -1;
      TinyMeshUtils::extract_shape(attrib, shapes[i], materials, indices,
                                   vertices, textureIndex);
      // apply scaling
      for (int vv = 0; vv < vertices.size(); vv++) {
        vertices[vv].x *= scaling[0];
        vertices[vv].y *= scaling[1];
        vertices[vv].z *= scaling[2];
      }

      textureIndex = -1;
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

            textureIndex =
                m_opengl_app.m_renderer->register_texture(image, width, height);
            free(image);
          }
        }
      }
      int shape = m_opengl_app.m_renderer->register_shape(
          &vertices[0].x, vertices.size(), &indices[0], indices.size(),
          B3_GL_TRIANGLES, textureIndex);
      shape_ids.push_back(shape);
      colors.push_back(color);
    }
  }

  void load_obj(const std::string &obj_filename,
                const ::TINY::TinyVector3f &pos,
                const ::TINY::TinyQuaternionf &orn,
                const ::TINY::TinyVector3f &scaling,
                std::vector<int> &instance_ids) {
    std::vector<int> shape_ids;
    std::vector<::TINY::TinyVector3f> colors;
    ::TINY::TinyVector3f unit_scaling(1, 1, 1);
    load_obj_shapes(obj_filename, shape_ids, colors, unit_scaling);

    for (int i = 0; i < shape_ids.size(); i++) {
      int shape = shape_ids[i];
      int instance_id = m_opengl_app.m_renderer->register_graphics_instance(
          shape, pos, orn, colors[i], scaling);
      instance_ids.push_back(instance_id);
    }

    m_opengl_app.m_renderer->write_transforms();
  }

  void convert_link_visuals(TinyUrdfStructures &urdf, TinyUrdfLink &link,
                            int link_index, bool useTextureUuid) {
    if (link.urdf_visual_shapes.empty()) {
      // convert collision shapes into visual shapes
      for (int col_index = 0;
           col_index < (int)link.urdf_collision_shapes.size(); col_index++) {
        const tds::UrdfCollision<Algebra> &c = link.urdf_collision_shapes[col_index];
        UrdfVisual v;
        v.geometry = c.geometry;
        v.has_local_material = false;
        v.origin_rpy = c.origin_rpy;
        v.origin_xyz = c.origin_xyz;
        v.visual_name = c.collision_name;
        v.visual_shape_uid = m_uid++;
        link.urdf_visual_shapes.push_back(v);
      }
    }

    for (int vis_index = 0; vis_index < (int)link.urdf_visual_shapes.size();
         vis_index++) {
      UrdfVisual &v = link.urdf_visual_shapes[vis_index];

      printf("v.geom_type=%d", v.geometry.geom_type);
      std::string vis_name =
          std::string("/opengl/") + link.link_name + std::to_string(m_uid);
      TinyVisualLinkInfo b2v;
      b2v.vis_name = vis_name;
      b2v.link_index = link_index;
      b2v.origin_rpy = v.origin_rpy;
      b2v.origin_xyz = v.origin_xyz;
      b2v.inertia_xyz = link.urdf_inertial.origin_xyz;
      b2v.inertia_rpy = link.urdf_inertial.origin_rpy;
      int color_rgb = 0xffffff;
      // todo(erwincoumans make local material colors work
      if (v.has_local_material) {
        // color_rgb = 0xff*v.material.material_rgb[0]+
        //    0xff00*v.material.material_rgb[1]+
        //    0xff0000*v.material.material_rgb[2];
      }
      double world_pos[3] = {0, 0, 0};
      switch (v.geometry.geom_type) {
        case ::tds::TINY_MESH_TYPE: {
          // printf("mesh filename=%s\n", v.geom_meshfilename.c_str());
          std::string obj_filename;
          std::string org_obj_filename =
              m_path_prefix + v.geometry.mesh.file_name;
          if (::tds::FileUtils::find_file(org_obj_filename, obj_filename)) {
            ::TINY::TinyVector3f pos(0, 0, 0);
            ::TINY::TinyVector3f scaling(
                Algebra::to_double(v.geometry.mesh.scale[0]),
                Algebra::to_double(v.geometry.mesh.scale[1]),
                Algebra::to_double(v.geometry.mesh.scale[2]));
            ::TINY::TinyQuaternionf orn(0, 0, 0, 1);
            load_obj_shapes(obj_filename, b2v.visual_shape_uids,
                            b2v.shape_colors, scaling);
          }
          break;
        }
        case ::tds::TINY_SPHERE_TYPE: {
          int textureIndex = -1;

          // Experimental texture binding
          if (v.material_name.size() > 0) {
            ::tds::VisualMaterial<Algebra> material =
                urdf.materials[v.material_name];
            std::string texture_file = material.texture_filename;
            if (!texture_file.empty()) {
              std::string texture_path;
              ::tds::FileUtils::find_file(texture_file, texture_path);
              std::vector<unsigned char> buffer;
              int width, height, n;
              unsigned char *image =
                  stbi_load(texture_path.c_str(), &width, &height, &n, 3);

              textureIndex = m_opengl_app.m_renderer->register_texture(
                  image, width, height);
              free(image);
            }
          }
#ifdef USE_SDF_TO_MESH
          // Yizhou: Instead of hardcoding the mesh, use the sdf_to_mesh
          // generator to generate the mesh
          ::tds::Sphere<Algebra> gen_sphere(v.geometry.sphere.radius);
          ::tds::RenderShape gen_mesh =
              ::tds::convert_sdf_to_mesh(gen_sphere, 50);

          int shape_id = m_opengl_app.m_instancingRenderer->register_shape(
              &gen_mesh.vertices[0].x, gen_mesh.vertices.size(),
              &gen_mesh.indices[0], gen_mesh.num_triangles * 3, B3_GL_TRIANGLES,
              textureIndex);

#else
          // int shape_id =
          // m_opengl_app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
          int up_axis = 2;
          int shape_id = m_opengl_app.register_graphics_capsule_shape(
              Algebra::to_double(v.geometry.sphere.radius), 0., up_axis,
              textureIndex);

#endif

          b2v.visual_shape_uids.push_back(shape_id);
          ::TINY::TinyVector3f color(1, 1, 1);
          b2v.shape_colors.push_back(color);
          break;
        }
        case ::tds::TINY_CAPSULE_TYPE: {
#ifdef USE_SDF_TO_MESH
          ::tds::Capsule<Algebra> gen_capsule(v.geometry.capsule.radius,
                                              v.geometry.capsule.length);
          ::tds::RenderShape gen_mesh =
              ::tds::convert_sdf_to_mesh(gen_capsule, 100);
          int shape_id = m_opengl_app.m_instancingRenderer->register_shape(
              &gen_mesh.vertices[0].x, gen_mesh.vertices.size(),
              &gen_mesh.indices[0], gen_mesh.num_triangles * 3, B3_GL_TRIANGLES,
              -1);

#else
          float radius = Algebra::to_double(v.geometry.capsule.radius);
          float half_height =
              Algebra::to_double(v.geometry.capsule.length) * 0.5;
          int up_axis = 2;
          int shape_id = m_opengl_app.register_graphics_capsule_shape(
              radius, half_height, up_axis, -1);

#endif

          b2v.visual_shape_uids.push_back(shape_id);
          ::TINY::TinyVector3f color(1, 1, 1);
          b2v.shape_colors.push_back(color);
          break;
        }
#ifdef USE_SDF_TO_MESH
        case ::tds::TINY_CYLINDER_TYPE: {
          ::tds::Cylinder<Algebra> gen_cylinder(v.geometry.cylinder.radius,
                                                v.geometry.cylinder.length);
          ::tds::RenderShape gen_mesh =
              ::tds::convert_sdf_to_mesh(gen_cylinder, 100);
          int shape_id = m_opengl_app.m_instancingRenderer->register_shape(
              &gen_mesh.vertices[0].x, gen_mesh.vertices.size(),
              &gen_mesh.indices[0], gen_mesh.num_triangles * 3, B3_GL_TRIANGLES,
              -1);
          b2v.visual_shape_uids.push_back(shape_id);
          ::TINY::TinyVector3f color(1, 1, 1);
          b2v.shape_colors.push_back(color);
          break;
        }
#endif
        case ::tds::TINY_BOX_TYPE: {
          float half_extentsx =
              0.5 * Algebra::to_double(v.geometry.box.extents[0]);
          float half_extents_y =
              0.5 * Algebra::to_double(v.geometry.box.extents[1]);
          float half_extents_z =
              0.5 * Algebra::to_double(v.geometry.box.extents[2]);
          int shape_id = m_opengl_app.register_cube_shape(
              half_extentsx, half_extents_y, half_extents_z);
          b2v.visual_shape_uids.push_back(shape_id);
          ::TINY::TinyVector3f color(1, 1, 1);
          b2v.shape_colors.push_back(color);

          break;
        }
        default: {
          std::cout << "Unsupported shape type" << std::endl;
        }
      }

      v.visual_shape_uid = m_uid;
      m_b2vis[m_uid++] = b2v;
    }
  }

#if 0
  void create_visual_instances(TinyMultiBody& body) {
	  for (int i = 0; i < b2v.visual_shape_uids.size(); i++) {
		  int shape = b2v.visual_shape_uids[i];
		  int instance_id = m_opengl_app.m_renderer->register_graphics_instance(shape, pos, orn, b2v.shape_colors[i], scaling);
		  if (link_index == -1) {
			  body.visual_instance_uids().push_back(instance_id);
		  }
		  else {
			  body.links_[link_index].visual_instance_uids.push_back(instance_id);
		  }
	  }
  }
#endif

  void convert_visuals(TinyUrdfStructures &urdf,
                       const std::string &texture_path) {
    m_link_name_to_index.clear();
    {
      int link_index = -1;
      std::string link_name = urdf.base_links[0].link_name;
      m_link_name_to_index[link_name] = link_index;
      convert_link_visuals(urdf, urdf.base_links[0], link_index, false);
    }

    for (int link_index = 0; link_index < (int)urdf.links.size();
         link_index++) {
      std::string link_name = urdf.links[link_index].link_name;
      m_link_name_to_index[link_name] = link_index;
      convert_link_visuals(urdf, urdf.links[link_index], link_index, false);
    }
  }

  void sync_visual_transforms(const TinyMultiBody *body) {
    // sync base transform
    for (int v = 0; v < body->visual_instance_uids().size(); v++) {
      int visual_instance_id = body->visual_instance_uids()[v];
      Transform geom_X_world = body->base_X_world() * body->X_visuals()[v];

      Quaternion rot = Algebra::matrix_to_quat(geom_X_world.rotation);
      ::TINY::TinyVector3f pos(geom_X_world.translation[0],
                               geom_X_world.translation[1],
                               geom_X_world.translation[2]);
      ::TINY::TinyQuaternionf orn(Algebra::quat_x(rot), Algebra::quat_y(rot),
                                  Algebra::quat_z(rot), Algebra::quat_w(rot));
      m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(
          pos, orn, visual_instance_id);
    }

    for (int l = 0; l < body->links().size(); l++) {
      for (int v = 0; v < body->links()[l].visual_instance_uids.size(); v++) {
        int visual_instance_id = body->links()[l].visual_instance_uids[v];
        if (visual_instance_id >= 0) {
          Transform geom_X_world =
              body->links()[l].X_world * body->links()[l].X_visuals[v];

          Quaternion rot = Algebra::matrix_to_quat(geom_X_world.rotation);
          ::TINY::TinyVector3f pos(geom_X_world.translation[0],
                                   geom_X_world.translation[1],
                                   geom_X_world.translation[2]);
          ::TINY::TinyQuaternionf orn(
              Algebra::quat_x(rot), Algebra::quat_y(rot), Algebra::quat_z(rot),
              Algebra::quat_w(rot));
          m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(
              pos, orn, visual_instance_id);
        }
      }
    }
  }
  void render() {
    int upAxis = 2;
    m_opengl_app.m_renderer->write_transforms();
    m_opengl_app.m_renderer->update_camera(upAxis);
    double lightPos[3] = {-50, 30, 40};
    m_opengl_app.m_renderer->set_light_position(lightPos);
    float specular[3] = {1, 1, 1};
    m_opengl_app.m_renderer->set_light_specular_intensity(specular);
    DrawGridData data;
    data.upAxis = 2;
    data.drawAxis = true;
    m_opengl_app.draw_grid(data);
    // const char* bla = "3d label";
    // m_opengl_app.draw_text_3d(bla, 0, 0, 1, 1);
    m_opengl_app.m_renderer->render_scene();
    m_opengl_app.swap_buffer();
  }
};

#endif  // OPENGL_URDF_VISUALIZER_H
