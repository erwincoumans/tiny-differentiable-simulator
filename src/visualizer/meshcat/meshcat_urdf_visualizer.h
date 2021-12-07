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

#ifndef MESHCAT_URDF_VISUALIZER_H
#define MESHCAT_URDF_VISUALIZER_H

#include <cpp_base64/base64.h>

#include <iostream>
#include <map>
#include <string>

#include "meshcat_cube_data.h"
#include "meshcat_zmq.h"
#include "multi_body.hpp"
#include "urdf_structures.hpp"

inline std::string correct_non_utf_8(const std::string &str) {
  int i, f_size = str.size();
  unsigned char c, c2, c3, c4;
  std::string to;
  to.reserve(f_size);

  for (i = 0; i < f_size; i++) {
    c = (unsigned char)(str)[i];
    if (c < 32) {                          // control char
      if (c == 9 || c == 10 || c == 13) {  // allow only \t \n \r
        to.append(1, c);
      }
      continue;
    } else if (c < 127) {  // normal ASCII
      to.append(1, c);
      continue;
    } else if (c < 160) {  // control char (nothing should be defined here
                           // either ASCI, ISO_8859-1 or UTF8, so skipping)
      if (c2 == 128) {     // fix microsoft mess, add euro
        to.append(1, 226);
        to.append(1, 130);
        to.append(1, 172);
      }
      if (c2 == 133) {  // fix IBM mess, add NEL = \n\r
        to.append(1, 10);
        to.append(1, 13);
      }
      continue;
    } else if (c < 192) {  // invalid for UTF8, converting ASCII
      to.append(1, (unsigned char)194);
      to.append(1, c);
      continue;
    } else if (c < 194) {  // invalid for UTF8, converting ASCII
      to.append(1, (unsigned char)195);
      to.append(1, c - 64);
      continue;
    } else if (c < 224 && i + 1 < f_size) {  // possibly 2byte UTF8
      c2 = (unsigned char)(str)[i + 1];
      if (c2 > 127 && c2 < 192) {    // valid 2byte UTF8
        if (c == 194 && c2 < 160) {  // control char, skipping
          ;
        } else {
          to.append(1, c);
          to.append(1, c2);
        }
        i++;
        continue;
      }
    } else if (c < 240 && i + 2 < f_size) {  // possibly 3byte UTF8
      c2 = (unsigned char)(str)[i + 1];
      c3 = (unsigned char)(str)[i + 2];
      if (c2 > 127 && c2 < 192 && c3 > 127 && c3 < 192) {  // valid 3byte UTF8
        to.append(1, c);
        to.append(1, c2);
        to.append(1, c3);
        i += 2;
        continue;
      }
    } else if (c < 245 && i + 3 < f_size) {  // possibly 4byte UTF8
      c2 = (unsigned char)(str)[i + 1];
      c3 = (unsigned char)(str)[i + 2];
      c4 = (unsigned char)(str)[i + 3];
      if (c2 > 127 && c2 < 192 && c3 > 127 && c3 < 192 && c4 > 127 &&
          c4 < 192) {  // valid 4byte UTF8
        to.append(1, c);
        to.append(1, c2);
        to.append(1, c3);
        to.append(1, c4);
        i += 3;
        continue;
      }
    }
    // invalid UTF8, converting ASCII (c>245 || string too short for
    // multi-byte))
    to.append(1, (unsigned char)195);
    to.append(1, c - 64);
  }
  return to;
}

template <typename Algebra>
struct MeshcatUrdfVisualizer {
    
  typedef ::tds::UrdfStructures<Algebra> TinyUrdfStructures;
  typedef ::tds::UrdfLink<Algebra> TinyUrdfLink;
  typedef ::tds::UrdfVisual<Algebra> UrdfVisual;
  
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;
  using Matrix3x3 = typename Algebra::Matrix3;
  typedef tds::Transform<Algebra> Transform;
  typedef ::tds::MultiBody<Algebra> TinyMultiBody;

  enum BASE_TRANSFORM_ORDER {
      QUAT_XYZW_POSXYZ = 1,
      POSXYZ_QUAT_XYZW,
      };

  struct TinyVisualLinkInfo {
    std::string vis_name;
    int link_index;
    Vector3 origin_rpy;
    Vector3 origin_xyz;
    Vector3 inertia_xyz;
    Vector3 inertia_rpy;
  };

  std::map<std::string, int> m_link_name_to_index;
  std::map<int, TinyVisualLinkInfo> m_b2vis;
  zmq::context_t m_context;
  zmq::socket_t m_sock;
  int m_uid;
  std::string m_texture_data;
  std::string m_texture_uuid;
  std::string m_path_prefix;

  MeshcatUrdfVisualizer(unsigned int port = 6000)
      : m_uid(1234), m_sock(zmq::socket_t(m_context, ZMQ_REQ)) {
    m_sock.connect("tcp://127.0.0.1:" + std::to_string(port));
  }

  void delete_all() {
    nlohmann::json del_cmd = create_delete_cmd();
    send_zmq(m_sock, del_cmd);
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
        m_texture_data = std::string("data:image/png;base64,") +
                         base64_encode(data, datasize);
      }
      free(data);
      fclose(fp);
    }
  }

  void create_sphere_instance(double radius, double world_pos[3], const std::string& meshcat_path, int color_rgb = 0xffffff )
  {
      if (meshcat_path.length())
      {
          nlohmann::json sphere_cmd =
                      create_sphere_cmd(radius, world_pos, color_rgb, meshcat_path.c_str());
          send_zmq(m_sock, sphere_cmd);
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
              if (v.geometry.sphere.radius>0)
              {
                nlohmann::json sphere_cmd =
                  create_sphere_cmd(v.geometry.sphere.radius, world_pos, color_rgb, meshcat_path);
                send_zmq(m_sock, sphere_cmd);
              }
            break;
         }
         case tds::TINY_CAPSULE_TYPE: 
         {
            nlohmann::json box_cmd = create_box_cmd(v.geometry.capsule.radius, 
                v.geometry.capsule.radius, v.geometry.capsule.length, world_pos, color_rgb, meshcat_path);
            send_zmq(m_sock, box_cmd);
            break;
         }
         case tds::TINY_BOX_TYPE: 
         {
           
            nlohmann::json box_cmd = create_box_cmd(v.geometry.box.extents[0], 
                v.geometry.box.extents[1], v.geometry.box.extents[2], world_pos, color_rgb, meshcat_path);
            send_zmq(m_sock, box_cmd);
            break;
          }

        case tds::TINY_MESH_TYPE: 
        {
            // printf("mesh filename=%s\n", v.geom_meshfilename.c_str());
            std::string obj_data;

            FILE *fp =
                fopen((m_path_prefix + v.geometry.mesh.file_name).c_str(), "r");
            if (fp) {
              fseek(fp, 0, SEEK_END);
              int datasize = (int)ftell(fp);
              fseek(fp, 0, SEEK_SET);
              char *data = static_cast<char *>(malloc(datasize + 1));
              if (data) {
                int bytesRead;
                bytesRead = fread(data, 1, datasize, fp);
                data[datasize] = 0;
                obj_data = std::string(data);
              }
              free(data);
              fclose(fp);
            }
            int str_len = obj_data.length();

            if (str_len) {
              std::string obj_data_utf8 = correct_non_utf_8(obj_data);
              if (useTextureUuid) {
                nlohmann::json cmd = create_textured_mesh_cmd2(
                    obj_data_utf8.c_str(), m_texture_uuid, world_pos, color_rgb,
                    meshcat_path);
                send_zmq(m_sock, cmd);
              } else {
                nlohmann::json cmd = create_textured_mesh_cmd(
                    obj_data_utf8.c_str(), m_texture_data.c_str(), world_pos,
                    color_rgb, meshcat_path);

                nlohmann::json ob = cmd["object"];
                nlohmann::json texs = ob["textures"];
                nlohmann::json tex = texs[0];
                nlohmann::json uuid = tex["uuid"];
                m_texture_uuid = uuid;
                send_zmq(m_sock, cmd);
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

    m_link_name_to_index.clear();
    {
      int link_index = -1;
      std::string link_name = urdf.base_links[0].link_name;
      m_link_name_to_index[link_name] = link_index;
      std::vector<int> indices;
      convert_link_visuals(urdf.base_links[0], link_index, false, body? body->visual_instance_uids() : indices);
      
    }

    for (int link_index = 0; link_index < (int)urdf.links.size();
         link_index++) {
      std::string link_name = urdf.links[link_index].link_name;
      m_link_name_to_index[link_name] = link_index;
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
        nlohmann::json tr_cmd =
            create_transform_cmd(world_pos, world_mat, viz.vis_name.c_str());
        send_zmq(m_sock, tr_cmd);
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
          double world_pos[3] = {geom_X_world.translation.getX(),
                                 geom_X_world.translation.getY(),
                                 geom_X_world.translation.getZ()};
          nlohmann::json tr_cmd =
              create_transform_cmd(world_pos, world_mat, viz.vis_name.c_str());
          send_zmq(m_sock, tr_cmd);
        }
      }
    }
  }

    void sync_visual_transforms2(const TinyMultiBody* body, const std::vector<Scalar>& params, int link_visual_transforms_start_index, BASE_TRANSFORM_ORDER base_xform_layout = QUAT_XYZW_POSXYZ) {
    // sync base transform
    int index = link_visual_transforms_start_index;
    for (int v = 0; v < body->visual_instance_uids().size(); v++) {
      int visual_id = body->visual_instance_uids()[v];
      if (m_b2vis.find(visual_id) != m_b2vis.end()) {
        Vector3 pos(0,0,0);
        Quaternion rot(0,0,0,1);
        switch (base_xform_layout) { 
            case POSXYZ_QUAT_XYZW:
                pos = Vector3(params[0],params[1],params[2]);
                rot = Quaternion (params[3],params[4],params[5],params[6]);
                break;
            case QUAT_XYZW_POSXYZ:
                pos = Vector3(params[4],params[5],params[6]);
                rot = Quaternion (params[0],params[1],params[2],params[3]);
                break;
            default: {
                }
         }
        Transform base_world;
        base_world.translation = pos;
        base_world.rotation = Algebra::quat_to_matrix(rot);

        Transform geom_X_world =
            base_world * body->X_visuals()[v];

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
        nlohmann::json tr_cmd =
            create_transform_cmd(world_pos, world_mat, viz.vis_name.c_str());
        send_zmq(m_sock, tr_cmd);
      }
    }

    for (int l = 0; l < body->links().size(); l++) {
      for (int v = 0; v < body->links()[l].visual_instance_uids.size(); v++) {
        int visual_id = body->links()[l].visual_instance_uids[v];
        if (m_b2vis.find(visual_id) != m_b2vis.end()) {
          
          Vector3 pos (params[index],params[index+1],params[index+2]);
          Quaternion rot(params[index+3],params[index+4],params[index+5],params[index+6]);
          index+=7;
          //Transform geom_X_world2a =
          //    body->links()[l].X_world * body->links()[l].X_visuals[v];
          Transform geom_X_world;
          geom_X_world.translation = pos;
          geom_X_world.rotation = Algebra::quat_to_matrix(rot);
          
          const Matrix3x3 &m =
              geom_X_world.rotation;
          const TinyVisualLinkInfo &viz = m_b2vis.at(visual_id);
          // printf("vis_name=%s\n", viz.vis_name.c_str());
          double world_mat[9] = {
              m.getRow(0)[0], m.getRow(1)[0], m.getRow(2)[0],
              m.getRow(0)[1], m.getRow(1)[1], m.getRow(2)[1],
              m.getRow(0)[2], m.getRow(1)[2], m.getRow(2)[2]};
          double world_pos[3] = {geom_X_world.translation.getX(),
                                 geom_X_world.translation.getY(),
                                 geom_X_world.translation.getZ()};
          nlohmann::json tr_cmd =
              create_transform_cmd(world_pos, world_mat, viz.vis_name.c_str());
          send_zmq(m_sock, tr_cmd);
        }
      }
    }
  }

  //no-op to be compatible
  void render() {
  }
};

#endif  // MESHCAT_URDF_VISUALIZER_H
