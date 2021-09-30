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

#include <iostream>

#include <crossguid/guid.hpp>
#include <nlohmann/json.hpp>
inline std::string generate_uuid() { return xg::newGuid().str(); }

#include "zmq.hpp"
#include "zmq_addon.hpp"

inline nlohmann::json create_delete_cmd(
    const std::string &path = std::string("/meshcat")) {
  nlohmann::json delete_cmd = {{"type", "delete"}, {"path", path.c_str()}};
  return delete_cmd;
}

inline nlohmann::json create_sphere_cmd(double radius, double world_pos[3],
                                        int color_rgb, const char *path,
                                        bool transparent = false,
                                        double opacity = 1.0) {
  std::string geom_uid = generate_uuid();
  std::string material_uid = generate_uuid();
  std::string object_uid = generate_uuid();

  nlohmann::json set_object_sphere_cmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{
             {"radius", radius},
             {"type", "SphereGeometry"},
             {"uuid", geom_uid},
         }}},
        {"materials",
         {{{"color", color_rgb},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", transparent},
           {"opacity", opacity},
           {"type", "MeshPhongMaterial"},
           {"uuid", material_uid}}}},
        {
            "object",
            {{"geometry", geom_uid},
             {"material", material_uid},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               world_pos[0], world_pos[1], world_pos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", object_uid}},
        }}}};
  return set_object_sphere_cmd;
}

inline nlohmann::json create_box_cmd(double width, double height, double length,
                                     double world_pos[3], int color_rgb,
                                     const char *path) {
  std::string geom_uid = generate_uuid();
  std::string material_uid = generate_uuid();
  std::string object_uid = generate_uuid();

  nlohmann::json set_object_box_cmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{{"depth", length},
           {"height", height},
           {"type", "BoxGeometry"},
           {"uuid", geom_uid},
           {"width", width}}}},
        {"materials",
         {{{"color", color_rgb},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"type", "MeshPhongMaterial"},
           {"uuid", material_uid}}}},
        {
            "object",
            {{"geometry", geom_uid},
             {"material", material_uid},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               world_pos[0], world_pos[1], world_pos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", object_uid}},
        }}}};

  return set_object_box_cmd;
}

inline nlohmann::json create_mesh_cmd(const char *obj_data, double world_pos[3],
                                      int color_rgb, const char *path) {
  std::string geom_uid = generate_uuid();
  std::string material_uid = generate_uuid();
  std::string object_uid = generate_uuid();

  nlohmann::json set_object_box_cmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{

             {"type", "_meshfile"},
             {"uuid", geom_uid},
             {"format", "obj"},
             {"data", obj_data},
         }}},
        {"materials",
         {{{"color", color_rgb},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"type", "MeshPhongMaterial"},
           {"uuid", material_uid}}}},
        {
            "object",
            {{"geometry", geom_uid},
             {"material", material_uid},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               world_pos[0], world_pos[1], world_pos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", object_uid}},
        }}}};

  return set_object_box_cmd;
}

inline nlohmann::json create_textured_mesh_cmd(const char *obj_data,
                                               const char *texture_data,
                                               double world_pos[3],
                                               int color_rgb,
                                               const char *path) {
  std::string map_uuid = generate_uuid();
  std::string image_uuid = generate_uuid();
  std::string geom_uuid = generate_uuid();
  std::string material_uuid = generate_uuid();
  std::string object_uuid = generate_uuid();

  nlohmann::json set_object_box_cmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{

             {"type", "_meshfile"},
             {"uuid", geom_uuid},
             {"format", "obj"},
             {"data", obj_data},
         }}},
        {"images",
         {{
             {"uuid", image_uuid},
             {"url", texture_data},
         }}},
        {"textures",
         {{{"wrap", {1, 1}},
           {"uuid", map_uuid},
           {"repeat", {1, 1}},
           {"image", {image_uuid}}}}},
        {"materials",
         {{{"color", color_rgb},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"map", map_uuid},
           {"type", "MeshPhongMaterial"},
           {"uuid", material_uuid}}}},
        {
            "object",
            {{"geometry", geom_uuid},
             {"material", material_uuid},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               world_pos[0], world_pos[1], world_pos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", object_uuid}},
        }}}};

  return set_object_box_cmd;
}

inline nlohmann::json create_textured_mesh_cmd2(const char *obj_data,
                                                std::string map_uuid,
                                                double world_pos[3],
                                                int color_rgb,
                                                const char *path) {
  std::string image_uuid = generate_uuid();
  std::string geom_uuid = generate_uuid();
  std::string material_uuid = generate_uuid();
  std::string object_uuid = generate_uuid();

  nlohmann::json set_object_box_cmd = {
      {"type", "set_object"},
      {"path", path},
      {"object",
       {{"metadata", {{"type", "Object"}, {"version", 4.5}}},
        {"geometries",
         {{

             {"type", "_meshfile"},
             {"uuid", geom_uuid},
             {"format", "obj"},
             {"data", obj_data},
         }}},
        {"materials",
         {{{"color", color_rgb},
           {"reflectivity", 0.5},
           {"side", 2},
           {"transparent", false},
           {"opacity", 1.0},
           {"map", map_uuid},
           {"type", "MeshPhongMaterial"},
           {"uuid", material_uuid}}}},
        {
            "object",
            {{"geometry", geom_uuid},
             {"material", material_uuid},
             {"matrix",
              {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               world_pos[0], world_pos[1], world_pos[2], 1.0}},
             {"type", "Mesh"},
             {"uuid", object_uuid}},
        }}}};

  return set_object_box_cmd;
}

inline nlohmann::json create_transform_cmd(double world_pos[3],
                                           double world_mat3x3[9],
                                           const char *path) {
  nlohmann::json transform_cmd = {
      {"type", "set_transform"},
      {"path", path},
      {"matrix",
       {world_mat3x3[0], world_mat3x3[1], world_mat3x3[2], 0., world_mat3x3[3],
        world_mat3x3[4], world_mat3x3[5], 0., world_mat3x3[6], world_mat3x3[7],
        world_mat3x3[8], 0., world_pos[0], world_pos[1], world_pos[2], 1.0}},
  };
  return transform_cmd;
}

inline void send_zmq(zmq::socket_t &sock, nlohmann::json cmd,
                     bool verbose = false) {
  if (verbose) {
    std::cout << cmd.dump(4) << std::endl;
  }
  std::vector<uint8_t> packed2 = nlohmann::json::to_msgpack(cmd);
  std::string type_str = cmd["type"];
  std::string path_str = cmd["path"];
  zmq::multipart_t multipart;
  multipart.add(zmq::message_t(type_str.c_str(), type_str.size()));
  multipart.add(zmq::message_t(path_str.c_str(), path_str.size()));
  multipart.add(zmq::message_t(&packed2[0], packed2.size()));
  multipart.send(sock);

  zmq::message_t reply(1024);
  char buf[1024];
  bool recv_result = sock.recv(&reply);
  if (recv_result) {
    memcpy(buf, reply.data(), reply.size());
    buf[reply.size()] = 0;
    //nlohmann::json result = nlohmann::json::from_msgpack(buf, false);
    if (verbose) {
      std::cout << buf << std::endl;
    }
  }
}
