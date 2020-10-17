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

#include <chrono>
#include <thread>

#include "visualizer/meshcat/meshcat_cube_data.h"
#include "visualizer/meshcat/meshcat_zmq.h"
#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_matrix3x3.h"
using namespace TINY;

using nlohmann::json;

int main(int argc, char* argv[]) {
  printf("Waiting for meshcat-server...\n");
  std::string texture_data = texture_data_broken_robot;

  zmq::context_t context;
  zmq::socket_t sock(context, ZMQ_REQ);

  sock.connect("tcp://127.0.0.1:6000");

  printf("Connected, running commands.");
  json del_cmd = create_delete_cmd();
  send_zmq(sock, del_cmd);
  if (1) {
    double world_pos_init[3] = {0, 4, 0};
    int color = 0xff0000;
    json sphere_cmd =
        create_sphere_cmd(1, world_pos_init, color, "/meshcat/sphere");

    send_zmq(sock, sphere_cmd);
  }

  {
    int color = 0xffffff;
    const char* path = "/meshcat/mesh";
    double world_pos[3] = {0, 0, 0};
    json mesh_cmd = create_textured_mesh_cmd(
        meshcat_cube_data, texture_data.c_str(), world_pos, color, path);
    send_zmq(sock, mesh_cmd);
  }
  if (1) {
    int color = 0x00ff00;
    double world_pos_init[3] = {0, 0, 0};
    const char* path = "/meshcat/box";
    json box_cmd = create_box_cmd(1, 1, 1, world_pos_init, color, path);
    send_zmq(sock, box_cmd);

    TinyMatrix3x3<double, DoubleUtils> m;
    double euler[3] = {0, 0, 0};

    for (int i = 0; i < 1000; i++) {
      euler[2] += 0.01;
      m.setEulerZYX(euler[0], euler[1], euler[2]);
      double dt = 1. / 60.;
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
      double world_mat[9] = {m.getRow(0)[0], m.getRow(1)[0], m.getRow(2)[0],
                             m.getRow(0)[1], m.getRow(1)[1], m.getRow(2)[1],
                             m.getRow(0)[2], m.getRow(1)[2], m.getRow(2)[2]};
      double world_pos[3] = {0, 0, 4};
      json tr_cmd = create_transform_cmd(world_pos, world_mat, path);
      send_zmq(sock, tr_cmd);
    }
  }
  return 0;
}
