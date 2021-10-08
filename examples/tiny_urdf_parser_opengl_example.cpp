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
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <thread>

#include "opengl_urdf_visualizer.h"
#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "urdf/urdf_cache.hpp"
#include "tiny_visual_instance_generator.h"

// If set to 1, uses sdf to generate the base plane. Otherwise, uses the default
// plane object
//#define USE_SDF_PLANE 1

int num_total_threads = 1;
bool enable_sleep = true;
int frameskip_gfx_sync =
    10; // only sync every 10 frames (sim at 1000 Hz, gfx at ~60hz)
bool use_plane = true;
// sphere8cube_inertia_xyzspherical.urdf

//#define USE_PANDA
#define USE_LAIKAGO

#ifdef USE_LAIKAGO
 //std::string urdf_name = "laikago/laikago_toes_zup_chassis_collision.urdf";
  std::string urdf_name = "amass_fixed.urdf";
//std::string urdf_name = "capsule.urdf";//sphere8cube.urdf";//laikago/laikago_toes_zup.urdf";
//std::string urdf_name = "sphere8cube.urdf";//laikago/laikago_toes_zup.urdf";
bool is_floating = true;
double knee_angle = -0.5;
double abduction_angle = 0.2;
std::vector<double> initial_poses = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};
#endif

#ifdef USE_PANDA
std::string urdf_name = "franka_panda/panda.urdf";
std::vector<double> initial_poses = {0.98, 0.458, 0.31, -2.24, -0.30,
                                     2.66, 2.32,  0.02, 0.02}; // panda
bool is_floating = false;
#endif
// std::string urdf_name = "sphere8cube_shift_shapes_render_spheres.urdf";

// std::string urdf_name = "sphere8cube_inertia.urdf";bool is_floating = true;
// std::string urdf_name = "sphere8cube_inertia_xyztrans_xyzrot.urdf";bool
// is_floating = false; std::string urdf_name =
// "sphere8cube_col_visual_shift.urdf";bool is_floating = true;

// std::string urdf_name = "two_cubes2special.urdf";bool is_floating = true;
// std::string urdf_name = "two_cubes2.urdf";bool is_floating = true;
// std::string urdf_name = "sphere8cube_inertia_xyzspherical.urdf";
// std::string urdf_name = "humanoid_partial.urdf";
// std::string urdf_name = "pendulum5spherical.urdf"; bool is_floating = false;
// std::string urdf_name = "humanoid.urdf"; bool is_floating = true;
// std::string urdf_name = "hopper_link0_1.urdf"; bool is_floating = false;
// std::string urdf_name = "sphere_small_visual.urdf"; bool is_floating = false;

// std::vector<double> initial_poses;

using namespace TINY;
using namespace tds;

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
#include "math/tiny/tiny_algebra.hpp"
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

typedef TinyVector3<double, DoubleUtils> Vector3;
typedef TinyQuaternion<double, DoubleUtils> Quaternion;

MyAlgebra::Vector3 start_pos(0, 0, 1.15); // 0.4002847
Quaternion cap_start_orn = 
    MyAlgebra::quat_from_euler_rpy(Vector3(0.,0.,0.));
//Quaternion cap_start_orn = 
//    MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(0.1,0.2,0.3));//0,MyAlgebra::pi()/2.,0));
 //MyAlgebra::Quaternion start_orn =
 //MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(-3.14/2.,0,0));
// MyAlgebra::Quaternion start_orn(0.23364591,0,0,0.97232174932);
// MyAlgebra::Quaternion start_orn(0,0,0,1);//0.23364591,0,0,0.97232174932);
// MyAlgebra::Quaternion start_orn =
// MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(3.14/5.,0,0));
// MyAlgebra::Quaternion start_orn =
// MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(0,3.14/2.,0));
//MyAlgebra::Quaternion start_orn(0, 0, 0, 1);

MyAlgebra::Quaternion plane_start_orn =
    MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(0,0,0));//0.1,0.2,0.3));


bool do_sim = true;

TinyKeyboardCallback prev_keyboard_callback = 0;

void my_keyboard_callback(int keycode, int state) {
  if (keycode == 's')
    do_sim = state;
  prev_keyboard_callback(keycode, state);
}

template <typename Algebra> struct ContactSimulation {
  using Scalar = typename Algebra::Scalar;
  tds::UrdfCache<Algebra> cache;
  std::string m_urdf_filename;
  tds::World<Algebra> world;
  tds::MultiBody<Algebra> *mb_ = nullptr;

  int num_timesteps{1};
  Scalar dt{Algebra::from_double(1e-3)};

  int input_dim() const { return mb_->dof() + mb_->dof_qd(); }
  int state_dim() const {
    return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7;
  }
  int output_dim() const { return num_timesteps * state_dim(); }

  ContactSimulation() {
    if (use_plane) {
      std::string plane_filename;
      tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
      cache.construct(plane_filename, world, false, false);
    }
    tds::FileUtils::find_file(urdf_name, m_urdf_filename);

    mb_ = cache.construct(m_urdf_filename, world, false, is_floating);
    mb_->base_X_world().translation = start_pos;
    mb_->base_X_world().rotation = Algebra::quat_to_matrix(cap_start_orn);
    world.default_friction = 1;
    world.set_gravity(Vector3(0,2,-10));
    // initial_poses.resize(mb_->q_.size());
    // for (int i=0;i<mb_->q_.size();i++)
    //{
    //    initial_poses[i] = mb_->q_[i];
    //}
  }

  std::vector<Scalar> operator()(const std::vector<Scalar> &v) {
    assert(static_cast<int>(v.size()) == input_dim());
    mb_->initialize();
    // copy input into q, qd
    for (int i = 0; i < mb_->dof(); ++i) {
      mb_->q(i) = v[i];
    }
    for (int i = 0; i < mb_->dof_qd(); ++i) {
      mb_->qd(i) = v[i + mb_->dof()];
    }
    std::vector<Scalar> result(output_dim());
    for (int t = 0; t < num_timesteps; ++t) {

      // pd control
      if (0) {
        // use PD controller to compute tau
        int qd_offset = mb_->is_floating() ? 6 : 0;
        int q_offset = mb_->is_floating() ? 7 : 0;
        int num_targets = mb_->tau_.size() - qd_offset;
        std::vector<double> q_targets;
        q_targets.resize(mb_->tau_.size());

        double kp = 150;
        double kd = 3;
        double max_force = 550;
        int param_index = 0;

        for (int i = 0; i < mb_->tau_.size(); i++) {
          mb_->tau_[i] = 0;
        }
        int tau_index = 0;
        int pose_index = 0;
        for (int i = 0; i < mb_->links_.size(); i++) {
          if (mb_->links_[i].joint_type != JOINT_FIXED) {

            if (pose_index < initial_poses.size()) {
              double q_desired = initial_poses[pose_index++];
              double q_actual = mb_->q_[q_offset];
              double qd_actual = mb_->qd_[qd_offset];
              double position_error = (q_desired - q_actual);
              double desired_velocity = 0;
              double velocity_error = (desired_velocity - qd_actual);
              double force = kp * position_error + kd * velocity_error;

              if (force < -max_force)
                force = -max_force;
              if (force > max_force)
                force = max_force;
              mb_->tau_[tau_index] = force;
              q_offset++;
              qd_offset++;
              param_index++;
              tau_index++;
            }
          }
        }
      }

      tds::forward_dynamics(*mb_, world.get_gravity());
      mb_->clear_forces();

      integrate_euler_qdd(*mb_, dt);

      world.step(dt);

      tds::integrate_euler(*mb_, dt);

      // copy q, qd, link world poses (for rendering) to output
      int j = 0;
      for (int i = 0; i < mb_->dof(); ++i, ++j) {
        result[j] = mb_->q(i);
      }
      for (int i = 0; i < mb_->dof_qd(); ++i, ++j) {
        result[j] = mb_->qd(i);
      }
      for (const auto link : *mb_) {
        // just copy the link world transform. Still have to multiple with
        // visual transform for each instance.
        Transform visual_X_world = link.X_world; //* link.X_visuals[0];//
        result[j++] = visual_X_world.translation[0];
        result[j++] = visual_X_world.translation[1];
        result[j++] = visual_X_world.translation[2];
        auto orn = Algebra::matrix_to_quat(visual_X_world.rotation);
        result[j++] = orn[0];
        result[j++] = orn[1];
        result[j++] = orn[2];
        result[j++] = orn[3];
      }
    }
    return result;
  }
};

int main(int argc, char *argv[]) {
  MyAlgebra::Vector3 n = plane_start_orn.rotate(MyAlgebra::Vector3(0,0,1));
  int sync_counter = 0;
  int frame = 0;
  World<MyAlgebra> world;
  UrdfParser<MyAlgebra> parser;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;

  visualizer.delete_all();

  ContactSimulation<MyAlgebra> contact_sim;

  int input_dim = contact_sim.input_dim();
  std::vector<MyAlgebra::Scalar> prep_inputs;

  std::vector<MyAlgebra::Scalar> prep_outputs;
  prep_inputs.resize(input_dim);
  prep_inputs[3] = 1;
  prep_inputs[6] = 1;

  // int sphere_shape =
  // visualizer.m_opengl_app.register_graphics_unit_sphere_shape(SPHERE_LOD_LOW);

  if (use_plane) {
#if USE_SDF_PLANE
    // Use the purple_checker as the texture file
    std::string texture_filename = "checker_purple.png";
    std::string texture_path;
    tds::FileUtils::find_file(texture_filename, texture_path);
    std::vector<unsigned char> buffer;
    int width, height, n;
    unsigned char *image =
        stbi_load(texture_path.c_str(), &width, &height, &n, 3);

    int textureIndex = visualizer.m_opengl_app.m_renderer->register_texture(
        image, width, height);
    free(image);

    tds::Plane<MyAlgebra> base_plane;
    tds::RenderShape gen_mesh = tds::convert_sdf_to_mesh(base_plane, 100);
    int shape_id = visualizer.m_opengl_app.m_instancingRenderer->register_shape(
        &gen_mesh.vertices[0].x, gen_mesh.vertices.size(), &gen_mesh.indices[0],
        gen_mesh.num_triangles * 3, B3_GL_TRIANGLES, textureIndex);
    TinyVector3f pos(0, 0, 0);
    TinyQuaternionf orn(0, 0, 0, 1);
    TinyVector3f scaling(1, 1, 1);
    TinyVector3f color(1, 1, 1);
    int instance_id = visualizer.m_opengl_app.m_instancingRenderer
                          ->register_graphics_instance(shape_id, pos, orn,
                                                       color, scaling, 1.0);
#else
    std::vector<int> shape_ids;
    std::string plane_filename;
    FileUtils::find_file("plane100.obj", plane_filename);
    TinyVector3f pos(0, 0, 0);
    TinyQuaternionf orn(plane_start_orn.x(),plane_start_orn.y(),plane_start_orn.z(),plane_start_orn.w());
    
    TinyVector3f scaling(1, 1, 1);
    visualizer.load_obj(plane_filename, pos, orn, scaling, shape_ids);
#endif
  }

  // int sphere_shape = shape_ids[0];
  // TinyVector3f color = colors[0];
  // typedef tds::Conversion<DiffAlgebra, tds::TinyAlgebraf> Conversion;

  bool create_instances = false;
  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string texture_path = "";
  std::string file_and_path;
  tds::FileUtils::find_file(urdf_name, file_and_path);
  auto urdf_structures = contact_sim.cache.retrieve(
      file_and_path); // contact_sim.m_urdf_filename);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
                          TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);

  std::vector<int> visual_instances;
  std::vector<int> visual_b2_uids;
  std::vector<int> num_link_instances;
  int num_base_instances = 0;

  for (int t = 0; t < num_total_threads; t++) {
    TinyVector3f pos(0, 0, 0);
    TinyQuaternionf orn(0, 0, 0, 1);
    TinyVector3f scaling(1, 1, 1);
    num_base_instances = 0;
    for (int bb = 0;
         bb < urdf_structures.base_links[0].urdf_visual_shapes.size(); bb++) {
      int uid =
          urdf_structures.base_links[0].urdf_visual_shapes[bb].visual_shape_uid;
      OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo &vis_link =
          visualizer.m_b2vis[uid];
      int instance = -1;
      int num_instances_per_link = 0;
      for (int v = 0; v < vis_link.visual_shape_uids.size(); v++) {
        int sphere_shape = vis_link.visual_shape_uids[v];
        ::TINY::TinyVector3f color(1, 1, 1);
        // visualizer.m_b2vis
        instance =
            visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                sphere_shape, pos, orn, color, scaling);
        visual_instances.push_back(instance);
        visual_b2_uids.push_back(uid);
        num_instances_per_link++;
        contact_sim.mb_->visual_instance_uids().push_back(instance);
      }
      num_base_instances += num_instances_per_link;
    }

    for (int i = 0; i < contact_sim.mb_->num_links(); ++i) {

      int num_instances_per_link = 0;
      for (int bb = 0; bb < urdf_structures.links[i].urdf_visual_shapes.size();
           bb++) {
        int uid =
            urdf_structures.links[i].urdf_visual_shapes[bb].visual_shape_uid;
        OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo &vis_link =
            visualizer.m_b2vis[uid];
        int instance = -1;

        // num_link_instances.clear();
        for (int v = 0; v < vis_link.visual_shape_uids.size(); v++) {
          int sphere_shape = vis_link.visual_shape_uids[v];
          ::TINY::TinyVector3f color(1, 1, 1);
          // visualizer.m_b2vis
          instance =
              visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                  sphere_shape, pos, orn, color, scaling);
          visual_instances.push_back(instance);
          visual_b2_uids.push_back(uid);
          num_instances_per_link++;
          contact_sim.mb_->links_[i].visual_instance_uids.push_back(instance);
        }
      }
      num_link_instances.push_back(num_instances_per_link);
    }
  }

  // app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, sphereId);

  std::vector<std::vector<MyScalar>> parallel_outputs(
      num_total_threads, std::vector<MyScalar>(contact_sim.output_dim()));

  std::vector<std::vector<MyScalar>> parallel_inputs(num_total_threads);

  for (int i = 0; i < num_total_threads; ++i) {
    parallel_inputs[i] =
        std::vector<MyScalar>(contact_sim.input_dim(), MyScalar(0));
    if (contact_sim.mb_->is_floating()) {
      parallel_inputs[i][0] = cap_start_orn.x();
      parallel_inputs[i][1] = cap_start_orn.y();
      parallel_inputs[i][2] = cap_start_orn.z();
      parallel_inputs[i][3] = cap_start_orn.w();

      parallel_inputs[i][4] = start_pos.x();
      parallel_inputs[i][5] = start_pos.y();
      parallel_inputs[i][6] = start_pos.z();
      parallel_inputs[i][7+3] = 1;//spherical joint
      
      //parallel_inputs[i][7] = 1;
      //parallel_inputs[i][8] = 1;
      //parallel_inputs[i][9] = 1;
      //parallel_inputs[i][11] = 10;
    }

#ifdef USE_PANDA
    for (int j = 0; j < initial_poses.size(); j++) {
      parallel_inputs[i][j] = initial_poses[j];
    }
#endif
  }

  while (!visualizer.m_opengl_app.m_window->requested_exit()) {

    for (int i = 0; i < num_total_threads; ++i) {
      parallel_outputs[i] = contact_sim(parallel_inputs[i]);
      for (int j = 0; j < contact_sim.input_dim(); ++j) {
        parallel_inputs[i][j] = parallel_outputs[i][j];
      }
    }

    sync_counter++;
    frame += 1;
    if (sync_counter > frameskip_gfx_sync) {
      sync_counter = 0;
      if (1) {
        bool manual_sync = false;
        if (manual_sync) {
          visualizer.sync_visual_transforms(contact_sim.mb_);
        } else {
          float sim_spacing = 2;
          const int square_id = (int)std::sqrt((double)num_total_threads);
          int offset = contact_sim.mb_->dof() + contact_sim.mb_->dof_qd();
          int instance_index = 0;
          for (int s = 0; s < num_total_threads; s++) {
            for (int v = 0; v < num_base_instances; v++) {
              int visual_instance_id = visual_instances[instance_index];
              if (visual_instance_id >= 0) {
                int uid1 = visual_b2_uids[instance_index];
                OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo &vis_link =
                    visualizer.m_b2vis[uid1];

                ::TINY::TinyVector3f pos(start_pos.x(), start_pos.y(),
                                         start_pos.z());
                ::TINY::TinyQuaternionf orn(plane_start_orn.x(), plane_start_orn.y(),
                                            plane_start_orn.z(), plane_start_orn.w());

                if (contact_sim.mb_->is_floating()) {
                  pos = ::TINY::TinyVector3f(parallel_outputs[s][4 + 0],
                                             parallel_outputs[s][4 + 1],
                                             parallel_outputs[s][4 + 2]);
                  orn = ::TINY::TinyQuaternionf(
                      parallel_outputs[s][0], parallel_outputs[s][1],
                      parallel_outputs[s][2], parallel_outputs[s][3]);
                }
                MyAlgebra::Vector3 visual_pos = vis_link.origin_xyz;
                MyAlgebra::Vector3 visual_orn = vis_link.origin_rpy;
                MyAlgebra::Quaternion orn2d =
                    MyAlgebra::quat_from_euler_rpy(vis_link.origin_rpy);
                ::TINY::TinyQuaternionf orn2(orn2d.x(), orn2d.y(), orn2d.z(),
                                             orn2d.w());
                ::TINY::TinyPosef global_tr(pos, orn);
                ::TINY::TinyPosef local_tr(::TINY::TinyVector3f(visual_pos.x(),
                                                                visual_pos.y(),
                                                                visual_pos.z()),
                                           orn2);
                ::TINY::TinyPosef full_tr = global_tr * local_tr;

                full_tr.m_position[0] +=
                    sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                full_tr.m_position[1] +=
                    sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                visualizer.m_opengl_app.m_renderer
                    ->write_single_instance_transform_to_cpu(
                        full_tr.m_position, full_tr.m_orientation,
                        visual_instance_id);
              }
              instance_index++;
            }

            for (int l = 0; l < contact_sim.mb_->links_.size(); l++) {

              for (int v = 0; v < num_link_instances[l]; v++) {
                int visual_instance_id = visual_instances[instance_index];
                if (visual_instance_id >= 0) {
                  int uid1 = visual_b2_uids[instance_index];
                  OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo
                      &vis_link = visualizer.m_b2vis[uid1];

                  ::TINY::TinyVector3f pos(
                      parallel_outputs[s][offset + l * 7 + 0],
                      parallel_outputs[s][offset + l * 7 + 1],
                      parallel_outputs[s][offset + l * 7 + 2]);
                  ::TINY::TinyQuaternionf orn(
                      parallel_outputs[s][offset + l * 7 + 3],
                      parallel_outputs[s][offset + l * 7 + 4],
                      parallel_outputs[s][offset + l * 7 + 5],
                      parallel_outputs[s][offset + l * 7 + 6]);

                  MyAlgebra::Vector3 visual_pos = vis_link.origin_xyz;
                  MyAlgebra::Vector3 visual_orn = vis_link.origin_rpy;
                  MyAlgebra::Quaternion orn2d =
                      MyAlgebra::quat_from_euler_rpy(vis_link.origin_rpy);
                  ::TINY::TinyQuaternionf orn2(orn2d.x(), orn2d.y(), orn2d.z(),
                                               orn2d.w());
                  ::TINY::TinyPosef global_tr(pos, orn);
                  ::TINY::TinyPosef local_tr(
                      ::TINY::TinyVector3f(visual_pos.x(), visual_pos.y(),
                                           visual_pos.z()),
                      orn2);
                  ::TINY::TinyPosef full_tr = global_tr * local_tr;

                  full_tr.m_position[0] += sim_spacing * (s % square_id) -
                                           square_id * sim_spacing / 2;
                  full_tr.m_position[1] += sim_spacing * (s / square_id) -
                                           square_id * sim_spacing / 2;

                  visualizer.m_opengl_app.m_renderer
                      ->write_single_instance_transform_to_cpu(
                          full_tr.m_position, full_tr.m_orientation,
                          visual_instance_id);
                }
                instance_index++;
              }
            }
          }
        }
      }
      visualizer.render();

      if (enable_sleep) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(frameskip_gfx_sync * contact_sim.dt));
      }
    }
  }
}
