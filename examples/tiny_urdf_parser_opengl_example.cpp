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

//#include "meshcat_urdf_visualizer.h"
#include "opengl_urdf_visualizer.h"

#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#include "urdf/urdf_cache.hpp"
#include "tiny_visual_instance_generator.h"

using namespace TINY;
using namespace tds;

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
#include "math/tiny/tiny_algebra.hpp"
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

typedef TinyVector3<double, DoubleUtils> Vector3;
typedef TinyQuaternion<double, DoubleUtils> Quaternion;

double knee_angle = -0.5;
double abduction_angle = 0.2;
int frameskip_gfx_sync =10;  // only sync every 10 frames (sim at 1000 Hz, gfx at ~60hz)

double initial_poses[] = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};

bool do_sim = true;


TinyKeyboardCallback prev_keyboard_callback = 0;

void my_keyboard_callback(int keycode, int state)
{
    if (keycode == 's')
        do_sim = state;
    prev_keyboard_callback(keycode, state);
}



template <typename Algebra>
struct ContactSimulation {
    using Scalar = typename Algebra::Scalar;
    tds::UrdfCache<Algebra> cache;
    std::string m_urdf_filename;
    tds::World<Algebra> world;
    tds::MultiBody<Algebra>* mb_ = nullptr;

    int num_timesteps{ 1 };
    Scalar dt{ Algebra::from_double(1e-3) };

    int input_dim() const { return mb_->dof() + mb_->dof_qd(); }
    int state_dim() const {
        return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7;
    }
    int output_dim() const { return num_timesteps * state_dim(); }

    ContactSimulation() {
        std::string plane_filename;
        tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
        cache.construct(plane_filename, world, false, false);
        tds::FileUtils::find_file("laikago/laikago_toes_zup.urdf", m_urdf_filename);
        mb_ = cache.construct(m_urdf_filename, world, false, true);
        mb_->base_X_world().translation = Algebra::unit3_z();
    }

    std::vector<Scalar> operator()(const std::vector<Scalar>& v) {
        assert(static_cast<int>(v.size()) == input_dim());
        mb_->initialize();
        //copy input into q, qd
        for (int i = 0; i < mb_->dof(); ++i) {
            mb_->q(i) = v[i];
        }
        for (int i = 0; i < mb_->dof_qd(); ++i) {
            mb_->qd(i) = v[i + mb_->dof()];
        }
        std::vector<Scalar> result(output_dim());
        for (int t = 0; t < num_timesteps; ++t) {

            // pd control
            if (1) {
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
                        double q_desired = initial_poses[pose_index++];
                        double q_actual = mb_->q_[q_offset];
                        double qd_actual = mb_->qd_[qd_offset];
                        double position_error = (q_desired - q_actual);
                        double desired_velocity = 0;
                        double velocity_error = (desired_velocity - qd_actual);
                        double force = kp * position_error + kd * velocity_error;

                        if (force < -max_force) force = -max_force;
                        if (force > max_force) force = max_force;
                        mb_->tau_[tau_index] = force;
                        q_offset++;
                        qd_offset++;
                        param_index++;
                        tau_index++;
                    }
                }
            }

            tds::forward_dynamics(*mb_, world.get_gravity());
            mb_->clear_forces();

            integrate_euler_qdd(*mb_, dt);

            world.step(dt);
            
            tds::integrate_euler(*mb_, dt);

            //copy q, qd, link world poses (for rendering) to output
            int j = 0;
            for (int i = 0; i < mb_->dof(); ++i, ++j) {
                result[j] = mb_->q(i);
            }
            for (int i = 0; i < mb_->dof_qd(); ++i, ++j) {
                result[j] = mb_->qd(i);
            }
            for (const auto link : *mb_) {
                if (link.X_visuals.size())
                {
                    Transform visual_X_world = link.X_world * link.X_visuals[0];
                    result[j++] = visual_X_world.translation[0];
                    result[j++] = visual_X_world.translation[1];
                    result[j++] = visual_X_world.translation[2];
                    auto orn = Algebra::matrix_to_quat(visual_X_world.rotation);
                    result[j++] = orn[0];
                    result[j++] = orn[1];
                    result[j++] = orn[2];
                    result[j++] = orn[3];
                }
                else
                {
                    //check if we have links without visuals
                    assert(0);
                    j += 7;
                }
            }
        }
        return result;
    }
};



int main(int argc, char* argv[]) {
  int sync_counter = 0;
  int frame = 0;
  World<MyAlgebra> world;
  UrdfParser<MyAlgebra> parser;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  
  
  
  visualizer.delete_all();
#if 1
  ContactSimulation<MyAlgebra> contact_sim;
  
  int input_dim = contact_sim.input_dim();
  std::vector<MyAlgebra::Scalar> prep_inputs;
  
  std::vector<MyAlgebra::Scalar> prep_outputs;
  prep_inputs.resize(input_dim);
  prep_inputs[3] = 1;
  prep_inputs[6] = 1;

  //int sphere_shape = visualizer.m_opengl_app.register_graphics_unit_sphere_shape(SPHERE_LOD_LOW);
  
  {
      std::vector<int> shape_ids;
      std::string plane_filename;
      FileUtils::find_file("plane100.obj", plane_filename);
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);
      visualizer.load_obj(plane_filename, pos, orn, scaling, shape_ids);
  }

  //int sphere_shape = shape_ids[0];
  //TinyVector3f color = colors[0];
  // typedef tds::Conversion<DiffAlgebra, tds::TinyAlgebraf> Conversion;
  
  bool create_instances = false;
  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string texture_path = "";
  std::string file_and_path;
  tds::FileUtils::find_file("laikago/laikago_toes_zup.urdf", file_and_path);
  auto urdf_structures = contact_sim.cache.retrieve(file_and_path);// contact_sim.m_urdf_filename);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);
  
  
  int num_total_threads = 2;
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances;
  

  for (int t = 0;t< num_total_threads;t++)
  {
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);
      int uid = urdf_structures.base_links[0].urdf_visual_shapes[0].visual_shape_uid;
      OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
      int instance = -1;
      int num_instances_per_link = 0;
      for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
      {
          int sphere_shape = vis_link.visual_shape_uids[v];
          ::TINY::TinyVector3f color(1, 1, 1);
          //visualizer.m_b2vis
          instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
              sphere_shape, pos, orn, color, scaling);
          visual_instances.push_back(instance);
          num_instances_per_link++;
          contact_sim.mb_->visual_instance_uids().push_back(instance);
      }
      num_base_instances = num_instances_per_link;

      for (int i = 0; i < contact_sim.mb_->num_links(); ++i) {
         

          int uid = urdf_structures.links[i].urdf_visual_shapes[0].visual_shape_uid;
          OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
          int instance = -1;
          int num_instances_per_link = 0;
          for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
          {
              int sphere_shape = vis_link.visual_shape_uids[v];
              ::TINY::TinyVector3f color(1, 1, 1);
              //visualizer.m_b2vis
              instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                  sphere_shape, pos, orn, color, scaling);
              visual_instances.push_back(instance);
              num_instances_per_link++;

              contact_sim.mb_->links_[i].visual_instance_uids.push_back(instance);
          }
          num_instances.push_back(num_instances_per_link);
      }
  }

  //app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, sphereId);

  

  std::vector<std::vector<MyScalar>> parallel_outputs(
      num_total_threads, std::vector<MyScalar>(contact_sim.output_dim()));

  std::vector<std::vector<MyScalar>> parallel_inputs(num_total_threads);

  for (int i = 0; i < num_total_threads; ++i) {
      parallel_inputs[i] = std::vector<MyScalar>(contact_sim.input_dim(), MyScalar(0));
      parallel_inputs[i][3] = 1;
      parallel_inputs[i][6] = 1;
      
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
              if (manual_sync)
              {
                  visualizer.sync_visual_transforms(contact_sim.mb_);
              }
              else
              {
                  float sim_spacing = 2;
                  const int square_id = (int)std::sqrt((double)num_total_threads);
                  int instance_index = 0;
                  int offset = contact_sim.mb_->dof() + contact_sim.mb_->dof_qd();
                  for (int s = 0; s < num_total_threads; s++)
                  {
                      
                      
                      for (int v = 0; v < num_base_instances; v++)
                      {
                          int visual_instance_id = visual_instances[instance_index++];
                          if (visual_instance_id >= 0)
                          {

                              ::TINY::TinyVector3f pos(parallel_outputs[s][4 + 0],
                                  parallel_outputs[s][4 + 1],
                                  parallel_outputs[s][4 + 2]);
                              ::TINY::TinyQuaternionf orn(parallel_outputs[s][0],
                                  parallel_outputs[s][1],
                                  parallel_outputs[s][2],
                                  parallel_outputs[s][3]);

                              pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                              pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                              visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
                          }
                      }
                      
                      for (int l = 0; l < contact_sim.mb_->links_.size(); l++) {
                          for (int v = 0; v < num_instances[l]; v++)
                          {
                              int visual_instance_id = visual_instances[instance_index++];
                              if (visual_instance_id >= 0)
                              {

                                  ::TINY::TinyVector3f pos(parallel_outputs[s][offset + l * 7 + 0],
                                      parallel_outputs[s][offset + l * 7 + 1],
                                      parallel_outputs[s][offset + l * 7 + 2]);
                                  ::TINY::TinyQuaternionf orn(parallel_outputs[s][offset + l * 7 + 3],
                                      parallel_outputs[s][offset + l * 7 + 4],
                                      parallel_outputs[s][offset + l * 7 + 5],
                                      parallel_outputs[s][offset + l * 7 + 6]);

                                  pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                                  pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                                  visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
                              }
                          }
                      }
                  }
              }
          }
          visualizer.render();
          std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync* contact_sim.dt));
      }
  }

   
#else
  std::string plane_file_name;
  FileUtils::find_file("plane_implicit.urdf", plane_file_name);
  char plane_search_path[TINY_MAX_EXE_PATH_LEN];
  FileUtils::extract_path(plane_file_name.c_str(), plane_search_path,
                              TINY_MAX_EXE_PATH_LEN);
  MultiBody<MyAlgebra>& plane_mb = *world.create_multi_body();
  plane_mb.set_floating_base(false);
  
  {
    TinyVisualInstanceGenerator<MyAlgebra> vig(visualizer);
    UrdfStructures<MyAlgebra> plane_urdf_structures =
        parser.load_urdf(plane_file_name);
    std::string texture_path = "checker_purple.png";
    visualizer.m_path_prefix = plane_search_path;
    visualizer.convert_visuals(plane_urdf_structures, texture_path);

    UrdfToMultiBody<MyAlgebra>::convert_to_multi_body(
        plane_urdf_structures, world, plane_mb, &vig);
    
    //
  }
  prev_keyboard_callback = visualizer.m_opengl_app.m_window->get_keyboard_callback();
  visualizer.m_opengl_app.m_window->set_keyboard_callback(my_keyboard_callback);

  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string file_name;
  FileUtils::find_file("laikago/laikago_toes_zup.urdf", file_name);
  FileUtils::extract_path(file_name.c_str(), search_path,
                              TINY_MAX_EXE_PATH_LEN);

  std::ifstream ifs(file_name);
  std::string urdf_string;
  if (!ifs.is_open()) {
    std::cout << "Error, cannot open file_name: " << file_name << std::endl;
    exit(-1);
  }

  urdf_string = std::string((std::istreambuf_iterator<char>(ifs)),
                            std::istreambuf_iterator<char>());
  StdLogger logger;
  UrdfStructures<MyAlgebra> urdf_structures;
  int flags = 0;
  parser.load_urdf_from_string(urdf_string, flags, logger, urdf_structures);
  // create graphics structures
  std::string texture_path = "laikago_tex.jpg";
  visualizer.m_path_prefix = search_path;
  MultiBody<MyAlgebra>& mb = *world.create_multi_body();
  bool floating_base = true;
  visualizer.convert_visuals(urdf_structures, texture_path);

  mb.set_floating_base(true);
  {
      TinyVisualInstanceGenerator<MyAlgebra> vig(visualizer);
      UrdfToMultiBody<MyAlgebra>::convert_to_multi_body(
          urdf_structures, world, mb, &vig);
  }
  mb.initialize();

  //visualizer.create_visual_instances(mb);




  int start_index = 0;
  if (floating_base) {
    start_index = 7;
    mb.q_[0] = 0;
    mb.q_[1] = 0;
    mb.q_[2] = 0;
    mb.q_[3] = 1;

    mb.q_[4] = 0;
    mb.q_[5] = 0;
    mb.q_[6] = 1.5;

    mb.qd_[0] = 0;
    mb.qd_[1] = 0;
    mb.qd_[2] = 0;
    mb.qd_[3] = 0;
  }
  if (mb.q_.size() >= 12) {
    for (int cc = 0; cc < 12; cc++) {
      mb.q_[start_index + cc] = initial_poses[cc];
    }
  }
  mb.set_position(TinyVector3<double, DoubleUtils>(0., 0., 0.6));
  mb.set_orientation(TinyQuaternion<double, DoubleUtils>(0.0, 0.0, 0.706825181105366, 0.7073882691671998 ));
  world.default_friction = 1.0;

  TinyVector3<double, DoubleUtils> grav(DoubleUtils::zero(),
                                        DoubleUtils::zero(),
                                        DoubleUtils::fraction(-1000, 100));
  double dt = 1. / 1000.;
  
  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
    
      forward_kinematics(mb);

      if (do_sim) {
          

          forward_dynamics(mb, grav);


          integrate_euler_qdd(mb, dt);

          // pd control
          if (1) {
              // use PD controller to compute tau
              int qd_offset = mb.is_floating() ? 6 : 0;
              int q_offset = mb.is_floating() ? 7 : 0;
              int num_targets = mb.tau_.size() - qd_offset;
              std::vector<double> q_targets;
              q_targets.resize(mb.tau_.size());

              double kp = 150;
              double kd = 3;
              double max_force = 550;
              int param_index = 0;

              for (int i = 0; i < mb.tau_.size(); i++) {
                  mb.tau_[i] = 0;
              }
              int tau_index = 0;
              int pose_index = 0;
              for (int i = 0; i < mb.links_.size(); i++) {
                  if (mb.links_[i].joint_type != JOINT_FIXED) {
                      double q_desired = initial_poses[pose_index++];
                      double q_actual = mb.q_[q_offset];
                      double qd_actual = mb.qd_[qd_offset];
                      double position_error = (q_desired - q_actual);
                      double desired_velocity = 0;
                      double velocity_error = (desired_velocity - qd_actual);
                      double force = kp * position_error + kd * velocity_error;

                      if (force < -max_force) force = -max_force;
                      if (force > max_force) force = max_force;
                      mb.tau_[tau_index] = force;
                      q_offset++;
                      qd_offset++;
                      param_index++;
                      tau_index++;
                  }
              }
          }



          world.step(dt);

          integrate_euler(mb, dt);

      }
      
    sync_counter++;
    frame += 1;
    if (sync_counter > frameskip_gfx_sync) {
      sync_counter = 0;
      visualizer.sync_visual_transforms(&mb);
      visualizer.render();
      std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync*dt));
    }
  }
#endif
  printf("finished\n");
  return EXIT_SUCCESS;

}

