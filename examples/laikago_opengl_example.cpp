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

std::string LAIKAGO_URDF_NAME="laikago/laikago_toes_zup_chassis_collision.urdf";
//std::string LAIKAGO_URDF_NAME="laikago/laikago_toes_zup.urdf";

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

using namespace tds;
using namespace TINY;
#include "math/tiny/tiny_algebra.hpp"

#ifdef USE_TINY

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyVector3<double, DoubleUtils> Vector3;
typedef TinyQuaternion<double, DoubleUtils> Quaternion;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;
#else
#include "math/eigen_algebra.hpp"
typedef EigenAlgebra MyAlgebra;
typedef typename MyAlgebra::Scalar MyScalar;
typedef typename MyAlgebra::Vector3 Vector3;
typedef typename MyAlgebra::Quaternion Quarternion;
typedef typename MyAlgebra::VectorX VectorX;
typedef typename MyAlgebra::Matrix3 Matrix3;
typedef typename MyAlgebra::Matrix3X Matrix3X;
typedef typename MyAlgebra::MatrixX MatrixX;
#endif





#ifdef _DEBUG
int frameskip_gfx_sync = 1;  // don't skip, we are debugging
#else
int frameskip_gfx_sync = 10;  // only sync every 10 frames (sim at 1000 Hz, gfx at ~60hz)
#endif


bool do_sim = true;


TinyKeyboardCallback prev_keyboard_callback = 0;

void my_keyboard_callback(int keycode, int state)
{
    if (keycode == 's')
        do_sim = state;
    prev_keyboard_callback(keycode, state);
}

double knee_angle = -0.5;
double abduction_angle = 0.2;


double initial_poses[] = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};


template <typename Algebra>
struct LaikagoSimulation {
    using Scalar = typename Algebra::Scalar;
    tds::UrdfCache<Algebra> cache;
    std::string m_urdf_filename;
    tds::World<Algebra> world;
    tds::MultiBody<Algebra>* system = nullptr;

    int num_timesteps{ 1 };
    Scalar dt{ Algebra::from_double(1e-3) };

    int input_dim() const { return system->dof() + system->dof_qd(); }
    int state_dim() const {
        return system->dof() + system->dof_qd() + system->num_links() * 7;
    }
    int output_dim() const { return num_timesteps * state_dim(); }

    LaikagoSimulation() {
        std::string plane_filename;
        tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
        cache.construct(plane_filename, world, false, false);
        tds::FileUtils::find_file(LAIKAGO_URDF_NAME, m_urdf_filename);
        system = cache.construct(m_urdf_filename, world, false, true);
        system->base_X_world().translation = Algebra::unit3_z();
    }

    std::vector<Scalar> operator()(const std::vector<Scalar>& v) {
        assert(static_cast<int>(v.size()) == input_dim());
        system->initialize();
        //copy input into q, qd
        for (int i = 0; i < system->dof(); ++i) {
            system->q(i) = v[i];
        }
        for (int i = 0; i < system->dof_qd(); ++i) {
            system->qd(i) = v[i + system->dof()];
        }
        std::vector<Scalar> result(output_dim());
        for (int t = 0; t < num_timesteps; ++t) {

#if 1
            // pd control
            if (1) {
                // use PD controller to compute tau
                int qd_offset = system->is_floating() ? 6 : 0;
                int q_offset = system->is_floating() ? 7 : 0;
                int num_targets = system->tau_.size() - qd_offset;
                std::vector<double> q_targets;
                q_targets.resize(system->tau_.size());

                Scalar kp = 150;
                Scalar kd = 3;
                Scalar max_force = 550;
                int param_index = 0;

                for (int i = 0; i < system->tau_.size(); i++) {
                    system->tau_[i] = 0;
                }
                int tau_index = 0;
                int pose_index = 0;
                for (int i = 0; i < system->links_.size(); i++) {
                    if (system->links_[i].joint_type != tds::JOINT_FIXED) {
                        Scalar q_desired = initial_poses[pose_index++];
                        Scalar q_actual = system->q_[q_offset];
                        Scalar qd_actual = system->qd_[qd_offset];
                        Scalar position_error = (q_desired - q_actual);
                        Scalar desired_velocity = 0;
                        Scalar velocity_error = (desired_velocity - qd_actual);
                        Scalar force = kp * position_error + kd * velocity_error;

                        force = Algebra::max(force, -max_force);
                        force = Algebra::min(force, max_force);
                        
                        system->tau_[tau_index] = force;
                        q_offset++;
                        qd_offset++;
                        param_index++;
                        tau_index++;
                    }
                }
            }
#endif
            tds::forward_dynamics(*system, world.get_gravity());
            system->clear_forces();

            integrate_euler_qdd(*system, dt);

            world.step(dt);

            tds::integrate_euler(*system, dt);

            //copy q, qd, link world poses (for rendering) to output
            int j = 0;
            for (int i = 0; i < system->dof(); ++i, ++j) {
                result[j] = system->q(i);
            }
            for (int i = 0; i < system->dof_qd(); ++i, ++j) {
                result[j] = system->qd(i);
            }
            for (const auto link : *system) {
                if (link.X_visuals.size())
                {
                    tds::Transform visual_X_world = link.X_world * link.X_visuals[0];
                    result[j++] = visual_X_world.translation[0];
                    result[j++] = visual_X_world.translation[1];
                    result[j++] = visual_X_world.translation[2];
                    auto orn = Algebra::matrix_to_quat(visual_X_world.rotation);
                    result[j++] = orn.x();
                    result[j++] = orn.y();
                    result[j++] = orn.z();
                    result[j++] = orn.w();
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
 
  UrdfParser<MyAlgebra> parser;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  visualizer.delete_all();

  LaikagoSimulation<MyAlgebra> contact_sim;
  
  int input_dim = contact_sim.input_dim();
  std::vector<MyAlgebra::Scalar> prep_inputs;
  
  std::vector<MyAlgebra::Scalar> prep_outputs;
  prep_inputs.resize(input_dim);
  //quaternion 'w' = 1
  prep_inputs[3] = 1;
  //start height at 0.7
  prep_inputs[6] = 0.7;

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
  tds::FileUtils::find_file(LAIKAGO_URDF_NAME, file_and_path);
  auto urdf_structures = contact_sim.cache.retrieve(file_and_path);// contact_sim.m_urdf_filename);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);
  
  
  int num_total_threads = 10;
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances = 0;
  
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
          contact_sim.system->visual_instance_uids().push_back(instance);
      }
      num_base_instances = num_instances_per_link;

      for (int i = 0; i < contact_sim.system->num_links(); ++i) {
         

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

              contact_sim.system->links_[i].visual_instance_uids.push_back(instance);
          }
          num_instances.push_back(num_instances_per_link);
      }
  }

  //app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, sphereId);

  

  std::vector<std::vector<MyScalar>> parallel_outputs(
      num_total_threads, std::vector<MyScalar>(contact_sim.output_dim()));

  std::vector<std::vector<MyScalar>> parallel_inputs(num_total_threads);

  
  
  for (int i = 0; i < num_total_threads; ++i) {
      //auto quat = MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(0.5,1.3,i*0.3));
      auto quat = MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(0,0,i*0.3));
      parallel_inputs[i] = std::vector<MyScalar>(contact_sim.input_dim(), MyScalar(0));
      parallel_inputs[i][0] = quat.x();
      parallel_inputs[i][1] = quat.y();
      parallel_inputs[i][2] = quat.z();
      parallel_inputs[i][3] = quat.w();
      //start height at 0.7
      parallel_inputs[i][6] = 0.7;
      
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
                  //visualizer.sync_visual_transforms(contact_sim.system);
              }
              else
              {
                  float sim_spacing = 2;
                  const int square_id = (int)std::sqrt((double)num_total_threads);
                  int instance_index = 0;
                  int offset = contact_sim.system->dof() + contact_sim.system->dof_qd();
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
                      
                      for (int l = 0; l < contact_sim.system->links_.size(); l++) {
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
          //std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync* contact_sim.dt));
      }
  }

  printf("finished\n");
  return EXIT_SUCCESS;

}

