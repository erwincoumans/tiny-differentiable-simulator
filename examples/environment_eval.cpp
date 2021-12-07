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


int frameskip_gfx_sync =1;  //use 60Hz, no need for skipping

bool do_sim = true;


TinyKeyboardCallback prev_keyboard_callback = 0;

void my_keyboard_callback(int keycode, int state)
{
    if (keycode == 's')
        do_sim = state;
    prev_keyboard_callback(keycode, state);
}

//#define USE_LAIKAGO
#ifdef USE_LAIKAGO
static MyAlgebra::Vector3 start_pos(0,0,.48);//0.4002847
static MyAlgebra::Quaternion start_orn (0,0,0,1);

static std::string urdf_name = "laikago/laikago_toes_zup.urdf";
static bool is_floating = true;
static double hip_angle = 0.07;//0
static double knee_angle = -0.59;//-0.5;
static double abduction_angle = 0.2;
static std::vector<double> initial_poses = {
    abduction_angle, hip_angle, knee_angle, abduction_angle, hip_angle, knee_angle,
    abduction_angle, hip_angle, knee_angle, abduction_angle, hip_angle, knee_angle,
};
#include "environments/laikago_environment.h"
typedef LaikagoEnv Environment;

#else
#include "environments/cartpole_environment2.h"
typedef CartpoleEnv<MyAlgebra> Environment;
std::vector<double> trained_weights={0.069278,5.483886,4.008912,7.406968,-0.219666};
#endif




int main(int argc, char* argv[]) {

    

  int sync_counter = 0;
  int frame = 0;
  World<MyAlgebra> world;
  UrdfParser<MyAlgebra> parser;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  
  
  
  visualizer.delete_all();

  CartpoleContactSimulation<MyAlgebra> contact_sim;
  
  int input_dim = contact_sim.input_dim();
  

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
  tds::FileUtils::find_file(contact_sim.m_urdf_filename, file_and_path);
  auto urdf_structures = contact_sim.cache.retrieve(file_and_path);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);
  
  
  int num_total_threads = 1;
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

  //CartpoleContactSimulation<MyAlgebra> sim;

  Environment env;
  auto obs = env.reset();
  double total_reward = 0;
  int max_steps = 1000;
  int num_steps = 0;

  int num_params = env.neural_network.num_weights() + env.neural_network.num_biases();
  std::vector<double> x(num_params);
  //rand
  for (int i=0;i<x.size();i++)
  {
      x[i] = -0.35;//*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
  }
  x = trained_weights;

  env.init_neural_network(x);

  while (!visualizer.m_opengl_app.m_window->requested_exit()) {


      {
          //auto action2 = env.policy2(x, obs);
          auto action = env.policy(obs);
          
          double reward;
          bool  done;
          env.step(action,obs,reward,done);
          total_reward += reward;
          num_steps++;
          int num_contacts = 0;
          for (int c=0;c<env.contact_sim.world.mb_contacts_.size();c++)
          {
              for (int j=0;j<env.contact_sim.world.mb_contacts_[c].size();j++)
              {
                if (env.contact_sim.world.mb_contacts_[c][j].distance<0.01)
                {
                    num_contacts++;
                }
              }
          }
          

          if(done || num_steps>=max_steps)
          {
              //printf("total_reward=%f\n",total_reward);
              total_reward=0;
              num_steps = 0;
              obs = env.reset();
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
                      
                      
                      if (contact_sim.mb_->is_floating())
                      {
                          for(int v = 0; v < num_base_instances; v++)
                          {
                              int visual_instance_id = visual_instances[instance_index++];
                              if(visual_instance_id >= 0)
                              {

                                  ::TINY::TinyVector3f pos(env.sim_state_with_graphics[4 + 0],
                                      env.sim_state_with_graphics[4 + 1],
                                      env.sim_state_with_graphics[4 + 2]);
                                  ::TINY::TinyQuaternionf orn(env.sim_state_with_graphics[0],
                                      env.sim_state_with_graphics[1],
                                      env.sim_state_with_graphics[2],
                                      env.sim_state_with_graphics[3]);

                                  pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                                  pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                                  visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos,orn,visual_instance_id);
                              }
                          }
                      }
                      else
                      {
                          instance_index+=num_base_instances;
                      }
                      
                      for (int l = 0; l < contact_sim.mb_->links_.size(); l++) {
                          for (int v = 0; v < num_instances[l]; v++)
                          {
                              int visual_instance_id = visual_instances[instance_index++];
                              if (visual_instance_id >= 0)
                              {

                                  ::TINY::TinyVector3f pos(env.sim_state_with_graphics[offset + l * 7 + 0],
                                      env.sim_state_with_graphics[offset + l * 7 + 1],
                                      env.sim_state_with_graphics[offset + l * 7 + 2]);
                                  ::TINY::TinyQuaternionf orn(
                                      env.sim_state_with_graphics[offset + l * 7 + 3],
                                      env.sim_state_with_graphics[offset + l * 7 + 4],
                                      env.sim_state_with_graphics[offset + l * 7 + 5],
                                      env.sim_state_with_graphics[offset + l * 7 + 6]);

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

   
  printf("finished\n");
  return EXIT_SUCCESS;

}
