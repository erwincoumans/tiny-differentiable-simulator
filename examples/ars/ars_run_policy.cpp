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
#include "../opengl_urdf_visualizer.h"

#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#include "urdf/urdf_cache.hpp"
#include "../tiny_visual_instance_generator.h"

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
#define USE_ANT
#ifdef USE_ANT





#include "../environments/ant_environment.h"
typedef AntEnv<MyAlgebra> Environment;
typedef AntContactSimulation<MyAlgebra> RobotSim;
#else//USE_ANT
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
#include "../environments/laikago_environment.h"
typedef LaikagoEnv Environment;
typedef ContactSimulation<MyAlgebra> RobotSim;
#else
#include "../environments/cartpole_environment.h"
typedef CartpoleEnv<MyAlgebra> Environment;
//std::vector<double> trained_weights={0.069278,5.483886,4.008912,7.406968,-0.219666};
//std::vector<double> trained_weights={0.15964059, 1.78998116, 0.79687186, 1.80107264, 0.01240305};
std::vector<double> trained_weights={0.820749,4.480032,4.589206,5.880079,0.204528};
typedef CartpoleContactSimulation<MyAlgebra> RobotSim;
                                     
#endif//USE_LAIKAGO
#endif//USE_ANT



int main(int argc, char* argv[]) {

    

  int sync_counter = 0;
  int frame = 0;
  World<MyAlgebra> world;
  UrdfParser<MyAlgebra> parser;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  
  
  
  visualizer.delete_all();

  RobotSim contact_sim;
  
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
#if 0
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
#else
      num_base_instances = 0;
#endif
      for (int i = 5; i < contact_sim.mb_->num_links(); ++i) {
         

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

  RobotSim sim;

  Environment env(sim);
  auto obs = env.reset();
  double total_reward = 0;
  int max_steps = 1000;
  int num_steps = 0;

  int num_params = env.neural_network.num_weights() + env.neural_network.num_biases();
  //std::vector<double> x(num_params);
  //rand
  //for (int i=0;i<x.size();i++)
  //{
  //    x[i] = -.35*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
  //}

  //weights trained using c++ ars_train_policy (without observation filter)
  std::vector<double> x = 
{-0.330694,0.100664,-0.050100,0.010913,-0.076245,-0.096708,-0.052048,-0.022072,-0.071742,-0.141120,-0.001460,-0.091223,-0.080160,-0.143914,-0.049341,0.009960,0.073633,0.094207,-0.104590,0.126970,-0.111545,-0.205443,0.164077,0.366699,0.039011,0.047922,0.181835,-0.041931,0.081036,-0.017474,-0.046704,-0.107542,0.009476,0.275584,0.076891,0.027139,0.077219,-0.030522,-0.036556,0.096170,0.044257,0.065861,-0.069107,0.048704,-0.065532,0.101776,0.057067,-0.010219,0.001186,-0.051902,-0.082907,-0.091851,-0.114649,0.055788,-0.103413,-0.334057,-0.047808,0.088291,-0.258178,0.020027,-0.021699,0.064720,0.114032,-0.088063,0.061418,-0.123903,-0.019698,-0.214701,0.057364,0.052798,0.105946,0.013900,-0.124420,0.036956,-0.088813,-0.280787,0.110632,-0.097571,0.042996,0.138702,0.114211,0.040666,0.152882,-0.038852,0.132004,0.159409,0.248187,0.144521,0.133852,-0.098142,0.014450,0.022485,0.044374,-0.175090,0.039559,0.058260,0.178692,-0.140327,-0.019914,0.088317,0.038902,-0.011281,0.050123,-0.107991,-0.038676,-0.084500,0.050126,0.033527,0.202266,-0.070085,0.046814,0.044315,0.082971,0.028167,-0.127337,-0.118247,0.088257,0.028016,-0.031211,0.057919,0.189745,0.213504,0.011288,0.050855,0.012990,0.018937,0.004396,-0.016475,-0.138815,0.086511,-0.140102,-0.106045,0.135993,-0.135477,-0.019576,0.046984,-0.115611,-0.198333,0.094186,0.137661,0.049339,0.077401,-0.162229,0.022853,0.146213,-0.096798,0.136300,-0.024680,0.144730,0.188925,0.025122,0.032732,-0.021309,-0.042702,-0.026105,-0.127947,-0.072268,0.279763,0.010303,0.042321,0.068728,0.037936,-0.061419,-0.202066,-0.128971,-0.127879,-0.048124,-0.037882,0.187363,-0.014169,0.009729,-0.032163,-0.122680,-0.007935,0.122371,0.195222,-0.031082,-0.142409,-0.047097,-0.179822,-0.194071,0.068477,-0.024994,-0.067355,-0.000895,0.060127,0.160594,0.138485,0.155649,-0.077018,-0.035986,0.098026,0.105008,-0.234295,0.107317,0.082829,0.062446,0.084337,0.090816,-0.055368,0.050477,0.054706,-0.146131,-0.150025,0.024232,-0.084176,-0.045031,-0.001584,-0.080642,-0.162535,-0.219500,-0.148941,0.024525,-0.058704,0.126188,-0.152128,0.075867,-0.104105,0.188959,0.049782,-0.173684,-0.041281,0.060444,-0.255056,-0.030986,-0.055905,0.038094,-0.049954,-0.124161,0.230230,0.202641,-0.018996};

  env.init_neural_network(x);

  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
    {
          auto action = env.policy(obs);
          //std::cout << "state= [" << obs[0] << ", " << obs[1] << ", " << obs[2] << ", " << obs[3] << "]" << std::endl;
          
          double reward;
          bool  done;
          env.step(action,obs,reward,done);
          total_reward += reward;
          num_steps++;
          int num_contacts = 0;
          for (int c=0;c<env.contact_sim_.world.mb_contacts_.size();c++)
          {
              for (int j=0;j<env.contact_sim_.world.mb_contacts_[c].size();j++)
              {
                if (env.contact_sim_.world.mb_contacts_[c][j].distance<0.01)
                {
                    num_contacts++;
                }
              }
          }
          

          if(done || num_steps>=max_steps)
          {
              printf("total_reward=%f\n",total_reward);
              total_reward=0;
              num_steps = 0;
              obs = env.reset();
          }
      }

      
      sync_counter++;
      frame += 1;
      //if (sync_counter > frameskip_gfx_sync) 
      {
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
                      
                      for (int ll = 5; ll < contact_sim.mb_->links_.size(); ll++) {
                          int l = ll-5;
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
