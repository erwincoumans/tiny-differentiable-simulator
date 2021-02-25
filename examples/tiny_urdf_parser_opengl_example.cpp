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
int frameskip_gfx_sync =1;  //use 60Hz, no need for skipping

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

    int num_timesteps{1};
    Scalar dt{Algebra::from_double(1./60.)};

    int input_dim() const {
        return mb_->dof() + mb_->dof_qd();
    }
    int state_dim() const {
        return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7;
    }
    int output_dim() const { return num_timesteps * state_dim(); }

    ContactSimulation() {
        std::string plane_filename;
        world.set_gravity(Vector3(0.,0.,-10));
        tds::FileUtils::find_file("plane_implicit.urdf",plane_filename);
        cache.construct(plane_filename,world,false,false);
        tds::FileUtils::find_file("cartpole.urdf",m_urdf_filename);
        mb_ = cache.construct(m_urdf_filename,world,false,false);
        mb_->base_X_world().translation = Algebra::unit3_z();
        std::cout << "ContactSimulation!" << std::endl;
    }

    virtual ~ContactSimulation()
    {
        std::cout << "~ContactSimulation" << std::endl;
    }
    std::vector<Scalar> operator()(const std::vector<Scalar>& v,Scalar tau=0.) {
        assert(static_cast<int>(v.size()) == input_dim());
        mb_->initialize();
        //copy input into q, qd
        for(int i = 0; i < mb_->dof(); ++i) {
            mb_->q(i) = v[i];
        }
        for(int i = 0; i < mb_->dof_qd(); ++i) {
            mb_->qd(i) = v[i + mb_->dof()];
        }

        static double t=0;
        //printf("t=%f, [%f,%f]\n",t,mb_->q(0),mb_->q(1));
        t+=dt;
        std::vector<Scalar> result(output_dim());
        for(int t = 0; t < num_timesteps; ++t) {


            mb_->tau_[0] = tau;
            mb_->tau_[1] = 0;


            //std::vector<Scalar> tau = policy(observation)

            tds::forward_dynamics(*mb_,world.get_gravity());
            mb_->clear_forces();

            integrate_euler_qdd(*mb_,dt);

            world.step(dt);

            tds::integrate_euler(*mb_,dt);



            //copy q, qd, link world poses (for rendering) to output
            int j = 0;
            for(int i = 0; i < mb_->dof(); ++i,++j) {
                result[j] = mb_->q(i);
            }
            for(int i = 0; i < mb_->dof_qd(); ++i,++j) {
                result[j] = mb_->qd(i);
            }
            for(const auto link : *mb_) {
                if(link.X_visuals.size())
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

struct CartpoleEnv
{
    ContactSimulation<MyAlgebra>& contact_sim;

    CartpoleEnv(ContactSimulation<MyAlgebra>& cartpole)
        :contact_sim(cartpole)
    {
        //std::cout << "CartpoleEnv!\n" << std::endl;
    }
    virtual ~CartpoleEnv()
    {
        //std::cout << "~CartpoleEnv\n" << std::endl;
    }

    std::vector<MyScalar> sim_state;
    std::vector<MyScalar> sim_state_with_graphics;

    std::vector<double> reset()
    {
        sim_state.resize(contact_sim.input_dim());
        for(int i=0;i<sim_state.size();i++)
        {
            sim_state[i] = 0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
            
            
        }
        for (auto v : sim_state)
            std::cout << v << std::endl;
        return sim_state;
    }
    void step(double action,std::vector<double>& obs,double& reward,bool& done)
    {
        //sim_state = [q0, q1, qd0, qd1]


        sim_state_with_graphics = contact_sim(sim_state,action);
        sim_state = sim_state_with_graphics;

        sim_state.resize(contact_sim.input_dim());
        obs = sim_state;
        reward = 1;
        double x = sim_state[0];
        double theta = sim_state[1];

        double theta_threshold_radians = 12. * 2. * M_PI / 360.;
        double x_threshold = 0.4;//  #2.4
        done =  (x < -x_threshold)
            || (x > x_threshold)
            || (theta < -theta_threshold_radians)
            || (theta > theta_threshold_radians);
    }
};


void testEnv()
{

}

ContactSimulation<MyAlgebra> m_cartpole_sim;


double policy(const std::vector<double> &x,const std::vector<double>& obs)
{
    double action = 0;

    for(int i=0;i<4;i++)
    {
        action+=x[i]*obs[i];
    }
    action+=x[4];
    if(action<-1)
        action=-1;
    if(action>1)
        action=1;
    return action;
}



int main(int argc, char* argv[]) {

    

  int sync_counter = 0;
  int frame = 0;
  World<MyAlgebra> world;
  UrdfParser<MyAlgebra> parser;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  
  
  
  visualizer.delete_all();

  ContactSimulation<MyAlgebra> contact_sim;
  
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
  tds::FileUtils::find_file("cartpole.urdf", file_and_path);
  auto urdf_structures = contact_sim.cache.retrieve(file_and_path);// contact_sim.m_urdf_filename);
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

  

  CartpoleEnv env(m_cartpole_sim);
  auto obs = env.reset();
  double total_reward = 0;
  int max_steps = 200;
  int num_steps = 0;

  //std::vector<double> x={6.038696,17.109466,2.287317,4.744410,0.859432};//trained on [-0.05. 0.05]
  //std::vector<double> x={262.931804,1265.481361,490.198848,497.031251,-22.723601};//89.645820,489.210318,181.857783,198.405684,8.169060
  std::vector<double> x={123.029492,1225.629406,339.628481,325.638617,-16.230902};//trained on [-0.27,0.27] instead of [-0.05. 0.05]
  
  while (!visualizer.m_opengl_app.m_window->requested_exit()) {


      {
          double action = 10.*policy(x,obs);
          double reward;
          bool  done;
          env.step(action,obs,reward,done);
          total_reward += reward;
          num_steps++;
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

