#define ARS_VISUALIZE

#define NOMINMAX 
#include <string>

#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include <iostream>
#include <fstream>
#include "shared_noise_table.h"
#include "running_stat.h"
#include "../opengl_urdf_visualizer.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include <thread>
#include <chrono>
#ifndef _WIN32
#include <dlfcn.h>
#endif



using namespace TINY;
using namespace tds;
typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;


#ifdef TRAIN_ANT
//#include "../environments/ant_environment.h"
#include "../environments/ant_environment2.h"
typedef AntEnv2<MyAlgebra> LocomotionEnvironment;
#else
#include "../environments/laikago_environment2.h"
typedef LaikagoEnv<MyAlgebra> LocomotionEnvironment;

#endif //TRAIN_ANT



#ifdef ARS_VISUALIZE
///////////////////////////////////////////
// create graphics
OpenGLUrdfVisualizer<MyAlgebra> visualizer;
bool urdf_from_file = false;


LocomotionEnvironment locomotion_simenv(true);


tds::UrdfStructures<MyAlgebra> urdf_structures;

std::vector<int> visual_instances;
std::vector<int> num_instances;
int num_instances_per_robot=0;
int num_base_instances;

void visualize_trajectories(std::vector<std::vector<std::vector<double>>>& trajectories, int step, bool sleep, int batch_size)
{
     float sim_spacing = 5;
     
     if ((step) % 16 != 0)
         return;

     for (int index=0;index<batch_size;index++)
     {
         std::vector<std::vector<double>>& sim_states_with_graphics = trajectories[index];
         if (sim_states_with_graphics.size()==0)
             continue;
          const int square_id = (int)std::sqrt((double)batch_size);
          int offset = locomotion_simenv.contact_sim.mb_->dof() + locomotion_simenv.contact_sim.mb_->dof_qd();
          int instance_index = index*num_instances_per_robot;
  
#if 0//ndef __APPLE__ //this text rendering is super slow on MacOS
          if (1)
            {
                char msg[1024];
                sprintf(msg, "(%d)", index);
                visualizer.m_opengl_app.draw_text_3d(msg, sim_spacing * (index % square_id) - square_id * sim_spacing / 2, 
                    sim_spacing * (index / square_id) - square_id * sim_spacing / 2, 1, 1);
            }
#endif //  #ifndef __APPLE__


          for (int v=0;v<num_base_instances;v++)
          {
              ::TINY::TinyQuaternionf orn(
                sim_states_with_graphics[step][0],
                sim_states_with_graphics[step][1],
                sim_states_with_graphics[step][2],
                sim_states_with_graphics[step][3]);
            ::TINY::TinyVector3f pos(sim_states_with_graphics[step][4],
                                    sim_states_with_graphics[step][5],
                                    sim_states_with_graphics[step][6]);

             pos[0] += sim_spacing * (index % square_id) - square_id * sim_spacing / 2;
             pos[1] += sim_spacing * (index / square_id) - square_id * sim_spacing / 2;

            int visual_instance_id = visual_instances[instance_index++];
            visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
          }

          int num_visual_links = 0;
          for (int li = 0; li < locomotion_simenv.contact_sim.mb_->links_.size(); li++) 
          {
            //int l = ll-5;
            for (int v = 0; v < num_instances[li]; v++)
            {
                int visual_instance_id = visual_instances[instance_index++];
                if (visual_instance_id >= 0)
                {
                    int l=num_visual_links++;
                    float x = sim_states_with_graphics[step][offset + l * 7 + 3];
                    float y = sim_states_with_graphics[step][offset + l * 7 + 4];
                    float z = sim_states_with_graphics[step][offset + l * 7 + 5];
                    float w = sim_states_with_graphics[step][offset + l * 7 + 6];

                    if (x == 0.f && y == 0.f && z == 0.f && w == 0.f) 
                    {
                    } else
                    {

                        ::TINY::TinyVector3f pos(sim_states_with_graphics[step][offset + l * 7 + 0],
                            sim_states_with_graphics[step][offset + l * 7 + 1],
                            sim_states_with_graphics[step][offset + l * 7 + 2]);
                        ::TINY::TinyQuaternionf orn(
                            sim_states_with_graphics[step][offset + l * 7 + 3],
                            sim_states_with_graphics[step][offset + l * 7 + 4],
                            sim_states_with_graphics[step][offset + l * 7 + 5],
                            sim_states_with_graphics[step][offset + l * 7 + 6]);

                        pos[0] += sim_spacing * (index % square_id) - square_id * sim_spacing / 2;
                        pos[1] += sim_spacing * (index / square_id) - square_id * sim_spacing / 2;

                       
                        visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
                    }
                }
            }
          }
     }

    visualizer.render();
   
    if (sleep)
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(1./60.));//frameskip_gfx_sync* contact_sim.dt));
    }
}
#else
void visualize_trajectory(const std::vector<double>& sim_state_with_graphics, int s)
{
}
///////////////////////////////////////////
#endif

#include "ars_vectorized_environment.h"

#ifdef TRAIN_ANT
typedef VectorizedEnvironment<MyAlgebra, AntContactSimulation2<MyAlgebra>> VecEnvironment;
#else
typedef VectorizedEnvironment<MyAlgebra, LaikagoContactSimulation<MyAlgebra>> VecEnvironment;
#endif


struct PolicyParams
{
};


#include "ars_vectorized_worker.h"

#include "ars_learner.h"

#include "visualizer/opengl/utils/tiny_commandline_args.h"

int main(int argc, char* argv[])
{
    TinyCommandLineArgs args(argc, argv);
    int num_iter = 50*1000;
    int eval_interval = 10;
        
    int batch_size = 128;
    int env_seed = 42;
    bool profile_timings = false;

    args.getCmdLineArgument("batch_size", batch_size);
    args.getCmdLineArgument("num_iter", num_iter);
    args.getCmdLineArgument("eval_interval", eval_interval);
    args.getCmdLineArgument("env_seed", env_seed);
    profile_timings = args.checkCmdLineFlag("profile_timings");
    
    if (profile_timings)
        TinyChromeUtilsStartTimings();

    printf("batch_size = %d\n", batch_size);
    printf("num_iter = %d\n", num_iter);
    printf("eval_interval = %d\n", eval_interval);
    printf("env_seed = %d\n", env_seed);


#ifdef ARS_VISUALIZE
  visualizer.delete_all();
  int input_dim = locomotion_simenv.contact_sim.input_dim();

  
  {
      std::vector<int> shape_ids;
      std::string plane_filename;
      FileUtils::find_file("plane100.obj", plane_filename);
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);
      visualizer.load_obj(plane_filename, pos, orn, scaling, shape_ids);
  }
  
  bool create_instances = false;
  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string texture_path = "";
  std::string file_and_path;
  tds::FileUtils::find_file(locomotion_simenv.contact_sim.urdf_filename_, file_and_path);
  urdf_structures = locomotion_simenv.contact_sim.cache.retrieve(file_and_path);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);
  
  
  for (int t = 0;t< batch_size;t++)
  {
      num_instances_per_robot=0;
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);

#define VISUALIZE_BASE_INSTANCES
#ifdef VISUALIZE_BASE_INSTANCES
      int num_base_instances = 0;
      for (int b=0;b<urdf_structures.base_links.size();b++) {
          for (int bv=0;bv<urdf_structures.base_links[b].urdf_visual_shapes.size();bv++) {
              int uid = urdf_structures.base_links[b].urdf_visual_shapes[bv].visual_shape_uid;
              OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
              int instance = -1;
              
              for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
              {
                  int sphere_shape = vis_link.visual_shape_uids[v];
                  ::TINY::TinyVector3f color(1, 1, 1);
                  //visualizer.m_b2vis
                  instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                      sphere_shape, pos, orn, color, scaling);
                  visual_instances.push_back(instance);
                  num_base_instances++;
                  locomotion_simenv.contact_sim.mb_->visual_instance_uids().push_back(instance);
              }
          }
      }
      num_instances_per_robot += num_base_instances;
#else
      num_base_instances = 0;
#endif
      for (int i = 0; i < locomotion_simenv.contact_sim.mb_->num_links(); ++i) {
         
          int instance = -1;
          int num_instances_per_link = 0;
          for (int lv=0;lv<urdf_structures.links[i].urdf_visual_shapes.size();lv++) {
              int uid = urdf_structures.links[i].urdf_visual_shapes[lv].visual_shape_uid;
              OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
          
              for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
              {
                  int sphere_shape = vis_link.visual_shape_uids[v];
                  ::TINY::TinyVector3f color(1, 1, 1);
                  //visualizer.m_b2vis
                  instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                      sphere_shape, pos, orn, color, scaling,1., false);
                  visual_instances.push_back(instance);
                  num_instances_per_link++;

                  locomotion_simenv.contact_sim.mb_->links_[i].visual_instance_uids.push_back(instance);
              }
          }
          num_instances.push_back(num_instances_per_link);
          num_instances_per_robot+=num_instances_per_link;
      }
  }

  visualizer.m_opengl_app.m_renderer->rebuild_graphics_instances();

#endif //ARS_VISUALIZE

   //srand(123);
  {

    
    VecEnvironment vec_env(locomotion_simenv.contact_sim, batch_size);
    //vec_env.default_stepper_ = &vec_env.serial_stepper_;

    //AntEnv<MyAlgebra> ant_env;
    //Environment env(ant_env.contact_sim_);
    ARSConfig config;
#ifdef TRAIN_ANT
    config.rollout_length_eval_ = 1000;
    config.rollout_length_train_ = 1000;
#else
    config.rollout_length_eval_ = 3000;
    config.rollout_length_train_ = 3000;
#endif

    config.eval_interval = eval_interval;
    config.batch_size = batch_size;
    config.num_iter = num_iter;
    config.env_seed = env_seed;
    ARSLearner<VecEnvironment> ars(vec_env, config);

    ars.train();
  }

      args.getCmdLineArgument("batch_size", batch_size);
    args.getCmdLineArgument("num_iter", num_iter);
    args.getCmdLineArgument("eval_interval", eval_interval);
    args.getCmdLineArgument("env_seed", env_seed);
    args.getCmdLineArgument("profile_timings", profile_timings);


  std::string filename = "ars_train_policy_omp_seed"+std::to_string(env_seed)+"_batch"+std::to_string(batch_size)+std::string(".json");

  if(profile_timings)
    TinyChromeUtilsStopTimingsAndWriteJsonFile(filename.c_str());
}

