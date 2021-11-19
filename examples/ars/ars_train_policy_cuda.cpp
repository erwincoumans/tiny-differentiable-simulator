#define ARS_VISUALIZE

#define NOMINMAX 
#include <string>

//#define USE_ANT
#ifdef USE_ANT
#include "../environments/ant_environment.h"
#define ContactSimulation AntContactSimulation
#else
#include "../environments/laikago_environment2.h"
#define ContactSimulation LaikagoContactSimulation;
#endif

#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include <iostream>
#include <fstream>
#include "shared_noise_table.h"
#include "running_stat.h"
#include "../opengl_urdf_visualizer.h"
#include <thread>
#include <chrono>
#ifndef _WIN32
#include <dlfcn.h>
#endif
int g_num_total_threads = 128;



using namespace TINY;
using namespace tds;
typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;




#ifdef ARS_VISUALIZE
///////////////////////////////////////////
// create graphics
OpenGLUrdfVisualizer<MyAlgebra> visualizer;
bool urdf_from_file = false;
LocomotionContactSimulation<MyAlgebra> contact_sim(urdf_from_file,
          "laikago/laikago_toes_zup_xyz_xyzrot.urdf",
          laikago_toes_zup_xyz_xyzrot,
          get_initial_poses<MyScalar>(),
          false, 1e-3);
tds::UrdfStructures<MyAlgebra> urdf_structures;

std::vector<int> visual_instances;
std::vector<int> num_instances;
int num_instances_per_robot=0;
int num_base_instances;

void visualize_trajectories(std::vector<std::vector<std::vector<double>>>& trajectories, int step, bool sleep)
{
     float sim_spacing = 5;
     

     for (int index=0;index<g_num_total_threads;index++)
     {
         std::vector<std::vector<double>>& sim_states_with_graphics = trajectories[index];
          const int square_id = (int)std::sqrt((double)g_num_total_threads);
          int offset = contact_sim.mb_->dof() + contact_sim.mb_->dof_qd();
          int instance_index = index*num_instances_per_robot;
  
            {
                char msg[1024];
                sprintf(msg, "(%d)", index);
                visualizer.m_opengl_app.draw_text_3d(msg, sim_spacing * (index % square_id) - square_id * sim_spacing / 2, 
                    sim_spacing * (index / square_id) - square_id * sim_spacing / 2, 1, 1);
            }


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

          for (int l = 0; l < contact_sim.mb_->links_.size(); l++) 
          {
            //int l = ll-5;
            for (int v = 0; v < num_instances[l]; v++)
            {
                int visual_instance_id = visual_instances[instance_index++];
                if (visual_instance_id >= 0)
                {

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
        std::this_thread::sleep_for(std::chrono::duration<double>(1./240.));//frameskip_gfx_sync* contact_sim.dt));
    }
}
#else
void visualize_trajectory(const std::vector<double>& sim_state_with_graphics, int s)
{
}
///////////////////////////////////////////
#endif

#include "ars_vectorized_environment.h"


typedef VectorizedEnvironment<MyAlgebra, LaikagoContactSimulation<MyAlgebra>> Environment;

struct PolicyParams
{
};


#include "ars_vectorized_worker.h"

#include "ars_learner.h"

int main()
{

  
#ifdef ARS_VISUALIZE
  visualizer.delete_all();
  int input_dim = contact_sim.input_dim();

  
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
  tds::FileUtils::find_file(contact_sim.urdf_filename_, file_and_path);
  urdf_structures = contact_sim.cache.retrieve(file_and_path);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);
  
  
  for (int t = 0;t< g_num_total_threads;t++)
  {
      num_instances_per_robot=0;
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);

#define VISUALIZE_BASE_INSTANCES
#ifdef VISUALIZE_BASE_INSTANCES
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
      num_instances_per_robot += num_base_instances;
#else
      num_base_instances = 0;
#endif
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
                  sphere_shape, pos, orn, color, scaling,1., false);
              visual_instances.push_back(instance);
              num_instances_per_link++;

              contact_sim.mb_->links_[i].visual_instance_uids.push_back(instance);
          }
          num_instances.push_back(num_instances_per_link);
          num_instances_per_robot+=num_instances_per_link;
      }
  }

  visualizer.m_opengl_app.m_renderer->rebuild_graphics_instances();

#endif //ARS_VISUALIZE

   //srand(123);
  {

    typedef LaikagoEnv<MyAlgebra> LaikagoEnvironment;
    LaikagoEnvironment laikago_env(false);
    Environment env(laikago_env.contact_sim);
    ARSLearner<Environment>::ARSLearnerConfig config;
    ARSLearner<Environment> ars(env, config);
    ars.train(50*1024*1024);
  }

}

