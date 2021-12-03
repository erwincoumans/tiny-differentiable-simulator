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
typedef AntEnv<MyAlgebra> LocomotionEnvironment;
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
     

     for (int index=0;index<batch_size;index++)
     {
         std::vector<std::vector<double>>& sim_states_with_graphics = trajectories[index];
         if (sim_states_with_graphics.size()==0)
             continue;
          const int square_id = (int)std::sqrt((double)batch_size);
          int offset = locomotion_simenv.contact_sim.mb_->dof() + locomotion_simenv.contact_sim.mb_->dof_qd();
          int instance_index = index*num_instances_per_robot;
  
#ifndef __APPLE__ //this text rendering is super slow on MacOS
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

#ifdef TRAIN_ANT
typedef VectorizedEnvironment<MyAlgebra, AntContactSimulation<MyAlgebra>> VecEnvironment;
#else
typedef VectorizedEnvironment<MyAlgebra, LaikagoContactSimulation<MyAlgebra>> VecEnvironment;
#endif


struct PolicyParams
{
};


#include "ars_vectorized_worker.h"

#include "ars_learner.h"




struct CudaFunctionMetaData {
  int output_dim;
  int input_dim;
  int global_dim;
};


template <typename FunctionPtrT>
FunctionPtrT load_function(const std::string& function_name, void* lib_handle) {
#if _WIN32
  auto ptr =
      (FunctionPtrT)GetProcAddress((HMODULE)lib_handle, function_name.c_str());
  if (!ptr) {
    throw std::runtime_error("Cannot load symbol '" + function_name +
                             "': error code " + std::to_string(GetLastError()));
  }
#else
  auto ptr = (FunctionPtrT)dlsym(lib_handle, function_name.c_str());
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    throw std::runtime_error("Cannot load symbol '" + function_name +
                             "': " + std::string(dlsym_error));
  }
#endif
  return ptr;
}

template <typename Scalar>
using DefaultCudaFunctionPtrT = void (*)(int, int, int, Scalar*, const Scalar*);
using MetaDataFunctionPtrT = CudaFunctionMetaData (*)();
using AllocateFunctionPtrT = void (*)(int);
using DeallocateFunctionPtrT = void (*)();

template <typename Scalar,
          typename kFunctionPtrT = DefaultCudaFunctionPtrT<Scalar>>
struct CudaFunction {
  using FunctionPtrT = kFunctionPtrT;

  std::string function_name;

  CudaFunctionMetaData meta_data;

  CudaFunction() = default;

  CudaFunction(const std::string& function_name, void* lib_handle)
      : function_name(function_name) {
    fun_ = load_function<FunctionPtrT>(function_name, lib_handle);
    auto meta_data_fun = load_function<MetaDataFunctionPtrT>(
        function_name + "_meta", lib_handle);
    meta_data = meta_data_fun();
    allocate_ = load_function<AllocateFunctionPtrT>(function_name + "_allocate",
                                                    lib_handle);
    deallocate_ = load_function<DeallocateFunctionPtrT>(
        function_name + "_deallocate", lib_handle);
  }

  inline void allocate(int num_total_threads) const {
    allocate_(num_total_threads);
  }

  inline void deallocate() const { deallocate_(); }

  inline bool operator()(int num_total_threads, int num_blocks,
                         int num_threads_per_block, Scalar* output,
                         const Scalar* input) const {
    assert(fun_);
    fun_(num_total_threads, num_blocks, num_threads_per_block, output, input);
    return true;
  }

  inline bool operator()(std::vector<std::vector<Scalar>>* thread_outputs,
                         const std::vector<std::vector<Scalar>>& thread_inputs,
                         int num_threads_per_block = 32,
                         const std::vector<Scalar>& global_input = {}) const {
    assert(fun_);
    if (thread_outputs == nullptr || thread_outputs->empty() ||
        (*thread_outputs)[0].empty()) {
      assert(false);
      return false;
    }
    if (thread_outputs->size() != thread_inputs.size()) {
      assert(false);
      return false;
    }
    if (static_cast<int>(thread_inputs[0].size()) != meta_data.input_dim) {
      assert(false);
      return false;
    }
    if (static_cast<int>((*thread_outputs)[0].size()) != meta_data.output_dim) {
      assert(false);
      return false;
    }
    if (static_cast<int>(global_input.size()) != meta_data.global_dim) {
      assert(false);
      return false;
    }

    auto num_total_threads = static_cast<int>(thread_inputs.size());
    // concatenate thread-wise inputs and global memory into contiguous input
    // array
    Scalar* input = new Scalar[global_input.size() +
                               thread_inputs[0].size() * num_total_threads];
    std::size_t i = 0;
    for (; i < global_input.size(); ++i) {
      input[i] = global_input[i];
    }
    for (const auto& thread : thread_inputs) {
      for (const Scalar& t : thread) {
        input[i] = t;
        ++i;
      }
    }
    Scalar* output =
        new Scalar[(*thread_outputs)[0].size() * num_total_threads];

    int num_blocks = ceil(num_total_threads * 1. / num_threads_per_block);

    // call GPU kernel
    fun_(num_total_threads, num_blocks, num_threads_per_block, output, input);

    // assign thread-wise outputs
    i = 0;
    for (auto& thread : *thread_outputs) {
      for (Scalar& t : thread) {
        t = output[i];
        ++i;
      }
    }

    delete[] input;
    delete[] output;
    return true;
  }

 protected:
  FunctionPtrT fun_{nullptr};
  AllocateFunctionPtrT allocate_{nullptr};
  DeallocateFunctionPtrT deallocate_{nullptr};
};

template <typename Scalar>
struct CudaModel {
 protected:
  const std::string model_name_;
  void* lib_handle_;

  CudaFunctionMetaData meta_data_;

 public:
  CudaFunction<Scalar> forward_zero;

#ifdef _WIN32
  CudaModel(const std::string& model_name) : model_name_(model_name) {
    // load the dynamic library
    std::string path = model_name + ".dll";
    std::string abs_path;
    bool found = tds::FileUtils::find_file(path, abs_path);
    assert(found);
    lib_handle_ = LoadLibrary(abs_path.c_str());
    if (lib_handle_ == nullptr) {
      throw std::runtime_error("Failed to dynamically load library '" +
                               model_name + "': error code " +
                               std::to_string(GetLastError()));
    }
    forward_zero =
        CudaFunction<Scalar>(model_name + "_forward_zero", lib_handle_);
  }
#else
  // loads the shared library
  CudaModel(const std::string& model_name, int dlOpenMode = RTLD_NOW)
      : model_name_(model_name) {
    // load the dynamic library
    std::string path = model_name + ".so";
    std::string abs_path;
    bool found = tds::FileUtils::find_file(path, abs_path);
    assert(found);
    lib_handle_ = dlopen(abs_path.c_str(), dlOpenMode);
    // _dynLibHandle = dlmopen(LM_ID_NEWLM, path.c_str(), RTLD_NOW);
    if (lib_handle_ == nullptr) {
      throw std::runtime_error("Failed to dynamically load library '" +
                               model_name + "': " + std::string(dlerror()));
    }
    forward_zero =
        CudaFunction<Scalar>(model_name + "_forward_zero", lib_handle_);
  }
#endif
};


int main()
{
    int batch_size = 128;

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


    struct CudaStepper : public VecEnvironment::CustomForwardDynamicsStepper {

        CudaModel<MyScalar> cuda_model_;
        int batch_size_;

        CudaStepper(const std::string& model_name, int batch_size)
        :cuda_model_(CudaModel<MyScalar>(model_name)),
            batch_size_(batch_size)
        {
            cuda_model_.forward_zero.allocate(batch_size_);

        }
        virtual ~CudaStepper(){
            cuda_model_.forward_zero.deallocate();
        }

         virtual void step(const std::vector<std::vector<MyAlgebra::Scalar>>& thread_inputs,
                    std::vector<std::vector<MyAlgebra::Scalar>>& thread_outputs,
                    std::vector<bool>& dones,
                    int num_threads_per_block = 32,
                    const std::vector<MyAlgebra::Scalar>& global_input = {}){

            cuda_model_.forward_zero(&thread_outputs, thread_inputs, 64); 
         }

    };


    ARSLearner<VecEnvironment> ars(vec_env, config);
    VecEnvironment::SerialForwardStepper serial_stepper(ars.worker_->env_.contact_sim);
    
    std::string model_name = "cuda_model_"+locomotion_simenv.contact_sim.env_name();

    CudaStepper cuda_stepper(model_name, config.batch_size);

    //ars.worker_->env_.default_stepper_ = &ars.worker_->env_.serial_stepper_;
    ars.worker_->env_.default_stepper_ = &cuda_stepper;

    ars.train();
  }
  
}

