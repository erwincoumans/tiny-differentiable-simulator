#define ARS_VISUALIZE

#define NOMINMAX 
#include <string>


//#define USE_ANT
#ifdef USE_ANT
#include "../environments/ant_environment.h"
#define ContactSimulation AntContactSimulation
std::string model_name = "cuda_model_ant";
#define ContactSimulation AntContactSimulation
#else
#include "../environments/laikago_environment.h"
//#define ant_initial_poses initial_poses_laikago2
std::string model_name = "cuda_model_laikago";
#define ContactSimulation LaikagoContactSimulation;

//#include "../environments/cartpole_environment.h"

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


//struct CudaFunctionMetaData {
//  int output_dim;
//  int local_input_dim;
//  int global_input_dim;
//  bool accumulated_output;
//};

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


using namespace TINY;
using namespace tds;
typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

//typedef CartpoleEnv<MyAlgebra> Environment;
//typedef AntEnv<MyAlgebra> Environment;



struct AntVecEnvOutput
{
    std::vector<double> obs;
    double reward;
    bool done;
};

struct AntVecRolloutOutput
{
    AntVecRolloutOutput()
        :total_reward(0),
        num_steps(0)
    {
    }
    double total_reward;
    int num_steps;
};



template <typename Algebra>
struct AntVecEnv
{
    LaikagoContactSimulation<Algebra> contact_sim;
    using Scalar = typename Algebra::Scalar;
    using Vector3 = typename Algebra::Vector3;
    using Transform = typename Algebra::Transform;

    
    std::vector<std::vector<Scalar>> sim_states_;
    std::vector<std::vector<Scalar>> sim_states_with_action_;
    std::vector<std::vector<Scalar>> sim_states_with_graphics_;

    std::vector<tds::NeuralNetwork<Algebra> > neural_networks_;
    int observation_dim_{0};


    AntVecEnv()
        :contact_sim(false)
    {
        neural_networks_.resize(g_num_total_threads);
        sim_states_.resize(g_num_total_threads);
        sim_states_with_action_.resize(g_num_total_threads);
        sim_states_with_graphics_.resize(g_num_total_threads);

        observation_dim_ = contact_sim.input_dim();
        bool use_input_bias = false;
        for (int index=0;index<g_num_total_threads;index++)
        {
            neural_networks_[index].set_input_dim(observation_dim_, use_input_bias);
            //network.add_linear_layer(tds::NN_ACT_RELU, 32);
            //neural_network.add_linear_layer(tds::NN_ACT_RELU, 64);
            bool learn_bias = true;
            neural_networks_[index].add_linear_layer(tds::NN_ACT_IDENTITY, initial_poses_laikago2.size(),learn_bias);
        }
        
    }
    virtual ~AntVecEnv()
    {
    }

    void init_neural_network(int index, const std::vector<double> &x)
    {
        neural_networks_[index].set_parameters(x);
    }


    void seed(long long int s) {
      //std::cout<<"seed:" << s << std::endl;
      std::srand(s);
    }
    
    

    std::vector< std::vector<double> > reset(CudaModel<MyScalar>& cuda_model_ant)
    {
        for (int index=0;index<g_num_total_threads;index++)
        {
            sim_states_[index].resize(0);
            sim_states_[index].resize(contact_sim.input_dim(), Scalar(0));
            MyAlgebra::Vector3 start_pos(0,0,.48);//0.4002847
            MyAlgebra::Quaternion start_orn (0,0,0,1);

            if (contact_sim.mb_->is_floating())
            {
            
                sim_states_[index][0] = start_orn.x();
                sim_states_[index][1] = start_orn.y();
                sim_states_[index][2] = start_orn.z();
                sim_states_[index][3] = start_orn.w();
                sim_states_[index][4] = start_pos.x();
                sim_states_[index][5] = start_pos.y();
                sim_states_[index][6] = start_pos.z();
                int qoffset = 7;
                for(int j=0;j<initial_poses_laikago2.size();j++)
                {
                    sim_states_[index][j+qoffset] = initial_poses_laikago2[j]+0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
                }
            }
            else
            {
                sim_states_[index][0] = start_pos.x();
                sim_states_[index][1] = start_pos.y();
                sim_states_[index][2] = start_pos.z();
                sim_states_[index][3] = 0;
                sim_states_[index][4] = 0;
                sim_states_[index][5] = 0;
                int qoffset = 6;
                for(int j=0;j<initial_poses_laikago2.size();j++)
                {
                    sim_states_[index][j+qoffset] = initial_poses_laikago2[j]+0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
                }

            }
        }
        std::vector< std::vector<double>>  zero_actions(g_num_total_threads);
        for (int i=0;i<g_num_total_threads;i++)
        {
            zero_actions[i].resize(initial_poses_laikago2.size(), Scalar(0));
        }
        std::vector< std::vector<double>> observations;
        observations.resize(g_num_total_threads);

        std::vector<double> rewards;
        rewards.resize(g_num_total_threads);

        std::vector<bool> dones;
        dones.resize(g_num_total_threads);

        //todo: tune this
        int settle_down_steps= 50;
        for (int i=0;i<settle_down_steps;i++)
        {
            step(cuda_model_ant, zero_actions, observations, rewards, dones);
        }
        
        //for (auto v : sim_state)
        //    std::cout << v << std::endl;
        return observations;
    }
    void step(CudaModel<MyScalar>& cuda_model_ant, std::vector< std::vector<double>>& actions,std::vector<std::vector<double>>& observations,
        std::vector<double>& rewards,std::vector<bool>& dones)
    {

        std::vector<std::vector<Scalar>> outputs(
        g_num_total_threads, std::vector<Scalar>(contact_sim.output_dim()));

        std::vector<std::vector<Scalar>> inputs(g_num_total_threads);


        for (int index=0;index<g_num_total_threads;index++)
        {
            int simstate_size = sim_states_[index].size();
            sim_states_with_action_[index] = sim_states_[index];
            sim_states_with_action_[index].resize(simstate_size+actions[index].size());

            for (int i=0;i<actions[index].size();i++)
            {
                sim_states_with_action_[index][i+simstate_size] = actions[index][i];
            }
            inputs[index] = sim_states_with_action_[index];
        }
    

//#define DEBUG_ON_CPU
#ifdef DEBUG_ON_CPU
        
        
        for (int index =0; index<g_num_total_threads;index++)
        {
            sim_states_with_graphics_[index] = contact_sim(sim_states_with_action_[index]);
        }
#else
        cuda_model_ant.forward_zero(&outputs, inputs,64);
        for (int index=0;index<g_num_total_threads;index++)
        {
            sim_states_with_graphics_[index] = outputs[index];
        }
#endif
        for (int index=0;index<g_num_total_threads;index++)
        {
            if (!dones[index])
            {
                sim_states_[index] = sim_states_with_graphics_[index];

                sim_states_[index].resize(contact_sim.input_dim());

                observations[index] = sim_states_[index];
                
        
               
            
#ifdef USE_ANT
                //reward forward along x-axis
                rewards[index] = sim_states_[index][0];
#else
                double laikago_x = sim_states_[index][4];
                double laikago_z = sim_states_[index][6];
                //reward forward along x-axis, minus angles to keep chassis straight
                rewards[index] = laikago_x;
#endif
                static double max_reward = -1e30;
                static double min_reward = 1e30;
                if (rewards[index] < min_reward)
                {
                    min_reward = rewards[index];
                    //printf("min_reward = %f\n",min_reward);
                }
                if (rewards[index] > max_reward)
                {
                    max_reward = rewards[index];
                    //printf("max_reward = %f\n",max_reward);
                }
            

                //Ant height needs to stay above 0.25a
#ifdef USE_ANT
                if (sim_states_[index][2] < 0.26)
#else

                typename Algebra::Quaternion base_orn(sim_states_[index][0],sim_states_[index][1],sim_states_[index][2],sim_states_[index][3]);
                auto base_mat = Algebra::quat_to_matrix(base_orn);
                Scalar up_dot_world_z = base_mat(2, 2);
    
                // terminate if Laikago position/orientation becomes invalid
                if (up_dot_world_z < 0.6 || ( laikago_z< 0.2))
#endif
                {
                    dones[index] =  true;
                }  else
                {
                    dones[index] = false;
                }
            } else
            {
                rewards[index] = 0;
            }
        }
    }

    inline const std::vector<double> policy(int index, const std::vector<double>& obs)
    {
        std::vector<double> action (initial_poses_laikago2.size(), Scalar(0));
    
        neural_networks_[index].compute(obs, action);
                
        return action;
    }

};

typedef AntVecEnv<MyAlgebra> Environment;

struct PolicyParams
{
};



#ifdef ARS_VISUALIZE
///////////////////////////////////////////
// create graphics
OpenGLUrdfVisualizer<MyAlgebra> visualizer;
LaikagoContactSimulation<MyAlgebra> contact_sim(false);
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

//Object class for parallel rollout generation.
struct Worker
{
    PolicyParams m_policy_params_;
    int rollout_length_eval_;
    int rollout_length_train_;
    double delta_std_;
    
    
    Environment env_;

    SharedNoiseTable noise_table_;

    std::vector<std::vector<RunningStat>> observation_filters_;
    std::vector<std::vector<double> > observation_means_;
    std::vector<std::vector<double> > observation_stds_;

    Worker(int env_seed, int params_dim, const PolicyParams& policy_params, const std::vector<double>& deltas, int rollout_length_train, int rollout_length_eval, double delta_std)
        :m_policy_params_(policy_params),
        rollout_length_train_ (rollout_length_train),
        rollout_length_eval_ (rollout_length_eval),
        delta_std_(delta_std),
        noise_table_(deltas, env_seed, env_.neural_networks_[0].num_parameters())
    {
        env_.seed(env_seed);

        observation_filters_.resize(g_num_total_threads);
        observation_means_.resize(g_num_total_threads);
        observation_stds_.resize(g_num_total_threads);

        for (int i=0;i<g_num_total_threads;i++)
        {
            observation_filters_[i].resize(env_.observation_dim_);
        }
    }
        
    virtual ~Worker()
    {
    }
    //Get current policy weights and current statistics of past states.
    void get_weights_plus_stats(std::vector<double>& weights) {
        //return self.policy.get_weights_plus_stats()
    }
    
    //Performs parallel rollouts of maximum length rollout_length. 
    //At each time-step it substracts shift from the reward.
    void rollouts(CudaModel<MyScalar>& cuda_model_ant, double shift, int rollout_length, std::vector<double>& total_rewards, std::vector<int>& vec_steps, std::vector< std::vector<std::vector<double> > >& trajectories)
    {

        

        if (rollout_length == 0)
        {
            printf("rollout_length =%d\n",rollout_length);
        }
        //if (rollout_length == 0) {
        //    rollout_length = rollout_length_;
        //}

        auto observations = env_.reset(cuda_model_ant);
        
        for (int index=0;index<g_num_total_threads;index++)
        {
            vec_steps[index] = 0;
            total_rewards[index] = 0.;
        }
        std::vector<double> rewards;
        rewards.resize(g_num_total_threads);
        std::vector<bool>  dones;
        dones.resize(g_num_total_threads);

        std::vector<std::vector<double> > actions(g_num_total_threads);
        
          
       for (int r =0;r<rollout_length;r++) 
       {
           for (int index=0;index<g_num_total_threads;index++)
           {
                actions[index] = env_.policy(index, observations[index]);
           }

           for (int index=0;index<g_num_total_threads;index++)
           {
                //update running stat for the observation filter
                for (int o=0;o<observations[index].size();o++)
                {
                    observation_filters_[index][o].Push(observations[index][o]);

                    if (observation_means_[index].size())
                    {
                        //apply filter
                        observations[index][o] -= observation_means_[index][o];
                        if (observation_stds_[index][o]>1e-7)
                        {
                            observations[index][o] /= observation_stds_[index][o];
                        }
                    }
                }
           }

           env_.step(cuda_model_ant, actions,observations,rewards,dones);

           for (int index=0;index<g_num_total_threads;index++)
           {
               if (dones[index])
               {
                   int sz = trajectories[index].size();
                   if (sz)
                   {
                       const auto& prev = trajectories[index][sz-1];
                        trajectories[index].push_back(prev);
                   }
               } else
               {
                    trajectories[index].push_back(env_.sim_states_with_graphics_[index]);
                    total_rewards[index] += (rewards[index] - shift);
                    vec_steps[index]++;
               }
               
                
           }
        }
    }

    double std_deviation(const std::vector<double>& vec)
    {
        double std_dev = 0;
        if (vec.size()>1)
        {
            double sum=0;
            for( int n = 0; n < vec.size(); n++ )
            {
                sum += vec[n];
            }
            double mean = sum / vec.size();
            for( int n = 0; n < vec.size(); n++ )
            {
                double v = (vec[n] - mean) ;
                std_dev += v*v;
            }
            std_dev /= vec.size(); // -1 for unbiased estimator
        }
        return sqrt(std_dev);
    }


    //Generate multiple rollouts with a policy parametrized by w_policy.
    void do_rollouts(CudaModel<MyScalar>& cuda_model_ant, std::vector<double>& rollout_rewards, std::vector<int>& deltas_idx, int& steps, 
        const std::vector<double>& w_policy, int num_rollouts, int shift, bool evaluate,
        std::vector<std::vector<std::vector<double> > >& trajectories) {

        steps = 0;
        
        std::vector<double> tmp;
        trajectories.resize(g_num_total_threads);

        bool use_std_deviation = false;//!evaluate;

        if (evaluate) {
                
            //policy.update_weights(w_policy);
            for (int index=0;index<g_num_total_threads;index++)
            {
                env_.init_neural_network(index, w_policy);
                deltas_idx.push_back(-1);
            }
                
            //set to false so that evaluation rollouts are not used for updating state statistics
            //policy.update_filter = false;

            //for evaluation we do not shift the rewards (shift = 0) and we use the default rollout_length
            //double reward, int r_steps = rollout(shift = 0., rollout_length = self.rollout_length);
            std::vector<double> rewards;
            rewards.resize(g_num_total_threads);
            std::vector<int> vec_r_steps;
            vec_r_steps.resize(g_num_total_threads);

            rollouts(cuda_model_ant, 0, rollout_length_eval_, rewards, vec_r_steps, trajectories);
            
            for (int step=0;step< trajectories[0].size();step++)
            {
                visualize_trajectories(trajectories, step, false);
            }
            for (int index=0;index<g_num_total_threads;index++)
            {
                steps += vec_r_steps[index];
                rollout_rewards.push_back(rewards[index]);
            }
        }
        else {
            std::vector<std::vector<double> > deltas;
            deltas.resize(g_num_total_threads);

            //idx, delta = deltas.get_delta(w_policy.size)

            for (int index=0;index<g_num_total_threads;index++)
            {
                int delta_idx = noise_table_.get_delta(delta_std_, deltas[index]);
                //delta = (self.delta_std * delta).reshape(w_policy.shape)
                deltas_idx.push_back(delta_idx);

                //# set to true so that state statistics are updated 
                //self.policy.update_filter = True

                //# compute reward and number of timesteps used for positive perturbation rollout
                std::vector<double> weights;
                weights.resize(w_policy.size());
                for (int i=0;i<w_policy.size();i++)
                {
                    weights[i] = w_policy[i]+deltas[index][i];
                }
                env_.init_neural_network(index, weights);
            }

            //self.policy.update_weights(w_policy + delta)
            //env.update_weights(weights)
            //pos_reward, pos_steps  = self.rollout(shift = shift)
            std::vector<double> pos_rewards;
            pos_rewards.resize(g_num_total_threads);
            std::vector<int> vec_pos_steps;
            vec_pos_steps.resize(g_num_total_threads);
            
            rollouts(cuda_model_ant, shift, rollout_length_train_, pos_rewards, vec_pos_steps, trajectories);
            
            //for (int step=0;step< trajectories[0].size();step++)
            //{
            //    visualize_trajectories(trajectories, step, false);
            //}
                        
            for (int index=0;index<g_num_total_threads;index++)
            {
                std::vector<double> weights;
                weights.resize(w_policy.size());
                //compute reward and number of timesteps used for negative pertubation rollout
                for (int i=0;i<w_policy.size();i++)
                {
                    weights[i] = w_policy[i]-deltas[index][i];
                }
                env_.init_neural_network(index, weights);
                trajectories[index].resize(0);
            }
            std::vector<double> neg_rewards;
            neg_rewards.resize(g_num_total_threads);
            std::vector<int> vec_neg_steps;
            vec_neg_steps.resize(g_num_total_threads);

            rollouts(cuda_model_ant, shift, rollout_length_train_, neg_rewards, vec_neg_steps,trajectories);

            //for (int step=0;step<trajectories[0].size();step++)
            //{
            //    visualize_trajectories(trajectories, step,false);
            //}

            for (int index=0;index<g_num_total_threads;index++)
            {
                steps += vec_pos_steps[index] + vec_neg_steps[index];
                //rollout_rewards[:,0] - rollout_rewards[:,1]
                if (use_std_deviation)
                {
                    tmp.push_back(pos_rewards[index]);
                    tmp.push_back(neg_rewards[index]);
                } else
                {
                    rollout_rewards.push_back(pos_rewards[index]-neg_rewards[index]);
                }
            }
        }
        
        if (use_std_deviation)
        {
            double std_dev = std_deviation(tmp);
            double inv_std_dev = 1;
            if (std_dev>1e-6)
                inv_std_dev = 1./std_dev;
            for (int i=0;i<tmp.size();i+=2)
            {
                double pos = tmp[i]*inv_std_dev;
                double neg = tmp[i+1]*inv_std_dev;
                double pmn = pos-neg;
                rollout_rewards.push_back(pmn);
            }
        }
    }
 
};



struct ARSLearner 
{
    std::vector<double> w_policy;

    int total_timesteps{0};
    int num_deltas_{g_num_total_threads};
    int shift_{0};
    
    std::vector<double> deltas_;

    
    Worker* worker_{0};
    int rollout_length_train_{3000};
    int rollout_length_eval_{4000};
    double delta_std_{0.03};
    double sgd_step_size { 0.02};

    std::ofstream myfile_;

    std::chrono::steady_clock::time_point time_point_;
    CudaModel<MyScalar>& cuda_model_ant_;

    ARSLearner(CudaModel<MyScalar>& cuda_model_ant)
        :cuda_model_ant_(cuda_model_ant)
    {
        init_deltas();

        int env_seed=12345;//421;
        
        PolicyParams policy_params;
        
        
        Environment tmpenv;
        int params_dim = tmpenv.neural_networks_[0].num_parameters();
        
        //does it have to be random to start? deltas will take care of it
        w_policy.resize(params_dim);
        
        worker_ = new Worker (env_seed, params_dim, policy_params, deltas_, rollout_length_train_, rollout_length_eval_, delta_std_);
        
        myfile_.open ("ars_cpp_log.txt");
        myfile_ << "Time	Iteration	AverageReward	MaxRewardRollout	MinRewardRollout	timesteps" << std::endl;

        time_point_ = std::chrono::steady_clock::now();

    }

    virtual ~ARSLearner()
    {
        delete worker_;
        myfile_.close();
    }

    //note: this get_deltas doesn't multiply with delta_std!
    void get_deltas(int start_index, std::vector<double>& delta)
    {
        delta.resize(w_policy.size());
        for (int i=0;i<delta.size();i++)
        {
            delta[i] = deltas_[start_index+i];
        }
    }

    void weighted_sum_custom(const std::vector<double>& weights, const std::vector<int>& deltas_idx,
        std::vector<double>& g_hat, int& num_items_summed ) {
        assert(deltas_idx.size()>=1);
        std::vector<double> tmp_delta;
        get_deltas(deltas_idx[0], tmp_delta);
        g_hat.resize(tmp_delta.size());
        num_items_summed = 0;
      
        for (int i=0;i< weights.size();i++)
        {
            get_deltas(deltas_idx[i], tmp_delta);
            double w = weights[i];
            for (int j =0; j< tmp_delta.size();j++)
            {
                g_hat[j] += w * tmp_delta[j]*delta_std_;//either multiply with delta_std_ or enable use_std_deviation
            }
          num_items_summed+=1;
        }
         //g_hat /= deltas_idx.size
        for (int i=0;i<  g_hat.size();i++)
        {
            g_hat[i] /= deltas_idx.size();
        }
    }
    

    void init_deltas()
    {
        //std::random_device rd;
        std::seed_seq seed3;
        std::mt19937 gen(seed3);
        //std::uniform_real_distribution<> dist(0, 10);
        double mean=0.0, sigma=1.0;
        //https://numpy.org/doc/1.16/reference/generated/numpy.random.RandomState.randn.html#numpy.random.RandomState.randn
        std::normal_distribution<double> distribution(mean, sigma);
        
        //what size is reasonable?
        int count=25000000;
        //int count=2500000;
        deltas_.resize(count);
        printf("Creating a normal distribution of size %d.\n", count);
        for (int i=0;i<count;i++)
        {
            deltas_[i] = distribution(gen);
        }
        printf("Finished creating a normal distribution.\n");
    }

    //Aggregate update step from rollouts generated in parallel.
    std::vector<double> aggregate_rollouts(int num_rollouts, bool evaluate, std::vector<std::vector<std::vector<double> > >& trajectories) {
          
        int num_deltas = (num_rollouts == 0)? num_deltas_ : num_rollouts;
            
        //# put policy weights in the object store
        //policy_id = ray.put(self.w_policy)

        //t1 = time.time()
        //num_rollouts = int(num_deltas / num_workers);
        num_rollouts = num_deltas; //assume 1 worker for a start

        std::vector<double> rollout_rewards;
        std::vector<int> deltas_idx;
        int steps;
        worker_->do_rollouts(cuda_model_ant_, rollout_rewards, deltas_idx, steps, w_policy, num_rollouts, shift_, evaluate, trajectories);
        
        total_timesteps += steps;


        if (evaluate)
        {
            printf("eval steps = %d\n", steps);
            return rollout_rewards;
        }

        //# select top performing directions if deltas_used < num_deltas
        //max_rewards = np.max(rollout_rewards, axis = 1)
        //if self.deltas_used > self.num_deltas:
        //    self.deltas_used = self.num_deltas
            
        //idx = np.arange(max_rewards.size)[max_rewards >= np.percentile(max_rewards, 100*(1 - (self.deltas_used / self.num_deltas)))]
        //deltas_idx = deltas_idx[idx]
        //rollout_rewards = rollout_rewards[idx,:]
        
       
        std::vector<double> g_hat;
        int num_items_summed=0;

        weighted_sum_custom(rollout_rewards, deltas_idx, g_hat, num_items_summed);

        return g_hat;

    }
    void train_step() {
        int num_rollouts = 0;
        bool evaluate = false;
        std::vector<std::vector<std::vector<double> > > trajectories;
        auto g_hat = aggregate_rollouts(num_rollouts, evaluate,trajectories);
#if 0
        if (g_hat.size())
        {
            double sum=0;
            for (int i=0;i<g_hat.size();i++)
            {
                sum += g_hat[i]*g_hat[i];
            }
            double norm = sqrt(sum);
            printf("Euclidean norm of update step:%f\n",norm);
        }
#endif

        //w_policy -= self.optimizer._compute_step(g_hat).reshape(self.w_policy.shape);
        for (int i=0;i<w_policy.size();i++) {
            w_policy[i] += sgd_step_size * g_hat[i];
        }
    }


    void train(int num_iter)
    {
        double best_mean_rewards = -1e30;
        
        for (int iter=0;iter< num_iter;iter++) {
            //printf("iteration=%d\n", iter);

            auto t1 = std::chrono::steady_clock::now();
            
            train_step();

            //update mean/std
            bool use_observation_filter = true;
            if (use_observation_filter)
            {
                for (int index=0;index<g_num_total_threads;index++)
                {
                    worker_->observation_means_[index].resize(worker_->env_.observation_dim_);
                    worker_->observation_stds_[index].resize(worker_->env_.observation_dim_);
                    for (int w=0;w<worker_->observation_filters_[index].size();w++)
                    {
                        worker_->observation_means_[index][w] = worker_->observation_filters_[index][w].Mean();
                        worker_->observation_stds_[index][w] = worker_->observation_filters_[index][w].StandardDeviation();
                    }
                }
            }

            auto t2 = std::chrono::steady_clock::now();
            double past_sec = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            printf("total time of step %d: %f\n", iter+1, past_sec);

            //print('iter ', i,' done')

            //record statistics every 10 iterations
            if ((iter + 1) % 10 == 0) 
            {

                int num_rollouts = g_num_total_threads;//todo: expose/tune this value (100 in ARS)
                bool evaluate = true;
                std::vector<std::vector<std::vector<double> > > trajectories;
                std::vector<double> rewards = aggregate_rollouts(num_rollouts, evaluate, trajectories);
                double sum =0;
                double min_reward = 1e30;
                double max_reward = -1e30;
                int max_reward_index = -1;
                int min_reward_index = -1;

                if (rewards.size())
                {
                    for (int i=0;i<rewards.size();i++)
                    {
                        double reward = rewards[i];
                        //printf("reward %d = %f\n", i, reward);
                        sum += reward;
                        if (reward < min_reward)
                        {
                            min_reward = reward;
                            min_reward_index = i;
                        }
                        if (reward > max_reward)
                        {
                            max_reward = reward;
                            max_reward_index = i;
                        }
                    }
                    double mean_rewards = sum / rewards.size();
                    if (mean_rewards > best_mean_rewards)
                    {
                        {
                            std::ofstream trajfile_;
                            std::string fileName = "laikago_trajectory_reward"+std::to_string(mean_rewards)+".bin";
                            trajfile_.open (fileName,std::ios_base::binary);
                            int num_steps = trajectories[0].size();
                            trajfile_.write((char*)&num_steps, sizeof(int));
                            int state_size = trajectories[0][0].size();
                            trajfile_.write((char*)&state_size, sizeof(int));
                        
                            for (int step=0;step<num_steps;step++)
                            {
                            
                                for (int state=0;state < state_size;state++)
                                {
                                    trajfile_.write((char*)&trajectories[0][step][state], sizeof(double));
                                }
                            }
                            trajfile_.close();
                            best_mean_rewards = mean_rewards;
                            //save policy
                            printf("best policy, mean = %f at %d steps\n", mean_rewards, total_timesteps);
                       
                   

                             for (int w=0;w<this->w_policy.size();w++)
                                {
                                    printf("%f,",w_policy[w]);
                                }
                                printf("\n");
                         }
                            {
                            std::ofstream weightsfile_;
                            std::string fileName = "ant_weights_"+std::to_string(mean_rewards)+".bin";
                            weightsfile_.open (fileName,std::ios_base::binary);
                            int num_weights =w_policy.size();
                            weightsfile_.write((char*)&num_weights, sizeof(int));
                            for (int i=0;i<num_weights;i++)
                            {
                                weightsfile_.write((char*)&w_policy[i], sizeof(double));
                            }
                            weightsfile_.close();
                        }
                     }

                        auto cur_point_ = std::chrono::steady_clock::now();

                        double past_sec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_point_ - time_point_).count();
                        //time_point_ = cur_point_;

                        myfile_ << past_sec/1000. << "    " << std::to_string(iter+1) << " " << std::to_string(mean_rewards) << "  " << max_reward << "    " << min_reward << "    " << total_timesteps << std::endl;

                    printf("Iteration = %d\n", iter+1);
                    printf("total_timesteps=%d\n", total_timesteps);
                    printf("AverageReward=%f\n", mean_rewards);
                    printf("MaxReward[%d]=%f\n", max_reward_index, max_reward);
                    printf("MinReward[%d]=%f\n", min_reward_index, min_reward);
                }
#if 0
                w = ray.get(self.workers[0].get_weights_plus_stats.remote())
                np.savez(self.logdir + "/lin_policy_plus_latest", w)
                
                mean_rewards = np.mean(rewards)
                if (mean_rewards > best_mean_rewards):
                  best_mean_rewards = mean_rewards
                  np.savez(self.logdir + "/lin_policy_plus_best_"+str(i+1), w)
                  
                
                print(sorted(self.params.items()))
                logz.log_tabular("Time", time.time() - start)
                logz.log_tabular("Iteration", i + 1)
                logz.log_tabular("AverageReward", np.mean(rewards))
                logz.log_tabular("StdRewards", np.std(rewards))
                logz.log_tabular("MaxRewardRollout", np.max(rewards))
                logz.log_tabular("MinRewardRollout", np.min(rewards))
                logz.log_tabular("timesteps", self.timesteps)
                logz.dump_tabular()
#endif
            }
#if 0
            t1 = time.time()
            # get statistics from all workers
            for j in range(self.num_workers):
                self.policy.observation_filter.update(ray.get(self.workers[j].get_filter.remote()))
            self.policy.observation_filter.stats_increment()

            # make sure master filter buffer is clear
            self.policy.observation_filter.clear_buffer()
            # sync all workers
            filter_id = ray.put(self.policy.observation_filter)
            setting_filters_ids = [worker.sync_filter.remote(filter_id) for worker in self.workers]
            # waiting for sync of all workers
            ray.get(setting_filters_ids)
         
            increment_filters_ids = [worker.stats_increment.remote() for worker in self.workers]
            # waiting for increment of all workers
            ray.get(increment_filters_ids)            
            t2 = time.time()
            print('Time to sync statistics:', t2 - t1)
#endif

        }
    }

};



int main()
{
  


    
    
    CudaModel<MyScalar> cuda_model_ant(model_name);

    
    cuda_model_ant.forward_zero.allocate(g_num_total_threads);

  
#ifdef ARS_VISUALIZE
  visualizer.delete_all();
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
  tds::FileUtils::find_file(contact_sim.m_laikago_urdf_filename, file_and_path);
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
#if 1
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

#endif
#if 0
  std::string file_name = "D:/dev/tds/tiny-differentiable-simulator/build_cmake/examples/ars/trajectory_reward117.566790.bin";
    std::ifstream trajfile_;
                        
    trajfile_.open (file_name,std::ios_base::binary);

    int num_steps;

    trajfile_.read((char*)&num_steps, sizeof(int));
    int state_size;
    trajfile_.read((char*)&state_size, sizeof(int));
    std::vector<double> sim_state_with_graphics;
    sim_state_with_graphics.resize(state_size);
    for (int step=0;step<num_steps;step++)
    {
        for (int state=0;state < state_size;state++)
        {
            trajfile_.read((char*)&sim_state_with_graphics[state], sizeof(double));
        }
   
        visualize_trajectory(sim_state_with_graphics);
  }
    
  trajfile_.close();
  #endif


  for (int i=0;i<240;i++)
  {
      visualizer.render();
      //std::this_thread::sleep_for(std::chrono::duration<double>(1./240.));//frameskip_gfx_sync* contact_sim.dt));
  }
   //srand(123);
  {
    ARSLearner ars(cuda_model_ant);
    ars.train(50*1024*1024);
  }

    cuda_model_ant.forward_zero.deallocate();
}

#if 0
  std::vector<std::vector<Scalar>> outputs(
      g_num_total_threads, std::vector<Scalar>(simulation.output_dim()));

  std::vector<std::vector<Scalar>> inputs(g_num_total_threads);

#endif
