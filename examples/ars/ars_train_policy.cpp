#define ARS_VISUALIZE

#define NOMINMAX 

//#define USE_ANT
#ifdef USE_ANT
#include "../environments/ant_environment.h"
#else

//#include "../environments/laikago_environment.h"
#include "../environments/cartpole_environment2.h"
//#include "../environments/reacher_environment.h"
//#include "../environments/reacher_environment.h"

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

using namespace TINY;
using namespace tds;
typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

#ifdef USE_ANT
typedef AntEnv<MyAlgebra> Environment;
#else//USE_ANT

typedef CartpoleEnv<MyAlgebra> Environment;
//typedef ReacherEnv<MyAlgebra> Environment;
//typedef LaikagoEnv<MyAlgebra> Environment;


#endif//USE_ANT

struct PolicyParams
{
};

int num_total_threads = 32;

#ifdef ARS_VISUALIZE
///////////////////////////////////////////
// create graphics
OpenGLUrdfVisualizer<MyAlgebra> visualizer;
#ifdef USE_ANT
AntContactSimulation<MyAlgebra> contact_sim;
#else
CartpoleContactSimulation<MyAlgebra> contact_sim;
//ReacherContactSimulation<MyAlgebra> contact_sim;
#endif


tds::UrdfStructures<MyAlgebra> urdf_structures;

std::vector<int> visual_instances;
std::vector<int> num_instances;
int num_instances_per_robot=0;
int num_base_instances;

void visualize_trajectory(const std::vector<double>& sim_state_with_graphics, int s)
{
     float sim_spacing = 5;
     
    const int square_id = (int)std::sqrt((double)num_total_threads);
  int offset = contact_sim.mb_->dof() + contact_sim.mb_->dof_qd();
  int instance_index = s*num_instances_per_robot;
  

  

  for (int ll = 0; ll < contact_sim.mb_->links_.size(); ll++) {
        int l = ll;
        for (int v = 0; v < num_instances[l]; v++)
        {
            int visual_instance_id = visual_instances[instance_index++];
            if (visual_instance_id >= 0)
            {

                ::TINY::TinyVector3f pos(sim_state_with_graphics[offset + l * 7 + 0],
                    sim_state_with_graphics[offset + l * 7 + 1],
                    sim_state_with_graphics[offset + l * 7 + 2]);
                ::TINY::TinyQuaternionf orn(
                    sim_state_with_graphics[offset + l * 7 + 3],
                    sim_state_with_graphics[offset + l * 7 + 4],
                    sim_state_with_graphics[offset + l * 7 + 5],
                    sim_state_with_graphics[offset + l * 7 + 6]);

                pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
            }
        }
    }
    visualizer.render();
   
    //std::this_thread::sleep_for(std::chrono::duration<double>(1./240.));//frameskip_gfx_sync* contact_sim.dt));
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

    std::vector<RunningStat> observation_filter_;
    std::vector<double> observation_mean_;
    std::vector<double> observation_std_;

    Worker(int env_seed, int params_dim, const PolicyParams& policy_params, const std::vector<double>& deltas, int rollout_length_train, int rollout_length_eval, double delta_std)
        :m_policy_params_(policy_params),
        rollout_length_train_ (rollout_length_train),
        rollout_length_eval_ (rollout_length_eval),
        delta_std_(delta_std),
        noise_table_(deltas, env_seed, env_.neural_network.num_parameters())
    {
        
        env_.seed(env_seed);
        observation_filter_.resize(env_.observation_dim_);
       
    }
        
    virtual ~Worker()
    {
    }
    //Get current policy weights and current statistics of past states.
    void get_weights_plus_stats(std::vector<double>& weights) {
        //return self.policy.get_weights_plus_stats()
    }
    
    //Performs one rollout of maximum length rollout_length. 
    //At each time-step it substracts shift from the reward.
    void rollout(double shift, int rollout_length, double& total_reward, int& steps, std::vector<std::vector<double> >& trajectory)
    {
#if 0
        std::vector<double> params;
        env_.neural_network.get_parameters(params);
        double sum=0;
        for (int i=0;i<5;i++)
        {
            if (i==2)
            {
                double v = (params[i]-4.);
                sum -= v*v;
            } 
            //else if (i==4)
            //{
            //    double v = (params[i]+1000);
            //    sum -= v*v;
            //    
            else
            {
                //sum -= params[i]*params[i];
            }
        }
        total_reward = sum;
        steps = 1;
        return;
#endif
        steps = 0;
        total_reward = 0.;

        if (rollout_length == 0)
        {
            printf("rollout_length =%d\n",rollout_length);
        }
        //if (rollout_length == 0) {
        //    rollout_length = rollout_length_;
        //}

        auto obs = env_.reset();
        
        for (int i =0;i<rollout_length;i++) {
          double reward;
          bool  done;
          //update running stat for the observation filter
          for (int o=0;o<obs.size();o++)
          {
              observation_filter_[o].Push(obs[o]);

              if (observation_mean_.size())
              {
                  //apply filter
                  obs[o] -= observation_mean_[o];
                  if (observation_std_[o]>1e-7)
                  {
                      obs[o] /= observation_std_[o];
                  }
              }
          }

          auto action = env_.policy(obs);
          

          env_.step(action,obs,reward,done);
          trajectory.push_back(env_.sim_state_with_graphics);
          total_reward += (reward - shift);
          steps++;
          if (done)
              break;
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
    void do_rollouts(std::vector<double>& rollout_rewards, std::vector<int>& deltas_idx, int& steps, 
        const std::vector<double>& w_policy, int num_rollouts, int shift, bool evaluate,
        std::vector<std::vector<std::vector<double> > >& trajectories) {

        steps = 0;
        
        std::vector<double> tmp;
        
        bool use_std_deviation = false;//!evaluate;

        for (int i =0; i< num_rollouts; i++) {
            if (evaluate) {
                
                //policy.update_weights(w_policy);
                env_.init_neural_network(w_policy);
                deltas_idx.push_back(-1);
                
                //set to false so that evaluation rollouts are not used for updating state statistics
                //policy.update_filter = false;

                //for evaluation we do not shift the rewards (shift = 0) and we use the default rollout_length
                //double reward, int r_steps = rollout(shift = 0., rollout_length = self.rollout_length);
                double reward;
                int r_steps = 0;
                std::vector<std::vector<double> > trajectory;
                rollout(0, rollout_length_eval_, reward, r_steps, trajectory);
                trajectories.push_back(trajectory);
                steps += r_steps;
                rollout_rewards.push_back(reward);

            }
            else {
                std::vector<double> delta;
                //idx, delta = deltas.get_delta(w_policy.size)
                int delta_idx = noise_table_.get_delta(delta_std_, delta);

                //delta = (self.delta_std * delta).reshape(w_policy.shape)
                deltas_idx.push_back(delta_idx);

                //# set to true so that state statistics are updated 
                //self.policy.update_filter = True

                //# compute reward and number of timesteps used for positive perturbation rollout
                std::vector<double> weights;
                weights.resize(w_policy.size());
                for (int i=0;i<w_policy.size();i++)
                {
                    weights[i] = w_policy[i]+delta[i];
                }
                env_.init_neural_network(weights);
                //self.policy.update_weights(w_policy + delta)
                //env.update_weights(weights)
                //pos_reward, pos_steps  = self.rollout(shift = shift)
                double pos_reward;
                int pos_steps;
                std::vector<std::vector<double> > trajectory;
                rollout(shift, rollout_length_train_, pos_reward, pos_steps, trajectory);
                static int cell=0;

                for (int t=0;t<trajectory.size();t++)
                {
                    visualize_trajectory(trajectory[t], cell);
                }
                cell++;
                if (cell>=num_total_threads)
                    cell=0;
                //compute reward and number of timesteps used for negative pertubation rollout
                for (int i=0;i<w_policy.size();i++)
                {
                    weights[i] = w_policy[i]-delta[i];
                }
                env_.init_neural_network(weights);
                double neg_reward;
                int neg_steps;
                trajectory.resize(0);
                rollout(shift, rollout_length_train_, neg_reward, neg_steps,trajectory);
                for (int t=0;t<trajectory.size();t++)
                {
                    visualize_trajectory(trajectory[t], cell);
                }
                cell++;
                if (cell>=num_total_threads)
                    cell=0;
                steps += pos_steps + neg_steps;
                //rollout_rewards[:,0] - rollout_rewards[:,1]
                if (use_std_deviation)
                {
                    tmp.push_back(pos_reward);
                    tmp.push_back(neg_reward);
                } else
                {
                    rollout_rewards.push_back(pos_reward-neg_reward);
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
    int num_deltas_{32};
    int shift_{0};
    
    std::vector<double> deltas_;

    
    Worker* worker_{0};
    int rollout_length_train_{400};
    int rollout_length_eval_{1000};
    double delta_std_{0.03};
    double sgd_step_size { 0.02};

    std::ofstream myfile_;

    std::chrono::steady_clock::time_point time_point_;

    ARSLearner()
    {
        init_deltas();

        int env_seed=12345;//421;
        
        PolicyParams policy_params;
        
        
        Environment tmpenv;
        int params_dim = tmpenv.neural_network.num_parameters();
        
        //does it have to be random to start? deltas will take care of it
        w_policy.resize(params_dim);
        
        worker_ = new Worker (env_seed, params_dim, policy_params, deltas_, rollout_length_train_, rollout_length_eval_, delta_std_);
        
        myfile_.open ("ars_cpp_log.txt");
        myfile_ << "Time	Iteration	AverageReward	StdRewards	MaxRewardRollout	MinRewardRollout	timesteps" << std::endl;

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
        worker_->do_rollouts(rollout_rewards, deltas_idx, steps, w_policy, num_rollouts, shift_, evaluate, trajectories);
        
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

            //t1 = time.time()
            train_step();

            //update mean/std
            bool use_observation_filter = true;
            if (use_observation_filter)
            {
                worker_->observation_mean_.resize(worker_->env_.observation_dim_);
                worker_->observation_std_.resize(worker_->env_.observation_dim_);
                for (int w=0;w<worker_->observation_filter_.size();w++)
                {
                    worker_->observation_mean_[w] = worker_->observation_filter_[w].Mean();
                    worker_->observation_std_[w] = worker_->observation_filter_[w].StandardDeviation();
                }
            }

            //t2 = time.time()
            //print('total time of one step', t2 - t1)           

            //print('iter ', i,' done')

            //record statistics every 10 iterations
            if (1)//(i + 1) % 10 == 0) 
            {

                int num_rollouts = 10;//todo: expose/tune this value (100 in ARS)
                bool evaluate = true;
                std::vector<std::vector<std::vector<double> > > trajectories;
                std::vector<double> rewards = aggregate_rollouts(num_rollouts, evaluate, trajectories);
                double sum =0;
                double min_reward = 1e30;
                double max_reward = -1e30;

                if (rewards.size())
                {

                    for (int i=0;i<rewards.size();i++)
                    {
                        double reward = rewards[i];
                        printf("reward %d = %f\n", i, reward);
                        sum += reward;
                        if (reward < min_reward)
                        {
                            min_reward = reward;
                        }
                        if (reward > max_reward)
                        {
                            max_reward = reward;
                        }
                    }
                    double mean_rewards = sum / rewards.size();
                    if (mean_rewards > best_mean_rewards)
                    {
                        
                        std::ofstream trajfile_;
                        std::string fileName = "trajectory_reward"+std::to_string(mean_rewards)+".bin";
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
                    printf("Iteration = %d\n", iter+1);
                    printf("total_timesteps=%d\n", total_timesteps);
                    printf("AverageReward=%f\n", mean_rewards);
                    printf("MaxReward=%f\n", max_reward);
                    printf("MinReward=%f\n", min_reward);
                    
                    auto cur_point_ = std::chrono::steady_clock::now();

                    double past_sec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_point_ - time_point_).count();
                    //time_point_ = cur_point_;

                    myfile_ << past_sec/1000. << "    " << std::to_string(iter+1) << " " << std::to_string(mean_rewards) << "  " << max_reward << "    " << min_reward << "    " << total_timesteps << std::endl;
                    
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
  tds::FileUtils::find_file(contact_sim.m_urdf_filename, file_and_path);
  urdf_structures = contact_sim.cache.retrieve(file_and_path);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);
  
  


  for (int t = 0;t< num_total_threads;t++)
  {
      num_instances_per_robot=0;
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
      for (int i = 0; i < contact_sim.mb_->num_links(); ++i) {
         

          
          int instance = -1;
          int num_instances_per_link = 0;
          if (urdf_structures.links[i].urdf_visual_shapes.size())
          {
              int uid = urdf_structures.links[i].urdf_visual_shapes[0].visual_shape_uid;
              OpenGLUrdfVisualizer<MyAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];

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
          }
          num_instances.push_back(num_instances_per_link);
          num_instances_per_robot+=num_instances_per_link;
      }
  }

    
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

   //srand(123);
    ARSLearner ars;
    ars.train(50*1024*1024);
}
