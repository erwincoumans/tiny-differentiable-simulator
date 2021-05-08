#include "../environments/cartpole_environment.h"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include <iostream>
#include <fstream>
#include "shared_noise_table.h"
#include "running_stat.h"

#include <chrono>

using namespace TINY;
using namespace tds;
typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

typedef CartpoleEnv<MyAlgebra> Environment;

struct PolicyParams
{
};



//Object class for parallel rollout generation.
struct Worker
{
    PolicyParams m_policy_params_;
    int rollout_length_;
    double delta_std_;
    
    CartpoleContactSimulation<MyAlgebra>& sim_;
    Environment env_{sim_};

    SharedNoiseTable noise_table_;

    std::vector<RunningStat> observation_filter_;
    std::vector<double> observation_mean_;
    std::vector<double> observation_std_;

    Worker(int env_seed, int params_dim, const PolicyParams& policy_params, CartpoleContactSimulation<MyAlgebra>& sim, const std::vector<double>& deltas, int rollout_length, double delta_std)
        :m_policy_params_(policy_params),
        rollout_length_ (rollout_length),
        delta_std_(delta_std),
        sim_(sim),
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
    void rollout(double shift, int rollout_length, double& total_reward, int& steps)
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

        if (rollout_length == 0) {
            rollout_length = rollout_length_;
        }

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
        const std::vector<double>& w_policy, int num_rollouts = 1, int shift = 1, bool evaluate = false) {

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
                int r_steps;
                rollout(0, rollout_length_, reward, r_steps);
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
                rollout(shift, rollout_length_, pos_reward, pos_steps);
                
                //compute reward and number of timesteps used for negative pertubation rollout
                for (int i=0;i<w_policy.size();i++)
                {
                    weights[i] = w_policy[i]-delta[i];
                }
                env_.init_neural_network(weights);
                double neg_reward;
                int neg_steps;
                rollout(shift, rollout_length_, neg_reward, neg_steps);
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
    int num_deltas_{8};
    int shift_{0};
    
    std::vector<double> deltas_;

    std::vector<CartpoleContactSimulation<MyAlgebra>> sims;
    std::vector<Worker> workers_;
    int rollout_length_{1000};
    double delta_std_{0.03};
    double sgd_step_size { 0.02};

    std::ofstream myfile_;

    std::chrono::steady_clock::time_point time_point_;

    ARSLearner()
    {
        init_deltas();

        int env_seed=12345;//421;
        
        PolicyParams policy_params;

        int params_dim = 5;//tmpenv_.neural_network.num_parameters();
        
        //does it have to be random to start? deltas will take care of it
        w_policy.resize(params_dim);
        sims.resize(1);
        Worker worker(env_seed, params_dim, policy_params, sims[0], deltas_, rollout_length_, delta_std_);
        workers_.resize(1,worker);
  
        myfile_.open ("ars_cpp_log.txt");
        myfile_ << "Time	Iteration	AverageReward	StdRewards	MaxRewardRollout	MinRewardRollout	timesteps" << std::endl;

        time_point_ = std::chrono::steady_clock::now();

    }

    virtual ~ARSLearner()
    {
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
    std::vector<double> aggregate_rollouts(int num_rollouts, bool evaluate) {
          
        int num_deltas = (num_rollouts == 0)? num_deltas_ : num_rollouts;
            
        //# put policy weights in the object store
        //policy_id = ray.put(self.w_policy)

        //t1 = time.time()
        //num_rollouts = int(num_deltas / num_workers);
        num_rollouts = num_deltas; //assume 1 worker for a start

        std::vector<double> rollout_rewards;
        std::vector<int> deltas_idx;
        int steps;
        workers_[0].do_rollouts(rollout_rewards, deltas_idx, steps, w_policy, num_rollouts, shift_, evaluate);
        total_timesteps += steps;


        if (evaluate)
            return rollout_rewards;

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
        auto g_hat = aggregate_rollouts(num_rollouts, evaluate);
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
        
        for (int i=0;i< num_iter;i++) {

            //t1 = time.time()
            train_step();

            //update mean/std
            bool use_observation_filter = true;
            if (use_observation_filter)
            {
                workers_[0].observation_mean_.resize(workers_[0].env_.observation_dim_);
                workers_[0].observation_std_.resize(workers_[0].env_.observation_dim_);
                for (int w=0;w<workers_[0].observation_filter_.size();w++)
                {
                    workers_[0].observation_mean_[w] = workers_[0].observation_filter_[w].Mean();
                    workers_[0].observation_std_[w] = workers_[0].observation_filter_[w].StandardDeviation();
                }
            }

            //t2 = time.time()
            //print('total time of one step', t2 - t1)           

            //print('iter ', i,' done')

            //record statistics every 10 iterations
            if ((i + 1) % 10 == 0) {

                int num_rollouts = 100;//todo: expose/tune this value (100 in ARS)
                bool evaluate = true;
                std::vector<double> rewards = aggregate_rollouts(num_rollouts, evaluate);
                double sum =0;
                double min_reward = 1e30;
                double max_reward = -1e30;

                if (rewards.size())
                {
                    for (int i=0;i<rewards.size();i++)
                    {
                        double reward = rewards[i];
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
                        best_mean_rewards = mean_rewards;
                        //save policy
                        printf("best policy, mean = %f at %d steps\n", mean_rewards, total_timesteps);
                       
                    }

                     for (int w=0;w<this->w_policy.size();w++)
                        {
                            printf("%f,",w_policy[w]);
                        }
                        printf("\n");
                       
                     printf("Iteration = %d\n", i+1);
                    printf("total_timesteps=%d\n", total_timesteps);
                    printf("AverageReward=%f\n", mean_rewards);
                    printf("MaxReward=%f\n", max_reward);
                    printf("MinReward=%f\n", min_reward);
                    
                    auto cur_point_ = std::chrono::steady_clock::now();

                    double past_sec = std::chrono::duration_cast<std::chrono::milliseconds>(cur_point_ - time_point_).count();
                    //time_point_ = cur_point_;

                    myfile_ << past_sec/1000. << "    " << std::to_string(i+1) << " " << std::to_string(mean_rewards) << "  " << max_reward << "    " << min_reward << "    " << total_timesteps << std::endl;
                    
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
   srand(123);
    ARSLearner ars;
    ars.train(10*1024*1024);
}
