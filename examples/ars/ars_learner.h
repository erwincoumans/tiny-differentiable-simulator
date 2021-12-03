#ifndef ARS_LEARNER_H
#define ARS_LEARNER_H

#pragma once

#include "ars_vectorized_worker.h"
#include "ars_config.h"
#include "visualizer/opengl/utils/tiny_logging.h"

template <typename ARSVecEnvironment>
struct ARSLearner 
{
    std::vector<double> w_policy;

    int total_timesteps{0};
    int num_deltas_{0};
    int shift_{0};
    
    std::vector<double> deltas_;

    
    Worker<ARSVecEnvironment>* worker_{0};

    ARSConfig config_;
    
    std::ofstream myfile_;

    std::chrono::steady_clock::time_point time_point_;
    

    ARSLearner(ARSVecEnvironment& tmpenv, const ARSConfig& config) :config_(config)
    {
        init_deltas();

        PolicyParams policy_params;
        
        int params_dim = tmpenv.neural_networks_[0].num_parameters();
        
        //does it have to be random to start? deltas will take care of it
        w_policy.resize(params_dim);
        
        worker_ = new Worker (tmpenv,  params_dim, policy_params, deltas_, config_);
        
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
                g_hat[j] += w * tmp_delta[j]*config_.delta_std_;//either multiply with delta_std_ or enable use_std_deviation
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
        
        deltas_.resize(config_.deltas_count);
        printf("Creating a normal distribution of size %d.\n", config_.deltas_count);
        for (int i=0;i<config_.deltas_count;i++)
        {
            deltas_[i] = distribution(gen);
        }
        printf("Finished creating a normal distribution.\n");
    }

    //Aggregate update step from rollouts generated in parallel.
    std::vector<double> aggregate_rollouts(int num_rollouts, bool evaluate, std::vector<std::vector<std::vector<double> > >& trajectories) {
          
        B3_PROFILE("aggregate_rollouts");
        int num_deltas = (num_rollouts == 0)? num_deltas_ : num_rollouts;
            
        //# put policy weights in the object store
        //policy_id = ray.put(self.w_policy)

        //t1 = time.time()
        //num_rollouts = int(num_deltas / num_workers);
        num_rollouts = num_deltas; //assume 1 worker for a start

        std::vector<double> rollout_rewards;
        std::vector<int> deltas_idx;
        int steps;
        {
            B3_PROFILE("weighted_sum_custom");
            worker_->do_rollouts( rollout_rewards, deltas_idx, steps, w_policy, num_rollouts, shift_, evaluate, trajectories);
        }
        
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

        {
            B3_PROFILE("weighted_sum_custom");
            weighted_sum_custom(rollout_rewards, deltas_idx, g_hat, num_items_summed);
        }
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

        
        {
            B3_PROFILE("update w_policy");
            //w_policy -= self.optimizer._compute_step(g_hat).reshape(self.w_policy.shape);
            for (int i=0;i<w_policy.size();i++) {
                w_policy[i] += config_.sgd_step_size * g_hat[i];
            }
        }
    }


    void train()
    {
        double best_mean_rewards = -1e30;
        
        for (int iter=0;iter< config_.num_iter;iter++) {
            printf("iteration=%d\n", iter);
            B3_PROFILE("iteration");
            auto t1 = std::chrono::steady_clock::now();
            
            
            {
                B3_PROFILE("train");
                train_step();
            }

            //update mean/std
            bool use_observation_filter = true;
            if (use_observation_filter)
            {
                B3_PROFILE("update observation_filter");
                for (int index=0;index<config_.batch_size;index++)
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
            //record statistics every n iterations
            if ((iter + 1) % config_.eval_interval == 0) 
            {
                B3_PROFILE("eval");
                int num_rollouts = config_.batch_size;
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
                    if (mean_rewards > best_mean_rewards && trajectories[0].size())
                    {
                        {
                            std::ofstream trajfile_;
                            std::string fileName = worker_->env_.contact_sim.env_name() + "_trajectory_reward"+std::to_string(mean_rewards)+".bin";
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
                            std::string fileName = worker_->env_.contact_sim.env_name()+"_weights_"+std::to_string(mean_rewards)+".bin";
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



#endif //ARS_LEARNER_H