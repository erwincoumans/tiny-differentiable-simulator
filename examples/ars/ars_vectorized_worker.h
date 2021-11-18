#ifndef ARS_VECTORIZER_WORKER_H
#define ARS_VECTORIZER_WORKER_H

//Object class for parallel rollout generation.
template <typename ARSVecEnvironment>
struct Worker
{
    PolicyParams m_policy_params_;
    int rollout_length_eval_;
    int rollout_length_train_;
    double delta_std_;
    
    
    ARSVecEnvironment env_;

    SharedNoiseTable noise_table_;

    std::vector<std::vector<RunningStat>> observation_filters_;
    std::vector<std::vector<double> > observation_means_;
    std::vector<std::vector<double> > observation_stds_;

    Worker(ARSVecEnvironment& env, int env_seed, int params_dim, const PolicyParams& policy_params, const std::vector<double>& deltas, int rollout_length_train, int rollout_length_eval, double delta_std)
        :env_(env),
        m_policy_params_(policy_params),
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
    void rollouts( double shift, int rollout_length, std::vector<double>& total_rewards, std::vector<int>& vec_steps, std::vector< std::vector<std::vector<double> > >& trajectories)
    {

        

        if (rollout_length == 0)
        {
            printf("rollout_length =%d\n",rollout_length);
        }
        //if (rollout_length == 0) {
        //    rollout_length = rollout_length_;
        //}

        auto observations = env_.reset();
        
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
           if (r%16==0) {
            //printf("r=%d\n", r);
           }
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

           env_.step(actions, observations, rewards, dones);

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
    void do_rollouts(std::vector<double>& rollout_rewards, std::vector<int>& deltas_idx, int& steps, 
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

            rollouts( 0, rollout_length_eval_, rewards, vec_r_steps, trajectories);
            
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
            
            rollouts( shift, rollout_length_train_, pos_rewards, vec_pos_steps, trajectories);
            
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

            rollouts( shift, rollout_length_train_, neg_rewards, vec_neg_steps,trajectories);

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

#endif //ARS_VECTORIZER_WORKER_H