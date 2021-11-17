#ifndef ARS_VECTORIZED_ENVIRONMENT_H
#define ARS_VECTORIZED_ENVIRONMENT_H

#include <algorithm>
#include <thread>
#include <functional>
#include <vector>

/// @param[in] nb_elements : size of your for loop
/// @param[in] functor(start, end) :
/// your function processing a sub chunk of the for loop.
/// "start" is the first index to process (included) until the index "end"
/// (excluded)
/// @code
///     for(int i = start; i < end; ++i)
///         computation(i);
/// @endcode
/// @param use_threads : enable / disable threads.
///
///
static
void parallel_for(unsigned nb_elements,
                  std::function<void (int start, int end)> functor,
                  bool use_threads = true)
{
    // -------
    unsigned nb_threads_hint = 20;//std::thread::hardware_concurrency();
    unsigned nb_threads = nb_threads_hint == 0 ? 8 : (nb_threads_hint);

    unsigned batch_size = nb_elements / nb_threads;
    unsigned batch_remainder = nb_elements % nb_threads;

    std::vector< std::thread > my_threads(nb_threads);

    if( use_threads )
    {
        // Multithread execution
        for(unsigned i = 0; i < nb_threads; ++i)
        {
            int start = i * batch_size;
            my_threads[i] = std::thread(functor, start, start+batch_size);
        }
    }
    else
    {
        // Single thread execution (for easy debugging)
        for(unsigned i = 0; i < nb_threads; ++i){
            int start = i * batch_size;
            functor( start, start+batch_size );
        }
    }

    // Deform the elements left
    //int start = nb_threads * batch_size;
    //functor( start, start+batch_remainder);

    // Wait for the other thread to finish their task
    if( use_threads )
        std::for_each(my_threads.begin(), my_threads.end(), std::mem_fn(&std::thread::join));
}

#define PARALLEL_FOR_BEGIN(nb_elements) parallel_for(nb_elements, [&](int start, int end){ for(int i = start; i < end; ++i)
#define PARALLEL_FOR_END()})

template <typename Algebra, typename AbstractSimulation>
struct VectorizedEnvironment
{
    AbstractSimulation& contact_sim;
    using Scalar = typename Algebra::Scalar;
    using Vector3 = typename Algebra::Vector3;
    using Transform = typename Algebra::Transform;

    
    std::vector<std::vector<Scalar>> sim_states_;
    std::vector<std::vector<Scalar>> sim_states_with_action_and_variables;
    std::vector<std::vector<Scalar>> sim_states_with_graphics_;

    std::vector<tds::NeuralNetwork<Algebra> > neural_networks_;
    int observation_dim_{0};


    VectorizedEnvironment(AbstractSimulation& sim)
        :contact_sim(sim)
    {
        neural_networks_.resize(g_num_total_threads);
        sim_states_.resize(g_num_total_threads);
        sim_states_with_action_and_variables.resize(g_num_total_threads);
        sim_states_with_graphics_.resize(g_num_total_threads);

        observation_dim_ = contact_sim.input_dim();
        bool use_input_bias = false;
        for (int index=0;index<g_num_total_threads;index++)
        {
            neural_networks_[index].set_input_dim(observation_dim_, use_input_bias);
            //network.add_linear_layer(tds::NN_ACT_RELU, 32);
            //neural_network.add_linear_layer(tds::NN_ACT_RELU, 64);
            bool learn_bias = true;
            neural_networks_[index].add_linear_layer(tds::NN_ACT_IDENTITY, sim.action_dim(),learn_bias);
        }
        
    }
    virtual ~VectorizedEnvironment()
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
    
    std::vector< std::vector<double> > reset()
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
                for(int j=0;j<get_initial_poses<Scalar>().size();j++)
                {
                    sim_states_[index][j+qoffset] = get_initial_poses<Scalar>()[j]+0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
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
                for(int j=0;j<get_initial_poses<Scalar>().size();j++)
                {
                    sim_states_[index][j+qoffset] = get_initial_poses<Scalar>()[j]+0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
                }

            }
        }
        std::vector< std::vector<double>>  zero_actions(g_num_total_threads);
        for (int i=0;i<g_num_total_threads;i++)
        {
            zero_actions[i].resize(get_initial_poses<Scalar>().size(), Scalar(0));
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
            step(zero_actions, observations, rewards, dones);
        }
        
        //for (auto v : sim_state)
        //    std::cout << v << std::endl;
        return observations;
    }
    void step( std::vector< std::vector<double>>& actions,std::vector<std::vector<double>>& observations,
        std::vector<double>& rewards,std::vector<bool>& dones)
    {

        std::vector<std::vector<Scalar>> outputs(
        g_num_total_threads, std::vector<Scalar>(contact_sim.output_dim()));

        std::vector<std::vector<Scalar>> inputs(g_num_total_threads);


        for (int index=0;index<g_num_total_threads;index++)
        {
            int simstate_size = sim_states_[index].size();
            sim_states_with_action_and_variables[index] = sim_states_[index];
            sim_states_with_action_and_variables[index].resize(contact_sim.input_dim_with_action_and_variables());
            
            contact_sim.prepare_sim_state_with_action_and_variables(
                sim_states_with_action_and_variables[index],
                actions[index]);
            inputs[index] = sim_states_with_action_and_variables[index];
        }
    
//#define DEBUG_ON_CPU
#ifdef DEBUG_ON_CPU
        
        
        for (int index =0; index<g_num_total_threads;index++)
        {
            sim_states_with_graphics_[index] = contact_sim.step_forward1(sim_states_with_action_and_variables[index]);
        }
#else
        
#if 1
        #pragma omp parallel
        #pragma omp for
        for (int index=0;index<g_num_total_threads;index++)
        {
            if (!dones[index]) 
            {
                contact_sim.forward_kernel(1,&outputs[index][0], &inputs[index][0]);
            }
        }
#else
        PARALLEL_FOR_BEGIN(g_num_total_threads)
        {
            if (!dones[i]) 
            {
                contact_sim.forward_kernel_fast(1,&outputs[i][0], &inputs[i][0]);
            }
        }PARALLEL_FOR_END();

#endif
        for (int index=0;index<g_num_total_threads;index++)
        {
            if (!dones[index]) 
            {
                sim_states_with_graphics_[index] = outputs[index];
            }
        }
#endif //DEBUG_ON_CPU
        for (int index=0;index<g_num_total_threads;index++)
        {
            if (!dones[index])
            {
                bool done;
                Scalar reward;
                contact_sim.compute_reward_done(
                    sim_states_[index], 
                    sim_states_with_graphics_[index],
                    reward, done);
                rewards[index] = reward;
                dones[index] = done;

                sim_states_[index] = sim_states_with_graphics_[index];
                sim_states_[index].resize(contact_sim.input_dim());
                observations[index] = sim_states_[index];
            } else
            {
                rewards[index] = 0;
            }
        }
    }

    inline const std::vector<double> policy(int index, const std::vector<double>& obs)
    {
        std::vector<double> action (get_initial_poses<Scalar>().size(), Scalar(0));
    
        neural_networks_[index].compute(obs, action);
                
        return action;
    }

};


#endif //ARS_VECTORIZED_ENVIRONMENT_H