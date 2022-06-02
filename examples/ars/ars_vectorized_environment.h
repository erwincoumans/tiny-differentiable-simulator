#ifndef ARS_VECTORIZED_ENVIRONMENT_H
#define ARS_VECTORIZED_ENVIRONMENT_H

#include <algorithm>
#include <thread>
#include <functional>
#include <vector>

#include "ars_config.h"
#include "visualizer/opengl/utils/tiny_logging.h"

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
    using Scalar = typename Algebra::Scalar;
    using Vector3 = typename Algebra::Vector3;
    using Transform = typename Algebra::Transform;

    struct CustomForwardDynamicsStepper {

        CustomForwardDynamicsStepper() {};
        virtual ~CustomForwardDynamicsStepper() {
            }
        virtual void step(const std::vector<std::vector<Scalar>>& thread_inputs,
                    std::vector<std::vector<Scalar>>& thread_outputs,
                    std::vector<bool>& dones,
                    int num_threads_per_block = 32,
                    const std::vector<Scalar>& global_input = {})=0;
    };
    
    
    struct SerialForwardStepper : public CustomForwardDynamicsStepper {

        AbstractSimulation& contact_sim_;
        SerialForwardStepper(AbstractSimulation& contact_sim)
        : contact_sim_(contact_sim) {
        };
        virtual ~SerialForwardStepper() {
        }
        virtual void step(const std::vector<std::vector<Scalar>>& thread_inputs,
                    std::vector<std::vector<Scalar>>& thread_outputs,
                    std::vector<bool>& dones,
                    int num_threads_per_block = 32,
                    const std::vector<Scalar>& global_input = {}){

            B3_PROFILE("SerialForwardStepper step_forward_original");
            for (int index =0; index<thread_inputs.size();index++)
            {
                contact_sim_.step_forward_original(thread_inputs[index], thread_outputs[index]);
            }
        }
    };

    struct OpenMPForwardStepper : public CustomForwardDynamicsStepper {

        AbstractSimulation& contact_sim_;
        int batch_size_;
        
        OpenMPForwardStepper(AbstractSimulation& contact_sim, int batch_size)
        : contact_sim_(contact_sim), batch_size_(batch_size){
        };
        virtual ~OpenMPForwardStepper() {
        }
        virtual void step(const std::vector<std::vector<Scalar>>& thread_inputs,
                     std::vector<std::vector<Scalar>>& thread_outputs,
                     std::vector<bool>& dones,
                    int num_threads_per_block = 32,
                    const std::vector<Scalar>& global_input = {}){
            B3_PROFILE("OpenMPForwardStepper step_forward");

            #pragma omp parallel
            #pragma omp for
            for (int index=0;index<batch_size_;index++)
            {
                if (!dones[index]) 
                {
                    contact_sim_.forward_kernel(1,&thread_outputs[index][0], &thread_inputs[index][0]);
                }
            }
        }
    };




    AbstractSimulation& contact_sim;
 
    
    std::vector<std::vector<Scalar>> sim_states_;
    std::vector<std::vector<Scalar>> sim_states_with_action_and_variables;
    std::vector<std::vector<Scalar>> sim_states_with_graphics_;

    std::vector<tds::NeuralNetwork<Algebra> > neural_networks_;
    int observation_dim_{0};

    SerialForwardStepper serial_stepper_;
    OpenMPForwardStepper omp_stepper_;
    CustomForwardDynamicsStepper* default_stepper_;


    VectorizedEnvironment(AbstractSimulation& sim, int batch_size)
        :contact_sim(sim),
        serial_stepper_(sim),
        omp_stepper_(sim, batch_size),
        default_stepper_(&omp_stepper_)
        //default_stepper_(&serial_stepper_)
    {
        std::cout << "Creating VectorizedEnvironment for " << sim.env_name() << std::endl;
        neural_networks_.resize(batch_size);
        sim_states_.resize(batch_size);
        sim_states_with_action_and_variables.resize(batch_size);
        sim_states_with_graphics_.resize(batch_size);

        observation_dim_ = contact_sim.input_dim();
        bool use_input_bias = false;
        for (int index=0;index<batch_size;index++)
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
    
    std::vector< std::vector<double> > reset(const ARSConfig& config)
    {
        
        std::vector< std::vector<double>> observations;
        observations.resize(config.batch_size);

        for (int index=0;index<config.batch_size;index++)
        {
            sim_states_[index].resize(0);
            sim_states_[index].resize(contact_sim.input_dim_with_action_and_variables(), Scalar(0));

            observations[index].resize(contact_sim.input_dim());
            contact_sim.reset(sim_states_[index], observations[index]);
        }
        return observations;
    }

    void step( std::vector< std::vector<double>>& actions,std::vector<std::vector<double>>& observations,
        std::vector<double>& rewards,std::vector<bool>& dones, const ARSConfig& config)
    {
        B3_PROFILE("VectorizedEnvironment step");
        std::vector<std::vector<Scalar>> outputs(
        config.batch_size, std::vector<Scalar>(contact_sim.output_dim()));

        std::vector<std::vector<Scalar>> inputs(config.batch_size);

        {
             B3_PROFILE("prepare_sim_state_with_action_and_variables");
            for (int index=0;index<config.batch_size;index++)
            {
                int simstate_size = sim_states_[index].size();
                sim_states_with_action_and_variables[index] = sim_states_[index];
                sim_states_with_action_and_variables[index].resize(contact_sim.input_dim_with_action_and_variables());
            
                contact_sim.prepare_sim_state_with_action_and_variables(
                    sim_states_with_action_and_variables[index],
                    actions[index]);
                inputs[index] = sim_states_with_action_and_variables[index];
            }
        }
        
        default_stepper_->step(inputs, outputs, dones);

        for (int index=0;index<config.batch_size;index++)
        {
            //if (!dones[index]) 
            {
                sim_states_with_graphics_[index] = outputs[index];
            }
        }

        {
            B3_PROFILE("compute_reward_done");
            for (int index=0;index<config.batch_size;index++)
            {
                if (!dones[index] || config.auto_reset_when_done)
                {
                    bool done;
                    Scalar reward;
                    contact_sim.compute_reward_done(
                        sim_states_[index], 
                        sim_states_with_graphics_[index],
                        reward, done);
                    rewards[index] = reward;
                    if (done && config.auto_reset_when_done) 
                    {
                      //printf("env %d is done\n", index);
                      sim_states_[index].resize(0);
                      sim_states_[index].resize(
                          contact_sim.input_dim_with_action_and_variables(),
                          Scalar(0));
                      observations[index].resize(contact_sim.input_dim());

                      contact_sim.reset(sim_states_[index],
                                        observations[index]);
                    } 
                    dones[index] = done;
                } else
                {
                    rewards[index] = 0;
                }
                if (dones[index] && config.auto_reset_when_done) {
                } else {
                  sim_states_[index] = sim_states_with_graphics_[index];
                }
                
                sim_states_[index].resize(contact_sim.input_dim());
                observations[index] = sim_states_[index];
                //don't provide x and y position
                observations[index][0] = 0.;
                observations[index][1] = 0.;

            }
        }
    }

    inline const std::vector<double> policy(int index, const std::vector<double>& obs)
    {
        std::vector<double> action (neural_networks_[index].input_dim(), Scalar(0));
    
        neural_networks_[index].compute(obs, action);
                
        return action;
    }


};


#endif //ARS_VECTORIZED_ENVIRONMENT_H
