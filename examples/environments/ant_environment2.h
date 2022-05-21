#ifndef ANT_ENVIRONMENT2_H
#define ANT_ENVIRONMENT2_H

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math.h"
#include "math/neural_network.hpp"
#include "plane_implicit_urdf.h"
#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "utils/file_utils.hpp"
#include "math/matrix_utils.hpp"

constexpr int ANT_VARIABLE_SIZE = 3;
#include "locomotion_contact_simulation.h"
#include "../environments/ant_org_xyz_xyzrot.h"
//omp_model_ant_forward_zero_kernel
#include "omp_model_ant_forward_zero.h"

//variables: kp, kd, max_force









template <typename Algebra>
struct AntContactSimulation2   : public LocomotionContactSimulation<Algebra, ANT_VARIABLE_SIZE> {
    using Scalar = typename Algebra::Scalar;
    using Quaternion = typename Algebra::Quaternion;
    using Vector3 = typename Algebra::Vector3;

    std::string env_name() const {
      static std::string env_name = "ant";
      return env_name;
    }


    
    static const std::vector<Scalar> get_initial_poses() {
        
        static Scalar ant_knee_angle{-0.5};
        static Scalar ant_hip = Scalar(0);
        static std::vector<Scalar> ant_initial_poses = {
            ant_hip, ant_knee_angle,
            ant_hip, ant_knee_angle,
            ant_hip, ant_knee_angle,
            ant_hip, ant_knee_angle,
        };
        return ant_initial_poses;
    }


    AntContactSimulation2(bool urdf_from_file, 
          const std::string& urdf_filename, 
          const std::string& urdf_string,
          const std::vector<Scalar>& initial_poses,
          bool floating)
      :LocomotionContactSimulation<Algebra, ANT_VARIABLE_SIZE>(urdf_from_file, urdf_filename, urdf_string,initial_poses,floating, Scalar(0.01))
    {
        this->set_kp(Scalar(15));
        this->set_kd(Scalar(0.3));
        this->set_max_force(Scalar(3));
        this->m_start_base_position = Vector3 (Scalar(0),Scalar(0),Scalar(0.48));
        this->m_start_base_orientation = Quaternion (Scalar(0),Scalar(0),Scalar(0),Scalar(1));


    }
  
    

    void compute_reward_done(const std::vector<Scalar>& prev_state, const std::vector<Scalar>& cur_state, 
        Scalar& reward,
        bool& done) {
            
            
            Scalar vel_x = (cur_state[0] - prev_state[0])/this->dt;
            Scalar pos_z = cur_state[2];
            

            #if 0
            if (this->is_floating()) {
                ant_x = cur_state[4];
                ant_z = cur_state[6];
            } else {
                ant_x = cur_state[0];
                ant_z = cur_state[2];
            }
            #endif

            //reward forward along x-axis

            

            if(pos_z < 0.26) {
                done  = true;
                reward = 0;
                } else {
                done  = false;
                reward = vel_x;
                }
      
     }

    
  void reset(std::vector<Scalar>& sim_state, std::vector<Scalar>& observation) const {

        if (this->mb_->is_floating()) {
          sim_state[0] = this->m_start_base_orientation[0];
          sim_state[1] = this->m_start_base_orientation[1];
          sim_state[2] = this->m_start_base_orientation[2];
          sim_state[3] = this->m_start_base_orientation[3];
          sim_state[4] = this->m_start_base_position[0];
          sim_state[5] = this->m_start_base_position[1];
          sim_state[6] = this->m_start_base_position[2];
          int qoffset = 7;
          for (int j = 0; j < this->initial_poses_.size(); j++) {
            sim_state[j + qoffset] = this->initial_poses_[j];
          }
        } else {
          sim_state[0] = this->m_start_base_position[0];
          sim_state[1] = this->m_start_base_position[1];
          sim_state[2] = this->m_start_base_position[2];
          sim_state[3] = 0;
          sim_state[4] = 0;
          sim_state[5] = 0;
          int qoffset = 6;

          for (int j = 0; j < this->initial_poses_.size(); j++) {
                sim_state[j + qoffset] = this->initial_poses_[j] +  0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
          }
        }

        std::vector<Scalar> action(this->action_dim(), Scalar(0));
    
        Scalar reward;
        bool done;
        // @todo(erwincoumans): tune this
        int settle_down_steps = 10;


        sim_state[this->input_dim_with_action2()+0] = this->kp_;
        sim_state[this->input_dim_with_action2()+1] = this->kd_;
        sim_state[this->input_dim_with_action2()+2] = this->max_force_;

        std::vector<Scalar> output(this->output_dim());
        for (int i = 0; i < settle_down_steps; i++) {
            //this->step_forward_original(sim_state, output);
            this->forward_kernel<Scalar>(1, &output[0], &sim_state[0]);
            for (int i=0;i<this->input_dim();i++) {
                sim_state[i] = output[i];
            }
        }
        observation.resize(this->input_dim());
        for (int i=0;i<observation.size();i++) {
          observation[i] = sim_state[i];
        }
        //don't provide x and y coordinate
        observation[0] = 0;
        observation[1] = 0;

    }

    template <typename Scalar2>
    void forward_kernel(int num_total_threads, Scalar2* output,
                                            const Scalar2* input) const {
         omp_model_ant_forward_zero_kernel(1,output, input);
    }
    
};




template <typename Algebra>
struct AntEnv2 {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;
 


  AntContactSimulation2<Algebra> contact_sim;
  

  int observation_dim_{0};

  AntEnv2(bool urdf_from_file) 
      : contact_sim(urdf_from_file,
          "gym/ant_org_xyz_xyzrot.urdf",
          ant_org_xyz_xyzrot,
          AntContactSimulation2<Algebra>::get_initial_poses(),
          false) {
    observation_dim_ = contact_sim.input_dim();
    bool use_input_bias = false;
    neural_network.set_input_dim(observation_dim_, use_input_bias);
    bool learn_bias = true;
    neural_network.add_linear_layer(tds::NN_ACT_IDENTITY, 
                                    contact_sim.action_dim(),
                                    learn_bias);
  }
  virtual ~AntEnv2() {}

  void init_neural_network(const std::vector<Scalar>& x) {
    neural_network.set_parameters(x);
  }

  Vector3 m_start_base_position{Scalar(0),Scalar(0),Scalar(0.45)};
  Quaternion m_start_base_orientation{Scalar(0),Scalar(0),Scalar(0),Scalar(1)};

  std::vector<Scalar> sim_state;
  std::vector<Scalar> sim_state_with_graphics;
  Scalar target_angle;


  tds::NeuralNetwork<Algebra> neural_network;

  std::vector<Scalar> reset(const Scalar new_kp = 15, const Scalar new_kd = 0.3,
                            const Scalar target_angle_degrees = 0.) {
    sim_state.resize(0);
    sim_state.resize(contact_sim.input_dim(), Scalar(0));

    contact_sim.set_kp(Algebra::from_double(new_kp));
    contact_sim.set_kd(Algebra::from_double(new_kd));
    contact_sim.set_max_force(Algebra::from_double(3));
    
    target_angle = target_angle_degrees * M_PI / 180.;

    std::vector<Scalar> observation;
    observation.resize(contact_sim.output_dim());

    contact_sim.reset(sim_state, observation);

    return observation;
  }

  inline const std::vector<Scalar> policy(const std::vector<Scalar>& obs) {
    std::vector<Scalar> action(contact_sim.action_dim(), Scalar(0));

    neural_network.compute(obs, action);

    return action;
  }
};

#endif  // ANT_ENVIRONMENT2_H
