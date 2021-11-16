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

constexpr int VARIABLE_SIZE = 3;
#include "locomotion_contact_simulation.h"
//omp_model_ant_forward_zero_kernel
#include "omp_model_ant_forward_zero.h"

//variables: kp, kd, max_force






template <typename Scalar>
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



template <typename Algebra>
struct AntContactSimulation   : public LocomotionContactSimulation<Algebra> {
    using Scalar = typename Algebra::Scalar;
    using Quaternion = typename Algebra::Quaternion;
    using Vector3 = typename Algebra::Vector3;
    AntContactSimulation(bool urdf_from_file, 
          const std::string& urdf_filename, 
          const std::string& urdf_string,
          const std::vector<Scalar>& initial_poses,
          bool floating)
      :LocomotionContactSimulation<Algebra>(urdf_from_file, urdf_filename, urdf_string,initial_poses,floating, Scalar(0.01))
    {
    }
  
  Scalar kp_{Algebra::from_double(15)};
  Scalar kd_{Algebra::from_double(0.3)};
  Scalar max_force_{Algebra::from_double(3.)};

  void set_kp(const Scalar new_kp) { 
      kp_ = new_kp; 
  }

  void set_kd(const Scalar new_kd) { 
      kd_ = new_kd; 
  }
  
  void set_max_force(const Scalar new_max_force) { 
      max_force_ = new_max_force; 
  }


    
   void prepare_sim_state_with_action_and_variables(std::vector<Scalar>& v, const std::vector<Scalar>& actions) {
        
        for (int i=0;i<actions.size();i++)
        {
            v[i+this->input_dim()] = actions[i];
        }

        v[this->input_dim_with_action2()+0] = kp_;
        v[this->input_dim_with_action2()+1] = kd_;
        v[this->input_dim_with_action2()+2] = max_force_;
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

  std::vector<Scalar> step_forward1(const std::vector<Scalar>& v) {
      std::vector<Scalar> action(this->action_dim_);
      for (int i = 0; i < this->action_dim_; i++) {
        action[i] = v[i + this->mb_->dof() + this->mb_->dof_qd()];
      }
      std::vector<Scalar> variables(this->variables_dim_);
      variables[0] = v[this->mb_->dof() + this->mb_->dof_qd() + this->action_dim() + 0];//kp_
      variables[1] = v[this->mb_->dof() + this->mb_->dof_qd() + this->action_dim() + 1];//kd_
      variables[2] = v[this->mb_->dof() + this->mb_->dof_qd() + this->action_dim() + 2];//max_force_
      return this->step_forward3(v, action, variables);
  }

    
    void forward_kernel(int num_total_threads,
                                            double *output,
                                            const double *input){
         omp_model_ant_forward_zero_kernel(1,output, input);
    }
    
};




template <typename Algebra>
struct AntEnv {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;
 


  AntContactSimulation<Algebra> contact_sim;
  

  int observation_dim_{0};
  AntEnv(bool urdf_from_file) 
      : contact_sim(urdf_from_file,
          "gym/ant_org_xyz_xyzrot.urdf",
          "",
          get_initial_poses<Scalar>(),
          false) {
    observation_dim_ = contact_sim.input_dim();
    bool use_input_bias = false;
    neural_network.set_input_dim(observation_dim_, use_input_bias);
    bool learn_bias = true;
    neural_network.add_linear_layer(tds::NN_ACT_IDENTITY, 
                                    contact_sim.action_dim(),
                                    learn_bias);
  }
  virtual ~AntEnv() {}

  void init_neural_network(const std::vector<Scalar>& x) {
    neural_network.set_parameters(x);
  }

  Vector3 m_start_base_position{0,0,0.45};
  Quaternion m_start_base_orientation{0,0,0,1};

  std::vector<Scalar> sim_state;
  std::vector<Scalar> sim_state_with_graphics;
  Scalar target_angle;


  tds::NeuralNetwork<Algebra> neural_network;
  #if 0
  std::vector<Scalar> reset2(const Scalar new_kp = 15, const Scalar new_kd = 0.3,
                            const Scalar target_angle_degrees = 0.) {
    sim_state.resize(0);
    sim_state.resize(contact_sim.input_dim(), Scalar(0));
    contact_sim.set_kp(Algebra::from_double(new_kp));
    contact_sim.set_kd(Algebra::from_double(new_kd));
    contact_sim.set_max_force(Algebra::from_double(3));

    target_angle = target_angle_degrees * M_PI / 180.;

    if (contact_sim.mb_->is_floating()) {
      sim_state[0] = m_start_base_orientation[0];
      sim_state[1] = m_start_base_orientation[1];
      sim_state[2] = m_start_base_orientation[2];
      sim_state[3] = m_start_base_orientation[3];
      sim_state[4] = m_start_base_position[0];
      sim_state[5] = m_start_base_position[1];
      sim_state[6] = m_start_base_position[2];
      int qoffset = 7;
      for (int j = 0; j < contact_sim.initial_poses_.size(); j++) {
        sim_state[j + qoffset] = contact_sim.initial_poses_[j];
      }
    } else {
      sim_state[0] = m_start_base_position[0];
      sim_state[1] = m_start_base_position[1];
      sim_state[2] = m_start_base_position[2];
      sim_state[3] = 0;
      sim_state[4] = 0;
      sim_state[5] = 0;
      int qoffset = 6;

      for (int j = 0; j < contact_sim.initial_poses_.size(); j++) {
        sim_state[j + qoffset] =
            contact_sim.initial_poses_[j]+ 0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
      }
    }

    std::vector<Scalar> zero_action(contact_sim.action_dim(), Scalar(0));
    std::vector<Scalar> observation;
    Scalar reward;
    bool done;
    // @todo(erwincoumans): tune this
    int settle_down_steps = 50;
    for (int i = 0; i < settle_down_steps; i++) {
      step(zero_action, observation, reward, done);
    }

    return observation;
  }

  #endif


  inline const std::vector<Scalar> policy(const std::vector<Scalar>& obs) {
    std::vector<Scalar> action(contact_sim.action_dim(), Scalar(0));

    neural_network.compute(obs, action);

    return action;
  }
};

#endif  // ANT_ENVIRONMENT2_H