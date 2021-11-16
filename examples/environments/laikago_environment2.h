#ifndef LAIKAGO_ENVIRONMENT2_H
#define LAIKAGO_ENVIRONMENT2_H

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math.h"
#include "math/neural_network.hpp"
#include "plane_implicit_urdf.h"
#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "utils/file_utils.hpp"
#include "math/matrix_utils.hpp"
#include "omp_model_laikago_forward_zero.h"

//variables: kp, kd, max_force
constexpr int VARIABLE_SIZE = 3;

#include "locomotion_contact_simulation.h"



#include "laikago_toes_zup_urdf.h"

template <typename Scalar>
static const std::vector<Scalar> get_initial_poses() {
    Scalar laikago_knee_angle(-0.7);
    Scalar laikago_abduction_angle(0.2);
    Scalar zero(0);
    static const std::vector<Scalar> initial_poses_laikago3 = {
        laikago_abduction_angle, zero, laikago_knee_angle,
        laikago_abduction_angle, zero, laikago_knee_angle,
        laikago_abduction_angle, zero, laikago_knee_angle,
        laikago_abduction_angle, zero, laikago_knee_angle,
    };
    return initial_poses_laikago3;
}



template <typename Algebra>
struct LaikagoContactSimulation   : public LocomotionContactSimulation<Algebra> {
    using Scalar = typename Algebra::Scalar;
    using Quaternion = typename Algebra::Quaternion;
    using Vector3 = typename Algebra::Vector3;

    LaikagoContactSimulation(bool urdf_from_file, 
          const std::string& urdf_filename, 
          const std::string& urdf_string,
          const std::vector<Scalar>& initial_poses,
          bool floating)
      :LocomotionContactSimulation<Algebra>(urdf_from_file, urdf_filename, urdf_string,initial_poses,floating, Scalar(1e-3))
    {
    }
    
    Scalar kp_{Algebra::from_double(100)};
    Scalar kd_{Algebra::from_double(2)};
    Scalar max_force_{Algebra::from_double(50)};

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

    void compute_reward_done(std::vector<Scalar>& prev_state, std::vector<Scalar>& cur_state, 
        Scalar& reward,
        bool& done) {
            
            Scalar laikago_x;
            Scalar laikago_z;
            Scalar up_dot_world_z;
            

            if (this->is_floating()) {
                laikago_x = cur_state[4];
                laikago_z = cur_state[6];
                Quaternion base_orn(cur_state[0],cur_state[1],cur_state[2],cur_state[3]);
                auto base_mat = Algebra::quat_to_matrix(base_orn);
                up_dot_world_z = base_mat(2, 2);
            } else {
                laikago_x = cur_state[0];
                laikago_z = cur_state[2];
                Vector3 rpy(cur_state[3],cur_state[4],cur_state[5]);
                Quaternion base_orn = Algebra::quat_from_euler_rpy(rpy);
                auto base_mat = Algebra::quat_to_matrix(base_orn);
                up_dot_world_z = base_mat(2, 2);
            }

            //previously: forward along x-axis minus angles to keep chassis straight (get_euler_rpy?)
            //reward forward along x-axis
            reward = laikago_x;

            //Quaternion base_orn(cur_state[0],cur_state[1],cur_state[2],cur_state[3]);
            //auto base_mat = Algebra::quat_to_matrix(base_orn);
            //Scalar up_dot_world_z = base_mat(2, 2);
    
            // terminate if Laikago position/orientation becomes invalid
            done = (up_dot_world_z < 0.6 || ( laikago_z< 0.2));
      
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
                                            Scalar* output,
                                            const Scalar* input){

            omp_model_laikago_forward_zero_kernel<Scalar>(num_total_threads, output, input);
     }

};




template <typename Algebra>
struct LaikagoEnv {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;
 


  LaikagoContactSimulation<Algebra> contact_sim;
  

  int observation_dim_{0};
  LaikagoEnv(bool urdf_from_file) 
      : contact_sim(urdf_from_file,
          "laikago/laikago_toes_zup_xyz_xyzrot.urdf",//laikago_toes_zup.urdf",
          laikago_toes_zup_urdf,
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
  virtual ~LaikagoEnv() {}

  void init_neural_network(const std::vector<Scalar>& x) {
    neural_network.set_parameters(x);
  }

  Vector3 m_start_base_position{0,0,0.48};
  Quaternion m_start_base_orientation{0,0,0,1};

  std::vector<Scalar> sim_state;
  std::vector<Scalar> sim_state_with_graphics;
  Scalar target_angle;


  tds::NeuralNetwork<Algebra> neural_network;

  std::vector<Scalar> reset(const Scalar new_kp = 100., const Scalar new_kd = 2.,
                            const Scalar target_angle_degrees = 0.) {
    sim_state.resize(0);
    sim_state.resize(contact_sim.input_dim(), Scalar(0));
    contact_sim.set_kp(Algebra::from_double(new_kp));
    contact_sim.set_kd(Algebra::from_double(new_kd));
    contact_sim.set_max_force(Algebra::from_double(50.));
    

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
            sim_state[j + qoffset] =contact_sim.initial_poses_[j];//+  0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
      }
    }

    std::vector<Scalar> zero_action(contact_sim.action_dim(), Scalar(0));
    std::vector<Scalar> observation;
    Scalar reward;
    bool done;
    // @todo(erwincoumans): tune this
    int settle_down_steps = 10;
    for (int i = 0; i < settle_down_steps; i++) {
      step(zero_action, observation, reward, done);
    }

    return observation;
  }


  void step(std::vector<Scalar>& action, std::vector<Scalar>& obs,
            Scalar& reward, bool& done) {
    Scalar previous_x;
    // Get previous x position.
    // laikago_is_floating is assumed to be true.
    previous_x =
        sim_state[4] * cos(target_angle) + sim_state[5] * sin(target_angle);

    std::vector<Scalar> sim_state_with_action = sim_state;
    sim_state_with_action.resize(contact_sim.input_dim_with_action_and_variables());

    
    contact_sim.prepare_sim_state_with_action_and_variables(sim_state_with_action,action);

    sim_state_with_graphics = contact_sim.step_forward1(sim_state_with_action);
    
    //cuda_model_laikago_forward_zero_kernel(1,&sim_state_with_graphics[0], &sim_state_with_action[0]);
    //sim_state_with_graphics.resize(contact_sim.output_dim());
    //contact_sim.forward_kernel(1,&sim_state_with_graphics[0], &sim_state_with_action[0]);

    contact_sim.compute_reward_done(sim_state, sim_state_with_graphics, reward, done);
    sim_state = sim_state_with_graphics;
    sim_state.resize(contact_sim.input_dim());
    obs = sim_state;
  }

  inline const std::vector<Scalar> policy(const std::vector<Scalar>& obs) {
    std::vector<Scalar> action(contact_sim.action_dim(), Scalar(0));

    neural_network.compute(obs, action);

    return action;
  }
};

#endif  // LAIKAGO_ENVIRONMENT2_H