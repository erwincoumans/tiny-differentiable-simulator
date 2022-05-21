#ifndef LAIKAGO_ENVIRONMENT2_H
#define LAIKAGO_ENVIRONMENT2_H

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math.h"
#include "math/neural_network.hpp"

#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "utils/file_utils.hpp"
#include "math/matrix_utils.hpp"
#include "omp_model_laikago_forward_zero.h"

//variables: kp, kd, max_force
constexpr int LAIKAGO_VARIABLE_SIZE = 3;

#include "locomotion_contact_simulation.h"
//#include "laikago_toes_zup_xyz_xyzrot_urdf.h"
#include "laikago_toes_zup_xyz_xyzrot.h"



template <typename Algebra>
struct LaikagoContactSimulation   :
    public LocomotionContactSimulation<Algebra, LAIKAGO_VARIABLE_SIZE> {
    using Scalar = typename Algebra::Scalar;
    using Quaternion = typename Algebra::Quaternion;
    using Vector3 = typename Algebra::Vector3;

    std::string env_name() const {
      static std::string env_name = "laikago";
      return env_name;
    }

    LaikagoContactSimulation(bool urdf_from_file, 
          const std::string& urdf_filename, 
          const std::string& urdf_string,
          const std::vector<Scalar>& initial_poses,
          bool floating)
      :LocomotionContactSimulation<Algebra, LAIKAGO_VARIABLE_SIZE>(urdf_from_file, urdf_filename, urdf_string,initial_poses,floating, Scalar(1e-3))
    {
            this->set_kp(Scalar(100));
            this->set_kd(Scalar(2));
            this->set_max_force(Scalar(50));
            this->m_start_base_position = Vector3(Scalar(0),Scalar(0),Scalar(0.48));
            this->m_start_base_orientation = Quaternion (Scalar(0),Scalar(0),Scalar(0),Scalar(1));
    }

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

        for (int i=0;i<this->input_dim();i++) {
            observation[i] = output[i];    
        }
    }
    
   void prepare_sim_state_with_action_and_variables(std::vector<Scalar>& v, const std::vector<Scalar>& actions) {
        
        for (int i=0;i<actions.size();i++)
        {
            v[i+this->input_dim()] = actions[i];
        }

        v[this->input_dim_with_action2()+0] = this->kp_;
        v[this->input_dim_with_action2()+1] = this->kd_;
        v[this->input_dim_with_action2()+2] = this->max_force_;
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
                laikago_x = cur_state[0];//(cur_state[0]-prev_state[0])/this->dt;
                laikago_z = cur_state[2];
                Vector3 rpy(cur_state[3],cur_state[4],cur_state[5]);
                Quaternion base_orn = Algebra::quat_from_euler_rpy(rpy);
                auto base_mat = Algebra::quat_to_matrix(base_orn);
                up_dot_world_z = base_mat(2, 2);
            }

            

            //Quaternion base_orn(cur_state[0],cur_state[1],cur_state[2],cur_state[3]);
            //auto base_mat = Algebra::quat_to_matrix(base_orn);
            //Scalar up_dot_world_z = base_mat(2, 2);
    
            // terminate if Laikago position/orientation becomes invalid
            done = (up_dot_world_z < 0.6 || ( laikago_z< 0.2));
      
            //previously: forward along x-axis minus angles to keep chassis straight (get_euler_rpy?)
            //reward forward along x-axis
            if (done) {
                reward = 0;
            } else 
            {
                reward = laikago_x;
            }
     }


    template <typename Scalar2>
    void forward_kernel(int num_total_threads, Scalar2* output,
                                            const Scalar2* input) const {

            omp_model_laikago_forward_zero_kernel<Scalar2>(num_total_threads, output, input);
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
          "laikago/laikago_toes_zup_xyz_xyzrot.urdf",
                   laikago_toes_zup_xyz_xyzrot,
          LaikagoContactSimulation<Algebra>::get_initial_poses(),
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

  std::vector<Scalar> sim_state;
  std::vector<Scalar> sim_state_with_graphics;
  Scalar target_angle;


  tds::NeuralNetwork<Algebra> neural_network;

  std::vector<Scalar> reset(const Scalar new_kp = 100., const Scalar new_kd = 2.,
                            const Scalar target_angle_degrees = 0.) {
    sim_state.resize(0);
    sim_state.resize(contact_sim.input_dim_with_action_and_variables(), Scalar(0));

    contact_sim.set_kp(Algebra::from_double(new_kp));
    contact_sim.set_kd(Algebra::from_double(new_kd));
    contact_sim.set_max_force(Algebra::from_double(50.));
    
    target_angle = target_angle_degrees * M_PI / 180.;

    std::vector<Scalar> observation;
    observation.resize(contact_sim.input_dim());

    contact_sim.reset(sim_state, observation);

    return observation;
  }


  void step(std::vector<Scalar>& action, std::vector<Scalar>& obs,
            Scalar& reward, bool& done) {
    std::vector<Scalar> sim_state_with_action = sim_state;
    sim_state_with_action.resize(contact_sim.input_dim_with_action_and_variables());
    
    contact_sim.prepare_sim_state_with_action_and_variables(sim_state_with_action,action);


    sim_state_with_graphics.resize(contact_sim.output_dim());
    contact_sim.step_forward_original(sim_state_with_action, sim_state_with_graphics );
    
    //cuda_model_laikago_forward_zero_kernel(1,&sim_state_with_graphics[0], &sim_state_with_action[0]);
    
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
