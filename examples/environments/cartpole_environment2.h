#ifndef CARTPOLE_ENVIRONMENT_H
#define CARTPOLE_ENVIRONMENT_H

#include "math/neural_network.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "urdf/urdf_cache.hpp"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf_structures.hpp"
#include "cartpole_urdf.h"
#include "math.h"

//#define COMPATIBILITY
template <typename Algebra>
struct CartpoleContactSimulation {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Transform = typename Algebra::Transform;

  tds::UrdfCache<Algebra> cache;
  std::string m_urdf_filename;
  tds::World<Algebra> world;
  tds::MultiBody<Algebra>* mb_ = nullptr;

  int num_timesteps{1};
  Scalar dt{Algebra::from_double(1. / 60.)};

  int input_dim() const { return mb_->dof() + mb_->dof_qd(); }
  int state_dim() const {
    return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7;
  }
  int output_dim() const { return num_timesteps * state_dim(); }

  CartpoleContactSimulation() {
    std::string plane_filename;
    world.set_gravity(Vector3(0., 0., -10));

    //tds::StdLogger logger;
    tds::NullLogger logger;
    {
      m_urdf_filename = "cartpole.urdf";
      int flags = 0;
      tds::UrdfStructures<Algebra> urdf_structures;
      std::string urdf_string = sCartPole;
      if (tds::UrdfParser<Algebra>::load_urdf_from_string(
              urdf_string, flags, logger, urdf_structures)) {
        mb_ = world.create_multi_body(m_urdf_filename);
        tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_structures,
                                                             world, *mb_, 0);
        bool is_floating = false;
        mb_->set_floating_base(is_floating);
        mb_->initialize();
      }
    }

    mb_->base_X_world().translation = Algebra::unit3_z();
    //std::cout << "CartpoleContactSimulation!" << std::endl;
  }

  virtual ~CartpoleContactSimulation() {
    //std::cout << "~CartpoleContactSimulation" << std::endl;
  }
  std::vector<Scalar> operator()(const std::vector<Scalar>& v,
                                 Scalar tau = 0.) {
    assert(static_cast<int>(v.size()) == input_dim());
    mb_->initialize();
    // copy input into q, qd
    for (int i = 0; i < mb_->dof(); ++i) {
      mb_->q(i) = v[i];
    }
    for (int i = 0; i < mb_->dof_qd(); ++i) {
      mb_->qd(i) = v[i + mb_->dof()];
    }

    static double t = 0;
    // printf("t=%f, [%f,%f]\n",t,mb_->q(0),mb_->q(1));
    t += dt;
    std::vector<Scalar> result(output_dim());
    for (int t = 0; t < num_timesteps; ++t) {
      mb_->tau_[0] = tau;
      mb_->tau_[1] = 0;

      // std::vector<Scalar> tau = policy(observation)

      tds::forward_dynamics(*mb_, world.get_gravity());
      mb_->clear_forces();

      //integrate_euler_qdd(*mb_, dt);

      //world.step(dt);

      tds::integrate_euler(*mb_, dt);

      // copy q, qd, link world poses (for rendering) to output
      int j = 0;
      for (int i = 0; i < mb_->dof(); ++i, ++j) {
        result[j] = mb_->q(i);
      }
      for (int i = 0; i < mb_->dof_qd(); ++i, ++j) {
        result[j] = mb_->qd(i);
      }
#if 1
      for (const auto& link : *mb_) {
        if (link.X_visuals.size()) {
          Transform visual_X_world = link.X_world * link.X_visuals[0];
          result[j++] = visual_X_world.translation[0];
          result[j++] = visual_X_world.translation[1];
          result[j++] = visual_X_world.translation[2];
          auto orn = Algebra::matrix_to_quat(visual_X_world.rotation);
          result[j++] = orn[0];
          result[j++] = orn[1];
          result[j++] = orn[2];
          result[j++] = orn[3];
        } else {
          // check if we have links without visuals
          assert(0);
          j += 7;
        }
      }
#endif
    }
    return result;
  }
};

struct CartpoleEnvOutput
{
    std::vector<double> obs;
    double reward;
    bool done;
    std::array<double,3> cart_graphics_pos;
    std::array<double,4> cart_graphics_orn;
    
    std::array<double,3> pole_graphics_pos;
    std::array<double,4> pole_graphics_orn;
};

struct CartpoleRolloutOutput
{
    CartpoleRolloutOutput()
        :total_reward(0),
        num_steps(0)
    {
    }
    double total_reward;
    int num_steps;
};


template <typename Algebra>
struct CartpoleEnv {
  using Scalar = typename Algebra::Scalar;
  CartpoleContactSimulation<Algebra> contact_sim;
  Scalar action_low_;
  Scalar action_high_;

  int action_dim_{1};
  int observation_dim_{4};
  int counter_{-1};
  CartpoleEnv() {
    bool use_input_bias = false;
    static int counter=0;
    counter_ = counter++;
    printf("CartPoleEnv counter_=%d\n", counter_);
    int observation_size = contact_sim.input_dim();
    neural_network.set_input_dim(observation_size, use_input_bias);
    bool learn_bias = true;
    neural_network.add_linear_layer(tds::NN_ACT_IDENTITY, 1,
                                    learn_bias);  // action is 1 number

    action_high_ = 10;
    action_low_ = -action_high_;
  }
  virtual ~CartpoleEnv() {
      printf("~CartPoleEnv counter_=%d\n", counter_);
  }

  std::vector<Scalar> sim_state;
  std::vector<Scalar> sim_state_with_graphics;

  std::vector<double> reset() {
    //std::srand(0);
    int input_dim = contact_sim.input_dim();
    sim_state.resize(input_dim);
    for (int i = 0; i < sim_state.size(); i++) {
      sim_state[i] = 0.05 * ((std::rand() * 1. / RAND_MAX) - 0.5) * 2.0;
    }
     //for (auto v : sim_state)
     //    std::cout << v << ",";
    //std::cout << std::endl;
    return sim_state;
  }

  std::vector<double> reset2() {
    
    std::vector<double> obs = reset();
    std::vector<double> sim_state;
    // change layout to [q1,qd1, q0, qd0] to be compatible with
#ifdef COMPATIBILITY
#error
    sim_state.push_back(obs[1]);
    sim_state.push_back(obs[3]);
    sim_state.push_back(obs[0]);
    sim_state.push_back(obs[2]);
#else
    sim_state.push_back(obs[0]);
    sim_state.push_back(obs[1]);
    sim_state.push_back(obs[2]);
    sim_state.push_back(obs[3]);
#endif
    
    return sim_state;
  }

  void seed(long long int s) {
      //std::cout<<"seed:" << s << std::endl;
      std::srand(s);
  }

  CartpoleEnvOutput step2(double action) {
      //std::cout << "action:" << action << std::endl;

      CartpoleEnvOutput env_out;
      
      std::vector<double> obs;
      step(action, obs, env_out.reward, env_out.done);
      // obs in format [q0,q1,qd0,qd1]
      // change layout to [q1,qd1, q0, qd0] to be compatible with
      // PyBullet'CartPoleContinuousBulletEnv-v0'
#ifdef COMPATIBILITY
#error
      env_out.obs.push_back(obs[1]);
      env_out.obs.push_back(obs[3]);
      env_out.obs.push_back(obs[0]);
      env_out.obs.push_back(obs[2]);
#else
      env_out.obs.push_back(obs[0]);
      env_out.obs.push_back(obs[1]);
      env_out.obs.push_back(obs[2]);
      env_out.obs.push_back(obs[3]);
#endif
      //std::cout << "env_out.done=" << env_out.done << std::endl;
      //std::cout << "env_out.reward=" << env_out.done << std::endl;
      //std::cout << "obs=" << obs << std::endl;
      int index=4;
      env_out.cart_graphics_pos[0] = this->sim_state_with_graphics[index++];
      env_out.cart_graphics_pos[1] = this->sim_state_with_graphics[index++];
      env_out.cart_graphics_pos[2] = this->sim_state_with_graphics[index++];
      env_out.cart_graphics_orn[0] = this->sim_state_with_graphics[index++];
      env_out.cart_graphics_orn[1] = this->sim_state_with_graphics[index++];
      env_out.cart_graphics_orn[2] = this->sim_state_with_graphics[index++];
      env_out.cart_graphics_orn[3] = this->sim_state_with_graphics[index++];

      env_out.pole_graphics_pos[0] = this->sim_state_with_graphics[index++];
      env_out.pole_graphics_pos[1] = this->sim_state_with_graphics[index++];
      env_out.pole_graphics_pos[2] = this->sim_state_with_graphics[index++];
      env_out.pole_graphics_orn[0] = this->sim_state_with_graphics[index++];
      env_out.pole_graphics_orn[1] = this->sim_state_with_graphics[index++];
      env_out.pole_graphics_orn[2] = this->sim_state_with_graphics[index++];
      env_out.pole_graphics_orn[3] = this->sim_state_with_graphics[index++];
      return env_out;
  }
  
  CartpoleRolloutOutput rollout(int rollout_length, double shift) {
      //std::cout << "action:" << action << std::endl;

      CartpoleRolloutOutput rollout_out;
      std::vector<double> obs = reset();
      bool done = false;
      int steps=0;
      while (rollout_out.num_steps<rollout_length && !done)
      {
         //double action = 0.f;
         auto action = policy(obs);
         double reward;
         step(action, obs, reward, done);
         rollout_out.total_reward += reward;
         rollout_out.num_steps++;
      }
      return rollout_out;
  }
  

  void step(double action, std::vector<double>& obs, double& reward,
            bool& done) {
    //clip
    if (action < action_low_) 
        action = action_low_;
    if (action > action_high_) 
        action = action_high_;

    // sim_state = [q0, q1, qd0, qd1]

    sim_state_with_graphics = contact_sim(sim_state, action);
    sim_state = sim_state_with_graphics;

    sim_state.resize(contact_sim.input_dim());
    obs = sim_state;
    reward = 1;
    double x = sim_state[0];
    double theta = sim_state[1];
    double theta_threshold_radians = 12. * 2. * M_PI / 360.;
    double x_threshold = 0.4;  //  #2.4
    done = (x < -x_threshold) || (x > x_threshold) ||
           (theta < -theta_threshold_radians) ||
           (theta > theta_threshold_radians);
  }

  tds::NeuralNetwork<Algebra> neural_network;

  void init_neural_network(const std::vector<double>& x) {
    neural_network.set_parameters(x);
  }

  inline double policy(const std::vector<double>& obs) {
    std::vector<double> action;
    neural_network.compute(obs, action);

    //normalize actions
    
    for (int i=0;i<action.size();i++)
    {

        if (action[i]<-1.0)
            action[i]=-1.0;
        if (action[i]>1.0)
            action[i]=1.0;
        action[i] *= (action_high_ - action_low_)/2.0;
        action[i] += (action_low_ + action_high_)/2.0;

    }

    return action[0];
  }

  inline double policy2(const std::vector<double>& x,
                        const std::vector<double>& obs) {
    double action = 0;

    for (int i = 0; i < 4; i++) {
      action += x[i] * obs[i];  // identity activation
    }
    action += x[4];  // bias

    if (action<-1.0)
        action=-1.0;
    if (action>1.0)
        action=1.0;
    action *= (action_high_ - action_low_)/2.0;
    action += (action_low_ + action_high_)/2.0;

    return action;
  }
};

#endif  // CARTPOLE_ENVIRONMENT_H
