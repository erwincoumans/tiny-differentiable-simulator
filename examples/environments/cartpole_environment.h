#ifndef CARTPOLE_ENVIRONMENT_H
#define CARTPOLE_ENVIRONMENT_H

#include "math/neural_network.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "urdf/urdf_cache.hpp"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "cartpole_urdf.h"

template <typename Algebra>
struct ContactSimulation {
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

  ContactSimulation() {
    std::string plane_filename;
    world.set_gravity(Vector3(0., 0., -10));

    tds::StdLogger logger;
    // NullLogger logger;
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
    // std::cout << "ContactSimulation!" << std::endl;
  }

  virtual ~ContactSimulation() {
    // std::cout << "~ContactSimulation" << std::endl;
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

      integrate_euler_qdd(*mb_, dt);

      world.step(dt);

      tds::integrate_euler(*mb_, dt);

      // copy q, qd, link world poses (for rendering) to output
      int j = 0;
      for (int i = 0; i < mb_->dof(); ++i, ++j) {
        result[j] = mb_->q(i);
      }
      for (int i = 0; i < mb_->dof_qd(); ++i, ++j) {
        result[j] = mb_->qd(i);
      }
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
    }
    return result;
  }
};

template <typename Algebra>
struct CartpoleEnv {
  using Scalar = typename Algebra::Scalar;
  ContactSimulation<Algebra>& contact_sim;

  CartpoleEnv(ContactSimulation<Algebra>& cartpole) : contact_sim(cartpole) {
    bool use_input_bias = false;
    int observation_size = contact_sim.input_dim();
    neural_network.set_input_dim(observation_size, use_input_bias);
    bool learn_bias = true;
    neural_network.add_linear_layer(tds::NN_ACT_IDENTITY, 1,
                                    learn_bias);  // action is 1 number
  }
  virtual ~CartpoleEnv() {}

  std::vector<Scalar> sim_state;
  std::vector<Scalar> sim_state_with_graphics;

  std::vector<double> reset() {
    sim_state.resize(contact_sim.input_dim());
    for (int i = 0; i < sim_state.size(); i++) {
      sim_state[i] = 0.05 * ((std::rand() * 1. / RAND_MAX) - 0.5) * 2.0;
    }
    // for (auto v : sim_state)
    //     std::cout << v << std::endl;
    return sim_state;
  }

  void step(double action, std::vector<double>& obs, double& reward,
            bool& done) {
    // clamp and scale action

    if (action < -1) action = -1;
    if (action > 1) action = 1;
    action *= 10;

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
    return action[0];
  }

  inline double policy2(const std::vector<double>& x,
                        const std::vector<double>& obs) {
    double action = 0;

    for (int i = 0; i < 4; i++) {
      action += x[i] * obs[i];  // identity activation
    }
    action += x[4];  // bias

    return action;
  }
};

#endif  // CARTPOLE_ENVIRONMENT_H
