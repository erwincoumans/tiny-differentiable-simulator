#ifndef CARTPOLE_ENVIRONMENT_H
#define CARTPOLE_ENVIRONMENT_H

#include "cartpole_urdf.h"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math/neural_network.hpp"
#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf_structures.hpp"
#include "utils/file_utils.hpp"

template <typename Algebra>
struct ContactSimulation {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Transform = typename Algebra::Transform;

  tds::UrdfCache<Algebra> cache;
  std::string m_urdf_filename;
  tds::World<Algebra> world;
  tds::MultiBody<Algebra>* mb_ = nullptr;
  tds::UrdfStructures<Algebra> all_urdf_structures;

  int num_timesteps{1};
  Scalar dt{Algebra::from_double(1. / 60.)};
  double angle_offset_{0.0};

  double angle_offset() const { return angle_offset_; }

  void set_angle(const double new_angle_offset) {
    angle_offset_ = new_angle_offset;
    const double angle_tangent = std::tan(angle_offset_);
    const double gravity_z =
        -9.8 / std::sqrt(1. + angle_tangent * angle_tangent);
    const double gravity_x = gravity_z * angle_tangent;
    world.set_gravity(Vector3(gravity_x, 0., gravity_z));
  }

  int input_dim() const { return mb_->dof() + mb_->dof_qd(); }
  int state_dim() const {
    return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7;
  }
  int output_dim() const { return num_timesteps * state_dim(); }

  ContactSimulation() {
    std::string plane_filename;
    world.set_gravity(Vector3(0., 0., -10));

    tds::NullLogger logger;
    {
      int flags = 0;
      tds::UrdfStructures<Algebra> urdf_structures;
      std::string urdf_string = sCartPole;
      if (tds::UrdfParser<Algebra>::load_urdf_from_string(
              urdf_string, flags, logger, urdf_structures)) {
        mb_ = world.create_multi_body("cartpole");
        tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_structures,
                                                             world, *mb_, 0);
        bool is_floating = false;
        mb_->set_floating_base(is_floating);
        mb_->initialize();
      }

      all_urdf_structures = urdf_structures;
    }

    mb_->base_X_world().translation = Algebra::unit3_z();
  }

  virtual ~ContactSimulation() {}
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
    t += dt;
    std::vector<Scalar> result(output_dim());
    for (int t = 0; t < num_timesteps; ++t) {
      mb_->tau_[0] = tau;
      mb_->tau_[1] = 0;

      tds::forward_dynamics(*mb_, world.get_gravity());
      mb_->clear_forces();

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
  using Vector3 = typename Algebra::Vector3;
  ContactSimulation<Algebra>& contact_sim;
  const int simulation_angle_index = 1;
  int reward_steps_remaining = 5000;

  explicit CartpoleEnv(ContactSimulation<Algebra>& cartpole)
      : contact_sim(cartpole) {
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

  std::vector<double> reset(const double angle_offset_degrees = 0.0) {
    const double angle_offset = angle_offset_degrees * M_PI / 180.;
    contact_sim.set_angle(angle_offset);
    sim_state.resize(contact_sim.input_dim());
    for (int i = 0; i < sim_state.size(); i++) {
      sim_state[i] = 0.05 * ((std::rand() * 1. / RAND_MAX) - 0.5) * 2.0;
    }
    sim_state[simulation_angle_index] += angle_offset;
    return sim_state;
  }

  void change_angle(const double angle_offset_degrees) {
    const double prev_angle = contact_sim.angle_offset();
    const double angle_offset = angle_offset_degrees * M_PI / 180.;
    contact_sim.set_angle(angle_offset);
    sim_state[simulation_angle_index] += (angle_offset - prev_angle);
  }

  void set_reward_steps_remaining(int reward_steps) {
    reward_steps_remaining = reward_steps;
  }

  void step(double action, std::vector<double>& obs, double& reward,
            bool& done) {
    // clamp and scale action

    if (action < -1) action = -1;
    if (action > 1) action = 1;
    action *= 10;

    sim_state_with_graphics = contact_sim(sim_state, action);
    sim_state = sim_state_with_graphics;

    sim_state.resize(contact_sim.input_dim());
    obs = sim_state;
    double x = obs[0];
    double theta = obs[1];
    double theta_deviation = std::abs(theta - contact_sim.angle_offset());
    double theta_threshold_radians = 12. * 2. * M_PI / 360.;
    reward = std::max(0., 1 - theta_deviation / theta_threshold_radians);
    reward = reward * reward;

    if (reward_steps_remaining <= 0) {
      reward *= 0;
    }
    reward_steps_remaining--;

    double x_threshold = 0.4;
    done = (x < -x_threshold) || (x > x_threshold) ||
           (theta_deviation > theta_threshold_radians);
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
