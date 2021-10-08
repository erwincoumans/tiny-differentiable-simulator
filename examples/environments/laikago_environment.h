#ifndef LAIKAGO_ENVIRONMENT_H
#define LAIKAGO_ENVIRONMENT_H

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "laikago_toes_zup_urdf.h"
#include "math.h"
#include "math/neural_network.hpp"
#include "plane_implicit_urdf.h"
#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "utils/file_utils.hpp"
#undef min
#undef max

static double start_pos1[3] = {0, 0, 0.48};
static double start_orn1[4] = {0, 0, 0, 1};

static bool laikago_is_floating = true;
static double laikago_knee_angle = -0.7;
static double laikago_abduction_angle = 0.2;
constexpr int LAIKAGO_POSE_SIZE = 12;

const std::vector<double> initial_poses_laikago2 = {
        laikago_abduction_angle, 0, laikago_knee_angle,
        laikago_abduction_angle, 0, laikago_knee_angle,
        laikago_abduction_angle, 0, laikago_knee_angle,
        laikago_abduction_angle, 0, laikago_knee_angle,
};

template <typename Algebra>
struct LaikagoContactSimulation {
  using Scalar = typename Algebra::Scalar;
  tds::UrdfCache<Algebra> cache;
  std::string m_laikago_urdf_filename;
  std::string m_laikago_search_path;
  tds::World<Algebra> world;
  tds::MultiBody<Algebra>* mb_ = nullptr;

  int num_timesteps{1};
  Scalar dt{Algebra::from_double(1e-3)};

  int input_dim() const { return mb_->dof() + mb_->dof_qd(); }

  int input_dim_with_action() const {
    return mb_->dof() + mb_->dof_qd() + LAIKAGO_POSE_SIZE;
  }

  int state_dim() const {
    return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7;
  }
  int output_dim() const { return state_dim(); }

  int action_dim_{LAIKAGO_POSE_SIZE};
  std::vector<Scalar> action_;

  LaikagoContactSimulation(bool urdf_from_file) {
    std::string plane_filename = "plane_impl";

    if (urdf_from_file) {
      tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
      cache.construct(plane_filename, world, false, false);

      //std::string urdf_name = "laikago/laikago_toes_zup_xyz_xyzrot.urdf";
      std::string urdf_name = "laikago/laikago_toes_zup.urdf";
      tds::FileUtils::find_file(urdf_name, m_laikago_urdf_filename);
      char laikago_search_path[TINY_MAX_EXE_PATH_LEN];
      tds::FileUtils::extract_path(m_laikago_urdf_filename.c_str(), laikago_search_path,
      TINY_MAX_EXE_PATH_LEN);
      m_laikago_search_path = laikago_search_path;

      mb_ = cache.construct(m_laikago_urdf_filename, world, false, laikago_is_floating);
    } else {
      std::string plane_string = plane_implicit_urdf;
      cache.construct_from_string(plane_filename, plane_string, world, false,
                                  false);

      m_laikago_urdf_filename = "laikago/laikago_toes_zup.urdf";
      std::string laikago_string = laikago_toes_zup_urdf;
      mb_ = cache.construct_from_string(m_laikago_urdf_filename, laikago_string, world,
                                        false, laikago_is_floating);
    }

    action_.resize(action_dim_);

    mb_->base_X_world().set_identity();
    world.default_friction = 1;
    world.get_mb_constraint_solver()->keep_all_points_ = true;
  }

  std::vector<Scalar> operator()(const std::vector<Scalar>& v) {
    std::vector<Scalar> action(action_dim_);
    for (int i = 0; i < action_dim_; i++) {
      action[i] = v[i + mb_->dof() + mb_->dof_qd()];
    }
    return (*this)(v, action);
  }

  std::vector<Scalar> operator()(const std::vector<Scalar>& v,
                                 const std::vector<Scalar>& action) {
    mb_->initialize();
    // copy input into q, qd
    for (int i = 0; i < mb_->dof(); ++i) {
      mb_->q(i) = v[i];
    }
    for (int i = 0; i < mb_->dof_qd(); ++i) {
      mb_->qd(i) = v[i + mb_->dof()];
    }

    for (int i = 0; i < action_dim_; i++) {
      action_[i] = action[i];
    }

    std::vector<Scalar> result(output_dim());
    for (int t = 0; t < num_timesteps; ++t) {
      if (1) {
        // use PD controller to compute tau
        int qd_offset = mb_->is_floating() ? 6 : 6;
        int q_offset = mb_->is_floating() ? 7 : 6;
        std::vector<double> q_targets;
        q_targets.resize(mb_->tau_.size());

        Scalar kp = Algebra::from_double(100.);
        Scalar kd = Algebra::from_double(2.);
        Scalar max_force = Algebra::from_double(50.);
        int param_index = 0;

        for (int i = 0; i < mb_->tau_.size(); i++) {
          mb_->tau_[i] = 0;
        }

        int pose_index = 0;
        int start_link = mb_->is_floating() ? 0 : 6;
        int tau_index = mb_->is_floating() ? 0 : 6;
        for (int i = start_link; i < mb_->links_.size(); i++) {
          if (mb_->links_[i].joint_type != tds::JOINT_FIXED) {
            if (pose_index < LAIKAGO_POSE_SIZE) {
              // clamp action
              Scalar clamped_action = action_[pose_index];
              Scalar ACTION_LIMIT = Algebra::from_double(0.4);
              clamped_action = Algebra::min(clamped_action, ACTION_LIMIT);
              clamped_action = Algebra::max(clamped_action, -ACTION_LIMIT);

              Scalar q_desired =
                  initial_poses_laikago2[pose_index++] + clamped_action;

              Scalar q_actual = mb_->q_[q_offset];
              Scalar qd_actual = mb_->qd_[qd_offset];
              Scalar position_error = (q_desired - q_actual);
              Scalar desired_velocity = Algebra::zero();
              Scalar velocity_error = (desired_velocity - qd_actual);
              Scalar force = kp * position_error + kd * velocity_error;

              force = Algebra::max(force, -max_force);
              force = Algebra::min(force, max_force);
              mb_->tau_[tau_index] = force;
              q_offset++;
              qd_offset++;
              param_index++;
              tau_index++;
            }
          }
        }
      }

      tds::forward_dynamics(*mb_, world.get_gravity());
      mb_->clear_forces();

      integrate_euler_qdd(*mb_, dt);

      world.step(dt);

      tds::integrate_euler(*mb_, dt);
    }
    // copy q, qd, link world poses (for rendering) to output
    int j = 0;
    for (int i = 0; i < mb_->dof(); ++i, ++j) {
      result[j] = mb_->q(i);
    }
    for (int i = 0; i < mb_->dof_qd(); ++i, ++j) {
      result[j] = mb_->qd(i);
    }
    for (const auto& link : *mb_) {
      // just copy the link world transform. Still have to multiple with visual
      // transform for each instance.
      if (link.X_visuals.size()) {
        auto visual_X_world = link.X_world * link.X_visuals[0];  //
        {
          result[j++] = visual_X_world.translation[0];
          result[j++] = visual_X_world.translation[1];
          result[j++] = visual_X_world.translation[2];
        }
        auto orn = Algebra::matrix_to_quat(visual_X_world.rotation);
        {
          result[j++] = orn.x();
          result[j++] = orn.y();
          result[j++] = orn.z();
          result[j++] = orn.w();
        }
      }
    }

    return result;
  }
};

template <typename Algebra>
struct LaikagoEnv {
  using Scalar = typename Algebra::Scalar;

  LaikagoContactSimulation<Algebra> contact_sim;

  int observation_dim_{0};
  LaikagoEnv(bool urdf_from_file) : contact_sim(urdf_from_file) {
    observation_dim_ = contact_sim.input_dim();
    bool use_input_bias = false;
    neural_network.set_input_dim(observation_dim_, use_input_bias);
    bool learn_bias = true;
    neural_network.add_linear_layer(tds::NN_ACT_IDENTITY, LAIKAGO_POSE_SIZE,
                                    learn_bias);
  }
  virtual ~LaikagoEnv() {}

  void init_neural_network(const std::vector<double>& x) {
    neural_network.set_parameters(x);
  }

  std::vector<Scalar> sim_state;
  std::vector<Scalar> sim_state_with_graphics;

  tds::NeuralNetwork<Algebra> neural_network;

  std::vector<double> reset() {
    sim_state.resize(0);
    sim_state.resize(contact_sim.input_dim(), Scalar(0));

    if (contact_sim.mb_->is_floating()) {
      sim_state[0] = start_orn1[0];
      sim_state[1] = start_orn1[1];
      sim_state[2] = start_orn1[2];
      sim_state[3] = start_orn1[3];
      sim_state[4] = start_pos1[0];
      sim_state[5] = start_pos1[1];
      sim_state[6] = start_pos1[2];
      int qoffset = 7;
      for (int j = 0; j < LAIKAGO_POSE_SIZE; j++) {
        sim_state[j + qoffset] =
            initial_poses_laikago2[j];
      }
    } else {
      sim_state[0] = start_pos1[0];
      sim_state[1] = start_pos1[1];
      sim_state[2] = start_pos1[2];
      sim_state[3] = 0;
      sim_state[4] = 0;
      sim_state[5] = 0;

      int qoffset = 6;
      for (int j = 0; j < LAIKAGO_POSE_SIZE; j++) {
        sim_state[j + qoffset] =
            initial_poses_laikago2[j];  // 0.05*((std::rand() * 1. /
                                        // RAND_MAX)-0.5)*2.0;
      }
    }

    std::vector<double> zero_action(LAIKAGO_POSE_SIZE, Scalar(0));
    std::vector<double> observation;
    double reward;
    bool done;
    // @todo(erwincoumans): tune this
    int settle_down_steps = 100;
    for (int i = 0; i < settle_down_steps; i++) {
      step(zero_action, observation, reward, done);
    }

    return observation;
  }
  void step(std::vector<double>& action, std::vector<double>& obs,
            double& reward, bool& done) {
    sim_state_with_graphics = contact_sim(sim_state, action);
    sim_state = sim_state_with_graphics;

    sim_state.resize(contact_sim.input_dim());
    obs = sim_state;
    reward = 1;
    auto base_tr = contact_sim.mb_->get_world_transform(-1);

    // reward forward along x-axis
    reward = sim_state[4];
    if (std::isnan(reward) || std::isinf(reward) || (reward != reward)) {
      reward = 0;
      done = true;
    }

    Scalar up_dot_world_z = base_tr.rotation(2, 2);
    
    // Laikago torso height needs to be in range 0.3 to 1. meter
    if (up_dot_world_z < 0.6 || (sim_state[6] < 0.2) || (sim_state[6] > 5.)) {
      done = true;
    } else {
      done = false;
    }
  }

  inline const std::vector<double> policy(const std::vector<double>& obs) {
    std::vector<double> action(LAIKAGO_POSE_SIZE, Scalar(0));

    neural_network.compute(obs, action);

    return action;
  }
};

#endif  // LAIKAGO_ENVIRONMENT_H
