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

void cuda_model_laikago_forward_zero_kernel(int num_total_threads,
                                            double *output,
                                            const double *input);

static double start_pos1[3] = {0, 0, 0.48};
static double start_orn1[4] = {0, 0, 0, 1};

static bool laikago_is_floating = true;
static double laikago_knee_angle = -0.7;
static double laikago_abduction_angle = 0.2;
constexpr int LAIKAGO_POSE_SIZE = 12;
//variables: kp, kd, max_force
constexpr int VARIABLE_SIZE = 3;

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
  Scalar kp_{Algebra::from_double(100.)};
  Scalar kd_{Algebra::from_double(2.)};
  Scalar max_force_{Algebra::from_double(50.)};

  void set_kp(const Scalar new_kp) { kp_ = new_kp; }

  void set_kd(const Scalar new_kd) { kd_ = new_kd; }

  int input_dim() const { return mb_->dof() + mb_->dof_qd(); }

  int input_dim_with_action_and_variables() const {
    return mb_->dof() + mb_->dof_qd() + LAIKAGO_POSE_SIZE + VARIABLE_SIZE;
  }

  // input, world transforms for all links and up_dot_world_z
  int state_dim() const {
    return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7 + 1;
  }
  int output_dim() const { return state_dim(); }

  int action_dim_{LAIKAGO_POSE_SIZE};
  
  int variables_dim_{VARIABLE_SIZE};
  std::vector<Scalar> action_;

  LaikagoContactSimulation(bool urdf_from_file) {
    std::string plane_filename = "plane_impl";

    if (urdf_from_file) {
      tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
      cache.construct(plane_filename, world, false, false);

      // std::string urdf_name = "laikago/laikago_toes_zup_xyz_xyzrot.urdf";
      std::string urdf_name = "laikago/laikago_toes_zup.urdf";
      tds::FileUtils::find_file(urdf_name, m_laikago_urdf_filename);
      char laikago_search_path[TINY_MAX_EXE_PATH_LEN];
      tds::FileUtils::extract_path(m_laikago_urdf_filename.c_str(),
                                   laikago_search_path, TINY_MAX_EXE_PATH_LEN);
      m_laikago_search_path = laikago_search_path;

      mb_ = cache.construct(m_laikago_urdf_filename, world, false,
                            laikago_is_floating);
    } else {
      std::string plane_string = plane_implicit_urdf;
      cache.construct_from_string(plane_filename, plane_string, world, false,
                                  false);

      m_laikago_urdf_filename = "laikago/laikago_toes_zup.urdf";
      std::string laikago_string = laikago_toes_zup_urdf;
      mb_ = cache.construct_from_string(m_laikago_urdf_filename, laikago_string,
                                        world, false, laikago_is_floating);
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
      std::vector<Scalar> variables(variables_dim_);
      variables[0] = v[mb_->dof() + mb_->dof_qd() + LAIKAGO_POSE_SIZE + 0];//kp_
      variables[1] = v[mb_->dof() + mb_->dof_qd() + LAIKAGO_POSE_SIZE + 1];//kd_
      variables[2] = v[mb_->dof() + mb_->dof_qd() + LAIKAGO_POSE_SIZE + 2];//max_force_
      return (*this)(v, action, variables);
  }

  std::vector<Scalar> operator()(const std::vector<Scalar>& v,
                                 const std::vector<Scalar>& action,
                                 const std::vector<Scalar>& variables) {
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

    Scalar kp = variables[0];
    Scalar kd = variables[1];
    Scalar max_force = variables[2];

    std::vector<Scalar> result(output_dim());
    for (int t = 0; t < num_timesteps; ++t) {
      if (1) {
        // use PD controller to compute tau
        int qd_offset = mb_->is_floating() ? 6 : 6;
        int q_offset = mb_->is_floating() ? 7 : 6;
        std::vector<double> q_targets;
        q_targets.resize(mb_->tau_.size());

        
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
              Scalar ACTION_LIMIT = 0.4;
              clamped_action = Algebra::min(clamped_action, ACTION_LIMIT);
              clamped_action = Algebra::max(clamped_action, -ACTION_LIMIT);

              Scalar q_desired =
                  initial_poses_laikago2[pose_index++] + clamped_action;

              Scalar q_actual = mb_->q_[q_offset];
              Scalar qd_actual = mb_->qd_[qd_offset];
              Scalar position_error = (q_desired - q_actual);
              Scalar desired_velocity = 0;
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
    
    auto base_tr = mb_->get_world_transform(-1);
    Scalar up_dot_world_z = base_tr.rotation(2, 2);
    result[j++] = up_dot_world_z;

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
  double target_angle;

  tds::NeuralNetwork<Algebra> neural_network;

  std::vector<double> reset(const double new_kp = 100., const double new_kd = 2.,
                            const double target_angle_degrees = 0.) {
    sim_state.resize(0);
    sim_state.resize(contact_sim.input_dim(), Scalar(0));
    contact_sim.set_kp(Algebra::from_double(new_kp));
    contact_sim.set_kd(Algebra::from_double(new_kd));
    target_angle = target_angle_degrees * M_PI / 180.;

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
        sim_state[j + qoffset] = initial_poses_laikago2[j];
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
    Scalar previous_x;
    // Get previous x position.
    // laikago_is_floating is assumed to be true.
    previous_x =
        sim_state[4] * cos(target_angle) + sim_state[5] * sin(target_angle);

    std::vector<double> sim_state_with_action = sim_state;
    sim_state_with_action.resize(contact_sim.input_dim_with_action_and_variables());
    for (int a=0;a<action.size();a++)
    {
        sim_state_with_action[sim_state.size()+a] = action[a];
    }
    sim_state_with_action[sim_state.size() + LAIKAGO_POSE_SIZE + 0] = contact_sim.kp_;
    sim_state_with_action[sim_state.size() + LAIKAGO_POSE_SIZE + 1] = contact_sim.kd_;
    sim_state_with_action[sim_state.size() + LAIKAGO_POSE_SIZE + 2] = contact_sim.max_force_;

    sim_state_with_graphics = contact_sim(sim_state_with_action);
    //sim_state_with_graphics.resize(contact_sim.output_dim());
    //cuda_model_laikago_forward_zero_kernel(1,&sim_state_with_graphics[0], &sim_state_with_action[0]);
    sim_state = sim_state_with_graphics;

    sim_state.resize(contact_sim.input_dim());
    obs = sim_state;
    reward = 1;
    

    // Get reward using current x position.
    // laikago_is_floating is assumed to be true.
    reward = sim_state[4] * cos(target_angle) +
             sim_state[5] * sin(target_angle) - previous_x;
    if (std::isnan(reward) || std::isinf(reward) || (reward != reward)) {
      reward = 0;
      done = true;
    }

    //auto base_tr = contact_sim.mb_->get_world_transform(-1);
    //Scalar up_dot_world_z = base_tr.rotation(2, 2);

    auto up_dot_world_z = sim_state_with_graphics[contact_sim.mb_->dof() + contact_sim.mb_->dof_qd() + contact_sim.mb_->num_links() * 7];

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