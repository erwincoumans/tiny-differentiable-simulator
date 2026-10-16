// Unitree G1 humanoid (29 dof body + two 7 dof Dex3 hands), used by the G1
// standing example and traced by g1_codegen.cpp into batched CPU/CUDA kernels.
//
// Both functions of the model only depend on their input arrays, and have no
// branches that depend on the input values, so that CppADCodeGen can trace
// them once and generate kernels that are valid for any input:
//   step:  state (q, qd) and control (q_target, kp, kd of each actuated
//          joint) -> next_state, after one time step of dt
//   poses: state -> world pose (position, quaternion xyzw) of each visual,
//          for rendering
//
// Only the 8 ankle spheres collide with the ground plane (feet-only
// contacts), and the contact solver keeps all 8 contact points, also when
// they are not touching, so that the contact set does not depend on the state.
//
// The model data/unitree_g1/g1_29dof_with_hand_rev_1_0.urdf is converted from
// the newton-assets unitree_g1 URDF by data/unitree_g1/convert_g1_urdf.py.

#pragma once

#include <string>
#include <vector>

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "dynamics/kinematics.hpp"
#include "urdf/urdf_cache.hpp"
#include "utils/file_utils.hpp"
#include "world.hpp"

template <typename Algebra>
struct G1Model {
  using Scalar = typename Algebra::Scalar;

  static constexpr const char* urdf_name =
      "unitree_g1/g1_29dof_with_hand_rev_1_0.urdf";
  // pelvis height when the feet touch the ground in the zero joint pose
  static constexpr double standing_height = 0.793;
  // the Newton example drops the robot from 0.2 m above standing height
  static constexpr double drop_height = 0.2;
  // 60 frames per second with 6 substeps, as in the Newton example
  static constexpr int substeps = 6;
  static constexpr double frame_dt = 1. / 60.;
  // joint position control. The Newton example sets joint_target_ke = 500
  // and joint_target_kd = 10, but its MuJoCo solver also keeps the position
  // actuators of the G1 MJCF model in the USD (kp = 500), so the effective
  // stiffness is 1000.
  static constexpr double default_kp = 1000.;
  static constexpr double default_kd = 10.;

  tds::UrdfCache<Algebra> cache;
  std::string urdf_filename;
  tds::World<Algebra> world;
  tds::MultiBody<Algebra>* system = nullptr;

  Scalar dt{Algebra::from_double(frame_dt / substeps)};
  // rotor inertia of each joint, the armature of the G1 MJCF/USD model
  Scalar armature{Algebra::from_double(0.01)};

  // Joint PD control is implicit by default: the PD torque uses the joint
  // state at the end of the time step, with semi-implicit Euler integration
  // qd' = qd + dt qdd and q' = q + dt qd':
  //   tau = kp (q_target - q') - kd qd'
  //       = kp (q_target - q - dt qd) - kd qd - (dt kd + dt^2 kp) qdd.
  // The last term acts as additional joint inertia, so it is added to the
  // joint armature, which forward dynamics and the contact solver (through
  // the mass matrix) take into account. Explicit PD, as in the laikago
  // example, is only stable for much smaller time steps with these gains.
  bool implicit_pd{true};

  // indices of the links with an actuated (non-fixed) joint
  std::vector<int> actuated_links;

  static bool is_foot(const std::string& link_name) {
    return link_name == "left_ankle_roll_link" ||
           link_name == "right_ankle_roll_link";
  }

  explicit G1Model(bool feet_only_contacts = true) {
    std::string plane_filename;
    tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
    cache.construct(plane_filename, world, false, false);
    tds::FileUtils::find_file(urdf_name, urdf_filename);
    system = cache.construct(urdf_filename, world, false, true);
    world.default_friction = Algebra::from_double(0.75);
    world.get_mb_constraint_solver()->pgs_iterations_ = 50;
    world.get_mb_constraint_solver()->keep_all_points_ = true;
    if (feet_only_contacts) {
      system->collision_geometries(-1).clear();
      system->collision_transforms(-1).clear();
      for (auto& link : *system) {
        if (!is_foot(link.link_name)) {
          link.collision_geometries.clear();
          link.X_collisions.clear();
        }
      }
    }
    for (const auto& link : *system) {
      if (link.joint_type != tds::JOINT_FIXED) {
        actuated_links.push_back(link.index);
      }
    }
  }

  int num_actuators() const { return static_cast<int>(actuated_links.size()); }
  // q (base quaternion xyzw, base position, joint angles), qd
  int state_dim() const { return system->dof() + system->dof_qd(); }
  // q_target, kp and kd of each actuated joint
  int control_dim() const { return 3 * num_actuators(); }
  int num_visuals() const {
    int num = static_cast<int>(system->X_visuals().size());
    for (const auto& link : *system) {
      num += static_cast<int>(link.X_visuals.size());
    }
    return num;
  }
  int poses_dim() const { return 7 * num_visuals(); }

  // the robot in the zero joint pose, dropped from above standing height
  std::vector<double> default_state() const {
    std::vector<double> state(state_dim(), 0.);
    state[3] = 1.;
    state[6] = standing_height + drop_height;
    return state;
  }
  // zero joint targets, with the default gains
  std::vector<double> default_control() const {
    const int n = num_actuators();
    std::vector<double> control(control_dim(), 0.);
    std::fill(control.begin() + n, control.begin() + 2 * n, default_kp);
    std::fill(control.begin() + 2 * n, control.end(), default_kd);
    return control;
  }

  void set_state(const Scalar* state) {
    system->initialize();
    for (int i = 0; i < system->dof(); ++i) {
      system->q(i) = state[i];
    }
    for (int i = 0; i < system->dof_qd(); ++i) {
      system->qd(i) = state[system->dof() + i];
    }
  }

  void get_state(Scalar* state) const {
    for (int i = 0; i < system->dof(); ++i) {
      state[i] = system->q(i);
    }
    for (int i = 0; i < system->dof_qd(); ++i) {
      state[system->dof() + i] = system->qd(i);
    }
  }

  void step(const Scalar* state, const Scalar* control, Scalar* next_state) {
    set_state(state);
    const int n = num_actuators();
    const Scalar* q_target = control;
    const Scalar* kp = control + n;
    const Scalar* kd = control + 2 * n;
    for (int i = 0; i < n; ++i) {
      auto& link = system->links_[actuated_links[i]];
      const Scalar q = system->q(link.q_index);
      const Scalar qd = system->qd(link.qd_index);
      Scalar force;
      if (implicit_pd) {
        link.armature = armature + dt * kd[i] + dt * dt * kp[i];
        force = kp[i] * (q_target[i] - q - dt * qd) - kd[i] * qd;
      } else {
        link.armature = armature;
        force = kp[i] * (q_target[i] - q) - kd[i] * qd;
      }
      system->tau_[link.qd_index - 6] = force;
    }
    tds::forward_dynamics(*system, world.get_gravity());
    system->clear_forces();

    integrate_euler_qdd(*system, dt);

    world.step(dt);

    tds::integrate_euler(*system, dt);
    get_state(next_state);
  }

  void poses(const Scalar* state, Scalar* poses) {
    set_state(state);
    tds::forward_kinematics(*system, system->q(), system->qd());
    int j = 0;
    auto write_transform = [&](const tds::Transform<Algebra>& tr) {
      auto orn = Algebra::matrix_to_quat(tr.rotation);
      poses[j++] = tr.translation[0];
      poses[j++] = tr.translation[1];
      poses[j++] = tr.translation[2];
      poses[j++] = Algebra::quat_x(orn);
      poses[j++] = Algebra::quat_y(orn);
      poses[j++] = Algebra::quat_z(orn);
      poses[j++] = Algebra::quat_w(orn);
    };
    for (const auto& X_visual : system->X_visuals()) {
      write_transform(system->base_X_world() * X_visual);
    }
    for (const auto& link : *system) {
      for (const auto& X_visual : link.X_visuals) {
        write_transform(link.X_world * X_visual);
      }
    }
  }
};
