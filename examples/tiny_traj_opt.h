/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TINY_TRAJ_OPT_H
#define TINY_TRAJ_OPT_H

#include "control_toolbox.h"
#include "cppad_utils.h"
#include "pybullet_urdf_import.h"
#include "pybullet_visualizer_api.h"
#include "tiny_pd_control.h"
#include "tiny_urdf_to_multi_body.h"
#include "tiny_world.h"

typedef PyBulletVisualizerAPI VisualizerAPI;

template <int QDim, bool QuatIntegration, int ControlDim, typename Scalar,
          typename Utils>
class TinySystem {
  static_assert(
      QDim >= ControlDim,
      "The dimension of q (qd) must be at least as high as the control "
      "dimension in TinySystem.");

 public:
  static const bool kQuatIntegration = QuatIntegration;
  static const int kQDim = QDim;
  static const int kQdDim = QDim - kQuatIntegration;
  static const int kStateDim = kQDim + kQdDim;
  static const int kControlDim = ControlDim;
  typedef ct::core::StateVector<kStateDim, Scalar> State;
  typedef ct::core::StateVector<kQDim, Scalar> Position;
  typedef ct::core::StateVector<kQdDim, Scalar> Velocity;
  typedef ct::core::ControlVector<kControlDim, Scalar> Control;
  typedef ct::core::StateFeedbackController<kStateDim, kControlDim> Controller;
  typedef TinyVector3<Scalar, Utils> Vector3;
  typedef TinyQuaternion<Scalar, Utils> Quaternion;

  TinyWorld<Scalar, Utils> m_world;
  // pointer to multi-body system inside m_world (managed by TinyWorld)
  TinyMultiBody<Scalar, Utils>* m_system{nullptr};

  Vector3 m_gravity{Utils::zero(), Utils::zero(), Utils::fraction(-981, 100)};

  bool use_soft_contact{true};

  typedef std::function<void(VisualizerAPI*, VisualizerAPI*,
                             TinyWorld<Scalar, Utils>&,
                             TinyMultiBody<Scalar, Utils>**)>
      ConstructionFunction;

 protected:
  VisualizerAPI* m_sim{nullptr};
  VisualizerAPI* m_vis{nullptr};
  Scalar m_dt;
  ConstructionFunction m_construction_function;
  mutable bool warning_shown{false};

  /**
   * Control to be sent to MultiBody's actuator.
   */
  std::vector<Scalar> m_control;

 public:
  TinySystem(const ConstructionFunction& construction_function, Scalar dt)
      : m_dt(dt), m_construction_function(construction_function) {}

  void initialize(VisualizerAPI* sim, VisualizerAPI* vis) {
    m_sim = sim;
    m_vis = vis;
    m_world.clear();
    m_control.resize(kControlDim);
    m_construction_function(sim, vis, m_world, &m_system);
    if (m_system->dof_actuated() != kControlDim) {
      fprintf(stderr,
              "Error: controllable degrees of freedom between MultiBody system "
              "(%i) and the control system (%i) do not match.\n",
              m_system->dof_actuated(), kControlDim);
      return;
    }
    printf("Created system.\n");
  }

  virtual void reset(const State& state = State::Base::Zero()) {
    set_state(state);
  }

  void integrate(Scalar dt) { m_system->integrate(dt); }

  template <size_t DIM>
  void set_q(const ct::core::StateVector<DIM, Scalar>& q,
             bool normalize_floating_base_rotation = true) {
    int q_offset = 0, mq_offset = 0;
    if (m_system->m_isFloating) {
      if constexpr (kQuatIntegration) {
        mq_offset = 0;
        if (normalize_floating_base_rotation) {
          Quaternion quat(q[0], q[1], q[2], q[3]);
          quat = quat.normalize();
          m_system->m_q[0] = quat.x();
          m_system->m_q[1] = quat.y();
          m_system->m_q[2] = quat.z();
          m_system->m_q[3] = quat.w();
          q_offset = 4;
        } else {
          q_offset = 0;
        }
      } else {
        q_offset = 3;
        mq_offset = 1;
        // convert Euler angles to quaternion
        Vector3 rpy(q[0], q[1], q[2]);
        Quaternion quat;
        quat.set_euler_rpy2(rpy);
#ifdef DEBUG
        if (q[0] < -Utils::pi() || q[0] > Utils::pi() ||
            q[1] < -Utils::half_pi() || q[1] > Utils::half_pi() ||
            q[2] < -Utils::pi() || q[2] > Utils::pi()) {
          fprintf(stderr,
                  "Warning: converting Euler angles [%.2f %.2f %.2f] to "
                  "Quaternion violating limits [-pi:pi, -pi/2:pi/2, -pi:pi].\n",
                  Utils::getDouble(q[0]), Utils::getDouble(q[1]),
                  Utils::getDouble(q[2]));
        }
#endif
        m_system->m_q[0] = quat.getX();
        m_system->m_q[1] = quat.getY();
        m_system->m_q[2] = quat.getZ();
        m_system->m_q[3] = quat.getW();
      }
    }
    for (int i = q_offset; i < kQDim; ++i) {
      m_system->m_q[mq_offset + i] = q[i];
    }
  }
  template <size_t DIM>
  void get_q(ct::core::StateVector<DIM, Scalar>& q) const {
    int q_offset = 0, mq_offset = 0;
    if (m_system->m_isFloating) {
      if constexpr (kQuatIntegration) {
        q_offset = 0;
        mq_offset = 0;
      } else {
        q_offset = 3;
        mq_offset = 1;
        // convert orientation quaternion to Euler angles
        Quaternion quat(m_system->m_q[0], m_system->m_q[1], m_system->m_q[2],
                        m_system->m_q[3]);
        Vector3 rpy = quat.normalize().get_euler_rpy2();
        q[0] = rpy[0];
        q[1] = rpy[1];
        q[2] = rpy[2];
      }
    }
    for (int i = q_offset; i < kQDim; ++i) {
      q[i] = m_system->m_q[mq_offset + i];
    }
  }

  void set_qd(const Velocity& qd) {
    for (int i = 0; i < kQdDim; ++i) {
      m_system->m_qd[i] = qd[i];
    }
  }
  void get_qd(Velocity& qd) const {
    for (int i = 0; i < kQdDim; ++i) {
      qd[i] = m_system->m_qd[i];
    }
  }

  void set_qdd(const Velocity& qdd) {
    for (int i = 0; i < kQdDim; ++i) {
      m_system->m_qdd[i] = qdd[i];
    }
  }
  void get_qdd(Velocity& qdd) const {
    for (int i = 0; i < kQdDim; ++i) {
      qdd[i] = m_system->m_qdd[i];
    }
  }

  void set_state(const State& state,
                 bool normalize_floating_base_rotation = true) {
    set_q(state, normalize_floating_base_rotation);
    for (int i = 0; i < kQdDim; ++i) {
      m_system->m_qd[i] = state[kQDim + i];
    }
  }

  void get_state(State& state) const {
    get_q(state);
    for (int i = 0; i < kQdDim; ++i) {
      state[kQDim + i] = m_system->m_qd[i];
    }
  }

  void set_control(const Control& control) {
    for (int i = 0; i < kControlDim; ++i) {
      m_control[i] = control[i];
    }
    m_system->control(m_dt, m_control);
  }

  void get_control(Control& control) const {
    for (int i = 0; i < kControlDim; ++i) {
      control[i] = m_control[i];
    }
  }

  /**
   * Computes the change in state (qd, qdd).
   */
  void get_derivative(State& derivative) const {
    for (int i = 0; i < kQdDim; ++i) {
      derivative[i] = m_system->m_qd[i];
      derivative[kQdDim + i] = m_system->m_qdd[i];
    }
  }

  /**
   * Checks that q, qd, qdd, tau are within sensible limits.
   * @return True if system state is valid, false otherwise.
   */
  bool verify_state(double q_bound = 1e5, double qd_bound = 1e5,
                    double qdd_bound = 1e5, double tau_bound = 1e5) const {
    if constexpr (!std::is_same_v<Scalar, double>) {
      // cannot convert CppAD scalars to double here
      return true;
    }
    if (warning_shown) return true;
    for (const auto& v : m_system->m_q) {
      double d = Utils::getDouble(v);
      if (std::isnan(d) || std::abs(d) > q_bound) {
        fprintf(stderr,
                "Warning: system q (%.2f) is NaN or out of +/- %.2f bounds.\n",
                d, q_bound);
        //        m_system->print_state();
        warning_shown = true;
        return false;
      }
    }
    for (const auto& v : m_system->m_qd) {
      double d = Utils::getDouble(v);
      if (std::isnan(d) || std::abs(d) > qd_bound) {
        fprintf(stderr,
                "Warning: system qd (%.2f) is NaN or out of +/- %.2f bounds.\n",
                d, qd_bound);
        //        m_system->print_state();
        warning_shown = true;
        return false;
      }
    }
    for (const auto& v : m_system->m_qdd) {
      double d = Utils::getDouble(v);
      if (std::isnan(d) || std::abs(d) > qdd_bound) {
        fprintf(
            stderr,
            "Warning: system qdd (%.2f) is NaN or out of +/- %.2f bounds.\n", d,
            qdd_bound);
        //        m_system->print_state();
        warning_shown = true;
        return false;
      }
    }
    for (const auto& v : m_system->m_tau) {
      double d = Utils::getDouble(v);
      if (std::isnan(d) || std::abs(d) > tau_bound) {
        fprintf(
            stderr,
            "Warning: system tau (%.2f) is NaN or out of +/- %.2f bounds.\n", d,
            tau_bound);
        //        m_system->print_state();
        warning_shown = true;
        return false;
      }
    }
    return true;
  }

  /**
   * Synchronizes physics simulation to visualization with the m_sim instance.
   */
  void update_visualization() {
    m_system->forward_kinematics();
    if constexpr (std::is_same_v<Scalar, double>) {
      PyBulletUrdfImport<Scalar, Utils>::sync_graphics_transforms(m_system,
                                                                  *m_vis);
    }
  }

  void playback_controller(VisualizerAPI* sim_api, VisualizerAPI* vis_api,
                           const Controller& controller,
                           bool print_states = false,
                           double slow_down_factor = 1) const {
    static_assert(
        std::is_same_v<Scalar, double>,
        "Playback is only available for systems with double scalar type.");

    TinySystem dynamics_vis(m_construction_function, m_dt);
    dynamics_vis.initialize(vis_api, vis_api);
    double last_time = controller.time()[0];
    for (int t = 0; t < controller.time().size(); ++t) {
      double time = controller.time()[t];
      double dt = time - last_time;
      last_time = time;
      const State& state = controller.x_ref()[t];
      dynamics_vis.set_state(state);
      if (t < controller.uff().size()) {
        // there are T-1 controls between T states
        const Control& control = controller.uff()[t];
        dynamics_vis.set_control(control);
      }
      dynamics_vis.update_visualization();

      if (print_states && t % 100 == 0) {
        printf("time: %.3f \t", time);
        if (t < controller.uff().size()) {
          const Control& control = controller.uff()[t];
          printf("control: ");
          for (int i = 0; i < kControlDim; ++i) {
            printf(" %.3f ", Utils::getDouble(control[i]));
          }
          printf("\t");
        }
        dynamics_vis.m_system->print_state();
        dynamics_vis.m_system->clear_forces();
      }

      std::this_thread::sleep_for(
          std::chrono::duration<double>(dt * slow_down_factor));
    }
  }

 protected:
  /**
   * To be called by the children's clone() functions.
   */
  template <typename System = TinySystem>
  System* make_copy() const {
    // TODO implement TinyWorld clone function
    auto* system = new System(m_construction_function, m_dt);
    system->initialize(m_sim, m_vis);
    system->m_gravity = m_gravity;
    system->m_system->m_q = m_system->m_q;
    system->m_system->m_qd = m_system->m_qd;
    system->m_system->m_qdd = m_system->m_qdd;
    system->m_system->m_tau = m_system->m_tau;
    system->m_system->m_integration_type = m_system->m_integration_type;
    system->m_system->m_actuator = m_system->m_actuator;
    system->m_system->m_control_indices = m_system->m_control_indices;
    system->m_world.m_mb_constraint_solver = m_world.m_mb_constraint_solver;
    system->use_soft_contact = use_soft_contact;
    return system;
  }
};

/**
 * Makes our simulator compatible with trajectory optimizers in Control Toolbox.
 * This is currently not used, but symplectic integrators would be more
 * accurate at larger integration time steps.
 */
template <int QDim, bool QuatIntegration, int ControlDim, typename Scalar,
          typename Utils>
class TinySymplecticSystem
    : public ct::core::SymplecticSystem<QDim, QDim - QuatIntegration,
                                        ControlDim, Scalar>,
      public TinySystem<QDim, QuatIntegration, ControlDim, Scalar, Utils> {
 public:
  typedef ct::core::SymplecticSystem<QDim, QDim - QuatIntegration, ControlDim,
                                     Scalar>
      SymplecticSystem;
  typedef TinySystem<QDim, QuatIntegration, ControlDim, Scalar, Utils> System;

  using System::get_q, System::set_q, System::get_qd, System::set_qd;
  using System::get_qdd, System::get_state;
  using System::kQDim, System::kQdDim, System::kStateDim, System::kControlDim;
  using System::kQuatIntegration;
  using System::m_gravity, System::m_last_time, System::m_dt, System::m_control;
  using System::m_system, System::m_world;
  using System::set_state, System::set_control, System::get_derivative;
  using System::update_visualization;
  using typename System::ConstructionFunction, typename System::Quaternion,
      typename System::Vector3;
  using typename System::State, typename System::Position,
      typename System::Velocity, typename System::Control;

  explicit TinySymplecticSystem(
      const ConstructionFunction& construction_function, Scalar dt)
      : SymplecticSystem(ct::core::SYSTEM_TYPE::SECOND_ORDER),
        System(construction_function, dt) {}

  TinySymplecticSystem* clone() const {
    // TODO implement TinyWorld clone function
    return this->template make_copy<TinySymplecticSystem>();
  }

  /**
   * @brief      Computes the derivative of the position
   *
   * @param[in]  x        The full state vector
   * @param[in]  v        The updated velocity
   * @param[in]  control  The control input
   * @param[out] pDot     The derivative of the position
   */
  void computePdot(const State& x, const Velocity& v, const Control& control,
                   Position& pDot) override {
    //        set_state(x);
    set_qd(v);
    if constexpr (kQuatIntegration) {
      m_system->forward_kinematics();
      Vector3 angular_velocity(v[0], v[1], v[2]);
      //      angular_velocity.print("angular_velocity");
      Quaternion base_rot;
      m_system->m_base_X_world.m_rotation.getRotation(base_rot);
      //      base_rot.print("base_rot");
      Quaternion omega = (angular_velocity * base_rot) * (Utils::half());
      //      omega.print("omega");
      pDot[0] = omega.x();
      pDot[1] = omega.y();
      pDot[2] = omega.z();
      pDot[3] = omega.w();
      for (int i = 4; i < kQDim; ++i) {
        pDot[i] = v[i - 1];
      }
    } else {
      pDot = v;
    }
  }

  /**
   * @brief      Computes the derivative of the velocity
   *
   * @param[in]  x        The full state vector
   * @param[in]  p        The position
   * @param[in]  control  The control input
   * @param[out] vDot     The derivative of the velocity
   */
  void computeVdot(const State& x, const Position& p, const Control& control,
                   Velocity& vDot) override {
    //        set_state(x);
    set_q(p);
    update_visualization();

    m_system->forward_kinematics();
    m_system->clear_forces();
    m_world.step(m_dt);
    set_control(control);
    //    std::cout << "u: \t" << control.transpose() << "\n";

    m_system->forward_dynamics(m_gravity);

    get_qdd(vDot);
  }

  /**
   * Roll out and visualize this controllable system until time T starting from
   * the given initial state.
   */
  template <typename VisualizerAPI>
  void playback_physics(VisualizerAPI* vis_api, Scalar T,
                        const State& initial_state,
                        const ct::core::ControlVectorArray<kControlDim>& us =
                            ct::core::ControlVectorArray<kControlDim>()) {
    this->initialize(vis_api, vis_api);
    Scalar time = Utils::zero();
    set_state(initial_state);
    std::cout << "Initial state:  " << initial_state.transpose() << "\n";
    this->update_visualization();
    m_system->print_state();
    Position p, pd;
    Velocity v, vd;
    get_q(p);
    get_qd(v);
    State x;
    Control u;
    u.setZero();
    int step = 0;
    while (time < T) {
      if (step < static_cast<int>(us.size())) {
        u = us[step];
      }
      get_state(x);
      computePdot(x, v, u, pd);
      p += m_dt * pd;
      computeVdot(x, p, u, vd);
      v += m_dt * vd;
      if (step % 100 == 0) {
        std::cout << "x:  " << x.transpose() << std::endl;
        std::cout << "p:  " << p.transpose() << std::endl;
        std::cout << "v:  " << v.transpose() << std::endl;
        std::cout << "u:  " << u.transpose() << std::endl;
        printf("t: %.3f\t", time);
        m_system->print_state();
      }

      time += m_dt;

      this->update_visualization();

      std::this_thread::sleep_for(
          std::chrono::duration<double>(Utils::getDouble(m_dt)));
      ++step;
    }
  }

  /**
   * Compute controls using PD control to serve as sensible initialization for
   * an optimal control problem.
   */
  void initialize_static_pd_control(
      ct::core::ControlVectorArray<kControlDim>& u0_ff,
      const std::vector<Scalar>& q_desired, const State& initial_state,
      double kp, double kd, double min_force, double max_force) {
    Scalar time = Utils::zero();
    State x = initial_state;
    set_state(initial_state);
    Control u;
    Position p, pd;
    Velocity v, vd;
    get_q(p);
    get_qd(v);
    for (int t = 0; t < static_cast<int>(u0_ff.size()); ++t) {
      //      std::cout << "state: " << x.transpose() << "\n";
      TinyPDController::compute<Scalar, Utils>(
          kControlDim, m_system->m_q, m_system->m_qd, m_control, kp, kd,
          min_force, max_force, q_desired);
      for (int i = 0; i < kControlDim; ++i) {
        u[i] = m_control[i];
      }
      u0_ff[t] = u;
      get_state(x);
      computePdot(x, v, u, pd);
      p += m_dt * pd;
      computeVdot(x, p, u, vd);
      v += m_dt * vd;

      time += m_dt;
    }
  }

  void initialize_static_pd_control(
      ct::core::ControlVectorArray<kControlDim>& u0_ff, const State& q_desired,
      const State& initial_state, double kd, double kp, double min_force,
      double max_force) {
    int q_offset = 0;
    if (m_system->m_isFloating) {
      q_offset = 6;
    }
    std::vector<Scalar> target(m_system->dof_actuated());
    for (int i = 0; i < m_system->dof_actuated(); ++i) {
      target[i] = q_desired[q_offset + i];
    }
    initialize_static_pd_control(u0_ff, target, initial_state, kd, kp,
                                 min_force, max_force);
  }
};

/**
 * Makes our simulator compatible with trajectory optimizers in Control Toolbox.
 */
template <int QDim, int ControlDim, typename Scalar, typename Utils>
class TinyControlledSystem
    : public ct::core::ControlledSystem<QDim + QDim, ControlDim, Scalar>,
      public TinySystem<QDim, false, ControlDim, Scalar, Utils> {
 public:
  typedef ct::core::ControlledSystem<QDim + QDim, ControlDim, Scalar>
      ControlledSystem;
  typedef TinySystem<QDim, false, ControlDim, Scalar, Utils> System;

  using System::kQDim, System::kQdDim, System::kStateDim, System::kControlDim;
  using System::m_gravity, System::m_dt, System::m_control;
  using System::m_system, System::m_world, System::verify_state, System::get_q;
  using System::set_state, System::set_control, System::get_derivative;
  using System::use_soft_contact;
  using typename System::ConstructionFunction, typename System::Position;
  using typename System::State, typename System::Control;

  TinyControlledSystem(const ConstructionFunction& construction_function,
                       Scalar dt)
      : ControlledSystem(ct::core::SYSTEM_TYPE::SECOND_ORDER),
        System(construction_function, dt) {
    assert(kQDim == kQdDim);
  }

  TinyControlledSystem* clone() const override {
    return this->template make_copy<TinyControlledSystem>();
  }

  /**
   * This function defines the system dynamics and is used by the trajectory
   * optimization algorithms in Control Toolbox.
   * @param state Input state [q, qd].
   * @param t Time.
   * @param control Control input.
   * @param derivative Resulting change in state [qd, qdd].
   */
  void computeControlledDynamics(const State& state, const Scalar& t,
                                 const Control& control,
                                 State& derivative) override {
    set_state(state);
    this->update_visualization();

    if (use_soft_contact) {
      m_system->clear_forces();
      // m_system->forward_kinematics();
      set_control(control);

      // m_system->integrate_q(m_dt);
      m_world.step(m_dt);
      // m_system->forward_dynamics(m_gravity);
      m_system->forward_dynamics(m_gravity);

      if (!this->verify_state()) {
        assert(0);
        return;
      }

      // m_system->forward_kinematics();
      // set_control(control);
      // m_system->forward_dynamics(m_gravity);
      // m_system->clear_forces();
      // m_system->integrate_q(m_dt);
      // m_world.step(m_dt);

      // if (!this->verify_state()) {
      //   assert(0);
      //   return;
      // }

    } else {
      set_control(control);

      if (!this->verify_state()) {
        assert(0);
        return;
      }

      m_system->forward_dynamics(m_gravity);
      m_system->clear_forces();
      m_system->integrate_q(m_dt);
      m_world.step(m_dt);
    }

    if (use_soft_contact) {
      get_derivative(derivative);
      // m_system->integrate_q(m_dt);
      // for (int i = 0 ; i < kQdDim; ++i) {
      //   derivative[i] = m_system->m_qd[i];
      // }
      return;
    }

    m_system->integrate(m_dt);

    Position q_after;
    get_q(q_after);

    int q_offset = m_system->m_isFloating ? 1 : 0;
    for (int i = 0; i < kQDim; ++i) {
      //            derivative[i] = m_system->m_qd[i];
      //      if (!m_system->m_isFloating || i > 2) {
      derivative[i] = (q_after[i] - state[i]) / m_dt;
      //      }
      // compute qdd from the change in qd (inverse of Euler integration)
      derivative[kQdDim + i] = (m_system->m_qd[i] - state[kQDim + i]) / m_dt;
    }

    //  std::cout << "   derivative: " << derivative.transpose() << "\n";
  }

  /**
   * Roll out and visualize this controllable system until time T starting from
   * the given initial state.
   */
  template <typename VisualizerAPI>
  void playback_physics(VisualizerAPI* vis_api, Scalar T,
                        const State& initial_state,
                        const ct::core::ControlVectorArray<kControlDim>& us =
                            ct::core::ControlVectorArray<kControlDim>()) {
    this->initialize(vis_api, vis_api);
    Scalar time = Utils::zero();
    State x = initial_state;
    State xd;
    Control u;
    u.setZero();
    int step = 0;
    while (time < T) {
      if (step < static_cast<int>(us.size())) {
        u = us[step];
      }
      computeControlledDynamics(x, time, u, xd);
      if (step % 100 == 0) {
        printf("t: %.3f\t", time);
        // m_system->print_state();
        std::cout << xd.transpose() << '\n';
      }
      time += m_dt;
      x += m_dt * xd;

      this->update_visualization();

      std::this_thread::sleep_for(
          std::chrono::duration<double>(Utils::getDouble(m_dt)));
      ++step;
    }
  }

  /**
   * Compute controls using PD control to serve as sensible initialization for
   * an optimal control problem.
   */
  void initialize_static_pd_control(
      ct::core::ControlVectorArray<kControlDim>& u0_ff,
      const std::vector<Scalar>& q_desired, const State& initial_state,
      double kp, double kd, double min_force, double max_force) {
    Scalar time = Utils::zero();
    State x = initial_state;
    State xd;
    std::vector<double> controllable_q(kControlDim);
    std::vector<double> controllable_qd(kControlDim);
    int q_offset = m_system->m_isFloating ? 7 : 0;
    int qd_offset = m_system->m_isFloating ? 6 : 0;
    for (int t = 0; t < static_cast<int>(u0_ff.size()); ++t) {
      this->set_state(x);
      //      std::cout << "state: " << x.transpose() << "\n";
      for (int i = 0; i < kControlDim; ++i) {
        int ci = m_system->m_control_indices[i];
        controllable_q[ci] = m_system->m_q[ci + q_offset];
        controllable_qd[ci] = m_system->m_qd[ci + qd_offset];
      }
      TinyPDController::compute<Scalar, Utils>(
          kControlDim, controllable_q, controllable_qd, m_control, kp, kd,
          min_force, max_force, q_desired);
      for (int i = 0; i < kControlDim; ++i) {
        u0_ff[t][i] = m_control[i];
      }
      computeControlledDynamics(x, time, u0_ff[t], xd);
      m_system->print_state();
      time += m_dt;
      x += m_dt * xd;
    }
  }

  void initialize_static_pd_control(
      ct::core::ControlVectorArray<kControlDim>& u0_ff, const State& q_desired,
      const State& initial_state, double kd, double kp, double min_force,
      double max_force) {
    int q_offset = 0;
    if (m_system->m_isFloating) {
      q_offset = 6;
    }
    std::vector<Scalar> target(m_system->dof_actuated());
    for (int i = 0; i < m_system->dof_actuated(); ++i) {
      target[i] = q_desired[q_offset + i];
    }
    initialize_static_pd_control(u0_ff, target, initial_state, kd, kp,
                                 min_force, max_force);
  }
};

#endif  // TINY_TRAJ_OPT_H
