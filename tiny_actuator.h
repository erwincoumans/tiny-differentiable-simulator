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

#ifndef TINY_ACTUATOR_H
#define TINY_ACTUATOR_H

#include "tiny_neural_network.h"
#include "tiny_pd_control.h"

/**
 * Base class for an actuator, directly applying control inputs `u` as joint
 * torques `tau`.
 * Note that the dimensions only consider the actuated degrees of freedom,
 * i.e. they do not reflect the floating base DOF or joints which attach the
 * robot base with the world.
 */
template <typename TinyScalar, typename TinyConstants>
struct TinyActuator {
  typedef std::vector<TinyScalar> VectorType;

  // Note that the dimensions of q, qd and tau have to match since only the
  // actuated degrees of freedom are considered.
  int dof;

  /**
   * Per-joint gear ratios to apply to the control input.
   * Ratios < 1 mean that the output torque is larger than the input control.
   */
  std::vector<TinyScalar> gear_ratios;
  /**
   * Per-joint control input bound.
   */
  std::vector<TinyScalar> limits;

  explicit TinyActuator(int dof) : dof(dof) {}

  template <typename Scalar, typename Utils>
  TinyActuator(const TinyActuator<Scalar, Utils>& rhs) : dof(rhs.dof) {
    gear_ratios.resize(dof);
    limits.resize(dof);
    for (int i = 0; i < dof; ++i) {
      gear_ratios[i] = TinyConstants::scalar_from_double(
          Utils::getDouble(rhs.gear_ratios[i]));
      limits[i] =
          TinyConstants::scalar_from_double(Utils::getDouble(rhs.limits[i]));
    }
  }

  virtual ~TinyActuator() = default;

  /**
   * Computes the actuator control output (i.e., joint torques `tau`).
   * @param q Actuated joint positions.
   * @param qd Actuated joint velocities.
   * @param u Control input.
   * @param tau Output joint torques.
   */
  virtual void compute_torques(const VectorType& q, const VectorType& qd,
                               const VectorType& u, VectorType& tau) {
    // by default, directly apply control input as joint torques
    tau = u;
    if (!gear_ratios.empty()) {
      for (int i = 0; i < dof; ++i) {
        tau[i] /= gear_ratios[i];
      }
    }
    apply_limits(tau);
    //    printf("tau: ");
    //    for (auto& t : tau) {
    //      printf(" %.3f ", TinyConstants::getDouble(t));
    //    }
    //    printf("\n");
  }

  /**
   * Computes the actuator dynamics. By default, it does nothing.
   * Actuators that implement this function are assumed to have an internal
   * state that is updated by the `integrate` function using the state
   * derivative computed by this dynamics function.
   * @param q Actuated joint positions.
   * @param qd Actuated joint velocities.
   * @param u Control input.
   * @param derivative Actuator state derivative.
   */
  virtual void forward_dynamics(const VectorType& q, const VectorType& qd,
                                const VectorType& u, VectorType& derivative) {}

  /**
   * Implements a simple Euler integration step, first compute actuator
   * dynamics, then update actuator state from the resulting state derivative
   * and the input time step.
   * @param time Current simulation time in seconds.
   * @param dt Time step in seconds.
   * @param q Actuated joint positions.
   * @param qd Actuated joint velocities.
   * @param u Control input.
   */
  virtual void integrate(const TinyScalar& dt, const VectorType& q,
                         const VectorType& qd, const VectorType& u) {}

 protected:
  virtual void apply_limits(VectorType& tau) {
    using std::min, std::max;
    if (!limits.empty()) {
      for (int i = 0; i < dof; ++i) {
        tau[i] = min(limits[i], max(-limits[i], tau[i]));
      }
    }
  }
};

/**
 * Series Elastic Actuator (SEA) modeled as a series of motor, gearbox and
 * spring. Control Input: Motor Velocity Actuator State: Gear Position Actuator
 * State Derivative: Gear Velocity Code adapted from Control Toolbox
 * (https://github.com/ethz-adrl/control-toolbox).
 */
template <typename TinyScalar, typename TinyConstants>
struct TinySeriesElasticActuator
    : public TinyActuator<TinyScalar, TinyConstants> {
  typedef TinyActuator<TinyScalar, TinyConstants> Actuator;
  using Actuator::apply_limits;
  using Actuator::dof, Actuator::gear_ratios;
  using typename Actuator::VectorType;

  /**
   * Spring constant.
   */
  TinyScalar spring_k{TinyConstants::one()};

  /**
   * Internal actuator state for the gear positions.
   */
  std::vector<TinyScalar> gear_positions;

  explicit TinySeriesElasticActuator(int dof) : Actuator(dof) {
    gear_positions.resize(dof, TinyConstants::zero());
  }

  /**
   * Computes the actuator control output (i.e., joint torques `tau`).
   * @param q Actuated joint positions.
   * @param qd Actuated joint velocities.
   * @param u Control input.
   * @param tau Output joint torques.
   */
  virtual void compute_torques(const VectorType& q, const VectorType& qd,
                               const VectorType& u, VectorType& tau) {
    // by default, directly apply control input as joint torques
    tau.resize(dof);
    for (int i = 0; i < dof; ++i) {
      tau[i] = (gear_positions[i] - q[i]) * spring_k;
    }
    apply_limits(tau);
  }

  /**
   * Computes the actuator dynamics. By default, it does nothing.
   * Actuators that implement this function are assumed to have an internal
   * state that is updated by the `integrate` function using the state
   * derivative computed by this dynamics function.
   * @param time Current simulation time in seconds.
   * @param q Actuated joint positions.
   * @param qd Actuated joint velocities.
   * @param u Control input.
   * @param derivative Actuator state derivative.
   */
  virtual void forward_dynamics(const VectorType& q, const VectorType& qd,
                                const VectorType& u, VectorType& derivative) {
    derivative.resize(dof);
    for (int i = 0; i < dof; ++i) {
      derivative[i] = u[i] / gear_ratios[i];
    }
  }

  /**
   * Implements a simple Euler integration step, first compute actuator
   * dynamics, then update actuator state from the resulting state derivative
   * and the input time step.
   * @param time Current simulation time in seconds.
   * @param dt Time step in seconds.
   * @param q Actuated joint positions.
   * @param qd Actuated joint velocities.
   * @param u Control input.
   */
  virtual void integrate(const TinyScalar& dt, const VectorType& q,
                         const VectorType& qd, const VectorType& u) {
    forward_dynamics(q, qd, u, derivative_);
    for (int i = 0; i < dof; ++i) {
      gear_positions[i] += dt * derivative_[i];
    }
  }

 private:
  // keep it stored to avoid memory allocations at each integration step
  VectorType derivative_;
};

/**
 * Implements position servo actuator as a PD controller.
 */
template <typename TinyScalar, typename TinyConstants>
struct TinyServoActuator : public TinyActuator<TinyScalar, TinyConstants> {
  typedef TinyActuator<TinyScalar, TinyConstants> Actuator;
  using Actuator::apply_limits;
  using Actuator::dof;
  using typename Actuator::VectorType;

  TinyScalar kp;
  TinyScalar kd;
  TinyScalar min_force;
  TinyScalar max_force;

  TinyServoActuator(int dof, const TinyScalar& kp, const TinyScalar& kd,
                    const TinyScalar& min_force, const TinyScalar& max_force)
      : Actuator(dof),
        kp(kp),
        kd(kd),
        min_force(min_force),
        max_force(max_force) {}

  /**
   * Implements the servo actuator.
   * @param q Joint positions.
   * @param qd Joint velocities.
   * @param u Desired joint positions.
   * @param tau Output joint torques.
   */
  virtual void compute_torques(const VectorType& q, const VectorType& qd,
                               const VectorType& u, VectorType& tau) {
    TinyPDController::compute<TinyScalar, TinyConstants>(
        dof, q, qd, tau, kp, kd, min_force, max_force, u);
    apply_limits(tau);
#ifdef DEBUG
    printf("servo output: ");
    for (auto& v : tau) {
      printf("%.4f  ", v);
    }
    printf("  qd: ");
    for (auto& v : qd) {
      printf("%.4f  ", v);
    }
    printf("\n");
#endif
  }
};

/**
 * Implements the architecture of Actuator Net introduced in Hwangbo et al.
 * "Learning Agile and Dynamic Motor Skills for Legged Robots", 2019.
 * Actuator Net is a neural-network driven servo actuator that given
 * a history of discrepancy of joint positions and target joint positions, plus
 * joint velocities, computes the output joint torques tau.
 */
template <typename TinyScalar, typename TinyConstants>
struct TinyActuatorNet : public TinyActuator<TinyScalar, TinyConstants> {
  typedef TinyActuator<TinyScalar, TinyConstants> Actuator;

  using Actuator::dof;
  using typename Actuator::VectorType;

  /**
   * Ring buffer storing the state history.
   */
  std::vector<TinyScalar> history;

  TinyNeuralNetwork<TinyScalar, TinyConstants> network;
  /**
   * Number of input states (q, qd) making up the history input to the network.
   */
  int history_size;

  TinyActuatorNet(int history_size, int dof)
      : Actuator(dof),
        network(history_size * dof * dof),
        history_size(history_size) {
    network.add_linear_layer(NN_ACT_SOFTSIGN, 32);
    network.add_linear_layer(NN_ACT_SOFTSIGN, 32);
    network.add_linear_layer(NN_ACT_SOFTSIGN, 32);
    network.add_linear_layer(NN_ACT_IDENTITY, dof);
    network.initialize();
    history.reserve(history_size * dof * dof);
  }

  /**
   * Implements the Actuator Net inference pass.
   * @param q Input joint positions.
   * @param qd Ignored.
   * @param u Target joint positions.
   * @param tau Output joint torques.
   */
  void compute_torques(const VectorType& q, const VectorType& qd,
                       const VectorType& u, VectorType& tau) override {
    if (history.empty()) {
      // fill history with current input
      auto it = history.begin();
      for (int h = 0; h < history_size; ++h) {
        it = history.insert(it, q.begin(), q.end());
        for (int i = 0; i < dof; ++i) {
          // compute position deviations
          it = history.insert(it, tau[i] - q[i]);
        }
      }
    }
    // add current input to state history, after removing oldest entries
    history.erase(history.begin(), history.begin() + 2 * dof);
    auto it = history.end();
    it = history.insert(it, q.begin(), q.end());
    for (int i = 0; i < dof; ++i) {
      // compute position deviations
      it = history.insert(it, tau[i] - q[i]);
    }

    network.compute(history, tau);
  }
};

#endif  // TINY_ACTUATOR_H
