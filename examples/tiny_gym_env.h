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

#ifndef TINY_GYM_ENV_H
#define TINY_GYM_ENV_H

#include <chrono>
#include <thread>

#include "tiny_multi_body.h"
#include "tiny_system_constructor.h"
#include "tiny_world.h"

/**
 * Implements a differentiable gym environment.
 */
template <typename Scalar, typename Utils, int StateDim, int ActionDim>
struct TinyGymEnvironment {
  static_assert(StateDim > 0, "StateDim must be strictly positive.");
  static_assert(ActionDim > 0, "ActionDim must be strictly positive.");

  static const int kStateDim = StateDim;
  static const int kActionDim = ActionDim;

  virtual void reset(std::vector<Scalar>& state) = 0;

  virtual void step(const std::vector<Scalar>& action,
                    std::vector<Scalar>& next_state, Scalar& cost,
                    bool& done) = 0;
};

template <typename Scalar, typename Utils, int StateDim, int ActionDim,
          template <typename, typename> typename Actuator = TinyActuator>
struct TinyUrdfGymEnvironment
    : public TinyGymEnvironment<Scalar, Utils, StateDim, ActionDim> {
  typedef TinyGymEnvironment<Scalar, Utils, StateDim, ActionDim> GymEnvironment;

  using GymEnvironment::kStateDim, GymEnvironment::kActionDim;

  TinyWorld<Scalar, Utils> world;
  TinyMultiBody<Scalar, Utils>* system{nullptr};

  TinySystemConstructor<Actuator> constructor;

  Scalar dt;
  /**
   * How many simulation time steps to skip within a single environment step.
   */
  int frame_skip{0};
  TinyVector3<Scalar, Utils> gravity{Utils::zero(), Utils::zero(),
                                     Utils::fraction(-981, 100)};

  TinyUrdfGymEnvironment(const TinySystemConstructor<Actuator>& constructor,
                         Scalar dt, int frame_skip = 0)
      : constructor(constructor), dt(dt), frame_skip(frame_skip) {}

  template <typename VisualizerAPI>
  void setup(VisualizerAPI* sim_api, VisualizerAPI* vis_api,
             bool clear_cache = false) {
    constructor(sim_api, vis_api, world, &system, clear_cache);
  }

  virtual void get_state(std::vector<Scalar>& state) const {
    state.resize(kStateDim);
    for (int i = 0; i < system->dof(); ++i) {
      state[i] = system->m_q[i];
    }
    for (int i = 0; i < system->dof_qd(); ++i) {
      if (i + system->dof() >= kStateDim) {
        break;
      }
      state[i + system->dof()] = system->m_qd[i];
    }
  }

  virtual void set_action(const std::vector<Scalar>& action) {
    for (int i = 0; i < std::min(system->dof_actuated(), ActionDim); ++i) {
      system->m_tau[i] = action[i];
    }
  }

  virtual void reset(std::vector<Scalar>& state) {
    system->initialize();
    // TODO add randomness to state initialization?
    get_state(state);
  }

  template <typename VisualizerAPI = PyBulletVisualizerAPI>
  void do_simulation(VisualizerAPI* vis_api = nullptr) {
    for (int i = 0; i <= frame_skip; ++i) {
      system->forward_dynamics(gravity);
      system->integrate_q(dt);
      world.step(dt);
      system->integrate(dt);
      if (vis_api) {
        render(vis_api);
      }
    }
  }

  virtual void step(const std::vector<Scalar>& action,
                    std::vector<Scalar>& next_state, Scalar& cost, bool& done) {
    set_action(action);
    std::vector<Scalar> prev_state;
    get_state(prev_state);
    do_simulation<>();
    get_state(next_state);
    cost = get_cost(prev_state, next_state, action);
    done = false;
  }

  template <typename VisualizerAPI>
  void render(VisualizerAPI* vis_api) const {
    if constexpr (std::is_same_v<Scalar, double>) {
      PyBulletUrdfImport<Scalar, Utils>::sync_graphics_transforms(system,
                                                                  *vis_api);
      std::this_thread::sleep_for(
          std::chrono::duration<double>(Utils::getDouble(dt)));
    }
  }

 protected:
  virtual Scalar get_cost(const std::vector<Scalar>& state_before,
                          const std::vector<Scalar>& state_after,
                          const std::vector<Scalar>& action) const {
    return Utils::zero();
  }
};

#endif  // TINY_GYM_ENV_H
