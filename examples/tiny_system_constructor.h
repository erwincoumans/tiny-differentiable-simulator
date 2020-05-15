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

#ifndef TINY_SYSTEM_CONSTRUCTOR_H
#define TINY_SYSTEM_CONSTRUCTOR_H

#include "pybullet_urdf_import.h"
#include "tiny_actuator.h"
#include "tiny_double_utils.h"
#include "tiny_urdf_to_multi_body.h"

/**
 * Provides the system construction function to TinySystem and derived classes.
 */
template <template <typename, typename> typename Actuator = TinyActuator>
struct TinySystemConstructor {
  std::string m_system_urdf_filename;
  // if empty, no ground plane is used
  std::string m_plane_urdf_filename{""};

  bool m_is_floating{false};

  // settings for stiffness and damping of all joints in the system
  double m_joint_stiffness{0};
  double m_joint_damping{0};

  std::vector<int> m_control_indices;

  Actuator<double, DoubleUtils>* m_actuator{nullptr};

  explicit TinySystemConstructor(const std::string& system_urdf_filename,
                                 const std::string& plane_urdf_filename = "")
      : m_system_urdf_filename(system_urdf_filename),
        m_plane_urdf_filename(plane_urdf_filename) {}

  TinySystemConstructor(const std::string& system_urdf_filename,
                        const std::string& plane_urdf_filename,
                        bool is_floating, double joint_stiffness,
                        double joint_damping)
      : m_system_urdf_filename(system_urdf_filename),
        m_plane_urdf_filename(plane_urdf_filename),
        m_is_floating(is_floating),
        m_joint_stiffness(joint_stiffness),
        m_joint_damping(joint_damping) {}

  template <typename VisualizerAPI, typename Scalar, typename Utils>
  void operator()(VisualizerAPI* sim, VisualizerAPI* vis,
                  TinyWorld<Scalar, Utils>& world,
                  TinyMultiBody<Scalar, Utils>** system,
                  bool clear_cache = false) const {
    static TinyUrdfCache<Scalar, Utils> cache;
    if (clear_cache) {
      cache.data.clear();
    }

    b3RobotSimulatorLoadUrdfFileArgs args;
    args.m_flags |= URDF_MERGE_FIXED_LINKS;
    if (!m_plane_urdf_filename.empty()) {
      TinyMultiBody<Scalar, Utils>* mb = world.create_multi_body();
      const auto& urdf_data =
          cache.retrieve(m_plane_urdf_filename, sim, vis, args);
      TinyUrdfToMultiBody<Scalar, Utils>::convert_to_multi_body(urdf_data,
                                                                world, *mb);
    }

    {
      TinyMultiBody<Scalar, Utils>* mb = world.create_multi_body();
      const auto& urdf_data =
          cache.retrieve(m_system_urdf_filename, sim, vis, args);
      TinyUrdfToMultiBody<Scalar, Utils>::convert_to_multi_body(urdf_data,
                                                                world, *mb);
      mb->m_isFloating = m_is_floating;
      if (!m_control_indices.empty()) {
        mb->m_control_indices = m_control_indices;
      }
      mb->initialize();
      if (m_actuator) {
        mb->m_actuator = new Actuator<Scalar, Utils>(*m_actuator);
      }
      for (auto& link : mb->m_links) {
        link.m_stiffness = Scalar(m_joint_stiffness);
        link.m_damping = Scalar(m_joint_damping);
      }
      *system = mb;
    }
  }
};

#endif  // TINY_SYSTEM_CONSTRUCTOR_H
