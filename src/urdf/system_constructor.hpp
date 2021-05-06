#pragma once


#include "urdf_cache.hpp"

namespace tds {

/**
 * Provides the system construction function to System and derived classes.
 */
struct SystemConstructor {
  std::string system_urdf_filename;
  // if empty, no ground plane is used
  std::string plane_urdf_filename{""};

  bool is_floating{false};

  // settings for stiffness and damping of all joints in the system
  double joint_stiffness{0};
  double joint_damping{0};

  std::vector<int> control_indices;

  explicit SystemConstructor(const std::string& system_urdf_filename,
                             const std::string& plane_urdf_filename = "")
      : system_urdf_filename(system_urdf_filename),
        plane_urdf_filename(plane_urdf_filename) {}

  SystemConstructor(const std::string& system_urdf_filename,
                    const std::string& plane_urdf_filename, bool is_floating,
                    double joint_stiffness, double joint_damping)
      : system_urdf_filename(system_urdf_filename),
        plane_urdf_filename(plane_urdf_filename),
        is_floating(is_floating),
        joint_stiffness(joint_stiffness),
        joint_damping(joint_damping) {}

  template <typename VisualizerAPI, typename Algebra>
  void operator()(VisualizerAPI* sim, VisualizerAPI* vis, World<Algebra>& world,
                  MultiBody<Algebra>** system, bool clear_cache = false) const {
    static UrdfCache<Algebra> cache;
    if (clear_cache) {
      cache.clear();
    }

    b3RobotSimulatorLoadUrdfFileArgs args;
    args.m_flags |= URDF_MERGE_FIXED_LINKS;
    if (!plane_urdf_filename.empty()) {
      MultiBody<Algebra>* mb = world.create_multi_body();
      const auto& urdf_data =
          cache.retrieve(plane_urdf_filename, sim, vis, args);
      UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_data, world, *mb);
    }

    {
      MultiBody<Algebra>* mb = world.create_multi_body();
      const auto& urdf_data =
          cache.retrieve(system_urdf_filename, sim, vis, args);
      UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_data, world, *mb);
      mb->set_floating_base(is_floating);
      if (!control_indices.empty()) {
        mb->set_control_indices(control_indices);
      }
      mb->initialize();
      for (auto& link : *mb) {
        link.stiffness = Algebra::from_double(joint_stiffness);
        link.damping = Algebra::from_double(joint_damping);
      }
      *system = mb;
    }
  }

  template <typename Algebra>
  void operator()(World<Algebra>& world, MultiBody<Algebra>** system,
                  bool clear_cache = false) const {
    thread_local static UrdfCache<Algebra> cache;
    if (clear_cache) {
      cache.data.clear();
    }

    if (!plane_urdf_filename.empty()) {
      MultiBody<Algebra>* mb = world.create_multi_body();
      const auto& urdf_data = cache.retrieve(plane_urdf_filename);
      UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_data, world, *mb);
    }

    {
      MultiBody<Algebra>* mb = world.create_multi_body();
      const auto& urdf_data = cache.retrieve(system_urdf_filename);
      UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_data, world, *mb);
      mb->set_floating_base(is_floating);
      if (!control_indices.empty()) {
        mb->set_control_indices(control_indices);
      }
      mb->initialize();
      //   if (actuator) {
      //     mb->actuator = new Actuator<Algebra>(*m_actuator);
      //   }
      for (auto& link : *mb) {
        link.stiffness = Algebra::from_double(joint_stiffness);
        link.damping = Algebra::from_double(joint_damping);
      }
      *system = mb;
    }
  }
};
}  // namespace tds
