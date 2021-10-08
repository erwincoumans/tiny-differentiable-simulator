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

#pragma once

#include "../multi_body.hpp"
#include "urdf_parser.hpp"
#include "urdf_to_multi_body.hpp"
#include "world.hpp"
#if USE_BULLET
#include "pybullet_urdf_import.hpp"
#endif

namespace tds {
template <typename Algebra>
class UrdfCache {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  typedef tds::UrdfStructures<Algebra> UrdfStructures;

#if USE_BULLET
  typedef tds::PyBulletUrdfImport<Algebra> UrdfImport;
  typedef b3RobotSimulatorLoadUrdfFileArgs UrdfFileArgs;
#endif

  std::map<std::string, UrdfStructures> data_;

 public:
#if USE_BULLET
  const UrdfStructures& retrieve(const std::string& urdf_filename,
                                 PyBulletVisualizerAPI* sim,
                                 PyBulletVisualizerAPI* vis,
                                 UrdfFileArgs args = UrdfFileArgs(),
                                 bool ignore_cache = false) {
    assert(sim);
    assert(vis);
    if (ignore_cache || data_.find(urdf_filename) == data_.end()) {
      printf("Loading URDF \"%s\".\n", urdf_filename.c_str());
      int robotId = sim->loadURDF(urdf_filename, args);
      if (robotId < 0) {
        std::cerr << "Error: Could not load URDF file \"" << urdf_filename
                  << "\".\n";
        exit(1);
      }
      data_[urdf_filename] = UrdfStructures();
      UrdfImport::extract_urdf_structs(data_[urdf_filename], robotId, sim, vis);
      // sim->removeBody(robotId);
    }
    return data_[urdf_filename];
  }
#endif

  const UrdfStructures& retrieve(const std::string& urdf_filename,
                                 bool ignore_cache = false) {
    if (ignore_cache || data_.find(urdf_filename) == data_.end()) {
      printf("Loading URDF \"%s\".\n", urdf_filename.c_str());
      UrdfParser<Algebra> parser;
      data_[urdf_filename] = parser.load_urdf(urdf_filename);
    }
    return data_[urdf_filename];
  }
  MultiBody<Algebra>* construct(const std::string& urdf_filename,
                                World<Algebra>& world,
                                bool ignore_cache = false,
                                bool is_floating = false,
                                const std::string& name = "") {
    MultiBody<Algebra>* mb = world.create_multi_body(name);
    const auto& urdf_data = retrieve(urdf_filename, ignore_cache);
    UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_data, world, *mb, 0);
    mb->set_floating_base(is_floating);
    mb->initialize();
    return mb;
  }

  const UrdfStructures& retrieve_from_string(const std::string& urdf_name,
                                             const std::string& urdf_string,
                                             bool ignore_cache = false) {
    if (ignore_cache || data_.find(urdf_name) == data_.end()) {
      printf("Loading URDF \"%s\" from a string.\n", urdf_name.c_str());
      UrdfParser<Algebra> parser;
      tds::UrdfStructures<Algebra> urdf_structures;
      int flags = 0;
      tds::NullLogger logger;
      parser.load_urdf_from_string(urdf_string, flags, logger, urdf_structures);
      data_[urdf_name] = urdf_structures;
    }
    return data_[urdf_name];
  }
  MultiBody<Algebra>* construct_from_string(const std::string& urdf_filename,
                                            const std::string& urdf_string,
                                            World<Algebra>& world,
                                            bool ignore_cache = false,
                                            bool is_floating = false,
                                            const std::string& name = "") {
    MultiBody<Algebra>* mb = world.create_multi_body(name);
    const auto& urdf_data =
        retrieve_from_string(urdf_filename, urdf_string, ignore_cache);
    UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_data, world, *mb, 0);
    mb->set_floating_base(is_floating);
    mb->initialize();
    return mb;
  }

#if USE_BULLET
  MultiBody<Algebra>* construct(const std::string& urdf_filename,
                                World<Algebra>& world,
                                PyBulletVisualizerAPI* sim,
                                PyBulletVisualizerAPI* vis,
                                bool ignore_cache = false,
                                bool is_floating = false,
                                const std::string& name = "") {
    assert(sim);
    assert(vis);
    b3RobotSimulatorLoadUrdfFileArgs args;
    args.m_flags |= URDF_MERGE_FIXED_LINKS;
    MultiBody<Algebra>* mb = world.create_multi_body(name);
    const auto& urdf_data =
        retrieve(urdf_filename, sim, vis, args, ignore_cache);
    UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_data, world, *mb);
    mb->set_floating_base(is_floating);
    mb->initialize();
    return mb;
  }
#endif

  void clear() { data_.clear(); }
};
}  // namespace tds
