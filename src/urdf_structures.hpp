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

#include <assert.h>

#include <map>
#include <string>
#include <vector>

#include "geometry.hpp"  // for GeometryTypes
#include "link.hpp"

namespace tds {
template <typename Algebra>
struct UrdfInertial {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfInertial()
      : mass(Algebra::zero()),
        inertia_xxyyzz(Algebra::zero3()),
        origin_rpy(Algebra::zero3()),
        origin_xyz(Algebra::zero3()) {}

  Scalar mass;
  Vector3 inertia_xxyyzz;
  Vector3 origin_rpy;
  Vector3 origin_xyz;
};

template <typename Algebra>
struct UrdfContact {
  using Scalar = typename Algebra::Scalar;

  UrdfContact()
      : lateral_friction(Algebra::fraction(1, 2)),
        restitution(Algebra::fraction(0, 1)),
        stiffness(Algebra::fraction(1, 1)),
        damping(Algebra::fraction(0, 1)) {}

  Scalar lateral_friction;
  Scalar restitution;
  Scalar stiffness;
  Scalar damping;
};

template <typename Algebra>
struct VisualMaterial {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  VisualMaterial()
      :material_rgb(Scalar(1.),Scalar(1.),Scalar(1.))
  {
  }
  Vector3 material_rgb;
  std::string texture_filename;
};

template <typename Algebra>
struct UrdfCollisionSphere {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfCollisionSphere() : radius(Algebra::one()) {}
  Scalar radius;
};

template <typename Algebra>
struct UrdfCollisionPlane {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfCollisionPlane()
      : normal(Vector3(Algebra::zero(), Algebra::zero(), Algebra::one())),
        constant(Algebra::zero()) {}
  Vector3 normal;
  Scalar constant;
};

template <typename Algebra>
struct UrdfCollisionCapsule {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfCollisionCapsule() : radius(Algebra::one()), length(Algebra::one()) {}
  Scalar radius;
  Scalar length;
};

template <typename Algebra>
struct UrdfCollisionCylinder {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfCollisionCylinder() : radius(Algebra::one()), length(Algebra::one()) {}
  Scalar radius;
  Scalar length;
};

template <typename Algebra>
struct UrdfCollisionBox {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfCollisionBox()
      : extents(Vector3(Algebra::one(), Algebra::one(), Algebra::one())) {}
  Vector3 extents;
};

template <typename Algebra>
struct UrdfCollisionMesh {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfCollisionMesh()
      : scale(Vector3(Algebra::one(), Algebra::one(), Algebra::one())) {}
  std::string file_name;
  Vector3 scale;
};

template <typename Algebra>
struct UrdfGeometry {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  UrdfGeometry() : geom_type(TINY_MAX_GEOM_TYPE) {}

  // pybind11 doesn't like enum GeometryTypes
  int geom_type;  // see GeometryTypes in tiny_geometry.h

  UrdfCollisionSphere<Algebra> sphere;
  UrdfCollisionCapsule<Algebra> capsule;
  UrdfCollisionBox<Algebra> box;
  UrdfCollisionMesh<Algebra> mesh;
  UrdfCollisionPlane<Algebra> plane;
  UrdfCollisionCylinder<Algebra> cylinder;
};

template <typename Algebra>
struct UrdfVisual {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  typedef tds::UrdfGeometry<Algebra> UrdfGeometry;
  typedef tds::VisualMaterial<Algebra> VisualMaterial;
  UrdfVisual()
      : origin_rpy(Algebra::zero3()),
        origin_xyz(Algebra::zero3()),
        has_local_material(false),
        visual_shape_uid(-1)
  {}
  Vector3 origin_rpy;
  Vector3 origin_xyz;
  UrdfGeometry geometry;
  std::string material_name;
  VisualMaterial material;
  std::string visual_name;
  bool has_local_material;
  int visual_shape_uid;
};

template <typename Algebra>
struct UrdfCollision {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  typedef tds::UrdfGeometry<Algebra> UrdfGeometry;

  UrdfCollision()
      : origin_xyz(Algebra::zero3()),
        origin_rpy(Algebra::zero3()),
        collision_group(0),
        collision_mask(0),
        flags(0) {}
  Vector3 origin_xyz;
  Vector3 origin_rpy;

  std::string collision_name;
  int collision_group;
  int collision_mask;
  int flags;
  UrdfGeometry geometry;
};

template <typename Algebra>
struct UrdfLink {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  typedef tds::UrdfCollision<Algebra> UrdfCollision;
  typedef tds::UrdfVisual<Algebra> UrdfVisual;
  typedef tds::UrdfContact<Algebra> UrdfContact;

  UrdfLink() : parent_index(-2) {}
  std::string link_name;
  UrdfInertial<Algebra> urdf_inertial;
  std::vector<UrdfVisual> urdf_visual_shapes;
  std::vector<UrdfCollision> urdf_collision_shapes;
  std::vector<int> child_link_indices;
  UrdfContact contact_info;
  int parent_index;
};

template <typename Algebra>
struct UrdfJoint {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  typedef tds::UrdfLink<Algebra> UrdfLink;

  UrdfJoint()
      : joint_type(tds::JOINT_INVALID),
        joint_lower_limit(Algebra::one()),
        joint_upper_limit(Algebra::zero()),
        joint_origin_xyz(
            Vector3(Algebra::zero(), Algebra::zero(), Algebra::zero())),
        joint_origin_rpy(
            Vector3(Algebra::zero(), Algebra::zero(), Algebra::zero())),
        joint_axis_xyz(
            Vector3(Algebra::zero(), Algebra::zero(), Algebra::one())) {}
  std::string joint_name;
  // pybind11 doesn't like enum JointType
  int joint_type;
  Scalar joint_lower_limit;
  Scalar joint_upper_limit;
  std::string parent_name;
  std::string child_name;
  Vector3 joint_origin_xyz;
  Vector3 joint_origin_rpy;
  Vector3 joint_axis_xyz;
};

enum ConversionReturnCode {
  kCONVERSION_OK = 1,
  kCONVERSION_JOINT_FAILED,
};

template <typename Algebra>
struct UrdfStructures {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  std::string robot_name;
  std::vector<UrdfLink<Algebra> > base_links;
  std::vector<UrdfLink<Algebra> > links;
  std::vector<UrdfJoint<Algebra> > joints;
  std::map<std::string, int> name_to_link_index;
  std::map<std::string, VisualMaterial<Algebra> > materials;
};
}  // namespace tds
