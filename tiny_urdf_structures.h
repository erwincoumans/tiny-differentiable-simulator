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

#ifndef TINY_URDF_STRUCTURES_H
#define TINY_URDF_STRUCTURES_H

#include <assert.h>

#include <map>
#include <string>
#include <vector>

#include "tiny_geometry.h"  // for TinyGeometryTypes
#include "tiny_quaternion.h"
#include "tiny_symmetric_spatial_dyad.h"
#include "tiny_world.h"

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfInertial {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;

  TinyUrdfInertial()
      : mass(TinyConstants::zero()),
        inertia_xxyyzz(TinyVector3::zero()),
        origin_rpy(TinyVector3::zero()),
        origin_xyz(TinyVector3::zero()) {}
  TinyScalar mass;
  TinyVector3 inertia_xxyyzz;
  TinyVector3 origin_rpy;
  TinyVector3 origin_xyz;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfContact {
  TinyUrdfContact()
      : lateral_friction(TinyConstants::fraction(1, 2)),
        restitution(TinyConstants::fraction(0, 1)),
        stiffness(TinyConstants::fraction(1, 1)),
        damping(TinyConstants::fraction(0, 1)) {}

  TinyScalar lateral_friction;
  TinyScalar restitution;
  TinyScalar stiffness;
  TinyScalar damping;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyVisualMaterial {
  TinyVector3<TinyScalar, TinyConstants> material_rgb;
  std::string texture_filename;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionSphere {
  TinyUrdfCollisionSphere() : m_radius(TinyConstants::one()) {}
  TinyScalar m_radius;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionPlane {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  TinyUrdfCollisionPlane()
      : m_normal(TinyVector3(TinyConstants::zero(), TinyConstants::zero(),
                             TinyConstants::one())),
        m_constant(TinyConstants::zero()) {}
  TinyVector3 m_normal;
  TinyScalar m_constant;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionCapsule {
  TinyUrdfCollisionCapsule()
      : m_radius(TinyConstants::one()), m_length(TinyConstants::one()) {}
  TinyScalar m_radius;
  TinyScalar m_length;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionBox {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  TinyUrdfCollisionBox()
      : m_extents(TinyVector3(TinyConstants::one(), TinyConstants::one(),
                              TinyConstants::one())) {}
  TinyVector3 m_extents;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionMesh {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;

  TinyUrdfCollisionMesh()
      : m_scale(TinyVector3(TinyConstants::one(), TinyConstants::one(),
                            TinyConstants::one())) {}
  std::string m_file_name;
  TinyVector3 m_scale;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfGeometry {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  TinyUrdfGeometry() : geom_type(TINY_MAX_GEOM_TYPE) {}

  // pybind11 doesn't like enum TinyGeometryTypes
  int geom_type;  // see TinyGeometryTypes in tiny_geometry.h

  TinyUrdfCollisionSphere<TinyScalar, TinyConstants> m_sphere;
  TinyUrdfCollisionCapsule<TinyScalar, TinyConstants> m_capsule;
  TinyUrdfCollisionBox<TinyScalar, TinyConstants> m_box;
  TinyUrdfCollisionMesh<TinyScalar, TinyConstants> m_mesh;
  TinyUrdfCollisionPlane<TinyScalar, TinyConstants> m_plane;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfVisual {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyUrdfGeometry<TinyScalar, TinyConstants> TinyUrdfGeometry;
  typedef ::TinyVisualMaterial<TinyScalar, TinyConstants> TinyVisualMaterial;
  TinyUrdfVisual()
      : origin_xyz(TinyVector3::zero()),
        origin_rpy(TinyVector3::zero()),
        has_local_material(false),
        sync_visual_body_uid1(-1),
        sync_visual_body_uid2(-1)

  {}
  TinyVector3 origin_rpy;
  TinyVector3 origin_xyz;
  TinyUrdfGeometry geometry;
  std::string material_name;
  TinyVisualMaterial m_material;
  std::string visual_name;
  bool has_local_material;
  int sync_visual_body_uid1;
  int sync_visual_body_uid2;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollision {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyUrdfGeometry<TinyScalar, TinyConstants> TinyUrdfGeometry;

  TinyUrdfCollision()
      : origin_xyz(TinyVector3::zero()),
        origin_rpy(TinyVector3::zero()),
        collision_group(0),
        collision_mask(0),
        flags(0) {}
  TinyVector3 origin_xyz;
  TinyVector3 origin_rpy;

  std::string collision_name;
  int collision_group;
  int collision_mask;
  int flags;
  TinyUrdfGeometry geometry;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfLink {
  typedef ::TinyUrdfCollision<TinyScalar, TinyConstants> TinyUrdfCollision;
  typedef ::TinyUrdfVisual<TinyScalar, TinyConstants> TinyUrdfVisual;
  typedef ::TinyUrdfContact<TinyScalar, TinyConstants> TinyUrdfContact;

  TinyUrdfLink() : m_parent_index(-2) {}
  std::string link_name;
  TinyUrdfInertial<TinyScalar, TinyConstants> urdf_inertial;
  std::vector<TinyUrdfVisual> urdf_visual_shapes;
  std::vector<TinyUrdfCollision> urdf_collision_shapes;
  std::vector<int> child_link_indices;
  TinyUrdfContact contact_info;
  int m_parent_index;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfJoint {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyUrdfLink<TinyScalar, TinyConstants> TinyUrdfLink;

  TinyUrdfJoint()
      : joint_type(JOINT_INVALID),
        joint_lower_limit(TinyConstants::one()),
        joint_upper_limit(TinyConstants::zero()),
        joint_origin_xyz(TinyVector3(TinyConstants::zero(),
                                     TinyConstants::zero(),
                                     TinyConstants::zero())),
        joint_origin_rpy(TinyVector3(TinyConstants::zero(),
                                     TinyConstants::zero(),
                                     TinyConstants::zero())),
        joint_axis_xyz(TinyVector3(TinyConstants::zero(), TinyConstants::zero(),
                                   TinyConstants::one())) {}
  std::string joint_name;
  // pybind11 doesn't like enum TinyJointType
  int joint_type;
  TinyScalar joint_lower_limit;
  TinyScalar joint_upper_limit;
  std::string parent_name;
  std::string child_name;
  TinyVector3 joint_origin_xyz;
  TinyVector3 joint_origin_rpy;
  TinyVector3 joint_axis_xyz;
};

enum TinyConversionReturnCode {
  kCONVERSION_OK = 1,
  kCONVERSION_JOINT_FAILED,
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfStructures {
  std::string m_robot_name;
  std::vector<TinyUrdfLink<TinyScalar, TinyConstants> > m_base_links;
  std::vector<TinyUrdfLink<TinyScalar, TinyConstants> > m_links;
  std::vector<TinyUrdfJoint<TinyScalar, TinyConstants> > m_joints;
  std::map<std::string, int> m_name_to_link_index;
  std::map<std::string, TinyVisualMaterial<TinyScalar, TinyConstants> >
      m_materials;
};

#endif  // TINY_URDF_STRUCTURES_H
