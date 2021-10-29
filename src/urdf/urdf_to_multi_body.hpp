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

#include <stdio.h>

#include "../math/transform.hpp"
#include "../world.hpp"
#include "urdf_structures.hpp"

struct VisualInstanceGenerator {
  virtual void create_visual_instance(int shape_uid,
                                      std::vector<int> &instances) = 0;
  virtual ~VisualInstanceGenerator() {}
};

namespace tds {
template <typename Algebra>
struct UrdfToMultiBody {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  using Transform = tds::Transform<Algebra>;
  using UrdfStructures = tds::UrdfStructures<Algebra>;
  using RigidBodyInertia = tds::RigidBodyInertia<Algebra>;

  static int convert_to_multi_body(const UrdfStructures &urdf_structures,
                                   World<Algebra> &world,
                                   MultiBody<Algebra> &mb,
                                   VisualInstanceGenerator *vig) {
    int return_code = kCONVERSION_OK;

    // start with base properties
    // mb->m_baseInertia

    const UrdfLink<Algebra> &base_link = urdf_structures.base_links[0];

    {
      Scalar mass = base_link.urdf_inertial.mass;
      Vector3 local_inertia = base_link.urdf_inertial.inertia_xxyyzz;
      Vector3 com(base_link.urdf_inertial.origin_xyz[0],
                  base_link.urdf_inertial.origin_xyz[1],
                  base_link.urdf_inertial.origin_xyz[2]);
      Matrix3 inertia_diag = Algebra::diagonal3(local_inertia);
      mb.base_rbi_ = RigidBodyInertia(mass, com, inertia_diag);
      // apply rotation
      Transform tf;
      tf.rotation =
          Algebra::rotation_zyx_matrix(base_link.urdf_inertial.origin_rpy[0],
                                       base_link.urdf_inertial.origin_rpy[1],
                                       base_link.urdf_inertial.origin_rpy[2]);
      mb.base_rbi_ = tf.apply(mb.base_rbi_);
    }
    for (std::size_t i = 0; i < base_link.urdf_visual_shapes.size(); i++) {
      const UrdfVisual<Algebra> &visual_shape = base_link.urdf_visual_shapes[i];

      if (vig) {
        vig->create_visual_instance(visual_shape.visual_shape_uid,
                                    mb.visual_instance_uids_);
      }
      Transform visual_offset;
      visual_offset.translation =
          Vector3(visual_shape.origin_xyz[0], visual_shape.origin_xyz[1],
                  visual_shape.origin_xyz[2]);
      Vector3 rpy;
      rpy = Vector3(visual_shape.origin_rpy[0], visual_shape.origin_rpy[1],
                    visual_shape.origin_rpy[2]);
      visual_offset.rotation =
          Algebra::rotation_zyx_matrix(rpy[0], rpy[1], rpy[2]);
      mb.X_visuals_.push_back(visual_offset);
    }

    // convert collision geoms, transforms for base
    Link<Algebra> dummy;
    convert_collisions(world, base_link, dummy);
    for (std::size_t i = 0; i < dummy.collision_geometries.size(); i++) {
      mb.collision_geometries().push_back(dummy.collision_geometries[i]);
      mb.collision_transforms().push_back(dummy.X_collisions[i]);
    }

    // then convert each link
    int num_links = urdf_structures.joints.size();

#if DEBUG
    printf("-----------------------\n");
    printf("num_links=%d\n", num_links);
    printf("-----------------------\n");
#endif
    for (int i = 0; i < num_links; i++) {
      Link<Algebra> l;
      bool joint_conversion_ok = false;

      // convert from enum JointType (SharedMemoryPublic.h) to JointType
      // (link.hpp)
      switch (urdf_structures.joints[i].joint_type) {
        case JOINT_FIXED: {
          l.set_joint_type(JOINT_FIXED);
          joint_conversion_ok = true;
          break;
        }
        case JOINT_REVOLUTE_AXIS: {
          int non_zero_joint_axis_index = -1;
          for (int j = 0; j < 3; j++) {
            // approximate check needed?
            if (urdf_structures.joints[i].joint_axis_xyz[j] == Algebra::one()) {
              if (non_zero_joint_axis_index >= 0) {
                break;
              }
              non_zero_joint_axis_index = j;
            }
          }
          if (non_zero_joint_axis_index >= 0) {
            l.set_joint_type(
                JointType(JOINT_REVOLUTE_X + non_zero_joint_axis_index));
          } else {
            l.set_joint_type(JOINT_REVOLUTE_AXIS,
                             urdf_structures.joints[i].joint_axis_xyz);
          }
          joint_conversion_ok = true;
          break;
        }
        case JOINT_PRISMATIC_AXIS: {
          int non_zero_joint_axis_index = -1;
          for (int j = 0; j < 3; j++) {
            // approximate check needed?
            if (urdf_structures.joints[i].joint_axis_xyz[j] == Algebra::one()) {
              if (non_zero_joint_axis_index >= 0) {
                break;
              }
              non_zero_joint_axis_index = j;
            }
          }
          if (non_zero_joint_axis_index >= 0) {
            l.set_joint_type(
                JointType(JOINT_PRISMATIC_X + non_zero_joint_axis_index));
          } else {
            l.set_joint_type(JOINT_PRISMATIC_AXIS,
                             urdf_structures.joints[i].joint_axis_xyz);
          }
          joint_conversion_ok = true;
          break;
        }
        case JOINT_SPHERICAL: {
          joint_conversion_ok = true;
          l.set_joint_type(JOINT_SPHERICAL);
          break;
        }
        default: {
          return_code = kCONVERSION_JOINT_FAILED;
        }
      };

      if (return_code == kCONVERSION_OK) {
        l.X_T.rotation = Algebra::eye3();
        const UrdfJoint<Algebra> &j = urdf_structures.joints[i];
        const UrdfLink<Algebra> &link = urdf_structures.links[i];
        l.X_T.translation =
            Vector3(j.joint_origin_xyz[0], j.joint_origin_xyz[1],
                    j.joint_origin_xyz[2]);
        l.X_T.rotation = Algebra::rotation_zyx_matrix(j.joint_origin_rpy[0],
                                                      j.joint_origin_rpy[1],
                                                      j.joint_origin_rpy[2]);
        Scalar mass = link.urdf_inertial.mass;
        Vector3 local_inertia = link.urdf_inertial.inertia_xxyyzz;
        Vector3 com(link.urdf_inertial.origin_xyz[0],
                    link.urdf_inertial.origin_xyz[1],
                    link.urdf_inertial.origin_xyz[2]);
        Matrix3 inertia_diag = Algebra::diagonal3(local_inertia);

        // apply rotation
        Transform tf;
        tf.rotation = Algebra::rotation_zyx_matrix(
            link.urdf_inertial.origin_rpy[0], link.urdf_inertial.origin_rpy[1],
            link.urdf_inertial.origin_rpy[2]);
        l.rbi = RigidBodyInertia(mass, com, inertia_diag);
        l.rbi = tf.apply(l.rbi);

        for (std::size_t i = 0; i < link.urdf_visual_shapes.size(); i++) {
          const UrdfVisual<Algebra> &visual_shape = link.urdf_visual_shapes[i];

          if (vig) {
            vig->create_visual_instance(visual_shape.visual_shape_uid,
                                        l.visual_instance_uids);
          }
          Transform visual_offset;
          visual_offset.translation =
              Vector3(visual_shape.origin_xyz[0], visual_shape.origin_xyz[1],
                      visual_shape.origin_xyz[2]);
          Vector3 rpy;
          rpy = Vector3(visual_shape.origin_rpy[0], visual_shape.origin_rpy[1],
                        visual_shape.origin_rpy[2]);
          visual_offset.rotation =
              Algebra::rotation_zyx_matrix(rpy[0], rpy[1], rpy[2]);
          l.X_visuals.push_back(visual_offset);
        }

        // convert collision geometries
        convert_collisions(world, link, l);

        l.link_name = link.link_name;
        l.joint_name = j.joint_name;
        mb.attach(l, urdf_structures.links[i].parent_index);
      }
    }
    return return_code;
  }

  static void convert_collisions(World<Algebra> &world,
                                 const UrdfLink<Algebra> &link,
                                 Link<Algebra> &l) {
    for (std::size_t c = 0; c < link.urdf_collision_shapes.size(); c++) {
      const UrdfCollision<Algebra> &col = link.urdf_collision_shapes[c];

      Transform collision_offset;
      collision_offset.translation =
          Vector3(col.origin_xyz[0], col.origin_xyz[1], col.origin_xyz[2]);
      Vector3 rpy = col.origin_rpy;
      collision_offset.rotation =
          Algebra::rotation_zyx_matrix(rpy[0], rpy[1], rpy[2]);

      switch (col.geometry.geom_type) {
        case TINY_SPHERE_TYPE: {
          Geometry<Algebra> *geom =
              world.create_sphere(col.geometry.sphere.radius);
          l.collision_geometries.push_back(geom);
          l.X_collisions.push_back(collision_offset);
          break;
        }
        case TINY_BOX_TYPE: {
          Vector3 extents(col.geometry.box.extents[0],
                          col.geometry.box.extents[1],
                          col.geometry.box.extents[2]);
          Geometry<Algebra> *geom = world.create_box(extents);
          l.collision_geometries.push_back(geom);
          l.X_collisions.push_back(collision_offset);
          break;
        }
        case TINY_CAPSULE_TYPE: {
          Geometry<Algebra> *geom =
              world.create_capsule(Scalar(col.geometry.capsule.radius),
                                   Scalar(col.geometry.capsule.length));
          l.collision_geometries.push_back(geom);
          l.X_collisions.push_back(collision_offset);
          break;
        }
        // case GEOM_MESH: {
        //    // col.mesh.file_name = colShapeData.meshAssetFileName;
        //    // col.mesh.scale = Vector3(colShapeData.dimensions[0],
        //    // colShapeData.dimensions[1], colShapeData.dimensions[2]);
        //    break;
        //}
        case TINY_PLANE_TYPE: {
          Plane<Algebra> *geom = world.create_plane();
          geom->set_normal(col.geometry.plane.normal);
          l.collision_geometries.push_back(geom);
          l.X_collisions.push_back(collision_offset);
          break;
        }
        default: {
        }
      };
    }
  }
};
}  // namespace tds
