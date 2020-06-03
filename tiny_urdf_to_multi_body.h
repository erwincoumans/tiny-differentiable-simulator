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

#ifndef TINY_URDF_TO_MULTI_BODY_H
#define TINY_URDF_TO_MULTI_BODY_H

#include <stdio.h>

#include "tiny_urdf_structures.h"

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfToMultiBody {
  typedef ::TinyUrdfStructures<TinyScalar, TinyConstants> TinyUrdfStructures;
  static int convert_to_multi_body(
      const TinyUrdfStructures& urdf_structures,
      TinyWorld<TinyScalar, TinyConstants>& world,
      TinyMultiBody<TinyScalar, TinyConstants>& mb) {
    typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
    int return_code = kCONVERSION_OK;

    // start with base properties
    // mb->m_baseInertia

    const TinyUrdfLink<TinyScalar, TinyConstants>& base_link =
        urdf_structures.m_base_links[0];

    {
      TinyScalar mass = base_link.urdf_inertial.mass;
      TinyVector3 local_inertia = base_link.urdf_inertial.inertia_xxyyzz;
      TinyVector3 com(base_link.urdf_inertial.origin_xyz[0],
                      base_link.urdf_inertial.origin_xyz[1],
                      base_link.urdf_inertial.origin_xyz[2]);
      TinyMatrix3x3<TinyScalar, TinyConstants> inertia_diag;
      inertia_diag.set_identity();
      inertia_diag(0, 0) = local_inertia[0];
      inertia_diag(1, 1) = local_inertia[1];
      inertia_diag(2, 2) = local_inertia[2];
      TinyMatrix3x3<TinyScalar, TinyConstants> rot;
      rot.setEulerZYX(base_link.urdf_inertial.origin_rpy[0],
                      base_link.urdf_inertial.origin_rpy[1],
                      base_link.urdf_inertial.origin_rpy[2]);
      TinyMatrix3x3<TinyScalar, TinyConstants> inertia_C = rot * inertia_diag;
      mb.m_baseInertia = TinySymmetricSpatialDyad<
          TinyScalar, TinyConstants>::computeInertiaDyad(mass, com, inertia_C);
    }

    for (int i = 0; i < base_link.urdf_visual_shapes.size(); i++) {
      const TinyUrdfVisual<TinyScalar, TinyConstants>& visual_shape =
          base_link.urdf_visual_shapes[i];

      mb.m_visual_uids1.push_back(visual_shape.sync_visual_body_uid1);
      mb.m_visual_uids2.push_back(visual_shape.sync_visual_body_uid2);
      TinySpatialTransform<TinyScalar, TinyConstants> visual_offset;
      visual_offset.m_translation.setValue(visual_shape.origin_xyz[0],
                                           visual_shape.origin_xyz[1],
                                           visual_shape.origin_xyz[2]);
      TinyVector3 rpy;
      rpy.setValue(visual_shape.origin_rpy[0], visual_shape.origin_rpy[1],
                   visual_shape.origin_rpy[2]);
      visual_offset.m_rotation.setEulerZYX(rpy[0], rpy[1], rpy[2]);
      mb.m_X_visuals.push_back(visual_offset);
    }

    TinyLink<TinyScalar, TinyConstants> dummy;
    convert_collisions(world, base_link, dummy);
    for (int i = 0; i < dummy.m_collision_geometries.size(); i++) {
      mb.m_collision_geometries.push_back(dummy.m_collision_geometries[i]);
      mb.m_X_collisions.push_back(dummy.m_X_collisions[i]);
    }

    // then convert each link
    int num_links = urdf_structures.m_joints.size();

    printf("-----------------------\n");
    printf("num_links=%d\n", num_links);
    printf("-----------------------\n");
    for (int i = 0; i < num_links; i++) {
      TinyLink<TinyScalar, TinyConstants> l;
      bool joint_conversion_ok = false;

      // convert from enum JointType (SharedMemoryPublic.h) to TinyJointType
      // (TinyMultiBody.h)
      switch (urdf_structures.m_joints[i].joint_type) {
        case JOINT_FIXED: {
          printf("FixedType!\n");
          l.set_joint_type(JOINT_FIXED);
          joint_conversion_ok = true;
          break;
        }
        case JOINT_REVOLUTE_AXIS: {
          int non_zero_joint_axis_index = -1;
          for (int j = 0; j < 3; j++) {
            // approximate check needed?
            if (urdf_structures.m_joints[i].joint_axis_xyz[j] ==
                TinyConstants::one()) {
              if (non_zero_joint_axis_index >= 0) {
                break;
              }
              non_zero_joint_axis_index = j;
            }
          }
          if (non_zero_joint_axis_index >= 0) {
            l.set_joint_type(
                TinyJointType(JOINT_REVOLUTE_X + non_zero_joint_axis_index));
          } else {
            l.set_joint_type(JOINT_REVOLUTE_AXIS,
                             urdf_structures.m_joints[i].joint_axis_xyz);
          }
          joint_conversion_ok = true;
          break;
        }
        case JOINT_PRISMATIC_AXIS: {
          int non_zero_joint_axis_index = -1;
          for (int j = 0; j < 3; j++) {
            // approximate check needed?
            if (urdf_structures.m_joints[i].joint_axis_xyz[j] ==
                TinyConstants::one()) {
              if (non_zero_joint_axis_index >= 0) {
                break;
              }
              non_zero_joint_axis_index = j;
            }
          }
          if (non_zero_joint_axis_index >= 0) {
            l.set_joint_type(
                TinyJointType(JOINT_PRISMATIC_X + non_zero_joint_axis_index));
          } else {
            l.set_joint_type(JOINT_PRISMATIC_AXIS,
                             urdf_structures.m_joints[i].joint_axis_xyz);
          }
          joint_conversion_ok = true;
          break;
        }
        default: { return_code = kCONVERSION_JOINT_FAILED; }
      };

      if (return_code == kCONVERSION_OK) {
        l.m_X_T.m_rotation.set_identity();
        const TinyUrdfJoint<TinyScalar, TinyConstants>& j =
            urdf_structures.m_joints[i];
        const TinyUrdfLink<TinyScalar, TinyConstants>& link =
            urdf_structures.m_links[i];
        l.m_X_T.m_translation.setValue(j.joint_origin_xyz[0],
                                       j.joint_origin_xyz[1],
                                       j.joint_origin_xyz[2]);
        l.m_X_T.m_rotation.setEulerZYX(j.joint_origin_rpy[0],
                                       j.joint_origin_rpy[1],
                                       j.joint_origin_rpy[2]);
        TinyScalar mass = link.urdf_inertial.mass;
        TinyVector3 local_inertia = link.urdf_inertial.inertia_xxyyzz;
        TinyVector3 com(link.urdf_inertial.origin_xyz[0],
                        link.urdf_inertial.origin_xyz[1],
                        link.urdf_inertial.origin_xyz[2]);
        TinyMatrix3x3<TinyScalar, TinyConstants> inertia_diag;
        inertia_diag.set_identity();
        inertia_diag(0, 0) = local_inertia[0];
        inertia_diag(1, 1) = local_inertia[1];
        inertia_diag(2, 2) = local_inertia[2];
        TinyMatrix3x3<TinyScalar, TinyConstants> rot;
        rot.setEulerZYX(link.urdf_inertial.origin_rpy[0],
                        link.urdf_inertial.origin_rpy[1],
                        link.urdf_inertial.origin_rpy[2]);
        TinyMatrix3x3<TinyScalar, TinyConstants> inertia_C = rot * inertia_diag;
        l.m_I = TinySymmetricSpatialDyad<
            TinyScalar, TinyConstants>::computeInertiaDyad(mass, com,
                                                           inertia_C);

        for (int i = 0; i < link.urdf_visual_shapes.size(); i++) {
          const TinyUrdfVisual<TinyScalar, TinyConstants>& visual_shape =
              link.urdf_visual_shapes[i];

          l.m_visual_uids1.push_back(visual_shape.sync_visual_body_uid1);
          l.m_visual_uids2.push_back(visual_shape.sync_visual_body_uid2);
          TinySpatialTransform<TinyScalar, TinyConstants> visual_offset;
          visual_offset.m_translation.setValue(visual_shape.origin_xyz[0],
                                               visual_shape.origin_xyz[1],
                                               visual_shape.origin_xyz[2]);
          TinyVector3 rpy;
          rpy.setValue(visual_shape.origin_rpy[0], visual_shape.origin_rpy[1],
                       visual_shape.origin_rpy[2]);
          visual_offset.m_rotation.setEulerZYX(rpy[0], rpy[1], rpy[2]);
          l.m_X_visuals.push_back(visual_offset);
        }

        // convert collision geometries
        convert_collisions(world, link, l);

        l.m_link_name = link.link_name;
        l.m_joint_name = j.joint_name;
        mb.attach(l, urdf_structures.m_links[i].m_parent_index);
      }
    }
    return return_code;
  }

  static void convert_collisions(
      TinyWorld<TinyScalar, TinyConstants>& world,
      const TinyUrdfLink<TinyScalar, TinyConstants>& link,
      TinyLink<TinyScalar, TinyConstants>& l) {
    typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
    for (int c = 0; c < link.urdf_collision_shapes.size(); c++) {
      const TinyUrdfCollision<TinyScalar, TinyConstants>& col =
          link.urdf_collision_shapes[c];

      TinySpatialTransform<TinyScalar, TinyConstants> collision_offset;
      collision_offset.m_translation.setValue(
          col.origin_xyz[0], col.origin_xyz[1], col.origin_xyz[2]);
      TinyVector3 rpy;
      rpy.setValue(col.origin_rpy[0], col.origin_rpy[1], col.origin_rpy[2]);
      collision_offset.m_rotation.setEulerZYX(rpy[0], rpy[1], rpy[2]);

      switch (col.geometry.geom_type) {
        case TINY_SPHERE_TYPE: {
          TinyGeometry<TinyScalar, TinyConstants>* geom =
              world.create_sphere(col.geometry.m_sphere.m_radius);
          l.m_collision_geometries.push_back(geom);
          l.m_X_collisions.push_back(collision_offset);
          break;
        }
          // case BOX_TYPE: {
          //     // col.m_box.m_extents.setValue(colShapeData.m_dimensions[0],
          //     // colShapeData.m_dimensions[1], colShapeData.m_dimensions[2]);
          //     // urdfLink.urdf_collision_shapes.push_back(col);
          //     break;
          // }
        case TINY_CAPSULE_TYPE: {
          TinyGeometry<TinyScalar, TinyConstants>* geom =
              world.create_capsule(TinyScalar(col.geometry.m_capsule.m_radius),
                                   TinyScalar(col.geometry.m_capsule.m_length));
          l.m_collision_geometries.push_back(geom);
          l.m_X_collisions.push_back(collision_offset);
          break;
        }
        // case GEOM_MESH: {
        //    // col.m_mesh.m_file_name = colShapeData.m_meshAssetFileName;
        //    // col.m_mesh.m_scale.setValue(colShapeData.m_dimensions[0],
        //    // colShapeData.m_dimensions[1], colShapeData.m_dimensions[2]);
        //    break;
        //}
        case TINY_PLANE_TYPE: {
          TinyGeometry<TinyScalar, TinyConstants>* geom = world.create_plane();
          l.m_collision_geometries.push_back(geom);
          l.m_X_collisions.push_back(collision_offset);
          break;
        }
        default: {}
      };
    }
  }
};

#endif  // TINY_URDF_TO_MULTI_BODY_H
