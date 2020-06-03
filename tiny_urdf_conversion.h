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

#ifndef _TINY_URDF_CONVERSION_H_
#define _TINY_URDF_CONVERSION_H_

#include <assert.h>

#include <map>
#include <string>
#include <vector>

#include "tiny_quaternion.h"
#include "tiny_symmetric_spatial_dyad.h"
#include "tiny_world.h"

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfInertial {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  TinyScalar mass;
  TinyVector3 inertia_xxyyzz;
  TinyVector3 origin_rpy;
  TinyVector3 origin_xyz;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfContact {
  TinyScalar lateral_friction;
  TinyScalar rolling_friction;
  TinyScalar spinning_friction;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfVisual {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  TinyVector3 origin_rpy;
  TinyVector3 origin_xyz;
  int geom_type;  // enum eUrdfGeomTypes in SharedMemoryPublic.h
  TinyScalar geom_radius;
  TinyVector3 geom_extents;
  TinyScalar geom_length;
  std::string geom_meshfilename;
  TinyVector3 geom_meshscale;
  TinyVector3 material_rgb;
  TinyScalar material_a;
  std::string material_name;
  int sync_visual_body_uid;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionSphere {
  TinyScalar m_radius;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionPlane {
  TinyVector3<TinyScalar, TinyConstants> m_normal;
  TinyScalar m_constant;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionCapsule {
  TinyScalar m_radius;
  TinyScalar m_length;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionBox {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  TinyVector3 m_extents;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollisionMesh {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  std::string m_file_name;
  TinyVector3 m_scale;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCollision {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  int geom_type;
  TinyVector3 origin_rpy;
  TinyVector3 origin_xyz;

  TinyUrdfCollisionSphere<TinyScalar, TinyConstants> m_sphere;
  TinyUrdfCollisionCapsule<TinyScalar, TinyConstants> m_capsule;
  TinyUrdfCollisionBox<TinyScalar, TinyConstants> m_box;
  TinyUrdfCollisionMesh<TinyScalar, TinyConstants> m_mesh;
  TinyUrdfCollisionPlane<TinyScalar, TinyConstants> m_plane;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfLink {
  typedef TinyUrdfCollision<TinyScalar, TinyConstants> TinyUrdfCollision;
  typedef TinyUrdfVisual<TinyScalar, TinyConstants> TinyUrdfVisual;
  std::string link_name;
  TinyUrdfInertial<TinyScalar, TinyConstants> urdf_inertial;
  std::vector<TinyUrdfVisual> urdf_visual_shapes;
  std::vector<TinyUrdfCollision> urdf_collision_shapes;
  std::vector<int> child_link_indices;
  int m_parent_index;
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfJoint {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef TinyUrdfLink<TinyScalar, TinyConstants> TinyUrdfLink;
  TinyUrdfLink link;
  std::string joint_name;
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
struct TinyUrdfEditor {
  std::vector<TinyUrdfLink<TinyScalar, TinyConstants> > m_base_links;
  std::vector<TinyUrdfLink<TinyScalar, TinyConstants> > m_links;
  std::vector<TinyUrdfJoint<TinyScalar, TinyConstants> > m_joints;

  std::map<std::string, int> m_name_to_link_index;

  void extract_link(int body_unique_id, int linkIndex,
                    class b3RobotSimulatorClientAPI_NoDirect& sim_api,
                    TinyUrdfLink<TinyScalar, TinyConstants>& urdfLink);

  void extract_urdf_structs(int body_unique_id,
                            class b3RobotSimulatorClientAPI_NoDirect& sim_api);

  void convert_collisions(TinyWorld<TinyScalar, TinyConstants>& world,
                          const TinyUrdfLink<TinyScalar, TinyConstants>& link,
                          TinyLink<TinyScalar, TinyConstants>& l,
                          class b3RobotSimulatorClientAPI_NoDirect& sim_api) {
    typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
    for (int c = 0; c < link.urdf_collision_shapes.size(); c++) {
      const TinyUrdfCollision<TinyScalar, TinyConstants>& col =
          link.urdf_collision_shapes[c];

      TinySpatialTransform<TinyScalar, TinyConstants> collision_offset;
      collision_offset.m_translation.setValue(
          col.origin_xyz[0], col.origin_xyz[1], col.origin_xyz[2]);
      TinyVector3 rpy;
      rpy.setValue(col.origin_rpy[0], col.origin_rpy[1], col.origin_rpy[2]);
      collision_offset.m_rotation.setEulerZYX(rpy[0], rpy[1], rpy[2]);

      switch (col.geom_type) {
        case GEOM_SPHERE: {
          TinyGeometry<TinyScalar, TinyConstants>* geom =
              world.create_sphere(col.m_sphere.m_radius);
          l.m_collision_geometries.push_back(geom);
          l.m_X_collisions.push_back(collision_offset);
          break;
        }
        case GEOM_BOX: {
          // col.m_box.m_extents.setValue(colShapeData.m_dimensions[0],
          // colShapeData.m_dimensions[1], colShapeData.m_dimensions[2]);
          // urdfLink.urdf_collision_shapes.push_back(col);
          break;
        }
        case GEOM_CAPSULE: {
          TinyGeometry<TinyScalar, TinyConstants>* geom =
              world.create_capsule(TinyScalar(col.m_capsule.m_radius),
                                   TinyScalar(col.m_capsule.m_length));
          l.m_collision_geometries.push_back(geom);
          l.m_X_collisions.push_back(collision_offset);
          break;
        }
        case GEOM_MESH: {
          // col.m_mesh.m_file_name = colShapeData.m_meshAssetFileName;
          // col.m_mesh.m_scale.setValue(colShapeData.m_dimensions[0],
          // colShapeData.m_dimensions[1], colShapeData.m_dimensions[2]);
          break;
        }
        case GEOM_PLANE: {
          TinyGeometry<TinyScalar, TinyConstants>* geom = world.create_plane();
          l.m_collision_geometries.push_back(geom);
          l.m_X_collisions.push_back(collision_offset);
          break;
        }
        default: {}
      };
    }
  }
  void convert_visuals(const TinyUrdfLink<TinyScalar, TinyConstants>& link,
                       TinyLink<TinyScalar, TinyConstants>& l,
                       class b3RobotSimulatorClientAPI_NoDirect& sim_api) {
    typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;

    for (int v = 0; v < link.urdf_visual_shapes.size(); v++) {
      const TinyUrdfVisual<TinyScalar, TinyConstants>& visual_shape =
          link.urdf_visual_shapes[v];
      b3RobotSimulatorCreateVisualShapeArgs args;
      args.m_shapeType = visual_shape.geom_type;
      TinySpatialTransform<TinyScalar, TinyConstants> visual_offset;
      visual_offset.m_translation.setValue(visual_shape.origin_xyz[0],
                                           visual_shape.origin_xyz[1],
                                           visual_shape.origin_xyz[2]);
      TinyVector3 rpy;
      rpy.setValue(visual_shape.origin_rpy[0], visual_shape.origin_rpy[1],
                   visual_shape.origin_rpy[2]);
      visual_offset.m_rotation.setEulerZYX(rpy[0], rpy[1], rpy[2]);

      printf("visual_shape.geom_type=%d\n", visual_shape.geom_type);
      switch (visual_shape.geom_type) {
        case GEOM_SPHERE: {
          args.m_radius = TinyConstants::getDouble(visual_shape.geom_radius);
          int vizShape = sim_api.createVisualShape(GEOM_SPHERE, args);
          if (vizShape < 0) {
            printf("Couldn't create sphere shape\n");
          }
          b3RobotSimulatorCreateMultiBodyArgs args2;
          args2.m_baseVisualShapeIndex = vizShape;
          args2.m_baseMass = 0;
          int viz_uid = sim_api.createMultiBody(args2);
          l.m_visual_uids.push_back(viz_uid);
          l.m_X_visuals.push_back(visual_offset);

          break;
        }
        case GEOM_CAPSULE: {
          args.m_radius = TinyConstants::getDouble(visual_shape.geom_radius);
          args.m_height = TinyConstants::getDouble(visual_shape.geom_length);

          int vizShape = sim_api.createVisualShape(GEOM_CAPSULE, args);
          if (vizShape < 0) {
            printf("Couldn't create capsule shape\n");
          }
          b3RobotSimulatorCreateMultiBodyArgs args2;
          args2.m_baseVisualShapeIndex = vizShape;
          args2.m_baseMass = 0;
          int viz_uid = sim_api.createMultiBody(args2);
          l.m_visual_uids.push_back(viz_uid);
          l.m_X_visuals.push_back(visual_offset);
          break;
        }
        case GEOM_BOX: {
          {
            TinyVector3 he =
                visual_shape.geom_extents * TinyConstants::fraction(1, 2);
            args.m_halfExtents.setValue(TinyConstants::getDouble(he[0]),
                                        TinyConstants::getDouble(he[1]),
                                        TinyConstants::getDouble(he[2]));
            int vizShape = sim_api.createVisualShape(GEOM_BOX, args);
            b3RobotSimulatorCreateMultiBodyArgs args2;
            args2.m_baseVisualShapeIndex = vizShape;
            args2.m_baseMass = 0;
            int viz_uid = sim_api.createMultiBody(args2);
            l.m_visual_uids.push_back(viz_uid);
            l.m_X_visuals.push_back(visual_offset);
            break;
          }
          case GEOM_MESH: {
            args.m_fileName = visual_shape.geom_meshfilename.c_str();
            // printf("mb mesh: %s\n", args.m_fileName);
            args.m_meshScale.setValue(
                TinyConstants::getDouble(visual_shape.geom_meshscale[0]),
                TinyConstants::getDouble(visual_shape.geom_meshscale[1]),
                TinyConstants::getDouble(visual_shape.geom_meshscale[2]));

            int vizShape = sim_api.createVisualShape(GEOM_MESH, args);
            if (vizShape < 0) {
              printf("Couldn't create sphere shape: %s\n", args.m_fileName);
            }

            // printf("vizShape=%d\n", vizShape);
            b3RobotSimulatorCreateMultiBodyArgs args2;
            args2.m_baseVisualShapeIndex = vizShape;
            args2.m_baseMass = 0;

            int viz_uid = sim_api.createMultiBody(args2);
            {
              b3RobotSimulatorChangeVisualShapeArgs args_change;
              args_change.m_objectUniqueId = viz_uid;
              args_change.m_linkIndex = -1;
              args_change.m_hasRgbaColor = true;
              args_change.m_rgbaColor.setValue(1, 1, 1, 1);
              sim_api.changeVisualShape(args_change);
            }
            l.m_visual_uids.push_back(viz_uid);
            l.m_X_visuals.push_back(visual_offset);
            break;
          }
          default: {}
        }
      }
    }
  }

  int convert_to_multi_body(TinyWorld<TinyScalar, TinyConstants>& world,
                            TinyMultiBody<TinyScalar, TinyConstants>& mb,
                            int body_unique_id,
                            class b3RobotSimulatorClientAPI_NoDirect& sim_api) {
    typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
    int return_code = kCONVERSION_OK;

    // start with base properties
    // mb->m_baseInertia
    TinyLink<TinyScalar, TinyConstants> dummy;
    convert_visuals(m_base_links[0], dummy, sim_api);
    {
      const TinyUrdfLink<TinyScalar, TinyConstants>& link = m_base_links[0];
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
      mb.m_baseInertia = TinySymmetricSpatialDyad<
          TinyScalar, TinyConstants>::computeInertiaDyad(mass, com, inertia_C);
    }

    for (int i = 0; i < dummy.m_visual_uids.size(); i++) {
      mb.m_visual_uids.push_back(dummy.m_visual_uids[i]);
      mb.m_X_visuals.push_back(dummy.m_X_visuals[i]);
    }

    convert_collisions(world, m_base_links[0], dummy, sim_api);
    for (int i = 0; i < dummy.m_collision_geometries.size(); i++) {
      mb.m_collision_geometries.push_back(dummy.m_collision_geometries[i]);
      mb.m_X_collisions.push_back(dummy.m_X_collisions[i]);
    }

    // then convert each link
    int num_links = this->m_joints.size();

    printf("-----------------------\n");
    printf("num_links=%d\n", num_links);
    printf("-----------------------\n");
    for (int i = 0; i < num_links; i++) {
      TinyLink<TinyScalar, TinyConstants> l;
      bool joint_conversion_ok = false;

      // convert from enum JointType (SharedMemoryPublic.h) to TinyJointType
      // (TinyMultiBody.h)
      switch (m_joints[i].joint_type) {
        case eFixedType: {
          printf("FixedType!\n");
          l.set_joint_type(JOINT_FIXED);
          joint_conversion_ok = true;
          break;
        }
        case eRevoluteType: {
          int non_zero_joint_axis_index = -1;
          for (int j = 0; j < 3; j++) {
            // approximate check needed?
            if (m_joints[i].joint_axis_xyz[j] == TinyConstants::one()) {
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
            l.set_joint_type(JOINT_REVOLUTE_AXIS, m_joints[i].joint_axis_xyz);
          }
          joint_conversion_ok = true;
          break;
        }
        case ePrismaticType: {
          int non_zero_joint_axis_index = -1;
          for (int j = 0; j < 3; j++) {
            // approximate check needed?
            if (m_joints[i].joint_axis_xyz[j] == TinyConstants::one()) {
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
            l.set_joint_type(JOINT_PRISMATIC_AXIS, m_joints[i].joint_axis_xyz);
          }
          joint_conversion_ok = true;
          break;
        }
        default: { return_code = kCONVERSION_JOINT_FAILED; }
      };

      if (return_code == kCONVERSION_OK) {
        l.m_X_T.m_rotation.set_identity();
        const TinyUrdfJoint<TinyScalar, TinyConstants>& j = m_joints[i];
        const TinyUrdfLink<TinyScalar, TinyConstants>& link = m_links[i];
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

        // convert visuals
        convert_visuals(link, l, sim_api);

        // convert collision geometries
        convert_collisions(world, link, l, sim_api);

        l.m_link_name = link.link_name;
        l.m_joint_name = j.joint_name;
        mb.attach(l, m_links[i].m_parent_index);
      }
    }
    return return_code;
  }
};

#endif  // _TINY_URDF_CONVERSION_H_
