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

#include "multi_body.hpp"
#include "urdf_structures.hpp"
#include "visualizer/pybullet/pybullet_visualizer_api.h"

namespace tds {
template <typename Algebra>
struct PyBulletUrdfImport {
  typedef tds::UrdfStructures<Algebra> UrdfStructures;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;
  typedef tds::Transform<Algebra> Transform;

  static void extract_urdf_structs(UrdfStructures& urdf_structures,
                                   int body_unique_id,
                                   PyBulletVisualizerAPI* sim_api,
                                   PyBulletVisualizerAPI* viz_api) {
    btVector3 basePos;
    btQuaternion baseOrn;
    sim_api->getBasePositionAndOrientation(body_unique_id, basePos, baseOrn);
    {
      int base_link_index = -1;
      UrdfLink<Algebra> base_link;
      extract_link(urdf_structures, body_unique_id, base_link_index, sim_api,
                   viz_api, base_link);
      base_link.parent_index = -2;
      urdf_structures.base_links.push_back(base_link);
    }

    int num_joints = sim_api->getNumJoints(body_unique_id);
    for (int link_index = 0; link_index < num_joints; link_index++) {
      UrdfLink<Algebra> child_link;
      extract_link(urdf_structures, body_unique_id, link_index, sim_api,
                   viz_api, child_link);

      b3JointInfo jointInfo;
      sim_api->getJointInfo(body_unique_id, link_index, &jointInfo);

      UrdfJoint<Algebra> joint;

      joint.child_name = child_link.link_name;
      joint.joint_name = jointInfo.m_jointName;
      switch (jointInfo.m_jointType) {
        case eFixedType: {
          joint.joint_type = JOINT_FIXED;
          break;
        }

        case eRevoluteType: {
          joint.joint_type = JOINT_REVOLUTE_AXIS;
          break;
        }
        case ePrismaticType: {
          joint.joint_type = JOINT_PRISMATIC_AXIS;
          break;
        }

        default: {
          joint.joint_type = JOINT_INVALID;
          printf("cannot convert joint type: %d\n", jointInfo.m_jointType);
        }
      };

      joint.joint_axis_xyz =
          Vector3(Algebra::from_double(jointInfo.m_jointAxis[0]),
                  Algebra::from_double(jointInfo.m_jointAxis[1]),
                  Algebra::from_double(jointInfo.m_jointAxis[2]));

      if (jointInfo.m_parentIndex < 0) {
        joint.parent_name = urdf_structures.base_links[0].link_name;
        urdf_structures.base_links[0].child_link_indices.push_back(link_index);

      } else {
        b3JointInfo parentJointInfo;
        sim_api->getJointInfo(body_unique_id, jointInfo.m_parentIndex,
                              &parentJointInfo);
        joint.parent_name = parentJointInfo.m_linkName;
        urdf_structures.links[jointInfo.m_parentIndex]
            .child_link_indices.push_back(link_index);
      }
      child_link.parent_index = jointInfo.m_parentIndex;

      b3DynamicsInfo dynamics_info_child;
      sim_api->getDynamicsInfo(body_unique_id, link_index,
                               &dynamics_info_child);
      btVector3 childInertiaPos(dynamics_info_child.m_localInertialFrame[0],
                                dynamics_info_child.m_localInertialFrame[1],
                                dynamics_info_child.m_localInertialFrame[2]);
      btQuaternion childInertiaOrn(dynamics_info_child.m_localInertialFrame[3],
                                   dynamics_info_child.m_localInertialFrame[4],
                                   dynamics_info_child.m_localInertialFrame[5],
                                   dynamics_info_child.m_localInertialFrame[6]);
      btTransform childInertia(childInertiaOrn, childInertiaPos);
      btVector3 parentCom2JointPos(jointInfo.m_parentFrame[0],
                                   jointInfo.m_parentFrame[1],
                                   jointInfo.m_parentFrame[2]);
      btQuaternion parentCom2JointOrn(
          jointInfo.m_parentFrame[3], jointInfo.m_parentFrame[4],
          jointInfo.m_parentFrame[5], jointInfo.m_parentFrame[6]);
      btTransform parentCom2Joint(parentCom2JointOrn, parentCom2JointPos);
      btTransform tmp_ = childInertia * parentCom2Joint;
      btTransform tmpInv = tmp_.inverse();
      b3DynamicsInfo dynamics_info_parent;
      sim_api->getDynamicsInfo(body_unique_id, jointInfo.m_parentIndex,
                               &dynamics_info_parent);
      btVector3 parentInertiaPos(dynamics_info_parent.m_localInertialFrame[0],
                                 dynamics_info_parent.m_localInertialFrame[1],
                                 dynamics_info_parent.m_localInertialFrame[2]);
      btQuaternion parentInertiaOrn(
          dynamics_info_parent.m_localInertialFrame[3],
          dynamics_info_parent.m_localInertialFrame[4],
          dynamics_info_parent.m_localInertialFrame[5],
          dynamics_info_parent.m_localInertialFrame[6]);
      btTransform parentInertia(parentInertiaOrn, parentInertiaPos);
      btTransform pos_ = parentInertia * tmpInv;

      btTransform tmp2(btQuaternion::getIdentity(), parentCom2JointPos);
      btTransform pos = parentInertia * tmp2;
      joint.joint_origin_xyz =
          Vector3(Algebra::from_double(pos.getOrigin()[0]),
                  Algebra::from_double(pos.getOrigin()[1]),
                  Algebra::from_double(pos.getOrigin()[2]));
      btScalar roll, pitch, yaw;
      pos_.getRotation().getEulerZYX(yaw, pitch, roll);
      btVector3 rpy = btVector3(roll, pitch, yaw);
      joint.joint_origin_rpy =
          Vector3(Algebra::from_double(rpy[0]), Algebra::from_double(rpy[1]),
                  Algebra::from_double(rpy[2]));
      urdf_structures.links.push_back(child_link);
      urdf_structures.joints.push_back(joint);
    }
  }

  static void sync_graphics_transforms(const MultiBody<Algebra>* body,
                                       PyBulletVisualizerAPI* viz_api) {
    for (std::size_t v = 0; v < body->visual_instance_uids().size(); v++) {
      int visual_id = body->visual_instance_uids()[v];
      Transform geom_X_world = body->base_X_world() * body->X_visuals()[v];
      btVector3 base_pos(Algebra::to_double(geom_X_world.translation[0]),
                         Algebra::to_double(geom_X_world.translation[1]),
                         Algebra::to_double(geom_X_world.translation[2]));
      typename Algebra::Matrix3 rot_mat = geom_X_world.rotation;
      Quaternion rot = Algebra::matrix_to_quat(rot_mat);
      btQuaternion base_orn(Algebra::to_double(Algebra::quat_x(rot)),
                            Algebra::to_double(Algebra::quat_y(rot)),
                            Algebra::to_double(Algebra::quat_z(rot)),
                            Algebra::to_double(Algebra::quat_w(rot)));
      viz_api->resetBasePositionAndOrientation(visual_id, base_pos, base_orn);
    }

    for (std::size_t l = 0; l < body->num_links(); l++) {
      for (std::size_t v = 0; v < (*body)[l].visual_instance_uids.size(); v++) {
        int visual_id = (*body)[l].visual_instance_uids[v];
        Transform geom_X_world = (*body)[l].X_world * (*body)[l].X_visuals[v];
        btVector3 base_pos(Algebra::to_double(geom_X_world.translation[0]),
                           Algebra::to_double(geom_X_world.translation[1]),
                           Algebra::to_double(geom_X_world.translation[2]));
        Quaternion rot = Algebra::matrix_to_quat(geom_X_world.rotation);
        btQuaternion base_orn(Algebra::to_double(Algebra::quat_x(rot)),
                              Algebra::to_double(Algebra::quat_y(rot)),
                              Algebra::to_double(Algebra::quat_z(rot)),
                              Algebra::to_double(Algebra::quat_w(rot)));
        viz_api->resetBasePositionAndOrientation(visual_id, base_pos, base_orn);
      }
    }
  }
  static void extract_link(UrdfStructures& urdf_structures, int body_unique_id,
                           int linkIndex, PyBulletVisualizerAPI* sim_api,
                           PyBulletVisualizerAPI* viz_api,
                           UrdfLink<Algebra>& urdfLink) {
    b3BodyInfo bodyInfo;
    sim_api->getBodyInfo(body_unique_id, &bodyInfo);

    if (linkIndex == -1) {
      urdfLink.link_name = bodyInfo.m_baseName;
    } else {
      b3JointInfo jointInfo;
      sim_api->getJointInfo(body_unique_id, linkIndex, &jointInfo);
      urdfLink.link_name = jointInfo.m_linkName;
    }

    b3DynamicsInfo dyn;
    sim_api->getDynamicsInfo(body_unique_id, linkIndex, &dyn);

    urdfLink.urdf_inertial.mass = Algebra::from_double(dyn.m_mass);
    urdfLink.urdf_inertial.inertia_xxyyzz =
        Vector3(Algebra::from_double(dyn.m_localInertialDiagonal[0]),
                Algebra::from_double(dyn.m_localInertialDiagonal[1]),
                Algebra::from_double(dyn.m_localInertialDiagonal[2]));
    urdfLink.urdf_inertial.origin_xyz =
        Vector3(Algebra::from_double(dyn.m_localInertialFrame[0]),
                Algebra::from_double(dyn.m_localInertialFrame[1]),
                Algebra::from_double(dyn.m_localInertialFrame[2]));
    btVector3 rpy = sim_api->getEulerFromQuaternion(
        btQuaternion(dyn.m_localInertialFrame[3], dyn.m_localInertialFrame[4],
                     dyn.m_localInertialFrame[5], dyn.m_localInertialFrame[6]));
    urdfLink.urdf_inertial.origin_rpy =
        Vector3(Algebra::from_double(rpy[0]), Algebra::from_double(rpy[1]),
                Algebra::from_double(rpy[2]));

    // visual shapes
    b3VisualShapeInformation visualShapeInfo;
    visualShapeInfo.m_visualShapeData = new b3VisualShapeData;
    sim_api->getVisualShapeData(body_unique_id, visualShapeInfo);

    for (int i = 0; i < visualShapeInfo.m_numVisualShapes; i++) {
      const b3VisualShapeData& visual = visualShapeInfo.m_visualShapeData[i];
      if (visual.m_linkIndex == linkIndex) {
        UrdfVisual<Algebra> viz;
        // offset
        viz.origin_xyz =
            Vector3(Algebra::from_double(visual.m_localVisualFrame[0]),
                    Algebra::from_double(visual.m_localVisualFrame[1]),
                    Algebra::from_double(visual.m_localVisualFrame[2]));
        btVector3 rpy = sim_api->getEulerFromQuaternion(btQuaternion(
            visual.m_localVisualFrame[3], visual.m_localVisualFrame[4],
            visual.m_localVisualFrame[5], visual.m_localVisualFrame[6]));
        viz.origin_rpy =
            Vector3(Algebra::from_double(rpy[0]), Algebra::from_double(rpy[1]),
                    Algebra::from_double(rpy[2]));

        viz.material.material_rgb =
            Vector3(Algebra::from_double(visual.m_rgbaColor[0]),
                    Algebra::from_double(visual.m_rgbaColor[1]),
                    Algebra::from_double(visual.m_rgbaColor[2]));
        // viz.material_a =
        // Algebra::from_double(visual.m_rgbaColor[3]);

        // try to load for now, until we can manually 'override' the shape world
        // transform
        switch (visual.m_visualGeometryType) {
          case GEOM_SPHERE: {
            viz.geometry.sphere.radius =
                Algebra::from_double(visual.m_dimensions[0]);
            viz.geometry.geom_type = TINY_SPHERE_TYPE;
            break;
          }
          case GEOM_CAPSULE: {
            viz.geometry.capsule.length =
                Algebra::from_double(visual.m_dimensions[0]);
            viz.geometry.capsule.radius =
                Algebra::from_double(visual.m_dimensions[1]);
            viz.geometry.geom_type = TINY_CAPSULE_TYPE;
            break;
          }
          case GEOM_BOX: {
            Vector3 halfExtents;
            halfExtents = Vector3(Algebra::from_double(visual.m_dimensions[0]),
                                  Algebra::from_double(visual.m_dimensions[1]),
                                  Algebra::from_double(visual.m_dimensions[2]));
            viz.geometry.box.extents = halfExtents * Algebra::fraction(2, 1);
            viz.geometry.geom_type = TINY_BOX_TYPE;
            break;
          }
          case GEOM_MESH: {
            viz.geometry.mesh.scale =
                Vector3(Algebra::from_double(visual.m_dimensions[0]),
                        Algebra::from_double(visual.m_dimensions[1]),
                        Algebra::from_double(visual.m_dimensions[2]));
            viz.geometry.mesh.file_name = visual.m_meshAssetFileName;
            printf("extract mesh: %s\n", viz.geometry.mesh.file_name.c_str());
            printf("extract scale: %f,%f,%f\n",
                   Algebra::to_double(viz.geometry.mesh.scale[0]),
                   Algebra::to_double(viz.geometry.mesh.scale[1]),
                   Algebra::to_double(viz.geometry.mesh.scale[2]));
            viz.geometry.geom_type = TINY_MESH_TYPE;
            break;
          }
          default: {
          }
        }

        urdfLink.urdf_visual_shapes.push_back(viz);
      }
    }

    convert_visuals(urdf_structures, urdfLink, viz_api);
    // collision shapes, only convert spheres for now
    b3CollisionShapeInformation collisionShapeInfo;
    sim_api->getCollisionShapeData(body_unique_id, linkIndex,
                                   collisionShapeInfo);

    for (int i = 0; i < collisionShapeInfo.m_numCollisionShapes; i++) {
      const b3CollisionShapeData& colShapeData =
          collisionShapeInfo.m_collisionShapeData[i];
      UrdfCollision<Algebra> col;
      btVector3 inertial_pos(dyn.m_localInertialFrame[0],
                             dyn.m_localInertialFrame[1],
                             dyn.m_localInertialFrame[2]);
      btQuaternion inertial_orn(
          dyn.m_localInertialFrame[3], dyn.m_localInertialFrame[4],
          dyn.m_localInertialFrame[5], dyn.m_localInertialFrame[6]);

      btTransform inertial_tr(inertial_orn, inertial_pos);
      btTransform col_local_tr;
      col_local_tr.setOrigin(btVector3(colShapeData.m_localCollisionFrame[0],
                                       colShapeData.m_localCollisionFrame[1],
                                       colShapeData.m_localCollisionFrame[2]));
      col_local_tr.setRotation(
          btQuaternion(colShapeData.m_localCollisionFrame[3],
                       colShapeData.m_localCollisionFrame[4],
                       colShapeData.m_localCollisionFrame[5],
                       colShapeData.m_localCollisionFrame[6]));
      btTransform col_tr = inertial_tr * col_local_tr;

      col.origin_xyz = Vector3(Algebra::from_double(col_tr.getOrigin()[0]),
                               Algebra::from_double(col_tr.getOrigin()[1]),
                               Algebra::from_double(col_tr.getOrigin()[2]));
      btVector3 rpy;
      col_tr.getRotation().getEulerZYX(rpy[0], rpy[1], rpy[2]);

      col.origin_rpy =
          Vector3(Algebra::from_double(rpy[0]), Algebra::from_double(rpy[1]),
                  Algebra::from_double(rpy[2]));

      switch (colShapeData.m_collisionGeometryType) {
        case GEOM_SPHERE: {
          col.geometry.sphere.radius =
              Algebra::from_double(colShapeData.m_dimensions[0]);
          col.geometry.geom_type = TINY_SPHERE_TYPE;
          urdfLink.urdf_collision_shapes.push_back(col);
          break;
        }
        // case GEOM_BOX: {
        //	col.box.extents =
        //Vector3(Algebra::from_double(colShapeData.m_dimensions[0]),
        //		Algebra::from_double(colShapeData.m_dimensions[1]),
        //		Algebra::from_double(colShapeData.m_dimensions[2]));
        //	col.geom_type1 = BOX_TYPE;
        //	urdfLink.urdf_collision_shapes.push_back(col);
        //	break;
        //}
        case GEOM_CAPSULE: {
          col.geometry.capsule.length =
              Algebra::from_double(colShapeData.m_dimensions[0]);
          col.geometry.capsule.radius =
              Algebra::from_double(colShapeData.m_dimensions[1]);
          col.geometry.geom_type = TINY_CAPSULE_TYPE;
          urdfLink.urdf_collision_shapes.push_back(col);
          break;
        }
        // case GEOM_MESH: {
        //	col.mesh.file_name = colShapeData.m_meshAssetFileName;
        //	col.mesh.scale =
        //Vector3(Algebra::from_double(colShapeData.m_dimensions[0]),
        //		Algebra::from_double(colShapeData.m_dimensions[1]),
        //		Algebra::from_double(colShapeData.m_dimensions[2]));
        //	col.geom_type1 = MESH_TYPE;
        //	break;
        //}
        case GEOM_PLANE: {
          col.geometry.plane.normal =
              Vector3(Algebra::zero(), Algebra::zero(), Algebra::one());
          col.geometry.plane.constant = Algebra::zero();
          col.geometry.geom_type = TINY_PLANE_TYPE;
          urdfLink.urdf_collision_shapes.push_back(col);
          break;
        }
        default: {
        }
      };
    }
  }

  static void convert_visuals(UrdfStructures& urdf_structures,
                              UrdfLink<Algebra>& link,
                              PyBulletVisualizerAPI* viz_api) {
    for (std::size_t v = 0; v < link.urdf_visual_shapes.size(); v++) {
      UrdfVisual<Algebra>& visual_shape = link.urdf_visual_shapes[v];
      b3RobotSimulatorCreateVisualShapeArgs args;
      args.m_shapeType = visual_shape.geometry.geom_type;

      printf("visual_shape.geom_type=%d\n", visual_shape.geometry.geom_type);
      switch (visual_shape.geometry.geom_type) {
        case TINY_SPHERE_TYPE: {
          args.m_radius =
              Algebra::to_double(visual_shape.geometry.sphere.radius);
          int vizShape = viz_api->createVisualShape(GEOM_SPHERE, args);
          if (vizShape < 0) {
            printf("Couldn't create sphere shape\n");
          }
          visual_shape.visual_shape_uid = vizShape;
          break;
        }
        case TINY_CAPSULE_TYPE: {
          args.m_radius =
              Algebra::to_double(visual_shape.geometry.capsule.radius);
          args.m_height =
              Algebra::to_double(visual_shape.geometry.capsule.length);

          int vizShape = viz_api->createVisualShape(GEOM_CAPSULE, args);
          if (vizShape < 0) {
            printf("Couldn't create capsule shape\n");
          }
          visual_shape.visual_shape_uid = vizShape;
          break;
        }
        case TINY_BOX_TYPE: {
          {
            Vector3 he =
                visual_shape.geometry.box.extents * Algebra::fraction(1, 2);
            args.m_halfExtents.setValue(Algebra::to_double(he[0]),
                                        Algebra::to_double(he[1]),
                                        Algebra::to_double(he[2]));
            int vizShape = viz_api->createVisualShape(GEOM_BOX, args);
            visual_shape.visual_shape_uid = vizShape;
            break;
          }
          case TINY_MESH_TYPE: {
            args.m_fileName =
                (char*)visual_shape.geometry.mesh.file_name.c_str();
            // printf("mb mesh: %s\n", args.m_fileName);
            args.m_meshScale.setValue(
                Algebra::to_double(visual_shape.geometry.mesh.scale[0]),
                Algebra::to_double(visual_shape.geometry.mesh.scale[1]),
                Algebra::to_double(visual_shape.geometry.mesh.scale[2]));

            int vizShape = viz_api->createVisualShape(GEOM_MESH, args);
            if (vizShape < 0) {
              printf("Couldn't create sphere shape: %s\n", args.m_fileName);
            }

            // printf("vizShape=%d\n", vizShape);
            b3RobotSimulatorCreateMultiBodyArgs args2;
            args2.m_baseVisualShapeIndex = vizShape;
            args2.m_baseMass = 0;

            visual_shape.visual_shape_uid = vizShape;

            break;
          }
          default: {
          }
        }
      }
    }
  }
};

}  // namespace tds
