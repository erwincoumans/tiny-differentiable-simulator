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

#ifndef PYBULLET_URDF_IMPORT_H
#define PYBULLET_URDF_IMPORT_H

#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "tiny_multi_body.h"
#include "tiny_urdf_structures.h"

template <typename TinyScalar, typename TinyConstants>
struct PyBulletUrdfImport {
  typedef ::TinyUrdfStructures<TinyScalar, TinyConstants> TinyUrdfStructures;

  static void extract_urdf_structs(
      TinyUrdfStructures& urdf_structures, int body_unique_id,
      class b3RobotSimulatorClientAPI_NoDirect& sim_api,
      class b3RobotSimulatorClientAPI_NoDirect& viz_api) {
    btVector3 basePos;
    btQuaternion baseOrn;
    sim_api.getBasePositionAndOrientation(body_unique_id, basePos, baseOrn);
    {
      int base_link_index = -1;
      TinyUrdfLink<TinyScalar, TinyConstants> base_link;
      extract_link(urdf_structures, body_unique_id, base_link_index, sim_api,
                   viz_api, base_link);
      base_link.m_parent_index = -2;
      urdf_structures.m_base_links.push_back(base_link);
    }

    int num_joints = sim_api.getNumJoints(body_unique_id);
    for (int link_index = 0; link_index < num_joints; link_index++) {
      TinyUrdfLink<TinyScalar, TinyConstants> child_link;
      extract_link(urdf_structures, body_unique_id, link_index, sim_api,
                   viz_api, child_link);

      b3JointInfo jointInfo;
      sim_api.getJointInfo(body_unique_id, link_index, &jointInfo);

      TinyUrdfJoint<TinyScalar, TinyConstants> joint;

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

      joint.joint_axis_xyz.setValue(TinyScalar(jointInfo.m_jointAxis[0]),
                                    TinyScalar(jointInfo.m_jointAxis[1]),
                                    TinyScalar(jointInfo.m_jointAxis[2]));

      if (jointInfo.m_parentIndex < 0) {
        joint.parent_name = urdf_structures.m_base_links[0].link_name;
        urdf_structures.m_base_links[0].child_link_indices.push_back(
            link_index);

      } else {
        b3JointInfo parentJointInfo;
        sim_api.getJointInfo(body_unique_id, jointInfo.m_parentIndex,
                             &parentJointInfo);
        joint.parent_name = parentJointInfo.m_linkName;
        urdf_structures.m_links[jointInfo.m_parentIndex]
            .child_link_indices.push_back(link_index);
      }
      child_link.m_parent_index = jointInfo.m_parentIndex;

      b3DynamicsInfo dynamics_info_child;
      sim_api.getDynamicsInfo(body_unique_id, link_index, &dynamics_info_child);
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
      sim_api.getDynamicsInfo(body_unique_id, jointInfo.m_parentIndex,
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
      joint.joint_origin_xyz.setValue(TinyScalar(pos.getOrigin()[0]),
                                      TinyScalar(pos.getOrigin()[1]),
                                      TinyScalar(pos.getOrigin()[2]));
      btScalar roll, pitch, yaw;
      pos_.getRotation().getEulerZYX(yaw, pitch, roll);
      btVector3 rpy = btVector3(roll, pitch, yaw);
      joint.joint_origin_rpy.setValue(TinyScalar(rpy[0]), TinyScalar(rpy[1]),
                                      TinyScalar(rpy[2]));
      urdf_structures.m_links.push_back(child_link);
      urdf_structures.m_joints.push_back(joint);
    }
  }

  static void sync_graphics_transforms(
      const TinyMultiBody<TinyScalar, TinyConstants>* body,
      class b3RobotSimulatorClientAPI_NoDirect& viz_api) {
    for (int v = 0; v < body->m_visual_uids1.size(); v++) {
      int visual_id = body->m_visual_uids1[v];
      TinyQuaternion<TinyScalar, TinyConstants> rot;
      TinySpatialTransform<TinyScalar, TinyConstants> geom_X_world =
          body->m_base_X_world * body->m_X_visuals[v];
      btVector3 base_pos(geom_X_world.m_translation.getX(),
                         geom_X_world.m_translation.getY(),
                         geom_X_world.m_translation.getZ());
      geom_X_world.m_rotation.getRotation(rot);
      btQuaternion base_orn(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
      viz_api.resetBasePositionAndOrientation(visual_id, base_pos, base_orn);
    }

    for (int l = 0; l < body->m_links.size(); l++) {
      for (int v = 0; v < body->m_links[l].m_visual_uids1.size(); v++) {
        int visual_id = body->m_links[l].m_visual_uids1[v];
        TinyQuaternion<TinyScalar, TinyConstants> rot;
        TinySpatialTransform<TinyScalar, TinyConstants> geom_X_world =
            body->m_links[l].m_X_world * body->m_links[l].m_X_visuals[v];
        btVector3 base_pos(geom_X_world.m_translation.getX(),
                           geom_X_world.m_translation.getY(),
                           geom_X_world.m_translation.getZ());
        geom_X_world.m_rotation.getRotation(rot);
        btQuaternion base_orn(rot.getX(), rot.getY(), rot.getZ(), rot.getW());
        viz_api.resetBasePositionAndOrientation(visual_id, base_pos, base_orn);
      }
    }
  }
  static void extract_link(TinyUrdfStructures& urdf_structures,
                           int body_unique_id, int linkIndex,
                           class b3RobotSimulatorClientAPI_NoDirect& sim_api,
                           class b3RobotSimulatorClientAPI_NoDirect& viz_api,
                           TinyUrdfLink<TinyScalar, TinyConstants>& urdfLink) {
    b3BodyInfo bodyInfo;
    sim_api.getBodyInfo(body_unique_id, &bodyInfo);

    if (linkIndex == -1) {
      urdfLink.link_name = bodyInfo.m_baseName;
    } else {
      b3JointInfo jointInfo;
      sim_api.getJointInfo(body_unique_id, linkIndex, &jointInfo);
      urdfLink.link_name = jointInfo.m_linkName;
    }

    b3DynamicsInfo dyn;
    sim_api.getDynamicsInfo(body_unique_id, linkIndex, &dyn);

    urdfLink.urdf_inertial.mass = TinyScalar(dyn.m_mass);
    urdfLink.urdf_inertial.inertia_xxyyzz.setValue(
        TinyScalar(dyn.m_localInertialDiagonal[0]),
        TinyScalar(dyn.m_localInertialDiagonal[1]),
        TinyScalar(dyn.m_localInertialDiagonal[2]));
    urdfLink.urdf_inertial.origin_xyz.setValue(
        TinyScalar(dyn.m_localInertialFrame[0]),
        TinyScalar(dyn.m_localInertialFrame[1]),
        TinyScalar(dyn.m_localInertialFrame[2]));
    btVector3 rpy = sim_api.getEulerFromQuaternion(
        btQuaternion(dyn.m_localInertialFrame[3], dyn.m_localInertialFrame[4],
                     dyn.m_localInertialFrame[5], dyn.m_localInertialFrame[6]));
    urdfLink.urdf_inertial.origin_rpy.setValue(
        TinyScalar(rpy[0]), TinyScalar(rpy[1]), TinyScalar(rpy[2]));

    // visual shapes
    b3VisualShapeInformation visualShapeInfo;
    sim_api.getVisualShapeData(body_unique_id, visualShapeInfo);

    for (int i = 0; i < visualShapeInfo.m_numVisualShapes; i++) {
      const b3VisualShapeData& visual = visualShapeInfo.m_visualShapeData[i];
      if (visual.m_linkIndex == linkIndex) {
        TinyUrdfVisual<TinyScalar, TinyConstants> viz;
        // offset
        viz.origin_xyz.setValue(TinyScalar(visual.m_localVisualFrame[0]),
                                TinyScalar(visual.m_localVisualFrame[1]),
                                TinyScalar(visual.m_localVisualFrame[2]));
        btVector3 rpy = sim_api.getEulerFromQuaternion(btQuaternion(
            visual.m_localVisualFrame[3], visual.m_localVisualFrame[4],
            visual.m_localVisualFrame[5], visual.m_localVisualFrame[6]));
        viz.origin_rpy.setValue(TinyScalar(rpy[0]), TinyScalar(rpy[1]),
                                TinyScalar(rpy[2]));

        viz.m_material.material_rgb.setValue(TinyScalar(visual.m_rgbaColor[0]),
                                             TinyScalar(visual.m_rgbaColor[1]),
                                             TinyScalar(visual.m_rgbaColor[2]));
        // viz.material_a = TinyScalar(visual.m_rgbaColor[3]);

        // try to load for now, until we can manually 'override' the shape world
        // transform
        switch (visual.m_visualGeometryType) {
          case GEOM_SPHERE: {
            viz.geometry.m_sphere.m_radius = TinyScalar(visual.m_dimensions[0]);
            viz.geometry.geom_type = TINY_SPHERE_TYPE;
            break;
          }
          case GEOM_CAPSULE: {
            viz.geometry.m_capsule.m_length =
                TinyScalar(visual.m_dimensions[0]);
            viz.geometry.m_capsule.m_radius =
                TinyScalar(visual.m_dimensions[1]);
            viz.geometry.geom_type = TINY_CAPSULE_TYPE;
            break;
          }
          case GEOM_BOX: {
            TinyVector3<TinyScalar, TinyConstants> halfExtents;
            halfExtents.setValue(TinyScalar(visual.m_dimensions[0]),
                                 TinyScalar(visual.m_dimensions[1]),
                                 TinyScalar(visual.m_dimensions[2]));
            viz.geometry.m_box.m_extents =
                halfExtents * TinyConstants::fraction(2, 1);
            viz.geometry.geom_type = TINY_BOX_TYPE;
            break;
          }
          case GEOM_MESH: {
            viz.geometry.m_mesh.m_scale.setValue(
                TinyScalar(visual.m_dimensions[0]),
                TinyScalar(visual.m_dimensions[1]),
                TinyScalar(visual.m_dimensions[2]));
            viz.geometry.m_mesh.m_file_name = visual.m_meshAssetFileName;
            printf("extract mesh: %s\n",
                   viz.geometry.m_mesh.m_file_name.c_str());
            printf("extract scale: %f,%f,%f\n",
                   TinyConstants::getDouble(viz.geometry.m_mesh.m_scale[0]),
                   TinyConstants::getDouble(viz.geometry.m_mesh.m_scale[1]),
                   TinyConstants::getDouble(viz.geometry.m_mesh.m_scale[2]));
            viz.geometry.geom_type = TINY_MESH_TYPE;
            break;
          }
          default: {}
        }

        urdfLink.urdf_visual_shapes.push_back(viz);
      }
    }

    convert_visuals(urdf_structures, urdfLink, viz_api);
    // collision shapes, only convert spheres for now
    b3CollisionShapeInformation collisionShapeInfo;
    sim_api.getCollisionShapeData(body_unique_id, linkIndex,
                                  collisionShapeInfo);

    for (int i = 0; i < collisionShapeInfo.m_numCollisionShapes; i++) {
      const b3CollisionShapeData& colShapeData =
          collisionShapeInfo.m_collisionShapeData[i];
      TinyUrdfCollision<TinyScalar, TinyConstants> col;
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

      col.origin_xyz.setValue(TinyScalar(col_tr.getOrigin()[0]),
                              TinyScalar(col_tr.getOrigin()[1]),
                              TinyScalar(col_tr.getOrigin()[2]));
      btVector3 rpy;
      col_tr.getRotation().getEulerZYX(rpy[0], rpy[1], rpy[2]);

      col.origin_rpy.setValue(TinyScalar(rpy[0]), TinyScalar(rpy[1]),
                              TinyScalar(rpy[2]));

      switch (colShapeData.m_collisionGeometryType) {
        case GEOM_SPHERE: {
          col.geometry.m_sphere.m_radius =
              TinyScalar(colShapeData.m_dimensions[0]);
          col.geometry.geom_type = TINY_SPHERE_TYPE;
          urdfLink.urdf_collision_shapes.push_back(col);
          break;
        }
        // case GEOM_BOX: {
        //	col.m_box.m_extents.setValue(TinyScalar(colShapeData.m_dimensions[0]),
        //		TinyScalar(colShapeData.m_dimensions[1]),
        //		TinyScalar(colShapeData.m_dimensions[2]));
        //	col.geom_type1 = BOX_TYPE;
        //	urdfLink.urdf_collision_shapes.push_back(col);
        //	break;
        //}
        case GEOM_CAPSULE: {
          col.geometry.m_capsule.m_length =
              TinyScalar(colShapeData.m_dimensions[0]);
          col.geometry.m_capsule.m_radius =
              TinyScalar(colShapeData.m_dimensions[1]);
          col.geometry.geom_type = TINY_CAPSULE_TYPE;
          urdfLink.urdf_collision_shapes.push_back(col);
          break;
        }
        // case GEOM_MESH: {
        //	col.m_mesh.m_file_name = colShapeData.m_meshAssetFileName;
        //	col.m_mesh.m_scale.setValue(TinyScalar(colShapeData.m_dimensions[0]),
        //		TinyScalar(colShapeData.m_dimensions[1]),
        //		TinyScalar(colShapeData.m_dimensions[2]));
        //	col.geom_type1 = MESH_TYPE;
        //	break;
        //}
        case GEOM_PLANE: {
          col.geometry.m_plane.m_normal.setValue(TinyConstants::zero(),
                                                 TinyConstants::zero(),
                                                 TinyConstants::one());
          col.geometry.m_plane.m_constant = TinyConstants::zero();
          col.geometry.geom_type = TINY_PLANE_TYPE;
          urdfLink.urdf_collision_shapes.push_back(col);
          break;
        }
        default: {}
      };
    }
  }

  static void convert_visuals(
      TinyUrdfStructures& urdf_structures,
      TinyUrdfLink<TinyScalar, TinyConstants>& link,
      class b3RobotSimulatorClientAPI_NoDirect& viz_api) {
    typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;

    for (int v = 0; v < link.urdf_visual_shapes.size(); v++) {
      TinyUrdfVisual<TinyScalar, TinyConstants>& visual_shape =
          link.urdf_visual_shapes[v];
      b3RobotSimulatorCreateVisualShapeArgs args;
      args.m_shapeType = visual_shape.geometry.geom_type;

      printf("visual_shape.geom_type=%d\n", visual_shape.geometry.geom_type);
      switch (visual_shape.geometry.geom_type) {
        case TINY_SPHERE_TYPE: {
          args.m_radius =
              TinyConstants::getDouble(visual_shape.geometry.m_sphere.m_radius);
          int vizShape = viz_api.createVisualShape(GEOM_SPHERE, args);
          if (vizShape < 0) {
            printf("Couldn't create sphere shape\n");
          }
          b3RobotSimulatorCreateMultiBodyArgs args2;
          args2.m_baseVisualShapeIndex = vizShape;
          args2.m_baseMass = 0;
          int viz_uid = viz_api.createMultiBody(args2);
          visual_shape.sync_visual_body_uid1 = viz_uid;
          break;
        }
        case TINY_CAPSULE_TYPE: {
          args.m_radius = TinyConstants::getDouble(
              visual_shape.geometry.m_capsule.m_radius);
          args.m_height = TinyConstants::getDouble(
              visual_shape.geometry.m_capsule.m_length);

          int vizShape = viz_api.createVisualShape(GEOM_CAPSULE, args);
          if (vizShape < 0) {
            printf("Couldn't create capsule shape\n");
          }
          b3RobotSimulatorCreateMultiBodyArgs args2;
          args2.m_baseVisualShapeIndex = vizShape;
          args2.m_baseMass = 0;
          int viz_uid = viz_api.createMultiBody(args2);
          visual_shape.sync_visual_body_uid1 = viz_uid;
          break;
        }
        case TINY_BOX_TYPE: {
          {
            TinyVector3 he = visual_shape.geometry.m_box.m_extents *
                             TinyConstants::fraction(1, 2);
            args.m_halfExtents.setValue(TinyConstants::getDouble(he[0]),
                                        TinyConstants::getDouble(he[1]),
                                        TinyConstants::getDouble(he[2]));
            int vizShape = viz_api.createVisualShape(GEOM_BOX, args);
            b3RobotSimulatorCreateMultiBodyArgs args2;
            args2.m_baseVisualShapeIndex = vizShape;
            args2.m_baseMass = 0;
            int viz_uid = viz_api.createMultiBody(args2);
            visual_shape.sync_visual_body_uid1 = viz_uid;
            break;
          }
          case TINY_MESH_TYPE: {
            args.m_fileName =
                (char*)visual_shape.geometry.m_mesh.m_file_name.c_str();
            // printf("mb mesh: %s\n", args.m_fileName);
            args.m_meshScale.setValue(
                TinyConstants::getDouble(
                    visual_shape.geometry.m_mesh.m_scale[0]),
                TinyConstants::getDouble(
                    visual_shape.geometry.m_mesh.m_scale[1]),
                TinyConstants::getDouble(
                    visual_shape.geometry.m_mesh.m_scale[2]));

            int vizShape = viz_api.createVisualShape(GEOM_MESH, args);
            if (vizShape < 0) {
              printf("Couldn't create sphere shape: %s\n", args.m_fileName);
            }

            // printf("vizShape=%d\n", vizShape);
            b3RobotSimulatorCreateMultiBodyArgs args2;
            args2.m_baseVisualShapeIndex = vizShape;
            args2.m_baseMass = 0;

            int viz_uid = viz_api.createMultiBody(args2);
            {
              b3RobotSimulatorChangeVisualShapeArgs args_change;
              args_change.m_objectUniqueId = viz_uid;
              args_change.m_linkIndex = -1;
              args_change.m_hasRgbaColor = true;
              args_change.m_rgbaColor.setValue(1, 1, 1, 1);
              viz_api.changeVisualShape(args_change);
            }
            visual_shape.sync_visual_body_uid1 = viz_uid;
            break;
          }
          default: {}
        }
      }
    }
  }
};

template <typename TinyScalar, typename TinyConstants>
struct TinyUrdfCache {
  typedef ::TinyUrdfStructures<TinyScalar, TinyConstants> UrdfStructures;
  typedef ::PyBulletUrdfImport<TinyScalar, TinyConstants> UrdfImport;
  typedef b3RobotSimulatorLoadUrdfFileArgs UrdfFileArgs;

  std::map<std::string, UrdfStructures> data;

  template <typename VisualizerAPI>
  const UrdfStructures& retrieve(const std::string& urdf_filename,
                                 VisualizerAPI* sim, VisualizerAPI* vis,
                                 UrdfFileArgs args = UrdfFileArgs()) {
    if (data.find(urdf_filename) == data.end()) {
      printf("Loading URDF \"%s\".\n", urdf_filename.c_str());
      int robotId = sim->loadURDF(urdf_filename, args);
      data[urdf_filename] = UrdfStructures();
      UrdfImport::extract_urdf_structs(data[urdf_filename], robotId, *sim,
                                       *vis);
      sim->removeBody(robotId);
    }
    return data[urdf_filename];
  }
};

#endif  // PYBULLET_URDF_IMPORT_H
