#!/usr/bin/python
#
# Copyright 2020 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pybullet_utils.urdfEditor as ued
import pytinydiffsim as dp
import pybullet


class UrdfConverterPyBullet(ued.UrdfEditor):
  """MeshCatVisualizer will convert visual shapes in MeshCat assets

     allows to synchronize transforms.
  """

  def __init__(self, **kwargs):
    self._kwargs = kwargs

    super(ued.UrdfEditor, self).__init__(**kwargs)

  def convert_vec(self, vec):
    vec = dp.Vector3(vec[0], vec[1], vec[2])
    return vec

  def convert_geom_type(self, geom_type):
    self.geom_map = {}
    self.geom_map[pybullet.GEOM_BOX]=dp.BOX_TYPE
    self.geom_map[pybullet.GEOM_SPHERE] = dp.SPHERE_TYPE
    self.geom_map[pybullet.GEOM_PLANE] = dp.PLANE_TYPE
    self.geom_map[pybullet.GEOM_CAPSULE] = dp.CAPSULE_TYPE
    self.geom_map[pybullet.GEOM_MESH] = dp.MESH_TYPE
    return self.geom_map[geom_type]

  def convert_visual(self, v):
    visual = dp.TinyUrdfVisual()
    visual.sync_visual_body_uid1 = -1
    #convert geom type

    geom = dp.TinyUrdfGeometry()
    geom.geom_type = self.convert_geom_type(v.geom_type)
    visual.origin_rpy = self.convert_vec(v.origin_rpy)
    visual.origin_xyz = self.convert_vec(v.origin_xyz)
    if geom.geom_type == dp.SPHERE_TYPE:
      sphere = dp.TinyUrdfCollisionSphere()
      sphere.radius = v.geom_radius
      geom.sphere = sphere
    if geom.geom_type == dp.MESH_TYPE:
      mesh = dp.TinyUrdfCollisionMesh()
      mesh.file_name = v.geom_meshfilename
      mesh.scale = self.convert_vec(v.geom_meshscale)
      geom.mesh = mesh
    if geom.geom_type == dp.CAPSULE_TYPE:
      capsule = dp.TinyUrdfCollisionCapsule()
      capsule.radius = v.geom_radius
      capsule.length = v.geom_length
      geom.capsule = capsule
    visual.geometry = geom
    return visual

  def convert_collision(self, col):
    urdf_col = dp.TinyUrdfCollision()
    geom = dp.TinyUrdfGeometry()
    #convert geom type
    geom.geom_type = self.convert_geom_type(col.geom_type)
    urdf_col.origin_rpy = self.convert_vec(col.origin_rpy)
    urdf_col.origin_xyz = self.convert_vec(col.origin_xyz)
    if geom.geom_type == dp.SPHERE_TYPE:
      sphere = dp.TinyUrdfCollisionSphere()
      sphere.radius = col.geom_radius
      geom.sphere = sphere
    elif geom.geom_type == dp.MESH_TYPE:
      mesh = dp.TinyUrdfCollisionMesh()
      mesh.file_name = col.geom_meshfilename
      mesh.scale = self.convert_vec(col.geom_meshscale)
      geom.mesh = mesh
    elif geom.geom_type == dp.CAPSULE_TYPE:
      capsule = dp.TinyUrdfCollisionCapsule()
      capsule.radius = col.geom_radius
      capsule.length = col.geom_length
      geom.capsule = capsule
    else:
      print("!!!!!!!!!!!!!! unknown type:", geom.geom_type)
    urdf_col.geometry = geom
    return urdf_col

  def convert_urdf_link(self, urdfLink, link_parent_index):
    print("converting link:", urdfLink.link_name)
    urdf_link = dp.TinyUrdfLink()
    urdf_link.parent_index = link_parent_index
    urdf_link.link_name = urdfLink.link_name
    #convert inertia
    urdf_inertial = dp.TinyUrdfInertial()
    urdf_inertial.mass = urdfLink.urdf_inertial.mass
    urdf_inertial.inertia_xxyyzz = self.convert_vec(
        urdfLink.urdf_inertial.inertia_xxyyzz)
    urdf_inertial.origin_rpy = self.convert_vec(
        urdfLink.urdf_inertial.origin_rpy)
    urdf_inertial.origin_xyz = self.convert_vec(
        urdfLink.urdf_inertial.origin_xyz)
    urdf_link.urdf_inertial = urdf_inertial
    urdf_link.urdf_inertial = urdf_inertial

    #convert visual shapes
    urdf_visual_shapes = []
    for v in urdfLink.urdf_visual_shapes:
      visual = self.convert_visual(v)
      urdf_visual_shapes.append(visual)
    urdf_link.urdf_visual_shapes = urdf_visual_shapes

    #convert collision shapes
    urdf_collision_shapes = []
    for c in urdfLink.urdf_collision_shapes:
      col = self.convert_collision(c)
      urdf_collision_shapes.append(col)
    urdf_link.urdf_collision_shapes = urdf_collision_shapes

    self.urdf_links.append(urdf_link)

  def convert_urdf_joint(self, urdfJoint):
    print("converting joint:", urdfJoint.joint_name)

    urdf_joint = dp.TinyUrdfJoint()
    urdf_joint.joint_name = urdfJoint.joint_name
    urdf_joint.parent_name = urdfJoint.parent_name
    urdf_joint.child_name = urdfJoint.child_name
    link_parent_index = self.linkNameToIndex[urdfJoint.parent_name] - 1
    link_child_index = self.linkNameToIndex[urdfJoint.child_name] - 1
    print("@#@!#@!#@!#@!: link_parent_index = ", link_parent_index)
    print("@#@!#@!#@!#@!: link_child_index = ", link_child_index)

    urdf_joint.joint_origin_xyz = self.convert_vec(urdfJoint.joint_origin_xyz)
    urdf_joint.joint_origin_rpy = self.convert_vec(urdfJoint.joint_origin_rpy)
    print("urdfJoint.joint_upper_limit=", urdfJoint.joint_upper_limit)

    if urdfJoint.joint_type == pybullet.JOINT_REVOLUTE:
      print("pybullet.JOINT_REVOLUTE=", pybullet.JOINT_REVOLUTE)
      urdf_joint.joint_upper_limit = urdfJoint.joint_upper_limit
      urdf_joint.joint_lower_limit = urdfJoint.joint_lower_limit
      urdf_joint.joint_axis_xyz = self.convert_vec(urdfJoint.joint_axis_xyz)
      urdf_joint.joint_type = dp.JOINT_REVOLUTE_AXIS
    elif urdfJoint.joint_type == pybullet.JOINT_PRISMATIC:
      urdf_joint.joint_upper_limit = urdfJoint.joint_upper_limit
      urdf_joint.joint_lower_limit = urdfJoint.joint_lower_limit
      urdf_joint.joint_axis_xyz = self.convert_vec(urdfJoint.joint_axis_xyz)
      urdf_joint.joint_type = dp.PRISMATIC_JOINT
    elif urdfJoint.joint_type == pybullet.JOINT_FIXED:
      urdf_joint.joint_type = dp.JOINT_FIXED
      urdf_joint.joint_axis_xyz = self.convert_vec(urdfJoint.joint_axis_xyz)
    elif urdfJoint.joint_type == pybullet.JOINT_SPHERICAL:
      urdf_joint.joint_type = dp.JOINT_SPHERICAL
      urdf_joint.joint_axis_xyz = self.convert_vec(urdfJoint.joint_axis_xyz)
    else:
      print("unsupported joint")

    self.urdf_joints.append(urdf_joint)

    self.convert_urdf_link(urdfJoint.link, link_parent_index)

  def convert_urdf_structures(self):
    self.urdf_structs = dp.TinyUrdfStructures()
    self.urdf_links = []
    self.urdf_joints = []

    #assume link[0] is base
    if (len(self.urdfLinks) == 0):
      return -1
    self.convert_urdf_link(self.urdfLinks[0], -2)
    self.urdf_structs.base_links = self.urdf_links
    self.urdf_links = []

    for joint in self.urdfJoints:
      self.convert_urdf_joint(joint)
    self.urdf_structs.links = self.urdf_links
    self.urdf_structs.joints = self.urdf_joints
