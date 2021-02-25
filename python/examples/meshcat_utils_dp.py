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

import os

import meshcat
import meshcat.geometry as g
import pytinydiffsim as dp
import numpy as np


class VisualLinkInfo(object):

  def __init__(self):
    self.uid = 1
    self.vis_name = ""
    self.origin_rpy = [1, 2, 3]
    self.origin_xyz = [4, 5, 6]


def convert_link_visuals(link, link_index, material, vis, uid, b2vis, path_prefix):
  print("convert_link_visuals:: num_visuals=", len(link.urdf_visual_shapes))
  print("link.urdf_visual_shapes=", link.urdf_visual_shapes)
  for v in link.urdf_visual_shapes:
    print("v.geom_type=", v.geometry.geom_type)
    vis_name = link.link_name + str(uid)
    b2v = VisualLinkInfo()
    b2v.vis_name = vis_name
    b2v.link_index = link_index
    b2v.origin_rpy = v.origin_rpy
    b2v.origin_xyz = v.origin_xyz
    b2v.inertia_xyz = link.urdf_inertial.origin_xyz
    b2v.inertia_rpy = link.urdf_inertial.origin_rpy

    if v.geometry.geom_type == dp.SPHERE_TYPE:

      print("v.geom_radius=", v.geometry.sphere.radius)
      if (v.geometry.sphere.radius > 0.):
        print("created sphere!")
        vis[vis_name].set_object(g.Sphere(v.geometry.sphere.radius), material)
        b2vis[uid] = b2v
        uid += 1

    if v.geometry.geom_type == dp.MESH_TYPE:
      print("mesh filename=", path_prefix+v.geometry.mesh.file_name)
      print("geom_meshscale=", v.geometry.mesh.scale)
      vis_name = link.link_name + str(uid)
      vis[vis_name].set_object(
          g.ObjMeshGeometry.from_file(path_prefix+v.geometry.mesh.file_name), material)
    b2v.uid = uid
    b2vis[uid] = b2v
    uid += 1
  return b2vis, uid


def convert_visuals(urdf, texture_path, vis, path_prefix=""):
  link_name_to_index = {}
  link_name_to_index[urdf.base_links[0].link_name] = -1
  for link_index in range(len(urdf.links)):
    l = urdf.links[link_index]
    link_name_to_index[l.link_name] = link_index
  b2vis = {}
  uid = -1
  if texture_path:
    material = g.MeshLambertMaterial(
        map=g.ImageTexture(
            wrap=[0, 0],
            repeat=[1, 1],
            image=g.PngImage.from_file(texture_path)))
  else:
    material = g.MeshLambertMaterial(color=0xffffff, reflectivity=0.8)
  
  #material.transparent=True
  #material.opacity=0.2
  #first the base link
  link_index = -1

  b2v, uid = convert_link_visuals(urdf.base_links[0], link_index, material, vis,
                                  uid, b2vis, path_prefix)

  #then convert each child link
  for joint in urdf.joints:
    link_index = link_name_to_index[joint.child_name]
    link = urdf.links[link_index]
    b2v, uid = convert_link_visuals(link, link_index, material, vis, uid, b2vis, path_prefix)

  return b2vis


def sync_visual_transforms(mb, b2vis, vis):
  
  for key in b2vis:
    
    v = b2vis[key]
    #print("v.link_index=",v.link_index)
    link_world_trans = mb.get_world_transform(v.link_index)

    vpos = v.origin_xyz
    vorn = dp.Quaternion(0.0, 0.0, 0.0, 1.0)
    vorn.set_euler_rpy(v.origin_rpy)
    trv = dp.TinySpatialTransform()
    trv.translation = vpos
    trv.rotation = dp.Matrix3(vorn)
    #print("link_world_trans.x=",link_world_trans.translation.x)
    #print("link_world_trans.y=",link_world_trans.translation.y)
    #print("link_world_trans.z=",link_world_trans.translation.z)
    gfx_world_trans = link_world_trans * trv  #trvi

    rot = gfx_world_trans.rotation
    #print("gfx_world_trans.translation=",gfx_world_trans.translation)
    transpose = False
    if transpose:
      mat = [[
          rot.get_row(0).x,
          rot.get_row(1).x,
          rot.get_row(2).x, gfx_world_trans.translation.x
      ],
             [
                 rot.get_row(0).y,
                 rot.get_row(1).y,
                 rot.get_row(2).y, gfx_world_trans.translation.y
             ],
             [
                 rot.get_row(0).z,
                 rot.get_row(1).z,
                 rot.get_row(2).z, gfx_world_trans.translation.z
             ], [0., 0., 0., 1.]]
    else:
         mat = [[
        rot.get_row(0).x,
        rot.get_row(0).y,
        rot.get_row(0).z, gfx_world_trans.translation.x
        ],
           [
               rot.get_row(1).x,
               rot.get_row(1).y,
               rot.get_row(1).z, gfx_world_trans.translation.y
           ],
           [
               rot.get_row(2).x,
               rot.get_row(2).y,
               rot.get_row(2).z, gfx_world_trans.translation.z
           ], [0., 0., 0., 1.]]


    vis[v.vis_name].set_transform(np.array(mat))
