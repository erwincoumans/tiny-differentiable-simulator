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
import meshcat.transformations as tf
import numpy as np


class VisualLinkInfo(object):
  def __init__(self):
    self.uid = 1
    self.vis_name = ""
    self.origin_rpy = [1, 2, 3]
    self.origin_xyz = [4, 5, 6]


def convert_visuals_pb(vis, urdfLinks, urdfJoints, p0, texture_path):
    b2vis={}
    uid = -1
    print("num_links=",len(urdfLinks) )
    for link in urdfLinks:
      print("num_visuals=", len(link.urdf_visual_shapes))
      for v in link.urdf_visual_shapes:
        print("v.geom_type=", v.geom_type)
        if v.geom_type==p0.GEOM_MESH:
          print("mesh filename=", v.geom_meshfilename)
          print("geom_meshscale=", v.geom_meshscale)

          
          vis_name = link.link_name + str(uid)
          b2v = VisualLinkInfo()
          b2v.vis_name = vis_name
          b2v.uid = uid
          b2v.origin_rpy = v.origin_rpy
          b2v.origin_xyz = v.origin_xyz
          b2v.inertia_xyz = link.urdf_inertial.origin_xyz
          b2v.inertia_rpy = link.urdf_inertial.origin_rpy
          

          vis[vis_name].set_object(g.ObjMeshGeometry.from_file(v.geom_meshfilename),
                          g.MeshLambertMaterial(
                              map=g.ImageTexture(
                                  image=g.PngImage.from_file(texture_path)
                                  )
                              ))
          v.uid = uid
          b2vis[v.uid] = b2v
          uid += 1

     
    print("num_joints=", len(urdfJoints) )
    return b2vis


def sync_visual_transforms_pb(b2vis, body_uid, bc, vis):
    for key in b2vis:
      v = b2vis[key]
      if v.uid == -1: #base
        pos,orn = bc.getBasePositionAndOrientation(body_uid)
      else:
        ls = bc.getLinkState(body_uid, v.uid)
        pos=ls[0]
        orn=ls[1]
      vpos = v.origin_xyz
      vorn = bc.getQuaternionFromEuler(v.origin_rpy)
      inertia_orn = bc.getQuaternionFromEuler(v.inertia_rpy)
      inv_inertia_pos, inv_inertia_orn = bc.invertTransform(v.inertia_xyz, inertia_orn)
      vpos,vorn = bc.multiplyTransforms(inv_inertia_pos, inv_inertia_orn, vpos, vorn)
      pos,orn = bc.multiplyTransforms(pos, orn, vpos, vorn)

      mat3 = bc.getMatrixFromQuaternion(orn)
      mat = [[mat3[0],mat3[1],mat3[2],pos[0]],
             [mat3[3],mat3[4],mat3[5], pos[1]],
             [mat3[6],mat3[7],mat3[8],pos[2]],
             [0,0,0,1]]
      vis[v.vis_name].set_transform(np.array(mat))

