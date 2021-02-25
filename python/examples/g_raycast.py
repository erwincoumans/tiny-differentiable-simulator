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

import pytinydiffsim as dp
#import meshcat_utils_dp
#import meshcat
import time
import copy
world = dp.TinyWorld()
mb_solver = dp.TinyMultiBodyConstraintSolver()

#vis = meshcat.Visualizer(zmq_url='tcp://127.0.0.1:6000')
#vis.delete()

urdf_parser = dp.TinyUrdfParser()

plane_urdf_data = urdf_parser.load_urdf("../../data/bunny.urdf")
#plane2vis = meshcat_utils_dp.convert_visuals(plane_urdf_data, "../../data/checker_purple.png", vis, "../../data/")
plane_mb = dp.TinyMultiBody(False)
plane2mb = dp.UrdfToMultiBody2()
res = plane2mb.convert2(plane_urdf_data, world, plane_mb)

colliders = plane_urdf_data.base_links[0].urdf_collision_shapes
print("num_colliders=",len(colliders))
ray_from=[]
ray_to=[]
ray_from.append(dp.Vector3(-2,.1,0))
ray_to.append(dp.Vector3(2,.1,0))
caster = dp.TinyRaycast()

colliders = []

if 1:
    collision_shape = dp.TinyUrdfCollision()
    collision_shape.geometry.geom_type = dp.SPHERE_TYPE
    collision_shape.geometry.sphere.radius = 1
    colliders.append(collision_shape)
    collision_shape = dp.TinyUrdfCollision()
    collision_shape.geometry.geom_type = dp.SPHERE_TYPE
    collision_shape.geometry.sphere.radius = 1.5
    colliders.append(collision_shape)

if 1:
    collision_shape = dp.TinyUrdfCollision()
    collision_shape.geometry.geom_type = dp.BOX_TYPE
    collision_shape.geometry.box.extents = dp.Vector3(1,0.1,0.1)
    colliders.append(collision_shape)
    collision_shape = dp.TinyUrdfCollision()
    collision_shape.geometry.geom_type = dp.BOX_TYPE
    collision_shape.geometry.box.extents = dp.Vector3(1.5,0.15,0.15)
    colliders.append(collision_shape)


results = caster.cast_rays(ray_from,ray_to,colliders)#colliders)
print("len(results)=",len(results))
for res in results:
    for hits in res:
        print("hit_fraction=", hits.hit_fraction, "hit object=", hits.collider_index)
        
        

vol = caster.volume(results,len(colliders))
print("vol=",vol)
intersect_vol = caster.intersection_volume(results,results, len(colliders))
print("intersect_vol=",intersect_vol)
dt = 1./1000.
skip_sync = 0
#while 1:
#    time.sleep(0.1)
    