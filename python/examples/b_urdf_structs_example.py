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

import pytinydiffsim as pd
import time

def create_multi_body(mass, collision_shapes, is_floating, world):
  urdf = pd.TinyUrdfStructures()
  base_link = pd.TinyUrdfLink()
  base_link.link_name = "plane_link"
  inertial = pd.TinyUrdfInertial()
  inertial.mass = mass 
  inertial.inertia_xxyyzz = pd.Vector3(mass,mass,mass)
  base_link.urdf_inertial = inertial
  
  base_link.urdf_collision_shapes = collision_shapes
  urdf.base_links = [base_link]
  mb = pd.TinyMultiBody(is_floating)
  urdf2mb = pd.UrdfToMultiBody2()
  res = urdf2mb.convert2(urdf, world, mb)  
  return mb

world = pd.TinyWorld()
collision_shape = pd.TinyUrdfCollision()

collision_shape.geometry.geom_type = pd.PLANE_TYPE
collision_shape.origin_xyz = pd.Vector3(0.,0.,2.)
plane_mb = create_multi_body(0, [collision_shape], False, world)

collision_shape = pd.TinyUrdfCollision()
collision_shape.geometry.geom_type = pd.SPHERE_TYPE
collision_shape.geometry.sphere.radius = 2.0
sphere_mb = create_multi_body(2.0, [collision_shape], True, world)
sphere_mb.set_base_position(pd.Vector3(0.,0.,25.))
dt = 1./240.

mb_solver = pd.TinyMultiBodyConstraintSolver()

while 1:
    
  pd.forward_dynamics(sphere_mb, pd.Vector3(0., 0., -10.))
  
  #collision detection
  multi_bodies = [plane_mb, sphere_mb]
  dispatcher = world.get_collision_dispatcher()
  contacts = world.compute_contacts_multi_body(multi_bodies,dispatcher)
  print("contacts=",contacts)
  
  #collision solver
  for cps in contacts:
    mb_solver.resolve_collision(cps, dt)
  
  pd.integrate_euler(sphere_mb, dt)
  
  
  time.sleep(dt)

