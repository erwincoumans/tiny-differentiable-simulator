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
import meshcat_utils_dp
import meshcat
import pd_control

world = dp.TinyWorld()
mb_solver = dp.TinyMultiBodyConstraintSolver()

vis = meshcat.Visualizer(zmq_url='tcp://127.0.0.1:6000')
vis.delete()

urdf_parser = dp.TinyUrdfParser()

plane_urdf_data = urdf_parser.load_urdf("../../data/plane_implicit.urdf")
plane2vis = meshcat_utils_dp.convert_visuals(plane_urdf_data, "../../data/checker_purple.png", vis, "../../data/")
plane_mb = dp.TinyMultiBody(False)
plane2mb = dp.UrdfToMultiBody2()
res = plane2mb.convert2(plane_urdf_data, world, plane_mb)

urdf_data = urdf_parser.load_urdf("../../data/laikago/laikago_toes_zup.urdf")
print("robot_name=",urdf_data.robot_name)
b2vis = meshcat_utils_dp.convert_visuals(urdf_data, "../../data/laikago/laikago_tex.jpg", vis, "../../data/laikago/")
is_floating=True
mb = dp.TinyMultiBody(is_floating)
urdf2mb = dp.UrdfToMultiBody2()
res = urdf2mb.convert2(urdf_data, world, mb)
mb.set_base_position(dp.TinyVector3(0,0,0.6))

knee_angle = -0.5
abduction_angle = 0.2

initial_poses = [abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
                 abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle]

qcopy = mb.q
for q_index in range (12):
  qcopy[q_index+7]=initial_poses[q_index]
mb.q = qcopy

dt = 1./1000.
skip_sync = 0
while 1:
  
  mb.forward_kinematics()
  
  pd_control.apply(mb,initial_poses)
  
  mb.forward_dynamics(dp.TinyVector3(0.,0.,-10.))
  mb.integrate_q(dt)
  multi_bodies = [plane_mb, mb]
  dispatcher = world.get_collision_dispatcher()
  contacts = world.compute_contacts_multi_body(multi_bodies,dispatcher)
    
  #collision solver
  for cps in contacts:
    mb_solver.resolve_collision(cps, dt)
    
  mb.integrate(dt)

  skip_sync-=1
  if skip_sync<=0:
    skip_sync=16
    meshcat_utils_dp.sync_visual_transforms(mb, b2vis, vis)