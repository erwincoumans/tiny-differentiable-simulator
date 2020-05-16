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

import pybullet_utils.bullet_client as bc
import pybullet
import pytinydiffsim as dp
#import meshcat_utils_pb
import meshcat_utils_dp
import pybullet_data as pd
import urdf_utils_pb
import time
import os
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np

vis = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
# Or don"t use this line to preserve state of visualization upon
# running script.
vis.delete()

world = dp.TinyWorld()
dispatcher = world.get_collision_dispatcher()

p0 = bc.BulletClient(connection_mode=1)  #pybullet.GUI)#DIRECT)
p0.configureDebugVisualizer(p0.COV_ENABLE_RENDERING, 0)
p0.setAdditionalSearchPath(pd.getDataPath())
flags = p0.URDF_MAINTAIN_LINK_ORDER
plane = p0.loadURDF("plane_implicit.urdf", flags=flags)
med1 = urdf_utils_pb.UrdfConverterPyBullet()
med1.initializeFromBulletBody(plane, p0._client)
med1.convert_urdf_structures()

texture_path = os.path.join(pd.getDataPath(), "checker_blue.png")

print("texture_path=", texture_path)
plane_vis = meshcat_utils_dp.convert_visuals(med1.urdf_structs, texture_path,
                                             vis)

urdf2mb = dp.UrdfToMultiBody2()
is_floating = False
plane_mb = dp.TinyMultiBody(is_floating)
multi_bodies = [plane_mb]

res = urdf2mb.convert2(med1.urdf_structs, world, plane_mb)

start_pos = [0, 0, 0.6]
laikago = p0.loadURDF("laikago/laikago_toes_zup.urdf", start_pos, flags=flags)

#laikago = p0.loadURDF("r2d2.urdf", start_pos, flags=flags)
p0.configureDebugVisualizer(p0.COV_ENABLE_RENDERING, 1)

med0 = urdf_utils_pb.UrdfConverterPyBullet()
med0.initializeFromBulletBody(laikago, p0._client)

contacts = world.compute_contacts_multi_body(multi_bodies, dispatcher)
#now convert urdf structs to diffphys urdf structs
med0.convert_urdf_structures()
print("conversion to TinyUrdfStructures done!")

texture_path = os.path.join(pd.getDataPath(), "laikago/laikago_tex.jpg")

laikago_vis = meshcat_utils_dp.convert_visuals(med0.urdf_structs, texture_path,
                                               vis)
print("convert_visuals done")

urdf2mb = dp.UrdfToMultiBody2()
is_floating = True
laikago_mb = dp.TinyMultiBody(is_floating)

res = urdf2mb.convert2(med0.urdf_structs, world, laikago_mb)
laikago_mb.set_base_position(dp.TinyVector3(0., 0., 2.))
mb_solver = dp.TinyMultiBodyConstraintSolver()

dt = 1. / 1000.
#p0.setGravity(0,0,-10)

while p0.isConnected():
  laikago_mb.forward_kinematics()
  laikago_mb.forward_dynamics(dp.TinyVector3(0., 0., -10.))
  laikago_mb.integrate_q(dt)

  multi_bodies = [plane_mb, laikago_mb]

  contacts = world.compute_contacts_multi_body(multi_bodies, dispatcher)

  #collision solver
  for cps in contacts:
    mb_solver.resolve_collision(cps, dt)

  laikago_mb.integrate(dt)

  meshcat_utils_dp.sync_visual_transforms(laikago_mb, laikago_vis, vis)
  time.sleep(1. / 240.)
