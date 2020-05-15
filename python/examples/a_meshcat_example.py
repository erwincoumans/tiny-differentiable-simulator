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
import pybullet_utils.bullet_client as bc
import pybullet_data as pd
import time
import os
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np


import meshcat_utils_pb


vis = meshcat.Visualizer(zmq_url='tcp://127.0.0.1:6000')
# Or don"t use this line to preserve state of visualization upon
# running script.
vis.delete()


p0 = bc.BulletClient(connection_mode=1)#pybullet.GUI)#DIRECT)
p0.configureDebugVisualizer(p0.COV_ENABLE_RENDERING,0)
p0.setAdditionalSearchPath(pd.getDataPath())
flags = p0.URDF_MAINTAIN_LINK_ORDER
laikago = p0.loadURDF("plane_implicit.urdf", flags=flags)
start_pos = [0,0,0.6]
laikago = p0.loadURDF("laikago/laikago_toes_zup.urdf", start_pos, flags=flags)
p0.configureDebugVisualizer(p0.COV_ENABLE_RENDERING,1)

med0 = ued.UrdfEditor()
med0.initializeFromBulletBody(laikago, p0._client)

texture_path = os.path.join(pd.getDataPath(),
                            'laikago/laikago_tex.jpg')
b2vis = meshcat_utils_pb.convert_visuals_pb(vis,med0.urdfLinks, med0.urdfJoints, p0, texture_path)


p0.setGravity(0,0,-10)
while p0.isConnected():
  
  p0.stepSimulation()
  meshcat_utils_pb.sync_visual_transforms_pb( b2vis, laikago, p0, vis)
  time.sleep(1./240.)

