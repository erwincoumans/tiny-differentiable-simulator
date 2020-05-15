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

# Lint as: python3
"""Example how to use differentiable physics engine using python.

We used pybind11 to expose the classes of pydiffphys.
At the moment, only double precision version is exposed.
Will also expose stan_math forward mode differentiation, dual and fixed point.
"""

from __future__ import absolute_import  # Not necessary in a Python 3-only module
from __future__ import division  # Not necessary in a Python 3-only module
from __future__ import print_function  # Not necessary in a Python 3-only module

from absl import app
from absl import flags
import copy
import pydiffphys

FLAGS = flags.FLAGS


def main(argv):
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  w = pydiffphys.TinyWorld()

  w.gravity = pydiffphys.TinyVector3(0, 0, -9.81)
  g = w.gravity
  print("g=", g.z)
  s = pydiffphys.TinySphere(2.0)
  p = pydiffphys.TinyPlane()
  mass = 1.0
  b = pydiffphys.TinyRigidBody(mass, s)
  b.world_pose.position.z = 10

  #print(s.get_radius())
  print("s.get_type()=", s.get_type())
  print("p.get_type()=", p.get_type())
  for i in range(10):
    w.step(0.1)
    print("b.m_world_pose", b.world_pose.position.z)


if __name__ == "__main__":
  app.run(main)
