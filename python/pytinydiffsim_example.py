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

from absl import app
from absl import flags

import pytinydiffsim as pydiffphys

FLAGS = flags.FLAGS


def pycompute_contacts(bodies, dispatcher):
  contacts = []
  num_bodies = len(bodies)
  for b0 in range(0, num_bodies):
    body0 = bodies[b0]
    for b1 in range(b0 + 1, num_bodies):
      body1 = bodies[b1]
      new_contacts = dispatcher.compute_contacts(body0.collision_geometry,
                                                 body0.pose,
                                                 body1.collision_geometry,
                                                 body1.pose)
      print("new_contacts=", new_contacts)


def main(argv):
  if len(argv) > 1:
    raise app.UsageError("Too many command-line arguments.")

  w = pydiffphys.TinyWorld()

  w.gravity = pydiffphys.Vector3(0, 0, -9.81)
  g = w.gravity
  print("g=", g.z)
  plane = pydiffphys.TinyPlane()
  sphere = pydiffphys.TinySphere(5.0)
  d = w.get_collision_dispatcher()
  mass = 0.0
  b1 = pydiffphys.TinyRigidBody(mass, plane)
  mass = 1.0
  b2 = pydiffphys.TinyRigidBody(mass, sphere)

  print("b.linear_velocity=", b1.linear_velocity)
  pos_a = pydiffphys.Vector3(0, 0, 0)
  orn_a = pydiffphys.Quaternion(0, 0, 0, 1)
  pos_b = pydiffphys.Vector3(0, 0, 50)
  orn_b = pydiffphys.Quaternion(0, 0, 0, 1)

  pose_a = pydiffphys.TinyPose(pos_a, orn_a)
  pose_b = pydiffphys.TinyPose(pos_b, orn_b)
  b1.world_pose = pose_a
  b2.world_pose = pose_b

  steps = 1000
  for step in range(steps):
    b2.apply_gravity(g)
    dt = 1. / 240.
    b2.apply_force_impulse(dt)
    b2.clear_forces()
    bodies = [b1, b2]
    rb_contacts = w.compute_contacts_rigid_body(bodies, d)
    num_iter = 50
    solver = pydiffphys.TinyConstraintSolver()
    for _ in range(num_iter):
      for contact in rb_contacts:
        solver.resolve_collision(contact, dt)
    for body in bodies:
      body.integrate(dt)

    print("step=", step, ": b2.world_pose.position=", b2.world_pose.position,
          ", b2.linear_velocity=", b2.linear_velocity)


if __name__ == "__main__":
  app.run(main)
