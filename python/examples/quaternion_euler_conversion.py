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

from pydiffphys import TinyQuaternion, TinyVector3
import numpy as np
import matplotlib.pyplot as plt

quat = TinyQuaternion(0., 0., 0., 1.)

tolerance = 1e-3


# Euler angles are in [0:pi]x[-pi:pi]x[-pi:pi]
def test_quat():
    roll = np.random.uniform(-np.pi, np.pi)
    pitch = np.random.uniform(-np.pi/2, np.pi/2)
    yaw = np.random.uniform(-np.pi, np.pi)
    quat.set_euler_rpy(TinyVector3(roll, pitch, yaw))
    q = quat.get_euler_rpy()

    if not (abs(roll - q.x) < tolerance and abs(pitch - q.y) < tolerance and abs(yaw - q.z) < tolerance):
        print("Error: ", [roll, pitch, yaw], "\t\t", q)
        return False
    return True


def convert(angle, dim):
    euler = np.zeros(3)
    euler[dim] = angle
    quat.set_euler_rpy2(TinyVector3(euler[0], euler[1], euler[2]))
    q = quat.get_euler_rpy2()
    return q[dim]

# N = 10000
# tests = [test_quat() for _ in range(N)]
# print("Success: %.3f%%" % (sum(map(int, tests)) / N * 100.))


xs = np.linspace(-10., 10., 500)
plt.plot(xs, [convert(x, 2) for x in xs])
plt.grid()
plt.axis("equal")
plt.show()
