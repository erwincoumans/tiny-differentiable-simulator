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

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from matplotlib import cm
import numpy as np
import json
import math

q_dim = 9
qd_dim = 9
state_dim = q_dim + qd_dim
control_dim = 6

model_name = "cheetah"

our_frames = np.loadtxt("traj_%s.csv" % model_name)
our_times = our_frames[:, 0]

fig, axs = plt.subplots(6, 3, figsize=(10, 9))
for i in range(state_dim):
    ax = axs[int(i / 3)][i % 3]
    ax.plot(our_times, our_frames[:, 1 + i])
    if i < q_dim:
        ax.set_title("$q_{%i}$" % i)
    else:
        ax.set_title("$\\dot{q}_{%i}$" % (i - q_dim))
    ax.grid()

plt.tight_layout()
plt.savefig("tracking_states_%s.png" % model_name)
fig.show()


fig, axs = plt.subplots(2, 3, figsize=(8, 5))
for i in range(control_dim):
    ax = axs[int(i / 3)][i % 3]
    ax.plot(our_times, our_frames[:, 1 + state_dim + i])
    ax.set_title("$u_{%i}$" % i)
    ax.grid()
#
plt.tight_layout()
plt.savefig("tracking_controls_%s.png" % model_name)
fig.show()

