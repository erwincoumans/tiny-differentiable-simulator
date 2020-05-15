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

q_dim = 18
qd_dim = 18
state_dim = q_dim + qd_dim
control_dim = 12
# motion_file = "laikago_dance_sidestep0"
motion_file = "laikago_builtin_walk_inplace"

with open("/home/eric/tiny-differentiable-simulator/data/%s.txt" % motion_file, "r") as fp:
    ref_json = json.load(fp)
    ref_frames = np.array(ref_json["Frames"])
    ref_duration = ref_json["FrameDuration"]


our_frames = np.loadtxt("traj_%s.csv" % motion_file)
our_times = our_frames[:, 0]

# crop / extend reference trajectory to match length of our trajectory
needed_ref_frames = int(np.round(our_times[-1] / ref_duration))
if needed_ref_frames <= ref_frames.shape[0]:
    ref_frames = ref_frames[:needed_ref_frames, :]
else:
    ref_frames = np.tile(ref_frames, (math.ceil(needed_ref_frames / ref_frames.shape[0]), 1))
    ref_frames = ref_frames[:needed_ref_frames, :]

ref_times = np.linspace(start=0., stop=ref_frames.shape[0] * ref_duration, num=ref_frames.shape[0])

fig, axs = plt.subplots(4, 3, figsize=(10, 9))
for i in range(12):
    ax = axs[int(i / 3)][i % 3]
    ax.plot(our_times, our_frames[:, 7 + i], label=("Ours" if i == 0 else None))
    ax.plot(ref_times, ref_frames[:, 7 + i], label=("Reference" if i == 0 else None))
    ax.set_title("$q_{%i}$" % i)
    ax.grid()
    if i == 0:
        ax.legend()

plt.tight_layout()
plt.savefig("tracking_states_%s.png" % motion_file)
fig.show()


fig, axs = plt.subplots(4, 3, figsize=(10, 9))
for i in range(12):
    ax = axs[int(i / 3)][i % 3]
    ax.plot(our_times, our_frames[:, 37 + i])
    ax.set_title("$u_{%i}$" % i)
    ax.grid()
#
plt.tight_layout()
plt.savefig("tracking_controls_%s.png" % motion_file)
fig.show()

