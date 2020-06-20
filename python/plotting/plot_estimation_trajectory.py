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

import numpy as np
import matplotlib.pyplot as plt
import os

folder = "../../"
estimation_file = "estimated_trajectory.csv"
true_file = "data/schmidt-lipson-exp-data/real_double_pend_h_1.txt"

path = os.path.join(folder, estimation_file)
estimation_data = np.loadtxt(path)
path = os.path.join(folder, true_file)
with open(path, "r") as f:
    header = f.readline().strip().split()[1:]
    print("header:", header)
    lines = f.readlines()
    true_data = np.array([np.fromstring(line, sep=" ") for line in lines])
    # the first joint angle is rotated by 90deg compared to our model
    true_data[:, 2] -= np.pi / 2
    # true_data[:, 3] -= np.pi / 2
    true_data = true_data[:500, :]

state_dim = 2

fig, axs = plt.subplots(state_dim, 1, figsize=(5, state_dim * 3.5), sharex=True)
for i in range(state_dim):
    axs[i].plot(estimation_data[:, 0], estimation_data[:, i+1], label="estimated")
    axs[i].plot(true_data[:, 1], true_data[:, i+2], label="actual")
    axs[i].grid()
    if i == 0:
        axs[i].legend()
    axs[i].set_title("$\\mathbf{q}_%i$" % i)

plt.savefig("estimation_traj.png")
plt.show()
