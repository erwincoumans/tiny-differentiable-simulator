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
evolution_file = "param_evolution.txt"

path = os.path.join(folder, evolution_file)
evolution_data = np.loadtxt(path)

fig, axs = plt.subplots(3, 1, figsize=(5,  3 * 3.5), sharex=True)
axs[0].plot(evolution_data[:, 0])
axs[0].grid()
axs[0].set_title("Spring $k$")

axs[1].plot(evolution_data[:, 1])
axs[1].grid()
axs[1].set_title("Damper $d$")

axs[2].plot(evolution_data[:, 2:])
axs[2].grid()
axs[2].set_title("Weights")

plt.savefig("param_evolution.png")
plt.show()
