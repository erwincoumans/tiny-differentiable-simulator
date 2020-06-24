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

folder = "../../data/schmidt-lipson-exp-data/"
file = "real_pend_a_1.txt" # "real_double_pend_h_1.txt"
path = os.path.join(folder, file)

with open(path, "r") as f:
    header = f.readline().strip().split()[1:]
    print("header:", header)
    lines = f.readlines()
    data = np.array([np.fromstring(line, sep=" ") for line in lines])

print("Loaded %i entries." % data.shape[0])
times = data[:, 1]

for i, h in enumerate(header):
    plt.plot(times, data[:, 2 + i], label=h)
plt.legend()
plt.grid()
plt.title(file)

deltas = []
for t1, t2 in zip(times[:-1], times[1:]):
    deltas.append(t2-t1)

# plt.plot(deltas)
dt = np.mean(deltas)
print("avg delta t: ", dt, " \t FPS: ", 1./dt)
plt.savefig("plot_%s.png" % file[:-4])
plt.show()
