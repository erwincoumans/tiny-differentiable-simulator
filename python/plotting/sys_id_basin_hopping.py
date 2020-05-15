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
import matplotlib.colors as colors
import matplotlib.cbook as cbook
import numpy as np

raw = np.loadtxt("grid_search_bhe.txt")
data = raw[:, 2].reshape(100, 100)
error = np.linalg.norm(raw[:, 3:5] - np.array([[3.0, 4.0]]), axis=1)
error = error.reshape(100, 100)
print(error)

# print(data[0])

# plt.imshow(data, cmap='hot', interpolation='nearest')
# plt.grid()
# plt.gca().set_xticks(np.arange(0, 100, 10))
# plt.gca().set_yticks(np.arange(0, 100, 10))
# plt.gca().set_xticklabels(np.arange(0, 10, 1))
# plt.gca().set_yticklabels(np.arange(0, 10, 1))
# plt.xlabel("Link length 1")
# plt.ylabel("Link length 2")
# plt.colorbar()
# plt.show()

X, Y = np.mgrid[0.1:10.1:0.1, 0.1:10.1:0.1]

# A low hump with a spike coming out of the top right.  Needs to have
# z/colour axis on a log scale so we see both hump and spike.  linear
# scale only shows the spike.
# Z1 = np.exp(-(X)**2 - (Y)**2)
# Z2 = np.exp(-(X * 10)**2 - (Y * 10)**2)
# Z = Z1 + 50 * Z2

fig, ax = plt.subplots(1, 2)

pcm = ax[0].pcolor(X, Y, data,
                   norm=colors.LogNorm(vmin=data.min(), vmax=data.max()),
                   cmap='PuBu_r')
fig.colorbar(pcm, ax=ax[0], extend='max')
ax[0].set_xlabel("Link length 1")
ax[0].set_ylabel("Link length 2")
ax[0].set_title("Final Cost")
ax[0].set_aspect('equal', 'box')

print("error.min()", error.min())
print("error.max()", error.max())

pcm = ax[1].pcolor(X, Y, error,
                   norm=colors.Normalize(vmin=error.min(), vmax=error.max()),
                   cmap='PuBu_r')
fig.colorbar(pcm, ax=ax[1], extend='max')
ax[1].set_xlabel("Link length 1")
ax[1].set_ylabel("Link length 2")
ax[1].set_title("True Parameter Error")
ax[1].set_aspect('equal', 'box')

# pcm = ax[1].pcolor(X, Y, Z, cmap='PuBu_r')
# fig.colorbar(pcm, ax=ax[1], extend='max')
plt.show()
