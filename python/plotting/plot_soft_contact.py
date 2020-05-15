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

import pydiffphys
from pydiffphys import TinyMultiBodyConstraintSolverSpring as SoftContactModel

model = SoftContactModel()
model.smoothing_method = pydiffphys.SMOOTH_VEL_NONE
model.exponent_n_air = 1e-6
model.exponent_vel_air = 1e-6
# model.damper_d = 100000


@np.vectorize
def normal_force(penetration, penetration_vel):
    return model.compute_contact_force(penetration, penetration_vel)


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_title("Contact Normal Force")

# Make data.
X = np.arange(-5, 5, 0.25)
Y = np.arange(-5, 5, 0.25)
X, Y = np.meshgrid(X, Y)
Z = normal_force(X, Y)

# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

ax.set_xlabel("Penetration")
ax.set_ylabel("Penetration Velocity")

# Add a color bar which maps values to colors.
# fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
