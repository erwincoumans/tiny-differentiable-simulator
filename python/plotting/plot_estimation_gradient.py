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
file = "estimation_gradient.csv"
path = os.path.join(folder, file)

data = np.loadtxt(path)
modified = data
# for i in range(data.shape[0]):
#     modified[i, :] /= i*i
    # modified[i, :] /= pow(1.01, i)
    # modified[i, :] = np.log(modified[i, :])
plt.plot(modified)

# plt.legend()
plt.grid()
plt.title(file)

plt.show()
