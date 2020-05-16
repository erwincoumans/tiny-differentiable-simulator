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

from pytinydiffsim import Motion
import matplotlib.pyplot as plt
import numpy as np

motion = Motion("/home/eric/tinyrigidbody/data/laikago_dance_sidestep0.txt")
# frames = np.array(motion.frames)

dt = motion.frame_duration
dt = 0.001

frames = [motion.calculate_frame(t * dt) for t in range(len(motion.frames))]
frames = np.array(frames)


print(motion.frames)

frames_future = [motion.calculate_frame(t * dt + motion.total_duration) for t in range(len(motion.frames))]
frames_future = np.array(frames_future)

plt.figure(figsize=(200, 6))
for i in range(frames.shape[1]):
    plt.subplot(1, frames.shape[1], i+1)
    plt.plot(frames[:, i], '.-', label="groundtruth")
    plt.plot(frames_future[:, i], label="extrapolated")
    if i == 0:
        plt.legend()

# times = [t * motion.frame_duration for t in range(len(motion.frames))]
# # plt.plot([np.floor(t / motion.frame_duration + motion.frame_duration / 4.) for t in times])
# ns = [np.floor(t / motion.frame_duration + motion.frame_duration / 4.) for t in times]
# plt.plot([np.fmod(t, motion.frame_duration) / motion.frame_duration for i, t in enumerate(times)], label="before")
# plt.plot([(t - ns[i] * motion.frame_duration) / motion.frame_duration for i, t in enumerate(times)], label="after")
# plt.legend()

plt.show()



