"""The AgileX Piper arm picks up a cube that rests on the ground, in thousands of
worlds at once, simulated with kernels baked by tds_bake.

  python piper_pick.py                          # 64 worlds in a window
  python piper_pick.py --num-worlds 4096 --mode scene --no-shadows
  python piper_pick.py --headless --backend openmp --num-worlds 16

The world is tds_bake.examples.piper_pick: the Piper of data/piper (the
piper_ros description, visual meshes decimated to 20k triangles by
data/piper/convert_piper_urdf.py), PD controlled, above a 3 cm cube. Here, the
joint targets come from inverse kinematics of the point between the
fingertips: down to the cube, close the gripper, up, and the same in every
world.
"""

import argparse
import os
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import tds_bake
from tds_bake import examples, urdf

CUBE_XY = (0.25, 0.)
CUBE_SIZE = 0.03
# the point between the fingertips in the frame of link6, on the cube
TOOL_POINT = np.array([0., 0., 0.130])
HOVER_HEIGHT = 0.12
OPEN = (0.035, -0.035)
CLOSED = (0., 0.)
# seconds: wait, down, close the gripper, hold, up, hold
SEGMENTS = (0.5, 1.5, 0.8, 0.3, 1.5, 1.0)


class Piper:
  """Forward and inverse kinematics of the six arm joints of the Piper URDF."""

  def __init__(self, urdf_file):
    import xml.etree.ElementTree as ET
    self.joints = []
    for joint in ET.parse(urdf_file).getroot().findall("joint"):
      if joint.get("type") != "revolute":
        continue
      origin = joint.find("origin")
      xyz = np.array(origin.get("xyz").split(), dtype=float)
      rpy = np.array(origin.get("rpy").split(), dtype=float)
      limit = joint.find("limit")
      transform = np.eye(4)
      transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
      transform[:3, 3] = xyz
      self.joints.append((joint.get("name"), transform, float(limit.get("lower")),
                          float(limit.get("upper"))))
    self.lower = np.array([j[2] for j in self.joints])
    self.upper = np.array([j[3] for j in self.joints])

  def pose(self, q):
    """Position of the tool point and rotation of link6 in the base frame."""
    t = np.eye(4)
    for (_, transform, _, _), angle in zip(self.joints, q):
      rz = np.eye(4)
      rz[:3, :3] = Rotation.from_euler("z", angle).as_matrix()
      t = t @ transform @ rz
    return t[:3, :3] @ TOOL_POINT + t[:3, 3], t[:3, :3]

  def solve(self, position, q0):
    """Joint angles near q0 that put the tool point at position with the
    gripper pointing down."""
    down = np.array([0., 0., -1.])

    def residual(q):
      p, r = self.pose(q)
      return np.concatenate([p - position, 0.3 * (r[:, 2] - down), 0.01 * (q - q0)])

    start = np.clip(q0, self.lower + 1e-3, self.upper - 1e-3)
    return least_squares(residual, start, bounds=(self.lower, self.upper)).x


def trajectory(piper, hover_pose, dt):
  """Joint targets (arm and fingers) of every frame of dt seconds."""
  q_hover = np.array([hover_pose["joint%d" % i] for i in range(1, 7)])
  waypoints = []  # (tool point height, gripper) at the end of each segment
  heights = [HOVER_HEIGHT, CUBE_SIZE / 2, CUBE_SIZE / 2, CUBE_SIZE / 2, HOVER_HEIGHT,
             HOVER_HEIGHT]
  grippers = [OPEN, OPEN, CLOSED, CLOSED, CLOSED, CLOSED]
  frames = []
  height, gripper = HOVER_HEIGHT, OPEN
  q = q_hover
  for duration, new_height, new_gripper in zip(SEGMENTS, heights, grippers):
    n = max(1, int(round(duration / dt)))
    for i in range(1, n + 1):
      u = i / n
      u = u * u * (3 - 2 * u)  # smooth start and end
      h = height + u * (new_height - height)
      g = np.array(gripper) + u * (np.array(new_gripper) - np.array(gripper))
      q = piper.solve(np.array([CUBE_XY[0], CUBE_XY[1], h]), q)
      frames.append(np.concatenate([q, g]))
    height, gripper = new_height, new_gripper
  return np.array(frames)


def main():
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("--num-worlds", type=int, default=64)
  parser.add_argument("--backend", default="cuda", choices=["cuda", "openmp", "serial"])
  parser.add_argument("--precision", default="float", choices=["float", "double"])
  parser.add_argument("--mode", default="tiles", choices=["tiles", "scene"])
  parser.add_argument("--tiles-x", type=int, default=8)
  parser.add_argument("--tiles-y", type=int, default=8)
  parser.add_argument("--no-shadows", action="store_true")
  parser.add_argument("--headless", action="store_true",
                      help="no window: run the pick and print the height of the cube")
  args = parser.parse_args()

  world = examples.piper_pick()
  baked = tds_bake.bake(world, backends=[args.backend], precisions=[args.precision])
  body = baked.body("piper")
  piper = Piper(urdf.find_file(examples.PIPER_URDF))
  frame_time = baked.dt * baked.substeps
  targets = trajectory(piper, examples.PIPER_HOVER_POSE, frame_time)
  # the actuators in the order of the control array
  index = [body.actuator_names.index("joint%d" % i) for i in range(1, 9)]
  control = baked.default_control()

  def controller(batch, frame):
    t = np.array(control)
    row = targets[min(frame, len(targets) - 1)]
    t[body.targets.start + np.array(index)] = row
    batch.set("control", t)

  if args.headless:
    batch = baked.batch(args.num_worlds, args.backend, args.precision)
    cube = baked.body("cube")
    for frame in range(len(targets)):
      controller(batch, frame)
      batch.step()
      if frame % 30 == 0 or frame == len(targets) - 1:
        z = batch.get("state")[:, cube.base_position][:, 2]
        print("t %.2f s: cube height %.4f m (min %.4f, max %.4f)" %
              (frame * frame_time, z.mean(), z.min(), z.max()))
    return

  from tds_bake.render import view
  view(baked, num_worlds=args.num_worlds, backend=args.backend, precision=args.precision,
       mode=args.mode, tiles=(args.tiles_x, args.tiles_y), shadows=not args.no_shadows,
       controller=controller)


if __name__ == "__main__":
  main()
