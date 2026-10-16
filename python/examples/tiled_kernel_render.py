"""Tiled rendering of many worlds simulated with the generated kernels.

The worlds run in a batch with the kernels of pytinykernels (generated with
CppADCodeGen, see examples/kernels), on the CPU or with CUDA. Each tile of the
window renders one world, as for training vision-based policies.

  python tiled_kernel_render.py --model g1 --num-worlds 4096 --backend cuda

--model g1       the G1 humanoid of examples/kernels/g1, dropped from 0.2 m
                 above standing height, holding the zero joint pose
--model laikago  the laikago quadruped of examples/environments, holding its
                 initial pose
"""

import argparse
import os
import time

import numpy as np

import pytinykernels as tk
import pytinyopengl3 as g


class G1:
  """Steps the g1 kernels: state, control -> next_state, and state -> poses."""

  camera_distance = 1.6
  camera_target = (0., 0., 0.6)
  texture_path = ""

  def __init__(self, batch):
    self.batch = batch
    c = batch.model.constants
    num_actuators = int(c["num_actuators"])
    state = np.zeros(batch.dim("state"))
    state[3] = 1.  # base quaternion w
    state[6] = c["standing_height"] + c["drop_height"]
    control = np.concatenate([
        np.zeros(num_actuators),  # joint targets
        np.full(num_actuators, c["default_kp"]),
        np.full(num_actuators, c["default_kd"])
    ])
    batch.fill("state", state)
    batch.fill("control", control)
    batch.upload("state")
    batch.upload("control")
    self.substeps = int(c["substeps"])

  def step(self):
    for _ in range(self.substeps):
      self.batch.launch("step")
      self.batch.swap("state", "next_state")
    self.batch.launch("poses")
    self.batch.download("poses")

  def poses(self):
    """world poses of the visuals, shape (7 * num_visuals, num_worlds)"""
    return self.batch.array("poses")


class Laikago:
  """Steps the laikago kernel: input (state, action, kp, kd, max_force) ->
  output (state, world transforms of the visuals, up_dot_world_z)."""

  camera_distance = 1.2
  camera_target = (0., 0., 0.2)
  texture_path = "laikago_tex.jpg"
  # see LaikagoContactSimulation::get_initial_poses, the actions are offsets
  # to these joint angles
  initial_joint_angles = [0.2, 0., -0.7] * 4

  def __init__(self, batch):
    self.batch = batch
    c = batch.model.constants
    self.state_dim = int(c["state_dim"])
    self.num_visuals = int(c["num_visuals"])
    action_dim = int(c["action_dim"])
    dof = self.state_dim // 2
    state = np.zeros(self.state_dim)
    # base position and rotation (x, y, z, roll, pitch, yaw), joint angles
    state[2] = 0.48
    state[6:dof] = self.initial_joint_angles
    input = np.concatenate([
        state,
        np.zeros(action_dim),
        [c["default_kp"], c["default_kd"], c["default_max_force"]]
    ])
    batch.fill("input", input)
    batch.upload("input")

  def step(self):
    # 1 ms per step, 16 steps per frame
    for _ in range(16):
      self.batch.launch("step")
      self.batch.copy_elements("input", 0, "output", 0, self.state_dim)
    self.batch.download("output")

  def poses(self):
    output = self.batch.array("output")
    return output[self.state_dim:self.state_dim + 7 * self.num_visuals]


MODELS = {"g1": G1, "laikago": Laikago}


def find_data_file(name):
  here = os.path.dirname(os.path.abspath(__file__))
  for root in [os.getcwd(), os.path.join(os.getcwd(), "data"),
               os.path.join(here, "..", "..", "data")]:
    path = os.path.join(root, name)
    if os.path.exists(path):
      return os.path.abspath(path)
  raise FileNotFoundError(name)


def main():
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawTextHelpFormatter)
  parser.add_argument("--model", default="g1", choices=sorted(MODELS))
  parser.add_argument("--num-worlds", type=int, default=1024)
  parser.add_argument("--backend", default="cuda",
                      choices=["serial", "openmp", "cuda"])
  parser.add_argument("--precision", default="float", choices=["float", "double"])
  parser.add_argument("--tiles-x", type=int, default=32)
  parser.add_argument("--tiles-y", type=int, default=24)
  parser.add_argument("--read-pixels", action="store_true",
                      help="read the rendered pixels back, as for training")
  vsync = parser.add_mutually_exclusive_group()
  vsync.add_argument("--vsync", dest="vsync", action="store_true", default=None)
  vsync.add_argument("--no-vsync", dest="vsync", action="store_false")
  args = parser.parse_args()

  batch = tk.KernelBatch(args.model, args.num_worlds, args.backend,
                         args.precision)
  robot = MODELS[args.model](batch)
  print("%s: %d worlds on %s (%s, %s)" % (args.model, args.num_worlds,
                                          batch.device_name, args.backend,
                                          args.precision))

  width, height = 1024, 768
  viz = g.OpenGLUrdfVisualizer(width=width, height=height)
  if args.vsync is not None and not viz.opengl_app.set_vsync(args.vsync):
    raise RuntimeError("The renderer could not set the requested VSYNC mode")
  viz.opengl_app.set_background_color(0.3, 0.3, 0.3)

  urdf_file = find_data_file(batch.model.strings["urdf"])
  urdf = g.UrdfParser().load_urdf(urdf_file)
  viz.path_prefix = g.extract_path(urdf_file)
  viz.convert_visuals(urdf, robot.texture_path)

  # graphics instances only for the worlds that are shown in a tile
  num_tiles = args.tiles_x * args.tiles_y
  num_shown = min(num_tiles, args.num_worlds)
  all_instances = viz.create_instances(urdf, robot.texture_path, num_shown)

  # instance -> (world, visual), to gather the instance transforms from the
  # poses with numpy
  instance_ids, worlds, visuals = [], [], []
  for world, pairs in enumerate(all_instances):
    for pair in pairs:
      instance_ids.append(pair.visual_instance)
      worlds.append(world)
      visuals.append(pair.visual_index)
  instance_ids = np.array(instance_ids, dtype=np.int32)
  worlds = np.array(worlds)
  rows = 7 * np.array(visuals)

  cam = g.TinyCamera()
  cam.set_camera_up_axis(2)
  cam.set_camera_distance(robot.camera_distance)
  cam.set_camera_pitch(-20)
  cam.set_camera_yaw(30)
  cam.set_camera_target_position(*robot.camera_target)
  viz.opengl_app.renderer.set_camera(cam)

  tiles = []
  for t in range(num_tiles):
    tile = g.TinyViewportTile()
    tile.visual_instances = [
        pair.visual_instance for pair in all_instances[t % num_shown]
    ]
    tiles.append(tile)

  timings = {"sim": 0., "sync": 0., "render": 0., "read": 0., "swap": 0.}
  frame = 0
  start = time.time()
  while not viz.opengl_app.window.requested_exit():
    t0 = time.time()
    robot.step()
    t1 = time.time()
    poses = robot.poses()
    positions = np.stack([poses[rows + k, worlds] for k in range(3)], axis=1)
    orientations = np.stack([poses[rows + 3 + k, worlds] for k in range(4)],
                            axis=1)
    viz.opengl_app.renderer.write_instance_transforms_to_cpu(
        instance_ids, positions, orientations)
    t2 = time.time()

    width = viz.opengl_app.renderer.get_screen_width()
    height = viz.opengl_app.renderer.get_screen_height()
    tile_width = width // args.tiles_x
    tile_height = height // args.tiles_y
    cam = viz.opengl_app.renderer.get_active_camera()
    projection = cam.get_camera_projection_matrix()
    view = cam.get_camera_view_matrix()
    for t, tile in enumerate(tiles):
      x, y = t % args.tiles_x, t // args.tiles_x
      tile.projection_matrix = projection
      tile.view_matrix = view
      tile.viewport_dims = [x * tile_width, y * tile_height, tile_width,
                            tile_height]
    viz.render_tiled(tiles, do_swap_buffer=False, render_segmentation_mask=False)
    t3 = time.time()
    if args.read_pixels:
      pixels = g.ReadPixelBuffer(viz.opengl_app)
    t4 = time.time()
    viz.swap_buffer()
    t5 = time.time()

    for key, dt in zip(timings, [t1 - t0, t2 - t1, t3 - t2, t4 - t3, t5 - t4]):
      timings[key] += dt
    frame += 1
    if frame % 60 == 0:
      elapsed = time.time() - start
      print("%.1f fps, %.0f world-frames/s, %d tiles, ms per frame: %s" %
            (frame / elapsed, frame * args.num_worlds / elapsed, num_tiles,
             ", ".join("%s %.2f" % (k, 1e3 * v / frame)
                       for k, v in timings.items())))
      frame = 0
      start = time.time()
      timings = dict.fromkeys(timings, 0.)


if __name__ == "__main__":
  main()
