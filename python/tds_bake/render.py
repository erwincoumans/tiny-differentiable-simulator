"""Rendering a batch of worlds with pytinyopengl3, in one of two modes:

  tiles   one world per tile of the window, each seen by the same camera, as
          for training vision-based policies
  scene   all worlds in one 3D scene, side by side on a grid, with a free
          camera (hold CTRL and the left / middle / right mouse button to
          rotate / move / zoom)

  viewer = tds_bake.render.Viewer(baked, num_shown=64, mode="tiles",
                                  tiles=(8, 8))
  while not viewer.requested_exit():
    batch.step()
    viewer.render_batch(batch)

With a CUDA batch, render_batch writes the poses of the visual shapes from
the batch into the vertex buffer of the renderer with CUDA-OpenGL interop
(pytinykernels.CudaGlInstanceWriter), without copies through the CPU. Set
TDS_BAKE_GL_INTEROP=0 to copy them through the CPU instead.
"""

import math
import os
import time

import numpy as np


class Viewer:
  """Renders the first num_shown worlds of a batch."""

  def __init__(self,
               baked,
               num_shown=64,
               mode="tiles",
               tiles=(8, 8),
               spacing=None,
               width=1024,
               height=768,
               camera_distance=None,
               camera_target=None,
               camera_pitch=-20.,
               camera_yaw=30.,
               vsync=None,
               interop=None,
               shadows=True):
    import pytinyopengl3 as g
    if mode not in ("tiles", "scene"):
      raise ValueError("mode must be tiles or scene")
    self.g = g
    self.mode = mode
    self.tiles = tiles
    self.num_tiles = tiles[0] * tiles[1]
    num_shown = max(1, num_shown)
    if mode == "tiles":
      num_shown = min(num_shown, self.num_tiles)
    self.num_shown = num_shown
    num_instances = num_shown * max(1, baked.num_visuals)
    self.viz = g.OpenGLUrdfVisualizer(
        width=width,
        height=height,
        max_num_object_capacity=max(256 * 1024, num_instances + 1024))
    if vsync is not None:
      self.viz.opengl_app.set_vsync(vsync)
    self.viz.opengl_app.set_background_color(0.3, 0.3, 0.3)
    # the shadow map pass draws all worlds once more
    self.viz.opengl_app.renderer.shadows = shadows

    # the shapes of all bodies first, then their instances: registering
    # shapes loses the instances of earlier shapes
    urdfs = {}
    for name in baked.bodies:
      path = baked.urdf_files[name]
      urdfs[name] = g.UrdfParser().load_urdf(path)
      self.viz.path_prefix = g.extract_path(path)
      self.viz.convert_visuals(urdfs[name], "")
    # instance -> (world, visual), to gather the instance transforms from the
    # poses with numpy
    instance_ids, worlds, visuals = [], [], []
    world_instances = [[] for _ in range(num_shown)]
    for name, body in baked.bodies.items():
      all_instances = self.viz.create_instances(urdfs[name], "", num_shown)
      for world, pairs in enumerate(all_instances):
        for pair in pairs:
          instance_ids.append(pair.visual_instance)
          worlds.append(world)
          visuals.append(body.visual_offset + pair.visual_index)
          world_instances[world].append(pair.visual_instance)
    self.instance_ids = np.array(instance_ids, dtype=np.int32)
    self.worlds = np.array(worlds, dtype=np.int64)
    self.visuals = np.array(visuals, dtype=np.int64)

    # scene: world i at a grid position, around the origin
    self.spacing = 1.5 if spacing is None else spacing
    columns = int(math.ceil(math.sqrt(num_shown)))
    index = np.arange(num_shown)
    offsets = np.zeros((num_shown, 3))
    offsets[:, 0] = (index % columns - (columns - 1) / 2.) * self.spacing
    offsets[:, 1] = (index // columns - (columns - 1) / 2.) * self.spacing
    self.offsets = offsets[self.worlds] if mode == "scene" else None

    self.tile_list = []
    if mode == "tiles":
      for t in range(self.num_tiles):
        tile = g.TinyViewportTile()
        tile.visual_instances = world_instances[t % num_shown]
        self.tile_list.append(tile)

    if camera_target is None:
      camera_target = (0., 0., 0.6)
    if camera_distance is None:
      camera_distance = 2.5
    if mode == "scene":
      # see the whole grid
      camera_distance = max(camera_distance, 0.9 * columns * self.spacing)
      camera_pitch = min(camera_pitch, -30.)
    cam = g.TinyCamera()
    cam.set_camera_up_axis(2)
    cam.set_camera_distance(float(camera_distance))
    cam.set_camera_pitch(float(camera_pitch))
    cam.set_camera_yaw(float(camera_yaw))
    cam.set_camera_target_position(*[float(x) for x in camera_target])
    self.viz.opengl_app.renderer.set_camera(cam)
    self.frames = 0
    if interop is None:
      interop = os.environ.get("TDS_BAKE_GL_INTEROP", "1") != "0"
    self.interop = interop
    self._writer = None

  def requested_exit(self):
    return self.viz.opengl_app.window.requested_exit()

  def set_camera_target(self, target):
    cam = self.viz.opengl_app.renderer.get_active_camera()
    cam.set_camera_target_position(*[float(x) for x in target])

  def _gl_writer(self):
    """The CUDA-OpenGL interop writer of the instance transforms, created on
    first use."""
    if self._writer is None:
      import pytinykernels as tk
      from . import baking
      tk.nvrtc(baking.find_nvrtc())
      renderer = self.viz.opengl_app.renderer
      # the vertex buffer gets the colors and scales of the instances
      renderer.write_transforms()
      buffer = renderer.get_instance_buffer()
      internal = renderer.get_internal_instance_indices(self.instance_ids)
      offsets = (self.offsets if self.offsets is not None else np.zeros(
          (len(self.instance_ids), 3)))
      # visual-major order: neighboring threads read neighboring worlds
      order = np.lexsort((self.worlds, self.visuals))
      self._writer = tk.CudaGlInstanceWriter(
          buffer["vbo"], buffer["positions_offset"],
          buffer["orientations_offset"],
          self.worlds[order].astype(np.int32).tolist(),
          self.visuals[order].astype(np.int32).tolist(),
          np.asarray(internal)[order].astype(np.int32).tolist(),
          np.asarray(offsets, dtype=np.float32)[order].reshape(-1).tolist(),
          self.device)
    return self._writer

  def render_batch(self, batch, read_pixels=False):
    """Renders the worlds of a Batch. With CUDA, the poses go from the batch
    to the renderer on the GPU (see interop). Returns the pixels if
    read_pixels."""
    if not (self.interop and batch.is_cuda):
      return self.render(batch.poses(), read_pixels)
    self.device = batch.baked.device
    batch._batch.launch("poses")
    self._gl_writer().write(batch._batch, "poses")
    return self._render(read_pixels, write_transforms=False)

  def render(self, poses, read_pixels=False):
    """Renders the worlds with the poses of Batch.poses(),
    (num_worlds, num_visuals, 7). Returns the pixels if read_pixels."""
    selected = poses[self.worlds, self.visuals]
    positions = selected[:, :3]
    if self.offsets is not None:
      positions = positions + self.offsets
    renderer = self.viz.opengl_app.renderer
    renderer.write_instance_transforms_to_cpu(
        self.instance_ids, np.ascontiguousarray(positions, dtype=np.float32),
        np.ascontiguousarray(selected[:, 3:7], dtype=np.float32))
    return self._render(read_pixels, write_transforms=True)

  def _render(self, read_pixels, write_transforms):
    g = self.g
    renderer = self.viz.opengl_app.renderer
    if self.mode == "scene":
      self.viz.render(do_swap_buffer=False, write_transforms=write_transforms)
    else:
      width = renderer.get_screen_width()
      height = renderer.get_screen_height()
      tile_width = width // self.tiles[0]
      tile_height = height // self.tiles[1]
      cam = renderer.get_active_camera()
      projection = cam.get_camera_projection_matrix()
      view = cam.get_camera_view_matrix()
      for t, tile in enumerate(self.tile_list):
        x, y = t % self.tiles[0], t // self.tiles[0]
        tile.projection_matrix = projection
        tile.view_matrix = view
        tile.viewport_dims = [x * tile_width, y * tile_height, tile_width,
                              tile_height]
      self.viz.render_tiled(self.tile_list, do_swap_buffer=False,
                            render_segmentation_mask=False,
                            write_transforms=write_transforms)
    pixels = g.ReadPixelBuffer(self.viz.opengl_app) if read_pixels else None
    self.viz.swap_buffer()
    self.frames += 1
    return pixels


# the name of the first version
TiledViewer = Viewer


def view(baked, num_worlds=64, backend="cuda", precision="float",
         mode="tiles", tiles=(8, 8), num_shown=None, spacing=None,
         shadows=True, controller=None, max_frames=None, real_time=True):
  """Simulates num_worlds worlds and renders them until the window is closed:
  in tiles (the first tiles[0] * tiles[1] worlds), or all in one 3D scene
  (mode "scene", num_shown worlds, default all). controller(batch, frame)
  can set the control before each step."""
  batch = baked.batch(num_worlds, backend, precision)
  links = batch.links()[0]
  target = links[:, :3].mean(axis=0)
  extent = np.ptp(links[:, :3], axis=0)
  if spacing is None:
    # the horizontal size of a world, with some room
    spacing = max(1., 2. * float(max(extent[0], extent[1])))
  if num_shown is None:
    num_shown = num_worlds
  num_shown = min(num_shown, num_worlds)
  if mode == "scene":
    target = (0., 0., float(target[2]))
  viewer = Viewer(baked, num_shown=num_shown, mode=mode, tiles=tiles,
                  spacing=spacing,
                  camera_distance=max(1., 1.4 * float(extent.max())),
                  camera_target=target, shadows=shadows)
  if mode == "scene":
    print("hold CTRL and the left / middle / right mouse button to rotate / "
          "move / zoom the camera", flush=True)
  frame_time = batch.substeps * batch.dt
  frame = 0
  start = time.time()
  while not viewer.requested_exit():
    if controller is not None:
      controller(batch, frame)
    batch.step()
    viewer.render_batch(batch)
    frame += 1
    if max_frames is not None and frame >= max_frames:
      break
    if real_time:
      delay = start + frame * frame_time - time.time()
      if delay > 0:
        time.sleep(delay)
    if frame % 120 == 0:
      print("%.1f frames/s" % (frame / (time.time() - start)), flush=True)
  return batch
