import os
import pytinyopengl3 as p


def load_vxl(data, mapx=512, mapy=512, mapz=64):
  """Parse a voxlap/Ace-of-Spades .vxl map (RLE-encoded columns).

  Returns a list of (x, y, z, r, g, b) tuples for every colored voxel.
  Uncolored (interior/underground) voxels are not stored in the format
  and are skipped here, since they are never visible anyway.
  """
  voxels = []
  i = 0
  for y in range(mapy):
    for x in range(mapx):
      while True:
        num_chunks = data[i]
        top_start = data[i + 1]
        top_end = data[i + 2]      # inclusive
        len_top = top_end - top_start + 1

        colors = i + 4
        for z in range(top_start, top_end + 1):
          off = colors + (z - top_start) * 4
          b, g, r, a = data[off], data[off + 1], data[off + 2], data[off + 3]
          voxels.append((x, y, z, r, g, b))

        if num_chunks == 0:
          # last span in this column: header + top colors only
          i += 4 * (len_top + 1)
          break

        len_bottom = (num_chunks - 1) - len_top
        # the end of the bottom run is stored in byte 3 of the *next*
        # span's header (it doubles as that span's "air start" marker)
        next_header = i + num_chunks * 4
        bottom_colors = colors + len_top * 4
        bottom_end = data[next_header + 3]
        bottom_start = bottom_end - len_bottom
        for z in range(bottom_start, bottom_end):
          off = bottom_colors + (z - bottom_start) * 4
          b, g, r, a = data[off], data[off + 1], data[off + 2], data[off + 3]
          voxels.append((x, y, z, r, g, b))

        i = next_header
  return voxels


vxl_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test.vxl")
with open(vxl_path, "rb") as f:
  vxl_data = f.read()

voxels = load_vxl(vxl_data)
num_objects = len(voxels)
print("loaded", num_objects, "voxels from", vxl_path)

app = p.TinyOpenGL3App("voxel", maxNumObjectCapacity=num_objects + 10)
app.renderer.init()

# center the voxel cluster around the origin
xs = [v[0] for v in voxels]
ys = [v[1] for v in voxels]
zs = [v[2] for v in voxels]
cx = (min(xs) + max(xs)) / 2. if voxels else 0.
cy = (min(ys) + max(ys)) / 2. if voxels else 0.
cz = (min(zs) + max(zs)) / 2. if voxels else 0.

voxel_scale = 0.1
extent = max(max(xs) - min(xs), max(ys) - min(ys)) * voxel_scale if voxels else 10.

cam = p.TinyCamera()
cam.set_camera_distance(extent * 1.5)
cam.set_camera_pitch(-35)
app.renderer.set_camera(cam)

opacity = 1
rebuild = True
# half-extents of 0.5 -> a unit cube, so with vec_scaling = voxel_scale the
# cubes exactly tile the voxel grid (spacing = voxel_scale) instead of
# overlapping their neighbors and z-fighting
shape = app.register_cube_shape(0.5, 0.5, 0.5, -1, 1)
orn = p.TinyQuaternionf(0., 0., 0., 1.)
scaling = p.TinyVector3f(voxel_scale, voxel_scale, voxel_scale)

vec_pos = []
vec_orn = []
vec_color = []
vec_scaling = []
for (x, y, z, r, g, b) in voxels:
  vec_pos.append(p.TinyVector3f((x - cx) * voxel_scale,
                                 (y - cy) * voxel_scale,
                                 (cz - z) * voxel_scale))
  vec_orn.append(orn)
  vec_color.append(p.TinyVector3f(r / 255., g / 255., b / 255.))
  vec_scaling.append(scaling)

app.renderer.register_graphics_instances(shape, vec_pos, vec_orn, vec_color, vec_scaling, opacity, rebuild)
app.renderer.write_transforms()

dg = p.DrawGridData()
dg.drawAxis = True

while not app.window.requested_exit():
  app.renderer.update_camera(2)
  app.draw_grid(dg)
  app.renderer.render_scene()
  app.swap_buffer()
