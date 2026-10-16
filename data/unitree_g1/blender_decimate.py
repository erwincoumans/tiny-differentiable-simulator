"""Decimates binary STL meshes with Blender, used by convert_g1_urdf.py:

  blender -b --factory-startup --python blender_decimate.py -- jobs.json

jobs.json contains a list of [input_stl, output_stl, max_triangles]. The
Decimate modifier (collapse) reduces each mesh to at most max_triangles.
"""

import json
import sys

import bmesh
import bpy
import numpy as np

STL_TRIANGLE = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                         ("attribute", "<u2")])


def read_stl(path):
  data = np.fromfile(path, dtype=np.uint8)
  num_triangles = int(data[80:84].view("<u4")[0])
  triangles = data[84:84 + num_triangles * STL_TRIANGLE.itemsize].view(STL_TRIANGLE)
  return triangles["vertices"].reshape(-1, 3).astype(np.float64)


def write_stl(path, vertices, faces):
  triangles = np.zeros(len(faces), dtype=STL_TRIANGLE)
  corners = vertices[faces]
  normals = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
  lengths = np.linalg.norm(normals, axis=1, keepdims=True)
  triangles["normal"] = normals / np.where(lengths > 0, lengths, 1)
  triangles["vertices"] = corners
  with open(path, "wb") as f:
    f.write(b"binary STL decimated by blender_decimate.py".ljust(80, b" "))
    f.write(np.uint32(len(faces)).tobytes())
    f.write(triangles.tobytes())


def decimate(input_stl, output_stl, max_triangles):
  corners = read_stl(input_stl)
  mesh = bpy.data.meshes.new("mesh")
  mesh.from_pydata(corners, [], np.arange(len(corners)).reshape(-1, 3))
  bm = bmesh.new()
  bm.from_mesh(mesh)
  # STL stores each triangle separately, connect them first
  bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
  bm.to_mesh(mesh)
  bm.free()

  obj = bpy.data.objects.new("object", mesh)
  bpy.context.scene.collection.objects.link(obj)
  if len(mesh.polygons) > max_triangles:
    modifier = obj.modifiers.new("decimate", "DECIMATE")
    modifier.decimate_type = "COLLAPSE"
    modifier.ratio = max_triangles / len(mesh.polygons)
    modifier.use_collapse_triangulate = True
  evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
  result = evaluated.to_mesh()
  result.calc_loop_triangles()
  vertices = np.empty(len(result.vertices) * 3)
  result.vertices.foreach_get("co", vertices)
  faces = np.empty(len(result.loop_triangles) * 3, dtype=np.int64)
  result.loop_triangles.foreach_get("vertices", faces)
  write_stl(output_stl, vertices.reshape(-1, 3), faces.reshape(-1, 3))
  evaluated.to_mesh_clear()
  bpy.data.objects.remove(obj)
  bpy.data.meshes.remove(mesh)
  return len(corners) // 3, len(faces) // 3


def main():
  with open(sys.argv[sys.argv.index("--") + 1]) as f:
    jobs = json.load(f)
  for input_stl, output_stl, max_triangles in jobs:
    before, after = decimate(input_stl, output_stl, max_triangles)
    print("decimated %s: %d -> %d triangles" % (input_stl, before, after))


main()
