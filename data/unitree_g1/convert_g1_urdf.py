#!/usr/bin/env python3
"""Converts the Unitree G1 URDF (29 dof + Dex3 hands) from newton-assets into a
URDF that Tiny Differentiable Simulator can load, see
examples/g1_opengl_example.cpp.

The conversion mirrors the model setup of the Newton example
newton/examples/robot/example_robot_g1.py:
  * fixed joints are collapsed (collapse_fixed_joints=True): mass properties,
    visuals and collision shapes are merged into the parent link,
  * collision meshes are replaced by oriented bounding boxes
    (ModelBuilder.approximate_meshes("bounding_box")). Cylinders become boxes
    too, TDS only supports sphere, capsule and box collision shapes,
  * inertia tensors are written with respect to the center of mass, in link
    frame orientation (rpy = 0), including the off-diagonal terms,
  * visual meshes are written as binary STL files, decimated by Blender
    (see blender_decimate.py) to a total of --total-triangles (default 20k,
    from 629k). The torso and the logo get a fixed budget
    (--fixed-triangles), so that the logo stays readable, the other meshes
    share the rest in proportion to the square root of their surface area,
    which gives the small parts (fingers) more than their share of the area.
    The meshes are only rendered, the collision shapes are boxes and spheres.
    With --total-triangles 0, meshes with more than --max-triangles are
    decimated to that number instead (0 keeps the original meshes).

Usage:
  python convert_g1_urdf.py [path/to/newton-assets/unitree_g1] [--out DIR]
      [--total-triangles N] [--fixed-triangles torso_link_rev_1_0=1500,...]
      [--max-triangles N] [--blender PATH]

Without a path, the Newton asset cache (~/.cache/newton) is searched, run
newton.utils.download_asset("unitree_g1") to populate it.
Requires numpy, trimesh and Blender (unless no mesh is decimated). The Blender
executable is taken from --blender, the BLENDER environment variable or PATH.
"""

import argparse
import glob
import json
import os
import subprocess
import tempfile

import xml.etree.ElementTree as ET

import numpy as np
import trimesh

URDF_NAME = "g1_29dof_with_hand_rev_1_0.urdf"


def rpy_to_matrix(rpy):
  r, p, y = rpy
  rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
  ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
  rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
  return rz @ ry @ rx


def matrix_to_rpy(m):
  # inverse of rpy_to_matrix, R = Rz(y) Ry(p) Rx(r)
  p = np.arcsin(np.clip(-m[2, 0], -1.0, 1.0))
  if abs(np.cos(p)) > 1e-9:
    r = np.arctan2(m[2, 1], m[2, 2])
    y = np.arctan2(m[1, 0], m[0, 0])
  else:
    r = 0.0
    y = np.arctan2(-m[0, 1], m[1, 1])
  return np.array([r, p, y])


def parse_vec(text, default=(0.0, 0.0, 0.0)):
  if text is None:
    return np.array(default, dtype=float)
  return np.array([float(v) for v in text.split()])


def parse_origin(elem):
  tf = np.eye(4)
  origin = elem.find("origin") if elem is not None else None
  if origin is not None:
    tf[:3, :3] = rpy_to_matrix(parse_vec(origin.get("rpy")))
    tf[:3, 3] = parse_vec(origin.get("xyz"))
  return tf


def fmt(values):
  return " ".join("%.9g" % (0.0 if abs(v) < 1e-12 else v) for v in values)


def origin_element(parent, tf):
  ET.SubElement(parent, "origin", xyz=fmt(tf[:3, 3]),
                rpy=fmt(matrix_to_rpy(tf[:3, :3])))


class Inertial:

  def __init__(self, mass, com, inertia):
    self.mass = mass
    self.com = com  # center of mass in link frame
    self.inertia = inertia  # 3x3 tensor about com, link frame orientation

  @staticmethod
  def from_urdf(elem):
    tf = parse_origin(elem)
    mass = float(elem.find("mass").get("value"))
    i = elem.find("inertia")
    ixx, ixy, ixz, iyy, iyz, izz = [
        float(i.get(k, "0")) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    ]
    inertia = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    rot = tf[:3, :3]
    return Inertial(mass, tf[:3, 3], rot @ inertia @ rot.T)

  def transformed(self, tf):
    rot = tf[:3, :3]
    return Inertial(self.mass, rot @ self.com + tf[:3, 3],
                    rot @ self.inertia @ rot.T)

  def merged(self, other):
    mass = self.mass + other.mass
    com = (self.mass * self.com + other.mass * other.com) / mass
    inertia = np.zeros((3, 3))
    for part in (self, other):
      d = part.com - com
      # parallel axis theorem
      inertia += part.inertia + part.mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return Inertial(mass, com, inertia)


class Link:

  def __init__(self, elem):
    self.name = elem.get("name")
    inertial = elem.find("inertial")
    self.inertial = Inertial.from_urdf(inertial) if inertial is not None else None
    # (link_X_shape, geometry element, material element)
    self.visuals = [(parse_origin(v), v.find("geometry")[0], v.find("material"))
                    for v in elem.findall("visual")]
    self.collisions = [(parse_origin(c), c.find("geometry")[0])
                       for c in elem.findall("collision")]


class Joint:

  def __init__(self, elem):
    self.elem = elem
    self.name = elem.get("name")
    self.type = elem.get("type")
    self.parent = elem.find("parent").get("link")
    self.child = elem.find("child").get("link")
    self.origin = parse_origin(elem)


def mesh_path(asset_dir, geom):
  # filenames look like package://unitree_g1/meshes/pelvis.STL
  return os.path.join(asset_dir, "meshes", os.path.basename(geom.get("filename")))


def collapse_fixed_joints(links, joints):
  for joint in [j for j in joints if j.type == "fixed"]:
    parent, child = links[joint.parent], links[joint.child]
    tf = joint.origin
    if child.inertial is not None:
      moved = child.inertial.transformed(tf)
      parent.inertial = moved if parent.inertial is None else parent.inertial.merged(moved)
    parent.visuals += [(tf @ x, g, m) for x, g, m in child.visuals]
    parent.collisions += [(tf @ x, g) for x, g in child.collisions]
    for other in joints:
      if other.parent == child.name:
        other.parent = parent.name
        other.origin = tf @ other.origin
    del links[child.name]
  return [j for j in joints if j.type != "fixed"]


def convert_collision(asset_dir, tf, geom):
  """Returns (link_X_shape, geometry element) using shapes TDS can collide."""
  if geom.tag in ("sphere", "box", "capsule"):
    return tf, geom
  if geom.tag == "cylinder":
    r, l = float(geom.get("radius")), float(geom.get("length"))
    return tf, ET.Element("box", size=fmt([2 * r, 2 * r, l]))
  if geom.tag == "mesh":
    mesh = trimesh.load(mesh_path(asset_dir, geom), force="mesh")
    obb = mesh.bounding_box_oriented
    return tf @ obb.primitive.transform, ET.Element("box", size=fmt(obb.primitive.extents))
  raise ValueError("unsupported collision geometry " + geom.tag)


def stl_triangle_count(path):
  with open(path, "rb") as f:
    f.seek(80)
    return int(np.frombuffer(f.read(4), dtype="<u4")[0])


def visual_mesh_files(asset_dir, geom, out_dir):
  """The input mesh and the output binary STL of a visual mesh, and its URDF
  filename."""
  name = os.path.splitext(os.path.basename(geom.get("filename")))[0] + ".stl"
  return mesh_path(asset_dir, geom), os.path.join(out_dir, "meshes", name), "meshes/" + name


def triangle_budgets(mesh_files, total, fixed, max_triangles, min_triangles=24):
  """The number of triangles of each output mesh, see the module docstring.
  fixed maps mesh names (without extension) to their budget."""
  sources = {i: trimesh.load(i, force="mesh") for i, _ in mesh_files}
  budgets = {}
  if total > 0:
    rest = [(i, o) for i, o in mesh_files
            if os.path.splitext(os.path.basename(o))[0] not in fixed]
    weights = {i: np.sqrt(sources[i].area) for i, _ in rest}
    remaining = total - sum(fixed.get(os.path.splitext(os.path.basename(o))[0], 0)
                            for _, o in mesh_files)
    for i, o in mesh_files:
      name = os.path.splitext(os.path.basename(o))[0]
      if name in fixed:
        budgets[o] = fixed[name]
      else:
        budgets[o] = max(min_triangles,
                         int(round(remaining * weights[i] / sum(weights.values()))))
  else:
    for i, o in mesh_files:
      budgets[o] = max_triangles if max_triangles > 0 else len(sources[i].faces)
  return sources, budgets


def decimate_with_blender(blender, jobs):
  """jobs: [input_stl, output_stl, max_triangles] of each mesh"""
  script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "blender_decimate.py")
  with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
    json.dump(jobs, f)
  try:
    result = subprocess.run(
        [blender, "-b", "--factory-startup", "--python-exit-code", "1", "--python", script,
         "--", f.name], capture_output=True, text=True)
  finally:
    os.remove(f.name)
  if result.returncode != 0:
    raise SystemExit("Blender decimation failed:\n" + result.stdout + result.stderr)


def find_asset_dir():
  pattern = os.path.expanduser("~/.cache/newton/newton-assets_unitree_g1_*/unitree_g1")
  candidates = sorted(glob.glob(pattern), key=os.path.getmtime)
  if not candidates:
    raise SystemExit("unitree_g1 asset not found, pass its path as argument")
  return candidates[-1]


def main():
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("asset_dir", nargs="?", help="newton-assets unitree_g1 folder")
  parser.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
  parser.add_argument("--total-triangles", type=int, default=20000,
                      help="triangles of all visual meshes, 0 uses --max-triangles")
  parser.add_argument("--fixed-triangles", default="torso_link_rev_1_0=1500,logo_link=1500",
                      help="budgets of single meshes, as name=triangles,...")
  parser.add_argument("--max-triangles", type=int, default=0,
                      help="with --total-triangles 0: decimate visual meshes to at most this "
                      "many triangles, 0 keeps all")
  parser.add_argument("--blender", default=os.environ.get("BLENDER", "blender"),
                      help="Blender executable, used for decimation")
  args = parser.parse_args()
  asset_dir = args.asset_dir or find_asset_dir()
  print("Converting", os.path.join(asset_dir, "urdf", URDF_NAME))

  robot = ET.parse(os.path.join(asset_dir, "urdf", URDF_NAME)).getroot()
  links = {l.get("name"): Link(l) for l in robot.findall("link")}
  joints = collapse_fixed_joints(links, [Joint(j) for j in robot.findall("joint")])

  os.makedirs(os.path.join(args.out, "meshes"), exist_ok=True)
  out_robot = ET.Element("robot", name=robot.get("name"))
  fixed = {}
  for item in filter(None, args.fixed_triangles.split(",")):
    name, count = item.split("=")
    fixed[name] = int(count)
  if args.total_triangles > 0:
    decimation = (", visual meshes decimated by Blender to %d triangles in total (%s)" %
                  (args.total_triangles, args.fixed_triangles))
  elif args.max_triangles > 0:
    decimation = (", visual meshes decimated by Blender to at most %d triangles" %
                  args.max_triangles)
  else:
    decimation = ""
  out_robot.append(ET.Comment(
      " Generated by convert_g1_urdf.py from newton-assets unitree_g1/urdf/" + URDF_NAME +
      ": fixed joints collapsed, collision meshes replaced by oriented bounding"
      " boxes" + decimation + ". "))
  decimate_jobs = []
  mesh_files = []  # (input, output) STL files of the visual meshes
  for link in links.values():
    out_link = ET.SubElement(out_robot, "link", name=link.name)
    if link.inertial is not None:
      inertial = ET.SubElement(out_link, "inertial")
      ET.SubElement(inertial, "origin", xyz=fmt(link.inertial.com), rpy="0 0 0")
      ET.SubElement(inertial, "mass", value="%.9g" % link.inertial.mass)
      i = link.inertial.inertia
      ET.SubElement(inertial, "inertia", ixx="%.9g" % i[0, 0], ixy="%.9g" % i[0, 1],
                    ixz="%.9g" % i[0, 2], iyy="%.9g" % i[1, 1], iyz="%.9g" % i[1, 2],
                    izz="%.9g" % i[2, 2])
    for tf, geom, material in link.visuals:
      visual = ET.SubElement(out_link, "visual")
      origin_element(visual, tf)
      geometry = ET.SubElement(visual, "geometry")
      if geom.tag == "mesh":
        in_path, out_path, filename = visual_mesh_files(asset_dir, geom, args.out)
        mesh_files.append((in_path, out_path))
        ET.SubElement(geometry, "mesh", filename=filename)
      else:
        geometry.append(geom)
      if material is not None:
        visual.append(material)
    for tf, geom in link.collisions:
      tf, geom = convert_collision(asset_dir, tf, geom)
      collision = ET.SubElement(out_link, "collision")
      origin_element(collision, tf)
      ET.SubElement(collision, "geometry").append(geom)

  for joint in joints:
    out_joint = ET.SubElement(out_robot, "joint", name=joint.name, type=joint.type)
    origin_element(out_joint, joint.origin)
    ET.SubElement(out_joint, "parent", link=joint.parent)
    ET.SubElement(out_joint, "child", link=joint.child)
    for tag in ("axis", "limit"):
      if joint.elem.find(tag) is not None:
        out_joint.append(joint.elem.find(tag))

  ET.indent(out_robot, space="  ")
  out_file = os.path.join(args.out, URDF_NAME)
  ET.ElementTree(out_robot).write(out_file, encoding="utf-8", xml_declaration=True)
  total_mass = sum(l.inertial.mass for l in links.values() if l.inertial is not None)
  sources, budgets = triangle_budgets(mesh_files, args.total_triangles, fixed,
                                      args.max_triangles)
  for in_path, out_path in mesh_files:
    if len(sources[in_path].faces) > budgets[out_path]:
      decimate_jobs.append([in_path, out_path, budgets[out_path]])
    else:
      sources[in_path].export(out_path, file_type="stl")
  if decimate_jobs:
    decimate_with_blender(args.blender, decimate_jobs)
  triangles_before = sum(stl_triangle_count(i) for i, _ in mesh_files)
  triangles_after = sum(stl_triangle_count(o) for _, o in mesh_files)
  print("Wrote %s: %d links, %d joints, total mass %.3f kg, visual triangles %d -> %d" %
        (out_file, len(links), len(joints), total_mass, triangles_before, triangles_after))


if __name__ == "__main__":
  main()
