#!/usr/bin/env python3
"""Converts the AgileX Piper arm with gripper from piper_ros (piper_description)
into a URDF that Tiny Differentiable Simulator and tds_bake can load:

  git clone --depth 1 -b noetic https://github.com/agilexrobotics/piper_ros
  python convert_piper_urdf.py piper_ros/src/piper_description

  * fixed joints are collapsed (the gripper base becomes part of link6, the
    dummy root link disappears): mass properties and visuals are merged into
    the parent link, the joints keep their names (joint1 .. joint6 revolute,
    joint7 and joint8 prismatic, the two fingers),
  * the collision meshes are dropped, TDS cannot collide meshes. The pick
    example (tds_bake.examples.piper_pick) adds contact spheres on the fingers,
  * inertia tensors are written with respect to the center of mass, in link
    frame orientation, as in convert_g1_urdf.py, which provides the URDF
    handling used here,
  * visual meshes are written as binary STL files, decimated to a total of
    --total-triangles (default 20k, from 184k) with quadric edge collapses
    (fast_simplification, no Blender needed): the triangles are shared in
    proportion to the square root of the surface area of the meshes, and the
    small ones keep at least --min-triangles. The vertices of the STL files
    are welded first, so that the meshes are closed and the collapses see their
    real edges. The converter prints the distance of the decimated surfaces to
    the original ones.

Usage:
  python convert_piper_urdf.py path/to/piper_description [--out DIR]
      [--total-triangles N] [--min-triangles N] [--agg A]

Requires numpy, scipy, trimesh and fast_simplification (pip install
fast-simplification).
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "unitree_g1"))
import convert_g1_urdf as g1  # noqa: E402

URDF_NAME = "piper_description.urdf"
SOURCE_URDF = "urdf/piper_description.urdf"


def welded(path):
  """The mesh of an STL file with merged vertices and without degenerate
  triangles."""
  mesh = trimesh.load(path, force="mesh", process=False)
  mesh.merge_vertices(merge_tex=True, merge_norm=True)
  mesh.update_faces(mesh.nondegenerate_faces())
  mesh.remove_unreferenced_vertices()
  return mesh


def decimated(mesh, target, agg):
  """The mesh with about target triangles, by quadric edge collapses. The
  collapses stop where one more would flip a triangle, so the result may keep
  more triangles than asked for."""
  import fast_simplification
  if len(mesh.faces) <= target:
    return mesh
  points, faces = fast_simplification.simplify(mesh.vertices, mesh.faces.astype(np.int64),
                                               target_count=target, agg=agg)
  out = trimesh.Trimesh(points, faces, process=False)
  out.update_faces(out.nondegenerate_faces())
  out.remove_unreferenced_vertices()
  return out


def surface_distances(original, simplified, samples=200000):
  """Distances from points on the decimated surface to the original surface
  (and the other way around): mean and maximum of both directions."""
  from scipy.spatial import cKDTree

  def one_way(a, b):
    points_a, _ = trimesh.sample.sample_surface(a, samples, seed=0)
    points_b, _ = trimesh.sample.sample_surface(b, samples, seed=1)
    # the closest sample of b is slightly more than the distance to its
    # surface, a sample spacing, which is small next to the errors of interest
    d, _ = cKDTree(points_b).query(points_a)
    return d

  d = np.concatenate([one_way(simplified, original), one_way(original, simplified)])
  return float(d.mean()), float(d.max())


def allocation(meshes, total, minimum):
  """Triangles per mesh, in proportion to the square root of the area, at
  least `minimum` and at most what the mesh has."""
  names = list(meshes)
  weights = np.array([np.sqrt(meshes[n].area) for n in names])
  budget = {n: len(meshes[n].faces) for n in names}
  free = set(names)
  remaining = total
  # meshes that have less than their share keep all triangles, the others
  # share the rest
  while free:
    share = {n: max(minimum, remaining * weights[names.index(n)] /
                    sum(weights[names.index(m)] for m in free)) for n in free}
    small = [n for n in free if len(meshes[n].faces) <= share[n]]
    if not small:
      for n in free:
        budget[n] = int(round(share[n]))
      break
    for n in small:
      remaining -= len(meshes[n].faces)
      free.discard(n)
  return budget


def main():
  parser = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("asset_dir", help="piper_description folder of piper_ros")
  parser.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
  parser.add_argument("--total-triangles", type=int, default=20000,
                      help="triangles of all visual meshes, 0 keeps all")
  parser.add_argument("--min-triangles", type=int, default=200,
                      help="triangles that a mesh keeps at least")
  parser.add_argument("--agg", type=float, default=3.,
                      help="aggressiveness of the decimation, 0 (careful, slow) to 10")
  args = parser.parse_args()
  source = os.path.join(args.asset_dir, SOURCE_URDF)
  print("Converting", source)

  robot = ET.parse(source).getroot()
  links = {l.get("name"): g1.Link(l) for l in robot.findall("link")}
  joints = g1.collapse_fixed_joints(links, [g1.Joint(j) for j in robot.findall("joint")])
  # the URDF has a dummy root link with a fixed joint to base_link, which
  # collapsing the joint made the base_link
  roots = [l for l in links if l not in {j.child for j in joints}]
  assert len(roots) == 1, roots
  if roots[0] == "dummy_link":
    root = links.pop("dummy_link")
    root.name = "base_link"
    links = {"base_link": root, **links}
    for j in joints:
      if j.parent == "dummy_link":
        j.parent = "base_link"

  os.makedirs(os.path.join(args.out, "meshes"), exist_ok=True)
  out_robot = ET.Element("robot", name=robot.get("name"))
  out_robot.append(ET.Comment(
      " Generated by convert_piper_urdf.py from piper_ros piper_description/" + SOURCE_URDF +
      ": fixed joints collapsed, collision meshes removed, visual meshes decimated"
      " to %d triangles in total. " % args.total_triangles))
  mesh_files = []  # (input STL, output STL, URDF filename)
  for link in links.values():
    out_link = ET.SubElement(out_robot, "link", name=link.name)
    if link.inertial is not None:
      inertial = ET.SubElement(out_link, "inertial")
      ET.SubElement(inertial, "origin", xyz=g1.fmt(link.inertial.com), rpy="0 0 0")
      ET.SubElement(inertial, "mass", value="%.9g" % link.inertial.mass)
      i = link.inertial.inertia
      ET.SubElement(inertial, "inertia", ixx="%.9g" % i[0, 0], ixy="%.9g" % i[0, 1],
                    ixz="%.9g" % i[0, 2], iyy="%.9g" % i[1, 1], iyz="%.9g" % i[1, 2],
                    izz="%.9g" % i[2, 2])
    for tf, geom, material in link.visuals:
      visual = ET.SubElement(out_link, "visual")
      g1.origin_element(visual, tf)
      geometry = ET.SubElement(visual, "geometry")
      if geom.tag == "mesh":
        in_path, out_path, filename = g1.visual_mesh_files(args.asset_dir, geom, args.out)
        mesh_files.append((in_path, out_path))
        ET.SubElement(geometry, "mesh", filename=filename)
      else:
        geometry.append(geom)
      if material is not None:
        visual.append(material)

  for joint in joints:
    out_joint = ET.SubElement(out_robot, "joint", name=joint.name, type=joint.type)
    g1.origin_element(out_joint, joint.origin)
    ET.SubElement(out_joint, "parent", link=joint.parent)
    ET.SubElement(out_joint, "child", link=joint.child)
    for tag in ("axis", "limit"):
      if joint.elem.find(tag) is not None:
        out_joint.append(joint.elem.find(tag))

  ET.indent(out_robot, space="  ")
  out_file = os.path.join(args.out, URDF_NAME)
  ET.ElementTree(out_robot).write(out_file, encoding="utf-8", xml_declaration=True)

  sources = {i: welded(i) for i, _ in mesh_files}
  if args.total_triangles > 0:
    budgets = allocation(sources, args.total_triangles, args.min_triangles)
  else:
    budgets = {i: len(m.faces) for i, m in sources.items()}
  before = after = 0
  for in_path, out_path in mesh_files:
    original = sources[in_path]
    mesh = decimated(original, budgets[in_path], args.agg)
    mesh.export(out_path, file_type="stl")
    mean, worst = surface_distances(original, mesh)
    size = float(np.linalg.norm(original.extents))
    print("  %-14s %6d -> %5d triangles, surface distance mean %.3f mm, max %.3f mm "
          "(%.2f%% of the diagonal)" % (os.path.basename(out_path), len(original.faces),
                                        len(mesh.faces), 1e3 * mean, 1e3 * worst,
                                        100 * worst / size))
    before += g1.stl_triangle_count(in_path)
    after += len(mesh.faces)
  total_mass = sum(l.inertial.mass for l in links.values() if l.inertial is not None)
  print("Wrote %s: %d links, %d joints, total mass %.3f kg, visual triangles %d -> %d" %
        (out_file, len(links), len(joints), total_mass, before, after))


if __name__ == "__main__":
  main()
