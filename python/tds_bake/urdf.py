"""URDF helpers: finding URDF files, reading joint limits, and writing the URDF
of primitive shapes (sphere, box, capsule)."""

import os
import xml.etree.ElementTree as ET

# directories where relative URDF paths are searched, after the current
# directory and the given directories (such as the directory of the JSON file
# of a world): TDS_DATA_PATH (separated by os.pathsep) and the data directory
# of the repository
_HERE = os.path.dirname(os.path.realpath(__file__))
_REPO_DATA = os.path.normpath(os.path.join(_HERE, "..", "..", "data"))


def data_paths(directories=()):
  paths = [os.getcwd()]
  paths += [p for p in directories if p]
  paths += [p for p in os.environ.get("TDS_DATA_PATH", "").split(os.pathsep) if p]
  paths.append(_REPO_DATA)
  try:
    import pytinydiffsim_data  # installed with pip, if any
    paths.append(os.path.dirname(pytinydiffsim_data.__file__))
  except ImportError:
    pass
  unique = []
  for p in paths:
    p = os.path.abspath(p)
    if p not in unique:
      unique.append(p)
  return unique


def find_file(path, directories=()):
  """The absolute path of a data file such as "unitree_g1/g1.urdf", searched in
  data_paths(directories)."""
  if os.path.isabs(path):
    if os.path.exists(path):
      return path
    raise FileNotFoundError(path)
  paths = data_paths(directories)
  for root in paths:
    candidate = os.path.join(root, path)
    if os.path.exists(candidate):
      return os.path.abspath(candidate)
  raise FileNotFoundError(
      "%s not found in %s (set TDS_DATA_PATH to add directories)" %
      (path, ", ".join(paths)))


def _numbers(text, default):
  if text is None:
    return list(default)
  return [float(x) for x in text.split()]


def _shape(element, directory):
  """The geometry of a URDF visual or collision element as a dictionary:
  "type" (box, sphere, cylinder, capsule or mesh), its dimensions ("size",
  "radius", "length", "filename" and "scale"), and "xyz" and "rpy" of its
  origin in the link frame. None if it has no geometry."""
  geometry = element.find("geometry")
  if geometry is None or not len(geometry):
    return None
  g = geometry[0]
  origin = element.find("origin")
  shape = {
      "type": g.tag,
      "xyz": _numbers(origin.get("xyz") if origin is not None else None,
                      (0., 0., 0.)),
      "rpy": _numbers(origin.get("rpy") if origin is not None else None,
                      (0., 0., 0.)),
  }
  if g.tag == "box":
    shape["size"] = _numbers(g.get("size"), (1., 1., 1.))
  elif g.tag == "sphere":
    shape["radius"] = float(g.get("radius", 1.))
  elif g.tag in ("cylinder", "capsule"):
    shape["radius"] = float(g.get("radius", 1.))
    shape["length"] = float(g.get("length", 1.))
  elif g.tag == "mesh":
    filename = g.get("filename", "")
    if filename.startswith("package://"):
      filename = filename[len("package://"):]
      filename = filename.split("/", 1)[1] if "/" in filename else filename
    if not os.path.isabs(filename):
      filename = os.path.join(directory, filename)
    shape["filename"] = os.path.normpath(filename)
    shape["scale"] = _numbers(g.get("scale"), (1., 1., 1.))
  return shape


class UrdfInfo:
  """The joints and links of a URDF file."""

  def __init__(self, path):
    self.path = path
    directory = os.path.dirname(os.path.abspath(path))
    root = ET.parse(path).getroot()
    self.name = root.get("name", "")
    self.links = [link.get("name") for link in root.findall("link")]
    # the geometry type names of the collision shapes of each link
    self.collisions = {}
    # the collision and visual shapes of each link, see _shape
    self.collision_shapes = {}
    self.visual_shapes = {}
    for link in root.findall("link"):
      shapes = []
      for collision in link.findall("collision"):
        geometry = collision.find("geometry")
        if geometry is not None and len(geometry):
          shapes.append(geometry[0].tag)
      self.collisions[link.get("name")] = shapes
      for kind, out in (("collision", self.collision_shapes),
                        ("visual", self.visual_shapes)):
        out[link.get("name")] = [
            s for s in (_shape(e, directory) for e in link.findall(kind)) if s
        ]
    self.joints = {}
    children = set()
    for joint in root.findall("joint"):
      limit = joint.find("limit")
      lim = limit.attrib if limit is not None else {}

      def number(key, default=None):
        return float(lim[key]) if key in lim else default

      self.joints[joint.get("name")] = {
          "type": joint.get("type"),
          "parent": joint.find("parent").get("link"),
          "child": joint.find("child").get("link"),
          "lower": number("lower"),
          "upper": number("upper"),
          "effort": number("effort"),
          "velocity": number("velocity"),
      }
      children.add(joint.find("child").get("link"))
    roots = [l for l in self.links if l not in children]
    self.root_link = roots[0] if roots else (self.links[0] if self.links else "")

  def movable_joints(self):
    return [n for n, j in self.joints.items() if j["type"] not in ("fixed",)]


def _inertia(shape, mass):
  kind = shape["type"]
  if kind == "sphere":
    i = 0.4 * mass * shape["radius"]**2
    return i, i, i
  if kind == "box":
    x, y, z = shape["size"]
    return (mass / 12. * (y * y + z * z), mass / 12. * (x * x + z * z),
            mass / 12. * (x * x + y * y))
  if kind == "capsule":
    # a cylinder along z with hemispherical ends, approximated by a cylinder
    # of the total length
    r, length = shape["radius"], shape["length"] + 2 * shape["radius"]
    ixx = mass * (3 * r * r + length * length) / 12.
    return ixx, ixx, 0.5 * mass * r * r
  raise ValueError("unknown shape type %r" % kind)


def _geometry(shape):
  kind = shape["type"]
  if kind == "sphere":
    return '<sphere radius="%r"/>' % shape["radius"]
  if kind == "box":
    return '<box size="%r %r %r"/>' % tuple(shape["size"])
  if kind == "capsule":
    return '<capsule radius="%r" length="%r"/>' % (shape["radius"],
                                                   shape["length"])
  raise ValueError("unknown shape type %r" % kind)


def primitive_urdf(name, shape, mass, color=(0.8, 0.8, 0.8, 1.0)):
  """The URDF of a body with one link of a primitive shape, centered at its
  origin."""
  ixx, iyy, izz = _inertia(shape, mass if mass > 0 else 1.)
  geometry = _geometry(shape)
  return """<?xml version="1.0"?>
<!-- written by tds_bake -->
<robot name="{name}">
  <material name="{name}_color">
    <color rgba="{r!r} {g!r} {b!r} {a!r}"/>
  </material>
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{mass!r}"/>
      <inertia ixx="{ixx!r}" ixy="0" ixz="0" iyy="{iyy!r}" iyz="0" izz="{izz!r}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>{geometry}</geometry>
      <material name="{name}_color"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>{geometry}</geometry>
    </collision>
  </link>
</robot>
""".format(name=name,
           r=float(color[0]),
           g=float(color[1]),
           b=float(color[2]),
           a=float(color[3]) if len(color) > 3 else 1.0,
           mass=float(mass if mass > 0 else 1.),
           ixx=ixx,
           iyy=iyy,
           izz=izz,
           geometry=geometry)
