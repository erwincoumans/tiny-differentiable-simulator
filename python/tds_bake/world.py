"""Description of a world of robots and objects, built in Python or read from
JSON, that tds_bake.bake turns into batched CUDA and CPU kernels.

  world = tds_bake.World("g1_stand", dt=1 / 360, substeps=6)
  world.add_ground()
  world.add_urdf("g1", "unitree_g1/g1_29dof_with_hand_rev_1_0.urdf",
                 position=(0, 0, 0.8), kp=1000, kd=10, armature=0.01,
                 collision_links=["left_ankle_roll_link",
                                  "right_ankle_roll_link"])
  world.add_sphere("ball", radius=0.1, mass=0.5, position=(0.5, 0, 0.1))
  world.save("g1_stand.json")

The JSON file is the dictionary of World.to_dict(), see SCHEMA_DOC.
"""

import copy
import json
import math
import os

from . import urdf as urdf_utils

FORMAT = "tds_world"
VERSION = 1

SCHEMA_DOC = """\
A tds_bake world (JSON):
{
  "format": "tds_world", "version": 1,
  "name": "g1_walk",                  identifier of the world
  "dt": 0.005,                        time step of one kernel step (seconds)
  "substeps": 4,                      kernel steps per Batch.step()
  "up_axis": "z",                     "z" or "y": the normal of the ground,
                                      the default gravity and the viewer.
                                      URDF files are z-up, for y-up rotate
                                      them with "orientation":
                                      [-0.7071068, 0, 0, 0.7071068]
  "solver": "tds",                    "tds": the kernels solve the contacts
                                      below; "external": another engine
                                      finds and solves all contacts of the
                                      links; the kernels then only compute
                                      the dynamics (predict, articulation,
                                      advance), and the ground, "contact" and
                                      the collision filters ("with") are not
                                      used
  "gravity": [0, 0, -9.81],           default: -9.81 along the up axis
  "contact": {
    "friction": 0.75,                 default friction coefficient
    "restitution": 0.0,
    "iterations": 20,                 contact solver (PGS) iterations
    "erp": 0.2, "cfm": 1e-5,          penetration correction, regularization
    "split": true                     separate solver kernel (faster to bake)
  },
  "ground": {"height": 0.0, "friction": null},   or null for no ground,
                                      height along the up axis
  "bodies": [                        robots (URDF) and objects (shapes)
    {
      "name": "g1",
      "urdf": "unitree_g1/g1_29dof_with_hand_rev_1_0.urdf",
                                      absolute, relative to the current
                                      directory, the directory of the JSON
                                      file or its data subdirectory,
                                      TDS_DATA_PATH or tds data
      "floating": true,               free base (false: fixed to the world)
      "position": [0, 0, 0.8], "orientation": [0, 0, 0, 1],   (x, y, z, w)
      "joint_positions": {"left_knee_joint": 0.3},   initial joint angles
      "control": {
        "mode": "pd",                 "pd": targets, kp, kd are inputs
                                      "torque": joint torques are inputs
                                      "none": passive joints
        "kp": 100, "kd": 2,           default gains of all joints
        "gains": {"left_ankle.*": [40, 2]},   per joint (regex) [kp, kd]
        "armature": 0.01,             rotor inertia of the joints
        "implicit": true,             implicit PD (stable with stiff gains)
        "effort_limits": "urdf"       null, "urdf" or {"joint": max torque}
      },
      "collision": {
        "links": "all",               "all", "none" or a list of link names
                                      (regex) whose URDF collision shapes
                                      collide
        "spheres": [{"link": "left_hand_palm_link", "radius": 0.02,
                     "position": [0, 0, 0], "with": ["ball"]}],
                                      additional contact spheres, "with" is
                                      optional (default: the body's)
        "with": "all"                 "all" or names of bodies ("ground")
      },
      "friction": null                friction of the body (default: world)
    },
    {
      "name": "ball",
      "shape": {"type": "sphere", "radius": 0.1},
                                      or {"type": "box", "size": [x, y, z]},
                                      {"type": "capsule", "radius": r,
                                       "length": l} (along z)
      "mass": 0.5, "color": [1, 0, 0, 1],
      "floating": true, "position": [0.5, 0, 0.1]
    }
  ]
}
Supported contacts: sphere-sphere, sphere-plane, capsule-plane, box-plane,
sphere-box, capsule-sphere. Other pairs are ignored (bake warns about them). The ground is
a plane. Collision meshes are not supported, use contact spheres instead.
"""

_DEFAULT_CONTACT = {
    "friction": 0.75,
    "restitution": 0.0,
    "iterations": 20,
    "erp": 0.2,
    "cfm": 1e-5,
    "split": True,
}

_SHAPES = ("sphere", "box", "capsule")

# the index of each up axis
UP_AXES = {"y": 1, "z": 2}

# who solves the contacts, see SCHEMA_DOC
SOLVERS = ("tds", "external")


def _vector(value, size, what):
  try:
    v = [float(x) for x in value]
  except TypeError:
    raise ValueError("%s must be a list of %d numbers" % (what, size))
  if len(v) != size:
    raise ValueError("%s must have %d numbers, not %d" % (what, size, len(v)))
  return v


def _normalized_quaternion(q, what):
  q = _vector(q, 4, what)
  n = math.sqrt(sum(x * x for x in q))
  if n < 1e-9:
    raise ValueError("%s must be a unit quaternion (x, y, z, w)" % what)
  return [x / n for x in q]


class World:
  """A world of robots (URDF files) and objects (primitive shapes) on an
  optional ground plane. Build it with the add_* methods or from a dictionary
  or JSON file (World.load), then bake it with tds_bake.bake. up_axis ("z" or
  "y") is the normal of the ground, and gravity is -9.81 along it unless
  given."""

  def __init__(self,
               name="world",
               dt=1. / 240.,
               substeps=4,
               gravity=None,
               friction=0.75,
               restitution=0.,
               iterations=20,
               erp=0.2,
               cfm=1e-5,
               split_solver=True,
               ground=True,
               up_axis="z",
               solver="tds"):
    if up_axis not in UP_AXES:
      raise ValueError('up_axis must be "z" or "y", not %r' % (up_axis,))
    if solver not in SOLVERS:
      raise ValueError('solver must be "tds" or "external", not %r' %
                       (solver,))
    self.name = name
    self.solver = solver
    self.dt = float(dt)
    self.substeps = int(substeps)
    self.up_axis = up_axis
    if gravity is None:
      gravity = [0., 0., 0.]
      gravity[UP_AXES[up_axis]] = -9.81
    self.gravity = _vector(gravity, 3, "gravity")
    self.contact = dict(friction=float(friction),
                        restitution=float(restitution),
                        iterations=int(iterations),
                        erp=float(erp),
                        cfm=float(cfm),
                        split=bool(split_solver))
    self.ground = {"height": 0.0, "friction": None} if ground else None
    self.bodies = []
    # the directory of the JSON file of the world (World.load), where relative
    # URDF paths are also searched, and in its data subdirectory
    self.directory = None

  # building

  def add_ground(self, height=0., friction=None):
    """Adds (or replaces) the ground plane at the given height along the up
    axis."""
    self.ground = {"height": float(height), "friction": friction}
    return self

  def remove_ground(self):
    self.ground = None
    return self

  def _check_name(self, name):
    if not isinstance(name, str) or not name:
      raise ValueError("a body needs a name")
    if name == "ground":
      raise ValueError('"ground" is the name of the ground plane')
    if any(b["name"] == name for b in self.bodies):
      raise ValueError("there is already a body named %r" % name)

  def add_urdf(self,
               name,
               urdf,
               floating=True,
               position=(0., 0., 0.),
               orientation=(0., 0., 0., 1.),
               joint_positions=None,
               control="pd",
               kp=0.,
               kd=0.,
               gains=None,
               armature=0.,
               implicit_pd=True,
               effort_limits=None,
               collision_links="all",
               collision_spheres=None,
               collide_with="all",
               friction=None):
    """Adds a robot from a URDF file.

    control: "pd" (inputs: joint targets, kp and kd of each movable joint),
      "torque" (inputs: joint torques) or "none".
    gains: {joint name regex: (kp, kd)} overrides kp and kd per joint.
    effort_limits: None, "urdf" (the effort limits of the URDF joints) or
      {joint name: max torque}.
    collision_links: "all", "none" or a list of link names (regex) whose URDF
      collision shapes collide.
    collision_spheres: a list of {"link": name, "radius": r,
      "position": (x, y, z)} contact spheres added to links, optionally with
      "with": [body names] that they collide with instead of collide_with.
    collide_with: "all" or a list of body names ("ground" for the ground).
    """
    self._check_name(name)
    body = {
        "name": name,
        "urdf": urdf,
        "floating": bool(floating),
        "position": _vector(position, 3, "position"),
        "orientation": _normalized_quaternion(orientation, "orientation"),
        "joint_positions": dict(joint_positions or {}),
        "control": {
            "mode": control,
            "kp": float(kp),
            "kd": float(kd),
            "gains": {k: [float(v[0]), float(v[1])]
                      for k, v in (gains or {}).items()},
            "armature": float(armature),
            "implicit": bool(implicit_pd),
            "effort_limits": effort_limits,
        },
        "collision": {
            "links": collision_links,
            "spheres": list(collision_spheres or []),
            "with": collide_with,
        },
        "friction": friction,
    }
    self.bodies.append(body)
    self.validate(references=False)
    return body

  def _add_shape(self, name, shape, mass, position, orientation, floating,
                 color, collide_with, friction):
    self._check_name(name)
    body = {
        "name": name,
        "shape": shape,
        "mass": float(mass),
        "color": _vector(color, 4, "color"),
        "floating": bool(floating),
        "position": _vector(position, 3, "position"),
        "orientation": _normalized_quaternion(orientation, "orientation"),
        "collision": {
            "links": "all",
            "with": collide_with
        },
        "friction": friction,
    }
    self.bodies.append(body)
    self.validate(references=False)
    return body

  def add_sphere(self,
                 name,
                 radius,
                 mass=1.,
                 position=(0., 0., 0.),
                 orientation=(0., 0., 0., 1.),
                 floating=True,
                 color=(0.8, 0.3, 0.2, 1.),
                 collide_with="all",
                 friction=None):
    return self._add_shape(name, {
        "type": "sphere",
        "radius": float(radius)
    }, mass, position, orientation, floating, color, collide_with, friction)

  def add_box(self,
              name,
              size,
              mass=1.,
              position=(0., 0., 0.),
              orientation=(0., 0., 0., 1.),
              floating=True,
              color=(0.3, 0.5, 0.8, 1.),
              collide_with="all",
              friction=None):
    """A box of the given full size (x, y, z). Boxes collide with planes (the
    ground) and spheres."""
    return self._add_shape(name, {
        "type": "box",
        "size": _vector(size, 3, "size")
    }, mass, position, orientation, floating, color, collide_with, friction)

  def add_capsule(self,
                  name,
                  radius,
                  length,
                  mass=1.,
                  position=(0., 0., 0.),
                  orientation=(0., 0., 0., 1.),
                  floating=True,
                  color=(0.3, 0.8, 0.4, 1.),
                  collide_with="all",
                  friction=None):
    """A capsule along z, length is the distance of the sphere centers."""
    return self._add_shape(name, {
        "type": "capsule",
        "radius": float(radius),
        "length": float(length)
    }, mass, position, orientation, floating, color, collide_with, friction)

  def body(self, name):
    for b in self.bodies:
      if b["name"] == name:
        return b
    raise KeyError("no body named %r" % name)

  # serialization

  def to_dict(self):
    return {
        "format": FORMAT,
        "version": VERSION,
        "name": self.name,
        "dt": self.dt,
        "substeps": self.substeps,
        "up_axis": self.up_axis,
        "solver": self.solver,
        "gravity": list(self.gravity),
        "contact": dict(self.contact),
        "ground": copy.deepcopy(self.ground),
        "bodies": copy.deepcopy(self.bodies),
    }

  def to_json(self, indent=2):
    return json.dumps(self.to_dict(), indent=indent)

  def save(self, path):
    with open(path, "w") as f:
      f.write(self.to_json())
    return path

  @classmethod
  def from_dict(cls, d):
    if not isinstance(d, dict):
      raise ValueError("a world is a dictionary (JSON object)")
    fmt = d.get("format", FORMAT)
    if fmt != FORMAT:
      raise ValueError("format must be %r, not %r" % (FORMAT, fmt))
    if d.get("version", VERSION) > VERSION:
      raise ValueError("world version %r is newer than this tds_bake (%d)" %
                       (d.get("version"), VERSION))
    unknown = set(d) - {
        "format", "version", "name", "dt", "substeps", "up_axis", "solver",
        "gravity", "contact", "ground", "bodies", "description"
    }
    if unknown:
      raise ValueError("unknown keys in the world: %s" %
                       ", ".join(sorted(unknown)))
    contact = dict(_DEFAULT_CONTACT)
    unknown = set(d.get("contact", {})) - set(contact)
    if unknown:
      raise ValueError("unknown keys in contact: %s" %
                       ", ".join(sorted(unknown)))
    contact.update(d.get("contact", {}))
    world = cls(name=d.get("name", "world"),
                dt=d.get("dt", 1. / 240.),
                substeps=d.get("substeps", 4),
                gravity=d.get("gravity"),
                friction=contact["friction"],
                restitution=contact["restitution"],
                iterations=contact["iterations"],
                erp=contact["erp"],
                cfm=contact["cfm"],
                split_solver=contact["split"],
                ground="ground" not in d or d["ground"] is not None,
                up_axis=d.get("up_axis", "z"),
                solver=d.get("solver", "tds"))
    if d.get("ground") is not None:
      g = d["ground"]
      if g is True:
        g = {}
      world.add_ground(g.get("height", 0.), g.get("friction"))
    world.bodies = copy.deepcopy(d.get("bodies", []))
    world.validate()
    return world

  @classmethod
  def from_json(cls, text):
    return cls.from_dict(json.loads(text))

  @classmethod
  def load(cls, path):
    with open(path) as f:
      world = cls.from_json(f.read())
    world.directory = os.path.dirname(os.path.abspath(path))
    return world

  # checks

  def validate(self, references=True):
    """Raises ValueError with the location of the first problem. references
    checks that the bodies in collision "with" lists exist."""
    if not isinstance(self.name, str) or not self.name:
      raise ValueError("name must be a non-empty string")
    if not self.dt > 0:
      raise ValueError("dt must be positive")
    if self.up_axis not in UP_AXES:
      raise ValueError('up_axis must be "z" or "y", not %r' % (self.up_axis,))
    if self.solver not in SOLVERS:
      raise ValueError('solver must be "tds" or "external", not %r' %
                       (self.solver,))
    if self.substeps < 1:
      raise ValueError("substeps must be at least 1")
    if self.contact["iterations"] < 0:
      raise ValueError("contact.iterations must not be negative")
    names = set()
    for i, b in enumerate(self.bodies):
      where = "bodies[%d]" % i
      if not isinstance(b, dict):
        raise ValueError("%s must be an object" % where)
      name = b.get("name")
      if not isinstance(name, str) or not name or name == "ground":
        raise ValueError("%s.name must be a string other than 'ground'" %
                         where)
      where = "body %r" % name
      if name in names:
        raise ValueError("%s: two bodies have this name" % where)
      names.add(name)
      allowed = {
          "name", "urdf", "shape", "mass", "color", "floating", "position",
          "orientation", "joint_positions", "control", "collision", "friction"
      }
      unknown = set(b) - allowed
      if unknown:
        raise ValueError("%s: unknown keys %s" %
                         (where, ", ".join(sorted(unknown))))
      if ("urdf" in b) == ("shape" in b):
        raise ValueError("%s needs either \"urdf\" or \"shape\"" % where)
      _vector(b.get("position", (0, 0, 0)), 3, where + ".position")
      _normalized_quaternion(b.get("orientation", (0, 0, 0, 1)),
                             where + ".orientation")
      if "shape" in b:
        shape = b["shape"]
        if not isinstance(shape, dict) or shape.get("type") not in _SHAPES:
          raise ValueError("%s.shape.type must be one of %s" %
                           (where, ", ".join(_SHAPES)))
        need = {"sphere": ["radius"], "box": ["size"],
                "capsule": ["radius", "length"]}[shape["type"]]
        for key in need:
          if key not in shape:
            raise ValueError("%s.shape needs %r" % (where, key))
        if b.get("mass", 1.) < 0:
          raise ValueError("%s.mass must not be negative" % where)
      control = b.get("control", {})
      mode = control.get("mode", "pd" if "urdf" in b else "none")
      if mode not in ("pd", "torque", "none"):
        raise ValueError("%s.control.mode must be pd, torque or none" % where)
      limits = control.get("effort_limits")
      if limits not in (None, "urdf") and not isinstance(limits, dict):
        raise ValueError('%s.control.effort_limits must be null, "urdf" or '
                         'a {joint: torque} object' % where)
      collision = b.get("collision", {})
      links = collision.get("links", "all")
      if not (links in ("all", "none") or isinstance(links, list)):
        raise ValueError('%s.collision.links must be "all", "none" or a list'
                         % where)
      for s in collision.get("spheres", []):
        if "radius" not in s:
          raise ValueError("%s.collision.spheres need a radius" % where)
        if not (s.get("with", "all") == "all" or
                isinstance(s.get("with"), list)):
          raise ValueError('%s.collision.spheres "with" must be "all" or a '
                           'list of body names' % where)
      with_ = collision.get("with", "all")
      if not (with_ == "all" or isinstance(with_, list)):
        raise ValueError('%s.collision.with must be "all" or a list of body '
                         'names' % where)
    for b in self.bodies if references else []:
      collision = b.get("collision", {})
      lists = [collision.get("with", "all")]
      lists += [s.get("with", "all") for s in collision.get("spheres", [])]
      for with_ in lists:
        if not isinstance(with_, list):
          continue
        for other in with_:
          if other != "ground" and other not in names:
            raise ValueError("body %r collides with unknown body %r" %
                             (b["name"], other))
    return self

  # resolving into the description of the C++ world model

  def resolve(self, body_directory):
    """The description of the world for the kernel generator
    (src/bake/world_spec.hpp): URDF paths are absolute, the URDF files of the
    shapes are written to body_directory, and effort limits are explicit.
    Returns (spec, urdf_files), urdf_files maps each body name to its URDF
    file."""
    self.validate()
    spec = {
        "name": self.name,
        "dt": self.dt,
        "substeps": self.substeps,
        "up_axis": self.up_axis,
        "gravity": list(self.gravity),
        "contact": dict(self.contact),
        "ground": copy.deepcopy(self.ground),
        "bodies": [],
    }
    if self.solver == "external":
      # the other engine has the ground and the contacts (only written when
      # not the default, so that the hash of other worlds stays the same)
      spec["solver"] = "external"
      spec["ground"] = None
    if spec["ground"] is not None and spec["ground"].get("friction") is None:
      spec["ground"].pop("friction", None)
    urdf_files = {}
    for b in self.bodies:
      out = {
          "name": b["name"],
          "floating": b.get("floating", True),
          "position": _vector(b.get("position", (0, 0, 0)), 3, "position"),
          "orientation": _normalized_quaternion(b.get("orientation",
                                                      (0, 0, 0, 1)),
                                                "orientation"),
      }
      if b.get("friction") is not None:
        out["friction"] = float(b["friction"])
      if "shape" in b:
        os.makedirs(body_directory, exist_ok=True)
        path = os.path.join(body_directory, b["name"] + ".urdf")
        text = urdf_utils.primitive_urdf(b["name"], b["shape"],
                                         b.get("mass", 1.),
                                         b.get("color", (0.8, 0.8, 0.8, 1.)))
        with open(path, "w") as f:
          f.write(text)
        out["urdf"] = os.path.abspath(path)
        out["control"] = {"mode": "none"}
        info = urdf_utils.UrdfInfo(path)
      else:
        directories = []
        if self.directory is not None:
          directories = [self.directory, os.path.join(self.directory, "data")]
        out["urdf"] = urdf_utils.find_file(b["urdf"], directories)
        info = urdf_utils.UrdfInfo(out["urdf"])
        control = dict(b.get("control", {}))
        control.setdefault("mode", "pd")
        limits = control.get("effort_limits")
        if limits == "urdf":
          limits = {
              name: j["effort"]
              for name, j in info.joints.items()
              if j["effort"] is not None and j["effort"] > 0
          }
        if limits:
          control["effort_limits"] = {k: float(v) for k, v in limits.items()}
        else:
          control.pop("effort_limits", None)
        out["control"] = control
        joint_positions = dict(b.get("joint_positions", {}))
        for joint in joint_positions:
          if joint not in info.joints:
            raise ValueError("body %r: the URDF has no joint %r" %
                             (b["name"], joint))
        out["joint_positions"] = joint_positions
      collision = dict(b.get("collision", {}))
      collision.setdefault("links", "all")
      collision.setdefault("with", "all")
      out["collision"] = collision
      urdf_files[b["name"]] = out["urdf"]
      spec["bodies"].append(out)
    return spec, urdf_files
