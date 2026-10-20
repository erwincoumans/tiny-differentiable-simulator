"""A baked world (tds_bake.bake) and batches of its worlds."""

import json
import os

import numpy as np

from . import baking
from . import urdf as urdf_utils


class BodyLayout:
  """Where a body is in the arrays of a batch: its slices of the state and
  control, its joints, actuators, links, visual shapes and contacts."""

  def __init__(self, layout, contacts):
    self._layout = layout
    self.name = layout["name"]
    self.floating = layout["floating"]
    q, nq = layout["q_offset"], layout["q_dim"]
    qd, nqd = layout["qd_offset"], layout["qd_dim"]
    # slices of the state
    self.q = slice(q, q + nq)
    self.qd = slice(qd, qd + nqd)
    if self.floating:
      self.base_orientation = slice(q, q + 4)  # quaternion x, y, z, w
      self.base_position = slice(q + 4, q + 7)
      self.base_angular_velocity = slice(qd, qd + 3)  # world frame
      self.base_linear_velocity = slice(qd + 3, qd + 6)  # world frame
    self.joints = layout["joints"]
    self.joint_names = [j["name"] for j in self.joints]
    # state indices of the joint positions and velocities, in joint order
    self.joint_q = np.array([j["q"] for j in self.joints], dtype=np.int64)
    self.joint_qd = np.array([j["qd"] for j in self.joints], dtype=np.int64)
    # actuators: one per movable joint for "pd" and "torque" control
    self.control_mode = layout["control_mode"]
    self.actuators = layout["actuators"]
    self.actuator_names = [a["joint"] for a in self.actuators]
    n = len(self.actuators)
    c = layout["control_offset"]
    self.num_actuators = n
    if self.control_mode == "pd":
      self.targets = slice(c, c + n)
      self.kp = slice(c + n, c + 2 * n)
      self.kd = slice(c + 2 * n, c + 3 * n)
    elif self.control_mode == "torque":
      self.torques = slice(c, c + n)
    joint_of = {j["name"]: j for j in self.joints}
    self.actuator_q = np.array([joint_of[a]["q"] for a in self.actuator_names],
                               dtype=np.int64)
    self.actuator_qd = np.array(
        [joint_of[a]["qd"] for a in self.actuator_names], dtype=np.int64)
    self.links = layout["links"]  # the base link first
    self.link_offset = layout["link_offset"]
    self.visual_offset = layout["visual_offset"]
    self.num_visuals = layout["num_visuals"]
    # contact points of the body: (first, count, other body, link, other link)
    self.contacts = []
    for c in contacts:
      if c["body_a"] == self.name:
        self.contacts.append((c["first"], c["count"], c["body_b"], c["link_a"],
                              c["link_b"]))
      elif c["body_b"] == self.name:
        self.contacts.append((c["first"], c["count"], c["body_a"], c["link_b"],
                              c["link_a"]))

  def link_index(self, name):
    """Index of a link in the links array of a batch (Batch.links())."""
    return self.link_offset + self.links.index(name)

  def contact_indices(self, link=None, other=None):
    """Indices of the contact points of a link (or of all links) of the body
    with another body (or all bodies), see Batch.contact_forces()."""
    indices = []
    for first, count, other_body, own_link, _ in self.contacts:
      if (link is None or own_link == link) and (other is None or
                                                 other_body == other):
        indices += list(range(first, first + count))
    return np.array(indices, dtype=np.int64)

  def __repr__(self):
    return "BodyLayout(%r, %d joints, %d actuators, %d links)" % (
        self.name, len(self.joints), self.num_actuators, len(self.links))


class BakedWorld:
  """The kernels of a world, loaded from its directory in the cache.

  batch(num_worlds, backend, precision) creates a Batch that simulates many
  worlds at once. The kernels for a backend and precision are compiled when
  they are first needed (or by tds_bake.bake)."""

  def __init__(self, directory, device=0, verbose=True, jobs=None):
    self.directory = os.path.abspath(directory)
    self.device = device
    self.verbose = verbose
    self.jobs = jobs
    with open(os.path.join(self.directory, "kernels.json")) as f:
      self.manifest = json.load(f)
    with open(os.path.join(self.directory, "resolved.json")) as f:
      self.spec = json.load(f)
    world_json = os.path.join(self.directory, "world.json")
    if os.path.exists(world_json):
      with open(world_json) as f:
        self.world = json.load(f)
    else:
      self.world = None
    self.layout = self.manifest["metadata"]
    self.name = self.spec["name"]
    self.up_axis = self.spec.get("up_axis", "z")
    self.dt = self.layout["dt"]
    self.substeps = self.layout["substeps"]
    self.iterations = self.layout["iterations"]
    self.split = self.layout["split"]
    self.state_dim = self.layout["state_dim"]
    self.control_dim = self.layout["control_dim"]
    self.num_contacts = self.layout["num_contacts"]
    self.num_visuals = self.layout["num_visuals"]
    self.num_links = self.layout["num_links"]
    self.bodies = {
        b["name"]: BodyLayout(b, self.layout["contacts"])
        for b in self.layout["bodies"]
    }
    self.urdf_files = {b["name"]: b["urdf"] for b in self.spec["bodies"]}
    self._urdf_info = {}
    self._compiled = {"cpu": {}, "cuda": {}}
    self._model = None

  def __repr__(self):
    return "BakedWorld(%r, %d bodies, state %d, control %d, %d contacts)" % (
        self.name, len(self.bodies), self.state_dim, self.control_dim,
        self.num_contacts)

  def body(self, name):
    return self.bodies[name]

  def urdf_info(self, body):
    """The joints (with limits) and links of the URDF of a body."""
    if body not in self._urdf_info:
      self._urdf_info[body] = urdf_utils.UrdfInfo(self.urdf_files[body])
    return self._urdf_info[body]

  def joint_limits(self, body):
    """Lower and upper limits of the actuated joints of a body, from its URDF
    (-inf, inf if there is none)."""
    info = self.urdf_info(body)
    lower, upper = [], []
    for name in self.bodies[body].actuator_names:
      j = info.joints[name]
      has = j["lower"] is not None and j["upper"] is not None and (
          j["upper"] > j["lower"])
      lower.append(j["lower"] if has else -np.inf)
      upper.append(j["upper"] if has else np.inf)
    return np.array(lower), np.array(upper)

  def default_state(self):
    return np.array(self.layout["default_state"])

  def default_control(self):
    return np.array(self.layout["default_control"])

  def describe(self):
    """A text description of the world: bodies, joints, actuators, contacts
    and kernels."""
    lines = ["world %r: dt %g s, %d substeps, %d contact points%s" %
             (self.name, self.dt, self.substeps, self.num_contacts,
              (", %d solver iterations" % self.iterations)
              if self.num_contacts else "")]
    lines.append("  state %d, control %d, %d visual shapes, %d links" %
                 (self.state_dim, self.control_dim, self.num_visuals,
                  self.num_links))
    for b in self.bodies.values():
      lines.append("  body %r: %s base, %d joints, control %s (%d actuators), "
                   "state q[%d:%d] qd[%d:%d]" %
                   (b.name, "floating" if b.floating else "fixed",
                    len(b.joints), b.control_mode, b.num_actuators,
                    b.q.start, b.q.stop, b.qd.start, b.qd.stop))
    # consecutive contacts of the same links in one line
    groups = []
    for c in self.layout["contacts"]:
      key = (c["body_a"], c["link_a"], c["body_b"], c["link_b"])
      if groups and groups[-1][0] == key:
        groups[-1][2] += c["count"]
      else:
        groups.append([key, c["first"], c["count"]])
    for (body_a, link_a, body_b, link_b), first, count in groups:
      lines.append("  contacts %d..%d: %s/%s - %s/%s" %
                   (first, first + count - 1, body_a, link_a or "plane",
                    body_b, link_b))
    for w in self.layout.get("warnings", []):
      lines.append("  warning: " + w)
    for f in self.manifest["functions"]:
      lines.append("  kernel %s: %d operations%s" %
                   (f["name"], f["num_operations"],
                    ", %d CUDA segments" % f["segments"]
                    if f["segments"] > 1 else ""))
    return "\n".join(lines)

  def compile(self, backend="cuda", precision="float"):
    """Compiles the kernels for the backend ("cuda", "cpu", "openmp" or
    "serial") and precision, unless they exist in the cache."""
    if precision not in ("float", "double"):
      raise ValueError("precision must be float or double")
    kind = "cuda" if backend == "cuda" else "cpu"
    if precision in self._compiled[kind]:
      return
    if kind == "cuda":
      self._compiled[kind][precision] = baking.compile_cuda(
          self.directory, precision, self.device, self.jobs, self.verbose)
    else:
      self._compiled[kind][precision] = baking.compile_cpu(
          self.directory, precision, self.jobs, self.verbose)
    self._model = None

  def model(self, backend="cuda", precision="float"):
    """The pytinykernels.KernelModel with the compiled kernels."""
    self.compile(backend, precision)
    if self._model is None:
      import pytinykernels
      self._model = pytinykernels.load_model(
          os.path.join(self.directory, "kernels.json"),
          dict(self._compiled["cpu"]),
          {k: list(v) for k, v in self._compiled["cuda"].items()},
          self.device)
    return self._model

  def batch(self, num_worlds, backend="cuda", precision="float",
            num_threads=0):
    """A batch of num_worlds worlds in their initial state, simulated on the
    backend ("cuda", "openmp" or "serial") in float or double precision."""
    from .batch import Batch
    return Batch(self, num_worlds, backend, precision, num_threads)
