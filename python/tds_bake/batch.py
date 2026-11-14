"""A batch of worlds of a baked world, simulated on the CPU or with CUDA."""

import numpy as np


class _CudaArray:
  """Device memory of a batch array, for torch.as_tensor and other libraries
  that support __cuda_array_interface__."""

  def __init__(self, pointer, shape, dtype, owner):
    self._owner = owner  # keeps the batch alive
    self.__cuda_array_interface__ = {
        "shape": tuple(shape),
        "typestr": np.dtype(dtype).str,
        "data": (pointer, False),
        "version": 3,
        "strides": None,
        # the stream of the kernels, 1: the legacy default stream
        "stream": owner.stream or 1,
    }


class Batch:
  """num_worlds independent copies of a baked world.

  Arrays of the batch (state, control, poses, ...) have one column per world:
  element k of world i is at array[k, i]. With CUDA the arrays stay in device
  memory, get() and set() copy them; torch() gives a view of the device
  memory without copying. The main arrays:
    state    q and qd of all bodies, see BakedWorld.bodies for the layout
    control  PD targets, kp and kd (or torques) of the actuated joints
  Example:
    batch = baked.batch(4096, "cuda")
    g1 = baked.body("g1")
    control = batch.get("control")          # (num_worlds, control_dim)
    control[:, g1.targets] = targets
    batch.set("control", control)
    batch.step()                            # baked.substeps time steps
    state = batch.get("state")              # (num_worlds, state_dim)
  """

  def __init__(self, baked, num_worlds, backend="cuda", precision="float",
               num_threads=0):
    import pytinykernels as tk
    if backend not in ("cuda", "openmp", "serial", "cpu"):
      raise ValueError("backend must be cuda, openmp or serial")
    if backend == "cpu":
      backend = "openmp"
    self.baked = baked
    self.num_worlds = int(num_worlds)
    self.backend = backend
    self.precision = precision
    self.dtype = np.float32 if precision == "float" else np.float64
    model = baked.model(backend, precision)
    self._batch = tk.KernelBatch(model, self.num_worlds, backend, precision,
                                 num_threads)
    self.split = baked.split
    self.solver = baked.solver
    # contact solver iterations, can be changed at any time
    self.iterations = baked.iterations
    # start the contact solver from the impulses of the previous time step
    self.warm_start = False
    self.substeps = baked.substeps
    self.dt = baked.dt
    self.set("control", baked.default_control())
    self.reset()

  def __repr__(self):
    return "Batch(%r, %d worlds, %s, %s)" % (self.baked.name, self.num_worlds,
                                              self.backend, self.precision)

  @property
  def stream(self):
    """The CUDA stream of the kernels and copies, as an integer (a
    cudaStream_t or CUstream), 0 for the legacy default stream. Set it to the
    stream of the library that reads or writes the arrays."""
    return self._batch.stream

  @stream.setter
  def stream(self, value):
    self._batch.stream = int(value)

  @property
  def device_name(self):
    return self._batch.device_name

  @property
  def is_cuda(self):
    return self.backend == "cuda"

  def dim(self, name):
    return self._batch.dim(name)

  def array_names(self):
    return self._batch.array_names()

  # data transfer

  def array(self, name):
    """numpy view (dim, num_worlds) of the host copy of an array."""
    return self._batch.array(name)

  def download(self, name):
    self._batch.download(name)

  def upload(self, name):
    self._batch.upload(name)

  def get(self, name, download=True):
    """The array as numpy array of shape (num_worlds, dim), a copy. Downloads
    it first from the device."""
    if download:
      self._batch.download(name)
    return np.array(self._batch.array(name).T)

  def set(self, name, values, worlds=None):
    """Sets the array of all worlds or of the given worlds (indices or a
    boolean mask): values of shape (dim,) for all of them, or
    (num_worlds or len(worlds), dim)."""
    host = self._batch.array(name)
    values = np.asarray(values, dtype=self.dtype)
    if worlds is None:
      host[:, :] = values.T if values.ndim == 2 else values[:, None]
    else:
      if self.is_cuda:
        # keep the other worlds
        self._batch.download(name)
      worlds = np.asarray(worlds)
      if worlds.dtype == bool:
        worlds = np.nonzero(worlds)[0]
      host[:, worlds] = values.T if values.ndim == 2 else values[:, None]
    self._batch.upload(name)

  def torch(self, name):
    """A torch tensor of shape (dim, num_worlds) that shares the memory of
    the array that the kernels use (device memory with CUDA). Writing it
    changes the array, reading it sees the latest kernel results."""
    import torch
    if self.is_cuda:
      shape = (self._batch.dim(name), self.num_worlds)
      array = _CudaArray(self._batch.device_pointer(name), shape, self.dtype,
                         self)
      return torch.as_tensor(array, device="cuda:%d" % self.baked.device)
    return torch.from_numpy(self._batch.array(name))

  # simulation

  def reset(self, worlds=None, state=None):
    """Sets the state of all worlds or of the given worlds to the initial
    state of the world, or to the given state."""
    if state is None:
      state = self.baked.default_state()
    self.set("state", state, worlds)
    if self.split and worlds is None:
      self._batch.fill("lambda", [0.] * self._batch.dim("lambda"))
      self._batch.upload("lambda")
      self._batch.fill("lambda_init", [0.] * self._batch.dim("lambda_init"))
      self._batch.upload("lambda_init")

  def step(self, substeps=None):
    """Advances all worlds by substeps time steps (default: the substeps of
    the world), with the current control. Asynchronous with CUDA. A world
    with the solver "external" has no contacts here: the engine that solves
    its contacts runs its kernels."""
    b = self._batch
    for _ in range(self.substeps if substeps is None else substeps):
      if self.solver == "external":
        # without the other engine there are no contacts
        b.launch("predict")
        b.launch("advance")
      elif self.split:
        b.launch("contact")
        if self.warm_start:
          b.copy("lambda_init", "lambda")
        b.launch("pgs", iterations=self.iterations)
        b.launch("integrate")
      else:
        b.launch("step")
      b.copy("state", "next_state")

  def synchronize(self):
    self._batch.synchronize()

  def poses(self):
    """World poses of the visual shapes, (num_worlds, num_visuals, 7) with
    position and quaternion (x, y, z, w)."""
    self._batch.launch("poses")
    self._batch.download("poses")
    return self._batch.array("poses").T.reshape(self.num_worlds, -1, 7)

  def links(self):
    """World poses of the bases and links of all bodies,
    (num_worlds, num_links, 7), see BodyLayout.link_index."""
    self._batch.launch("links")
    self._batch.download("links")
    return self._batch.array("links").T.reshape(self.num_worlds, -1, 7)

  def contact_forces(self):
    """Normal force of each contact point in the last time step,
    (num_worlds, num_contacts), see BodyLayout.contact_indices."""
    n = self.baked.num_contacts
    if n == 0:
      return np.zeros((self.num_worlds, 0), self.dtype)
    if not self.split:
      raise RuntimeError("contact forces need the split contact solver")
    self._batch.download("lambda")
    return self._batch.array("lambda")[:n].T / self.dt
