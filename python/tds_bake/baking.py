"""Baking a world: generating the sources of its kernels, compiling them for
CUDA with NVRTC and for the CPU with a C++ compiler, and caching the result.

The cache directory (TDS_BAKE_CACHE, default ~/.cache/tds_bake) holds one
directory per baked world, named after the world and a hash of its
description and URDF files:

  g1_walk-3f2a9c0e1b7d4a65/
    world.json          the world as given
    resolved.json       the description of the kernel generator
    kernels.json        functions, arrays and the layout of the world
    kernels_cpu.cpp     generated sources
    kernels_cuda.cu
    bodies/*.urdf       URDF files of the shapes
    cuda/sm_120/float/  CUBIN (or PTX) of each part of the CUDA kernels
    cpu/float.so        CPU kernels
    __init__.py         makes the directory an importable module
"""

import concurrent.futures
import contextlib
import hashlib
import json
import os
import shutil
import subprocess
import sys
import sysconfig
import tempfile
import time

from . import urdf as urdf_utils
from .world import World

# changes when the generated sources change for the same world
BAKE_VERSION = 3


def _native():
  import pytinykernels
  return pytinykernels


def cache_root():
  root = os.environ.get("TDS_BAKE_CACHE")
  if not root:
    root = os.path.join(os.path.expanduser("~"), ".cache", "tds_bake")
  return root


def _log(verbose, message):
  if verbose:
    print("[tds_bake] " + message, flush=True)


@contextlib.contextmanager
def _file_lock(path):
  """An exclusive lock between processes, so that two processes do not bake
  or compile the same world at the same time."""
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "a+") as f:
    if os.name == "nt":
      import msvcrt
      while True:
        try:
          msvcrt.locking(f.fileno(), msvcrt.LK_LOCK, 1)
          break
        except OSError:
          time.sleep(0.1)
      try:
        yield
      finally:
        f.seek(0)
        msvcrt.locking(f.fileno(), msvcrt.LK_UNLCK, 1)
    else:
      import fcntl
      fcntl.flock(f.fileno(), fcntl.LOCK_EX)
      try:
        yield
      finally:
        fcntl.flock(f.fileno(), fcntl.LOCK_UN)


def _sha256_file(path):
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for chunk in iter(lambda: f.read(1 << 20), b""):
      h.update(chunk)
  return h.hexdigest()


def world_hash(world, max_segment_kb=256):
  """The hash of the description of the world and its URDF files, which
  identifies the baked kernels."""
  with tempfile.TemporaryDirectory() as tmp:
    spec, urdf_files = world.resolve(os.path.join(tmp, "bodies"))
    key = json.loads(json.dumps(spec))
    for b in key["bodies"]:
      # the content of the URDF, not its location
      b["urdf"] = _sha256_file(b["urdf"])
  key["_bake"] = [BAKE_VERSION, _native().kernel_format_version,
                  max_segment_kb]
  text = json.dumps(key, sort_keys=True)
  return hashlib.sha256(text.encode()).hexdigest()[:16]


_INIT_PY = '''"""Baked tds_bake world {name!r}, load it with
  import importlib; world = importlib.import_module("<module>").world
or tds_bake.load(<this directory>)."""
import os as _os

import tds_bake as _tds_bake

world = _tds_bake.load(_os.path.dirname(_os.path.abspath(__file__)))
batch = world.batch
'''


def generate(world, directory=None, max_segment_kb=256, force=False,
             verbose=True):
  """Generates the sources of the kernels of the world into its cache
  directory (or the given one), unless they exist. Returns the directory."""
  if isinstance(world, dict):
    world = World.from_dict(world)
  tk = _native()
  if not tk.has_codegen:
    raise RuntimeError("pytinykernels is built without the kernel generator")
  if directory is None:
    directory = os.path.join(
        cache_root(), "%s-%s" % (world.name, world_hash(world, max_segment_kb)))
  directory = os.path.abspath(directory)
  manifest = os.path.join(directory, "kernels.json")
  with _file_lock(directory + ".lock"):
    if os.path.exists(manifest) and not force:
      return directory
    start = time.time()
    _log(verbose, "generating the kernels of %r in %s" % (world.name,
                                                         directory))
    parent = os.path.dirname(directory)
    os.makedirs(parent, exist_ok=True)
    tmp = tempfile.mkdtemp(prefix=".tmp-", dir=parent)
    try:
      spec, _ = world.resolve(os.path.join(tmp, "bodies"))
      layout = json.loads(
          tk.generate_world(json.dumps(spec), tmp, max_segment_kb))
      for warning in layout.get("warnings", []):
        _log(True, "warning: " + warning)
      # the paths of the final directory
      for b in spec["bodies"]:
        if b["urdf"].startswith(tmp):
          b["urdf"] = directory + b["urdf"][len(tmp):]
      with open(os.path.join(tmp, "resolved.json"), "w") as f:
        json.dump(spec, f, indent=2)
      with open(os.path.join(tmp, "world.json"), "w") as f:
        f.write(world.to_json())
      with open(os.path.join(tmp, "__init__.py"), "w") as f:
        f.write(_INIT_PY.format(name=world.name))
      if os.path.exists(directory):
        shutil.rmtree(directory)
      os.replace(tmp, directory)
    except BaseException:
      shutil.rmtree(tmp, ignore_errors=True)
      raise
    _log(verbose, "generated in %.1f s" % (time.time() - start))
  return directory


# NVRTC


def find_nvrtc():
  """The path of the newest NVRTC library found: TDS_NVRTC_LIBRARY, the
  nvidia-cuda-nvrtc Python packages, CUDA_HOME / CUDA_PATH, or the library
  search path (an empty string lets the native module search)."""
  env = os.environ.get("TDS_NVRTC_LIBRARY")
  if env:
    return env
  candidates = []
  site_dirs = set()
  for key in ("purelib", "platlib"):
    path = sysconfig.get_paths().get(key)
    if path:
      site_dirs.add(path)
  for p in sys.path:
    if p.endswith("site-packages") or p.endswith("dist-packages"):
      site_dirs.add(p)
  if os.name == "nt":
    names = [("nvrtc64_130_0.dll", 13), ("nvrtc64_120_0.dll", 12)]
    subdirs = ["nvidia/cu13/bin", "nvidia/cuda_nvrtc/bin"]
  else:
    names = [("libnvrtc.so.13", 13), ("libnvrtc.so.12", 12)]
    subdirs = ["nvidia/cu13/lib", "nvidia/cuda_nvrtc/lib"]
  for site in site_dirs:
    for sub in subdirs:
      for name, major in names:
        path = os.path.join(site, sub, name)
        if os.path.exists(path):
          candidates.append((major, path))
  for var in ("CUDA_HOME", "CUDA_PATH"):
    root = os.environ.get(var)
    if root:
      for name, major in names:
        for sub in ("lib64", "lib", "bin"):
          path = os.path.join(root, sub, name)
          if os.path.exists(path):
            candidates.append((major, path))
  if candidates:
    return max(candidates)[1]
  return ""


def _memory_gb():
  try:
    return os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE") / 1e9
  except (AttributeError, ValueError, OSError):
    return 16.


def _jobs(jobs, gb_per_job):
  if jobs:
    return jobs
  cpus = os.cpu_count() or 4
  return max(1, min(cpus, int(_memory_gb() / gb_per_job)))


def compile_cuda(directory, precision="float", device=0, jobs=None,
                 verbose=True):
  """Compiles the CUDA kernels of a generated world with NVRTC for the
  compute capability of the device, unless they exist. Returns the list of
  CUBIN or PTX files."""
  tk = _native()
  info = tk.cuda_device(device)
  cc = info["compute_capability"]
  out_dir = os.path.join(directory, "cuda", "sm_%d" % cc, precision)
  done = os.path.join(out_dir, "done.json")
  with _file_lock(os.path.join(directory, "cuda.lock")):
    if os.path.exists(done):
      with open(done) as f:
        return [os.path.join(out_dir, p) for p in json.load(f)["images"]]
    with open(os.path.join(directory, "kernels.json")) as f:
      manifest = json.load(f)
    with open(os.path.join(directory, "kernels_cuda.cu")) as f:
      source = f.read()
    library = find_nvrtc()
    nvrtc = tk.nvrtc(library)
    parts = manifest["parts"]
    start = time.time()
    _log(verbose, "compiling %d CUDA parts (%s) for %s with NVRTC %d.%d" %
         (len(parts), precision, info["name"], nvrtc["version"] // 1000,
          nvrtc["version"] % 1000 // 10))
    define = "-DTDS_KERNELS_%s" % precision.upper()

    def compile_part(p):
      t = time.time()
      result = tk.nvrtc_compile(source, "kernels_cuda.cu", cc,
                                [define, "-DTDS_KERNEL_PART=%d" % p], library)
      name = "part_%d.%s" % (p, "ptx" if result["is_ptx"] else "cubin")
      return p, name, result, time.time() - t

    os.makedirs(out_dir, exist_ok=True)
    images = [None] * len(parts)
    # NVRTC needs about 1 GB per part of 256 KB
    workers = _jobs(jobs, 2.)
    # the largest parts first
    order = sorted(range(len(parts)), key=lambda p: -parts[p]["code_kb"])
    with concurrent.futures.ThreadPoolExecutor(workers) as pool:
      for p, name, result, seconds in pool.map(compile_part, order):
        with open(os.path.join(out_dir, name), "wb") as f:
          f.write(result["image"])
        images[p] = name
        if result["log"].strip() and verbose:
          _log(verbose, "part %d: %s" % (p, result["log"].strip()))
    with open(done, "w") as f:
      json.dump({"images": images, "nvrtc": nvrtc["version"],
                 "library": nvrtc["library"],
                 "seconds": time.time() - start}, f)
    _log(verbose, "compiled the CUDA kernels in %.1f s" % (time.time() - start))
  return [os.path.join(out_dir, p) for p in images]


def _cxx():
  return os.environ.get("CXX") or ("cl" if os.name == "nt" else "c++")


def compile_cpu(directory, precision="float", jobs=None, verbose=True,
                optimization="-O2"):
  """Compiles the CPU kernels of a generated world with the C++ compiler
  (CXX or c++) into a shared library, unless it exists. Returns its path."""
  suffix = ".dll" if os.name == "nt" else ".so"
  out_dir = os.path.join(directory, "cpu")
  library = os.path.join(out_dir, precision + suffix)
  with _file_lock(os.path.join(directory, "cpu.lock")):
    if os.path.exists(library):
      return library
    with open(os.path.join(directory, "kernels.json")) as f:
      manifest = json.load(f)
    parts = manifest["cpu_parts"]
    cxx = _cxx()
    if shutil.which(cxx) is None:
      raise RuntimeError(
          "The CPU kernels need a C++ compiler (set CXX), %r is not found. "
          "The CUDA kernels only need NVRTC." % cxx)
    source = os.path.join(directory, "kernels_cpu.cpp")
    build = os.path.join(out_dir, "build-" + precision)
    os.makedirs(build, exist_ok=True)
    start = time.time()
    _log(verbose, "compiling %d CPU parts (%s) with %s" %
         (len(parts), precision, cxx))
    define = "-DTDS_KERNELS_%s" % precision.upper()
    msvc = os.path.basename(cxx).lower() in ("cl", "cl.exe")

    def compile_part(p):
      obj = os.path.join(build, "part_%d.%s" % (p, "obj" if msvc else "o"))
      if msvc:
        cmd = [cxx, "/nologo", "/c", "/O2", "/std:c++17", define,
               "/DTDS_KERNEL_PART=%d" % p, source, "/Fo" + obj]
      else:
        cmd = [cxx, optimization, "-fPIC", "-c", "-std=c++17", define,
               "-DTDS_KERNEL_PART=%d" % p, source, "-o", obj]
      result = subprocess.run(cmd, capture_output=True, text=True)
      if result.returncode != 0:
        raise RuntimeError("compiling %s failed:\n%s\n%s" %
                           (" ".join(cmd), result.stdout, result.stderr))
      return obj

    workers = _jobs(jobs, 3.)
    with concurrent.futures.ThreadPoolExecutor(workers) as pool:
      objects = list(pool.map(compile_part, range(len(parts))))
    tmp = library + ".tmp"
    if msvc:
      cmd = [cxx, "/nologo", "/LD"] + objects + ["/Fe" + tmp]
    elif sys.platform == "darwin":
      cmd = [cxx, "-dynamiclib"] + objects + ["-o", tmp]
    else:
      cmd = [cxx, "-shared"] + objects + ["-o", tmp]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
      raise RuntimeError("linking %s failed:\n%s\n%s" %
                         (library, result.stdout, result.stderr))
    os.replace(tmp, library)
    shutil.rmtree(build, ignore_errors=True)
    _log(verbose, "compiled the CPU kernels in %.1f s" % (time.time() - start))
  return library


def bake(world,
         backends=("cuda",),
         precisions=("float",),
         directory=None,
         force=False,
         max_segment_kb=256,
         jobs=None,
         device=0,
         verbose=True):
  """Bakes a world (a World, a dictionary, or the path of a JSON file) into
  batched kernels: generates their sources and compiles them for the backends
  ("cuda", "cpu") and precisions ("float", "double"), or reuses them from the
  cache. Returns a BakedWorld, whose batch() method creates batches of
  worlds. Other backends and precisions are compiled when a batch needs them.
  """
  from .baked import BakedWorld
  if isinstance(world, str):
    world = World.load(world)
  elif isinstance(world, dict):
    world = World.from_dict(world)
  directory = generate(world, directory, max_segment_kb, force, verbose)
  baked = BakedWorld(directory, device=device, verbose=verbose, jobs=jobs)
  for backend in backends:
    for precision in precisions:
      baked.compile(backend, precision)
  return baked


def load(directory, device=0, verbose=True):
  """Loads a baked world from its directory (see bake)."""
  from .baked import BakedWorld
  return BakedWorld(directory, device=device, verbose=verbose)


def cached_worlds():
  """The baked worlds in the cache: a list of dictionaries with their name,
  directory, size and compiled kernels."""
  root = cache_root()
  result = []
  if not os.path.isdir(root):
    return result
  for entry in sorted(os.listdir(root)):
    path = os.path.join(root, entry)
    manifest = os.path.join(path, "kernels.json")
    if not os.path.exists(manifest):
      continue
    size = 0
    for dirpath, _, files in os.walk(path):
      size += sum(os.path.getsize(os.path.join(dirpath, f)) for f in files)
    compiled = []
    cuda = os.path.join(path, "cuda")
    if os.path.isdir(cuda):
      for arch in os.listdir(cuda):
        for precision in os.listdir(os.path.join(cuda, arch)):
          if os.path.exists(os.path.join(cuda, arch, precision, "done.json")):
            compiled.append("cuda/%s/%s" % (arch, precision))
    cpu = os.path.join(path, "cpu")
    if os.path.isdir(cpu):
      compiled += ["cpu/" + os.path.splitext(f)[0] for f in os.listdir(cpu)
                   if f.endswith((".so", ".dll"))]
    with open(manifest) as f:
      name = json.load(f).get("strings", {}).get("world", entry)
    result.append({"name": name, "directory": path, "megabytes": size / 1e6,
                   "compiled": sorted(compiled)})
  return result


def clear_cache(name=None):
  """Removes the baked worlds with the given name (all if None) from the
  cache. Returns the removed directories."""
  removed = []
  for entry in cached_worlds():
    if name is None or entry["name"] == name:
      shutil.rmtree(entry["directory"], ignore_errors=True)
      lock = entry["directory"] + ".lock"
      if os.path.exists(lock):
        os.remove(lock)
      removed.append(entry["directory"])
  return removed
