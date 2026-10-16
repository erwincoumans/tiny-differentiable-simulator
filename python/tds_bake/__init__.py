"""tds_bake: bake worlds of robots and objects into batched CUDA and CPU
kernels, and simulate thousands of them at once.

  import tds_bake

  world = tds_bake.World("g1_stand", dt=1 / 360, substeps=6)
  world.add_urdf("g1", "unitree_g1/g1_29dof_with_hand_rev_1_0.urdf",
                 position=(0, 0, 0.8), kp=1000, kd=10, armature=0.01,
                 collision_links=["left_ankle_roll_link",
                                  "right_ankle_roll_link"])
  baked = tds_bake.bake(world)          # or tds_bake.bake("world.json")
  batch = baked.batch(4096, backend="cuda", precision="float")
  for frame in range(100):
    batch.step()
  state = batch.get("state")            # (4096, state_dim)

bake traces the world with CppADCodeGen into straight-line C++ and CUDA code,
compiles the CUDA kernels with NVRTC (pip install nvidia-cuda-nvrtc, or the
CUDA toolkit) and the CPU kernels with the C++ compiler, and keeps them in a
cache (~/.cache/tds_bake, or TDS_BAKE_CACHE), so that baking the same world
again takes no time. See SCHEMA_DOC for the JSON description of a world, and
python -m tds_bake --help for the command line.
"""

from .baking import (bake, cache_root, cached_worlds, clear_cache, find_nvrtc,
                   generate, load, world_hash)
from .baked import BakedWorld, BodyLayout
from .batch import Batch
from .urdf import data_paths, find_file
from .world import SCHEMA_DOC, World

__all__ = [
    "World", "bake", "load", "generate", "BakedWorld", "BodyLayout", "Batch",
    "SCHEMA_DOC", "cache_root", "cached_worlds", "clear_cache", "find_nvrtc",
    "find_file", "data_paths", "world_hash"
]
