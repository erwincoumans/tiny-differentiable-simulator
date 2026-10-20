# tds_bake

Bake worlds of robots and objects into batched CUDA and CPU kernels, and
simulate thousands of copies at once. A world is described in Python or JSON,
traced with CppADCodeGen (src/bake), compiled with NVRTC for CUDA and with the
C++ compiler for the CPU, and kept in a cache (`~/.cache/tds_bake`, or
`TDS_BAKE_CACHE`), so that baking it again takes no time.

```bash
python -m tds_bake example g1_stand > g1.json     # a world to start from
python -m tds_bake schema                         # the JSON format
python -m tds_bake bake g1.json                   # ~20 s the first time
python -m tds_bake bench g1.json --num-worlds 4096
python -m tds_bake view g1.json --num-worlds 4096 --tiles-x 32 --tiles-y 24
python -m tds_bake view g1.json --num-worlds 4096 --mode scene --no-shadows
python -m tds_bake info g1.json                   # bodies, joints, contacts
python -m tds_bake list                           # the cache
```

```python
import tds_bake

world = tds_bake.World("g1_stand", dt=1 / 360, substeps=6, iterations=50)
world.add_urdf("g1", "unitree_g1/g1_29dof_with_hand_rev_1_0.urdf",
               position=(0, 0, 0.993), kp=1000, kd=10, armature=0.01,
               collision_links=["left_ankle_roll_link", "right_ankle_roll_link"])
baked = tds_bake.bake(world)            # or tds_bake.bake("g1.json")
batch = baked.batch(4096, backend="cuda", precision="float")
for frame in range(600):
  batch.step()                          # substeps time steps, on the GPU
g1 = baked.body("g1")
state = batch.get("state")              # numpy (4096, state_dim)
print(state[:, g1.base_position])
```

- `examples.py`: the example worlds (`g1_stand`, `g1_walk`, `g1_walk_body`,
  `g1_ball`, `piper_pick`: the AgileX Piper picking up a cube, moved by
  `python/examples/piper_pick.py`).
- `world.py`: `World`, the JSON format (`SCHEMA_DOC`) and its checks.
- `baking.py`: generating, compiling (in parallel parts) and caching.
- `baked.py`, `batch.py`: a baked world, the layout of its arrays
  (`BodyLayout`) and batches of worlds, with numpy and zero-copy torch views.
- `render.py`: tiled or single 3D scene rendering with pytinyopengl3. With
  CUDA, the poses go to the OpenGL instance buffer on the GPU
  (CUDA-OpenGL interop, `TDS_BAKE_GL_INTEROP=0` copies them through the
  CPU).

Contacts: every pair of colliding geometries gives a fixed number of contact
points in every step (sphere-sphere, sphere-plane, capsule-plane, box-plane,
sphere-box, capsule-sphere), and all of them are in the contact problem of every step, so
the cost grows with the number of candidate contacts. List only the colliding
links and contact spheres that a task needs, and restrict them with the
collision filters ("with"). CUDA needs NVRTC (the CUDA toolkit or
`pip install nvidia-cuda-nvrtc`), the CPU kernels a C++ compiler.
