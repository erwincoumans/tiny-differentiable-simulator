# Generated batched kernels

A TDS model can be turned into batched kernels that simulate thousands of
independent worlds at once, on the CPU (serially or with OpenMP) or on the GPU
with CUDA, in float or double precision. The model is traced once with
CppADCodeGen, and the generated code is straight-line C++ that runs one world
per CPU iteration or CUDA thread.

| directory | model |
|---|---|
| [g1](g1) | Unitree G1 humanoid with hands (49 dof), PD control with targets and gains as inputs, feet-only contacts, used by `g1_opengl_*_example --backend` |
| [laikago](laikago) | the laikago quadruped of `examples/environments/laikago_environment2.h`, used by the `pytinydiffsim` vectorized environments and the ARS examples |

Worlds of URDF robots and objects can also be baked without CMake or nvcc,
from Python or JSON, with the kernels compiled by NVRTC at run time and
cached: see [python/tds_bake](../../python/tds_bake/README.md).

## Adding a model

1. Write the model as a template on the algebra, with functions that map input
   arrays to output arrays, see [g1/g1_model.hpp](g1/g1_model.hpp). The trace
   records one path through the code, so a function must not branch on its
   input values. Use `tds::where_lt` and friends instead of `if`. For contacts,
   set `keep_all_points_` on the multibody constraint solver, so that the
   contact points do not depend on the state, and keep the number of
   candidate contact points small (the G1 only keeps its 8 foot spheres).
2. Write a generator, see [g1/g1_codegen.cpp](g1/g1_codegen.cpp): create a
   `tds::KernelCodegen`, add constants (time step, defaults) and one
   `add_function` per function with its named input and output arrays, then
   `write` the generated sources to the directory in `argv[1]`.
3. Add the kernels to [CMakeLists.txt](CMakeLists.txt):
   ```cmake
   tds_add_kernels(mymodel
     GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mymodel/generated
     GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/mymodel/mymodel_codegen.cpp)
   ```
   This creates the static library `mymodel_kernels`, with
   `mymodel_kernel_model()` declared in `mymodel_kernels.h`.
4. Generate the sources and check them in, so that a build does not need
   CppAD:
   ```
   cmake -DUSE_CPPAD=ON -DTDS_GENERATE_KERNELS=ON ..
   make update_mymodel_kernels
   ```
   With `TDS_GENERATE_KERNELS=ON` the kernels are always generated at build
   time from the current model. `TDS_KERNELS_CUDA=OFF` skips the CUDA kernels,
   they are compiled when CMake finds a CUDA compiler.

## Running the kernels

`tds::KernelBatch` ([src/utils/kernel_batch.hpp](../../src/utils/kernel_batch.hpp))
owns the arrays of a batch of worlds, in structure-of-arrays layout (element k
of world i is at `array[k * num_worlds + i]`). With CUDA the arrays stay on the
GPU between kernel launches, `upload` and `download` copy them.

```cpp
tds::KernelBatch<float> batch(g1_kernel_model(), 4096, tds::KernelBackend::CUDA);
batch.fill("state", model.default_state());
batch.fill("control", model.default_control());
batch.upload("state");
batch.upload("control");
for (int s = 0; s < g1_kernels::substeps; ++s) {
  batch.launch("step");                 // state, control -> next_state
  batch.swap("state", "next_state");
}
batch.launch("poses");                  // state -> poses
batch.download("poses");
```

The same from Python, with the `pytinykernels` module that the CMake build
creates next to `pytinydiffsim`:

```python
import pytinykernels as tk
batch = tk.KernelBatch("g1", 4096, backend="cuda", precision="float")
batch.fill("state", state)
batch.upload("state")
batch.launch("step")
batch.swap("state", "next_state")
batch.download("state")
states = batch.array("state").T  # numpy view, (num_worlds, state_dim)
```

[python/examples/tiled_kernel_render.py](../../python/examples/tiled_kernel_render.py)
simulates a batch of G1 or laikago worlds and renders one world per tile.

## Checking and measuring

`g1_kernel_benchmark` compares each backend and precision with the generic
TDS model (one step from random states, and the 2 second drop of the G1
example), and measures the speed. `ctest -R g1_kernels` runs the comparison.

The double kernels match the generic model to about 1e-13; the float
kernels differ by about 1e-4 per step in the joint velocities, and give the
same standing pose.

G1 frame time (6 substeps of 1/360 s, including the poses kernel and their
download) on an RTX 5090 and a 12-core Ryzen 9 3900X:

| worlds | generic TDS, 1 thread | OpenMP, 24 threads | CUDA float | CUDA double |
|---|---|---|---|---|
| 1024 | 1315 ms | 68 ms | 6.9 ms | |
| 4096 | 4733 ms | 135 ms | 7.2 ms | 30.8 ms |
| 16384 | | | 15.3 ms | 44.2 ms |

Each world runs in one CUDA thread, so up to a few thousand worlds the frame
time stays at about 6.5 ms, the time one thread takes for 6 substeps of the
140k operation kernel.
