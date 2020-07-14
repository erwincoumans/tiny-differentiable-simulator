# Tiny Differentiable Simulator

Tiny Differentiable Simulator is a header-only C++ physics library with zero dependencies.

It currently implements various rigid-body dynamics algorithms, including forward and inverse dynamics, as well as contact models based on impulse-level LCP and force-based nonlinear spring-dampers. Actuator models for motors, servos, and Series-Elastic Actuator (SEA) dynamics are implemented.

The entire codebase is templatized so you can use forward- and reverse-mode automatic differentiation
scalar types, such as CppAD, Stan Math fvar and ceres::Jet. The library can also be used with
regular float or double precision values. Another option is to use the included
fix-point integer math, that provide cross-platform deterministic computation.

Examples to exploit the gradients for system identification using Ceres and
trajectory generation using the Control Toolbox are provided.

For optional visualization, a [MeshCat](https://github.com/rdeits/meshcat) ZMQ interface is provided and a [PyBullet](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3) based
visualizer.

![](https://github.com/erwincoumans/tiny-differentiable-simulator/blob/master/data/trb_meshcat.jpg)

## Related Papers

* “Augmenting Differentiable Simulators with Neural Networks to Close the Sim2Real Gap”, RSS 2020 sim-to-real workshop, Eric Heiden, David Millard, Erwin Coumans, Gaurav Sukhatme. [PDF on Arxiv](https://sim2real.github.io/assets/papers/2020/heiden.pdf) and [video](https://www.youtube.com/watch?v=awhkI5xtFa0)
* "Interactive Differentiable Simulation", 2020, Eric Heiden, David Millard, Hejia Zhang, Gaurav S. Sukhatme. [PDF on Arxiv](https://arxiv.org/abs/1905.10706)

## Getting started
The open-source version builds using CMake and requires a compiler with C++17 support.

```
mkdir build
cd build
cmake ..
make -j
```

## Examples

For visualization, two options are supported:

* MeshCat, a web-based visualizer that uses WebGL

Before running the example, install python, pip and meshcat, run the meshcat-server 
and open the web browser (Chrome is recommended for a good three.js experience.)
```
pip install meshcat
meshcat-server --open
This should open Chrome at http://localhost:7000/static/
Then compile and run tiny_urdf_parser_meshcat_example in optimized/release build.
```

* PyBullet visualization, a cross-platform solution that visualizes in a native OpenGL window

URDF files can be loaded either throught the PyBullet interface or a provided parser based on TinyXML2. All simulation features are therefore available without requiring PyBullet.

Several examples (see `examples/` folder) use either Ceres or Control Toolbox. To install:

* Ceres Solver (optional) >1.14.0
* Control Toolbox (optional) >3.0.2

### Ceres Solver
Check out the [installation](http://ceres-solver.org/installation.html) page from the manual.
Make sure the following dependencies are met: 
* Eigen 3 (install `libeigen3-dev`)
* BLAS and LAPACK via ATLAS >3.10.3 (install `libatlas-base-dev`)
* (optionally) SuiteSparse >5.1.2 (install `libsuitesparse-dev`)
* Google glog and gflags (install `libgoogle-glog-dev`)
```
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout v1.14.0
mkdir build
cd build
cmake ..
make
make install
```

### Control Toolbox
It is recommended to first set up BLASFEO and HPIPM for more efficient solvers that can also handle constraints in the optimization problems:
```
git clone https://github.com/giaf/blasfeo.git
cd blasfeo
mkdir build
cd build
cmake ..
make
make install
```
```
git clone https://github.com/giaf/hpipm.git
cd hpipm
mkdir build
cd build
cmake ..
make
make install
```
Next, install CppAD:
```
git clone https://github.com/coin-or/CppAD.git
cd CppAD
mkdir build
cd build
cmake ..
make
make install
```
Then install CppADCodeGen (we don't really need it but CT requires the CPPADCG flag to use the linearizers).
```
git clone https://github.com/joaoleal/CppADCodeGen.git
cd CppADCodeGen
mkdir build
cd build
cmake ..
make
make install
```

For the trajectory optimization experiments, the following modules from Control Toolbox are needed:
* `ct_core`
* `ct_optcon`
To build these, make sure Eigen and Boost (`libboost-dev` ~1.65) are installed. Next, issue the following commands:
```
git clone https://github.com/ethz-adrl/control-toolbox.git
cd ct_core
mkdir build
cd build
cmake .. -DCPPAD=1 -DHPIPM=1
make
make install
cd ../../ct_optcon
mkdir build
cd build
cmake .. -DHPIPM=1
make
make install
```

### MeshCat Visualization
To support visualization through the web-based visualizer MeshCat, the following packages need to be installed:
* [zero/libzmq](https://github.com/zeromq/libzmq) ~4.3.2
* [zeromq/cppzmq](https://github.com/zeromq/cppzmq) ~4.6.0
* [graeme-hill/crossguid](https://github.com/graeme-hill/crossguid) ~0.2.2
* [nlohmann/json](https://github.com/nlohmann/json) ~3.7.3
* [eric-heiden/cpp-base64](https://github.com/eric-heiden/cpp-base64)

#### libzmq
It is recommended to install the binary packages, as shown on the [ZeroMQ README file](https://github.com/zeromq/libzmq/blob/master/README.md).

#### cppzmq
```
git clone https://github.com/zeromq/cppzmq.git
cd cppzmq
git checkout v4.6.0
mkdir build && cd build
cmake ..
make -j
make install
```

#### crossguid
```
git clone https://github.com/graeme-hill/crossguid.git
cd crossguid
git checkout v0.2.2
mkdir build && cd build
cmake ..
make -j
make install
```

#### json
```
git clone https://github.com/nlohmann/json.git
cd json
git checkout v3.7.3
mkdir build && cd build
cmake ..
make -j
make install
```

#### base64
```
git clone https://github.com/eric-heiden/cpp-base64.git
cd cpp-base64
mkdir build && cd build
cmake ..
make -j
make install
```

#### MeshCat server
To install the MeshCat server, issue
```
pip install meshcat
```
To start the server and open the visualization in the browser, run
```
meshcat-server --open
```
This will start MeshCat with ZMQ listening on port 7000 which is the default setting in the C++ MeshCat visualization client.

### PyBullet visualization
```
git clone https://github.com/bulletphysics/bullet3.git
cd bullet3
./build_cmake_pybullet_double.sh
cd build_cmake
make install
```
To install the visualizer server to be able to run the visualizer window in shared-memory mode, issue
```
pip install pybullet
```
If the simulator is running with PyBullet visualization in shared-memory mode,
first open the visualizer server using
```
python -m pybullet_utils.runServer
```

### TinyXML2
To load URDF models without pybullet, our C++-based importer requires TinyXML2 which is installed as follows:
```
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2
mkdir build && cd build
cmake ..
make -j
make install
```

---

*Disclaimer: This is not an official Google product.*

