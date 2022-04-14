# Tiny Differentiable Simulator

Tiny Differentiable Simulator is a header-only C++ (and CUDA) physics library with zero dependencies.

+ Note that the main repository is transfered from google-research to [Erwin Coumans](https://github.com/erwincoumans/tiny-differentiable-simulator)

It currently implements various rigid-body dynamics algorithms, including forward and inverse dynamics, as well as contact models based on impulse-level LCP and force-based nonlinear spring-dampers. Actuator models for motors, servos, and Series-Elastic Actuator (SEA) dynamics are implemented.

The entire codebase is templatized so you can use forward- and reverse-mode automatic differentiation
scalar types, such as CppAD, Stan Math fvar and ceres::Jet. The library can also be used with
regular float or double precision values. Another option is to use the included
fix-point integer math, that provide cross-platform deterministic computation.

TDS can run thousands of simulations in parallel on a single RTX 2080 CUDA GPU at 50 frames per second:

https://user-images.githubusercontent.com/725468/135697035-7df34b85-c73e-4739-9a76-dc114ce84c4c.mp4

Multiple visualizers are available, see below.

![](https://github.com/erwincoumans/tiny-differentiable-simulator/blob/master/data/trb_meshcat.jpg)

## Bibtex
Please use the following reference to cite this research:

```
@inproceedings{heiden2021neuralsim,
  author =	  {Heiden, Eric and Millard, David and Coumans, Erwin and Sheng, Yizhou and Sukhatme, Gaurav S},
  year =		  {2021},
  title =		  {Neural{S}im: Augmenting Differentiable Simulators with Neural Networks},
  booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)},
  url =		    {https://github.com/google-research/tiny-differentiable-simulator}
}
```

## Related Papers
* "Inferring Articulated Rigid Body Dynamics from RGBD Video" (IROS 2022 submission) Eric Heiden, Ziang Liu, Vibhav Vineet, Erwin Coumans, Gaurav S. Sukhatme [Project Page](https://eric-heiden.com/publication/2022-video2sim-iros)
* "NeuralSim: Augmenting Differentiable Simulators with Neural Networks" (ICRA 2021) Eric Heiden, David Millard, Erwin Coumans, Yizhou Sheng, Gaurav S. Sukhatme. [PDF on Arxiv](https://arxiv.org/abs/2011.04217)
* “Augmenting Differentiable Simulators with Neural Networks to Close the Sim2Real Gap” (RSS 2020 sim-to-real workshop), Eric Heiden, David Millard, Erwin Coumans, Gaurav Sukhatme. [PDF on Arxiv](https://sim2real.github.io/assets/papers/2020/heiden.pdf) and [video](https://www.youtube.com/watch?v=awhkI5xtFa0)
* "Interactive Differentiable Simulation", 2020, Eric Heiden, David Millard, Hejia Zhang, Gaurav S. Sukhatme. [PDF on Arxiv](https://arxiv.org/abs/1905.10706)

## Related Research using TDS by Others
* "Efficient Differentiable Simulation of Articulated Bodies" (ICML 2021) Yi-Ling Qiao, Junbang Liang, Vladlen Koltun, Ming C. Lin [PDF on Arxiv](https://arxiv.org/abs/2109.07719)

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

### OpenGL 3+ Visualization

* tiny_opengl3_app, an OpenGL3 visualizer

This visualizer is native part of this library under src/visualizer/opengl

### MeshCat Visualization

* [MeshCat](https://github.com/rdeits/meshcat), a web-based visualizer that uses WebGL

A C++ ZMQ interface is provided.

Before running the example, install python, pip and meshcat, run the meshcat-server 
and open the web browser (Chrome is recommended for a good three.js experience.)
```
pip install meshcat
meshcat-server --open
This should open Chrome at http://localhost:7000/static/
Then compile and run tiny_urdf_parser_meshcat_example in optimized/release build.
```

URDF files can be loaded using a provided parser based on TinyXML2.

All dependencies for meshcat visualization are included in third_party.

---

*Disclaimer: This is not an official Google product.*

