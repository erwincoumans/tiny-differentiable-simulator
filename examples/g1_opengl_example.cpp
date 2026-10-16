// Unitree G1 standing example, a reimplementation of the Newton example
// newton/examples/robot/example_robot_g1.py using Tiny Differentiable
// Simulator.
//
// A G1 humanoid (29 dof body + two 7 dof Dex3 hands) is dropped from 0.2 m
// above its standing height. Joint PD controllers (kp = 1000, kd = 10) hold
// the zero joint pose. Each world is simulated independently, the worlds are
// only offset for rendering. The model is in kernels/g1/g1_model.hpp.
//
// Usage: g1_opengl_tiny_example [--num-x NX] [--num-y NY]
//            [--backend generic|serial|openmp|cuda] [--precision float|double]
// simulates NX x NY copies of the G1, laid out in a grid along X and Y.
// The generic backend runs the TDS model directly, one world after the other.
// The other backends run the kernels generated from the same model by
// kernels/g1/g1_codegen.cpp: serially or with OpenMP on the CPU, or with CUDA
// on the GPU, in float (default) or double precision.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "opengl_urdf_visualizer.h"

#include "kernels/g1/g1_model.hpp"
#include "g1_kernels.h"
#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "utils/kernel_batch.hpp"
#include "utils/stopwatch.hpp"

using namespace tds;
using namespace TINY;
#include "math/tiny/tiny_algebra.hpp"

#ifdef USE_TINY
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;
#else
#include "math/eigen_algebra.hpp"
typedef EigenAlgebra MyAlgebra;
typedef typename MyAlgebra::Scalar MyScalar;
#endif

typedef G1Model<MyAlgebra> Model;

// Simulates all worlds for one frame, and provides the world pose (position,
// quaternion xyzw) of each visual for rendering.
struct Simulation {
  virtual ~Simulation() = default;
  virtual void step_frame() = 0;
  virtual void get_pose(int world, int visual, float pose[7]) const = 0;
};

// the TDS model, one world after the other
struct GenericSimulation : Simulation {
  Model& model;
  std::vector<std::vector<MyScalar>> states;
  std::vector<std::vector<MyScalar>> poses;
  std::vector<MyScalar> control;
  std::vector<MyScalar> next_state;

  GenericSimulation(Model& model, int world_count) : model(model) {
    std::vector<double> state = model.default_state();
    states.assign(world_count,
                  std::vector<MyScalar>(state.begin(), state.end()));
    poses.assign(world_count, std::vector<MyScalar>(model.poses_dim()));
    std::vector<double> c = model.default_control();
    control.assign(c.begin(), c.end());
    next_state.resize(model.state_dim());
  }

  void step_frame() override {
    for (std::size_t w = 0; w < states.size(); ++w) {
      for (int s = 0; s < Model::substeps; ++s) {
        model.step(states[w].data(), control.data(), next_state.data());
        std::swap(states[w], next_state);
      }
      model.poses(states[w].data(), poses[w].data());
    }
  }

  void get_pose(int world, int visual, float pose[7]) const override {
    for (int k = 0; k < 7; ++k) {
      pose[k] = static_cast<float>(poses[world][visual * 7 + k]);
    }
  }
};

// the generated kernels, all worlds at once
template <typename Float>
struct KernelSimulation : Simulation {
  tds::KernelBatch<Float> batch;

  KernelSimulation(const Model& model, int world_count,
                   tds::KernelBackend backend)
      : batch(g1_kernel_model(), world_count, backend) {
    batch.fill("state", model.default_state());
    batch.fill("control", model.default_control());
    batch.upload("state");
    batch.upload("control");
  }

  void step_frame() override {
    for (int s = 0; s < g1_kernels::substeps; ++s) {
      batch.launch("step");
      batch.swap("state", "next_state");
    }
    batch.launch("poses");
    batch.download("poses");
  }

  void get_pose(int world, int visual, float pose[7]) const override {
    const Float* poses = batch.host("poses");
    for (int k = 0; k < 7; ++k) {
      pose[k] = static_cast<float>(
          poses[static_cast<std::size_t>(visual * 7 + k) * batch.num_worlds() +
                world]);
    }
  }
};

int main(int argc, char* argv[]) {
  int num_x = 2;
  int num_y = 2;
  std::string backend = "generic";
  std::string precision = "float";
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if ((arg == "--num-x" || arg == "--num_x") && i + 1 < argc) {
      num_x = std::max(1, atoi(argv[++i]));
    } else if ((arg == "--num-y" || arg == "--num_y") && i + 1 < argc) {
      num_y = std::max(1, atoi(argv[++i]));
    } else if (arg == "--backend" && i + 1 < argc) {
      backend = argv[++i];
    } else if (arg == "--precision" && i + 1 < argc) {
      precision = argv[++i];
    }
  }
  const int world_count = num_x * num_y;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  visualizer.delete_all();

  Model model;

  std::unique_ptr<Simulation> sim;
  if (backend == "generic") {
    sim = std::make_unique<GenericSimulation>(model, world_count);
  } else {
    tds::KernelBackend kernel_backend =
        tds::kernel_backend_from_string(backend);
    if (precision == "double") {
      sim = std::make_unique<KernelSimulation<double>>(model, world_count,
                                                       kernel_backend);
    } else {
      sim = std::make_unique<KernelSimulation<float>>(model, world_count,
                                                      kernel_backend);
    }
    backend += " " + precision;
  }
  printf("Simulating %d G1 worlds with the %s backend\n", world_count,
         backend.c_str());

  {
    std::vector<int> shape_ids;
    std::string plane_filename;
    FileUtils::find_file("plane100.obj", plane_filename);
    TinyVector3f pos(0, 0, 0);
    TinyQuaternionf orn(0, 0, 0, 1);
    TinyVector3f scaling(1, 1, 1);
    visualizer.load_obj(plane_filename, pos, orn, scaling, shape_ids);
  }

  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string texture_path = "";
  auto urdf_structures = model.cache.retrieve(model.urdf_filename);
  FileUtils::extract_path(model.urdf_filename.c_str(), search_path,
                          TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);

  // one graphics instance per world and visual shape, instance_visuals maps
  // an instance to its visual in the simulation poses
  std::vector<std::vector<int>> instances(world_count);
  std::vector<int> instance_visuals;
  int visual_index = 0;
  auto create_instances = [&](const tds::UrdfLink<MyAlgebra>& link) {
    for (const auto& visual_shape : link.urdf_visual_shapes) {
      const auto& vis_link = visualizer.m_b2vis[visual_shape.visual_shape_uid];
      for (int v = 0; v < vis_link.visual_shape_uids.size(); v++) {
        for (int w = 0; w < world_count; w++) {
          TinyVector3f pos(0, 0, 0);
          TinyQuaternionf orn(0, 0, 0, 1);
          TinyVector3f scaling(1, 1, 1);
          int instance =
              visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                  vis_link.visual_shape_uids[v], pos, orn,
                  vis_link.shape_colors[v], scaling, 1.f, false);
          instances[w].push_back(instance);
        }
        instance_visuals.push_back(visual_index);
      }
      visual_index++;
    }
  };
  create_instances(urdf_structures.base_links[0]);
  for (const auto& link : urdf_structures.links) {
    create_instances(link);
  }
  visualizer.m_opengl_app.m_renderer->rebuild_graphics_instances();
  assert(visual_index == model.num_visuals());

  // arrange the worlds in a grid for rendering
  const float spacing = 1.5f;
  auto camera = visualizer.m_opengl_app.m_renderer->get_active_camera();
  camera->set_camera_target_position(0, 0, 0.6);
  camera->set_camera_distance(0.5f + spacing * std::max(num_x, num_y));
  camera->set_camera_pitch(-20);
  camera->set_camera_yaw(30);

  tds::Stopwatch sim_timer;
  double sim_time = 0;
  int frame = 0;
  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
    sim_timer.start();
    sim->step_frame();
    sim_time += sim_timer.stop();
    if (++frame % 60 == 0) {
      printf("%s: %.3f ms per frame of %d worlds (%.0f world-frames/s)\n",
             backend.c_str(), 1e3 * sim_time / 60, world_count,
             60 * world_count / sim_time);
      sim_time = 0;
    }

    for (int w = 0; w < world_count; ++w) {
      float offset_x = spacing * (w % num_x - 0.5f * (num_x - 1));
      float offset_y = spacing * (w / num_x - 0.5f * (num_y - 1));
      for (int i = 0; i < instances[w].size(); i++) {
        float pose[7];
        sim->get_pose(w, instance_visuals[i], pose);
        TinyVector3f pos(pose[0] + offset_x, pose[1] + offset_y, pose[2]);
        TinyQuaternionf orn(pose[3], pose[4], pose[5], pose[6]);
        visualizer.m_opengl_app.m_renderer
            ->write_single_instance_transform_to_cpu(pos, orn, instances[w][i]);
      }
    }
    visualizer.render();
  }

  printf("finished\n");
  return EXIT_SUCCESS;
}
