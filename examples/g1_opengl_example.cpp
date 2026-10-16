// Unitree G1 standing example, a reimplementation of the Newton example
// newton/examples/robot/example_robot_g1.py using Tiny Differentiable
// Simulator.
//
// A G1 humanoid (29 dof body + two 7 dof Dex3 hands) is dropped from 0.2 m
// above its standing height. Joint PD controllers (kp = 1000, kd = 10) hold
// the zero joint pose. Each world is simulated independently, the worlds are
// only offset for rendering.
//
// The model data/unitree_g1/g1_29dof_with_hand_rev_1_0.urdf is converted from
// the newton-assets unitree_g1 URDF by data/unitree_g1/convert_g1_urdf.py.
//
// Usage: g1_opengl_tiny_example [--num-x NX] [--num-y NY]
// simulates NX x NY copies of the G1, laid out in a grid along X and Y.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

std::string G1_URDF_NAME = "unitree_g1/g1_29dof_with_hand_rev_1_0.urdf";

#include "opengl_urdf_visualizer.h"

#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#include "urdf/urdf_cache.hpp"

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

// pelvis height when the feet touch the ground in the zero joint pose
const double pelvis_standing_height = 0.793;
// the Newton example places the robot 0.2 m higher
const double drop_height = 0.2;

template <typename Algebra>
struct G1Simulation {
  using Scalar = typename Algebra::Scalar;
  tds::UrdfCache<Algebra> cache;
  std::string m_urdf_filename;
  tds::World<Algebra> world;
  tds::MultiBody<Algebra>* system = nullptr;

  // 60 frames per second with 6 substeps, as in the Newton example
  int sim_substeps{6};
  Scalar dt{Algebra::from_double(1. / 360.)};

  // joint position control. The Newton example sets joint_target_ke = 500
  // and joint_target_kd = 10, but its MuJoCo solver also keeps the position
  // actuators of the G1 MJCF model in the USD (kp = 500), so the effective
  // stiffness is 1000.
  Scalar kp{Algebra::from_double(1000.)};
  Scalar kd{Algebra::from_double(10.)};
  // rotor inertia of each joint, the armature of the G1 MJCF/USD model
  Scalar armature{Algebra::from_double(0.01)};

  int input_dim() const { return system->dof() + system->dof_qd(); }
  int num_visuals() const {
    int num = system->X_visuals().size();
    for (const auto& link : *system) {
      num += link.X_visuals.size();
    }
    return num;
  }
  // q, qd and the world transform (position, quaternion) of each visual
  int output_dim() const { return input_dim() + num_visuals() * 7; }

  G1Simulation() {
    std::string plane_filename;
    tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
    cache.construct(plane_filename, world, false, false);
    tds::FileUtils::find_file(G1_URDF_NAME, m_urdf_filename);
    system = cache.construct(m_urdf_filename, world, false, true);
    world.default_friction = Algebra::from_double(0.75);
    world.get_mb_constraint_solver()->pgs_iterations_ = 50;
    set_joint_armature();
  }

  // Joint PD control is implicit by default: the PD torque uses the joint
  // state at the end of the time step, with semi-implicit Euler integration
  // qd' = qd + dt qdd and q' = q + dt qd':
  //   tau = kp (q_target - q') - kd qd'
  //       = kp (q_target - q - dt qd) - kd qd - (dt kd + dt^2 kp) qdd.
  // The last term acts as additional joint inertia, so it is added to the
  // joint armature, which forward dynamics and the contact solver (through
  // the mass matrix) take into account. Explicit PD, as in the laikago
  // example, is only stable for much smaller time steps with these gains.
  bool implicit_pd{true};

  void set_joint_armature() {
    Scalar joint_armature = armature;
    if (implicit_pd) {
      joint_armature += dt * kd + dt * dt * kp;
    }
    for (auto& link : *system) {
      link.armature = joint_armature;
    }
  }

  void compute_pd_torques() {
    for (const auto& link : *system) {
      if (link.joint_type == tds::JOINT_FIXED) continue;
      // the joint targets are zero, see JointDofConfig.target_pos in Newton
      const Scalar q_target = Algebra::zero();
      const Scalar q = system->q(link.q_index);
      const Scalar qd = system->qd(link.qd_index);
      Scalar force = implicit_pd ? kp * (q_target - q - dt * qd) - kd * qd
                                 : kp * (q_target - q) - kd * qd;
      system->tau_[link.qd_index - 6] = force;
    }
  }

  void step() {
    compute_pd_torques();
    tds::forward_dynamics(*system, world.get_gravity());
    system->clear_forces();

    integrate_euler_qdd(*system, dt);

    world.step(dt);

    tds::integrate_euler(*system, dt);
  }

  // simulates one frame, starting from the q, qd in v
  std::vector<Scalar> operator()(const std::vector<Scalar>& v) {
    assert(static_cast<int>(v.size()) == input_dim());
    system->initialize();
    for (int i = 0; i < system->dof(); ++i) {
      system->q(i) = v[i];
    }
    for (int i = 0; i < system->dof_qd(); ++i) {
      system->qd(i) = v[i + system->dof()];
    }
    for (int s = 0; s < sim_substeps; ++s) {
      step();
    }
    tds::forward_kinematics(*system, system->q(), system->qd());

    // copy q, qd, visual world transforms (for rendering) to output
    std::vector<Scalar> result(output_dim());
    int j = 0;
    for (int i = 0; i < system->dof(); ++i, ++j) {
      result[j] = system->q(i);
    }
    for (int i = 0; i < system->dof_qd(); ++i, ++j) {
      result[j] = system->qd(i);
    }
    auto write_transform = [&](const tds::Transform<Algebra>& tr) {
      auto orn = Algebra::matrix_to_quat(tr.rotation);
      result[j++] = tr.translation[0];
      result[j++] = tr.translation[1];
      result[j++] = tr.translation[2];
      result[j++] = Algebra::quat_x(orn);
      result[j++] = Algebra::quat_y(orn);
      result[j++] = Algebra::quat_z(orn);
      result[j++] = Algebra::quat_w(orn);
    };
    for (const auto& X_visual : system->X_visuals()) {
      write_transform(system->base_X_world() * X_visual);
    }
    for (const auto& link : *system) {
      for (const auto& X_visual : link.X_visuals) {
        write_transform(link.X_world * X_visual);
      }
    }
    return result;
  }
};

int main(int argc, char* argv[]) {
  int num_x = 2;
  int num_y = 2;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if ((arg == "--num-x" || arg == "--num_x") && i + 1 < argc) {
      num_x = std::max(1, atoi(argv[++i]));
    } else if ((arg == "--num-y" || arg == "--num_y") && i + 1 < argc) {
      num_y = std::max(1, atoi(argv[++i]));
    }
  }
  const int world_count = num_x * num_y;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  visualizer.delete_all();

  G1Simulation<MyAlgebra> sim;

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
  auto urdf_structures = sim.cache.retrieve(sim.m_urdf_filename);
  FileUtils::extract_path(sim.m_urdf_filename.c_str(), search_path,
                          TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);

  // one graphics instance per world and visual shape, instance_visuals maps
  // an instance to its visual transform in the simulation output
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
  assert(visual_index == sim.num_visuals());

  // arrange the worlds in a grid for rendering
  const float spacing = 1.5f;
  auto camera = visualizer.m_opengl_app.m_renderer->get_active_camera();
  camera->set_camera_target_position(0, 0, 0.6);
  camera->set_camera_distance(0.5f + spacing * std::max(num_x, num_y));
  camera->set_camera_pitch(-20);
  camera->set_camera_yaw(30);

  std::vector<std::vector<MyScalar>> states(
      world_count, std::vector<MyScalar>(sim.input_dim(), MyScalar(0)));
  for (auto& state : states) {
    // identity base orientation, joint angles and velocities are zero
    state[3] = 1;
    state[6] = pelvis_standing_height + drop_height;
  }
  std::vector<std::vector<MyScalar>> outputs(world_count);

  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
    for (int w = 0; w < world_count; ++w) {
      outputs[w] = sim(states[w]);
      std::copy(outputs[w].begin(), outputs[w].begin() + sim.input_dim(),
                states[w].begin());
    }

    for (int w = 0; w < world_count; ++w) {
      float offset_x = spacing * (w % num_x - 0.5f * (num_x - 1));
      float offset_y = spacing * (w / num_x - 0.5f * (num_y - 1));
      for (int i = 0; i < instances[w].size(); i++) {
        const MyScalar* tr =
            &outputs[w][sim.input_dim() + instance_visuals[i] * 7];
        TinyVector3f pos(tr[0] + offset_x, tr[1] + offset_y, tr[2]);
        TinyQuaternionf orn(tr[3], tr[4], tr[5], tr[6]);
        visualizer.m_opengl_app.m_renderer
            ->write_single_instance_transform_to_cpu(pos, orn, instances[w][i]);
      }
    }
    visualizer.render();
  }

  printf("finished\n");
  return EXIT_SUCCESS;
}
