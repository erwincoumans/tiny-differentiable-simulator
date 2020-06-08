// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for
#include <type_traits>

#include <ceres/ceres.h>

#include "assert.h"
#include "fix64_scalar.h"
#include "tiny_dual.h"
#include "tiny_matrix3x3.h"
#include "tiny_quaternion.h"
#include "tiny_smooth_constraint_solver.h"
#include "tiny_vector3.h"
#include "tiny_world.h"

#include "ceres_utils.h"
#include "tiny_double_utils.h"
#include "tiny_dual_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_rigid_body.h"

#include "pybullet_visualizer_api.h"

std::string sphere2red;

typedef PyBulletVisualizerAPI Visualizer;
Visualizer *visualizer = nullptr;

const int TARGET_ID = 5;
const int kNumSteps = 300;

template <typename TinyScalar, typename TinyConstants>
TinyScalar rollout(TinyScalar force_x, TinyScalar force_y, bool smooth,
                   Visualizer *vis = nullptr,
                   TinyScalar *target_end_x = nullptr,
                   TinyScalar *target_end_y = nullptr,
                   TinyScalar *desired_x = nullptr,
                   TinyScalar *desired_y = nullptr,
                   TinyScalar dt = TinyConstants::fraction(1, 60)) {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef TinyRigidBody<TinyScalar, TinyConstants> TinyRigidBody;
  typedef TinyGeometry<TinyScalar, TinyConstants> TinyGeometry;

  std::vector<int> visuals;
  TinyVector3 target(TinyConstants::fraction(35, 10),
                     TinyConstants::fraction(8, 1), TinyConstants::zero());
  if (desired_x) {
    *desired_x = target.m_x;
  }
  if (desired_y) {
    *desired_y = target.m_y;
  }
  if (vis) {
    vis->resetSimulation();
  }

  TinyScalar gravity_z = TinyConstants::zero();
  TinyWorld<TinyScalar, TinyConstants> world(gravity_z);
  world.default_friction = TinyConstants::half();
  delete world.m_constraint_solver;
  if (smooth) {
    world.m_constraint_solver =
        new TinySmoothConstraintSolver<TinyScalar, TinyConstants>;
  } else {
    world.m_constraint_solver =
        new TinyConstraintSolver<TinyScalar, TinyConstants>;
  }

  std::vector<TinyRigidBody *> bodies;

  // set up billiard balls
  TinyScalar radius = TinyConstants::half();
  TinyScalar mass = TinyConstants::one();
  TinyScalar deg_60 =
      TinyConstants::pi() / TinyConstants::fraction(3, 1);  // even triangle
  TinyScalar dx = TinyConstants::cos1(deg_60) * radius * TinyConstants::two();
  TinyScalar dy = TinyConstants::sin1(deg_60) * radius * TinyConstants::two();
  TinyScalar rx = TinyConstants::zero(), y = TinyConstants::zero();
  int ball_id = 0;
  for (int column = 1; column <= 3; ++column) {
    TinyScalar x = rx;
    for (int i = 0; i < column; ++i) {
      const TinyGeometry *geom = world.create_sphere(radius);
      TinyRigidBody *body = world.create_rigid_body(mass, geom);
      body->m_world_pose.m_position.setValue(x, y, TinyConstants::zero());
      bodies.push_back(body);
      if (vis) {
        b3RobotSimulatorLoadUrdfFileArgs args;
        args.m_startPosition.setX(TinyConstants::getDouble(x));
        args.m_startPosition.setY(TinyConstants::getDouble(y));
        int sphere_id = visualizer->loadURDF(sphere2red, args);
        visuals.push_back(sphere_id);
        if (ball_id == TARGET_ID) {
          b3RobotSimulatorChangeVisualShapeArgs vargs;
          vargs.m_objectUniqueId = sphere_id;
          vargs.m_hasRgbaColor = true;
          vargs.m_rgbaColor = btVector4(0, 0.6, 1, 1);
          vis->changeVisualShape(vargs);
        }
      }
      ++ball_id;
      x += radius * TinyConstants::two();
    }
    rx = rx - dx;
    y = y + dy;
  }

  // Create white ball
  TinyVector3 white(TinyConstants::zero(), -TinyConstants::two(),
                    TinyConstants::zero());
  const TinyGeometry *white_geom = world.create_sphere(radius);
  TinyRigidBody *white_ball = world.create_rigid_body(mass, white_geom);
  white_ball->m_world_pose.m_position.setValue(white.x(), white.y(), white.z());
  bodies.push_back(white_ball);
  white_ball->apply_central_force(
      TinyVector3::create(force_x, force_y, TinyConstants::zero()));

  if (vis) {
    {
      // visualize white ball
      b3RobotSimulatorLoadUrdfFileArgs args;
      args.m_startPosition.setX(TinyConstants::getDouble(white.x()));
      args.m_startPosition.setY(TinyConstants::getDouble(white.y()));
      args.m_startPosition.setZ(TinyConstants::getDouble(white.z()));
      int sphere_id = visualizer->loadURDF(sphere2red, args);
      visuals.push_back(sphere_id);
      b3RobotSimulatorChangeVisualShapeArgs vargs;
      vargs.m_objectUniqueId = sphere_id;
      vargs.m_hasRgbaColor = true;
      vargs.m_rgbaColor = btVector4(1, 1, 1, 1);
      vis->changeVisualShape(vargs);
    }

    {
      // visualize target
      b3RobotSimulatorLoadUrdfFileArgs args;
      args.m_startPosition.setX(TinyConstants::getDouble(target.x()));
      args.m_startPosition.setY(TinyConstants::getDouble(target.y()));
      args.m_startPosition.setZ(TinyConstants::getDouble(target.z()));
      int sphere_id = visualizer->loadURDF(sphere2red, args);
      visuals.push_back(sphere_id);
      b3RobotSimulatorChangeVisualShapeArgs vargs;
      vargs.m_objectUniqueId = sphere_id;
      vargs.m_hasRgbaColor = true;
      vargs.m_rgbaColor = btVector4(1, 0.6, 0, 0.8);
      vis->changeVisualShape(vargs);
    }
  }

  for (int i = 0; i < kNumSteps; i++) {
    world.step(dt);

    if (vis) {
      double dtd = TinyConstants::getDouble(dt);
      // update visualization
      std::this_thread::sleep_for(std::chrono::duration<double>(dtd));
      for (int b = 0; b < bodies.size(); b++) {
        const TinyRigidBody *body = bodies[b];
        int sphere_id = visuals[b];
        btVector3 base_pos(
            TinyConstants::getDouble(body->m_world_pose.m_position.getX()),
            TinyConstants::getDouble(body->m_world_pose.m_position.getY()),
            TinyConstants::getDouble(body->m_world_pose.m_position.getZ()));
        btQuaternion base_orn(
            TinyConstants::getDouble(body->m_world_pose.m_orientation.getX()),
            TinyConstants::getDouble(body->m_world_pose.m_orientation.getY()),
            TinyConstants::getDouble(body->m_world_pose.m_orientation.getZ()),
            TinyConstants::getDouble(body->m_world_pose.m_orientation.getW()));
        visualizer->resetBasePositionAndOrientation(sphere_id, base_pos,
                                                    base_orn);
      }
    }
  }

  // Compute error
  TinyVector3 delta = bodies[TARGET_ID]->m_world_pose.m_position - target;
  if (target_end_x) {
    *target_end_x = bodies[TARGET_ID]->m_world_pose.m_position.m_x;
  }
  if (target_end_y) {
    *target_end_y = bodies[TARGET_ID]->m_world_pose.m_position.m_y;
  }
  return delta.sqnorm();
}

struct CostFunctor {
  template <typename T>
  bool operator()(const T *const x, T *residual) const {
    bool smooth = true;
    T target_end_x(0.), target_end_y(0.);
    T desired_x(0.), desired_y(0.);
    // Sometimes this function gets called with parameters of type double,
    // sometimes with parameters of type ceres::Jet<double, 2>. Therefore,
    // we have to switch between the utility struct depending on input type.
    typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                               CeresUtils<2>>
        Utils;
    rollout<T, Utils>(x[0], x[1], smooth, nullptr, &target_end_x, &target_end_y,
                      &desired_x, &desired_y);
    residual[0] = desired_x - target_end_x;
    residual[1] = desired_y - target_end_y;
    return true;
  }
};

int main(int argc, char *argv[]) {
  TinyFileUtils::find_file("sphere2red.urdf", sphere2red);

  std::string connection_mode = "gui";

  using namespace std::chrono;

  visualizer = new Visualizer;
  visualizer->setTimeOut(1e30);
  printf("mode=%s\n", const_cast<char *>(connection_mode.c_str()));
  int mode = eCONNECT_GUI;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;
  visualizer->connect(mode);

  if (visualizer->canSubmitCommand()) {
    visualizer->resetSimulation();

    double init_force_x = 500., init_force_y = 400.;
    rollout<double, DoubleUtils>(init_force_x, init_force_y, true, visualizer);

    double x[2] = {init_force_x, init_force_y};
    ceres::Problem problem;
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 2, 2>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, x);
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    printf("Solution: [%.2f %.2f]\n", x[0], x[1]);
    fflush(stdout);

    rollout<double, DoubleUtils>(x[0], x[1], true, visualizer);
  }

  return 0;
}
