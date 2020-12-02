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

#include "assert.h"
#include "world.hpp"

#include "math/tiny/tiny_dual.h"
#include "math/tiny/tiny_matrix3x3.h"
#include "math/tiny/tiny_quaternion.h"
#include "math/tiny/tiny_vector3.h"
#include "math/tiny/fix64_scalar.h"

#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "utils/pendulum.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "multi_body.hpp"
#include "world.hpp"

using namespace TINY;
using namespace tds;
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "math/tiny/tiny_algebra.hpp"

#ifdef USE_CERES
#include <ceres/autodiff_cost_function.h>
#include "math/tiny/ceres_utils.h"
#endif //USE_CERES
#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for



#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_dual_double_utils.h"
#include "math/pose.hpp"
#include "multi_body.hpp"
#include "rigid_body.hpp"

#include "utils/file_utils.hpp"


std::string sphere2red;


// ID of the ball whose position is optimized for
const int TARGET_ID = 5;

template <typename Algebra>
typename Algebra::Scalar rollout(typename Algebra::Scalar force_x, typename Algebra::Scalar force_y, int steps = 300,
                   TinyOpenGL3App* app=0,
    typename Algebra::Scalar dt = Algebra::fraction(1, 60)) {

  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  typedef RigidBody<Algebra> TinyRigidBody;
  typedef Geometry<Algebra> TinyGeometry;

  std::vector<int> visuals;
  Vector3 target(Algebra::fraction(35, 10),Algebra::fraction(8, 1), Algebra::zero());
  int sphere_shape = -1;
  if (app) {
      app->m_instancingRenderer->remove_all_instances();
      sphere_shape = app->register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
  }

  Scalar gravity_z = Algebra::zero();
  World<Algebra> world(gravity_z);

  std::vector<TinyRigidBody*> bodies;

  Scalar radius = Algebra::half();
  Scalar mass = Algebra::one();
  Scalar deg_60 =
      Algebra::pi() / Algebra::fraction(3, 1);  // even triangle
  Scalar dx = Algebra::cos(deg_60) * radius * Algebra::two();
  Scalar dy = Algebra::sin(deg_60) * radius * Algebra::two();
  Scalar rx = Algebra::zero(), y = Algebra::zero();
  int ball_id = 0;
  for (int column = 1; column <= 3; ++column) {
    Scalar x = rx;
    for (int i = 0; i < column; ++i) {
      const TinyGeometry* geom = world.create_sphere(radius);
      TinyRigidBody* body = world.create_rigid_body(mass, geom);
      body->world_pose_.position_ =
          Vector3::create(x, y, Algebra::zero());
      bodies.push_back(body);
      if (app) {
          TinyVector3f pos(Algebra::to_double(x), Algebra::to_double(y), Algebra::to_double(Algebra::zero()));
          TinyQuaternionf orn(0, 0, 0, 1);
          TinyVector3f color(0, 1, 0);
          if (ball_id == TARGET_ID)
            color = TinyVector3f(1, 0, 0);
          TinyVector3f scaling(0.5, 0.5, 0.5);
          int instance = app->m_renderer->register_graphics_instance(sphere_shape, pos, orn, color, scaling);
          visuals.push_back(instance);
      }
      ++ball_id;
      x += radius * Algebra::two();
    }
    rx = rx - dx;
    y = y + dy;
  }

  // Create white ball
  Vector3 white = Vector3::create(
      Algebra::zero(), -Algebra::two(), Algebra::zero());
  const TinyGeometry* white_geom = world.create_sphere(radius);
  TinyRigidBody* white_ball = world.create_rigid_body(mass, white_geom);
  white_ball->world_pose_.position_ =
      Vector3::create(white.x(), white.y(), white.z());
  bodies.push_back(white_ball);
  white_ball->apply_central_force(
      Vector3::create(force_x, force_y, Algebra::zero()));

  if (app) {
      {
          TinyVector3f pos(Algebra::to_double(white.x()), Algebra::to_double(white.y()),
              Algebra::to_double(white.z()));
          TinyQuaternionf orn(0, 0, 0, 1);
          TinyVector3f color(1, 1, 1);
          TinyVector3f scaling(0.5, 0.5, 0.5);
          int instance = app->m_renderer->register_graphics_instance(sphere_shape, pos, orn, color, scaling);
          visuals.push_back(instance);
      }
      {
          TinyVector3f pos(Algebra::to_double(target.x()), Algebra::to_double(target.y()),
              Algebra::to_double(target.z()));
          TinyQuaternionf orn(0, 0, 0, 1);
          TinyVector3f color(0, 0, 1);
          TinyVector3f scaling(0.5, 0.5, 0.5);
          int instance = app->m_renderer->register_graphics_instance(sphere_shape, pos, orn, color, scaling);
          visuals.push_back(instance);
      }

  }

  for (int i = 0; i < steps; i++) {
    
    world.step(dt);
    int upAxis = 2;
    if (app) {

        app->m_renderer->update_camera(upAxis);
        DrawGridData data;
        data.drawAxis = true;
        data.upAxis = upAxis;
        app->draw_grid(data);
      double dtd = Algebra::to_double(dt);
      // update visualization
      std::this_thread::sleep_for(std::chrono::duration<double>(dtd));

      for (int b = 0; b < bodies.size(); b++) {
        const TinyRigidBody* body = bodies[b];
        int sphere_id = visuals[b];
        TinyVector3f base_pos(
            Algebra::to_double(body->world_pose_.position_.getX()),
            Algebra::to_double(body->world_pose_.position_.getY()),
            Algebra::to_double(body->world_pose_.position_.getZ()));
        TinyQuaternionf base_orn(
            Algebra::to_double(body->world_pose_.orientation_.getX()),
            Algebra::to_double(body->world_pose_.orientation_.getY()),
            Algebra::to_double(body->world_pose_.orientation_.getZ()),
            Algebra::to_double(body->world_pose_.orientation_.getW()));
        app->m_instancingRenderer->write_single_instance_transform_to_cpu(base_pos, base_orn, sphere_id);
      }

      app->m_renderer->render_scene();
      app->m_renderer->write_transforms();
      app->swap_buffer();
    }
  }

  // Compute error
  Vector3 delta = bodies[TARGET_ID]->world_pose_.position_ - target;
  return delta.sqnorm();
}

// Computes gradient using finite differences
void grad_finite(double force_x, double force_y, double* cost,
                 double* d_force_x, double* d_force_y, int steps = 300,
                 double eps = 1e-5) {
  *cost = rollout<TinyAlgebra<double, DoubleUtils>>(force_x, force_y, steps);
  double cx = rollout< TinyAlgebra<double, DoubleUtils>>(force_x + eps, force_y, steps);
  double cy = rollout< TinyAlgebra<double, DoubleUtils>>(force_x, force_y + eps, steps);
  *d_force_x = (cx - *cost) / eps;
  *d_force_y = (cy - *cost) / eps;
}

// void grad_stan(double force_x, double force_y, double* cost, double*
// d_force_x,
//               double* d_force_y, int steps = 300, double eps = 1e-5) {
//  standouble fx = force_x;
//  fx.d_ = 1;
//  standouble fy = force_y;
//
//  standouble c = rollout<standouble, StanDoubleUtils>(fx, fy, steps);
//  *cost = c.val();
//  *d_force_x = c.tangent();
//
//  fx.d_ = 0;
//  fy.d_ = 1;
//  c = rollout<standouble, StanDoubleUtils>(fx, fy, steps);
//  *d_force_y = c.tangent();
//}

void grad_dual(double force_x, double force_y, double* cost, double* d_force_x,
               double* d_force_y, int steps = 300, double eps = 1e-5) {
  typedef TinyDual<double> TinyDual;
  {
    TinyDual fx(force_x, 1.);
    TinyDual fy(force_y, 0.);

    TinyDual c = rollout< TinyAlgebra<TinyDual, TinyDualDoubleUtils>>(fx, fy, steps);
    *cost = c.real();
    *d_force_x = c.dual();
  }
  {
    TinyDual fx(force_x, 0.);
    TinyDual fy(force_y, 1.);

    TinyDual c = rollout< TinyAlgebra<TinyDual, TinyDualDoubleUtils>>(fx, fy, steps);
    *d_force_y = c.dual();
  }
}

#ifdef USE_CERES

struct CeresFunctional {
  int steps{300};

  template <typename T>
  bool operator()(const T* const x, T* e) const {
    typedef ceres::Jet<double, 2> Jet;
    T fx(x[0]), fy(x[1]);
    typedef std::conditional_t<std::is_same_v<T, double>, DoubleUtils,
                               CeresUtils<2>>
        Utils;
    *e = rollout<TinyAlgebra<T, Utils> >(fx, fy, steps);
    return true;
  }
};


ceres::AutoDiffCostFunction<CeresFunctional, 1, 2> cost_function(
    new CeresFunctional);
double* parameters = new double[2];
double* gradient = new double[2];

void grad_ceres(double force_x, double force_y, double* cost, double* d_force_x,
                double* d_force_y, int steps = 300) {
  parameters[0] = force_x;
  parameters[1] = force_y;
  double const* const* params = &parameters;
  cost_function.Evaluate(params, cost, &gradient);
  *d_force_x = gradient[0];
  *d_force_y = gradient[1];
}
#endif //USE_CERES

int main(int argc, char* argv[]) {
  FileUtils::find_file("sphere2red.urdf", sphere2red);
  using namespace std::chrono;

  TinyOpenGL3App app("billiard_opt_example_gui", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);


  double init_force_x = 0., init_force_y = 500.;
  int steps = 300;
  rollout< TinyAlgebra<double, DoubleUtils>>(init_force_x, init_force_y, steps, &app);

  {
    auto start = high_resolution_clock::now();
    double cost, d_force_x, d_force_y;
    double learning_rate = 1e2;
    double force_x = init_force_x, force_y = init_force_y;
    for (int iter = 0; iter < 50; ++iter) {
      grad_finite(force_x, force_y, &cost, &d_force_x, &d_force_y, steps);
      printf("Iteration %02d - cost: %.3f \tforce: [%.2f %2.f]\n", iter, cost,
             force_x, force_y);
      force_x -= learning_rate * d_force_x;
      force_y -= learning_rate * d_force_y;
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    printf("Finite differences took %ld microseconds.",
           static_cast<long>(duration.count()));
    rollout< TinyAlgebra<double, DoubleUtils>>(force_x, force_y, steps, &app);
  }
  
  {
    auto start = high_resolution_clock::now();
    double cost, d_force_x, d_force_y;
    double learning_rate = 1e2;
    double force_x = init_force_x, force_y = init_force_y;
    for (int iter = 0; iter < 50; ++iter) {
      grad_dual(force_x, force_y, &cost, &d_force_x, &d_force_y, steps);
      printf("Iteration %02d - cost: %.3f \tforce: [%.2f %2.f]\n", iter, cost,
             force_x, force_y);
      force_x -= learning_rate * d_force_x;
      force_y -= learning_rate * d_force_y;
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    printf("TinyDual took %ld microseconds.",
           static_cast<long>(duration.count()));
    rollout< TinyAlgebra<double, DoubleUtils>>(force_x, force_y, steps, &app);
  }
  

#ifdef USE_CERES
  {
    auto start = high_resolution_clock::now();
    double cost, d_force_x, d_force_y;
    double learning_rate = 1e2;
    double force_x = init_force_x, force_y = init_force_y;
    for (int iter = 0; iter < 50; ++iter) {
      grad_ceres(force_x, force_y, &cost, &d_force_x, &d_force_y, steps);
      printf("Iteration %02d - cost: %.3f \tforce: [%.2f %2.f]\n", iter, cost,
             force_x, force_y);
      force_x -= learning_rate * d_force_x;
      force_y -= learning_rate * d_force_y;
    }
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    printf("Ceres Jet took %ld microseconds.",
           static_cast<long>(duration.count()));
    rollout<TinyAlgebra<double, DoubleUtils> >(force_x, force_y, steps, &app);
  }
#endif //USE_CERES

  
  return 0;
}
