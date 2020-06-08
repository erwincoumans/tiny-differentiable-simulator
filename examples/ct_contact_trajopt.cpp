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

#include <assert.h>
#include <fenv.h>
#include <stdio.h>
#include <string.h>

#include <chrono>

#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_mb_constraint_solver_spring.h"
#include "tiny_system_constructor.h"
#include "tiny_traj_opt.h"

using namespace ct::core;
using namespace ct::optcon;

//#define USE_HOPPER

int main(int argc, char* argv[]) {
  std::string connection_mode = "gui";
  std::string plane_urdf_filename;

  std::string system_urdf_filename;
#ifdef USE_HOPPER
  TinyFileUtils::find_file("hopper_link0_1.urdf", plane_urdf_filename);
#else
  TinyFileUtils::find_file("cheetah_link0_1.urdf", plane_urdf_filename);
#endif

  TinyFileUtils::find_file("plane_implicit.urdf", plane_urdf_filename);

  char search_path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(plane_urdf_filename.c_str(), search_path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string our_search_path = search_path;

  // Set NaN trap
  // TODO investigate: NANs are encountered with the HPIPM solver but not with
  //  the Riccati solver.
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

  printf("urdf_filename=%s\n", system_urdf_filename.c_str());

  VisualizerAPI* vis_api = new VisualizerAPI();
  printf("connection_mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_SHARED_MEMORY;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "gui") mode = eCONNECT_GUI;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;
  vis_api->setTimeOut(1e30);
  vis_api->connect(mode);
  vis_api->resetSimulation();

  VisualizerAPI* sim_api = new VisualizerAPI();
  sim_api->setTimeOut(1e30);
  sim_api->connect(eCONNECT_DIRECT);

#ifdef USE_HOPPER
  const int q_dim = 6;
  const int qd_dim = 6;
  const int state_dim = q_dim + qd_dim;
  const int control_dim = 3;
  /**
   * Define indices of controllable degrees of freedom.
   */
  std::vector<int> control_indices{3, 4, 5};
#else
  const int q_dim = 9;
  const int qd_dim = 9;
  const int state_dim = q_dim + qd_dim;
  const int control_dim = 6;
  /**
   * Define indices of controllable degrees of freedom.
   */
  std::vector<int> control_indices{3, 4, 5, 6, 7, 8};
#endif

  // Load solver settings from file
  NLOptConSettings ilqr_settings;

#ifdef USE_HOPPER
  ilqr_settings.load(our_search_path + "hopper_opt_settings.info", true);
#else
  ilqr_settings.load(our_search_path + "cheetah_opt_settings.info", true);
#endif

  const double dt = ilqr_settings.dt;

  ct::core::Time time_horizon = 2.5;  // final time horizon in [sec]

  // verify settings
  bool ok = ilqr_settings.parametersOk();
  fflush(stdout);
  assert(ok);

  ilqr_settings.print();

  /* provide an initial guess */
  // calculate the number of time steps K
  size_t K = ilqr_settings.computeK(time_horizon);

  typedef TinyControlledSystem<q_dim, control_dim, double, DoubleUtils>
      Dynamics;

  Dynamics::State x0;
  x0.setZero();
  x0[1] = -0.05;  // initial z coordinate (should not fall from a large height)

  typedef TermQuadratic<state_dim, control_dim> CostTerm;
  typedef CostFunctionAnalytical<state_dim, control_dim> CostFunction;

  // TODO implement linearizer using autodiff. CT uses CppAD that requires
  //   us to have no conditions in the code, or use special functions, such as
  //   CppAD::CondExpLt. Therefore, we can use CT's autodiff linearizer only
  //   for scenarios without contact at the moment.

  typedef ControlInputConstraint<state_dim, control_dim> ControlConstraint;
  typedef StateConstraint<state_dim, control_dim> StateConstraint;
  typedef ConstraintContainerAnalytical<state_dim, control_dim>
      ConstraintContainer;

  // TinySystemConstructor<TinyActuator> constructor(system_urdf_filename,
  //                                                 plane_urdf_filename);
  // constructor.m_actuator = new TinyActuator<double,
  // DoubleUtils>(control_dim);
  TinySystemConstructor<TinyServoActuator> constructor(system_urdf_filename,
                                                       plane_urdf_filename);
  constructor.m_actuator = new TinyServoActuator<double, DoubleUtils>(
      control_dim, 50., 3., -500., 500.);
  constructor.m_control_indices = control_indices;
  for (int i = 0; i < control_dim; ++i) {
    constructor.m_actuator->limits.push_back(1500.);
  }

  auto dynamics = std::make_shared<Dynamics>(constructor, dt);
  dynamics->initialize(sim_api, vis_api);
  dynamics->use_soft_contact = false;
  // auto* soft_contact_model =
  //     new TinyMultiBodyConstraintSolverSpring<double, DoubleUtils>;
  // soft_contact_model->spring_k = 5000;
  // soft_contact_model->damper_d = 500;
  // dynamics->m_world.m_mb_constraint_solver = soft_contact_model;

  auto intermediate_cost = std::make_shared<CostTerm>();
  auto final_cost = std::make_shared<CostTerm>();
  bool verbose = true;
#ifdef USE_HOPPER
  intermediate_cost->loadConfigFile(our_search_path + "hopper_cost.info",
                                    "intermediateCost", verbose);
  final_cost->loadConfigFile(our_search_path + "hopper_cost.info", "finalCost",
                             verbose);
#else
  intermediate_cost->loadConfigFile(our_search_path + "cheetah_cost.info",
                                    "intermediateCost", verbose);
  final_cost->loadConfigFile(our_search_path + "cheetah_cost.info", "finalCost",
                             verbose);
#endif

  //  while (true) dynamics->playback_physics(vis_api, 3., x0);

  // Since we are using quadratic cost function terms in this example, the first
  // and second order derivatives are immediately known and we define the cost
  // function to be an "Analytical Cost Function". Let's create the
  // corresponding object and add the previously loaded intermediate and final
  // term.
  auto cost_function = std::make_shared<CostFunction>();
  cost_function->addIntermediateTerm(intermediate_cost);
  cost_function->addFinalTerm(final_cost);

  // create and initialize an "optimal control problem"
  ContinuousOptConProblem<state_dim, control_dim> optConProblem(
      time_horizon, x0, dynamics, cost_function);

  /* design trivial initial controller for iLQR. Note that in this simple
   * example, we can simply use zero feedforward with zero feedback gains
   around  the initial position. In more complex examples, a more elaborate
   initial guess may be required.*/
  FeedbackArray<state_dim, control_dim> u0_fb(
      K, FeedbackMatrix<state_dim, control_dim>::Zero());
  ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
  StateVectorArray<state_dim> x_ref_init(K + 1, x0);
  // std::vector<double> q_desired(control_dim, 0.0);
  // dynamics->initialize_static_pd_control(u0_ff, q_desired, x0, 150, 3., -500,
  // 500);
  NLOptConSolver<state_dim, control_dim>::Policy_t initController(
      x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);
  //  while (true) dynamics->playback_physics(vis_api, time_horizon, x0, u0_ff);

  // create an NLOptConSolver instance
  NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);

  // set the initial guess
  iLQR.setInitialGuess(initController);

  // solve the optimal control problem
  using namespace std::chrono;
  auto start = high_resolution_clock::now();
  iLQR.solve();
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  printf("Trajectory optimization took %.6f seconds.\n",
         duration.count() / 1e6);

  // retrieve the solution
  ct::core::StateFeedbackController<state_dim, control_dim> controller =
      iLQR.getSolution();

  //  for (int i = 0; i < controller.x_ref().size(); ++i) {
  //    std::cout << " " << controller.x_ref()[i] << std::endl;
  //  }
  //  return 0;

  fflush(stdout);

  {
#if USE_HOPPER
    std::string model_name = "hopper";
#else
    std::string model_name = "cheetah";
#endif

    std::string traj_filename =
        "/home/eric/tiny-differentiable-simulator/python/plotting/traj_" +
        model_name + ".csv";
    std::ofstream traj_out(traj_filename);
    for (std::size_t i = 0; i < controller.x_ref().size(); ++i) {
      traj_out << (i * dt) << '\t';
      // state
      for (int j = 0; j < state_dim; ++j) {
        traj_out << controller.x_ref()[i][j] << '\t';
      }
      // control
      if (i < controller.uff().size()) {
        for (int j = 0; j < control_dim; ++j) {
          traj_out << controller.uff()[i][j] << '\t';
        }
      } else {
        for (int j = 0; j < control_dim; ++j) {
          traj_out << 0. << '\t';
        }
      }
      traj_out << '\n';
    }
    std::cout << "Saved trajectory as " << traj_filename << "\n";
  }

  while (vis_api->isConnected()) {
    printf("Playing back controller with states...\n");
    dynamics->playback_controller(sim_api, vis_api, controller, true, 1.);
    std::this_thread::sleep_for(std::chrono::duration<double>(5.));
    printf("Playing back physics...\n");
    dynamics->playback_physics(vis_api, time_horizon, x0, controller.uff());
    std::this_thread::sleep_for(std::chrono::duration<double>(5.));
  }

  delete vis_api;
  delete sim_api;

  printf("exit\n");
  return EXIT_SUCCESS;
}
