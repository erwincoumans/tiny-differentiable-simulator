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

#include <fenv.h>
#include <stdio.h>

#include <chrono>
#include <cstddef>

#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_system_constructor.h"
#include "tiny_traj_opt.h"

using namespace ct::core;
using namespace ct::optcon;

int main(int argc, char *argv[]) {
  std::string system_urdf_filename;
  TinyFileUtils::find_file("cartpole.urdf", system_urdf_filename);
  char search_path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(system_urdf_filename.c_str(), search_path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string connection_mode = "gui";
  const std::string our_search_path = search_path;

  // Set NaN trap
  //   feenableexcept(FE_INVALID | FE_OVERFLOW);

  VisualizerAPI *vis_api = new VisualizerAPI();
  printf("connection_mode=%s\n", const_cast<char *>(connection_mode.c_str()));
  int mode = eCONNECT_SHARED_MEMORY;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "gui") mode = eCONNECT_GUI;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;
  vis_api->setTimeOut(1e30);
  vis_api->connect(mode);
  vis_api->resetSimulation();

  VisualizerAPI *sim_api = new VisualizerAPI();
  sim_api->setTimeOut(1e30);
  sim_api->connect(eCONNECT_DIRECT);

  const int Steps = 1000;
  constexpr double dt = 1. / 600.;

  /*
   * The cartpole state is 4-dimensional:
   *    x-coordinate of cart
   *    angle of pole
   *    x-coordinate of cart velocity
   *    angular velocity of pole
   */
  const int q_dim = 2;
  const int qd_dim = 2;
  const int state_dim = q_dim + qd_dim;
  /**
   * The control input is one-dimensional (the scalar force applied to the cart)
   */
  const int control_dim = 1;

  typedef TinyControlledSystem<q_dim, control_dim, double, DoubleUtils>
      Cartpole;
  typedef TinyControlledSystem<q_dim, control_dim, ct::core::ADScalar,
                               CppADUtils<>>
      CartpoleAD;
  typedef ct::optcon::TermQuadratic<state_dim, control_dim> CostTerm;
  typedef CostFunctionAnalytical<state_dim, control_dim> CostFunction;
  typedef ct::core::AutoDiffLinearizer<state_dim, control_dim> Linearizer;

  typedef ControlInputConstraint<state_dim, control_dim> ControlConstraint;
  typedef StateConstraint<state_dim, control_dim> StateConstraint;
  typedef ConstraintContainerAnalytical<state_dim, control_dim>
      ConstraintContainer;

  TinySystemConstructor<> constructor(system_urdf_filename);
  // only the first joint (cart position) is controllable
  constructor.m_control_indices = {0};

  auto cartpole_dynamics_ad = std::make_shared<CartpoleAD>(constructor, dt);
  cartpole_dynamics_ad->initialize(sim_api, sim_api);
  sim_api->resetSimulation();

  auto cartpole_dynamics = std::make_shared<Cartpole>(constructor, dt);
  cartpole_dynamics->initialize(sim_api, vis_api);

  auto linearizer = std::make_shared<Linearizer>(cartpole_dynamics_ad);

  auto intermediate_cost = std::make_shared<CostTerm>();
  auto final_cost = std::make_shared<CostTerm>();
  bool verbose = true;

  intermediate_cost->loadConfigFile(
      our_search_path + "cartpole_swingup_cost.info", "intermediateCost",
      verbose);
  final_cost->loadConfigFile(our_search_path + "cartpole_swingup_cost.info",
                             "finalCost", verbose);

  // Since we are using quadratic cost function terms in this example, the first
  // and second order derivatives are immediately known and we define the cost
  // function to be an "Analytical Cost Function". Let's create the
  // corresponding object and add the previously loaded intermediate and final
  // term.
  auto cost_function = std::make_shared<CostFunction>();
  cost_function->addIntermediateTerm(intermediate_cost);
  cost_function->addFinalTerm(final_cost);

  /* STEP 1-D: set up the box constraints for the control input*/
  // input box constraint boundaries with sparsities in constraint toolbox
  // format
  Eigen::VectorXi sp_control(control_dim);
  sp_control << 1;
  Eigen::VectorXd u_lb(control_dim);
  Eigen::VectorXd u_ub(control_dim);
  u_lb.setConstant(-10);
  u_ub = -u_lb;

  // constraint terms
  auto control_limits =
      std::make_shared<ControlConstraint>(u_lb, u_ub, sp_control);
  control_limits->setName("ControlInputBound");

  // input box constraint constraint container
  auto control_box_constraints = std::make_shared<ConstraintContainer>();

  // add and initialize constraint terms
  control_box_constraints->addIntermediateConstraint(control_limits, verbose);
  control_box_constraints->initialize();

  /* STEP 1-E: set up the box constraints for the states */
  // state box constraint boundaries with sparsities in constraint toolbox
  // format we put a box constraint on the velocity, hence the overall
  // constraint dimension is 1.
  Eigen::VectorXi sp_state(state_dim);
  sp_state << 1, 0, 0, 0;  // limit the x-coordinate of the cart
  Eigen::VectorXd x_lb(1);
  Eigen::VectorXd x_ub(1);
  x_lb.setConstant(-1);
  x_ub = -x_lb;
  // constraint terms
  auto state_limits = std::make_shared<StateConstraint>(x_lb, x_ub, sp_state);
  state_limits->setName("StateBound");

  // input box constraint constraint container
  auto state_box_constraints = std::make_shared<ConstraintContainer>();

  // add and initialize constraint terms
  state_box_constraints->addIntermediateConstraint(state_limits, verbose);
  state_box_constraints->initialize();

  /* STEP 1-F: initialization with initial state and desired time horizon */

  StateVector<state_dim> x0;
  x0.setZero();
  // initial cartpole angle
  x0[1] = M_PI;

  ct::core::Time time_horizon =
      Steps * dt;  // and a final time horizon in [sec]

  // STEP 1-G: create and initialize an "optimal control problem"
  ContinuousOptConProblem<state_dim, control_dim> optConProblem(
      time_horizon, x0, cartpole_dynamics, cost_function, linearizer);

#ifndef GOOGLE3
  // add the box constraints to the optimal control problem
  //! currently not supported with ct_optcon in //third_party
  optConProblem.setInputBoxConstraints(control_box_constraints);
  optConProblem.setStateBoxConstraints(state_box_constraints);
#endif

  /* STEP 2: set up a nonlinear optimal control solver. */

  /* STEP 2-A: Create the settings.
   * the type of solver, and most parameters, like number of shooting intervals,
   * etc., can be chosen using the following settings struct. Let's use, the
   * iterative linear quadratic regulator, iLQR, for this example. In the
   * following, we modify only a few settings, for more detail, check out the
   * NLOptConSettings class. */
  NLOptConSettings ilqr_settings;
  ilqr_settings.dt = dt;  // the control discretization in [sec]
  ilqr_settings.integrator = ct::core::IntegrationType::EULERCT;
  ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
  ilqr_settings.max_iterations = 100;
  ilqr_settings.nThreads = 1;
  ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
  ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;
  ilqr_settings.K_shot = 1;
  ilqr_settings.K_sim = 1;
  ilqr_settings.maxDefectSum = -1.;
  ilqr_settings.lineSearchSettings.type = LineSearchSettings::ARMIJO;
  //  ilqr_settings.lqocp_solver =
  //      NLOptConSettings::LQOCP_SOLVER::GNRICCATI_SOLVER;  // solve
  //      LQ-problems
  // using HPIPM
  ilqr_settings.lqoc_solver_settings.num_lqoc_iterations =
      10;  // number of riccati sub-iterations
  ilqr_settings.printSummary = true;

  /* STEP 2-B: provide an initial guess */
  // calculate the number of time steps K
  size_t K = ilqr_settings.computeK(time_horizon);

  /* design trivial initial controller for iLQR. Note that in this simple
   * example, we can simply use zero feedforward with zero feedback gains around
   * the initial position. In more complex examples, a more elaborate initial
   * guess may be required.*/
  FeedbackArray<state_dim, control_dim> u0_fb(
      K, FeedbackMatrix<state_dim, control_dim>::Zero());
  ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
  StateVectorArray<state_dim> x_ref_init(K + 1, x0);
  NLOptConSolver<state_dim, control_dim>::Policy_t initController(
      x_ref_init, u0_ff, u0_fb, ilqr_settings.dt);

  // STEP 2-C: create an NLOptConSolver instance
  NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);

  // set the initial guess
  iLQR.setInitialGuess(initController);

  // STEP 3: solve the optimal control problem
  using namespace std::chrono;
  auto start = high_resolution_clock::now();
  iLQR.solve();
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  printf("iLQR optimization took %ld microseconds.",
         static_cast<long>(duration.count()));

  // STEP 4: retrieve the solution
  ct::core::StateFeedbackController<state_dim, control_dim> controller =
      iLQR.getSolution();

  //  for (int i = 0; i < controller.x_ref().size(); ++i) {
  //    std::cout << " " << controller.x_ref()[i] << std::endl;
  //  }
  //  return 0;

  fflush(stdout);

  while (vis_api->isConnected()) {
    cartpole_dynamics->playback_controller(vis_api, vis_api, controller, false,
                                           1.);
    std::this_thread::sleep_for(std::chrono::duration<double>(10.));
  }

  delete vis_api;
  delete sim_api;

  printf("exit\n");
}
