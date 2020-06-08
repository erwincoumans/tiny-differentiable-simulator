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

#include <ct/optcon/optcon.h>

#include "examples/cppad_utils.h"
#include "examples/tiny_traj_opt.h"
#include "examples/visualizer_api.h"
#include "testing/base/public/gunit.h"
#include "tiny_double_utils.h"

#define GOOGLE3
#ifdef GOOGLE3
#include "devtools/build/runtime/get_runfiles_dir.h"
#endif

namespace {

class TinyRigidBodyTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

TEST_F(TinyRigidBodyTest, TestCartpoleSwingUp) {
  using namespace ct::core;
  using namespace ct::optcon;
#ifdef GOOGLE3
  const std::string search_path = ::devtools_build::GetDataDependencyFilepath(
      "google3/third_party/bullet/examples/pybullet/gym/pybullet_data");
#else
  const std::string search_path =
      "d:/develop/bullet3/examples/pybullet/gym/pybullet_data";
#endif
  std::string urdf_filename = "cartpole.urdf";

  VisualizerAPI* sim_api = new VisualizerAPI();
  EXPECT_TRUE(sim_api->connect(eCONNECT_DIRECT));
  sim_api->setAdditionalSearchPath(search_path.c_str());

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

  typedef TinyControlledSystem<q_dim, qd_dim, control_dim, double, DoubleUtils>
      Cartpole;
  typedef TinyControlledSystem<q_dim, qd_dim, control_dim, ct::core::ADScalar,
                               CppADUtils<>>
      CartpoleAD;
  typedef ct::optcon::TermQuadratic<state_dim, control_dim> CostTerm;
  typedef CostFunctionAnalytical<state_dim, control_dim> CostFunction;
  typedef ct::core::AutoDiffLinearizer<state_dim, control_dim> Linearizer;

  typedef ControlInputConstraint<state_dim, control_dim> ControlConstraint;
  typedef StateConstraint<state_dim, control_dim> StateConstraint;
  typedef ConstraintContainerAnalytical<state_dim, control_dim>
      ConstraintContainer;

  SystemConstructor constructor(urdf_filename);
  auto cartpole_dynamics = std::make_shared<Cartpole>(constructor, dt);
  cartpole_dynamics->initialize(sim_api, sim_api);
  cartpole_dynamics->m_system->m_integration_type = INT_EULER;
  auto cartpole_dynamics_ad = std::make_shared<CartpoleAD>(constructor, dt);
  cartpole_dynamics_ad->initialize(sim_api, sim_api);
  cartpole_dynamics_ad->m_system->m_integration_type = INT_EULER;

  auto linearizer = std::make_shared<Linearizer>(cartpole_dynamics_ad);

  auto intermediate_cost = std::make_shared<CostTerm>();
  auto final_cost = std::make_shared<CostTerm>();
  bool verbose = true;
  const std::string our_search_path =
      ::devtools_build::GetDataDependencyFilepath(
          "google3/experimental/users/erwincoumans/differentiable/"
          "TinyRigidBody/data/");
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
  printf("iLQR optimization took %lld microseconds.", duration.count());

  // STEP 4: retrieve the solution
  ct::core::StateFeedbackController<state_dim, control_dim> controller =
      iLQR.getSolution();

  // check if the goal state is reached at the end of the optimized trajectory
  for (int i = 0; i < state_dim; ++i) {
    EXPECT_NEAR(controller.x_ref().back()[i], 0., 5e-2);
  }

  sim_api->disconnect();
  delete sim_api;
}

}  // namespace
