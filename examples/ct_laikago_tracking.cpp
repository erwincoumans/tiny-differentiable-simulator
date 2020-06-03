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

#include "motion_import.h"
#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_mb_constraint_solver_spring.h"
#include "tiny_system_constructor.h"
#include "tiny_traj_opt.h"

using namespace ct::core;
using namespace ct::optcon;

// we convert quaternions to Euler angles to have 6D floating-base coordinates
#define quat_integration false
const int q_dim = 18 + quat_integration;
const int qd_dim = 18;
const int state_dim = q_dim + qd_dim;
const int control_dim = 12;

bool use_contact = true;
// initial z translation
double initial_z = 0.48;
double control_limit = 30.;
#define use_servo_actuator true
#define symplectic_system false
std::string motion_file = "laikago_dance_sidestep0";
// std::string motion_file = "laikago_builtin_walk_inplace";
bool initialize_from_pd_control = true;
bool use_spring_contact = true;
bool playback_controller_with_uff = false;  // or just play back states

int main(int argc, char *argv[]) {
  std::string connection_mode = "gui";

  std::string plane_urdf_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_urdf_filename);

  std::string system_urdf_filename;
  TinyFileUtils::find_file("laikago/laikago_toes_zup.urdf",
                           system_urdf_filename);

  char search_path[TINY_MAX_EXE_PATH_LEN];
  TinyFileUtils::extract_path(system_urdf_filename.c_str(), search_path,
                              TINY_MAX_EXE_PATH_LEN);
  std::string our_search_path = search_path;
  std::string settings_filename =
      our_search_path + "laikago_tracking_settings.info";

  boost::property_tree::ptree exp_settings;
  boost::property_tree::read_info(settings_filename, exp_settings);
  double joint_damping = 0.;
  size_t K = 10000;
  try {
    use_contact = exp_settings.get<bool>("experiment.use_contact");
    use_spring_contact = !exp_settings.get<bool>("experiment.hard_contact");
    joint_damping = exp_settings.get<double>("experiment.joint_damping");
    initial_z = exp_settings.get<double>("experiment.initial_z");
    control_limit = exp_settings.get<double>("experiment.control_limit");
    motion_file = exp_settings.get<std::string>("experiment.motion_file");
    initialize_from_pd_control =
        exp_settings.get<bool>("experiment.initialize_from_pd_control");
    playback_controller_with_uff =
        exp_settings.get<bool>("experiment.playback_controller_with_uff");
    K = static_cast<std::size_t>(exp_settings.get<int>("experiment.K"));
    printf("Successfully loaded experiment settings from %s.\n",
           settings_filename.c_str());
  } catch (...) {
    fprintf(stderr, "Failed to load experiment settings from %s.\n",
            settings_filename.c_str());
  }

  // Set NaN trap
  // TODO investigate: NANs are encountered with the HPIPM solver but not with
  //  the Riccati solver.
  //     feenableexcept(FE_INVALID | FE_OVERFLOW);

  printf("urdf_filename=%s\n", system_urdf_filename.c_str());

  auto *vis_api = new VisualizerAPI();
  printf("connection_mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_SHARED_MEMORY;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "gui") mode = eCONNECT_GUI;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;
  vis_api->setTimeOut(1e30);
  vis_api->connect(mode);
  vis_api->resetSimulation();

  auto *sim_api = new VisualizerAPI();
  sim_api->setTimeOut(1e30);
  sim_api->connect(eCONNECT_DIRECT);

  /**
   * Define indices of controllable degrees of freedom.
   */
  std::vector<int> control_indices(control_dim);
  for (int i = 0; i < control_dim; ++i) {
    control_indices[i] = i;
  }

  // Load solver settings from file
  NLOptConSettings ilqr_settings;
  ilqr_settings.load(settings_filename, true);

  const double dt = ilqr_settings.dt;

  // load reference trajectory from motion file
  Motion reference;
  bool load_success = Motion::load_from_file(
      our_search_path + motion_file + ".txt", &reference);
  assert(load_success);

  // verify settings
  bool ok = ilqr_settings.parametersOk();
  fflush(stdout);
  assert(ok);

  ilqr_settings.print();

  /* provide an initial guess */
  // calculate the number of time steps K
  //  size_t K = 10000;  //ilqr_settings.computeK(time_horizon);

  // final time horizon in [sec]
  ct::core::Time time_horizon = K * dt;  // reference.total_duration() * 2.;

#if symplectic_system
  typedef TinySymplecticSystem<q_dim, quat_integration, control_dim, double,
                               DoubleUtils>
      Dynamics;
#else
  typedef TinyControlledSystem<q_dim, control_dim, double, DoubleUtils>
      Dynamics;
#endif

  Dynamics::State x0;
  x0.setZero();
  if (quat_integration) {
    x0[3] = 1.0;  // Quaternion w coordinate
  }
  x0[5 + quat_integration] = initial_z;
  double knee_angle = -0.5;
  double abduction_angle = 0.2;
  x0[6 + quat_integration] = abduction_angle;
  x0[8 + quat_integration] = knee_angle;
  x0[9 + quat_integration] = abduction_angle;
  x0[11 + quat_integration] = knee_angle;
  x0[12 + quat_integration] = abduction_angle;
  x0[14 + quat_integration] = knee_angle;
  x0[15 + quat_integration] = abduction_angle;
  x0[17 + quat_integration] = knee_angle;

  typedef TermQuadratic<state_dim, control_dim> CostTerm;
  typedef TermQuadTracking<state_dim, control_dim> TrackingTerm;
  typedef CostFunctionAnalytical<state_dim, control_dim> CostFunction;

  // TODO implement linearizer using autodiff. CT uses CppAD that requires
  //   us to have no conditions in the code, or use special functions, such as
  //   CppAD::CondExpLt. Therefore, we can use CT's autodiff linearizer only
  //   for scenarios without contact at the moment.

  typedef ControlInputConstraint<state_dim, control_dim> ControlConstraint;
  typedef StateConstraint<state_dim, control_dim> StateConstraint;
  typedef ConstraintContainerAnalytical<state_dim, control_dim>
      ConstraintContainer;
  typedef NLOptConSolver<state_dim, control_dim, q_dim, qd_dim> Solver;
#if use_servo_actuator
  TinySystemConstructor<TinyServoActuator> constructor(
      system_urdf_filename, use_contact ? plane_urdf_filename : "");
#else
  TinySystemConstructor<TinyActuator> constructor(
      system_urdf_filename, use_contact ? plane_urdf_filename : "");
#endif
  constructor.m_is_floating = true;
  constructor.m_joint_damping = joint_damping;
#if use_servo_actuator
  constructor.m_actuator = new TinyServoActuator<double, DoubleUtils>(
      control_dim, 150., 3., -500., 500.);
#else
  constructor.m_actuator = new TinyActuator<double, DoubleUtils>(control_dim);
#endif
  constructor.m_actuator->limits =
      std::vector<double>(control_dim, control_limit);
  constructor.m_control_indices = control_indices;
  auto dynamics = std::make_shared<Dynamics>(constructor, dt);
  dynamics->use_soft_contact = use_spring_contact;
  dynamics->initialize(sim_api, vis_api);
  if (!use_contact) {
    dynamics->m_gravity.set_zero();
  } else if (use_contact && use_spring_contact) {
    dynamics->m_world.m_mb_constraint_solver =
        new TinyMultiBodyConstraintSolverSpring<double, DoubleUtils>;
  }

  auto intermediate_cost = std::make_shared<CostTerm>();
  auto final_cost = std::make_shared<CostTerm>();
  bool verbose = true;
  //  intermediate_cost->loadConfigFile(our_search_path + "laikago_cost.info",
  //                                    "intermediateCost", verbose);
  //  final_cost->loadConfigFile(our_search_path + "laikago_cost.info",
  //  "finalCost",
  //                             verbose);

  // Since we are using quadratic cost function terms in this example, the first
  // and second order derivatives are immediately known and we define the cost
  // function to be an "Analytical Cost Function". Let's create the
  // corresponding object and add the previously loaded intermediate and final
  // term.
  auto cost_function = std::make_shared<CostFunction>();
  //  cost_function->addIntermediateTerm(intermediate_cost);
  //  cost_function->addFinalTerm(final_cost);

  auto tracking_cost = std::make_shared<TrackingTerm>();
  tracking_cost->loadConfigFile(our_search_path + "laikago_cost.info",
                                "trackingCost", verbose);
  ControlVectorArray<control_dim> ref_controls;
  StateVectorArray<state_dim> ref_states(K + 1, Dynamics::State::Zero());
  ct::core::tpl::TimeArray<double> ref_times;
  double time_factor = dt;  // reference.total_duration() / (double(K + 1));
  for (int k = 0; k < K + 1; ++k) {
    double time = k * time_factor;
    ref_times.push_back(time);
    std::vector<double> frame = reference.calculate_frame(time);
    // the floating-base coordinates (6) are ignored in the cost function
    for (int i = 0; i < control_dim; ++i) {
      ref_states[k][i + 6 + quat_integration] = frame[i + 7];
    }
    ref_states[k][5 + quat_integration] = initial_z;
    if (k % 100 == 0) {
      std::cout << "k=" << k << " \t" << ref_states[k].transpose() << std::endl;
    }
    if (k < K) {
      Dynamics::Control control = Dynamics::Control::Zero();
      ref_controls.push_back(control);
    }
  }
  printf("Reference times: %i, states: %i, controls: %i, frames: %i\n",
         static_cast<int>(ref_times.size()),
         static_cast<int>(ref_states.size()),
         static_cast<int>(ref_controls.size()),
         static_cast<int>(reference.frames.size()));
  StateTrajectory<state_dim> state_traj(ref_times, ref_states);
  ControlTrajectory<control_dim> control_traj(ref_times, ref_controls);
  tracking_cost->setStateAndControlReference(state_traj, control_traj);

  cost_function->addIntermediateTerm(tracking_cost);
  cost_function->addFinalTerm(tracking_cost);

  // create and initialize an "optimal control problem"
  Solver::OptConProblem_t optConProblem(time_horizon, x0, dynamics,
                                        cost_function);

  // state box constraint boundaries with sparsities in constraint toolbox
  // format we put a box constraint on the velocity, hence the overall
  // constraint dimension is 1.
  Eigen::VectorXi sp_state(state_dim);
  sp_state.setZero();
  Eigen::VectorXd x_lb(12);
  Eigen::VectorXd x_ub(12);
  // set Euler angle constraints
  int limit_offset = 0;
#if (!quat_integration)
  x_lb = Eigen::VectorXd(12 + 3);
  x_ub = Eigen::VectorXd(12 + 3);

  sp_state[0] = 1;
  x_lb[0] = 0.;
  x_ub[0] = M_PI;
  sp_state[1] = 1;
  x_lb[1] = -M_PI_2;
  x_ub[1] = M_PI;
  sp_state[2] = 1;
  x_lb[2] = -M_PI;
  x_ub[2] = M_PI;

  limit_offset = 3;
#endif
  // set joint limits
  for (int i = 0; i < 12; ++i) {
    sp_state[i + 6 + quat_integration] = 1;
    x_lb[i + limit_offset] = -90. * M_PI / 180;
    x_ub[i + limit_offset] = 90. * M_PI / 180;
  }
  // constraint terms
  //  auto state_limits = std::make_shared<StateConstraint>(x_lb, x_ub,
  //  sp_state); state_limits->setName("StateBound");

  // input box constraint constraint container
  //  auto state_box_constraints = std::make_shared<ConstraintContainer>();

  // add and initialize constraint terms
  //  state_box_constraints->addIntermediateConstraint(state_limits, verbose);
  //  state_box_constraints->initialize();
  //  optConProblem.setStateBoxConstraints(state_box_constraints);

  // design trivial initial controller for iLQR. Note that in this simple
  // example, we can simply use zero feedforward with zero feedback gains
  // around  the initial position. In more complex examples, a more elaborate
  // initial guess may be required.
  FeedbackArray<state_dim, control_dim> u0_fb(
      K, FeedbackMatrix<state_dim, control_dim>::Zero());
#if use_servo_actuator
  for (std::size_t i = 0; i < ref_controls.size(); ++i) {
    for (int j = 0; j < control_dim; ++j) {
      // ref_controls[i][j] = ref_states[i][j + 6 + quat_integration];
      ref_controls[i][j] = x0[j + 6 + quat_integration];
    }
  }
#else
  if (initialize_from_pd_control) {
    dynamics->initialize_static_pd_control(ref_controls, x0, x0, 150., 3.,
                                           -550., 550.);
    //    dynamics->initialize_static_pd_control(ref_controls, x0, x0, 15., 3.,
    //                                           -50., 50.);
    for (std::size_t i = 0; i < ref_controls.size(); i += 100) {
      printf("k=%03d\tPD control: ", static_cast<int>(i));
      std::cout << ref_controls[i].transpose() << '\n';
    }
  }
#endif

  //  while (vis_api->isConnected()) {
  //    dynamics->playback_physics(vis_api, time_horizon, x0, ref_controls);
  //    std::this_thread::sleep_for(std::chrono::duration<double>(10.));
  //    std::cout << "\n\n\n\n\n";
  //  }

  // P and V must have the same dimensions for the NLopt solver in
  // Control Toolbox
#if (!quat_integration)
  StateVectorArray<state_dim> init_states(K + 1, x0);
  Solver::Policy_t initController(init_states, ref_controls, u0_fb,
                                  ilqr_settings.dt);

  // create an NLOptConSolver instance
  Solver iLQR(optConProblem, ilqr_settings);

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

  //  std::cout << "SOLUTION:\n";
  //  for (std::size_t i = 0; i < controller.x_ref().size(); ++i) {
  //    if (i % 100 != 0) continue;
  //    printf("%03d\t", static_cast<int>(i));
  //    std::cout << controller.x_ref()[i].transpose() << std::endl;
  //  }

  fflush(stdout);

  {
    std::string traj_filename =
        "/home/eric/tiny-differentiable-simulator/python/plotting/traj_" +
        motion_file + ".csv";
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
    if (playback_controller_with_uff) {
      dynamics->playback_physics(vis_api, time_horizon, x0, controller.uff());
    } else {
      dynamics->playback_controller(sim_api, vis_api, controller, true, 10.);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(10.));
  }
#endif

  vis_api->disconnect();
  sim_api->disconnect();
  delete vis_api;
  delete sim_api;

  return EXIT_SUCCESS;
}
