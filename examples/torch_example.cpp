#include <torch/torch.h>

#include <fenv.h>
#include <stdio.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "pybullet_visualizer_api.h"
#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_mb_constraint_solver_spring.h"
#include "tiny_multi_body.h"
#include "tiny_system_constructor.h"

struct TorchUtils {
  static torch::Tensor zero() { return torch::tensor({{0.}}); }
  static torch::Tensor one() { return torch::tensor({{1.}}); }

  static torch::Tensor two() { return torch::tensor({{2.}}); }
  static torch::Tensor half() { return torch::tensor({{0.5}}); }
  static torch::Tensor pi() { return torch::tensor({{M_PI}}); }
  static torch::Tensor half_pi() { return torch::tensor({{M_PI_2}}); }

  static torch::Tensor cos1(torch::Tensor v) { return torch::cos(v); }
  static torch::Tensor sin1(torch::Tensor v) { return torch::sin(v); }
  static torch::Tensor atan2(torch::Tensor dy, torch::Tensor dx) {
    return torch::atan2(dy, dx);
  }
  static torch::Tensor asin(torch::Tensor v) { return torch::asin(v); }
  static torch::Tensor copysign(torch::Tensor x, torch::Tensor y) {
    return torch::sign(y) * torch::abs(x);
  }
  static torch::Tensor abs(torch::Tensor v) { return torch::abs(v); }

  static double getDouble(torch::Tensor v) { return v.item().toDouble(); }

  template <class T>
  static double getDouble(T v) {
    return (double)v;
  }

  template <class T>
  static torch::Tensor convert(T) = delete;  // C++11

  static torch::Tensor convert(int value) { return torch::tensor({{value}}); }

  template <class T>
  static torch::Tensor fraction(T, T) = delete;  // C++11

  static torch::Tensor fraction(int num, int denom) {
    return torch::tensor({{num * 1. / denom}});
  }

  static torch::Tensor scalar_from_string(const std::string &txt) {
    torch::Tensor result = torch::tensor({{atof(txt.c_str())}});
    return result;
  }

  static torch::Tensor scalar_from_double(double d) {
    return torch::tensor({{d}});
  }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};


typedef PyBulletVisualizerAPI VisualizerAPI;

int main(int argc, char *argv[]) {
  std::string connection_mode = "gui";
  //"pendulum5.urdf";
  //"sphere8cube.urdf";
  //"cheetah_link0_1.urdf";
  std::string urdf_filename;
  TinyFileUtils::find_file("laikago/laikago_toes_zup.urdf", urdf_filename);

  std::string plane_filename;
  TinyFileUtils::find_file("plane_implicit.urdf", plane_filename);

  if (argc > 1) urdf_filename = std::string(argv[1]);
  bool floating_base = true;

  // Set NaN trap
  // feenableexcept(FE_INVALID | FE_OVERFLOW);

  printf("floating_base=%d\n", floating_base);
  printf("urdf_filename=%s\n", urdf_filename.c_str());
  auto *sim2 = new VisualizerAPI();
  bool isConnected2 = sim2->connect(eCONNECT_DIRECT);

  auto *sim = new VisualizerAPI();
  printf("connection_mode=%s\n", connection_mode.c_str());
  int mode = eCONNECT_SHARED_MEMORY;
  if (connection_mode == "direct") mode = eCONNECT_DIRECT;
  if (connection_mode == "gui") mode = eCONNECT_GUI;
  if (connection_mode == "shared_memory") mode = eCONNECT_SHARED_MEMORY;

  bool isConnected = sim->connect(mode);
  if (!isConnected) {
    printf("Cannot connect\n");
    return -1;
  }

  sim->setTimeOut(10);
  sim->resetSimulation();
  int grav_id = sim->addUserDebugParameter("gravity", -10, 10, -9.81);

  typedef torch::Tensor Scalar;
  typedef TorchUtils Utils;

  TinyWorld<Scalar, Utils> world;
  TinyMultiBody<Scalar, Utils> *system = world.create_multi_body();
  TinySystemConstructor<> constructor(urdf_filename, plane_filename);
  constructor.m_is_floating = floating_base;
  constructor(sim2, sim, world, &system);
  //  delete world.m_mb_constraint_solver;
  //  world.m_mb_constraint_solver =
  //      new TinyMultiBodyConstraintSolverSpring<Scalar, Utils>;

  fflush(stdout);

  if (floating_base) {
    Scalar knee_angle = -Utils::half();
    Scalar abduction_angle = Utils::fraction(2, 10);
    Scalar initial_poses[] = {
        abduction_angle, Utils::zero(), knee_angle, abduction_angle, Utils::zero(), knee_angle,
        abduction_angle, Utils::zero(), knee_angle, abduction_angle, Utils::zero(), knee_angle,
    };

    if (system->m_q.size() >= 12) {
      for (int cc = 0; cc < 12; cc++) {
        system->m_q[7 + cc] = initial_poses[cc];
      }
    }
    system->m_q[6] = Utils::fraction(55, 100);
  }

  std::vector<Scalar> qd_des(system->m_qd);
  std::vector<Scalar> qdd_des(system->m_qdd);
  std::vector<Scalar> tau_res(system->m_tau);

  Scalar dt = Utils::fraction(1, 1000);
  Scalar yaw = Utils::zero();
  Scalar distance = Utils::one();
  while (sim->canSubmitCommand()) {
    double gravZ = sim->readUserDebugParameter(grav_id);
    world.set_gravity(TinyVector3<Scalar, Utils>(Utils::zero(), Utils::zero(), Utils::scalar_from_double(gravZ)));

    system->forward_kinematics();
    system->clear_forces();
    system->forward_dynamics(world.get_gravity());
    system->inverse_dynamics(system->m_q, qd_des, qdd_des, -world.get_gravity(),
                             system->m_tau);
    system->m_baseAppliedForce = system->m_baseBiasForce;
    std::cout << "Inverse dynamics tau:  ";
    for (auto &t : system->m_tau) {
      std::cout << t << "\t";
    }
    std::cout << "\n";
    world.step(dt);
    system->forward_dynamics(world.get_gravity());

    system->integrate(dt);

    PyBulletUrdfImport<Scalar, Utils>::sync_graphics_transforms(system,
                                                                      *sim);
    yaw += 0.05;
    btVector3 basePos(0, 0, 0.2);
    btQuaternion baseOrn(0, 0, 0, 1);
    sim->resetDebugVisualizerCamera(Utils::getDouble(distance), -20, Utils::getDouble(yaw), basePos);
    std::this_thread::sleep_for(std::chrono::duration<double>(Utils::getDouble(dt)));
  }

  sim->disconnect();
  sim2->disconnect();

  delete sim;
  delete sim2;

  return EXIT_SUCCESS;
}
