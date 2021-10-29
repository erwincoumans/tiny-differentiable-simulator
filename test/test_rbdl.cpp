#include <gtest/gtest.h>

//#define TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS

#include "math/eigen_algebra.hpp"
#include "rbdl_test_utils.hpp"


using namespace tds;
using namespace TINY;

template <typename Algebra> void test_urdf_kinematics(std::string filename) {
  using Tf = Transform<Algebra>;
  using Vector3 = typename Algebra::Vector3;
  using VectorX = typename Algebra::VectorX;
  using Matrix3 = typename Algebra::Matrix3;
  using RigidBodyInertia = RigidBodyInertia<Algebra>;

  World<Algebra> world;
  MultiBody<Algebra> *mb = nullptr;

  std::string urdf_filename;
  bool is_floating = false;
  bool result = FileUtils::find_file(filename, urdf_filename);
  ASSERT_TRUE(result);
  printf("urdf_filename=%s\n", urdf_filename.c_str());
#ifdef USE_BULLET_URDF_PARSER
  UrdfCache<Algebra> cache;
  mb = cache.construct(urdf_filename, world, false, is_floating);
#else  // USE_BULLET_URDF_PARSER
  UrdfParser<Algebra> parser;
  MultiBody<Algebra> mb1;
  mb = &mb1;
  std::string full_path;
  FileUtils::find_file(filename, full_path);
  UrdfStructures<Algebra> urdf_structures = parser.load_urdf(full_path);
  UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_structures, world, mb1);
  mb1.initialize();
#endif // USE_BULLET_URDF_PARSER

  std::string fail_message = "Failure at time step ";

  {
    RigidBodyDynamics::Model rbdl_model = to_rbdl(*mb);

    using VectorND = RigidBodyDynamics::Math::VectorNd;
    VectorND rbdl_q = VectorND::Zero(rbdl_model.q_size);
    VectorND rbdl_qd = VectorND::Zero(rbdl_model.qdot_size);
    VectorND rbdl_qdd = VectorND::Zero(rbdl_model.qdot_size);
    VectorND rbdl_tau = VectorND::Zero(rbdl_model.qdot_size);

    int q_offset = 0, qd_offset = 0;
    if (mb->is_floating()) {
      rbdl_q[0] = Algebra::to_double(mb->q(4));
      rbdl_q[1] = Algebra::to_double(mb->q(5));
      rbdl_q[2] = Algebra::to_double(mb->q(6));
      // w coordinate of quat is stored at the end
      rbdl_q[mb->dof() - 1] = Algebra::to_double(mb->q(3));
      rbdl_q[3] = Algebra::to_double(mb->q(0));
      rbdl_q[4] = Algebra::to_double(mb->q(1));
      rbdl_q[5] = Algebra::to_double(mb->q(2));
      q_offset = 7;
      rbdl_qd[0] = Algebra::to_double(mb->qd(3));
      rbdl_qd[1] = Algebra::to_double(mb->qd(4));
      rbdl_qd[2] = Algebra::to_double(mb->qd(5));
      rbdl_qd[3] = Algebra::to_double(mb->qd(0));
      rbdl_qd[4] = Algebra::to_double(mb->qd(1));
      rbdl_qd[5] = Algebra::to_double(mb->qd(2));
      qd_offset = 6;
    }

    for (int i = q_offset; i < mb->dof(); ++i) {
      rbdl_q[i - int(mb->is_floating())] = Algebra::to_double(mb->q(i));
    }
    for (int i = qd_offset; i < mb->dof_qd(); ++i) {
      rbdl_qd[i] = Algebra::to_double(mb->qd(i));
      rbdl_qdd[i] = Algebra::to_double(mb->qdd(i));
    }
    RigidBodyDynamics::UpdateKinematics(rbdl_model, rbdl_q, rbdl_qd, rbdl_qdd);
    forward_kinematics(*mb);

    srand(1234);

    for (int i = 0; i < 200; ++i) {
      for (int i = q_offset; i < mb->dof(); ++i) {
        mb->q(i) = double(rand()) / RAND_MAX;
        rbdl_q[i - int(mb->is_floating())] = Algebra::to_double(mb->q(i));
      }
      for (int i = qd_offset; i < mb->dof_qd(); ++i) {
        mb->qd(i) = 0.0;  // double(rand()) / RAND_MAX;
        mb->qdd(i) = 0.0; // double(rand()) / RAND_MAX;
        rbdl_qd[i] = Algebra::to_double(mb->qd(i));
        rbdl_qdd[i] = Algebra::to_double(mb->qdd(i));
      }
      // mb->print_state(false);
      RigidBodyDynamics::UpdateKinematics(rbdl_model, rbdl_q, rbdl_qd,
                                          rbdl_qdd);
      forward_kinematics(*mb);

      EXPECT_TRUE(is_equal<Algebra>(*mb, rbdl_model, false))
          << fail_message << i;
    }
  }
}

template <typename Algebra>
void test_urdf_dynamics(std::string filename, bool is_floating = false) {
  using Tf = Transform<Algebra>;
  using Vector3 = typename Algebra::Vector3;
  using VectorX = typename Algebra::VectorX;
  using Matrix3 = typename Algebra::Matrix3;
  using Quaternion = typename Algebra::Quaternion;
  using RigidBodyInertia = RigidBodyInertia<Algebra>;

  Vector3 gravity(0., 0., -9.81);
  // Vector3 gravity(0., 0., 0.);

  World<Algebra> world;
  MultiBody<Algebra> *mb = nullptr;

  std::string urdf_filename;
  bool result = FileUtils::find_file(filename, urdf_filename);
  ASSERT_TRUE(result);
  printf("urdf_filename=%s\n", urdf_filename.c_str());
#ifdef USE_BULLET_URDF_PARSER
  UrdfCache<Algebra> cache;
  mb = cache.construct(urdf_filename, world, false, is_floating);
#else  // USE_BULLET_URDF_PARSER
  UrdfParser<Algebra> parser;
  MultiBody<Algebra> mb1(is_floating);
  mb = &mb1;
  std::string full_path;
  FileUtils::find_file(filename, full_path);
  UrdfStructures<Algebra> urdf_structures = parser.load_urdf(full_path);
  UrdfToMultiBody<Algebra>::convert_to_multi_body(urdf_structures, world, mb1,
                                                  0);
  mb1.initialize();
#endif // USE_BULLET_URDF_PARSER

  if (mb->is_floating()) {
    // apply some random base velocity for testing
    mb->qd(0) = -0.5;
    mb->qd(2) = 1.7;
    mb->qd(3) = 1.3;
    mb->qd(4) = 0.87;

    // mb->qd(0) = -545.35;
    // mb->qd(2) = 172.56;
    // mb->qd(3) = 13.1204;
    // mb->qd(4) = 87.439;

    // set a random base orientation
    mb->set_orientation(
        Algebra::matrix_to_quat(Algebra::rotation_zyx_matrix(0.5, -0.7, 0.3)));
    // mb->set_orientation(
    //     Algebra::matrix_to_quat(Algebra::rotation_x_matrix(M_PI_2)));
    //     mb->set_position(Vector3(2034.0, -102.1879, 732.73));
  }

  std::string fail_message = "Failure at time step ";

  {
    RigidBodyDynamics::Model rbdl_model = to_rbdl(*mb);
    rbdl_model.gravity = to_rbdl<Algebra>(gravity);

    using VectorND = RigidBodyDynamics::Math::VectorNd;
    VectorND rbdl_q = VectorND::Zero(rbdl_model.q_size);
    VectorND rbdl_qd = VectorND::Zero(rbdl_model.qdot_size);
    VectorND rbdl_qdd = VectorND::Zero(rbdl_model.qdot_size);
    VectorND rbdl_tau = VectorND::Zero(rbdl_model.qdot_size);

    assign(*mb, &rbdl_q, &rbdl_qd, &rbdl_qdd, &rbdl_tau);
    RigidBodyDynamics::UpdateKinematics(rbdl_model, rbdl_q, rbdl_qd, rbdl_qdd);
    forward_kinematics(*mb);

    int qd_offset = 0;
    if (mb->is_floating()) {
      qd_offset = 6;
    }

    double dt = 0.003;
    for (int i = 0; i < 200; ++i) {
#ifdef VERBOSE_PRINT
      printf("\n\n\nt: %i\n", i);
#endif
      // forward_kinematics(*mb);
      // traj.push_back(mb->q);
      int nd = mb->dof_actuated();
      // Algebra::Index j = 2;
      // for (Algebra::Index j = 3; j < nd; ++j) {
      //   mb->tau(j) = Algebra::sin(i * dt * 10.) * 1e-4;
      //   rbdl_tau[j] = Algebra::to_double(mb->tau(j));
      // }

      for (int i = 0; i < mb->dof_actuated(); ++i) {
        mb->tau(i) = Algebra::cos(i * dt * 10.) * 0.1;
        rbdl_tau[i + qd_offset] = Algebra::to_double(mb->tau(i));
      }

      bool rbdl_convention = true;
      forward_dynamics(*mb, gravity, rbdl_convention);

      RigidBodyDynamics::ForwardDynamics(rbdl_model, rbdl_q, rbdl_qd, rbdl_tau,
                                         rbdl_qdd);

#ifdef VERBOSE_PRINT
      mb->print_state();

      std::cout << "RBDL q: " << rbdl_q.transpose()
                << "   qd: " << rbdl_qd.transpose()
                << "   qdd:  " << rbdl_qdd.transpose()
                << "   tau:  " << rbdl_tau.transpose() << std::endl;
#endif
      bool equal_model = is_equal<Algebra>(*mb, rbdl_model);
      EXPECT_TRUE(equal_model) << fail_message << i;
      if (!equal_model) {
        break;
      }

      {
        bool equal_coordinates =
            is_equal<Algebra>(*mb, rbdl_q, rbdl_qd, rbdl_qdd);
        ASSERT_TRUE(equal_coordinates) << fail_message << i;
        if (!equal_coordinates) {
          break;
        }
      }

      integrate_euler(*mb, dt);
      rbdl_qd += rbdl_qdd * dt;
      if (mb->is_floating()) {
        // need to integrate quaternion
        Quaternion quat = Algebra::quat_from_xyzw(
            rbdl_q[3], rbdl_q[4], rbdl_q[5], rbdl_q[mb->dof() - 1]);
        // Algebra::print("Base quat (RBDL): ", quat);
        Vector3 ang_vel(rbdl_qd[3], rbdl_qd[4], rbdl_qd[5]);
#ifdef VERBOSE_PRINT
        Algebra::print("Angular velocity (RBDL): ", ang_vel);
#endif
        Vector3 ang_vel_tds(mb->qd(0), mb->qd(1), mb->qd(2));
#ifdef VERBOSE_PRINT
        Algebra::print("Angular velocity (TDS):  ", ang_vel_tds);
#endif
        Quaternion dquat = Algebra::quat_velocity(quat, ang_vel, dt);
        Algebra::quat_increment(quat, dquat);
        quat = Algebra::normalize(quat);
        rbdl_q[3] = Algebra::quat_x(quat);
        rbdl_q[4] = Algebra::quat_y(quat);
        rbdl_q[5] = Algebra::quat_z(quat);
        rbdl_q[mb->dof() - 1] = Algebra::quat_w(quat);
        // linear velocity integration
        rbdl_q[0] += rbdl_qd[0] * dt;
        rbdl_q[1] += rbdl_qd[1] * dt;
        rbdl_q[2] += rbdl_qd[2] * dt;
        for (int i = 6; i < mb->dof_qd(); ++i) {
          rbdl_q[i] += rbdl_qd[i] * dt;
        }
      } else {
        rbdl_q += rbdl_qd * dt;
      }

      {
        bool equal_coordinates =
            is_equal<Algebra>(*mb, rbdl_q, rbdl_qd, rbdl_qdd);
        ASSERT_TRUE(equal_coordinates) << fail_message << i;
        if (!equal_coordinates) {
          break;
        }
      }

      mb->clear_forces();

      // compare Jacobians
      // tds::forward_kinematics(*mb);
      // int jac_link_id = 2;
      // Vector3 world_point(1., 2., 3.);
      // auto tds_jac = tds::point_jacobian2(*mb, jac_link_id, world_point,
      // false); RigidBodyDynamics::Math::MatrixNd
      // rbdl_jac(Algebra::num_rows(tds_jac),
      //                                            Algebra::num_cols(tds_jac));
      // rbdl_jac.setZero();
      // // left-associative inverse transform of body_to_world transform
      // // (rotation matrix is not inverted)
      // Transform<Algebra> link_tf = (*mb)[jac_link_id].X_world;
      // Vector3 body_point =
      //     link_tf.rotation * (world_point - link_tf.translation);
      // RigidBodyDynamics::CalcPointJacobian(rbdl_model, rbdl_q, jac_link_id +
      // 1,
      //                                      to_rbdl<Algebra>(body_point),
      //                                      rbdl_jac);

      // Algebra::print("TDS Jacobian", tds_jac);
      // std::cout << "RBDL Jacobian:\n" << rbdl_jac << std::endl;
      // if (!is_equal<Algebra>(tds_jac, rbdl_jac)) {
      //   exit(1);
      // }
    }
  }
}

// TEST(RBDLTest, SwimmerDynamics) {
//   test_urdf_dynamics<TinyAlgebra<double, TINY::DoubleUtils>>(
//       "swimmer/swimmer05/swimmer05.urdf");
//   test_urdf_dynamics<tds::EigenAlgebra>("swimmer/swimmer05/swimmer05.urdf");
// }

// TEST(RBDLTest, PendulumDynamics) {
//   std::cout << "\n\n### TinyAlgebra:\n";
//   test_urdf_dynamics<TinyAlgebra<double,
//   TINY::DoubleUtils>>("pendulum5.urdf"); std::cout << "\n\n###
//   EigenAlgebra:\n"; test_urdf_dynamics<tds::EigenAlgebra>("pendulum5.urdf");
// }

// TEST(RBDLTest, PandaDynamics) {
//   std::cout << "\n\n### TinyAlgebra:\n";
//   test_urdf_dynamics<TinyAlgebra<double, TINY::DoubleUtils>>(
//       "/home/eric/pds-risk-aware/data/franka_panda/panda_with_pendulum.urdf");
//   std::cout << "\n\n### EigenAlgebra:\n";
//   test_urdf_dynamics<tds::EigenAlgebra>(
//       "/home/eric/pds-risk-aware/data/franka_panda/panda_with_pendulum.urdf");
// }

//TEST(RBDLTest, FloatingCube_Tiny) {
//  test_urdf_dynamics<TinyAlgebra<double, TINY::DoubleUtils>>("sphere8cube.urdf",
//                                                             true);
//}

//TEST(RBDLTest, FloatingCube_Eigen) {
//  test_urdf_dynamics<tds::EigenAlgebra>("sphere8cube.urdf", true);
//}

// TEST(RBDLTest, PandaWithBoxDynamics) {
//   std::cout << "\n\n### TinyAlgebra:\n";
//   test_urdf_dynamics<TinyAlgebra<double, TINY::DoubleUtils>>(
//       "/home/eric/pds-risk-aware/data/franka_panda/panda_with_box.urdf");
//   std::cout << "\n\n### EigenAlgebra:\n";
//   test_urdf_dynamics<tds::EigenAlgebra>(
//       "/home/eric/pds-risk-aware/data/franka_panda/panda_with_box.urdf");
// }

// TEST(RBDLTest, LaikagoNoToesDynamics_Tiny) {
//  test_urdf_dynamics<TinyAlgebra<double, TINY::DoubleUtils>>(
//      "laikago/laikago_no_toes.urdf", true);
//}

//TEST(RBDLTest, LaikagoNoToesDynamics_Eigen) {
//  test_urdf_dynamics<tds::EigenAlgebra>("laikago/laikago_no_toes.urdf", true);
//}

TEST(RBDLTest, AntLegDynamics_Tiny) {
  test_urdf_dynamics<TinyAlgebra<double, TINY::DoubleUtils>>(
      "gym/ant_lite_one_leg_xyz_xyzrot.urdf", false);
}

TEST(RBDLTest, AntLegDynamics_Eigen) {
  test_urdf_dynamics<tds::EigenAlgebra>("gym/ant_lite_one_leg_xyz_xyzrot.urdf",
                                        false);
}

// TEST(RBDLTest, LaikagoDynamics) {
//   std::cout << "\n\n### TinyAlgebra:\n";
//   test_urdf_dynamics<TinyAlgebra<double, TINY::DoubleUtils>>(
//       "laikago/laikago_toes_zup.urdf", true);
//   std::cout << "\n\n### EigenAlgebra:\n";
//   test_urdf_dynamics<tds::EigenAlgebra>(
//       "laikago/laikago_toes_zup.urdf", true);
// }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
