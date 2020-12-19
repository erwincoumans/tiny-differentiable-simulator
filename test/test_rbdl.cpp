#include <gtest/gtest.h>

#include "rbdl_test_utils.hpp"

using namespace tds;
using namespace TINY;

using Algebra = TinyAlgebra<double, DoubleUtils>;
using Tf = Transform<Algebra>;
using Vector3 = Algebra::Vector3;
using VectorX = typename Algebra::VectorX;
using Matrix3 = Algebra::Matrix3;
using RigidBodyInertia = RigidBodyInertia<Algebra>;

void TestOnURDF(std::string filename)
{
  Vector3 gravity(0., 0., -9.81);

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
#else //USE_BULLET_URDF_PARSER
 UrdfParser<Algebra> parser;
  MultiBody<Algebra> mb1;
  mb = &mb1;
  std::string full_path;
  FileUtils::find_file(filename, full_path);
  UrdfStructures<Algebra> urdf_structures =
      parser.load_urdf(full_path);
  UrdfToMultiBody<Algebra>::convert_to_multi_body(
      urdf_structures, world, mb1, 0);
  mb1.initialize();
#endif//USE_BULLET_URDF_PARSER

  std::string fail_message = "Failure at iteration ";

  {
    RigidBodyDynamics::Model rbdl_model = to_rbdl(*mb);
    rbdl_model.gravity = to_rbdl<Algebra>(gravity);

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
    for (int i = 0; i < mb->dof_actuated(); ++i) {
      rbdl_tau[i + qd_offset] = Algebra::to_double(mb->tau(i));
    }
    RigidBodyDynamics::UpdateKinematics(rbdl_model, rbdl_q, rbdl_qd, rbdl_qdd);
    forward_kinematics(*mb);
    // if (!is_equal<Algebra>(*mb, rbdl_model)) {
    //   // exit(1);
    // }

    double dt = 0.001;
    for (int i = 0; i < 200; ++i) {
      printf("\n\n\nt: %i\n", i);
      forward_kinematics(*mb);
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

      forward_dynamics(*mb, gravity);
      for (auto &link : *mb) {
        // Algebra::print(
        //     ("link[" + std::to_string(link.q_index) + "].D").c_str(),
        //     link.D);
        // Algebra::print(
        //     ("link[" + std::to_string(link.q_index) + "].U").c_str(),
        //     link.U);
        // Algebra::print(
        //     ("link[" + std::to_string(link.q_index) + "].S").c_str(),
        //     link.S);
        // Algebra::print(
        //     ("link[" + std::to_string(link.q_index) + "].u").c_str(),
        //     link.u);
        // Algebra::print(
        //     ("TDS link[" + std::to_string(link.q_index) + "].X_world")
        //         .c_str(),
        //     link.X_world);
        // std::cout << "RBDL link[" << link.q_index << "].X_base\n"
        //           << rbdl_model.X_base[link.q_index + 1] << std::endl;
      }

      RigidBodyDynamics::ForwardDynamics(rbdl_model, rbdl_q, rbdl_qd, rbdl_tau,
                                         rbdl_qdd);

      mb->print_state();
      std::cout << "RBDL q: " << rbdl_q.transpose()
                << "   qd: " << rbdl_qd.transpose()
                << "   qdd:  " << rbdl_qdd.transpose()
                << "   tau:  " << rbdl_tau.transpose() << std::endl;

      // if (!is_equal<Algebra>(*mb, rbdl_model)) {
      //   assert(0);
      //   exit(1);
      // }

      ASSERT_TRUE(is_equal<Algebra>(*mb, rbdl_model)) << fail_message << i;

      // if (!is_equal<Algebra>(*mb, rbdl_q, rbdl_qd, rbdl_qdd)) {
      //   exit(1);
      // }
      ASSERT_TRUE(is_equal<Algebra>(*mb, rbdl_q, rbdl_qd, rbdl_qdd))
          << fail_message << i;

      integrate_euler(*mb, dt);
      rbdl_qd += rbdl_qdd * dt;
      if (mb->is_floating()) {
        // need to integrate quaternion
        Algebra::Quaternion quat = Algebra::quat_from_xyzw(
            rbdl_q[3], rbdl_q[4], rbdl_q[5], rbdl_q[mb->dof() - 1]);
        // Algebra::print("Base quat (RBDL): ", quat);
        Algebra::Vector3 ang_vel(rbdl_qd[3], rbdl_qd[4], rbdl_qd[5]);
        // Algebra::print("Angular velocity (RBDL): ", ang_vel);
        // Algebra::Vector3 ang_vel_tds(mb->qd(0), mb->qd(1), mb->qd(2));
        // Algebra::print("Angular velocity (TDS):  ", ang_vel_tds);
        Algebra::Quaternion dquat = Algebra::quat_velocity(quat, ang_vel, dt);
        quat += dquat;
        Algebra::normalize(quat);
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

      // std::cout << "RBDL q (before mod): " << rbdl_q.transpose() <<
      // std::endl;

      // if (!is_equal<Algebra>(*mb, rbdl_q, rbdl_qd, rbdl_qdd)) {
      //   exit(1);
      // }
      ASSERT_TRUE(is_equal<Algebra>(*mb, rbdl_q, rbdl_qd, rbdl_qdd))
          << fail_message << i;

      mb->clear_forces();

      // compare Jacobians
      tds::forward_kinematics(*mb);
      int jac_link_id = 2;
      Algebra::Vector3 world_point(1., 2., 3.);
      auto tds_jac = tds::point_jacobian2(*mb, jac_link_id, world_point, false);
      RigidBodyDynamics::Math::MatrixNd rbdl_jac(Algebra::num_rows(tds_jac),
                                                 Algebra::num_cols(tds_jac));
      rbdl_jac.setZero();
      // left-associative inverse transform of body_to_world transform
      // (rotation matrix is not inverted)
      Transform<Algebra> link_tf = (*mb)[jac_link_id].X_world;
      Algebra::Vector3 body_point =
          link_tf.rotation * (world_point - link_tf.translation);
      RigidBodyDynamics::CalcPointJacobian(rbdl_model, rbdl_q, jac_link_id + 1,
                                           to_rbdl<Algebra>(body_point),
                                           rbdl_jac);

      // Algebra::print("TDS Jacobian", tds_jac);
      // std::cout << "RBDL Jacobian:\n" << rbdl_jac << std::endl;
      // if (!is_equal<Algebra>(tds_jac, rbdl_jac)) {
      //   exit(1);
      // }
    }
  }
}

TEST(RBDLTest, Swimmer05) {
  TestOnURDF("swimmer/swimmer05/swimmer05.urdf");
}

TEST(RBDLTest, Pendulum) {
  TestOnURDF("pendulum5.urdf");
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
