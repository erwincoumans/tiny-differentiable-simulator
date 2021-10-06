#pragma once

#include <cmath>

#include "math/conditionals.hpp"
#include "../multi_body.hpp"
#include "kinematics.hpp"

namespace tds {
template <typename Algebra>
void forward_dynamics(MultiBody<Algebra> &mb,
                      const typename Algebra::VectorX &q,
                      const typename Algebra::VectorX &qd,
                      const typename Algebra::VectorX &tau,
                      const typename Algebra::Vector3 &gravity,
                      typename Algebra::VectorX &qdd,
                      bool rbdl_convention = false) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using VectorX = typename Algebra::VectorX;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix6 = typename Algebra::Matrix6;
  using Quaternion = typename Algebra::Quaternion;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::Link<Algebra> Link;
  typedef tds::RigidBodyInertia<Algebra> RigidBodyInertia;
  typedef tds::ArticulatedBodyInertia<Algebra> ArticulatedBodyInertia;

  assert(Algebra::size(q) == mb.dof());
  assert(Algebra::size(qd) == mb.dof_qd());
  assert(Algebra::size(qdd) == mb.dof_qd());
  assert(Algebra::size(tau) == mb.dof_actuated());

  MotionVector spatial_gravity;
  spatial_gravity.bottom = gravity;

  // #ifdef NEURAL_SIM
  //     for (int i = 0; i < dof(); ++i) {
  //       NEURAL_ASSIGN(q[i], "q_" + std::to_string(i));
  //     }
  //     for (int i = 0; i < dof_qd(); ++i) {
  //       NEURAL_ASSIGN(qd[i], "qd_" + std::to_string(i));
  //     }
  // #endif

  forward_kinematics(mb, q, qd);

  for (int i = static_cast<int>(mb.num_links()) - 1; i >= 0; i--) {
    const Link &link = mb[i];
    int parent = link.parent_index;

    ArticulatedBodyInertia u_dinv_ut;
    ForceVector UuD;
    if (link.joint_type == JOINT_SPHERICAL){
        link.U_3d = link.abi * link.S_3d;

        link.D_3d = Algebra::transpose(link.S_3d) * link.U_3d;

        VectorX tau_temp = mb.get_tau_for_link(tau, i);

        // apply linear joint stiffness and damping
        // see Eqns. (2.76), (2.78) in Rigid Body Dynamics Notes
        // by Shinjiro Sueda
        // https://github.com/sueda/redmax/blob/master/notes.pdf
        // TODO implement joint stiffness for spherical joints
        // TODO consider nonzero resting position of joint for stiffness?
        // TODO consider non-scalar joint damping and stiffness?
        VectorX quat = mb.get_q_for_link(q, i);
        Quaternion qquat = Algebra::quat_from_xyzw(quat[0], quat[1], quat[2], quat[3]);
        Vector3 Axes_angle = Algebra::quaternion_axis_angle(qquat);
        tau_temp -= link.stiffness * Axes_angle;
        tau_temp -= link.damping * mb.get_qd_for_link(qd, i);

        Vector3 tau_val(tau_temp[0], tau_temp[1], tau_temp[2]);
        link.u_3d = tau_val - Algebra::dot(link.S_3d, link.pA);

#ifdef DEBUG
        Algebra::print("ABI", link.abi);
        Algebra::print("S", link.S);
        Algebra::print("U", link.U);
        printf("links[%d].D=", i);
        double d1 = Algebra::to_double(link.D);
        printf("%.24f\n", d1);
        printf("links[%d].u=", i);
        double u = Algebra::to_double(link.u);
        assert(!std::isnan(u));
        printf("%.24f\n", u);

        printf("LINK  %i\n", i);
#endif
        assert(link.D_3d.determinant() != Algebra::zero());
//        assert(link.joint_type == JOINT_FIXED ||
//               Algebra::abs(link.D) > Algebra::zero());
//        Scalar invD = link.joint_type == JOINT_FIXED ? Algebra::zero()
//                                                     : Algebra::one() / link.D;

        // I made this a member to avoid double calculation of the inverse: it is used again at line 256
        link.invD_3d = link.D_3d.inverse();
#ifdef DEBUG
        printf("invD[%d]=%f\n", i, Algebra::to_double(invD));
#endif
        // Todo: Unused?
//        auto tmp = link.U * link.invD_3d;
        u_dinv_ut =
                ArticulatedBodyInertia::mul_transpose(link.U_3d, link.U_3d * link.invD_3d);
        UuD = Algebra::mul_2_force_vector(link.U_3d, (link.invD_3d * link.u_3d));
        //UuD.print("UuD\n");
    }else {
      link.U = link.abi * link.S;
        // std::cout << "link.abi.matrix() * link.S:\n" << link.abi.matrix() *
        // link.S << std::endl; std::cout << "link.abi * link.S:\n" << link.abi *
        // link.S << std::endl; std::cout << "\n\n";
        link.D = Algebra::dot(link.S, link.U);
        VectorX tau_val = mb.get_tau_for_link(tau, i);
        // apply linear joint stiffness and damping
        // see Eqns. (2.76), (2.78) in Rigid Body Dynamics Notes
        // by Shinjiro Sueda
        // https://github.com/sueda/redmax/blob/master/notes.pdf
        // TODO consider nonzero resting position of joint for stiffness?
        tau_val -= link.stiffness * mb.get_q_for_link(q, i);
        tau_val -= link.damping * mb.get_qd_for_link(qd, i);

        // #ifdef NEURAL_SIM
        //       NEURAL_ASSIGN(tau_val, "tau_" + std::to_string(i));
        // #endif

        link.u = Scalar(tau_val[0]) - Algebra::dot(link.S, link.pA);

#ifdef DEBUG
        Algebra::print("ABI", link.abi);
        Algebra::print("S", link.S);
        Algebra::print("U", link.U);
        printf("links[%d].D=", i);
        double d1 = Algebra::to_double(link.D);
        printf("%.24f\n", d1);
        printf("links[%d].u=", i);
        double u = Algebra::to_double(link.u);
        assert(!std::isnan(u));
        printf("%.24f\n", u);

        printf("LINK  %i\n", i);
#endif

    if constexpr (is_cppad_scalar<Scalar>::value) {
      assert(link.joint_type == JOINT_FIXED ||
             Algebra::to_double(Algebra::abs(link.D)) > 0.0);
    } else {
      assert(link.joint_type == JOINT_FIXED ||
             Algebra::abs(link.D) > Algebra::zero());
    }
    Scalar invD = link.joint_type == JOINT_FIXED ? Algebra::zero()
                                                 : Algebra::one() / link.D;
#ifdef DEBUG
    printf("invD[%d]=%f\n", i, Algebra::to_double(invD));
#endif
        // Todo: Unused?
//        auto tmp = link.U * invD;
        u_dinv_ut =
                ArticulatedBodyInertia::mul_transpose(link.U, link.U * invD);
        UuD = link.U * (link.u * invD);
        //UuD.print("UuD\n");
    }

    //u_dinv_ut.print("u_dinv_ut\n");
    //link.abi.print("link.abi\n");
    ArticulatedBodyInertia Ia = link.abi - u_dinv_ut;
    //Ia.print("Ia\n");
    //ForceVector Ia_c = Ia.mul_inv(link.c);
    ForceVector Ia_c = Ia * link.c;
    //Ia_c.print("Ia_c\n");
    ForceVector pa = link.pA + Ia_c + UuD;
#ifdef DEBUG
    Algebra::print("u_dinv_ut", u_dinv_ut);
    Algebra::print("Ia", Ia);
    Algebra::print("Ia*c", Ia_c);
    Algebra::print("pa", pa);
#endif

    ForceVector delta_pA = link.X_parent.apply(pa);
#ifdef DEBUG
    Algebra::print("delta_pA", delta_pA);
#endif
    // ArticulatedBodyInertia delta_I = link.X_parent.apply(Ia);
    // ArticulatedBodyInertia delta_I = link.X_parent.apply_transpose(Ia);
    Matrix6 xix =
        link.X_parent.matrix_transpose() * Ia.matrix() * link.X_parent.matrix();
    ArticulatedBodyInertia delta_I = xix;
    if (parent >= 0) {
      // Algebra::print(
      //     ("TDS ABI at parent of " + std::to_string(link.index) +
      //     ":").c_str(), links[parent].abi);
      // Algebra::print(
      //     ("TDS Ia at link " + std::to_string(link.index) + ":").c_str(),
      //     Ia);
      // Algebra::print(
      //     ("TDS X_parent at link " + std::to_string(link.index) +
      //     ":").c_str(), link.X_parent);
      mb[parent].pA += delta_pA;
      mb[parent].abi += delta_I;
#ifdef DEBUG
      Algebra::print("pa update", mb[parent].pA);
      Algebra::print("mIA", mb[parent].abi);
#endif
    } else if (mb.is_floating()) {
      mb.base_bias_force() += delta_pA;
      mb.base_abi() += delta_I;
#ifdef DEBUG
      Algebra::print("base_abi", mb.base_abi());
      Algebra::print("base_bias_force", mb.base_bias_force());
      Algebra::print("delta_I", delta_I);
      Algebra::print("delta_pA", delta_pA);
#endif
    }
  }

  if (mb.is_floating()) {
    // #ifdef NEURAL_SIM
    //       NEURAL_ASSIGN(base_bias_force[0], "base_bias_force_0");
    //       NEURAL_ASSIGN(base_bias_force[1], "base_bias_force_1");
    //       NEURAL_ASSIGN(base_bias_force[2], "base_bias_force_2");
    //       NEURAL_ASSIGN(base_bias_force[3], "base_bias_force_3");
    //       NEURAL_ASSIGN(base_bias_force[4], "base_bias_force_4");
    //       NEURAL_ASSIGN(base_bias_force[5], "base_bias_force_5");
    // #endif

     if (rbdl_convention) {
       Matrix6 inv_abi = Algebra::inverse(mb.base_abi().matrix());
       mb.base_acceleration() = -MotionVector( inv_abi * mb.base_bias_force());
     } else {
       mb.base_acceleration() = -mb.base_abi().inv_mul(mb.base_bias_force());
     }
    //mb.base_acceleration() = -MotionVector(
    //    Algebra::inverse(mb.base_abi().matrix()) * mb.base_bias_force());

  } else {
    if (rbdl_convention) {
      // convert gravity to base frame for fixed-base systems
      spatial_gravity = mb.base_X_world().apply(spatial_gravity);
    }
    mb.base_acceleration() = -spatial_gravity;
  }

  for (int i = 0; i < static_cast<int>(mb.num_links()); i++) {
    const Link &link = mb[i];
    int parent = link.parent_index;
    const Transform &X_parent = link.X_parent;
    const MotionVector &a_parent =
        (parent >= 0) ? mb[parent].a : mb.base_acceleration();
#if DEBUG
    if (parent < 0) {
      printf("final loop for parent %i\n", parent);
      Algebra::print("base_abi", mb.base_abi());
      Algebra::print("base_bias_force", mb.base_bias_force());
      Algebra::print("a_parent", a_parent);
    }
#endif
    MotionVector x_a = X_parent.apply(a_parent);
    link.a = x_a + link.c;

    // model.a[i] = X_parent.apply(model.a[parent]) + model.c[i];
    // LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

    if (link.qd_index >= 0) {
      
      
#if DEBUG
      Algebra::print("x_a", x_a);
      Algebra::print("a'", link.a);
#endif
      if (link.joint_type == JOINT_SPHERICAL){
          Vector3 Ut_a_vec = link.U_3d * link.a;
          Vector3 u_Ut_a = link.u_3d - Ut_a_vec;
          Vector3 qdd_val = link.invD_3d * u_Ut_a;


          assert(!std::isnan(Algebra::to_double(qdd_val[0])));
          assert(!std::isnan(Algebra::to_double(qdd_val[1])));
          assert(!std::isnan(Algebra::to_double(qdd_val[2])));
          qdd[link.qd_index] = qdd_val[0];
          qdd[link.qd_index + 1] = qdd_val[1];
          qdd[link.qd_index + 2] = qdd_val[2];

          link.a += Algebra::mul_2_motion_vector(link.S_3d, qdd_val);

      } else{
          Scalar invD = link.joint_type == JOINT_FIXED ? Algebra::zero()
                                                       : Algebra::one() / link.D;
          Scalar Ut_a = Algebra::dot(link.U, link.a);
          Scalar u_Ut_a = link.u - Ut_a;
          Scalar qdd_val = invD * u_Ut_a;
          assert(!std::isnan(Algebra::to_double(qdd_val)));
          qdd[link.qd_index] = qdd_val;

          link.a += link.S * qdd_val;
      }
    }
#if DEBUG
    Algebra::print("a", link.a);
#endif
  }
  if (mb.is_floating()) {
    if (rbdl_convention) {
      MotionVector xa  = mb.base_acceleration();
      xa.bottom += Algebra::cross(mb.base_velocity().top, mb.base_velocity().bottom);
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
      xa.bottom = Algebra::transpose(mb.base_X_world().rotation) *  xa.bottom;
#else
      xa.bottom = mb.base_X_world().rotation *  xa.bottom;
#endif
      xa += spatial_gravity;
      for (int i = 0; i < 6; i++) {
      qdd[i] = xa[i];
      }
    }
    else {
      mb.base_acceleration() += spatial_gravity;
      for (int i = 0; i < 6; i++) {
        qdd[i] = mb.base_acceleration()[i];
      }
    }
  } else {
    mb.base_acceleration().set_zero();
  }
}

template <typename Algebra>
void forward_dynamics(MultiBody<Algebra> &mb,
                      const typename Algebra::Vector3 &gravity,
                      bool rbdl_convention = false) {
  forward_dynamics(mb, mb.q(), mb.qd(), mb.tau(), gravity, mb.qdd(), rbdl_convention);
}
}  // namespace tds
