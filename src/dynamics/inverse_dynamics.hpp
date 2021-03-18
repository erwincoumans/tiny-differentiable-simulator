#pragma once

#include <cmath>

#include "../multi_body.hpp"
#include "kinematics.hpp"
#include "math/conditionals.hpp"

namespace tds {
/**
 * Computes the joint torques to achieve the given joint velocities and
 * accelerations. To compute gravity compensation terms, set qd, qdd to zero
 * and pass in negative gravity. To compute only the Coriolis and centrifugal
 * terms, set gravity, qdd and external forces to zero while keeping the joint
 * velocities qd unchanged.
 * @param q Joint positions.
 * @param qd Joint velocities.
 * @param qdd_desired Desired joint accelerations.
 * @param gravity Gravity.
 * @param tau Joint forces (output).
 */
template <typename Algebra>
void inverse_dynamics(MultiBody<Algebra> &mb,
                      const typename Algebra::VectorX &q,
                      const typename Algebra::VectorX &qd,
                      const typename Algebra::VectorX &qdd_desired,
                      const typename Algebra::Vector3 &gravity,
                      typename Algebra::VectorX &tau) {
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
  assert(Algebra::size(qdd_desired) == mb.dof_qd());
  assert(Algebra::size(tau) == mb.dof_actuated());

  MotionVector spatial_gravity;
  spatial_gravity.bottom = gravity;

  // in the following, the variable names for articulated terms I^A, p^A are
  // used for composite rigid body terms I^c, p^c to avoid introducing more
  // variables

  mb.base_acceleration() = spatial_gravity;
  forward_kinematics(mb, q, qd, qdd_desired);

  if (!mb.is_floating()) {
    int tau_index = mb.dof() - 1;
    for (int i = static_cast<int>(mb.size() - 1); i >= 0; i--) {
      Link &link = mb[i];
      int parent = link.parent_index;
      if (link.joint_type != JOINT_FIXED) {
        tau[tau_index] = Algebra::dot(link.S, link.f);
        --tau_index;
      }
      if (parent >= 0) {
        // mb[parent].f += link.X_parent.apply_transpose(link.f);
        mb[parent].f += link.X_parent.apply(link.f);
      }
    }
    return;
  }

  // TODO fix floating-base case
  assert(false);

  // // I_0^c, p_0^c are (partially) computed by forward_kinematics
  // mb.base_bias_force() += mb.base_abi().mul_inv(mb.base_acceleration());

  // for (int i = static_cast<int>(mb.size() - 1); i >= 0; i--) {
  //   Link &link = mb[i];
  //   int parent = link.parent_index;
  //   ArticulatedBodyInertia &parent_Ic =
  //       parent >= 0 ? mb[parent].abi : mb.base_abi();
  //   // forward kinematics computes composite rigid-body bias force p^c as f
  //   ForceVector &parent_pc = parent >= 0 ? mb[parent].f : mb.base_bias_force();
  //   parent_Ic += ArticulatedBodyInertia::shift(link.abi, link.X_parent);
  //   parent_pc += link.X_parent.apply_transpose(link.f);
  // }

  // mb.base_acceleration() =
  //     -mb.base_abi().inverse().mul_inv(mb.base_bias_force());

  // int tau_index = 0;
  // for (int i = 0; i < static_cast<int>(mb.size()); i++) {
  //   Link &link = mb[i];
  //   //!!! The implementation is different from Featherstone Table 9.6, the
  //   //!!! commented-out lines correspond to the book implementation that
  //   //!!! leads to forces too low to compensate gravity in the joints (see
  //   //!!! gravity_compensation target).
  //   //      int parent = link.parent_index;
  //   //      const TinySpatialMotionVector& parent_a =
  //   //          (parent >= 0) ? mb[parent].m_a : mb.base_acceleration();
  //   //
  //   //      link.m_a = link.X_parent.apply(parent_a);

  //   if (link.joint_type != JOINT_FIXED) {
  //     tau[tau_index] = Algebra::dot(link.S, 
  //         link.f);  // Algebra::dot(link.S, link.abi.mul_inv(link.m_a) + link.f);
  //     ++tau_index;
  //   }
  // }
}
}  // namespace tds