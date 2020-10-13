#pragma once

#include "kinematics.hpp"

namespace tds {

/**
 * Composite Rigid Body Algorithm (CRBA) to compute the joint space inertia
 * matrix. M must be a properly initialized square matrix of size dof_qd().
 * The inertia matrix is computed in the base frame.
 */
template <typename Algebra>
void mass_matrix(MultiBody<Algebra> &mb, const typename Algebra::VectorX &q,
                 typename Algebra::MatrixX *M) {
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
  assert(M != nullptr);
  int n = static_cast<int>(mb.size());
  // printf("n is %i\n", n);
  assert(Algebra::num_rows(*M) == mb.dof_qd());
  assert(Algebra::num_cols(*M) == mb.dof_qd());

  forward_kinematics(mb, q);

  Algebra::set_zero(*M);
  for (int i = n - 1; i >= 0; --i) {
    const Link &link = mb[i];
    int parent = link.parent_index;
    const ArticulatedBodyInertia &Ic = link.abi;
    const Transform &Xp = link.X_parent;
    // ArticulatedBodyInertia delta_I = Xp.apply_transpose(Ic);  // shift(Xp, Ic)
    Matrix6 xix = Xp.matrix_transpose() * Ic.matrix() * Xp.matrix();
    ArticulatedBodyInertia delta_I = xix;
    // ArticulatedBodyInertia delta_I = Xp.matrix() * Ic.matrix() * Xp.matrix_transpose();
    if (parent >= 0) {
      mb[parent].abi += delta_I;
    } else if (mb.is_floating()) {
      mb.base_abi() += delta_I;
    }
    ForceVector Fi = Ic * link.S;  // Ic.mul_inv(link.S);
    int qd_i = link.qd_index;
    if (link.joint_type == JOINT_FIXED) continue;

    (*M)(qd_i, qd_i) = Algebra::dot(link.S, Fi);

    int j = i;
    while (mb[j].parent_index != -1) {
      Fi = mb[j].X_parent.apply(Fi);
      j = mb[j].parent_index;
      if (mb[j].joint_type == JOINT_FIXED) continue;
      int qd_j = mb[j].qd_index;
      (*M)(qd_i, qd_j) = Algebra::dot(Fi, mb[j].S);
      (*M)(qd_j, qd_i) = (*M)(qd_i, qd_j);
    }

    if (mb.is_floating()) {
      Fi = mb[j].X_parent.apply(Fi);
      Algebra::assign_column(*M, qd_i, Fi);
      Algebra::assign_row(*M, qd_i, Fi);
    }
  }
  if (mb.is_floating()) {
    Matrix3 Ht = Algebra::transpose(mb.base_abi().H);
    // assign Ic_0 to M(0:6, 0:6)
    Algebra::assign_block(*M, mb.base_abi().I, 0, 0);
    Algebra::assign_block(*M, mb.base_abi().H, 0, 3);
    Algebra::assign_block(*M, Ht, 3, 0);
    Algebra::assign_block(*M, mb.base_abi().M, 3, 3);
  }

  // Algebra::print("Mass matrix", *M);
}

template <typename Algebra>
void mass_matrix(MultiBody<Algebra> &mb, typename Algebra::MatrixX *M) {
  mass_matrix(mb, mb.q(), M);
}
}  // namespace tds