#pragma once

#include "../multi_body.hpp"

namespace tds {
/**
 * Semi-implicit Euler integration.
 */
template <typename Algebra>
void integrate_euler(MultiBody<Algebra> &mb, typename Algebra::VectorX &q,
                     typename Algebra::VectorX &qd,
                     const typename Algebra::VectorX &qdd,
                     const typename Algebra::Scalar &dt) {
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;

  assert(Algebra::size(q) == mb.dof());
  assert(Algebra::size(qd) == mb.dof_qd());
  assert(Algebra::size(qdd) == mb.dof_qd());

  int q_offset, qd_offset;
  if (mb.is_floating()) {
    mb.base_acceleration().top = Vector3(qdd[0], qdd[1], qdd[2]);
    mb.base_acceleration().bottom = Vector3(qdd[3], qdd[4], qdd[5]);
    // Algebra::print("mb.base_acceleration()", mb.base_acceleration());
    // Algebra::print("qdd", qdd);

    mb.base_velocity().top = Vector3(qd[0], qd[1], qd[2]);
    mb.base_velocity().bottom = Vector3(qd[3], qd[4], qd[5]);

    mb.base_velocity() += mb.base_acceleration() * dt;
    // Algebra::print("mb.base_velocity()", mb.base_velocity());
    qd[0] = mb.base_velocity().top[0];
    qd[1] = mb.base_velocity().top[1];
    qd[2] = mb.base_velocity().top[2];

    const Vector3 &linear_velocity = mb.base_velocity().bottom;
    mb.base_X_world().translation += linear_velocity * dt;

    // update base orientation using Quaternion derivative
    const Vector3 &angular_velocity = mb.base_velocity().top;

    Quaternion base_rot = Algebra::quat_from_xyzw(q[0], q[1], q[2], q[3]);
    // Quaternion base_rot =
    // Algebra::matrix_to_quat(mb.base_X_world().rotation); Algebra::print("Base
    // quat (TDS): ", base_rot); Algebra::print("base_rot", base_rot); update
    // 4-dimensional q from 3-dimensional qd for the base rotation
    Algebra::quat_increment(
        base_rot, Algebra::quat_velocity(base_rot, angular_velocity, dt));
    Algebra::normalize(base_rot);
    mb.base_X_world().rotation = Algebra::quat_to_matrix(base_rot);

    q[0] = Algebra::quat_x(base_rot);
    q[1] = Algebra::quat_y(base_rot);
    q[2] = Algebra::quat_z(base_rot);
    q[3] = Algebra::quat_w(base_rot);
    q_offset = 4;
    qd_offset = 3;
  } else {
    q_offset = 0;
    qd_offset = 0;
  }

  for (int i = 0; i < mb.dof_qd() - qd_offset; i++) {
    int qindex = i + q_offset;
    int qdindex = i + qd_offset;
    qd[qdindex] += qdd[qdindex] * dt;
    q[qindex] += qd[qdindex] * dt;
  }
}

/**
 * Part of semi-implicit Euler integration.
 * Integrate acceleration into change of velocity.
 * This presents the unconstrained velocity to the constraint solver.
 */
template <typename Algebra>
void integrate_euler_qdd(MultiBody<Algebra>& mb, typename Algebra::VectorX& q,
    typename Algebra::VectorX& qd,
    const typename Algebra::VectorX& qdd,
    const typename Algebra::Scalar& dt) {
    using Vector3 = typename Algebra::Vector3;
    using Quaternion = typename Algebra::Quaternion;

    assert(Algebra::size(q) == mb.dof());
    assert(Algebra::size(qd) == mb.dof_qd());
    assert(Algebra::size(qdd) == mb.dof_qd());

    int q_offset, qd_offset;
    if (mb.is_floating()) {
        mb.base_acceleration().top = Vector3(qdd[0], qdd[1], qdd[2]);
        mb.base_acceleration().bottom = Vector3(qdd[3], qdd[4], qdd[5]);
        // Algebra::print("mb.base_acceleration()", mb.base_acceleration());
        // Algebra::print("qdd", qdd);

        mb.base_velocity().top = Vector3(qd[0], qd[1], qd[2]);
        mb.base_velocity().bottom = Vector3(qd[3], qd[4], qd[5]);

        mb.base_velocity() += mb.base_acceleration() * dt;
        mb.base_acceleration().set_zero();

        // Algebra::print("mb.base_velocity()", mb.base_velocity());
        qd[0] = mb.base_velocity().top[0];
        qd[1] = mb.base_velocity().top[1];
        qd[2] = mb.base_velocity().top[2];
        q_offset = 4;
        qd_offset = 3;
    }
    else {
        q_offset = 0;
        qd_offset = 0;
    }

    for (int i = 0; i < mb.dof_qd() - qd_offset; i++) {
        int qindex = i + q_offset;
        int qdindex = i + qd_offset;
        qd[qdindex] += qdd[qdindex] * dt;
    }
}

template <typename Algebra>
void integrate_euler(MultiBody<Algebra> &mb,
                     const typename Algebra::Scalar &dt) {
  integrate_euler(mb, mb.q(), mb.qd(), mb.qdd(), dt);
}

template <typename Algebra>
void integrate_euler_qdd(MultiBody<Algebra>& mb,
    const typename Algebra::Scalar& dt) {
    integrate_euler_qdd(mb, mb.q(), mb.qd(), mb.qdd(), dt);
    mb.qdd().set_zero();
}
}  // namespace tds