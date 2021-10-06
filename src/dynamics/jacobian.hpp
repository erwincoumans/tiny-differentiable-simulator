#pragma once

#include "kinematics.hpp"

namespace tds {

/**
 * Computes the body Jacobian for a point in world frame on certain link.
 * This function does not update the robot configuration with the given
 * joint positions.
 */
template <typename Algebra>
typename Algebra::Matrix3X point_jacobian(
    const MultiBody<Algebra> &mb, const typename Algebra::VectorX &q, int link_index,
    const typename Algebra::Vector3 &point, bool is_local_point) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix6x3 = typename Algebra::Matrix6x3;
  using Matrix3X = typename Algebra::Matrix3X;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::Link<Algebra> Link;


  assert(Algebra::size(q)  == mb.dof());
  assert(link_index < static_cast<int>(mb.num_links()));
  Matrix3X jac( 3, mb.dof_qd());
  Algebra::set_zero(jac);
  std::vector<Transform> links_X_world;
  std::vector<Transform> links_X_base;
  Transform base_X_world;
  forward_kinematics_q(mb, q, &base_X_world, &links_X_world, &links_X_base);
  Transform point_tf;
  point_tf.set_identity();
  point_tf.translation = point;
                         
  if (mb.is_floating()) {
      Vector3 base_point = is_local_point ? point : point - mb.base_X_world().translation;

    // see (Eq. 2.238) in
    // https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/FloatingBaseKinematics.pdf
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    // Matrix3 cr = Algebra::cross_matrix(point_tf.translation);
    Matrix3 cr = Algebra::transpose(Algebra::cross_matrix(base_point));
    Algebra::assign_block(jac, cr, 0, 0);
    jac(0, 3) = Algebra::one();
    jac(1, 4) = Algebra::one();
    jac(2, 5) = Algebra::one();
#else
    Matrix3 cr = Algebra::transpose(Algebra::cross_matrix(base_point));
    Algebra::assign_block(jac, cr, 0, 0);
    jac(0, 3) = Algebra::one();
    jac(1, 4) = Algebra::one();
    jac(2, 5) = Algebra::one();
#endif //TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    
  } else {
    point_tf.translation = point;
  }
  // loop over all links that lie on the path from the given link to world
  if (link_index >= 0) {
    const Link *body = &mb[link_index];
    while (true) {
      int i = body->index;
      if (body->joint_type == JOINT_SPHERICAL){
        Matrix6x3 st = is_local_point ? links_X_base[i].apply_inverse(body->S_3d, false) : links_X_world[i].apply_inverse(body->S_3d, false);
        Matrix6x3 xs = point_tf.apply(st, false);
        Algebra::assign_block(jac, Algebra::bottom(xs), 0, body->qd_index);
      }
      else if (body->joint_type != JOINT_FIXED) {
        MotionVector st = is_local_point ? links_X_base[i].apply_inverse(body->S) : links_X_world[i].apply_inverse(body->S);
        MotionVector xs = point_tf.apply(st);
        Algebra::assign_column(jac, body->qd_index, xs.bottom);
      }
      if (body->parent_index < 0) break;
      body = &mb[body->parent_index];
    }
  }
  //jac.print("jac");
  return jac;
}

template <typename Algebra>
typename Algebra::Matrix3X point_jacobian2(
    MultiBody<Algebra> &mb, int link_index,
    const typename Algebra::Vector3 &point, bool is_local_point) {
  return point_jacobian(mb, mb.q(), link_index, point, is_local_point);
}

/**
 * Estimate the point Jacobian using finite differences.
 * This function should only be called for testing purposes.
 */
template <typename Algebra>
typename Algebra::Matrix3X point_jacobian_fd(
    const MultiBody<Algebra> &mb, const typename Algebra::VectorX &q,
    int link_index, const typename Algebra::Vector3 &start_point,
    const typename Algebra::Scalar &eps = Algebra::fraction(1, 1000)) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using VectorX = typename Algebra::VectorX;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix3X = typename Algebra::Matrix3X;
  using Quaternion = typename Algebra::Quaternion;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::Link<Algebra> Link;

  assert(Algebra::size(q) == mb.dof());
  assert(link_index < static_cast<int>(mb.size()));
  Matrix3X jac(3, mb.dof_qd());
  Algebra::set_zero(jac);
  std::vector<Transform> links_X_world;
  Transform base_X_world;
  // compute world point transform for the initial joint angles
  forward_kinematics_q(mb, q, &base_X_world, &links_X_world);
  // if (mb.empty()) {
  //   return jac;
  // }
  // convert start point in world coordinates to base frame
  const Vector3
      base_point =   start_point - base_X_world.translation;
                    // start_point - (mb.empty() ? base_X_world.translation
                    //                 : links_X_world[link_index].translation);
      //mb.empty() ? base_X_world.apply_inverse(start_point)
      //           : links_X_world[link_index].apply_inverse(start_point);
  Vector3 world_point;

  VectorX q_x;
  Transform base_X_world_temp;
  for (int i = 0; i < mb.dof_qd(); ++i) {
    q_x = q;
    if (mb.is_floating() && i < 3) {
      // special handling of quaternion differencing via angular velocity
      Quaternion base_rot = Algebra::matrix_to_quat(base_X_world.rotation);

      Vector3 angular_velocity = Algebra::zero3();
      angular_velocity[i] = Algebra::one();
      Algebra::quat_increment(
              base_rot, Algebra::quat_velocity(base_rot, angular_velocity, eps * Algebra::half()));
      // base_rot += (angular_velocity * base_rot) * (eps * Algebra::half());
      base_rot.normalize();
      q_x[0] = Algebra::quat_x(base_rot);
      q_x[1] = Algebra::quat_y(base_rot);
      q_x[2] = Algebra::quat_z(base_rot);
      q_x[3] = Algebra::quat_w(base_rot);
    } else {
      // adjust for the +1 offset with the 4 DOF orientation in q vs. 3 in qd
      int q_index = mb.is_floating() ? i + 1 : i;
      q_x[q_index] += eps;
    }
    // Algebra::print(("q_x " + std::to_string(i)).c_str(), q_x);
    forward_kinematics_q(mb, q_x, &base_X_world_temp, &links_X_world);
    world_point = mb.empty() ? base_X_world_temp.apply(base_point)
                             : links_X_world[link_index].apply(base_point);
    for (int j = 0; j < 3; ++j) {
      jac(j, i) = (world_point[j] - start_point[j]) / eps;
    }
  }

  return jac;
}

template <typename Algebra>
typename Algebra::Matrix3X point_jacobian_fd(
    const MultiBody<Algebra> &mb, int link_index,
    const typename Algebra::Vector3 &start_point,
    const typename Algebra::Scalar &eps = Algebra::fraction(1, 1000)) {
  return point_jacobian_fd(mb, mb.q(), link_index, start_point, eps);
}

}  // namespace tds