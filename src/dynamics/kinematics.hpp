#pragma once

#define SWAP_TRANSFORM_ASSOCIATIVITY false

#include "../multi_body.hpp"

namespace tds {
/**
 * Implements the first phase in ABA, CRBA and RNEA, that computes the
 * joint and body transforms, velocities and bias forces.
 * Initializes articulated inertia with the local body inertia.
 *
 * Joint positions q must have dimension of dof().
 * Joint velocities qd must have dimension of dof_qd().
 * If no joint velocities qd are given, qd is assumed to be zero.
 * If no joint accelerations qdd are given, qdd is assumed to be zero.
 */
template <typename Algebra>
void forward_kinematics(
    MultiBody<Algebra> &mb, const typename Algebra::VectorX &q,
    const typename Algebra::VectorX &qd,
    const typename Algebra::VectorX &qdd = typename Algebra::VectorX()) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::Link<Algebra> Link;

  assert(Algebra::size(q) == mb.dof());
  assert(Algebra::size(qd) == 0 || Algebra::size(qd) == mb.dof_qd());
  assert(Algebra::size(qdd) == 0 || Algebra::size(qdd) == mb.dof_qd());

  if (mb.is_floating()) {
    // update base-world transform from q, and update base velocity from qd
    mb.base_X_world().rotation =
        Algebra::quat_to_matrix(q[0], q[1], q[2], q[3]);
    mb.base_X_world().translation = Vector3(q[4], q[5], q[6]);
    if (Algebra::size(qd) != 0) {
      mb.base_velocity().top = Vector3(qd[0], qd[1], qd[2]);
      mb.base_velocity().bottom = Vector3(qd[3], qd[4], qd[5]);
    } else {
      mb.base_velocity().set_zero();
    }

    mb.base_abi() = mb.base_rbi();
    ForceVector I0_mul_v0 = mb.base_abi() * mb.base_velocity();
    mb.base_bias_force() =
        Algebra::cross(mb.base_velocity(), I0_mul_v0) - mb.base_applied_force();
  }

  for (int i = 0; i < static_cast<int>(mb.size()); i++) {
    const Link &link = mb[i];
    int parent = link.parent_index;

    // update joint transforms, joint velocity (if available)
    Scalar q_val = mb.get_q_for_link(q, i);
    Scalar qd_val = mb.get_qd_for_link(qd, i);
    link.jcalc(q_val, qd_val);

    // std::cout << "Link " << i << " transform: " << link.X_parent <<
    // std::endl;

    if (parent >= 0 || mb.is_floating()) {
      const Transform &parent_X_world =
          parent >= 0 ? mb[parent].X_world : mb.base_X_world();
#if SWAP_TRANSFORM_ASSOCIATIVITY
      link.X_world = link.X_parent * parent_X_world;  // RBDL style
#else
      link.X_world = parent_X_world * link.X_parent;
#endif
      const MotionVector &parentVelocity =
          parent >= 0 ? mb[parent].v : mb.base_velocity();
      MotionVector xv = link.X_parent.apply(parentVelocity);
      link.v = xv + link.vJ;
    } else {
      #if SWAP_TRANSFORM_ASSOCIATIVITY
      link.X_world = link.X_parent * mb.base_X_world();
      #else
      link.X_world = mb.base_X_world() * link.X_parent;
      #endif
      link.v = link.vJ;
    }
    MotionVector v_x_vJ = Algebra::cross(link.v, link.vJ);
    link.c = v_x_vJ /*+link.c_J[i]*/;

    link.abi = link.rbi;
    ForceVector I_mul_v = link.abi * link.v;
    ForceVector f_ext = link.X_world.apply_inverse(link.f_ext);
    // #ifdef NEURAL_SIM
    //       if (i >= 3) {
    //         if constexpr (is_neural_scalar<Scalar, Algebra>::value) {
    //           // Inputs: Position.
    //           link.X_world.translation[0].assign("link/pos/x");
    //           link.X_world.translation[1].assign("link/pos/y");
    //           Scalar link_pos_yaw = Algebra::atan2(
    //               link.X_world.rotation(0, 1), link.X_world.rotation(0,
    //               0));
    //           link_pos_yaw.assign("link/pos/yaw");

    //           // Inputs: Velocity.
    //           link.v[3].assign("link/vel/x");
    //           link.v[4].assign("link/vel/y");
    //           link.v[2].assign("link/vel/yaw");

    //           // Outputs: Applied Force.
    //           f_ext[3].assign("link/external_force/x");
    //           f_ext[4].assign("link/external_force/y");
    //           f_ext[2].assign("link/external_force/yaw");

    //           // Cache the outputs.
    //           f_ext[3].evaluate();
    //           f_ext[4].evaluate();
    //           f_ext[2].evaluate();
    //         }
    //       }
    // #endif

    link.pA = Algebra::cross(link.v, I_mul_v) - f_ext;
#ifdef DEBUG
    // Algebra::print("link.abi", link.abi);
    Algebra::print("I_mul_v", I_mul_v);
    Algebra::print("link.pA", link.pA);
#endif
    // compute helper temporary variables for floating-base RNEA
    // const SpatialVector &parent_a =
    //     parent >= 0 ? links[parent].a : mb.base_acceleration_;
    // link.a = link.X_parent.apply(parent_a) + v_x_vJ;
    // if (!qdd.empty()) {
    //   link.a += link.S * get_qdd_for_link(qdd, i);
    // }
    // link.f = link.abi * link.a + link.pA;
  }
}

template <typename Algebra>
void forward_kinematics(MultiBody<Algebra> &mb,
                        const typename Algebra::VectorX &q) {
  forward_kinematics(mb, q, typename Algebra::VectorX());
}

template <typename Algebra>
void forward_kinematics(MultiBody<Algebra> &mb) {
  forward_kinematics(mb, mb.q(), typename Algebra::VectorX());
}

/**
 * Computes the forward kinematics given joint positions q and assigns
 * base_X_world the base transform in world frame, and optionally the link
 * transforms in world and in base frame.
 * Input q must have dimensions of dof().
 */
template <typename Algebra>
void forward_kinematics_q(
    const MultiBody<Algebra> &mb, const typename Algebra::VectorX &q,
    tds::Transform<Algebra> *base_X_world,
    std::vector<tds::Transform<Algebra>> *links_X_world = nullptr,
    std::vector<tds::Transform<Algebra>> *links_X_base = nullptr) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::Link<Algebra> Link;

  assert(Algebra::size(q) == mb.dof());
  assert(base_X_world != nullptr);

  if (mb.is_floating()) {
    base_X_world->rotation = Algebra::quat_to_matrix(q[0], q[1], q[2], q[3]);
    base_X_world->translation = Vector3(q[4], q[5], q[6]);
  } else {
    *base_X_world = mb.base_X_world();
  }
  if (links_X_world) links_X_world->resize(mb.size());
  if (links_X_base) links_X_base->resize(mb.size());
  Transform x_j;
  Transform x_parent;
  Transform ident;
  ident.set_identity();
  for (std::size_t i = 0; i < mb.size(); i++) {
    const Link &link = mb[i];
    int parent = link.parent_index;

    Scalar q_val = mb.get_q_for_link(q, i);
    link.jcalc(q_val, &x_j, &x_parent);

    if (parent >= 0 || mb.is_floating()) {
      if (links_X_world) {
        const Transform &parent_X_world =
            parent >= 0 ? (*links_X_world)[parent] : *base_X_world;

#if SWAP_TRANSFORM_ASSOCIATIVITY
        (*links_X_world)[i] = x_parent * parent_X_world;
#else
        (*links_X_world)[i] = parent_X_world * x_parent;
#endif
      }
      if (links_X_base) {
        const Transform &parent_X_base =
            parent >= 0 ? (*links_X_base)[parent] : ident;

#if SWAP_TRANSFORM_ASSOCIATIVITY
        (*links_X_base)[i] = x_parent * parent_X_base;
#else
        (*links_X_base)[i] = parent_X_base * x_parent;
#endif
      }
    } else {
      // first link in fixed-base system

#if SWAP_TRANSFORM_ASSOCIATIVITY
      if (links_X_world) (*links_X_world)[i] = x_parent * *base_X_world;
#else
      if (links_X_world) (*links_X_world)[i] = *base_X_world * x_parent;
#endif
      if (links_X_base) (*links_X_base)[i] = x_parent;
    }
  }
}
}  // namespace tds