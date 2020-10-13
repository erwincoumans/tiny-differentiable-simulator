#pragma once

#include <vector>

#include "geometry.hpp"
#include "multi_body.hpp"
#include "world.hpp"

template <typename Algebra>
void init_compound_pendulum(
    tds::MultiBody<Algebra> &mb, tds::World<Algebra> &world, int num_links = 2,
    std::vector<typename Algebra::Scalar> link_lengths = {},
    std::vector<typename Algebra::Scalar> masses = {}) {
  typedef typename Algebra::Scalar Scalar;
  typedef typename Algebra::Vector3 Vector3;
  typedef typename Algebra::Matrix3 Matrix3;
  using Link = tds::Link<Algebra>;
  using Sphere = tds::Sphere<Algebra>;
  using RigidBodyInertia = tds::RigidBodyInertia<Algebra>;
  using Transform = tds::Transform<Algebra>;

  for (int i = 0; i < num_links; i++) {
    Link l;
    l.set_joint_type(tds::JOINT_REVOLUTE_X);
    l.X_T.rotation = Algebra::eye3();
    Scalar pos_y = i == 0 ? Algebra::zero()
                          : (!link_lengths.empty() ? link_lengths[i]
                                                   : Algebra::fraction(5, 10));

    l.X_T.translation = Vector3(Algebra::zero(), pos_y, Algebra::zero());
    Scalar mass =
        static_cast<int>(masses.size()) > i ? masses[i] : Algebra::one();
    Vector3 com;
    Scalar length =
        !link_lengths.empty() ? link_lengths[i] : Algebra::fraction(5, 10);

    com = Vector3(Algebra::zero(), length, Algebra::zero());

    Scalar radius = Algebra::fraction(15, 100);
    Sphere *sphere = world.create_sphere(radius);
    Vector3 local_inertia = sphere->compute_local_inertia(mass);
    Matrix3 inertia_C = Algebra::diagonal3(local_inertia);
    l.rbi = RigidBodyInertia(mass, com, inertia_C);
    Transform base_X_geom;
    base_X_geom.set_identity();
    base_X_geom.translation = com;
    l.collision_geometries.push_back(sphere);
    l.X_collisions.push_back(base_X_geom);
    l.X_visuals.push_back(base_X_geom);
    // l.rbi.print("inertia");
    mb.attach(l);
  }

  mb.initialize();
}

template <typename Algebra>
void double_pendulum_trajectory(
    typename Algebra::Scalar l1, typename Algebra::Scalar l2,
    typename Algebra::Scalar m1, typename Algebra::Scalar m2,
    typename Algebra::Scalar q1, typename Algebra::Scalar q2,
    typename Algebra::Scalar qd1, typename Algebra::Scalar qd2,
    typename Algebra::Scalar g, typename Algebra::Scalar dt, int steps,
    std::vector<typename Algebra::Scalar> *q1s,
    std::vector<typename Algebra::Scalar> *q2s,
    std::vector<typename Algebra::Scalar> *qd1s,
    std::vector<typename Algebra::Scalar> *qd2s) {
  // Reference: https://www.myphysicslab.com/pendulum/double-pendulum-en.html

  typedef typename Algebra::Scalar Scalar;

  Scalar two = Algebra::two();

  // convert joint angles to world space
  q1 += Algebra::half_pi();
  q2 += Algebra::half_pi();

  q1s->reserve(steps);
  q2s->reserve(steps);
  qd1s->reserve(steps);
  qd2s->reserve(steps);
  Scalar qdd1, qdd2;
  Scalar denom, s12, c12;

  for (int t = 0; t < steps; ++t) {
    // joint angles are simulated in world space, need to convert to joint space
    (*q1s)[t] = q1 - Algebra::half_pi();
    (*q2s)[t] = q2 - q1;
    (*qd1s)[t] = qd1;
    (*qd2s)[t] = qd2 - qd1;

    s12 = Algebra::sin(q1 - q2);
    c12 = Algebra::cos(q1 - q2);
    denom = two * m1 + m2 - m2 * Algebra::cos(two * (q1 - q2));

    qdd1 = -g * (two * m1 + m2) * Algebra::sin(q1) -
           m2 * g * Algebra::sin(q1 - two * q2) -
           two * m2 * qd2 * qd2 * l2 * s12 -
           m2 * qd1 * qd1 * l1 * Algebra::sin(two * (q1 - q2));
    qdd1 = qdd1 / (l1 * denom);

    qdd2 = two * s12 *
           (qd1 * qd1 * l1 * (m1 + m2) + g * (m1 + m2) * Algebra::cos(q1) +
            qd2 * qd2 * l2 * m2 * c12);
    qdd2 = qdd2 / (l2 * denom);

    qd1 += dt * qdd1;
    qd2 += dt * qdd2;
    q1 += dt * qd1;
    q2 += dt * qd2;
  }
}