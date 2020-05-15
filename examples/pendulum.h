/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PENDULUM_H
#define PENDULUM_H

#include <vector>

#include "tiny_geometry.h"
#include "tiny_multi_body.h"
#include "tiny_spatial_transform.h"
#include "tiny_world.h"

template <typename TinyScalar, typename TinyConstants>
void init_compound_pendulum(TinyMultiBody<TinyScalar, TinyConstants> &mb,
                            TinyWorld<TinyScalar, TinyConstants> &world,
                            int num_links = 2,
                            std::vector<TinyScalar> link_lengths = {}) {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
  typedef TinySymmetricSpatialDyad<TinyScalar, TinyConstants>
      TinySymmetricSpatialDyad;
  typedef TinySpatialTransform<TinyScalar, TinyConstants> TinySpatialTransform;

  for (int i = 0; i < num_links; i++) {
    TinyLink<TinyScalar, TinyConstants> l;
    l.set_joint_type(JOINT_REVOLUTE_X);
    l.m_X_T.m_rotation.set_identity();
    TinyScalar pos_y =
        !link_lengths.empty()
            ? link_lengths[i]
            : (i == 0 ? TinyConstants::zero() : TinyConstants::fraction(5, 10));

    l.m_X_T.m_translation.setValue(TinyConstants::zero(), pos_y,
                                   TinyConstants::zero());
    TinyScalar mass = TinyConstants::fraction(1, 1);
    TinyVector3 com;
    TinyScalar length = !link_lengths.empty() ? link_lengths[i]
                                              : TinyConstants::fraction(5, 10);
    com.setValue(TinyConstants::zero(), length, TinyConstants::zero());

    TinyScalar radius = TinyConstants::fraction(5, 100);
    TinySphere<TinyScalar, TinyConstants> *sphere = world.create_sphere(radius);
    TinyVector3 local_inertia = sphere->compute_local_inertia(mass);
    TinyMatrix3x3 inertia_C;
    inertia_C.set_identity();
    inertia_C(0, 0) = local_inertia[0];
    inertia_C(1, 1) = local_inertia[1];
    inertia_C(2, 2) = local_inertia[2];
    l.m_I = TinySymmetricSpatialDyad::computeInertiaDyad(mass, com, inertia_C);
    TinySpatialTransform base_X_geom;
    base_X_geom.set_identity();
    base_X_geom.m_translation = com;
    l.m_collision_geometries.push_back(sphere);
    l.m_X_collisions.push_back(base_X_geom);
    l.m_X_visuals.push_back(base_X_geom);
    // l.m_I.print("inertia");
    mb.attach(l);
  }
  mb.initialize();
}

template <typename TinyScalar, typename TinyConstants>
void double_pendulum_trajectory(
    TinyScalar l1, TinyScalar l2, TinyScalar m1, TinyScalar m2, TinyScalar q1,
    TinyScalar q2, TinyScalar qd1, TinyScalar qd2, TinyScalar g, TinyScalar dt,
    int steps, std::vector<TinyScalar> *q1s, std::vector<TinyScalar> *q2s,
    std::vector<TinyScalar> *qd1s, std::vector<TinyScalar> *qd2s) {
  // Reference: https://www.myphysicslab.com/pendulum/double-pendulum-en.html
  TinyScalar two = TinyConstants::two();
  // convert joint angles to world space
  q1 += TinyConstants::half_pi();
  q2 += TinyConstants::half_pi();
  q1s->reserve(steps);
  q2s->reserve(steps);
  qd1s->reserve(steps);
  qd2s->reserve(steps);
  TinyScalar qdd1, qdd2;
  TinyScalar denom, s12, c12;
  for (int t = 0; t < steps; ++t) {
    // joint angles are simulated in world space, need to convert to joint space
    (*q1s)[t] = q1 - TinyConstants::half_pi();
    (*q2s)[t] = q2 - q1;
    (*qd1s)[t] = qd1;
    (*qd2s)[t] = qd2 - qd1;

    s12 = TinyConstants::sin1(q1 - q2);
    c12 = TinyConstants::cos1(q1 - q2);
    denom = two * m1 + m2 - m2 * TinyConstants::cos1(two * (q1 - q2));

    qdd1 = -g * (two * m1 + m2) * TinyConstants::sin1(q1) -
           m2 * g * TinyConstants::sin1(q1 - two * q2) -
           two * m2 * qd2 * qd2 * l2 * s12 -
           m2 * qd1 * qd1 * l1 * TinyConstants::sin1(two * (q1 - q2));
    qdd1 = qdd1 / (l1 * denom);

    qdd2 =
        two * s12 *
        (qd1 * qd1 * l1 * (m1 + m2) + g * (m1 + m2) * TinyConstants::cos1(q1) +
         qd2 * qd2 * l2 * m2 * c12);
    qdd2 = qdd2 / (l2 * denom);

    qd1 += dt * qdd1;
    qd2 += dt * qdd2;
    q1 += dt * qd1;
    q2 += dt * qd2;
  }
}

#endif  // PENDULUM_H
