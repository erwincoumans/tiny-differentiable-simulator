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

#ifndef TINY_GEOMETRY_H
#define TINY_GEOMETRY_H

#include <vector>

#include "tiny_pose.h"

enum TinyGeometryTypes {
  TINY_SPHERE_TYPE = 0,
  TINY_PLANE_TYPE,
  TINY_CAPSULE_TYPE,
  TINY_MESH_TYPE,      // only for visual shapes at the moment
  TINY_BOX_TYPE,       // only for visual shapes at the moment
  TINY_CYLINDER_TYPE,  // unsupported
  TINY_MAX_GEOM_TYPE,
};

template <typename TinyScalar, typename TinyConstants>
class TinyGeometry {
  int m_type;

 public:
  explicit TinyGeometry(int type) : m_type(type) {}
  virtual ~TinyGeometry() {}
  int get_type() const { return m_type; }
};

template <typename TinyScalar, typename TinyConstants>
class TinySphere : public TinyGeometry<TinyScalar, TinyConstants> {
  TinyScalar m_radius;

 public:
  explicit TinySphere(TinyScalar radius)
      : TinyGeometry<TinyScalar, TinyConstants>(TINY_SPHERE_TYPE),
        m_radius(radius) {}

  TinyScalar get_radius() const { return m_radius; }

  TinyVector3<TinyScalar, TinyConstants> compute_local_inertia(
      TinyScalar mass) const {
    TinyScalar elem =
        TinyConstants::fraction(4, 10) * mass * m_radius * m_radius;
    return TinyVector3<TinyScalar, TinyConstants>(elem, elem, elem);
  }
};

// capsule aligned with the Z axis
template <typename TinyScalar, typename TinyConstants>
class TinyCapsule : public TinyGeometry<TinyScalar, TinyConstants> {
  TinyScalar m_radius;
  TinyScalar m_length;

 public:
  explicit TinyCapsule(TinyScalar radius, TinyScalar length)
      : TinyGeometry<TinyScalar, TinyConstants>(TINY_CAPSULE_TYPE),
        m_radius(radius),
        m_length(length) {}

  TinyScalar get_radius() const { return m_radius; }
  TinyScalar get_length() const { return m_length; }

  TinyVector3<TinyScalar, TinyConstants> compute_local_inertia(
      TinyScalar mass) const {
    TinyScalar lx = TinyConstants::fraction(2, 1) * (m_radius);
    TinyScalar ly = TinyConstants::fraction(2, 1) * (m_radius);
    TinyScalar lz = m_length + TinyConstants::fraction(2, 1) * (m_radius);
    TinyScalar x2 = lx * lx;
    TinyScalar y2 = ly * ly;
    TinyScalar z2 = lz * lz;
    TinyScalar scaledmass = mass * TinyConstants::fraction(1, 12);

    TinyVector3<TinyScalar, TinyConstants> inertia;
    inertia[0] = scaledmass * (y2 + z2);
    inertia[1] = scaledmass * (x2 + z2);
    inertia[2] = scaledmass * (x2 + y2);
    return inertia;
  }
};

template <typename TinyScalar, typename TinyConstants>
class TinyPlane : public TinyGeometry<TinyScalar, TinyConstants> {
  TinyVector3<TinyScalar, TinyConstants> m_normal;
  TinyScalar m_constant;

 public:
  TinyPlane()
      : TinyGeometry<TinyScalar, TinyConstants>(TINY_PLANE_TYPE),
        m_normal(TinyConstants::zero(), TinyConstants::zero(),
                 TinyConstants::one()),
        m_constant(TinyConstants::zero()) {}

  const TinyVector3<TinyScalar, TinyConstants>& get_normal() const {
    return m_normal;
  }
  TinyScalar get_constant() const { return m_constant; }
};

template <typename TinyScalar, typename TinyConstants>
struct TinyContactPoint {
  TinyVector3<TinyScalar, TinyConstants> m_world_normal_on_b;
  TinyVector3<TinyScalar, TinyConstants> m_world_point_on_a;
  TinyVector3<TinyScalar, TinyConstants> m_world_point_on_b;
  TinyScalar m_distance;
};

template <typename TinyScalar, typename TinyConstants>
int contactSphereSphere(
    const TinyGeometry<TinyScalar, TinyConstants>* geomA,
    const TinyPose<TinyScalar, TinyConstants>& poseA,
    const TinyGeometry<TinyScalar, TinyConstants>* geomB,
    const TinyPose<TinyScalar, TinyConstants>& poseB,
    std::vector<TinyContactPoint<TinyScalar, TinyConstants> >& contactsOut) {
  TinyScalar CONTACT_EPSILON = TinyConstants::fraction(1, 100000);
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinySphere<TinyScalar, TinyConstants> TinySphere;
  typedef ::TinyContactPoint<TinyScalar, TinyConstants> TinyContactPoint;
  assert(geomA->get_type() == TINY_SPHERE_TYPE);
  assert(geomB->get_type() == TINY_SPHERE_TYPE);
  TinySphere* sphereA = (TinySphere*)geomA;
  TinySphere* sphereB = (TinySphere*)geomB;

  TinyVector3 diff = poseA.m_position - poseB.m_position;
  TinyScalar length = diff.length();
  TinyScalar distance =
      length - (sphereA->get_radius() + sphereB->get_radius());
  TinyVector3 normal_on_b;
  normal_on_b.setValue(TinyConstants::one(), TinyConstants::zero(),
                       TinyConstants::zero());
  if (length > CONTACT_EPSILON) {
    TinyVector3 normal_on_b = TinyConstants::one() / length * diff;
    TinyVector3 point_a_world =
        poseA.m_position - sphereA->get_radius() * normal_on_b;
    TinyVector3 point_b_world = point_a_world - distance * normal_on_b;
    TinyContactPoint pt;
    pt.m_world_normal_on_b = normal_on_b;
    pt.m_world_point_on_a = point_a_world;
    pt.m_world_point_on_b = point_b_world;
    pt.m_distance = distance;
    contactsOut.push_back(pt);
    return 1;
  }
  return 0;
}

template <typename TinyScalar, typename TinyConstants>
int contactPlaneSphere(
    const TinyGeometry<TinyScalar, TinyConstants>* geomA,
    const TinyPose<TinyScalar, TinyConstants>& poseA,
    const TinyGeometry<TinyScalar, TinyConstants>* geomB,
    const TinyPose<TinyScalar, TinyConstants>& poseB,
    std::vector<TinyContactPoint<TinyScalar, TinyConstants> >& contactsOut) {
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinySphere<TinyScalar, TinyConstants> TinySphere;
  typedef ::TinyPlane<TinyScalar, TinyConstants> TinyPlane;
  typedef ::TinyContactPoint<TinyScalar, TinyConstants> TinyContactPoint;
  assert(geomA->get_type() == TINY_PLANE_TYPE);
  assert(geomB->get_type() == TINY_SPHERE_TYPE);
  TinyPlane* planeA = (TinyPlane*)geomA;
  TinySphere* sphereB = (TinySphere*)geomB;

  TinyScalar t =
      -(poseB.m_position.dot(-planeA->get_normal()) + planeA->get_constant());
  TinyVector3 pointAWorld = poseB.m_position + t * -planeA->get_normal();
  TinyScalar distance = t - sphereB->get_radius();
  TinyVector3 pointBWorld =
      poseB.m_position - sphereB->get_radius() * planeA->get_normal();
  TinyContactPoint pt;
  pt.m_world_normal_on_b = -planeA->get_normal();
  pt.m_world_point_on_a = pointAWorld;
  pt.m_world_point_on_b = pointBWorld;
  pt.m_distance = distance;
  contactsOut.push_back(pt);
  return 1;
}

template <typename TinyScalar, typename TinyConstants>
int contactPlaneCapsule(
    const TinyGeometry<TinyScalar, TinyConstants>* geomA,
    const TinyPose<TinyScalar, TinyConstants>& poseA,
    const TinyGeometry<TinyScalar, TinyConstants>* geomB,
    const TinyPose<TinyScalar, TinyConstants>& poseB,
    std::vector<TinyContactPoint<TinyScalar, TinyConstants> >& contactsOut) {
  typedef ::TinyPose<TinyScalar, TinyConstants> TinyPose;
  typedef ::TinyPlane<TinyScalar, TinyConstants> TinyPlane;
  typedef ::TinyCapsule<TinyScalar, TinyConstants> TinyCapsule;
  typedef ::TinyContactPoint<TinyScalar, TinyConstants> TinyContactPoint;
  typedef ::TinySphere<TinyScalar, TinyConstants> TinySphere;
  assert(geomA->get_type() == TINY_PLANE_TYPE);
  assert(geomB->get_type() == TINY_CAPSULE_TYPE);
  TinyCapsule* capsule = (TinyCapsule*)geomB;

  // create twice a plane-sphere contact
  TinySphere sphere(capsule->get_radius());
  // shift the sphere to each end-point
  TinyPose offset;
  offset.m_orientation.set_identity();
  offset.m_position.setValue(
      TinyConstants::zero(), TinyConstants::zero(),
      TinyConstants::fraction(1, 2) * capsule->get_length());
  TinyPose poseEndSphere = poseB * offset;
  contactPlaneSphere<TinyScalar, TinyConstants>(geomA, poseA, &sphere,
                                                poseEndSphere, contactsOut);
  offset.m_position.setValue(
      TinyConstants::zero(), TinyConstants::zero(),
      TinyConstants::fraction(-1, 2) * capsule->get_length());
  poseEndSphere = poseB * offset;
  contactPlaneSphere<TinyScalar, TinyConstants>(geomA, poseA, &sphere,
                                                poseEndSphere, contactsOut);

  return 2;
}

template <typename TinyScalar, typename TinyConstants>
struct TinyCollisionDispatcher {
  typedef ::TinyGeometry<TinyScalar, TinyConstants> TinyGeometry;
  typedef ::TinyPose<TinyScalar, TinyConstants> TinyPose;
  typedef ::TinyContactPoint<TinyScalar, TinyConstants> TinyContactPoint;

  typedef int (*contact_func)(const TinyGeometry* geomA, const TinyPose& poseA,
                              const TinyGeometry* geomB, const TinyPose& poseB,
                              std::vector<TinyContactPoint>& contactsOut);

  contact_func m_contactFuncs[TINY_MAX_GEOM_TYPE][TINY_MAX_GEOM_TYPE];

  TinyCollisionDispatcher() {
    for (int i = 0; i < TINY_MAX_GEOM_TYPE; i++) {
      for (int j = 0; j < TINY_MAX_GEOM_TYPE; j++) {
        m_contactFuncs[i][j] = 0;
      }
    }
    m_contactFuncs[TINY_SPHERE_TYPE][TINY_SPHERE_TYPE] = contactSphereSphere;
    m_contactFuncs[TINY_PLANE_TYPE][TINY_SPHERE_TYPE] = contactPlaneSphere;
    m_contactFuncs[TINY_PLANE_TYPE][TINY_CAPSULE_TYPE] = contactPlaneCapsule;
  }

  int computeContacts(const TinyGeometry* geomA, const TinyPose& poseA,
                      const TinyGeometry* geomB, const TinyPose& poseB,
                      std::vector<TinyContactPoint>& contactsOut) {
    contact_func f = m_contactFuncs[geomA->get_type()][geomB->get_type()];
    if (f) {
      return f(geomA, poseA, geomB, poseB, contactsOut);
    }
    return 0;
  }

  std::vector<TinyContactPoint> compute_contacts(const TinyGeometry* geomA,
                                                 const TinyPose& poseA,
                                                 const TinyGeometry* geomB,
                                                 const TinyPose& poseB) {
    std::vector<TinyContactPoint> pts;
    int num = computeContacts(geomA, poseA, geomB, poseB, pts);
    return pts;
  }
};

#endif  // TINY_GEOMETRY_H
