#pragma once

#include "geometry.hpp"

namespace tds {
template <typename Algebra>
struct ContactPoint {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Vector3 world_normal_on_b;
  Vector3 world_point_on_a;
  Vector3 world_point_on_b;
  Scalar distance;
};

template <typename Algebra>
int contact_sphere_sphere(const tds::Geometry<Algebra>* geomA,
                          const tds::Pose<Algebra>& poseA,
                          const tds::Geometry<Algebra>* geomB,
                          const tds::Pose<Algebra>& poseB,
                          std::vector<ContactPoint<Algebra> >& contactsOut) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  Scalar CONTACT_EPSILON = Algebra::fraction(1, 100000);

  typedef tds::Sphere<Algebra> Sphere;
  typedef tds::ContactPoint<Algebra> ContactPoint;
  assert(geomA->get_type() == TINY_SPHERE_TYPE);
  assert(geomB->get_type() == TINY_SPHERE_TYPE);
  Sphere* sphereA = (Sphere*)geomA;
  Sphere* sphereB = (Sphere*)geomB;

  Vector3 diff = poseA.position - poseB.position;
  Scalar length = Algebra::norm(diff);
  Scalar distance = length - (sphereA->get_radius() + sphereB->get_radius());
  Vector3 normal_on_b;
  normal_on_b = Algebra::unit3_x();
  if (Algebra::greater_than(length, CONTACT_EPSILON)) {
    Vector3 normal_on_b = Algebra::one() / length * diff;
    Vector3 point_a_world =
        poseA.position - sphereA->get_radius() * normal_on_b;
    Vector3 point_b_world = point_a_world - distance * normal_on_b;
    ContactPoint pt;
    pt.world_normal_on_b = normal_on_b;
    pt.world_point_on_a = point_a_world;
    pt.world_point_on_b = point_b_world;
    pt.distance = distance;
    contactsOut.push_back(pt);
    return 1;
  }
  return 0;
}

template <typename Algebra>
int contact_plane_sphere(const tds::Geometry<Algebra>* geomA,
                         const tds::Pose<Algebra>& poseA,
                         const tds::Geometry<Algebra>* geomB,
                         const tds::Pose<Algebra>& poseB,
                         std::vector<ContactPoint<Algebra> >& contactsOut) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  typedef tds::Sphere<Algebra> Sphere;
  typedef tds::Plane<Algebra> Plane;
  typedef tds::ContactPoint<Algebra> ContactPoint;
  assert(geomA->get_type() == TINY_PLANE_TYPE);
  assert(geomB->get_type() == TINY_SPHERE_TYPE);
  Plane* planeA = (Plane*)geomA;
  Sphere* sphereB = (Sphere*)geomB;

  Scalar t =
      -(Algebra::dot(poseB.position, -planeA->get_normal()) + planeA->get_constant());
  Vector3 pointAWorld = poseB.position + t * -planeA->get_normal();
  Scalar distance = t - sphereB->get_radius();
  Vector3 pointBWorld =
      poseB.position - sphereB->get_radius() * planeA->get_normal();
  ContactPoint pt;
  pt.world_normal_on_b = -planeA->get_normal();
  pt.world_point_on_a = pointAWorld;
  pt.world_point_on_b = pointBWorld;
  pt.distance = distance;
  contactsOut.push_back(pt);
  return 1;
}

template <typename Algebra>
int contact_plane_capsule(const tds::Geometry<Algebra>* geomA,
                          const tds::Pose<Algebra>& poseA,
                          const tds::Geometry<Algebra>* geomB,
                          const tds::Pose<Algebra>& poseB,
                          std::vector<ContactPoint<Algebra> >& contactsOut) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  typedef tds::Pose<Algebra> Pose;
  typedef tds::Plane<Algebra> Plane;
  typedef tds::Capsule<Algebra> Capsule;
  typedef tds::ContactPoint<Algebra> ContactPoint;
  typedef tds::Sphere<Algebra> Sphere;
  assert(geomA->get_type() == TINY_PLANE_TYPE);
  assert(geomB->get_type() == TINY_CAPSULE_TYPE);
  Capsule* capsule = (Capsule*)geomB;

  // create twice a plane-sphere contact
  Sphere sphere(capsule->get_radius());
  // shift the sphere to each end-point
  Pose offset;
  Algebra::set_identity(offset.orientation);
  offset.position = Vector3(Algebra::zero(), Algebra::zero(),
                             Algebra::fraction(1, 2) * capsule->get_length());
  Pose poseEndSphere = poseB * offset;
  contact_plane_sphere<Algebra>(geomA, poseA, &sphere, poseEndSphere,
                              contactsOut);
  offset.position = Vector3(Algebra::zero(), Algebra::zero(),
                             Algebra::fraction(-1, 2) * capsule->get_length());
  poseEndSphere = poseB * offset;
  contact_plane_sphere<Algebra>(geomA, poseA, &sphere, poseEndSphere,
                              contactsOut);

  return 2;
}

template <typename Algebra>
class CollisionDispatcher {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  typedef tds::Geometry<Algebra> Geometry;
  typedef tds::Pose<Algebra> Pose;
  typedef tds::ContactPoint<Algebra> ContactPoint;

  typedef int (*contact_func)(const Geometry* geomA, const Pose& poseA,
                              const Geometry* geomB, const Pose& poseB,
                              std::vector<ContactPoint>& contactsOut);

  contact_func contactFuncs[TINY_MAX_GEOM_TYPE][TINY_MAX_GEOM_TYPE];

 public:
  CollisionDispatcher() {
    for (int i = 0; i < TINY_MAX_GEOM_TYPE; i++) {
      for (int j = 0; j < TINY_MAX_GEOM_TYPE; j++) {
        contactFuncs[i][j] = 0;
      }
    }
    contactFuncs[TINY_SPHERE_TYPE][TINY_SPHERE_TYPE] = contact_sphere_sphere;
    contactFuncs[TINY_PLANE_TYPE][TINY_SPHERE_TYPE] = contact_plane_sphere;
    contactFuncs[TINY_PLANE_TYPE][TINY_CAPSULE_TYPE] = contact_plane_capsule;
  }

  inline int compute_contacts(const Geometry* geomA, const Pose& poseA,
                              const Geometry* geomB, const Pose& poseB,
                              std::vector<ContactPoint>& contactsOut) {
    contact_func f = contactFuncs[geomA->get_type()][geomB->get_type()];
    if (f) {
      return f(geomA, poseA, geomB, poseB, contactsOut);
    }
    return 0;
  }

  inline std::vector<ContactPoint> compute_contacts(const Geometry* geomA,
                                                    const Pose& poseA,
                                                    const Geometry* geomB,
                                                    const Pose& poseB) {
    std::vector<ContactPoint> pts;
    int num = compute_contacts(geomA, poseA, geomB, poseB, pts);
    return pts;
  }
};
}  // namespace tds