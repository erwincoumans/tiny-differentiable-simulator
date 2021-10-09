#pragma once

#include <cassert>

#include "geometry.hpp"
#include "math/conditionals.hpp"
#undef max
#undef min

namespace tds {
template <typename Algebra>
struct ContactPoint {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Vector3 world_normal_on_b;
  Vector3 world_point_on_a;
  Vector3 world_point_on_b;
  Scalar distance;
  Scalar normal_force;
  Scalar lateral_friction_force_1;
  Scalar lateral_friction_force_2;
  Vector3 fr_direction_1;
  Vector3 fr_direction_2;

  template <typename AlgebraTo = Algebra>
  ContactPoint<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    ContactPoint<AlgebraTo> conv;
    conv.world_normal_on_b = C::convert(world_normal_on_b);
    conv.world_point_on_a = C::convert(world_point_on_a);
    conv.world_point_on_b = C::convert(world_point_on_b);
    conv.distance = C::convert(distance);
    conv.normal_force = C::convert(normal_force);
    conv.lateral_friction_1 = C::convert(lateral_friction_force_1);
    conv.lateral_friction_2 = C::convert(lateral_friction_force_2);
    conv.fr_direction_1 = C::convert(fr_direction_1);
    conv.fr_direction_2 = C::convert(fr_direction_2);
    return conv;
  }
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
  const Sphere* sphereA = (const Sphere*)geomA;
  const Sphere* sphereB = (const Sphere*)geomB;

  Vector3 diff = poseA.position_ - poseB.position_;
  Scalar length = Algebra::norm(diff);
  Scalar distance = length - (sphereA->get_radius() + sphereB->get_radius());
  Vector3 normal_on_b;
  normal_on_b = Algebra::unit3_x();
  if constexpr (is_cppad_scalar<Scalar>::value) {
    // always return contact point so that we can trace it
    Vector3 normal_on_b = Algebra::normalize(diff);
    Vector3 point_a_world =
        poseA.position_ - sphereA->get_radius() * normal_on_b;
    Vector3 point_b_world = point_a_world - distance * normal_on_b;
    ContactPoint pt;
    pt.world_normal_on_b = normal_on_b;
    pt.world_point_on_a = point_a_world;
    pt.world_point_on_b = point_b_world;
    pt.distance = distance;
    contactsOut.push_back(pt);
    return 1;
  } else {
    if (Algebra::greater_than(length, CONTACT_EPSILON)) {
      Vector3 normal_on_b = Algebra::one() / length * diff;
      Vector3 point_a_world =
          poseA.position_ - sphereA->get_radius() * normal_on_b;
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
  const Plane* planeA = (const Plane*)geomA;
  const Sphere* sphereB = (const Sphere*)geomB;

  Scalar t = -(Algebra::dot(poseB.position_, -planeA->get_normal()) +
               planeA->get_constant());
  Vector3 pointAWorld = poseB.position_ + t * -planeA->get_normal();
  Scalar distance = t - sphereB->get_radius();
  Vector3 pointBWorld =
      poseB.position_ - sphereB->get_radius() * planeA->get_normal();
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
  const Capsule* capsule = (const Capsule*)geomB;

  // create twice a plane-sphere contact
  Sphere sphere(capsule->get_radius());
  // shift the sphere to each end-point
  Pose offset;
  Algebra::set_identity(offset.orientation_);
  offset.position_ = Vector3(Algebra::zero(), Algebra::zero(),
                             Algebra::fraction(1, 2) * capsule->get_length());
  Pose poseEndSphere = poseB * offset;
  contact_plane_sphere<Algebra>(geomA, poseA, &sphere, poseEndSphere,
                                contactsOut);
  offset.position_ = Vector3(Algebra::zero(), Algebra::zero(),
                             Algebra::fraction(-1, 2) * capsule->get_length());
  poseEndSphere = poseB * offset;
  contact_plane_sphere<Algebra>(geomA, poseA, &sphere, poseEndSphere,
                                contactsOut);

  return 2;
}

template <typename Algebra>
int contact_plane_box(const tds::Geometry<Algebra>* geomA,
                      const tds::Pose<Algebra>& poseA,
                      const tds::Geometry<Algebra>* geomB,
                      const tds::Pose<Algebra>& poseB,
                      std::vector<ContactPoint<Algebra> >& contactsOut) {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  typedef tds::Pose<Algebra> Pose;
  typedef tds::Plane<Algebra> Plane;
  typedef tds::Box<Algebra> Box;
  typedef tds::ContactPoint<Algebra> ContactPoint;
  typedef tds::Sphere<Algebra> Sphere;
  assert(geomA->get_type() == TINY_PLANE_TYPE);
  assert(geomB->get_type() == TINY_BOX_TYPE);
  const Box* box = (const Box*)geomB;

  // enforce some minimum radius for the spheres at the corner points
  const Scalar collision_radius =
      Algebra::max(Algebra::from_double(1e-2), box->get_radius());

  // create twice a plane-sphere contact
  Sphere sphere(collision_radius);
  // shift the sphere to each end-point
  Pose offset;
  Algebra::set_identity(offset.orientation_);
  // compute contact_plane_sphere for each corner
  for (const Vector3& corner : box->get_corner_points(collision_radius)) {
    offset.position_ = corner;
    Pose poseEndSphere = poseB * offset;
    contact_plane_sphere<Algebra>(geomA, poseA, &sphere, poseEndSphere,
                                  contactsOut);
  }

  return 8;
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
    contactFuncs[TINY_PLANE_TYPE][TINY_BOX_TYPE] = contact_plane_box;
  }

  inline int compute_contacts(const Geometry* geomA, const Pose& poseA,
                              const Geometry* geomB, const Pose& poseB,
                              std::vector<ContactPoint>& contactsOut) {
    contact_func f = contactFuncs[geomA->get_type()][geomB->get_type()];
    if (f) {
      return f(geomA, poseA, geomB, poseB, contactsOut);
    }
    contact_func g = contactFuncs[geomB->get_type()][geomA->get_type()];
    if (g) {
      int prev_contacts = contactsOut.size();
      int num_contacts = g(geomB, poseB, geomA, poseA, contactsOut);
      // swap normal and points a,b
      for (int i = prev_contacts; i < num_contacts; i++) {
        auto tmp = contactsOut[i].world_point_on_a;
        contactsOut[i].world_point_on_a = contactsOut[i].world_point_on_b;
        contactsOut[i].world_point_on_b = tmp;
        contactsOut[i].world_normal_on_b = -contactsOut[i].world_normal_on_b;
      }
      return num_contacts;
    }
    return 0;
  }

  inline std::vector<ContactPoint> compute_contacts2(const Geometry* geomA,
                                                     const Pose& poseA,
                                                     const Geometry* geomB,
                                                     const Pose& poseB) {
    std::vector<ContactPoint> pts;
    int num = compute_contacts(geomA, poseA, geomB, poseB, pts);
    return pts;
  }
};
}  // namespace tds
