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

#pragma once

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <vector>

#include "math/pose.hpp"

#undef min
#undef max

namespace tds {
enum GeometryTypes {
  TINY_SPHERE_TYPE = 0,
  TINY_PLANE_TYPE,
  TINY_CAPSULE_TYPE,
  TINY_MESH_TYPE,      // only for visual shapes at the moment
  TINY_BOX_TYPE,       // only for visual shapes at the moment
  TINY_CYLINDER_TYPE,  // unsupported
  TINY_MAX_GEOM_TYPE,
};

template <typename Algebra>
class Geometry {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  int type;

 public:
  explicit Geometry(int type) : type(type) {}
  virtual ~Geometry() = default;
  int get_type() const { return type; }

  // SDF related members
 public:
  const Vector3 &get_max_boundaries() const { return max_boundaries; }
  const Vector3 &get_min_boundaries() const { return min_boundaries; }
  virtual Scalar distance(const Vector3 &point) const {
    // SDF distance computation is not supported for this geometry type
    assert(0);
    return Algebra::zero();
  }

 protected:
  Vector3 max_boundaries;
  Vector3 min_boundaries;
};

template <typename Algebra>
class Sphere : public Geometry<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar radius;

 public:
  explicit Sphere(const Scalar &radius)
      : Geometry<Algebra>(TINY_SPHERE_TYPE), radius(radius) {
    this->max_boundaries = Vector3(Algebra::fraction(2, 1) * radius,
                                   Algebra::fraction(2, 1) * radius,
                                   Algebra::fraction(2, 1) * radius);
    this->min_boundaries = Vector3(Algebra::fraction(-2, 1) * radius,
                                   Algebra::fraction(-2, 1) * radius,
                                   Algebra::fraction(-2, 1) * radius);
  }

  template <typename AlgebraTo = Algebra>
  Sphere<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return Sphere<AlgebraTo>(C::convert(radius));
  }

  const Scalar &get_radius() const { return radius; }

  Vector3 compute_local_inertia(const Scalar &mass) const {
    Scalar elem = Algebra::fraction(4, 10) * mass * radius * radius;
    return Vector3(elem, elem, elem);
  }

  Scalar distance(const Vector3 &p) const override {
    return Algebra::norm(p) - radius;
  }
};

// capsule aligned with the Z axis
template <typename Algebra>
class Capsule : public Geometry<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar radius;
  Scalar length;

 public:
  explicit Capsule(const Scalar &radius, const Scalar &length)
      : Geometry<Algebra>(TINY_CAPSULE_TYPE), radius(radius), length(length) {
    Scalar bound =
        Algebra::fraction(12, 10) * (radius + length / Algebra::fraction(2, 1));
    this->max_boundaries = Vector3(bound, bound, bound);
    this->min_boundaries = Vector3(-bound, -bound, -bound);
  }

  template <typename AlgebraTo = Algebra>
  Capsule<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return Capsule<AlgebraTo>(C::convert(radius), C::convert(length));
  }

  const Scalar &get_radius() const { return radius; }
  const Scalar &get_length() const { return length; }

  Vector3 compute_local_inertia(const Scalar &mass) const {
    Scalar lx = Algebra::fraction(2, 1) * (radius);
    Scalar ly = Algebra::fraction(2, 1) * (radius);
    Scalar lz = length + Algebra::fraction(2, 1) * (radius);
    Scalar x2 = lx * lx;
    Scalar y2 = ly * ly;
    Scalar z2 = lz * lz;
    Scalar scaledmass = mass * Algebra::fraction(1, 12);

    Vector3 inertia;
    inertia[0] = scaledmass * (y2 + z2);
    inertia[1] = scaledmass * (x2 + z2);
    inertia[2] = scaledmass * (x2 + y2);
    return inertia;
  }

  Scalar distance(const Vector3 &p) const override {
    Vector3 pt(p.x(), p.y(),
               p.z() - std::clamp(p.z(), -this->length / Scalar(2),
                                  this->length / Scalar(2)));
    return Algebra::norm(pt) - this->radius;
  }
};

template <typename Algebra>
class Plane : public Geometry<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Vector3 normal;
  Scalar constant;

 public:
  Plane(const Vector3 &normal = Algebra::unit3_z(),
        const Scalar &constant = Algebra::zero(),
        const Scalar &bound = Scalar(500))
      : Geometry<Algebra>(TINY_PLANE_TYPE),
        normal(Algebra::normalize(normal)),
        constant(constant) {
    // TODO: Find a good boundary for rendering planes
    // Scalar bound = constant > Scalar(5.0) ? Scalar(2) * constant :
    // Scalar(10.);
    this->max_boundaries = Vector3(bound, bound, bound);
    this->min_boundaries = Vector3(-bound, -bound, -bound);
  }

  template <typename AlgebraTo = Algebra>
  Plane<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return Plane<AlgebraTo>(C::convert(normal), C::convert(constant));
  }

  const Vector3 &get_normal() const { return normal; }
  void set_normal(const Vector3 &n) { normal = Algebra::normalize(n); }
  const Scalar &get_constant() const { return constant; }

  Scalar distance(const Vector3 &p) const override {
    return Algebra::dot(p, this->normal) / Algebra::norm(this->normal) -
           this->constant;
  }
};

template <typename Algebra>
class Box : public Geometry<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Vector3 extents;
  // radius for rounded corners
  Scalar radius;

 public:
  Box(const Vector3 &extents = Vector3(Algebra::one(), Algebra::one(),
                                       Algebra::one()),
      const Scalar &radius = Algebra::zero())
      : Geometry<Algebra>(TINY_BOX_TYPE), extents(extents), radius(radius) {
    this->max_boundaries = extents;
    this->min_boundaries = -extents;
  }

  template <typename AlgebraTo = Algebra>
  Box<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return Box<AlgebraTo>(C::convert(extents), C::convert(radius));
  }

  const Vector3 &get_extents() const { 
      return extents; 
  }
  Vector3 get_half_extents() const { 
      Vector3 he = extents * Algebra::fraction(1, 2);
      return he;
  }

  void set_extents(const Vector3 &extents) { this->extents = extents; }

  const Scalar &get_radius() const { return radius; }
  void set_radius(const Scalar &radius) { this->radius = radius; }

  Scalar distance(const Vector3 &p) const override {
    Scalar qx = Algebra::abs(p[0]) - extents[0] * Algebra::fraction(1, 2);
    Scalar qy = Algebra::abs(p[1]) - extents[1] * Algebra::fraction(1, 2);
    Scalar qz = Algebra::abs(p[2]) - extents[2] * Algebra::fraction(1, 2);
    Vector3 q(Algebra::max(qx, Algebra::zero()),
              Algebra::max(qy, Algebra::zero()),
              Algebra::max(qz, Algebra::zero()));
    Scalar nq = Algebra::norm(q);
    Scalar d =
        nq +
        Algebra::min(Algebra::max(qx, Algebra::max(qy, qz)), Algebra::zero()) -
        radius;
    return d;
  }

  std::vector<Vector3> get_corner_points(
      const Scalar &radius = Algebra::zero()) const {
    const Scalar dx = extents[0] * Algebra::fraction(1, 2) - radius;
    const Scalar dy = extents[1] * Algebra::fraction(1, 2) - radius;
    const Scalar dz = extents[2] * Algebra::fraction(1, 2) - radius;
    std::vector<Vector3> points(8);
    points[0] = Vector3(dx, dy, dz);
    points[1] = Vector3(dx, dy, -dz);
    points[2] = Vector3(dx, -dy, dz);
    points[3] = Vector3(dx, -dy, -dz);
    points[4] = Vector3(-dx, dy, dz);
    points[5] = Vector3(-dx, dy, -dz);
    points[6] = Vector3(-dx, -dy, dz);
    points[7] = Vector3(-dx, -dy, -dz);
    return points;
  }
};

template <typename Algebra>
class Cylinder : public Geometry<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar radius;
  Scalar length;

 public:
  Cylinder(const Scalar &radius, const Scalar &length)
      : Geometry<Algebra>(TINY_CYLINDER_TYPE), radius(radius), length(length) {
    Scalar bound = (radius + length / Algebra::fraction(2, 1));
    this->max_boundaries = Vector3(bound, bound, bound);
    this->min_boundaries = Vector3(-bound, -bound, -bound);
  }

  template <typename AlgebraTo = Algebra>
  Cylinder<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return Cylinder<AlgebraTo>(C::convert(radius), C::convert(length));
  }

  const Scalar &get_radius() const { return radius; }
  const Scalar &get_length() const { return length; }

  Scalar distance(const Vector3 &p) const override {
    // The Marching Cubes algorithm is not very good at sharp edges
    // Define a small rounding radius to smooth the edge of the two faces
    Scalar rb =
        radius / Algebra::fraction(20, 1);  // Can be exposed to the user

    Scalar lxy = Algebra::sqrt(p.x() * p.x() + p.y() * p.y());
    Scalar lz = Algebra::abs(p.z());
    Scalar dx = lxy - radius + rb;
    Scalar dy = lz - length / Algebra::fraction(2, 1) + rb;
    Scalar min_d = Algebra::min(Algebra::max(dx, dy), Algebra::zero());
    Scalar max_d = Algebra::sqrt(
        Algebra::max(dx, Algebra::zero()) * Algebra::max(dx, Algebra::zero()) +
        Algebra::max(dy, Algebra::zero()) * Algebra::max(dy, Algebra::zero()));
    return min_d + max_d - rb;
  }
};

template <typename AlgebraFrom, typename AlgebraTo>
static TINY_INLINE Geometry<AlgebraTo> *clone(const Geometry<AlgebraFrom> *g) {
  switch (g->get_type()) {
    case TINY_SPHERE_TYPE:
      return new Sphere<AlgebraTo>(
          ((Sphere<AlgebraFrom> *)g)->template clone<AlgebraTo>());
    case TINY_CAPSULE_TYPE:
      return new Capsule<AlgebraTo>(
          ((Capsule<AlgebraFrom> *)g)->template clone<AlgebraTo>());
    case TINY_PLANE_TYPE:
      return new Plane<AlgebraTo>(
          ((Plane<AlgebraFrom> *)g)->template clone<AlgebraTo>());
    case TINY_CYLINDER_TYPE:
      return new Cylinder<AlgebraTo>(
          ((Cylinder<AlgebraFrom> *)g)->template clone<AlgebraTo>());
    case TINY_BOX_TYPE:
      return new Box<AlgebraTo>(
          ((Box<AlgebraFrom> *)g)->template clone<AlgebraTo>());
  }
}
}  // namespace tds

namespace std {
inline std::string to_string(tds::GeometryTypes type) {
  switch (type) {
    case tds::TINY_SPHERE_TYPE:
      return "sphere";
    case tds::TINY_CAPSULE_TYPE:
      return "capsule";
    case tds::TINY_PLANE_TYPE:
      return "plane";
    case tds::TINY_BOX_TYPE:
      return "box";
    default:
      return "unknown";
  }
}
}  // namespace std
