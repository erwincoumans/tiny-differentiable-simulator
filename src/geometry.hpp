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

#include <stdexcept>
#include <vector>

#include "math/pose.hpp"
#include "sdf_utils.hpp"

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
};

template <typename Algebra>
class Sphere : public Geometry<Algebra>, public SDF<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar radius;

 public:
  explicit Sphere(const Scalar &radius)
      : Geometry<Algebra>(TINY_SPHERE_TYPE), radius(radius) {
    this->max_boundaries = Vector3(Scalar(2) * radius, Scalar(2) * radius, Scalar(2) * radius);
    this->min_boundaries = Vector3(Scalar(-2) * radius, Scalar(-2) * radius, Scalar(-2) * radius);
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
class Capsule : public Geometry<Algebra>, public SDF<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar radius;
  Scalar length;

 public:
  explicit Capsule(const Scalar &radius, const Scalar &length)
      : Geometry<Algebra>(TINY_CAPSULE_TYPE), radius(radius), length(length) {
    Scalar bound = Scalar(1.2) * (radius + length / Scalar(2));
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
               p.z() - std::clamp(p.z(), -this->length / Scalar(2), this->length / Scalar(2)));
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
        const Scalar &constant = Algebra::zero())
      : Geometry<Algebra>(TINY_PLANE_TYPE),
        normal(normal),
        constant(constant) {}

  template <typename AlgebraTo = Algebra>
  Plane<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return Plane<AlgebraTo>(C::convert(normal), C::convert(constant));
  }

  const Vector3 &get_normal() const { return normal; }
  const Scalar &get_constant() const { return constant; }
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
  }
}

template <typename Algebra>
class Cylinder : public Geometry<Algebra>, public SDF<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar radius;
  Scalar length;

 public:
  Cylinder(const Scalar &radius, const Scalar &length)
      : Geometry<Algebra>(TINY_CYLINDER_TYPE), radius(radius), length(length) {
    Scalar bound = (radius + length / Scalar(2));
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
    Scalar rb = radius / Scalar(20);  // Can be exposed to the users

    Scalar lxy = Algebra::sqrt(p.x() * p.x() + p.y() * p.y());
    Scalar lz = Algebra::abs(p.z());
    Scalar dx = lxy - radius + rb;
    Scalar dy = lz - length / Scalar(2) + rb;
    Scalar min_d = Algebra::min(Algebra::max(dx, dy), Algebra::zero());
    Scalar max_d = Algebra::sqrt(
        Algebra::max(dx, Algebra::zero()) * Algebra::max(dx, Algebra::zero()) +
        Algebra::max(dy, Algebra::zero()) * Algebra::max(dy, Algebra::zero()));
    return min_d + max_d - rb;
  }
};
}  // namespace tds
