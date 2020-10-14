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

#include <vector>

#include "math/pose.hpp"

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
class Sphere : public Geometry<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Scalar radius;

 public:
  explicit Sphere(Scalar radius)
      : Geometry<Algebra>(TINY_SPHERE_TYPE), radius(radius) {}

  Scalar get_radius() const { return radius; }

  Vector3 compute_local_inertia(Scalar mass) const {
    Scalar elem = Algebra::fraction(4, 10) * mass * radius * radius;
    return Vector3(elem, elem, elem);
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
  explicit Capsule(Scalar radius, Scalar length)
      : Geometry<Algebra>(TINY_CAPSULE_TYPE), radius(radius), length(length) {}

  Scalar get_radius() const { return radius; }
  Scalar get_length() const { return length; }

  Vector3 compute_local_inertia(Scalar mass) const {
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
};

template <typename Algebra>
class Plane : public Geometry<Algebra> {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;

  Vector3 normal;
  Scalar constant;

 public:
  Plane()
      : Geometry<Algebra>(TINY_PLANE_TYPE),
        normal(Algebra::zero(), Algebra::zero(), Algebra::one()),
        constant(Algebra::zero()) {}

  const Vector3& get_normal() const { return normal; }
  Scalar get_constant() const { return constant; }
};
}