#pragma once

#include <cassert>

#include "spatial_vector.hpp"

namespace tds {
template <typename Algebra>
struct RigidBodyInertia {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix6 = typename Algebra::Matrix6;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;

  /**
   * Mass \f$m\f$.
   */
  Scalar mass{Algebra::zero()};

  /**
   * Center of mass, also denoted as \f$h\f$.
   */
  Vector3 com{Algebra::zero3()};

  Matrix3 inertia{Algebra::diagonal3(Algebra::one())};

  RigidBodyInertia() = default;

  RigidBodyInertia(const RigidBodyInertia<Algebra> &rbi) = default;

  RigidBodyInertia(const Scalar &mass) : mass(mass) {}

  RigidBodyInertia(const Scalar &mass, const Vector3 &com,
                   const Matrix3 &inertia)
      : mass(mass), com(com), inertia(inertia) {}

  RigidBodyInertia(const Matrix6 &m)
      : mass(m(3, 3)),
        com{-m(1, 5), m(0, 5), -m(0, 4)},
        inertia(m(0, 0), m(1, 0), m(2, 0), m(0, 1), m(1, 1), m(2, 1), m(0, 2),
                m(1, 2), m(2, 2)) {}

  template <typename AlgebraTo = Algebra>
  RigidBodyInertia<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    RigidBodyInertia<AlgebraTo> rbi(C::convert(mass), C::convert(com),
                                    C::convert(inertia));
    return rbi;
  }

  void set_zero() {
    mass = Algebra::zero();
    Algebra::set_zero(com);
    Algebra::set_zero(inertia);
  }

  RigidBodyInertia operator+(const RigidBodyInertia &rbi) const {
    return RigidBodyInertia(mass + rbi.mass, com + rbi.com,
                            inertia + rbi.inertia);
  }

  RigidBodyInertia &operator+=(const RigidBodyInertia &rbi) {
    mass += rbi.mass;
    com += rbi.com;
    inertia += rbi.inertia;
    return *this;
  }

  void print(const char *name) const {
    printf("%s\n", name);
    printf("  mass:    %.8f\n", Algebra::to_double(mass));
    printf("  com:     %.8f\t%.8f\t%.8f\n", Algebra::to_double(com[0]),
           Algebra::to_double(com[1]), Algebra::to_double(com[2]));
    printf("  inertia: %.8f\t%.8f\t%.8f\n", Algebra::to_double(inertia(0, 0)),
           Algebra::to_double(inertia(0, 1)),
           Algebra::to_double(inertia(0, 2)));
    printf("           %.8f\t%.8f\t%.8f\n", Algebra::to_double(inertia(1, 0)),
           Algebra::to_double(inertia(1, 1)),
           Algebra::to_double(inertia(1, 2)));
    printf("           %.8f\t%.8f\t%.8f\n", Algebra::to_double(inertia(2, 0)),
           Algebra::to_double(inertia(2, 1)),
           Algebra::to_double(inertia(2, 2)));
  }
};

/**
 * The articulated body inertia matrix has the form
 *   [  I   H ]
 *   [ H^T  M ]
 * where M and I are symmetric 3x3 matrices.
 */
template <typename Algebra>
struct ArticulatedBodyInertia {
  using Index = typename Algebra::Index;
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix6 = typename Algebra::Matrix6;
  using Matrix6x3 = typename Algebra::Matrix6x3;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::RigidBodyInertia<Algebra> RigidBodyInertia;

  Matrix3 I{Algebra::diagonal3(Algebra::one())};
  Matrix3 H{Algebra::zero33()};
  Matrix3 M{Algebra::diagonal3(Algebra::one())};

  ArticulatedBodyInertia() = default;
  ArticulatedBodyInertia(const Matrix3 &I, const Matrix3 &H, const Matrix3 &M)
      : I(I), H(H), M(M) {}

  ArticulatedBodyInertia(const RigidBodyInertia &rbi) {
    H = Algebra::cross_matrix(rbi.com);
    I = rbi.inertia + H * Algebra::transpose(H) * rbi.mass;
    M = Algebra::diagonal3(rbi.mass);
    H = H * rbi.mass;
  }

  ArticulatedBodyInertia &operator=(const RigidBodyInertia &rbi) {
    H = Algebra::cross_matrix(rbi.com);
    I = rbi.inertia + H * Algebra::transpose(H) * rbi.mass;
    M = Algebra::diagonal3(rbi.mass);
    H = H * rbi.mass;
    // printf("Creating ABI from RBI:\n");
    // Algebra::print("RBI", rbi);
    // Algebra::print("Resulting ABI", *this);
    return *this;
  }

  ArticulatedBodyInertia(const Matrix6 &m) {
    Algebra::assign_block(I, m, 0, 0, 3, 3, 0, 0);
    Algebra::assign_block(H, m, 0, 0, 3, 3, 0, 3);
    Algebra::assign_block(M, m, 0, 0, 3, 3, 3, 3);
  }

  ArticulatedBodyInertia &operator=(const Matrix6 &m) {
    Algebra::assign_block(I, m, 0, 0, 3, 3, 0, 0);
    Algebra::assign_block(H, m, 0, 0, 3, 3, 0, 3);
    Algebra::assign_block(M, m, 0, 0, 3, 3, 3, 3);
    return *this;
  }

  template <typename AlgebraTo = Algebra>
  ArticulatedBodyInertia<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return ArticulatedBodyInertia<AlgebraTo>(C::convert(I), C::convert(H),
                                             C::convert(M));
  }

  Matrix6 matrix() const {
    Matrix6 m;
    Matrix3 Ht = Algebra::transpose(H);
    Algebra::assign_block(m, I, 0, 0);
    Algebra::assign_block(m, H, 0, 3);
    Algebra::assign_block(m, Ht, 3, 0);
    Algebra::assign_block(m, M, 3, 3);
    return m;
  }

  Scalar &operator()(int i, int j) {
    assert(0 <= i && i < 6);
    assert(0 <= j && j < 6);

    if (i < 3) {
      if (j < 3) {
        return I(i, j);
      } else {
        return H(i, j - 3);
      }
    } else {
      if (j < 3) {
        // need to transpose H here
        return H(j, i - 3);
      } else {
        return M(i - 3, j - 3);
      }
    }
  }
  const Scalar &operator()(int i, int j) const {
    assert(0 <= i && i < 6);
    assert(0 <= j && j < 6);

    if (i < 3) {
      if (j < 3) {
        return I(i, j);
      } else {
        return H(i, j - 3);
      }
    } else {
      if (j < 3) {
        // need to transpose H here
        return H(j, i - 3);
      } else {
        return M(i - 3, j - 3);
      }
    }
  }

  /**
   * V = mv(w, v)
   * Ia*v = mv(Iw + Hv, Mv + H^T w)
   */
  ForceVector operator*(const MotionVector &v) const {
    ForceVector result;
    result.top = I * v.top + H * v.bottom;
    result.bottom = M * v.bottom + Algebra::transpose(H) * v.top;
    return result;
  }
  Matrix6x3 operator*(const Matrix6x3 &v) const {
    Matrix6x3 result;
    Matrix3 temp = I * Algebra::top(v) + H * Algebra::bottom(v);
    Algebra::assign_block(result, temp, 0, 0);
    temp = M * Algebra::bottom(v) + Algebra::transpose(H) * Algebra::top(v);
    Algebra::assign_block(result, temp, 3, 0);
    return result;
  }

  ForceVector mul_org(const MotionVector &v) const {
    ForceVector result;
    auto bottomleft = Algebra::transpose(H);
    auto topleft_transpose = Algebra::transpose(I);
    auto topleft = I;
    auto top_right = H;
    result.top =
        Algebra::transpose(H) * v.top + Algebra::transpose(I) * v.bottom;
    // result.top = Algebra::transpose(H) * v.top + M * v.bottom;
    result.bottom = I * v.top + H * v.bottom;
    return result;
  }

  ArticulatedBodyInertia operator+(const ArticulatedBodyInertia &abi) const {
    return ArticulatedBodyInertia(I + abi.I, H + abi.H, M + abi.M);
  }
  ArticulatedBodyInertia operator-(const ArticulatedBodyInertia &abi) const {
    return ArticulatedBodyInertia(I - abi.I, H - abi.H, M - abi.M);
  }

  ArticulatedBodyInertia &operator+=(const ArticulatedBodyInertia &abi) {
    I += abi.I;
    H += abi.H;
    M += abi.M;
    return *this;
  }

  ArticulatedBodyInertia &operator+=(const Matrix6 &m) {
    Matrix3 tmp;
    Algebra::assign_block(tmp, m, 0, 0, 3, 3, 0, 0);
    I += tmp;
    Algebra::assign_block(tmp, m, 0, 0, 3, 3, 0, 3);
    H += tmp;
    Algebra::assign_block(tmp, m, 0, 0, 3, 3, 3, 3);
    M += tmp;
    return *this;
  }

  ArticulatedBodyInertia &operator-=(const ArticulatedBodyInertia &abi) {
    I -= abi.I;
    H -= abi.H;
    M -= abi.M;
    return *this;
  }

  ArticulatedBodyInertia &operator-=(const Matrix6 &m) {
    Matrix3 tmp;
    Algebra::assign_block(tmp, m, 0, 0, 3, 3, 0, 0);
    I -= tmp;
    Algebra::assign_block(tmp, m, 0, 0, 3, 3, 0, 3);
    H -= tmp;
    Algebra::assign_block(tmp, m, 0, 0, 3, 3, 3, 3);
    M -= tmp;
    return *this;
  }

  ArticulatedBodyInertia operator+(const Matrix6 &m) const {
    ArticulatedBodyInertia abi(*this);
    abi += m;
    return abi;
  }

  ArticulatedBodyInertia operator-(const Matrix6 &m) const {
    ArticulatedBodyInertia abi(*this);
    abi -= m;
    return abi;
  }

  bool is_invertible() const {
    if (Algebra::is_zero(Algebra::determinant(I))) {
      return false;
    }
    Matrix3 Ainv = Algebra::inverse(I);
    Matrix3 B = H;
    Matrix3 C = -B;
    Matrix3 MCAinvB = M - C * Ainv * B;
    if (Algebra::is_zero(Algebra::determinant(MCAinvB))) {
      return false;
    }
    return true;
  }

  ArticulatedBodyInertia inverse() const {
    // Inverse of a symmetric block matrix
    // according to (4.1) in
    //
    // http://msvlab.hre.ntou.edu.tw/grades/now/inte/Inverse%20&%20Border/border-LuTT.pdf
    Matrix3 Ainv = Algebra::inverse(I);
    Matrix3 B = H;
    Matrix3 C = -B;
    Matrix3 MCAinvB = M - C * Ainv * B;
    Matrix3 DCAB = Algebra::inverse(MCAinvB);
    Matrix3 AinvBDCAB = Ainv * B * DCAB;

    ArticulatedBodyInertia abi;
    abi.I = Ainv + AinvBDCAB * C * Ainv;
    abi.H = -AinvBDCAB;
    abi.M = DCAB;
    return abi;
  }

  MotionVector inv_mul(const ForceVector &v) const {
    // TODO verify
    ArticulatedBodyInertia abi = inverse();
    MotionVector result;
    result.top = abi.I * v.top + abi.H * v.bottom;
    result.bottom = abi.M * v.bottom + Algebra::transpose(abi.H) * v.top;
    return result;
  }

  /**
   * Multiplies force vectors a and b as a * b^T, resulting in a 6x6 matrix.
   */
  static ArticulatedBodyInertia mul_transpose(const ForceVector &a,
                                              const ForceVector &b) {
    // printf("mul_transpose:\n");
    // Algebra::print("a", a);
    // Algebra::print("b", b);
    ArticulatedBodyInertia abi;
    for (Index i = 0; i < 3; i++) {
      Vector3 a_top_b_top = a.top * b.top[i];
      Algebra::assign_column(abi.I, i, a_top_b_top);
      Vector3 a_top_b_bottom = a.top * b.bottom[i];
      Algebra::assign_column(abi.H, i, a_top_b_bottom);
      Vector3 a_bottom_b_bottom = a.bottom * b.bottom[i];
      Algebra::assign_column(abi.M, i, a_bottom_b_bottom);
    }
    return abi;
  }

  /**
   * Multiplies Matrix6x3 a and b as a * b^T, resulting in a 6x6 matrix.
   */
  static ArticulatedBodyInertia mul_transpose(const Matrix6x3 &a,
                                              const Matrix6x3 &b) {
    // printf("mul_transpose:\n");
    // Algebra::print("a", a);
    // Algebra::print("b", b);
    ArticulatedBodyInertia abi;
    abi.I = Algebra::top(a) * Algebra::top(b).transpose();
    abi.M = Algebra::bottom(a) * Algebra::bottom(b).transpose();
    abi.H = Algebra::top(a) * Algebra::bottom(b).transpose();

    //#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    //        abi.H = Htemp.transpose();
    //#else
    //        abi.H = Htemp;
    //#endif
    return abi;
  }

  void print(const char *name) const {
    printf("%s\n", name);
    print("I", I, 4);
    print("H", H, 4);
    print("M", M, 4);
  }

 private:
  static void print(const char *name, const Matrix3 &m, int indent) {
    for (int i = 0; i < indent; ++i) {
      printf(" ");
    }
    printf("%s:\n", name);
    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < indent; ++i) {
        printf(" ");
      }
      printf("%.8f  %.8f  %.8f\n", Algebra::to_double(m(j, 0)),
             Algebra::to_double(m(j, 1)), Algebra::to_double(m(j, 2)));
    }
  }
};
}  // namespace tds
