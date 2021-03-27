#pragma once

#include "inertia.hpp"
#include "spatial_vector.hpp"

// right-associative means transforms are multiplied like parent_transform * child_transform.
// RBDL uses left-associative transforms
//#define TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS


namespace tds {
template <typename Algebra>
struct Transform {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix6 = typename Algebra::Matrix6;
  using Matrix6x3 = typename Algebra::Matrix6x3;
  using RigidBodyInertia = tds::RigidBodyInertia<Algebra>;
  using ArticulatedBodyInertia = tds::ArticulatedBodyInertia<Algebra>;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;

  Vector3 translation{Algebra::zero3()};
  Matrix3 rotation{Algebra::eye3()};

  Transform() = default;
  Transform(const Vector3 &translation) : translation(translation) {}
  Transform(const Matrix3 &rotation) : rotation(rotation) {}
  Transform(const Vector3 &translation, const Matrix3 &rotation)
      : translation(translation), rotation(rotation) {}
  Transform(const Scalar &trans_x, const Scalar &trans_y, const Scalar &trans_z)
      : translation(trans_x, trans_y, trans_z) {}

  template <typename AlgebraTo = Algebra>
  Transform<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    return Transform<AlgebraTo>(C::convert(translation), C::convert(rotation));
  }

  friend std::ostream &operator<<(std::ostream &os, const Transform &tf) {
    os << "[ translation: " << tf.translation << "  rotation: " << tf.rotation
       << " ]";
    return os;
  }
  void print(const char *title) const {
    printf("%s\n", title);
    printf("  translation:  %.4f\t%.4f\t%.4f\n",
           Algebra::to_double(translation[0]),
           Algebra::to_double(translation[1]),
           Algebra::to_double(translation[2]));
    printf("  rotation:     %.4f\t%.4f\t%.4f\n",
           Algebra::to_double(rotation(0, 0)),
           Algebra::to_double(rotation(0, 1)),
           Algebra::to_double(rotation(0, 2)));
    printf("                %.4f\t%.4f\t%.4f\n",
           Algebra::to_double(rotation(1, 0)),
           Algebra::to_double(rotation(1, 1)),
           Algebra::to_double(rotation(1, 2)));
    printf("                %.4f\t%.4f\t%.4f\n",
           Algebra::to_double(rotation(2, 0)),
           Algebra::to_double(rotation(2, 1)),
           Algebra::to_double(rotation(2, 2)));
  }

  TINY_INLINE void set_identity() {
    Algebra::set_zero(translation);
    // set diagonal entries to one, others to zero
    rotation = Algebra::eye3();
  }

  Matrix6 matrix() const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
    Matrix3 Et = Algebra::transpose(rotation);
#endif
    Matrix6 m;
    Matrix3 mErx = -E * Algebra::cross_matrix(translation);
    Algebra::assign_block(m, E, 0, 0);
    Algebra::assign_block(m, Algebra::zero33(), 0, 3);
    Algebra::assign_block(m, mErx, 3, 0);
    Algebra::assign_block(m, E, 3, 3);
    return m;
  }

  Matrix6 matrix_transpose() const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
    Matrix3 Et = Algebra::transpose(rotation);
#endif
    Matrix3 mErxT = Algebra::transpose(-E * Algebra::cross_matrix(translation));
    Matrix6 m;
    Algebra::assign_block(m, Et, 0, 0);
    Algebra::assign_block(m, mErxT, 0, 3);
    Algebra::assign_block(m, Algebra::zero33(), 3, 0);
    Algebra::assign_block(m, Et, 3, 3);
    return m;
  }

/**
 * X1*X2 = plx(E1*E2, r2 + E2T*r1)
 */
// Transform operator*(const Transform &t) const {
//   Transform tr = *this;
//   tr.translation = t.translation + t.rotation * translation;
//   tr.rotation *= t.rotation;
//   return tr;
// }
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
  // Transform operator*(const Transform &t) const {
  //   /// XXX this is different from Featherstone
  //   Transform tr = *this;
  //   tr.translation += rotation * t.translation;
  //   tr.rotation *= t.rotation;
  //   return tr;
  // }
  Transform operator*(const Transform &t) const {
    /// XXX this is different from Featherstone: we assume transforms are
    /// right-associative
    Transform tr = *this;
    tr.translation += rotation * t.translation;
    //tr.translation += Algebra::transpose(rotation) * t.translation;
    tr.rotation *= t.rotation;
    return tr;
  }
  TINY_INLINE Vector3 apply(const Vector3 &point) const {
    return rotation * point + translation;
  }
  TINY_INLINE Vector3 apply_inverse(const Vector3 &point) const {
    return Algebra::transpose(rotation) * (point - translation);
  }
  TINY_INLINE Vector3 apply_inverse2(const Vector3& point) const {
      return Algebra::transpose(rotation) * (point - translation);
  }
#else
    // implementation from RBDL
  Transform operator*(const Transform &t) const {
    Transform tr = *this;
    // tr.translation = t.translation + t.rotation * translation;
    tr.translation =
        t.translation + Algebra::transpose(t.rotation) * translation;
    tr.rotation *= t.rotation;
    return tr;
  }

// Transform operator*(const Transform &t) const {
//   Transform tr = *this;
//   tr.translation = t.translation + t.rotation * translation;
//   // tr.translation = t.translation + Algebra::transpose(t.rotation) * translation;
//   tr.rotation *= t.rotation;
//   return tr;
// }
// Transform operator*(const Transform &t) const {
//   Transform tr = *this;
//   tr.translation = t.translation + t.rotation * translation;
//   tr.rotation *= t.rotation;
//   return tr;
// }
 

  TINY_INLINE Vector3 apply(const Vector3 &point) const {
    return rotation * point + translation;
  }
  TINY_INLINE Vector3 apply_inverse(const Vector3 &point) const {
    return Algebra::transpose(rotation) * (point - translation);
  }

  // Transform operator*(const Transform &t) const {
  //   /// XXX this is different from Featherstone
  //   Transform tr = *this;
  //   tr.translation += rotation * t.translation;
  //   tr.rotation *= t.rotation;
  //   return tr;
  // }
  // TINY_INLINE Vector3 apply(const Vector3 &point) const {
  //   return rotation * point + translation;
  // }
  // TINY_INLINE Vector3 apply_inverse(const Vector3 &point) const {
  //   return Algebra::transpose(rotation) * (point - translation);
  // }
#endif

  Transform inverse() const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    Matrix3 E = Algebra::transpose(rotation);
    const Matrix3 &Et = rotation;
    Transform inv;
    inv.rotation = E;
    inv.translation = E * -translation;
#else
    const Matrix3 &E = rotation;
    Matrix3 Et = Algebra::transpose(rotation);
    Transform inv;
    inv.rotation = Et;
    inv.translation = E * -translation;
#endif
    return inv;
  }

  /**
   * V = mv(w, v)
   * X*V = mv(E*w, E*(v - r x w))
   */
  inline MotionVector apply(const MotionVector &inVec) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
#endif

    MotionVector outVec;

    Vector3 rxw = Algebra::cross(translation, inVec.top);
    Vector3 v_rxw = inVec.bottom - rxw;

    outVec.top = E * inVec.top;
    outVec.bottom = E * v_rxw;

    return outVec;
  }

  /**
   * V = mv(w, v)
   * inv(X)*V = mv(ET*w, ET*v + r x (ET*w))
   */
  inline MotionVector apply_inverse(const MotionVector &inVec) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
#else
    Matrix3 Et = Algebra::transpose(rotation);
#endif

    MotionVector outVec;
    outVec.top = Et * inVec.top;
    outVec.bottom = Et * inVec.bottom + Algebra::cross(translation, outVec.top);
    return outVec;
  }

  /**
   * F = fv(n, f)
   * XT*F = fv(ETn + rxETf, ETf)
   */
  inline ForceVector apply(const ForceVector &inVec) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
#else
    Matrix3 Et = Algebra::transpose(rotation);
#endif

    ForceVector outVec;
    outVec.bottom = Et * inVec.bottom;
    outVec.top = Et * inVec.top;
    outVec.top += Algebra::cross(translation, outVec.bottom);

    return outVec;
  }

  /**
   * F = fv(n, f)
   * X^* F = fv(E(n - rxf), Ef)
   */
  inline ForceVector apply_inverse(const ForceVector &inVec) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
#endif
    const Vector3 &n = inVec.top;
    const Vector3 &f = inVec.bottom;
    ForceVector outVec;
    outVec.top = E * (n - Algebra::cross(translation, f));
    outVec.bottom = E * f;
    return outVec;
  }

  Matrix6x3 apply(const Matrix6x3 &inMat, bool is_force) const {
    if (is_force) return apply_to_3d_force(inMat);
    else return apply_to_3d_motion(inMat);
  }

  /**
   * F = fv(n, f)
   * XT*F = fv(ETn + rxETf, ETf)
   */
  inline Matrix6x3 apply_to_3d_force(const Matrix6x3 &inMat) const{
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
#else
    Matrix3 Et = Algebra::transpose(rotation);
#endif

    Matrix6x3 outMat;
    Matrix3 top = Et * Algebra::top(inMat);
    Matrix3 bottom = Et * Algebra::bottom(inMat);

    top += Algebra::cross_matrix(translation) * bottom;

    Algebra::assign_block(outMat, top, 0, 0);
    Algebra::assign_block(outMat, bottom, 3, 0);

    return outMat;
  }

  /**
   * V = mv(w, v)
   * X*V = mv(E*w, E*(v - r x w))
   */
  inline Matrix6x3 apply_to_3d_motion(const Matrix6x3 &inMat) const{
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
#endif

    Matrix6x3 outMat;
    Matrix3 top = Algebra::top(inMat);
    Matrix3 bottom = Algebra::bottom(inMat);

    Matrix3 rxw = Algebra::cross_matrix(translation) * top;
    Matrix3 v_rxw = bottom - rxw;

    Algebra::assign_block(outMat, E * top, 0, 0);
    Algebra::assign_block(outMat, E * v_rxw, 3, 0);
    return outMat;
  }

  inline Matrix6x3 apply_inverse(const Matrix6x3 &inMat, bool is_force) const {
    if (is_force) return apply_inverse_to_3d_force(inMat);
    else return apply_inverse_to_3d_motion(inMat);
  }

  /**
  * F = fv(n, f)
  * X^* F = fv(E(n - rxf), Ef)
  */
  inline Matrix6x3 apply_inverse_to_3d_force(const Matrix6x3 &inMat) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
#endif

    Matrix6x3 outMat;
    Matrix3 top = Algebra::top(inMat);
    Matrix3 bottom = Algebra::bottom(inMat);

    top -= Algebra::cross_matrix(translation) * bottom;

    Algebra::assign_block(outMat, E * top, 0, 0);
    Algebra::assign_block(outMat, E * bottom, 3, 0);

    return outMat;
  }

  /**
  * V = mv(w, v)
  * inv(X)*V = mv(ET*w, ET*v + r x (ET*w))
  */
  inline Matrix6x3 apply_inverse_to_3d_motion(const Matrix6x3 &inMat) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
#else
    Matrix3 Et = Algebra::transpose(rotation);
#endif

    Matrix6x3 outMat;
    Matrix3 top = Et * Algebra::top(inMat);
    Matrix3 bottom = Et * Algebra::bottom(inMat);
    bottom = Et * bottom + Algebra::cross_matrix(translation) * top;
    Algebra::assign_block(outMat, top, 0, 0);
    Algebra::assign_block(outMat, bottom, 3, 0);

    return outMat;
  }

  /**
   * Computes \f$ X^* I X^{-1} \f$.
   */
  inline RigidBodyInertia apply(const RigidBodyInertia &rbi) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
    Matrix3 Et = Algebra::transpose(rotation);
#endif
    RigidBodyInertia result(rbi.mass);
    const Matrix3 rx = Algebra::cross_matrix(translation);
    // E(I + rx hx + (h - mr)x rx) E^T
    result.inertia =
        E *
        (rbi.inertia + rx * Algebra::cross_matrix(rbi.com) +
         Algebra::cross_matrix(rbi.com - rbi.mass * translation) * rx) *
        Et;
    // E(h - mr)
    result.com = E * (rbi.com - rbi.mass * translation);
    return result;
  }

  /**
   * Computes \f$ X^T I X \f$.
   */
  inline RigidBodyInertia apply_transpose(const RigidBodyInertia &rbi) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
    Matrix3 Et = Algebra::transpose(rotation);
#endif
    RigidBodyInertia result(rbi.mass);
    // E^T h + mr
    const Vector3 Eth_mr = Et * rbi.com + rbi.mass * translation;
    const Matrix3 rx = Algebra::cross_matrix(translation);
    // E^T I E - rx(E^T h)x - (E^T h + mr)x rx
    result.inertia =
        (Et * rbi.inertia * E - rx * Algebra::cross_matrix(Et * rbi.com) -
         Algebra::cross_matrix(Eth_mr) * rx);
    // E^T h + mr
    result.com = Eth_mr;
    return result;
  }

  /**
   * Computes \f$ X^* I^A X^{-1} \f$.
   */
  inline ArticulatedBodyInertia apply(const ArticulatedBodyInertia &abi) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
    Matrix3 Et = Algebra::transpose(rotation);
#endif
    // modified version that matches the output of RBDL
    ArticulatedBodyInertia result;
    const Matrix3 rx = Algebra::cross_matrix(translation);
    // M' = E^T M E
    const Matrix3 Mp = Et * abi.M * E;
    result.M = Mp;
    // H' = E^T H E
    const Matrix3 Hp = Et * abi.H * E;
    // H' + rx M'
    const Matrix3 HrxM = Hp + rx * Mp;
    // E^T I E - rx H'^T - (H' + rx M') rx
    result.I = Et * abi.I * E - rx * Algebra::transpose(Hp) - HrxM * rx;
    // H' + rx M'
    result.H = HrxM;
    return result;
  }

  /**
   * Computes \f$ X^T I^A X \f$.
   */
  inline ArticulatedBodyInertia apply_transpose(
      const ArticulatedBodyInertia &abi) const {
#ifndef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
    const Matrix3 &Et = rotation;
    Matrix3 E = Algebra::transpose(rotation);
#else
    const Matrix3 &E = rotation;
    Matrix3 Et = Algebra::transpose(rotation);
#endif
    ArticulatedBodyInertia result;
    const Matrix3 rx = Algebra::cross_matrix(translation);
    // M' = E^T M E
    const Matrix3 Mp = Et * abi.M * E;
    result.M = Mp;
    // H' = E^T H E
    const Matrix3 Hp = Et * abi.H * E;
    // H' + rx M'
    const Matrix3 HrxM = Hp + rx * Mp;
    // E^T I E - rx H'^T - (H' + rx M') rx
    result.I = Et * abi.I * E - rx * Algebra::transpose(Hp) - HrxM * rx;
    // H' + rx M'
    result.H = HrxM;
    return result;
  }
};

template <typename AlgebraFrom, typename AlgebraTo = AlgebraFrom>
static TINY_INLINE Transform<AlgebraTo> clone(const Transform<AlgebraFrom>& x) {
  return x.template clone<AlgebraTo>();
}
}  // namespace tds
